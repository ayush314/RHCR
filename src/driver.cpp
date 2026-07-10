#include "KivaSystem.h"
#include "SortingSystem.h"
#include "OnlineSystem.h"
#include "BeeSystem.h"
#include "WorkstationGraph.h"
#include "WorkstationSystem.h"
#include "ID.h"
#include "PIBT.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

void set_parameters(BasicSystem& system, const boost::program_options::variables_map& vm)
{
	system.outfile = vm["output"].as<std::string>();
	system.screen = vm["screen"].as<int>();
	system.log = vm["log"].as<bool>();
	system.num_of_drives = vm["agentNum"].as<int>();
	system.time_limit = vm["cutoffTime"].as<int>();
	system.simulation_window = vm["simulation_window"].as<int>();
	system.planning_window = vm["planning_window"].as<int>();
	system.travel_time_window = vm["travel_time_window"].as<int>();
	system.consider_rotation = vm["rotation"].as<bool>();
	system.k_robust = vm["robust"].as<int>();
	system.hold_endpoints = vm["hold_endpoints"].as<bool>();
	system.useDummyPaths = vm["dummy_paths"].as<bool>();
	if (vm.count("seed"))
		system.seed = vm["seed"].as<int>();
	else
		system.seed = (int)time(0);
	srand(system.seed);
}


MAPFSolver* set_solver(const BasicGraph& G, const boost::program_options::variables_map& vm)
{
	string solver_name = vm["single_agent_solver"].as<string>();
	SingleAgentSolver* path_planner;
	MAPFSolver* mapf_solver;
	if (solver_name == "ASTAR")
	{
		path_planner = new StateTimeAStar();
	}
	else if (solver_name == "SIPP")
	{
		path_planner = new SIPP();
	}
	else
	{
		cout << "Single-agent solver " << solver_name << "does not exist!" << endl;
		exit(-1);
	}

	solver_name = vm["solver"].as<string>();
	if (solver_name == "ECBS")
	{
		ECBS* ecbs = new ECBS(G, *path_planner);
		ecbs->potential_function = vm["potential_function"].as<string>();
		ecbs->potential_threshold = vm["potential_threshold"].as<double>();
		ecbs->suboptimal_bound = vm["suboptimal_bound"].as<double>();
		mapf_solver = ecbs;
	}
	else if (solver_name == "PBS")
	{
		PBS* pbs = new PBS(G, *path_planner);
		pbs->lazyPriority = vm["lazyP"].as<bool>();
        auto prioritize_start = vm["prioritize_start"].as<bool>();
        if (vm["hold_endpoints"].as<bool>() or vm["dummy_paths"].as<bool>())
            prioritize_start = false;
        pbs->prioritize_start = prioritize_start;
        pbs->setRT(vm["CAT"].as<bool>(), prioritize_start);
		mapf_solver = pbs;
	}
	else if (solver_name == "PIBT")
	{
		PIBT* pibt = new PIBT(G, *path_planner);
		pibt->set_pibt_policy(vm["pibt_policy"].as<string>());
		pibt->set_pressure_profile(vm["pibt_pressure_profile"].as<string>());
		pibt->set_hindrance_scope(vm["pibt_hindrance_scope"].as<string>());
		pibt->pressure_entry_penalty = vm["pibt_pressure_entry_penalty"].as<double>();
		pibt->set_pressure_inbound_limit(vm["pibt_pressure_inbound_limit"].as<int>());
		pibt->wait_penalty = vm["pibt_wait_penalty"].as<double>();
		pibt->exit_bonus = vm["pibt_exit_bonus"].as<double>();
		pibt->front_bonus = vm["pibt_front_bonus"].as<double>();
		pibt->soft_collision_penalty = vm["pibt_soft_collision_penalty"].as<double>();
		pibt->hindrance_tiebreak = vm["pibt_hindrance"].as<bool>();
		pibt->regret_iterations = vm["pibt_regret_iterations"].as<int>();
		pibt->regret_weight = vm["pibt_regret_weight"].as<double>();
		pibt->regret_scope = vm["pibt_regret_scope"].as<string>();
		pibt->random_tiebreak = vm["pibt_random_tiebreak"].as<bool>();
		pibt->front_priority_enabled = vm["pibt_front_priority"].as<bool>();
		pibt->phase_priority_enabled = vm["pibt_phase_priority"].as<bool>();
		mapf_solver = pibt;
	}
    else if (solver_name == "WHCA")
	{
		mapf_solver = new WHCAStar(G, *path_planner);
	}
	else if (solver_name == "LRA")
	{
		mapf_solver = new LRAStar(G, *path_planner);
	}
	else
	{
		cout << "Solver " << solver_name << "does not exist!" << endl;
		exit(-1);
	}

	if (vm["id"].as<bool>())
	{
		return new ID(G, *path_planner, *mapf_solver);
	}
	else
	{
		return mapf_solver;
	}
}


int main(int argc, char** argv) 
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("scenario", po::value<std::string>()->required(), "scenario (SORTING, KIVA, ONLINE, BEE, WORKSTATION)")
		("map,m", po::value<std::string>()->default_value(""), "input map file")
        ("benchmark", po::value<std::string>()->default_value(""), "input workstation benchmark json")
		("task", po::value<std::string>()->default_value(""), "input task file")
		("output,o", po::value<std::string>()->default_value("../exp/test"), "output folder name")
		("agentNum,k", po::value<int>()->required(), "number of drives")
		("cutoffTime,t", po::value<int>()->default_value(60), "cutoff time (seconds)")
		("seed,d", po::value<int>(), "random seed")
		("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
			("solver", po::value<string>()->default_value("PBS"), "solver (LRA, PBS, PIBT, WHCA, ECBS)")
		("id", po::value<bool>()->default_value(false), "independence detection")
		("single_agent_solver", po::value<string>()->default_value("SIPP"), "single-agent solver (ASTAR, SIPP)")
		("lazyP", po::value<bool>()->default_value(false), "use lazy priority")
		("simulation_time", po::value<int>()->default_value(5000), "run simulation")
		("simulation_window", po::value<int>()->default_value(5), "call the planner every simulation_window timesteps")
		("travel_time_window", po::value<int>()->default_value(0), "consider the traffic jams within the given window")
        ("planning_window", po::value<int>()->default_value(20),
			        "the planner outputs plans with first planning_window timesteps collision-free")
        ("service_time", po::value<int>()->default_value(3), "workstation dwell time")
        ("pressure_threshold", po::value<int>()->default_value(-1), "workstation pressure trigger threshold override; -1 uses the station policy default")
        ("station_policy", po::value<string>()->default_value("vanilla"), "workstation planning policy (vanilla, distance_age, pressure_aware)")
        ("stop_at_traffic_jam", po::value<bool>()->default_value(true), "stop workstation simulations when the traffic-jam detector triggers")
        ("pibt_policy", po::value<string>()->default_value("vanilla"), "PIBT workstation policy (vanilla, distance_age, pressure)")
        ("pibt_pressure_entry_penalty", po::value<double>()->default_value(2), "PIBT pressure-policy penalty for entering pressured station zones")
        ("pibt_pressure_inbound_limit", po::value<int>()->default_value(4), "maximum target-bound PIBT agents admitted inside each pressured station zone")
        ("pibt_pressure_profile", po::value<string>()->default_value("thirds"), "PIBT pressure admission profile (none, half, severe, thirds)")
        ("pibt_wait_penalty", po::value<double>()->default_value(2), "PIBT pressure-policy wait penalty")
        ("pibt_exit_bonus", po::value<double>()->default_value(1), "PIBT pressure-policy exit progress bonus")
        ("pibt_front_bonus", po::value<double>()->default_value(3), "PIBT pressure-policy front-runner progress bonus")
        ("pibt_soft_collision_penalty", po::value<double>()->default_value(0), "PIBT pressure-policy occupied-candidate penalty")
        ("pibt_hindrance", po::value<bool>()->default_value(true), "use the lightweight PIBT hindrance tiebreaker")
        ("pibt_hindrance_scope", po::value<string>()->default_value("inherited"), "PIBT hindrance scope")
        ("pibt_regret_iterations", po::value<int>()->default_value(1), "number of PIBT regret-learning passes; 1 disables regret")
        ("pibt_regret_weight", po::value<double>()->default_value(0.5), "exponential update weight for PIBT regret learning")
        ("pibt_regret_scope", po::value<string>()->default_value("all"), "PIBT regret scope (all, pickup, exit_pickup, outside_zone, pickup_outside_zone)")
        ("pibt_random_tiebreak", po::value<bool>()->default_value(true), "use a seeded random final PIBT preference tiebreak")
        ("pibt_front_priority", po::value<bool>()->default_value(true), "give the pressure front runner an ordering boost")
        ("pibt_phase_priority", po::value<bool>()->default_value(false), "give service and exit phases an ordering boost")
		("potential_function", po::value<string>()->default_value("NONE"), "potential function (NONE, SOC, IC)")
		("potential_threshold", po::value<double>()->default_value(0), "potential threshold")
		("rotation", po::value<bool>()->default_value(false), "consider rotation")
		("robust", po::value<int>()->default_value(0), "k-robust (for now, only work for PBS)")
		("CAT", po::value<bool>()->default_value(false), "use conflict-avoidance table")
		// ("PG", po::value<bool>()->default_value(false),
		//        "reuse the priority graph of the goal node of the previous search")
		("hold_endpoints", po::value<bool>()->default_value(false),
		        "Hold endpoints from Ma et al, AAMAS 2017")
		("dummy_paths", po::value<bool>()->default_value(false),
				"Find dummy paths from Liu et al, AAMAS 2019")
		("prioritize_start", po::value<bool>()->default_value(true), "Prioritize waiting at start locations")
		("suboptimal_bound", po::value<double>()->default_value(1), "Suboptimal bound for ECBS")
		("log", po::value<bool>()->default_value(false), "save the search trees (and the priority trees)")
		;
	clock_t start_time = clock();
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	po::notify(vm);
    string scenario = vm["scenario"].as<string>();
    string map_path = vm["map"].as<string>();
    string benchmark_path = vm["benchmark"].as<string>();

    // check params
    if (vm["hold_endpoints"].as<bool>() or vm["dummy_paths"].as<bool>())
    {
        if (vm["hold_endpoints"].as<bool>() and vm["dummy_paths"].as<bool>())
        {
            std::cerr << "Hold endpoints and dummy paths cannot be used simultaneously" << endl;
            exit(-1);
        }
        if (vm["simulation_window"].as<int>() != 1)
        {
            std::cerr << "Hold endpoints and dummy paths can only work when the simulation window is 1" << endl;
            exit(-1);
        }
        if (vm["planning_window"].as<int>() < INT_MAX / 2)
        {
            std::cerr << "Hold endpoints and dummy paths cannot work with planning windows" << endl;
            exit(-1);
        }
    }
    if (scenario == "WORKSTATION")
    {
        if (benchmark_path.empty())
        {
            std::cerr << "WORKSTATION requires --benchmark" << endl;
            exit(-1);
        }
        if (vm["hold_endpoints"].as<bool>() || vm["dummy_paths"].as<bool>())
        {
            std::cerr << "WORKSTATION does not support hold_endpoints or dummy_paths" << endl;
            exit(-1);
        }
        if (vm["agentNum"].as<int>() <= 0 || vm["simulation_window"].as<int>() <= 0 ||
            vm["planning_window"].as<int>() <= 0 || vm["service_time"].as<int>() < 0)
        {
            std::cerr << "WORKSTATION requires positive agent and window counts and nonnegative service time" << endl;
            exit(-1);
        }
        string workstation_solver = vm["solver"].as<string>();
        if ((workstation_solver != "PBS" && workstation_solver != "PIBT") || vm["id"].as<bool>())
        {
            std::cerr << "WORKSTATION requires --solver PBS or --solver PIBT with --id false" << endl;
            exit(-1);
        }
        if (workstation_solver == "PIBT")
        {
            string pibt_policy = vm["pibt_policy"].as<string>();
            if (pibt_policy != "vanilla" &&
                pibt_policy != "distance_age" &&
                pibt_policy != "pressure")
            {
                std::cerr << "WORKSTATION PIBT policy must be vanilla, distance_age, or pressure" << endl;
                exit(-1);
            }
            if (vm["pibt_pressure_inbound_limit"].as<int>() < 1)
            {
                std::cerr << "WORKSTATION PIBT pressure inbound limit must be positive" << endl;
                exit(-1);
            }
            string pressure_profile = vm["pibt_pressure_profile"].as<string>();
            if (pressure_profile != "none" && pressure_profile != "half" &&
                pressure_profile != "severe" && pressure_profile != "thirds")
            {
                std::cerr << "WORKSTATION PIBT pressure profile must be none, half, severe, or thirds" << endl;
                exit(-1);
            }
            string hindrance_scope = vm["pibt_hindrance_scope"].as<string>();
            if (hindrance_scope != "all" && hindrance_scope != "inherited" &&
                hindrance_scope != "dense" && hindrance_scope != "inherited_dense" &&
                hindrance_scope != "station" && hindrance_scope != "inherited_station" &&
                hindrance_scope != "outside_zone" && hindrance_scope != "inherited_outside_zone" &&
                hindrance_scope != "pickup" && hindrance_scope != "inherited_pickup")
            {
                std::cerr << "Invalid WORKSTATION PIBT hindrance scope" << endl;
                exit(-1);
            }
            if (vm["pibt_regret_iterations"].as<int>() < 1)
            {
                std::cerr << "WORKSTATION PIBT regret iterations must be positive" << endl;
                exit(-1);
            }
            double regret_weight = vm["pibt_regret_weight"].as<double>();
            if (regret_weight < 0 || regret_weight > 1)
            {
                std::cerr << "WORKSTATION PIBT regret weight must be in [0, 1]" << endl;
                exit(-1);
            }
            string regret_scope = vm["pibt_regret_scope"].as<string>();
            if (regret_scope != "all" && regret_scope != "pickup" &&
                regret_scope != "exit_pickup" && regret_scope != "outside_zone" &&
                regret_scope != "pickup_outside_zone")
            {
                std::cerr << "Invalid WORKSTATION PIBT regret scope" << endl;
                exit(-1);
            }
        }
    }
    else if (map_path.empty())
    {
        std::cerr << "Scenario " << scenario << " requires --map" << endl;
        exit(-1);
    }

    // make dictionary
	boost::filesystem::path dir(vm["output"].as<std::string>() +"/");
	boost::filesystem::create_directories(dir);
	if (vm["log"].as<bool>())
	{
		boost::filesystem::path dir1(vm["output"].as<std::string>() + "/goal_nodes/");
		boost::filesystem::path dir2(vm["output"].as<std::string>() + "/search_trees/");
		boost::filesystem::create_directories(dir1);
		boost::filesystem::create_directories(dir2);
	}


	if (scenario == "KIVA")
	{
		KivaGrid G;
		if (!G.load_map(map_path))
			return -1;
		MAPFSolver* solver = set_solver(G, vm);
		KivaSystem system(G, *solver);
		set_parameters(system, vm);
		G.preprocessing(system.consider_rotation);
		system.simulate(vm["simulation_time"].as<int>());
		return 0;
	}
	else if (scenario == "SORTING")
	{
		 SortingGrid G;
		 if (!G.load_map(map_path))
			 return -1;
		 MAPFSolver* solver = set_solver(G, vm);
		 SortingSystem system(G, *solver);
		 assert(!system.hold_endpoints);
		 assert(!system.useDummyPaths);
		 set_parameters(system, vm);
		 G.preprocessing(system.consider_rotation);
		 system.simulate(vm["simulation_time"].as<int>());
		 return 0;
	}
	else if (scenario == "ONLINE")
	{
		OnlineGrid G;
		if (!G.load_map(map_path))
			return -1;
		MAPFSolver* solver = set_solver(G, vm);
		OnlineSystem system(G, *solver);
		assert(!system.hold_endpoints);
		assert(!system.useDummyPaths);
		set_parameters(system, vm);
		G.preprocessing(system.consider_rotation);
		system.simulate(vm["simulation_time"].as<int>());
		return 0;
	}
	else if (scenario == "BEE")
	{
		BeeGraph G;
		if (!G.load_map(map_path))
			return -1;
		MAPFSolver* solver = set_solver(G, vm);
		BeeSystem system(G, *solver);
		assert(!system.hold_endpoints);
		assert(!system.useDummyPaths);
		set_parameters(system, vm);
		G.preprocessing(vm["task"].as<std::string>(), system.consider_rotation);
		system.load_task_assignments(vm["task"].as<std::string>());
		system.simulate();
		double runtime = (double)(clock() - start_time)/ CLOCKS_PER_SEC;
		cout << "Overall runtime:			" << runtime << " seconds." << endl;
		// cout << "	Reading from file:		" << G.loading_time + system.loading_time << " seconds." << endl;
		// cout << "	Preprocessing:			" << G.preprocessing_time << " seconds." << endl;
		// cout << "	Writing to file:		" << system.saving_time << " seconds." << endl;
		cout << "Makespan:		" << system.get_makespan() << " timesteps." << endl;
		cout << "Flowtime:		" << system.get_flowtime() << " timesteps." << endl;
		cout << "Flowtime lowerbound:	" << system.get_flowtime_lowerbound() << " timesteps." << endl;
		auto flower_ids = system.get_missed_flower_ids();
		cout << "Missed tasks:";
		for (auto id : flower_ids)
			cout << " " << id;
		cout << endl;
		// cout << "Remaining tasks: " << system.get_num_of_remaining_tasks() << endl;
		cout << "Objective: " << system.get_objective() << endl;
		std::ofstream output;
		output.open(vm["output"].as<std::string>() + "/MAPF_results.txt", std::ios::out);
		output << "Overall runtime: " << runtime << " seconds." << endl;;
		output << "Makespan: " << system.get_makespan() << " timesteps." << endl;
		output << "Flowtime: " << system.get_flowtime() << " timesteps." << endl;
		output << "Flowtime lowerbound: " << system.get_flowtime_lowerbound() << " timesteps." << endl;
		output << "Missed tasks:";
		for (auto id : flower_ids)
			output << " " << id;
		output << endl;
		output << "Objective: " << system.get_objective() << endl;
		output.close();
        return 0;
	}
    else if (scenario == "WORKSTATION")
    {
        WorkstationGrid G;
        if (!G.load_map(benchmark_path))
            return -1;
        int agent_count = vm["agentNum"].as<int>();
        if (agent_count > (int)G.free_start_cells.size())
        {
            std::cerr << "WORKSTATION requested " << agent_count << " agents, but benchmark has only "
                      << G.free_start_cells.size() << " valid start cells" << endl;
            return -1;
        }
        MAPFSolver* solver = set_solver(G, vm);
        WorkstationSystem system(G, *solver);
        set_parameters(system, vm);
        system.workstation_service_time = vm["service_time"].as<int>();
        system.station_policy = vm["station_policy"].as<string>();
        system.pibt_policy = vm["pibt_policy"].as<string>();
        system.workstation_pressure_threshold = vm["pressure_threshold"].as<int>();
        system.stop_at_traffic_jam = vm["stop_at_traffic_jam"].as<bool>();
        G.preprocessing(system.consider_rotation);
        system.simulate(vm["simulation_time"].as<int>());
        return 0;
    }
	else
	{
		cout << "Scenario " << scenario << " does not exist!" << endl;
		return -1;
	}
}
