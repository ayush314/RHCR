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
        pibt->set_pressure_admission(vm["pressure_admission"].as<string>());
        pibt->set_pressure_cost_mode(vm["pressure_cost_mode"].as<string>());
        pibt->pressure_cost_scope = vm["pressure_cost_scope"].as<string>();
        pibt->pressure_cost_activation = vm["pressure_cost_activation"].as<string>();
        pibt->set_pressure_population(vm["pressure_population"].as<string>());
        pibt->pressure_zone_cost = vm["pressure_zone_cost"].as<double>();
        pibt->set_pressure_inbound_limit(vm["pressure_inbound_limit"].as<int>());
        pibt->set_pressure_lookahead_radius(vm["pressure_lookahead_radius"].as<int>());
        pibt->set_network_pressure_fraction(vm["pibt_network_pressure_fraction"].as<int>());
        pibt->set_network_pressure_min_agents_per_station(
            vm["pibt_network_pressure_min_agents_per_station"].as<int>());
		pibt->set_assignment_budget_factor(vm["pibt_assignment_budget_factor"].as<int>());
        pibt->set_pressure_assignment_extension_factor(
			vm["pibt_pressure_assignment_extension_factor"].as<int>());
		pibt->random_tiebreak = vm["pibt_random_tiebreak"].as<bool>();
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
        ("pressure_profile", po::value<string>()->default_value("fixed"), "pressure profile (fixed, prevalence_adaptive)")
        ("station_policy", po::value<string>()->default_value("vanilla"), "workstation planning policy (vanilla, phase_aware, pressure_aware)")
        ("stop_at_traffic_jam", po::value<bool>()->default_value(true), "stop workstation simulations when the traffic-jam detector triggers")
        ("pibt_policy", po::value<string>()->default_value("vanilla"), "PIBT workstation policy (vanilla, phase_aware, pressure_aware)")
        ("pressure_admission", po::value<string>()->default_value("adaptive"), "pressure admission (single, adaptive, scale_adaptive)")
        ("pressure_cost_mode", po::value<string>()->default_value("fixed"), "pressure cost mode (fixed, escalating, occupancy_escalating, priority_only)")
        ("pressure_cost_scope", po::value<string>()->default_value("zone"), "pressure action-cost scope (zone, queue, holding, approach, entry, lookahead)")
        ("pressure_cost_activation", po::value<string>()->default_value("zone"), "pressure soft-cost activation (zone, excess_wip, outside_only, progress_only, wait_only, incumbent_grace, entry_only, enter_only, deeper_only, busy_only)")
        ("pressure_population", po::value<string>()->default_value("inbound_only"), "pressure population (all_phases, inbound_only)")
        ("pressure_zone_cost", po::value<double>()->default_value(1), "soft pressured-zone occupancy cost")
        ("pressure_front_progress_cost", po::value<int>()->default_value(0), "soft cost when the privileged inbound agent does not move closer to its workstation")
        ("pressure_exit_progress_cost", po::value<int>()->default_value(0), "soft cost when an exiting agent does not move closer to a station exit")
        ("pressure_ready_slot_priority", po::value<bool>()->default_value(false), "give a ready pressured-station front runner local precedence over unprotected traffic")
        ("pressure_inbound_limit", po::value<int>()->default_value(3), "base adaptive privileged inbound budget")
        ("pressure_cost_occupancy_threshold", po::value<int>()->default_value(3), "minimum physical station-zone occupancy before applying soft pressure cost")
        ("pressure_cost_horizon", po::value<int>()->default_value(0), "maximum local PBS soft pressure-cost horizon; 0 uses the full planning window")
        ("pressure_cost_horizon_profile", po::value<string>()->default_value("fixed"), "PBS pressure-cost horizon profile (fixed, network_adaptive)")
        ("pressure_local_action_only", po::value<bool>()->default_value(false), "apply PBS soft pressure cost only to agents currently adjacent to the pressured zone")
        ("pressure_front_runner_priority", po::value<bool>()->default_value(false), "let the selected pressure front runner win PBS same-station conflicts")
        ("pressure_front_runner_zone_only", po::value<bool>()->default_value(false), "limit PBS front-runner promotion to conflicts touching the station zone")
        ("pressure_front_runner_ready_priority", po::value<bool>()->default_value(false), "only promote the PBS front runner when it is within one move of the workstation")
        ("pressure_lookahead_radius", po::value<int>()->default_value(0), "near-station WIP pressure lookahead radius; 0 uses zone-only pressure")
        ("pressure_lookahead_profile", po::value<string>()->default_value("fixed"), "lookahead profile (fixed, scale_adaptive)")
        ("pressure_lookahead_min_agents_per_station", po::value<int>()->default_value(40), "scale-adaptive lookahead activation density")
        ("pibt_network_pressure_fraction", po::value<int>()->default_value(25), "minimum percentage of pressured stations for global PIBT front-runner priority")
        ("pibt_network_pressure_min_agents_per_station", po::value<int>()->default_value(0), "minimum agents per station before global PIBT front-runner priority; zero disables the scale gate")
        ("pibt_global_front_runner_priority", po::value<bool>()->default_value(false), "allow selected pressure front runners to precede unrelated agents globally")
        ("pibt_assignment_budget_factor", po::value<int>()->default_value(90), "PIBT assignment search budget per agent")
        ("pibt_pressure_assignment_extension_factor", po::value<int>()->default_value(20), "shared PIBT search extension after exhaustion; zero disables it")
        ("pibt_front_runner_priority", po::value<bool>()->default_value(false), "let selected pressure front runners precede native PIBT priority ordering")
        ("pibt_front_runner_ready_priority", po::value<bool>()->default_value(false), "prioritize a ready pressure front runner above unrelated PIBT agents")
        ("native_failures_only", po::value<bool>()->default_value(false), "disable PBS LRA fallback and commitment repair")
        ("pibt_random_tiebreak", po::value<bool>()->default_value(true), "use a seeded random final PIBT preference tiebreak")
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
        string station_policy = canonical_workstation_policy(vm["station_policy"].as<string>());
        string pibt_policy = canonical_workstation_policy(vm["pibt_policy"].as<string>());
        auto valid_policy = [](const string& policy) {
            return policy == "vanilla" || policy == "phase_aware" ||
                policy == "pressure_aware";
        };
        if ((workstation_solver == "PBS" && !valid_policy(station_policy)) ||
            (workstation_solver == "PIBT" && !valid_policy(pibt_policy)))
        {
            std::cerr << "WORKSTATION policy must be vanilla, phase_aware, or pressure_aware" << endl;
            exit(-1);
        }
        string pressure_admission = vm["pressure_admission"].as<string>();
        if (pressure_admission != "single" && pressure_admission != "adaptive" &&
            pressure_admission != "wide" &&
            pressure_admission != "scale_adaptive")
        {
            std::cerr << "WORKSTATION pressure admission must be single, adaptive, wide, or scale_adaptive" << endl;
            exit(-1);
        }
        string pressure_cost_mode = vm["pressure_cost_mode"].as<string>();
        if (pressure_cost_mode != "fixed" && pressure_cost_mode != "escalating" &&
            pressure_cost_mode != "occupancy_escalating" &&
            pressure_cost_mode != "priority_only")
        {
            std::cerr << "WORKSTATION pressure cost mode must be fixed, escalating, occupancy_escalating, or priority_only" << endl;
            exit(-1);
        }
        string pressure_cost_scope = vm["pressure_cost_scope"].as<string>();
        if (pressure_cost_scope != "zone" && pressure_cost_scope != "queue" &&
            pressure_cost_scope != "holding" && pressure_cost_scope != "approach" &&
            pressure_cost_scope != "entry" &&
            pressure_cost_scope != "lookahead")
        {
            std::cerr << "WORKSTATION pressure cost scope must be zone, queue, holding, approach, entry, or lookahead" << endl;
            exit(-1);
        }
        string pressure_cost_activation = vm["pressure_cost_activation"].as<string>();
        if (pressure_cost_activation != "zone" && pressure_cost_activation != "excess_wip" &&
            pressure_cost_activation != "outside_only" && pressure_cost_activation != "progress_only" &&
            pressure_cost_activation != "wait_only" && pressure_cost_activation != "incumbent_grace" &&
            pressure_cost_activation != "entry_only" &&
            pressure_cost_activation != "enter_only" &&
            pressure_cost_activation != "deeper_only" &&
            pressure_cost_activation != "busy_only")
        {
            std::cerr << "WORKSTATION pressure cost activation must be zone, excess_wip, outside_only, progress_only, wait_only, incumbent_grace, entry_only, enter_only, deeper_only, or busy_only" << endl;
            exit(-1);
        }
        string pressure_population = vm["pressure_population"].as<string>();
        if (pressure_population != "all_phases" && pressure_population != "inbound_only")
        {
            std::cerr << "WORKSTATION pressure population must be all_phases or inbound_only" << endl;
            exit(-1);
        }
        string pressure_profile = vm["pressure_profile"].as<string>();
        if (pressure_profile != "fixed" && pressure_profile != "prevalence_adaptive")
        {
            std::cerr << "WORKSTATION pressure profile must be fixed or prevalence_adaptive" << endl;
            exit(-1);
        }
        double pressure_zone_cost = vm["pressure_zone_cost"].as<double>();
        if (vm["pressure_inbound_limit"].as<int>() < 1 || pressure_zone_cost <= 0 ||
            pressure_zone_cost != static_cast<int>(pressure_zone_cost))
        {
            std::cerr << "WORKSTATION pressure budget and integer zone cost must be positive" << endl;
            exit(-1);
        }
        if (vm["pressure_front_progress_cost"].as<int>() < 0 ||
            vm["pressure_exit_progress_cost"].as<int>() < 0)
        {
            std::cerr << "WORKSTATION progress costs must be nonnegative" << endl;
            exit(-1);
        }
        if (vm["pibt_assignment_budget_factor"].as<int>() < 1)
        {
            std::cerr << "WORKSTATION PIBT assignment budget factor must be positive" << endl;
            exit(-1);
        }
        if (vm["pibt_pressure_assignment_extension_factor"].as<int>() < 0)
        {
            std::cerr << "WORKSTATION pressure assignment extension factor must be nonnegative" << endl;
            exit(-1);
        }
        if (vm["pressure_lookahead_radius"].as<int>() < 0)
        {
            std::cerr << "WORKSTATION pressure lookahead radius must be nonnegative" << endl;
            exit(-1);
        }
        string pressure_lookahead_profile = vm["pressure_lookahead_profile"].as<string>();
        if (pressure_lookahead_profile != "fixed" &&
            pressure_lookahead_profile != "scale_adaptive")
        {
            std::cerr << "WORKSTATION pressure lookahead profile must be fixed or scale_adaptive" << endl;
            exit(-1);
        }
        if (vm["pressure_lookahead_min_agents_per_station"].as<int>() < 1)
        {
            std::cerr << "WORKSTATION scale-adaptive lookahead density must be positive" << endl;
            exit(-1);
        }
        if (vm["pressure_cost_horizon"].as<int>() < 0)
        {
            std::cerr << "WORKSTATION pressure cost horizon must be nonnegative" << endl;
            exit(-1);
        }
        string pressure_cost_horizon_profile = vm["pressure_cost_horizon_profile"].as<string>();
        if (pressure_cost_horizon_profile != "fixed" &&
            pressure_cost_horizon_profile != "network_adaptive")
        {
            std::cerr << "WORKSTATION pressure cost horizon profile must be fixed or network_adaptive" << endl;
            exit(-1);
        }
        if (vm["pibt_network_pressure_fraction"].as<int>() < 1 ||
            vm["pibt_network_pressure_fraction"].as<int>() > 100)
        {
            std::cerr << "WORKSTATION PIBT network pressure fraction must be in [1, 100]" << endl;
            exit(-1);
        }
        if (vm["pibt_network_pressure_min_agents_per_station"].as<int>() < 0)
        {
            std::cerr << "WORKSTATION PIBT network pressure scale gate must be nonnegative" << endl;
            exit(-1);
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
        system.station_policy = canonical_workstation_policy(vm["station_policy"].as<string>());
        system.pibt_policy = canonical_workstation_policy(vm["pibt_policy"].as<string>());
        system.workstation_pressure_threshold = vm["pressure_threshold"].as<int>();
        system.pressure_profile = vm["pressure_profile"].as<string>();
        system.pressure_admission = vm["pressure_admission"].as<string>();
        system.pressure_cost_mode = vm["pressure_cost_mode"].as<string>();
        system.pressure_cost_scope = vm["pressure_cost_scope"].as<string>();
        system.pressure_cost_activation = vm["pressure_cost_activation"].as<string>();
        system.pressure_population = vm["pressure_population"].as<string>();
        system.pressure_zone_cost = (int)vm["pressure_zone_cost"].as<double>();
        system.pressure_front_progress_cost = vm["pressure_front_progress_cost"].as<int>();
        system.pressure_exit_progress_cost = vm["pressure_exit_progress_cost"].as<int>();
        system.pressure_ready_slot_priority = vm["pressure_ready_slot_priority"].as<bool>();
        system.pressure_inbound_limit = vm["pressure_inbound_limit"].as<int>();
        system.pressure_cost_occupancy_threshold = vm["pressure_cost_occupancy_threshold"].as<int>();
        system.pressure_cost_horizon = vm["pressure_cost_horizon"].as<int>();
        system.pressure_cost_horizon_profile = vm["pressure_cost_horizon_profile"].as<string>();
        system.pressure_local_action_only = vm["pressure_local_action_only"].as<bool>();
    system.pressure_front_runner_priority = vm["pressure_front_runner_priority"].as<bool>();
    system.pressure_front_runner_zone_only = vm["pressure_front_runner_zone_only"].as<bool>();
    system.pressure_front_runner_ready_priority = vm["pressure_front_runner_ready_priority"].as<bool>();
        system.pressure_lookahead_radius = vm["pressure_lookahead_radius"].as<int>();
        system.pressure_lookahead_profile = vm["pressure_lookahead_profile"].as<string>();
        system.pressure_lookahead_min_agents_per_station =
            vm["pressure_lookahead_min_agents_per_station"].as<int>();
    system.pibt_network_pressure_fraction = vm["pibt_network_pressure_fraction"].as<int>();
    system.pibt_network_pressure_min_agents_per_station =
        vm["pibt_network_pressure_min_agents_per_station"].as<int>();
    system.pibt_global_front_runner_priority = vm["pibt_global_front_runner_priority"].as<bool>();
    system.pibt_assignment_budget_factor = vm["pibt_assignment_budget_factor"].as<int>();
    system.pibt_pressure_assignment_extension_factor =
        vm["pibt_pressure_assignment_extension_factor"].as<int>();
    system.pibt_front_runner_priority = vm["pibt_front_runner_priority"].as<bool>();
    system.pibt_front_runner_ready_priority = vm["pibt_front_runner_ready_priority"].as<bool>();
        system.native_failures_only = vm["native_failures_only"].as<bool>();
        system.stop_at_traffic_jam = vm["stop_at_traffic_jam"].as<bool>();
        if (solver->get_name() == "PIBT" && G.has_shared_sortation_heuristics())
            G.preprocessing_compact(system.consider_rotation);
        else
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
