#pragma once
#include "PBSNode.h"
#include "MAPFSolver.h"
#include "WorkstationCommon.h"
#include <ctime>

// TODO: add topological sorting

class PBS:
	public MAPFSolver
{
public:
    bool lazyPriority;
    bool prioritize_start = true;
    string station_policy = "vanilla";
    int workstation_pressure_threshold = -1;
    int workstation_pressure_zone_cost = 1;
    int pressure_inbound_limit = 3;
    int pressure_cost_occupancy_threshold = 3;
    int pressure_cost_horizon = 0;
    string pressure_cost_horizon_profile = "fixed";
    bool pressure_local_action_only = false;
    bool pressure_front_runner_priority = false;
    bool pressure_front_runner_zone_only = false;
    bool pressure_front_runner_ready_priority = false;
    bool pressure_ready_slot_priority = false;
    int pressure_front_progress_cost = 0;
    int pressure_exit_progress_cost = 0;
    int pressure_lookahead_radius = 0;
    string pressure_lookahead_profile = "fixed";
    int pressure_lookahead_min_agents_per_station = 40;
    int network_pressure_fraction = 25;
    string pressure_admission = "adaptive";
    string pressure_cost_mode = "fixed";
    string pressure_cost_scope = "zone";
    string pressure_cost_activation = "zone";
    string pressure_population = "inbound_only";
    string pressure_profile = "fixed";
    vector<WorkstationAgentContext> workstation_context;
    vector<vector<WorkstationAgentContext>> projected_goal_context;
    vector<int> station_pressure_cache;
    vector<int> station_zone_occupancy_cache;
    vector<char> station_service_busy_cache;
    int pressure_threshold_cache = 1;
    int pressure_zone_cost_cache = 1;
    int pressure_lookahead_radius_cache = 0;
    bool pressure_state_cache_ready = false;
    vector<vector<int>> station_privileged_agents_cache;
    vector<vector<char>> station_privileged_flags_cache;
    vector<vector<int>> pressure_zone_cells_cache;
    vector<vector<int>> pressure_lookahead_cells_cache;
    vector<vector<int>> pressure_cost_cells_cache;
    bool pressure_cache_ready = false;
    bool network_pressure_active = false;

	 // runtime breakdown
    double runtime_rt = 0;
    double runtime_plan_paths = 0;
    double runtime_get_higher_priority_agents = 0;
    double runtime_copy_priorities = 0;
    double runtime_detect_conflicts = 0;
    double runtime_copy_conflicts = 0;
    double runtime_choose_conflict = 0;
    double runtime_find_consistent_paths = 0;
    double runtime_find_replan_agents = 0;


	PBSNode* dummy_start = nullptr;
	PBSNode* best_node;

	uint64_t HL_num_expanded = 0;
	uint64_t HL_num_generated = 0;
	uint64_t LL_num_expanded = 0;
	uint64_t LL_num_generated = 0;

	// Runs the algorithm until the problem is solved or time is exhausted 
    bool run(const vector<State>& starts,
            const vector< vector<pair<int, int> > >& goal_locations, // an ordered list of pairs of <location, release time>
            int time_limit);


    PBS(const BasicGraph& G, SingleAgentSolver& path_planner);
	~PBS();

    void update_paths(PBSNode* curr);
	// Save results
	void save_results(const std::string &fileName, const std::string &instanceName) const;
	void save_search_tree(const std::string &fileName) const;
	void save_constraints_in_goal_node(const std::string &fileName) const;

    string get_name() const {return "PBS"; }
    void set_workstation_policy(const string& policy) { station_policy = canonical_workstation_policy(policy); }
    void set_workstation_pressure_threshold(int threshold) { workstation_pressure_threshold = threshold; }
    void set_pressure_profile(const string& profile) { pressure_profile = profile; }
    void set_pressure_lookahead_radius(int radius) { pressure_lookahead_radius = radius; }
    void set_pressure_lookahead_profile(const string& profile) { pressure_lookahead_profile = profile; }
    void set_pressure_lookahead_min_agents_per_station(int value) { pressure_lookahead_min_agents_per_station = value; }
    void set_pressure_cost_horizon(int value) { pressure_cost_horizon = value; }
    void set_pressure_cost_horizon_profile(const string& profile) { pressure_cost_horizon_profile = profile; }
    void set_pressure_local_action_only(bool value) { pressure_local_action_only = value; }
    void set_pressure_front_runner_priority(bool value) { pressure_front_runner_priority = value; }
    void set_pressure_front_runner_zone_only(bool value) { pressure_front_runner_zone_only = value; }
    void set_pressure_front_runner_ready_priority(bool value) { pressure_front_runner_ready_priority = value; }
    void set_pressure_ready_slot_priority(bool value) { pressure_ready_slot_priority = value; }
    void set_pressure_front_progress_cost(int value) { pressure_front_progress_cost = value; }
    void set_pressure_exit_progress_cost(int value) { pressure_exit_progress_cost = value; }
    void set_network_pressure_fraction(int fraction) { network_pressure_fraction = fraction; }
    void set_workstation_context(const vector<WorkstationAgentContext>& context) { workstation_context = context; }
    void set_projected_goal_context(const vector<vector<WorkstationAgentContext>>& context) { projected_goal_context = context; }

	void clear();

	void setRT(bool use_cat, bool prioritize_start)
	{
		rt.use_cat = use_cat;
		rt.prioritize_start = prioritize_start;
	}

private:

    std::vector< Path* > paths;
    list<PBSNode*> allNodes_table;
    list<PBSNode*> dfs;

   //  vector<State> starts;
    // vector< vector<int> > goal_locations;

    std::clock_t start = 0;

	// double focal_w = 1.0;
    unordered_set<pair<int, int>> nogood;

    // SingleAgentICBS astar;


    bool generate_root_node();
    void push_node(PBSNode* node);
    PBSNode* pop_node();

    // high level search
	bool find_path(PBSNode*  node, int ag);
    bool find_consistent_paths(PBSNode* node, int a); // find paths consistent with priorities
    static void resolve_conflict(const Conflict& conflict, PBSNode* n1, PBSNode* n2);
	bool generate_child(PBSNode* child, PBSNode* curr);

	// conflicts
    void remove_conflicts(list<Conflict>& conflicts, int excluded_agent);
    void find_conflicts(const list<Conflict>& old_conflicts, list<Conflict> & new_conflicts, int new_agent);
	void find_conflicts(list<Conflict> & conflicts, int a1, int a2);
    void find_conflicts(list<Conflict> & new_conflicts, int new_agent);
    void find_conflicts(list<Conflict> & new_conflicts);

	void choose_conflict(PBSNode &parent);
	void copy_conflicts(const list<Conflict>& conflicts, list<Conflict>& copy, int excluded_agent);
    void copy_conflicts(const list<Conflict>& conflicts,
                       list<Conflict>& copy, const vector<bool>& excluded_agents);

    double get_path_cost(const Path& path) const;
	
    // update information
    //void collect_constraints(const boost::unordered_set<int>& agents, int current_agent);
    void get_solution();

    void update_CAT(int ex_ag); // update conflict avoidance table
	void update_focal_list();
	inline void release_closed_list();
    void update_best_node(PBSNode* node);

	// print and save
	void print_paths() const;
	void print_results() const;
	static void print_conflicts(const PBSNode &curr) ;


	// validate
	bool validate_solution();
    static bool validate_consistence(const list<Conflict>& conflicts, const PriorityGraph &G) ;


    // tools
    static bool wait_at_start(const Path& path, int start_location, int timestep) ;
    void find_replan_agents(PBSNode* node, const list<Conflict>& conflicts,
            unordered_set<int>& replan);
    bool prefer_workstation_branch(const Conflict& conflict, pair<int, int>& preferred_priority) const;
    WorkstationAgentContext projected_context_at(int agent, int local_t) const;
    tuple<int, int, int, int> pressure_privilege_key(int agent, int station_id) const;
    vector<int> workstation_privileged_agents(int station_id, int pressure) const;
    int effective_workstation_pressure_threshold() const;
    int workstation_pressure(int station_id) const;
    void initialize_pressure_cache();
    bool workstation_pressure_active(int pressure) const;
    bool maybe_add_workstation_softzone(ReservationTable& rt, int agent);
    bool maybe_add_workstation_progress_cost(ReservationTable& rt, int agent);
};
