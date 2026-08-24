#pragma once

#include "MAPFSolver.h"
#include "WorkstationCommon.h"

#include <cstdint>
#include <ctime>

class WorkstationGrid;

class PIBT : public MAPFSolver
{
public:
    string pibt_policy = "vanilla";
    string pressure_admission = "adaptive";
    int workstation_pressure_threshold = -1;
    int pressure_cost_occupancy_threshold = 3;
    int pressure_inbound_limit = 3;
    int pressure_lookahead_radius = 0;
    string pressure_lookahead_profile = "fixed";
    int pressure_lookahead_min_agents_per_station = 40;
    int network_pressure_fraction = 25;
    int network_pressure_min_agents_per_station = 0;
    bool global_front_runner_priority = false;
    bool front_runner_priority = false;
    bool front_runner_ready_priority = false;
    bool pressure_ready_slot_priority = false;
    int pressure_front_progress_cost = 0;
    int pressure_exit_progress_cost = 0;
    int assignment_budget_factor = 90;
    int pressure_assignment_extension_factor = 20;
    bool random_tiebreak = true;
    string pressure_cost_mode = "fixed";
    string pressure_cost_scope = "zone";
    string pressure_cost_activation = "zone";
    string pressure_population = "inbound_only";
    string pressure_profile = "fixed";
    uint64_t tie_seed = 0;
    vector<WorkstationAgentContext> workstation_context;
    vector<vector<WorkstationAgentContext>> projected_goal_context;

    double pressure_zone_cost = 1;

    uint64_t inheritance_calls = 0;
    uint64_t backtracks = 0;
    uint64_t wait_fallbacks = 0;
    uint64_t greedy_repairs = 0;
    uint64_t pressure_rank_changes = 0;
    uint64_t pressure_active_agents = 0;
    uint64_t pressure_candidate_hits = 0;
    uint64_t budget_extensions = 0;

    PIBT(const BasicGraph& G, SingleAgentSolver& path_planner);
    ~PIBT() {}

    bool run(const vector<State>& starts,
             const vector< vector<pair<int, int> > >& goal_locations,
             int time_limit);

    string get_name() const { return "PIBT"; }
    void save_results(const std::string& fileName, const std::string& instanceName) const;
    void save_search_tree(const std::string& fileName) const {}
    void save_constraints_in_goal_node(const std::string& fileName) const {}
    void clear();

    void set_pibt_policy(const string& policy) { pibt_policy = canonical_workstation_policy(policy); }
    void set_pressure_admission(const string& admission) { pressure_admission = admission; }
    void set_workstation_pressure_threshold(int threshold) { workstation_pressure_threshold = threshold; }
    void set_pressure_cost_occupancy_threshold(int threshold) { pressure_cost_occupancy_threshold = threshold; }
    void set_pressure_profile(const string& profile) { pressure_profile = profile; }
    void set_pressure_lookahead_radius(int radius) { pressure_lookahead_radius = radius; }
    void set_pressure_lookahead_profile(const string& profile) { pressure_lookahead_profile = profile; }
    void set_pressure_lookahead_min_agents_per_station(int value) { pressure_lookahead_min_agents_per_station = value; }
    void set_network_pressure_fraction(int fraction) { network_pressure_fraction = fraction; }
    void set_network_pressure_min_agents_per_station(int value) { network_pressure_min_agents_per_station = value; }
    void set_global_front_runner_priority(bool enabled) { global_front_runner_priority = enabled; }
    void set_front_runner_priority(bool enabled) { front_runner_priority = enabled; }
    void set_front_runner_ready_priority(bool enabled) { front_runner_ready_priority = enabled; }
    void set_pressure_ready_slot_priority(bool enabled) { pressure_ready_slot_priority = enabled; }
    void set_pressure_front_progress_cost(int value) { pressure_front_progress_cost = value; }
    void set_pressure_exit_progress_cost(int value) { pressure_exit_progress_cost = value; }
    void set_pressure_inbound_limit(int limit) { pressure_inbound_limit = limit; }
    void set_assignment_budget_factor(int factor) { assignment_budget_factor = factor; }
    void set_pressure_assignment_extension_factor(int factor) { pressure_assignment_extension_factor = factor; }
    void set_pressure_cost_mode(const string& mode) { pressure_cost_mode = mode; }
    void set_pressure_population(const string& population) { pressure_population = population; }
    void set_workstation_context(const vector<WorkstationAgentContext>& context) { workstation_context = context; }
    void set_projected_goal_context(const vector<vector<WorkstationAgentContext>>& context) { projected_goal_context = context; }
    void set_executed_priority_age(const vector<int>& age) { executed_priority_age = age; }
    const vector<int>& get_executed_priority_age() const { return executed_priority_age; }

private:
    enum class Policy
    {
        VANILLA,
        PHASE_AWARE,
        PRESSURE_AWARE,
    };

    Policy active_policy_mode = Policy::VANILLA;

    struct Candidate
    {
        State state;
        int base_score = 0;
        int score = 0;
        int pressure_cost = 0;
        uint64_t tie_break = 0;
        int loc = -1;
        bool wait = false;
    };

    vector<int> priority_age;
    vector<int> executed_priority_age;
    vector<int> goal_indices;
    vector<int> last_goal_advance_timestep;
    vector<int> last_goal_advance_location;
    vector<int> station_pressure_values;
    vector<int> station_zone_occupancy_values;
    vector<int> station_pressure_cost_values;
    vector<char> station_service_busy_values;
    int pressure_threshold_cache = 1;
    int pressure_zone_cost_cache = 1;
    int pressure_lookahead_radius_cache = 0;
    bool pressure_state_cache_ready = false;
    vector<vector<int>> station_privileged_agents;
    vector<int> privileged_station_by_agent;
    vector<vector<int>> station_inbound_agents;
    vector<int> pressure_target_station_cache;
    vector<char> pressure_cost_active_cache;
    vector<WorkstationAgentContext> active_context_cache;
    vector<vector<Candidate>> assignment_candidate_cache;
    vector<char> assignment_candidate_cache_ready;
    vector<int> station_by_location_cache;
    vector<int> station_workstation_cache;
    vector<vector<pair<int, int>>> neighbor_cache;
    vector<vector<pair<int, int>>> initial_constraint_cache;
    vector<char> neighbor_cache_ready;
    vector<vector<char>> pressure_lookahead_location_cache;
    vector<vector<char>> pressure_cost_location_cache;
    vector<vector<int8_t>> pressure_action_relevant_cache;
    // Reused by every one-step assignment to avoid reallocating the same
    // occupancy and recursion buffers at large agent counts.
    vector<int> assignment_current_occupant;
    vector<int> assignment_next_occupant;
    // Only current-state locations need explicit sparse clearing. The next
    // occupancy is cleared from the previous complete assignment below.
    vector<int> assignment_current_touched;
    vector<int> assignment_order;
    vector<char> assignment_protected_class;
    vector<int> assignment_station_id;
    vector<char> assignment_to_station;
    vector<char> assignment_front_runner;
    vector<char> assignment_front_runner_ready;
    vector<int> assignment_progress_target;
    vector<int> assignment_progress_current_distance;
    vector<int> assignment_progress_cost;
    vector<char> assignment_assigned;
    vector<char> assignment_visiting;
    vector<int> assignment_log;
    vector<State> assignment_next_states;
    const WorkstationGrid* workstation_grid_cache = nullptr;
    bool network_pressure_active = false;
    std::clock_t run_start = 0;
    int remaining_assignment_budget = 0;
    int remaining_assignment_extension_budget = 0;
    int assignment_pass_index = 0;

    Policy active_policy() const;
    bool use_exit_priority() const;
    int effective_workstation_pressure_threshold() const;
    int step_assignment_budget() const;
    int pressure_assignment_extension_budget() const;

    void initialize_run_state();
    const vector<pair<int, int>>& cached_neighbors(const State& state);
    void refresh_active_context_cache();
    void update_pressure_state(const vector<State>& current_states);
    vector<int> workstation_privileged_agents(int station_id, const vector<State>& current_states) const;
    bool is_station_front_runner(int agent, int station_id) const;
    bool pressure_action_relevant(const State& state, int station_id);
    tuple<int, int, int, int> front_runner_key(int agent, const vector<State>& current_states) const;

    void advance_goal_index(int agent, const State& state, int local_t);
    int current_target(int agent) const;
    WorkstationAgentContext active_context(int agent) const;
    int distance_to_next_goal(int agent, int loc, int goal_idx) const;
    int distance_between(int from, int to) const;
    bool is_station_privileged(int agent, int station_id) const;
    bool must_hold_for_workstation_service(int agent, const State& state) const;

    const vector<Candidate>& ranked_candidates(int agent,
                                               int local_t,
                                               const vector<State>& current_states,
                                               const vector<int>& current_occupant);
    int pressure_score(int agent,
                       int current_location,
                       const State& candidate) const;
    int progress_score(int agent, const State& candidate) const;

    bool violates_initial_constraint(int agent, int loc, int local_t) const;
    bool has_edge_swap(int agent,
                       const State& candidate,
                       const vector<State>& current_states,
                       const vector<int>& current_occupant,
                       const vector<State>& next_states,
                       const vector<char>& assigned) const;
    bool assign_agent(int agent,
                      int local_t,
                      const vector<State>& current_states,
                      const vector<int>& current_occupant,
                      vector<State>& next_states,
                      vector<int>& next_occupant,
                      vector<char>& assigned,
                      vector<char>& visiting,
                      vector<int>& assignment_log,
                      int forbidden_next_loc);
    void rollback_assignments(size_t checkpoint,
                              const vector<State>& current_states,
                              vector<State>& next_states,
                              vector<int>& next_occupant,
                              vector<char>& assigned,
                              vector<int>& assignment_log) const;
    bool force_wait(int agent,
                    int local_t,
                    const vector<State>& current_states,
                    vector<State>& next_states,
                    vector<int>& next_occupant,
                    vector<char>& assigned) const;
    bool greedy_repair(int agent,
                       int local_t,
                       const vector<State>& current_states,
                       const vector<int>& current_occupant,
                       vector<State>& next_states,
                       vector<int>& next_occupant,
                       vector<char>& assigned);
    bool validate_step(const vector<State>& current_states,
                       const vector<State>& next_states,
                       const vector<int>& current_occupant,
                       const vector<int>& next_occupant) const;
    bool assignment_pass(int local_t,
                         const vector<State>& current_states,
                         vector<State>& next_states,
                         bool count_fallbacks);
    bool plan_one_step(int local_t, vector<State>& current_states);
    void update_dynamic_priorities(const vector<State>& current_states, int local_t);
    void print_results() const;
};
