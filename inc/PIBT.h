#pragma once

#include "MAPFSolver.h"
#include "WorkstationCommon.h"

#include <cstdint>
#include <ctime>

class PIBT : public MAPFSolver
{
public:
    string pibt_policy = "vanilla";
    string pressure_profile = "thirds";
    string hindrance_scope = "inherited";
    int workstation_pressure_threshold = -1;
    int pressure_inbound_limit = 4;
    bool hindrance_tiebreak = true;
    int regret_iterations = 1;
    double regret_weight = 0.5;
    string regret_scope = "all";
    bool random_tiebreak = true;
    uint64_t tie_seed = 0;
    bool front_priority_enabled = true;
    bool phase_priority_enabled = false;
    vector<WorkstationAgentContext> workstation_context;

    double wait_penalty = 2;
    double exit_bonus = 1;
    double front_bonus = 3;
    double soft_collision_penalty = 0;
    double pressure_entry_penalty = 2;

    uint64_t inheritance_calls = 0;
    uint64_t backtracks = 0;
    uint64_t wait_fallbacks = 0;
    uint64_t pressure_rank_changes = 0;
    uint64_t regret_updates = 0;

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

    void set_pibt_policy(const string& policy) { pibt_policy = policy; }
    void set_pressure_profile(const string& profile) { pressure_profile = profile; }
    void set_hindrance_scope(const string& scope) { hindrance_scope = scope; }
    void set_workstation_pressure_threshold(int threshold) { workstation_pressure_threshold = threshold; }
    void set_pressure_inbound_limit(int limit) { pressure_inbound_limit = limit; }
    void set_workstation_context(const vector<WorkstationAgentContext>& context) { workstation_context = context; }

private:
    enum class Policy
    {
        VANILLA,
        DISTANCE_AGE,
        PRESSURE,
    };

    struct Candidate
    {
        State state;
        double score = 0;
        int hindrance = 0;
        double regret = 0;
        double base_distance = 0;
        uint64_t tie_break = 0;
        int loc = -1;
        bool wait = false;
    };

    vector<int> priority_age;
    vector<int> goal_indices;
    vector<int> last_goal_advance_timestep;
    vector<int> last_goal_advance_location;
    vector<int> station_pressure_values;
    vector<int> station_front_runners;
    vector<int> station_entry_runners;
    vector<vector<double>> regret_values;
    vector<bool> regret_agent_enabled;
    std::clock_t run_start = 0;
    int remaining_assignment_budget = 0;
    int assignment_pass_index = 0;

    Policy active_policy() const;
    bool use_exit_priority() const;
    bool use_front_runner() const;
    bool use_full_terms() const;
    bool use_regret() const;
    bool regret_applies_to_agent(int agent) const;
    void update_regret_scope(const vector<State>& current_states);
    int effective_workstation_pressure_threshold() const;
    int step_assignment_budget() const;

    void initialize_run_state();
    void update_pressure_state(const vector<State>& current_states);
    int workstation_front_runner(int station_id) const;
    int workstation_front_runner(int station_id, const vector<State>& current_states) const;
    int workstation_entry_runner(int station_id, const vector<State>& current_states) const;
    tuple<int, int, int, int> front_runner_key(int agent, const vector<State>& current_states) const;
    tuple<int, int, int> distance_age_key(int agent, const vector<State>& current_states) const;

    void advance_goal_index(int agent, const State& state, int local_t);
    int current_target(int agent) const;
    double distance_to_remaining_goals(int agent, int loc, int goal_idx) const;
    int distance_between(int from, int to) const;
    int agent_class_rank(int agent) const;
    bool is_front_runner(int agent) const;
    bool is_station_privileged(int agent, int station_id) const;
    bool must_hold_for_workstation_service(int agent, const State& state) const;

    vector<Candidate> ranked_candidates(int agent,
                                        int local_t,
                                        const vector<State>& current_states,
                                        const vector<int>& current_occupant,
                                        bool inherited);
    double pressure_score(int agent,
                          const State& candidate,
                          const vector<State>& current_states,
                          const vector<int>& current_occupant) const;
    int hindrance_score(int agent,
                        const State& candidate,
                        const vector<State>& current_states,
                        const vector<int>& current_occupant,
                        bool inherited) const;
    double self_regret(const vector<Candidate>& candidates, int chosen_loc) const;
    void update_regret(int agent, int loc, double observed_regret);

    bool violates_initial_constraint(int agent, int loc, int local_t) const;
    bool has_edge_swap(int agent,
                       const State& candidate,
                       const vector<State>& current_states,
                       const vector<int>& current_occupant,
                       const vector<State>& next_states,
                       const vector<bool>& assigned) const;
    bool assign_agent(int agent,
                      int local_t,
                      const vector<State>& current_states,
                      const vector<int>& current_occupant,
                      vector<State>& next_states,
                      vector<int>& next_occupant,
                      vector<bool>& assigned,
                      vector<bool>& visiting,
                      vector<int>& assignment_log,
                      int forbidden_next_loc,
                      double& propagated_regret);
    void rollback_assignments(size_t checkpoint,
                              const vector<State>& current_states,
                              vector<State>& next_states,
                              vector<int>& next_occupant,
                              vector<bool>& assigned,
                              vector<int>& assignment_log) const;
    bool force_wait(int agent,
                    int local_t,
                    const vector<State>& current_states,
                    vector<State>& next_states,
                    vector<int>& next_occupant,
                    vector<bool>& assigned) const;
    bool validate_step(const vector<State>& current_states,
                       const vector<State>& next_states) const;
    bool assignment_pass(int local_t,
                         const vector<State>& current_states,
                         vector<State>& next_states,
                         bool count_fallbacks);
    bool plan_one_step(int local_t, vector<State>& current_states);
    void update_dynamic_priorities(const vector<State>& current_states, int local_t);
    void print_results() const;
};
