#pragma once

#include "MAPFSolver.h"
#include "WorkstationCommon.h"
#include "WorkstationPolicy.h"

#include <cstdint>
#include <ctime>

class PIBT : public MAPFSolver
{
public:
    string pibt_policy = "vanilla";
    int pressure_privileged_inbound_count =
        kWorkstationPrivilegedInboundCount;
    bool random_tiebreak = true;
    uint64_t tie_seed = 0;
    vector<WorkstationAgentContext> workstation_context;
    vector<vector<WorkstationAgentContext>> projected_goal_context;

    uint64_t inheritance_calls = 0;
    uint64_t backtracks = 0;
    uint64_t wait_fallbacks = 0;
    uint64_t pressure_rank_changes = 0;

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
    void set_workstation_context(const vector<WorkstationAgentContext>& context) { workstation_context = context; }
    void set_projected_goal_context(const vector<vector<WorkstationAgentContext>>& context) { projected_goal_context = context; }
    void set_executed_priority_age(const vector<int>& age) { executed_priority_age = age; }
    const vector<int>& get_executed_priority_age() const { return executed_priority_age; }

private:
    enum class Policy
    {
        VANILLA,
        LEAD_AWARE,
        DEPARTURE_AWARE,
        PRESSURE_AWARE,
    };

    struct Candidate
    {
        State state;
        double score = 0;
        uint64_t tie_break = 0;
        int loc = -1;
        bool wait = false;
    };

    vector<int> priority_age;
    vector<int> executed_priority_age;
    vector<int> goal_indices;
    vector<int> last_goal_advance_timestep;
    vector<int> last_goal_advance_location;
    WorkstationPressureSnapshot pressure_snapshot;
    vector<WorkstationAgentContext> pressure_contexts;
    vector<int> lead_agent_by_station;
    std::clock_t run_start = 0;
    int remaining_assignment_budget = 0;
    int assignment_pass_index = 0;

    Policy active_policy() const;
    bool use_departure_priority() const;
    bool use_lead_priority() const;
    bool lead_protected(int agent) const;
    bool lead_queue_tiebreak_active(int agent, const State& state) const;
    int step_assignment_budget() const;

    void initialize_run_state();
    void update_workstation_policy_state(const vector<State>& current_states);

    void advance_goal_index(int agent, const State& state, int local_t);
    int current_target(int agent) const;
    WorkstationAgentContext active_context(int agent) const;
    double distance_to_remaining_goals(int agent, int loc, int goal_idx) const;
    int distance_between(int from, int to) const;
    bool must_hold_for_workstation_service(int agent, const State& state) const;

    vector<Candidate> ranked_candidates(int agent,
                                        int local_t,
                                        const vector<State>& current_states,
                                        const vector<int>& current_occupant);
    double pressure_score(int agent,
                          const State& candidate,
                          const vector<State>& current_states,
                          const vector<int>& current_occupant) const;

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
                      int forbidden_next_loc);
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
