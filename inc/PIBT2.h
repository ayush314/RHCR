#pragma once

#include "MAPFSolver.h"
#include "WorkstationCommon.h"
#include "WorkstationPolicy.h"

#include <cstdint>
#include <ctime>
#include <random>

class WorkstationGrid;

// RHCR adapter for Kei18/pibt2. The canonical PIBT priority, preference,
// inheritance, and backtracking behavior is retained in the vanilla policy.
class PIBT2 : public MAPFSolver
{
public:
    string pibt_policy = "vanilla";
    bool random_tiebreak = true;
    uint64_t tie_seed = 0;
    vector<WorkstationAgentContext> workstation_context;
    vector<vector<WorkstationAgentContext>> projected_goal_context;

    uint64_t inheritance_calls = 0;
    uint64_t backtracks = 0;
    uint64_t wait_fallbacks = 0;
    uint64_t pressure_rank_changes = 0;

    PIBT2(const BasicGraph& G, SingleAgentSolver& path_planner);
    ~PIBT2() {}

    bool run(const vector<State>& starts,
             const vector<vector<pair<int, int>>>& goal_locations,
             int time_limit);

    string get_name() const { return "PIBT2"; }
    void save_results(const string& file_name, const string& instance_name) const;
    void save_search_tree(const string&) const {}
    void save_constraints_in_goal_node(const string&) const {}
    void clear();

    void set_pibt_policy(const string& policy);
    void set_episode_start_timestep(int timestep)
    {
        episode_start_timestep = timestep;
    }
    void set_workstation_context(const vector<WorkstationAgentContext>& context)
    {
        workstation_context = context;
    }
    void set_projected_goal_context(
        const vector<vector<WorkstationAgentContext>>& context)
    {
        projected_goal_context = context;
    }
    void set_executed_priority_age(const vector<int>& age)
    {
        executed_priority_age = age;
    }
    const vector<int>& get_executed_priority_age() const
    {
        return executed_priority_age;
    }
    const vector<int>& get_priority_initial_distance() const
    {
        return priority_initial_distance;
    }

private:
    enum class Policy
    {
        VANILLA,
        DEPARTURE_AWARE,
        PRESSURE_AWARE,
    };

    struct Agent
    {
        int id = -1;
        State current;
        State next;
        bool has_next = false;
        int elapsed = 0;
        int initial_distance = 0;
        float tie_breaker = 0;
    };

    vector<Agent> agents;
    vector<int> occupied_now;
    vector<int> occupied_next;
    vector<int> executed_priority_age;
    vector<int> priority_initial_distance;
    vector<int> priority_target;
    vector<float> priority_tie_breaker;
    vector<int> goal_indices;
    vector<int> last_goal_advance_timestep;
    vector<int> last_goal_advance_location;
    WorkstationPressureSnapshot pressure_snapshot;
    vector<WorkstationAgentContext> pressure_contexts;
    const WorkstationGrid* workstation_grid;
    std::mt19937 random_engine;
    int episode_start_timestep = 0;
    std::clock_t run_start = 0;
    Policy selected_policy = Policy::VANILLA;

    Policy active_policy() const;
    bool departure_protected(int agent) const;
    bool initialize_run_state();
    void seed_step_random_engine(int local_t);
    bool plan_one_step(int local_t, vector<State>& current_states);
    bool func_pibt(int agent, int parent, int local_t);
    vector<State> ranked_candidates(int agent, int local_t);
    bool validate_step(const vector<State>& current_states,
                       const vector<State>& next_states) const;

    bool advance_goal_index(int agent, const State& state, int local_t);
    int current_target(int agent) const;
    WorkstationAgentContext active_context(int agent) const;
    int distance_between(int from, int to) const;
    int distance_to_target(int agent, int loc) const;
    bool must_hold_for_workstation_service(int agent, const State& state) const;
    bool violates_initial_constraint(int agent, int loc, int local_t) const;

    void update_pressure_state(const vector<State>& current_states);
    int pressure_cost(int agent, int candidate_loc) const;

    void update_dynamic_priorities(const vector<State>& next_states, int local_t);
    void print_results() const;
};
