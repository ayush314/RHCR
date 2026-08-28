#include "PIBT.h"

#include "WorkstationGraph.h"
#include "WorkstationPolicy.h"

#include <algorithm>
#include <climits>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>

namespace
{
constexpr int kMissingPriorityValue = std::numeric_limits<int>::max() / 4;

uint64_t mix64(uint64_t value)
{
    value += 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31);
}

} // namespace

PIBT::PIBT(const BasicGraph& G, SingleAgentSolver& path_planner) :
    MAPFSolver(G, path_planner) {}

PIBT::Policy PIBT::active_policy() const
{
    if (pibt_policy == "departure_aware")
        return Policy::DEPARTURE_AWARE;
    if (pibt_policy == "pressure_aware")
        return Policy::PRESSURE_AWARE;
    return Policy::VANILLA;
}

bool PIBT::use_departure_priority() const
{
    return uses_workstation_departure_priority(pibt_policy);
}

int PIBT::step_assignment_budget() const
{
    return std::max(20000, num_of_agents * 600);
}

void PIBT::clear()
{
    runtime = 0;
    solution_found = false;
    solution_cost = -2;
    avg_path_length = -1;
    min_sum_of_costs = 0;
    solution.clear();
    inheritance_calls = 0;
    backtracks = 0;
    wait_fallbacks = 0;
    pressure_rank_changes = 0;
}

void PIBT::initialize_run_state()
{
    if ((int)executed_priority_age.size() != num_of_agents)
        executed_priority_age.assign(num_of_agents, 0);
    priority_age = executed_priority_age;
    goal_indices.assign(num_of_agents, 0);
    last_goal_advance_timestep.assign(num_of_agents, -1);
    last_goal_advance_location.assign(num_of_agents, -1);
    solution.assign(num_of_agents, Path());
    for (int i = 0; i < num_of_agents; i++)
        solution[i].push_back(starts[i]);
}

void PIBT::update_pressure_state(const vector<State>& current_states)
{
    pressure_snapshot = WorkstationPressureSnapshot();
    pressure_contexts.clear();
    if (active_policy() != Policy::PRESSURE_AWARE)
        return;
    const auto* grid = dynamic_cast<const WorkstationGrid*>(&G);
    if (grid == nullptr)
        return;

    vector<int> locations;
    locations.reserve(current_states.size());
    pressure_contexts.reserve(current_states.size());
    for (int agent = 0; agent < static_cast<int>(current_states.size()); agent++)
    {
        locations.push_back(current_states[agent].location);
        pressure_contexts.push_back(active_context(agent));
    }
    pressure_snapshot = evaluate_workstation_pressure(
        *grid, locations, pressure_contexts);
}

void PIBT::advance_goal_index(int agent, const State& state, int local_t)
{
    if (goal_indices[agent] >= (int)goal_locations[agent].size())
        return;
    const auto& goal = goal_locations[agent][goal_indices[agent]];
    if (state.location != goal.first || local_t < goal.second ||
        (last_goal_advance_timestep[agent] == local_t &&
         last_goal_advance_location[agent] == state.location))
    {
        return;
    }
    goal_indices[agent]++;
    last_goal_advance_timestep[agent] = local_t;
    last_goal_advance_location[agent] = state.location;
}

int PIBT::current_target(int agent) const
{
    if (agent < 0 || agent >= (int)goal_locations.size())
        return -1;
    int goal_idx = goal_indices[agent];
    if (goal_idx < 0 || goal_idx >= (int)goal_locations[agent].size())
        return -1;
    return goal_locations[agent][goal_idx].first;
}

WorkstationAgentContext PIBT::active_context(int agent) const
{
    if (agent < 0 || agent >= (int)workstation_context.size())
        return WorkstationAgentContext();
    WorkstationAgentContext context = workstation_context[agent];
    if (agent < (int)projected_goal_context.size() &&
        goal_indices[agent] >= 0 &&
        !projected_goal_context[agent].empty())
    {
        context = workstation_context_for_goal(
            context, projected_goal_context[agent], goal_indices[agent]);
    }
    return context;
}

int PIBT::distance_between(int from, int to) const
{
    if (from < 0 || to < 0)
        return kMissingPriorityValue;
    const auto* workstation_grid = dynamic_cast<const WorkstationGrid*>(&G);
    if (workstation_grid != nullptr)
        return workstation_grid->distance_between(from, to);
    auto it = G.heuristics.find(to);
    if (it != G.heuristics.end() && from < (int)it->second.size())
        return (int)it->second[from];
    return G.get_Manhattan_distance(from, to);
}

double PIBT::distance_to_remaining_goals(int agent, int loc, int goal_idx) const
{
    if (goal_idx >= (int)goal_locations[agent].size())
        return 0;

    double dist = 0;
    int from = loc;
    for (int i = goal_idx; i < (int)goal_locations[agent].size(); i++)
    {
        int to = goal_locations[agent][i].first;
        dist += distance_between(from, to);
        from = to;
    }
    return dist;
}

bool PIBT::must_hold_for_workstation_service(int agent, const State& state) const
{
    if (workstation_context.size() != (size_t)num_of_agents)
        return false;
    const auto* grid = dynamic_cast<const WorkstationGrid*>(&G);
    if (grid == nullptr)
        return false;

    const auto ctx = active_context(agent);
    if (ctx.phase != WorkstationAgentPhase::SERVICE &&
        ctx.phase != WorkstationAgentPhase::TO_STATION)
    {
        return false;
    }
    if (ctx.station_id < 0 || ctx.station_id >= (int)grid->stations.size())
        return false;

    int workstation = grid->stations[ctx.station_id].workstation;
    if (state.location != workstation)
        return false;

    int target = current_target(agent);
    return target < 0 || target == workstation;
}

double PIBT::pressure_score(int agent,
                            const State& candidate,
                            const vector<State>& current_states,
                            const vector<int>& current_occupant) const
{
    if (workstation_context.size() != (size_t)num_of_agents)
        return 0;
    const auto* grid = dynamic_cast<const WorkstationGrid*>(&G);
    if (grid == nullptr)
        return 0;

    if (active_policy() != Policy::PRESSURE_AWARE)
        return 0;
    return workstation_pressure_action_cost(
        *grid, pressure_snapshot, pressure_contexts, agent, candidate.location);
}

vector<PIBT::Candidate> PIBT::ranked_candidates(int agent,
                                                int local_t,
                                                const vector<State>& current_states,
                                                const vector<int>& current_occupant)
{
    vector<Candidate> candidates;
    auto set_tie_break = [&](Candidate& candidate) {
        uint64_t key = tie_seed;
        int episode_t = workstation_context.size() == (size_t)num_of_agents
            ? workstation_context[agent].current_t
            : 0;
        key ^= mix64((uint64_t)(agent + 1));
        key ^= mix64((uint64_t)(episode_t + local_t + 1) << 16);
        key ^= mix64((uint64_t)(assignment_pass_index + 1) << 32);
        key ^= mix64((uint64_t)(candidate.loc + 1) << 1);
        candidate.tie_break = mix64(key);
    };
    if (current_target(agent) < 0)
    {
        Candidate wait;
        wait.state = current_states[agent];
        wait.state.timestep = local_t;
        wait.loc = wait.state.location;
        wait.wait = true;
        wait.score = 0;
        set_tie_break(wait);
        candidates.push_back(wait);
        return candidates;
    }

    if (must_hold_for_workstation_service(agent, current_states[agent]))
    {
        Candidate wait;
        wait.state = current_states[agent];
        wait.state.timestep = local_t;
        wait.loc = wait.state.location;
        wait.wait = true;
        wait.score = distance_to_remaining_goals(agent, wait.loc, goal_indices[agent]);
        set_tie_break(wait);
        candidates.push_back(wait);
        return candidates;
    }

    if (agent < (int)initial_paths.size() && local_t < (int)initial_paths[agent].size() &&
        !initial_paths[agent].empty())
    {
        Candidate committed;
        committed.state = initial_paths[agent][local_t];
        committed.state.timestep = local_t;
        committed.loc = committed.state.location;
        committed.wait = committed.loc == current_states[agent].location;
        committed.score = distance_to_remaining_goals(agent, committed.loc, goal_indices[agent]);
        set_tie_break(committed);
        candidates.push_back(committed);
        return candidates;
    }

    for (const State& next : G.get_neighbors(current_states[agent]))
    {
        Candidate candidate;
        candidate.state = next;
        candidate.state.timestep = local_t;
        candidate.loc = next.location;
        candidate.wait = candidate.loc == current_states[agent].location;
        candidate.score = distance_to_remaining_goals(agent, candidate.loc, goal_indices[agent]);
        set_tie_break(candidate);
        candidates.push_back(candidate);
    }

    auto base_candidates = candidates;
    auto base_less = [&](const Candidate& lhs, const Candidate& rhs) {
        if (lhs.score != rhs.score)
            return lhs.score < rhs.score;
        if (random_tiebreak && lhs.tie_break != rhs.tie_break)
            return lhs.tie_break < rhs.tie_break;
        if (lhs.wait != rhs.wait)
            return !lhs.wait;
        return lhs.loc < rhs.loc;
    };
    std::sort(base_candidates.begin(), base_candidates.end(), base_less);

    for (Candidate& candidate : candidates)
    {
        candidate.score += pressure_score(agent, candidate.state, current_states, current_occupant);
    }

    std::sort(candidates.begin(), candidates.end(), base_less);
    if (active_policy() != Policy::VANILLA &&
        !base_candidates.empty() && !candidates.empty() &&
        base_candidates.front().loc != candidates.front().loc)
    {
        pressure_rank_changes++;
    }
    return candidates;
}

bool PIBT::violates_initial_constraint(int agent, int loc, int local_t) const
{
    for (const auto& constraint : initial_constraints)
    {
        int owner = std::get<0>(constraint);
        int constraint_loc = std::get<1>(constraint);
        int constraint_until = std::get<2>(constraint);
        if (owner != agent && loc == constraint_loc &&
            local_t < std::min(window, constraint_until))
        {
            return true;
        }
    }
    return false;
}

bool PIBT::has_edge_swap(int agent,
                         const State& candidate,
                         const vector<State>& current_states,
                         const vector<int>& current_occupant,
                         const vector<State>& next_states,
                         const vector<bool>& assigned) const
{
    if (candidate.location == current_states[agent].location)
        return false;
    if (candidate.location < 0 || candidate.location >= (int)current_occupant.size())
        return true;
    int other = current_occupant[candidate.location];
    return other >= 0 && other != agent && assigned[other] &&
           next_states[other].location == current_states[agent].location;
}

bool PIBT::assign_agent(int agent,
                        int local_t,
                        const vector<State>& current_states,
                        const vector<int>& current_occupant,
                        vector<State>& next_states,
                        vector<int>& next_occupant,
                        vector<bool>& assigned,
                        vector<bool>& visiting,
                        vector<int>& assignment_log,
                        int forbidden_next_loc)
{
    if (assigned[agent])
        return true;
    if (remaining_assignment_budget <= 0)
    {
        backtracks++;
        return false;
    }
    remaining_assignment_budget--;
    if (visiting[agent])
        return false;

    visiting[agent] = true;
    auto candidates = ranked_candidates(
        agent, local_t, current_states, current_occupant);
    for (const Candidate& candidate : candidates)
    {
        if (candidate.loc < 0 || candidate.loc >= (int)next_occupant.size())
            continue;
        if (candidate.loc == forbidden_next_loc)
            continue;
        if (violates_initial_constraint(agent, candidate.loc, local_t))
            continue;

        const size_t assignment_checkpoint = assignment_log.size();
        int occupant = candidate.loc < (int)current_occupant.size() ? current_occupant[candidate.loc] : -1;
        if (occupant >= 0 && occupant != agent && !assigned[occupant])
        {
            inheritance_calls++;
            bool inherited_valid = assign_agent(
                occupant, local_t, current_states, current_occupant,
                next_states, next_occupant, assigned, visiting, assignment_log,
                candidate.loc);
            if (!inherited_valid)
            {
                rollback_assignments(
                    assignment_checkpoint, current_states, next_states,
                    next_occupant, assigned, assignment_log);
                continue;
            }
        }

        if (next_occupant[candidate.loc] >= 0)
        {
            rollback_assignments(
                assignment_checkpoint, current_states, next_states,
                next_occupant, assigned, assignment_log);
            continue;
        }
        if (has_edge_swap(
                agent, candidate.state, current_states, current_occupant,
                next_states, assigned))
        {
            rollback_assignments(
                assignment_checkpoint, current_states, next_states,
                next_occupant, assigned, assignment_log);
            continue;
        }

        next_states[agent] = candidate.state;
        next_states[agent].timestep = local_t;
        assigned[agent] = true;
        next_occupant[candidate.loc] = agent;
        assignment_log.push_back(agent);
        visiting[agent] = false;
        return true;
    }

    visiting[agent] = false;
    backtracks++;
    return false;
}

void PIBT::rollback_assignments(size_t checkpoint,
                                const vector<State>& current_states,
                                vector<State>& next_states,
                                vector<int>& next_occupant,
                                vector<bool>& assigned,
                                vector<int>& assignment_log) const
{
    while (assignment_log.size() > checkpoint)
    {
        int agent = assignment_log.back();
        assignment_log.pop_back();
        int loc = next_states[agent].location;
        if (loc >= 0 && loc < (int)next_occupant.size() && next_occupant[loc] == agent)
            next_occupant[loc] = -1;
        next_states[agent] = current_states[agent];
        assigned[agent] = false;
    }
}

bool PIBT::force_wait(int agent,
                      int local_t,
                      const vector<State>& current_states,
                      vector<State>& next_states,
                      vector<int>& next_occupant,
                      vector<bool>& assigned) const
{
    int loc = current_states[agent].location;
    if (loc < 0 || loc >= (int)next_occupant.size() || next_occupant[loc] >= 0)
        return false;
    if (violates_initial_constraint(agent, loc, local_t))
        return false;

    next_states[agent] = current_states[agent];
    next_states[agent].timestep = local_t;
    assigned[agent] = true;
    next_occupant[loc] = agent;
    return true;
}

bool PIBT::validate_step(const vector<State>& current_states,
                         const vector<State>& next_states) const
{
    vector<int> current_occupant(G.size(), -1);
    vector<int> next_occupant(G.size(), -1);
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        int current_loc = current_states[agent].location;
        int next_loc = next_states[agent].location;
        if (current_loc < 0 || current_loc >= (int)current_occupant.size() ||
            next_loc < 0 || next_loc >= (int)next_occupant.size())
            return false;
        if (current_occupant[current_loc] >= 0 || next_occupant[next_loc] >= 0)
            return false;
        current_occupant[current_loc] = agent;
        next_occupant[next_loc] = agent;
    }
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        int current_loc = current_states[agent].location;
        int next_loc = next_states[agent].location;
        if (current_loc == next_loc)
            continue;
        int other = current_occupant[next_loc];
        if (other >= 0 && other != agent && next_states[other].location == current_loc)
            return false;
    }
    return true;
}

bool PIBT::assignment_pass(int local_t,
                           const vector<State>& current_states,
                           vector<State>& next_states,
                           bool count_fallbacks)
{
    vector<int> current_occupant(G.size(), -1);
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        int loc = current_states[agent].location;
        if (loc >= 0 && loc < (int)current_occupant.size())
            current_occupant[loc] = agent;
    }

    next_states = current_states;
    vector<int> next_occupant(G.size(), -1);
    vector<bool> assigned(num_of_agents, false);
    vector<bool> visiting(num_of_agents, false);
    vector<int> assignment_log;
    assignment_log.reserve(num_of_agents);
    remaining_assignment_budget = step_assignment_budget();

    // Reserve framework-mandated service waits before policy-independent PIBT
    // assignment. These assignments are fixed and are not rolled back.
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        if (!must_hold_for_workstation_service(agent, current_states[agent]))
            continue;
        if (!force_wait(agent, local_t, current_states, next_states,
                        next_occupant, assigned))
        {
            return false;
        }
    }

    vector<int> order;
    order.reserve(num_of_agents);
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        if (!assigned[agent])
            order.push_back(agent);
    }
    std::sort(order.begin(), order.end(), [&](int lhs, int rhs) {
        if (use_departure_priority())
        {
            const auto lhs_phase = active_context(lhs).phase;
            const auto rhs_phase = active_context(rhs).phase;
            if (workstation_protected_precedes(pibt_policy, lhs_phase, rhs_phase))
                return true;
            if (workstation_protected_precedes(pibt_policy, rhs_phase, lhs_phase))
                return false;
        }
        if (priority_age[lhs] != priority_age[rhs])
            return priority_age[lhs] > priority_age[rhs];
        return lhs < rhs;
    });

    for (int agent : order)
    {
        if (assigned[agent])
            continue;
        if (!assign_agent(agent, local_t, current_states, current_occupant,
                          next_states, next_occupant, assigned, visiting,
                          assignment_log, -1))
        {
            if (count_fallbacks)
                wait_fallbacks++;
            if (!force_wait(agent, local_t, current_states, next_states, next_occupant, assigned))
                return false;
        }
    }
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        if (!assigned[agent])
        {
            if (count_fallbacks)
                wait_fallbacks++;
            if (!force_wait(agent, local_t, current_states, next_states, next_occupant, assigned))
                return false;
        }
    }

    if (!validate_step(current_states, next_states))
        return false;

    return true;
}

bool PIBT::plan_one_step(int local_t, vector<State>& current_states)
{
    for (int agent = 0; agent < num_of_agents; agent++)
        advance_goal_index(agent, current_states[agent], local_t - 1);
    update_pressure_state(current_states);
    vector<State> next_states;
    assignment_pass_index = 0;
    if (!assignment_pass(local_t, current_states, next_states, true))
        return false;

    for (int agent = 0; agent < num_of_agents; agent++)
        solution[agent].push_back(next_states[agent]);
    current_states = next_states;
    update_dynamic_priorities(current_states, local_t);
    return true;
}

void PIBT::update_dynamic_priorities(const vector<State>& current_states, int local_t)
{
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        advance_goal_index(agent, current_states[agent], local_t);
        int target = current_target(agent);
        if (target < 0 || current_states[agent].location == target)
            priority_age[agent] = 0;
        else
            priority_age[agent]++;
    }
}

bool PIBT::run(const vector<State>& starts,
               const vector< vector<pair<int, int> > >& goal_locations,
               int time_limit)
{
    clear();
    run_start = std::clock();
    this->starts = starts;
    this->goal_locations = goal_locations;
    this->num_of_agents = (int)starts.size();
    this->time_limit = time_limit;

    initialize_run_state();

    vector<State> current_states = starts;
    int horizon = std::max(1, window);
    if (horizon > 10000)
        horizon = 10000;

    bool success = true;
    for (int local_t = 1; local_t <= horizon; local_t++)
    {
        runtime = (std::clock() - run_start) * 1.0 / CLOCKS_PER_SEC;
        if (runtime > time_limit || !plan_one_step(local_t, current_states))
        {
            success = false;
            break;
        }
    }

    while (success && (int)solution.front().size() <= horizon)
    {
        int local_t = (int)solution.front().size();
        for (int agent = 0; agent < num_of_agents; agent++)
        {
            State wait = solution[agent].back();
            wait.timestep = local_t;
            solution[agent].push_back(wait);
        }
    }

    runtime = (std::clock() - run_start) * 1.0 / CLOCKS_PER_SEC;
    solution_found = success;
    solution_cost = success ? 0 : -1;
    avg_path_length = 0;
    min_sum_of_costs = 0;
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        avg_path_length += solution[agent].size();
        solution_cost += success ? (double)solution[agent].size() - 1 : 0;
        int from = starts[agent].location;
        for (const auto& goal : goal_locations[agent])
        {
            min_sum_of_costs += distance_between(from, goal.first);
            from = goal.first;
        }
    }
    if (num_of_agents > 0)
        avg_path_length /= num_of_agents;

    print_results();
    return success;
}

void PIBT::print_results() const
{
    std::cout << "PIBT:";
    if (solution_found)
        std::cout << "Succeed,";
    else
        std::cout << "Failed,";
    std::cout << runtime << ","
              << inheritance_calls << ","
              << backtracks << ","
              << wait_fallbacks << ","
              << pressure_rank_changes << ","
              << solution_cost << ","
              << min_sum_of_costs << ","
              << avg_path_length << std::endl;
}

void PIBT::save_results(const std::string& fileName, const std::string& instanceName) const
{
    std::ofstream stats;
    stats.open(fileName, std::ios::app);
    stats << runtime << ","
          << inheritance_calls << ","
          << backtracks << ","
          << wait_fallbacks << ","
          << pressure_rank_changes << ","
          << solution_cost << ","
          << min_sum_of_costs << ","
          << avg_path_length << ","
          << "0" << ","
          << instanceName << std::endl;
    stats.close();
}
