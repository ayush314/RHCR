/*
 * Adapted from Kei18/pibt2 at commit
 * faab5b916649549f1cd563df8dbf6e4f6382f631.
 * Copyright 2021 Keisuke Okumura. Used under the MIT License.
 */

#include "PIBT2.h"

#include "WorkstationGraph.h"
#include "WorkstationPolicy.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>

namespace
{
constexpr int kMissingDistance = std::numeric_limits<int>::max() / 4;

} // namespace

PIBT2::PIBT2(const BasicGraph& G, SingleAgentSolver& path_planner) :
    MAPFSolver(G, path_planner), random_engine(0) {}

PIBT2::Policy PIBT2::active_policy() const
{
    if (pibt_policy == "departure_aware")
        return Policy::DEPARTURE_AWARE;
    if (pibt_policy == "pressure_aware")
        return Policy::PRESSURE_AWARE;
    return Policy::VANILLA;
}

void PIBT2::clear()
{
    runtime = 0;
    solution_found = false;
    solution_cost = -2;
    avg_path_length = -1;
    min_sum_of_costs = 0;
    solution.clear();
    agents.clear();
    occupied_now.clear();
    occupied_next.clear();
    inheritance_calls = 0;
    backtracks = 0;
    wait_fallbacks = 0;
    pressure_rank_changes = 0;
}

bool PIBT2::initialize_run_state()
{
    if ((int)executed_priority_age.size() != num_of_agents)
        executed_priority_age.assign(num_of_agents, 0);
    const bool initialize_priorities =
        (int)priority_initial_distance.size() != num_of_agents ||
        (int)priority_target.size() != num_of_agents ||
        (int)priority_tie_breaker.size() != num_of_agents;
    if (initialize_priorities)
    {
        priority_initial_distance.assign(num_of_agents, 0);
        priority_target.assign(num_of_agents, -1);
        priority_tie_breaker.assign(num_of_agents, 0);
    }

    goal_indices.assign(num_of_agents, 0);
    last_goal_advance_timestep.assign(num_of_agents, -1);
    last_goal_advance_location.assign(num_of_agents, -1);
    occupied_now.assign(G.size(), -1);
    occupied_next.assign(G.size(), -1);
    agents.resize(num_of_agents);
    solution.assign(num_of_agents, Path());
    std::mt19937 priority_engine(
        static_cast<std::mt19937::result_type>(tie_seed));
    std::uniform_real_distribution<float> tie_distribution(0.0f, 1.0f);

    for (int i = 0; i < num_of_agents; i++)
    {
        const int loc = starts[i].location;
        if (loc < 0 || loc >= G.size() || occupied_now[loc] >= 0)
            return false;

        advance_goal_index(i, starts[i], 0);
        Agent& agent = agents[i];
        agent.id = i;
        agent.current = starts[i];
        agent.next = State();
        agent.has_next = false;
        agent.elapsed = executed_priority_age[i];
        const int target = current_target(i);
        if (initialize_priorities)
            priority_tie_breaker[i] = random_tiebreak ? tie_distribution(priority_engine) : 0;
        if (initialize_priorities || priority_target[i] != target)
        {
            priority_target[i] = target;
            priority_initial_distance[i] = distance_to_target(i, loc);
        }
        agent.initial_distance = priority_initial_distance[i];
        agent.tie_breaker = random_tiebreak ? priority_tie_breaker[i] : 0;
        occupied_now[loc] = i;
        solution[i].push_back(starts[i]);
    }
    return true;
}

void PIBT2::seed_step_random_engine(int local_t)
{
    const uint64_t absolute_timestep = static_cast<uint64_t>(
        std::max(0, episode_start_timestep) + std::max(1, local_t));
    std::seed_seq seed{
        static_cast<uint32_t>(tie_seed),
        static_cast<uint32_t>(tie_seed >> 32),
        static_cast<uint32_t>(absolute_timestep),
        static_cast<uint32_t>(absolute_timestep >> 32),
        0x50494254U,
    };
    random_engine.seed(seed);
}

bool PIBT2::advance_goal_index(int agent, const State& state, int local_t)
{
    if (goal_indices[agent] >= (int)goal_locations[agent].size())
        return false;
    const auto& goal = goal_locations[agent][goal_indices[agent]];
    if (state.location != goal.first || local_t < goal.second ||
        (last_goal_advance_timestep[agent] == local_t &&
         last_goal_advance_location[agent] == state.location))
    {
        return false;
    }
    goal_indices[agent]++;
    last_goal_advance_timestep[agent] = local_t;
    last_goal_advance_location[agent] = state.location;
    return true;
}

int PIBT2::current_target(int agent) const
{
    if (agent < 0 || agent >= (int)goal_locations.size())
        return -1;
    const int goal_index = goal_indices[agent];
    if (goal_index < 0 || goal_index >= (int)goal_locations[agent].size())
        return -1;
    return goal_locations[agent][goal_index].first;
}

WorkstationAgentContext PIBT2::active_context(int agent) const
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

int PIBT2::distance_between(int from, int to) const
{
    if (from < 0 || to < 0)
        return kMissingDistance;
    const auto* workstation_grid = dynamic_cast<const WorkstationGrid*>(&G);
    if (workstation_grid != nullptr)
        return workstation_grid->distance_between(from, to);
    const auto heuristic = G.heuristics.find(to);
    if (heuristic != G.heuristics.end() && from < (int)heuristic->second.size())
        return static_cast<int>(heuristic->second[from]);
    return G.get_Manhattan_distance(from, to);
}

int PIBT2::distance_to_target(int agent, int loc) const
{
    const int target = current_target(agent);
    return target < 0 ? 0 : distance_between(loc, target);
}

bool PIBT2::departure_protected(int agent) const
{
    return workstation_policy_protects_phase(
        pibt_policy, active_context(agent).phase);
}

void PIBT2::update_pressure_state(const vector<State>& current_states)
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

int PIBT2::pressure_cost(int agent, int candidate_loc) const
{
    if (active_policy() != Policy::PRESSURE_AWARE)
        return 0;
    const auto* grid = dynamic_cast<const WorkstationGrid*>(&G);
    if (grid == nullptr || agent < 0 || agent >= (int)workstation_context.size())
        return 0;

    return workstation_pressure_action_cost(
        *grid, pressure_snapshot, pressure_contexts, agent, candidate_loc);
}

bool PIBT2::must_hold_for_workstation_service(int agent, const State& state) const
{
    if (agent < 0 || agent >= (int)workstation_context.size())
        return false;
    const auto* grid = dynamic_cast<const WorkstationGrid*>(&G);
    if (grid == nullptr)
        return false;

    const WorkstationAgentContext context = active_context(agent);
    if (context.phase != WorkstationAgentPhase::SERVICE &&
        context.phase != WorkstationAgentPhase::TO_STATION)
    {
        return false;
    }
    if (context.station_id < 0 || context.station_id >= (int)grid->stations.size())
        return false;
    if (state.location != grid->stations[context.station_id].workstation)
        return false;

    const int target = current_target(agent);
    return target < 0 || target == state.location;
}

bool PIBT2::violates_initial_constraint(int agent, int loc, int local_t) const
{
    for (const auto& constraint : initial_constraints)
    {
        const int owner = std::get<0>(constraint);
        const int constraint_loc = std::get<1>(constraint);
        const int constraint_until = std::get<2>(constraint);
        if (owner != agent && loc == constraint_loc &&
            local_t < std::min(window, constraint_until))
        {
            return true;
        }
    }
    return false;
}

vector<State> PIBT2::ranked_candidates(int agent, int local_t)
{
    const State& current = agents[agent].current;
    vector<State> candidates;

    if (current_target(agent) < 0 ||
        must_hold_for_workstation_service(agent, current))
    {
        State wait = current;
        wait.timestep = local_t;
        candidates.push_back(wait);
        return candidates;
    }

    if (agent < (int)initial_paths.size() && !initial_paths[agent].empty() &&
        local_t < (int)initial_paths[agent].size())
    {
        State committed = initial_paths[agent][local_t];
        committed.timestep = local_t;
        candidates.push_back(committed);
        return candidates;
    }

    for (State candidate : G.get_neighbors(current))
    {
        candidate.timestep = local_t;
        candidates.push_back(candidate);
    }

    if (random_tiebreak)
        std::shuffle(candidates.begin(), candidates.end(), random_engine);
    else
        std::sort(candidates.begin(), candidates.end(), [](const State& lhs, const State& rhs) {
            if (lhs.location != rhs.location)
                return lhs.location < rhs.location;
            return lhs.orientation < rhs.orientation;
        });

    auto canonical_less = [&](const State& lhs, const State& rhs) {
        const int lhs_distance = distance_to_target(agent, lhs.location);
        const int rhs_distance = distance_to_target(agent, rhs.location);
        if (lhs_distance != rhs_distance)
            return lhs_distance < rhs_distance;
        const bool lhs_occupied = occupied_now[lhs.location] >= 0;
        const bool rhs_occupied = occupied_now[rhs.location] >= 0;
        if (lhs_occupied != rhs_occupied)
            return !lhs_occupied;
        return false;
    };
    auto policy_less = [&](const State& lhs, const State& rhs) {
        const long long lhs_distance = static_cast<long long>(
            distance_to_target(agent, lhs.location)) + pressure_cost(agent, lhs.location);
        const long long rhs_distance = static_cast<long long>(
            distance_to_target(agent, rhs.location)) + pressure_cost(agent, rhs.location);
        if (lhs_distance != rhs_distance)
            return lhs_distance < rhs_distance;
        const bool lhs_occupied = occupied_now[lhs.location] >= 0;
        const bool rhs_occupied = occupied_now[rhs.location] >= 0;
        if (lhs_occupied != rhs_occupied)
            return !lhs_occupied;
        return false;
    };

    if (active_policy() != Policy::PRESSURE_AWARE)
    {
        std::sort(candidates.begin(), candidates.end(), canonical_less);
        return candidates;
    }

    const State canonical_front = *std::min_element(
        candidates.begin(), candidates.end(), canonical_less);
    std::sort(candidates.begin(), candidates.end(), policy_less);
    if (!candidates.empty() && canonical_front.location != candidates.front().location)
    {
        pressure_rank_changes++;
    }
    return candidates;
}

bool PIBT2::func_pibt(int agent, int parent, int local_t)
{
    const vector<State> candidates = ranked_candidates(agent, local_t);
    for (const State& candidate : candidates)
    {
        const int loc = candidate.location;
        if (loc < 0 || loc >= G.size())
            continue;
        if (occupied_next[loc] >= 0)
            continue;
        if (parent >= 0 && loc == agents[parent].current.location)
            continue;
        if (violates_initial_constraint(agent, loc, local_t))
            continue;

        occupied_next[loc] = agent;
        agents[agent].next = candidate;
        agents[agent].has_next = true;

        const int occupant = occupied_now[loc];
        if (occupant >= 0 && !agents[occupant].has_next)
        {
            inheritance_calls++;
            if (!func_pibt(occupant, agent, local_t))
                continue;
        }
        return true;
    }

    // This is PIBT's native backtracking result, not an external fallback.
    const int current_loc = agents[agent].current.location;
    occupied_next[current_loc] = agent;
    agents[agent].next = agents[agent].current;
    agents[agent].next.timestep = local_t;
    agents[agent].has_next = true;
    backtracks++;
    return false;
}

bool PIBT2::validate_step(const vector<State>& current_states,
                          const vector<State>& next_states) const
{
    vector<int> current_occupant(G.size(), -1);
    vector<int> next_occupant(G.size(), -1);
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        const int current_loc = current_states[agent].location;
        const int next_loc = next_states[agent].location;
        if (current_loc < 0 || current_loc >= G.size() ||
            next_loc < 0 || next_loc >= G.size())
        {
            return false;
        }
        if (current_occupant[current_loc] >= 0 || next_occupant[next_loc] >= 0 ||
            violates_initial_constraint(agent, next_loc, next_states[agent].timestep))
        {
            return false;
        }
        current_occupant[current_loc] = agent;
        next_occupant[next_loc] = agent;
    }
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        const int current_loc = current_states[agent].location;
        const int next_loc = next_states[agent].location;
        if (current_loc == next_loc)
            continue;
        const int other = current_occupant[next_loc];
        if (other >= 0 && other != agent && next_states[other].location == current_loc)
            return false;
    }
    return true;
}

void PIBT2::update_dynamic_priorities(
    const vector<State>& next_states, int local_t)
{
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        const int target = current_target(agent);
        const bool reached_target = target < 0 || next_states[agent].location == target;
        const bool advanced = advance_goal_index(agent, next_states[agent], local_t);
        if (advanced)
            agents[agent].initial_distance = distance_to_target(agent, next_states[agent].location);
        if (reached_target || advanced || current_target(agent) < 0)
            agents[agent].elapsed = 0;
        else
            agents[agent].elapsed++;
    }
}

bool PIBT2::plan_one_step(int local_t, vector<State>& current_states)
{
    // Key candidate shuffling to committed simulation time so discarded RHCR
    // lookahead cannot restart or advance randomness for later replans.
    seed_step_random_engine(local_t);

    for (int agent = 0; agent < num_of_agents; agent++)
    {
        if (advance_goal_index(agent, current_states[agent], local_t - 1))
            agents[agent].elapsed = 0;
        agents[agent].current = current_states[agent];
        agents[agent].next = State();
        agents[agent].has_next = false;
    }

    std::fill(occupied_now.begin(), occupied_now.end(), -1);
    std::fill(occupied_next.begin(), occupied_next.end(), -1);
    for (int agent = 0; agent < num_of_agents; agent++)
        occupied_now[current_states[agent].location] = agent;

    // SERVICE dwell is a framework-mandated action, so reserve it before
    // applying solver or workstation-policy priorities.
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        if (!must_hold_for_workstation_service(agent, current_states[agent]))
            continue;
        const int loc = current_states[agent].location;
        if (loc < 0 || loc >= G.size() || occupied_next[loc] >= 0 ||
            violates_initial_constraint(agent, loc, local_t))
        {
            return false;
        }
        agents[agent].next = current_states[agent];
        agents[agent].next.timestep = local_t;
        agents[agent].has_next = true;
        occupied_next[loc] = agent;
    }

    update_pressure_state(current_states);
    vector<int> order;
    order.reserve(num_of_agents);
    vector<char> protected_class(num_of_agents, 0);
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        if (!agents[agent].has_next)
            order.push_back(agent);
    }
    if (active_policy() != Policy::VANILLA)
    {
        for (int agent = 0; agent < num_of_agents; agent++)
            protected_class[agent] = departure_protected(agent);
    }
    std::sort(order.begin(), order.end(), [&](int lhs, int rhs) {
        if (protected_class[lhs] != protected_class[rhs])
            return protected_class[lhs] != 0;
        if (agents[lhs].elapsed != agents[rhs].elapsed)
            return agents[lhs].elapsed > agents[rhs].elapsed;
        if (agents[lhs].initial_distance != agents[rhs].initial_distance)
            return agents[lhs].initial_distance > agents[rhs].initial_distance;
        return agents[lhs].tie_breaker > agents[rhs].tie_breaker;
    });

    for (int agent : order)
    {
        if (!agents[agent].has_next)
            func_pibt(agent, -1, local_t);
    }

    vector<State> next_states(num_of_agents);
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        if (!agents[agent].has_next)
            return false;
        next_states[agent] = agents[agent].next;
        next_states[agent].timestep = local_t;
    }
    if (!validate_step(current_states, next_states))
        return false;

    for (int agent = 0; agent < num_of_agents; agent++)
        solution[agent].push_back(next_states[agent]);
    update_dynamic_priorities(next_states, local_t);
    current_states = next_states;
    return true;
}

bool PIBT2::run(const vector<State>& starts,
                const vector<vector<pair<int, int>>>& goal_locations,
                int time_limit)
{
    clear();
    run_start = std::clock();
    this->starts = starts;
    this->goal_locations = goal_locations;
    num_of_agents = static_cast<int>(starts.size());
    this->time_limit = time_limit;

    bool success = initialize_run_state();
    vector<State> current_states = starts;
    int horizon = std::max(1, window);
    if (horizon > 10000)
        horizon = 10000;

    for (int local_t = 1; success && local_t <= horizon; local_t++)
    {
        runtime = (std::clock() - run_start) * 1.0 / CLOCKS_PER_SEC;
        if (runtime > time_limit || !plan_one_step(local_t, current_states))
            success = false;
    }

    runtime = (std::clock() - run_start) * 1.0 / CLOCKS_PER_SEC;
    solution_found = success;
    solution_cost = success ? 0 : -1;
    avg_path_length = 0;
    min_sum_of_costs = 0;
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        avg_path_length += solution[agent].size();
        if (success)
            solution_cost += static_cast<double>(solution[agent].size()) - 1;
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

void PIBT2::print_results() const
{
    std::cout << "PIBT2:" << (solution_found ? "Succeed," : "Failed,")
              << runtime << ","
              << inheritance_calls << ","
              << backtracks << ","
              << wait_fallbacks << ","
              << pressure_rank_changes << ","
              << solution_cost << ","
              << min_sum_of_costs << ","
              << avg_path_length << std::endl;
}

void PIBT2::save_results(
    const string& file_name, const string& instance_name) const
{
    std::ofstream stats(file_name, std::ios::app);
    stats << runtime << ","
          << inheritance_calls << ","
          << backtracks << ","
          << wait_fallbacks << ","
          << pressure_rank_changes << ","
          << solution_cost << ","
          << min_sum_of_costs << ","
          << avg_path_length << ",0,"
          << instance_name << std::endl;
}
