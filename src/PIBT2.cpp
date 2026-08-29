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
    MAPFSolver(G, path_planner),
    workstation_grid(dynamic_cast<const WorkstationGrid*>(&G)),
    random_engine(0) {}

void PIBT2::set_pibt_policy(const string& policy)
{
    pibt_policy = policy;
    if (policy == "departure_aware")
        selected_policy = Policy::DEPARTURE_AWARE;
    else if (policy == "pressure_aware")
        selected_policy = Policy::PRESSURE_AWARE;
    else
        selected_policy = Policy::VANILLA;
}

PIBT2::Policy PIBT2::active_policy() const
{
    return selected_policy;
}

void PIBT2::clear()
{
    runtime = 0;
    solution_found = false;
    solution_cost = -2;
    avg_path_length = -1;
    min_sum_of_costs = 0;
    for (Path& path : solution)
        path.clear();
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
    solution.resize(num_of_agents);
    const size_t path_capacity = static_cast<size_t>(std::max(1, window)) + 1;
    for (Path& path : solution)
    {
        path.clear();
        path.reserve(path_capacity);
    }
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
    const size_t cache_size = static_cast<size_t>(std::max(2, window + 1));
    if (step_random_cache.size() != cache_size ||
        !step_random_cache_seed_valid || step_random_cache_seed != tie_seed)
    {
        step_random_cache.assign(cache_size, StepRandomCacheEntry());
        step_random_cache_seed = tie_seed;
        step_random_cache_seed_valid = true;
    }

    StepRandomCacheEntry& cached =
        step_random_cache[absolute_timestep % cache_size];
    if (!cached.valid || cached.timestep != absolute_timestep)
    {
        std::seed_seq seed{
            static_cast<uint32_t>(tie_seed),
            static_cast<uint32_t>(tie_seed >> 32),
            static_cast<uint32_t>(absolute_timestep),
            static_cast<uint32_t>(absolute_timestep >> 32),
            0x50494254U,
        };
        cached.engine.seed(seed);
        cached.timestep = absolute_timestep;
        cached.valid = true;
    }
    random_engine = cached.engine;
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
    if (agent >= 0 && agent < static_cast<int>(pressure_contexts.size()))
    {
        return workstation_policy_protects_phase(
            pibt_policy, pressure_contexts[agent].phase);
    }
    return workstation_policy_protects_phase(
        pibt_policy, active_context(agent).phase);
}

void PIBT2::update_pressure_state(const vector<State>& current_states)
{
    pressure_contexts.clear();
    pressure_penalty_station.assign(current_states.size(), -1);
    if (active_policy() != Policy::PRESSURE_AWARE)
        return;
    if (workstation_grid == nullptr)
        return;

    pressure_locations_scratch.resize(current_states.size());
    pressure_contexts.reserve(current_states.size());
    for (int agent = 0; agent < static_cast<int>(current_states.size()); agent++)
    {
        pressure_locations_scratch[agent] = current_states[agent].location;
        pressure_contexts.push_back(active_context(agent));
    }
    evaluate_workstation_pressure(
        *workstation_grid, pressure_locations_scratch, pressure_contexts,
        pressure_snapshot, pressure_workspace);

    for (int agent = 0; agent < static_cast<int>(pressure_contexts.size()); agent++)
    {
        const WorkstationAgentContext& context = pressure_contexts[agent];
        const int station_id = context.station_id;
        if (context.phase != WorkstationAgentPhase::TO_STATION ||
            station_id < 0 ||
            station_id >= static_cast<int>(workstation_grid->stations.size()) ||
            station_id >= static_cast<int>(pressure_snapshot.station_pressure.size()) ||
            !workstation_pressure_active(
                pressure_snapshot.station_pressure[station_id]))
        {
            continue;
        }

        const vector<int>& privileged =
            pressure_snapshot.privileged_inbound_agents[station_id];
        if (std::find(privileged.begin(), privileged.end(), agent) ==
            privileged.end())
        {
            pressure_penalty_station[agent] = station_id;
        }
    }
}

bool PIBT2::must_hold_for_workstation_service(int agent, const State& state) const
{
    if (agent < 0 || agent >= (int)workstation_context.size())
        return false;
    if (workstation_grid == nullptr)
        return false;

    const WorkstationAgentContext context = active_context(agent);
    if (context.phase != WorkstationAgentPhase::SERVICE &&
        context.phase != WorkstationAgentPhase::TO_STATION)
    {
        return false;
    }
    if (context.station_id < 0 ||
        context.station_id >= (int)workstation_grid->stations.size())
        return false;
    if (state.location !=
        workstation_grid->stations[context.station_id].workstation)
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

PIBT2::CandidateList PIBT2::ranked_candidates(int agent, int local_t)
{
    const State& current = agents[agent].current;
    CandidateList candidates;
    auto push_candidate = [&](const State& candidate) {
        candidates.states[candidates.size++] = candidate;
    };

    if (current_target(agent) < 0)
    {
        State wait = current;
        wait.timestep = local_t;
        push_candidate(wait);
        return candidates;
    }

    if (agent < (int)initial_paths.size() && !initial_paths[agent].empty() &&
        local_t < (int)initial_paths[agent].size())
    {
        State committed = initial_paths[agent][local_t];
        committed.timestep = local_t;
        push_candidate(committed);
        return candidates;
    }

    if (current.location >= 0 && current.orientation >= 0)
    {
        push_candidate(State(current.location, local_t, current.orientation));
        if (G.valid_move(current.location, current.orientation))
        {
            push_candidate(State(
                current.location + G.move[current.orientation], local_t,
                current.orientation));
        }
        const int next_orientation1 = (current.orientation + 1) % 4;
        const int next_orientation2 = (current.orientation + 3) % 4;
        push_candidate(State(current.location, local_t, next_orientation1));
        push_candidate(State(current.location, local_t, next_orientation2));
    }
    else if (current.location >= 0)
    {
        push_candidate(State(current.location, local_t));
        for (int direction = 0; direction < 4; direction++)
        {
            if (G.valid_move(current.location, direction))
            {
                push_candidate(State(
                    current.location + G.move[direction], local_t));
            }
        }
    }

    if (random_tiebreak)
    {
        std::shuffle(
            candidates.states.begin(),
            candidates.states.begin() + candidates.size,
            random_engine);
    }
    else
    {
        std::sort(
            candidates.states.begin(),
            candidates.states.begin() + candidates.size,
            [](const State& lhs, const State& rhs) {
                if (lhs.location != rhs.location)
                    return lhs.location < rhs.location;
                return lhs.orientation < rhs.orientation;
            });
    }

    struct RankedCandidate
    {
        State state;
        int distance;
        long long policy_score;
        bool occupied;
    };
    const bool pressure_aware = active_policy() == Policy::PRESSURE_AWARE;
    const int pressure_station =
        pressure_aware && agent >= 0 &&
        agent < static_cast<int>(pressure_penalty_station.size()) ?
            pressure_penalty_station[agent] : -1;
    std::array<RankedCandidate, 5> ranked;
    for (size_t index = 0; index < candidates.size; index++)
    {
        const State& candidate = candidates.states[index];
        const int distance = distance_to_target(agent, candidate.location);
        int cost = 0;
        if (pressure_station >= 0)
        {
            const bool in_station_zone =
                workstation_grid->has_complete_zone_index() ?
                    workstation_grid->station_for_zone_cell(candidate.location) ==
                        pressure_station :
                    workstation_grid->stations[pressure_station].zone_cells.find(
                        candidate.location) !=
                        workstation_grid->stations[pressure_station].zone_cells.end();
            if (in_station_zone)
                cost = kWorkstationPressureQueueCost;
        }
        ranked[index] = {
            candidate,
            distance,
            static_cast<long long>(distance) + cost,
            occupied_now[candidate.location] >= 0,
        };
    }

    auto canonical_less = [](const RankedCandidate& lhs,
                             const RankedCandidate& rhs) {
        if (lhs.distance != rhs.distance)
            return lhs.distance < rhs.distance;
        if (lhs.occupied != rhs.occupied)
            return !lhs.occupied;
        return false;
    };
    auto policy_less = [](const RankedCandidate& lhs,
                          const RankedCandidate& rhs) {
        if (lhs.policy_score != rhs.policy_score)
            return lhs.policy_score < rhs.policy_score;
        if (lhs.occupied != rhs.occupied)
            return !lhs.occupied;
        return false;
    };

    if (!pressure_aware)
    {
        std::sort(ranked.begin(), ranked.begin() + candidates.size, canonical_less);
        for (size_t index = 0; index < candidates.size; index++)
            candidates.states[index] = ranked[index].state;
        return candidates;
    }

    const RankedCandidate canonical_front = *std::min_element(
        ranked.begin(), ranked.begin() + candidates.size, canonical_less);
    std::sort(ranked.begin(), ranked.begin() + candidates.size, policy_less);
    if (candidates.size > 0 &&
        canonical_front.state.location != ranked.front().state.location)
    {
        pressure_rank_changes++;
    }
    for (size_t index = 0; index < candidates.size; index++)
        candidates.states[index] = ranked[index].state;
    return candidates;
}

bool PIBT2::func_pibt(int agent, int parent, int local_t)
{
    const CandidateList candidates = ranked_candidates(agent, local_t);
    for (size_t index = 0; index < candidates.size; index++)
    {
        const State& candidate = candidates.states[index];
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
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        const int current_loc = current_states[agent].location;
        const int next_loc = next_states[agent].location;
        if (current_loc < 0 || current_loc >= G.size() ||
            next_loc < 0 || next_loc >= G.size())
        {
            return false;
        }
        if (occupied_now[current_loc] != agent || occupied_next[next_loc] != agent ||
            violates_initial_constraint(agent, next_loc, next_states[agent].timestep))
        {
            return false;
        }
    }
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        const int current_loc = current_states[agent].location;
        const int next_loc = next_states[agent].location;
        if (current_loc == next_loc)
            continue;
        const int other = occupied_now[next_loc];
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
    order_scratch.clear();
    order_scratch.reserve(num_of_agents);
    protected_class_scratch.assign(num_of_agents, 0);
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        if (!agents[agent].has_next)
            order_scratch.push_back(agent);
    }
    if (active_policy() != Policy::VANILLA)
    {
        for (int agent = 0; agent < num_of_agents; agent++)
            protected_class_scratch[agent] = departure_protected(agent);
    }
    std::sort(order_scratch.begin(), order_scratch.end(), [&](int lhs, int rhs) {
        if (protected_class_scratch[lhs] != protected_class_scratch[rhs])
            return protected_class_scratch[lhs] != 0;
        if (agents[lhs].elapsed != agents[rhs].elapsed)
            return agents[lhs].elapsed > agents[rhs].elapsed;
        if (agents[lhs].initial_distance != agents[rhs].initial_distance)
            return agents[lhs].initial_distance > agents[rhs].initial_distance;
        return agents[lhs].tie_breaker > agents[rhs].tie_breaker;
    });

    for (int agent : order_scratch)
    {
        if (!agents[agent].has_next)
            func_pibt(agent, -1, local_t);
    }

    next_states_scratch.resize(num_of_agents);
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        if (!agents[agent].has_next)
            return false;
        next_states_scratch[agent] = agents[agent].next;
        next_states_scratch[agent].timestep = local_t;
    }
    if (!validate_step(current_states, next_states_scratch))
        return false;

    for (int agent = 0; agent < num_of_agents; agent++)
        solution[agent].push_back(next_states_scratch[agent]);
    update_dynamic_priorities(next_states_scratch, local_t);
    current_states = next_states_scratch;
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
