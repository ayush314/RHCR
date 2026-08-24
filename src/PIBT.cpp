#include "PIBT.h"

#include "WorkstationGraph.h"

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
    return active_policy_mode;
}

bool PIBT::use_exit_priority() const
{
    if (active_policy() == Policy::VANILLA)
        return false;
    return true;
}

int PIBT::effective_workstation_pressure_threshold() const
{
    if (pressure_state_cache_ready && workstation_pressure_threshold <= 0)
        return pressure_threshold_cache;
    return workstation_pressure_threshold_for_profile(
        pressure_profile, workstation_pressure_threshold, station_pressure_values);
}

int PIBT::step_assignment_budget() const
{
    return std::max(20000, num_of_agents * std::max(1, assignment_budget_factor));
}

int PIBT::pressure_assignment_extension_budget() const
{
    // Keep the bounded search profile identical across the policy ladder;
    // pressure changes candidate preference, not the solver's search budget.
    if (pressure_assignment_extension_factor <= 0)
        return 0;
    return num_of_agents * pressure_assignment_extension_factor;
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
    greedy_repairs = 0;
    pressure_rank_changes = 0;
    pressure_active_agents = 0;
    pressure_candidate_hits = 0;
    budget_extensions = 0;
    active_policy_mode = Policy::VANILLA;
    active_context_cache.clear();
    pressure_threshold_cache = 1;
    pressure_zone_cost_cache = 1;
    station_zone_occupancy_values.clear();
    station_pressure_cost_values.clear();
    station_service_busy_values.clear();
    pressure_lookahead_radius_cache = 0;
    pressure_state_cache_ready = false;
    assignment_candidate_cache.clear();
    assignment_candidate_cache_ready.clear();
    station_by_location_cache.clear();
    station_workstation_cache.clear();
    privileged_station_by_agent.clear();
    pressure_target_station_cache.clear();
    pressure_cost_active_cache.clear();
    station_inbound_agents.clear();
    neighbor_cache.clear();
    neighbor_cache_ready.clear();
    pressure_lookahead_location_cache.clear();
    pressure_action_relevant_cache.clear();
    initial_constraint_cache.clear();
    assignment_current_occupant.clear();
    assignment_next_occupant.clear();
    assignment_current_touched.clear();
    assignment_order.clear();
    assignment_protected_class.clear();
    assignment_front_runner.clear();
    assignment_front_runner_ready.clear();
    assignment_progress_target.clear();
    assignment_progress_current_distance.clear();
    assignment_progress_cost.clear();
    assignment_assigned.clear();
    assignment_visiting.clear();
    assignment_log.clear();
    assignment_next_states.clear();
    workstation_grid_cache = nullptr;
    network_pressure_active = false;
}

void PIBT::initialize_run_state()
{
    const string policy = canonical_workstation_policy(pibt_policy);
    if (policy == "phase_aware")
        active_policy_mode = Policy::PHASE_AWARE;
    else if (is_pressure_aware_policy(policy))
        active_policy_mode = Policy::PRESSURE_AWARE;
    else
        active_policy_mode = Policy::VANILLA;
    if ((int)executed_priority_age.size() != num_of_agents)
        executed_priority_age.assign(num_of_agents, 0);
    priority_age = executed_priority_age;
    goal_indices.assign(num_of_agents, 0);
    last_goal_advance_timestep.assign(num_of_agents, -1);
    last_goal_advance_location.assign(num_of_agents, -1);
    assignment_candidate_cache.assign(num_of_agents, vector<Candidate>());
    // Four moves plus wait are the bounded candidate degree of the grid.
    for (auto& candidates : assignment_candidate_cache)
        candidates.reserve(5);
    assignment_candidate_cache_ready.assign(num_of_agents, 0);
    workstation_grid_cache = dynamic_cast<const WorkstationGrid*>(&G);
    const auto* grid = workstation_grid_cache;
    station_by_location_cache.assign(G.size(), -1);
    station_workstation_cache.assign(
        grid != nullptr ? grid->stations.size() : 0, -1);
    neighbor_cache.assign(G.size() * 5, vector<pair<int, int>>());
    neighbor_cache_ready.assign(G.size() * 5, 0);
    pressure_action_relevant_cache.assign(
        grid != nullptr ? grid->stations.size() : 0,
        vector<int8_t>(G.size() * 5, -1));
    pressure_cost_location_cache.assign(
        grid != nullptr ? grid->stations.size() : 0,
        vector<char>(G.size(), 0));
    initial_constraint_cache.assign(G.size(), vector<pair<int, int>>());
    for (const auto& constraint : initial_constraints)
    {
        const int location = std::get<1>(constraint);
        if (location >= 0 && location < G.size())
            initial_constraint_cache[location].emplace_back(
                std::get<0>(constraint), std::get<2>(constraint));
    }
    assignment_current_occupant.assign(G.size(), -1);
    assignment_next_occupant.assign(G.size(), -1);
    station_inbound_agents.assign(
        grid != nullptr ? grid->stations.size() : 0, vector<int>());
    if (grid != nullptr)
    {
        pressure_lookahead_radius_cache = workstation_effective_pressure_lookahead_radius(
            pressure_lookahead_profile, pressure_lookahead_radius, num_of_agents,
            (int)grid->stations.size(), pressure_lookahead_min_agents_per_station);
        for (const auto& station : grid->stations)
        {
            station_workstation_cache[station.station_id] = station.workstation;
            if (pressure_cost_scope == "holding")
            {
                for (int loc : station.standby_cells)
                    if (loc >= 0 && loc < G.size())
                        pressure_cost_location_cache[station.station_id][loc] = 1;
                for (int loc : station.buffer_cells)
                    if (loc >= 0 && loc < G.size())
                        pressure_cost_location_cache[station.station_id][loc] = 1;
            }
            if (pressure_cost_scope == "approach")
            {
                for (int loc : station.approach_cells)
                    if (loc >= 0 && loc < G.size())
                        pressure_cost_location_cache[station.station_id][loc] = 1;
            }
            if (pressure_cost_scope == "entry")
            {
                for (int loc : station.buffer_cells)
                    if (loc >= 0 && loc < G.size())
                        pressure_cost_location_cache[station.station_id][loc] = 1;
                for (int loc : station.approach_cells)
                    if (loc >= 0 && loc < G.size())
                        pressure_cost_location_cache[station.station_id][loc] = 1;
            }
            for (int loc : station.zone_cells)
            {
                const bool is_exit_cell = std::find(
                    station.exit_cells.begin(), station.exit_cells.end(), loc) !=
                    station.exit_cells.end();
                if (loc >= 0 && loc < (int)station_by_location_cache.size())
                    station_by_location_cache[loc] = station.station_id;
                if (pressure_cost_scope != "holding" &&
                    pressure_cost_scope != "approach" &&
                    pressure_cost_scope != "entry" &&
                    loc >= 0 && loc < G.size() &&
                    loc != station.workstation &&
                    (pressure_cost_scope != "queue" || !is_exit_cell))
                    pressure_cost_location_cache[station.station_id][loc] = 1;
            }
        }
        if (pressure_lookahead_radius_cache > 0)
        {
            pressure_lookahead_location_cache.assign(
                grid->stations.size(), vector<char>(G.size(), 0));
            for (size_t station_id = 0; station_id < grid->stations.size(); station_id++)
            {
                const int workstation = grid->stations[station_id].workstation;
                auto& lookahead = pressure_lookahead_location_cache[station_id];
                for (int loc = 0; loc < G.size(); loc++)
                {
                    if (loc != workstation &&
                        grid->distance_to_workstation((int)station_id, loc) <=
                            pressure_lookahead_radius_cache)
                        lookahead[loc] = 1;
                }
            }
        }
    }
    solution.assign(num_of_agents, Path());
    for (int i = 0; i < num_of_agents; i++)
        solution[i].push_back(starts[i]);
}

const vector<pair<int, int>>& PIBT::cached_neighbors(const State& state)
{
    static const vector<pair<int, int>> empty;
    if (state.location < 0 || state.location >= G.size() ||
        state.orientation < -1 || state.orientation > 3)
        return empty;

    const int key = state.location * 5 + state.orientation + 1;
    if (!neighbor_cache_ready[key])
    {
        for (const State& next : G.get_neighbors(state))
            neighbor_cache[key].emplace_back(next.location, next.orientation);
        neighbor_cache_ready[key] = 1;
    }
    return neighbor_cache[key];
}

tuple<int, int, int, int> PIBT::front_runner_key(int agent,
                                                 const vector<State>& current_states) const
{
    const WorkstationAgentContext& ctx = active_context_cache[agent];
    int inside_rank = 1;
    int dist = kMissingPriorityValue;
    const auto* grid = workstation_grid_cache;
    if (grid != nullptr && ctx.station_id >= 0 && agent < (int)current_states.size())
    {
        const int location_station = current_states[agent].location >= 0 &&
            current_states[agent].location < (int)station_by_location_cache.size()
            ? station_by_location_cache[current_states[agent].location]
            : -1;
        const bool boundary_seen = ctx.boundary_entry_t >= 0 ||
            location_station == ctx.station_id;
        inside_rank = boundary_seen ? 0 : 1;
        dist = grid->distance_to_workstation(ctx.station_id, current_states[agent].location);
    }
    int task_issue = ctx.task_issue_t >= 0 ? ctx.task_issue_t : kMissingPriorityValue;
    return workstation_privilege_key(inside_rank == 0, dist, task_issue, agent);
}

void PIBT::update_pressure_state(const vector<State>& current_states)
{
    const auto* grid = workstation_grid_cache;
    pressure_state_cache_ready = false;
    pressure_threshold_cache = 1;
    pressure_zone_cost_cache = 1;
    if (grid == nullptr)
    {
        station_pressure_values.clear();
        station_pressure_cost_values.clear();
        station_privileged_agents.clear();
        privileged_station_by_agent.clear();
        station_inbound_agents.clear();
        return;
    }

    const size_t station_count = grid->stations.size();
    if (station_pressure_values.size() != station_count)
        station_pressure_values.assign(station_count, 0);
    else
        std::fill(station_pressure_values.begin(), station_pressure_values.end(), 0);
    if (station_zone_occupancy_values.size() != station_count)
        station_zone_occupancy_values.assign(station_count, 0);
    else
        std::fill(station_zone_occupancy_values.begin(),
                  station_zone_occupancy_values.end(), 0);
    if (station_service_busy_values.size() != station_count)
        station_service_busy_values.assign(station_count, 0);
    else
        std::fill(station_service_busy_values.begin(),
                  station_service_busy_values.end(), 0);
    if (station_privileged_agents.size() != station_count)
        station_privileged_agents.resize(station_count);
    for (auto& agents : station_privileged_agents)
        agents.clear();
    if (privileged_station_by_agent.size() != (size_t)num_of_agents)
        privileged_station_by_agent.assign(num_of_agents, -1);
    else
        std::fill(privileged_station_by_agent.begin(),
                  privileged_station_by_agent.end(), -1);
    if (station_inbound_agents.size() != station_count)
        station_inbound_agents.resize(station_count);
    for (auto& agents : station_inbound_agents)
        agents.clear();
    const Policy policy = active_policy();
    if (pressure_target_station_cache.size() != (size_t)num_of_agents)
        pressure_target_station_cache.assign(num_of_agents, -1);
    else
        std::fill(pressure_target_station_cache.begin(),
                  pressure_target_station_cache.end(), -1);
    if (pressure_cost_active_cache.size() != (size_t)num_of_agents)
        pressure_cost_active_cache.assign(num_of_agents, 0);
    else
        std::fill(pressure_cost_active_cache.begin(),
                  pressure_cost_active_cache.end(), 0);
    if (policy != Policy::PRESSURE_AWARE)
        return;
    for (const State& state : current_states)
    {
        if (state.location < 0 || state.location >= (int)station_by_location_cache.size())
            continue;
        const int station_id = station_by_location_cache[state.location];
        if (station_id < 0 || station_id >= (int)grid->stations.size() ||
            state.location == grid->stations[station_id].workstation)
            continue;
        station_zone_occupancy_values[station_id]++;
    }
    for (int agent = 0; agent < (int)current_states.size(); agent++)
    {
        const auto& context = active_context_cache[agent];
        if (context.station_id < 0 || context.station_id >= (int)grid->stations.size())
            continue;
        const auto& station = grid->stations[context.station_id];
        if (current_states[agent].location == station.workstation &&
            (context.phase == WorkstationAgentPhase::SERVICE ||
             context.phase == WorkstationAgentPhase::TO_STATION))
            station_service_busy_values[context.station_id] = 1;
    }
    const bool include_protected_phases = pressure_population != "inbound_only";
    for (int agent = 0; agent < (int)current_states.size(); agent++)
    {
        const auto& context = active_context_cache[agent];
        const int station_id = context.station_id;
        if (context.phase == WorkstationAgentPhase::TO_STATION &&
            station_id >= 0 && station_id < (int)grid->stations.size())
            station_inbound_agents[station_id].push_back(agent);
        if (station_id < 0 || station_id >= (int)grid->stations.size() ||
            !contributes_to_workstation_pressure(
                context, station_id, include_protected_phases))
            continue;
        const State& state = current_states[agent];
        const bool in_zone = state.location >= 0 &&
            state.location < (int)station_by_location_cache.size() &&
            state.location != grid->stations[station_id].workstation &&
            station_by_location_cache[state.location] == station_id;
        const bool in_lookahead = pressure_lookahead_radius_cache > 0 &&
            context.phase == WorkstationAgentPhase::TO_STATION &&
            state.location != grid->stations[station_id].workstation &&
            (station_id < (int)pressure_lookahead_location_cache.size() &&
             state.location >= 0 && state.location < G.size()
                ? pressure_lookahead_location_cache[station_id][state.location] != 0
                : grid->distance_to_workstation(station_id, state.location) <=
                    pressure_lookahead_radius_cache);
        if (in_zone || in_lookahead)
            station_pressure_values[station_id]++;
    }
    const int threshold = workstation_pressure_threshold_for_profile(
        pressure_profile, workstation_pressure_threshold, station_pressure_values);
    pressure_threshold_cache = threshold;
    pressure_zone_cost_cache = workstation_pressure_zone_cost_for_profile(
        pressure_profile, (int)pressure_zone_cost, threshold);
    station_pressure_cost_values.assign(station_count, 0);
    for (size_t station_id = 0; station_id < station_count; station_id++)
    {
        const int zone_capacity = std::max(
            1, (int)grid->stations[station_id].zone_cells.size() -
                (grid->stations[station_id].zone_cells.find(
                    grid->stations[station_id].workstation) !=
                    grid->stations[station_id].zone_cells.end() ? 1 : 0));
        const bool escalating = workstation_pressure_cost_escalates(
            pressure_cost_mode, station_pressure_values[station_id], threshold,
            station_zone_occupancy_values[station_id], zone_capacity);
        station_pressure_cost_values[station_id] = workstation_pressure_cost(
            pressure_zone_cost_cache,
            station_pressure_values[station_id],
            threshold,
            escalating);
    }
    pressure_state_cache_ready = true;
    if (policy == Policy::PRESSURE_AWARE)
    {
        for (size_t station_id = 0; station_id < grid->stations.size(); station_id++)
        {
            if (station_pressure_values[station_id] >= threshold)
            {
                station_privileged_agents[station_id] =
                    workstation_privileged_agents((int)station_id, current_states);
                for (int agent : station_privileged_agents[station_id])
                {
                    if (agent >= 0 && agent < num_of_agents)
                        privileged_station_by_agent[agent] = (int)station_id;
                }
            }

        }

        // A global front-runner boost is useful only when congestion is
        // widespread. At lower densities, keep front-runner ordering local
        // to the station so unrelated queues do not compete globally.
        const bool scale_eligible = workstation_network_pressure_scale_eligible(
            num_of_agents, (int)grid->stations.size(),
            network_pressure_min_agents_per_station);
        network_pressure_active = scale_eligible &&
            workstation_network_pressure_active(
                station_pressure_values, threshold, network_pressure_fraction);

    }

    for (int agent = 0; agent < num_of_agents; agent++)
    {
        const auto& ctx = active_context_cache[agent];
        if (ctx.phase == WorkstationAgentPhase::TO_STATION &&
            ctx.station_id >= 0 && ctx.station_id < (int)station_pressure_values.size() &&
            station_pressure_values[ctx.station_id] >= effective_workstation_pressure_threshold())
        {
            const bool privileged = agent >= 0 &&
                agent < (int)privileged_station_by_agent.size() &&
                privileged_station_by_agent[agent] == ctx.station_id;
            if (!privileged)
            {
                pressure_active_agents++;
                if (!pressure_action_relevant(current_states[agent], ctx.station_id))
                    continue;
                const auto& station = grid->stations[ctx.station_id];
                const bool already_inside =
                    current_states[agent].location != station.workstation &&
                    current_states[agent].location >= 0 &&
                    current_states[agent].location < (int)station_by_location_cache.size() &&
                    station_by_location_cache[current_states[agent].location] ==
                        ctx.station_id;
                if (pressure_cost_activation == "outside_only" && already_inside)
                    continue;
                const int zone_occupancy = ctx.station_id <
                    (int)station_zone_occupancy_values.size()
                    ? station_zone_occupancy_values[ctx.station_id] : 0;
                if (!workstation_busy_only_cost_applies(
                        pressure_cost_activation,
                        ctx.station_id < (int)station_service_busy_values.size() &&
                            station_service_busy_values[ctx.station_id]))
                    continue;
                if (!workstation_pressure_cost_active(
                        station_pressure_values[ctx.station_id], zone_occupancy,
                        pressure_cost_occupancy_threshold,
                        pressure_cost_activation))
                    continue;
                pressure_target_station_cache[agent] = ctx.station_id;
                pressure_cost_active_cache[agent] = 1;
            }
        }

    }
}

bool PIBT::pressure_action_relevant(const State& state, int station_id)
{
    const auto* grid = workstation_grid_cache;
    if (grid == nullptr || station_id < 0 || station_id >= (int)grid->stations.size())
        return false;
    const bool cacheable_state = state.location >= 0 && state.location < G.size() &&
        state.orientation >= -1 && state.orientation <= 3;
    const int state_key = cacheable_state
        ? state.location * 5 + state.orientation + 1
        : -1;
    if (state_key >= 0 && station_id < (int)pressure_action_relevant_cache.size())
    {
        const int8_t cached = pressure_action_relevant_cache[station_id][state_key];
        if (cached >= 0)
            return cached != 0;
    }

    const auto& station = grid->stations[station_id];
    bool relevant = false;
    if (state.location == station.workstation)
        relevant = true;
    else if (state.location >= 0 && state.location < (int)station_by_location_cache.size() &&
             station_by_location_cache[state.location] == station_id)
        relevant = true;
    else
    {
        for (const auto& neighbor : cached_neighbors(state))
        {
            const int neighbor_location = neighbor.first;
            if (neighbor_location < 0 ||
                neighbor_location >= (int)station_by_location_cache.size())
                continue;
            const bool neighbor_in_lookahead = pressure_cost_scope == "lookahead" &&
                pressure_lookahead_radius_cache > 0 &&
                (station_id < (int)pressure_lookahead_location_cache.size()
                    ? pressure_lookahead_location_cache[station_id][neighbor_location] != 0
                    : grid->distance_to_workstation(station_id, neighbor_location) <=
                        pressure_lookahead_radius_cache);
            if (station_by_location_cache[neighbor_location] == station_id ||
                neighbor_in_lookahead)
            {
                relevant = true;
                break;
            }
        }
    }
    if (state_key >= 0 && station_id < (int)pressure_action_relevant_cache.size())
        pressure_action_relevant_cache[station_id][state_key] = relevant ? 1 : 0;
    return relevant;
}

vector<int> PIBT::workstation_privileged_agents(int station_id,
                                                const vector<State>& current_states) const
{
    const auto* grid = workstation_grid_cache;
    if (grid == nullptr || station_id < 0 || station_id >= (int)grid->stations.size())
        return vector<int>();

    if (station_id >= (int)station_inbound_agents.size())
        return vector<int>();
    int zone_capacity = std::max(1, (int)grid->stations[station_id].zone_cells.size() - 1);
    int pressure = station_id < (int)station_pressure_values.size()
        ? station_pressure_values[station_id]
        : 0;
    return select_workstation_privileged_agents(
        station_inbound_agents[station_id],
        [&](int agent) { return front_runner_key(agent, current_states); },
        pressure_admission, pressure_inbound_limit, pressure, zone_capacity,
        station_id < (int)station_zone_occupancy_values.size()
            ? station_zone_occupancy_values[station_id] : -1,
        num_of_agents, (int)grid->stations.size(),
        pressure_lookahead_min_agents_per_station);
}

bool PIBT::is_station_front_runner(int agent, int station_id) const
{
    if (active_policy() != Policy::PRESSURE_AWARE || station_id < 0 ||
        station_id >= (int)station_privileged_agents.size() ||
        station_id >= (int)station_pressure_values.size() ||
        station_pressure_values[station_id] < effective_workstation_pressure_threshold())
        return false;
    const int zone_occupancy = station_id <
        (int)station_zone_occupancy_values.size()
        ? station_zone_occupancy_values[station_id] : 0;
    if (!workstation_soft_pressure_active(
            zone_occupancy, pressure_cost_occupancy_threshold))
        return false;
    const auto& privileged = station_privileged_agents[station_id];
    return !privileged.empty() && privileged.front() == agent;
}

void PIBT::advance_goal_index(int agent, const State& state, int local_t)
{
    while (goal_indices[agent] < (int)goal_locations[agent].size())
    {
        const auto& goal = goal_locations[agent][goal_indices[agent]];
        if (state.location != goal.first || local_t < goal.second)
            break;
        if (last_goal_advance_timestep[agent] == local_t &&
            last_goal_advance_location[agent] == state.location)
        {
            break;
        }
        goal_indices[agent]++;
        last_goal_advance_timestep[agent] = local_t;
        last_goal_advance_location[agent] = state.location;
    }
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
    if (agent >= 0 && agent < (int)active_context_cache.size())
        return active_context_cache[agent];
    if (agent < 0 || agent >= (int)workstation_context.size())
        return WorkstationAgentContext();
    WorkstationAgentContext context = workstation_context[agent];
    if (agent < (int)projected_goal_context.size() &&
        goal_indices[agent] >= 0 &&
        goal_indices[agent] < (int)projected_goal_context[agent].size())
    {
        context = projected_goal_context[agent][goal_indices[agent]];
    }
    else if (context.phase == WorkstationAgentPhase::TO_STATION)
    {
        context.phase = WorkstationAgentPhase::SERVICE;
    }
    if (context.phase == WorkstationAgentPhase::TO_STATION && context.task_issue_t < 0 &&
        agent < (int)last_goal_advance_timestep.size() &&
        last_goal_advance_timestep[agent] >= 0)
    {
        context.task_issue_t = context.current_t + last_goal_advance_timestep[agent];
    }
    return context;
}

void PIBT::refresh_active_context_cache()
{
    active_context_cache.assign(num_of_agents, WorkstationAgentContext());
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        if (agent >= (int)workstation_context.size())
            continue;
        WorkstationAgentContext context = workstation_context[agent];
        if (agent < (int)projected_goal_context.size() &&
            goal_indices[agent] >= 0 &&
            goal_indices[agent] < (int)projected_goal_context[agent].size())
        {
            context = projected_goal_context[agent][goal_indices[agent]];
        }
        else if (context.phase == WorkstationAgentPhase::TO_STATION)
        {
            context.phase = WorkstationAgentPhase::SERVICE;
        }
        if (context.phase == WorkstationAgentPhase::TO_STATION && context.task_issue_t < 0 &&
            agent < (int)last_goal_advance_timestep.size() &&
            last_goal_advance_timestep[agent] >= 0)
        {
            context.task_issue_t = context.current_t + last_goal_advance_timestep[agent];
        }
        active_context_cache[agent] = context;
    }
}

int PIBT::distance_between(int from, int to) const
{
    if (from < 0 || to < 0)
        return kMissingPriorityValue;
    const auto* workstation_grid = workstation_grid_cache;
    if (workstation_grid != nullptr)
        return workstation_grid->distance_between(from, to);
    auto it = G.heuristics.find(to);
    if (it != G.heuristics.end() && from < (int)it->second.size())
        return (int)it->second[from];
    return G.get_Manhattan_distance(from, to);
}

int PIBT::distance_to_next_goal(int agent, int loc, int goal_idx) const
{
    if (goal_idx >= (int)goal_locations[agent].size())
        return 0;
    return distance_between(loc, goal_locations[agent][goal_idx].first);
}

bool PIBT::is_station_privileged(int agent, int station_id) const
{
    if (workstation_context.size() != (size_t)num_of_agents)
        return false;
    const auto ctx = active_context(agent);
    if (ctx.station_id != station_id)
        return false;
    if (ctx.phase == WorkstationAgentPhase::SERVICE || ctx.phase == WorkstationAgentPhase::TO_EXIT)
        return true;
    if (active_policy() != Policy::PRESSURE_AWARE || station_id < 0 ||
        station_id >= (int)station_privileged_agents.size())
        return false;
    return agent >= 0 && agent < (int)privileged_station_by_agent.size() &&
        privileged_station_by_agent[agent] == station_id;
}

bool PIBT::must_hold_for_workstation_service(int agent, const State& state) const
{
    if (workstation_context.size() != (size_t)num_of_agents)
        return false;
    const auto* grid = workstation_grid_cache;
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

int PIBT::pressure_score(int agent,
                         int current_location,
                         const State& candidate) const
{
    // Development-only ablation: retain pressure ordering at the assignment
    // level while removing the negative zone cost from candidate ranking.
    if (pressure_cost_mode == "priority_only")
        return 0;
    if (agent < 0 || agent >= (int)pressure_cost_active_cache.size() ||
        !pressure_cost_active_cache[agent])
        return 0;
    const int target_station = pressure_target_station_cache[agent];
    const int target_pressure = target_station >= 0 &&
        target_station < (int)station_pressure_values.size()
        ? station_pressure_values[target_station]
        : -1;
    const int cached_cost = target_station >= 0 &&
        target_station < (int)station_pressure_cost_values.size()
        ? station_pressure_cost_values[target_station]
        : -1;
    const int base_cost = pressure_state_cache_ready
        ? pressure_zone_cost_cache
        : workstation_pressure_zone_cost_for_profile(
            pressure_profile, (int)pressure_zone_cost,
            effective_workstation_pressure_threshold());
    const auto* pressure_grid = workstation_grid_cache;
    const int zone_occupancy = target_station >= 0 &&
        target_station < (int)station_zone_occupancy_values.size()
        ? station_zone_occupancy_values[target_station] : 0;
    int zone_capacity = 1;
    if (pressure_grid != nullptr && target_station >= 0 &&
        target_station < (int)pressure_grid->stations.size())
    {
        const auto& station = pressure_grid->stations[target_station];
        zone_capacity = std::max(
            1, (int)station.zone_cells.size() -
                (station.zone_cells.find(station.workstation) != station.zone_cells.end() ? 1 : 0));
    }
    const bool escalating = workstation_pressure_cost_escalates(
        pressure_cost_mode, target_pressure,
        effective_workstation_pressure_threshold(), zone_occupancy, zone_capacity);
    if (pressure_cost_activation == "wait_only")
    {
        const auto* grid = workstation_grid_cache;
        if (grid == nullptr || target_station < 0 ||
            target_station >= (int)grid->stations.size() ||
            candidate.location != current_location ||
            current_location == grid->stations[target_station].workstation)
            return 0;
        const auto& station = grid->stations[target_station];
        if (current_location < 0 ||
            station.zone_cells.find(current_location) == station.zone_cells.end())
            return 0;
    }
    if (pressure_cost_activation == "incumbent_grace")
    {
        const auto* grid = workstation_grid_cache;
        if (grid != nullptr && target_station >= 0 &&
            target_station < (int)grid->stations.size() &&
            workstation_incumbent_grace_applies(
                pressure_cost_activation, current_location, candidate.location,
                grid->stations[target_station].workstation,
                current_location >= 0 &&
                grid->stations[target_station].zone_cells.find(current_location) !=
                    grid->stations[target_station].zone_cells.end()))
            return 0;
    }
    if (pressure_cost_activation == "entry_only")
    {
        const auto* grid = workstation_grid_cache;
        if (grid != nullptr && target_station >= 0 &&
            target_station < (int)grid->stations.size())
        {
            const auto& station = grid->stations[target_station];
            const bool current_in_zone = current_location >= 0 &&
                station.zone_cells.find(current_location) != station.zone_cells.end() &&
                current_location != station.workstation;
            const bool candidate_is_exit =
                std::find(station.exit_cells.begin(), station.exit_cells.end(),
                          candidate.location) != station.exit_cells.end();
            if (!workstation_entry_only_cost_applies(
                    pressure_cost_activation, candidate.location,
                    station.workstation, current_in_zone, candidate_is_exit))
                return 0;
        }
    }
    if (pressure_cost_activation == "enter_only")
    {
        const auto* grid = workstation_grid_cache;
        if (grid != nullptr && target_station >= 0 &&
            target_station < (int)grid->stations.size())
        {
            const auto& station = grid->stations[target_station];
            const bool current_in_zone = current_location >= 0 &&
                station.zone_cells.find(current_location) != station.zone_cells.end() &&
                current_location != station.workstation;
            if (!workstation_enter_only_cost_applies(
                    pressure_cost_activation, current_in_zone))
                return 0;
        }
    }
    if (pressure_cost_activation == "deeper_only")
    {
        const auto* grid = workstation_grid_cache;
        if (grid != nullptr && target_station >= 0 &&
            target_station < (int)grid->stations.size())
        {
            const int current_distance = grid->distance_to_workstation(
                target_station, current_location);
            const int candidate_distance = grid->distance_to_workstation(
                target_station, candidate.location);
            if (!workstation_deeper_only_cost_applies(
                    pressure_cost_activation, current_distance, candidate_distance))
                return 0;
        }
    }
    if (pressure_cost_activation == "progress_only")
    {
        const auto* grid = workstation_grid_cache;
        if (grid == nullptr || target_station < 0 ||
            target_station >= (int)grid->stations.size())
            return 0;
        const int current_distance =
            grid->distance_to_workstation(target_station, current_location);
        const int candidate_distance =
            grid->distance_to_workstation(target_station, candidate.location);
        if (candidate_distance < current_distance)
            return 0;
    }
    if (pressure_cost_scope == "lookahead" && pressure_lookahead_radius_cache > 0)
    {
        const auto* grid = workstation_grid_cache;
        if (grid != nullptr && target_station >= 0 &&
            target_station < (int)grid->stations.size() &&
            candidate.location != grid->stations[target_station].workstation &&
            (target_station < (int)pressure_lookahead_location_cache.size() &&
             candidate.location >= 0 && candidate.location < G.size()
                ? pressure_lookahead_location_cache[target_station][candidate.location] != 0
                : grid->distance_to_workstation(target_station, candidate.location) <=
                    pressure_lookahead_radius_cache))
        {
            return cached_cost >= 0
                ? cached_cost
                : workstation_pressure_cost(
                    base_cost,
                    target_pressure, effective_workstation_pressure_threshold(),
                    escalating);
        }
    }
    // Zone and queue membership are precomputed once per station. Reusing
    // that cache keeps the pressure ranking off the large-scale hot path.
    if (target_station < 0 ||
        target_station >= (int)pressure_cost_location_cache.size() ||
        candidate.location < 0 || candidate.location >= G.size() ||
        !pressure_cost_location_cache[target_station][candidate.location])
        return 0;
    return cached_cost >= 0
        ? cached_cost
        : workstation_pressure_cost(
            base_cost,
            target_pressure, effective_workstation_pressure_threshold(),
            escalating);
}

int PIBT::progress_score(int agent, const State& candidate) const
{
    if (active_policy() != Policy::PRESSURE_AWARE || agent < 0 ||
        agent >= num_of_agents || workstation_grid_cache == nullptr ||
        agent >= (int)assignment_progress_target.size())
        return 0;
    const int target = assignment_progress_target[agent];
    const int cost = assignment_progress_cost[agent];
    if (target < 0 || cost <= 0)
        return 0;
    const int current_distance = assignment_progress_current_distance[agent];
    const int candidate_distance = distance_between(candidate.location, target);
    return workstation_progress_cost_applies(current_distance, candidate_distance)
        ? cost : 0;
}

const vector<PIBT::Candidate>& PIBT::ranked_candidates(int agent,
                                                       int local_t,
                                                       const vector<State>& current_states,
                                                       const vector<int>& current_occupant)
{
    if (agent >= 0 && agent < (int)assignment_candidate_cache.size() &&
        assignment_candidate_cache_ready[agent])
        return assignment_candidate_cache[agent];

    auto& candidates = assignment_candidate_cache[agent];
    candidates.clear();
    auto finish = [&]() -> const vector<Candidate>& {
        assignment_candidate_cache_ready[agent] = 1;
        return candidates;
    };
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
        wait.base_score = 0;
        wait.score = 0;
        set_tie_break(wait);
        candidates.push_back(wait);
        return finish();
    }

    if (must_hold_for_workstation_service(agent, current_states[agent]))
    {
        Candidate wait;
        wait.state = current_states[agent];
        wait.state.timestep = local_t;
        wait.loc = wait.state.location;
        wait.wait = true;
        wait.base_score = distance_to_next_goal(agent, wait.loc, goal_indices[agent]);
        wait.score = wait.base_score;
        set_tie_break(wait);
        candidates.push_back(wait);
        return finish();
    }

    if (agent < (int)initial_paths.size() && local_t < (int)initial_paths[agent].size() &&
        !initial_paths[agent].empty())
    {
        Candidate committed;
        committed.state = initial_paths[agent][local_t];
        committed.state.timestep = local_t;
        committed.loc = committed.state.location;
        committed.wait = committed.loc == current_states[agent].location;
        committed.base_score = distance_to_next_goal(agent, committed.loc, goal_indices[agent]);
        committed.score = committed.base_score;
        set_tie_break(committed);
        candidates.push_back(committed);
        return finish();
    }

    const int target_location = current_target(agent) >= 0
        ? goal_locations[agent][goal_indices[agent]].first
        : -1;
    const auto* workstation_grid = workstation_grid_cache;
    const uint16_t* compact_distance_table = workstation_grid == nullptr
        ? nullptr
        : workstation_grid->compact_distance_table(target_location);
    auto distance_to_target = [&](int loc) {
        if (compact_distance_table != nullptr && loc >= 0 && loc < G.size())
        {
            const uint16_t distance = compact_distance_table[loc];
            return distance == UINT16_MAX ? INT_MAX / 4 : (int)distance;
        }
        return distance_between(loc, target_location);
    };

    for (const auto& next_location : cached_neighbors(current_states[agent]))
    {
        Candidate candidate;
        candidate.state = State(next_location.first, local_t, next_location.second);
        candidate.state.timestep = local_t;
        candidate.loc = next_location.first;
        candidate.wait = candidate.loc == current_states[agent].location;
        candidate.base_score = distance_to_target(candidate.loc);
        candidate.score = candidate.base_score;
        set_tie_break(candidate);
        candidates.push_back(candidate);
    }

    auto base_less = [&](const Candidate& lhs, const Candidate& rhs) {
        if (lhs.base_score != rhs.base_score)
            return lhs.base_score < rhs.base_score;
        if (random_tiebreak && lhs.tie_break != rhs.tie_break)
            return lhs.tie_break < rhs.tie_break;
        if (lhs.wait != rhs.wait)
            return !lhs.wait;
        return lhs.loc < rhs.loc;
    };
    auto score_less = [&](const Candidate& lhs, const Candidate& rhs) {
        if (lhs.score != rhs.score)
            return lhs.score < rhs.score;
        // Preserve the pressure preference when a one-step distance gain
        // exactly offsets the soft zone cost. Random tie-breaking here can
        // otherwise erase the pressure signal on the most common boundary
        // transition.
        if (lhs.pressure_cost != rhs.pressure_cost)
            return lhs.pressure_cost < rhs.pressure_cost;
        if (lhs.base_score != rhs.base_score)
            return lhs.base_score < rhs.base_score;
        if (random_tiebreak && lhs.tie_break != rhs.tie_break)
            return lhs.tie_break < rhs.tie_break;
        if (lhs.wait != rhs.wait)
            return !lhs.wait;
        return lhs.loc < rhs.loc;
    };
    auto sort_candidates = [&](bool use_score) {
        // Workstation moves have a small bounded branching factor. Insertion
        // sort avoids the general-purpose sort overhead on this hot path.
        for (size_t i = 1; i < candidates.size(); i++)
        {
            Candidate value = candidates[i];
            size_t j = i;
            while (j > 0 && (use_score
                ? score_less(value, candidates[j - 1])
                : base_less(value, candidates[j - 1])))
            {
                candidates[j] = candidates[j - 1];
                --j;
            }
            candidates[j] = value;
        }
    };
    const bool pressure_cost_active = agent >= 0 &&
        agent < (int)pressure_cost_active_cache.size() &&
        pressure_cost_active_cache[agent];
    const bool progress_cost_active = active_policy() == Policy::PRESSURE_AWARE &&
        agent >= 0 && agent < (int)assignment_progress_target.size() &&
        assignment_progress_target[agent] >= 0 &&
        assignment_progress_cost[agent] > 0;
    if (!pressure_cost_active && !progress_cost_active)
    {
        sort_candidates(false);
        return finish();
    }

    const auto base_top = candidates.empty()
        ? candidates.end()
        : std::min_element(candidates.begin(), candidates.end(), base_less);
    const int base_top_loc = base_top == candidates.end() ? -1 : base_top->loc;
    bool pressure_applied = false;
    int min_pressure_cost = std::numeric_limits<int>::max();
    int max_pressure_cost = 0;
    for (Candidate& candidate : candidates)
    {
        const int cost = pressure_score(
            agent, current_states[agent].location, candidate.state) +
            progress_score(agent, candidate.state);
        candidate.pressure_cost = cost;
        min_pressure_cost = std::min(min_pressure_cost, cost);
        max_pressure_cost = std::max(max_pressure_cost, cost);
        if (cost > 0)
        {
            candidate.score = candidate.base_score + cost;
            pressure_applied = true;
            pressure_candidate_hits++;
        }
    }

    if (!pressure_applied)
    {
        sort_candidates(false);
        return finish();
    }

    // A uniform pressure cost cannot change the native PIBT ordering. This
    // occurs frequently for agents already inside a pressured zone, so avoid
    // a second comparison sort while preserving the exact candidate order.
    // Include zero-cost candidates in the span: otherwise a mixed set of
    // outside-zone and inside-zone actions can be misclassified as uniform.
    if (min_pressure_cost == max_pressure_cost)
    {
        sort_candidates(false);
        return finish();
    }

    sort_candidates(true);
    if (active_policy() != Policy::VANILLA &&
        base_top_loc >= 0 && !candidates.empty() &&
        base_top_loc != candidates.front().loc)
    {
        // Count the intervention when the list is built; cache reuse is not
        // another pressure-induced change.
        pressure_rank_changes++;
    }
    return finish();
}

bool PIBT::violates_initial_constraint(int agent, int loc, int local_t) const
{
    if (loc < 0 || loc >= (int)initial_constraint_cache.size())
        return false;
    for (const auto& constraint : initial_constraint_cache[loc])
    {
        int owner = constraint.first;
        int constraint_until = constraint.second;
        if (owner != agent &&
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
                         const vector<char>& assigned) const
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
                        vector<char>& assigned,
                        vector<char>& visiting,
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
    const auto& candidates = ranked_candidates(
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
                                vector<char>& assigned,
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
                      vector<char>& assigned) const
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

bool PIBT::greedy_repair(int agent,
                         int local_t,
                         const vector<State>& current_states,
                         const vector<int>& current_occupant,
                         vector<State>& next_states,
                         vector<int>& next_occupant,
                         vector<char>& assigned)
{
    if (assigned[agent])
        return true;

    // Once recursive search is budgeted out, use the same ranked candidates
    // without inheritance. This preserves collision checks while avoiding a
    // long tail of unconditional waits for agents with a free direct move.
    const auto& candidates = ranked_candidates(
        agent, local_t, current_states, current_occupant);
    for (const Candidate& candidate : candidates)
    {
        if (candidate.loc < 0 || candidate.loc >= (int)next_occupant.size())
            continue;
        if (violates_initial_constraint(agent, candidate.loc, local_t))
            continue;
        const int occupant = candidate.loc < (int)current_occupant.size()
            ? current_occupant[candidate.loc]
            : -1;
        if (occupant >= 0 && occupant != agent && !assigned[occupant])
            continue;
        if (next_occupant[candidate.loc] >= 0)
            continue;
        if (has_edge_swap(
                agent, candidate.state, current_states, current_occupant,
                next_states, assigned))
            continue;

        next_states[agent] = candidate.state;
        next_states[agent].timestep = local_t;
        assigned[agent] = true;
        next_occupant[candidate.loc] = agent;
        return true;
    }
    return false;
}

bool PIBT::validate_step(const vector<State>& current_states,
                         const vector<State>& next_states,
                         const vector<int>& current_occupant,
                         const vector<int>& next_occupant) const
{
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        int current_loc = current_states[agent].location;
        int next_loc = next_states[agent].location;
        if (current_loc < 0 || current_loc >= (int)current_occupant.size() ||
            next_loc < 0 || next_loc >= (int)next_occupant.size())
            return false;
        if (current_occupant[current_loc] != agent || next_occupant[next_loc] != agent)
            return false;
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
    // The occupancy arrays are indexed by map location for O(1) collision
    // checks, but clearing every map cell on every one-step assignment is
    // needlessly expensive on large sparse sortation maps. Clear only the
    // locations populated by the previous current-state snapshot.
    for (int loc : assignment_current_touched)
    {
        if (loc >= 0 && loc < (int)assignment_current_occupant.size())
            assignment_current_occupant[loc] = -1;
    }
    assignment_current_touched.clear();
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        int loc = current_states[agent].location;
        if (loc < 0 || loc >= (int)assignment_current_occupant.size() ||
            assignment_current_occupant[loc] >= 0)
            return false;
        assignment_current_occupant[loc] = agent;
        assignment_current_touched.push_back(loc);
    }

    assignment_order.resize(num_of_agents);
    std::iota(assignment_order.begin(), assignment_order.end(), 0);
    assignment_protected_class.assign(num_of_agents, 0);
    assignment_station_id.assign(num_of_agents, -1);
    assignment_to_station.assign(num_of_agents, 0);
    assignment_front_runner.assign(num_of_agents, 0);
    assignment_front_runner_ready.assign(num_of_agents, 0);
    assignment_progress_target.assign(num_of_agents, -1);
    assignment_progress_current_distance.assign(num_of_agents, -1);
    assignment_progress_cost.assign(num_of_agents, 0);
    if (use_exit_priority())
    {
        const auto* grid = workstation_grid_cache;
        for (int agent = 0; agent < num_of_agents; agent++)
        {
            const auto context = active_context(agent);
            assignment_station_id[agent] = context.station_id;
            assignment_to_station[agent] =
                context.phase == WorkstationAgentPhase::TO_STATION ? 1 : 0;
            assignment_protected_class[agent] =
                is_protected_workstation_phase(context.phase) ? 0 : 1;
            if (context.phase == WorkstationAgentPhase::TO_STATION &&
                context.station_id >= 0)
            {
                assignment_front_runner[agent] =
                    is_station_front_runner(agent, context.station_id) ? 1 : 0;
                if ((front_runner_ready_priority || pressure_ready_slot_priority) &&
                    assignment_front_runner[agent] &&
                    grid != nullptr && context.station_id < (int)grid->stations.size() &&
                    grid->distance_to_workstation(
                        context.station_id, current_states[agent].location) <= 1)
                    assignment_front_runner_ready[agent] = 1;
                if (pressure_front_progress_cost > 0 &&
                    assignment_front_runner[agent] && grid != nullptr &&
                    context.station_id < (int)grid->stations.size())
                {
                    assignment_progress_target[agent] =
                        grid->stations[context.station_id].workstation;
                    assignment_progress_cost[agent] = pressure_front_progress_cost;
                }
            }
            else if (context.phase == WorkstationAgentPhase::TO_EXIT &&
                     pressure_exit_progress_cost > 0 && grid != nullptr &&
                     context.station_id >= 0 &&
                     context.station_id < (int)grid->stations.size())
            {
                int best_distance = INT_MAX / 4;
                for (int exit : grid->stations[context.station_id].exit_cells)
                {
                    const int distance = distance_between(
                        current_states[agent].location, exit);
                    if (distance < best_distance ||
                        (distance == best_distance &&
                         (assignment_progress_target[agent] < 0 ||
                          exit < assignment_progress_target[agent])))
                    {
                        best_distance = distance;
                        assignment_progress_target[agent] = exit;
                    }
                }
                assignment_progress_cost[agent] = pressure_exit_progress_cost;
            }
            if (assignment_progress_target[agent] >= 0)
                assignment_progress_current_distance[agent] = distance_between(
                    current_states[agent].location,
                    assignment_progress_target[agent]);
        }
    }
    // A positive scale gate is intended to defer all front-runner promotion,
    // including the same-station fallback of global mode, until widespread
    // pressure is present. With the default gate of zero, preserve the
    // original global/local behavior.
    const bool scale_gated_global_front = global_front_runner_priority &&
        network_pressure_min_agents_per_station > 0 && !network_pressure_active;
    std::sort(assignment_order.begin(), assignment_order.end(), [&](int lhs, int rhs) {
        if (use_exit_priority())
        {
            if (assignment_protected_class[lhs] < assignment_protected_class[rhs])
                return true;
            if (assignment_protected_class[rhs] < assignment_protected_class[lhs])
                return false;
        }
        if (pressure_ready_slot_priority &&
            assignment_front_runner_ready[lhs] != assignment_front_runner_ready[rhs])
            return assignment_front_runner_ready[lhs] > assignment_front_runner_ready[rhs];
        if (!scale_gated_global_front &&
            global_front_runner_priority && front_runner_priority &&
            front_runner_ready_priority &&
            assignment_front_runner_ready[lhs] != assignment_front_runner_ready[rhs])
        {
            if (global_front_runner_priority && network_pressure_active)
                return assignment_front_runner_ready[lhs] > assignment_front_runner_ready[rhs];
            if (assignment_to_station[lhs] && assignment_to_station[rhs] &&
                assignment_station_id[lhs] >= 0 &&
                assignment_station_id[lhs] == assignment_station_id[rhs])
                return assignment_front_runner_ready[lhs] > assignment_front_runner_ready[rhs];
        }
        // Once a station is beyond its pressure trigger, let its selected
        // front-runner win same-station conflicts before dynamic age.
        if (!scale_gated_global_front &&
            front_runner_priority && active_policy() == Policy::PRESSURE_AWARE &&
            (!front_runner_ready_priority ||
             assignment_front_runner_ready[lhs] || assignment_front_runner_ready[rhs]) &&
            assignment_front_runner[lhs] != assignment_front_runner[rhs])
        {
            if (global_front_runner_priority && network_pressure_active)
                return assignment_front_runner[lhs] > assignment_front_runner[rhs];
            if (assignment_to_station[lhs] && assignment_to_station[rhs] &&
                assignment_station_id[lhs] >= 0 &&
                assignment_station_id[lhs] == assignment_station_id[rhs])
            {
                return assignment_front_runner[lhs] > assignment_front_runner[rhs];
            }
        }
        if (priority_age[lhs] != priority_age[rhs])
            return priority_age[lhs] > priority_age[rhs];
        return lhs < rhs;
    });

    // On a successful pass every agent has a next state, so the previous
    // next-occupancy entries can be cleared in O(number of agents). A failed
    // pass terminates the episode and never reaches another assignment pass.
    for (const State& state : assignment_next_states)
    {
        const int loc = state.location;
        if (loc >= 0 && loc < (int)assignment_next_occupant.size())
            assignment_next_occupant[loc] = -1;
    }
    assignment_next_states = current_states;
    assignment_assigned.assign(num_of_agents, 0);
    assignment_visiting.assign(num_of_agents, 0);
    assignment_log.reserve(num_of_agents);
    assignment_log.clear();
    for (auto& entries : assignment_candidate_cache)
        entries.clear();
    std::fill(assignment_candidate_cache_ready.begin(),
              assignment_candidate_cache_ready.end(), 0);
    remaining_assignment_budget = step_assignment_budget();
    remaining_assignment_extension_budget = pressure_assignment_extension_budget();

    auto assign_with_budget_extension = [&](int agent) {
        bool assigned = assign_agent(
            agent, local_t, current_states, assignment_current_occupant,
            assignment_next_states, assignment_next_occupant,
            assignment_assigned, assignment_visiting, assignment_log, -1);
        if (!assigned && remaining_assignment_budget <= 0 &&
            remaining_assignment_extension_budget > 0)
        {
            remaining_assignment_budget = remaining_assignment_extension_budget;
            remaining_assignment_extension_budget = 0;
            budget_extensions++;
            assigned = assign_agent(
                agent, local_t, current_states, assignment_current_occupant,
                assignment_next_states, assignment_next_occupant,
                assignment_assigned, assignment_visiting, assignment_log, -1);
        }
        return assigned;
    };

    for (int agent : assignment_order)
    {
        if (assignment_assigned[agent])
            continue;
        if (!assign_with_budget_extension(agent))
        {
            if (greedy_repair(agent, local_t, current_states, assignment_current_occupant,
                              assignment_next_states, assignment_next_occupant,
                              assignment_assigned))
            {
                if (count_fallbacks)
                    greedy_repairs++;
                continue;
            }
            if (!force_wait(agent, local_t, current_states, assignment_next_states,
                            assignment_next_occupant, assignment_assigned))
                return false;
            if (count_fallbacks)
                wait_fallbacks++;
        }
    }
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        if (!assignment_assigned[agent])
        {
            if (greedy_repair(agent, local_t, current_states, assignment_current_occupant,
                              assignment_next_states, assignment_next_occupant,
                              assignment_assigned))
            {
                if (count_fallbacks)
                    greedy_repairs++;
                continue;
            }
            if (!force_wait(agent, local_t, current_states, assignment_next_states,
                            assignment_next_occupant, assignment_assigned))
                return false;
            if (count_fallbacks)
                wait_fallbacks++;
        }
    }

    next_states = assignment_next_states;
    if (!validate_step(current_states, assignment_next_states,
                       assignment_current_occupant, assignment_next_occupant))
        return false;

    return true;
}

bool PIBT::plan_one_step(int local_t, vector<State>& current_states)
{
    for (int agent = 0; agent < num_of_agents; agent++)
        advance_goal_index(agent, current_states[agent], local_t - 1);
    refresh_active_context_cache();
    update_pressure_state(current_states);
    assignment_pass_index = 0;
    if (!assignment_pass(local_t, current_states, assignment_next_states, true))
        return false;

    for (int agent = 0; agent < num_of_agents; agent++)
        solution[agent].push_back(assignment_next_states[agent]);
    current_states = assignment_next_states;
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
              << greedy_repairs << ","
              << pressure_rank_changes << ","
              << pressure_active_agents << ","
              << pressure_candidate_hits << ","
              << solution_cost << ","
              << min_sum_of_costs << ","
              << avg_path_length << ","
              << budget_extensions << std::endl;
}

void PIBT::save_results(const std::string& fileName, const std::string& instanceName) const
{
    std::ofstream stats;
    stats.open(fileName, std::ios::app);
    stats << runtime << ","
          << inheritance_calls << ","
          << backtracks << ","
          << wait_fallbacks << ","
          << greedy_repairs << ","
          << pressure_rank_changes << ","
          << pressure_active_agents << ","
          << pressure_candidate_hits << ","
          << solution_cost << ","
          << min_sum_of_costs << ","
          << avg_path_length << ","
          << "0" << ","
          << instanceName << ","
          << budget_extensions << std::endl;
    stats.close();
}
