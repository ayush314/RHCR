#include "WorkstationPolicy.h"

namespace
{
int configured_pressure_threshold = kWorkstationPressureThreshold;
}

int workstation_pressure_threshold()
{
    return configured_pressure_threshold;
}

void set_workstation_pressure_threshold(int threshold)
{
    configured_pressure_threshold = threshold;
}

#include <algorithm>
#include <limits>

namespace
{
constexpr int kMissingPriorityValue = std::numeric_limits<int>::max() / 4;

int station_for_location(const WorkstationGrid& grid, int location)
{
    int station_id = grid.station_for_zone_cell(location);
    if (station_id >= 0 || grid.has_complete_zone_index())
        return station_id;
    for (size_t candidate = 0; candidate < grid.stations.size(); candidate++)
    {
        if (grid.stations[candidate].zone_cells.find(location) !=
            grid.stations[candidate].zone_cells.end())
        {
            return static_cast<int>(candidate);
        }
    }
    return -1;
}

tuple<int, int, int, int> pressure_key(
    const WorkstationGrid& grid,
    int station_id,
    int agent,
    int location,
    const WorkstationAgentContext& context)
{
    const int issue_time = context.task_issue_t >= 0 ?
        context.task_issue_t : kMissingPriorityValue;
    const bool inside_target_queue =
        station_for_location(grid, location) == station_id;
    return workstation_privilege_key(
        inside_target_queue,
        grid.distance_to_workstation(station_id, location),
        issue_time,
        agent);
}

}

vector<int> evaluate_workstation_lead_agents(
    const WorkstationGrid& grid,
    const vector<int>& agent_locations,
    const vector<WorkstationAgentContext>& agent_contexts)
{
    vector<int> leads(grid.stations.size(), -1);
    vector<tuple<int, int, int, int>> lead_keys(
        grid.stations.size(),
        tuple<int, int, int, int>{
            kMissingPriorityValue, kMissingPriorityValue,
            kMissingPriorityValue, kMissingPriorityValue});

    const size_t agent_count = std::min(
        agent_locations.size(), agent_contexts.size());
    for (size_t agent = 0; agent < agent_count; agent++)
    {
        const WorkstationAgentContext& context = agent_contexts[agent];
        const int station_id = context.station_id;
        if (context.phase != WorkstationAgentPhase::TO_STATION ||
            station_id < 0 ||
            station_id >= static_cast<int>(grid.stations.size()))
        {
            continue;
        }

        const auto key = pressure_key(
            grid, station_id, static_cast<int>(agent),
            agent_locations[agent], context);
        if (leads[station_id] < 0 || key < lead_keys[station_id])
        {
            leads[station_id] = static_cast<int>(agent);
            lead_keys[station_id] = key;
        }
    }
    return leads;
}

bool workstation_conflict_touches_station_zone(
    const WorkstationGrid& grid,
    int station_id,
    int first_location,
    int second_location)
{
    if (station_id < 0 ||
        station_id >= static_cast<int>(grid.stations.size()))
    {
        return false;
    }
    return station_for_location(grid, first_location) == station_id ||
        station_for_location(grid, second_location) == station_id;
}

bool workstation_state_or_action_touches_station_zone(
    const WorkstationGrid& grid,
    int station_id,
    const State& state)
{
    if (station_id < 0 ||
        station_id >= static_cast<int>(grid.stations.size()))
    {
        return false;
    }
    auto touches_target_queue = [&](int location) {
        return location >= 0 && location < grid.size() &&
            station_for_location(grid, location) == station_id;
    };

    if (touches_target_queue(state.location))
        return true;
    if (state.orientation >= 0)
    {
        return grid.valid_move(state.location, state.orientation) &&
            touches_target_queue(
                state.location + grid.move[state.orientation]);
    }
    for (int direction = 0; direction < 4; direction++)
    {
        if (grid.valid_move(state.location, direction) &&
            touches_target_queue(state.location + grid.move[direction]))
        {
            return true;
        }
    }
    return false;
}

WorkstationPressureSnapshot evaluate_workstation_pressure(
    const WorkstationGrid& grid,
    const vector<int>& agent_locations,
    const vector<WorkstationAgentContext>& agent_contexts,
    int privileged_inbound_count)
{
    WorkstationPressureSnapshot snapshot;
    WorkstationPressureWorkspace workspace;
    evaluate_workstation_pressure(
        grid, agent_locations, agent_contexts, snapshot, workspace,
        privileged_inbound_count);
    return snapshot;
}

void evaluate_workstation_pressure(
    const WorkstationGrid& grid,
    const vector<int>& agent_locations,
    const vector<WorkstationAgentContext>& agent_contexts,
    WorkstationPressureSnapshot& snapshot,
    WorkstationPressureWorkspace& workspace,
    int privileged_inbound_count)
{
    const size_t station_count = grid.stations.size();
    snapshot.station_pressure.assign(station_count, 0);
    snapshot.privileged_inbound_agents.resize(station_count);
    workspace.ranked_inbound_by_station.resize(station_count);
    workspace.lead_agents.assign(station_count, -1);
    workspace.lead_keys.assign(
        station_count,
        tuple<int, int, int, int>{
            kMissingPriorityValue, kMissingPriorityValue,
            kMissingPriorityValue, kMissingPriorityValue});
    for (size_t station_id = 0; station_id < station_count; station_id++)
    {
        snapshot.privileged_inbound_agents[station_id].clear();
        workspace.ranked_inbound_by_station[station_id].clear();
    }

    for (size_t agent = 0; agent < agent_locations.size(); agent++)
    {
        const int occupied_station = station_for_location(
            grid, agent_locations[agent]);
        if (occupied_station >= 0 &&
            occupied_station < static_cast<int>(grid.stations.size()))
        {
            snapshot.station_pressure[occupied_station]++;
        }

        if (agent >= agent_contexts.size())
            continue;
        const WorkstationAgentContext& context = agent_contexts[agent];
        if (context.phase == WorkstationAgentPhase::TO_STATION &&
            context.station_id >= 0 &&
            context.station_id < static_cast<int>(grid.stations.size()))
        {
            const int station_id = context.station_id;
            const auto key = pressure_key(
                grid, station_id, static_cast<int>(agent),
                agent_locations[agent], context);
            workspace.ranked_inbound_by_station[station_id].emplace_back(
                key, static_cast<int>(agent));
            if (workspace.lead_agents[station_id] < 0 ||
                key < workspace.lead_keys[station_id])
            {
                workspace.lead_agents[station_id] = static_cast<int>(agent);
                workspace.lead_keys[station_id] = key;
            }
        }
    }

    for (size_t station_id = 0; station_id < station_count; station_id++)
    {
        if (!workstation_pressure_active(snapshot.station_pressure[station_id]))
            continue;

        auto& ranked_inbound =
            workspace.ranked_inbound_by_station[station_id];
        vector<int>& privileged =
            snapshot.privileged_inbound_agents[station_id];
        const size_t privilege_limit = static_cast<size_t>(
            std::max(0, privileged_inbound_count));
        const size_t privileged_count = std::min(
            ranked_inbound.size(), privilege_limit);
        std::partial_sort(
            ranked_inbound.begin(),
            ranked_inbound.begin() + privileged_count,
            ranked_inbound.end(),
            [](const pair<tuple<int, int, int, int>, int>& lhs,
               const pair<tuple<int, int, int, int>, int>& rhs) {
                return lhs.first < rhs.first;
            });
        privileged.reserve(privileged_count);
        for (size_t index = 0; index < privileged_count; index++)
            privileged.push_back(ranked_inbound[index].second);
    }
}

bool workstation_pressure_agent_is_privileged(
    const WorkstationPressureSnapshot& snapshot,
    const WorkstationAgentContext& context,
    int agent)
{
    if (agent < 0)
        return false;
    const int station_id = context.station_id;
    if (context.phase != WorkstationAgentPhase::TO_STATION ||
        station_id < 0 ||
        station_id >= static_cast<int>(snapshot.station_pressure.size()) ||
        station_id >=
            static_cast<int>(snapshot.privileged_inbound_agents.size()) ||
        !workstation_pressure_active(snapshot.station_pressure[station_id]))
    {
        return false;
    }
    const vector<int>& privileged =
        snapshot.privileged_inbound_agents[station_id];
    return std::find(privileged.begin(), privileged.end(), agent) !=
        privileged.end();
}

bool workstation_pressure_agent_is_privileged(
    const WorkstationPressureSnapshot& snapshot,
    const vector<WorkstationAgentContext>& agent_contexts,
    int agent)
{
    return agent >= 0 && agent < static_cast<int>(agent_contexts.size()) &&
        workstation_pressure_agent_is_privileged(
            snapshot, agent_contexts[agent], agent);
}

int workstation_pressure_action_cost(
    const WorkstationGrid& grid,
    const WorkstationPressureSnapshot& snapshot,
    const vector<WorkstationAgentContext>& agent_contexts,
    int agent,
    int candidate_location)
{
    if (agent < 0 || agent >= static_cast<int>(agent_contexts.size()))
        return 0;

    const WorkstationAgentContext& context = agent_contexts[agent];
    const int station_id = context.station_id;
    if (context.phase != WorkstationAgentPhase::TO_STATION ||
        station_id < 0 || station_id >= static_cast<int>(grid.stations.size()) ||
        station_id >= static_cast<int>(snapshot.station_pressure.size()) ||
        !workstation_pressure_active(snapshot.station_pressure[station_id]) ||
        grid.stations[station_id].zone_cells.find(candidate_location) ==
            grid.stations[station_id].zone_cells.end())
    {
        return 0;
    }

    if (workstation_pressure_agent_is_privileged(
            snapshot, agent_contexts, agent))
        return 0;
    return kWorkstationPressureQueueCost;
}

WorkstationPressureBaseSnapshot evaluate_workstation_pressure_without_agent(
    const WorkstationGrid& grid,
    const vector<int>& agent_locations,
    const vector<WorkstationAgentContext>& agent_contexts,
    int excluded_agent,
    int privileged_inbound_count)
{
    WorkstationPressureBaseSnapshot snapshot;
    snapshot.station_pressure.assign(grid.stations.size(), 0);
    snapshot.leading_inbound_keys.assign(
        grid.stations.size(), vector<tuple<int, int, int, int>>());

    for (int agent = 0; agent < static_cast<int>(agent_locations.size()); agent++)
    {
        if (agent == excluded_agent)
            continue;
        const int occupied_station = station_for_location(
            grid, agent_locations[agent]);
        if (occupied_station >= 0 &&
            occupied_station < static_cast<int>(grid.stations.size()))
        {
            snapshot.station_pressure[occupied_station]++;
        }

        if (agent >= static_cast<int>(agent_contexts.size()))
            continue;
        const WorkstationAgentContext& context = agent_contexts[agent];
        if (context.phase != WorkstationAgentPhase::TO_STATION ||
            context.station_id < 0 ||
            context.station_id >= static_cast<int>(grid.stations.size()))
        {
            continue;
        }
        snapshot.leading_inbound_keys[context.station_id].push_back(
            pressure_key(grid, context.station_id, agent,
                         agent_locations[agent], context));
    }

    for (vector<tuple<int, int, int, int>>& keys :
         snapshot.leading_inbound_keys)
    {
        const size_t privilege_limit = static_cast<size_t>(
            std::max(0, privileged_inbound_count));
        if (keys.size() > privilege_limit)
        {
            std::partial_sort(
                keys.begin(),
                keys.begin() + privilege_limit,
                keys.end());
            keys.resize(privilege_limit);
        }
    }
    return snapshot;
}

int workstation_pressure_penalty_station_from_base(
    const WorkstationGrid& grid,
    const WorkstationPressureBaseSnapshot& base_snapshot,
    const WorkstationAgentContext& agent_context,
    int agent,
    int reference_location,
    int privileged_inbound_count)
{
    const int station_id = agent_context.station_id;
    if (agent_context.phase != WorkstationAgentPhase::TO_STATION ||
        station_id < 0 || station_id >= static_cast<int>(grid.stations.size()) ||
        station_id >= static_cast<int>(base_snapshot.station_pressure.size()))
    {
        return -1;
    }

    int pressure = base_snapshot.station_pressure[station_id];
    if (grid.stations[station_id].zone_cells.find(reference_location) !=
        grid.stations[station_id].zone_cells.end())
    {
        pressure++;
    }
    if (!workstation_pressure_active(pressure))
        return -1;

    const tuple<int, int, int, int> agent_key = pressure_key(
        grid, station_id, agent, reference_location, agent_context);
    int agents_ahead = 0;
    if (station_id < static_cast<int>(base_snapshot.leading_inbound_keys.size()))
    {
        for (const tuple<int, int, int, int>& key :
             base_snapshot.leading_inbound_keys[station_id])
        {
            if (key < agent_key)
                agents_ahead++;
        }
    }
    return agents_ahead < std::max(0, privileged_inbound_count) ?
        -1 : station_id;
}

int workstation_pressure_action_cost_from_base(
    const WorkstationGrid& grid,
    const WorkstationPressureBaseSnapshot& base_snapshot,
    const WorkstationAgentContext& agent_context,
    int agent,
    int current_location,
    int candidate_location,
    int privileged_inbound_count)
{
    const int station_id = workstation_pressure_penalty_station_from_base(
        grid, base_snapshot, agent_context, agent, current_location,
        privileged_inbound_count);
    if (station_id < 0 ||
        grid.stations[station_id].zone_cells.find(candidate_location) ==
            grid.stations[station_id].zone_cells.end())
    {
        return 0;
    }
    return kWorkstationPressureQueueCost;
}
