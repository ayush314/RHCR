#include "WorkstationPolicy.h"

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

WorkstationPressureSnapshot evaluate_workstation_pressure(
    const WorkstationGrid& grid,
    const vector<int>& agent_locations,
    const vector<WorkstationAgentContext>& agent_contexts)
{
    WorkstationPressureSnapshot snapshot;
    WorkstationPressureWorkspace workspace;
    evaluate_workstation_pressure(
        grid, agent_locations, agent_contexts, snapshot, workspace);
    return snapshot;
}

void evaluate_workstation_pressure(
    const WorkstationGrid& grid,
    const vector<int>& agent_locations,
    const vector<WorkstationAgentContext>& agent_contexts,
    WorkstationPressureSnapshot& snapshot,
    WorkstationPressureWorkspace& workspace)
{
    const size_t station_count = grid.stations.size();
    snapshot.station_pressure.assign(station_count, 0);
    snapshot.privileged_inbound_agents.resize(station_count);
    workspace.inbound_by_station.resize(station_count);
    for (size_t station_id = 0; station_id < station_count; station_id++)
    {
        snapshot.privileged_inbound_agents[station_id].clear();
        workspace.inbound_by_station[station_id].clear();
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
            workspace.inbound_by_station[context.station_id].push_back(
                static_cast<int>(agent));
        }
    }

    for (size_t station_id = 0; station_id < station_count; station_id++)
    {
        if (!workstation_pressure_active(snapshot.station_pressure[station_id]))
            continue;

        workspace.ranked_inbound.clear();
        workspace.ranked_inbound.reserve(
            workspace.inbound_by_station[station_id].size());
        for (int agent : workspace.inbound_by_station[station_id])
        {
            workspace.ranked_inbound.emplace_back(
                pressure_key(
                    grid, static_cast<int>(station_id), agent,
                    agent_locations[agent], agent_contexts[agent]),
                agent);
        }
        vector<int>& privileged =
            snapshot.privileged_inbound_agents[station_id];
        const size_t privileged_count = std::min(
            workspace.ranked_inbound.size(),
            static_cast<size_t>(kWorkstationPrivilegedInboundCount));
        std::partial_sort(
            workspace.ranked_inbound.begin(),
            workspace.ranked_inbound.begin() + privileged_count,
            workspace.ranked_inbound.end(),
            [](const pair<tuple<int, int, int, int>, int>& lhs,
               const pair<tuple<int, int, int, int>, int>& rhs) {
                return lhs.first < rhs.first;
            });
        privileged.reserve(privileged_count);
        for (size_t index = 0; index < privileged_count; index++)
            privileged.push_back(workspace.ranked_inbound[index].second);
    }
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

    if (station_id < static_cast<int>(snapshot.privileged_inbound_agents.size()))
    {
        const vector<int>& privileged =
            snapshot.privileged_inbound_agents[station_id];
        if (std::find(privileged.begin(), privileged.end(), agent) !=
            privileged.end())
        {
            return 0;
        }
    }
    return kWorkstationPressureQueueCost;
}

WorkstationPressureBaseSnapshot evaluate_workstation_pressure_without_agent(
    const WorkstationGrid& grid,
    const vector<int>& agent_locations,
    const vector<WorkstationAgentContext>& agent_contexts,
    int excluded_agent)
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
        if (keys.size() > static_cast<size_t>(kWorkstationPrivilegedInboundCount))
        {
            std::partial_sort(
                keys.begin(),
                keys.begin() + kWorkstationPrivilegedInboundCount,
                keys.end());
            keys.resize(kWorkstationPrivilegedInboundCount);
        }
    }
    return snapshot;
}

int workstation_pressure_action_cost_from_base(
    const WorkstationGrid& grid,
    const WorkstationPressureBaseSnapshot& base_snapshot,
    const WorkstationAgentContext& agent_context,
    int agent,
    int current_location,
    int candidate_location)
{
    const int station_id = agent_context.station_id;
    if (agent_context.phase != WorkstationAgentPhase::TO_STATION ||
        station_id < 0 || station_id >= static_cast<int>(grid.stations.size()) ||
        station_id >= static_cast<int>(base_snapshot.station_pressure.size()) ||
        grid.stations[station_id].zone_cells.find(candidate_location) ==
            grid.stations[station_id].zone_cells.end())
    {
        return 0;
    }

    int pressure = base_snapshot.station_pressure[station_id];
    if (grid.stations[station_id].zone_cells.find(current_location) !=
        grid.stations[station_id].zone_cells.end())
    {
        pressure++;
    }
    if (!workstation_pressure_active(pressure))
        return 0;

    const tuple<int, int, int, int> agent_key = pressure_key(
        grid, station_id, agent, current_location, agent_context);
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
    return agents_ahead < kWorkstationPrivilegedInboundCount ?
        0 : kWorkstationPressureQueueCost;
}
