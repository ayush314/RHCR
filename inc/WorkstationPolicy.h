#pragma once

#include "WorkstationGraph.h"

struct WorkstationPressureSnapshot
{
    vector<int> station_pressure;
    vector<vector<int>> privileged_inbound_agents;
};

struct WorkstationPressureBaseSnapshot
{
    vector<int> station_pressure;
    vector<vector<tuple<int, int, int, int>>> leading_inbound_keys;
};

WorkstationPressureSnapshot evaluate_workstation_pressure(
    const WorkstationGrid& grid,
    const vector<int>& agent_locations,
    const vector<WorkstationAgentContext>& agent_contexts);

int workstation_pressure_action_cost(
    const WorkstationGrid& grid,
    const WorkstationPressureSnapshot& snapshot,
    const vector<WorkstationAgentContext>& agent_contexts,
    int agent,
    int candidate_location);

WorkstationPressureBaseSnapshot evaluate_workstation_pressure_without_agent(
    const WorkstationGrid& grid,
    const vector<int>& agent_locations,
    const vector<WorkstationAgentContext>& agent_contexts,
    int excluded_agent);

int workstation_pressure_action_cost_from_base(
    const WorkstationGrid& grid,
    const WorkstationPressureBaseSnapshot& base_snapshot,
    const WorkstationAgentContext& agent_context,
    int agent,
    int current_location,
    int candidate_location);
