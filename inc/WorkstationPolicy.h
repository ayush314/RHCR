#pragma once

#include "WorkstationGraph.h"

struct WorkstationPressureSnapshot
{
    vector<int> station_pressure;
    vector<vector<int>> privileged_inbound_agents;
};

struct WorkstationPressureWorkspace
{
    vector<vector<pair<tuple<int, int, int, int>, int>>>
        ranked_inbound_by_station;
    vector<int> lead_agents;
    vector<tuple<int, int, int, int>> lead_keys;
};

struct WorkstationPressureBaseSnapshot
{
    vector<int> station_pressure;
    vector<vector<tuple<int, int, int, int>>> leading_inbound_keys;
};

vector<int> evaluate_workstation_lead_agents(
    const WorkstationGrid& grid,
    const vector<int>& agent_locations,
    const vector<WorkstationAgentContext>& agent_contexts);

bool workstation_conflict_touches_station_zone(
    const WorkstationGrid& grid,
    int station_id,
    int first_location,
    int second_location);

bool workstation_state_or_action_touches_station_zone(
    const WorkstationGrid& grid,
    int station_id,
    const State& state);

WorkstationPressureSnapshot evaluate_workstation_pressure(
    const WorkstationGrid& grid,
    const vector<int>& agent_locations,
    const vector<WorkstationAgentContext>& agent_contexts,
    int privileged_inbound_count = kWorkstationPrivilegedInboundCount);

void evaluate_workstation_pressure(
    const WorkstationGrid& grid,
    const vector<int>& agent_locations,
    const vector<WorkstationAgentContext>& agent_contexts,
    WorkstationPressureSnapshot& snapshot,
    WorkstationPressureWorkspace& workspace,
    int privileged_inbound_count = kWorkstationPrivilegedInboundCount);

bool workstation_pressure_agent_is_privileged(
    const WorkstationPressureSnapshot& snapshot,
    const WorkstationAgentContext& agent_context,
    int agent);

bool workstation_pressure_agent_is_privileged(
    const WorkstationPressureSnapshot& snapshot,
    const vector<WorkstationAgentContext>& agent_contexts,
    int agent);

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
    int excluded_agent,
    int privileged_inbound_count = kWorkstationPrivilegedInboundCount);

int workstation_pressure_penalty_station_from_base(
    const WorkstationGrid& grid,
    const WorkstationPressureBaseSnapshot& base_snapshot,
    const WorkstationAgentContext& agent_context,
    int agent,
    int reference_location,
    int privileged_inbound_count = kWorkstationPrivilegedInboundCount);

int workstation_pressure_action_cost_from_base(
    const WorkstationGrid& grid,
    const WorkstationPressureBaseSnapshot& base_snapshot,
    const WorkstationAgentContext& agent_context,
    int agent,
    int current_location,
    int candidate_location,
    int privileged_inbound_count = kWorkstationPrivilegedInboundCount);
