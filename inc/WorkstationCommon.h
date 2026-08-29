#pragma once

#include <algorithm>
#include <string>
#include <tuple>
#include <vector>

enum class WorkstationAgentPhase
{
    NONE = 0,
    TO_PICKUP = 1,
    TO_STATION = 2,
    TO_EXIT = 3,
    SERVICE = 4,
};

struct WorkstationAgentContext
{
    int station_id = -1;
    int current_t = -1;
    int boundary_entry_t = -1;
    int task_issue_t = -1;
    WorkstationAgentPhase phase = WorkstationAgentPhase::NONE;
};

inline WorkstationAgentContext workstation_context_for_goal(
    const WorkstationAgentContext& current_context,
    const std::vector<WorkstationAgentContext>& projected_contexts,
    int goal_index)
{
    if (projected_contexts.empty())
        return current_context;
    const int context_index = std::min(
        std::max(goal_index, 0),
        static_cast<int>(projected_contexts.size()) - 1);
    return projected_contexts[context_index];
}

constexpr int kWorkstationPressureThreshold = 3;
constexpr int kWorkstationPrivilegedInboundCount = 4;
constexpr int kWorkstationPressureQueueCost = 2;

inline bool workstation_window_is_stalled(
    bool majority_waited,
    int completed_services_before,
    int completed_services_after)
{
    return majority_waited &&
        completed_services_after == completed_services_before;
}

inline bool is_valid_workstation_policy(const std::string& policy)
{
    return policy == "vanilla" || policy == "departure_aware" ||
           policy == "pressure_aware";
}

inline bool is_pressure_aware_policy(const std::string& policy)
{
    return policy == "pressure_aware";
}

inline bool uses_workstation_departure_priority(const std::string& policy)
{
    return policy == "departure_aware" || policy == "pressure_aware";
}

inline bool workstation_policy_protects_phase(
    const std::string& policy, WorkstationAgentPhase phase)
{
    return uses_workstation_departure_priority(policy) &&
           phase == WorkstationAgentPhase::TO_EXIT;
}

inline bool workstation_protected_precedes(
    const std::string& policy, WorkstationAgentPhase lhs,
    WorkstationAgentPhase rhs)
{
    return workstation_policy_protects_phase(policy, lhs) &&
           !workstation_policy_protects_phase(policy, rhs);
}

inline bool workstation_preferred_priority(
    const std::string& policy, int lhs_agent, WorkstationAgentPhase lhs_phase,
    int rhs_agent, WorkstationAgentPhase rhs_phase,
    std::pair<int, int>& preferred_priority)
{
    if (workstation_protected_precedes(policy, lhs_phase, rhs_phase))
    {
        preferred_priority = std::make_pair(rhs_agent, lhs_agent);
        return true;
    }
    if (workstation_protected_precedes(policy, rhs_phase, lhs_phase))
    {
        preferred_priority = std::make_pair(lhs_agent, rhs_agent);
        return true;
    }
    return false;
}

inline bool workstation_mandatory_dwell_preferred_priority(
    int lhs_agent, WorkstationAgentPhase lhs_phase,
    int rhs_agent, WorkstationAgentPhase rhs_phase,
    std::pair<int, int>& preferred_priority)
{
    const bool lhs_dwell = lhs_phase == WorkstationAgentPhase::SERVICE;
    const bool rhs_dwell = rhs_phase == WorkstationAgentPhase::SERVICE;
    if (lhs_dwell == rhs_dwell)
        return false;
    if (lhs_dwell)
        preferred_priority = std::make_pair(rhs_agent, lhs_agent);
    else
        preferred_priority = std::make_pair(lhs_agent, rhs_agent);
    return true;
}

inline bool workstation_pressure_active(int pressure)
{
    return pressure >= kWorkstationPressureThreshold;
}

template<typename OccupiesQueue>
inline int count_workstation_pressure(int num_agents, OccupiesQueue occupies_queue)
{
    int pressure = 0;
    for (int agent = 0; agent < num_agents; agent++)
    {
        if (occupies_queue(agent))
            pressure++;
    }
    return pressure;
}

inline std::tuple<int, int, int, int> workstation_privilege_key(
    bool inside_target_queue, int distance, int station_leg_issue_time, int agent_id)
{
    return std::make_tuple(inside_target_queue ? 0 : 1, distance,
                           station_leg_issue_time, agent_id);
}

template<typename KeyFunction>
inline std::vector<int> select_workstation_privileged_agents(
    std::vector<int> inbound_agents, KeyFunction key)
{
    std::sort(inbound_agents.begin(), inbound_agents.end(),
              [&](int lhs, int rhs) { return key(lhs) < key(rhs); });
    if ((int)inbound_agents.size() > kWorkstationPrivilegedInboundCount)
        inbound_agents.resize(kWorkstationPrivilegedInboundCount);
    return inbound_agents;
}
