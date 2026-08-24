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

inline std::string canonical_workstation_policy(const std::string& policy)
{
    if (policy == "distance_age")
        return "phase_aware";
    if (policy == "pressure")
        return "pressure_aware";
    return policy;
}

inline bool is_phase_aware_policy(const std::string& policy)
{
    const std::string canonical = canonical_workstation_policy(policy);
    return canonical == "phase_aware" || canonical == "pressure_aware";
}

inline bool is_pressure_aware_policy(const std::string& policy)
{
    const std::string canonical = canonical_workstation_policy(policy);
    return canonical == "pressure_aware";
}

inline int workstation_pressure_threshold_for_profile(
    const std::string& profile, int override_threshold,
    const std::vector<int>& station_pressures)
{
    if (override_threshold > 0)
        return override_threshold;
    if (profile == "prevalence_adaptive" && !station_pressures.empty())
    {
        int stations_with_any_pressure = 0;
        for (int pressure : station_pressures)
        {
            if (pressure >= 1)
                stations_with_any_pressure++;
        }
        // Use the more permissive local trigger only when pressure is sparse.
        if (stations_with_any_pressure * 100 <=
            static_cast<int>(station_pressures.size()) * 25)
            return 1;
        return 2;
    }
    return 1;
}

inline int workstation_pressure_zone_cost_for_profile(
    const std::string& profile, int configured_cost, int active_threshold)
{
    if (profile == "prevalence_adaptive")
        return active_threshold == 1 ? 1 : 2;
    return std::max(1, configured_cost);
}

inline bool is_protected_workstation_phase(WorkstationAgentPhase phase)
{
    return phase == WorkstationAgentPhase::SERVICE ||
           phase == WorkstationAgentPhase::TO_EXIT;
}

inline bool workstation_protected_precedes(WorkstationAgentPhase lhs,
                                           WorkstationAgentPhase rhs)
{
    return is_protected_workstation_phase(lhs) &&
           !is_protected_workstation_phase(rhs);
}

inline bool contributes_to_workstation_pressure(
    const WorkstationAgentContext& context, int station_id,
    bool include_protected_phases = true)
{
    if (context.station_id != station_id)
        return false;
    if (context.phase == WorkstationAgentPhase::TO_STATION)
        return true;
    return include_protected_phases &&
        (context.phase == WorkstationAgentPhase::SERVICE ||
         context.phase == WorkstationAgentPhase::TO_EXIT);
}

inline bool is_sustained_workstation_stall(int completed_services,
                                           int eligible_steps,
                                           int moved_steps)
{
    return completed_services == 0 && eligible_steps > 0 &&
           static_cast<double>(moved_steps) / eligible_steps < 0.05;
}

inline std::tuple<int, int, int, int> workstation_privilege_key(
    bool boundary_seen, int distance, int station_leg_issue_time, int agent_id)
{
    return std::make_tuple(boundary_seen ? 0 : 1, distance,
                           station_leg_issue_time, agent_id);
}

inline int workstation_privilege_limit(const std::string& admission, int base_limit,
                                       int pressure, int zone_capacity,
                                       int zone_occupancy = -1,
                                       int agent_count = -1,
                                       int station_count = -1,
                                       int min_agents_per_station = -1)
{
    if (admission == "single")
        return 1;
    int limit = base_limit < 1 ? 1 : base_limit;
    // Development-only wide admission keeps the configured privileged prefix
    // intact. It tests whether throughput is being limited by the adaptive
    // occupancy cap rather than by pressure ranking itself.
    if (admission == "wide")
        return limit;
    if (admission == "scale_adaptive" &&
        (zone_capacity < 6 || agent_count < 0 || station_count <= 0 ||
         min_agents_per_station <= 0 ||
         static_cast<long long>(agent_count) <
             static_cast<long long>(station_count) * min_agents_per_station))
        limit = std::min(limit, 3);
    zone_capacity = zone_capacity < 1 ? 1 : zone_capacity;
    // Lookahead pressure describes approaching WIP; admission should tighten
    // only when the physical workstation zone itself is filling.
    const int occupancy = zone_occupancy < 0 ? pressure : zone_occupancy;
    if (occupancy * 3 >= zone_capacity)
        limit = limit < 3 ? limit : 3;
    if (occupancy * 3 >= zone_capacity * 2)
        limit = limit < 2 ? limit : 2;
    return limit;
}

inline bool workstation_network_pressure_active(const std::vector<int>& pressures,
                                                int threshold,
                                                int fraction)
{
    if (pressures.empty())
        return false;
    threshold = threshold < 1 ? 1 : threshold;
    fraction = std::max(1, std::min(100, fraction));
    int active = 0;
    for (int pressure : pressures)
    {
        if (pressure >= threshold)
            active++;
    }
    return active * 100 >= static_cast<int>(pressures.size()) * fraction;
}

// PBS uses a longer soft reservation horizon when pressure is localized, but
// limits the soft preference to the next executed step when pressure is
// widespread. A horizon of zero retains the existing full-window behavior.
inline int workstation_pressure_cost_horizon_for_profile(
    const std::string& profile, int configured_horizon,
    bool network_pressure_active)
{
    if (profile == "network_adaptive")
        return network_pressure_active ? 1 : 0;
    return std::max(0, configured_horizon);
}

inline bool workstation_network_pressure_scale_eligible(
    int agent_count, int station_count, int min_agents_per_station)
{
    if (min_agents_per_station <= 0)
        return true;
    if (agent_count < 0 || station_count <= 0)
        return false;
    return static_cast<long long>(agent_count) >=
        static_cast<long long>(station_count) * min_agents_per_station;
}

inline int workstation_pressure_cost(int base_cost, int pressure = -1,
                                     int threshold = -1,
                                     bool escalating = false)
{
    int cost = std::max(1, base_cost);
    if (escalating && pressure > threshold && threshold > 0)
        cost++;
    return cost;
}

inline bool workstation_pressure_cost_escalates(
    const std::string& mode, int pressure, int threshold,
    int zone_occupancy, int zone_capacity)
{
    if (pressure <= threshold || threshold <= 0)
        return false;
    if (mode == "escalating")
        return true;
    return mode == "occupancy_escalating" && zone_capacity > 0 &&
        zone_occupancy * 3 >= zone_capacity * 2;
}

inline bool workstation_soft_pressure_active(int zone_occupancy,
                                             int configured_threshold)
{
    const int threshold = std::max(2, configured_threshold);
    return zone_occupancy >= threshold;
}

inline bool workstation_pressure_cost_active(
    int pressure, int zone_occupancy, int configured_threshold,
    const std::string& activation)
{
    if (!workstation_soft_pressure_active(zone_occupancy, configured_threshold))
        return false;
    if (activation == "excess_wip")
        return pressure > zone_occupancy;
    return true;
}

inline bool workstation_incumbent_grace_applies(
    const std::string& activation, int current_location, int candidate_location,
    int workstation, bool current_in_zone)
{
    return activation == "incumbent_grace" && current_in_zone &&
        current_location != workstation && candidate_location == current_location;
}

inline bool workstation_entry_only_cost_applies(
    const std::string& activation, int candidate_location, int workstation,
    bool current_in_zone, bool candidate_is_exit)
{
    if (activation != "entry_only" || !current_in_zone)
        return true;
    return candidate_location != workstation && candidate_is_exit;
}

// A pure admission preference: once an agent is already in the zone, do not
// charge it for holding or leaving. The caller still restricts the cost to
// the configured pressure-cost cells, so outside agents are charged only when
// their candidate would occupy the pressured zone.
inline bool workstation_enter_only_cost_applies(
    const std::string& activation, bool current_in_zone)
{
    return activation != "enter_only" || !current_in_zone;
}

// A directional admission preference: only a candidate that advances toward
// the workstation receives the soft cost. Holding, lateral motion, and exit
// actions remain available at their native preference.
inline bool workstation_deeper_only_cost_applies(
    const std::string& activation, int current_distance, int candidate_distance)
{
    return activation != "deeper_only" || candidate_distance < current_distance;
}

// A capacity-aware admission preference: only apply the soft cost while the
// target workstation is occupied by an agent in its service-holding phase.
inline bool workstation_busy_only_cost_applies(
    const std::string& activation, bool workstation_busy)
{
    return activation != "busy_only" || workstation_busy;
}

inline bool workstation_front_runner_ready(int distance)
{
    return distance >= 0 && distance <= 1;
}

// Shared progress term used by both PBS path construction and PIBT action
// ranking. It remains a soft preference: waiting and detours stay feasible.
inline bool workstation_progress_cost_applies(int current_distance,
                                              int candidate_distance)
{
    return current_distance > 0 && candidate_distance >= current_distance;
}

// At PBS local timestep t, this is the first distance that fails to maintain
// strict one-cell-per-step progress. Timesteps after arrival are uncharged.
inline int workstation_progress_distance_limit(int current_distance,
                                               int local_timestep)
{
    if (current_distance <= 0 || local_timestep <= 0 ||
        local_timestep > current_distance)
        return -1;
    return current_distance - local_timestep + 1;
}

inline int workstation_effective_pressure_lookahead_radius(
    const std::string& profile, int configured_radius, int agent_count,
    int station_count, int min_agents_per_station)
{
    if (configured_radius <= 0 || profile != "scale_adaptive")
        return std::max(0, configured_radius);
    if (station_count <= 0 || min_agents_per_station <= 0 ||
        static_cast<long long>(agent_count) <
            static_cast<long long>(station_count) * min_agents_per_station)
        return 0;
    return configured_radius;
}

template<typename KeyFunction>
inline std::vector<int> select_workstation_privileged_agents(
    std::vector<int> inbound_agents, KeyFunction key,
    const std::string& admission, int base_limit, int pressure, int zone_capacity,
    int zone_occupancy = -1, int agent_count = -1, int station_count = -1,
    int min_agents_per_station = -1)
{
    const int limit = workstation_privilege_limit(
        admission, base_limit, pressure, zone_capacity, zone_occupancy,
        agent_count, station_count, min_agents_per_station);
    auto less = [&](int lhs, int rhs) { return key(lhs) < key(rhs); };
    if ((int)inbound_agents.size() > limit)
    {
        // Only the privileged prefix is observable by either solver. Avoid a
        // full sort of the inbound population when the prefix is small.
        std::partial_sort(inbound_agents.begin(), inbound_agents.begin() + limit,
                          inbound_agents.end(), less);
        inbound_agents.resize(limit);
    }
    else
    {
        std::sort(inbound_agents.begin(), inbound_agents.end(), less);
    }
    return inbound_agents;
}
