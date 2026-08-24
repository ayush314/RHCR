#include "ReservationTable.h"
#include "PIBT.h"
#include "SIPP.h"
#include "StateTimeAStar.h"
#include "WorkstationCommon.h"

#include <cassert>
#include <climits>
#include <iostream>
#include <map>

class TestGraph : public BasicGraph
{
public:
    TestGraph()
    {
        rows = 1;
        cols = 2;
        move[0] = -2;
        move[1] = 1;
        move[2] = 2;
        move[3] = -1;
        types.assign(2, "Travel");
        weights.assign(2, vector<double>(4, WEIGHT_MAX));
        weights[0][1] = 1;
        weights[1][3] = 1;
    }

    bool load_map(string) override { return true; }
};

int main()
{
    auto require = [](bool condition, const char* message) {
        if (!condition)
        {
            std::cerr << "workstation policy test failed: " << message << std::endl;
            return false;
        }
        return true;
    };
    if (!require(
            workstation_pressure_threshold_for_profile(
                "prevalence_adaptive", -1, {1, 1, 0, 0}) == 2,
            "prevalence-adaptive threshold must rise when pressure is widespread"))
        return 1;
    assert(canonical_workstation_policy("distance_age") == "phase_aware");
    assert(canonical_workstation_policy("pressure") == "pressure_aware");
    assert(workstation_pressure_threshold_for_profile("fixed", -1, {1, 1, 1, 1}) == 1);
    assert(workstation_pressure_threshold_for_profile("prevalence_adaptive", -1, {1, 0, 0, 0}) == 1);
    assert(workstation_pressure_threshold_for_profile("prevalence_adaptive", -1, {1, 1, 0, 0}) == 2);
    assert(workstation_pressure_threshold_for_profile("prevalence_adaptive", 3, {1, 0, 0, 0}) == 3);
    assert(workstation_pressure_zone_cost_for_profile("fixed", 7, 1) == 7);
    assert(workstation_pressure_zone_cost_for_profile("prevalence_adaptive", 7, 1) == 1);
    assert(workstation_pressure_zone_cost_for_profile("prevalence_adaptive", 7, 2) == 2);
    assert(workstation_progress_cost_applies(3, 3));
    assert(workstation_progress_cost_applies(3, 4));
    assert(!workstation_progress_cost_applies(3, 2));
    assert(!workstation_progress_cost_applies(0, 0));
    assert(workstation_progress_distance_limit(3, 1) == 3);
    assert(workstation_progress_distance_limit(3, 2) == 2);
    assert(workstation_progress_distance_limit(3, 3) == 1);
    assert(workstation_progress_distance_limit(3, 4) == -1);
    assert(is_phase_aware_policy("phase_aware"));
    assert(is_phase_aware_policy("pressure_aware"));
    assert(!is_phase_aware_policy("vanilla"));
    assert(is_protected_workstation_phase(WorkstationAgentPhase::SERVICE));
    assert(is_protected_workstation_phase(WorkstationAgentPhase::TO_EXIT));
    assert(!is_protected_workstation_phase(WorkstationAgentPhase::TO_STATION));
    assert(workstation_protected_precedes(
        WorkstationAgentPhase::SERVICE, WorkstationAgentPhase::TO_STATION));
    assert(workstation_protected_precedes(
        WorkstationAgentPhase::TO_EXIT, WorkstationAgentPhase::TO_PICKUP));
    assert(!workstation_protected_precedes(
        WorkstationAgentPhase::SERVICE, WorkstationAgentPhase::TO_EXIT));
    assert(!workstation_protected_precedes(
        WorkstationAgentPhase::TO_STATION, WorkstationAgentPhase::TO_PICKUP));
    assert(is_sustained_workstation_stall(0, 100, 4));
    assert(!is_sustained_workstation_stall(0, 100, 5));
    assert(!is_sustained_workstation_stall(1, 100, 0));
    assert(!is_sustained_workstation_stall(0, 0, 0));
    WorkstationAgentContext pressure_context;
    pressure_context.station_id = 2;
    pressure_context.phase = WorkstationAgentPhase::TO_STATION;
    assert(contributes_to_workstation_pressure(pressure_context, 2));
    pressure_context.phase = WorkstationAgentPhase::TO_PICKUP;
    assert(!contributes_to_workstation_pressure(pressure_context, 2));
    assert(!contributes_to_workstation_pressure(pressure_context, 1));
    pressure_context.phase = WorkstationAgentPhase::SERVICE;
    assert(contributes_to_workstation_pressure(pressure_context, 2));
    assert(!contributes_to_workstation_pressure(pressure_context, 2, false));
    assert(workstation_privilege_key(true, 9, 20, 4) <
           workstation_privilege_key(false, 1, 1, 1));
    assert(workstation_privilege_limit("single", 4, 2, 9) == 1);
    assert(workstation_privilege_limit("wide", 6, 6, 9, 6) == 6);
    assert(workstation_privilege_limit("adaptive", 4, 3, 9) == 3);
    assert(workstation_privilege_limit("adaptive", 4, 6, 9) == 2);
    assert(workstation_privilege_limit("adaptive", 4, 6, 9, 1) == 4);
    assert(workstation_privilege_limit("adaptive", 4, 1, 9, 6) == 2);
    assert(workstation_privilege_limit("scale_adaptive", 4, 1, 4, 0, 12400, 24, 40) == 3);
    assert(workstation_privilege_limit("scale_adaptive", 4, 1, 11, 0, 28750, 312, 40) == 4);
    assert(workstation_privilege_limit("scale_adaptive", 4, 1, 11, 0, 80, 6, 40) == 3);
    assert(!workstation_network_pressure_active({1, 2, 0}, 2, 75));
    assert(workstation_network_pressure_active({2, 2, 0}, 2, 50));
    assert(workstation_pressure_cost_horizon_for_profile("fixed", 5, true) == 5);
    assert(workstation_pressure_cost_horizon_for_profile("network_adaptive", 0, false) == 0);
    assert(workstation_pressure_cost_horizon_for_profile("network_adaptive", 5, true) == 1);
    assert(workstation_network_pressure_scale_eligible(12400, 162, 0));
    assert(!workstation_network_pressure_scale_eligible(12400, 162, 80));
    assert(workstation_network_pressure_scale_eligible(28750, 162, 80));
    assert(!workstation_network_pressure_scale_eligible(100, 0, 80));
    assert(workstation_pressure_cost(2) == 2);
    assert(workstation_pressure_cost(2, 2, 2, true) == 2);
    assert(workstation_pressure_cost(2, 3, 2, true) == 3);
    assert(!workstation_pressure_cost_escalates("occupancy_escalating", 2, 1, 3, 9));
    assert(workstation_pressure_cost_escalates("occupancy_escalating", 2, 1, 6, 9));
    assert(workstation_pressure_cost_escalates("escalating", 2, 1, 0, 9));
    assert(!workstation_soft_pressure_active(1, 1));
    assert(workstation_soft_pressure_active(2, 1));
    assert(workstation_soft_pressure_active(3, 3));
    assert(workstation_pressure_cost_active(3, 3, 3, "zone"));
    assert(!workstation_pressure_cost_active(3, 3, 3, "excess_wip"));
    assert(workstation_pressure_cost_active(4, 3, 3, "excess_wip"));
    assert(workstation_incumbent_grace_applies("incumbent_grace", 5, 5, 9, true));
    assert(!workstation_incumbent_grace_applies("incumbent_grace", 5, 6, 9, true));
    assert(!workstation_incumbent_grace_applies("incumbent_grace", 5, 5, 9, false));
    assert(!workstation_incumbent_grace_applies("zone", 5, 5, 9, true));
    assert(workstation_entry_only_cost_applies("entry_only", 5, 9, false, false));
    assert(workstation_entry_only_cost_applies("entry_only", 5, 9, true, true));
    assert(!workstation_entry_only_cost_applies("entry_only", 5, 9, true, false));
    assert(!workstation_entry_only_cost_applies("entry_only", 9, 9, true, true));
    assert(workstation_entry_only_cost_applies("zone", 5, 9, true, false));
    assert(workstation_enter_only_cost_applies("enter_only", false));
    assert(!workstation_enter_only_cost_applies("enter_only", true));
    assert(workstation_enter_only_cost_applies("zone", true));
    assert(workstation_deeper_only_cost_applies("deeper_only", 3, 2));
    assert(!workstation_deeper_only_cost_applies("deeper_only", 3, 3));
    assert(!workstation_deeper_only_cost_applies("deeper_only", 3, 4));
    assert(workstation_deeper_only_cost_applies("zone", 3, 4));
    assert(workstation_busy_only_cost_applies("busy_only", true));
    assert(!workstation_busy_only_cost_applies("busy_only", false));
    assert(workstation_busy_only_cost_applies("zone", false));
    assert(workstation_front_runner_ready(0));
    assert(workstation_front_runner_ready(1));
    assert(!workstation_front_runner_ready(2));
    assert(workstation_effective_pressure_lookahead_radius(
               "fixed", 50, 80, 6, 40) == 50);
    assert(workstation_effective_pressure_lookahead_radius(
               "scale_adaptive", 50, 80, 6, 40) == 0);
    assert(workstation_effective_pressure_lookahead_radius(
               "scale_adaptive", 50, 8000, 162, 40) == 50);
    const std::map<int, std::tuple<int, int, int, int>> privilege_keys = {
        {4, workstation_privilege_key(false, 2, 10, 4)},
        {7, workstation_privilege_key(true, 8, 20, 7)},
        {9, workstation_privilege_key(false, 1, 30, 9)},
    };
    const auto selected = select_workstation_privileged_agents(
        std::vector<int>{4, 7, 9},
        [&](int agent) { return privilege_keys.at(agent); },
        "adaptive", 4, 6, 9);
    assert((selected == std::vector<int>{7, 9}));
    const auto lookahead_selected = select_workstation_privileged_agents(
        std::vector<int>{4, 7, 9},
        [&](int agent) { return privilege_keys.at(agent); },
        "adaptive", 4, 6, 9, 1);
    assert((lookahead_selected == std::vector<int>{7, 9, 4}));

    TestGraph graph;
    ReservationTable reservations(graph);
    reservations.map_size = graph.size();
    reservations.addSoftVertexConstraint(1, 1, 4, 2);
    assert(reservations.getConflictCost(0, 1, 1) == 2);
    assert(reservations.getSoftVertexCost(1, 1) == 2);
    assert(!reservations.isConstrained(0, 1, 1));
    assert(reservations.getConflictCost(0, 1, 4) == 0);
    assert(reservations.isConflicting(0, 1, 2));

    reservations.clear();
    reservations.window = 5;
    reservations.k_robust = 0;
    reservations.use_cat = false;
    reservations.prioritize_start = false;
    reservations.num_of_agents = 2;
    const list<tuple<int, int, int>> initial_constraints = {
        make_tuple(0, 1, 3), make_tuple(1, 0, 2)};
    reservations.prepareInitialConstraints(initial_constraints);
    vector<Path*> empty_paths(2, nullptr);
    reservations.build(empty_paths, initial_constraints, unordered_set<int>(), 1, 0);
    assert(reservations.isConstrained(0, 1, 2));
    assert(!reservations.isConstrained(0, 1, 3));

    graph.heuristics[1] = {1, 0};
    StateTimeAStar astar;
    Path weighted_path = astar.run(graph, State(0, 0, -1), {{1, 0}}, reservations);
    assert(!weighted_path.empty());
    assert(astar.path_cost == 3);

    SIPP sipp;
    Path weighted_sipp_path = sipp.run(graph, State(0, 0, -1), {{1, 0}}, reservations);
    assert(!weighted_sipp_path.empty());
    assert(sipp.path_cost == 3);

    StateTimeAStar pibt_path_planner;
    PIBT age_pibt(graph, pibt_path_planner);
    age_pibt.window = 2;
    age_pibt.set_executed_priority_age({7});
    assert(age_pibt.run({State(0, 0, -1)}, {{{1, 0}}}, 10));
    assert((age_pibt.get_executed_priority_age() == std::vector<int>{7}));

    PIBT swap_pibt(graph, pibt_path_planner);
    swap_pibt.window = 1;
    assert(swap_pibt.run(
        {State(0, 0, -1), State(1, 0, -1)},
        {{{1, 0}}, {{0, 0}}}, 10));
    assert(swap_pibt.solution[0][1].location == 0);
    assert(swap_pibt.solution[1][1].location == 1);

    std::cout << "workstation policy tests passed" << std::endl;
    return 0;
}
