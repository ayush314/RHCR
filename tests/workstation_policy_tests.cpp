#include "ReservationTable.h"
#include "LRAStar.h"
#include "PIBT.h"
#include "PIBT2.h"
#include "SIPP.h"
#include "StateTimeAStar.h"
#include "WorkstationCommon.h"
#include "WorkstationGraph.h"
#include "WorkstationPolicy.h"

#include <cassert>
#include <climits>
#include <iostream>
#include <map>

#define REQUIRE(condition) do { \
    if (!(condition)) { \
        std::cerr << "Requirement failed: " #condition << std::endl; \
        return 1; \
    } \
} while (false)

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
        weights.assign(2, vector<double>(5, WEIGHT_MAX));
        weights[0][1] = 1;
        weights[1][3] = 1;
        weights[0][4] = 1;
        weights[1][4] = 1;
    }

    bool load_map(string) override { return true; }
};

class ThreeCellGraph : public BasicGraph
{
public:
    ThreeCellGraph()
    {
        rows = 1;
        cols = 3;
        move[0] = -3;
        move[1] = 1;
        move[2] = 3;
        move[3] = -1;
        types.assign(3, "Travel");
        weights.assign(3, vector<double>(5, WEIGHT_MAX));
        weights[0][1] = 1;
        weights[1][3] = 1;
        weights[1][1] = 1;
        weights[2][3] = 1;
        for (int loc = 0; loc < 3; loc++)
            weights[loc][4] = 1;
        heuristics[2] = {2, 1, 0};
    }

    bool load_map(string) override { return true; }
};

class FourCellTieGraph : public BasicGraph
{
public:
    FourCellTieGraph()
    {
        rows = 2;
        cols = 2;
        move[0] = -2;
        move[1] = 1;
        move[2] = 2;
        move[3] = -1;
        types.assign(4, "Travel");
        weights.assign(4, vector<double>(5, WEIGHT_MAX));
        weights[0][1] = weights[0][2] = 1;
        weights[1][3] = weights[1][2] = 1;
        weights[2][0] = weights[2][1] = 1;
        weights[3][0] = weights[3][3] = 1;
        for (int loc = 0; loc < 4; loc++)
            weights[loc][4] = 1;
        heuristics[3] = {0, 0, 0, 0};
    }

    bool load_map(string) override { return true; }
};

class ThreeCellWorkstationGraph : public WorkstationGrid
{
public:
    ThreeCellWorkstationGraph()
    {
        rows = 1;
        cols = 3;
        move[0] = -3;
        move[1] = 1;
        move[2] = 3;
        move[3] = -1;
        types.assign(3, "Travel");
        types[1] = "Workstation";
        weights.assign(3, vector<double>(5, WEIGHT_MAX));
        weights[0][1] = 1;
        weights[1][3] = 1;
        weights[1][1] = 1;
        weights[2][3] = 1;
        for (int loc = 0; loc < 3; loc++)
            weights[loc][4] = 1;
        heuristics[1] = {1, 0, 1};
        heuristics[2] = {2, 1, 0};
        WorkstationStation station;
        station.station_id = 0;
        station.workstation = 1;
        station.exit_cells = {2};
        station.zone_cells = {1, 2};
        stations.push_back(station);
    }

    bool load_map(string) override { return true; }
};

class FiveCellWorkstationGraph : public WorkstationGrid
{
public:
    FiveCellWorkstationGraph()
    {
        rows = 1;
        cols = 5;
        move[0] = -5;
        move[1] = 1;
        move[2] = 5;
        move[3] = -1;
        types.assign(5, "Travel");
        types[4] = "Workstation";
        weights.assign(5, vector<double>(5, WEIGHT_MAX));
        for (int loc = 0; loc < 4; loc++)
        {
            weights[loc][1] = 1;
            weights[loc + 1][3] = 1;
        }
        for (int loc = 0; loc < 5; loc++)
            weights[loc][4] = 1;
        heuristics[4] = {4, 3, 2, 1, 0};
        WorkstationStation station;
        station.station_id = 0;
        station.workstation = 4;
        station.exit_cells = {3};
        station.zone_cells = {1, 2, 3, 4};
        stations.push_back(station);
    }

    bool load_map(string) override { return true; }
};

int main()
{
    assert(is_valid_workstation_policy("vanilla"));
    assert(is_valid_workstation_policy("departure_aware"));
    assert(is_valid_workstation_policy("pressure_aware"));
    assert(!is_valid_workstation_policy("phase_aware"));
    assert(!is_valid_workstation_policy("distance_age"));
    assert(!is_valid_workstation_policy("pressure"));
    assert(uses_workstation_departure_priority("departure_aware"));
    assert(uses_workstation_departure_priority("pressure_aware"));
    assert(!uses_workstation_departure_priority("phase_aware"));
    assert(!uses_workstation_departure_priority("vanilla"));
    assert(workstation_protected_precedes(
        "departure_aware", WorkstationAgentPhase::TO_EXIT,
        WorkstationAgentPhase::TO_PICKUP));
    assert(!workstation_protected_precedes(
        "departure_aware", WorkstationAgentPhase::SERVICE,
        WorkstationAgentPhase::TO_STATION));
    assert(!workstation_protected_precedes(
        "vanilla", WorkstationAgentPhase::TO_EXIT,
        WorkstationAgentPhase::TO_STATION));
    std::pair<int, int> preferred_priority(-1, -1);
    assert(workstation_preferred_priority(
        "departure_aware", 3, WorkstationAgentPhase::TO_EXIT,
        7, WorkstationAgentPhase::TO_STATION, preferred_priority));
    assert(preferred_priority == std::make_pair(7, 3));
    assert(!workstation_preferred_priority(
        "departure_aware", 3, WorkstationAgentPhase::SERVICE,
        7, WorkstationAgentPhase::TO_STATION, preferred_priority));
    assert(workstation_preferred_priority(
        "pressure_aware", 3, WorkstationAgentPhase::TO_EXIT,
        7, WorkstationAgentPhase::TO_STATION, preferred_priority));
    assert(!workstation_preferred_priority(
        "pressure_aware", 3, WorkstationAgentPhase::SERVICE,
        7, WorkstationAgentPhase::TO_STATION, preferred_priority));
    assert(!workstation_policy_protects_phase(
        "pressure_aware", WorkstationAgentPhase::SERVICE));
    assert(!workstation_policy_protects_phase(
        "pressure_aware", WorkstationAgentPhase::TO_STATION));
    assert(workstation_mandatory_dwell_preferred_priority(
        3, WorkstationAgentPhase::SERVICE,
        7, WorkstationAgentPhase::TO_STATION, preferred_priority));
    assert(preferred_priority == std::make_pair(7, 3));
    assert(workstation_mandatory_dwell_preferred_priority(
        3, WorkstationAgentPhase::TO_EXIT,
        7, WorkstationAgentPhase::SERVICE, preferred_priority));
    assert(preferred_priority == std::make_pair(3, 7));
    assert(!workstation_mandatory_dwell_preferred_priority(
        3, WorkstationAgentPhase::SERVICE,
        7, WorkstationAgentPhase::SERVICE, preferred_priority));
    assert(kWorkstationPressureThreshold == 3);
    assert(kWorkstationPrivilegedInboundCount == 2);
    assert(kWorkstationPressureQueueCost == 2);
    const std::vector<bool> occupies_queue = {true, false, true, true};
    assert(count_workstation_pressure(4, [&](int agent) {
        return occupies_queue[agent];
    }) == 3);
    assert(workstation_pressure_active(3));
    assert(!workstation_pressure_active(2));
    assert(workstation_privilege_key(true, 9, 20, 4) <
           workstation_privilege_key(false, 1, 1, 1));
    const std::map<int, std::tuple<int, int, int, int>> privilege_keys = {
        {4, workstation_privilege_key(false, 2, 10, 4)},
        {7, workstation_privilege_key(true, 8, 20, 7)},
        {9, workstation_privilege_key(false, 1, 30, 9)},
    };
    const auto selected = select_workstation_privileged_agents(
        std::vector<int>{4, 7, 9},
        [&](int agent) { return privilege_keys.at(agent); });
    assert((selected == std::vector<int>{7, 9}));

    FiveCellWorkstationGraph pressure_graph;
    vector<WorkstationAgentContext> aligned_contexts(4);
    for (WorkstationAgentContext& context : aligned_contexts)
    {
        context.station_id = 0;
        context.phase = WorkstationAgentPhase::TO_STATION;
        context.task_issue_t = 10;
    }
    const vector<int> pressured_locations = {0, 2, 3, 4};
    const WorkstationPressureSnapshot pressure_snapshot =
        evaluate_workstation_pressure(
            pressure_graph, pressured_locations, aligned_contexts);
    REQUIRE(pressure_snapshot.station_pressure[0] == 3);
    REQUIRE((pressure_snapshot.privileged_inbound_agents[0] ==
             vector<int>{3, 2}));
    REQUIRE(workstation_pressure_action_cost(
        pressure_graph, pressure_snapshot, aligned_contexts, 0, 1) == 2);
    const WorkstationPressureBaseSnapshot pressure_base =
        evaluate_workstation_pressure_without_agent(
            pressure_graph, pressured_locations, aligned_contexts, 0);
    REQUIRE(workstation_pressure_action_cost_from_base(
        pressure_graph, pressure_base, aligned_contexts[0], 0, 0, 1) ==
        workstation_pressure_action_cost(
            pressure_graph, pressure_snapshot, aligned_contexts, 0, 1));
    for (int agent = 0; agent < 4; agent++)
    {
        const WorkstationPressureBaseSnapshot agent_base =
            evaluate_workstation_pressure_without_agent(
                pressure_graph, pressured_locations, aligned_contexts, agent);
        for (int candidate = 0; candidate < 5; candidate++)
        {
            REQUIRE(workstation_pressure_action_cost_from_base(
                pressure_graph, agent_base, aligned_contexts[agent], agent,
                pressured_locations[agent], candidate) ==
                workstation_pressure_action_cost(
                    pressure_graph, pressure_snapshot, aligned_contexts,
                    agent, candidate));
        }
    }
    REQUIRE(workstation_pressure_action_cost(
        pressure_graph, pressure_snapshot, aligned_contexts, 2, 2) == 0);
    vector<WorkstationAgentContext> exit_contexts = aligned_contexts;
    exit_contexts[0].phase = WorkstationAgentPhase::TO_EXIT;
    REQUIRE(workstation_pressure_action_cost(
        pressure_graph, pressure_snapshot, exit_contexts, 0, 1) == 0);
    const WorkstationPressureSnapshot unpressured_snapshot =
        evaluate_workstation_pressure(
            pressure_graph, vector<int>{0, 0, 1, 2}, aligned_contexts);
    REQUIRE(workstation_pressure_action_cost(
        pressure_graph, unpressured_snapshot, aligned_contexts, 0, 1) == 0);

    TestGraph graph;
    ReservationTable reservations(graph);
    reservations.map_size = graph.size();
    reservations.addSoftVertexConstraint(1, 1, 4, 2);
    assert(reservations.getConflictCost(0, 1, 1) == 2);
    assert(reservations.getSoftVertexCost(1, 1) == 2);
    assert(!reservations.isConstrained(0, 1, 1));
    assert(reservations.getConflictCost(0, 1, 4) == 0);
    assert(reservations.isConflicting(0, 1, 2));

    graph.heuristics[0] = {0, 1};
    graph.heuristics[1] = {1, 0};
    StateTimeAStar astar;
    Path weighted_path = astar.run(graph, State(0, 0, -1), {{1, 0}}, reservations);
    assert(!weighted_path.empty());
    assert(astar.path_cost == 3);

    ReservationTable transition_reservations(graph);
    transition_reservations.map_size = graph.size();
    astar.set_transition_cost(
        [](const State&, const State& next, int) {
            return next.location == 1 ? 2 : 0;
        });
    Path transition_path = astar.run(
        graph, State(0, 0, -1), {{1, 0}}, transition_reservations);
    REQUIRE(!transition_path.empty());
    REQUIRE(astar.path_cost == 3);
    astar.clear_transition_cost();
    Path native_path = astar.run(
        graph, State(0, 0, -1), {{1, 0}}, transition_reservations);
    REQUIRE(!native_path.empty());
    REQUIRE(astar.path_cost == 1);

    SIPP sipp;
    Path weighted_sipp_path = sipp.run(graph, State(0, 0, -1), {{1, 0}}, reservations);
    assert(!weighted_sipp_path.empty());
    assert(sipp.path_cost == 3);
    sipp.set_transition_cost(
        [](const State&, const State& next, int) {
            return next.location == 1 ? 2 : 0;
        });
    Path transition_sipp_path = sipp.run(
        graph, State(0, 0, -1), {{1, 0}}, transition_reservations);
    REQUIRE(!transition_sipp_path.empty());
    REQUIRE(sipp.path_cost == 3);
    sipp.clear_transition_cost();
    Path native_sipp_path = sipp.run(
        graph, State(0, 0, -1), {{1, 0}}, transition_reservations);
    REQUIRE(!native_sipp_path.empty());
    REQUIRE(sipp.path_cost == 1);

    ReservationTable dwell_reservations(graph);
    dwell_reservations.map_size = graph.size();
    dwell_reservations.window = 10;
    dwell_reservations.k_robust = 0;
    dwell_reservations.use_cat = false;
    dwell_reservations.hold_endpoints = false;
    dwell_reservations.prioritize_start = true;
    dwell_reservations.addSoftVertexConstraint(1, 2, 4, 10);
    const vector<pair<int, int>> dwell_goals = {
        {1, 0}, {1, 0}, {1, 0}, {0, 0},
    };
    Path astar_dwell_path = astar.run(
        graph, State(0, 0, -1), dwell_goals, dwell_reservations);
    auto verify_dwell_path = [](const Path& path) {
        size_t arrival = 0;
        while (arrival < path.size() && path[arrival].location != 1)
            arrival++;
        return arrival + 3 < path.size() &&
            path[arrival].location == 1 &&
            path[arrival + 1].location == 1 &&
            path[arrival + 2].location == 1 &&
            path[arrival + 3].location == 0;
    };
    REQUIRE(verify_dwell_path(astar_dwell_path));
    Path sipp_dwell_path = sipp.run(
        graph, State(0, 0, -1), dwell_goals, dwell_reservations);
    REQUIRE(verify_dwell_path(sipp_dwell_path));

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

    LRAStar swap_lra(graph, pibt_path_planner);
    swap_lra.simulation_window = 1;
    swap_lra.k_robust = 0;
    const vector<Path> proposed_swap = {
        {State(0, 0, -1), State(1, 1, -1)},
        {State(1, 0, -1), State(0, 1, -1)},
    };
    for (int iteration = 0; iteration < 20; iteration++)
    {
        swap_lra.resolve_conflicts(proposed_swap);
        assert(swap_lra.solution[0][1].location == 0);
        assert(swap_lra.solution[1][1].location == 1);
        assert(swap_lra.num_wait_commands == 2);
    }

    const vector<Path> exhausted_prefixes = {
        {State(0, 0, -1)},
        {State(1, 0, -1)},
    };
    swap_lra.resolve_conflicts(exhausted_prefixes);
    assert(swap_lra.solution[0][1].location == 0);
    assert(swap_lra.solution[1][1].location == 1);

    PIBT failed_pibt(graph, pibt_path_planner);
    failed_pibt.window = 2;
    assert(!failed_pibt.run(
        {State(0, 0, -1), State(1, 0, -1)},
        {{{1, 0}}, {{0, 0}}}, -1));
    assert(failed_pibt.solution.size() == 2);
    assert(failed_pibt.solution[0].size() == 1);
    assert(failed_pibt.solution[1].size() == 1);

    PIBT2 pibt2_age(graph, pibt_path_planner);
    pibt2_age.window = 2;
    pibt2_age.tie_seed = 17;
    pibt2_age.set_executed_priority_age({7});
    REQUIRE(pibt2_age.run({State(0, 0, -1)}, {{{1, 0}}}, 10));
    REQUIRE((pibt2_age.get_executed_priority_age() == std::vector<int>{7}));

    PIBT2 pibt2_swap(graph, pibt_path_planner);
    pibt2_swap.window = 1;
    pibt2_swap.tie_seed = 17;
    REQUIRE(pibt2_swap.run(
        {State(0, 0, -1), State(1, 0, -1)},
        {{{1, 0}}, {{0, 0}}}, 10));
    REQUIRE(pibt2_swap.solution[0][1].location == 0);
    REQUIRE(pibt2_swap.solution[1][1].location == 1);
    REQUIRE(pibt2_swap.backtracks > 0);
    REQUIRE(pibt2_swap.wait_fallbacks == 0);

    StateTimeAStar aligned_pibt_path_planner;
    PIBT2 vanilla_pressure_control(pressure_graph, aligned_pibt_path_planner);
    vanilla_pressure_control.window = 1;
    vanilla_pressure_control.random_tiebreak = false;
    vanilla_pressure_control.set_pibt_policy("vanilla");
    vanilla_pressure_control.set_workstation_context(aligned_contexts);
    vanilla_pressure_control.set_projected_goal_context(
        vector<vector<WorkstationAgentContext>>(4, aligned_contexts));
    const vector<State> pressure_starts = {
        State(0, 0, -1), State(2, 0, -1),
        State(3, 0, -1), State(4, 0, -1),
    };
    const vector<vector<pair<int, int>>> pressure_goals = {
        {{4, 0}}, {{2, 0}}, {{3, 0}}, {{4, 0}},
    };
    REQUIRE(vanilla_pressure_control.run(
        pressure_starts, pressure_goals, 10));
    REQUIRE(vanilla_pressure_control.solution[0][1].location == 1);

    PIBT2 aligned_pressure_pibt(pressure_graph, aligned_pibt_path_planner);
    aligned_pressure_pibt.window = 1;
    aligned_pressure_pibt.random_tiebreak = false;
    aligned_pressure_pibt.set_pibt_policy("pressure_aware");
    aligned_pressure_pibt.set_workstation_context(aligned_contexts);
    aligned_pressure_pibt.set_projected_goal_context(
        vector<vector<WorkstationAgentContext>>(4, aligned_contexts));
    REQUIRE(aligned_pressure_pibt.run(pressure_starts, pressure_goals, 10));
    REQUIRE(aligned_pressure_pibt.solution[0][1].location == 0);
    REQUIRE(aligned_pressure_pibt.pressure_rank_changes > 0);

    ThreeCellGraph three_cell_graph;
    StateTimeAStar pibt2_path_planner;
    PIBT2 pibt2_initial_distance(three_cell_graph, pibt2_path_planner);
    pibt2_initial_distance.window = 1;
    REQUIRE(pibt2_initial_distance.run(
        {State(0, 0, -1)}, {{{2, 0}}}, 10));
    REQUIRE((pibt2_initial_distance.get_priority_initial_distance() ==
             std::vector<int>{2}));
    REQUIRE(pibt2_initial_distance.run(
        {State(1, 0, -1)}, {{{2, 0}}}, 10));
    REQUIRE((pibt2_initial_distance.get_priority_initial_distance() ==
             std::vector<int>{2}));
    REQUIRE(pibt2_initial_distance.run(
        {State(1, 0, -1)}, {{{0, 0}}}, 10));
    REQUIRE((pibt2_initial_distance.get_priority_initial_distance() ==
             std::vector<int>{1}));

    PIBT2 pibt2_dwell(graph, pibt_path_planner);
    pibt2_dwell.window = 4;
    REQUIRE(pibt2_dwell.run(
        {State(0, 0, -1)}, {dwell_goals}, 10));
    REQUIRE(pibt2_dwell.solution[0].size() == 5);
    REQUIRE(pibt2_dwell.solution[0][1].location == 1);
    REQUIRE(pibt2_dwell.solution[0][2].location == 1);
    REQUIRE(pibt2_dwell.solution[0][3].location == 1);
    REQUIRE(pibt2_dwell.solution[0][4].location == 0);

    WorkstationAgentContext inbound_context;
    inbound_context.station_id = 0;
    inbound_context.phase = WorkstationAgentPhase::TO_STATION;
    WorkstationAgentContext exit_context;
    exit_context.station_id = 0;
    exit_context.phase = WorkstationAgentPhase::TO_EXIT;
    PIBT2 departure_order(three_cell_graph, pibt2_path_planner);
    departure_order.window = 1;
    departure_order.set_pibt_policy("departure_aware");
    departure_order.set_executed_priority_age({100, 0});
    departure_order.set_workstation_context({inbound_context, exit_context});
    departure_order.set_projected_goal_context(
        {{inbound_context}, {exit_context}});
    REQUIRE(departure_order.run(
        {State(0, 0, -1), State(1, 0, -1)},
        {{{1, 0}}, {{2, 0}}}, 10));
    REQUIRE(departure_order.solution[0][1].location == 1);
    REQUIRE(departure_order.solution[1][1].location == 2);
    REQUIRE(departure_order.inheritance_calls == 0);

    WorkstationAgentContext service_context;
    service_context.station_id = 0;
    service_context.phase = WorkstationAgentPhase::SERVICE;
    ThreeCellWorkstationGraph three_cell_workstation_graph;
    PIBT2 departure_service_order(three_cell_workstation_graph, pibt2_path_planner);
    departure_service_order.window = 1;
    departure_service_order.set_pibt_policy("departure_aware");
    departure_service_order.set_executed_priority_age({100, 0});
    departure_service_order.set_workstation_context({inbound_context, service_context});
    departure_service_order.set_projected_goal_context(
        {{inbound_context}, {service_context, service_context}});
    REQUIRE(departure_service_order.run(
        {State(0, 0, -1), State(1, 0, -1)},
        {{{1, 0}}, {{1, 0}, {1, 0}}}, 10));
    REQUIRE(departure_service_order.inheritance_calls == 0);

    PIBT2 pressure_service_order(three_cell_workstation_graph, pibt2_path_planner);
    pressure_service_order.window = 1;
    pressure_service_order.set_pibt_policy("pressure_aware");
    pressure_service_order.set_executed_priority_age({100, 0});
    pressure_service_order.set_workstation_context({inbound_context, service_context});
    pressure_service_order.set_projected_goal_context(
        {{inbound_context}, {service_context, service_context}});
    REQUIRE(pressure_service_order.run(
        {State(0, 0, -1), State(1, 0, -1)},
        {{{1, 0}}, {{1, 0}, {1, 0}}}, 10));
    REQUIRE(pressure_service_order.inheritance_calls == 0);

    PIBT2 vanilla_service_order(three_cell_workstation_graph, pibt2_path_planner);
    vanilla_service_order.window = 1;
    vanilla_service_order.set_executed_priority_age({100, 0});
    vanilla_service_order.set_workstation_context({inbound_context, service_context});
    vanilla_service_order.set_projected_goal_context(
        {{inbound_context}, {service_context, service_context}});
    REQUIRE(vanilla_service_order.run(
        {State(0, 0, -1), State(1, 0, -1)},
        {{{1, 0}}, {{1, 0}, {1, 0}}}, 10));
    REQUIRE(vanilla_service_order.inheritance_calls == 0);

    PIBT legacy_departure_service_order(
        three_cell_workstation_graph, pibt2_path_planner);
    legacy_departure_service_order.window = 1;
    legacy_departure_service_order.set_pibt_policy("departure_aware");
    legacy_departure_service_order.set_executed_priority_age({100, 0});
    legacy_departure_service_order.set_workstation_context(
        {inbound_context, service_context});
    legacy_departure_service_order.set_projected_goal_context(
        {{inbound_context}, {service_context, service_context}});
    REQUIRE(legacy_departure_service_order.run(
        {State(0, 0, -1), State(1, 0, -1)},
        {{{1, 0}}, {{1, 0}, {1, 0}}}, 10));
    REQUIRE(legacy_departure_service_order.inheritance_calls == 0);

    WorkstationAgentContext pickup_context;
    pickup_context.station_id = 0;
    pickup_context.phase = WorkstationAgentPhase::TO_PICKUP;
    PIBT2 exhausted_departure_context(three_cell_graph, pibt2_path_planner);
    exhausted_departure_context.window = 1;
    exhausted_departure_context.set_pibt_policy("departure_aware");
    exhausted_departure_context.set_executed_priority_age({0, 100});
    exhausted_departure_context.set_workstation_context(
        {inbound_context, inbound_context});
    exhausted_departure_context.set_projected_goal_context(
        {{pickup_context}, {inbound_context}});
    REQUIRE(exhausted_departure_context.run(
        {State(0, 0, -1), State(1, 0, -1)},
        {{{0, 0}}, {{0, 0}}}, 10));

    PIBT2 exhausted_vanilla_context(three_cell_graph, pibt2_path_planner);
    exhausted_vanilla_context.window = 1;
    exhausted_vanilla_context.set_pibt_policy("vanilla");
    exhausted_vanilla_context.set_executed_priority_age({0, 100});
    exhausted_vanilla_context.set_workstation_context(
        {inbound_context, inbound_context});
    exhausted_vanilla_context.set_projected_goal_context(
        {{pickup_context}, {inbound_context}});
    REQUIRE(exhausted_vanilla_context.run(
        {State(0, 0, -1), State(1, 0, -1)},
        {{{0, 0}}, {{0, 0}}}, 10));
    REQUIRE(exhausted_departure_context.inheritance_calls ==
            exhausted_vanilla_context.inheritance_calls);
    REQUIRE(exhausted_departure_context.backtracks ==
            exhausted_vanilla_context.backtracks);

    PIBT2 inherited_pibt2(three_cell_graph, pibt2_path_planner);
    inherited_pibt2.window = 1;
    inherited_pibt2.tie_seed = 31;
    REQUIRE(inherited_pibt2.run(
        {State(0, 0, -1), State(1, 0, -1)},
        {{{2, 0}}, {{2, 0}}}, 10));
    REQUIRE(inherited_pibt2.solution[0][1].location == 1);
    REQUIRE(inherited_pibt2.solution[1][1].location == 2);
    REQUIRE(inherited_pibt2.inheritance_calls == 1);

    PIBT2 replay_pibt2(three_cell_graph, pibt2_path_planner);
    replay_pibt2.window = 1;
    replay_pibt2.tie_seed = 31;
    REQUIRE(replay_pibt2.run(
        {State(0, 0, -1), State(1, 0, -1)},
        {{{2, 0}}, {{2, 0}}}, 10));
    REQUIRE(replay_pibt2.solution[0][1].location == inherited_pibt2.solution[0][1].location);
    REQUIRE(replay_pibt2.solution[1][1].location == inherited_pibt2.solution[1][1].location);

    FourCellTieGraph tie_graph;
    StateTimeAStar tie_path_planner;
    for (uint64_t tie_seed = 0; tie_seed < 32; tie_seed++)
    {
        PIBT2 absolute_time_pibt(tie_graph, tie_path_planner);
        absolute_time_pibt.tie_seed = tie_seed;
        absolute_time_pibt.window = 2;
        absolute_time_pibt.set_episode_start_timestep(0);
        REQUIRE(absolute_time_pibt.run(
            {State(0, 0, -1)}, {{{3, 0}}}, 10));
        const int replan_start = absolute_time_pibt.solution[0][1].location;
        const int projected_destination =
            absolute_time_pibt.solution[0][2].location;

        absolute_time_pibt.window = 1;
        absolute_time_pibt.set_episode_start_timestep(1);
        absolute_time_pibt.set_executed_priority_age({1});
        REQUIRE(absolute_time_pibt.run(
            {State(replan_start, 0, -1)}, {{{3, 0}}}, 10));
        REQUIRE(absolute_time_pibt.solution[0][1].location ==
                projected_destination);
    }

    PIBT2 failed_pibt2(graph, pibt_path_planner);
    failed_pibt2.window = 2;
    REQUIRE(!failed_pibt2.run(
        {State(0, 0, -1), State(1, 0, -1)},
        {{{1, 0}}, {{0, 0}}}, -1));
    REQUIRE(failed_pibt2.solution.size() == 2);
    REQUIRE(failed_pibt2.solution[0].size() == 1);
    REQUIRE(failed_pibt2.solution[1].size() == 1);

    std::cout << "workstation policy tests passed" << std::endl;
    return 0;
}
