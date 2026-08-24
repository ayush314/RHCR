#include "WorkstationSystem.h"

#include "PBS.h"
#include "PIBT.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <numeric>
#include <random>
#include <sys/resource.h>
#include <unordered_map>

namespace
{
constexpr int kDefaultPressureThreshold = 1;

double percentile(std::vector<int> values, double pct)
{
    if (values.empty())
        return 0;
    std::sort(values.begin(), values.end());
    double rank = (pct / 100.0) * (values.size() - 1);
    size_t lo = (size_t)std::floor(rank);
    size_t hi = (size_t)std::ceil(rank);
    if (lo == hi)
        return values[lo];
    double frac = rank - lo;
    return values[lo] * (1.0 - frac) + values[hi] * frac;
}

double percentile(std::vector<double> values, double pct)
{
    if (values.empty())
        return 0;
    std::sort(values.begin(), values.end());
    double rank = (pct / 100.0) * (values.size() - 1);
    size_t lo = (size_t)std::floor(rank);
    size_t hi = (size_t)std::ceil(rank);
    if (lo == hi)
        return values[lo];
    double frac = rank - lo;
    return values[lo] * (1.0 - frac) + values[hi] * frac;
}

int effective_pressure_threshold(int override_threshold)
{
    if (override_threshold > 0)
        return override_threshold;
    return kDefaultPressureThreshold;
}
} // namespace

WorkstationSystem::WorkstationSystem(const WorkstationGrid& G, MAPFSolver& solver) :
    BasicSystem(G, solver), G(G) {}

WorkstationSystem::~WorkstationSystem() {}

void WorkstationSystem::initialize()
{
    initialize_solvers();
    timestep = 0;
    starts.resize(num_of_drives);
    goal_locations.resize(num_of_drives);
    paths.resize(num_of_drives);
    finished_tasks.resize(num_of_drives);
    workstation_agents.resize(num_of_drives);
    projected_goal_context.resize(num_of_drives);
    queue_wait_samples.clear();
    mean_plan_ms_samples.clear();
    plan_timestep_samples.clear();
    pressure_active_samples.clear();
    pressured_station_fraction_samples.clear();
    zone_occupancy_fraction_samples.clear();
    target_queue_occupancy_samples.clear();
    pibt_executed_priority_age.assign(num_of_drives, 0);
    execution_progress.clear();
    distance_traveled = 0;
    completed_services = 0;
    planning_episodes = 0;
    pressure_active_episodes = 0;
    traffic_jam_episodes = 0;
    pibt_inheritance_calls_total = 0;
    pibt_backtracks_total = 0;
    pibt_wait_fallbacks_total = 0;
    pibt_greedy_repairs_total = 0;
    pibt_pressure_rank_changes_total = 0;
    pibt_pressure_active_agents_total = 0;
    pibt_pressure_candidate_hits_total = 0;
    pibt_budget_extensions_total = 0;
    termination_reason = "not_started";
    termination_timestep = -1;
    terminated_by_traffic_jam = false;
    terminated_by_commit_repair_failure = false;
    terminated_by_solver_failure = false;
    initialize_start_locations();
    initialize_tasks();
}

void WorkstationSystem::initialize_start_locations()
{
    std::vector<int> free_cells = G.free_start_cells;
    std::shuffle(free_cells.begin(), free_cells.end(), std::default_random_engine(seed));
    for (int k = 0; k < num_of_drives; k++)
    {
        starts[k] = State(free_cells[k], 0, -1);
        paths[k].emplace_back(starts[k]);
        finished_tasks[k].emplace_back(free_cells[k], 0);
    }
}

void WorkstationSystem::initialize_tasks()
{
    for (int k = 0; k < num_of_drives; k++)
    {
        append_random_task(workstation_agents[k]);
        ensure_lookahead_tasks(workstation_agents[k], 2);
    }
}

int WorkstationSystem::pick_next_station(int previous_station) const
{
    if ((int)G.stations.size() <= 1)
        return 0;
    int next = rand() % G.stations.size();
    while (next == previous_station)
    {
        next = rand() % G.stations.size();
    }
    return next;
}

int WorkstationSystem::pick_next_endpoint(int previous_endpoint) const
{
    if ((int)G.pickup_endpoints.size() <= 1)
        return G.pickup_endpoints.front();
    int next = G.pickup_endpoints[rand() % G.pickup_endpoints.size()];
    while (next == previous_endpoint)
    {
        next = G.pickup_endpoints[rand() % G.pickup_endpoints.size()];
    }
    return next;
}

void WorkstationSystem::append_random_task(WorkstationAgentState& agent)
{
    int previous_station = agent.tasks.empty() ? -1 : agent.tasks.back().station_id;
    int previous_endpoint = agent.tasks.empty() ? agent.last_endpoint : agent.tasks.back().endpoint_target;
    WorkstationTask task;
    task.station_id = pick_next_station(previous_station);
    task.endpoint_target = pick_next_endpoint(previous_endpoint);
    agent.tasks.push_back(task);
}

void WorkstationSystem::ensure_lookahead_tasks(WorkstationAgentState& agent, size_t min_tasks)
{
    while (agent.tasks.size() < min_tasks)
    {
        append_random_task(agent);
    }
}

int WorkstationSystem::agent_station_id(int agent_id) const
{
    const auto& agent = workstation_agents[agent_id];
    if (agent.phase == WorkstationRuntimePhase::TO_STATION ||
        agent.phase == WorkstationRuntimePhase::TO_PICKUP ||
        agent.phase == WorkstationRuntimePhase::SERVICE)
    {
        if (agent.tasks.empty())
            return -1;
        return agent.tasks.front().station_id;
    }
    if (agent.phase == WorkstationRuntimePhase::TO_EXIT)
        return agent.exit_station_id;
    return -1;
}

int WorkstationSystem::station_pressure(int station_id) const
{
    if (station_id < 0 || station_id >= (int)G.stations.size())
        return 0;

    const auto& station = G.stations[station_id];
    const int effective_lookahead_radius =
        workstation_effective_pressure_lookahead_radius(
            pressure_lookahead_profile, pressure_lookahead_radius,
            num_of_drives, (int)G.stations.size(),
            pressure_lookahead_min_agents_per_station);
    int pressure = 0;
    for (int agent_id = 0; agent_id < num_of_drives; agent_id++)
    {
        const auto& agent = workstation_agents[agent_id];
        if (pressure_population == "inbound_only" &&
            agent.phase != WorkstationRuntimePhase::TO_STATION)
            continue;
        bool targets_station =
            ((agent.phase == WorkstationRuntimePhase::TO_STATION ||
              agent.phase == WorkstationRuntimePhase::SERVICE) &&
             !agent.tasks.empty() && agent.tasks.front().station_id == station_id) ||
            (agent.phase == WorkstationRuntimePhase::TO_EXIT &&
             agent.exit_station_id == station_id);
        if (!targets_station)
            continue;
        int loc = starts[agent_id].location;
        const bool in_zone = loc != station.workstation &&
            station.zone_cells.find(loc) != station.zone_cells.end();
        const bool in_lookahead = effective_lookahead_radius > 0 &&
            agent.phase == WorkstationRuntimePhase::TO_STATION &&
            loc != station.workstation &&
            G.distance_to_workstation(station_id, loc) <= effective_lookahead_radius;
        if (in_zone || in_lookahead)
            pressure++;
    }
    return pressure;
}

void WorkstationSystem::build_goal_sequence(int agent_id)
{
    goal_locations[agent_id].clear();
    projected_goal_context[agent_id].clear();
    auto& agent = workstation_agents[agent_id];
    ensure_lookahead_tasks(agent, 2);
    auto append_goal = [&](int loc, WorkstationAgentPhase phase, int station_id,
                           const WorkstationTask& task) {
        goal_locations[agent_id].emplace_back(loc, 0);
        WorkstationAgentContext context;
        context.station_id = station_id;
        context.current_t = timestep;
        context.boundary_entry_t = task.boundary_t;
        context.task_issue_t = task.issue_t;
        context.phase = phase;
        projected_goal_context[agent_id].push_back(context);
    };
    auto append_service_wait = [&](int station_id, int remaining_dwell,
                                   WorkstationAgentPhase phase,
                                   const WorkstationTask& task) {
        (void)remaining_dwell;
        int workstation = G.stations[station_id].workstation;
        // A terminal workstation goal is held by RHCR path semantics. Repeated
        // co-located SIPP goals can force a leave-and-return cycle.
        append_goal(workstation, phase, station_id, task);
    };

    const WorkstationTask& current = agent.tasks.front();
    const WorkstationTask& next = agent.tasks[1];

    if (agent.phase == WorkstationRuntimePhase::TO_PICKUP)
    {
        append_goal(current.endpoint_target, WorkstationAgentPhase::TO_PICKUP,
                    current.station_id, current);
        append_service_wait(current.station_id, workstation_service_time,
                            WorkstationAgentPhase::TO_STATION, current);
        return;
    }

    if (agent.phase == WorkstationRuntimePhase::TO_STATION)
    {
        append_service_wait(current.station_id, workstation_service_time,
                            WorkstationAgentPhase::TO_STATION, current);
        return;
    }

    if (agent.phase == WorkstationRuntimePhase::SERVICE)
    {
        int remaining_dwell = std::max(agent.service_complete_t - timestep, 0);
        append_service_wait(current.station_id, remaining_dwell,
                            WorkstationAgentPhase::SERVICE, current);
        return;
    }

    append_goal(agent.exit_target, WorkstationAgentPhase::TO_EXIT,
                agent.exit_station_id, current);
    append_goal(next.endpoint_target, WorkstationAgentPhase::TO_PICKUP,
                next.station_id, next);
    append_service_wait(next.station_id, workstation_service_time,
                        WorkstationAgentPhase::TO_STATION, next);
}

void WorkstationSystem::update_goal_locations()
{
    for (int k = 0; k < num_of_drives; k++)
    {
        build_goal_sequence(k);
    }
}

void WorkstationSystem::sync_solver_context()
{
    PBS* pbs = dynamic_cast<PBS*>(&solver);
    PIBT* pibt = dynamic_cast<PIBT*>(&solver);
    std::vector<WorkstationAgentContext> context(num_of_drives);
    for (int k = 0; k < num_of_drives; k++)
    {
        context[k].current_t = timestep;
        const auto& agent = workstation_agents[k];
        auto& ctx = context[k];
        if (agent.phase == WorkstationRuntimePhase::TO_STATION)
        {
            ctx.station_id = agent.tasks.front().station_id;
            ctx.boundary_entry_t = agent.tasks.front().boundary_t;
            ctx.task_issue_t = agent.tasks.front().issue_t;
            ctx.phase = WorkstationAgentPhase::TO_STATION;
        }
        else if (agent.phase == WorkstationRuntimePhase::SERVICE)
        {
            ctx.station_id = agent.tasks.front().station_id;
            ctx.boundary_entry_t = agent.tasks.front().boundary_t;
            ctx.task_issue_t = agent.tasks.front().issue_t;
            ctx.phase = WorkstationAgentPhase::SERVICE;
        }
        else if (agent.phase == WorkstationRuntimePhase::TO_PICKUP)
        {
            ctx.station_id = agent.tasks.front().station_id;
            ctx.task_issue_t = agent.tasks.front().issue_t;
            ctx.phase = WorkstationAgentPhase::TO_PICKUP;
        }
        else if (agent.phase == WorkstationRuntimePhase::TO_EXIT)
        {
            ctx.station_id = agent.exit_station_id;
            ctx.phase = WorkstationAgentPhase::TO_EXIT;
        }
    }

    if (pbs != nullptr)
    {
        pbs->set_workstation_policy(station_policy);
        pbs->set_workstation_pressure_threshold(workstation_pressure_threshold);
        pbs->set_pressure_profile(pressure_profile);
        pbs->set_pressure_lookahead_radius(pressure_lookahead_radius);
        pbs->set_pressure_lookahead_profile(pressure_lookahead_profile);
        pbs->set_pressure_lookahead_min_agents_per_station(
            pressure_lookahead_min_agents_per_station);
        pbs->set_network_pressure_fraction(pibt_network_pressure_fraction);
        pbs->set_workstation_context(context);
        pbs->set_projected_goal_context(projected_goal_context);
        pbs->pressure_admission = pressure_admission;
        pbs->pressure_cost_mode = pressure_cost_mode;
        pbs->pressure_cost_scope = pressure_cost_scope;
        pbs->pressure_cost_activation = pressure_cost_activation;
        pbs->pressure_population = pressure_population;
        pbs->pressure_inbound_limit = pressure_inbound_limit;
        pbs->pressure_cost_occupancy_threshold = pressure_cost_occupancy_threshold;
        pbs->set_pressure_cost_horizon(pressure_cost_horizon);
        pbs->set_pressure_cost_horizon_profile(pressure_cost_horizon_profile);
        pbs->set_pressure_local_action_only(pressure_local_action_only);
        pbs->set_pressure_front_runner_priority(pressure_front_runner_priority);
        pbs->set_pressure_front_runner_zone_only(pressure_front_runner_zone_only);
        pbs->set_pressure_front_runner_ready_priority(pressure_front_runner_ready_priority);
        pbs->workstation_pressure_zone_cost = pressure_zone_cost;
        pbs->set_pressure_front_progress_cost(pressure_front_progress_cost);
        pbs->set_pressure_exit_progress_cost(pressure_exit_progress_cost);
        pbs->set_pressure_ready_slot_priority(pressure_ready_slot_priority);
    }

    if (pibt != nullptr)
    {
        pibt->set_pibt_policy(pibt_policy);
        pibt->tie_seed = (uint64_t)seed;
        pibt->set_workstation_pressure_threshold(workstation_pressure_threshold);
        pibt->set_pressure_profile(pressure_profile);
        pibt->set_pressure_lookahead_radius(pressure_lookahead_radius);
        pibt->set_pressure_lookahead_profile(pressure_lookahead_profile);
        pibt->set_pressure_lookahead_min_agents_per_station(
            pressure_lookahead_min_agents_per_station);
        pibt->set_workstation_context(context);
        pibt->set_projected_goal_context(projected_goal_context);
        pibt->set_pressure_admission(pressure_admission);
        pibt->set_pressure_cost_mode(pressure_cost_mode);
        pibt->pressure_cost_scope = pressure_cost_scope;
        pibt->pressure_cost_activation = pressure_cost_activation;
        pibt->set_pressure_population(pressure_population);
        pibt->set_pressure_inbound_limit(pressure_inbound_limit);
        pibt->set_pressure_cost_occupancy_threshold(pressure_cost_occupancy_threshold);
        pibt->set_assignment_budget_factor(pibt_assignment_budget_factor);
        pibt->set_pressure_assignment_extension_factor(
            pibt_pressure_assignment_extension_factor);
        pibt->set_network_pressure_fraction(pibt_network_pressure_fraction);
        pibt->set_network_pressure_min_agents_per_station(
            pibt_network_pressure_min_agents_per_station);
        pibt->set_global_front_runner_priority(pibt_global_front_runner_priority);
        pibt->set_front_runner_priority(pibt_front_runner_priority);
        pibt->set_front_runner_ready_priority(pibt_front_runner_ready_priority);
        pibt->pressure_zone_cost = pressure_zone_cost;
        pibt->set_pressure_front_progress_cost(pressure_front_progress_cost);
        pibt->set_pressure_exit_progress_cost(pressure_exit_progress_cost);
        pibt->set_pressure_ready_slot_priority(pressure_ready_slot_priority);
        pibt->set_executed_priority_age(pibt_executed_priority_age);
    }
}

bool WorkstationSystem::solve_workstation_episode()
{
    update_initial_constraints(solver.initial_constraints);
    bool solved = solver.run(starts, goal_locations, time_limit);
    if (solved)
    {
        update_paths(solver.solution, INT_MAX);
    }
    else if (solver.get_name() == "PBS" && !native_failures_only)
    {
        LRAStar lra(G, solver.path_planner);
        lra.simulation_window = simulation_window;
        lra.k_robust = k_robust;
        lra.resolve_conflicts(solver.solution);
        update_paths(lra.solution, INT_MAX);
        solved = true;
    }
    if (log && solved)
        solver.save_constraints_in_goal_node(outfile + "/goal_nodes/" + std::to_string(timestep) + ".gv");
    if (log)
        solver.save_search_tree(outfile + "/search_trees/" + std::to_string(timestep) + ".gv");
    solver.save_results(outfile + "/solver.csv", std::to_string(timestep) + ","
                        + std::to_string(num_of_drives) + "," + std::to_string(seed));
    return solved;
}

void WorkstationSystem::record_episode_diagnostics()
{
    planning_episodes++;
    const int pressure_threshold = effective_pressure_threshold(workstation_pressure_threshold);
    int pressured_stations = 0;
    double occupancy_fraction_sum = 0;
    for (size_t station_id = 0; station_id < G.stations.size(); station_id++)
    {
        int pressure = station_pressure((int)station_id);
        if (pressure >= pressure_threshold)
            pressured_stations++;
        const auto& station = G.stations[station_id];
        int zone_capacity = std::max(
            1,
            (int)station.zone_cells.size() -
                (station.zone_cells.find(station.workstation) != station.zone_cells.end() ? 1 : 0));
        occupancy_fraction_sum += static_cast<double>(pressure) / zone_capacity;
    }
    bool pressure_active = pressured_stations > 0;
    if (pressure_active)
        pressure_active_episodes++;
    pressure_active_samples.push_back(pressure_active ? 1 : 0);
    double station_count = std::max<size_t>(1, G.stations.size());
    pressured_station_fraction_samples.push_back(pressured_stations / station_count);
    zone_occupancy_fraction_samples.push_back(occupancy_fraction_sum / station_count);
    int target_queue_agents = 0;
    for (int agent_id = 0; agent_id < num_of_drives; agent_id++)
    {
        const auto& agent = workstation_agents[agent_id];
        if (agent.phase == WorkstationRuntimePhase::TO_STATION && !agent.tasks.empty() &&
            G.station_for_zone_cell(starts[agent_id].location) == agent.tasks.front().station_id)
            target_queue_agents++;
    }
    target_queue_occupancy_samples.push_back(target_queue_agents / station_count);
}

void WorkstationSystem::seed_fixed_service_paths()
{
    solver.initial_paths.clear();
    solver.initial_paths.resize(num_of_drives);
    for (int k = 0; k < num_of_drives; k++)
    {
        const auto& agent = workstation_agents[k];
        if (agent.phase != WorkstationRuntimePhase::SERVICE)
            continue;

        int loc = starts[k].location;
        solver.initial_paths[k].clear();
        solver.initial_paths[k].reserve(simulation_window + 1);
        for (int t = 0; t <= simulation_window; t++)
        {
            solver.initial_paths[k].emplace_back(loc, t, starts[k].orientation);
        }
    }
}

void WorkstationSystem::pad_paths_through_execution_window()
{
    int end_timestep = timestep + simulation_window;
    for (int k = 0; k < num_of_drives; k++)
    {
        while ((int)paths[k].size() <= end_timestep)
        {
            State final_state = paths[k].back();
            paths[k].emplace_back(final_state.location, final_state.timestep + 1, final_state.orientation);
        }
    }
}

void WorkstationSystem::enforce_workstation_episode_commitments()
{
    int end_timestep = timestep + simulation_window;
    for (int k = 0; k < num_of_drives; k++)
    {
        auto& agent = workstation_agents[k];
        if (agent.tasks.empty())
            continue;
        if (agent.phase == WorkstationRuntimePhase::TO_EXIT)
            continue;

        int target_workstation = -1;
        if (agent.phase == WorkstationRuntimePhase::TO_STATION ||
            agent.phase == WorkstationRuntimePhase::SERVICE)
        {
            target_workstation = G.stations[agent.tasks.front().station_id].workstation;
        }
        if (target_workstation < 0)
            continue;

        int hit_t = -1;
        int workstation = -1;
        for (int t = timestep; t <= end_timestep; t++)
        {
            int loc = paths[k][t].location;
            if (loc == target_workstation)
            {
                hit_t = t;
                workstation = loc;
                break;
            }
        }
        if (hit_t < 0)
            continue;

        int hold_orientation = paths[k][hit_t].orientation;
        for (int t = hit_t; t <= end_timestep; t++)
        {
            paths[k][t].location = workstation;
            paths[k][t].orientation = hold_orientation;
            paths[k][t].timestep = t;
        }
    }
}

bool WorkstationSystem::validate_execution_slice(const string& label) const
{
    int end_timestep = timestep + simulation_window;
    vector<int> occupants(G.size(), -1);
    vector<int> outgoing_owner(G.size(), -1);
    vector<int> outgoing_destination(G.size(), -1);
    for (int t = timestep; t <= end_timestep; t++)
    {
        std::fill(occupants.begin(), occupants.end(), -1);
        std::unordered_map<int, int> invalid_occupants;
        for (int agent = 0; agent < num_of_drives; agent++)
        {
            if ((int)paths[agent].size() <= t)
                continue;

            const int location = paths[agent][t].location;
            int other = -1;
            if (location >= 0 && location < G.size())
            {
                other = occupants[location];
                occupants[location] = agent;
            }
            else
            {
                auto inserted = invalid_occupants.emplace(location, agent);
                if (!inserted.second)
                    other = inserted.first->second;
            }
            if (other >= 0)
            {
                cout << "[" << label << "] vertex conflict between " << other << " and " << agent
                     << " at timestep " << t << " on " << paths[agent][t] << endl;
                return false;
            }
        }

        if (t >= end_timestep)
            continue;

        std::fill(outgoing_owner.begin(), outgoing_owner.end(), -1);
        std::unordered_map<int, pair<int, int>> invalid_outgoing;
        for (int agent = 0; agent < num_of_drives; agent++)
        {
            if ((int)paths[agent].size() <= t + 1)
                continue;

            const int from = paths[agent][t].location;
            const int to = paths[agent][t + 1].location;
            int other = -1;
            if (from >= 0 && from < G.size() && to >= 0 && to < G.size())
            {
                if (outgoing_owner[to] >= 0 && outgoing_destination[to] == from)
                    other = outgoing_owner[to];
                outgoing_owner[from] = agent;
                outgoing_destination[from] = to;
            }
            else
            {
                auto reverse = invalid_outgoing.find(to);
                if (reverse != invalid_outgoing.end() && reverse->second.first == from)
                    other = reverse->second.second;
                invalid_outgoing.emplace(from, make_pair(to, agent));
            }
            if (other >= 0)
            {
                cout << "[" << label << "] edge conflict between " << other << " and " << agent
                     << " at timestep " << (t + 1) << endl;
                return false;
            }
        }
    }
    return true;
}

bool WorkstationSystem::resolve_committed_conflicts()
{
    struct SliceConflict
    {
        int a1 = -1;
        int a2 = -1;
        int t = -1;
        int loc = -1;
        int loc2 = -1;
        bool edge = false;
    };

    auto find_first_conflict = [&](SliceConflict& conf) {
        int end_timestep = timestep + simulation_window;
        vector<int> occupants(G.size(), -1);
        vector<int> outgoing_owner(G.size(), -1);
        vector<int> outgoing_destination(G.size(), -1);
        for (int t = timestep; t <= end_timestep; t++)
        {
            std::fill(occupants.begin(), occupants.end(), -1);
            std::unordered_map<int, int> invalid_occupants;
            for (int agent = 0; agent < num_of_drives; agent++)
            {
                if ((int)paths[agent].size() <= t)
                    continue;
                const int location = paths[agent][t].location;
                int other = -1;
                if (location >= 0 && location < G.size())
                {
                    other = occupants[location];
                    occupants[location] = agent;
                }
                else
                {
                    auto inserted = invalid_occupants.emplace(location, agent);
                    if (!inserted.second)
                        other = inserted.first->second;
                }
                if (other >= 0)
                {
                    conf.a1 = other;
                    conf.a2 = agent;
                    conf.t = t;
                    conf.loc = location;
                    conf.loc2 = -1;
                    conf.edge = false;
                    return true;
                }
            }

            if (t >= end_timestep)
                continue;

            std::fill(outgoing_owner.begin(), outgoing_owner.end(), -1);
            std::unordered_map<int, pair<int, int>> invalid_outgoing;
            for (int agent = 0; agent < num_of_drives; agent++)
            {
                if ((int)paths[agent].size() <= t + 1)
                    continue;
                const int from = paths[agent][t].location;
                const int to = paths[agent][t + 1].location;
                int other = -1;
                if (from >= 0 && from < G.size() && to >= 0 && to < G.size())
                {
                    if (outgoing_owner[to] >= 0 && outgoing_destination[to] == from)
                        other = outgoing_owner[to];
                    outgoing_owner[from] = agent;
                    outgoing_destination[from] = to;
                }
                else
                {
                    auto reverse = invalid_outgoing.find(to);
                    if (reverse != invalid_outgoing.end() && reverse->second.first == from)
                        other = reverse->second.second;
                    invalid_outgoing.emplace(from, make_pair(to, agent));
                }
                if (other >= 0)
                {
                    conf.a1 = other;
                    conf.a2 = agent;
                    conf.t = t + 1;
                    conf.loc = paths[other][t].location;
                    conf.loc2 = paths[other][t + 1].location;
                    conf.edge = true;
                    return true;
                }
            }
        }
        return false;
    };

    auto first_hit = [&](int agent, int loc) {
        int end_timestep = timestep + simulation_window;
        for (int t = timestep; t <= end_timestep && t < (int)paths[agent].size(); t++)
        {
            if (paths[agent][t].location == loc)
                return t;
        }
        return INT_MAX / 2;
    };

    auto phase_rank = [&](int agent_id) {
        switch (workstation_agents[agent_id].phase)
        {
        case WorkstationRuntimePhase::SERVICE:
            return 0;
        case WorkstationRuntimePhase::TO_EXIT:
            return 1;
        case WorkstationRuntimePhase::TO_STATION:
            return 2;
        case WorkstationRuntimePhase::TO_PICKUP:
        default:
            return 3;
        }
    };

    auto can_repair_agent = [&](int agent_id) {
        return workstation_agents[agent_id].phase != WorkstationRuntimePhase::SERVICE;
    };

    auto choose_repair_loser = [&](const SliceConflict& conf) {
        int hit1 = first_hit(conf.a1, conf.loc);
        int hit2 = first_hit(conf.a2, conf.loc);
        int loser = conf.a1;
        if (!can_repair_agent(conf.a1) && can_repair_agent(conf.a2))
            return conf.a2;
        if (!can_repair_agent(conf.a2) && can_repair_agent(conf.a1))
            return conf.a1;
        int rank1 = phase_rank(conf.a1);
        int rank2 = phase_rank(conf.a2);
        if (rank1 != rank2)
            loser = rank1 > rank2 ? conf.a1 : conf.a2;
        else if (!conf.edge)
        {
            if (hit1 < hit2)
                loser = conf.a2;
            else if (hit2 < hit1)
                loser = conf.a1;
            else
                loser = std::max(conf.a1, conf.a2);
        }
        else
        {
            loser = std::max(conf.a1, conf.a2);
        }

        return loser;
    };

    vector< set<tuple<int, int, int> > > blocked_constraints(num_of_drives);

    auto replan_agent = [&](int agent) {
        if (!can_repair_agent(agent))
            return false;

        list< tuple<int, int, int> > initial_constraints;
        update_initial_constraints(initial_constraints);

        vector<Path> local_paths(num_of_drives);
        vector<Path*> local_path_ptrs(num_of_drives, nullptr);
        int end_timestep = timestep + simulation_window;
        for (int k = 0; k < num_of_drives; k++)
        {
            if ((int)paths[k].size() <= timestep)
                continue;
            int last = min(end_timestep, (int)paths[k].size() - 1);
            local_paths[k].reserve(last - timestep + 1);
            for (int t = timestep; t <= last; t++)
            {
                State s = paths[k][t];
                s.timestep = t - timestep;
                local_paths[k].push_back(s);
            }
            local_path_ptrs[k] = &local_paths[k];
        }

        ReservationTable rt(G);
        rt.num_of_agents = num_of_drives;
        rt.map_size = G.size();
        rt.k_robust = k_robust;
        rt.window = planning_window;
        rt.use_cat = false;
        rt.prioritize_start = false;
        rt.hold_endpoints = solver.hold_endpoints;
        solver.path_planner.hold_endpoints = solver.hold_endpoints;
        solver.path_planner.prioritize_start = false;
        list<Constraint> hard_constraints;
        for (const auto& blocked : blocked_constraints[agent])
            hard_constraints.emplace_back(agent, std::get<0>(blocked), std::get<1>(blocked), std::get<2>(blocked), false);
        rt.build(local_path_ptrs, initial_constraints, hard_constraints, agent);

        Path path = solver.path_planner.run(G, starts[agent], goal_locations[agent], rt);
        if (path.empty())
        {
            if (screen >= 2)
            {
                cout << "[repair] agent " << agent << " replanning failed with constraints:";
                for (const auto& blocked : blocked_constraints[agent])
                    cout << " <" << std::get<0>(blocked) << "," << std::get<1>(blocked) << "," << std::get<2>(blocked) << ">";
                cout << endl;
            }
            return false;
        }

        if (screen >= 2)
        {
            cout << "[repair] agent " << agent << " constraints:";
            for (const auto& blocked : blocked_constraints[agent])
                cout << " <" << std::get<0>(blocked) << "," << std::get<1>(blocked) << "," << std::get<2>(blocked) << ">";
            cout << " path:";
            for (int i = 0; i < (int)path.size() && i <= simulation_window; i++)
                cout << " " << path[i];
            cout << endl;
        }

        paths[agent].resize(timestep + (int)path.size());
        for (int i = 0; i < (int)path.size(); i++)
        {
            paths[agent][timestep + i] = path[i];
            paths[agent][timestep + i].timestep = timestep + i;
        }
        if (screen >= 2)
        {
            cout << "[repair-copy] agent " << agent << " global:";
            for (int t = timestep; t <= min(timestep + simulation_window, timestep + (int)path.size() - 1); t++)
                cout << " " << paths[agent][t];
            cout << endl;
        }
        return true;
    };

    auto add_conflict_block = [&](int agent, const SliceConflict& conf) {
        if (conf.edge)
        {
            int local_t = conf.t - timestep;
            int loser_from = paths[agent][conf.t - 1].location;
            int loser_to = paths[agent][conf.t].location;
            blocked_constraints[agent].insert(make_tuple(loser_from, loser_to, local_t));
            blocked_constraints[agent].insert(make_tuple(loser_to, -1, local_t));
        }
        else
        {
            int first_local_hit = max(0, first_hit(agent, conf.loc) - timestep);
            if (starts[agent].location == conf.loc)
                first_local_hit = max(first_local_hit, 1);
            for (int local_t = first_local_hit; local_t <= simulation_window; local_t++)
                blocked_constraints[agent].insert(make_tuple(conf.loc, -1, local_t));
        }
    };

    auto attempt_conflict_replan = [&](int agent, const SliceConflict& conf) {
        auto saved_constraints = blocked_constraints[agent];
        add_conflict_block(agent, conf);
        if (replan_agent(agent))
            return true;
        blocked_constraints[agent] = std::move(saved_constraints);
        return false;
    };

    auto attempt_pair_replan = [&](int first, int second, const SliceConflict& conf) {
        if (!can_repair_agent(first) || !can_repair_agent(second))
            return false;

        auto saved_first_constraints = blocked_constraints[first];
        auto saved_second_constraints = blocked_constraints[second];
        Path saved_first_path = paths[first];
        Path saved_second_path = paths[second];

        blocked_constraints[first].clear();
        blocked_constraints[second].clear();
        add_conflict_block(first, conf);
        if (!replan_agent(first))
        {
            blocked_constraints[first] = std::move(saved_first_constraints);
            blocked_constraints[second] = std::move(saved_second_constraints);
            paths[first] = std::move(saved_first_path);
            paths[second] = std::move(saved_second_path);
            return false;
        }

        if (!replan_agent(second))
        {
            blocked_constraints[first] = std::move(saved_first_constraints);
            blocked_constraints[second] = std::move(saved_second_constraints);
            paths[first] = std::move(saved_first_path);
            paths[second] = std::move(saved_second_path);
            return false;
        }
        return true;
    };

    auto write_repair_failure_artifact = [&](const SliceConflict& conf, int iter) {
        std::ofstream out(outfile + "/repair_failure_t" + std::to_string(timestep) +
                          "_iter" + std::to_string(iter) + ".txt");
        if (!out.is_open())
            return;

        auto phase_name = [&](int agent_id) {
            switch (workstation_agents[agent_id].phase)
            {
            case WorkstationRuntimePhase::SERVICE:
                return "SERVICE";
            case WorkstationRuntimePhase::TO_EXIT:
                return "TO_EXIT";
            case WorkstationRuntimePhase::TO_STATION:
                return "TO_STATION";
            case WorkstationRuntimePhase::TO_PICKUP:
            default:
                return "TO_PICKUP";
            }
        };

        out << "station_policy: " << station_policy << "\n";
        out << "pressure_threshold: " << effective_pressure_threshold(workstation_pressure_threshold) << "\n";
        out << "timestep: " << timestep << "\n";
        out << "simulation_window: " << simulation_window << "\n";
        out << "planning_window: " << planning_window << "\n";
        out << "repair_iteration: " << iter << "\n";
        out << "conflict: a" << conf.a1 << " vs a" << conf.a2
            << " at t=" << conf.t << (conf.edge ? " edge" : " vertex")
            << " loc=" << conf.loc << " loc2=" << conf.loc2 << "\n";
        for (int agent : {conf.a1, conf.a2})
        {
            out << "agent " << agent
                << " phase=" << phase_name(agent)
                << " station=" << agent_station_id(agent)
                << " start=" << starts[agent]
                << "\n";
            out << "blocked_constraints:";
            for (const auto& blocked : blocked_constraints[agent])
            {
                out << " <" << std::get<0>(blocked)
                    << "," << std::get<1>(blocked)
                    << "," << std::get<2>(blocked) << ">";
            }
            out << "\npath_slice:";
            int lo = std::max(timestep, conf.t - 2);
            int hi = std::min(timestep + simulation_window, conf.t + 2);
            for (int tt = lo; tt <= hi && tt < (int)paths[agent].size(); tt++)
                out << " " << paths[agent][tt];
            out << "\n";
        }
    };

    int max_iterations = std::max(num_of_drives * 4, 32);
    for (int iter = 0; iter < max_iterations; iter++)
    {
        pad_paths_through_execution_window();
        enforce_workstation_episode_commitments();
        SliceConflict conf;
        if (!find_first_conflict(conf))
            return true;

        int loser = choose_repair_loser(conf);

        if (screen >= 2)
        {
            cout << "[post_commit] repairing conflict between " << conf.a1 << " and " << conf.a2
                 << " at timestep " << conf.t << (conf.edge ? " via edge " : " on ")
                 << paths[conf.a1][conf.t]
                 << " by replanning agent " << loser << endl;
            int window_lo = max(timestep, conf.t - 1);
            int window_hi = min(timestep + simulation_window, conf.t + 1);
            cout << "[post_commit] a" << conf.a1 << ":";
            for (int tt = window_lo; tt <= window_hi; tt++)
                cout << " " << paths[conf.a1][tt];
            cout << " | a" << conf.a2 << ":";
            for (int tt = window_lo; tt <= window_hi; tt++)
                cout << " " << paths[conf.a2][tt];
            cout << endl;
        }

        if (!attempt_conflict_replan(loser, conf))
        {
            int alternate = loser == conf.a1 ? conf.a2 : conf.a1;
            if (screen >= 2)
                cout << "[repair] fallback replanning agent " << alternate << " after agent "
                     << loser << " failed for the same conflict" << endl;
            if (!can_repair_agent(alternate) || !attempt_conflict_replan(alternate, conf))
            {
                if (screen >= 2)
                    cout << "[repair] both single-agent repair choices failed for conflict between "
                         << conf.a1 << " and " << conf.a2 << " at timestep " << conf.t
                         << "; trying local pair repair" << endl;

                if (!attempt_pair_replan(loser, alternate, conf))
                {
                    if (!attempt_pair_replan(alternate, loser, conf))
                    {
                        if (screen >= 2)
                            cout << "[repair] both pair-repair orders failed for conflict between "
                                 << conf.a1 << " and " << conf.a2 << " at timestep " << conf.t << endl;
                        write_repair_failure_artifact(conf, iter);
                        return false;
                    }
                }
            }
        }
    }
    pad_paths_through_execution_window();
    enforce_workstation_episode_commitments();
    SliceConflict final_conf;
    if (find_first_conflict(final_conf))
        write_repair_failure_artifact(final_conf, max_iterations);
    return false;
}

void WorkstationSystem::update_initial_constraints(list< tuple<int, int, int> >& initial_constraints) const
{
    BasicSystem::update_initial_constraints(initial_constraints);
    for (int k = 0; k < num_of_drives; k++)
    {
        const auto& agent = workstation_agents[k];
        if (agent.phase != WorkstationRuntimePhase::SERVICE)
            continue;

        int remaining_dwell = agent.service_complete_t - timestep;
        if (remaining_dwell < 0)
            continue;

        initial_constraints.emplace_back(k, starts[k].location, remaining_dwell + 1);
    }
}

bool WorkstationSystem::validate_move(int agent_id, const State& prev, const State& curr) const
{
    if (curr.location == prev.location)
    {
        return G.get_rotate_degree(prev.orientation, curr.orientation) != 2;
    }
    if (consider_rotation)
    {
        if (prev.orientation != curr.orientation)
            return false;
        return G.valid_move(prev.location, prev.orientation) &&
               prev.location + G.move[prev.orientation] == curr.location;
    }
    int dir = G.get_direction(prev.location, curr.location);
    return dir >= 0 && G.valid_move(prev.location, dir);
}

bool WorkstationSystem::workstation_congested() const
{
    if (timestep + simulation_window < 100 || execution_progress.size() < 10)
        return false;
    int services = 0;
    int eligible_steps = 0;
    int moved_steps = 0;
    for (auto it = execution_progress.rbegin();
         it != execution_progress.rend() && std::distance(execution_progress.rbegin(), it) < 10;
         ++it)
    {
        services += it->completed_services;
        eligible_steps += it->eligible_steps;
        moved_steps += it->moved_steps;
    }
    return is_sustained_workstation_stall(
        services, eligible_steps, moved_steps);
}

void WorkstationSystem::move_workstations()
{
    ExecutionProgress progress;
    int services_before = completed_services;
    int start_timestep = timestep;
    int end_timestep = timestep + simulation_window;
    for (int t = start_timestep; t <= end_timestep; t++)
    {
        for (int k = 0; k < num_of_drives; k++)
        {
            while ((int)paths[k].size() <= t)
            {
                State final_state = paths[k].back();
                paths[k].emplace_back(final_state.location, final_state.timestep + 1, final_state.orientation);
            }
        }
    }

    for (int t = start_timestep; t <= end_timestep; t++)
    {
        vector<bool> phase_advanced(num_of_drives, false);
        if (k_robust == 0)
        {
            // The paper configuration only checks same-timestep occupancy.
            // Store earlier non-Magic agents so this preserves the legacy
            // asymmetric handling of Magic cells while avoiding an O(N^2) scan.
            vector<int> collision_occupant(G.size(), -1);
            for (int k = 0; k < num_of_drives; k++)
            {
                const State& curr = paths[k][t];
                if (t > 0)
                {
                    const State& prev = paths[k][t - 1];
                    if (!validate_move(k, prev, curr))
                    {
                        set_termination("invalid_move", t);
                        cout << "Invalid move for drive " << k << " from " << prev << " to " << curr << endl;
                        save_results();
                        exit(-1);
                    }
                }
                if (G.types[curr.location] == "Magic")
                    continue;

                int other = collision_occupant[curr.location];
                if (other >= 0)
                {
                    set_termination("fatal_collision", t);
                    cout << "Drive " << k << " at " << curr << " has a conflict with drive " << other
                         << " at " << paths[other][t] << endl;
                    save_results();
                    exit(-1);
                }
                collision_occupant[curr.location] = k;
            }
        }
        else
        {
            for (int k = 0; k < num_of_drives; k++)
            {
                const State& curr = paths[k][t];
                if (t > 0)
                {
                    const State& prev = paths[k][t - 1];
                    if (!validate_move(k, prev, curr))
                    {
                        set_termination("invalid_move", t);
                        cout << "Invalid move for drive " << k << " from " << prev << " to " << curr << endl;
                        save_results();
                        exit(-1);
                    }
                }
                if (G.types[curr.location] != "Magic")
                {
                    for (int j = k + 1; j < num_of_drives; j++)
                    {
                        for (int i = max(0, t - k_robust); i <= min(t + k_robust, end_timestep); i++)
                        {
                            if ((int)paths[j].size() <= i)
                                break;
                            if (paths[j][i].location == curr.location)
                            {
                                set_termination("fatal_collision", t);
                                cout << "Drive " << k << " at " << curr << " has a conflict with drive " << j
                                     << " at " << paths[j][i] << endl;
                                save_results();
                                exit(-1);
                            }
                        }
                    }
                }
            }
        }

        for (int k = 0; k < num_of_drives; k++)
        {
            auto& agent = workstation_agents[k];
            State curr = paths[k][t];
            if (t > start_timestep)
            {
                bool mandatory_service = agent.phase == WorkstationRuntimePhase::SERVICE &&
                    t <= agent.service_complete_t;
                if (!mandatory_service)
                {
                    progress.eligible_steps++;
                    const State& prev = paths[k][t - 1];
                    // Traffic progress means reaching a different cell;
                    // in-place rotations must not mask an all-wait stall.
                    if (prev.location != curr.location)
                        progress.moved_steps++;
                    if (prev.location != curr.location)
                        distance_traveled++;
                }
            }
            if (agent.phase == WorkstationRuntimePhase::TO_PICKUP)
            {
                auto& task = agent.tasks.front();
                if (curr.location == task.endpoint_target)
                {
                    agent.last_endpoint = task.endpoint_target;
                    task.issue_t = t;
                    agent.phase = WorkstationRuntimePhase::TO_STATION;
                    phase_advanced[k] = true;
                }
            }

            if (agent.phase == WorkstationRuntimePhase::TO_STATION)
            {
                auto& task = agent.tasks.front();
                if (task.boundary_t < 0 && G.station_for_zone_cell(curr.location) == task.station_id)
                    task.boundary_t = t;
                if (curr.location == G.stations[task.station_id].workstation)
                {
                    task.service_in_t = t;
                    if (task.boundary_t >= 0)
                        queue_wait_samples.push_back(t - task.boundary_t);
                    agent.phase = WorkstationRuntimePhase::SERVICE;
                    phase_advanced[k] = true;
                    agent.service_complete_t = t + workstation_service_time;
                }
            }

            if (agent.phase == WorkstationRuntimePhase::SERVICE)
            {
                auto& task = agent.tasks.front();
                if (t >= agent.service_complete_t)
                {
                    completed_services++;
                    task.service_complete_t = t;
                    ensure_lookahead_tasks(agent, 2);
                    agent.exit_target = G.choose_exit_for_endpoint(task.station_id, agent.tasks[1].endpoint_target);
                    agent.exit_station_id = task.station_id;
                    agent.phase = WorkstationRuntimePhase::TO_EXIT;
                    phase_advanced[k] = true;
                }
                else if (curr.location != G.stations[task.station_id].workstation)
                {
                    set_termination("left_workstation_early", t);
                    cout << "Drive " << k << " left workstation early at timestep " << t
                         << " (location " << curr.location << ", workstation "
                         << G.stations[task.station_id].workstation << ", service completes "
                         << agent.service_complete_t << ")" << endl;
                    save_results();
                    exit(-1);
                }
            }

            if (agent.phase == WorkstationRuntimePhase::TO_EXIT)
            {
                if (curr.location == agent.exit_target)
                {
                    if (!agent.tasks.empty())
                        agent.tasks.pop_front();
                    ensure_lookahead_tasks(agent, 2);
                    agent.phase = WorkstationRuntimePhase::TO_PICKUP;
                    agent.exit_target = -1;
                    agent.exit_station_id = -1;
                    agent.service_complete_t = -1;
                    phase_advanced[k] = true;
                }
            }
            if (t > start_timestep)
            {
                if (phase_advanced[k] || agent.phase == WorkstationRuntimePhase::SERVICE)
                    pibt_executed_priority_age[k] = 0;
                else
                    pibt_executed_priority_age[k]++;
            }
        }
    }
    progress.completed_services = completed_services - services_before;
    execution_progress.push_back(progress);
    while (execution_progress.size() > 10)
        execution_progress.pop_front();
}

double WorkstationSystem::compute_service_rate() const
{
    if (simulation_time <= 0 || G.stations.empty())
        return 0;
    return completed_services * 100.0 / (simulation_time * G.stations.size());
}

double WorkstationSystem::compute_observed_service_rate() const
{
    int observed = termination_timestep >= 0 ? termination_timestep : timestep;
    if (observed <= 0 || G.stations.empty())
        return 0;
    return completed_services * 100.0 / (observed * G.stations.size());
}

double WorkstationSystem::compute_queue_wait_p95() const
{
    return percentile(queue_wait_samples, 95);
}

int WorkstationSystem::compute_active_queue_agents() const
{
    int count = 0;
    for (const auto& agent : workstation_agents)
    {
        if (agent.phase == WorkstationRuntimePhase::TO_STATION &&
            !agent.tasks.empty() && agent.tasks.front().boundary_t >= 0)
        {
            count++;
        }
    }
    return count;
}

int WorkstationSystem::compute_queue_observation_count() const
{
    return (int)queue_wait_samples.size() + compute_active_queue_agents();
}

double WorkstationSystem::compute_queue_wait_km_p95() const
{
    vector<pair<int, bool>> observations;
    observations.reserve(queue_wait_samples.size() + workstation_agents.size());
    for (int wait : queue_wait_samples)
        observations.emplace_back(wait, true);

    const int observation_t = termination_timestep >= 0 ? termination_timestep : timestep;
    for (const auto& agent : workstation_agents)
    {
        if (agent.phase != WorkstationRuntimePhase::TO_STATION || agent.tasks.empty())
            continue;
        int boundary_t = agent.tasks.front().boundary_t;
        if (boundary_t >= 0)
            observations.emplace_back(std::max(0, observation_t - boundary_t), false);
    }
    if (observations.empty())
        return 0;

    std::sort(observations.begin(), observations.end());
    int at_risk = observations.size();
    double survival = 1.0;
    size_t index = 0;
    while (index < observations.size())
    {
        int duration = observations[index].first;
        int events = 0;
        int censored = 0;
        while (index < observations.size() && observations[index].first == duration)
        {
            if (observations[index].second)
                events++;
            else
                censored++;
            index++;
        }
        if (events > 0 && at_risk > 0)
        {
            survival *= 1.0 - static_cast<double>(events) / at_risk;
            if (1.0 - survival >= 0.95)
                return duration;
        }
        at_risk -= events + censored;
    }

    // Cap waits beyond the observable tail at this run's horizon.
    return observation_t;
}

double WorkstationSystem::compute_queue_wait_rmst(int horizon) const
{
    vector<pair<int, bool>> observations;
    for (int wait : queue_wait_samples)
        observations.emplace_back(wait, true);
    const int observation_t = termination_timestep >= 0 ? termination_timestep : timestep;
    for (const auto& agent : workstation_agents)
    {
        if (agent.phase == WorkstationRuntimePhase::TO_STATION && !agent.tasks.empty() &&
            agent.tasks.front().boundary_t >= 0)
        {
            observations.emplace_back(
                std::max(0, observation_t - agent.tasks.front().boundary_t), false);
        }
    }
    if (observations.empty())
        return 0;
    std::sort(observations.begin(), observations.end());
    int at_risk = observations.size();
    int previous = 0;
    double survival = 1.0;
    double area = 0;
    size_t index = 0;
    while (index < observations.size() && previous < horizon)
    {
        int duration = std::min(observations[index].first, horizon);
        area += survival * std::max(0, duration - previous);
        previous = duration;
        int events = 0;
        int censored = 0;
        int raw_duration = observations[index].first;
        while (index < observations.size() && observations[index].first == raw_duration)
        {
            observations[index].second ? events++ : censored++;
            index++;
        }
        if (raw_duration <= horizon && at_risk > 0)
            survival *= 1.0 - static_cast<double>(events) / at_risk;
        at_risk -= events + censored;
    }
    if (previous < horizon)
        area += survival * (horizon - previous);
    return area;
}

double WorkstationSystem::compute_queue_wait_survival(int threshold) const
{
    vector<pair<int, bool>> observations;
    for (int wait : queue_wait_samples)
        observations.emplace_back(wait, true);
    const int observation_t = termination_timestep >= 0 ? termination_timestep : timestep;
    for (const auto& agent : workstation_agents)
    {
        if (agent.phase == WorkstationRuntimePhase::TO_STATION && !agent.tasks.empty() &&
            agent.tasks.front().boundary_t >= 0)
            observations.emplace_back(std::max(0, observation_t - agent.tasks.front().boundary_t), false);
    }
    if (observations.empty())
        return 0;
    std::sort(observations.begin(), observations.end());
    int at_risk = observations.size();
    double survival = 1.0;
    size_t index = 0;
    while (index < observations.size() && observations[index].first <= threshold)
    {
        int duration = observations[index].first;
        int events = 0;
        int censored = 0;
        while (index < observations.size() && observations[index].first == duration)
        {
            observations[index].second ? events++ : censored++;
            index++;
        }
        if (at_risk > 0)
            survival *= 1.0 - static_cast<double>(events) / at_risk;
        at_risk -= events + censored;
    }
    return survival;
}

double WorkstationSystem::compute_mean_plan_ms() const
{
    if (mean_plan_ms_samples.empty())
        return 0;
    double sum = std::accumulate(mean_plan_ms_samples.begin(), mean_plan_ms_samples.end(), 0.0);
    return sum / mean_plan_ms_samples.size();
}

double WorkstationSystem::compute_plan_runtime_p95_ms() const
{
    return percentile(mean_plan_ms_samples, 95);
}

double WorkstationSystem::compute_plan_runtime_max_ms() const
{
    if (mean_plan_ms_samples.empty())
        return 0;
    return *std::max_element(mean_plan_ms_samples.begin(), mean_plan_ms_samples.end());
}

double WorkstationSystem::compute_plan_runtime_slope() const
{
    if (mean_plan_ms_samples.size() < 2 ||
        mean_plan_ms_samples.size() != plan_timestep_samples.size())
        return 0;

    const double mean_t = std::accumulate(plan_timestep_samples.begin(),
                                          plan_timestep_samples.end(), 0.0) /
                          plan_timestep_samples.size();
    const double mean_ms = compute_mean_plan_ms();
    double covariance = 0;
    double variance = 0;
    for (size_t i = 0; i < mean_plan_ms_samples.size(); i++)
    {
        const double centered_t = plan_timestep_samples[i] - mean_t;
        covariance += centered_t * (mean_plan_ms_samples[i] - mean_ms);
        variance += centered_t * centered_t;
    }
    return variance == 0 ? 0 : 1000.0 * covariance / variance;
}

void WorkstationSystem::set_termination(const string& reason, int t)
{
    termination_reason = reason;
    termination_timestep = t;
    terminated_by_traffic_jam = (reason == "traffic_jam");
    terminated_by_commit_repair_failure = (reason == "commit_repair_failure");
    terminated_by_solver_failure = (reason == "solver_failure");
}

void WorkstationSystem::simulate(int simulation_time)
{
    std::cout << "*** Simulating workstation benchmark " << seed << " ***" << std::endl;
    this->simulation_time = simulation_time;
    initialize();

    for (; timestep < simulation_time; timestep += simulation_window)
    {
        std::cout << "Timestep " << timestep << std::endl;
        update_start_locations();
        update_goal_locations();
        record_episode_diagnostics();
        sync_solver_context();
        seed_fixed_service_paths();
        bool solved = solve_workstation_episode();
        mean_plan_ms_samples.push_back(solver.runtime * 1000.0);
        plan_timestep_samples.push_back(timestep);
        PIBT* pibt = dynamic_cast<PIBT*>(&solver);
        if (pibt != nullptr)
        {
            pibt_inheritance_calls_total += pibt->inheritance_calls;
            pibt_backtracks_total += pibt->backtracks;
            pibt_wait_fallbacks_total += pibt->wait_fallbacks;
            pibt_greedy_repairs_total += pibt->greedy_repairs;
            pibt_pressure_rank_changes_total += pibt->pressure_rank_changes;
            pibt_pressure_active_agents_total += pibt->pressure_active_agents;
            pibt_pressure_candidate_hits_total += pibt->pressure_candidate_hits;
            pibt_budget_extensions_total += pibt->budget_extensions;
        }
        if (!solved)
        {
            set_termination("solver_failure", timestep);
            cout << solver.get_name() << " failed to produce a valid workstation plan at timestep "
                 << timestep << endl;
            break;
        }
        pad_paths_through_execution_window();
        if (!validate_execution_slice("post_solve"))
        {
            set_termination("invalid_execution", timestep);
            break;
        }
        if (!native_failures_only && !resolve_committed_conflicts())
        {
            set_termination("commit_repair_failure", timestep);
            cout << "Failed to repair workstation commitment conflicts at timestep " << timestep << endl;
            save_results();
            exit(-1);
        }
        // Native paper runs do not mutate committed paths after post-solve
        // validation; avoid rescanning the same slice. Repair mode can edit
        // paths, so it retains the post-commit safety check.
        if (!native_failures_only && !validate_execution_slice("post_commit"))
        {
            set_termination("invalid_execution", timestep);
            break;
        }
        move_workstations();
        bool traffic_jam = workstation_congested();
        if (traffic_jam)
        {
            traffic_jam_episodes++;
            if (stop_at_traffic_jam)
            {
                set_termination("traffic_jam", std::min(timestep + simulation_window, simulation_time));
                cout << "***** Too many traffic jams ***" << endl;
                break;
            }
        }
    }

    update_start_locations();
    if (termination_timestep < 0)
        set_termination("completed_simulation", std::min(timestep, simulation_time));
    std::cout << std::endl << "Done!" << std::endl;
    save_results();
}

void WorkstationSystem::save_results()
{
    std::ofstream output;
    output.open(outfile + "/config.txt", std::ios::out);
    output << "map: " << G.map_name << std::endl
           << "#drives: " << num_of_drives << std::endl
           << "seed: " << seed << std::endl
           << "solver: " << solver.get_name() << std::endl
           << "station_policy: " << station_policy << std::endl
           << "pibt_policy: " << pibt_policy << std::endl
           << "stop_at_traffic_jam: " << (stop_at_traffic_jam ? 1 : 0) << std::endl
           << "time_limit: " << time_limit << std::endl
           << "simulation_window: " << simulation_window << std::endl
           << "planning_window: " << planning_window << std::endl
           << "simulation_time: " << simulation_time << std::endl
           << "service_time: " << workstation_service_time << std::endl
           << "pressure_threshold: "
           << (pressure_profile == "prevalence_adaptive" && workstation_pressure_threshold <= 0
               ? "profile_managed"
               : std::to_string(effective_pressure_threshold(workstation_pressure_threshold)))
           << std::endl
           << "pressure_threshold_override: " << workstation_pressure_threshold << std::endl
           << "pressure_profile: " << pressure_profile << std::endl
           << "pressure_lookahead_profile: " << pressure_lookahead_profile << std::endl
           << "pressure_lookahead_radius: " << pressure_lookahead_radius << std::endl
           << "pressure_lookahead_radius_effective: "
           << workstation_effective_pressure_lookahead_radius(
               pressure_lookahead_profile, pressure_lookahead_radius,
               num_of_drives, (int)G.stations.size(),
               pressure_lookahead_min_agents_per_station) << std::endl
           << "pressure_lookahead_min_agents_per_station: "
           << pressure_lookahead_min_agents_per_station << std::endl
           << "pressure_admission: " << pressure_admission << std::endl
           << "pressure_cost_mode: " << pressure_cost_mode << std::endl
           << "pressure_cost_scope: " << pressure_cost_scope << std::endl
           << "pressure_cost_activation: " << pressure_cost_activation << std::endl
           << "pressure_population: " << pressure_population << std::endl
           << "pressure_zone_cost: " << pressure_zone_cost << std::endl
           << "pressure_front_progress_cost: " << pressure_front_progress_cost << std::endl
           << "pressure_exit_progress_cost: " << pressure_exit_progress_cost << std::endl
           << "pressure_ready_slot_priority: " << (pressure_ready_slot_priority ? 1 : 0) << std::endl
           << "pressure_cost_rule: "
           << (pressure_cost_mode == "priority_only"
               ? "priority_only_no_soft_cost"
               : pressure_cost_mode == "escalating"
                   ? "base_plus_one_above_local_threshold"
                   : pressure_cost_mode == "occupancy_escalating"
                       ? "base_plus_one_at_two_thirds_occupancy"
                   : "base") << std::endl
           << "pbs_pressure_branch_rule: "
           << (pressure_front_runner_priority
               ? "phase_protection_then_front_runner"
               : "phase_protection_only") << std::endl
           << "pbs_soft_cost_in_high_level_objective: 0" << std::endl
           << "pressure_inbound_limit: " << pressure_inbound_limit << std::endl
           << "pressure_cost_occupancy_threshold: " << pressure_cost_occupancy_threshold << std::endl
           << "pressure_cost_horizon: " << pressure_cost_horizon << std::endl
           << "pressure_cost_horizon_profile: " << pressure_cost_horizon_profile << std::endl
           << "pressure_local_action_only: " << (pressure_local_action_only ? 1 : 0) << std::endl
           << "pressure_front_runner_priority: " << (pressure_front_runner_priority ? 1 : 0) << std::endl
           << "pressure_front_runner_zone_only: " << (pressure_front_runner_zone_only ? 1 : 0) << std::endl
           << "pressure_front_runner_ready_priority: " << (pressure_front_runner_ready_priority ? 1 : 0) << std::endl
           << "pibt_network_pressure_fraction: " << pibt_network_pressure_fraction << std::endl
           << "pibt_network_pressure_min_agents_per_station: "
           << pibt_network_pressure_min_agents_per_station << std::endl
           << "pibt_global_front_runner_priority: "
           << (pibt_global_front_runner_priority ? 1 : 0) << std::endl
           << "pibt_front_runner_priority: " << (pibt_front_runner_priority ? 1 : 0) << std::endl
           << "pibt_assignment_budget_factor: " << pibt_assignment_budget_factor << std::endl
           << "pibt_pressure_assignment_extension_factor: "
           << pibt_pressure_assignment_extension_factor << std::endl
           << "pibt_front_runner_ready_priority: " << (pibt_front_runner_ready_priority ? 1 : 0) << std::endl
           << "native_failures_only: " << (native_failures_only ? 1 : 0) << std::endl;
    if (const auto* pibt = dynamic_cast<const PIBT*>(&solver))
    {
        output << "pibt_pressure_admission: " << pibt->pressure_admission << std::endl
               << "pibt_pressure_zone_cost: " << pibt->pressure_zone_cost << std::endl
               << "pibt_pressure_front_progress_cost: " << pibt->pressure_front_progress_cost << std::endl
               << "pibt_pressure_exit_progress_cost: " << pibt->pressure_exit_progress_cost << std::endl
               << "pibt_pressure_ready_slot_priority: " << (pibt->pressure_ready_slot_priority ? 1 : 0) << std::endl
               << "pibt_pressure_cost_mode: " << pibt->pressure_cost_mode << std::endl
               << "pibt_pressure_cost_scope: " << pibt->pressure_cost_scope << std::endl
               << "pibt_pressure_cost_activation: " << pibt->pressure_cost_activation << std::endl
               << "pibt_pressure_population: " << pibt->pressure_population << std::endl
               << "pibt_pressure_inbound_limit: " << pibt->pressure_inbound_limit << std::endl
               << "pibt_pressure_assignment_extension_factor: "
               << pibt->pressure_assignment_extension_factor << std::endl
               << "pibt_random_tiebreak: " << (pibt->random_tiebreak ? 1 : 0) << std::endl;
    }
    output.close();

    output.open(outfile + "/summary.csv", std::ios::out);
    output << "service_rate,observed_service_rate,queue_wait_p95,queue_wait_km_p95,"
           << "queue_wait_rmst50,queue_wait_rmst100,queue_wait_rmst200,queue_wait_rmst500,"
           << "queue_wait_survival_20,queue_wait_survival_50,queue_wait_survival_100,"
           << "mean_target_queue_occupancy_per_station,active_queue_agents,"
           << "mean_plan_ms,plan_runtime_p95_ms,plan_runtime_max_ms,"
           << "mean_amortized_ms_per_step,p95_amortized_ms_per_step,max_amortized_ms_per_step,"
           << "plan_runtime_slope_ms_per_1000_steps,completed_services,"
           << "distance_per_completed_service,peak_rss_kb,clean_completion,time_to_stall,stall_event,"
           << "termination_reason,termination_timestep,terminated_by_traffic_jam,terminated_by_commit_repair_failure,"
           << "terminated_by_solver_failure,"
           << "pressure_active_fraction,pressured_station_fraction,mean_zone_occupancy_fraction,"
           << "traffic_jam_fraction,"
           << "pibt_inheritance_calls,pibt_backtracks,pibt_wait_fallbacks,pibt_greedy_repairs,pibt_pressure_rank_changes,"
           << "pibt_pressure_active_agents,pibt_pressure_candidate_hits,pibt_budget_extensions"
           << std::endl;
    auto episode_fraction = [&](int count) {
        if (planning_episodes == 0)
            return 0.0;
        return static_cast<double>(count) / planning_episodes;
    };
    auto sample_mean = [](const vector<double>& values) {
        if (values.empty())
            return 0.0;
        return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
    };
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    const bool clean = termination_reason == "completed_simulation" &&
        termination_timestep >= simulation_time;
    const bool stalled = termination_reason == "traffic_jam";
    const double distance_per_service = completed_services > 0 ?
        static_cast<double>(distance_traveled) / completed_services : 0.0;
    output << compute_service_rate() << ","
           << compute_observed_service_rate() << ",";
    if (compute_queue_observation_count() > 0)
    {
        output << compute_queue_wait_p95() << ","
               << compute_queue_wait_km_p95() << ","
               << compute_queue_wait_rmst(50) << ","
               << compute_queue_wait_rmst(100) << ","
               << compute_queue_wait_rmst(200) << ","
               << compute_queue_wait_rmst(500) << ","
               << compute_queue_wait_survival(20) << ","
               << compute_queue_wait_survival(50) << ","
               << compute_queue_wait_survival(100) << ",";
    }
    else
    {
        output << ",,,,,,,,,";
    }
    output << sample_mean(target_queue_occupancy_samples) << ","
           << compute_active_queue_agents() << ","
           << compute_mean_plan_ms() << ","
           << compute_plan_runtime_p95_ms() << ","
           << compute_plan_runtime_max_ms() << ","
           << compute_mean_plan_ms() / std::max(1, simulation_window) << ","
           << compute_plan_runtime_p95_ms() / std::max(1, simulation_window) << ","
           << compute_plan_runtime_max_ms() / std::max(1, simulation_window) << ","
           << compute_plan_runtime_slope() << ","
           << completed_services << ","
           << distance_per_service << ","
           << usage.ru_maxrss << ","
           << (clean ? 1 : 0) << ","
           << std::max(0, termination_timestep) << ","
           << (stalled ? 1 : 0) << ","
           << termination_reason << ","
           << termination_timestep << ","
           << (terminated_by_traffic_jam ? 1 : 0) << ","
           << (terminated_by_commit_repair_failure ? 1 : 0) << ","
           << (terminated_by_solver_failure ? 1 : 0) << ","
           << episode_fraction(pressure_active_episodes) << ","
           << sample_mean(pressured_station_fraction_samples) << ","
           << sample_mean(zone_occupancy_fraction_samples) << ","
           << episode_fraction(traffic_jam_episodes) << ","
           << pibt_inheritance_calls_total << ","
           << pibt_backtracks_total << ","
           << pibt_wait_fallbacks_total << ","
           << pibt_greedy_repairs_total << ","
           << pibt_pressure_rank_changes_total << ","
           << pibt_pressure_active_agents_total << ","
           << pibt_pressure_candidate_hits_total << ","
           << pibt_budget_extensions_total
           << std::endl;
    output.close();

    output.open(outfile + "/queue_wait_observations.csv", std::ios::out);
    output << "duration,event" << std::endl;
    for (int wait : queue_wait_samples)
        output << wait << ",1" << std::endl;
    const int queue_observation_t = termination_timestep >= 0 ? termination_timestep : timestep;
    for (const auto& agent : workstation_agents)
    {
        if (agent.phase == WorkstationRuntimePhase::TO_STATION && !agent.tasks.empty() &&
            agent.tasks.front().boundary_t >= 0)
        {
            output << std::max(0, queue_observation_t - agent.tasks.front().boundary_t)
                   << ",0" << std::endl;
        }
    }
    output.close();

    output.open(outfile + "/planning_runtime.csv", std::ios::out);
    output << "episode,timestep,plan_ms,pressure_active,pressured_station_fraction,"
           << "mean_zone_occupancy_fraction" << std::endl;
    for (size_t i = 0; i < mean_plan_ms_samples.size(); i++)
    {
        output << i << ","
               << plan_timestep_samples[i] << ","
               << mean_plan_ms_samples[i] << ","
               << pressure_active_samples[i] << ","
               << pressured_station_fraction_samples[i] << ","
               << zone_occupancy_fraction_samples[i]
               << std::endl;
    }
    output.close();

    output.open(outfile + "/paths.txt", std::ios::out);
    output << num_of_drives << std::endl;
    for (int k = 0; k < num_of_drives; k++)
    {
        for (const auto& p : paths[k])
        {
            if (p.timestep <= timestep)
                output << p << ";";
        }
        output << std::endl;
    }
    output.close();
}
