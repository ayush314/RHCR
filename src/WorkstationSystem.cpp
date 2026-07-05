#include "WorkstationSystem.h"

#include "PBS.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <numeric>
#include <random>

namespace
{
constexpr int kDefaultPressureThreshold = 2;

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
    queue_wait_samples.clear();
    mean_plan_ms_samples.clear();
    completed_services = 0;
    planning_episodes = 0;
    pressure_active_episodes = 0;
    termination_reason = "not_started";
    termination_timestep = -1;
    terminated_by_traffic_jam = false;
    terminated_by_commit_repair_failure = false;
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
    int pressure = 0;
    for (int agent_id = 0; agent_id < num_of_drives; agent_id++)
    {
        int loc = starts[agent_id].location;
        if (std::find(station.standby_cells.begin(), station.standby_cells.end(), loc) != station.standby_cells.end() ||
            std::find(station.buffer_cells.begin(), station.buffer_cells.end(), loc) != station.buffer_cells.end())
        {
            pressure++;
        }
    }
    return pressure;
}

void WorkstationSystem::build_goal_sequence(int agent_id)
{
    goal_locations[agent_id].clear();
    auto& agent = workstation_agents[agent_id];
    ensure_lookahead_tasks(agent, 2);
    auto append_goal = [&](int loc) {
        goal_locations[agent_id].emplace_back(loc, 0);
    };
    auto append_service_wait = [&](int station_id, int remaining_dwell) {
        int workstation = G.stations[station_id].workstation;
        for (int i = 0; i <= remaining_dwell; i++)
        {
            goal_locations[agent_id].emplace_back(workstation, i);
        }
    };

    const WorkstationTask& current = agent.tasks.front();
    const WorkstationTask& next = agent.tasks[1];

    if (agent.phase == WorkstationRuntimePhase::TO_PICKUP)
    {
        append_goal(current.endpoint_target);
        append_service_wait(current.station_id, workstation_service_time);
        return;
    }

    if (agent.phase == WorkstationRuntimePhase::TO_STATION)
    {
        append_service_wait(current.station_id, workstation_service_time);
        return;
    }

    if (agent.phase == WorkstationRuntimePhase::SERVICE)
    {
        int remaining_dwell = std::max(agent.service_complete_t - timestep, 0);
        append_service_wait(current.station_id, remaining_dwell);
        return;
    }

    append_goal(agent.exit_target);
    append_goal(next.endpoint_target);
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
    std::vector<WorkstationAgentContext> context(num_of_drives);
    for (int k = 0; k < num_of_drives; k++)
    {
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
        pbs->set_workstation_context(context);
    }

}

void WorkstationSystem::record_episode_diagnostics()
{
    planning_episodes++;
    const int pressure_threshold = effective_pressure_threshold(workstation_pressure_threshold);
    for (size_t station_id = 0; station_id < G.stations.size(); station_id++)
    {
        if (station_pressure((int)station_id) >= pressure_threshold)
        {
            pressure_active_episodes++;
            break;
        }
    }
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
    for (int t = timestep; t <= end_timestep; t++)
    {
        for (int a1 = 0; a1 < num_of_drives; a1++)
        {
            if ((int)paths[a1].size() <= t)
                continue;
            for (int a2 = a1 + 1; a2 < num_of_drives; a2++)
            {
                if ((int)paths[a2].size() <= t)
                    continue;
                if (paths[a1][t].location == paths[a2][t].location)
                {
                    cout << "[" << label << "] vertex conflict between " << a1 << " and " << a2
                         << " at timestep " << t << " on " << paths[a1][t] << endl;
                    return false;
                }
                if (t < end_timestep &&
                    (int)paths[a1].size() > t + 1 &&
                    (int)paths[a2].size() > t + 1 &&
                    paths[a1][t].location == paths[a2][t + 1].location &&
                    paths[a2][t].location == paths[a1][t + 1].location)
                {
                    cout << "[" << label << "] edge conflict between " << a1 << " and " << a2
                         << " at timestep " << (t + 1) << endl;
                    return false;
                }
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
        for (int t = timestep; t <= end_timestep; t++)
        {
            for (int a1 = 0; a1 < num_of_drives; a1++)
            {
                if ((int)paths[a1].size() <= t)
                    continue;
                for (int a2 = a1 + 1; a2 < num_of_drives; a2++)
                {
                    if ((int)paths[a2].size() <= t)
                        continue;
                    if (paths[a1][t].location == paths[a2][t].location)
                    {
                        conf.a1 = a1;
                        conf.a2 = a2;
                        conf.t = t;
                        conf.loc = paths[a1][t].location;
                        conf.loc2 = -1;
                        conf.edge = false;
                        return true;
                    }
                    if (t < end_timestep &&
                        (int)paths[a1].size() > t + 1 &&
                        (int)paths[a2].size() > t + 1 &&
                        paths[a1][t].location == paths[a2][t + 1].location &&
                        paths[a2][t].location == paths[a1][t + 1].location)
                    {
                        conf.a1 = a1;
                        conf.a2 = a2;
                        conf.t = t + 1;
                        conf.loc = paths[a1][t].location;
                        conf.loc2 = paths[a1][t + 1].location;
                        conf.edge = true;
                        return true;
                    }
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

    auto choose_repair_loser = [&](const SliceConflict& conf) {
        int hit1 = first_hit(conf.a1, conf.loc);
        int hit2 = first_hit(conf.a2, conf.loc);
        int loser = conf.a1;
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
            if (!attempt_conflict_replan(alternate, conf))
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

void WorkstationSystem::move_workstations()
{
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

        for (int k = 0; k < num_of_drives; k++)
        {
            auto& agent = workstation_agents[k];
            State curr = paths[k][t];
            if (agent.phase == WorkstationRuntimePhase::TO_PICKUP)
            {
                auto& task = agent.tasks.front();
                if (curr.location == task.endpoint_target)
                {
                    agent.last_endpoint = task.endpoint_target;
                    task.issue_t = t;
                    agent.phase = WorkstationRuntimePhase::TO_STATION;
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
                    agent.service_complete_t = t + workstation_service_time;
                }
            }

            if (agent.phase == WorkstationRuntimePhase::SERVICE)
            {
                auto& task = agent.tasks.front();
                if (curr.location != G.stations[task.station_id].workstation)
                {
                    set_termination("left_workstation_early", t);
                    cout << "Drive " << k << " left workstation early at timestep " << t << endl;
                    save_results();
                    exit(-1);
                }
                if (t >= agent.service_complete_t)
                {
                    completed_services++;
                    task.service_complete_t = t;
                    ensure_lookahead_tasks(agent, 2);
                    agent.exit_target = G.choose_exit_for_endpoint(task.station_id, agent.tasks[1].endpoint_target);
                    agent.exit_station_id = task.station_id;
                    agent.phase = WorkstationRuntimePhase::TO_EXIT;
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
                }
            }
        }
    }
}

double WorkstationSystem::compute_service_rate() const
{
    if (simulation_time <= 0 || G.stations.empty())
        return 0;
    return completed_services * 100.0 / (simulation_time * G.stations.size());
}

double WorkstationSystem::compute_queue_wait_p95() const
{
    return percentile(queue_wait_samples, 95);
}

double WorkstationSystem::compute_mean_plan_ms() const
{
    if (mean_plan_ms_samples.empty())
        return 0;
    double sum = std::accumulate(mean_plan_ms_samples.begin(), mean_plan_ms_samples.end(), 0.0);
    return sum / mean_plan_ms_samples.size();
}

void WorkstationSystem::set_termination(const string& reason, int t)
{
    termination_reason = reason;
    termination_timestep = t;
    terminated_by_traffic_jam = (reason == "traffic_jam");
    terminated_by_commit_repair_failure = (reason == "commit_repair_failure");
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
        solve();
        pad_paths_through_execution_window();
        validate_execution_slice("post_solve");
        if (!resolve_committed_conflicts())
        {
            set_termination("commit_repair_failure", timestep);
            cout << "Failed to repair workstation commitment conflicts at timestep " << timestep << endl;
            save_results();
            exit(-1);
        }
        validate_execution_slice("post_commit");
        mean_plan_ms_samples.push_back(solver.runtime * 1000.0);
        move_workstations();
        if (congested())
        {
            set_termination("traffic_jam", timestep);
            cout << "***** Too many traffic jams ***" << endl;
            break;
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
           << "time_limit: " << time_limit << std::endl
           << "simulation_window: " << simulation_window << std::endl
           << "planning_window: " << planning_window << std::endl
           << "simulation_time: " << simulation_time << std::endl
           << "service_time: " << workstation_service_time << std::endl
           << "pressure_threshold: " << effective_pressure_threshold(workstation_pressure_threshold) << std::endl;
    output.close();

    output.open(outfile + "/summary.csv", std::ios::out);
    output << "service_rate,queue_wait_p95,mean_plan_ms,completed_services,"
           << "termination_reason,termination_timestep,terminated_by_traffic_jam,terminated_by_commit_repair_failure,"
           << "pressure_active_fraction"
           << std::endl;
    auto episode_fraction = [&](int count) {
        if (planning_episodes == 0)
            return 0.0;
        return static_cast<double>(count) / planning_episodes;
    };
    output << compute_service_rate() << ","
           << compute_queue_wait_p95() << ","
           << compute_mean_plan_ms() << ","
           << completed_services << ","
           << termination_reason << ","
           << termination_timestep << ","
           << (terminated_by_traffic_jam ? 1 : 0) << ","
           << (terminated_by_commit_repair_failure ? 1 : 0) << ","
           << episode_fraction(pressure_active_episodes)
           << std::endl;
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
