#include "PBS.h"
#include <numeric>
#include <ctime>
#include <iostream>
#include <limits>
#include <utility>
#include "PathTable.h"
#include "WorkstationGraph.h"

namespace
{
constexpr int kDefaultPressureThreshold = 1;
constexpr int kMissingPriorityValue = std::numeric_limits<int>::max() / 4;

}


void PBS::clear()
{
    runtime = 0;
    runtime_rt = 0;
	runtime_plan_paths = 0;
    runtime_get_higher_priority_agents = 0;
    runtime_copy_priorities = 0;
    runtime_detect_conflicts = 0;
    runtime_copy_conflicts = 0;
    runtime_choose_conflict = 0;
    runtime_find_consistent_paths = 0;
    runtime_find_replan_agents = 0;

    HL_num_expanded = 0;
    HL_num_generated = 0;
    LL_num_expanded = 0;
    LL_num_generated = 0;
    solution_found = false;
    solution_cost = -2;
    // focal_list_threshold = -1;
    avg_path_length = -1;
    paths.clear();
    nogood.clear();
    // focal_list.clear();
    dfs.clear();
    release_closed_list();
    starts.clear();
    goal_locations.clear();
    best_node = nullptr;
    station_pressure_cache.clear();
    station_zone_occupancy_cache.clear();
    station_service_busy_cache.clear();
    station_privileged_agents_cache.clear();
    station_privileged_flags_cache.clear();
    pressure_zone_cells_cache.clear();
    pressure_lookahead_cells_cache.clear();
    pressure_cost_cells_cache.clear();
    pressure_cache_ready = false;
    pressure_threshold_cache = 1;
    pressure_zone_cost_cache = 1;
    pressure_lookahead_radius_cache = 0;
    pressure_state_cache_ready = false;
    network_pressure_active = false;

}

// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
// also, do the same for ll_min_f_vals and paths_costs (since its already "on the way").
void PBS::update_paths(PBSNode* curr)
{
    vector<bool> updated(num_of_agents, false);  // initialized for false
	while (curr != nullptr)
	{
        for (auto p = curr->paths.begin(); p != curr->paths.end(); ++p)
        {
		    if (!updated[std::get<0>(*p)])
		    {
			    paths[std::get<0>(*p)] = &(std::get<1>(*p));
			    updated[std::get<0>(*p)] = true;
		    }
        }
		curr = curr->parent;
	}
}


// deep copy of all conflicts except ones that involve the particular agent
// used for copying conflicts from the parent node to the child nodes
void PBS::copy_conflicts(const list<Conflict>& conflicts,
	list<Conflict>& copy, const vector<bool>& excluded_agents)
{
    clock_t t = clock();
	for (auto conflict : conflicts)
	{
		if (!excluded_agents[std::get<0>(conflict)] && !excluded_agents[std::get<1>(conflict)])
		{
			copy.push_back(conflict);
		}
	}
    runtime_copy_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
}
void PBS::copy_conflicts(const list<Conflict>& conflicts, list<Conflict>& copy, int excluded_agent)
{
    clock_t t = clock();
    for (auto conflict : conflicts)
    {
        if (excluded_agent != std::get<0>(conflict) && excluded_agent != std::get<1>(conflict))
        {
            copy.push_back(conflict);
        }
    }
    runtime_copy_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
}


void PBS::find_conflicts(list<Conflict>& conflicts, int a1, int a2)
{
    clock_t t = clock();
    if (paths[a1] == nullptr || paths[a2] == nullptr)
        return;
	if (hold_endpoints)
	{ 
		// TODO: add k-robust
		size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
		for (size_t timestep = 0; timestep < min_path_length; timestep++)
		{
			int loc1 = paths[a1]->at(timestep).location;
			int loc2 = paths[a2]->at(timestep).location;
			if (loc1 == loc2 && G.types[loc1] != "Magic")
			{
				conflicts.emplace_back(a1, a2, loc1, -1, timestep);
				return;
			}
			else if (timestep < min_path_length - 1
				&& loc1 == paths[a2]->at(timestep + 1).location
				&& loc2 == paths[a1]->at(timestep + 1).location)
			{
				conflicts.emplace_back(a1, a2, loc1, loc2, timestep + 1); // edge conflict
				return;
			}
		}

		if (paths[a1]->size() != paths[a2]->size())
		{
			int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
			int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
			int loc1 = paths[a1_]->back().location;
			for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
			{
				int loc2 = paths[a2_]->at(timestep).location;
				if (loc1 == loc2 && G.types[loc1] != "Magic")
				{
					conflicts.emplace_back(a1_, a2_, loc1, -1, timestep); // It's at least a semi conflict		
					return;
				}
			}
		}
	}
	else
	{
		int size1 = min(window + 1, (int)paths[a1]->size());
		int size2 = min(window + 1, (int)paths[a2]->size());
		for (int timestep = 0; timestep < size1; timestep++)
		{
			if (size2 <= timestep - k_robust)
				break;
			int loc = paths[a1]->at(timestep).location;
			for (int i = max(0, timestep - k_robust); i <= min(timestep + k_robust, size2 - 1); i++)
			{
				if (loc == paths[a2]->at(i).location && G.types[loc] != "Magic")
				{
					conflicts.emplace_back(a1, a2, loc, -1, min(i, timestep)); // k-robust vertex conflict
					runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
					return;
				}
			}
			if (k_robust == 0 && timestep < size1 - 1 && timestep < size2 - 1) // detect edge conflicts
			{
				int loc1 = paths[a1]->at(timestep).location;
				int loc2 = paths[a2]->at(timestep).location;
				if (loc1 != loc2 && loc1 == paths[a2]->at(timestep + 1).location
						 && loc2 == paths[a1]->at(timestep + 1).location)
				{
					conflicts.emplace_back(a1, a2, loc1, loc2, timestep + 1); // edge conflict
					runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
					return;
				}
			}

		}
        if (size1 != size2)
        {
            int shorter = size1 < size2 ? a1 : a2;
            int longer = size1 < size2 ? a2 : a1;
            int shorter_size = min(window + 1, (int)paths[shorter]->size());
            int longer_size = min(window + 1, (int)paths[longer]->size());
            int terminal = paths[shorter]->back().location;
            if (G.types[terminal] == "Workstation")
            {
                for (int timestep = shorter_size; timestep < longer_size; timestep++)
                {
                    if (paths[longer]->at(timestep).location == terminal)
                    {
                        conflicts.emplace_back(shorter, longer, terminal, -1, timestep);
                        runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
                        return;
                    }
                }
            }
        }
    }
	runtime_detect_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
}

void PBS::find_conflicts(list<Conflict>& conflicts)
{
    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
        {
            find_conflicts(conflicts, a1, a2);
        }
    }
}

void PBS::find_conflicts(list<Conflict>& new_conflicts, int new_agent)
{
    for (int a2 = 0; a2 < num_of_agents; a2++)
    {
        if(new_agent == a2)
            continue;
        find_conflicts(new_conflicts, new_agent, a2);
    }
}



void PBS::find_conflicts(const list<Conflict>& old_conflicts, list<Conflict>& new_conflicts, int new_agent)
{
    // Copy from parent
    copy_conflicts(old_conflicts, new_conflicts, new_agent);

    // detect new conflicts
    find_conflicts(new_conflicts, new_agent);
}

void PBS::remove_conflicts(list<Conflict>& conflicts, int excluded_agent)
{
    clock_t t = clock();
    for (auto it = conflicts.begin(); it != conflicts.end();)
    {
        if(std::get<0>(*it) == excluded_agent || std::get<1>(*it) == excluded_agent)
        {
            it = conflicts.erase(it);
        }
		else
		{
			++it;
		}
    }
    runtime_copy_conflicts += (double)(std::clock() - t) / CLOCKS_PER_SEC;
}

void PBS::choose_conflict(PBSNode &node)
{
    clock_t t = clock();
	if (node.conflicts.empty())
	    return;

    node.conflict = node.conflicts.front();


	/*vector<int> lower_nodes(num_of_agents, -1);
	 * double product = -1;
    for (auto conflict : node.conflicts)
    {
        int a1 = std::get<0>(*conflict);
        int a2 = std::get<1>(*conflict);
        node.priorities.update_number_of_lower_nodes(lower_nodes, a1);
        node.priorities.update_number_of_lower_nodes(lower_nodes, a2);
        double new_product = (lower_nodes[a1] + 0.01) * (lower_nodes[a2] + 0.01);
        if (new_product > product)
        {
            node.conflict = conflict;
            product = new_product;
        }
        else if (new_product == product && std::get<4>(*conflict) < std::get<4>(*node.conflict)) // choose the earliest
        {
            node.conflict = conflict;
        }
    }

    return;*/

	// choose the earliest
    for (auto conflict : node.conflicts)
    {
        /*int a1 = std::get<0>(*conflict);
        int a2 = std::get<1>(*conflict);
        if (goal_locations[a1] == goal_locations[a2])
        {
            node.conflict = conflict;
            return;
        }*/
        if (std::get<4>(conflict) < std::get<4>(node.conflict))
            node.conflict = conflict;
    }
    node.earliest_collision = std::get<4>(node.conflict);

    // choose the pair of agents with smaller indices
    /*for (auto conflict : node.conflicts)
    {
        if (min(std::get<0>(*conflict), std::get<1>(*conflict)) <
            min(std::get<0>(*node.conflict), std::get<1>(*node.conflict)) ||
                (min(std::get<0>(*conflict), std::get<1>(*conflict)) ==
                 min(std::get<0>(*node.conflict), std::get<1>(*node.conflict)) &&
                    max(std::get<0>(*conflict), std::get<1>(*conflict)) <
                    max(std::get<0>(*node.conflict), std::get<1>(*node.conflict))))
            node.conflict = conflict;
    }*/

    if (!nogood.empty())
    {
        for (auto conflict : node.conflicts)
        {
            int a1 = std::get<0>(conflict);
            int a2 = std::get<1>(conflict);
            for (auto p : nogood)
            {
                if ((a1 == p.first && a2 == p.second) || (a1 == p.second && a2 == p.first))
                {
                    node.conflict = conflict;
                    runtime_choose_conflict += (double)(std::clock() - t) / CLOCKS_PER_SEC;
                    return;
                }
            }
        }
    }

    runtime_choose_conflict += (double)(std::clock() - t) / CLOCKS_PER_SEC;
}




double PBS::get_path_cost(const Path& path) const
{
    double cost = 0;
    for (int i = 0; i < (int)path.size() - 1; i++)
    {
        double travel_time = 1;
        if (i > window && travel_times.find(path[i].location) != travel_times.end())
        {
            travel_time += travel_times.at(path[i].location);
        }
        cost += G.get_weight(path[i].location, path[i + 1].location) * travel_time;
    }
    return cost;
}

bool PBS::find_path(PBSNode* node, int agent)
{
    Path path;
    double path_cost;

    clock_t t = std::clock();
	rt.copy(initial_rt);
    rt.build(paths, initial_constraints, node->priorities.get_reachable_nodes(agent),
             agent, starts[agent].location);
    maybe_add_workstation_softzone(rt, agent);
    maybe_add_workstation_progress_cost(rt, agent);
    runtime_get_higher_priority_agents += node->priorities.runtime;

    runtime_rt += (double)(std::clock() - t) / CLOCKS_PER_SEC;

    t = std::clock();
    path = path_planner.run(G, starts[agent], goal_locations[agent], rt);
	runtime_plan_paths += (double)(std::clock() - t) / CLOCKS_PER_SEC;
    // t = std::clock();
    // rt.clear();
    // runtime_rt += (double)(std::clock() - t) / CLOCKS_PER_SEC;
    LL_num_expanded += path_planner.num_expanded;
    LL_num_generated += path_planner.num_generated;

    if (path.empty())
    {
        if (screen == 2)
            std::cout << "Fail to find a path" << std::endl;
        return false;
    }
    // Pressure soft costs guide this low-level path, but are not part of the
    // persistent PBS objective. get_path_cost uses the same base travel-cost
    // accounting as the old path being replaced, avoiding accumulation across
    // repeated replans.
    path_cost = get_path_cost(path);
    double old_cost = 0;
    if (paths[agent] != nullptr)
        old_cost = get_path_cost(*paths[agent]);
    node->g_val = node->g_val - old_cost + path_cost;
    for (auto it = node->paths.begin(); it != node->paths.end(); ++it)
    {
        if (std::get<0>(*it) == agent)
        {
            node->paths.erase(it);
            break;
        }
    }
    node->paths.emplace_back(agent, path);
    paths[agent] = &node->paths.back().second;
    return true;
}

tuple<int, int, int, int> PBS::pressure_privilege_key(int agent, int station_id) const
{
    const auto* grid = dynamic_cast<const WorkstationGrid*>(&G);
    const auto& ctx = workstation_context[agent];
    const bool boundary_seen = ctx.boundary_entry_t >= 0 ||
        (grid != nullptr &&
         grid->station_for_zone_cell(starts[agent].location) == station_id);
    int inside_rank = boundary_seen ? 0 : 1;
    int dist = grid == nullptr ? kMissingPriorityValue :
        grid->distance_to_workstation(station_id, starts[agent].location);
    int task_issue = ctx.task_issue_t >= 0 ? ctx.task_issue_t : kMissingPriorityValue;
    return workstation_privilege_key(inside_rank == 0, dist, task_issue, agent);
}

vector<int> PBS::workstation_privileged_agents(int station_id, int pressure) const
{
    const auto* grid = dynamic_cast<const WorkstationGrid*>(&G);
    vector<int> inbound;
    if (pressure_cache_ready && station_id >= 0 &&
        station_id < (int)station_pressure_cache.size() &&
        station_pressure_cache[station_id] == pressure)
    {
        return station_privileged_agents_cache[station_id];
    }
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        const auto& ctx = workstation_context[agent];
        if (ctx.phase == WorkstationAgentPhase::TO_STATION && ctx.station_id == station_id)
            inbound.push_back(agent);
    }
    int capacity = grid == nullptr ? 1 :
        std::max(1, (int)grid->stations[station_id].zone_cells.size() - 1);
    return select_workstation_privileged_agents(
        std::move(inbound),
        [&](int agent) { return pressure_privilege_key(agent, station_id); },
        pressure_admission, pressure_inbound_limit, pressure, capacity,
        station_id < (int)station_zone_occupancy_cache.size()
            ? station_zone_occupancy_cache[station_id] : -1,
        num_of_agents, (int)grid->stations.size(),
        pressure_lookahead_min_agents_per_station);
}

int PBS::effective_workstation_pressure_threshold() const
{
    if (pressure_state_cache_ready && workstation_pressure_threshold <= 0)
        return pressure_threshold_cache;
    return workstation_pressure_threshold_for_profile(
        pressure_profile, workstation_pressure_threshold, station_pressure_cache);
}

int PBS::workstation_pressure(int station_id) const
{
    const auto* grid = dynamic_cast<const WorkstationGrid*>(&G);
    if (grid == nullptr || station_id < 0 || station_id >= (int)grid->stations.size())
        return 0;

    const auto& station = grid->stations[station_id];
    int pressure = 0;
    const bool include_protected_phases = pressure_population != "inbound_only";
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        if (!contributes_to_workstation_pressure(
                workstation_context[agent], station_id, include_protected_phases))
            continue;
        const int loc = starts[agent].location;
        const auto& context = workstation_context[agent];
        const bool has_cached_cells = station_id <
            (int)pressure_zone_cells_cache.size();
        const bool in_zone = has_cached_cells
            ? std::binary_search(
                pressure_zone_cells_cache[station_id].begin(),
                pressure_zone_cells_cache[station_id].end(), loc)
            : station.zone_cells.find(loc) != station.zone_cells.end();
        const bool in_lookahead = pressure_lookahead_radius_cache > 0 &&
            context.phase == WorkstationAgentPhase::TO_STATION &&
            (has_cached_cells
                ? std::binary_search(
                    pressure_lookahead_cells_cache[station_id].begin(),
                    pressure_lookahead_cells_cache[station_id].end(), loc)
                : (loc != station.workstation &&
                   grid->distance_to_workstation(station_id, loc) <=
                       pressure_lookahead_radius_cache));
        if (in_zone || in_lookahead)
            pressure++;
    }
    return pressure;
}

void PBS::initialize_pressure_cache()
{
    const auto* grid = dynamic_cast<const WorkstationGrid*>(&G);
    station_pressure_cache.clear();
    station_privileged_agents_cache.clear();
    station_privileged_flags_cache.clear();
    pressure_cache_ready = false;
    pressure_state_cache_ready = false;
    pressure_threshold_cache = 1;
    pressure_zone_cost_cache = 1;
    pressure_lookahead_radius_cache = 0;
    network_pressure_active = false;
    if (grid == nullptr || workstation_context.size() != (size_t)num_of_agents)
        return;
    pressure_lookahead_radius_cache = workstation_effective_pressure_lookahead_radius(
        pressure_lookahead_profile, pressure_lookahead_radius, num_of_agents,
        (int)grid->stations.size(), pressure_lookahead_min_agents_per_station);

    station_pressure_cache.assign(grid->stations.size(), 0);
    station_zone_occupancy_cache.assign(grid->stations.size(), 0);
    station_service_busy_cache.assign(grid->stations.size(), 0);
    station_privileged_agents_cache.assign(grid->stations.size(), vector<int>());
    station_privileged_flags_cache.assign(
        grid->stations.size(), vector<char>(num_of_agents, 0));
    pressure_zone_cells_cache.assign(grid->stations.size(), vector<int>());
    pressure_lookahead_cells_cache.assign(grid->stations.size(), vector<int>());
    pressure_cost_cells_cache.assign(grid->stations.size(), vector<int>());
    for (size_t station_id = 0; station_id < grid->stations.size(); station_id++)
    {
        auto& cells = pressure_zone_cells_cache[station_id];
        for (int loc : grid->stations[station_id].zone_cells)
        {
            if (loc != grid->stations[station_id].workstation)
                cells.push_back(loc);
        }
        std::sort(cells.begin(), cells.end());
        if (pressure_lookahead_radius_cache > 0)
        {
            auto& lookahead = pressure_lookahead_cells_cache[station_id];
            const int workstation = grid->stations[station_id].workstation;
            for (int loc = 0; loc < G.size(); loc++)
            {
                if (loc != workstation &&
                    grid->distance_to_workstation((int)station_id, loc) <=
                        pressure_lookahead_radius_cache &&
                    !std::binary_search(cells.begin(), cells.end(), loc))
                    lookahead.push_back(loc);
            }
        }
        auto& cost_cells = pressure_cost_cells_cache[station_id];
        cost_cells = cells;
        if (pressure_cost_scope == "queue")
        {
            vector<int> queue_cells;
            queue_cells.reserve(cost_cells.size());
            for (int loc : cost_cells)
            {
                if (std::find(grid->stations[station_id].exit_cells.begin(),
                              grid->stations[station_id].exit_cells.end(), loc) ==
                    grid->stations[station_id].exit_cells.end())
                    queue_cells.push_back(loc);
            }
            cost_cells.swap(queue_cells);
        }
        if (pressure_cost_scope == "holding")
        {
            cost_cells.clear();
            cost_cells.insert(cost_cells.end(),
                              grid->stations[station_id].standby_cells.begin(),
                              grid->stations[station_id].standby_cells.end());
            cost_cells.insert(cost_cells.end(),
                              grid->stations[station_id].buffer_cells.begin(),
                              grid->stations[station_id].buffer_cells.end());
        }
        if (pressure_cost_scope == "approach")
        {
            cost_cells.clear();
            cost_cells.insert(cost_cells.end(),
                              grid->stations[station_id].approach_cells.begin(),
                              grid->stations[station_id].approach_cells.end());
        }
        if (pressure_cost_scope == "entry")
        {
            cost_cells.clear();
            cost_cells.insert(cost_cells.end(),
                              grid->stations[station_id].buffer_cells.begin(),
                              grid->stations[station_id].buffer_cells.end());
            cost_cells.insert(cost_cells.end(),
                              grid->stations[station_id].approach_cells.begin(),
                              grid->stations[station_id].approach_cells.end());
        }
        if (pressure_cost_scope == "lookahead")
        {
            cost_cells.insert(cost_cells.end(),
                              pressure_lookahead_cells_cache[station_id].begin(),
                              pressure_lookahead_cells_cache[station_id].end());
        }
        std::sort(cost_cells.begin(), cost_cells.end());
    }
    for (size_t station_id = 0; station_id < grid->stations.size(); station_id++)
    {
        const int pressure = workstation_pressure((int)station_id);
        station_pressure_cache[station_id] = pressure;
    }
    for (const State& state : starts)
    {
        const int station_id = grid->station_for_zone_cell(state.location);
        if (station_id < 0 || station_id >= (int)grid->stations.size() ||
            state.location == grid->stations[station_id].workstation)
            continue;
        station_zone_occupancy_cache[station_id]++;
    }
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        const auto& context = workstation_context[agent];
        if (context.station_id < 0 || context.station_id >= (int)grid->stations.size())
            continue;
        if (starts[agent].location == grid->stations[context.station_id].workstation &&
            (context.phase == WorkstationAgentPhase::SERVICE ||
             context.phase == WorkstationAgentPhase::TO_STATION))
            station_service_busy_cache[context.station_id] = 1;
    }
    const int threshold = workstation_pressure_threshold_for_profile(
        pressure_profile, workstation_pressure_threshold, station_pressure_cache);
    pressure_threshold_cache = threshold;
    pressure_zone_cost_cache = workstation_pressure_zone_cost_for_profile(
        pressure_profile, workstation_pressure_zone_cost, threshold);
    pressure_state_cache_ready = true;
    for (size_t station_id = 0; station_id < grid->stations.size(); station_id++)
    {
        const int pressure = station_pressure_cache[station_id];
        if (is_pressure_aware_policy(station_policy) &&
            pressure >= threshold)
        {
            station_privileged_agents_cache[station_id] =
                workstation_privileged_agents((int)station_id, pressure);
            for (int agent : station_privileged_agents_cache[station_id])
            {
                if (agent >= 0 && agent < num_of_agents)
                    station_privileged_flags_cache[station_id][agent] = 1;
            }
        }
    }
    network_pressure_active = is_pressure_aware_policy(station_policy) &&
        workstation_network_pressure_active(
            station_pressure_cache, threshold,
            network_pressure_fraction);
    pressure_cache_ready = true;
}

bool PBS::workstation_pressure_active(int pressure) const
{
    return pressure >= effective_workstation_pressure_threshold();
}

bool PBS::maybe_add_workstation_softzone(ReservationTable& rt, int agent)
{
    if (!is_pressure_aware_policy(station_policy) || workstation_context.size() != (size_t)num_of_agents)
        return false;

    const auto* grid = dynamic_cast<const WorkstationGrid*>(&G);
    if (grid == nullptr)
        return false;

    const auto& ctx = workstation_context[agent];
    if (ctx.phase != WorkstationAgentPhase::TO_STATION || ctx.station_id < 0 ||
        ctx.station_id >= (int)grid->stations.size())
        return false;

    const int station_id = ctx.station_id;
    const auto& station = grid->stations[station_id];
    if (pressure_cost_activation == "outside_only" &&
        station.zone_cells.find(starts[agent].location) != station.zone_cells.end() &&
        starts[agent].location != station.workstation)
        return false;
    int pressure = pressure_cache_ready && station_id < (int)station_pressure_cache.size()
        ? station_pressure_cache[station_id]
        : workstation_pressure(station_id);
    if (!workstation_pressure_active(pressure))
        return false;
    const int zone_occupancy = station_id < (int)station_zone_occupancy_cache.size()
        ? station_zone_occupancy_cache[station_id] : 0;
    if (!workstation_pressure_cost_active(
            pressure, zone_occupancy, pressure_cost_occupancy_threshold,
            pressure_cost_activation))
        return false;
    if (!workstation_busy_only_cost_applies(
            pressure_cost_activation,
            station_id < (int)station_service_busy_cache.size() &&
                station_service_busy_cache[station_id]))
        return false;
    // Development-only ablation: keep pressure privilege and front-runner
    // conflict ordering, but do not add a negative soft path preference.
    if (pressure_cost_mode == "priority_only")
        return false;
    if (pressure_local_action_only)
    {
        auto relevant_location = [&](int location) {
            if (location == station.workstation)
                return true;
            if (station_id < (int)pressure_zone_cells_cache.size() &&
                std::binary_search(
                    pressure_zone_cells_cache[station_id].begin(),
                    pressure_zone_cells_cache[station_id].end(), location))
                return true;
            return pressure_cost_scope == "lookahead" &&
                station_id < (int)pressure_lookahead_cells_cache.size() &&
                std::binary_search(
                    pressure_lookahead_cells_cache[station_id].begin(),
                    pressure_lookahead_cells_cache[station_id].end(), location);
        };
        bool relevant = relevant_location(starts[agent].location);
        if (!relevant)
        {
            for (const State& neighbor : G.get_neighbors(starts[agent]))
            {
                if (relevant_location(neighbor.location))
                {
                    relevant = true;
                    break;
                }
            }
        }
        if (!relevant)
            return false;
    }
    const int zone_capacity = std::max(
        1, (int)station.zone_cells.size() -
            (station.zone_cells.find(station.workstation) != station.zone_cells.end() ? 1 : 0));
    const bool escalating = workstation_pressure_cost_escalates(
        pressure_cost_mode, pressure,
        effective_workstation_pressure_threshold(), zone_occupancy, zone_capacity);
    const int base_cost = pressure_state_cache_ready
        ? pressure_zone_cost_cache
        : workstation_pressure_zone_cost_for_profile(
            pressure_profile, workstation_pressure_zone_cost,
            effective_workstation_pressure_threshold());
    const int effective_horizon = workstation_pressure_cost_horizon_for_profile(
        pressure_cost_horizon_profile, pressure_cost_horizon,
        network_pressure_active);
    const int soft_cost_end = effective_horizon > 0
        ? std::min(rt.window + 1, effective_horizon + 1)
        : rt.window + 1;
    const int current_distance =
        grid->distance_to_workstation(station_id, starts[agent].location);
    auto should_charge_location = [&](int location) {
        if (workstation_incumbent_grace_applies(
                pressure_cost_activation, starts[agent].location, location,
                station.workstation,
                station.zone_cells.find(starts[agent].location) != station.zone_cells.end()))
            return false;
        if (!workstation_entry_only_cost_applies(
                pressure_cost_activation, location, station.workstation,
                starts[agent].location != station.workstation &&
                    station.zone_cells.find(starts[agent].location) !=
                        station.zone_cells.end(),
                std::find(station.exit_cells.begin(), station.exit_cells.end(), location) !=
                    station.exit_cells.end()))
                return false;
        if (!workstation_enter_only_cost_applies(
                pressure_cost_activation,
                starts[agent].location != station.workstation &&
                    station.zone_cells.find(starts[agent].location) !=
                    station.zone_cells.end()))
            return false;
        if (!workstation_deeper_only_cost_applies(
                pressure_cost_activation, current_distance,
                grid->distance_to_workstation(station_id, location)))
            return false;
        if (pressure_cost_activation == "wait_only")
        {
            if (location != starts[agent].location ||
                location == station.workstation ||
                station.zone_cells.find(location) == station.zone_cells.end())
                return false;
            if (pressure_cost_scope == "zone" ||
                (pressure_cost_scope == "lookahead" && pressure_lookahead_radius_cache <= 0))
                return true;
            if (station_id >= (int)pressure_cost_cells_cache.size())
                return false;
            return std::find(
                pressure_cost_cells_cache[station_id].begin(),
                pressure_cost_cells_cache[station_id].end(), location) !=
                pressure_cost_cells_cache[station_id].end();
        }
        if (pressure_cost_activation != "progress_only")
            return true;
        return grid->distance_to_workstation(station_id, location) >= current_distance;
    };

    bool privileged = false;
    if (pressure_cache_ready && station_id < (int)station_privileged_flags_cache.size() &&
        agent >= 0 && agent < (int)station_privileged_flags_cache[station_id].size())
    {
        privileged = station_privileged_flags_cache[station_id][agent] != 0;
    }
    else
    {
        const vector<int> fallback = workstation_privileged_agents(station_id, pressure);
        privileged = std::find(fallback.begin(), fallback.end(), agent) != fallback.end();
    }
    if (privileged)
        return false;

    if (pressure_cost_scope == "zone" ||
        (pressure_cost_scope == "lookahead" && pressure_lookahead_radius_cache <= 0))
    {
        for (int loc : pressure_zone_cells_cache[station_id])
        {
            if (!should_charge_location(loc))
                continue;
            rt.addSoftVertexConstraint(
                loc, 0, soft_cost_end,
                workstation_pressure_cost(
                    base_cost, pressure,
                    effective_workstation_pressure_threshold(), escalating));
        }
    }
    else if (pressure_cost_scope == "queue" ||
             pressure_cost_scope == "holding" ||
             pressure_cost_scope == "approach" ||
             pressure_cost_scope == "entry")
    {
        for (int loc : pressure_cost_cells_cache[station_id])
        {
            if (!should_charge_location(loc))
                continue;
            rt.addSoftVertexConstraint(
                loc, 0, soft_cost_end,
                workstation_pressure_cost(
                    base_cost, pressure,
                    effective_workstation_pressure_threshold(), escalating));
        }
    }
    else
    {
        if (station_id < (int)pressure_cost_cells_cache.size())
        {
            for (int loc : pressure_cost_cells_cache[station_id])
            {
                if (!should_charge_location(loc))
                    continue;
                rt.addSoftVertexConstraint(
                    loc, 0, soft_cost_end,
                    workstation_pressure_cost(
                        base_cost, pressure,
                        effective_workstation_pressure_threshold(), escalating));
            }
        }
        else
        {
            for (int loc = 0; loc < G.size(); loc++)
            {
                const bool in_zone = station.zone_cells.find(loc) != station.zone_cells.end();
                const bool in_lookahead = loc != station.workstation &&
                    grid->distance_to_workstation(station_id, loc) <= pressure_lookahead_radius_cache;
                if ((in_zone || in_lookahead) && should_charge_location(loc))
                    rt.addSoftVertexConstraint(
                        loc, 0, soft_cost_end,
                        workstation_pressure_cost(
                            base_cost, pressure,
                            effective_workstation_pressure_threshold(), escalating));
            }
        }
    }

    return true;
}

bool PBS::maybe_add_workstation_progress_cost(ReservationTable& rt, int agent)
{
    if (!is_pressure_aware_policy(station_policy) ||
        workstation_context.size() != (size_t)num_of_agents ||
        agent < 0 || agent >= num_of_agents)
        return false;

    const auto* grid = dynamic_cast<const WorkstationGrid*>(&G);
    if (grid == nullptr)
        return false;

    const auto& ctx = workstation_context[agent];
    int target = -1;
    int cost = 0;
    if (ctx.phase == WorkstationAgentPhase::TO_STATION &&
        pressure_front_progress_cost > 0 && ctx.station_id >= 0 &&
        ctx.station_id < (int)grid->stations.size() &&
        ctx.station_id < (int)station_pressure_cache.size() &&
        station_pressure_cache[ctx.station_id] >=
            effective_workstation_pressure_threshold() &&
        ctx.station_id < (int)station_zone_occupancy_cache.size() &&
        workstation_soft_pressure_active(
            station_zone_occupancy_cache[ctx.station_id],
            pressure_cost_occupancy_threshold) &&
        ctx.station_id < (int)station_privileged_agents_cache.size() &&
        !station_privileged_agents_cache[ctx.station_id].empty() &&
        station_privileged_agents_cache[ctx.station_id].front() == agent)
    {
        target = grid->stations[ctx.station_id].workstation;
        cost = pressure_front_progress_cost;
    }
    else if (ctx.phase == WorkstationAgentPhase::TO_EXIT &&
             pressure_exit_progress_cost > 0 && ctx.station_id >= 0 &&
             ctx.station_id < (int)grid->stations.size())
    {
        int best_distance = kMissingPriorityValue;
        for (int exit : grid->stations[ctx.station_id].exit_cells)
        {
            const int distance = grid->distance_between(starts[agent].location, exit);
            if (distance < best_distance)
            {
                best_distance = distance;
                target = exit;
            }
        }
        cost = pressure_exit_progress_cost;
    }

    if (target < 0 || cost <= 0)
        return false;
    const int current_distance = grid->distance_between(
        starts[agent].location, target);
    if (current_distance <= 0 || current_distance >= kMissingPriorityValue)
        return false;

    const int final_t = std::min(rt.window, current_distance);
    for (int local_t = 1; local_t <= final_t; local_t++)
    {
        const int distance_limit = workstation_progress_distance_limit(
            current_distance, local_t);
        if (distance_limit < 0)
            continue;
        for (int location = 0; location < G.size(); location++)
        {
            const int distance = grid->distance_between(location, target);
            if (distance < kMissingPriorityValue && distance >= distance_limit)
                rt.addSoftVertexConstraint(location, local_t, local_t + 1, cost);
        }
    }
    return true;
}

WorkstationAgentContext PBS::projected_context_at(int agent, int local_t) const
{
    if (agent < 0 || agent >= (int)workstation_context.size())
        return WorkstationAgentContext();
    WorkstationAgentContext context = workstation_context[agent];
    if (agent >= (int)projected_goal_context.size() ||
        agent >= (int)goal_locations.size() || paths[agent] == nullptr)
        return context;

    int goal_index = 0;
    const Path& path = *paths[agent];
    int final_t = std::min(local_t, (int)path.size() - 1);
    for (int t = 0; t <= final_t && goal_index < (int)goal_locations[agent].size(); t++)
    {
        if (path[t].location == goal_locations[agent][goal_index].first)
            goal_index++;
    }
    if (goal_index < (int)projected_goal_context[agent].size())
        return projected_goal_context[agent][goal_index];
    if (!projected_goal_context[agent].empty())
    {
        context = projected_goal_context[agent].back();
        if (context.phase == WorkstationAgentPhase::TO_STATION)
            context.phase = WorkstationAgentPhase::SERVICE;
    }
    return context;
}

bool PBS::prefer_workstation_branch(const Conflict& conflict, pair<int, int>& preferred_priority) const
{
    if (!is_phase_aware_policy(station_policy) || workstation_context.size() != (size_t)num_of_agents)
        return false;

    int a1, a2, v1, v2, t;
    std::tie(a1, a2, v1, v2, t) = conflict;

    const auto c1 = projected_context_at(a1, t);
    const auto c2 = projected_context_at(a2, t);
    auto higher_first = [&](int higher, int lower) {
        preferred_priority = make_pair(lower, higher);
        return true;
    };

    if (workstation_protected_precedes(c1.phase, c2.phase))
        return higher_first(a1, a2);
    if (workstation_protected_precedes(c2.phase, c1.phase))
        return higher_first(a2, a1);

    if (pressure_ready_slot_priority && is_pressure_aware_policy(station_policy))
    {
        const auto* grid = dynamic_cast<const WorkstationGrid*>(&G);
        auto ready_station = [&](int agent,
                                 const WorkstationAgentContext& context) {
            if (grid == nullptr || agent < 0 || agent >= (int)starts.size() ||
                context.phase != WorkstationAgentPhase::TO_STATION ||
                context.station_id < 0 ||
                context.station_id >= (int)grid->stations.size() ||
                context.station_id >= (int)station_pressure_cache.size() ||
                station_pressure_cache[context.station_id] <
                    effective_workstation_pressure_threshold() ||
                context.station_id >= (int)station_zone_occupancy_cache.size() ||
                !workstation_soft_pressure_active(
                    station_zone_occupancy_cache[context.station_id],
                    pressure_cost_occupancy_threshold) ||
                context.station_id >= (int)station_privileged_agents_cache.size() ||
                station_privileged_agents_cache[context.station_id].empty() ||
                station_privileged_agents_cache[context.station_id].front() != agent ||
                !workstation_front_runner_ready(grid->distance_to_workstation(
                    context.station_id, starts[agent].location)))
                return -1;
            return context.station_id;
        };
        const int ready1 = ready_station(a1, c1);
        const int ready2 = ready_station(a2, c2);
        const bool same_station_inbound =
            c1.phase == WorkstationAgentPhase::TO_STATION &&
            c2.phase == WorkstationAgentPhase::TO_STATION &&
            c1.station_id >= 0 && c1.station_id == c2.station_id;
        if (same_station_inbound && (ready1 >= 0) != (ready2 >= 0))
        {
            const int station_id = ready1 >= 0 ? ready1 : ready2;
            const auto& station = grid->stations[station_id];
            auto touches_service_zone = [&](int location) {
                return location == station.workstation ||
                    station.zone_cells.find(location) != station.zone_cells.end();
            };
            if (touches_service_zone(v1) || touches_service_zone(v2))
                return ready1 >= 0 ? higher_first(a1, a2) : higher_first(a2, a1);
        }
    }

    if (pressure_front_runner_priority &&
        is_pressure_aware_policy(station_policy) &&
        c1.phase == WorkstationAgentPhase::TO_STATION &&
        c2.phase == WorkstationAgentPhase::TO_STATION &&
        c1.station_id >= 0 && c1.station_id == c2.station_id &&
        c1.station_id < (int)station_pressure_cache.size() &&
        station_pressure_cache[c1.station_id] >=
            effective_workstation_pressure_threshold() &&
        c1.station_id < (int)station_zone_occupancy_cache.size() &&
        workstation_soft_pressure_active(
            station_zone_occupancy_cache[c1.station_id],
            pressure_cost_occupancy_threshold) &&
        c1.station_id < (int)station_privileged_agents_cache.size() &&
        !station_privileged_agents_cache[c1.station_id].empty())
    {
        if (pressure_front_runner_zone_only)
        {
            const auto* grid = dynamic_cast<const WorkstationGrid*>(&G);
            if (grid == nullptr || c1.station_id >= (int)grid->stations.size())
                return false;
            const auto& station = grid->stations[c1.station_id];
            auto touches_station_zone = [&](int location) {
                return location == station.workstation ||
                    station.zone_cells.find(location) != station.zone_cells.end();
            };
            if (!touches_station_zone(v1) && !touches_station_zone(v2))
                return false;
        }
        const int front_runner = station_privileged_agents_cache[c1.station_id].front();
        if (pressure_front_runner_ready_priority)
        {
            const auto* grid = dynamic_cast<const WorkstationGrid*>(&G);
            if (grid == nullptr || front_runner < 0 || front_runner >= (int)starts.size() ||
                !workstation_front_runner_ready(grid->distance_to_workstation(
                    c1.station_id, starts[front_runner].location)))
                return false;
        }
        if (front_runner == a1 && front_runner != a2)
            return higher_first(a1, a2);
        if (front_runner == a2 && front_runner != a1)
            return higher_first(a2, a1);
    }

    return false;
}


// return true if the agent keeps waiting at its start location until at least timestep
bool PBS::wait_at_start(const Path& path, int start_location, int timestep)
{
    for (auto& state : path)
    {
        if (state.timestep > timestep)
            return true;
        else if (state.location != start_location)
            return false;
    }
    return false; // when the path is empty
}


void PBS::find_replan_agents(PBSNode* node, const list<Conflict>& conflicts,
        unordered_set<int>& replan)
{
    clock_t t2 = clock();
    for (const auto& conflict : conflicts)
    {

        int a1, a2, v1, v2, t;
        std::tie(a1, a2, v1, v2, t) = conflict;
        if (replan.find(a1) != replan.end() || replan.find(a2) != replan.end())
            continue;
        else if (prioritize_start && wait_at_start(*paths[a1], v1, t))
        {
            replan.insert(a2);
            continue;
        }
        else if (prioritize_start && wait_at_start(*paths[a2], v2, t))
        {
            replan.insert(a1);
            continue;
        }
        if (node->priorities.connected(a1, a2))
        {
            replan.insert(a1);
            continue;
        }
        if (node->priorities.connected(a2, a1))
        {
            replan.insert(a2);
            continue;
        }
    }
    runtime_find_replan_agents += (double)(std::clock() - t2) / CLOCKS_PER_SEC;
}


bool PBS::find_consistent_paths(PBSNode* node, int agent)
{
    clock_t t = clock();
    int count = 0; // count the times that we call the low-level search.
    unordered_set<int> replan;
    if (agent >= 0 && agent < num_of_agents)
        replan.insert(agent);
    find_replan_agents(node, node->conflicts, replan);
    /*clock_t t2 = clock();
    PathTable pt(paths, window, k_robust);
    runtime_detect_conflicts += (double)(std::clock() - t2) / CLOCKS_PER_SEC;*/
    while (!replan.empty())
    {
        if (count > (int) node->paths.size() * 5)
        {
            runtime_find_consistent_paths += (double)(std::clock() - t) / CLOCKS_PER_SEC;
            return false;
        }
        int a = *replan.begin();
        replan.erase(a);
        count++;
        /*t2 = clock();
        pt.remove(paths[a], a);
        runtime_detect_conflicts += (double)(std::clock() - t2) / CLOCKS_PER_SEC;*/
        if (!find_path(node, a))
        {
            runtime_find_consistent_paths += (double)(std::clock() - t) / CLOCKS_PER_SEC;
            return false;
        }
        remove_conflicts(node->conflicts, a);
        list<Conflict> new_conflicts;
        find_conflicts(new_conflicts, a);
        /*t2 = clock();
        std::list< std::shared_ptr<Conflict> > new_conflicts = pt.add(paths[a], a);
        runtime_detect_conflicts += (double)(std::clock() - t2) / CLOCKS_PER_SEC;*/
        find_replan_agents(node, new_conflicts, replan);

        node->conflicts.splice(node->conflicts.end(), new_conflicts);
    }
    runtime_find_consistent_paths += (double)(std::clock() - t) / CLOCKS_PER_SEC;
    if (screen == 2)
        return validate_consistence(node->conflicts, node->priorities);
    return true;
}


bool PBS::validate_consistence(const list<Conflict>& conflicts, const PriorityGraph &G)
{
    for (auto conflict : conflicts)
    {
        int a1 = std::get<0>(conflict);
        int a2 = std::get<1>(conflict);
        if (G.connected(a1, a2) || G.connected(a2, a1))
            return false;
    }
    return true;
}



bool PBS::generate_child(PBSNode* node, PBSNode* parent)
{
	node->parent = parent;
	node->g_val = parent->g_val;
	node->makespan = parent->makespan;
	node->depth = parent->depth + 1;

    if (lazyPriority)
    {
        // TODO: add wait at starts
        if (node->parent->priorities.connected(node->priority.second, node->priority.first))
            return false;
        node->priorities.copy(node->parent->priorities);
        node->priorities.add(node->priority.first, node->priority.second);
        int a = node->priority.first;
        if (!find_path(node, a))
            return false;
        find_conflicts(node->parent->conflicts, node->conflicts, a);
        if (screen == 2)
        {
            for (auto conflict : node->conflicts)
            {
                int a1 = std::get<0>(conflict);
                int a2 = std::get<1>(conflict);
                if (a1 == a)
                {
                    if (parent->priority.first == a2 || parent->priority.second == a2)
                    {
                        assert(!node->priorities.connected(a1, a2));
                    }
                }

            }
        }

    }
    else
    {
        clock_t t = clock();
        node->priorities.copy(node->parent->priorities);
        node->priorities.add(node->priority.first, node->priority.second);
        runtime_copy_priorities += (double)(std::clock() - t) / CLOCKS_PER_SEC;
        copy_conflicts(node->parent->conflicts, node->conflicts, -1); // copy all conflicts
        if (!find_consistent_paths(node, node->priority.first))
            return false;
    }





    node->num_of_collisions = node->conflicts.size();

	//Estimate h value
	node->h_val = 0;
	node->f_val = node->g_val + node->h_val;

    // push_node(node);

	return true;
}


bool PBS::generate_root_node()
{
    clock_t time = std::clock();
	dummy_start = new PBSNode();
	
	// initialize paths_found_initially
	paths.resize(num_of_agents, nullptr);
	
    if (screen == 2)
        std::cout << "Generate root CT node ..." << std::endl;

    if (!initial_paths.empty())
    {
        for (int i = 0; i < num_of_agents; i++)
        {
            if (!initial_paths[i].empty())
            {
                dummy_start->paths.emplace_back(make_pair(i, initial_paths[i]));
                paths[i] = &dummy_start->paths.back().second;
                dummy_start->makespan = std::max(dummy_start->makespan, paths[i]->size() - 1);
                dummy_start->g_val += get_path_cost(*paths[i]);
            }
        }
    }


    vector<int> root_order(num_of_agents);
    std::iota(root_order.begin(), root_order.end(), 0);
    if (!dummy_start->priorities.empty())
    {
        vector<int> lower_nodes(num_of_agents, -1);
        for (int i = 0; i < num_of_agents; i++)
            dummy_start->priorities.update_number_of_lower_nodes(lower_nodes, i);
        std::sort(root_order.begin(), root_order.end(), [&](int lhs, int rhs) {
            if (lower_nodes[lhs] != lower_nodes[rhs])
                return lower_nodes[lhs] > lower_nodes[rhs];
            return lhs < rhs;
        });
    }

    for (int i : root_order)
	{
        if (paths[i] != nullptr)
            continue;
        Path path;
        double path_cost;
        int start_location  = starts[i].location;
        clock_t t = std::clock();
        rt.copy(initial_rt);
        rt.build(paths, initial_constraints, dummy_start->priorities.get_reachable_nodes(i), i, start_location);
        runtime_get_higher_priority_agents += dummy_start->priorities.runtime;
        runtime_rt += (double)(std::clock() - t) / CLOCKS_PER_SEC;
        t = std::clock();
        path = path_planner.run(G, starts[i], goal_locations[i], rt);
        runtime_plan_paths += (double)(std::clock() - t) / CLOCKS_PER_SEC;
        path_cost = path_planner.path_cost;
        rt.clear();
        LL_num_expanded += path_planner.num_expanded;
        LL_num_generated += path_planner.num_generated;

        if (path.empty())
        {
            std::cout << "NO SOLUTION EXISTS";
            return false;
        }

        dummy_start->paths.emplace_back(i, path);
        paths[i] = &dummy_start->paths.back().second;
        dummy_start->makespan = std::max(dummy_start->makespan, paths[i]->size() - 1);
        dummy_start->g_val += path_cost;
	}
    find_conflicts(dummy_start->conflicts);
    if (!lazyPriority)
    {
        if(!find_consistent_paths(dummy_start, -1))
            return false;
    }

	dummy_start->f_val = dummy_start->g_val;
    dummy_start->num_of_collisions = dummy_start->conflicts.size();
    // focal_list_threshold = min_f_val * focal_w;
    best_node = dummy_start;
    HL_num_generated++;
    dummy_start->time_generated = HL_num_generated;
    push_node(dummy_start);
    if (screen == 2)
    {
        double runtime = (double)(std::clock() - time) / CLOCKS_PER_SEC;
        std::cout << "Done! (" << runtime << "s)" << std::endl;
    }
    return true;
}

void PBS::push_node(PBSNode* node)
{
    dfs.push_back(node);
    allNodes_table.push_back(node);
}

PBSNode* PBS::pop_node()
{
    PBSNode* node = dfs.back();
    dfs.pop_back();
    return node;
}

void PBS::update_best_node(PBSNode* node)
{
    if (node->earliest_collision > best_node->earliest_collision or
        (node->earliest_collision == best_node->earliest_collision &&
            node->f_val < best_node->f_val))
        best_node = node;
}

bool PBS::run(const vector<State>& starts,
                    const vector< vector<pair<int, int> > >& goal_locations,
                    int _time_limit)
{
    clear();

    // set timer
	start = std::clock();
    
    this->starts = starts;
    this->goal_locations = goal_locations;
    this->num_of_agents = starts.size();
    this->time_limit = _time_limit;
    initial_rt.prepareInitialConstraints(initial_constraints);
    initialize_pressure_cache();

    solution_cost = -2;
    solution_found = false;

    rt.num_of_agents = num_of_agents;
    rt.map_size = G.size();
    rt.k_robust = k_robust;
    rt.window = window;
	rt.hold_endpoints = hold_endpoints;
    path_planner.travel_times = travel_times;
	path_planner.hold_endpoints = hold_endpoints;
	path_planner.prioritize_start = prioritize_start;

    if (!generate_root_node())
        return false;

    if (dummy_start->num_of_collisions == 0) //no conflicts at the root node
    {// found a solution (and finish the while look)
        solution_found = true;
        solution_cost = dummy_start->g_val;
        best_node = dummy_start;
    }

    // start the loop
	while (!dfs.empty() && !solution_found)
	{
		runtime = (double)(std::clock() - start)  / CLOCKS_PER_SEC;
        if (runtime > time_limit)
		{  // timeout
			solution_cost = -1;
			solution_found = false;
			break;
		}

		PBSNode* curr = pop_node();
		update_paths(curr);

        if (curr->conflicts.empty())
        {// found a solution (and finish the while look)
            solution_found = true;
            solution_cost = curr->g_val;
            best_node = curr;
            break;
        }
	
		choose_conflict(*curr);

        update_best_node(curr);

		 //Expand the node
		HL_num_expanded++;

		curr->time_expanded = HL_num_expanded;
		if(screen == 2)
			std::cout << "Expand Node " << curr->time_generated << " ( cost = " << curr->f_val << " , #conflicts = " <<
			curr->num_of_collisions << " ) on conflict " << curr->conflict << std::endl;
		PBSNode* n[2];
        for (auto & i : n)
                i = new PBSNode();
	    resolve_conflict(curr->conflict, n[0], n[1]);
        pair<int, int> preferred_priority(-1, -1);
        bool biased_branch = prefer_workstation_branch(curr->conflict, preferred_priority);
        if (biased_branch && n[0]->priority != preferred_priority)
            std::swap(n[0], n[1]);

        // int loc = std::get<2>(*curr->conflict);
        vector<Path*> copy(paths);
        for (auto & i : n)
        {
            bool sol = generate_child(i, curr);
            if (sol)
            {
                HL_num_generated++;
                i->time_generated = HL_num_generated;
            }
            if (sol)
            {
                if (screen == 2)
                {
                    std::cout << "Generate #" << i->time_generated << " with "
                              << i->paths.size() << " new paths, "
                              << i->g_val - curr->g_val << " delta cost and "
                              << i->num_of_collisions << " conflicts " << std::endl;
                }
                if (i->num_of_collisions == 0) //no conflicts
                {// found a solution (and finish the while look)
                    solution_found = true;
                    solution_cost = i->g_val;
                    best_node = i;
                    allNodes_table.push_back(i);
                    break;
                }
            }
		    else
		    {
			    delete i;
			    i = nullptr;
		    }
		    paths = copy;
        }

        if (!solution_found)
        {
            if (n[0] != nullptr && n[1] != nullptr)
            {
                if (biased_branch)
                {
                    push_node(n[1]);
                    push_node(n[0]);
                }
                else if (n[0]->f_val < n[1]->f_val ||
                    (n[0]->f_val == n[1]->f_val && n[0]->num_of_collisions < n[1]->num_of_collisions))
                {
                    push_node(n[1]);
                    push_node(n[0]);
                }
                else
                {
                    push_node(n[0]);
                    push_node(n[1]);
                }
            }
            else if (n[0] != nullptr)
            {
                push_node(n[0]);
            }
            else if (n[1] != nullptr)
            {
                push_node(n[1]);
            } else
            {
                // std::cout << "*******A new nogood ********" << endl;
                nogood.emplace(std::get<0>(curr->conflict), std::get<1>(curr->conflict));
            }
            curr->clear();
        }
	}  // end of while loop


	runtime = (double)(std::clock() - start) / CLOCKS_PER_SEC;
    get_solution();
	if (solution_found && !validate_solution())
	{
        std::cout << "Solution invalid!!!" << std::endl;
        // print_paths();
        exit(-1);
	}
    min_sum_of_costs = 0;
    for (int i = 0; i < num_of_agents; i++)
    {
        int start_loc = starts[i].location;
        for (const auto& goal : goal_locations[i])
        {
            min_sum_of_costs += G.heuristics.at(goal.first)[start_loc];
            start_loc = goal.first;
        }
    }
	if (screen > 0) // 1 or 2
		print_results();
	return solution_found;
}


void PBS::resolve_conflict(const Conflict& conflict, PBSNode* n1, PBSNode* n2)
{
    int a1, a2, v1, v2, t;
    std::tie(a1, a2, v1, v2, t) = conflict;
    n1->priority = std::make_pair(a1, a2);
    n2->priority = std::make_pair(a2, a1);
}



PBS::PBS(const BasicGraph& G, SingleAgentSolver& path_planner) : MAPFSolver(G, path_planner),
        lazyPriority(false), best_node(nullptr) {}


inline void PBS::release_closed_list()
{
	for (auto & it : allNodes_table)
		delete it;
	allNodes_table.clear();
}


PBS::~PBS()
{
	release_closed_list();
}


bool PBS::validate_solution()
{
    list<Conflict> conflict;
	for (int a1 = 0; a1 < num_of_agents; a1++)
	{
		for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
		{
            find_conflicts(conflict, a1, a2);
            if (!conflict.empty())
            {
                int a1_, a2_, loc1, loc2, t;
                std::tie(a1_, a2_, loc1, loc2, t) = conflict.front();
                if (loc2 < 0)
                    std::cout << "Agents "  << a1 << " and " << a2 << " collides at " << loc1 <<
                    " at timestep " << t << std::endl;
                else
                    std::cout << "Agents " << a1 << " and " << a2 << " collides at (" <<
                              loc1 << "-->" << loc2 << ") at timestep " << t << std::endl;
                return false;
            }
		}
	}
	return true;
}

void PBS::print_paths() const
{
	for (int i = 0; i < num_of_agents; i++)
	{
		if (paths[i] == nullptr)
            continue;
        std::cout << "Agent " << i << " (" << paths[i]->size() - 1 << "): ";
		for (const auto& s : (*paths[i]))
			std::cout << s.location << "->";
		std::cout << std::endl;
	}
}


// adding new nodes to FOCAL (those with min-f-val*f_weight between the old and new LB)
void PBS::update_focal_list()
{
	/*PBSNode* open_head = open_list.top();
	if (open_head->f_val > min_f_val)
	{
		if (screen == 2)
		{
			std::cout << "  Note -- FOCAL UPDATE!! from |FOCAL|=" << focal_list.size() << " with |OPEN|=" << open_list.size() << " to |FOCAL|=";
		}
		min_f_val = open_head->f_val;
		double new_focal_list_threshold = min_f_val * focal_w;
		for (PBSNode* n : open_list)
		{
			if (n->f_val > focal_list_threshold &&
				n->f_val <= new_focal_list_threshold)
				n->focal_handle = focal_list.push(n);
		}
		focal_list_threshold = new_focal_list_threshold;
		if (screen == 2)
		{
			std::cout << focal_list.size() << std::endl;
		}
	}*/
}

void PBS::update_CAT(int ex_ag)
{
    size_t makespan = 0;
	for (int ag = 0; ag < num_of_agents; ag++) 
    {
        if (ag == ex_ag || paths[ag] == nullptr)
            continue;
        makespan = std::max(makespan, paths[ag]->size());
    }
	cat.clear();
    cat.resize(makespan);
    for (int t = 0; t < (int)makespan; t++)
        cat[t].resize(G.size(), false);

    for (int ag = 0; ag < num_of_agents; ag++) 
	{
        if (ag == ex_ag || paths[ag] == nullptr)
            continue; 
		for (int timestep = 0; timestep < (int)paths[ag]->size() - 1; timestep++)
		{
			int loc = paths[ag]->at(timestep).location;
			if (loc < 0)
			    continue;
			for (int t = max(0, timestep - k_robust); t <= timestep + k_robust; t++)
			    cat[t][loc] = true;
		}
	}
}

void PBS::print_results() const
{
    std::cout << "PBS:";
	if(solution_cost >= 0) // solved
		std::cout << "Succeed,";
	else if(solution_cost == -1) // time_out
		std::cout << "Timeout,";
	else if(solution_cost == -2) // no solution
		std::cout << "No solutions,";
	else if (solution_cost == -3) // nodes out
		std::cout << "Nodesout,";

	std::cout << runtime << "," <<
		HL_num_expanded << "," << HL_num_generated << "," <<
		solution_cost << "," << min_sum_of_costs << "," <<
		avg_path_length << "," << dummy_start->num_of_collisions << "," <<
		runtime_plan_paths << "," << runtime_rt << "," <<
		runtime_get_higher_priority_agents << "," <<
		runtime_copy_priorities << "," <<
		runtime_detect_conflicts << "," <<
		runtime_copy_conflicts << "," <<
		runtime_choose_conflict << "," <<
		runtime_find_consistent_paths << "," <<
		runtime_find_replan_agents <<
		std::endl;
}

void PBS::save_results(const std::string &fileName, const std::string &instanceName) const
{
	std::ofstream stats;
	stats.open(fileName, std::ios::app);
	stats << runtime << "," <<
		HL_num_expanded << "," << HL_num_generated << "," <<
		LL_num_expanded << "," << LL_num_generated << "," <<
		solution_cost << "," << min_sum_of_costs << "," <<
		avg_path_length << "," << dummy_start->num_of_collisions << "," <<
		instanceName << std::endl;
	stats.close();
}

void PBS::print_conflicts(const PBSNode &curr)
{
	for (auto c : curr.conflicts)
	{
		std::cout << c << std::endl;
	}
}


void PBS::save_search_tree(const std::string &fname) const
{
    std::ofstream output;
    output.open(fname, std::ios::out);
    output << "digraph G {" << std::endl;
    output << "size = \"5,5\";" << std::endl;
    output << "center = true;" << std::endl;
    for (auto node : allNodes_table)
    {
        if (node == dummy_start)
            continue;
        else if (node->time_expanded == 0) // this node is in the openlist
            output << node->time_generated << " [color=blue]" << std::endl;
        output << node->parent->time_generated << " -> " << node->time_generated << std::endl;
    }
    output << "}" << std::endl;
    output.close();
}

void PBS::save_constraints_in_goal_node(const std::string &fileName) const
{
	best_node->priorities.save_as_digraph(fileName );
}

void PBS::get_solution()
{
    update_paths(best_node);
    solution.resize(num_of_agents);
    for (int k = 0; k < num_of_agents; k++)
    {
        solution[k] = *paths[k];
    }

    //solution_cost  = 0;
    avg_path_length = 0;

    for (int k = 0; k < num_of_agents; k++)
    {
        /*if (k == 250)
            cout << solution[k] << endl;
        solution[k].clear();
        rt.build(solution, initial_constraints, k);
        vector< vector<double> > h_values(goal_locations[k].size());
        for (int j = 0; j < (int) goal_locations[k].size(); j++)
        {
            h_values[j] = G.heuristics.at(goal_locations[k][j]);
        }
        solution[k] = path_planner.run(G, starts[k], goal_locations[k], rt, h_values);
        if (solution[k].empty())
            cout << "ERROR" << endl;
        rt.clear();
        LL_num_expanded += path_planner.num_expanded;
        LL_num_generated += path_planner.num_generated;*/
        //solution_cost += path_planner.path_cost;
        avg_path_length += paths[k]->size();
    }
    avg_path_length /= num_of_agents;
}
