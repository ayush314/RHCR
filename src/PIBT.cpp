#include "PIBT.h"

#include "WorkstationGraph.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>

namespace
{
constexpr int kDefaultPressureThreshold = 2;
constexpr int kMissingPriorityValue = std::numeric_limits<int>::max() / 4;

bool is_zone_member(const WorkstationStation& station, int loc)
{
    if (loc == station.workstation)
        return false;
    return station.zone_cells.find(loc) != station.zone_cells.end();
}

uint64_t mix64(uint64_t value)
{
    value += 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31);
}

} // namespace

PIBT::PIBT(const BasicGraph& G, SingleAgentSolver& path_planner) :
    MAPFSolver(G, path_planner) {}

PIBT::Policy PIBT::active_policy() const
{
    if (pibt_policy == "distance_age")
        return Policy::DISTANCE_AGE;
    if (pibt_policy == "pressure")
        return Policy::PRESSURE;
    return Policy::VANILLA;
}

bool PIBT::use_exit_priority() const
{
    if (active_policy() == Policy::VANILLA)
        return false;
    return active_policy() != Policy::PRESSURE || phase_priority_enabled;
}

bool PIBT::use_front_runner() const
{
    return active_policy() == Policy::PRESSURE;
}

bool PIBT::use_full_terms() const
{
    return active_policy() == Policy::PRESSURE;
}

bool PIBT::use_regret() const
{
    return active_policy() == Policy::PRESSURE && regret_iterations > 1;
}

bool PIBT::regret_applies_to_agent(int agent) const
{
    return use_regret() && agent >= 0 && agent < (int)regret_agent_enabled.size() &&
           regret_agent_enabled[agent];
}

void PIBT::update_regret_scope(const vector<State>& current_states)
{
    regret_agent_enabled.assign(num_of_agents, use_regret());
    if (!use_regret() || regret_scope == "all")
        return;

    const auto* grid = dynamic_cast<const WorkstationGrid*>(&G);
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        bool enabled = false;
        WorkstationAgentPhase phase = WorkstationAgentPhase::NONE;
        if (workstation_context.size() == (size_t)num_of_agents)
            phase = workstation_context[agent].phase;

        if (regret_scope == "pickup")
            enabled = phase == WorkstationAgentPhase::TO_PICKUP;
        else if (regret_scope == "exit_pickup")
            enabled = phase == WorkstationAgentPhase::TO_EXIT ||
                      phase == WorkstationAgentPhase::TO_PICKUP;
        else if (regret_scope == "outside_zone")
            enabled = grid == nullptr || agent >= (int)current_states.size() ||
                      grid->station_for_zone_cell(current_states[agent].location) < 0;
        else if (regret_scope == "pickup_outside_zone")
            enabled = phase == WorkstationAgentPhase::TO_PICKUP &&
                      (grid == nullptr || agent >= (int)current_states.size() ||
                       grid->station_for_zone_cell(current_states[agent].location) < 0);
        regret_agent_enabled[agent] = enabled;
    }
}

int PIBT::effective_workstation_pressure_threshold() const
{
    if (workstation_pressure_threshold > 0)
        return workstation_pressure_threshold;
    return kDefaultPressureThreshold;
}

int PIBT::step_assignment_budget() const
{
    return std::max(20000, num_of_agents * 600);
}

void PIBT::clear()
{
    runtime = 0;
    solution_found = false;
    solution_cost = -2;
    avg_path_length = -1;
    min_sum_of_costs = 0;
    solution.clear();
    inheritance_calls = 0;
    backtracks = 0;
    wait_fallbacks = 0;
    pressure_rank_changes = 0;
    regret_updates = 0;
    regret_values.clear();
    regret_agent_enabled.clear();
}

void PIBT::initialize_run_state()
{
    if ((int)priority_age.size() != num_of_agents)
        priority_age.assign(num_of_agents, 0);
    goal_indices.assign(num_of_agents, 0);
    last_goal_advance_timestep.assign(num_of_agents, -1);
    last_goal_advance_location.assign(num_of_agents, -1);
    solution.assign(num_of_agents, Path());
    for (int i = 0; i < num_of_agents; i++)
        solution[i].push_back(starts[i]);
}

tuple<int, int, int, int> PIBT::front_runner_key(int agent,
                                                 const vector<State>& current_states) const
{
    const WorkstationAgentContext& ctx = workstation_context[agent];
    int boundary_entry = ctx.boundary_entry_t >= 0 ? ctx.boundary_entry_t : kMissingPriorityValue;
    int dist = kMissingPriorityValue;
    const auto* grid = dynamic_cast<const WorkstationGrid*>(&G);
    if (grid != nullptr && ctx.station_id >= 0 && agent < (int)current_states.size())
        dist = grid->distance_to_workstation(ctx.station_id, current_states[agent].location);
    int task_issue = ctx.task_issue_t >= 0 ? ctx.task_issue_t : kMissingPriorityValue;
    return make_tuple(boundary_entry, dist, task_issue, agent);
}

tuple<int, int, int> PIBT::distance_age_key(int agent,
                                            const vector<State>& current_states) const
{
    const WorkstationAgentContext& ctx = workstation_context[agent];
    int dist = kMissingPriorityValue;
    const auto* grid = dynamic_cast<const WorkstationGrid*>(&G);
    if (grid != nullptr && ctx.station_id >= 0 && agent < (int)current_states.size())
        dist = grid->distance_to_workstation(ctx.station_id, current_states[agent].location);
    int task_issue = ctx.task_issue_t >= 0 ? ctx.task_issue_t : kMissingPriorityValue;
    return make_tuple(dist, task_issue, agent);
}

int PIBT::workstation_front_runner(int station_id) const
{
    return workstation_front_runner(station_id, starts);
}

int PIBT::workstation_front_runner(int station_id,
                                   const vector<State>& current_states) const
{
    int best_agent = -1;
    tuple<int, int, int, int> best_key;
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        const auto& ctx = workstation_context[agent];
        if (ctx.phase != WorkstationAgentPhase::TO_STATION || ctx.station_id != station_id)
            continue;
        auto key = front_runner_key(agent, current_states);
        if (best_agent < 0 || key < best_key)
        {
            best_agent = agent;
            best_key = key;
        }
    }
    return best_agent;
}

void PIBT::update_pressure_state(const vector<State>& current_states)
{
    const auto* grid = dynamic_cast<const WorkstationGrid*>(&G);
    station_pressure_values.clear();
    station_front_runners.clear();
    station_entry_runners.clear();
    if (grid == nullptr)
        return;

    station_pressure_values.assign(grid->stations.size(), 0);
    station_front_runners.assign(grid->stations.size(), -1);
    station_entry_runners.assign(grid->stations.size(), -1);
    for (size_t station_id = 0; station_id < grid->stations.size(); station_id++)
    {
        const auto& station = grid->stations[station_id];
        int pressure = 0;
        for (const State& state : current_states)
        {
            if (is_zone_member(station, state.location))
                pressure++;
        }
        station_pressure_values[station_id] = pressure;
        if (pressure >= effective_workstation_pressure_threshold() && use_front_runner())
        {
            station_front_runners[station_id] = workstation_front_runner((int)station_id, current_states);
            station_entry_runners[station_id] = workstation_entry_runner((int)station_id, current_states);
        }
    }
}

int PIBT::workstation_entry_runner(int station_id,
                                   const vector<State>& current_states) const
{
    const auto* grid = dynamic_cast<const WorkstationGrid*>(&G);
    if (grid == nullptr || station_id < 0 || station_id >= (int)grid->stations.size())
        return -1;

    int inbound_in_zone = 0;
    int best_agent = -1;
    tuple<int, int, int, int> best_key;
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        const auto& ctx = workstation_context[agent];
        if (ctx.phase != WorkstationAgentPhase::TO_STATION || ctx.station_id != station_id)
            continue;

        bool inside_target_zone =
            agent < (int)current_states.size() &&
            grid->station_for_zone_cell(current_states[agent].location) == station_id;
        if (inside_target_zone)
        {
            inbound_in_zone++;
            continue;
        }

        auto key = front_runner_key(agent, current_states);
        if (best_agent < 0 || key < best_key)
        {
            best_agent = agent;
            best_key = key;
        }
    }

    // Bound target-bound work in progress separately from incidental zone traffic.
    int inbound_limit = std::max(1, pressure_inbound_limit);
    int zone_capacity = std::max(1, (int)grid->stations[station_id].zone_cells.size() - 1);
    int pressure = station_id < (int)station_pressure_values.size()
        ? station_pressure_values[station_id]
        : 0;
    if (pressure_profile == "thirds" && pressure * 3 >= zone_capacity)
        inbound_limit = std::max(1, inbound_limit - 1);
    if ((pressure_profile == "severe" || pressure_profile == "thirds") &&
        pressure * 3 >= zone_capacity * 2)
        inbound_limit = std::max(1, inbound_limit - 1);
    if (pressure_profile == "half" && pressure * 2 >= zone_capacity)
        inbound_limit = std::max(1, inbound_limit - 1);
    if (inbound_in_zone >= inbound_limit)
        return -1;
    return best_agent;
}

void PIBT::advance_goal_index(int agent, const State& state, int local_t)
{
    while (goal_indices[agent] < (int)goal_locations[agent].size())
    {
        const auto& goal = goal_locations[agent][goal_indices[agent]];
        if (state.location != goal.first || local_t < goal.second)
            break;
        if (last_goal_advance_timestep[agent] == local_t &&
            last_goal_advance_location[agent] == state.location)
        {
            break;
        }
        goal_indices[agent]++;
        last_goal_advance_timestep[agent] = local_t;
        last_goal_advance_location[agent] = state.location;
    }
}

int PIBT::current_target(int agent) const
{
    if (agent < 0 || agent >= (int)goal_locations.size())
        return -1;
    int goal_idx = goal_indices[agent];
    if (goal_idx < 0 || goal_idx >= (int)goal_locations[agent].size())
        return -1;
    return goal_locations[agent][goal_idx].first;
}

int PIBT::distance_between(int from, int to) const
{
    if (from < 0 || to < 0)
        return kMissingPriorityValue;
    auto it = G.heuristics.find(to);
    if (it != G.heuristics.end() && from < (int)it->second.size())
        return (int)it->second[from];
    return G.get_Manhattan_distance(from, to);
}

double PIBT::distance_to_remaining_goals(int agent, int loc, int goal_idx) const
{
    if (goal_idx >= (int)goal_locations[agent].size())
        return 0;

    double dist = 0;
    int from = loc;
    for (int i = goal_idx; i < (int)goal_locations[agent].size(); i++)
    {
        int to = goal_locations[agent][i].first;
        dist += distance_between(from, to);
        from = to;
    }
    return dist;
}

bool PIBT::is_front_runner(int agent) const
{
    if (!use_front_runner() || workstation_context.size() != (size_t)num_of_agents)
        return false;
    const auto& ctx = workstation_context[agent];
    return ctx.station_id >= 0 &&
           ctx.station_id < (int)station_front_runners.size() &&
           station_front_runners[ctx.station_id] == agent;
}

int PIBT::agent_class_rank(int agent) const
{
    if (!use_exit_priority() || workstation_context.size() != (size_t)num_of_agents)
        return agent;

    const auto& ctx = workstation_context[agent];
    if (ctx.phase == WorkstationAgentPhase::SERVICE)
        return 0;
    if (ctx.phase == WorkstationAgentPhase::TO_EXIT)
        return 1;
    if (front_priority_enabled && is_front_runner(agent))
        return 2;
    if (ctx.phase == WorkstationAgentPhase::TO_STATION)
        return 3;
    if (ctx.phase == WorkstationAgentPhase::TO_PICKUP)
        return 4;
    return 5;
}

bool PIBT::is_station_privileged(int agent, int station_id) const
{
    if (workstation_context.size() != (size_t)num_of_agents)
        return false;
    const auto& ctx = workstation_context[agent];
    if (ctx.station_id != station_id)
        return false;
    if (ctx.phase == WorkstationAgentPhase::SERVICE || ctx.phase == WorkstationAgentPhase::TO_EXIT)
        return true;
    return use_front_runner() &&
           station_id >= 0 && station_id < (int)station_entry_runners.size() &&
           station_entry_runners[station_id] == agent;
}

bool PIBT::must_hold_for_workstation_service(int agent, const State& state) const
{
    if (workstation_context.size() != (size_t)num_of_agents)
        return false;
    const auto* grid = dynamic_cast<const WorkstationGrid*>(&G);
    if (grid == nullptr)
        return false;

    const auto& ctx = workstation_context[agent];
    if (ctx.phase != WorkstationAgentPhase::SERVICE &&
        ctx.phase != WorkstationAgentPhase::TO_STATION)
    {
        return false;
    }
    if (ctx.station_id < 0 || ctx.station_id >= (int)grid->stations.size())
        return false;

    int workstation = grid->stations[ctx.station_id].workstation;
    if (state.location != workstation)
        return false;

    int target = current_target(agent);
    return target < 0 || target == workstation;
}

double PIBT::pressure_score(int agent,
                            const State& candidate,
                            const vector<State>& current_states,
                            const vector<int>& current_occupant) const
{
    if (workstation_context.size() != (size_t)num_of_agents)
        return 0;
    const auto* grid = dynamic_cast<const WorkstationGrid*>(&G);
    if (grid == nullptr)
        return 0;

    double score = 0;
    if (active_policy() == Policy::PRESSURE)
    {
        int next_station = grid->station_for_zone_cell(candidate.location);
        int curr_station = grid->station_for_zone_cell(current_states[agent].location);
        bool entering_pressured_zone =
            next_station >= 0 &&
            next_station < (int)station_pressure_values.size() &&
            curr_station != next_station &&
            station_pressure_values[next_station] >= effective_workstation_pressure_threshold();
        if (entering_pressured_zone && !is_station_privileged(agent, next_station))
        {
            score += pressure_entry_penalty;
        }

    }

    if (use_front_runner() && is_front_runner(agent))
    {
        int station_id = workstation_context[agent].station_id;
        if (station_id >= 0)
        {
            int curr_dist = grid->distance_to_workstation(station_id, current_states[agent].location);
            int next_dist = grid->distance_to_workstation(station_id, candidate.location);
            if (next_dist < curr_dist)
                score -= front_bonus * (curr_dist - next_dist);
        }
    }

    if (use_full_terms())
    {
        const auto& ctx = workstation_context[agent];
        if (ctx.phase == WorkstationAgentPhase::TO_EXIT)
        {
            int target = current_target(agent);
            int curr_dist = distance_between(current_states[agent].location, target);
            int next_dist = distance_between(candidate.location, target);
            if (next_dist < curr_dist)
                score -= exit_bonus * (curr_dist - next_dist);
        }
        if (candidate.location == current_states[agent].location &&
            current_target(agent) >= 0 &&
            current_states[agent].location != current_target(agent))
        {
            bool apply_wait_penalty = true;
            if (active_policy() == Policy::PRESSURE &&
                ctx.phase == WorkstationAgentPhase::TO_STATION &&
                ctx.station_id >= 0 &&
                ctx.station_id < (int)station_pressure_values.size())
            {
                int curr_station = grid->station_for_zone_cell(current_states[agent].location);
                bool outside_station_zones = curr_station < 0;
                bool target_station_pressured =
                    station_pressure_values[ctx.station_id] >= effective_workstation_pressure_threshold();
                if (outside_station_zones &&
                    target_station_pressured &&
                    !is_station_privileged(agent, ctx.station_id))
                {
                    apply_wait_penalty = false;
                }
            }
            if (apply_wait_penalty)
                score += wait_penalty;
        }
        if (candidate.location >= 0 && candidate.location < (int)current_occupant.size())
        {
            int other = current_occupant[candidate.location];
            if (other >= 0 && other != agent)
                score += soft_collision_penalty;
        }
    }
    return score;
}

int PIBT::hindrance_score(int agent,
                          const State& candidate,
                          const vector<State>& current_states,
                          const vector<int>& current_occupant,
                          bool inherited) const
{
    if (!hindrance_tiebreak || active_policy() != Policy::PRESSURE)
        return 0;
    if ((hindrance_scope == "inherited" || hindrance_scope == "inherited_dense" ||
         hindrance_scope == "inherited_station" ||
         hindrance_scope == "inherited_outside_zone" ||
         hindrance_scope == "inherited_pickup") && !inherited)
    {
        return 0;
    }
    if ((hindrance_scope == "station" || hindrance_scope == "inherited_station") &&
        workstation_context.size() == (size_t)num_of_agents)
    {
        auto phase = workstation_context[agent].phase;
        if (phase != WorkstationAgentPhase::TO_STATION &&
            phase != WorkstationAgentPhase::TO_EXIT &&
            phase != WorkstationAgentPhase::SERVICE)
        {
            return 0;
        }
    }
    if ((hindrance_scope == "pickup" || hindrance_scope == "inherited_pickup") &&
        workstation_context.size() == (size_t)num_of_agents &&
        workstation_context[agent].phase != WorkstationAgentPhase::TO_PICKUP)
    {
        return 0;
    }
    if (hindrance_scope == "outside_zone" || hindrance_scope == "inherited_outside_zone")
    {
        const auto* grid = dynamic_cast<const WorkstationGrid*>(&G);
        if (grid != nullptr && grid->station_for_zone_cell(current_states[agent].location) >= 0)
            return 0;
    }

    if (hindrance_scope == "dense" || hindrance_scope == "inherited_dense")
    {
        int occupied_neighbors = 0;
        for (const State& neighbor : G.get_neighbors(current_states[agent]))
        {
            if (neighbor.location >= 0 && neighbor.location < (int)current_occupant.size())
            {
                int other = current_occupant[neighbor.location];
                if (other >= 0 && other != agent)
                    occupied_neighbors++;
            }
        }
        if (occupied_neighbors < 2)
            return 0;
    }

    int hindrance = 0;
    for (const State& neighbor : G.get_neighbors(current_states[agent]))
    {
        if (neighbor.location < 0 || neighbor.location >= (int)current_occupant.size())
            continue;
        int other = current_occupant[neighbor.location];
        if (other < 0 || other == agent || candidate.location == current_states[other].location)
            continue;
        int other_target = current_target(other);
        if (other_target < 0)
            continue;
        if (distance_between(candidate.location, other_target) <
            distance_between(current_states[agent].location, other_target))
        {
            hindrance++;
        }
    }
    return hindrance;
}

double PIBT::self_regret(const vector<Candidate>& candidates, int chosen_loc) const
{
    if (candidates.empty())
        return 0;
    double best = std::numeric_limits<double>::infinity();
    double chosen = std::numeric_limits<double>::infinity();
    for (const Candidate& candidate : candidates)
    {
        best = std::min(best, candidate.base_distance);
        if (candidate.loc == chosen_loc)
            chosen = candidate.base_distance;
    }
    if (!std::isfinite(best) || !std::isfinite(chosen))
        return 0;
    return std::max(0.0, chosen - best);
}

void PIBT::update_regret(int agent, int loc, double observed_regret)
{
    if (!regret_applies_to_agent(agent) || agent >= (int)regret_values.size() ||
        loc < 0 || loc >= (int)regret_values[agent].size())
    {
        return;
    }
    double& value = regret_values[agent][loc];
    value = (1.0 - regret_weight) * value + regret_weight * observed_regret;
    regret_updates++;
}

vector<PIBT::Candidate> PIBT::ranked_candidates(int agent,
                                                int local_t,
                                                const vector<State>& current_states,
                                                const vector<int>& current_occupant,
                                                bool inherited)
{
    vector<Candidate> candidates;
    auto set_tie_break = [&](Candidate& candidate) {
        uint64_t key = tie_seed;
        int episode_t = workstation_context.size() == (size_t)num_of_agents
            ? workstation_context[agent].current_t
            : 0;
        key ^= mix64((uint64_t)(agent + 1));
        key ^= mix64((uint64_t)(episode_t + local_t + 1) << 16);
        key ^= mix64((uint64_t)(assignment_pass_index + 1) << 32);
        key ^= mix64((uint64_t)(candidate.loc + 1) << 1);
        candidate.tie_break = mix64(key);
    };
    if (current_target(agent) < 0)
    {
        Candidate wait;
        wait.state = current_states[agent];
        wait.state.timestep = local_t;
        wait.loc = wait.state.location;
        wait.wait = true;
        wait.score = 0;
        wait.base_distance = 0;
        set_tie_break(wait);
        candidates.push_back(wait);
        return candidates;
    }

    if (must_hold_for_workstation_service(agent, current_states[agent]))
    {
        Candidate wait;
        wait.state = current_states[agent];
        wait.state.timestep = local_t;
        wait.loc = wait.state.location;
        wait.wait = true;
        wait.score = distance_to_remaining_goals(agent, wait.loc, goal_indices[agent]);
        wait.base_distance = wait.score;
        set_tie_break(wait);
        candidates.push_back(wait);
        return candidates;
    }

    if (agent < (int)initial_paths.size() && local_t < (int)initial_paths[agent].size() &&
        !initial_paths[agent].empty())
    {
        Candidate committed;
        committed.state = initial_paths[agent][local_t];
        committed.state.timestep = local_t;
        committed.loc = committed.state.location;
        committed.wait = committed.loc == current_states[agent].location;
        committed.score = distance_to_remaining_goals(agent, committed.loc, goal_indices[agent]);
        committed.base_distance = committed.score;
        set_tie_break(committed);
        candidates.push_back(committed);
        return candidates;
    }

    for (const State& next : G.get_neighbors(current_states[agent]))
    {
        Candidate candidate;
        candidate.state = next;
        candidate.state.timestep = local_t;
        candidate.loc = next.location;
        candidate.wait = candidate.loc == current_states[agent].location;
        candidate.score = distance_to_remaining_goals(agent, candidate.loc, goal_indices[agent]);
        candidate.base_distance = candidate.score;
        set_tie_break(candidate);
        candidates.push_back(candidate);
    }

    auto base_candidates = candidates;
    auto base_less = [&](const Candidate& lhs, const Candidate& rhs) {
        if (lhs.score != rhs.score)
            return lhs.score < rhs.score;
        if (lhs.hindrance != rhs.hindrance)
            return lhs.hindrance < rhs.hindrance;
        if (lhs.regret != rhs.regret)
            return lhs.regret < rhs.regret;
        if (random_tiebreak && lhs.tie_break != rhs.tie_break)
            return lhs.tie_break < rhs.tie_break;
        if (lhs.wait != rhs.wait)
            return !lhs.wait;
        return lhs.loc < rhs.loc;
    };
    std::sort(base_candidates.begin(), base_candidates.end(), base_less);

    for (Candidate& candidate : candidates)
    {
        candidate.score += pressure_score(agent, candidate.state, current_states, current_occupant);
        candidate.hindrance = hindrance_score(
            agent, candidate.state, current_states, current_occupant, inherited);
        if (regret_applies_to_agent(agent) && agent < (int)regret_values.size() &&
            candidate.loc >= 0 && candidate.loc < (int)regret_values[agent].size())
        {
            candidate.regret = regret_values[agent][candidate.loc];
        }
    }

    std::sort(candidates.begin(), candidates.end(), base_less);
    if (active_policy() != Policy::VANILLA &&
        !base_candidates.empty() && !candidates.empty() &&
        base_candidates.front().loc != candidates.front().loc)
    {
        pressure_rank_changes++;
    }
    return candidates;
}

bool PIBT::violates_initial_constraint(int agent, int loc, int local_t) const
{
    for (const auto& constraint : initial_constraints)
    {
        int owner = std::get<0>(constraint);
        int constraint_loc = std::get<1>(constraint);
        int constraint_until = std::get<2>(constraint);
        if (owner != agent && loc == constraint_loc &&
            local_t < std::min(window, constraint_until))
        {
            return true;
        }
    }
    return false;
}

bool PIBT::has_edge_swap(int agent,
                         const State& candidate,
                         const vector<State>& current_states,
                         const vector<State>& next_states,
                         const vector<bool>& assigned) const
{
    if (candidate.location == current_states[agent].location)
        return false;
    for (int other = 0; other < num_of_agents; other++)
    {
        if (other == agent || !assigned[other])
            continue;
        if (current_states[other].location == candidate.location &&
            next_states[other].location == current_states[agent].location)
        {
            return true;
        }
    }
    return false;
}

bool PIBT::assign_agent(int agent,
                        int local_t,
                        const vector<State>& current_states,
                        const vector<int>& current_occupant,
                        vector<State>& next_states,
                        vector<int>& next_occupant,
                        vector<bool>& assigned,
                        vector<bool>& visiting,
                        int forbidden_next_loc,
                        double& propagated_regret)
{
    propagated_regret = 0;
    if (assigned[agent])
        return true;
    if (remaining_assignment_budget <= 0)
    {
        backtracks++;
        return false;
    }
    remaining_assignment_budget--;
    if (visiting[agent])
        return false;

    visiting[agent] = true;
    auto candidates = ranked_candidates(
        agent, local_t, current_states, current_occupant, forbidden_next_loc >= 0);
    for (const Candidate& candidate : candidates)
    {
        if (candidate.loc < 0 || candidate.loc >= (int)next_occupant.size())
            continue;
        if (candidate.loc == forbidden_next_loc)
            continue;
        if (violates_initial_constraint(agent, candidate.loc, local_t))
            continue;

        vector<State> saved_next_states = next_states;
        vector<int> saved_next_occupant = next_occupant;
        vector<bool> saved_assigned = assigned;
        vector<bool> saved_visiting = visiting;
        double inherited_regret = 0;

        int occupant = candidate.loc < (int)current_occupant.size() ? current_occupant[candidate.loc] : -1;
        if (occupant >= 0 && occupant != agent && !assigned[occupant])
        {
            inheritance_calls++;
            bool inherited_valid = assign_agent(
                occupant, local_t, current_states, current_occupant,
                next_states, next_occupant, assigned, visiting, candidate.loc,
                inherited_regret);
            update_regret(agent, candidate.loc, inherited_regret);
            if (!inherited_valid)
            {
                next_states = saved_next_states;
                next_occupant = saved_next_occupant;
                assigned = saved_assigned;
                visiting = saved_visiting;
                continue;
            }
        }

        if (next_occupant[candidate.loc] >= 0)
        {
            next_states = saved_next_states;
            next_occupant = saved_next_occupant;
            assigned = saved_assigned;
            visiting = saved_visiting;
            continue;
        }
        if (has_edge_swap(agent, candidate.state, current_states, next_states, assigned))
        {
            next_states = saved_next_states;
            next_occupant = saved_next_occupant;
            assigned = saved_assigned;
            visiting = saved_visiting;
            continue;
        }

        next_states[agent] = candidate.state;
        next_states[agent].timestep = local_t;
        assigned[agent] = true;
        next_occupant[candidate.loc] = agent;
        visiting[agent] = false;
        propagated_regret = inherited_regret + self_regret(candidates, candidate.loc);
        return true;
    }

    visiting[agent] = false;
    backtracks++;
    propagated_regret = self_regret(candidates, current_states[agent].location);
    return false;
}

bool PIBT::force_wait(int agent,
                      int local_t,
                      const vector<State>& current_states,
                      vector<State>& next_states,
                      vector<int>& next_occupant,
                      vector<bool>& assigned) const
{
    int loc = current_states[agent].location;
    if (loc < 0 || loc >= (int)next_occupant.size() || next_occupant[loc] >= 0)
        return false;
    if (violates_initial_constraint(agent, loc, local_t))
        return false;

    next_states[agent] = current_states[agent];
    next_states[agent].timestep = local_t;
    assigned[agent] = true;
    next_occupant[loc] = agent;
    return true;
}

bool PIBT::validate_step(const vector<State>& current_states,
                         const vector<State>& next_states) const
{
    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
        {
            if (next_states[a1].location == next_states[a2].location)
                return false;
            if (current_states[a1].location == next_states[a2].location &&
                current_states[a2].location == next_states[a1].location &&
                current_states[a1].location != current_states[a2].location)
            {
                return false;
            }
        }
    }
    return true;
}

bool PIBT::assignment_pass(int local_t,
                           const vector<State>& current_states,
                           vector<State>& next_states,
                           bool count_fallbacks)
{
    vector<int> current_occupant(G.size(), -1);
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        int loc = current_states[agent].location;
        if (loc >= 0 && loc < (int)current_occupant.size())
            current_occupant[loc] = agent;
    }

    vector<int> order(num_of_agents);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int lhs, int rhs) {
        int lhs_class = agent_class_rank(lhs);
        int rhs_class = agent_class_rank(rhs);
        if (use_exit_priority())
        {
            bool lhs_service_exit = lhs_class <= 1;
            bool rhs_service_exit = rhs_class <= 1;
            if (lhs_service_exit != rhs_service_exit)
                return lhs_service_exit;
        }
        if (active_policy() == Policy::PRESSURE && front_priority_enabled)
        {
            bool lhs_front = is_front_runner(lhs);
            bool rhs_front = is_front_runner(rhs);
            if (lhs_front != rhs_front)
                return lhs_front;
        }
        if (priority_age[lhs] != priority_age[rhs])
            return priority_age[lhs] > priority_age[rhs];
        if (use_exit_priority() && lhs_class != rhs_class)
            return lhs_class < rhs_class;
        if (active_policy() == Policy::DISTANCE_AGE &&
            workstation_context.size() == (size_t)num_of_agents)
        {
            bool lhs_station_bound = workstation_context[lhs].phase == WorkstationAgentPhase::TO_STATION;
            bool rhs_station_bound = workstation_context[rhs].phase == WorkstationAgentPhase::TO_STATION;
            if (lhs_station_bound && rhs_station_bound)
                return distance_age_key(lhs, current_states) < distance_age_key(rhs, current_states);
        }
        return lhs < rhs;
    });

    next_states = current_states;
    vector<int> next_occupant(G.size(), -1);
    vector<bool> assigned(num_of_agents, false);
    vector<bool> visiting(num_of_agents, false);
    remaining_assignment_budget = step_assignment_budget();

    for (int agent : order)
    {
        if (assigned[agent])
            continue;
        double propagated_regret = 0;
        if (!assign_agent(agent, local_t, current_states, current_occupant,
                          next_states, next_occupant, assigned, visiting, -1,
                          propagated_regret))
        {
            if (count_fallbacks)
                wait_fallbacks++;
            if (!force_wait(agent, local_t, current_states, next_states, next_occupant, assigned))
                return false;
        }
    }
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        if (!assigned[agent])
        {
            if (count_fallbacks)
                wait_fallbacks++;
            if (!force_wait(agent, local_t, current_states, next_states, next_occupant, assigned))
                return false;
        }
    }

    if (!validate_step(current_states, next_states))
        return false;

    return true;
}

bool PIBT::plan_one_step(int local_t, vector<State>& current_states)
{
    for (int agent = 0; agent < num_of_agents; agent++)
        advance_goal_index(agent, current_states[agent], local_t - 1);
    update_pressure_state(current_states);
    update_regret_scope(current_states);

    if (use_regret())
        regret_values.assign(num_of_agents, vector<double>(G.size(), 0));
    else
        regret_values.clear();

    int passes = use_regret() ? regret_iterations : 1;
    vector<State> next_states;
    for (int pass = 0; pass < passes; pass++)
    {
        assignment_pass_index = pass;
        bool final_pass = pass == passes - 1;
        if (!assignment_pass(local_t, current_states, next_states, final_pass))
            return false;
    }

    for (int agent = 0; agent < num_of_agents; agent++)
        solution[agent].push_back(next_states[agent]);
    current_states = next_states;
    update_dynamic_priorities(current_states, local_t);
    return true;
}

void PIBT::update_dynamic_priorities(const vector<State>& current_states, int local_t)
{
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        advance_goal_index(agent, current_states[agent], local_t);
        int target = current_target(agent);
        if (target < 0 || current_states[agent].location == target)
            priority_age[agent] = 0;
        else
            priority_age[agent]++;
    }
}

bool PIBT::run(const vector<State>& starts,
               const vector< vector<pair<int, int> > >& goal_locations,
               int time_limit)
{
    clear();
    run_start = std::clock();
    this->starts = starts;
    this->goal_locations = goal_locations;
    this->num_of_agents = (int)starts.size();
    this->time_limit = time_limit;

    initialize_run_state();

    vector<State> current_states = starts;
    int horizon = std::max(1, window);
    if (horizon > 10000)
        horizon = 10000;

    bool success = true;
    for (int local_t = 1; local_t <= horizon; local_t++)
    {
        runtime = (std::clock() - run_start) * 1.0 / CLOCKS_PER_SEC;
        if (runtime > time_limit || !plan_one_step(local_t, current_states))
        {
            success = false;
            break;
        }
    }

    while (success && (int)solution.front().size() <= horizon)
    {
        int local_t = (int)solution.front().size();
        for (int agent = 0; agent < num_of_agents; agent++)
        {
            State wait = solution[agent].back();
            wait.timestep = local_t;
            solution[agent].push_back(wait);
        }
    }

    runtime = (std::clock() - run_start) * 1.0 / CLOCKS_PER_SEC;
    solution_found = success;
    solution_cost = success ? 0 : -1;
    avg_path_length = 0;
    min_sum_of_costs = 0;
    for (int agent = 0; agent < num_of_agents; agent++)
    {
        avg_path_length += solution[agent].size();
        solution_cost += success ? (double)solution[agent].size() - 1 : 0;
        int from = starts[agent].location;
        for (const auto& goal : goal_locations[agent])
        {
            min_sum_of_costs += distance_between(from, goal.first);
            from = goal.first;
        }
    }
    if (num_of_agents > 0)
        avg_path_length /= num_of_agents;

    print_results();
    return success;
}

void PIBT::print_results() const
{
    std::cout << "PIBT:";
    if (solution_found)
        std::cout << "Succeed,";
    else
        std::cout << "Failed,";
    std::cout << runtime << ","
              << inheritance_calls << ","
              << backtracks << ","
              << wait_fallbacks << ","
              << pressure_rank_changes << ","
              << regret_updates << ","
              << solution_cost << ","
              << min_sum_of_costs << ","
              << avg_path_length << std::endl;
}

void PIBT::save_results(const std::string& fileName, const std::string& instanceName) const
{
    std::ofstream stats;
    stats.open(fileName, std::ios::app);
    stats << runtime << ","
          << inheritance_calls << ","
          << backtracks << ","
          << wait_fallbacks << ","
          << pressure_rank_changes << ","
          << regret_updates << ","
          << solution_cost << ","
          << min_sum_of_costs << ","
          << avg_path_length << ","
          << "0" << ","
          << instanceName << std::endl;
    stats.close();
}
