#pragma once

#include "BasicSystem.h"
#include "ReservationTable.h"
#include "WorkstationGraph.h"

#include <cstdint>
#include <deque>

struct WorkstationTask
{
    int station_id = -1;
    int endpoint_target = -1;
    int issue_t = -1;
    int boundary_t = -1;
    int service_in_t = -1;
    int service_complete_t = -1;
};

enum class WorkstationRuntimePhase
{
    TO_PICKUP = 0,
    TO_STATION = 1,
    SERVICE = 2,
    TO_EXIT = 3,
};

struct WorkstationAgentState
{
    std::deque<WorkstationTask> tasks;
    WorkstationRuntimePhase phase = WorkstationRuntimePhase::TO_PICKUP;
    int exit_target = -1;
    int exit_station_id = -1;
    int service_complete_t = -1;
    int last_endpoint = -1;
};

class WorkstationSystem : public BasicSystem
{
public:
    WorkstationSystem(const WorkstationGrid& G, MAPFSolver& solver);
    ~WorkstationSystem();

    int workstation_service_time = 3;
    string station_policy = "vanilla";
    string pibt_policy = "vanilla";
    int workstation_pressure_threshold = -1;
    int pressure_zone_cost = 1;
    int pressure_inbound_limit = 3;
    int pressure_cost_occupancy_threshold = 3;
    int pressure_cost_horizon = 0;
    string pressure_cost_horizon_profile = "fixed";
    bool pressure_local_action_only = false;
    bool pressure_front_runner_priority = false;
    bool pressure_front_runner_zone_only = false;
    bool pressure_front_runner_ready_priority = false;
    int pressure_front_progress_cost = 0;
    int pressure_exit_progress_cost = 0;
    bool pressure_ready_slot_priority = false;
    int pressure_lookahead_radius = 0;
    string pressure_lookahead_profile = "fixed";
    int pressure_lookahead_min_agents_per_station = 40;
    int pibt_network_pressure_fraction = 25;
    int pibt_network_pressure_min_agents_per_station = 0;
    bool pibt_global_front_runner_priority = false;
    int pibt_assignment_budget_factor = 90;
    int pibt_pressure_assignment_extension_factor = 20;
    bool pibt_front_runner_priority = false;
    bool pibt_front_runner_ready_priority = false;
    string pressure_admission = "adaptive";
    string pressure_cost_mode = "fixed";
    string pressure_cost_scope = "zone";
    string pressure_cost_activation = "zone";
    string pressure_population = "inbound_only";
    string pressure_profile = "fixed";
    bool native_failures_only = false;
    bool stop_at_traffic_jam = true;

    void simulate(int simulation_time);
    void save_results();

private:
    const WorkstationGrid& G;
    vector<WorkstationAgentState> workstation_agents;
    vector<vector<WorkstationAgentContext>> projected_goal_context;
    vector<int> queue_wait_samples;
    vector<double> mean_plan_ms_samples;
    vector<int> plan_timestep_samples;
    vector<int> pressure_active_samples;
    vector<double> pressured_station_fraction_samples;
    vector<double> zone_occupancy_fraction_samples;
    vector<double> target_queue_occupancy_samples;
    vector<int> pibt_executed_priority_age;
    struct ExecutionProgress
    {
        int completed_services = 0;
        int eligible_steps = 0;
        int moved_steps = 0;
    };
    std::deque<ExecutionProgress> execution_progress;
    long long distance_traveled = 0;
    int completed_services = 0;
    int planning_episodes = 0;
    int pressure_active_episodes = 0;
    int traffic_jam_episodes = 0;
    uint64_t pibt_inheritance_calls_total = 0;
    uint64_t pibt_backtracks_total = 0;
    uint64_t pibt_wait_fallbacks_total = 0;
    uint64_t pibt_greedy_repairs_total = 0;
    uint64_t pibt_pressure_rank_changes_total = 0;
    uint64_t pibt_pressure_active_agents_total = 0;
    uint64_t pibt_pressure_candidate_hits_total = 0;
    uint64_t pibt_budget_extensions_total = 0;
    string termination_reason = "not_started";
    int termination_timestep = -1;
    bool terminated_by_traffic_jam = false;
    bool terminated_by_commit_repair_failure = false;
    bool terminated_by_solver_failure = false;

    void initialize();
    void initialize_start_locations();
    void initialize_tasks();
    void append_random_task(WorkstationAgentState& agent);
    void ensure_lookahead_tasks(WorkstationAgentState& agent, size_t min_tasks);
    int pick_next_station(int previous_station) const;
    int pick_next_endpoint(int previous_endpoint) const;
    int agent_station_id(int agent_id) const;
    int station_pressure(int station_id) const;

    void update_goal_locations();
    void build_goal_sequence(int agent_id);
    void sync_solver_context();
    bool solve_workstation_episode();
    void record_episode_diagnostics();
    void seed_fixed_service_paths();
    void pad_paths_through_execution_window();
    void enforce_workstation_episode_commitments();
    bool resolve_committed_conflicts();
    bool validate_execution_slice(const string& label) const;
    bool workstation_congested() const;
    void move_workstations();
    void set_termination(const string& reason, int t);
    void update_initial_constraints(list< tuple<int, int, int> >& initial_constraints) const override;
    bool validate_move(int agent_id, const State& prev, const State& curr) const;
    double compute_service_rate() const;
    double compute_queue_wait_p95() const;
    double compute_queue_wait_km_p95() const;
    double compute_queue_wait_rmst(int horizon) const;
    double compute_queue_wait_survival(int threshold) const;
    double compute_observed_service_rate() const;
    int compute_active_queue_agents() const;
    int compute_queue_observation_count() const;
    double compute_mean_plan_ms() const;
    double compute_plan_runtime_p95_ms() const;
    double compute_plan_runtime_max_ms() const;
    double compute_plan_runtime_slope() const;
};
