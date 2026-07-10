# Pressure-Aware PBS and PIBT for Lifelong MAPF with Workstations

This project studies pressure-aware Priority-Based Search (PBS) and Priority Inheritance with Backtracking (PIBT) for lifelong Multi-Agent Path Finding (MAPF) with workstations, built on Rolling-Horizon Collision Resolution (RHCR).

The code requires the external library BOOST (https://www.boost.org/).
On Ubuntu, you can install it with:

```bash
sudo apt install libboost-all-dev
```

After you install BOOST and download the source code, compile it with CMake:

```bash
cmake .
make
```

For example:

```bash
./lifelong \
  --scenario WORKSTATION \
  --benchmark benchmarks/alley.json \
  --solver PBS \
  --station_policy pressure_aware \
  --agentNum 20 \
  --simulation_time 5000 \
  --simulation_window 5 \
  --planning_window 20 \
  --service_time 3 \
  --seed 1
```

This command runs RHCR with PBS and the pressure-aware policy on the `alley` workstation benchmark.

The scalable PIBT comparison uses:

```bash
./lifelong \
  --scenario WORKSTATION \
  --benchmark benchmarks/lorr/warehouse_small.json \
  --solver PIBT \
  --pibt_policy pressure \
  --agentNum 783 \
  --simulation_time 500 \
  --simulation_window 5 \
  --planning_window 20 \
  --service_time 3 \
  --seed 1
```

Pressure PIBT is a preference-construction extension of a shared PIBT core. The three reportable methods are `vanilla`, `distance_age`, and `pressure`. They use the same recursive one-step assignment, dynamic priority ages, workstation simulator, and seeded random final preference tie. PIBT is scalable but not a complete MAPF solver; `clean` results here are empirical horizon completions, not a completeness claim.

Pressure PIBT keeps the dynamic priority order instead of globally reordering service and exit phases. Its one ordering intervention boosts the primary front runner at each pressured station, chosen by boundary-entry time, distance, task-issue time, and agent ID.

Candidate actions are ranked by remaining-goal distance plus pressure-local terms for pressured-zone entry, front-runner progress, exit progress, and waiting. A lightweight hindrance value, based on the preference construction of Okumura et al., breaks ties only during inherited PIBT calls. Pressure changes candidate order but never removes an otherwise valid action; vertex and edge-swap constraints remain hard constraints.

Each pressured station normally admits up to four target-bound agents. The `thirds` profile contracts that limit from four to three at one-third non-service-zone occupancy and from three to two at two-thirds occupancy. Entering above that soft limit adds a preference cost of `2` by default; it does not prune the move. This balanced setting preserves the high-density throughput gain while preventing the long queues produced by the more aggressive cost of `1`. If recursive assignment cannot produce a valid move, the implementation attempts a deterministic wait repair and records it in `pibt_wait_fallbacks`. This is a safety repair after assignment fails, not PIBT's normal wait action or an additional planner.

The LoRR transfer uses the official 2023 `warehouse_small.map` evaluation map and `sortation_small.map` problem-generator map. The adapter treats LoRR `S` cells as storage pickups and a deterministic spaced subset of `E` cells as serviced workstations. The checked-in warehouse sidecar has 342 pickups, 12 workstations, and 783 valid start cells; the sortation sidecar has 517 pickups, 12 workstations, and 870 valid start cells. Sortation uses the same pressure parameters fixed on the earlier maps, with no map-specific retuning. Regenerate the sidecars with:

```bash
python3 scripts/import_lorr_workstation.py \
  --map benchmarks/lorr/warehouse_small.map \
  --output benchmarks/lorr/warehouse_small.json \
  --station-count 12

python3 scripts/import_lorr_workstation.py \
  --map benchmarks/lorr/sortation_small.map \
  --output benchmarks/lorr/sortation_small.json \
  --station-count 12 \
  --source-url 'https://github.com/MAPF-Competition/Benchmark-Archive/tree/main/2023%20Competition/Problem%20Generator/script/sortation_small.map' \
  --description 'LoRR sortation map adapted to alternating storage-pickup and serviced-emitter tasks.'
```

The main arguments are:

- `scenario`: the simulation scenario. Here it is `WORKSTATION`.
- `benchmark`: a workstation benchmark JSON (`alley`, `plaza`, or either adapted LoRR map).
- `solver`: the windowed MAPF solver (`PBS` or `PIBT`).
- `station_policy`: the workstation-local PBS policy (`vanilla`, `distance_age`, or `pressure_aware`).
- `pibt_policy`: the PIBT policy (`vanilla`, `distance_age`, or `pressure`).
- `pibt_pressure_entry_penalty`: soft action-ranking cost for entering a pressured zone above its admission limit (default `2`).
- `pibt_pressure_inbound_limit`: the normal target-bound admission limit for a pressured station zone (default `4`).
- `pibt_pressure_profile`: occupancy-based admission rule (default `thirds`).
- `pibt_hindrance`: enable the lightweight hindrance tiebreaker (default `true`).
- `pibt_hindrance_scope`: where hindrance is applied (default `inherited`).
- `pibt_front_priority`: boost the primary station front runner (default `true`).
- `pibt_phase_priority`: globally boost service and exit phases (default `false`).
- `pibt_regret_iterations`: preference-learning passes from Okumura et al.; `1` disables this optional ablation (default `1`).
- `agentNum`: the number of agents.
- `simulation_time`: the simulation horizon.
- `simulation_window`: the replanning period `h`.
- `planning_window`: the planning window `w`.
- `service_time`: the workstation service time `tau`.
- `seed`: the random seed.

For the full list of parameters:

```bash
./lifelong --help
```

To run the current PIBT comparison:

```bash
python3 scripts/run_comparison.py \
  --root results/pibt_primary_random_preference_h5_seed1to20 \
  --methods pibt_vanilla,pibt_distance_age,pibt_pressure \
  --seed-start 1 \
  --seed-count 20 \
  --simulation-time 500 \
  --alley-counts 20,48,76,104,131,158 \
  --plaza-counts 40,90,140,190,240,288
```

To run the six-point LoRR capacity curve:

```bash
python3 scripts/run_comparison.py \
  --root results/pibt_lorr_warehouse_small_tau3_h5_seed1to20 \
  --methods pibt_vanilla,pibt_distance_age,pibt_pressure \
  --seed-start 1 \
  --seed-count 20 \
  --simulation-time 500 \
  --service-time 3 \
  --alley-counts '' \
  --plaza-counts '' \
  --lorr-counts 50,197,343,490,636,783 \
  --continue-on-traffic-jam
```

To run the six-point LoRR sortation curve:

```bash
python3 scripts/run_comparison.py \
  --root results/pibt_lorr_sortation_small_tau3_h5_seed1to20 \
  --methods pibt_vanilla,pibt_distance_age,pibt_pressure \
  --seed-start 1 \
  --seed-count 20 \
  --simulation-time 500 \
  --service-time 3 \
  --alley-counts '' \
  --plaza-counts '' \
  --lorr-sortation-counts 50,214,378,542,706,870 \
  --continue-on-traffic-jam
```

To aggregate the results:

```bash
python3 scripts/aggregate_results.py \
  --root results/pibt_primary_random_preference_h5_seed1to20
```

Each run writes `summary.csv` and per-replan `planning_runtime.csv`. The summary includes completed-wait `queue_wait_p95`, Kaplan-Meier right-censor-aware `queue_wait_km_p95`, the number of still-waiting agents at the horizon in `active_queue_agents`, and mean/p95/max planning time so runtime tails remain visible. Load diagnostics distinguish the legacy any-station `pressure_active_fraction` from normalized `pressured_station_fraction` and `mean_zone_occupancy_fraction`. Aggregation derives `pibt_wait_fallback_rate_per_1000_agent_steps` so fallback use is comparable across horizons and agent counts, then produces `combined_summary.csv`, `aggregate.csv`, and same-seed pressure-vs-baseline effects with 95% confidence intervals in `paired_comparison.csv`. For a one-factor ablation stored in separate roots, pass `--paired-baseline-root` to write a same-seed `paired_root_comparison.csv` with clean-seed accounting. Batch manifests fingerprint the binary and benchmark inputs, status files fingerprint each run configuration, and aggregation admits only cells selected by the current manifest. Batch runs also remove large `paths.txt` trajectories after clean runs by default; pass `--keep-paths` only when a trajectory-level diagnosis needs them. The automated candidate search is in `scripts/auto_research_pibt.py`; rejected configurations and leaderboards belong under `results/_archive` rather than in the primary comparison.

References:

- Okumura et al., [Priority Inheritance with Backtracking for Iterative Multi-agent Path Finding](https://arxiv.org/abs/1901.11282).
- Okumura et al., [Lightweight and Effective Preference Construction in PIBT for Iterative MAPF](https://arxiv.org/abs/2505.12623).
- [League of Robot Runners 2023 Benchmark Archive](https://github.com/MAPF-Competition/Benchmark-Archive/tree/main/2023%20Competition).

## License

This repository follows the RHCR license. See `license.md` for further details.
