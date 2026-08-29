# Unified Pressure-Aware PBS and PIBT

This RHCR extension studies lifelong MAPF with serviced workstation queues. It implements one nested policy ladder in PBS and PIBT:

1. `vanilla`: solver-native priorities and path/action preferences.
2. `departure_aware`: `TO_EXIT` agents form a globally protected class; native ordering is preserved within each class.
3. `pressure_aware`: Departure-Aware plus a soft occupancy preference for pressured workstation queues.

Departure-Aware was selected after a paired development ablation showed that
additional `SERVICE` priority was redundant. Mandatory service dwell already
fixes every servicing agent at its workstation, so Departure-Aware and
Pressure-Aware protect only departures.

PIBT is an empirical scaling method, not a complete MAPF solver. Pressure changes cost or ordering only; it never removes a collision-free action.

All three methods run in the same workstation task environment. Each task sends
an agent to a pickup, then to its assigned workstation, holds it there for the
configured service dwell, and requires it to leave through a station exit
before beginning the next task. This shared task model is separate from the
Departure-Aware and Pressure-Aware solver modifications.

Service dwell is encoded as consecutive co-located workstation goals. A dwell
of `tau` therefore produces exactly `tau` wait actions after arrival, followed
by the selected exit goal. The SIPP adapter adds a same-interval wait successor
only for this repeated-goal construct. Ordinary RHCR goals keep the published
SIPP behavior. Mandatory dwell is handled before method-specific ordering:
PIBT preassigns each forced service wait, and PBS explores the branch that
preserves a projected service wait first while retaining both conflict
branches. This shared handling applies to every policy.

## Build

```bash
sudo apt install libboost-all-dev
cmake .
cmake --build . --target lifelong -j2
```

## Shared Policy

A station queue region `Q_s` contains its approach, buffer, standby, workstation, and exit cells. A station is pressured when at least three agents occupy `Q_s`. Every physical occupant counts, regardless of its assigned station or task phase.

`TO_EXIT` agents form the Departure-Aware protected class. `SERVICE` agents
remain fixed by mandatory dwell without receiving solver priority. Agents
assigned to station `s` in `TO_STATION` are ranked by:

```text
(currently inside the target queue first,
 distance to workstation,
 station-leg issue time,
 agent ID)
```

The first four inbound agents are privileged (`K=4`). When pressure is active, every other inbound agent assigned to that station receives a soft occupancy cost of `2` for entering or remaining in `Q_s`. Agents assigned elsewhere can activate pressure by occupying the queue, but do not receive this cost.

At every projected step, both solvers evaluate pressure and privilege from the joint state at time `t`, then score candidate occupancy at time `t+1`. PBS applies the shared cost through an optional low-level transition-cost hook. PIBT applies it through one-step candidate ranking. Only executed task metadata persists between replans. The cost changes preference only and never removes a collision-free path or action. There is no adaptive `K`, foreign-agent cost, progress cost, exit bonus, ready-slot priority, separate pressure forecast, hard pruning, or pressure-specific fallback.

## Paper Runs

The paper-facing `pibt_vanilla`, `pibt_departure_aware`, and
`pibt_pressure_aware` methods use the canonical `Kei18/pibt2` core through
`--solver PIBT2`. The previous in-tree implementation remains available for
validation as `pibt_legacy_vanilla`, `pibt_legacy_departure_aware`, and
`pibt_legacy_pressure_aware`.

PIBT2 retains seeded random candidate tie-breaking. Candidate shuffles are
keyed by the simulation seed and absolute destination timestep so discarded
RHCR lookahead neither restarts nor advances randomness for later committed
steps.

Paper runs use 1,000 requested simulation steps, `w=20`, `h=5`, `tau=3`, paired seeds, and the public RHCR failure-time LRA fallback:

```bash
./lifelong \
  --scenario WORKSTATION \
  --benchmark benchmarks/alley.json \
  --solver PIBT2 \
  --pibt_policy pressure_aware \
  --native_failures_only false \
  --commitment_repair false \
  --agentNum 80 \
  --simulation_time 1000 \
  --simulation_window 5 \
  --planning_window 20 \
  --service_time 3 \
  --seed 6 \
  --output results/example
```

The publication workflow fixes `theta=3`, `K=4`, and `lambda=2` in the binary. First pretest and freeze the count ladders:

```bash
python3 scripts/run_publication.py \
  --config configs/publication_experiments.example.json \
  --root results/publication_reset \
  --stage pretest \
  --pretest-conditions alley,plaza

python3 scripts/run_publication.py \
  --config results/publication_reset/pretested_experiment_config.json \
  --root results/publication_reset \
  --stage discover
```

The human stage can use `pretested_experiment_config.json` after the Alley and
Plaza pretest. Run the Sortation pretest and `discover` before the Sortation
stage. `discover` writes `results/publication_reset/frozen_experiment_config.json`.
The count values in the example configuration are scaffolding, not paper
ladders. Sortation final ladders contain eight equally spaced reportable counts
plus the first all-method terminal or a separately marked physical-capacity
terminal.

Human-centric development uses seeds `1-25`; confirmatory runs use untouched seeds `26-45`. Final sortation runs use pickup-layout seeds `2,3,4`, simulation seeds `6-10`, and pickup retention `5%,20%,100%` on Sortation Small, Medium, and Large. Generated sidecars are named `sortation_<size>_p<density>_layout<seed>.json`.

## Failure And Metrics

After each execution window, a run stops for a traffic jam when more than half the agents remained at the same location and orientation for that entire window and no service completed during the window. Requiring both signals avoids classifying intentional admission control as a jam while still detecting a true no-service fixed point. Solver failure, invalid LRA output, traffic jam, collision, and physical capacity are separate outcomes.

Paper runs pass a valid path prefix from a failed PBS or PIBT2 episode to the
same RHCR LRA repair. The repair inserts waits for vertex contention and makes
both agents wait on a proposed edge swap. A valid LRA slice counts as a
recovered planning episode. An invalid repaired slice is a fallback failure,
while a valid execution window with majority waiting and no completed service
is a traffic jam.
Post-solve commitment repair is disabled, so successful native plans are not
modified externally. `--no-lra-fallback` remains available for native-failure
ablations.

PIBT's recursive backtracking to wait is native solver behavior, not an LRA
fallback. It is reported through backtracking diagnostics; LRA episodes and
inserted LRA wait commands are reported separately.

Each run reports:

- clean completion, time to stall, and horizon-normalized service yield;
- queue-wait `RMST100`, survival at 20/50/100 steps, capped KM p95, queue-region occupancy, and active queue agents;
- observed-time and requested-horizon service rates, plus distance per completed service;
- mean/p95/max runtime per replan and amortized per executed step, runtime slope, and peak RSS;
- PIBT inheritance, backtracking, wait-repair, and pressure-rank-change diagnostics.
- LRA fallback episodes and inserted wait commands when external fallback is enabled.

Aggregate with:

```bash
python3 scripts/aggregate_results.py --root results/publication_reset/final
```

`hierarchical_effects.csv` contains paired confidence intervals that resample pickup-layout seed first and simulation seed second. Existing result roots produced by earlier pressure variants are exploratory provenance and must not be mixed with finalized runs.

## References

- [RHCR](https://arxiv.org/abs/2005.07371)
- [PIBT](https://arxiv.org/abs/1901.11282)
- [PIBT preference construction](https://arxiv.org/abs/2505.12623)
- [LoRR Benchmark Archive](https://github.com/MAPF-Competition/Benchmark-Archive)

## License

This repository follows the RHCR license in `license.md`.
The adapted `Kei18/pibt2` core is MIT licensed. Its pinned source attribution
and license are in `third_party/pibt2/`.
