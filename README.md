# Pressure-Aware Workstation Queue Control

This branch is the source snapshot for the AAMAS paper
**Pressure-Aware Workstation Queue Control for Lifelong Multi-Agent Path
Finding**. It extends RHCR with serviced workstation tasks and one nested policy
ladder implemented in PBS and PIBT.

The branch is intentionally publication-focused. Historical experiments and
generated result directories are not versioned here.

## Policy Ladder

All methods use the same task phases, workstation geometry, motion constraints,
service dwell, and collision checks.

1. **Vanilla:** solver-native priority and shortest-path preference, with no
   workstation-specific term.
2. **Phase-Aware:** process SERVICE and TO_EXIT agents before unprotected agents
   while preserving native order within each class.
3. **Pressure-Aware:** retain Phase-Aware precedence and add workstation-local
   soft admission and progress preferences.

The workstation terms change candidate cost or exploration order. They do not
remove an action or branch; native motion, task, vertex-conflict, and edge-swap
constraints remain binding.

## Frozen Pressure Profile

Paper-facing runs use one configuration on every map and solver:

    pressure_threshold=2
    pressure_admission=adaptive
    pressure_inbound_limit=4
    pressure_cost_occupancy_threshold=2
    pressure_population=all_phases
    pressure_zone_cost=2
    pressure_front_progress_cost=3
    pressure_exit_progress_cost=1
    pressure_ready_slot_priority=true
    pressure_cost_scope=zone
    pressure_cost_activation=zone
    pressure_profile=fixed
    pressure_lookahead_radius=0

At a pressured station, inbound agents are ordered by:

    (boundary crossed on the current station leg first,
     distance to the service cell,
     station-leg issue time,
     agent ID)

The privileged prefix starts at four agents and tightens to three when physical
zone occupancy reaches one third, then to two at two thirds. Other inbound
agents receive a soft cost while occupying the pressured handoff zone. SERVICE
and TO_EXIT agents remain Phase-Aware protected independently of the prefix.

PBS adds the workstation cost to low-level path planning and uses protected or
ready-admission-head status only to choose which valid conflict branch to
explore first. PIBT adds the same terms to one-step candidate ranking and
assignment order; native inheritance and backtracking are unchanged.

## Failure Semantics

Paper runs pass --native_failures_only 1.

- PBS terminates on native planning failure or the 60-second replan cutoff.
  LRA* fallback and external commitment repair are disabled.
- Every PIBT policy uses the same bounded recursive assignment, one shared
  budget extension, deterministic greedy repair, and constraint-valid wait
  fallback. A run terminates if no valid joint step remains.
- The extension does not claim PIBT completeness because workstation phases,
  finite assignment budgets, and repair behavior differ from the assumptions
  of the original completeness result.

## Build And Test

Requirements include a C++11 compiler, CMake, and Boost program options,
filesystem, and system libraries.

    cmake -S . -B build
    cmake --build build --target lifelong workstation_policy_tests -j2
    ctest --test-dir build --output-on-failure
    python3 -m unittest tests.test_result_tools

The C++ tests cover privilege selection, Phase-Aware ordering, pressure
preference without pruning, dynamic-age execution semantics, phase projection,
vertex and edge-swap validation, rollback, and deterministic repair. The Python
tests cover result aggregation, Kaplan--Meier queue summaries, RMST, failure
detection, and paired reporting.

## Smoke Run

The following command exercises the frozen Pressure-Aware PIBT profile:

    ./build/lifelong \
      --scenario WORKSTATION \
      --benchmark benchmarks/alley.json \
      --solver PIBT \
      --pibt_policy pressure_aware \
      --agentNum 20 \
      --simulation_time 100 \
      --simulation_window 5 \
      --planning_window 20 \
      --service_time 3 \
      --seed 6 \
      --screen 0 \
      --output /tmp/pressure-aware-pibt-smoke \
      --native_failures_only 1 \
      --pressure_threshold 2 \
      --pressure_admission adaptive \
      --pressure_inbound_limit 4 \
      --pressure_cost_occupancy_threshold 2 \
      --pressure_population all_phases \
      --pressure_zone_cost 2 \
      --pressure_front_progress_cost 3 \
      --pressure_exit_progress_cost 1 \
      --pressure_ready_slot_priority 1

For PBS, replace --solver PIBT --pibt_policy pressure_aware with
--solver PBS --station_policy pressure_aware.

## Evaluation Protocol

The paper requests 1,000 simulation steps with planning window w=20, execution
window h=5, and service dwell tau=3.

- Alley and Plaza use pickup-layout seed 1 and simulation seeds 6--25.
- Sortation Small, Medium, and Large cross pickup-layout seeds 2--4 with
  simulation seeds 6--10.
- Sortation uses 5%, 20%, and 100% retained pickup subsets and eight equally
  spaced reportable counts per size-retention setting.
- Policies use identical counts and paired seeds.

The LoRR-derived JSON sidecars under benchmarks/lorr preserve benchmark obstacle
geometry but add serviced workstation zones and non-native task semantics.
scripts/run_publication.py, scripts/run_publication_extensions.py, and
scripts/validate_publication_extensions.py record and validate the complete
configuration.

Generated result roots and archived executables are excluded from Git because
of their size. The paper workspace records their paths and fingerprints.

## Snapshot Validation

This branch was built in a clean worktree and checked with:

- the C++ workstation-policy test target;
- all 26 Python result-tool tests;
- all six PBS/PIBT policy combinations on Alley; and
- a 1,024-agent, 1,000-step Sortation Small P20 trajectory comparison.

The clean build and archived publication executable produced byte-identical
paths for that endpoint and identical values in every common non-runtime
summary field. Runtime values are not expected to be byte-identical across
separate builds.

## License

The upstream RHCR license is retained in [license.md](license.md).
