# Paper Overview

## Working Title

**Pressure-Aware Workstation Queue Control for Lifelong Multi-Agent Path Finding**

## Claim

A shared task-local workstation policy can improve PBS reliability at its
congestion boundary and reduce PIBT queue delay at much larger scales. The
policy transfers across solver families as a soft preference; it does not
remove a collision-free action or imply identical trajectories.

## Method Ladder

### Vanilla

- PBS keeps standard priority-tree branching and low-level replanning.
- PIBT keeps dynamic priority, shortest-path action preference, priority
  inheritance, and backtracking.

### Departure-Aware

`TO_EXIT` agents form a protected class. PBS explores the
protected-agent-first branch in a mixed conflict. PIBT assigns protected agents
before unprotected agents. Each solver retains its native ordering within each
class. Mandatory service dwell remains a framework constraint rather than a
solver-priority rule. PIBT preassigns forced service waits before policy
ordering. PBS orders a conflict involving projected service dwell so that the
dwell-preserving branch is explored first, but retains both branches. This
framework handling is identical for all methods.

Paired development experiments found no benefit from additionally prioritizing
`SERVICE` agents, whose actions are already fixed by mandatory dwell.

### Pressure-Aware

Pressure-Aware inherits Departure-Aware and adds workstation-local queue control. For station `s`, `Q_s` contains its approach, buffer, standby, workstation, and exit cells.

1. Pressure activates at `P_s >= 3`, where `P_s` counts every agent physically
   occupying `Q_s`, regardless of assignment or task phase.
2. Inbound agents are ordered by `(boundary seen, distance to service,
   station-leg issue time, agent ID)`.
3. The first `K=2` assigned `TO_STATION` agents are exempt from the pressure
   cost. They receive no priority over `TO_EXIT` agents.
4. Each nonprivileged assigned `TO_STATION` agent receives cost `lambda=2` for
   occupying `Q_s` while pressure is active. Foreign agents count toward
   pressure but do not receive this cost.

At each projected step, both solvers compute pressure and privilege from the
joint state at time `t`, then score occupancy at time `t+1`. PBS applies the
cost through its low-level transition cost and PIBT applies it through
one-step candidate ranking. Both retain the Departure-Aware protected
ordering. Pressure changes preference only. Vertex conflicts and edge swaps
remain policy-independent feasibility exclusions.

## Failure Boundary

- Paper-facing PBS and PIBT runs use the shared RHCR LRA* fallback after a
  native planning failure. Commitment repair remains disabled.
- Every PIBT policy uses the same bounded recursive assignment and
  collision-free wait attempt. Failure of either step ends native PIBT.
- LRA* inserts waits for vertex contention and makes both agents wait on a
  proposed undirected edge swap. The shared execution validator rejects an
  invalid repaired slice.
- No completeness or real-time claim is made.

## Evaluation

All reported runs request 1,000 steps with `w=20`, `h=5`, and `tau=3`.

### Compact Solver Transfer

- Alley and Plaza; PBS and PIBT; Vanilla, Departure-Aware, and Pressure-Aware.
- Pickup seed 1 and simulation seeds 6-25: 20 paired runs per condition.
- PBS counts: Alley 20-80 by 10; Plaza 20-100 by 20.
- PIBT counts: Alley 20-80 by 10 then 100-140 by 20; Plaza 20-120 by
  20 then 160-280 by 40.

### Sortation Scaling

- LoRR-derived Sortation Small, Medium, and Large.
- Pickup retention 5%, 20%, and 100%.
- Pickup-subset seeds 2-4 and simulation seeds 6-10: 15 paired runs per point.
- Eight equally spaced reportable counts per size-retention setting.
- Physical-capacity probes are retained for provenance but excluded from
  reportable quality curves and endpoint effects.

### Service-Time Sensitivity

Fixed-parameter checks use `tau=1` and `tau=5`. The PIBT queue-delay direction
persists, but PBS loses its Alley completion advantage at `tau=5`; the main
claim is therefore limited to the frozen `tau=3` setting.

## Result Status

The service-priority development ablation is stored under
`results/departure_ablation/human_paired`. Paper-facing outcomes must be
regenerated with the selected Departure-Aware parent and the fixed all-occupant
`P_s`, `theta=3`, `K=2`, and `lambda=2` implementation.

## Paper Structure

1. Introduction
2. Related Work
3. Lifelong MAPF with Workstations
4. Unified Workstation Policy
5. Experimental Design
6. Results
7. Discussion and Limitations
8. Conclusion

## Submission-Hardening Work

- Run a confirmatory matrix on untouched seeds and geometries.
- Persist raw queue observations and add direct RMST reconstruction tests.
- Archive the completed parameter ablations separately from confirmatory runs.
- Recheck the `tau=5` PBS reversal and retain it as a stated scope boundary if
  it persists under the finalized method.
