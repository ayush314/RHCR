# Pre-Plaza Throughput Method Snapshot

## Purpose

This branch preserves the three workstation policies immediately before the
Plaza-throughput prototypes. The Pressure-Aware PIBT implementation is the
single-lead, per-queue-occupancy version used for the last P20 Sortation Small,
Medium, and Large evaluation.

Later prototypes are intentionally absent:

- K-wide PIBT right-of-way
- entry-only PIBT pressure cost
- alternative pressure thresholds
- alternative K values
- Plaza-specific throughput mechanisms

## Provenance

- Archive branch: archive/sortation-p20-pre-plaza-throughput
- Base Git commit: 33ba5489e35ab66ef496a418f961c787477de863
- RHCR upstream: Jiaoyang-Li/RHCR
- RHCR upstream commit: d009a3bd716419b0d6c04aead9dbca1720c012da
- PIBT upstream: Kei18/pibt2
- PIBT upstream commit: faab5b916649549f1cd563df8dbf6e4f6382f631
- Original Sortation result root:
  results/final_sortation_p20_pressure_lead_right_of_way
- Original Sortation binary SHA-256:
  e94e954bb3b5796902735dc0f001f87b44f60fb7d374926109904f58b0996f30

The historical run recorded a dirty source tree and the original executable has
since been rebuilt. This archive is a behavioral reconstruction from the
recorded run manifests, representative config.txt, and the aligned method
source. The original manifests are under
provenance/sortation_p20_pressure_lead_right_of_way/.

## Shared Workstation Framework

Every method uses the same task state machine:

1. TO_PICKUP: travel to the assigned pickup cell.
2. TO_STATION: travel from pickup to the assigned workstation.
3. SERVICE: remain on the workstation for exactly tau steps.
4. TO_EXIT: move from the workstation to its selected adjacent exit cell.
5. Start the next pickup leg only after reaching that exit.

Mandatory service dwell is a framework constraint, not a method-specific
priority. A servicing agent is forced to wait on the workstation until its
dwell ends.

Exit clearance is also shared by all three conditions. TO_EXIT agents are
processed ahead of agents in other movable phases. This keeps the required
service-to-exit transition consistent across Vanilla, Lead-Aware, and
Pressure-Aware.

Collision feasibility is unchanged across policies. No lead or pressure rule
removes a collision-free branch or action.

## Shared Lead Definition

For every workstation s, inbound agents assigned to s are ordered by

    (already inside Q_s first,
     distance to workstation,
     station-leg issue time,
     agent ID)

The first agent is the station lead.

## Vanilla PBS

Vanilla PBS retains the published PBS search structure:

- A high-level node stores a partial priority ordering.
- A detected conflict produces both possible priority branches.
- Lower-priority agents are replanned against paths of reachable
  higher-priority agents.
- Native child path cost and remaining collision count determine search order.

The workstation framework adds only mandatory service dwell and shared TO_EXIT
clearance. Vanilla PBS has no station-lead preference and no pressure cost.

## Lead-Aware PBS

Lead-Aware PBS starts from Vanilla PBS.

When a conflict touches a station's queue region and exactly one conflicting
agent is that station's lead, the lead-favoring priority branch is used only as
a tie-break after native child path cost and collision count. Both priority
branches remain available.

Lead-Aware does not define pressure, privileged inbound agents, or queue costs.

## Pressure-Aware PBS

Pressure-Aware PBS starts from Lead-Aware PBS and uses fixed parameters:

    theta = 3
    K = 2
    lambda = 2

For station s, pressure is

    O_s = number of all agents physically occupying Q_s
    P_s = [O_s >= theta]

Q_s includes the mapped queue cells, workstation, and exit cells. Occupancy
counts every agent in those cells, including agents assigned to another
station.

When P_s is active, the first K=2 inbound agents under the shared key are
privileged and pay no pressure cost. Other inbound agents assigned to s
receive cost lambda=2 for every planned transition whose destination occupies
Q_s.

PBS evaluates projected pressure at each low-level planning step using the
other agents' current paths. The cost enters low-level A* path ranking. It does
not change path feasibility.

Only the first privileged inbound agent, the station lead, receives
pressure-time right-of-way. For a conflict touching Q_s, its preferred priority
branch is generated and explored first. The other branch remains in the
search. The second privileged inbound agent is cost-exempt but receives no
extra branch priority.

## Vanilla PIBT

The PIBT2 solver is adapted from Kei18/pibt2.

At each planned step it:

1. Forces mandatory service-dwell waits.
2. Processes shared TO_EXIT agents before other movable agents.
3. Orders the remaining agents by descending dynamic age.
4. Breaks an age tie by descending initial distance for the current goal leg.
5. Uses the seeded native tie-breaker.
6. Orders actions by shortest-path distance, preferring currently unoccupied
   destinations when distances tie.
7. Uses priority inheritance when the desired cell is occupied.
8. Uses native PIBT backtracking and a wait at the current cell when recursive
   assignment cannot proceed.

Dynamic age is persisted only for simulator-executed steps. Planning-window
projection uses a temporary copy.

## Lead-Aware PIBT

Lead-Aware PIBT starts from Vanilla PIBT.

A station lead whose current state or one-step action touches its target queue
wins only the final local ordering tie after dynamic age and initial distance,
before the seeded native tie-breaker. It does not outrank an older native PIBT
agent.

Candidate action ranking remains shortest-path based. No pressure cost is used.

## Pressure-Aware PIBT Used On Sortation

Pressure-Aware PIBT starts from Lead-Aware PIBT and uses the same theta=3,
K=2, lambda=2, queue occupancy, and privilege key as PBS.

At each projected step:

1. Count all agents occupying each Q_s.
2. If O_s >= 3, select the first two inbound agents as cost-exempt.
3. For each other inbound agent assigned to s, score a candidate action as

       shortest-path distance after the action
       + 2 if the candidate destination occupies Q_s

The cost is charged for every candidate occupancy of the pressured queue,
including continued movement or waiting inside it. It is not an entry-only
cost.

Only the first privileged inbound agent gets pressure-time right-of-way near
the queue. That lead is processed before native dynamic age. The second
privileged agent is cost-exempt but receives no right-of-way. Other action
feasibility, inheritance, and backtracking are unchanged.

## Failure And Fallback Behavior

PBS and PIBT first run their native solver with the 60-second per-replan cutoff
used by the experiment.

If the native solver fails but returns a valid path prefix for every agent, the
shared RHCR layer invokes LRA*. The archived modification detects undirected
edge swaps and commands both members of a proposed swap to wait. LRA* wait
commands are external fallback behavior and are recorded separately.

PIBT's recursive backtracking wait is native PIBT behavior. It is not the LRA*
fallback.

If there is no valid native prefix, or the fallback produces an invalid
execution slice, the run fails. The Sortation driver also used a 600-second
outer process limit.

## Sortation Configuration

The archived Sortation evaluation used:

    PIBT2
    Pressure-Aware
    1,000 simulation steps
    planning window w = 20
    execution window h = 5
    service time tau = 3
    theta = 3
    K = 2
    lambda = 2
    pickup retention = 20%
    pickup-layout seeds = 2, 3, 4
    simulation seeds = 6, 7, 8, 9, 10

The runnable configuration is
configs/sortation_p20_pibt_pressure_right_of_way.json.
