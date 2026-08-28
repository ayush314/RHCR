# pibt2 attribution

`src/PIBT2.cpp` adapts the one-step PIBT configuration-generation core from
[`Kei18/pibt2`](https://github.com/Kei18/pibt2) at commit
`faab5b916649549f1cd563df8dbf6e4f6382f631`.

The upstream code accompanies:

Keisuke Okumura, Manao Machida, Xavier Defago, and Yasumasa Tamura. "Priority
Inheritance with Backtracking for Iterative Multi-agent Path Finding."
Artificial Intelligence 310 (2022), 103752.

The adapter retains upstream vanilla PIBT dynamic priority, fixed initial-goal
distance, random tie-breaking, distance-and-vacancy candidate ordering,
recursive priority inheritance, and native backtracking to wait. The fixed
initial distance is renewed only when a workstation goal leg changes. RHCR
windowing, committed service dwell, and optional workstation-policy ordering
are adapter functionality shared or gated outside the vanilla core. See
`LICENSE.txt` in this directory for the upstream MIT license.
