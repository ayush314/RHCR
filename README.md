# Pressure-Aware PBS for Lifelong MAPF with Workstations

Pressure-Aware PBS is a Priority-Based Search method for lifelong Multi-Agent Path Finding (MAPF) with workstations, built on Rolling-Horizon Collision Resolution (RHCR).

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
  --seed 0
```

This command runs RHCR with PBS and the pressure-aware policy on the `alley` workstation benchmark.

The main arguments are:

- `scenario`: the simulation scenario. Here it is `WORKSTATION`.
- `benchmark`: the workstation benchmark JSON (`benchmarks/alley.json` or `benchmarks/plaza.json`).
- `solver`: the windowed MAPF solver. Here it is `PBS`.
- `station_policy`: the workstation-local PBS policy (`vanilla`, `distance_age`, or `pressure_aware`).
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

To run the canonical comparison:

```bash
python3 scripts/run_comparison.py
```

To aggregate the results:

```bash
python3 scripts/aggregate_results.py
```

## License

This repository follows the RHCR license. See `license.md` for further details.
