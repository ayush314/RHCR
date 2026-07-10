#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import os
from collections import deque
from pathlib import Path


def read_movingai_map(path: Path) -> tuple[int, int, list[str]]:
    lines = path.read_text().splitlines()
    if len(lines) < 5 or lines[3] != "map":
        raise ValueError(f"Invalid MovingAI map header: {path}")
    rows = int(lines[1].split()[1])
    cols = int(lines[2].split()[1])
    grid = lines[4:]
    if len(grid) != rows or any(len(row) != cols for row in grid):
        raise ValueError(f"Invalid MovingAI map dimensions: {path}")
    return rows, cols, grid


def traversable(cell: str) -> bool:
    return cell not in "@OTW"


def graph_layers(grid: list[str], start: tuple[int, int], radius: int) -> dict[int, list[tuple[int, int]]]:
    rows = len(grid)
    cols = len(grid[0])
    distances = {start: 0}
    queue = deque([start])
    while queue:
        x, y = queue.popleft()
        distance = distances[(x, y)]
        if distance == radius:
            continue
        for dx, dy in ((1, 0), (0, -1), (-1, 0), (0, 1)):
            nxt = (x + dx, y + dy)
            nx, ny = nxt
            if not (0 <= nx < cols and 0 <= ny < rows):
                continue
            if nxt in distances or not traversable(grid[ny][nx]):
                continue
            distances[nxt] = distance + 1
            queue.append(nxt)
    return {
        layer: sorted(cell for cell, distance in distances.items() if distance == layer)
        for layer in range(1, radius + 1)
    }


def select_spaced(candidates: list[tuple[int, int]], count: int) -> list[tuple[int, int]]:
    if count < 1 or count > len(candidates):
        raise ValueError(f"station count must be between 1 and {len(candidates)}")
    selected = [min(candidates)]
    remaining = set(candidates) - set(selected)
    while len(selected) < count:
        nxt = max(
            remaining,
            key=lambda cell: (
                min(abs(cell[0] - other[0]) + abs(cell[1] - other[1]) for other in selected),
                -cell[1],
                -cell[0],
            ),
        )
        selected.append(nxt)
        remaining.remove(nxt)
    return sorted(selected, key=lambda cell: (cell[1], cell[0]))


def build_benchmark(source: Path, station_count: int) -> dict:
    rows, cols, grid = read_movingai_map(source)
    pickups = [
        (x, y)
        for y, row in enumerate(grid)
        for x, cell in enumerate(row)
        if cell == "S"
    ]
    emitters = [
        (x, y)
        for y, row in enumerate(grid)
        for x, cell in enumerate(row)
        if cell == "E"
    ]
    workstations = select_spaced(emitters, station_count)
    stations = []
    claimed = set(workstations)
    for station_id, workstation in enumerate(workstations):
        layers = graph_layers(grid, workstation, 3)
        station_layers = {}
        for layer in range(1, 4):
            cells = [cell for cell in layers[layer] if cell not in claimed and cell not in pickups]
            station_layers[layer] = cells
            claimed.update(cells)
        exits = station_layers[1]
        if not exits:
            raise ValueError(f"Workstation {workstation} has no unclaimed exit cell")
        stations.append({
            "station_id": station_id,
            "workstation_cell": list(workstation),
            "standby_cells": [list(cell) for cell in station_layers[1]],
            "buffer_cells": [list(cell) for cell in station_layers[2]],
            "approach_cells": [list(cell) for cell in station_layers[3]],
            "exit_cells": [list(cell) for cell in exits],
        })

    return {
        "benchmark_id": f"lorr_{source.stem}",
        "description": "LoRR warehouse map adapted to alternating storage-pickup and serviced-emitter tasks.",
        "source": "https://github.com/MAPF-Competition/Benchmark-Archive/tree/main/2023%20Competition/Example%20Instances/warehouse.domain",
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "adapter_station_count": station_count,
        "rows": rows,
        "cols": cols,
        "movingai_map": source.name,
        "simulation_time_default": 2000,
        "planning_window_default": 20,
        "simulation_window_default": 5,
        "seed_start_default": 1,
        "seed_count_default": 20,
        "service_time_default": 5,
        "task_model": "endpoint_station_alternating",
        "restrict_station_zones": False,
        "pickup_endpoints": [list(cell) for cell in pickups],
        "stations": stations,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Adapt a LoRR E/S MovingAI map to the workstation benchmark schema.")
    parser.add_argument("--map", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--station-count", type=int, default=12)
    args = parser.parse_args()

    benchmark = build_benchmark(args.map, args.station_count)
    benchmark["movingai_map"] = os.path.relpath(args.map.resolve(), args.output.resolve().parent)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(benchmark, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
