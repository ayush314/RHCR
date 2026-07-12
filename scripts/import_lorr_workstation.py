#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import os
import random
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


def select_spaced(candidates: list[tuple[int, int]], count: int) -> list[tuple[int, int]]:
    if count < 1 or count > len(candidates):
        raise ValueError(f"station count must be between 1 and {len(candidates)}")
    selected = [min(candidates)]
    remaining = set(candidates) - set(selected)
    nearest_distance = {
        cell: abs(cell[0] - selected[0][0]) + abs(cell[1] - selected[0][1])
        for cell in remaining
    }
    while len(selected) < count:
        nxt = max(
            remaining,
            key=lambda cell: (
                nearest_distance[cell],
                -cell[1],
                -cell[0],
            ),
        )
        selected.append(nxt)
        remaining.remove(nxt)
        nearest_distance.pop(nxt)
        for cell in remaining:
            distance = abs(cell[0] - nxt[0]) + abs(cell[1] - nxt[1])
            nearest_distance[cell] = min(nearest_distance[cell], distance)
    return sorted(selected, key=lambda cell: (cell[1], cell[0]))


SIDE_ORDER = ("top", "right", "bottom", "left")
INWARD = {
    "top": (0, 1),
    "right": (-1, 0),
    "bottom": (0, -1),
    "left": (1, 0),
}
TANGENT = {
    "top": (1, 0),
    "right": (0, 1),
    "bottom": (-1, 0),
    "left": (0, -1),
}


def perimeter_side(cell: tuple[int, int], rows: int, cols: int) -> str:
    x, y = cell
    distances = {
        "top": y,
        "right": cols - 1 - x,
        "bottom": rows - 1 - y,
        "left": x,
    }
    return min(SIDE_ORDER, key=lambda side: (distances[side], SIDE_ORDER.index(side)))


def select_balanced_perimeter(
    candidates: list[tuple[int, int]],
    count: int,
    rows: int,
    cols: int,
) -> list[tuple[tuple[int, int], str]]:
    if count % len(SIDE_ORDER) != 0:
        raise ValueError("balanced perimeter station count must be divisible by four")
    per_side = count // len(SIDE_ORDER)
    grouped = {side: [] for side in SIDE_ORDER}
    for cell in candidates:
        grouped[perimeter_side(cell, rows, cols)].append(cell)

    selected_by_side: dict[str, list[tuple[int, int]]] = {}
    for side in SIDE_ORDER:
        available = set(grouped[side])
        if len(available) < per_side:
            raise ValueError(f"not enough {side} emitters for {per_side} stations")
        axis_limit = cols - 1 if side in {"top", "bottom"} else rows - 1
        axis = (lambda cell: cell[0]) if side in {"top", "bottom"} else (lambda cell: cell[1])
        selected = []
        for index in range(per_side):
            target = (index + 1) * axis_limit / (per_side + 1)
            cell = min(
                available,
                key=lambda candidate: (
                    abs(axis(candidate) - target),
                    axis(candidate),
                    candidate[1],
                    candidate[0],
                ),
            )
            selected.append(cell)
            available.remove(cell)
        selected_by_side[side] = selected

    clockwise = {
        "top": sorted(selected_by_side["top"], key=lambda cell: cell[0]),
        "right": sorted(selected_by_side["right"], key=lambda cell: cell[1]),
        "bottom": sorted(selected_by_side["bottom"], key=lambda cell: cell[0], reverse=True),
        "left": sorted(selected_by_side["left"], key=lambda cell: cell[1], reverse=True),
    }
    return [(cell, side) for side in SIDE_ORDER for cell in clockwise[side]]


def add(cell: tuple[int, int], direction: tuple[int, int], distance: int = 1) -> tuple[int, int]:
    return cell[0] + distance * direction[0], cell[1] + distance * direction[1]


def retained_pickup_count(candidate_count: int, retention_percent: int) -> int:
    if retention_percent < 1 or retention_percent > 100:
        raise ValueError("pickup retention must be between 1 and 100 percent")
    return (candidate_count * retention_percent + 50) // 100


def select_nested_random(
    candidates: list[tuple[int, int]],
    retention_percent: int,
    sample_seed: int,
) -> list[tuple[int, int]]:
    ordered = sorted(candidates, key=lambda cell: (cell[1], cell[0]))
    random.Random(sample_seed).shuffle(ordered)
    count = retained_pickup_count(len(ordered), retention_percent)
    return sorted(ordered[:count], key=lambda cell: (cell[1], cell[0]))


def ordered_perimeter_emitters(
    emitters: list[tuple[int, int]], rows: int, cols: int
) -> list[tuple[tuple[int, int], str]]:
    grouped = {side: [] for side in SIDE_ORDER}
    for cell in emitters:
        grouped[perimeter_side(cell, rows, cols)].append(cell)
    grouped["top"].sort(key=lambda cell: cell[0])
    grouped["right"].sort(key=lambda cell: cell[1])
    grouped["bottom"].sort(key=lambda cell: cell[0], reverse=True)
    grouped["left"].sort(key=lambda cell: cell[1], reverse=True)
    return [(cell, side) for side in SIDE_ORDER for cell in grouped[side]]


def centered_station(
    grid: list[str], workstation: tuple[int, int], side: str
) -> dict | None:
    rows = len(grid)
    cols = len(grid[0])

    def usable(cell: tuple[int, int]) -> bool:
        x, y = cell
        return 0 <= x < cols and 0 <= y < rows and traversable(grid[y][x])

    queue_layers: dict[str, list[tuple[int, int]]] = {}
    for field, depth in (("approach_cells", 3), ("buffer_cells", 2), ("standby_cells", 1)):
        center = add(workstation, INWARD[side], depth)
        queue_layers[field] = [add(center, TANGENT[side], offset) for offset in (-1, 0, 1)]
    exits = [add(workstation, TANGENT[side], offset) for offset in (-1, 1)]
    cells = [
        workstation,
        *queue_layers["approach_cells"],
        *queue_layers["buffer_cells"],
        *queue_layers["standby_cells"],
        *exits,
    ]
    if len(set(cells)) != 12 or not all(usable(cell) for cell in cells):
        return None
    return {
        "perimeter_side": side,
        "workstation_cell": list(workstation),
        "standby_cells": [list(cell) for cell in queue_layers["standby_cells"]],
        "buffer_cells": [list(cell) for cell in queue_layers["buffer_cells"]],
        "approach_cells": [list(cell) for cell in queue_layers["approach_cells"]],
        "exit_cells": [list(cell) for cell in exits],
    }


def build_centered_sortation_stations(
    grid: list[str],
    emitters: list[tuple[int, int]],
    pickups: list[tuple[int, int]],
) -> list[dict]:
    rows = len(grid)
    cols = len(grid[0])
    claimed = set(pickups)
    stations = []
    for workstation, side in ordered_perimeter_emitters(emitters, rows, cols):
        station = centered_station(grid, workstation, side)
        if station is None:
            continue
        footprint = {
            tuple(station["workstation_cell"]),
            *(tuple(cell) for field in ("approach_cells", "buffer_cells", "standby_cells", "exit_cells")
              for cell in station[field]),
        }
        if not footprint.isdisjoint(claimed):
            continue
        station["station_id"] = len(stations)
        stations.append(station)
        claimed.update(footprint)
    return stations


def station_footprint(stations: list[dict]) -> set[tuple[int, int]]:
    cells: set[tuple[int, int]] = set()
    for station in stations:
        cells.add(tuple(station["workstation_cell"]))
        for field in ("approach_cells", "buffer_cells", "standby_cells", "exit_cells"):
            cells.update(tuple(cell) for cell in station[field])
    return cells


def build_sortation_density_benchmark(
    source: Path,
    retention_percent: int,
    pickup_sample_seed: int,
    source_url: str,
    description: str,
) -> dict:
    rows, cols, grid = read_movingai_map(source)
    all_pickups = [
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
    pickups = select_nested_random(all_pickups, retention_percent, pickup_sample_seed)
    stations = build_centered_sortation_stations(grid, emitters, pickups)
    footprint = station_footprint(stations)
    traversable_count = sum(traversable(cell) for row in grid for cell in row)
    valid_start_capacity = traversable_count - len(set(pickups) | footprint)
    suffix = f"p{retention_percent:02d}"
    return {
        "benchmark_id": f"lorr_{source.stem}_{suffix}",
        "description": description,
        "source": source_url,
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "adapter_station_count": len(stations),
        "adapter_station_layout": "maximal_nonoverlapping_perimeter",
        "adapter_queue_layout": "centered_funnel_3x3",
        "adapter_pickup_retention_percent": retention_percent,
        "adapter_pickup_sample_seed": pickup_sample_seed,
        "adapter_original_pickup_count": len(all_pickups),
        "adapter_pickup_count": len(pickups),
        "adapter_valid_start_capacity": valid_start_capacity,
        "rows": rows,
        "cols": cols,
        "movingai_map": source.name,
        "simulation_time_default": 500,
        "planning_window_default": 20,
        "simulation_window_default": 5,
        "seed_start_default": 1,
        "seed_count_default": 10,
        "service_time_default": 3,
        "task_model": "endpoint_station_alternating",
        "restrict_station_zones": False,
        "pickup_endpoints": [list(cell) for cell in pickups],
        "stations": stations,
    }


def build_directional_stations(
    grid: list[str],
    selected: list[tuple[tuple[int, int], str]],
    pickups: list[tuple[int, int]],
) -> list[dict]:
    rows = len(grid)
    cols = len(grid[0])
    inward = {
        "top": (0, 1),
        "right": (-1, 0),
        "bottom": (0, -1),
        "left": (1, 0),
    }
    clockwise_tangent = {
        "top": (1, 0),
        "right": (0, 1),
        "bottom": (-1, 0),
        "left": (0, -1),
    }

    def usable(cell: tuple[int, int]) -> bool:
        x, y = cell
        return 0 <= x < cols and 0 <= y < rows and traversable(grid[y][x])

    workstations = {cell for cell, _side in selected}
    claimed = set(pickups) | workstations
    stations = []
    for station_id, (workstation, side) in enumerate(selected):
        lane = [add(workstation, inward[side], distance) for distance in range(1, 4)]
        if any(not usable(cell) for cell in lane):
            raise ValueError(f"station {workstation} has a blocked inward queue lane")
        if any(cell in claimed for cell in lane):
            raise ValueError(f"station {workstation} has an overlapping inward queue lane")
        claimed.update(lane)

        tangent = clockwise_tangent[side]
        exit_candidates = [add(workstation, tangent), add(workstation, (-tangent[0], -tangent[1]))]
        exit_cell = next(
            (cell for cell in exit_candidates if usable(cell) and cell not in claimed),
            None,
        )
        if exit_cell is None:
            raise ValueError(f"station {workstation} has no separate lateral exit")
        claimed.add(exit_cell)

        stations.append({
            "station_id": station_id,
            "perimeter_side": side,
            "workstation_cell": list(workstation),
            "standby_cells": [list(lane[0])],
            "buffer_cells": [list(lane[1])],
            "approach_cells": [list(lane[2])],
            "exit_cells": [list(exit_cell)],
        })
    return stations


def build_benchmark(
    source: Path,
    station_count: int,
    source_url: str,
    description: str,
    pickup_count: int | None = None,
) -> dict:
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
    if pickup_count is not None:
        pickups = select_spaced(pickups, pickup_count)
    selected = select_balanced_perimeter(emitters, station_count, rows, cols)
    stations = build_directional_stations(grid, selected, pickups)

    benchmark = {
        "benchmark_id": f"lorr_{source.stem}",
        "description": description,
        "source": source_url,
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "adapter_station_count": station_count,
        "adapter_station_layout": "balanced_perimeter",
        "adapter_queue_layout": "inward_lane_3",
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
    if pickup_count is not None:
        benchmark["adapter_pickup_count"] = pickup_count
    return benchmark


def main() -> int:
    parser = argparse.ArgumentParser(description="Adapt a LoRR E/S MovingAI map to the workstation benchmark schema.")
    parser.add_argument("--map", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--station-count", type=int, default=12)
    parser.add_argument(
        "--pickup-count",
        type=int,
        help="Deterministically select this many spaced S cells instead of using every pickup.",
    )
    parser.add_argument(
        "--pickup-retention",
        type=int,
        choices=range(1, 101),
        metavar="PERCENT",
        help="Build a centered Sortation adapter retaining a nested percentage of S cells.",
    )
    parser.add_argument(
        "--pickup-sample-seed",
        type=int,
        default=1,
        help="Seed for the deterministic nested pickup permutation.",
    )
    parser.add_argument(
        "--source-url",
        default=(
            "https://github.com/MAPF-Competition/Benchmark-Archive/blob/main/"
            "2023%20Competition/Example%20Instances/warehouse.domain/maps/warehouse_small.map"
        ),
    )
    parser.add_argument(
        "--description",
        default="LoRR warehouse map adapted to alternating storage-pickup and serviced-emitter tasks.",
    )
    args = parser.parse_args()

    if args.pickup_retention is not None and args.pickup_count is not None:
        parser.error("--pickup-retention and --pickup-count are mutually exclusive")
    if args.pickup_retention is not None:
        benchmark = build_sortation_density_benchmark(
            args.map,
            args.pickup_retention,
            args.pickup_sample_seed,
            args.source_url,
            args.description,
        )
    else:
        benchmark = build_benchmark(
            args.map,
            args.station_count,
            args.source_url,
            args.description,
            args.pickup_count,
        )
    benchmark["movingai_map"] = os.path.relpath(args.map.resolve(), args.output.resolve().parent)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(benchmark, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
