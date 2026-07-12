#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
from pathlib import Path

import run_comparison


DENSITIES = (5, 10, 20, 50, 100)
METHODS = ("pibt_vanilla", "pibt_distance_age", "pibt_pressure")
MAP_CONFIG = {
    "small": {
        "stem": "sortation_small",
        "label": "lorr_sortation_small",
        "start_count": 50,
        "resolution": 10,
        "count_option": "--lorr-sortation-counts",
        "benchmark_option": "--lorr-sortation-benchmark",
        "name_option": "--lorr-sortation-name",
    },
    "medium": {
        "stem": "sortation_medium",
        "label": "lorr_sortation_medium",
        "start_count": 1000,
        "resolution": 100,
        "count_option": "--lorr-sortation-medium-counts",
        "benchmark_option": "--lorr-sortation-medium-benchmark",
        "name_option": "--lorr-sortation-medium-name",
    },
}


def density_tag(density: int) -> str:
    return f"p{density:02d}"


def condition_label(map_key: str, density: int) -> str:
    return f"{MAP_CONFIG[map_key]['label']}_{density_tag(density)}"


def benchmark_path(repo_root: Path, map_key: str, density: int) -> Path:
    stem = MAP_CONFIG[map_key]["stem"]
    return repo_root / "benchmarks" / "lorr" / f"{stem}_{density_tag(density)}.json"


def load_capacity(path: Path) -> int:
    payload = json.loads(path.read_text())
    return int(payload["adapter_valid_start_capacity"])


def status_path(
    root: Path,
    map_key: str,
    density: int,
    count: int,
    method: str,
    seed: int,
) -> Path:
    label = condition_label(map_key, density)
    return root / "conditions" / label / label / f"agents_{count}" / method / f"seed_{seed}" / "status.json"


def load_cell_statuses(
    root: Path,
    map_key: str,
    density: int,
    count: int,
    seeds: list[int],
) -> dict[str, list[dict]]:
    statuses: dict[str, list[dict]] = {}
    for method in METHODS:
        entries = []
        for seed in seeds:
            path = status_path(root, map_key, density, count, method, seed)
            if not path.exists():
                raise RuntimeError(f"missing result status: {path}")
            entries.append(json.loads(path.read_text()))
        statuses[method] = entries
    return statuses


def all_methods_failed(statuses: dict[str, list[dict]]) -> bool:
    return all(
        all(entry.get("status") == "failed" for entry in entries)
        for entries in statuses.values()
    )


def method_clean_counts(statuses: dict[str, list[dict]]) -> dict[str, int]:
    return {
        method: sum(entry.get("status") == "clean" for entry in entries)
        for method, entries in statuses.items()
    }


def rounded_probe(value: float, resolution: int, capacity: int) -> int:
    rounded = int(round(value / resolution)) * resolution
    return min(capacity, max(1, rounded))


def coarse_probe_counts(start_count: int, capacity: int, resolution: int) -> list[int]:
    fractions = (0.25, 0.45, 0.65, 0.80, 0.90, 0.95, 0.98, 1.0)
    counts = {start_count, capacity}
    counts.update(rounded_probe(capacity * fraction, resolution, capacity) for fraction in fractions)
    return sorted(count for count in counts if start_count <= count <= capacity)


class DensityExperiment:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.repo_root = Path(__file__).resolve().parents[1]
        self.root = Path(args.root).resolve()
        self.binary = Path(args.binary).resolve()
        self.discovery_seeds = list(range(1, args.discovery_seed_count + 1))
        self.full_seeds = list(range(1, args.seed_count + 1))
        self.frontier_path = self.root / "frontier_manifest.json"
        self.frontier = self.load_frontier()

    def load_frontier(self) -> dict:
        if self.frontier_path.exists():
            return json.loads(self.frontier_path.read_text())
        return {
            "schema_version": 1,
            "methods": list(METHODS),
            "discovery_seeds": self.discovery_seeds,
            "full_seeds": self.full_seeds,
            "simulation_time": self.args.simulation_time,
            "planning_window": self.args.planning_window,
            "simulation_window": self.args.simulation_window,
            "service_time": self.args.service_time,
            "maps": {},
        }

    def save_frontier(self) -> None:
        self.root.mkdir(parents=True, exist_ok=True)
        self.frontier_path.write_text(json.dumps(self.frontier, indent=2, sort_keys=True) + "\n")

    def condition_root(self, map_key: str, density: int) -> Path:
        return self.root / "conditions" / condition_label(map_key, density)

    def runner_command(
        self,
        map_key: str,
        density: int,
        counts: list[int],
        seeds: list[int],
    ) -> list[str]:
        config = MAP_CONFIG[map_key]
        label = condition_label(map_key, density)
        command = [
            sys.executable,
            str(self.repo_root / "scripts" / "run_comparison.py"),
            "--root", str(self.condition_root(map_key, density)),
            "--binary", str(self.binary),
            "--seed-list", ",".join(str(seed) for seed in seeds),
            "--simulation-time", str(self.args.simulation_time),
            "--planning-window", str(self.args.planning_window),
            "--simulation-window", str(self.args.simulation_window),
            "--service-time", str(self.args.service_time),
            "--cutoff-time", str(self.args.cutoff_time),
            "--process-timeout", str(self.args.process_timeout),
            "--jobs", str(self.args.jobs),
            "--screen", "0",
            "--methods", ",".join(METHODS),
            "--alley-counts", "",
            "--plaza-counts", "",
            config["count_option"], ",".join(str(count) for count in counts),
            config["benchmark_option"], str(benchmark_path(self.repo_root, map_key, density)),
            config["name_option"], label,
        ]
        if self.args.force:
            command.append("--force")
        return command

    def run_counts(
        self,
        map_key: str,
        density: int,
        counts: list[int],
        seeds: list[int],
    ) -> None:
        if not counts:
            return
        subprocess.run(self.runner_command(map_key, density, counts, seeds), check=True)

    def terminal(self, map_key: str, density: int, count: int, seeds: list[int]) -> bool:
        self.run_counts(map_key, density, [count], seeds)
        statuses = load_cell_statuses(self.root, map_key, density, count, seeds)
        clean = method_clean_counts(statuses)
        print(
            f"[frontier] {condition_label(map_key, density)} agents={count} "
            + " ".join(f"{method}={clean[method]}/{len(seeds)}" for method in METHODS),
            flush=True,
        )
        return all_methods_failed(statuses)

    def precompute(self) -> None:
        for map_key in self.args.maps:
            for density in DENSITIES:
                label = condition_label(map_key, density)
                output = self.root / "precompute" / label
                output.mkdir(parents=True, exist_ok=True)
                command = [
                    str(self.binary),
                    "--scenario", "WORKSTATION",
                    "--benchmark", str(benchmark_path(self.repo_root, map_key, density)),
                    "--solver", "PIBT",
                    "--pibt_policy", "vanilla",
                    "--agentNum", "1",
                    "--simulation_time", str(self.args.simulation_window),
                    "--simulation_window", str(self.args.simulation_window),
                    "--planning_window", str(self.args.planning_window),
                    "--service_time", str(self.args.service_time),
                    "--cutoffTime", str(self.args.cutoff_time),
                    "--seed", "1",
                    "--screen", "0",
                    "--stop_at_traffic_jam", "false",
                    "--output", str(output),
                ]
                print(f"[precompute] {label}", flush=True)
                with (output / "precompute.log").open("w") as log:
                    subprocess.run(
                        command,
                        stdout=log,
                        stderr=subprocess.STDOUT,
                        timeout=self.args.precompute_timeout,
                        check=True,
                    )

    def bracket_five_percent(self, map_key: str) -> tuple[int, int | None]:
        config = MAP_CONFIG[map_key]
        capacity = load_capacity(benchmark_path(self.repo_root, map_key, 5))
        low = config["start_count"]
        if self.terminal(map_key, 5, low, self.discovery_seeds):
            raise RuntimeError(f"starting count {low} already fails on {map_key}")
        high = -1
        for count in coarse_probe_counts(low, capacity, config["resolution"]):
            if count == low:
                continue
            if self.terminal(map_key, 5, count, self.discovery_seeds):
                high = count
                break
            low = count
        if high < 0:
            return low, None

        resolution = config["resolution"]
        while high - low > resolution:
            midpoint = rounded_probe((low + high) / 2, resolution, capacity)
            if midpoint <= low:
                midpoint = min(high, low + resolution)
            if midpoint >= high:
                break
            if self.terminal(map_key, 5, midpoint, self.discovery_seeds):
                high = midpoint
            else:
                low = midpoint
        return low, high

    def lock_five_percent_ladder(self, map_key: str) -> tuple[list[int], int, str]:
        config = MAP_CONFIG[map_key]
        start = config["start_count"]
        resolution = config["resolution"]
        capacity = load_capacity(benchmark_path(self.repo_root, map_key, 5))
        _low, high = self.bracket_five_percent(map_key)
        target_intervals = 10
        if high is None:
            step = max(
                resolution,
                math.ceil((capacity + 1 - start) / (target_intervals * resolution)) * resolution,
            )
            ladder = []
            count = start
            while count <= capacity:
                ladder.append(count)
                count += step
            if len(ladder) < 2:
                raise RuntimeError(f"capacity-boundary ladder for {map_key} has fewer than two points")
            return ladder, count, "capacity_exceeded"

        step = max(
            resolution,
            math.ceil((high - start) / (target_intervals * resolution)) * resolution,
        )

        visited = set()
        while step not in visited:
            visited.add(step)
            reported_last = start + (target_intervals - 1) * step
            terminal_count = start + target_intervals * step
            if terminal_count > capacity:
                step -= resolution
                continue
            last_fails = self.terminal(map_key, 5, reported_last, self.discovery_seeds)
            terminal_fails = self.terminal(map_key, 5, terminal_count, self.discovery_seeds)
            if last_fails:
                step -= resolution
                continue
            if not terminal_fails:
                step += resolution
                continue
            if step <= 0:
                break

            last_fails_full = self.terminal(map_key, 5, reported_last, self.full_seeds)
            terminal_fails_full = self.terminal(map_key, 5, terminal_count, self.full_seeds)
            if last_fails_full:
                step -= resolution
                continue
            if not terminal_fails_full:
                step += resolution
                continue
            ladder = [start + index * step for index in range(target_intervals + 1)]
            return ladder[:-1], ladder[-1], "traffic_jam"
        raise RuntimeError(f"could not construct a verified equal-spacing ladder for {map_key}")

    def discover_map(self, map_key: str) -> None:
        accepted, terminal_count, terminal_mode = self.lock_five_percent_ladder(map_key)
        map_entry = {
            "base_ladder": accepted + [terminal_count],
            "step": accepted[1] - accepted[0],
            "densities": {},
        }
        map_entry["densities"]["5"] = {
            "capacity": load_capacity(benchmark_path(self.repo_root, map_key, 5)),
            "accepted_counts": accepted,
            "terminal_count": terminal_count,
            "terminal_mode": terminal_mode,
        }
        self.frontier["maps"][map_key] = map_entry
        self.save_frontier()

        for density in DENSITIES[1:]:
            capacity = load_capacity(benchmark_path(self.repo_root, map_key, density))
            density_accepted = []
            density_terminal = None
            terminal_mode = None
            base_terminal = map_entry["base_ladder"][-1]
            for count in map_entry["base_ladder"]:
                if count > capacity:
                    density_terminal = count
                    terminal_mode = "capacity_exceeded"
                    break
                if self.terminal(map_key, density, count, self.discovery_seeds):
                    if self.terminal(map_key, density, count, self.full_seeds):
                        density_terminal = count
                        terminal_mode = "traffic_jam"
                        break
                if count == base_terminal:
                    density_terminal = count
                    terminal_mode = "five_percent_frontier_cap"
                    break
                density_accepted.append(count)
            if density_terminal is None:
                raise RuntimeError(f"no terminal decision recorded for {density}% {map_key}")
            map_entry["densities"][str(density)] = {
                "capacity": capacity,
                "accepted_counts": density_accepted,
                "terminal_count": density_terminal,
                "terminal_mode": terminal_mode,
            }
            self.save_frontier()

    def discover(self) -> None:
        for map_key in self.args.maps:
            self.discover_map(map_key)

    def write_master_manifest(self) -> None:
        grids = {}
        fingerprints = {}
        density_metadata = {}
        for map_key in self.args.maps:
            map_entry = self.frontier["maps"][map_key]
            for density in DENSITIES:
                label = condition_label(map_key, density)
                density_entry = map_entry["densities"][str(density)]
                grids[label] = density_entry["accepted_counts"]
                sidecar = benchmark_path(self.repo_root, map_key, density)
                fingerprints[label] = run_comparison.benchmark_fingerprints(sidecar)
                density_metadata[label] = {
                    "map": map_key,
                    "retention_percent": density,
                    "benchmark": str(sidecar.resolve()),
                    **density_entry,
                }
        manifest = {
            "methods": list(METHODS),
            "seeds": self.full_seeds,
            "simulation_time": self.args.simulation_time,
            "planning_window": self.args.planning_window,
            "simulation_window": self.args.simulation_window,
            "service_time": self.args.service_time,
            "cutoff_time": self.args.cutoff_time,
            "process_timeout": self.args.process_timeout,
            "jobs": self.args.jobs,
            "stop_at_traffic_jam": True,
            "binary": str(self.binary),
            "binary_sha256": run_comparison.sha256_file(self.binary),
            "benchmark_fingerprints": fingerprints,
            "grids": grids,
            "density_conditions": density_metadata,
        }
        (self.root / "run_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")

    def write_frontier_diagnostics(self) -> None:
        rows = []
        for map_key in self.args.maps:
            map_entry = self.frontier["maps"][map_key]
            for density in DENSITIES:
                entry = map_entry["densities"][str(density)]
                count = int(entry["terminal_count"])
                if entry["terminal_mode"] == "capacity_exceeded":
                    for method in METHODS:
                        rows.append({
                            "map": map_key,
                            "retention_percent": density,
                            "capacity": entry["capacity"],
                            "terminal_count": count,
                            "terminal_mode": entry["terminal_mode"],
                            "method": method,
                            "seed_count": 0,
                            "clean_seed_count": 0,
                            "failure_reasons": "capacity_exceeded",
                        })
                    continue
                statuses = load_cell_statuses(
                    self.root, map_key, density, count, self.full_seeds
                )
                for method, method_statuses in statuses.items():
                    failures = sorted({
                        status.get("failure_reason", "")
                        for status in method_statuses
                        if status.get("failure_reason")
                    })
                    rows.append({
                        "map": map_key,
                        "retention_percent": density,
                        "capacity": entry["capacity"],
                        "terminal_count": count,
                        "terminal_mode": entry["terminal_mode"],
                        "method": method,
                        "seed_count": len(method_statuses),
                        "clean_seed_count": sum(status.get("status") == "clean" for status in method_statuses),
                        "failure_reasons": ";".join(failures),
                    })
        with (self.root / "frontier_diagnostics.csv").open("w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

    def full(self) -> None:
        for map_key in self.args.maps:
            if map_key not in self.frontier["maps"]:
                raise RuntimeError(f"missing discovered frontier for {map_key}")
            map_entry = self.frontier["maps"][map_key]
            for density in DENSITIES:
                counts = map_entry["densities"][str(density)]["accepted_counts"]
                self.run_counts(map_key, density, counts, self.full_seeds)
        self.write_master_manifest()
        self.write_frontier_diagnostics()
        subprocess.run(
            [
                sys.executable,
                str(self.repo_root / "scripts" / "aggregate_results.py"),
                "--root", str(self.root),
            ],
            check=True,
        )


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Discover and run nested-pickup Sortation density frontiers.")
    parser.add_argument(
        "--root",
        default=str(repo_root / "results" / "pibt_sortation_density_tau3_w20_h5_seed1to10"),
    )
    parser.add_argument("--binary", default=str(repo_root / "lifelong"))
    parser.add_argument("--stage", choices=("precompute", "discover", "full", "all"), default="all")
    parser.add_argument("--map-set", default="small,medium")
    parser.add_argument("--discovery-seed-count", type=int, default=3)
    parser.add_argument("--seed-count", type=int, default=10)
    parser.add_argument("--simulation-time", type=int, default=500)
    parser.add_argument("--planning-window", type=int, default=20)
    parser.add_argument("--simulation-window", type=int, default=5)
    parser.add_argument("--service-time", type=int, default=3)
    parser.add_argument("--cutoff-time", type=int, default=60)
    parser.add_argument("--process-timeout", type=int, default=1800)
    parser.add_argument("--precompute-timeout", type=int, default=14400)
    parser.add_argument("--jobs", type=int, default=6)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    args.maps = [part.strip() for part in args.map_set.split(",") if part.strip()]
    unknown_maps = [name for name in args.maps if name not in MAP_CONFIG]
    if unknown_maps:
        parser.error(f"unknown maps in --map-set: {','.join(unknown_maps)}")
    if args.discovery_seed_count < 1 or args.seed_count < args.discovery_seed_count:
        parser.error("seed counts must satisfy 1 <= discovery <= full")

    experiment = DensityExperiment(args)
    if args.stage in {"precompute", "all"}:
        experiment.precompute()
    if args.stage in {"discover", "all"}:
        experiment.discover()
    if args.stage in {"full", "all"}:
        experiment.full()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
