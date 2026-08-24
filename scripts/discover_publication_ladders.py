#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import run_comparison


METHODS = ("pibt_vanilla", "pibt_phase_aware", "pibt_pressure_aware")
STARTS = {"small": 50, "medium": 500, "large": 2000}
RESOLUTION = {"small": 10, "medium": 100, "large": 500}


def classify_terminal(statuses: list[dict[str, object]]) -> tuple[bool, str]:
    """Classify a terminal without conflating congestion and infrastructure failures."""
    if not statuses or not all(status.get("status") == "failed" for status in statuses):
        return False, "none"
    reasons = {str(status.get("failure_reason", "unknown")) for status in statuses}
    if reasons == {"traffic_jam"}:
        return True, "empirical_stall"
    if reasons == {"physical_capacity"}:
        return True, "physical_capacity"
    if "wall_timeout" in reasons:
        return True, "runtime_limit"
    return True, "native_failure_boundary"


def frozen_counts(last_nonterminal: int, terminal: int) -> list[int]:
    """Return eight evenly spaced nonterminal probes followed by the terminal."""
    step = max(1, last_nonterminal // 8)
    reportable = [step * index for index in range(1, 9)]
    if reportable[-1] >= terminal:
        raise ValueError("terminal must be above all reportable counts")
    return reportable + [terminal]


def descending_probes(start: int, resolution: int) -> list[int]:
    """Return decreasing probes that can recover when the configured start is terminal."""
    probes = []
    probe = start
    while probe > resolution:
        probe = max(resolution, (probe // 2 // resolution) * resolution)
        if not probes or probe != probes[-1]:
            probes.append(probe)
    return probes


def status_path(root: Path, label: str, count: int, method: str, seed: int) -> Path:
    return root / label / f"agents_{count}" / method / f"seed_{seed}" / "status.json"


def main() -> int:
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Discover and freeze publication Sortation count ladders.")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--binary", default=str(repo / "lifelong"))
    parser.add_argument("--pressure-admission", choices=("single", "adaptive"), required=True)
    parser.add_argument("--jobs", type=int, default=6)
    parser.add_argument("--process-timeout", type=int, default=1800)
    parser.add_argument("--maps", default="small,medium,large")
    parser.add_argument("--densities", default="5,20,100")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    seeds = list(range(1, 6))
    maps = [value.strip() for value in args.maps.split(",") if value.strip()]
    densities = [int(value) for value in args.densities.split(",") if value.strip()]
    if any(value not in STARTS for value in maps):
        parser.error("--maps must contain only small, medium, or large")
    if any(value not in (5, 20, 100) for value in densities):
        parser.error("--densities must contain only 5, 20, or 100")

    def terminal(map_name: str, density: int, count: int, capacity: int) -> tuple[bool, str]:
        if count > capacity:
            return True, "physical_capacity"
        label = f"lorr_sortation_{map_name}_p{density:02d}"
        condition_root = args.root / map_name / f"p{density:02d}"
        option_prefix = "--lorr-sortation"
        if map_name != "small":
            option_prefix += f"-{map_name}"
        benchmark = repo / f"benchmarks/lorr/sortation_{map_name}_p{density:02d}.json"
        command = [
            sys.executable, str(repo / "scripts/run_comparison.py"),
            "--root", str(condition_root), "--binary", str(Path(args.binary).resolve()),
            "--seed-list", ",".join(map(str, seeds)), "--simulation-time", "1000",
            "--planning-window", "20", "--simulation-window", "5", "--service-time", "3",
            "--pressure-threshold", "1", "--pressure-zone-cost", "1",
            "--pressure-cost-scope", "zone",
            "--pressure-cost-occupancy-threshold", "4",
            "--pressure-inbound-limit", "3", "--pressure-admission", args.pressure_admission,
            "--methods", ",".join(METHODS), "--alley-counts", "", "--plaza-counts", "",
            f"{option_prefix}-counts", str(count), f"{option_prefix}-benchmark", str(benchmark),
            f"{option_prefix}-name", label, "--jobs", str(args.jobs),
            "--process-timeout", str(args.process_timeout),
        ]
        if args.force:
            command.append("--force")
        subprocess.run(command, check=True)
        statuses = [
            json.loads(status_path(condition_root, label, count, method, seed).read_text())
            for method in METHODS for seed in seeds
        ]
        return classify_terminal(statuses)

    for map_name in maps:
        for density in densities:
            benchmark = repo / f"benchmarks/lorr/sortation_{map_name}_p{density:02d}.json"
            capacity = int(json.loads(benchmark.read_text())["adapter_valid_start_capacity"])
            resolution = RESOLUTION[map_name]
            low = 0
            probe = min(STARTS[map_name], capacity)
            terminal_type = "empirical_stall"
            failed, terminal_type = terminal(map_name, density, probe, capacity)
            if failed:
                high = probe
                for lower_probe in descending_probes(probe, resolution):
                    lower_failed, lower_type = terminal(
                        map_name, density, lower_probe, capacity)
                    if not lower_failed:
                        low = lower_probe
                        break
                    high = lower_probe
                    terminal_type = lower_type
                if low == 0:
                    raise RuntimeError(
                        f"{map_name} p{density:02d} has no nonterminal probe at or above "
                        f"resolution {resolution}; it cannot support an eight-point ladder"
                    )
            while True:
                if failed:
                    break
                failed, terminal_type = terminal(map_name, density, probe, capacity)
                if failed:
                    high = probe
                    break
                low = probe
                if probe == capacity:
                    high = capacity + 1
                    terminal_type = "physical_capacity"
                    break
                probe = min(capacity, probe * 2)

            while terminal_type == "empirical_stall" and high - low > resolution:
                middle = ((low + high) // (2 * resolution)) * resolution
                if middle <= low:
                    middle = low + resolution
                failed, middle_type = terminal(map_name, density, middle, capacity)
                if failed:
                    high = middle
                    terminal_type = middle_type
                else:
                    low = middle

            counts = frozen_counts(low, high)
            config["sortation"][map_name][f"p{density:02d}"] = {
                "counts": counts,
                "terminal_type": terminal_type,
                "capacity": capacity,
                "discovery_seeds": seeds,
            }
            print(f"[freeze] {map_name} p{density:02d}: {counts} ({terminal_type})", flush=True)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(config, indent=2, sort_keys=True) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
