#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path


METHODS = {
    "pbs_vanilla": "vanilla",
    "pbs_distance_age": "distance_age",
    "pbs_pressure_aware": "pressure_aware",
}


def parse_counts(value: str) -> list[int]:
    return [int(part.strip()) for part in value.split(",") if part.strip()]


def parse_methods(value: str) -> list[str]:
    methods = [part.strip() for part in value.split(",") if part.strip()]
    unknown = [name for name in methods if name not in METHODS]
    if unknown:
        raise argparse.ArgumentTypeError(f"Unknown methods: {', '.join(unknown)}")
    return methods


def parse_seed_list(value: str) -> list[int]:
    seeds = [int(part.strip()) for part in value.split(",") if part.strip()]
    if not seeds:
        raise argparse.ArgumentTypeError("Seed list must contain at least one seed.")
    if len(seeds) != len(set(seeds)):
        raise argparse.ArgumentTypeError("Seed list must not contain duplicates.")
    return seeds


def load_status(path: Path) -> dict | None:
    if not path.exists():
        return None
    return json.loads(path.read_text())


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def classify_run(log_path: Path, summary_path: Path, return_code: int, timed_out: bool) -> tuple[str, str]:
    if timed_out:
        return "failed", "wall_timeout"
    if return_code != 0:
        log_text = log_path.read_text(errors="ignore") if log_path.exists() else ""
        if "Failed to repair workstation commitment conflicts" in log_text:
            return "failed", "internal_repair_failure"
        if "Invalid move" in log_text:
            return "failed", "invalid_move"
        if "has a conflict with drive" in log_text or "left workstation early" in log_text:
            return "failed", "fatal_collision"
        return "failed", f"returncode_{return_code}"
    if not summary_path.exists():
        return "failed", "missing_summary"
    log_text = log_path.read_text(errors="ignore") if log_path.exists() else ""
    if "Failed to repair workstation commitment conflicts" in log_text:
        return "failed", "internal_repair_failure"
    if "Invalid move" in log_text:
        return "failed", "invalid_move"
    if "has a conflict with drive" in log_text or "left workstation early" in log_text:
        return "failed", "fatal_collision"
    return "clean", "clean"


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]

    parser = argparse.ArgumentParser(description="Run the paper comparison on the workstation benchmarks.")
    parser.add_argument("--root", default=str(repo_root / "results" / "main_tau3_w20_h5_seed0to19"))
    parser.add_argument("--binary", default=str(repo_root / "lifelong"))
    parser.add_argument("--seed-start", type=int, default=0)
    parser.add_argument("--seed-count", type=int, default=20)
    parser.add_argument("--seed-list", type=parse_seed_list)
    parser.add_argument("--simulation-time", type=int, default=5000)
    parser.add_argument("--planning-window", type=int, default=20)
    parser.add_argument("--simulation-window", type=int, default=5)
    parser.add_argument("--service-time", type=int, default=3)
    parser.add_argument("--pressure-threshold", type=int)
    parser.add_argument("--cutoff-time", type=int, default=60)
    parser.add_argument("--process-timeout", type=int, default=1800)
    parser.add_argument("--jobs", type=int, default=6)
    parser.add_argument("--screen", type=int, default=0)
    parser.add_argument("--alley-counts", default="10,20,30,40,50")
    parser.add_argument("--plaza-counts", default="20,30,40,50,60")
    parser.add_argument(
        "--methods",
        type=parse_methods,
        default=["pbs_vanilla", "pbs_distance_age", "pbs_pressure_aware"],
    )
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    root = Path(args.root)
    root.mkdir(parents=True, exist_ok=True)
    binary = Path(args.binary)
    if not binary.exists():
        raise SystemExit(f"Missing binary: {binary}")

    seeds = args.seed_list if args.seed_list is not None else list(range(args.seed_start, args.seed_start + args.seed_count))
    grids = {
        "alley": {
            "benchmark": repo_root / "benchmarks" / "alley.json",
            "counts": parse_counts(args.alley_counts),
        },
        "plaza": {
            "benchmark": repo_root / "benchmarks" / "plaza.json",
            "counts": parse_counts(args.plaza_counts),
        },
    }

    write_json(
        root / "run_manifest.json",
        {
            "methods": args.methods,
            "seeds": seeds,
            "simulation_time": args.simulation_time,
            "planning_window": args.planning_window,
            "simulation_window": args.simulation_window,
            "service_time": args.service_time,
            "pressure_threshold": args.pressure_threshold,
            "cutoff_time": args.cutoff_time,
            "process_timeout": args.process_timeout,
            "grids": {name: cfg["counts"] for name, cfg in grids.items()},
        },
    )

    print_lock = threading.Lock()

    def log(message: str) -> None:
        with print_lock:
            print(message, flush=True)

    def run_case(map_name: str, agent_count: int, method_name: str, seed: int) -> None:
        benchmark = grids[map_name]["benchmark"]
        cell_dir = root / map_name / f"agents_{agent_count}" / method_name / f"seed_{seed}"
        status_path = cell_dir / "status.json"
        summary_path = cell_dir / "summary.csv"
        log_path = cell_dir / "run.log"

        existing = load_status(status_path)
        if existing and not args.force and existing.get("status") in {"clean", "failed"}:
            log(f"[reuse] {map_name} {agent_count} {method_name} seed {seed}")
            return

        cell_dir.mkdir(parents=True, exist_ok=True)
        cmd = [
            str(binary),
            "--scenario", "WORKSTATION",
            "--benchmark", str(benchmark),
            "--solver", "PBS",
            "--station_policy", METHODS[method_name],
            "--agentNum", str(agent_count),
            "--simulation_time", str(args.simulation_time),
            "--simulation_window", str(args.simulation_window),
            "--planning_window", str(args.planning_window),
            "--service_time", str(args.service_time),
            "--cutoffTime", str(args.cutoff_time),
            "--seed", str(seed),
            "--screen", str(args.screen),
            "--output", str(cell_dir),
        ]
        if args.pressure_threshold is not None:
            cmd.extend(["--pressure_threshold", str(args.pressure_threshold)])

        log(f"[run] {map_name} {agent_count} {method_name} seed {seed}")
        timed_out = False
        return_code = 0
        with log_path.open("w") as log_file:
            try:
                completed = subprocess.run(
                    cmd,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    timeout=args.process_timeout,
                    check=False,
                )
                return_code = completed.returncode
            except subprocess.TimeoutExpired:
                timed_out = True
                return_code = 124

        status, failure_reason = classify_run(log_path, summary_path, return_code, timed_out)
        write_json(
            status_path,
            {
                "map": map_name,
                "agent_count": agent_count,
                "method": method_name,
                "station_policy": METHODS[method_name],
                "seed": seed,
                "status": status,
                "failure_reason": failure_reason,
                "return_code": return_code,
                "output_dir": str(cell_dir),
            },
        )
        log(f"[done] {map_name} {agent_count} {method_name} seed {seed} -> {status}")

    futures = []
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        for map_name, config in grids.items():
            for agent_count in config["counts"]:
                for method_name in args.methods:
                    for seed in seeds:
                        futures.append(pool.submit(run_case, map_name, agent_count, method_name, seed))
        for future in futures:
            future.result()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
