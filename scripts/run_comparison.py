#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import platform
import subprocess
import threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path


METHODS = {
    "pbs_vanilla": {"solver": "PBS", "station_policy": "vanilla"},
    "pbs_distance_age": {"solver": "PBS", "station_policy": "distance_age"},
    "pbs_pressure_aware": {"solver": "PBS", "station_policy": "pressure_aware"},
    "pibt_vanilla": {"solver": "PIBT", "pibt_policy": "vanilla"},
    "pibt_distance_age": {"solver": "PIBT", "pibt_policy": "distance_age"},
    "pibt_pressure": {"solver": "PIBT", "pibt_policy": "pressure"},
}
RUN_OUTPUT_FILES = (
    "config.txt",
    "paths.txt",
    "planning_runtime.csv",
    "solver.csv",
    "summary.csv",
)


def parse_counts(value: str) -> list[int]:
    return [int(part.strip()) for part in value.split(",") if part.strip()]


def plaza_counts_below_floor(counts: list[int]) -> list[int]:
    return [count for count in counts if count < 20]


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


def reset_run_outputs(cell_dir: Path) -> None:
    for name in RUN_OUTPUT_FILES:
        (cell_dir / name).unlink(missing_ok=True)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def benchmark_fingerprints(path: Path) -> dict[str, str]:
    fingerprints = {path.name: sha256_file(path)}
    payload = json.loads(path.read_text())
    movingai_map = payload.get("movingai_map")
    if movingai_map:
        dependency = path.parent / str(movingai_map)
        fingerprints[dependency.name] = sha256_file(dependency)
    return fingerprints


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
    with summary_path.open() as fh:
        summary = next(csv.DictReader(fh), {})
    termination_reason = summary.get("termination_reason", "")
    if termination_reason == "traffic_jam":
        return "failed", "traffic_jam"
    if termination_reason == "solver_failure":
        return "failed", "solver_failure"
    if termination_reason == "commit_repair_failure":
        return "failed", "internal_repair_failure"
    if "Failed to repair workstation commitment conflicts" in log_text:
        return "failed", "internal_repair_failure"
    if "failed to produce a valid workstation plan" in log_text:
        return "failed", "solver_failure"
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
    parser.add_argument("--seed-start", type=int, default=1)
    parser.add_argument("--seed-count", type=int, default=20)
    parser.add_argument("--seed-list", type=parse_seed_list)
    parser.add_argument("--simulation-time", type=int, default=5000)
    parser.add_argument("--planning-window", type=int, default=20)
    parser.add_argument("--simulation-window", type=int, default=5)
    parser.add_argument("--service-time", type=int, default=3)
    parser.add_argument("--pressure-threshold", type=int)
    parser.add_argument("--pibt-pressure-entry-penalty", type=float, default=2.0)
    parser.add_argument("--pibt-pressure-inbound-limit", type=int, default=4)
    parser.add_argument("--pibt-pressure-profile", choices=("none", "half", "severe", "thirds"), default="thirds")
    parser.add_argument("--pibt-wait-penalty", type=float, default=2.0)
    parser.add_argument("--pibt-exit-bonus", type=float, default=1.0)
    parser.add_argument("--pibt-front-bonus", type=float, default=3.0)
    parser.add_argument("--pibt-soft-collision-penalty", type=float, default=0.0)
    parser.add_argument("--pibt-hindrance", dest="pibt_hindrance", action="store_true")
    parser.add_argument("--no-pibt-hindrance", dest="pibt_hindrance", action="store_false")
    parser.add_argument("--pibt-regret-iterations", type=int, default=1)
    parser.add_argument("--pibt-regret-weight", type=float, default=0.5)
    parser.add_argument(
        "--pibt-regret-scope",
        choices=("all", "pickup", "exit_pickup", "outside_zone", "pickup_outside_zone"),
        default="all",
    )
    parser.add_argument("--pibt-random-tiebreak", dest="pibt_random_tiebreak", action="store_true")
    parser.add_argument("--no-pibt-random-tiebreak", dest="pibt_random_tiebreak", action="store_false")
    parser.add_argument(
        "--pibt-hindrance-scope",
        choices=(
            "all", "inherited", "dense", "inherited_dense", "station", "inherited_station",
            "outside_zone", "inherited_outside_zone", "pickup", "inherited_pickup",
        ),
        default="inherited",
    )
    parser.add_argument("--pibt-front-priority", dest="no_pibt_front_priority", action="store_false")
    parser.add_argument("--no-pibt-front-priority", dest="no_pibt_front_priority", action="store_true")
    parser.add_argument("--pibt-phase-priority", dest="no_pibt_phase_priority", action="store_false")
    parser.add_argument("--no-pibt-phase-priority", dest="no_pibt_phase_priority", action="store_true")
    parser.set_defaults(
        pibt_hindrance=True,
        no_pibt_front_priority=False,
        no_pibt_phase_priority=True,
        pibt_random_tiebreak=True,
    )
    parser.add_argument("--cutoff-time", type=int, default=60)
    parser.add_argument("--process-timeout", type=int, default=1800)
    parser.add_argument("--jobs", type=int, default=6)
    parser.add_argument("--screen", type=int, default=0)
    parser.add_argument(
        "--continue-on-traffic-jam",
        action="store_true",
        help="Record traffic-jam episodes but do not terminate the workstation simulation when they occur.",
    )
    parser.add_argument("--alley-counts", default="10,20,30,40,50")
    parser.add_argument("--plaza-counts", default="20,30,40,50,60")
    parser.add_argument("--lorr-counts", default="", help="Agent counts for the adapted LoRR warehouse-small benchmark.")
    parser.add_argument(
        "--lorr-sortation-counts",
        default="",
        help="Agent counts for the adapted LoRR sortation-small benchmark.",
    )
    parser.add_argument(
        "--lorr-sortation-medium-counts",
        default="",
        help="Agent counts for the adapted LoRR sortation-medium scaling benchmark.",
    )
    parser.add_argument(
        "--methods",
        type=parse_methods,
        default=list(METHODS.keys()),
    )
    parser.add_argument("--force", action="store_true")
    parser.add_argument(
        "--keep-paths",
        action="store_true",
        help="Keep per-agent paths.txt trajectories for clean runs (disabled by default).",
    )
    args = parser.parse_args()

    root = Path(args.root)
    root.mkdir(parents=True, exist_ok=True)
    binary = Path(args.binary)
    if not binary.exists():
        raise SystemExit(f"Missing binary: {binary}")
    binary_sha256 = sha256_file(binary)

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
    lorr_counts = parse_counts(args.lorr_counts)
    if lorr_counts:
        grids["lorr_warehouse_small"] = {
            "benchmark": repo_root / "benchmarks" / "lorr" / "warehouse_small.json",
            "counts": lorr_counts,
        }
    lorr_sortation_counts = parse_counts(args.lorr_sortation_counts)
    if lorr_sortation_counts:
        grids["lorr_sortation_small"] = {
            "benchmark": repo_root / "benchmarks" / "lorr" / "sortation_small.json",
            "counts": lorr_sortation_counts,
        }
    lorr_sortation_medium_counts = parse_counts(args.lorr_sortation_medium_counts)
    if lorr_sortation_medium_counts:
        grids["lorr_sortation_medium"] = {
            "benchmark": repo_root / "benchmarks" / "lorr" / "sortation_medium.json",
            "counts": lorr_sortation_medium_counts,
        }
    for config in grids.values():
        config["benchmark_fingerprints"] = benchmark_fingerprints(config["benchmark"])
    invalid_plaza_counts = plaza_counts_below_floor(grids["plaza"]["counts"])
    if invalid_plaza_counts:
        parser.error(
            "Plaza counts must be at least 20 for this workstation comparison; "
            f"got {invalid_plaza_counts}."
        )

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
            "pibt_pressure_entry_penalty": args.pibt_pressure_entry_penalty,
            "pibt_pressure_inbound_limit": args.pibt_pressure_inbound_limit,
            "pibt_pressure_profile": args.pibt_pressure_profile,
            "pibt_wait_penalty": args.pibt_wait_penalty,
            "pibt_exit_bonus": args.pibt_exit_bonus,
            "pibt_front_bonus": args.pibt_front_bonus,
            "pibt_soft_collision_penalty": args.pibt_soft_collision_penalty,
            "pibt_hindrance": args.pibt_hindrance,
            "pibt_hindrance_scope": args.pibt_hindrance_scope,
            "pibt_regret_iterations": args.pibt_regret_iterations,
            "pibt_regret_weight": args.pibt_regret_weight,
            "pibt_regret_scope": args.pibt_regret_scope,
            "pibt_random_tiebreak": args.pibt_random_tiebreak,
            "pibt_front_priority": not args.no_pibt_front_priority,
            "pibt_phase_priority": not args.no_pibt_phase_priority,
            "continue_on_traffic_jam": args.continue_on_traffic_jam,
            "cutoff_time": args.cutoff_time,
            "process_timeout": args.process_timeout,
            "jobs": args.jobs,
            "screen": args.screen,
            "keep_paths": args.keep_paths,
            "platform": platform.platform(),
            "logical_cpu_count": os.cpu_count(),
            "binary": str(binary.resolve()),
            "binary_sha256": binary_sha256,
            "benchmark_fingerprints": {
                name: config["benchmark_fingerprints"] for name, config in grids.items()
            },
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
        method_config = METHODS[method_name]
        run_signature = {
            "binary_sha256": binary_sha256,
            "benchmark_fingerprints": grids[map_name]["benchmark_fingerprints"],
            "map": map_name,
            "agent_count": agent_count,
            "method": method_name,
            "seed": seed,
            "solver": method_config["solver"],
            "station_policy": method_config.get("station_policy", "vanilla"),
            "pibt_policy": method_config.get("pibt_policy", ""),
            "simulation_time": args.simulation_time,
            "planning_window": args.planning_window,
            "simulation_window": args.simulation_window,
            "service_time": args.service_time,
            "pressure_threshold": args.pressure_threshold,
            "cutoff_time": args.cutoff_time,
            "process_timeout": args.process_timeout,
            "batch_jobs": args.jobs,
            "continue_on_traffic_jam": args.continue_on_traffic_jam,
        }
        if "pibt_policy" in method_config:
            run_signature.update(
                {
                    "pibt_pressure_entry_penalty": args.pibt_pressure_entry_penalty,
                    "pibt_pressure_inbound_limit": args.pibt_pressure_inbound_limit,
                    "pibt_pressure_profile": args.pibt_pressure_profile,
                    "pibt_wait_penalty": args.pibt_wait_penalty,
                    "pibt_exit_bonus": args.pibt_exit_bonus,
                    "pibt_front_bonus": args.pibt_front_bonus,
                    "pibt_soft_collision_penalty": args.pibt_soft_collision_penalty,
                    "pibt_hindrance": args.pibt_hindrance,
                    "pibt_hindrance_scope": args.pibt_hindrance_scope,
                    "pibt_regret_iterations": args.pibt_regret_iterations,
                    "pibt_regret_weight": args.pibt_regret_weight,
                    "pibt_regret_scope": args.pibt_regret_scope,
                    "pibt_random_tiebreak": args.pibt_random_tiebreak,
                    "pibt_front_priority": not args.no_pibt_front_priority,
                    "pibt_phase_priority": not args.no_pibt_phase_priority,
                }
            )

        existing = load_status(status_path)
        if (
            existing
            and not args.force
            and existing.get("status") in {"clean", "failed"}
            and existing.get("run_signature") == run_signature
        ):
            if existing.get("status") == "clean" and not args.keep_paths:
                (cell_dir / "paths.txt").unlink(missing_ok=True)
            log(f"[reuse] {map_name} {agent_count} {method_name} seed {seed}")
            return

        cell_dir.mkdir(parents=True, exist_ok=True)
        reset_run_outputs(cell_dir)
        status_base = {
            "map": map_name,
            "agent_count": agent_count,
            "method": method_name,
            "solver": method_config["solver"],
            "station_policy": method_config.get("station_policy", ""),
            "pibt_policy": method_config.get("pibt_policy", ""),
            "seed": seed,
            "run_signature": run_signature,
            "output_dir": str(cell_dir),
        }
        write_json(
            status_path,
            {
                **status_base,
                "status": "running",
                "failure_reason": "",
                "return_code": None,
            },
        )
        cmd = [
            str(binary),
            "--scenario", "WORKSTATION",
            "--benchmark", str(benchmark),
            "--solver", method_config["solver"],
            "--station_policy", method_config.get("station_policy", "vanilla"),
            "--agentNum", str(agent_count),
            "--simulation_time", str(args.simulation_time),
            "--simulation_window", str(args.simulation_window),
            "--planning_window", str(args.planning_window),
            "--service_time", str(args.service_time),
            "--cutoffTime", str(args.cutoff_time),
            "--seed", str(seed),
            "--screen", str(args.screen),
            "--stop_at_traffic_jam", "false" if args.continue_on_traffic_jam else "true",
            "--output", str(cell_dir),
        ]
        if "pibt_policy" in method_config:
            cmd.extend(["--pibt_policy", method_config["pibt_policy"]])
            cmd.extend(["--pibt_pressure_inbound_limit", str(args.pibt_pressure_inbound_limit)])
            cmd.extend(["--pibt_pressure_profile", args.pibt_pressure_profile])
            cmd.extend(["--pibt_wait_penalty", str(args.pibt_wait_penalty)])
            cmd.extend(["--pibt_exit_bonus", str(args.pibt_exit_bonus)])
            cmd.extend(["--pibt_front_bonus", str(args.pibt_front_bonus)])
            cmd.extend(["--pibt_soft_collision_penalty", str(args.pibt_soft_collision_penalty)])
            cmd.extend(["--pibt_hindrance", str(args.pibt_hindrance).lower()])
            cmd.extend(["--pibt_hindrance_scope", args.pibt_hindrance_scope])
            cmd.extend(["--pibt_regret_iterations", str(args.pibt_regret_iterations)])
            cmd.extend(["--pibt_regret_weight", str(args.pibt_regret_weight)])
            cmd.extend(["--pibt_regret_scope", args.pibt_regret_scope])
            cmd.extend(["--pibt_random_tiebreak", str(args.pibt_random_tiebreak).lower()])
            cmd.extend(["--pibt_front_priority", str(not args.no_pibt_front_priority).lower()])
            cmd.extend(["--pibt_phase_priority", str(not args.no_pibt_phase_priority).lower()])
            cmd.extend(["--pibt_pressure_entry_penalty", str(args.pibt_pressure_entry_penalty)])
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
                **status_base,
                "status": status,
                "failure_reason": failure_reason,
                "return_code": return_code,
            },
        )
        if status == "clean" and not args.keep_paths:
            (cell_dir / "paths.txt").unlink(missing_ok=True)
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
