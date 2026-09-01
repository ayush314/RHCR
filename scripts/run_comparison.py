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
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path


METHODS = {
    "pbs_vanilla": {"solver": "PBS", "station_policy": "vanilla"},
    "pbs_lead_aware": {"solver": "PBS", "station_policy": "lead_aware"},
    "pbs_departure_aware": {"solver": "PBS", "station_policy": "departure_aware"},
    "pbs_pressure_aware": {"solver": "PBS", "station_policy": "pressure_aware"},
    "pibt_vanilla": {"solver": "PIBT2", "pibt_policy": "vanilla"},
    "pibt_lead_aware": {"solver": "PIBT2", "pibt_policy": "lead_aware"},
    "pibt_departure_aware": {"solver": "PIBT2", "pibt_policy": "departure_aware"},
    "pibt_pressure_aware": {"solver": "PIBT2", "pibt_policy": "pressure_aware"},
    "pibt2_vanilla": {"solver": "PIBT2", "pibt_policy": "vanilla"},
    "pibt2_lead_aware": {"solver": "PIBT2", "pibt_policy": "lead_aware"},
    "pibt2_departure_aware": {"solver": "PIBT2", "pibt_policy": "departure_aware"},
    "pibt2_pressure_aware": {"solver": "PIBT2", "pibt_policy": "pressure_aware"},
    "pibt_legacy_vanilla": {"solver": "PIBT", "pibt_policy": "vanilla"},
    "pibt_legacy_lead_aware": {"solver": "PIBT", "pibt_policy": "lead_aware"},
    "pibt_legacy_departure_aware": {"solver": "PIBT", "pibt_policy": "departure_aware"},
    "pibt_legacy_pressure_aware": {"solver": "PIBT", "pibt_policy": "pressure_aware"},
}
PRESSURE_DEFINITION = {
    "pressure_population": "all_agents_in_queue_region",
    "pressure_evaluation": "projected_each_step",
    "pressure_action_timing": "state_t_scores_action_t_plus_1",
    "pressure_task_metadata": "executed_only",
    "pressure_threshold": 3,
    "pressure_queue_cost": 2,
    "pressure_privileged_inbound_count": 2,
    "pressure_privileged_right_of_way": "top_ranked_lead_queue_local_priority",
    "pbs_pressure_right_of_way_count_per_station": 1,
    "pibt_pressure_right_of_way_count_per_station": 1,
    "pbs_pressure_queue_cost_scope": "each_planned_zone_occupancy",
    "pibt_pressure_queue_cost_scope": "each_planned_zone_occupancy",
    "pressure_priority_order": [
        "mandatory_service_dwell", "to_exit",
        "pressure_lead_near_target_queue", "native_solver_priority",
    ],
    "pbs_pressure_right_of_way": (
        "preferred_conflict_branch_generated_first_both_branches_retained"
    ),
    "pibt_pressure_right_of_way": (
        "pressure_lead_before_native_age_near_target_queue"
    ),
    "pressure_privilege_key": [
        "inside_target_queue", "distance_to_workstation",
        "station_leg_issue_time", "agent_id",
    ],
    "pressure_priority_parent": "lead_aware",
    "lead_aware_priority_count_per_station": 1,
    "lead_aware_priority_key": [
        "inside_target_queue", "distance_to_workstation",
        "station_leg_issue_time", "agent_id",
    ],
    "pbs_lead_aware_scope": "conflicts_touching_target_queue",
    "pbs_lead_aware_branch_rule": (
        "native_path_cost_then_conflict_count_then_lead_tiebreak"
    ),
    "pibt_lead_aware_scope": "current_or_one_action_touches_target_queue",
    "pibt_lead_aware_branch_rule": (
        "native_age_then_initial_distance_then_local_lead_tiebreak_then_seeded_tie"
    ),
    "shared_exit_requirement": "service_to_selected_exit_before_next_task",
    "shared_exit_clearance_enabled": True,
    "shared_exit_clearance_phase": "TO_EXIT",
    "shared_exit_clearance_feasibility": "collision_constraints_unchanged",
    "mandatory_service_dwell": True,
    "mandatory_service_dwell_handling": "policy_independent",
    "pbs_mandatory_service_handling": (
        "service_preserving_branch_first_both_branches_retained"
    ),
    "pibt_mandatory_service_handling": "forced_wait_before_policy_order",
    "method_specific_service_priority_enabled": False,
    "traffic_jam_rule": (
        "rhcr_majority_wait_full_execution_window_and_no_service_completion"
    ),
}
PUBLICATION_METHODS = (
    "pbs_vanilla", "pbs_lead_aware", "pbs_pressure_aware",
    "pibt_vanilla", "pibt_lead_aware", "pibt_pressure_aware",
)
PIBT_RANDOMNESS = "simulation_seed_and_absolute_destination_timestep"
RUN_OUTPUT_FILES = (
    "config.txt",
    "paths.txt",
    "planning_runtime.csv",
    "solver.csv",
    "summary.csv",
)


def parse_counts(value: str) -> list[int]:
    return [int(part.strip()) for part in value.split(",") if part.strip()]


def workstation_start_capacity(path: Path) -> int:
    payload = json.loads(path.read_text())
    explicit = payload.get("adapter_valid_start_capacity")
    if explicit is not None:
        return int(explicit)

    if "movingai_map" in payload:
        map_path = (path.parent / payload["movingai_map"]).resolve()
        lines = map_path.read_text().splitlines()
        try:
            map_start = lines.index("map") + 1
        except ValueError as exc:
            raise ValueError(f"Missing map section in {map_path}") from exc
        traversable = {
            (col, row)
            for row, line in enumerate(lines[map_start:])
            for col, cell in enumerate(line)
            if cell in {".", "G", "S"}
        }
    else:
        row_bounds = payload.get("walkable_rows", [0, int(payload["rows"]) - 1])
        col_bounds = payload.get("walkable_cols", [0, int(payload["cols"]) - 1])
        traversable = {
            (col, row)
            for row in range(int(row_bounds[0]), int(row_bounds[1]) + 1)
            for col in range(int(col_bounds[0]), int(col_bounds[1]) + 1)
        }

    reserved = {tuple(cell) for cell in payload.get("pickup_endpoints", [])}
    station_fields = (
        "standby_cells", "buffer_cells", "approach_cells", "exit_cells"
    )
    for station in payload.get("stations", []):
        reserved.add(tuple(station["workstation_cell"]))
        for field in station_fields:
            reserved.update(tuple(cell) for cell in station.get(field, []))
    return len(traversable - reserved)


def capacity_spaced_counts(capacity: int, start: int = 20, points: int = 19) -> list[int]:
    if start < 1 or points < 2 or capacity <= start:
        raise ValueError("capacity ladder requires capacity > start >= 1 and at least two points")
    step = (capacity - start) // points
    if step < 1:
        raise ValueError(
            f"Cannot fit {points} points from {start} below capacity {capacity}"
        )
    counts = [start + index * step for index in range(points)]
    if counts[-1] + step > capacity:
        raise AssertionError("capacity ladder must leave one full interval below capacity")
    return counts


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


def signatures_match(existing: dict | None, expected: dict) -> bool:
    if not isinstance(existing, dict):
        return False
    comparable = dict(existing)
    comparable.pop("batch_jobs", None)
    return comparable == expected


def status_is_reusable(existing: dict | None, run_signature: dict) -> bool:
    if not existing or existing.get("status") not in {"clean", "failed"}:
        return False
    return_code = existing.get("return_code")
    if isinstance(return_code, int) and return_code < 0:
        return False
    return signatures_match(existing.get("run_signature"), run_signature)


def all_cells_failed(statuses: list[dict | None]) -> bool:
    return bool(statuses) and all(
        status is not None and status.get("status") == "failed"
        for status in statuses
    )


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def git_provenance(repo_root: Path) -> dict[str, str | bool]:
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=repo_root,
            check=True, capture_output=True, text=True,
        ).stdout.strip()
        dirty = bool(subprocess.run(
            ["git", "status", "--porcelain"], cwd=repo_root,
            check=True, capture_output=True, text=True,
        ).stdout.strip())
        return {"source_git_commit": commit, "source_git_dirty": dirty}
    except (OSError, subprocess.CalledProcessError):
        return {"source_git_commit": "unknown", "source_git_dirty": True}


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
        if "valid start cells" in log_text:
            return "failed", "physical_capacity"
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
    if summary.get("clean_completion") == "1":
        return "clean", "clean"
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
    return "failed", termination_reason or "unclean_termination"


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]

    parser = argparse.ArgumentParser(description="Run the paper comparison on the workstation benchmarks.")
    parser.add_argument("--root", default=str(repo_root / "results" / "main_tau3_w20_h5_seed26to30"))
    parser.add_argument("--binary", default=str(repo_root / "lifelong"))
    parser.add_argument("--seed-start", type=int, default=26)
    parser.add_argument("--seed-count", type=int, default=5)
    parser.add_argument("--seed-list", type=parse_seed_list)
    parser.add_argument("--pickup-layout-seed", type=int, default=1)
    parser.add_argument("--simulation-time", type=int, default=1000)
    parser.add_argument("--planning-window", type=int, default=20)
    parser.add_argument("--simulation-window", type=int, default=5)
    parser.add_argument("--service-time", type=int, default=3)
    parser.add_argument(
        "--pressure-k",
        type=int,
        default=PRESSURE_DEFINITION["pressure_privileged_inbound_count"],
        help="Number of inbound agents exempt from the pressure queue cost.",
    )
    parser.add_argument(
        "--pressure-threshold",
        type=int,
        default=PRESSURE_DEFINITION["pressure_threshold"],
        help="Queue-region occupancy that activates pressure.",
    )
    fallback_group = parser.add_mutually_exclusive_group()
    fallback_group.add_argument(
        "--lra-fallback", "--allow-fallbacks", dest="lra_fallback",
        action="store_true", help="Enable the common failure-time LRA fallback (default).",
    )
    fallback_group.add_argument(
        "--no-lra-fallback", dest="lra_fallback", action="store_false",
        help="Disable LRA so a native solver failure terminates the run.",
    )
    parser.set_defaults(lra_fallback=True)
    parser.add_argument(
        "--commitment-repair", action="store_true",
        help="Exploratory only: repair conflicts introduced by post-solve workstation commitments.",
    )
    parser.add_argument("--pibt-random-tiebreak", dest="pibt_random_tiebreak", action="store_true")
    parser.add_argument("--no-pibt-random-tiebreak", dest="pibt_random_tiebreak", action="store_false")
    parser.set_defaults(pibt_random_tiebreak=True)
    parser.add_argument("--cutoff-time", type=int, default=60)
    parser.add_argument("--process-timeout", type=int, default=600)
    parser.add_argument("--jobs", type=int, default=6)
    parser.add_argument("--screen", type=int, default=0)
    parser.add_argument(
        "--continue-on-traffic-jam",
        action="store_true",
        help="Record traffic-jam episodes but do not terminate the workstation simulation when they occur.",
    )
    parser.add_argument("--alley-counts", default="10,20,30,40,50")
    parser.add_argument("--plaza-counts", default="20,30,40,50,60")
    parser.add_argument(
        "--human-capacity-points",
        type=int,
        default=0,
        help=(
            "Override Alley and Plaza counts with this many equally spaced points "
            "from 20, leaving one full interval below valid-start capacity."
        ),
    )
    parser.add_argument(
        "--stop-at-first-all-failed-count",
        action="store_true",
        help=(
            "Evaluate each map in ascending count order and retire each method "
            "after its first count where every seed fails."
        ),
    )
    parser.add_argument("--lorr-counts", default="", help="Agent counts for the adapted LoRR warehouse-small benchmark.")
    parser.add_argument(
        "--lorr-sortation-counts",
        default="",
        help="Agent counts for the adapted LoRR sortation-small benchmark.",
    )
    parser.add_argument(
        "--lorr-sortation-benchmark",
        type=Path,
        default=repo_root / "benchmarks" / "lorr" / "sortation_small.json",
        help="Benchmark sidecar used for --lorr-sortation-counts.",
    )
    parser.add_argument(
        "--lorr-sortation-name",
        default="lorr_sortation_small",
        help="Result map label used for the sortation-small benchmark.",
    )
    parser.add_argument(
        "--lorr-sortation-medium-counts",
        default="",
        help="Agent counts for the adapted LoRR sortation-medium scaling benchmark.",
    )
    parser.add_argument(
        "--lorr-sortation-medium-benchmark",
        type=Path,
        default=repo_root / "benchmarks" / "lorr" / "sortation_medium.json",
        help="Benchmark sidecar used for --lorr-sortation-medium-counts.",
    )
    parser.add_argument(
        "--lorr-sortation-medium-name",
        default="lorr_sortation_medium",
        help="Result map label used for the sortation-medium benchmark.",
    )
    parser.add_argument(
        "--lorr-sortation-large-counts",
        default="",
        help="Agent counts for the adapted LoRR sortation-large scaling benchmark.",
    )
    parser.add_argument(
        "--lorr-sortation-large-benchmark",
        type=Path,
        default=repo_root / "benchmarks" / "lorr" / "sortation_large_p05.json",
        help="Benchmark sidecar used for --lorr-sortation-large-counts.",
    )
    parser.add_argument(
        "--lorr-sortation-large-name",
        default="lorr_sortation_large",
        help="Result map label used for the sortation-large benchmark.",
    )
    parser.add_argument(
        "--methods",
        type=parse_methods,
        default=list(PUBLICATION_METHODS),
    )
    parser.add_argument("--force", action="store_true")
    parser.add_argument(
        "--keep-paths",
        action="store_true",
        help="Keep per-agent paths.txt trajectories for clean runs (disabled by default).",
    )
    args = parser.parse_args()
    if args.process_timeout <= 0:
        parser.error("--process-timeout must be positive")
    if args.jobs <= 0:
        parser.error("--jobs must be positive")
    if args.pressure_k < 0:
        parser.error("--pressure-k must be nonnegative")
    if args.pressure_threshold <= 0:
        parser.error("--pressure-threshold must be positive")

    root = Path(args.root)
    root.mkdir(parents=True, exist_ok=True)
    binary = Path(args.binary).resolve()
    if not binary.exists():
        raise SystemExit(f"Missing binary: {binary}")
    binary_sha256 = sha256_file(binary)
    source_provenance = git_provenance(repo_root)

    seeds = args.seed_list if args.seed_list is not None else list(range(args.seed_start, args.seed_start + args.seed_count))
    human_benchmarks = {
        "alley": repo_root / "benchmarks" / "alley.json",
        "plaza": repo_root / "benchmarks" / "plaza.json",
    }
    human_capacities = {
        name: workstation_start_capacity(benchmark)
        for name, benchmark in human_benchmarks.items()
    }
    if args.human_capacity_points:
        if args.human_capacity_points < 2:
            parser.error("--human-capacity-points must be at least 2")
        human_counts = {
            name: capacity_spaced_counts(
                human_capacities[name], points=args.human_capacity_points
            )
            for name in human_benchmarks
        }
    else:
        human_counts = {
            "alley": parse_counts(args.alley_counts),
            "plaza": parse_counts(args.plaza_counts),
        }
    grids = {
        "alley": {
            "benchmark": human_benchmarks["alley"],
            "counts": human_counts["alley"],
        },
        "plaza": {
            "benchmark": human_benchmarks["plaza"],
            "counts": human_counts["plaza"],
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
        grids[args.lorr_sortation_name] = {
            "benchmark": args.lorr_sortation_benchmark,
            "counts": lorr_sortation_counts,
        }
    lorr_sortation_medium_counts = parse_counts(args.lorr_sortation_medium_counts)
    if lorr_sortation_medium_counts:
        grids[args.lorr_sortation_medium_name] = {
            "benchmark": args.lorr_sortation_medium_benchmark,
            "counts": lorr_sortation_medium_counts,
        }
    lorr_sortation_large_counts = parse_counts(args.lorr_sortation_large_counts)
    if lorr_sortation_large_counts:
        grids[args.lorr_sortation_large_name] = {
            "benchmark": args.lorr_sortation_large_benchmark,
            "counts": lorr_sortation_large_counts,
        }
    grids = {name: config for name, config in grids.items() if config["counts"]}
    for config in grids.values():
        config["benchmark_fingerprints"] = benchmark_fingerprints(config["benchmark"])
    invalid_plaza_counts = plaza_counts_below_floor(grids.get("plaza", {}).get("counts", []))
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
            "pickup_layout_seed": args.pickup_layout_seed,
            "simulation_time": args.simulation_time,
            "planning_window": args.planning_window,
            "simulation_window": args.simulation_window,
            "service_time": args.service_time,
            **PRESSURE_DEFINITION,
            "pressure_threshold": args.pressure_threshold,
            "pressure_privileged_inbound_count": args.pressure_k,
            "lra_fallback_enabled": args.lra_fallback,
            "native_failures_only": not args.lra_fallback,
            "commitment_repair": args.commitment_repair,
            "pibt_random_tiebreak": args.pibt_random_tiebreak,
            "continue_on_traffic_jam": args.continue_on_traffic_jam,
            "cutoff_time": args.cutoff_time,
            "process_timeout": args.process_timeout,
            "jobs": args.jobs,
            "screen": args.screen,
            "keep_paths": args.keep_paths,
            "human_count_ladder": {
                "rule": "capacity_spaced_below_valid_start_limit" if args.human_capacity_points else "explicit",
                "start": 20 if args.human_capacity_points else None,
                "points": args.human_capacity_points or None,
                "valid_start_capacity": human_capacities,
            },
            "stop_at_first_all_failed_count": args.stop_at_first_all_failed_count,
            "platform": platform.platform(),
            "logical_cpu_count": os.cpu_count(),
            "binary": str(binary.resolve()),
            "binary_sha256": binary_sha256,
            **source_provenance,
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
            **source_provenance,
            "benchmark_fingerprints": grids[map_name]["benchmark_fingerprints"],
            "map": map_name,
            "agent_count": agent_count,
            "method": method_name,
            "seed": seed,
            "pickup_layout_seed": args.pickup_layout_seed,
            "solver": method_config["solver"],
            "heuristic_backend": "exact_uint16_mmap" if method_config["solver"] in {"PIBT", "PIBT2"} and "movingai_map" in json.loads(benchmark.read_text()) else "exact_in_memory",
            "station_policy": method_config.get("station_policy", "vanilla"),
            "pibt_policy": method_config.get("pibt_policy", ""),
            "simulation_time": args.simulation_time,
            "planning_window": args.planning_window,
            "simulation_window": args.simulation_window,
            "service_time": args.service_time,
            **PRESSURE_DEFINITION,
            "pressure_threshold": args.pressure_threshold,
            "pressure_privileged_inbound_count": args.pressure_k,
            "lra_fallback_enabled": args.lra_fallback,
            "native_failures_only": not args.lra_fallback,
            "commitment_repair": args.commitment_repair,
            "cutoff_time": args.cutoff_time,
            "process_timeout": args.process_timeout,
            "continue_on_traffic_jam": args.continue_on_traffic_jam,
        }
        if "pibt_policy" in method_config:
            run_signature.update(
                {
                    "pibt_random_tiebreak": args.pibt_random_tiebreak,
                    "pibt_candidate_randomness": PIBT_RANDOMNESS,
                }
            )

        existing = load_status(status_path)
        if not args.force and status_is_reusable(existing, run_signature):
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
            "pickup_layout_seed": args.pickup_layout_seed,
            "run_signature": run_signature,
            "output_dir": str(cell_dir),
        }
        started_at_unix = time.time()
        started_at_monotonic = time.monotonic()
        write_json(
            status_path,
            {
                **status_base,
                "status": "running",
                "failure_reason": "",
                "return_code": None,
                "started_at_unix": started_at_unix,
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
            "--pressure_k", str(args.pressure_k),
            "--pressure_threshold", str(args.pressure_threshold),
            "--cutoffTime", str(args.cutoff_time),
            "--seed", str(seed),
            "--screen", str(args.screen),
            "--stop_at_traffic_jam", "false" if args.continue_on_traffic_jam else "true",
            "--native_failures_only", str(not args.lra_fallback).lower(),
            "--commitment_repair", str(args.commitment_repair).lower(),
            "--output", str(cell_dir),
        ]
        if "pibt_policy" in method_config:
            cmd.extend(["--pibt_policy", method_config["pibt_policy"]])
            cmd.extend(["--pibt_random_tiebreak", str(args.pibt_random_tiebreak).lower()])
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
        finished_at_unix = time.time()
        write_json(
            status_path,
            {
                **status_base,
                "status": status,
                "failure_reason": failure_reason,
                "return_code": return_code,
                "started_at_unix": started_at_unix,
                "finished_at_unix": finished_at_unix,
                "wall_time_seconds": time.monotonic() - started_at_monotonic,
            },
        )
        if status == "clean" and not args.keep_paths:
            (cell_dir / "paths.txt").unlink(missing_ok=True)
        log(f"[done] {map_name} {agent_count} {method_name} seed {seed} -> {status}")

    evaluated_grids: dict[str, list[int]] = {name: [] for name in grids}
    evaluated_method_grids: dict[str, dict[str, list[int]]] = {
        name: {method_name: [] for method_name in args.methods}
        for name in grids
    }
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        if args.stop_at_first_all_failed_count:
            def run_method_ladder(
                map_name: str, counts: list[int], method_name: str
            ) -> None:
                for agent_count in counts:
                    futures = [
                        pool.submit(
                            run_case, map_name, agent_count, method_name, seed
                        )
                        for seed in seeds
                    ]
                    for future in futures:
                        future.result()
                    evaluated_method_grids[map_name][method_name].append(agent_count)
                    statuses = [
                        load_status(
                            root / map_name / f"agents_{agent_count}" /
                            method_name / f"seed_{seed}" / "status.json"
                        )
                        for seed in seeds
                    ]
                    if all_cells_failed(statuses):
                        log(
                            f"[terminal] {map_name} {agent_count} "
                            f"{method_name}: all seeds failed"
                        )
                        return

            ladder_count = len(grids) * len(args.methods)
            with ThreadPoolExecutor(max_workers=ladder_count) as ladder_pool:
                ladder_futures = [
                    ladder_pool.submit(
                        run_method_ladder,
                        map_name,
                        config["counts"],
                        method_name,
                    )
                    for map_name, config in grids.items()
                    for method_name in args.methods
                ]
                for future in ladder_futures:
                    future.result()
            for map_name, method_grids in evaluated_method_grids.items():
                evaluated_grids[map_name] = sorted({
                    count
                    for method_counts in method_grids.values()
                    for count in method_counts
                })
        else:
            futures = []
            for map_name, config in grids.items():
                evaluated_grids[map_name].extend(config["counts"])
                for method_name in args.methods:
                    evaluated_method_grids[map_name][method_name].extend(
                        config["counts"]
                    )
                for agent_count in config["counts"]:
                    for method_name in args.methods:
                        for seed in seeds:
                            futures.append(
                                pool.submit(
                                    run_case, map_name, agent_count,
                                    method_name, seed
                                )
                            )
            for future in futures:
                future.result()

    manifest_path = root / "run_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["evaluated_grids"] = evaluated_grids
    manifest["evaluated_method_grids"] = evaluated_method_grids
    write_json(manifest_path, manifest)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
