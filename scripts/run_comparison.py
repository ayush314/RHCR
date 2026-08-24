#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import platform
import signal
import subprocess
import threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path


METHODS = {
    "pbs_vanilla": {"solver": "PBS", "station_policy": "vanilla"},
    "pbs_phase_aware": {"solver": "PBS", "station_policy": "phase_aware"},
    "pbs_pressure_aware": {"solver": "PBS", "station_policy": "pressure_aware"},
    "pbs_pressure_single": {"solver": "PBS", "station_policy": "pressure_aware", "pressure_admission": "single"},
    "pbs_pressure_adaptive": {"solver": "PBS", "station_policy": "pressure_aware", "pressure_admission": "adaptive"},
    "pibt_vanilla": {"solver": "PIBT", "pibt_policy": "vanilla"},
    "pibt_phase_aware": {"solver": "PIBT", "pibt_policy": "phase_aware"},
    "pibt_pressure_aware": {"solver": "PIBT", "pibt_policy": "pressure_aware"},
    "pibt_pressure_single": {"solver": "PIBT", "pibt_policy": "pressure_aware", "pressure_admission": "single"},
    "pibt_pressure_adaptive": {"solver": "PIBT", "pibt_policy": "pressure_aware", "pressure_admission": "adaptive"},
}
PUBLICATION_METHODS = (
    "pbs_vanilla", "pbs_phase_aware", "pbs_pressure_aware",
    "pibt_vanilla", "pibt_phase_aware", "pibt_pressure_aware",
)
RUN_OUTPUT_FILES = (
    "config.txt",
    "paths.txt",
    "planning_runtime.csv",
    "queue_wait_observations.csv",
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
    parser.add_argument("--root", default=str(repo_root / "results" / "main_tau3_w20_h5_seed0to19"))
    parser.add_argument("--binary", default=str(repo_root / "lifelong"))
    parser.add_argument("--seed-start", type=int, default=6)
    parser.add_argument("--seed-count", type=int, default=20)
    parser.add_argument("--seed-list", type=parse_seed_list)
    parser.add_argument("--pickup-layout-seed", type=int, default=1)
    parser.add_argument("--simulation-time", type=int, default=1000)
    parser.add_argument("--planning-window", type=int, default=20)
    parser.add_argument("--simulation-window", type=int, default=5)
    parser.add_argument("--service-time", type=int, default=3)
    parser.add_argument(
        "--pressure-threshold", type=int, default=None,
        help="Fixed trigger override; defaults to 1 for fixed profile and profile-managed for adaptive profile.",
    )
    parser.add_argument(
        "--pressure-profile",
        choices=("fixed", "prevalence_adaptive"),
        default="fixed",
    )
    parser.add_argument("--pressure-admission", choices=("single", "adaptive", "wide", "scale_adaptive"), default="adaptive")
    parser.add_argument("--pressure-cost-mode", choices=("fixed", "escalating", "occupancy_escalating", "priority_only"), default="fixed")
    parser.add_argument("--pressure-cost-scope", choices=("zone", "queue", "holding", "approach", "entry", "lookahead"), default="zone")
    parser.add_argument("--pressure-cost-activation", choices=("zone", "excess_wip", "outside_only", "progress_only", "wait_only", "incumbent_grace", "entry_only", "enter_only", "deeper_only", "busy_only"), default="zone")
    parser.add_argument("--pressure-population", choices=("all_phases", "inbound_only"), default="inbound_only")
    parser.add_argument("--pressure-zone-cost", type=float, default=1.0)
    parser.add_argument("--pressure-front-progress-cost", type=int, default=0)
    parser.add_argument("--pressure-exit-progress-cost", type=int, default=0)
    parser.add_argument("--pressure-ready-slot-priority", action="store_true", default=False)
    parser.add_argument("--no-pressure-ready-slot-priority", dest="pressure_ready_slot_priority", action="store_false")
    parser.add_argument("--pressure-inbound-limit", type=int, default=3)
    parser.add_argument("--pressure-cost-occupancy-threshold", type=int, default=3)
    parser.add_argument(
        "--pressure-cost-horizon", type=int, default=0,
        help="maximum local PBS soft pressure-cost horizon; 0 uses the full planning window",
    )
    parser.add_argument(
        "--pressure-cost-horizon-profile", choices=("fixed", "network_adaptive"),
        default="fixed",
        help="PBS pressure-cost horizon profile; network_adaptive uses one step during widespread pressure",
    )
    parser.add_argument(
        "--pressure-local-action-only", dest="pressure_local_action_only",
        action="store_true", default=False,
        help="apply PBS soft pressure cost only to agents currently adjacent to the pressured zone",
    )
    parser.add_argument(
        "--no-pressure-local-action-only", dest="pressure_local_action_only",
        action="store_false",
    )
    parser.add_argument(
        "--pressure-front-runner-priority", dest="pressure_front_runner_priority",
        action="store_true", default=False,
        help="let the pressure front runner win PBS same-station conflicts",
    )
    parser.add_argument(
        "--no-pressure-front-runner-priority", dest="pressure_front_runner_priority",
        action="store_false",
    )
    parser.add_argument(
        "--pressure-front-runner-zone-only", dest="pressure_front_runner_zone_only",
        action="store_true", default=False,
        help="limit PBS front-runner promotion to conflicts touching the station zone",
    )
    parser.add_argument(
        "--no-pressure-front-runner-zone-only", dest="pressure_front_runner_zone_only",
        action="store_false",
    )
    parser.add_argument(
        "--pressure-front-runner-ready-priority", dest="pressure_front_runner_ready_priority",
        action="store_true", default=False,
        help="only promote the PBS front runner when it is within one move of the workstation",
    )
    parser.add_argument(
        "--no-pressure-front-runner-ready-priority", dest="pressure_front_runner_ready_priority",
        action="store_false",
    )
    parser.add_argument("--pressure-lookahead-radius", type=int, default=0)
    parser.add_argument(
        "--pressure-lookahead-profile", choices=("fixed", "scale_adaptive"),
        default="fixed",
    )
    parser.add_argument("--pressure-lookahead-min-agents-per-station", type=int, default=40)
    parser.add_argument("--pressure-lookahead-cost", type=int, default=1)
    parser.add_argument("--pibt-network-pressure-fraction", type=int, default=25)
    parser.add_argument("--pibt-network-pressure-min-agents-per-station", type=int, default=0)
    parser.add_argument(
        "--pibt-global-front-runner-priority", dest="pibt_global_front_runner_priority",
        action="store_true", default=False,
        help="allow pressure front runners to precede unrelated PIBT agents globally",
    )
    parser.add_argument(
        "--no-pibt-global-front-runner-priority", dest="pibt_global_front_runner_priority",
        action="store_false",
        help="retain same-station front-runner ordering but disable global PIBT promotion",
    )
    parser.add_argument("--pibt-assignment-budget-factor", type=int, default=90)
    parser.add_argument("--pibt-pressure-assignment-extension-factor", type=int, default=20)
    parser.add_argument("--pibt-front-runner-priority", dest="pibt_front_runner_priority", action="store_true", default=False, help="let selected pressure front runners precede native PIBT priority ordering")
    parser.add_argument("--no-pibt-front-runner-priority", dest="pibt_front_runner_priority", action="store_false", help="keep native PIBT dynamic priority ordering while retaining pressure action ranking")
    parser.add_argument("--pibt-front-runner-ready-priority", dest="pibt_front_runner_ready_priority", action="store_true", default=False, help="prioritize a ready pressure front runner above unrelated agents")
    parser.add_argument("--no-pibt-front-runner-ready-priority", dest="pibt_front_runner_ready_priority", action="store_false", help="disable ready pressure front-runner priority for an ablation")
    parser.add_argument("--allow-fallbacks", action="store_true", help="Exploratory only: enable PBS fallback and commitment repair.")
    parser.add_argument("--pibt-random-tiebreak", dest="pibt_random_tiebreak", action="store_true")
    parser.add_argument("--no-pibt-random-tiebreak", dest="pibt_random_tiebreak", action="store_false")
    parser.set_defaults(pibt_random_tiebreak=True)
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

    if args.pressure_threshold is None:
        args.pressure_threshold = -1 if args.pressure_profile == "prevalence_adaptive" else 1

    if not 1 <= args.pibt_network_pressure_fraction <= 100:
        parser.error("--pibt-network-pressure-fraction must be between 1 and 100")
    if args.pibt_network_pressure_min_agents_per_station < 0:
        parser.error("--pibt-network-pressure-min-agents-per-station must be nonnegative")
    if args.pressure_lookahead_min_agents_per_station < 1:
        parser.error("--pressure-lookahead-min-agents-per-station must be positive")
    if args.pressure_cost_horizon < 0:
        parser.error("--pressure-cost-horizon must be nonnegative")

    root = Path(args.root)
    root.mkdir(parents=True, exist_ok=True)
    binary = Path(args.binary).resolve()
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
            "pressure_threshold": args.pressure_threshold,
            "pressure_profile": args.pressure_profile,
            "pressure_admission": args.pressure_admission,
            "pressure_cost_mode": args.pressure_cost_mode,
            "pressure_cost_scope": args.pressure_cost_scope,
            "pressure_cost_activation": args.pressure_cost_activation,
            "pressure_population": args.pressure_population,
            "pressure_zone_cost": args.pressure_zone_cost,
            "pressure_front_progress_cost": args.pressure_front_progress_cost,
            "pressure_exit_progress_cost": args.pressure_exit_progress_cost,
            "pressure_ready_slot_priority": args.pressure_ready_slot_priority,
            "pressure_cost_rule": (
                "priority_only_no_soft_cost"
                if args.pressure_cost_mode == "priority_only"
                else "base_plus_one_above_local_threshold"
                if args.pressure_cost_mode == "escalating"
                else "base_plus_one_at_two_thirds_occupancy"
                if args.pressure_cost_mode == "occupancy_escalating"
                else "base"
            ),
            "pbs_pressure_branch_rule": (
                "phase_protection_then_front_runner"
                if args.pressure_front_runner_priority
                else "phase_protection_only"
            ),
            "pbs_soft_cost_in_high_level_objective": False,
            "pressure_inbound_limit": args.pressure_inbound_limit,
            "pressure_cost_occupancy_threshold": args.pressure_cost_occupancy_threshold,
            "pressure_cost_horizon": args.pressure_cost_horizon,
            "pressure_cost_horizon_profile": args.pressure_cost_horizon_profile,
            "pressure_local_action_only": args.pressure_local_action_only,
            "pressure_front_runner_priority": args.pressure_front_runner_priority,
            "pressure_front_runner_zone_only": args.pressure_front_runner_zone_only,
            "pressure_front_runner_ready_priority": args.pressure_front_runner_ready_priority,
            "pressure_lookahead_radius": args.pressure_lookahead_radius,
            "pressure_lookahead_profile": args.pressure_lookahead_profile,
            "pressure_lookahead_min_agents_per_station": args.pressure_lookahead_min_agents_per_station,
            "pibt_network_pressure_fraction": args.pibt_network_pressure_fraction,
            "pibt_network_pressure_min_agents_per_station": args.pibt_network_pressure_min_agents_per_station,
            "pibt_global_front_runner_priority": args.pibt_global_front_runner_priority,
            "pibt_assignment_budget_factor": args.pibt_assignment_budget_factor,
            "pibt_pressure_assignment_extension_factor": args.pibt_pressure_assignment_extension_factor,
            "pibt_front_runner_priority": args.pibt_front_runner_priority,
            "pibt_front_runner_ready_priority": args.pibt_front_runner_ready_priority,
            "native_failures_only": not args.allow_fallbacks,
            "pibt_random_tiebreak": args.pibt_random_tiebreak,
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
            "pickup_layout_seed": args.pickup_layout_seed,
            "solver": method_config["solver"],
            "heuristic_backend": "exact_uint16_mmap" if method_config["solver"] == "PIBT" and "movingai_map" in json.loads(benchmark.read_text()) else "exact_in_memory",
            "station_policy": method_config.get("station_policy", "vanilla"),
            "pibt_policy": method_config.get("pibt_policy", ""),
            "simulation_time": args.simulation_time,
            "planning_window": args.planning_window,
            "simulation_window": args.simulation_window,
            "service_time": args.service_time,
            "pressure_threshold": args.pressure_threshold,
            "pressure_profile": args.pressure_profile,
            "pressure_admission": method_config.get("pressure_admission", args.pressure_admission),
            "pressure_cost_mode": args.pressure_cost_mode,
            "pressure_cost_scope": args.pressure_cost_scope,
            "pressure_cost_activation": args.pressure_cost_activation,
            "pressure_population": args.pressure_population,
            "pressure_zone_cost": args.pressure_zone_cost,
            "pressure_front_progress_cost": args.pressure_front_progress_cost,
            "pressure_exit_progress_cost": args.pressure_exit_progress_cost,
            "pressure_ready_slot_priority": args.pressure_ready_slot_priority,
            "pressure_cost_rule": (
                "priority_only_no_soft_cost"
                if args.pressure_cost_mode == "priority_only"
                else "base_plus_one_above_local_threshold"
                if args.pressure_cost_mode == "escalating"
                else "base_plus_one_at_two_thirds_occupancy"
                if args.pressure_cost_mode == "occupancy_escalating"
                else "base"
            ),
            "pbs_pressure_branch_rule": (
                "phase_protection_then_front_runner"
                if args.pressure_front_runner_priority
                else "phase_protection_only"
            ),
            "pbs_soft_cost_in_high_level_objective": False,
            "pressure_inbound_limit": args.pressure_inbound_limit,
            "pressure_cost_occupancy_threshold": args.pressure_cost_occupancy_threshold,
            "pressure_cost_horizon": args.pressure_cost_horizon,
            "pressure_cost_horizon_profile": args.pressure_cost_horizon_profile,
            "pressure_local_action_only": args.pressure_local_action_only,
            "pressure_front_runner_priority": args.pressure_front_runner_priority,
            "pressure_front_runner_zone_only": args.pressure_front_runner_zone_only,
            "pressure_front_runner_ready_priority": args.pressure_front_runner_ready_priority,
            "pressure_lookahead_radius": args.pressure_lookahead_radius,
            "pressure_lookahead_profile": args.pressure_lookahead_profile,
            "pressure_lookahead_min_agents_per_station": args.pressure_lookahead_min_agents_per_station,
            "pibt_network_pressure_fraction": args.pibt_network_pressure_fraction,
            "pibt_network_pressure_min_agents_per_station": args.pibt_network_pressure_min_agents_per_station,
            "pibt_global_front_runner_priority": args.pibt_global_front_runner_priority,
            "pibt_assignment_budget_factor": args.pibt_assignment_budget_factor,
            "pibt_pressure_assignment_extension_factor": args.pibt_pressure_assignment_extension_factor,
            "pibt_front_runner_priority": args.pibt_front_runner_priority,
            "pibt_front_runner_ready_priority": args.pibt_front_runner_ready_priority,
            "native_failures_only": not args.allow_fallbacks,
            "cutoff_time": args.cutoff_time,
            "process_timeout": args.process_timeout,
            "continue_on_traffic_jam": args.continue_on_traffic_jam,
        }
        if "pibt_policy" in method_config:
            run_signature.update(
                {
                    "pibt_random_tiebreak": args.pibt_random_tiebreak,
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
            "--pressure_admission", method_config.get("pressure_admission", args.pressure_admission),
            "--pressure_profile", args.pressure_profile,
            "--pressure_cost_mode", args.pressure_cost_mode,
            "--pressure_cost_scope", args.pressure_cost_scope,
            "--pressure_cost_activation", args.pressure_cost_activation,
            "--pressure_population", args.pressure_population,
            "--pressure_zone_cost", str(args.pressure_zone_cost),
            "--pressure_front_progress_cost", str(args.pressure_front_progress_cost),
            "--pressure_exit_progress_cost", str(args.pressure_exit_progress_cost),
            "--pressure_ready_slot_priority", str(args.pressure_ready_slot_priority).lower(),
            "--pressure_inbound_limit", str(args.pressure_inbound_limit),
            "--pressure_cost_occupancy_threshold", str(args.pressure_cost_occupancy_threshold),
            "--pressure_cost_horizon", str(args.pressure_cost_horizon),
            "--pressure_cost_horizon_profile", args.pressure_cost_horizon_profile,
            "--pressure_local_action_only", str(args.pressure_local_action_only).lower(),
            "--pressure_front_runner_priority", str(args.pressure_front_runner_priority).lower(),
            "--pressure_front_runner_zone_only", str(args.pressure_front_runner_zone_only).lower(),
            "--pressure_front_runner_ready_priority", str(args.pressure_front_runner_ready_priority).lower(),
            "--pressure_lookahead_radius", str(args.pressure_lookahead_radius),
            "--pressure_lookahead_profile", args.pressure_lookahead_profile,
            "--pressure_lookahead_min_agents_per_station", str(args.pressure_lookahead_min_agents_per_station),
            "--pibt_network_pressure_fraction", str(args.pibt_network_pressure_fraction),
            "--pibt_network_pressure_min_agents_per_station", str(args.pibt_network_pressure_min_agents_per_station),
            "--pibt_global_front_runner_priority", str(args.pibt_global_front_runner_priority).lower(),
            "--pibt_assignment_budget_factor", str(args.pibt_assignment_budget_factor),
            "--pibt_pressure_assignment_extension_factor", str(args.pibt_pressure_assignment_extension_factor),
            "--pibt_front_runner_priority", str(args.pibt_front_runner_priority).lower(),
            "--pibt_front_runner_ready_priority", str(args.pibt_front_runner_ready_priority).lower(),
            "--native_failures_only", str(not args.allow_fallbacks).lower(),
            "--output", str(cell_dir),
        ]
        if "pibt_policy" in method_config:
            cmd.extend(["--pibt_policy", method_config["pibt_policy"]])
            cmd.extend(["--pibt_random_tiebreak", str(args.pibt_random_tiebreak).lower()])
        if args.pressure_threshold is not None:
            cmd.extend(["--pressure_threshold", str(args.pressure_threshold)])

        log(f"[run] {map_name} {agent_count} {method_name} seed {seed}")
        timed_out = False
        return_code = 0
        with log_path.open("w") as log_file:
            process = subprocess.Popen(
                cmd,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            try:
                return_code = process.wait(timeout=args.process_timeout)
            except subprocess.TimeoutExpired:
                timed_out = True
                return_code = 124
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait()

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
