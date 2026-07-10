#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import statistics
import subprocess
from pathlib import Path


DEFAULT_CANDIDATES = [
    {"id": "control_s3", "pressure_profile": "severe", "inbound_limit": 3},
    {"id": "severe4", "pressure_profile": "severe", "inbound_limit": 4},
    {"id": "half4", "pressure_profile": "half", "inbound_limit": 4},
    {"id": "thirds4", "pressure_profile": "thirds", "inbound_limit": 4},
    {"id": "thirds5", "pressure_profile": "thirds", "inbound_limit": 5},
    {"id": "none3", "pressure_profile": "none", "inbound_limit": 3},
    {"id": "control_s3_h", "pressure_profile": "severe", "inbound_limit": 3, "hindrance": True},
    {"id": "severe4_h", "pressure_profile": "severe", "inbound_limit": 4, "hindrance": True},
    {"id": "half4_h", "pressure_profile": "half", "inbound_limit": 4, "hindrance": True},
    {"id": "thirds4_h", "pressure_profile": "thirds", "inbound_limit": 4, "hindrance": True},
]

DEFAULTS = {
    "pressure_profile": "thirds",
    "inbound_limit": 4,
    "pressure_threshold": 2,
    "entry_penalty": 2.0,
    "wait_penalty": 2.0,
    "exit_bonus": 1.0,
    "front_bonus": 3.0,
    "soft_collision_penalty": 0.0,
    "hindrance": True,
    "hindrance_scope": "inherited",
    "regret_iterations": 1,
    "regret_weight": 0.5,
    "regret_scope": "all",
    "random_tiebreak": True,
    "front_priority": True,
    "phase_priority": False,
}


def parse_int_list(value: str) -> list[int]:
    return [int(item.strip()) for item in value.split(",") if item.strip()]


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open() as handle:
        return list(csv.DictReader(handle))


def mean(values: list[float]) -> float:
    return statistics.fmean(values) if values else 0.0


def candidate_command(
    repo: Path,
    output: Path,
    candidate: dict,
    seeds: list[int],
    alley_counts: list[int],
    plaza_counts: list[int],
    args: argparse.Namespace,
) -> list[str]:
    config = DEFAULTS | candidate
    command = [
        "python3", str(repo / "scripts" / "run_comparison.py"),
        "--root", str(output),
        "--methods", "pibt_pressure",
        "--seed-list", ",".join(map(str, seeds)),
        "--simulation-time", str(args.simulation_time),
        "--planning-window", str(args.planning_window),
        "--simulation-window", str(args.simulation_window),
        "--service-time", str(args.service_time),
        "--alley-counts", ",".join(map(str, alley_counts)),
        "--plaza-counts", ",".join(map(str, plaza_counts)),
        "--pressure-threshold", str(config["pressure_threshold"]),
        "--pibt-pressure-entry-penalty", str(config["entry_penalty"]),
        "--pibt-pressure-inbound-limit", str(config["inbound_limit"]),
        "--pibt-pressure-profile", str(config["pressure_profile"]),
        "--pibt-wait-penalty", str(config["wait_penalty"]),
        "--pibt-exit-bonus", str(config["exit_bonus"]),
        "--pibt-front-bonus", str(config["front_bonus"]),
        "--pibt-soft-collision-penalty", str(config["soft_collision_penalty"]),
        "--pibt-regret-iterations", str(config["regret_iterations"]),
        "--pibt-regret-weight", str(config["regret_weight"]),
        "--pibt-regret-scope", str(config["regret_scope"]),
        "--pibt-random-tiebreak" if config["random_tiebreak"] else "--no-pibt-random-tiebreak",
        "--jobs", str(args.jobs),
        "--process-timeout", str(args.process_timeout),
    ]
    if config["hindrance"]:
        command.append("--pibt-hindrance")
        command.extend(["--pibt-hindrance-scope", str(config["hindrance_scope"])])
    else:
        command.append("--no-pibt-hindrance")
    if config["front_priority"]:
        command.append("--pibt-front-priority")
    else:
        command.append("--no-pibt-front-priority")
    if config["phase_priority"]:
        command.append("--pibt-phase-priority")
    else:
        command.append("--no-pibt-phase-priority")
    if args.force:
        command.append("--force")
    return command


def evaluate(
    candidate: dict,
    rows: list[dict[str, str]],
    baseline: dict[tuple[str, int, str, int], dict[str, str]],
) -> dict:
    gains_by_map: dict[str, list[float]] = {"alley": [], "plaza": []}
    queue_ratios: list[float] = []
    runtime_ratios: list[float] = []
    failed = 0
    fallback_total = 0.0
    groups: dict[tuple[str, int], list[dict[str, str]]] = {}

    for row in rows:
        if row["status"] != "clean":
            failed += 1
            continue
        groups.setdefault((row["map"], int(row["agent_count"])), []).append(row)
        fallback_total += float(row["pibt_wait_fallbacks"])

    for (map_name, count), candidate_rows in groups.items():
        seeds = [int(row["seed"]) for row in candidate_rows]
        vanilla = [baseline.get((map_name, count, "pibt_vanilla", seed)) for seed in seeds]
        distance_age = [baseline.get((map_name, count, "pibt_distance_age", seed)) for seed in seeds]
        if any(row is None for row in vanilla + distance_age):
            raise RuntimeError(f"Missing baseline rows for {(map_name, count, seeds)}")
        vanilla_clean = [row for row in vanilla if row["status"] == "clean"]
        distance_clean = [row for row in distance_age if row["status"] == "clean"]
        if not vanilla_clean or not distance_clean:
            raise RuntimeError(f"No clean baseline rows for {(map_name, count, seeds)}")

        service = mean([float(row["service_rate"]) for row in candidate_rows])
        vanilla_service = mean([float(row["service_rate"]) for row in vanilla_clean])
        distance_service = mean([float(row["service_rate"]) for row in distance_clean])
        reference = vanilla_clean if vanilla_service >= distance_service else distance_clean
        reference_service = max(vanilla_service, distance_service)
        gains_by_map[map_name].append(100.0 * (service / reference_service - 1.0))

        candidate_queue = mean([float(row["queue_wait_p95"]) for row in candidate_rows])
        reference_queue = max(mean([float(row["queue_wait_p95"]) for row in reference]), 1.0)
        candidate_runtime = mean([float(row["mean_plan_ms"]) for row in candidate_rows])
        reference_runtime = max(mean([float(row["mean_plan_ms"]) for row in reference]), 0.05)
        queue_ratios.append(candidate_queue / reference_queue)
        runtime_ratios.append(candidate_runtime / reference_runtime)

    gains = gains_by_map["alley"] + gains_by_map["plaza"]
    alley_gain = mean(gains_by_map["alley"])
    plaza_gain = mean(gains_by_map["plaza"])
    mean_gain = mean(gains)
    worst_gain = min(gains, default=-100.0)
    queue_excess = mean([max(0.0, ratio - 1.0) for ratio in queue_ratios]) * 100.0
    runtime_excess = mean([max(0.0, ratio - 1.0) for ratio in runtime_ratios]) * 100.0
    max_queue_ratio = max(queue_ratios, default=99.0)
    max_runtime_ratio = max(runtime_ratios, default=99.0)
    safe = (
        failed == 0
        and worst_gain >= -1.0
        and max_queue_ratio <= 1.30
        and max_runtime_ratio <= 1.60
    )
    score = (
        mean_gain
        + 0.25 * min(alley_gain, plaza_gain)
        - 0.08 * queue_excess
        - 0.03 * runtime_excess
        - 100.0 * failed
    )
    return {
        "candidate": candidate["id"],
        "safe": safe,
        "score": score,
        "mean_gain_pct": mean_gain,
        "alley_gain_pct": alley_gain,
        "plaza_gain_pct": plaza_gain,
        "worst_gain_pct": worst_gain,
        "mean_queue_ratio": mean(queue_ratios),
        "max_queue_ratio": max_queue_ratio,
        "mean_runtime_ratio": mean(runtime_ratios),
        "max_runtime_ratio": max_runtime_ratio,
        "failed_runs": failed,
        "fallback_total": fallback_total,
        "config": json.dumps(DEFAULTS | candidate, sort_keys=True),
    }


def write_leaderboard(root: Path, evaluations: list[dict]) -> None:
    fields = [key for key in evaluations[0] if key != "config"] + ["config"]
    with (root / "leaderboard.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(evaluations)

    lines = [
        "# PIBT Auto-Research Leaderboard",
        "",
        "| Candidate | Safe | Score | Mean gain | Alley | Plaza | Worst | Queue ratio | Runtime ratio |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in evaluations:
        lines.append(
            f"| {row['candidate']} | {row['safe']} | {row['score']:.3f} | "
            f"{row['mean_gain_pct']:.2f}% | {row['alley_gain_pct']:.2f}% | "
            f"{row['plaza_gain_pct']:.2f}% | {row['worst_gain_pct']:.2f}% | "
            f"{row['mean_queue_ratio']:.3f} | {row['mean_runtime_ratio']:.3f} |"
        )
    (root / "LEADERBOARD.md").write_text("\n".join(lines) + "\n")


def main() -> int:
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Screen pressure-PIBT preference configurations on both workstation maps.")
    parser.add_argument("--root", required=True, type=Path)
    parser.add_argument("--baseline-root", type=Path, default=repo / "results" / "pibt_primary_preference_h5_seed1to20")
    parser.add_argument("--candidates-json", type=Path)
    parser.add_argument("--seed-list", type=parse_int_list, default=[1, 2, 3])
    parser.add_argument("--alley-counts", type=parse_int_list, default=[20, 50, 80, 110, 140, 155])
    parser.add_argument("--plaza-counts", type=parse_int_list, default=[40, 100, 160, 220, 250, 280])
    parser.add_argument("--simulation-time", type=int, default=500)
    parser.add_argument("--planning-window", type=int, default=20)
    parser.add_argument("--simulation-window", type=int, default=5)
    parser.add_argument("--service-time", type=int, default=3)
    parser.add_argument("--jobs", type=int, default=8)
    parser.add_argument("--process-timeout", type=int, default=900)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    candidates = DEFAULT_CANDIDATES
    if args.candidates_json is not None:
        candidates = json.loads(args.candidates_json.read_text())
    ids = [candidate["id"] for candidate in candidates]
    if len(ids) != len(set(ids)):
        raise SystemExit("Candidate ids must be unique")

    args.root.mkdir(parents=True, exist_ok=True)
    baseline_rows = load_rows(args.baseline_root / "combined_summary.csv")
    baseline = {
        (row["map"], int(row["agent_count"]), row["method"], int(row["seed"])): row
        for row in baseline_rows
    }
    (args.root / "research_manifest.json").write_text(json.dumps({
        "candidates": candidates,
        "baseline_root": str(args.baseline_root),
        "seeds": args.seed_list,
        "alley_counts": args.alley_counts,
        "plaza_counts": args.plaza_counts,
        "simulation_time": args.simulation_time,
    }, indent=2, sort_keys=True) + "\n")

    evaluations = []
    for index, candidate in enumerate(candidates, start=1):
        output = args.root / candidate["id"]
        print(f"[{index}/{len(candidates)}] {candidate['id']}", flush=True)
        subprocess.run(candidate_command(
            repo, output, candidate, args.seed_list, args.alley_counts, args.plaza_counts, args
        ), cwd=repo, check=True)
        subprocess.run([
            "python3", str(repo / "scripts" / "aggregate_results.py"), "--root", str(output)
        ], cwd=repo, check=True)
        evaluation = evaluate(candidate, load_rows(output / "combined_summary.csv"), baseline)
        evaluations.append(evaluation)
        evaluations.sort(key=lambda row: (row["safe"], row["score"]), reverse=True)
        write_leaderboard(args.root, evaluations)
        print(
            f"  score={evaluation['score']:.3f} safe={evaluation['safe']} "
            f"alley={evaluation['alley_gain_pct']:.2f}% plaza={evaluation['plaza_gain_pct']:.2f}%",
            flush=True,
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
