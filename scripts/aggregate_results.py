#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import random
from collections import defaultdict
from pathlib import Path


METHOD_ORDER = [
    "pbs_vanilla",
    "pbs_lead_aware",
    "pbs_departure_aware",
    "pbs_pressure_aware",
    "pibt_vanilla",
    "pibt_lead_aware",
    "pibt_departure_aware",
    "pibt_pressure_aware",
    "pibt2_vanilla",
    "pibt2_lead_aware",
    "pibt2_departure_aware",
    "pibt2_pressure_aware",
    "pibt_legacy_vanilla",
    "pibt_legacy_lead_aware",
    "pibt_legacy_departure_aware",
    "pibt_legacy_pressure_aware",
]
MAP_ORDER = [
    "alley",
    "plaza",
    "lorr_warehouse_small",
    "lorr_sortation_small",
    "lorr_sortation_medium",
    "lorr_sortation_large",
]
METRICS = [
    "service_rate",
    "observed_service_rate",
    "queue_wait_p95",
    "queue_wait_km_p95",
    "queue_wait_rmst100",
    "queue_wait_survival_20",
    "queue_wait_survival_50",
    "queue_wait_survival_100",
    "mean_queue_region_occupancy_per_station",
    "active_queue_agents",
    "mean_plan_ms",
    "plan_runtime_p95_ms",
    "plan_runtime_max_ms",
    "mean_amortized_ms_per_step",
    "p95_amortized_ms_per_step",
    "max_amortized_ms_per_step",
    "plan_runtime_slope_ms_per_1000_steps",
    "termination_timestep",
    "clean_completion",
    "time_to_stall",
    "stall_event",
    "distance_per_completed_service",
    "peak_rss_kb",
    "terminated_by_traffic_jam",
    "terminated_by_commit_repair_failure",
    "terminated_by_solver_failure",
    "terminated_by_fallback_failure",
    "pressure_active_fraction",
    "pressured_station_fraction",
    "mean_queue_region_occupancy_fraction",
    "traffic_jam_fraction",
    "lra_fallback_episodes",
    "lra_fallback_wait_commands",
    "pbs_pressure_cost_evaluations",
    "pbs_pressure_cost_applications",
    "pbs_pressure_cost_application_fraction",
    "pibt_inheritance_calls",
    "pibt_backtracks",
    "pibt_wait_fallbacks",
    "pibt_wait_fallback_rate_per_1000_agent_steps",
    "pibt_pressure_rank_changes",
]
PAIRED_METRICS = [
    "service_rate",
    "queue_wait_rmst100",
    "queue_wait_km_p95",
    "mean_plan_ms",
    "plan_runtime_p95_ms",
    "plan_runtime_max_ms",
    "pibt_wait_fallbacks",
    "pibt_wait_fallback_rate_per_1000_agent_steps",
]
PAIRED_COMPARISONS = {
    "pbs_lead_aware": ["pbs_vanilla"],
    "pbs_departure_aware": ["pbs_vanilla"],
    "pbs_pressure_aware": ["pbs_lead_aware", "pbs_vanilla"],
    "pibt_lead_aware": ["pibt_vanilla"],
    "pibt_departure_aware": ["pibt_vanilla"],
    "pibt_pressure_aware": ["pibt_lead_aware", "pibt_vanilla"],
    "pibt2_lead_aware": ["pibt2_vanilla"],
    "pibt2_departure_aware": ["pibt2_vanilla"],
    "pibt2_pressure_aware": ["pibt2_lead_aware", "pibt2_vanilla"],
}
T_CRITICAL_95 = [
    0.0,
    12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262,
    2.228, 2.201, 2.179, 2.160, 2.145, 2.131, 2.120, 2.110, 2.101, 2.093,
    2.086, 2.080, 2.074, 2.069, 2.064, 2.060, 2.056, 2.052, 2.048, 2.045,
    2.042,
]

HIERARCHICAL_METRICS = [
    "clean_completion", "time_to_stall", "service_rate", "queue_wait_rmst100",
    "queue_wait_survival_50", "mean_queue_region_occupancy_per_station",
    "mean_plan_ms", "peak_rss_kb",
]


def hierarchical_effect_rows(
    rows: list[dict[str, str | int | float]], iterations: int = 2000,
) -> list[dict[str, str | int | float]]:
    rng = random.Random(314159)
    by_cell: dict[tuple[str, int, str], dict[tuple[int, int], dict]] = defaultdict(dict)
    for row in rows:
        key = (str(row["map"]), int(row["agent_count"]), str(row["method"]))
        pair_key = (int(row.get("pickup_layout_seed", 1)), int(row["seed"]))
        by_cell[key][pair_key] = row

    output = []
    for reference, baselines in PAIRED_COMPARISONS.items():
        conditions = {(key[0], key[1]) for key in by_cell if key[2] == reference}
        for map_name, agent_count in sorted(conditions, key=lambda item: (map_sort_key(item[0]), item[1])):
            reference_rows = by_cell.get((map_name, agent_count, reference), {})
            for baseline in baselines:
                baseline_rows = by_cell.get((map_name, agent_count, baseline), {})
                paired_keys = sorted(set(reference_rows) & set(baseline_rows))
                layouts = sorted({key[0] for key in paired_keys})
                if not layouts:
                    continue
                keys_by_layout = {
                    layout: [key for key in paired_keys if key[0] == layout]
                    for layout in layouts
                }
                for metric in HIERARCHICAL_METRICS:
                    valid = [
                        key for key in paired_keys
                        if reference_rows[key].get(metric, "") != ""
                        and baseline_rows[key].get(metric, "") != ""
                    ]
                    if not valid:
                        continue
                    differences = [
                        float(reference_rows[key][metric]) - float(baseline_rows[key][metric])
                        for key in valid
                    ]
                    bootstrap = []
                    for _ in range(iterations):
                        sampled = []
                        for layout in rng.choices(layouts, k=len(layouts)):
                            candidates = [key for key in keys_by_layout[layout] if key in valid]
                            if candidates:
                                sampled.extend(rng.choices(candidates, k=len(candidates)))
                        if sampled:
                            bootstrap.append(mean([
                                float(reference_rows[key][metric]) -
                                float(baseline_rows[key][metric]) for key in sampled
                            ]))
                    bootstrap.sort()
                    output.append({
                        "map": map_name,
                        "agent_count": agent_count,
                        "reference": reference,
                        "baseline": baseline,
                        "metric": metric,
                        "layout_seed_count": len(layouts),
                        "paired_run_count": len(valid),
                        "mean_difference": mean(differences),
                        "bootstrap_ci95_low": percentile(bootstrap, 2.5),
                        "bootstrap_ci95_high": percentile(bootstrap, 97.5),
                    })
    return output


def method_sort_key(name: str) -> tuple[int, str]:
    try:
        return (METHOD_ORDER.index(name), name)
    except ValueError:
        return (len(METHOD_ORDER), name)


def map_sort_key(name: str) -> tuple[int, str]:
    try:
        return (MAP_ORDER.index(name), name)
    except ValueError:
        return (len(MAP_ORDER), name)


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else float("nan")


def stddev(values: list[float]) -> float:
    if not values:
        return float("nan")
    if len(values) == 1:
        return 0.0
    mu = mean(values)
    return math.sqrt(sum((value - mu) ** 2 for value in values) / (len(values) - 1))


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    rank = (pct / 100.0) * (len(ordered) - 1)
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return ordered[lower]
    fraction = rank - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def kaplan_meier_curve(
    entries: list[dict[str, str | int | float]],
    time_key: str = "time_to_stall",
    event_key: str = "stall_event",
) -> list[dict[str, float | int]]:
    observations = sorted(
        (float(row[time_key]), int(float(row[event_key])))
        for row in entries
        if row.get(time_key, "") != "" and row.get(event_key, "") != ""
    )
    if not observations:
        return []
    survival = 1.0
    output: list[dict[str, float | int]] = [{
        "timestep": 0.0,
        "at_risk": len(observations),
        "events": 0,
        "censored": 0,
        "survival_probability": survival,
    }]
    for timestep in sorted({time for time, _ in observations}):
        at_risk = sum(time >= timestep for time, _ in observations)
        events = sum(time == timestep and event == 1 for time, event in observations)
        censored = sum(time == timestep and event == 0 for time, event in observations)
        if events:
            survival *= 1.0 - events / at_risk
        output.append({
            "timestep": timestep,
            "at_risk": at_risk,
            "events": events,
            "censored": censored,
            "survival_probability": survival,
        })
    return output


def metric_values(entries: list[dict[str, str | int | float]], metric: str) -> list[float]:
    return [float(row.get(metric, "")) for row in entries if row.get(metric, "") != ""]


def cap_legacy_queue_wait_at_horizon(
    rows: list[dict[str, str | int | float]],
) -> None:
    for row in rows:
        value = row.get("queue_wait_km_p95", "")
        horizon = row.get("termination_timestep", "")
        if value == "" or horizon == "":
            continue
        if float(value) < 0:
            row["queue_wait_km_p95"] = float(horizon)


def ci95_halfwidth(values: list[float]) -> float:
    if len(values) < 2:
        return float("nan")
    degrees_of_freedom = len(values) - 1
    critical = T_CRITICAL_95[degrees_of_freedom] if degrees_of_freedom < len(T_CRITICAL_95) else 1.96
    return critical * stddev(values) / math.sqrt(len(values))


def paired_comparison_rows(
    combined_rows: list[dict[str, str | int | float]],
) -> list[dict[str, str | int | float]]:
    clean_by_cell: dict[tuple[str, int, str, int], dict[str, str | int | float]] = {}
    for row in combined_rows:
        if row["status"] != "clean":
            continue
        key = (str(row["map"]), int(row["agent_count"]), str(row["method"]), int(row["seed"]))
        clean_by_cell[key] = row

    map_counts = sorted(
        {(key[0], key[1]) for key in clean_by_cell},
        key=lambda item: (map_sort_key(item[0]), item[1]),
    )
    output: list[dict[str, str | int | float]] = []
    for map_name, agent_count in map_counts:
        for reference, baselines in PAIRED_COMPARISONS.items():
            for baseline in baselines:
                for metric in PAIRED_METRICS:
                    reference_by_seed = {
                        seed: row
                        for (row_map, row_count, method, seed), row in clean_by_cell.items()
                        if row_map == map_name and row_count == agent_count and method == reference
                    }
                    baseline_by_seed = {
                        seed: row
                        for (row_map, row_count, method, seed), row in clean_by_cell.items()
                        if row_map == map_name and row_count == agent_count and method == baseline
                    }
                    reference_values: list[float] = []
                    baseline_values: list[float] = []
                    for seed in sorted(reference_by_seed.keys() & baseline_by_seed.keys()):
                        reference_value = reference_by_seed[seed].get(metric, "")
                        baseline_value = baseline_by_seed[seed].get(metric, "")
                        if reference_value == "" or baseline_value == "":
                            continue
                        reference_values.append(float(reference_value))
                        baseline_values.append(float(baseline_value))
                    if not reference_values:
                        continue
                    differences = [
                        reference_value - baseline_value
                        for reference_value, baseline_value in zip(reference_values, baseline_values)
                    ]
                    baseline_mean = mean(baseline_values)
                    output.append(
                        {
                            "map": map_name,
                            "agent_count": agent_count,
                            "reference": reference,
                            "baseline": baseline,
                            "metric": metric,
                            "paired_seed_count": len(differences),
                            "reference_mean": mean(reference_values),
                            "baseline_mean": baseline_mean,
                            "mean_difference": mean(differences),
                            "relative_difference_percent": "" if baseline_mean == 0 else 100 * mean(differences) / baseline_mean,
                            "difference_ci95_halfwidth": "" if len(differences) < 2 else ci95_halfwidth(differences),
                        }
                    )
    return output


def paired_root_comparison_rows(
    reference_rows: list[dict[str, str | int | float]],
    baseline_rows: list[dict[str, str | int | float]],
    reference_label: str,
    baseline_label: str,
) -> list[dict[str, str | int | float]]:
    def group_rows(
        rows: list[dict[str, str | int | float]],
    ) -> dict[tuple[str, int, str], list[dict[str, str | int | float]]]:
        grouped: dict[tuple[str, int, str], list[dict[str, str | int | float]]] = defaultdict(list)
        for row in rows:
            grouped[(str(row["map"]), int(row["agent_count"]), str(row["method"]))].append(row)
        return grouped

    reference_groups = group_rows(reference_rows)
    baseline_groups = group_rows(baseline_rows)
    cells = sorted(
        reference_groups.keys() & baseline_groups.keys(),
        key=lambda item: (map_sort_key(item[0]), item[1], method_sort_key(item[2])),
    )
    output: list[dict[str, str | int | float]] = []
    for map_name, agent_count, method in cells:
        reference_entries = reference_groups[(map_name, agent_count, method)]
        baseline_entries = baseline_groups[(map_name, agent_count, method)]
        reference_clean = {
            int(row["seed"]): row for row in reference_entries if row["status"] == "clean"
        }
        baseline_clean = {
            int(row["seed"]): row for row in baseline_entries if row["status"] == "clean"
        }
        paired_seeds = sorted(reference_clean.keys() & baseline_clean.keys())
        for metric in PAIRED_METRICS:
            reference_values: list[float] = []
            baseline_values: list[float] = []
            for seed in paired_seeds:
                reference_value = reference_clean[seed].get(metric, "")
                baseline_value = baseline_clean[seed].get(metric, "")
                if reference_value == "" or baseline_value == "":
                    continue
                reference_values.append(float(reference_value))
                baseline_values.append(float(baseline_value))
            if not reference_values:
                continue
            differences = [
                reference_value - baseline_value
                for reference_value, baseline_value in zip(reference_values, baseline_values)
            ]
            baseline_mean = mean(baseline_values)
            output.append(
                {
                    "map": map_name,
                    "agent_count": agent_count,
                    "method": method,
                    "reference": reference_label,
                    "baseline": baseline_label,
                    "metric": metric,
                    "reference_seed_count": len(reference_entries),
                    "reference_clean_seed_count": len(reference_clean),
                    "baseline_seed_count": len(baseline_entries),
                    "baseline_clean_seed_count": len(baseline_clean),
                    "paired_seed_count": len(reference_values),
                    "reference_mean": mean(reference_values),
                    "baseline_mean": baseline_mean,
                    "mean_difference": mean(differences),
                    "relative_difference_percent": (
                        "" if baseline_mean == 0 else 100 * mean(differences) / baseline_mean
                    ),
                    "difference_ci95_halfwidth": (
                        "" if len(differences) < 2 else ci95_halfwidth(differences)
                    ),
                }
            )
    return output


def read_summary(path: Path) -> dict[str, float | str]:
    with path.open() as fh:
        row = next(csv.DictReader(fh))
    metrics: dict[str, float | str] = {}
    for metric in METRICS:
        value = row.get(metric, "")
        metrics[metric] = "" if value == "" else float(value)
    return metrics


def backfill_runtime_tail(metrics: dict[str, float | str], runtime_path: Path) -> None:
    if not runtime_path.exists():
        return
    with runtime_path.open() as fh:
        samples = [
            float(row["plan_ms"])
            for row in csv.DictReader(fh)
            if row.get("plan_ms", "") != ""
        ]
    if not samples:
        return
    if metrics.get("plan_runtime_p95_ms", "") == "":
        metrics["plan_runtime_p95_ms"] = percentile(samples, 95)
    if metrics.get("plan_runtime_max_ms", "") == "":
        metrics["plan_runtime_max_ms"] = max(samples)


def read_status_rows(root: Path) -> list[dict[str, str | int | float]]:
    combined_rows: list[dict[str, str | int | float]] = []
    for status_path in sorted(root.rglob("status.json")):
        payload = json.loads(status_path.read_text())
        if "map" not in payload or "method" not in payload or "agent_count" not in payload:
            continue
        summary_path = status_path.parent / "summary.csv"
        metrics = {metric: "" for metric in METRICS}
        if summary_path.exists():
            try:
                metrics = read_summary(summary_path)
            except Exception:
                pass
        try:
            backfill_runtime_tail(metrics, status_path.parent / "planning_runtime.csv")
        except (OSError, KeyError, ValueError):
            pass
        signature = payload.get("run_signature", {})
        fallback_count = metrics.get("pibt_wait_fallbacks", "")
        termination_timestep = metrics.get("termination_timestep", "")
        simulation_window = int(signature.get("simulation_window", 0))
        planning_window = int(signature.get("planning_window", 0))
        if (fallback_count != "" and termination_timestep != "" and
                simulation_window > 0 and planning_window > 0):
            episodes = max(1, math.ceil(float(termination_timestep) / simulation_window))
            agent_steps = episodes * int(payload["agent_count"]) * planning_window
            metrics["pibt_wait_fallback_rate_per_1000_agent_steps"] = (
                1000.0 * float(fallback_count) / agent_steps
            )
        status = payload["status"]
        failure_reason = payload.get("failure_reason", "")
        termination_failures = (
            ("terminated_by_traffic_jam", "traffic_jam"),
            ("terminated_by_commit_repair_failure", "internal_repair_failure"),
            ("terminated_by_solver_failure", "solver_failure"),
            ("terminated_by_fallback_failure", "fallback_failure"),
        )
        for metric, reason in termination_failures:
            if metrics.get(metric) == 1.0:
                status = "failed"
                failure_reason = reason
                break
        combined_rows.append(
            {
                "map": payload["map"],
                "agent_count": int(payload["agent_count"]),
                "method": payload["method"],
                "seed": int(payload.get("seed", 0)),
                "pickup_layout_seed": int(payload.get("pickup_layout_seed", 1)),
                "status": status,
                "failure_reason": failure_reason,
                **metrics,
                "output_dir": payload.get("output_dir", str(status_path.parent)),
            }
        )
    return combined_rows


def read_existing_combined(path: Path) -> list[dict[str, str]]:
    with path.open() as fh:
        return list(csv.DictReader(fh))


def filter_manifest_cells(
    rows: list[dict[str, str | int | float]],
    manifest_path: Path,
) -> list[dict[str, str | int | float]]:
    if not manifest_path.exists():
        return rows
    manifest = json.loads(manifest_path.read_text())
    methods = set(manifest.get("methods", []))
    seeds = {int(seed) for seed in manifest.get("seeds", [])}
    manifest_grids = manifest.get("evaluated_grids") or manifest.get("grids", {})
    grids = {
        str(map_name): {int(count) for count in counts}
        for map_name, counts in manifest_grids.items()
    }
    if not methods or not seeds or not grids:
        return rows
    return [
        row
        for row in rows
        if str(row["method"]) in methods
        and int(row["seed"]) in seeds
        and str(row["map"]) in grids
        and int(row["agent_count"]) in grids[str(row["map"])]
    ]


def derive_fallback_rate(
    rows: list[dict[str, str | int | float]],
    manifest_path: Path,
) -> None:
    if not manifest_path.exists():
        return
    manifest = json.loads(manifest_path.read_text())
    simulation_window = int(manifest.get("simulation_window", 0))
    planning_window = int(manifest.get("planning_window", 0))
    if simulation_window <= 0 or planning_window <= 0:
        return
    for row in rows:
        fallbacks = row.get("pibt_wait_fallbacks", "")
        termination_timestep = row.get("termination_timestep", "")
        if fallbacks == "" or termination_timestep == "":
            continue
        planning_episodes = max(1, math.ceil(float(termination_timestep) / simulation_window))
        planned_agent_steps = planning_episodes * int(row["agent_count"]) * planning_window
        row["pibt_wait_fallback_rate_per_1000_agent_steps"] = (
            1000.0 * float(fallbacks) / planned_agent_steps
        )


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Aggregate workstation comparison results.")
    parser.add_argument("--root", default=str(repo_root / "results" / "main_tau3_w20_h5_seed0to19"))
    parser.add_argument("--paired-baseline-root")
    parser.add_argument("--reference-label", default="reference")
    parser.add_argument("--baseline-label", default="baseline")
    parser.add_argument("--paired-root-output")
    args = parser.parse_args()

    root = Path(args.root)
    combined_path = root / "combined_summary.csv"
    combined_rows: list[dict[str, str | int | float]] = read_status_rows(root)
    write_combined = True
    if not combined_rows and combined_path.exists():
        combined_rows = read_existing_combined(combined_path)
        write_combined = False
    if not combined_rows:
        raise SystemExit(f"No status.json files or existing combined_summary.csv found under {root}")
    combined_rows = filter_manifest_cells(combined_rows, root / "run_manifest.json")
    if not combined_rows:
        raise SystemExit(f"No result cells under {root} match its run_manifest.json")
    derive_fallback_rate(combined_rows, root / "run_manifest.json")
    cap_legacy_queue_wait_at_horizon(combined_rows)

    combined_rows.sort(
        key=lambda row: (
            map_sort_key(str(row["map"])),
            int(row["agent_count"]),
            method_sort_key(str(row["method"])),
            int(row["seed"]),
        )
    )

    if write_combined:
        with combined_path.open("w", newline="") as fh:
            writer = csv.DictWriter(
                fh,
                fieldnames=[
                    "map",
                    "agent_count",
                    "method",
                    "seed",
                    "pickup_layout_seed",
                    "status",
                    "failure_reason",
                    *METRICS,
                    "output_dir",
                ],
            )
            writer.writeheader()
            writer.writerows(combined_rows)

    grouped: dict[tuple[str, int, str], list[dict[str, str | int | float]]] = defaultdict(list)
    for row in combined_rows:
        grouped[(str(row["map"]), int(row["agent_count"]), str(row["method"]))].append(row)

    aggregate_rows: list[dict[str, str | int | float]] = []
    for (map_name, agent_count, method_name), entries in sorted(
        grouped.items(),
        key=lambda item: (map_sort_key(item[0][0]), item[0][1], method_sort_key(item[0][2])),
    ):
        clean_entries = [row for row in entries if row["status"] == "clean"]
        failure_reasons = sorted(
            {
                str(row["failure_reason"])
                for row in entries
                if row["status"] != "clean" and row["failure_reason"]
            }
        )
        aggregate = {
            "map": map_name,
            "agent_count": agent_count,
            "method": method_name,
            "seed_count": len(entries),
            "clean_seed_count": len(clean_entries),
            "failed_seed_count": len(entries) - len(clean_entries),
            "all_clean": len(clean_entries) == len(entries),
            "failure_reasons": ";".join(failure_reasons),
        }
        for metric in METRICS:
            all_values = metric_values(entries, metric)
            aggregate[metric] = "" if not all_values else mean(all_values)
            aggregate[f"{metric}_std"] = "" if not all_values else stddev(all_values)
            aggregate[f"{metric}_all"] = "" if not all_values else mean(all_values)
            aggregate[f"{metric}_all_std"] = "" if not all_values else stddev(all_values)
        aggregate_rows.append(aggregate)

    with (root / "aggregate.csv").open("w", newline="") as fh:
        writer = csv.DictWriter(
            fh,
            fieldnames=[
                "map",
                "agent_count",
                "method",
                "seed_count",
                "clean_seed_count",
                "failed_seed_count",
                "all_clean",
                "failure_reasons",
                *METRICS,
                *(f"{metric}_std" for metric in METRICS),
                *(f"{metric}_all" for metric in METRICS),
                *(f"{metric}_all_std" for metric in METRICS),
            ],
        )
        writer.writeheader()
        writer.writerows(aggregate_rows)

    survival_rows = []
    for (map_name, agent_count, method_name), entries in sorted(
        grouped.items(),
        key=lambda item: (map_sort_key(item[0][0]), item[0][1], method_sort_key(item[0][2])),
    ):
        for point in kaplan_meier_curve(entries):
            survival_rows.append({
                "map": map_name,
                "agent_count": agent_count,
                "method": method_name,
                **point,
            })
    with (root / "stall_survival.csv").open("w", newline="") as fh:
        fieldnames = [
            "map", "agent_count", "method", "timestep", "at_risk",
            "events", "censored", "survival_probability",
        ]
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(survival_rows)

    paired_rows = paired_comparison_rows(combined_rows)
    with (root / "paired_comparison.csv").open("w", newline="") as fh:
        fieldnames = [
            "map",
            "agent_count",
            "reference",
            "baseline",
            "metric",
            "paired_seed_count",
            "reference_mean",
            "baseline_mean",
            "mean_difference",
            "relative_difference_percent",
            "difference_ci95_halfwidth",
        ]
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(paired_rows)

    hierarchical_rows = hierarchical_effect_rows(combined_rows)
    with (root / "hierarchical_effects.csv").open("w", newline="") as fh:
        fieldnames = [
            "map", "agent_count", "reference", "baseline", "metric",
            "layout_seed_count", "paired_run_count", "mean_difference",
            "bootstrap_ci95_low", "bootstrap_ci95_high",
        ]
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(hierarchical_rows)

    if args.paired_baseline_root:
        baseline_root = Path(args.paired_baseline_root)
        baseline_rows = read_status_rows(baseline_root)
        if not baseline_rows and (baseline_root / "combined_summary.csv").exists():
            baseline_rows = read_existing_combined(baseline_root / "combined_summary.csv")
        if not baseline_rows:
            raise SystemExit(f"No baseline result rows found under {baseline_root}")
        baseline_rows = filter_manifest_cells(baseline_rows, baseline_root / "run_manifest.json")
        derive_fallback_rate(baseline_rows, baseline_root / "run_manifest.json")
        cap_legacy_queue_wait_at_horizon(baseline_rows)
        comparison_rows = paired_root_comparison_rows(
            combined_rows,
            baseline_rows,
            args.reference_label,
            args.baseline_label,
        )
        output_path = (
            Path(args.paired_root_output)
            if args.paired_root_output
            else root / "paired_root_comparison.csv"
        )
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = [
            "map",
            "agent_count",
            "method",
            "reference",
            "baseline",
            "metric",
            "reference_seed_count",
            "reference_clean_seed_count",
            "baseline_seed_count",
            "baseline_clean_seed_count",
            "paired_seed_count",
            "reference_mean",
            "baseline_mean",
            "mean_difference",
            "relative_difference_percent",
            "difference_ci95_halfwidth",
        ]
        with output_path.open("w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(comparison_rows)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
