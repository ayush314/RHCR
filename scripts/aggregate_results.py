#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path


METHOD_ORDER = [
    "pbs_vanilla",
    "pbs_distance_age",
    "pbs_pressure_aware",
    "pibt_vanilla",
    "pibt_distance_age",
    "pibt_pressure",
]
MAP_ORDER = ["alley", "plaza", "lorr_warehouse_small"]
METRICS = [
    "service_rate",
    "queue_wait_p95",
    "queue_wait_km_p95",
    "active_queue_agents",
    "mean_plan_ms",
    "plan_runtime_slope_ms_per_1000_steps",
    "termination_timestep",
    "terminated_by_traffic_jam",
    "terminated_by_commit_repair_failure",
    "terminated_by_solver_failure",
    "pressure_active_fraction",
    "pressured_station_fraction",
    "mean_zone_occupancy_fraction",
    "traffic_jam_fraction",
    "pibt_inheritance_calls",
    "pibt_backtracks",
    "pibt_wait_fallbacks",
    "pibt_pressure_rank_changes",
    "pibt_regret_updates",
]
PAIRED_METRICS = [
    "service_rate",
    "queue_wait_km_p95",
    "mean_plan_ms",
    "pibt_wait_fallbacks",
]
PAIRED_COMPARISONS = {
    "pibt_pressure": ["pibt_vanilla", "pibt_distance_age"],
}
T_CRITICAL_95 = [
    0.0,
    12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262,
    2.228, 2.201, 2.179, 2.160, 2.145, 2.131, 2.120, 2.110, 2.101, 2.093,
    2.086, 2.080, 2.074, 2.069, 2.064, 2.060, 2.056, 2.052, 2.048, 2.045,
    2.042,
]


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


def metric_values(entries: list[dict[str, str | int | float]], metric: str) -> list[float]:
    values = [float(row.get(metric, "")) for row in entries if row.get(metric, "") != ""]
    if metric == "queue_wait_km_p95":
        values = [value for value in values if value >= 0]
    return values


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
                        reference_float = float(reference_value)
                        baseline_float = float(baseline_value)
                        if metric == "queue_wait_km_p95" and (reference_float < 0 or baseline_float < 0):
                            continue
                        reference_values.append(reference_float)
                        baseline_values.append(baseline_float)
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


def read_summary(path: Path) -> dict[str, float | str]:
    with path.open() as fh:
        row = next(csv.DictReader(fh))
    metrics: dict[str, float | str] = {}
    for metric in METRICS:
        value = row.get(metric, "")
        metrics[metric] = "" if value == "" else float(value)
    return metrics


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
        status = payload["status"]
        failure_reason = payload.get("failure_reason", "")
        termination_failures = (
            ("terminated_by_traffic_jam", "traffic_jam"),
            ("terminated_by_commit_repair_failure", "internal_repair_failure"),
            ("terminated_by_solver_failure", "solver_failure"),
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
    grids = {
        str(map_name): {int(count) for count in counts}
        for map_name, counts in manifest.get("grids", {}).items()
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


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Aggregate workstation comparison results.")
    parser.add_argument("--root", default=str(repo_root / "results" / "main_tau3_w20_h5_seed0to19"))
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
            values = metric_values(clean_entries, metric)
            all_values = metric_values(entries, metric)
            aggregate[metric] = "" if not values else mean(values)
            aggregate[f"{metric}_std"] = "" if not values else stddev(values)
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

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
