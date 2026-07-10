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
    "mean_plan_ms",
    "plan_runtime_slope_ms_per_1000_steps",
    "termination_timestep",
    "terminated_by_traffic_jam",
    "terminated_by_commit_repair_failure",
    "terminated_by_solver_failure",
    "pressure_active_fraction",
    "traffic_jam_fraction",
    "pibt_inheritance_calls",
    "pibt_backtracks",
    "pibt_wait_fallbacks",
    "pibt_pressure_rank_changes",
    "pibt_regret_updates",
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
            values = [float(row.get(metric, "")) for row in clean_entries if row.get(metric, "") != ""]
            all_values = [float(row.get(metric, "")) for row in entries if row.get(metric, "") != ""]
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

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
