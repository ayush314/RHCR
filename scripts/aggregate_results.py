#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path


METHOD_ORDER = ["pbs_vanilla", "pbs_distance_age", "pbs_pressure_aware"]
MAP_ORDER = ["alley", "plaza"]
METRICS = ["service_rate", "queue_wait_p95", "mean_plan_ms", "pressure_active_fraction"]


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


def read_summary(path: Path) -> dict[str, float]:
    with path.open() as fh:
        row = next(csv.DictReader(fh))
    return {metric: float(row[metric]) for metric in METRICS}


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Aggregate workstation comparison results.")
    parser.add_argument("--root", default=str(repo_root / "results" / "main_tau3_w20_h5_seed0to19"))
    args = parser.parse_args()

    root = Path(args.root)
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
        combined_rows.append(
            {
                "map": payload["map"],
                "agent_count": int(payload["agent_count"]),
                "method": payload["method"],
                "seed": int(payload.get("seed", 0)),
                "status": payload["status"],
                "failure_reason": payload.get("failure_reason", ""),
                **metrics,
                "output_dir": payload.get("output_dir", str(status_path.parent)),
            }
        )

    combined_rows.sort(
        key=lambda row: (
            map_sort_key(str(row["map"])),
            int(row["agent_count"]),
            method_sort_key(str(row["method"])),
            int(row["seed"]),
        )
    )

    with (root / "combined_summary.csv").open("w", newline="") as fh:
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
            values = [float(row[metric]) for row in clean_entries if row[metric] != ""]
            aggregate[metric] = "" if not values else mean(values)
            aggregate[f"{metric}_std"] = "" if not values else stddev(values)
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
            ],
        )
        writer.writeheader()
        writer.writerows(aggregate_rows)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
