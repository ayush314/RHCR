#!/usr/bin/env python3
"""Assemble frozen baselines and reengineered pressure results."""

import argparse
import csv
import re
from collections import defaultdict
from pathlib import Path

from aggregate_results import hierarchical_effect_rows


KEY_COLUMNS = (
    "map", "agent_count", "method", "seed_count", "clean_seed_count",
    "failed_seed_count", "clean_rate", "service_rate", "observed_service_rate",
    "queue_wait_rmst100", "queue_wait_p95", "queue_wait_km_p95",
    "mean_target_queue_occupancy_per_station", "active_queue_agents",
    "mean_plan_ms", "plan_runtime_p95_ms", "plan_runtime_max_ms",
    "mean_amortized_ms_per_step", "peak_rss_kb",
)


def read_rows(path: Path):
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def normalize_sortation_map(name: str):
    name = re.sub(r"_layout[234]$", "", name)
    return name if name.startswith("lorr_") else f"lorr_{name}"


def numeric(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def combine_layout_rows(rows):
    groups = defaultdict(list)
    for row in rows:
        row = dict(row)
        row["map"] = normalize_sortation_map(row["map"])
        groups[(row["map"], row["agent_count"], row["method"])].append(row)

    combined = []
    for (map_name, count, method), group in sorted(groups.items()):
        result = {"map": map_name, "agent_count": count, "method": method}
        total = sum(int(row["seed_count"]) for row in group)
        result["seed_count"] = str(total)
        result["clean_seed_count"] = str(sum(int(row["clean_seed_count"]) for row in group))
        result["failed_seed_count"] = str(sum(int(row["failed_seed_count"]) for row in group))
        for field in group[0]:
            if field in result or field.endswith("_std") or field in {"all_clean", "failure_reasons"}:
                continue
            values = []
            for row in group:
                value = numeric(row.get(field, ""))
                if value is not None:
                    values.append((value, int(row["seed_count"])))
            result[field] = "" if not values else str(
                sum(value * weight for value, weight in values) /
                sum(weight for _, weight in values)
            )
        result["clean_rate"] = str(int(result["clean_seed_count"]) / total)
        combined.append(result)
    return combined


def relabel(row, suffix):
    row = dict(row)
    if row["method"].endswith("pressure_aware"):
        row["method"] = f"{row['method']}_{suffix}"
    return row


def write_rows(path, rows):
    fields = []
    for row in rows:
        for field in row:
            if field not in fields:
                fields.append(field)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frozen-aggregate", type=Path, required=True)
    parser.add_argument("--new-root", type=Path, required=True)
    args = parser.parse_args()

    frozen = read_rows(args.frozen_aggregate)
    selected = []
    for row in frozen:
        human = row["map"] in {"alley", "plaza"}
        sortation_p20 = row["map"] in {
            "lorr_sortation_small_p20", "lorr_sortation_medium_p20",
            "lorr_sortation_large_p20",
        }
        if (human or sortation_p20) and row["method"].endswith("pressure_aware"):
            selected.append(relabel(row, "old"))

    baseline_sources = (
        ("human_common_baselines_current", {"alley"}, 0, 50),
        ("human_common_baselines_current_alley60", {"alley"}, 60, 60),
        ("human_common_baselines_current_alley_high", {"alley"}, 70, 70),
        ("human_common_baselines_current_alley80", {"alley"}, 80, 80),
        ("human_common_baselines_current_plaza", {"plaza"}, 0, 60),
        ("human_common_baselines_current_plaza80", {"plaza"}, 80, 80),
        ("human_common_baselines_current_plaza100", {"plaza"}, 100, 100),
        ("human_pibt_extended_baselines_current", {"alley", "plaza"}, 0, 10**9),
    )
    for name, maps, minimum, maximum in baseline_sources:
        selected.extend(
            row for row in read_rows(args.new_root / name / "aggregate.csv")
            if row["map"] in maps and minimum <= int(row["agent_count"]) <= maximum
        )

    for name in ("human_common", "human_pibt_extended"):
        selected.extend(
            relabel(row, "reengineered")
            for row in read_rows(args.new_root / name / "aggregate.csv")
        )

    baseline_rows = []
    pressure_rows = []
    for layout in (2, 3, 4):
        baseline_rows.extend(read_rows(
            args.new_root / f"sortation_p20_baselines_layout{layout}" / "aggregate.csv"))
        pressure_rows.extend(read_rows(
            args.new_root / f"sortation_p20_layout{layout}" / "aggregate.csv"))
    selected.extend(combine_layout_rows(baseline_rows))
    selected.extend(
        relabel(row, "reengineered") for row in combine_layout_rows(pressure_rows)
    )

    for row in selected:
        seeds = numeric(row.get("seed_count", ""))
        clean = numeric(row.get("clean_seed_count", ""))
        if seeds and clean is not None:
            row["clean_rate"] = str(clean / seeds)

    selected.sort(key=lambda row: (
        row["map"], int(row["agent_count"]), row["method"]))
    output = args.new_root / "comparison_all_metrics.csv"
    write_rows(output, selected)

    key_output = args.new_root / "comparison_key_metrics.csv"
    with key_output.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=KEY_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(selected)

    current_raw = []
    for name, maps, minimum, maximum in baseline_sources:
        current_raw.extend(
            row for row in read_rows(args.new_root / name / "combined_summary.csv")
            if row["map"] in maps and minimum <= int(row["agent_count"]) <= maximum
        )
    for name in ("human_common", "human_pibt_extended"):
        current_raw.extend(read_rows(args.new_root / name / "combined_summary.csv"))
    for layout in (2, 3, 4):
        for prefix in ("sortation_p20_baselines_layout", "sortation_p20_layout"):
            for row in read_rows(
                args.new_root / f"{prefix}{layout}" / "combined_summary.csv"
            ):
                row["map"] = normalize_sortation_map(row["map"])
                current_raw.append(row)
    current_raw.sort(key=lambda row: (
        row["map"], int(row["agent_count"]), row["method"],
        int(row.get("pickup_layout_seed", 1)), int(row["seed"]),
    ))
    write_rows(args.new_root / "current_combined_summary.csv", current_raw)
    write_rows(
        args.new_root / "reengineered_hierarchical_effects.csv",
        hierarchical_effect_rows(current_raw),
    )


if __name__ == "__main__":
    main()
