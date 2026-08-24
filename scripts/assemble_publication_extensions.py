#!/usr/bin/env python3
"""Assemble service-time and pickup-density publication extensions."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path


METHODS = {"pibt_vanilla", "pibt_phase_aware", "pibt_pressure_aware"}
SORTATION_RE = re.compile(r"^(?:lorr_)?(sortation_(?:small|medium|large)_p\d+)(?:_layout[234])?$")


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_rows(path: Path, rows: list[dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields: list[str] = []
    for row in rows:
        for field in row:
            if field not in fields:
                fields.append(field)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def normalize_sortation_map(name: str) -> str:
    match = SORTATION_RE.match(name)
    if not match:
        raise ValueError(f"unexpected sortation map name: {name}")
    return f"lorr_{match.group(1)}"


def aggregate(repo: Path, output: Path, rows: list[dict[str, str]]) -> None:
    rows.sort(key=lambda row: (
        row["map"], int(row["agent_count"]), row["method"],
        int(row.get("pickup_layout_seed", 1)), int(row["seed"]),
    ))
    write_rows(output / "combined_summary.csv", rows)
    subprocess.run([
        sys.executable, str(repo / "scripts" / "aggregate_results.py"),
        "--root", str(output),
    ], cwd=repo, check=True)


def extension_density_rows(root: Path, density: str) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for layout in (2, 3, 4):
        path = root / "sortation_density" / density / f"layout{layout}" / "combined_summary.csv"
        if not path.exists():
            raise FileNotFoundError(path)
        for source in read_rows(path):
            row = dict(source)
            row["map"] = normalize_sortation_map(row["map"])
            rows.append(row)
    return rows


def p20_rows(path: Path, extension_root: Path) -> list[dict[str, str]]:
    rows = [
        dict(row) for row in read_rows(path)
        if row["method"] in METHODS
        and re.match(r"^lorr_sortation_(small|medium|large)_p20$", row["map"])
        and not (
            (row["map"] == "lorr_sortation_medium_p20" and int(row["agent_count"]) == 16000)
            or (row["map"] == "lorr_sortation_large_p20" and int(row["agent_count"]) == 32000)
        )
    ]
    for layout in (2, 3, 4):
        terminal_path = (
            extension_root / "sortation_density" / "p20_terminal" /
            f"layout{layout}" / "combined_summary.csv"
        )
        if not terminal_path.exists():
            raise FileNotFoundError(terminal_path)
        for source in read_rows(terminal_path):
            row = dict(source)
            row["map"] = normalize_sortation_map(row["map"])
            rows.append(row)
    return rows


def tau_rows(root: Path, tau: int) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    base = root / "tau_sensitivity" / f"tau{tau}"
    for name in ("human_pbs", "human_pibt"):
        rows.extend(read_rows(base / name / "combined_summary.csv"))
    for layout in (2, 3, 4):
        for source in read_rows(base / f"sortation_medium_p20_layout{layout}" / "combined_summary.csv"):
            row = dict(source)
            row["map"] = normalize_sortation_map(row["map"])
            rows.append(row)
    return rows


def tau3_rows(path: Path) -> list[dict[str, str]]:
    allowed = {
        ("alley", 50, "pbs"), ("alley", 60, "pbs"),
        ("alley", 100, "pibt"), ("alley", 140, "pibt"),
        ("lorr_sortation_medium_p20", 6000, "pibt"),
        ("lorr_sortation_medium_p20", 8000, "pibt"),
    }
    rows = []
    for source in read_rows(path):
        solver = source["method"].split("_", 1)[0]
        if (source["map"], int(source["agent_count"]), solver) in allowed:
            rows.append(dict(source))
    return rows


def assert_unique(rows: list[dict[str, str]], label: str) -> None:
    keys = [(
        row["map"], int(row["agent_count"]), row["method"],
        int(row.get("pickup_layout_seed", 1)), int(row["seed"]),
    ) for row in rows]
    if len(keys) != len(set(keys)):
        raise RuntimeError(f"duplicate run keys in {label}")


def frontier_rows(rows: list[dict[str, str]]) -> list[dict[str, str | int]]:
    grouped: dict[tuple[str, str], list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        grouped[(row["map"], row["method"])].append(row)
    output = []
    for (map_name, method), entries in sorted(grouped.items()):
        entries.sort(key=lambda row: int(row["agent_count"]))
        any_clean = [
            int(row["agent_count"]) for row in entries
            if int(row["clean_seed_count"]) > 0
        ]
        all_clean = [
            int(row["agent_count"]) for row in entries
            if int(row["clean_seed_count"]) == int(row["seed_count"])
        ]
        no_clean = [
            int(row["agent_count"]) for row in entries
            if int(row["clean_seed_count"]) == 0
        ]
        output.append({
            "map": map_name,
            "method": method,
            "highest_any_clean_count": max(any_clean) if any_clean else "",
            "highest_all_clean_count": max(all_clean) if all_clean else "",
            "first_all_failed_count": min(no_clean) if no_clean else "",
        })
    return output


def high_load_rows(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    by_map_count: dict[tuple[str, int], list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        by_map_count[(row["map"], int(row["agent_count"]))].append(row)
    selected_counts = {}
    for (map_name, count), entries in by_map_count.items():
        if len(entries) == 3 and all(int(row["clean_seed_count"]) > 0 for row in entries):
            selected_counts[map_name] = max(count, selected_counts.get(map_name, 0))
    fields = (
        "map", "agent_count", "method", "seed_count", "clean_seed_count",
        "clean_completion", "service_rate", "queue_wait_rmst100",
        "queue_wait_p95", "mean_target_queue_occupancy_per_station",
        "mean_plan_ms", "peak_rss_kb",
    )
    return [
        {field: row.get(field, "") for field in fields}
        for row in rows
        if int(row["agent_count"]) == selected_counts.get(row["map"])
    ]


def pressure_phase_effect_rows(rows: list[dict[str, str]]) -> list[dict[str, str | float]]:
    lookup = {(row["map"], row["agent_count"], row["method"]): row for row in rows}
    output = []
    metrics = (
        "clean_completion", "service_rate", "queue_wait_rmst100",
        "queue_wait_p95", "mean_target_queue_occupancy_per_station", "mean_plan_ms",
    )
    for row in rows:
        if row["method"] != "pibt_pressure_aware":
            continue
        phase = lookup.get((row["map"], row["agent_count"], "pibt_phase_aware"))
        if phase is None:
            continue
        result: dict[str, str | float] = {
            "map": row["map"], "agent_count": row["agent_count"],
        }
        for metric in metrics:
            pressure_value = float(row[metric])
            phase_value = float(phase[metric])
            result[f"pressure_{metric}"] = pressure_value
            result[f"phase_{metric}"] = phase_value
            result[f"difference_{metric}"] = pressure_value - phase_value
            result[f"relative_difference_percent_{metric}"] = (
                "" if phase_value == 0 else 100 * (pressure_value - phase_value) / phase_value
            )
        output.append(result)
    return output


def main() -> int:
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--extension-root", type=Path,
        default=repo / "results" / "publication_extensions_v2",
    )
    parser.add_argument(
        "--p20-summary", type=Path,
        default=repo / "results" / "publication_joint_reengineered_v1" /
        "current_combined_summary.csv",
    )
    args = parser.parse_args()
    root = args.extension_root.resolve()
    output = root / "assembled"

    density_counts = {}
    all_density_rows: list[dict[str, str]] = []
    for density in ("p05", "p100"):
        rows = extension_density_rows(root, density)
        assert_unique(rows, density)
        aggregate(repo, output / density, rows)
        density_counts[density] = len(rows)
        all_density_rows.extend(rows)
    rows20 = p20_rows(args.p20_summary.resolve(), root)
    assert_unique(rows20, "p20")
    aggregate(repo, output / "p20", rows20)
    density_counts["p20"] = len(rows20)
    all_density_rows.extend(rows20)
    assert_unique(all_density_rows, "all densities")
    aggregate(repo, output / "density_all", all_density_rows)
    density_aggregate = read_rows(output / "density_all" / "aggregate.csv")
    write_rows(
        output / "density_frontiers.csv",
        frontier_rows(density_aggregate),
    )
    high_load = high_load_rows(density_aggregate)
    write_rows(output / "density_high_load_metrics.csv", high_load)
    write_rows(
        output / "density_pressure_vs_phase_high_load.csv",
        pressure_phase_effect_rows(high_load),
    )

    tau_aggregates: list[dict[str, str]] = []
    tau_counts = {}
    for tau in (1, 3, 5):
        rows = tau3_rows(args.p20_summary.resolve()) if tau == 3 else tau_rows(root, tau)
        assert_unique(rows, f"tau{tau}")
        tau_output = output / f"tau{tau}"
        aggregate(repo, tau_output, rows)
        tau_counts[str(tau)] = len(rows)
        for row in read_rows(tau_output / "aggregate.csv"):
            row["service_time"] = str(tau)
            tau_aggregates.append(row)
    write_rows(output / "tau_all_aggregate.csv", tau_aggregates)

    p20_artifacts = list((args.p20_summary.resolve().parent / "artifacts").glob("lifelong-*"))
    extension_manifest = json.loads((root / "extension_manifest.json").read_text())
    manifest = {
        "density_run_counts": density_counts,
        "density_total_runs": len(all_density_rows),
        "tau_run_counts": tau_counts,
        "tau_total_runs": sum(tau_counts.values()),
        "p20_source": str(args.p20_summary.resolve()),
        "p20_source_binary_sha256": sha256(p20_artifacts[0]) if len(p20_artifacts) == 1 else "",
        "extension_binary_sha256": extension_manifest["binary_sha256"],
    }
    (output / "assembly_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
