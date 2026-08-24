#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path


CANDIDATES = ("single", "adaptive")


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else float("nan")


def numeric(row: dict, key: str) -> float | None:
    value = row.get(key, "")
    if value in {"", None}:
        return None
    return float(value)


def load_rows(root: Path) -> list[dict]:
    rows = []
    for status_path in root.rglob("status.json"):
        if "phase_pretest" in status_path.parts or "frontier_discovery" in status_path.parts:
            continue
        status = json.loads(status_path.read_text())
        summary_path = status_path.parent / "summary.csv"
        if not summary_path.exists():
            continue
        with summary_path.open() as fh:
            summary = next(csv.DictReader(fh))
        rows.append({**status, **summary})
    return rows


def candidate_name(method: str) -> str | None:
    for candidate in CANDIDATES:
        if method.endswith(f"pressure_{candidate}"):
            return candidate
    return None


def phase_method(method: str) -> str:
    return "pbs_phase_aware" if method.startswith("pbs_") else "pibt_phase_aware"


def main() -> int:
    parser = argparse.ArgumentParser(description="Freeze the shared pressure-admission method from development runs.")
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    rows = load_rows(args.root)
    if not rows:
        raise SystemExit(f"No development rows found under {args.root}")
    lookup = {
        (row["map"], int(row["agent_count"]), row["method"], int(row.get("seed", 0))): row
        for row in rows
    }
    score = {}
    for candidate in CANDIDATES:
        candidate_rows = [row for row in rows if candidate_name(str(row["method"])) == candidate]
        if not candidate_rows:
            raise SystemExit(f"Missing {candidate} candidate rows")
        paired = []
        for row in candidate_rows:
            key = (row["map"], int(row["agent_count"]), phase_method(str(row["method"])), int(row.get("seed", 0)))
            baseline = lookup.get(key)
            if baseline is not None:
                paired.append((row, baseline))
        if not paired:
            raise SystemExit(f"No paired Phase-Aware rows for {candidate}")

        throughput_ratios = []
        rmst_changes = []
        for row, base in paired:
            row_service = numeric(row, "service_rate")
            base_service = numeric(base, "service_rate")
            if row_service is not None and base_service is not None and base_service > 0:
                throughput_ratios.append(row_service / base_service)
            row_rmst = numeric(row, "queue_wait_rmst100")
            base_rmst = numeric(base, "queue_wait_rmst100")
            if row_rmst is not None and base_rmst is not None and base_rmst > 0:
                rmst_changes.append((base_rmst - row_rmst) / base_rmst)
        clean_count = sum(int(float(row.get("clean_completion", 0))) for row in candidate_rows)
        time_to_stall = mean([float(row["time_to_stall"]) for row in candidate_rows])
        score[candidate] = {
            "clean_completions": clean_count,
            "mean_time_to_stall": time_to_stall,
            "mean_rmst100": mean([
                value for row in candidate_rows
                if (value := numeric(row, "queue_wait_rmst100")) is not None
            ]),
            "mean_throughput_ratio_to_phase": mean(throughput_ratios),
            "mean_rmst100_reduction_fraction": mean(rmst_changes),
        }

    eligible = [
        name for name in CANDIDATES
        if score[name]["mean_throughput_ratio_to_phase"] >= 0.98
    ]
    if not eligible:
        raise SystemExit("Neither candidate satisfies the 2% throughput guardrail")
    winner = min(
        eligible,
        key=lambda name: (
            -score[name]["clean_completions"],
            -score[name]["mean_time_to_stall"],
            score[name]["mean_rmst100"],
            0 if name == "single" else 1,
        ),
    )

    grouped: dict[tuple[str, str], dict[str, set[int]]] = defaultdict(lambda: defaultdict(set))
    for row in rows:
        method = str(row["method"])
        if int(float(row.get("clean_completion", 0))) != 1:
            continue
        solver = "pbs" if method.startswith("pbs_") else "pibt"
        if method == f"{solver}_phase_aware":
            grouped[(str(row["map"]), solver)]["phase"].add(int(row["agent_count"]))
        elif method == f"{solver}_pressure_{winner}":
            grouped[(str(row["map"]), solver)]["pressure"].add(int(row["agent_count"]))
    frontier_extended = any(
        values["pressure"] and max(values["pressure"]) > max(values["phase"], default=-1)
        for values in grouped.values()
    )
    rmst_gate = score[winner]["mean_rmst100_reduction_fraction"] >= 0.25
    passed = frontier_extended or rmst_gate
    payload = {
        "schema_version": 1,
        "development_root": str(args.root.resolve()),
        "winner": winner,
        "pressure_threshold": 1,
        "pressure_zone_cost": 1,
        "pressure_cost_scope": "zone",
        "pressure_cost_occupancy_threshold": 3,
        "pressure_cost_mode": "fixed",
        "pressure_cost_horizon": 0,
        "pressure_cost_horizon_profile": "fixed",
        "pressure_lookahead_profile": "scale_adaptive",
        "pressure_lookahead_radius": 50,
        "pressure_lookahead_min_agents_per_station": 40,
        "pressure_definition": "inbound_target_station_wip_in_physical_station_zone",
        "pressure_population": "inbound_only",
        "pibt_front_runner_priority": False,
        "pbs_front_runner_priority": False,
        "pibt_global_front_runner_priority": False,
        "pibt_front_runner_ready_priority": False,
        "adaptive_base_budget": 3,
        "scores": score,
        "frontier_extended": frontier_extended,
        "rmst_gate_passed": rmst_gate,
        "publication_gate_passed": passed,
    }
    output = args.output or args.root / "selected_pressure_method.json"
    output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    print(json.dumps(payload, indent=2, sort_keys=True))
    if not passed:
        raise SystemExit("Winner does not meet the publication progression gate")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
