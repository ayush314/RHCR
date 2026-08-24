#!/usr/bin/env python3
"""Validate completeness and provenance of publication extension runs."""

from __future__ import annotations

import argparse
import hashlib
import json
from collections import Counter
from pathlib import Path


EXPECTED_SIGNATURE = {
    "simulation_time": 1000,
    "planning_window": 20,
    "simulation_window": 5,
    "pressure_threshold": 2,
    "pressure_profile": "fixed",
    "pressure_admission": "adaptive",
    "pressure_inbound_limit": 4,
    "pressure_zone_cost": 2.0,
    "pressure_cost_occupancy_threshold": 2,
    "pressure_cost_mode": "fixed",
    "pressure_cost_scope": "zone",
    "pressure_cost_activation": "zone",
    "pressure_population": "all_phases",
    "pressure_front_progress_cost": 3,
    "pressure_exit_progress_cost": 1,
    "pressure_ready_slot_priority": True,
    "pressure_front_runner_priority": False,
    "pibt_global_front_runner_priority": False,
    "pibt_front_runner_priority": False,
    "pibt_assignment_budget_factor": 90,
    "pibt_pressure_assignment_extension_factor": 20,
    "native_failures_only": True,
}
ALLOWED_OUTCOMES = {"clean", "solver_failure", "traffic_jam", "physical_capacity"}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def key(payload: dict) -> tuple[str, int, str, int]:
    return (
        str(payload["map"]), int(payload["agent_count"]),
        str(payload["method"]), int(payload["seed"]),
    )


def validate_root(root: Path, binary_hash: str) -> dict:
    manifest = json.loads((root / "run_manifest.json").read_text())
    expected = {
        (map_name, int(count), method, int(seed))
        for map_name, counts in manifest["grids"].items()
        for count in counts
        for method in manifest["methods"]
        for seed in manifest["seeds"]
    }
    statuses = []
    for path in root.rglob("status.json"):
        payload = json.loads(path.read_text())
        if "map" in payload and "agent_count" in payload and "method" in payload:
            statuses.append((path, payload))
    keyed = Counter(key(payload) for _, payload in statuses)
    present = set(keyed)
    errors = []
    for run_key in sorted(expected):
        matches = [(path, payload) for path, payload in statuses if key(payload) == run_key]
        if len(matches) != 1:
            errors.append(f"{run_key}: expected one status, found {len(matches)}")
            continue
        path, payload = matches[0]
        if payload.get("status") not in {"clean", "failed"}:
            errors.append(f"{path}: nonterminal status {payload.get('status')}")
        if payload.get("failure_reason") not in ALLOWED_OUTCOMES:
            errors.append(f"{path}: unexpected outcome {payload.get('failure_reason')!r}")
        signature = payload.get("run_signature", {})
        if signature.get("binary_sha256") != binary_hash:
            errors.append(f"{path}: binary hash mismatch")
        for field, expected_value in EXPECTED_SIGNATURE.items():
            if signature.get(field) != expected_value:
                errors.append(
                    f"{path}: {field}={signature.get(field)!r}, expected {expected_value!r}"
                )
        if any("relief" in field.lower() for field in signature):
            errors.append(f"{path}: relief field found in signature")
    for filename in (
        "aggregate.csv", "combined_summary.csv", "paired_comparison.csv",
        "hierarchical_effects.csv", "stall_survival.csv",
    ):
        if not (root / filename).exists():
            errors.append(f"{root / filename}: missing aggregate artifact")
    extras = sorted(present - expected)
    return {
        "root": str(root),
        "expected_cells": len(expected),
        "terminal_expected_cells": sum(
            payload.get("status") in {"clean", "failed"}
            for _, payload in statuses if key(payload) in expected
        ),
        "extra_provenance_cells": len(extras),
        "extra_keys": extras,
        "errors": errors,
    }


def main() -> int:
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--root", type=Path,
        default=repo / "results" / "publication_extensions_v2",
    )
    args = parser.parse_args()
    root = args.root.resolve()
    extension = json.loads((root / "extension_manifest.json").read_text())
    binary = Path(extension["binary"])
    expected_hash = extension["binary_sha256"]
    top_errors = []
    if not binary.exists() or sha256(binary) != expected_hash:
        top_errors.append("archived binary is missing or has the wrong SHA-256")
    if extension.get("method_profile", {}).get("pressure_relief") is not False:
        top_errors.append("extension manifest does not explicitly disable pressure relief")

    manifests = sorted(
        path.parent for path in root.rglob("run_manifest.json")
        if "assembled" not in path.parts
    )
    reports = [validate_root(run_root, expected_hash) for run_root in manifests]
    errors = top_errors + [error for report in reports for error in report["errors"]]
    report = {
        "valid": not errors,
        "binary_sha256": expected_hash,
        "run_root_count": len(reports),
        "expected_cell_count": sum(item["expected_cells"] for item in reports),
        "terminal_expected_cell_count": sum(
            item["terminal_expected_cells"] for item in reports
        ),
        "extra_provenance_cell_count": sum(
            item["extra_provenance_cells"] for item in reports
        ),
        "errors": errors,
        "roots": reports,
    }
    (root / "validation_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n"
    )
    print(json.dumps({key: report[key] for key in (
        "valid", "run_root_count", "expected_cell_count",
        "terminal_expected_cell_count", "extra_provenance_cell_count",
    )}, indent=2))
    if errors:
        print(f"validation errors: {len(errors)}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
