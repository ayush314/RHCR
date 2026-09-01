#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

from discover_publication_ladders import classify_terminal


CONDITIONS = {
    "alley": {"start": 20, "resolution": 10, "methods": ("pbs_lead_aware", "pibt_lead_aware")},
    "plaza": {"start": 40, "resolution": 20, "methods": ("pbs_lead_aware", "pibt_lead_aware")},
    "sortation_medium_p20": {"start": 500, "resolution": 100, "methods": ("pibt_lead_aware",)},
}


def main() -> int:
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Pretest Lead-Aware boundaries for final experiment ladders.")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--binary", default=str(repo / "lifelong"))
    parser.add_argument("--jobs", type=int, default=6)
    parser.add_argument("--process-timeout", type=int, default=600)
    parser.add_argument(
        "--conditions", default=",".join(CONDITIONS),
        help="Comma-separated development conditions to probe.",
    )
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    seeds = (1, 2, 3)
    active_conditions = [
        value.strip() for value in args.conditions.split(",") if value.strip()
    ]
    unknown_conditions = sorted(set(active_conditions) - set(CONDITIONS))
    if unknown_conditions:
        parser.error(f"Unknown conditions: {', '.join(unknown_conditions)}")

    def run_probe(condition: str, method: str, count: int) -> tuple[bool, str]:
        root = args.root / condition / method
        command = [
            sys.executable, str(repo / "scripts/run_comparison.py"),
            "--root", str(root), "--binary", str(Path(args.binary).resolve()),
            "--seed-list", ",".join(map(str, seeds)), "--simulation-time", "1000",
            "--planning-window", "20", "--simulation-window", "5", "--service-time", "3",
            "--lra-fallback",
            "--methods", method,
            "--jobs", str(args.jobs), "--process-timeout", str(args.process_timeout),
            "--alley-counts", str(count) if condition == "alley" else "",
            "--plaza-counts", str(count) if condition == "plaza" else "",
        ]
        label = condition
        if condition == "sortation_medium_p20":
            command += [
                "--lorr-sortation-medium-counts", str(count),
                "--lorr-sortation-medium-benchmark", str(repo / "benchmarks/lorr/sortation_medium_p20.json"),
                "--lorr-sortation-medium-name", label,
            ]
        if args.force:
            command.append("--force")
        subprocess.run(command, check=True)
        statuses = []
        for seed in seeds:
            path = root / label / f"agents_{count}" / method / f"seed_{seed}" / "status.json"
            statuses.append(json.loads(path.read_text()))
        return classify_terminal(statuses)

    for condition in active_conditions:
        details = CONDITIONS[condition]
        resolution = details["resolution"]
        boundaries = []
        for method in details["methods"]:
            low = 0
            high = details["start"]
            failed, terminal_type = run_probe(condition, method, high)
            while not failed:
                low = high
                high *= 2
                failed, terminal_type = run_probe(condition, method, high)
            while high - low > resolution:
                middle = ((low + high) // (2 * resolution)) * resolution
                if middle <= low:
                    middle = low + resolution
                failed, middle_type = run_probe(condition, method, middle)
                if failed:
                    high = middle
                    terminal_type = middle_type
                else:
                    low = middle
            frontier = max(resolution, low)
            if terminal_type != "physical_capacity":
                selected = (max(resolution, frontier - resolution), frontier, high)
            else:
                selected = (
                    max(resolution, frontier - 2 * resolution),
                    max(resolution, frontier - resolution),
                    frontier,
                )
            boundaries.extend(selected)
            solver = "pbs" if method.startswith("pbs_") else "pibt"
            config.setdefault("development_solver_counts", {}).setdefault(condition, {})[solver] = \
                sorted(set(selected))
            config.setdefault("development_boundaries", {}).setdefault(condition, {})[method] = {
                "last_nonterminal": low,
                "terminal_probe": high,
                "terminal_type": terminal_type,
                "selected_counts": list(selected),
            }
            print(
                f"[pretest] {condition} {method}: frontier={frontier}, "
                f"first_terminal={high}, terminal_type={terminal_type}, selected={list(selected)}",
                flush=True,
            )
        config["development"][condition] = sorted(set(boundaries))

    for condition in ("alley", "plaza"):
        if condition not in active_conditions:
            continue
        human = config["human"]
        pbs_boundary = config["development_boundaries"][condition]["pbs_lead_aware"]
        pibt_boundary = config["development_boundaries"][condition]["pibt_lead_aware"]
        common = sorted(set(human["common"][condition]) | set(pbs_boundary["selected_counts"]))
        extended = sorted(
            count for count in
            (set(human["pibt_extended"][condition]) | {pibt_boundary["terminal_probe"]})
            if count > common[-1]
        )
        human["common"][condition] = common
        human["pibt_extended"][condition] = extended

    if "sortation_medium_p20" in active_conditions:
        medium_counts = config["development"]["sortation_medium_p20"]
        for condition in config.get("sensitivity", []):
            if "sortation_medium_p20" in condition:
                condition["sortation_medium_p20"] = medium_counts[-2:]

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(config, indent=2, sort_keys=True) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
