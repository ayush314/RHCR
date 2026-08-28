#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path


HUMAN_METHODS = (
    "pbs_vanilla,pbs_departure_aware,pbs_pressure_aware,"
    "pibt_vanilla,pibt_departure_aware,pibt_pressure_aware"
)
PIBT_METHODS = "pibt_vanilla,pibt_departure_aware,pibt_pressure_aware"


def counts_arg(values: list[int]) -> str:
    return ",".join(str(value) for value in values)


def validate_ladder(name: str, counts: list[int]) -> None:
    if len(counts) != 9:
        raise SystemExit(f"{name} must contain eight reportable counts plus one terminal probe")
    reportable = counts[:8]
    gaps = [b - a for a, b in zip(reportable, reportable[1:])]
    if not gaps or len(set(gaps)) != 1 or gaps[0] <= 0 or counts[-1] <= reportable[-1]:
        raise SystemExit(f"{name} must have eight equally spaced counts followed by a higher terminal")


def run(command: list[str]) -> None:
    print("+ " + " ".join(command), flush=True)
    subprocess.run(command, check=True)


def comparison_base(repo: Path, root: Path, args: argparse.Namespace) -> list[str]:
    command = [
        sys.executable, str(repo / "scripts" / "run_comparison.py"),
        "--root", str(root), "--binary", str(Path(args.binary).resolve()),
        "--simulation-time", "1000", "--planning-window", "20",
        "--simulation-window", "5", "--service-time", str(args.service_time),
        "--lra-fallback",
        "--jobs", str(args.jobs), "--process-timeout", str(args.process_timeout),
    ]
    if args.force:
        command.append("--force")
    return command


def main() -> int:
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Run the publication-reset PBS/PIBT experiment matrix.")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--root", type=Path, default=repo / "results" / "publication_reset")
    parser.add_argument("--binary", default=str(repo / "lifelong"))
    parser.add_argument("--stage", choices=("pretest", "discover", "human", "sortation", "sensitivity", "aggregate", "all"), required=True)
    parser.add_argument("--service-time", type=int, default=3)
    parser.add_argument("--jobs", type=int, default=6)
    parser.add_argument("--process-timeout", type=int, default=1800)
    parser.add_argument(
        "--pretest-conditions", default="alley,plaza,sortation_medium_p20",
        help="Comma-separated conditions used by the pretest stage.",
    )
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    active_config = args.config.resolve()
    config = json.loads(active_config.read_text())
    root = args.root.resolve()
    root.mkdir(parents=True, exist_ok=True)
    (root / "publication_matrix.json").write_text(json.dumps(config, indent=2, sort_keys=True) + "\n")

    stages = ("pretest", "discover", "human", "sortation", "sensitivity", "aggregate") if args.stage == "all" else (args.stage,)
    for stage in stages:
        if stage == "pretest":
            pretested = root / "pretested_experiment_config.json"
            command = [sys.executable, str(repo / "scripts/discover_development_counts.py"),
                       "--config", str(active_config), "--output", str(pretested),
                       "--root", str(root / "development/departure_pretest"),
                       "--binary", str(Path(args.binary).resolve()), "--jobs", str(args.jobs),
                       "--process-timeout", str(args.process_timeout),
                       "--conditions", args.pretest_conditions]
            if args.force:
                command.append("--force")
            run(command)
            active_config = pretested
            config = json.loads(pretested.read_text())
        elif stage == "discover":
            frozen = root / "frozen_experiment_config.json"
            command = [sys.executable, str(repo / "scripts/discover_publication_ladders.py"),
                       "--config", str(active_config), "--output", str(frozen),
                       "--root", str(root / "development/frontier_discovery"),
                       "--binary", str(Path(args.binary).resolve()),
                       "--jobs", str(args.jobs),
                       "--process-timeout", str(args.process_timeout)]
            if args.force:
                command.append("--force")
            run(command)
            active_config = frozen
            config = json.loads(frozen.read_text())
        elif stage == "human":
            human = config["human"]
            for map_name in ("alley", "plaza"):
                overlap = set(human["common"][map_name]) & set(human["pibt_extended"][map_name])
                if overlap:
                    raise SystemExit(f"Human common and extended ladders overlap for {map_name}: {sorted(overlap)}")
                if human["pibt_extended"][map_name] and human["common"][map_name] and \
                        min(human["pibt_extended"][map_name]) <= max(human["common"][map_name]):
                    raise SystemExit(f"Extended PIBT ladder must start above the common {map_name} ladder")
            command = comparison_base(repo, root / "final" / "human_common", args)
            command += ["--seed-list", counts_arg(list(range(6, 26))), "--methods", HUMAN_METHODS,
                        "--alley-counts", counts_arg(human["common"]["alley"]),
                        "--plaza-counts", counts_arg(human["common"]["plaza"])]
            run(command)
            command = comparison_base(repo, root / "final" / "human_pibt_extended", args)
            command += ["--seed-list", counts_arg(list(range(6, 26))), "--methods", PIBT_METHODS,
                        "--alley-counts", counts_arg(human["pibt_extended"]["alley"]),
                        "--plaza-counts", counts_arg(human["pibt_extended"]["plaza"])]
            run(command)
        elif stage == "sortation":
            for map_name in ("small", "medium", "large"):
                for density in (5, 20, 100):
                    ladder = config["sortation"][map_name][f"p{density:02d}"]
                    validate_ladder(f"{map_name} p{density:02d}", ladder["counts"])
                    for layout in (2, 3, 4):
                        cell_root = root / "final/sortation" / map_name / f"p{density:02d}" / f"layout_{layout}"
                        command = comparison_base(repo, cell_root, args)
                        benchmark = repo / f"benchmarks/lorr/sortation_{map_name}_p{density:02d}_layout{layout}.json"
                        option = f"--lorr-sortation-{'medium-' if map_name == 'medium' else 'large-' if map_name == 'large' else ''}counts"
                        bench_option = option.replace("counts", "benchmark")
                        name_option = option.replace("counts", "name")
                        command += ["--seed-list", "6,7,8,9,10", "--pickup-layout-seed", str(layout),
                                    "--methods", PIBT_METHODS, "--alley-counts", "", "--plaza-counts", "",
                                    option, counts_arg(ladder["counts"]), bench_option, str(benchmark),
                                    name_option, f"lorr_sortation_{map_name}_p{density:02d}"]
                        run(command)
        elif stage == "sensitivity":
            for condition in config["sensitivity"]:
                for tau in (1, 5):
                    args.service_time = tau
                    command = comparison_base(repo, root / f"sensitivity/{condition['name']}/tau_{tau}", args)
                    is_human = bool(condition.get("alley"))
                    seeds = list(range(6, 26)) if is_human else list(range(6, 11))
                    methods = condition.get("methods", HUMAN_METHODS if is_human else PIBT_METHODS)
                    command += ["--seed-list", counts_arg(seeds),
                                "--methods", methods,
                                "--alley-counts", counts_arg(condition.get("alley", [])),
                                "--plaza-counts", ""]
                    if "sortation_medium_p20" in condition:
                        command += ["--pickup-layout-seed", "2",
                                    "--lorr-sortation-medium-counts", counts_arg(condition["sortation_medium_p20"]),
                                    "--lorr-sortation-medium-benchmark", str(repo / "benchmarks/lorr/sortation_medium_p20_layout2.json"),
                                    "--lorr-sortation-medium-name", "lorr_sortation_medium_p20"]
                    run(command)
            args.service_time = 3
        elif stage == "aggregate":
            run([sys.executable, str(repo / "scripts/aggregate_results.py"), "--root", str(root / "final")])
            sensitivity_index = {"tau_3_root": str(root / "final"), "conditions": {}}
            for condition in config["sensitivity"]:
                condition_roots = {}
                for tau in (1, 5):
                    sensitivity_root = root / f"sensitivity/{condition['name']}/tau_{tau}"
                    run([sys.executable, str(repo / "scripts/aggregate_results.py"),
                         "--root", str(sensitivity_root)])
                    condition_roots[f"tau_{tau}"] = str(sensitivity_root)
                sensitivity_index["conditions"][condition["name"]] = {
                    "counts": condition.get("alley", condition.get("sortation_medium_p20", [])),
                    **condition_roots,
                }
            (root / "sensitivity_index.json").write_text(
                json.dumps(sensitivity_index, indent=2, sort_keys=True) + "\n"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
