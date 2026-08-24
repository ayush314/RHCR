#!/usr/bin/env python3
"""Run the frozen main-paper evaluation matrix."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path


METHODS = {
    "pbs": "pbs_vanilla,pbs_phase_aware,pbs_pressure_aware",
    "pibt": "pibt_vanilla,pibt_phase_aware,pibt_pressure_aware",
}

SORTATION_OPTIONS = {
    "small": (
        "--lorr-sortation-counts",
        "--lorr-sortation-benchmark",
        "--lorr-sortation-name",
    ),
    "medium": (
        "--lorr-sortation-medium-counts",
        "--lorr-sortation-medium-benchmark",
        "--lorr-sortation-medium-name",
    ),
    "large": (
        "--lorr-sortation-large-counts",
        "--lorr-sortation-large-benchmark",
        "--lorr-sortation-large-name",
    ),
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def csv(values: list[int]) -> str:
    return ",".join(str(value) for value in values)


def validate_config(config: dict) -> None:
    required_profile = {
        "pressure_threshold": 2,
        "pressure_admission": "adaptive",
        "pressure_inbound_limit": 4,
        "pressure_cost_occupancy_threshold": 2,
        "pressure_population": "all_phases",
        "pressure_zone_cost": 2,
        "pressure_front_progress_cost": 3,
        "pressure_exit_progress_cost": 1,
        "pressure_ready_slot_priority": True,
        "pressure_cost_scope": "zone",
        "pressure_cost_activation": "zone",
        "pressure_profile": "fixed",
        "pressure_lookahead_radius": 0,
    }
    profile = config.get("pressure_profile", {})
    mismatches = {
        key: (profile.get(key), expected)
        for key, expected in required_profile.items()
        if profile.get(key) != expected
    }
    if mismatches:
        raise SystemExit(f"configuration is not the frozen pressure profile: {mismatches}")

    for size, densities in config["sortation"].items():
        if size in {"pickup_layout_seeds", "simulation_seeds"}:
            continue
        for density, cell in densities.items():
            counts = cell["counts"]
            if len(counts) != 8:
                raise SystemExit(f"{size} {density} must have eight reportable counts")
            gaps = [right - left for left, right in zip(counts, counts[1:])]
            if len(set(gaps)) != 1 or gaps[0] <= 0:
                raise SystemExit(f"{size} {density} counts are not equally spaced")
            if cell["terminal_probe"] <= counts[-1]:
                raise SystemExit(f"{size} {density} terminal probe is not above its endpoint")


def profile_args(config: dict) -> list[str]:
    protocol = config["protocol"]
    profile = config["pressure_profile"]
    args = [
        "--simulation-time", str(protocol["simulation_time"]),
        "--planning-window", str(protocol["planning_window"]),
        "--simulation-window", str(protocol["simulation_window"]),
        "--service-time", str(protocol["service_time"]),
        "--pressure-threshold", str(profile["pressure_threshold"]),
        "--pressure-profile", profile["pressure_profile"],
        "--pressure-admission", profile["pressure_admission"],
        "--pressure-inbound-limit", str(profile["pressure_inbound_limit"]),
        "--pressure-zone-cost", str(profile["pressure_zone_cost"]),
        "--pressure-cost-occupancy-threshold",
        str(profile["pressure_cost_occupancy_threshold"]),
        "--pressure-cost-mode", "fixed",
        "--pressure-cost-scope", profile["pressure_cost_scope"],
        "--pressure-cost-activation", profile["pressure_cost_activation"],
        "--pressure-population", profile["pressure_population"],
        "--pressure-front-progress-cost", str(profile["pressure_front_progress_cost"]),
        "--pressure-exit-progress-cost", str(profile["pressure_exit_progress_cost"]),
        "--pressure-lookahead-profile", "fixed",
        "--pressure-lookahead-radius", str(profile["pressure_lookahead_radius"]),
        "--pressure-cost-horizon", "0",
        "--pressure-cost-horizon-profile", "fixed",
        "--no-pressure-front-runner-priority",
        "--no-pressure-front-runner-zone-only",
        "--no-pressure-front-runner-ready-priority",
        "--no-pibt-global-front-runner-priority",
        "--no-pibt-front-runner-priority",
        "--no-pibt-front-runner-ready-priority",
        "--pibt-assignment-budget-factor", "90",
        "--pibt-pressure-assignment-extension-factor", "20",
        "--screen", "0",
    ]
    args.append(
        "--pressure-ready-slot-priority"
        if profile["pressure_ready_slot_priority"]
        else "--no-pressure-ready-slot-priority"
    )
    return args


def comparison_base(
    repo: Path,
    binary: Path,
    root: Path,
    config: dict,
    jobs: int,
    process_timeout: int,
    force: bool,
) -> list[str]:
    command = [
        sys.executable,
        str(repo / "scripts" / "run_comparison.py"),
        "--root", str(root),
        "--binary", str(binary),
        "--jobs", str(jobs),
        "--process-timeout", str(process_timeout),
        *profile_args(config),
    ]
    if force:
        command.append("--force")
    return command


def human_commands(
    repo: Path,
    binary: Path,
    root: Path,
    config: dict,
    args: argparse.Namespace,
) -> list[list[str]]:
    human = config["human"]
    commands = []
    for solver in ("pbs", "pibt"):
        command = comparison_base(
            repo, binary, root / f"human_{solver}", config,
            args.jobs, args.process_timeout, args.force,
        )
        command += [
            "--pickup-layout-seed", str(human["pickup_layout_seeds"][0]),
            "--seed-list", csv(human["simulation_seeds"]),
            "--methods", METHODS[solver],
            "--alley-counts", csv(human[solver]["alley"]),
            "--plaza-counts", csv(human[solver]["plaza"]),
        ]
        commands.append(command)
    return commands


def sortation_commands(
    repo: Path,
    binary: Path,
    root: Path,
    config: dict,
    args: argparse.Namespace,
) -> list[list[str]]:
    sortation = config["sortation"]
    commands = []
    for density in ("p05", "p20", "p100"):
        for layout in sortation["pickup_layout_seeds"]:
            command = comparison_base(
                repo, binary, root / "sortation" / density / f"layout{layout}",
                config, args.jobs, args.process_timeout, args.force,
            )
            command += [
                "--pickup-layout-seed", str(layout),
                "--seed-list", csv(sortation["simulation_seeds"]),
                "--methods", METHODS["pibt"],
                "--alley-counts", "",
                "--plaza-counts", "",
            ]
            for size, (counts_flag, benchmark_flag, name_flag) in SORTATION_OPTIONS.items():
                cell = sortation[size][density]
                counts = list(cell["counts"])
                if args.include_terminal_probes:
                    counts.append(cell["terminal_probe"])
                benchmark = repo / "benchmarks" / "lorr" / (
                    f"sortation_{size}_{density}_layout{layout}.json"
                )
                command += [
                    counts_flag, csv(counts),
                    benchmark_flag, str(benchmark),
                    name_flag, f"lorr_sortation_{size}_{density}",
                ]
            commands.append(command)
    return commands


def run_command(repo: Path, command: list[str], dry_run: bool) -> None:
    print("+ " + " ".join(command), flush=True)
    if dry_run:
        return
    subprocess.run(command, cwd=repo, check=True)
    root = Path(command[command.index("--root") + 1])
    subprocess.run(
        [
            sys.executable,
            str(repo / "scripts" / "aggregate_results.py"),
            "--root", str(root),
        ],
        cwd=repo,
        check=True,
    )


def main() -> int:
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=repo / "configs" / "publication_experiments.example.json",
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=repo / "results" / "publication_reproduction",
    )
    parser.add_argument("--binary", type=Path, default=repo / "build" / "lifelong")
    parser.add_argument("--stage", choices=("human", "sortation", "all"), default="all")
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--process-timeout", type=int, default=1800)
    parser.add_argument("--include-terminal-probes", action="store_true")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if args.jobs < 1 or args.process_timeout < 1:
        parser.error("jobs and process timeout must be positive")
    config = json.loads(args.config.read_text())
    validate_config(config)

    binary = args.binary.resolve()
    if not binary.exists():
        parser.error(f"missing binary: {binary}")
    root = args.root.resolve()
    root.mkdir(parents=True, exist_ok=True)

    digest = sha256(binary)
    artifact = root / "artifacts" / f"lifelong-{digest[:16]}"
    artifact.parent.mkdir(parents=True, exist_ok=True)
    if not args.dry_run:
        if not artifact.exists():
            shutil.copy2(binary, artifact)
        if sha256(artifact) != digest:
            raise RuntimeError("archived binary hash mismatch")
        manifest = {
            "binary": str(artifact),
            "binary_sha256": digest,
            "config": config,
            "terminal_probes_included": args.include_terminal_probes,
        }
        (root / "publication_manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n"
        )
    else:
        artifact = binary

    commands = []
    if args.stage in {"human", "all"}:
        commands.extend(human_commands(repo, artifact, root, config, args))
    if args.stage in {"sortation", "all"}:
        commands.extend(sortation_commands(repo, artifact, root, config, args))
    for command in commands:
        run_command(repo, command, args.dry_run)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
