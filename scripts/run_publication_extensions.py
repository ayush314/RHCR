#!/usr/bin/env python3
"""Run the remaining no-relief publication experiments."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path


METHODS = {
    "PBS": "pbs_vanilla,pbs_phase_aware,pbs_pressure_aware",
    "PIBT": "pibt_vanilla,pibt_phase_aware,pibt_pressure_aware",
}

DENSITY_COUNTS = {
    "p05": {
        "small": "138,276,414,552,690,828,966,1104,1107",
        "medium": "1000,2000,3000,4000,5000,6000,7000,8000,19372",
        "large": "2000,4000,6000,8000,10000,12000,14000,16000,49012",
    },
    "p100": {
        "small": "76,152,228,304,380,456,532,608,616",
        "medium": "985,1970,2955,3940,4925,5910,6895,7880,7881",
        "large": "2410,4820,7230,9640,12050,14460,16870,19280,19281",
    },
}
P20_TERMINALS = {"medium": "17558", "large": "44318"}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def profile_args() -> list[str]:
    return [
        "--simulation-time", "1000",
        "--planning-window", "20",
        "--simulation-window", "5",
        "--pressure-threshold", "2",
        "--pressure-profile", "fixed",
        "--pressure-admission", "adaptive",
        "--pressure-inbound-limit", "4",
        "--pressure-zone-cost", "2",
        "--pressure-cost-occupancy-threshold", "2",
        "--pressure-cost-mode", "fixed",
        "--pressure-cost-scope", "zone",
        "--pressure-cost-activation", "zone",
        "--pressure-population", "all_phases",
        "--pressure-front-progress-cost", "3",
        "--pressure-exit-progress-cost", "1",
        "--pressure-ready-slot-priority",
        "--pressure-lookahead-profile", "fixed",
        "--pressure-lookahead-radius", "0",
        "--pressure-cost-horizon", "0",
        "--pressure-cost-horizon-profile", "fixed",
        "--no-pressure-front-runner-priority",
        "--no-pibt-global-front-runner-priority",
        "--no-pibt-front-runner-priority",
        "--pibt-assignment-budget-factor", "90",
        "--pibt-pressure-assignment-extension-factor", "20",
        "--process-timeout", "1800",
        "--screen", "0",
    ]


def base_command(repo: Path, binary: Path, root: Path, jobs: int) -> list[str]:
    return [
        sys.executable,
        str(repo / "scripts" / "run_comparison.py"),
        "--root", str(root),
        "--binary", str(binary),
        "--jobs", str(jobs),
        *profile_args(),
    ]


def tau_commands(repo: Path, binary: Path, root: Path, jobs: int) -> list[list[str]]:
    commands = []
    for tau in (1, 5):
        tau_root = root / "tau_sensitivity" / f"tau{tau}"
        commands.append(base_command(repo, binary, tau_root / "human_pbs", jobs) + [
            "--service-time", str(tau),
            "--seed-list", ",".join(str(seed) for seed in range(6, 26)),
            "--methods", METHODS["PBS"],
            "--alley-counts", "50,60",
            "--plaza-counts", "",
        ])
        commands.append(base_command(repo, binary, tau_root / "human_pibt", jobs) + [
            "--service-time", str(tau),
            "--seed-list", ",".join(str(seed) for seed in range(6, 26)),
            "--methods", METHODS["PIBT"],
            "--alley-counts", "100,140",
            "--plaza-counts", "",
        ])
        for layout in (2, 3, 4):
            commands.append(base_command(
                repo, binary,
                tau_root / f"sortation_medium_p20_layout{layout}", jobs,
            ) + [
                "--service-time", str(tau),
                "--seed-list", "6,7,8,9,10",
                "--pickup-layout-seed", str(layout),
                "--methods", METHODS["PIBT"],
                "--alley-counts", "",
                "--plaza-counts", "",
                "--lorr-sortation-medium-counts", "6000,8000",
                "--lorr-sortation-medium-benchmark",
                str(repo / "benchmarks" / "lorr" /
                    f"sortation_medium_p20_layout{layout}.json"),
                "--lorr-sortation-medium-name",
                f"sortation_medium_p20_layout{layout}",
            ])
    return commands


def density_commands(
    repo: Path, binary: Path, root: Path, density: str, jobs: int,
) -> list[list[str]]:
    counts = DENSITY_COUNTS[density]
    commands = []
    for layout in (2, 3, 4):
        command = base_command(
            repo, binary, root / "sortation_density" / density / f"layout{layout}", jobs,
        ) + [
            "--service-time", "3",
            "--seed-list", "6,7,8,9,10",
            "--pickup-layout-seed", str(layout),
            "--methods", METHODS["PIBT"],
            "--alley-counts", "",
            "--plaza-counts", "",
        ]
        for size, option in (
            ("small", "--lorr-sortation-counts"),
            ("medium", "--lorr-sortation-medium-counts"),
            ("large", "--lorr-sortation-large-counts"),
        ):
            command.extend([
                option, counts[size],
                option.replace("-counts", "-benchmark"),
                str(repo / "benchmarks" / "lorr" /
                    f"sortation_{size}_{density}_layout{layout}.json"),
                option.replace("-counts", "-name"),
                f"sortation_{size}_{density}_layout{layout}",
            ])
        commands.append(command)
    return commands


def p20_terminal_commands(
    repo: Path, binary: Path, root: Path, jobs: int,
) -> list[list[str]]:
    commands = []
    for layout in (2, 3, 4):
        command = base_command(
            repo, binary,
            root / "sortation_density" / "p20_terminal" / f"layout{layout}", jobs,
        ) + [
            "--service-time", "3",
            "--seed-list", "6,7,8,9,10",
            "--pickup-layout-seed", str(layout),
            "--methods", METHODS["PIBT"],
            "--alley-counts", "",
            "--plaza-counts", "",
            "--lorr-sortation-counts", "",
        ]
        for size, option in (
            ("medium", "--lorr-sortation-medium-counts"),
            ("large", "--lorr-sortation-large-counts"),
        ):
            command.extend([
                option, P20_TERMINALS[size],
                option.replace("-counts", "-benchmark"),
                str(repo / "benchmarks" / "lorr" /
                    f"sortation_{size}_p20_layout{layout}.json"),
                option.replace("-counts", "-name"),
                f"sortation_{size}_p20_layout{layout}",
            ])
        commands.append(command)
    return commands


def run_and_aggregate(repo: Path, command: list[str]) -> None:
    root = Path(command[command.index("--root") + 1])
    subprocess.run(command, cwd=repo, check=True)
    subprocess.run([
        sys.executable, str(repo / "scripts" / "aggregate_results.py"),
        "--root", str(root),
    ], cwd=repo, check=True)


def main() -> int:
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--root", type=Path,
        default=repo / "results" / "publication_extensions_v2",
    )
    parser.add_argument("--binary", type=Path, default=repo / "build-slot" / "lifelong")
    parser.add_argument(
        "--stage", choices=("tau", "p05", "p20-terminal", "p100", "all"),
        default="all",
    )
    parser.add_argument("--root-jobs", type=int, default=2)
    parser.add_argument("--case-jobs", type=int, default=4)
    args = parser.parse_args()

    root = args.root.resolve()
    binary = args.binary.resolve()
    if args.root_jobs < 1 or args.case_jobs < 1:
        parser.error("job counts must be positive")
    if not binary.exists():
        parser.error(f"missing binary: {binary}")

    root.mkdir(parents=True, exist_ok=True)
    digest = sha256(binary)
    artifact = root / "artifacts" / f"lifelong-{digest[:16]}"
    artifact.parent.mkdir(parents=True, exist_ok=True)
    if not artifact.exists():
        shutil.copy2(binary, artifact)
    if sha256(artifact) != digest:
        raise RuntimeError("archived binary hash mismatch")

    manifest = {
        "binary": str(artifact),
        "binary_sha256": digest,
        "method_profile": {
            "pressure_threshold": 2,
            "pressure_zone_cost": 2,
            "pressure_admission": "adaptive",
            "pressure_inbound_limit": 4,
            "pressure_cost_occupancy_threshold": 2,
            "pressure_population": "all_phases",
            "pressure_front_progress_cost": 3,
            "pressure_exit_progress_cost": 1,
            "pressure_ready_slot_priority": True,
            "pressure_relief": False,
        },
        "tau_conditions": {
            "values": [1, 5],
            "pbs_alley_counts": [50, 60],
            "pibt_alley_counts": [100, 140],
            "pibt_sortation_medium_p20_counts": [6000, 8000],
        },
        "density_counts": DENSITY_COUNTS,
        "p20_terminal_counts": P20_TERMINALS,
        "human_seeds": list(range(6, 26)),
        "sortation_layout_seeds": [2, 3, 4],
        "sortation_simulation_seeds": list(range(6, 11)),
    }
    (root / "extension_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n")

    commands = []
    if args.stage in {"tau", "all"}:
        commands.extend(tau_commands(repo, artifact, root, args.case_jobs))
    if args.stage in {"p05", "all"}:
        commands.extend(density_commands(repo, artifact, root, "p05", args.case_jobs))
    if args.stage in {"p20-terminal", "all"}:
        commands.extend(p20_terminal_commands(repo, artifact, root, args.case_jobs))
    if args.stage in {"p100", "all"}:
        commands.extend(density_commands(repo, artifact, root, "p100", args.case_jobs))

    with ThreadPoolExecutor(max_workers=args.root_jobs) as pool:
        futures = [pool.submit(run_and_aggregate, repo, command) for command in commands]
        for future in futures:
            future.result()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
