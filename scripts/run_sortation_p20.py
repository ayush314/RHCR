#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path


OPTION_PREFIX = {
    "small": "--lorr-sortation",
    "medium": "--lorr-sortation-medium",
    "large": "--lorr-sortation-large",
}
CANONICAL_METHODS = [
    "pibt_vanilla",
    "pibt_lead_aware",
    "pibt_pressure_aware",
]


def parse_int_list(value: str) -> list[int]:
    return [int(part.strip()) for part in value.split(",") if part.strip()]


def parse_name_list(value: str) -> list[str]:
    return [part.strip() for part in value.split(",") if part.strip()]


def comma_join(values: list[int] | list[str]) -> str:
    return ",".join(str(value) for value in values)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_config(repo: Path, config: dict[str, object]) -> None:
    methods = config.get("methods")
    if (not isinstance(methods, list) or not methods or
            methods != [method for method in CANONICAL_METHODS
                        if method in methods]):
        raise ValueError(
            "Sortation P20 methods must be a nonempty ordered subset of the "
            "finalized PIBT ladder")
    if int(config.get("pickup_retention_percent", 0)) != 20:
        raise ValueError("Sortation P20 config must retain exactly 20 percent of pickups")
    if int(config.get("pressure_privileged_inbound_count", -1)) < 0:
        raise ValueError("Sortation P20 config must define a nonnegative pressure K")

    maps = config.get("maps")
    if not isinstance(maps, dict) or set(maps) != set(OPTION_PREFIX):
        raise ValueError("Sortation P20 config must define small, medium, and large")
    layouts = [int(value) for value in config["pickup_layout_seeds"]]

    for map_name, raw_map in maps.items():
        if not isinstance(raw_map, dict):
            raise ValueError(f"Invalid map config for {map_name}")
        counts = [int(value) for value in raw_map["counts"]]
        if len(counts) != 20:
            raise ValueError(f"{map_name} must have exactly 20 agent counts")
        gaps = [right - left for left, right in zip(counts, counts[1:])]
        if not gaps or len(set(gaps)) != 1 or gaps[0] <= 0:
            raise ValueError(f"{map_name} counts must be strictly and equally spaced")
        capacity = int(raw_map["valid_start_capacity"])
        if counts[-1] >= capacity:
            raise ValueError(f"{map_name} final count must stay below valid-start capacity")

        template = str(raw_map["benchmark_template"])
        for layout in layouts:
            benchmark = repo / template.format(layout=layout)
            if not benchmark.exists():
                raise FileNotFoundError(benchmark)
            payload = json.loads(benchmark.read_text())
            if int(payload["adapter_pickup_retention_percent"]) != 20:
                raise ValueError(f"Unexpected pickup retention in {benchmark}")
            if int(payload["adapter_pickup_sample_seed"]) != layout:
                raise ValueError(f"Unexpected pickup-layout seed in {benchmark}")
            if int(payload["adapter_valid_start_capacity"]) != capacity:
                raise ValueError(f"Unexpected valid-start capacity in {benchmark}")


def build_layout_command(
    repo: Path,
    binary: Path,
    root: Path,
    config: dict[str, object],
    layout: int,
    maps: list[str],
    seeds: list[int],
    jobs: int,
    process_timeout: int,
    pressure_k: int,
    force: bool,
) -> list[str]:
    command = [
        sys.executable,
        str(repo / "scripts" / "run_comparison.py"),
        "--root",
        str(root / f"layout_{layout}"),
        "--binary",
        str(binary),
        "--seed-list",
        comma_join(seeds),
        "--pickup-layout-seed",
        str(layout),
        "--simulation-time",
        str(config["simulation_time"]),
        "--planning-window",
        str(config["planning_window"]),
        "--simulation-window",
        str(config["simulation_window"]),
        "--service-time",
        str(config["service_time"]),
        "--pressure-k",
        str(pressure_k),
        "--cutoff-time",
        str(config["cutoff_time"]),
        "--process-timeout",
        str(process_timeout),
        "--jobs",
        str(jobs),
        "--methods",
        comma_join(config["methods"]),
        "--alley-counts",
        "",
        "--plaza-counts",
        "",
        "--lra-fallback",
        "--pibt-random-tiebreak",
        "--stop-at-first-all-failed-count",
    ]
    map_configs = config["maps"]
    for map_name in maps:
        map_config = map_configs[map_name]
        prefix = OPTION_PREFIX[map_name]
        benchmark = repo / str(map_config["benchmark_template"]).format(layout=layout)
        command.extend(
            [
                f"{prefix}-counts",
                comma_join(map_config["counts"]),
                f"{prefix}-benchmark",
                str(benchmark),
                f"{prefix}-name",
                str(map_config["result_name"]),
            ]
        )
    if force:
        command.append("--force")
    return command


def main() -> int:
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Run the finalized 20-count PIBT evaluation on all P20 Sortation maps."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=repo / "configs" / "sortation_p20_pibt.json",
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=repo / "results" / "final_sortation_p20",
    )
    parser.add_argument("--binary", type=Path, default=repo / "lifelong")
    parser.add_argument("--maps", default="small,medium,large")
    parser.add_argument("--layouts", default="2,3,4")
    parser.add_argument("--seeds", default="6,7,8,9,10")
    parser.add_argument("--jobs-per-layout", type=int, default=6)
    parser.add_argument("--layout-jobs", type=int, default=3)
    parser.add_argument("--process-timeout", type=int)
    parser.add_argument("--pressure-k", type=int)
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--aggregate-only", action="store_true")
    args = parser.parse_args()

    config_path = args.config.resolve()
    config = json.loads(config_path.read_text())
    validate_config(repo, config)
    binary = args.binary.resolve()
    if not binary.exists():
        raise FileNotFoundError(binary)

    maps = parse_name_list(args.maps)
    if not maps or any(map_name not in OPTION_PREFIX for map_name in maps):
        parser.error("--maps must contain small, medium, and/or large")
    layouts = parse_int_list(args.layouts)
    configured_layouts = {int(value) for value in config["pickup_layout_seeds"]}
    if not layouts or any(layout not in configured_layouts for layout in layouts):
        parser.error("--layouts must be selected from the configured pickup-layout seeds")
    seeds = parse_int_list(args.seeds)
    if not seeds:
        parser.error("--seeds cannot be empty")
    if args.jobs_per_layout <= 0 or args.layout_jobs <= 0:
        parser.error("job counts must be positive")
    process_timeout = args.process_timeout or int(config["process_timeout"])
    if process_timeout <= 0:
        parser.error("--process-timeout must be positive")
    pressure_k = (
        args.pressure_k
        if args.pressure_k is not None
        else int(config["pressure_privileged_inbound_count"])
    )
    if pressure_k < 0:
        parser.error("--pressure-k must be nonnegative")

    root = args.root.resolve()
    root.mkdir(parents=True, exist_ok=True)
    manifest = {
        "config": {
            **config,
            "pressure_privileged_inbound_count": pressure_k,
        },
        "config_path": str(config_path),
        "binary": str(binary),
        "binary_sha256": sha256_file(binary),
        "selected_maps": maps,
        "selected_layouts": layouts,
        "selected_seeds": seeds,
        "jobs_per_layout": args.jobs_per_layout,
        "layout_jobs": args.layout_jobs,
        "process_timeout": process_timeout,
        "pressure_privileged_inbound_count": pressure_k,
        "started_at_unix": time.time(),
    }
    (root / "experiment_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n"
    )

    if not args.aggregate_only:
        commands = {
            layout: build_layout_command(
                repo,
                binary,
                root,
                config,
                layout,
                maps,
                seeds,
                args.jobs_per_layout,
                process_timeout,
                pressure_k,
                args.force,
            )
            for layout in layouts
        }
        if args.dry_run:
            for layout, command in commands.items():
                print(f"[layout {layout}] {' '.join(command)}")
            return 0

        def run_layout(layout: int) -> None:
            log_path = root / f"layout_{layout}.driver.log"
            print(f"[start] pickup layout {layout}: {log_path}", flush=True)
            with log_path.open("a") as log_file:
                completed = subprocess.run(
                    commands[layout],
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    check=False,
                )
            if completed.returncode != 0:
                raise RuntimeError(
                    f"pickup layout {layout} failed with exit code {completed.returncode}"
                )
            print(f"[done] pickup layout {layout}", flush=True)

        with ThreadPoolExecutor(max_workers=min(args.layout_jobs, len(layouts))) as pool:
            futures = [pool.submit(run_layout, layout) for layout in layouts]
            for future in futures:
                future.result()

    aggregate_command = [
        sys.executable,
        str(repo / "scripts" / "aggregate_results.py"),
        "--root",
        str(root),
    ]
    subprocess.run(aggregate_command, check=True)
    manifest["finished_at_unix"] = time.time()
    (root / "experiment_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
