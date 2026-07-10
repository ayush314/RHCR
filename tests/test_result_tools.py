import json
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import aggregate_results  # noqa: E402
import import_lorr_workstation  # noqa: E402
import run_comparison  # noqa: E402


class AggregateResultsTests(unittest.TestCase):
    def test_manifest_filter_excludes_stale_cells(self) -> None:
        rows = [
            {"map": "alley", "agent_count": 20, "method": "pibt_pressure", "seed": 1},
            {"map": "alley", "agent_count": 40, "method": "pibt_pressure", "seed": 1},
            {"map": "alley", "agent_count": 20, "method": "pibt_vanilla", "seed": 1},
            {"map": "alley", "agent_count": 20, "method": "pibt_pressure", "seed": 2},
        ]
        manifest = {
            "methods": ["pibt_pressure"],
            "seeds": [1],
            "grids": {"alley": [20]},
        }
        with tempfile.TemporaryDirectory() as temp_dir:
            manifest_path = Path(temp_dir) / "run_manifest.json"
            manifest_path.write_text(json.dumps(manifest))
            filtered = aggregate_results.filter_manifest_cells(rows, manifest_path)
        self.assertEqual(filtered, rows[:1])

    def test_paired_ci_is_zero_for_identical_differences(self) -> None:
        self.assertEqual(aggregate_results.ci95_halfwidth([2.0, 2.0, 2.0]), 0.0)

    def test_runtime_tail_backfills_legacy_summary(self) -> None:
        metrics = {"plan_runtime_p95_ms": "", "plan_runtime_max_ms": ""}
        with tempfile.TemporaryDirectory() as temp_dir:
            runtime_path = Path(temp_dir) / "planning_runtime.csv"
            runtime_path.write_text(
                "episode,timestep,plan_ms\n"
                "0,0,1\n"
                "1,5,2\n"
                "2,10,10\n"
            )
            aggregate_results.backfill_runtime_tail(metrics, runtime_path)
        self.assertAlmostEqual(metrics["plan_runtime_p95_ms"], 9.2)
        self.assertEqual(metrics["plan_runtime_max_ms"], 10.0)

    def test_cross_root_comparison_pairs_only_clean_matching_seeds(self) -> None:
        reference = [
            {"map": "alley", "agent_count": 20, "method": "pibt_pressure", "seed": 1,
             "status": "clean", "service_rate": 5.0},
            {"map": "alley", "agent_count": 20, "method": "pibt_pressure", "seed": 2,
             "status": "clean", "service_rate": 7.0},
        ]
        baseline = [
            {"map": "alley", "agent_count": 20, "method": "pibt_pressure", "seed": 1,
             "status": "clean", "service_rate": 3.0},
            {"map": "alley", "agent_count": 20, "method": "pibt_pressure", "seed": 2,
             "status": "failed", "service_rate": 4.0},
        ]
        rows = aggregate_results.paired_root_comparison_rows(
            reference, baseline, "front", "no_front"
        )
        service = next(row for row in rows if row["metric"] == "service_rate")
        self.assertEqual(service["paired_seed_count"], 1)
        self.assertEqual(service["baseline_clean_seed_count"], 1)
        self.assertEqual(service["mean_difference"], 2.0)

    def test_fallback_rate_normalizes_planned_agent_steps(self) -> None:
        rows = [
            {
                "agent_count": 100,
                "termination_timestep": 500,
                "pibt_wait_fallbacks": 200,
            }
        ]
        manifest = {"simulation_window": 5, "planning_window": 20}
        with tempfile.TemporaryDirectory() as temp_dir:
            manifest_path = Path(temp_dir) / "run_manifest.json"
            manifest_path.write_text(json.dumps(manifest))
            aggregate_results.derive_fallback_rate(rows, manifest_path)
        self.assertEqual(rows[0]["pibt_wait_fallback_rate_per_1000_agent_steps"], 1.0)


class RunComparisonTests(unittest.TestCase):
    def test_benchmark_fingerprint_includes_movingai_dependency(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            (root / "layout.map").write_text("type octile\nheight 1\nwidth 1\nmap\n.\n")
            benchmark = root / "benchmark.json"
            benchmark.write_text(json.dumps({"movingai_map": "layout.map"}))
            fingerprints = run_comparison.benchmark_fingerprints(benchmark)
        self.assertEqual(set(fingerprints), {"benchmark.json", "layout.map"})
        self.assertTrue(all(len(value) == 64 for value in fingerprints.values()))


class LorrAdapterTests(unittest.TestCase):
    def test_spaced_selection_matches_reference_farthest_point_rule(self) -> None:
        candidates = [(x, y) for y in range(4) for x in range(7)]
        expected = [min(candidates)]
        remaining = set(candidates) - set(expected)
        while len(expected) < 8:
            selected = max(
                remaining,
                key=lambda cell: (
                    min(
                        abs(cell[0] - other[0]) + abs(cell[1] - other[1])
                        for other in expected
                    ),
                    -cell[1],
                    -cell[0],
                ),
            )
            expected.append(selected)
            remaining.remove(selected)
        expected.sort(key=lambda cell: (cell[1], cell[0]))
        self.assertEqual(import_lorr_workstation.select_spaced(candidates, 8), expected)

    def test_sortation_sidecar_is_reproducible_and_has_expected_capacity(self) -> None:
        map_path = REPO_ROOT / "benchmarks" / "lorr" / "sortation_small.map"
        sidecar_path = REPO_ROOT / "benchmarks" / "lorr" / "sortation_small.json"
        source_url = (
            "https://github.com/MAPF-Competition/Benchmark-Archive/tree/main/"
            "2023%20Competition/Problem%20Generator/script/sortation_small.map"
        )
        description = (
            "LoRR sortation map adapted to alternating storage-pickup and serviced-emitter tasks."
        )
        generated = import_lorr_workstation.build_benchmark(
            map_path,
            12,
            source_url,
            description,
        )
        self.assertEqual(generated, json.loads(sidecar_path.read_text()))

        _rows, _cols, grid = import_lorr_workstation.read_movingai_map(map_path)
        traversable = sum(import_lorr_workstation.traversable(cell) for row in grid for cell in row)
        reserved = {tuple(cell) for cell in generated["pickup_endpoints"]}
        for station in generated["stations"]:
            reserved.add(tuple(station["workstation_cell"]))
            for field in ("standby_cells", "buffer_cells", "approach_cells", "exit_cells"):
                reserved.update(tuple(cell) for cell in station[field])
        self.assertEqual(traversable - len(reserved), 870)

    def test_sortation_medium_sidecar_is_reproducible_and_has_expected_capacity(self) -> None:
        map_path = REPO_ROOT / "benchmarks" / "lorr" / "sortation_medium.map"
        sidecar_path = REPO_ROOT / "benchmarks" / "lorr" / "sortation_medium.json"
        source_url = (
            "https://github.com/MAPF-Competition/Benchmark-Archive/blob/main/"
            "2023%20Competition/Problem%20Generator/script/sortation_medium.map"
        )
        description = (
            "LoRR medium sortation map adapted to alternating spaced storage-pickup and "
            "serviced-emitter tasks for thousand-agent scaling."
        )
        generated = import_lorr_workstation.build_benchmark(
            map_path,
            24,
            source_url,
            description,
            512,
        )
        self.assertEqual(generated, json.loads(sidecar_path.read_text()))

        _rows, _cols, grid = import_lorr_workstation.read_movingai_map(map_path)
        traversable = sum(import_lorr_workstation.traversable(cell) for row in grid for cell in row)
        reserved = {tuple(cell) for cell in generated["pickup_endpoints"]}
        for station in generated["stations"]:
            reserved.add(tuple(station["workstation_cell"]))
            for field in ("standby_cells", "buffer_cells", "approach_cells", "exit_cells"):
                reserved.update(tuple(cell) for cell in station[field])
        self.assertEqual(traversable - len(reserved), 21030)


if __name__ == "__main__":
    unittest.main()
