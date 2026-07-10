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


if __name__ == "__main__":
    unittest.main()
