import json
import sys
import tempfile
import unittest
from collections import Counter
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

import aggregate_results  # noqa: E402
import discover_publication_ladders  # noqa: E402
import import_lorr_workstation  # noqa: E402
import run_comparison  # noqa: E402
import run_sortation_density  # noqa: E402


class AggregateResultsTests(unittest.TestCase):
    def test_hierarchical_effects_preserve_layout_pairing(self) -> None:
        rows = []
        for layout in (2, 3):
            for seed in (6, 7):
                for method, value in (("pibt_pressure_aware", 1.0), ("pibt_lead_aware", 0.0)):
                    rows.append({
                        "map": "sortation_small_p20",
                        "agent_count": 100,
                        "method": method,
                        "pickup_layout_seed": layout,
                        "seed": seed,
                        "clean_completion": value,
                    })
        effects = aggregate_results.hierarchical_effect_rows(rows, iterations=100)
        clean = next(row for row in effects if row["metric"] == "clean_completion")
        self.assertEqual(clean["layout_seed_count"], 2)
        self.assertEqual(clean["paired_run_count"], 4)
        self.assertEqual(clean["mean_difference"], 1.0)
        self.assertEqual(clean["bootstrap_ci95_low"], 1.0)
        self.assertEqual(clean["bootstrap_ci95_high"], 1.0)

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

    def test_manifest_filter_prefers_evaluated_capacity_ladder(self) -> None:
        rows = [
            {"map": "alley", "agent_count": 62, "method": "pbs_vanilla", "seed": 1},
            {"map": "alley", "agent_count": 76, "method": "pbs_vanilla", "seed": 1},
        ]
        manifest = {
            "methods": ["pbs_vanilla"],
            "seeds": [1],
            "grids": {"alley": [62, 69, 76]},
            "evaluated_grids": {"alley": [62, 69]},
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

    def test_legacy_unreached_queue_p95_is_capped_at_run_horizon(self) -> None:
        rows = [
            {"queue_wait_km_p95": -1.0, "termination_timestep": 500.0},
            {"queue_wait_km_p95": 120.0, "termination_timestep": 500.0},
        ]
        aggregate_results.cap_legacy_queue_wait_at_horizon(rows)
        self.assertEqual(rows[0]["queue_wait_km_p95"], 500.0)
        self.assertEqual(rows[1]["queue_wait_km_p95"], 120.0)

    def test_stall_kaplan_meier_uses_actual_censor_times(self) -> None:
        curve = aggregate_results.kaplan_meier_curve([
            {"time_to_stall": 100, "stall_event": 1},
            {"time_to_stall": 200, "stall_event": 0},
            {"time_to_stall": 300, "stall_event": 1},
        ])
        self.assertEqual(curve[1]["at_risk"], 3)
        self.assertAlmostEqual(curve[1]["survival_probability"], 2 / 3)
        self.assertEqual(curve[2]["censored"], 1)
        self.assertEqual(curve[3]["at_risk"], 1)
        self.assertEqual(curve[3]["survival_probability"], 0.0)


class RunComparisonTests(unittest.TestCase):
    def test_publication_pibt_methods_use_public_core_port(self) -> None:
        for method in run_comparison.PUBLICATION_METHODS:
            if method.startswith("pibt_"):
                self.assertEqual(run_comparison.METHODS[method]["solver"], "PIBT2")
        self.assertEqual(run_comparison.METHODS["pibt_legacy_vanilla"]["solver"], "PIBT")

    def test_lead_aware_methods_are_publication_defaults(self) -> None:
        self.assertEqual(
            run_comparison.METHODS["pbs_lead_aware"],
            {"solver": "PBS", "station_policy": "lead_aware"},
        )
        self.assertEqual(
            run_comparison.METHODS["pibt_lead_aware"],
            {"solver": "PIBT2", "pibt_policy": "lead_aware"},
        )
        self.assertIn("pbs_lead_aware", run_comparison.PUBLICATION_METHODS)
        self.assertIn("pibt_lead_aware", run_comparison.PUBLICATION_METHODS)
        self.assertNotIn("pbs_departure_aware", run_comparison.PUBLICATION_METHODS)
        self.assertNotIn("pibt_departure_aware", run_comparison.PUBLICATION_METHODS)
        self.assertNotIn("pbs_phase_aware", run_comparison.METHODS)
        self.assertNotIn("pibt_phase_aware", run_comparison.METHODS)

    def test_pressure_manifest_records_aligned_timestep_semantics(self) -> None:
        self.assertEqual(
            run_comparison.PRESSURE_DEFINITION["pressure_evaluation"],
            "projected_each_step",
        )
        self.assertEqual(
            run_comparison.PRESSURE_DEFINITION["pressure_action_timing"],
            "state_t_scores_action_t_plus_1",
        )
        self.assertEqual(
            run_comparison.PRESSURE_DEFINITION["pressure_task_metadata"],
            "executed_only",
        )

    def test_human_capacity_ladders_match_the_shared_19_point_counts(self) -> None:
        alley = REPO_ROOT / "benchmarks" / "alley.json"
        plaza = REPO_ROOT / "benchmarks" / "plaza.json"
        self.assertEqual(run_comparison.workstation_start_capacity(alley), 158)
        self.assertEqual(run_comparison.workstation_start_capacity(plaza), 288)
        self.assertEqual(
            run_comparison.capacity_spaced_counts(158),
            list(range(20, 147, 7)),
        )
        self.assertEqual(
            run_comparison.capacity_spaced_counts(288),
            list(range(20, 273, 14)),
        )

    def test_publication_terminal_classification_keeps_runtime_distinct(self) -> None:
        stalled = [{"status": "failed", "failure_reason": "traffic_jam"}] * 3
        timed_out = [{"status": "failed", "failure_reason": "wall_timeout"}] * 3
        mixed = [stalled[0], timed_out[0], {"status": "clean", "failure_reason": "clean"}]
        self.assertEqual(
            discover_publication_ladders.classify_terminal(stalled),
            (True, "empirical_stall"),
        )
        self.assertEqual(
            discover_publication_ladders.classify_terminal(timed_out),
            (True, "runtime_limit"),
        )
        self.assertEqual(
            discover_publication_ladders.classify_terminal(mixed),
            (False, "none"),
        )
        native = [{"status": "failed", "failure_reason": "solver_failure"}] * 3
        self.assertEqual(
            discover_publication_ladders.classify_terminal(native),
            (True, "native_failure_boundary"),
        )

    def test_publication_counts_are_even_and_below_terminal(self) -> None:
        counts = discover_publication_ladders.frozen_counts(8000, 16000)
        self.assertEqual(counts, [1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 16000])

    def test_publication_discovery_backs_off_from_failed_start(self) -> None:
        self.assertEqual(
            discover_publication_ladders.descending_probes(2000, 500),
            [1000, 500],
        )

    def test_classification_requires_explicit_clean_completion(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            log = root / "run.log"
            log.write_text("")
            summary = root / "summary.csv"
            summary.write_text("clean_completion,termination_reason\n0,invalid_execution\n")
            self.assertEqual(
                run_comparison.classify_run(log, summary, 0, False),
                ("failed", "invalid_execution"),
            )
            summary.write_text("clean_completion,termination_reason\n1,completed_simulation\n")
            self.assertEqual(
                run_comparison.classify_run(log, summary, 0, False),
                ("clean", "clean"),
            )

    def test_signal_terminated_status_is_not_reused(self) -> None:
        signature = {"seed": 1}
        self.assertFalse(
            run_comparison.status_is_reusable(
                {"status": "failed", "return_code": -2, "run_signature": signature},
                signature,
            )
        )
        self.assertTrue(
            run_comparison.status_is_reusable(
                {"status": "failed", "return_code": 0, "run_signature": signature},
                signature,
            )
        )

    def test_all_cells_failed_requires_every_completed_cell_to_fail(self) -> None:
        self.assertTrue(run_comparison.all_cells_failed([
            {"status": "failed"}, {"status": "failed"},
        ]))
        self.assertFalse(run_comparison.all_cells_failed([
            {"status": "failed"}, {"status": "clean"},
        ]))
        self.assertFalse(run_comparison.all_cells_failed([
            {"status": "failed"}, None,
        ]))

    def test_reset_run_outputs_removes_append_only_diagnostics(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            cell_dir = Path(temp_dir)
            for name in run_comparison.RUN_OUTPUT_FILES:
                (cell_dir / name).write_text("stale")
            keep = cell_dir / "notes.txt"
            keep.write_text("keep")
            run_comparison.reset_run_outputs(cell_dir)
            self.assertTrue(keep.exists())
            self.assertFalse(any((cell_dir / name).exists() for name in run_comparison.RUN_OUTPUT_FILES))

    def test_benchmark_fingerprint_includes_movingai_dependency(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            (root / "layout.map").write_text("type octile\nheight 1\nwidth 1\nmap\n.\n")
            benchmark = root / "benchmark.json"
            benchmark.write_text(json.dumps({"movingai_map": "layout.map"}))
            fingerprints = run_comparison.benchmark_fingerprints(benchmark)
        self.assertEqual(set(fingerprints), {"benchmark.json", "layout.map"})
        self.assertTrue(all(len(value) == 64 for value in fingerprints.values()))

    def test_density_frontier_requires_every_method_and_seed_to_fail(self) -> None:
        failed = {"status": "failed"}
        clean = {"status": "clean"}
        statuses = {method: [failed, failed] for method in run_sortation_density.METHODS}
        self.assertTrue(run_sortation_density.all_methods_failed(statuses))
        statuses["pibt_pressure"] = [failed, clean]
        self.assertFalse(run_sortation_density.all_methods_failed(statuses))

    def test_run_signature_ignores_outer_batch_parallelism(self) -> None:
        expected = {"method": "pibt_pressure_aware", "seed": 1}
        existing = {**expected, "batch_jobs": 6}
        self.assertTrue(run_comparison.signatures_match(existing, expected))
        self.assertFalse(
            run_comparison.signatures_match({**existing, "seed": 2}, expected)
        )

    def test_density_coarse_probes_are_ordered_bounded_and_include_capacity(self) -> None:
        probes = run_sortation_density.coarse_probe_counts(50, 1106, 10)
        self.assertEqual(probes, sorted(set(probes)))
        self.assertEqual(probes[0], 50)
        self.assertEqual(probes[-1], 1106)
        self.assertTrue(all(50 <= count <= 1106 for count in probes))

    def test_density_frontier_reuses_known_clean_status(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            path = run_sortation_density.status_path(
                root, "large", 5, 22750, "pibt_pressure_aware", 2
            )
            path.parent.mkdir(parents=True)
            path.write_text(json.dumps({"status": "clean"}))
            self.assertTrue(
                run_sortation_density.has_known_clean_status(
                    root, "large", 5, 22750, list(range(1, 11))
                )
            )
            self.assertFalse(
                run_sortation_density.has_known_clean_status(
                    root, "large", 5, 22750, [1]
                )
            )

    def test_capped_frontier_diagnostics_load_only_evaluated_seeds(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            for method in run_sortation_density.METHODS:
                for seed in (1, 2, 3):
                    path = run_sortation_density.status_path(
                        root, "large", 10, 25000, method, seed
                    )
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_text(json.dumps({"status": "clean"}))
            statuses = run_sortation_density.load_cell_statuses(
                root,
                "large",
                10,
                25000,
                list(range(1, 11)),
                require_all=False,
            )
        self.assertEqual(
            {method: len(entries) for method, entries in statuses.items()},
            {method: 3 for method in run_sortation_density.METHODS},
        )


class LorrAdapterTests(unittest.TestCase):
    def assert_directional_station_layout(self, benchmark: dict, per_side: int) -> None:
        self.assertEqual(benchmark["adapter_station_layout"], "balanced_perimeter")
        self.assertEqual(benchmark["adapter_queue_layout"], "inward_lane_3")
        self.assertEqual(
            Counter(station["perimeter_side"] for station in benchmark["stations"]),
            Counter({"top": per_side, "right": per_side, "bottom": per_side, "left": per_side}),
        )
        occupied: set[tuple[int, int]] = set()
        for station in benchmark["stations"]:
            workstation = tuple(station["workstation_cell"])
            approach = {tuple(cell) for cell in station["approach_cells"]}
            buffer_cells = {tuple(cell) for cell in station["buffer_cells"]}
            standby = {tuple(cell) for cell in station["standby_cells"]}
            exits = {tuple(cell) for cell in station["exit_cells"]}
            self.assertEqual((len(approach), len(buffer_cells), len(standby), len(exits)), (1, 1, 1, 1))
            approach_cell = next(iter(approach))
            buffer_cell = next(iter(buffer_cells))
            standby_cell = next(iter(standby))
            exit_cell = next(iter(exits))
            manhattan = lambda first, second: abs(first[0] - second[0]) + abs(first[1] - second[1])
            self.assertEqual(manhattan(approach_cell, buffer_cell), 1)
            self.assertEqual(manhattan(buffer_cell, standby_cell), 1)
            self.assertEqual(manhattan(standby_cell, workstation), 1)
            self.assertEqual(manhattan(exit_cell, workstation), 1)
            station_cells = {workstation} | approach | buffer_cells | standby | exits
            self.assertEqual(len(station_cells), 5)
            self.assertFalse(occupied & station_cells)
            occupied.update(station_cells)

    def assert_centered_funnel_layout(
        self,
        benchmark: dict,
        expected_side_counts: dict[str, int],
    ) -> None:
        self.assertEqual(benchmark["adapter_station_layout"], "maximal_nonoverlapping_perimeter")
        self.assertEqual(benchmark["adapter_queue_layout"], "centered_funnel_3x3")
        self.assertEqual(
            Counter(station["perimeter_side"] for station in benchmark["stations"]),
            Counter(expected_side_counts),
        )
        occupied: set[tuple[int, int]] = set()
        for station in benchmark["stations"]:
            workstation = tuple(station["workstation_cell"])
            approach = {tuple(cell) for cell in station["approach_cells"]}
            buffer_cells = {tuple(cell) for cell in station["buffer_cells"]}
            standby = {tuple(cell) for cell in station["standby_cells"]}
            exits = {tuple(cell) for cell in station["exit_cells"]}
            self.assertEqual((len(approach), len(buffer_cells), len(standby), len(exits)), (3, 3, 3, 2))
            station_cells = {workstation} | approach | buffer_cells | standby | exits
            self.assertEqual(len(station_cells), 12)
            self.assertFalse(occupied & station_cells)
            occupied.update(station_cells)

            side = station["perimeter_side"]
            if side in {"top", "bottom"}:
                self.assertEqual({cell[0] for cell in standby}, {workstation[0] - 1, workstation[0], workstation[0] + 1})
                self.assertEqual({cell[0] for cell in exits}, {workstation[0] - 1, workstation[0] + 1})
            else:
                self.assertEqual({cell[1] for cell in standby}, {workstation[1] - 1, workstation[1], workstation[1] + 1})
                self.assertEqual({cell[1] for cell in exits}, {workstation[1] - 1, workstation[1] + 1})

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
            "https://github.com/MAPF-Competition/Benchmark-Archive/blob/main/"
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
        self.assert_directional_station_layout(generated, 3)

        _rows, _cols, grid = import_lorr_workstation.read_movingai_map(map_path)
        traversable = sum(import_lorr_workstation.traversable(cell) for row in grid for cell in row)
        reserved = {tuple(cell) for cell in generated["pickup_endpoints"]}
        for station in generated["stations"]:
            reserved.add(tuple(station["workstation_cell"]))
            for field in ("standby_cells", "buffer_cells", "approach_cells", "exit_cells"):
                reserved.update(tuple(cell) for cell in station[field])
        self.assertEqual(traversable - len(reserved), 987)

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
        self.assert_directional_station_layout(generated, 6)

        _rows, _cols, grid = import_lorr_workstation.read_movingai_map(map_path)
        traversable = sum(import_lorr_workstation.traversable(cell) for row in grid for cell in row)
        reserved = {tuple(cell) for cell in generated["pickup_endpoints"]}
        for station in generated["stations"]:
            reserved.add(tuple(station["workstation_cell"]))
            for field in ("standby_cells", "buffer_cells", "approach_cells", "exit_cells"):
                reserved.update(tuple(cell) for cell in station[field])
        self.assertEqual(traversable - len(reserved), 21288)

    def test_sortation_density_sidecars_are_nested_reproducible_and_centered(self) -> None:
        cases = {
            "sortation_small": {
                "source_url": (
                    "https://github.com/MAPF-Competition/Benchmark-Archive/blob/main/"
                    "2023%20Competition/Problem%20Generator/script/sortation_small.map"
                ),
                "pickup_counts": [26, 52, 103, 259, 517],
                "capacities": [1106, 1080, 1029, 873, 615],
                "side_counts": {"top": 12, "right": 6, "bottom": 12, "left": 6},
            },
            "sortation_medium": {
                "source_url": (
                    "https://github.com/MAPF-Competition/Benchmark-Archive/blob/main/"
                    "2023%20Competition/Problem%20Generator/script/sortation_medium.map"
                ),
                "pickup_counts": [605, 1210, 2419, 6048, 12096],
                "capacities": [19371, 18766, 17557, 13928, 7880],
                "side_counts": {"top": 48, "right": 33, "bottom": 48, "left": 33},
            },
            "sortation_large": {
                "source_url": (
                    "https://github.com/MAPF-Competition/Benchmark-Archive/blob/main/"
                    "2023%20Competition/Problem%20Generator/script/sortation_large.map"
                ),
                "pickup_counts": [1565, 3130, 6259, 15648, 31296],
                "capacities": [49011, 47446, 44317, 34928, 19280],
                "side_counts": {"top": 123, "right": 33, "bottom": 123, "left": 33},
            },
        }
        retentions = [5, 10, 20, 50, 100]
        for map_name, expected in cases.items():
            map_path = REPO_ROOT / "benchmarks" / "lorr" / f"{map_name}.map"
            previous_pickups: set[tuple[int, int]] = set()
            for index, retention in enumerate(retentions):
                description = (
                    f"LoRR {map_name.replace('_', ' ').title()} with centered workstation queues "
                    f"and {retention}% nested pickup retention."
                )
                generated = import_lorr_workstation.build_sortation_density_benchmark(
                    map_path,
                    retention,
                    1,
                    expected["source_url"],
                    description,
                )
                sidecar_path = REPO_ROOT / "benchmarks" / "lorr" / f"{map_name}_p{retention:02d}.json"
                self.assertEqual(generated, json.loads(sidecar_path.read_text()))
                self.assertEqual(generated["adapter_pickup_count"], expected["pickup_counts"][index])
                self.assertEqual(generated["adapter_valid_start_capacity"], expected["capacities"][index])
                self.assert_centered_funnel_layout(generated, expected["side_counts"])
                pickups = {tuple(cell) for cell in generated["pickup_endpoints"]}
                self.assertTrue(previous_pickups <= pickups)
                previous_pickups = pickups

    def test_warehouse_sidecar_has_balanced_directional_stations(self) -> None:
        map_path = REPO_ROOT / "benchmarks" / "lorr" / "warehouse_small.map"
        sidecar_path = REPO_ROOT / "benchmarks" / "lorr" / "warehouse_small.json"
        expected = json.loads(sidecar_path.read_text())
        generated = import_lorr_workstation.build_benchmark(
            map_path,
            12,
            expected["source"],
            expected["description"],
        )
        self.assertEqual(generated, expected)
        self.assert_directional_station_layout(generated, 3)

        _rows, _cols, grid = import_lorr_workstation.read_movingai_map(map_path)
        traversable = sum(import_lorr_workstation.traversable(cell) for row in grid for cell in row)
        reserved = {tuple(cell) for cell in generated["pickup_endpoints"]}
        for station in generated["stations"]:
            reserved.add(tuple(station["workstation_cell"]))
            for field in ("standby_cells", "buffer_cells", "approach_cells", "exit_cells"):
                reserved.update(tuple(cell) for cell in station[field])
        self.assertEqual(traversable - len(reserved), 875)


if __name__ == "__main__":
    unittest.main()
