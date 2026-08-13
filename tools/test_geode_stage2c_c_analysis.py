#!/usr/bin/env python3

import csv
import importlib.util
import json
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parent


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ANALYSIS = load_module("geode_stage2c_c_analysis",
                       ROOT / "geode_stage2c_c_analysis.py")
PILOT = load_module("geode_stage2c_b_evaluator",
                    ROOT / "geode_stage2c_b_evaluator.py")
PLAN_PATH = ROOT.parent / "config" / "geode_stage2c_c_analysis_plan.json"


FIELDS = [
    "schema_version", "configuration_fingerprint", "sensor_identifier",
    "registration_type", "pose_ordering", "tangent_convention", "timestamp",
    "frame_id", "record_kind", "factor_id", "resolution", "valid",
    "factorization_ok", "registration_converged", "linear_solve_success",
    "reference_ready", "health_available", "reference_source",
    "source_point_count", "inlier_count", "inlier_fraction", "initial_cost",
    "final_cost", "rotation_condition_ratio_0", "rotation_condition_ratio_1",
    "rotation_condition_ratio_2", "translation_condition_ratio_0",
    "translation_condition_ratio_1", "translation_condition_ratio_2",
    "absolute_rotation_mask_0", "absolute_rotation_mask_1",
    "absolute_rotation_mask_2", "absolute_translation_mask_0",
    "absolute_translation_mask_1", "absolute_translation_mask_2",
    "baseline_update_accepted", "baseline_update_reason",
]


def aggregate(frame, timestamp=None, rotation=4.0, translation=4.0,
              accepted=True, ready=False, fingerprint="gamma", valid=True,
              absolute_rotation=False, absolute_translation=False):
    timestamp = frame * 0.1 if timestamp is None else timestamp
    row = {field: "" for field in FIELDS}
    row.update({
        "schema_version": "2", "configuration_fingerprint": fingerprint,
        "sensor_identifier": "geode_gamma_livox_xsens",
        "registration_type": "VGICP", "pose_ordering": "rotation_translation",
        "tangent_convention": "local_right", "timestamp": str(timestamp),
        "frame_id": str(frame), "record_kind": "aggregate", "factor_id": "-1",
        "resolution": "nan", "valid": str(int(valid)),
        "factorization_ok": str(int(valid)), "registration_converged": "1",
        "linear_solve_success": "1", "reference_ready": str(int(ready)),
        "health_available": str(int(ready)), "reference_source": "session",
        "source_point_count": "500", "inlier_count": "250",
        "inlier_fraction": "0.5", "initial_cost": "100", "final_cost": "50",
        "baseline_update_accepted": str(int(accepted)),
        "baseline_update_reason": "session_reference_initialized" if ready else "bootstrap_sample_accepted",
    })
    for index, scale in enumerate((1.0, 0.75, 0.5)):
        row[f"rotation_condition_ratio_{index}"] = str(rotation * scale)
        row[f"translation_condition_ratio_{index}"] = str(translation * scale)
        row[f"absolute_rotation_mask_{index}"] = str(int(absolute_rotation and index == 0))
        row[f"absolute_translation_mask_{index}"] = str(int(absolute_translation and index == 0))
    return row


def factor(frame, resolution):
    row = aggregate(frame)
    row["record_kind"] = "factor"
    row["factor_id"] = "0" if resolution == 0.5 else "1"
    row["resolution"] = str(resolution)
    row["baseline_update_accepted"] = "0"
    return row


def write_health(path, rows, include_factors=True):
    output = []
    for row in rows:
        output.append(row)
        if include_factors:
            output.extend((factor(int(row["frame_id"]), 0.5),
                           factor(int(row["frame_id"]), 1.0)))
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(output)


class GeodeStage2cCAnalysisTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.plan = json.loads(PLAN_PATH.read_text())
        self.plan["temporal_block_bootstrap"]["replicates"] = 100

    def tearDown(self):
        self.temporary.cleanup()

    def healthy_rows(self):
        return [aggregate(frame, accepted=frame <= 30, ready=frame >= 30)
                for frame in range(1, 36)]

    def test_healthy_stable_candidate_qualifies(self):
        path = self.root / "healthy.csv"
        write_health(path, self.healthy_rows())
        result = ANALYSIS.reference_screening(path, self.plan, "gamma")
        self.assertTrue(result["qualified"])
        self.assertEqual(result["bootstrap_accepted_candidate_count"], 30)

    def test_isolated_accepted_frames_do_not_qualify(self):
        rows = [aggregate(frame, timestamp=float(frame), ready=frame >= 30)
                for frame in range(1, 36)]
        path = self.root / "isolated.csv"
        write_health(path, rows)
        result = ANALYSIS.reference_screening(path, self.plan, "gamma")
        self.assertFalse(result["qualified"])
        self.assertFalse(result["qualification_checks"]["maximum_accepted_gap"])

    def test_absolute_degenerate_startup_does_not_qualify(self):
        rows = [aggregate(frame, rotation=30.0, translation=80.0,
                          accepted=False, ready=False, absolute_rotation=True,
                          absolute_translation=True) for frame in range(1, 50)]
        path = self.root / "degraded.csv"
        write_health(path, rows)
        result = ANALYSIS.reference_screening(path, self.plan, "gamma")
        self.assertFalse(result["qualified"])
        self.assertEqual(result["accepted_reference_candidate_count"], 0)

    def test_support_is_kept_separate_by_vgicp_resolution(self):
        path = self.root / "support.csv"
        write_health(path, self.healthy_rows())
        result = ANALYSIS.reference_screening(path, self.plan, "gamma")
        support = result["per_resolution_support_and_matching"]
        self.assertEqual(set(support), {"0.5", "1"})
        self.assertEqual(support["0.5"]["source_point_count"]["median"], 500.0)
        self.assertEqual(support["1"]["inlier_count"]["median"], 250.0)
        self.assertNotIn("source_point_count", result)

    def test_profile_fingerprint_must_match(self):
        path = self.root / "healthy.csv"
        write_health(path, self.healthy_rows())
        self.assertFalse(ANALYSIS.reference_screening(
            path, self.plan, "different")["qualified"])

    def test_incompatible_profile_is_rejected_by_qualification(self):
        path = self.root / "healthy.csv"
        write_health(path, self.healthy_rows())
        result = ANALYSIS.reference_screening(path, self.plan, "inland")
        self.assertFalse(result["qualification_checks"]
                         ["configuration_fingerprint_match"])

    def test_profile_export_preserves_ratios_and_spread(self):
        path = self.root / "healthy.csv"
        write_health(path, self.healthy_rows())
        output = self.root / "profile.json"
        args = SimpleNamespace(
            plan=PLAN_PATH, dataset="synthetic", health_csv=path,
            expected_fingerprint="gamma", gamma_calibration_sha256="a" * 64,
            adapter_manifest_sha256="b" * 64,
            glim_config_manifest_sha256="c" * 64, output=output)
        profile = ANALYSIS.export_profile(args)
        loaded = json.loads(output.read_text())
        self.assertEqual(profile["rotation_log_condition_ratios"],
                         loaded["rotation_log_condition_ratios"])
        self.assertEqual(len(loaded["translation_log_condition_mad"]), 3)
        self.assertFalse(loaded["provenance"]["ground_truth_used"])

    def test_degenerate_inland_rows_cannot_change_exported_profile(self):
        path = self.root / "healthy.csv"
        write_health(path, self.healthy_rows())
        before = ANALYSIS.reference_screening(path, self.plan, "gamma")
        degraded = self.healthy_rows() + [aggregate(
            frame, rotation=100.0, translation=100.0, accepted=False, ready=True)
            for frame in range(36, 50)]
        changed_path = self.root / "with_degraded.csv"
        write_health(changed_path, degraded)
        after = ANALYSIS.reference_screening(changed_path, self.plan, "gamma")
        self.assertEqual(before["bootstrap_accepted_candidate_count"],
                         after["bootstrap_accepted_candidate_count"])

    def test_no_reference_keeps_relative_health_unavailable(self):
        row = aggregate(1, accepted=False, ready=False)
        self.assertFalse(ANALYSIS.truth(row["health_available"]))

    def test_unique_stable_edge_deduplication(self):
        rows = [
            {"belief_index": "0", "message_index": "0", "from_pose_index": "1",
             "to_pose_index": "2", "from_stamp_sec": "1", "to_stamp_sec": "2"},
            {"belief_index": "0", "message_index": "1", "from_pose_index": "1",
             "to_pose_index": "2", "from_stamp_sec": "1", "to_stamp_sec": "2"},
        ]
        selected, summary = PILOT.select_last_stable_edges(rows)
        self.assertEqual(len(selected), 1)
        self.assertEqual(selected[0]["message_index"], "1")
        self.assertEqual(summary["republication_row_count"], 1)

    def test_temporal_blocks_are_deterministic(self):
        rows = [{"to_stamp_sec": value} for value in (0.0, 9.9, 10.0, 21.0)]
        first, origin_a = ANALYSIS.block_groups(rows, 10.0)
        second, origin_b = ANALYSIS.block_groups(rows, 10.0)
        self.assertEqual(list(first), [0, 1, 2])
        self.assertEqual(first, second)
        self.assertEqual(origin_a, origin_b)

    def test_conditioning_bins_are_deterministic(self):
        config = self.plan["conditioning_bins"]
        self.assertEqual(ANALYSIS.conditioning_bin(
            10.0, config["boundaries"], config["labels"]), "[10,30)")
        self.assertEqual(ANALYSIS.conditioning_bin(
            300.0, config["boundaries"], config["labels"]), "[300,inf)")

    def test_spearman_known_monotonic_data(self):
        self.assertAlmostEqual(ANALYSIS.spearman([1, 2, 3, 4], [10, 20, 30, 40]), 1.0)
        self.assertAlmostEqual(ANALYSIS.spearman([1, 2, 3, 4], [40, 30, 20, 10]), -1.0)

    def test_narrow_dynamic_range_warning(self):
        rows = [{"c": 20.0 + index * 0.001, "bin": "[10,30)"}
                for index in range(100)]
        result = ANALYSIS.dynamic_range(rows, "c", "bin", self.plan)
        self.assertFalse(result["informative"])

    def test_rotation_translation_metrics_remain_separate(self):
        edge = {"from_pose_index": "0", "to_pose_index": "1"}
        indexed = {1: aggregate(1, rotation=4.0, translation=8.0)}
        summary = ANALYSIS.edge_scan_summary(edge, indexed, [0, 1])
        self.assertEqual(summary["stage2c_c_max_rotation_condition_ratio"], 4.0)
        self.assertEqual(summary["stage2c_c_max_translation_condition_ratio"], 8.0)

    def test_no_ratio_to_error_regression_is_emitted(self):
        forbidden = ("regression_coefficient", "predicted_metres", "covariance_multiplier")
        source = (ROOT / "geode_stage2c_c_analysis.py").read_text()
        for key in forbidden:
            self.assertNotIn(f'"{key}"', source)

    def test_ground_truth_is_not_an_input_to_reference_screening(self):
        self.assertEqual(ANALYSIS.reference_screening.__code__.co_varnames[:3],
                         ("path", "plan", "expected_fingerprint"))

    def test_edge_association_excludes_start_and_includes_end(self):
        indexed = {1: aggregate(1, rotation=2.0), 2: aggregate(2, rotation=9.0)}
        edge = {"from_pose_index": "1", "to_pose_index": "2"}
        result = ANALYSIS.edge_scan_summary(edge, indexed, [0, 1, 2])
        self.assertEqual(result["expected_health_sample_count"], 1)
        self.assertEqual(result["stage2c_c_max_rotation_condition_ratio"], 9.0)

    def test_invalid_sample_breaks_absolute_run(self):
        indexed = {
            1: aggregate(1, absolute_translation=True),
            2: aggregate(2, valid=False, absolute_translation=True),
            3: aggregate(3, absolute_translation=True),
        }
        edge = {"from_pose_index": "0", "to_pose_index": "3"}
        result = ANALYSIS.edge_scan_summary(edge, indexed, [0, 1, 2, 3])
        self.assertEqual(result["stage2c_c_longest_translation_absolute_degenerate_run"], 1)

    def test_passivity_plan_prohibits_active_paths(self):
        text = json.dumps(self.plan)
        self.assertIn("covariance multiplier", text)
        self.assertIn("active factor weighting", text)
        self.assertTrue(self.plan["inland_offline_profile_primary_policy"]
                        ["inland_may_not_change_reference"])


if __name__ == "__main__":
    unittest.main()
