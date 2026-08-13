#!/usr/bin/env python3

import importlib.util
import argparse
import csv
import json
import math
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("dcreg_shadow_stage2b_analysis.py")
SPEC = importlib.util.spec_from_file_location("stage2b", MODULE_PATH)
stage2b = importlib.util.module_from_spec(SPEC)
assert SPEC.loader
import sys
sys.modules[SPEC.name] = stage2b
SPEC.loader.exec_module(stage2b)


class Stage2bOfflineAnalysisTest(unittest.TestCase):
    def test_pose_exp_log_roundtrip_and_known_disagreements(self):
        for xi in ([0, 0, 0, 1, 2, 3], [0.1, -0.2, 0.3, 1, -2, 0.5]):
            recovered = stage2b.pose_log(stage2b.pose_exp(xi))
            for actual, expected in zip(recovered, xi):
                self.assertAlmostEqual(actual, expected, places=10)

    def test_interpolation_uses_linear_position_and_quaternion_slerp(self):
        poses = [
            stage2b.TimedPose(0.0, (0, 0, 0), (0, 0, 0, 1)),
            stage2b.TimedPose(2.0, (2, 0, 0), (0, 0, 1, 0)),
        ]
        middle = stage2b.interpolate_pose(poses, 1.0)
        self.assertIsNotNone(middle)
        self.assertAlmostEqual(middle[0][3], 1.0)
        self.assertAlmostEqual(middle[0][0], 0.0, places=10)
        self.assertAlmostEqual(middle[1][0], 1.0, places=10)

    def test_ground_truth_errors_are_exact_for_matching_edges(self):
        poses = [
            stage2b.TimedPose(0.0, (0, 0, 0), (0, 0, 0, 1)),
            stage2b.TimedPose(1.0, (1, 0, 0), (0, 0, 0, 1)),
        ]
        row = {"from_stamp_sec": 0.0, "to_stamp_sec": 1.0,
               "sender_mu": [0, 0, 0, 1, 0, 0],
               "receiver_mu": [0, 0, 0, 1, 0, 0]}
        self.assertEqual(stage2b.enrich_with_ground_truth([row], poses, stage2b.eye4()), 1)
        self.assertAlmostEqual(row["sender_trans_error_m"], 0.0, places=12)
        self.assertAlmostEqual(row["receiver_rot_error_rad"], 0.0, places=12)

    def test_ground_truth_skips_unmatched_or_nonfinite_shadow_rows(self):
        poses = [
            stage2b.TimedPose(0.0, (0, 0, 0), (0, 0, 0, 1)),
            stage2b.TimedPose(1.0, (1, 0, 0), (0, 0, 0, 1)),
        ]
        row = {"from_stamp_sec": 0.0, "to_stamp_sec": 1.0,
               "metadata_match": True, "receiver_match": False,
               "sender_mu": [0, 0, 0, 1, 0, 0],
               "receiver_mu": [math.nan] * 6}
        self.assertEqual(
            stage2b.enrich_with_ground_truth([row], poses, stage2b.eye4()), 0)
        self.assertFalse(row["gt_available"])

    def test_ground_truth_requires_explicit_verified_extrinsic(self):
        transform, reason = stage2b.load_verified_extrinsic(None)
        self.assertIsNone(transform)
        self.assertIn("no verified", reason)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "extrinsic.json"
            path.write_text(json.dumps({"status": "unverified",
                                        "gt_body_T_kimera_base_xyz_xyzw": [0,0,0,0,0,0,1]}))
            transform, reason = stage2b.load_verified_extrinsic(path)
            self.assertIsNone(transform)
            self.assertIn("not marked", reason)

    def test_spearman_and_fixed_seed_bootstrap(self):
        x = [1, 2, 3, 4, 5]
        y = [10, 20, 30, 40, 50]
        self.assertAlmostEqual(stage2b.spearman(x, y), 1.0)
        first = stage2b.bootstrap_spearman(x, y, 100, 7)
        second = stage2b.bootstrap_spearman(x, y, 100, 7)
        self.assertEqual(first, second)

    def test_primary_correlations_use_only_active_accepted_factors(self):
        fields = [
            "from_index", "to_index", "from_stamp_sec", "to_stamp_sec",
            "metadata_match", "receiver_match", "active_factor_accepted",
            "health_state", "reference_ready", "relative_health_valid",
            "rot_health_min", "trans_health_min", "rot_degraded_fraction",
            "trans_degraded_fraction", "rot_absolute_fraction",
            "trans_absolute_fraction", "inlier_fraction_mean",
            "cost_reduction_mean", "rot_disagreement_rad",
            "trans_disagreement_m", "start_dt_sec", "end_dt_sec",
            *[f"sender_r{i}" for i in range(3)],
            *[f"sender_t{i}" for i in range(3)],
            *[f"receiver_r{i}" for i in range(3)],
            *[f"receiver_t{i}" for i in range(3)],
            "stable_edge_digest", "payload_digest",
        ]
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            source = directory / "shadow.csv"
            with source.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                for index in range(6):
                    active = index < 3
                    value = index + 1
                    row = {key: "0" for key in fields}
                    row.update({
                        "from_index": str(index), "to_index": str(index + 1),
                        "from_stamp_sec": str(index),
                        "to_stamp_sec": str(index + 1),
                        "metadata_match": "1", "receiver_match": "1",
                        "active_factor_accepted": "1" if active else "0",
                        "health_state": "1", "reference_ready": "1",
                        "relative_health_valid": "1",
                        "rot_health_min": str(value),
                        "trans_health_min": str(value),
                        "rot_disagreement_rad": str(value if active else 10 - value),
                        "trans_disagreement_m": str(value if active else 10 - value),
                    })
                    writer.writerow(row)
            output = directory / "analysis"
            args = argparse.Namespace(
                shadow_csv=source, output_dir=output, dataset="synthetic",
                ground_truth=None, ground_truth_kind="full_pose",
                gt_body_extrinsic=None, bootstrap_samples=0,
                bootstrap_seed=7, rerun_rrd=None)
            summary = stage2b.analyze(args)
            self.assertEqual(summary["active_accepted_disagreement_available"], 3)
            self.assertEqual(
                summary["primary_active_accepted_correlations"]
                       ["trans_health_vs_trans_disagreement"]["n"], 3)
            self.assertEqual(
                summary["supplementary_all_matched_correlations"]
                       ["trans_health_vs_trans_disagreement"]["n"], 6)


if __name__ == "__main__":
    unittest.main()
