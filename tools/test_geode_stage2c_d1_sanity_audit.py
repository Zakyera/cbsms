#!/usr/bin/env python3
"""Synthetic tests for the passive Stage 2C-D1 evaluator sanity audit."""

import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

import geode_stage2c_b_evaluator as base
import geode_stage2c_d1_sanity_audit as audit


class GeodeStage2CD1SanityAuditTest(unittest.TestCase):
    def test_pose_direction_inversion_is_detected(self):
        expected = base.se3_exp([0.0, 0.0, 0.0, 1.0, 0.0, 0.0])
        correct = audit.residual_metrics(expected, expected)
        inverted = audit.residual_metrics(expected, base.pose_inverse(expected))
        self.assertLess(correct["translation_error_m"], 1.0e-12)
        self.assertAlmostEqual(inverted["translation_error_m"], 2.0, places=12)

    def test_endpoint_order_inversion_is_detected(self):
        first = base.pose(translation=[0.0, 0.0, 0.0])
        second = base.pose(translation=[2.0, -1.0, 0.5])
        forward = base.pose_between(first, second)
        reverse = base.pose_between(second, first)
        self.assertLess(audit.residual_metrics(forward, forward)[
            "translation_error_m"], 1.0e-12)
        self.assertGreater(audit.residual_metrics(forward, reverse)[
            "translation_error_m"], 4.0)

    def test_translation_vector_cosine(self):
        self.assertAlmostEqual(audit.vector_cosine([1, 0, 0], [2, 0, 0]), 1.0)
        self.assertAlmostEqual(audit.vector_cosine([1, 0, 0], [-2, 0, 0]), -1.0)
        self.assertAlmostEqual(audit.vector_cosine([1, 0, 0], [0, 2, 0]), 0.0)
        self.assertTrue(math.isnan(audit.vector_cosine([0, 0, 0], [1, 0, 0])))

    def test_official_formula_equals_local_conversion(self):
        world_imu = base.se3_exp([0.2, -0.1, 0.05, 3.0, -2.0, 1.0])
        e_transform = base.se3_exp([-0.03, 0.02, 0.01, 0.05, -0.01, 0.03])
        s_transform = base.se3_exp([0.01, -0.02, 0.03, 0.03, -0.6, 0.09])
        local = base.imu_pose_to_beta(world_imu, e_transform, s_transform)
        official = world_imu @ e_transform @ np.linalg.inv(s_transform)
        error = base.se3_log(base.pose_inverse(local) @ official)
        self.assertLess(np.linalg.norm(error), 1.0e-12)

    def test_official_script_and_local_tum_comparison(self):
        with tempfile.TemporaryDirectory() as directory:
            first = Path(directory) / "local.txt"
            second = Path(directory) / "official.txt"
            first.write_text(
                "1.123456789 1.0000001 2 3 0 0 0 1\n", encoding="utf-8")
            second.write_text(
                "1.123457 1.000000 2.000000 3.000000 0 0 0 1\n",
                encoding="utf-8")
            result = audit.compare_tum_trajectories(first, second)
            self.assertTrue(result["valid"])
            self.assertLess(result["maximum_timestamp_difference_sec"], 5.1e-7)
            self.assertLess(result["translation_difference_m"]["maximum"], 2e-7)

    def test_constant_speed_time_offset_example(self):
        self.assertAlmostEqual(
            audit.constant_speed_time_offset_displacement(4.3, 0.2), 0.86)
        self.assertAlmostEqual(
            audit.constant_speed_time_offset_displacement(4.3, -0.2), 0.86)

    def test_diagnostic_alternative_never_replaces_primary(self):
        rows = [{
            "A_primary_rotation_error_rad": 0.1,
            "A_primary_translation_error_m": 1.0,
            "A_primary_raw_group_translation_residual_m": 1.0,
            "B_invert_glim_rotation_error_rad": 0.0,
            "B_invert_glim_translation_error_m": 0.01,
            "B_invert_glim_raw_group_translation_residual_m": 0.01,
            "C_invert_gt_rotation_error_rad": 0.0,
            "C_invert_gt_translation_error_m": 2.0,
            "C_invert_gt_raw_group_translation_residual_m": 2.0,
            "D_invert_both_rotation_error_rad": 0.1,
            "D_invert_both_translation_error_m": 1.0,
            "D_invert_both_raw_group_translation_residual_m": 1.0,
        }]
        result = audit.alternative_summaries(rows, 10.0)
        self.assertIn("B_invert_glim", result["flagged_alternatives"])
        self.assertFalse(result["primary_was_replaced"])

    def test_tunnel4_cannot_bypass_frozen_reference_gates(self):
        plan = {"tunnel4_reference_screen": {
            "minimum_accepted_samples": 30,
            "minimum_accepted_span_sec": 2.9,
            "maximum_accepted_gap_sec": 0.5,
            "minimum_acceptance_fraction": 0.5,
        }}
        screening = {
            "reference_naturally_ready": True,
            "accepted_candidate_count": 29,
            "accepted_span_sec": 10.0,
            "maximum_accepted_gap_sec": 0.1,
            "acceptance_fraction": 0.9,
        }
        self.assertFalse(audit.tunnel_reference_contract(screening, plan))
        screening["accepted_candidate_count"] = 30
        self.assertTrue(audit.tunnel_reference_contract(screening, plan))
        screening["maximum_accepted_gap_sec"] = 0.5000001
        self.assertFalse(audit.tunnel_reference_contract(screening, plan))

    def test_ground_truth_has_no_reference_output_interface(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "plan.json"
            path.write_text(json.dumps({"primary": {}}), encoding="utf-8")
            parser_arguments = (
                "--dataset x --plan p --profile p --ground-truth gt "
                "--glim-csv g --edge-csv e --output-dir o").split()
            parsed = audit.parse_args(parser_arguments)
            self.assertFalse(hasattr(parsed, "health_csv"))
            self.assertFalse(hasattr(parsed, "reference_profile"))


if __name__ == "__main__":
    unittest.main()
