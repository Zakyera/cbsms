#!/usr/bin/env python3

import csv
import importlib.util
import json
import math
import sys
import tempfile
import unittest
from decimal import Decimal
from pathlib import Path

import numpy as np


TOOLS = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location(
    "geode_stage2c_b_evaluator", TOOLS / "geode_stage2c_b_evaluator.py")
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def transform(t=(0, 0, 0), w=(0, 0, 0)):
    return MODULE.pose(MODULE.so3_exp(np.asarray(w, float)), np.asarray(t, float))


class FrameChainTest(unittest.TestCase):
    def assertPoseNear(self, first, second, tolerance=1.0e-12):
        error = MODULE.se3_log(MODULE.pose_inverse(first) @ second)
        self.assertLessEqual(float(np.linalg.norm(error)), tolerance)

    def check_paths(self, first, second, e, s):
        path_a = MODULE.pose_between(
            MODULE.imu_pose_to_beta(first, e, s),
            MODULE.imu_pose_to_beta(second, e, s))
        path_b = MODULE.imu_edge_to_beta(MODULE.pose_between(first, second), e, s)
        self.assertPoseNear(path_a, path_b)
        return path_a

    def test_identity_e_and_s(self):
        self.check_paths(np.eye(4), transform((1, 2, 3), (.1, .2, -.1)),
                         np.eye(4), np.eye(4))

    def test_nontrivial_rotations_and_lever_arms(self):
        self.check_paths(transform((.2, -.1, .4), (.1, -.2, .3)),
                         transform((1.2, .5, -.3), (-.2, .4, .1)),
                         transform((.05, -.02, .03), (.02, .01, -.03)),
                         transform((.03, -.596, .09), (-.01, .004, .028)))

    def test_constant_left_world_alignment_cancels(self):
        first = transform((.2, .3, -.1), (.1, .2, .3))
        second = transform((.7, -.4, .8), (-.2, .1, .05))
        alignment = transform((10, -5, 3), (.4, -.2, .1))
        self.assertPoseNear(MODULE.pose_between(first, second),
                            MODULE.pose_between(alignment @ first, alignment @ second))

    def test_omitting_e_changes_edge(self):
        first, second = np.eye(4), transform((1, 0, 0), (0, 0, math.pi / 2))
        e = transform((.5, 0, 0), (0, 0, .1))
        correct = MODULE.imu_edge_to_beta(MODULE.pose_between(first, second), e, np.eye(4))
        wrong = MODULE.pose_between(first, second)
        self.assertGreater(np.linalg.norm(MODULE.se3_log(MODULE.pose_inverse(correct) @ wrong)), .1)

    def test_omitting_s_changes_edge(self):
        edge = transform((1, 0, 0), (0, 0, math.pi / 2))
        s = transform((0, -.6, .09), (.01, -.02, .03))
        correct = MODULE.imu_edge_to_beta(edge, np.eye(4), s)
        self.assertGreater(np.linalg.norm(MODULE.se3_log(MODULE.pose_inverse(correct) @ edge)), .1)

    def test_quaternion_sign_is_identical(self):
        quaternion = MODULE.matrix_to_quaternion_xyzw(MODULE.so3_exp([.2, -.1, .3]))
        np.testing.assert_allclose(MODULE.quaternion_to_matrix_xyzw(quaternion),
                                   MODULE.quaternion_to_matrix_xyzw(-quaternion), atol=1e-15)

    def test_se3_exp_log_roundtrip(self):
        tangent = np.array([.1, -.2, .3, .4, -.5, .6])
        np.testing.assert_allclose(MODULE.se3_log(MODULE.se3_exp(tangent)),
                                   tangent, atol=2e-15)


class GroundTruthTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)

    def tearDown(self):
        self.temporary.cleanup()

    def write(self, lines):
        path = self.root / "gt.txt"
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        return path

    def test_stable_sort_and_original_indices(self):
        records, stats = MODULE.load_ground_truth(self.write([
            "1.10 1 0 0 0 0 0 1", "1.00 0 0 0 0 0 0 1",
            "1.05 0.5 0 0 0 0 0 1"]))
        self.assertEqual([str(row.stamp) for row in records], ["1.00", "1.05", "1.10"])
        self.assertEqual([row.original_index for row in records], [1, 2, 0])
        self.assertEqual(stats["source_backward_step_count"], 1)

    def test_exact_timestamp(self):
        records, _ = MODULE.load_ground_truth(self.write([
            "1.00 0 0 0 0 0 0 1", "1.05 1 0 0 0 0 0 1"]))
        result, info = MODULE.interpolate_ground_truth(records, Decimal("1.00"))
        self.assertEqual(info["status"], "exact")
        np.testing.assert_allclose(result[:3, 3], [0, 0, 0])

    def test_valid_slerp_and_sign_flip(self):
        half = math.sqrt(.5)
        records, _ = MODULE.load_ground_truth(self.write([
            "1.00 0 0 0 0 0 0 1",
            f"1.04 1 0 0 0 0 {-half} {-half}"]))
        result, info = MODULE.interpolate_ground_truth(records, Decimal("1.02"))
        self.assertTrue(info["valid"])
        angle = np.linalg.norm(MODULE.so3_log(result[:3, :3]))
        self.assertAlmostEqual(angle, math.pi / 4, places=12)

    def test_harmless_duplicate_quaternion_sign(self):
        records, stats = MODULE.load_ground_truth(self.write([
            "1.00 0 0 0 0 0 0 1", "1.00 0 0 0 0 0 0 -1"]))
        self.assertEqual(len(records), 1)
        self.assertEqual(stats["harmless_duplicate_count"], 1)

    def test_conflicting_duplicate_fails(self):
        with self.assertRaises(ValueError):
            MODULE.load_ground_truth(self.write([
                "1.00 0 0 0 0 0 0 1", "1.00 1 0 0 0 0 0 1"]))

    def test_no_extrapolation(self):
        records, _ = MODULE.load_ground_truth(self.write([
            "1.00 0 0 0 0 0 0 1", "1.04 1 0 0 0 0 0 1"]))
        result, info = MODULE.interpolate_ground_truth(records, Decimal("0.99"))
        self.assertIsNone(result)
        self.assertEqual(info["reason"], "outside_coverage")

    def test_exact_point_zero_five_boundary_passes(self):
        records, _ = MODULE.load_ground_truth(self.write([
            "1.00 0 0 0 0 0 0 1", "1.05 1 0 0 0 0 0 1"]))
        result, info = MODULE.interpolate_ground_truth(records, Decimal("1.02"))
        self.assertIsNotNone(result)
        self.assertEqual(info["total_bracket_sec"], 0.05)

    def test_bracket_above_point_zero_five_fails(self):
        records, _ = MODULE.load_ground_truth(self.write([
            "1.00 0 0 0 0 0 0 1", "1.050000001 1 0 0 0 0 0 1"]))
        result, info = MODULE.interpolate_ground_truth(records, Decimal("1.02"))
        self.assertIsNone(result)
        self.assertEqual(info["reason"], "bracket_exceeds_limit")

    def test_nonfinite_pose_rejected(self):
        records, stats = MODULE.load_ground_truth(self.write([
            "1.00 nan 0 0 0 0 0 1", "1.01 0 0 0 0 0 0 1"]))
        self.assertEqual(len(records), 1)
        self.assertEqual(stats["nonfinite_rejected_count"], 1)


class EvaluatorSemanticsTest(unittest.TestCase):
    def test_zero_pure_rotation_pure_translation_and_mixed(self):
        zero = MODULE.se3_log(np.eye(4))
        self.assertLess(np.linalg.norm(zero), 1e-15)
        rotation = MODULE.se3_log(MODULE.se3_exp([0, 0, .1, 0, 0, 0]))
        translation = MODULE.se3_log(MODULE.se3_exp([0, 0, 0, .2, 0, 0]))
        mixed = np.array([.1, -.2, .3, .4, -.5, .6])
        np.testing.assert_allclose(rotation, [0, 0, .1, 0, 0, 0], atol=1e-15)
        np.testing.assert_allclose(translation, [0, 0, 0, .2, 0, 0], atol=1e-15)
        np.testing.assert_allclose(MODULE.se3_log(MODULE.se3_exp(mixed)), mixed, atol=2e-15)

    def test_repeated_beliefs_deduplicate_to_last_publication(self):
        base = {"belief_index": "0", "from_pose_index": "1", "to_pose_index": "3",
                "from_stamp_sec": "1.0", "to_stamp_sec": "1.2"}
        rows = [dict(base, message_index=str(index)) for index in range(10)]
        selected, stats = MODULE.select_last_stable_edges(rows)
        self.assertEqual(len(selected), 1)
        self.assertEqual(selected[0]["message_index"], "9")
        self.assertEqual(stats["republication_row_count"], 9)

    def test_invalid_endpoint_excludes_whole_edge(self):
        with tempfile.TemporaryDirectory() as directory:
            gt = Path(directory) / "gt.txt"
            gt.write_text("1.0 0 0 0 0 0 0 1\n1.04 0 0 0 0 0 0 1\n")
            records, _ = MODULE.load_ground_truth(gt)
            self.assertIsNone(MODULE.interpolate_ground_truth(records, Decimal("1.1"))[0])

    def test_rotation_and_translation_are_separate(self):
        residual = MODULE.se3_log(MODULE.se3_exp([.1, 0, 0, 1.0, 0, 0]))
        self.assertAlmostEqual(np.linalg.norm(residual[:3]), .1)
        self.assertAlmostEqual(np.linalg.norm(residual[3:]), 1.0)

    def test_ground_truth_module_has_no_reference_update_api(self):
        self.assertFalse(hasattr(MODULE, "update_reference"))
        self.assertFalse(hasattr(MODULE, "calibrate_health"))


if __name__ == "__main__":
    unittest.main()
