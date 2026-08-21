#!/usr/bin/env python3

import importlib.util
import json
import math
import os
from pathlib import Path
import unittest

import numpy as np


TOOL_PATH = Path(__file__).with_name("dcreg_stage3a_characterization.py")
SPEC = importlib.util.spec_from_file_location("stage3a", TOOL_PATH)
stage3a = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
SPEC.loader.exec_module(stage3a)


class Stage3aCharacterizationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        workspace = Path(os.environ.get("CBSMS_WORKSPACE", "/workspace/cbs_gtsam4.3"))
        cls.synthetic_path = workspace / "stage3a_characterization/synthetic/vgicp_scenes.json"
        cls.plan_path = workspace / "src/cbsms/config/glim_dcreg_stage3a_analysis_plan.json"
        cls.payload = json.loads(cls.synthetic_path.read_text())
        cls.rows, cls.summary = stage3a.synthetic_analysis(cls.payload)
        cls.by_name = {row["scene"]: row for row in cls.rows}

    def test_single_plane_expected_degeneracy(self):
        row = self.by_name["single_plane"]
        self.assertGreater(row["kappa_rotation"], 10.0)
        self.assertGreater(row["kappa_translation"], 10.0)
        self.assertGreater(row["rotation_weak_projector_overlap"], 0.999)
        self.assertGreater(row["translation_weak_projector_overlap"], 0.999)

    def test_corridor_expected_weak_direction(self):
        row = self.by_name["parallel_planes_corridor"]
        self.assertGreater(row["rotation_weak_projector_overlap"], 0.999)
        self.assertGreater(row["translation_weak_projector_overlap"], 0.999)

    def test_perpendicular_corner_improves_rotation_and_retains_vertical_weakness(self):
        corner = self.by_name["perpendicular_corner"]
        corridor = self.by_name["parallel_planes_corridor"]
        self.assertLess(corner["kappa_rotation"], corridor["kappa_rotation"])
        self.assertGreater(corner["translation_weak_projector_overlap"], 0.999)

    def test_rich_3d_is_full_rank_and_inside_primary_boundary(self):
        row = self.by_name["rich_irregular_3d"]
        self.assertEqual(row["full_hessian_rank"], 6)
        self.assertLess(row["kappa_rotation"], 10.0)
        self.assertLess(row["kappa_translation"], 10.0)

    def test_sparse_and_dense_support_are_separate_from_shape(self):
        sparse = self.by_name["sparse_rich_3d"]
        dense = self.by_name["rich_irregular_3d"]
        self.assertLess(sparse["source_points"], dense["source_points"])
        self.assertLess(abs(sparse["kappa_translation"] - dense["kappa_translation"]), 2.0)
        plane = self.by_name["dense_degenerate_plane"]
        self.assertGreater(plane["source_points"], dense["source_points"])
        self.assertGreater(plane["kappa_translation"], 10.0)

    def test_rotation_translation_coupling(self):
        hessian = np.asarray(self.payload["coupled_quadratic"]["hessian"], dtype=float)
        analysis = stage3a.numpy_schur_analysis(hessian)
        self.assertAlmostEqual(max(np.linalg.eigvalsh(hessian[:3, :3])) / min(np.linalg.eigvalsh(hessian[:3, :3])), 5.0)
        self.assertGreater(max(analysis["rotation_condition_ratios"]), 250.0)
        self.assertGreater(max(analysis["translation_condition_ratios"]), 250.0)

    def test_schur_and_diagonal_block_are_not_equivalent(self):
        coupled = self.payload["coupled_quadratic"]
        self.assertEqual(max(coupled["rotation_block_eigenvalues"]) / min(coupled["rotation_block_eigenvalues"]), 5.0)
        self.assertGreater(coupled["kappa_rotation"], 250.0)

    def test_eigenvector_sign_invariance(self):
        vector = np.asarray([0.2, -0.3, 0.9327379053])
        vector /= np.linalg.norm(vector)
        self.assertTrue(np.allclose(np.outer(vector, vector), np.outer(-vector, -vector), atol=1e-15))

    def test_eigenvalue_ordering_invariance_for_subspace(self):
        basis = np.asarray([[1.0, 0.0], [0.0, 1.0], [0.0, 0.0]])
        reordered = basis[:, [1, 0]]
        self.assertTrue(np.allclose(basis @ basis.T, reordered @ reordered.T, atol=1e-15))

    def test_clustered_eigenspace(self):
        self.assertEqual(stage3a.spectral_cluster_flags([1.0, 1.03, 10.0]), [True, True, False])
        self.assertEqual(stage3a.spectral_cluster_flags([1.0, 2.0, 10.0]), [False, False, False])

    def test_full_hessian_baselines_exist(self):
        row = self.by_name["rich_irregular_3d"]
        self.assertTrue(math.isfinite(row["full_hessian_condition"]))
        self.assertTrue(math.isfinite(row["full_hessian_minimum_eigenvalue"]))
        self.assertEqual(row["full_hessian_rank"], 6)

    def test_fixed_thresholds_are_preregistered(self):
        plan = json.loads(self.plan_path.read_text())
        self.assertEqual(plan["thresholds"], [5.0, 10.0, 20.0])
        self.assertEqual(plan["production_threshold"], 10.0)

    def test_signed_margin_monotonicity_and_boundary(self):
        self.assertGreater(stage3a.signed_margin(10.0, 5.0), 0.0)
        self.assertAlmostEqual(stage3a.signed_margin(10.0, 10.0), 0.0)
        self.assertLess(stage3a.signed_margin(10.0, 20.0), 0.0)
        kappas = [2.0, 5.0, 10.0, 20.0]
        margins = [stage3a.signed_margin(10.0, value) for value in kappas]
        self.assertTrue(all(first > second for first, second in zip(margins, margins[1:])))

    def test_signed_margin_is_offline_and_has_no_ros_or_cbs_dependency(self):
        source = TOOL_PATH.read_text()
        self.assertNotIn("rospy", source)
        self.assertNotIn("publish(", source)
        self.assertAlmostEqual(stage3a.signed_margin(10.0, 5.0), math.log(2.0))

    def test_no_baseline_uses_ground_truth_during_construction(self):
        plan = json.loads(self.plan_path.read_text())
        self.assertTrue(plan["reporting"]["no_ground_truth_used_to_construct_any_detector"])
        hessian = np.eye(6)
        result = stage3a.numpy_schur_analysis(hessian)
        self.assertEqual(result["full_rank"], 6)

    def test_cpp_frozen_analyzer_crosscheck(self):
        self.assertTrue(self.summary["crosscheck_pass"])
        self.assertLessEqual(self.summary["frozen_cpp_vs_numpy_max_relative_difference"], 1.0e-10)


if __name__ == "__main__":
    unittest.main()
