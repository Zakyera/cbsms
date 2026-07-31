#!/usr/bin/env python3

import math
import unittest

from glim_dcreg_directional_audit import (
    build_directional_audit,
    directional_relative_residual,
    interpolate_pose,
)


class DirectionalAuditMathTest(unittest.TestCase):
    def test_relative_residual_identity_and_translation(self) -> None:
        identity = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        translated = ((1.0, -2.0, 3.0), (0.0, 0.0, 0.0, 1.0))
        self.assertEqual(
            directional_relative_residual(identity, identity),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )
        residual = directional_relative_residual(identity, translated)
        self.assertEqual(residual[:3], (0.0, 0.0, 0.0))
        self.assertEqual(residual[3:], (1.0, -2.0, 3.0))

    def test_pose_interpolation_uses_quaternion_slerp(self) -> None:
        poses = [
            {
                "t": 0.0,
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.0,
                "qw": 1.0,
            },
            {
                "t": 1.0,
                "x": 2.0,
                "y": 0.0,
                "z": 0.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 1.0,
                "qw": 0.0,
            },
        ]
        pose, gap = interpolate_pose(poses, 0.5, max_bracket_sec=2.0)
        self.assertIsNotNone(pose)
        self.assertAlmostEqual(gap, 1.0)
        assert pose is not None
        self.assertAlmostEqual(pose[0][0], 1.0)
        self.assertAlmostEqual(abs(pose[1][2]), math.sqrt(0.5), places=12)
        self.assertAlmostEqual(abs(pose[1][3]), math.sqrt(0.5), places=12)

    def test_synthetic_projection_recovers_expected_nis(self) -> None:
        dcreg = {
            "frame_id": 1,
            "valid": 1,
            **{
                f"{space}_spectral_ratio_{mode}": (20.0 if mode == 0 else 1.0)
                for space in ("rotation", "translation")
                for mode in range(3)
            },
            **{
                f"{space}_eigenvalue_{mode}": float(mode + 1)
                for space in ("rotation", "translation")
                for mode in range(3)
            },
        }
        basis = {"frame_id": 1}
        for space in ("rotation", "translation"):
            for mode in range(3):
                for axis_index, axis in enumerate(("x", "y", "z")):
                    basis[f"{space}_mode_{mode}_{axis}"] = float(
                        mode == axis_index
                    )
        shadow = {
            "publish_stamp": 2.0,
            "from_index": 0,
            "to_index": 1,
            "from_stamp": 0.0,
            "to_stamp": 1.0,
            "measured_tx": 0.0,
            "measured_ty": 0.0,
            "measured_tz": 0.0,
            "measured_qx": 0.0,
            "measured_qy": 0.0,
            "measured_qz": 0.0,
            "measured_qw": 1.0,
            "sender_health_alpha": math.nan,
            "covariance_mode": "test",
        }
        for row in range(6):
            for col in range(6):
                shadow[f"covariance_{row}{col}"] = 0.25 if row == col else 0.0
        gt = [
            {
                "t": 0.0,
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.0,
                "qw": 1.0,
            },
            {
                "t": 1.0,
                "x": 1.0,
                "y": 0.0,
                "z": 0.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.0,
                "qw": 1.0,
            },
        ]
        audit, _, _, _, metadata = build_directional_audit(
            [dcreg],
            [basis],
            [shadow],
            gt,
            max_gt_bracket_sec=2.0,
        )
        self.assertEqual(metadata["matched_edges"], 1)
        self.assertEqual(len(audit), 6)
        tx = next(
            row
            for row in audit
            if row["space"] == "translation" and row["mode"] == 0
        )
        self.assertAlmostEqual(tx["projected_error"], 1.0)
        self.assertAlmostEqual(tx["projected_sigma"], 0.5)
        self.assertAlmostEqual(tx["directional_nis"], 4.0)

    def test_position_only_truth_excludes_rotation_and_adds_reference_noise(
        self,
    ) -> None:
        dcreg = {
            "frame_id": 1,
            "valid": 1,
            **{
                f"{space}_spectral_ratio_{mode}": 1.0
                for space in ("rotation", "translation")
                for mode in range(3)
            },
            **{
                f"{space}_eigenvalue_{mode}": 1.0
                for space in ("rotation", "translation")
                for mode in range(3)
            },
        }
        basis = {"frame_id": 1}
        for space in ("rotation", "translation"):
            for mode in range(3):
                for axis_index, axis in enumerate(("x", "y", "z")):
                    basis[f"{space}_mode_{mode}_{axis}"] = float(
                        mode == axis_index
                    )
        shadow = {
            "publish_stamp": 2.0,
            "from_index": 0,
            "to_index": 1,
            "from_stamp": 0.0,
            "to_stamp": 1.0,
            "measured_tx": 0.0,
            "measured_ty": 0.0,
            "measured_tz": 0.0,
            "measured_qx": 0.0,
            "measured_qy": 0.0,
            "measured_qz": 0.0,
            "measured_qw": 1.0,
            "sender_health_alpha": math.nan,
            "covariance_mode": "test",
        }
        for row in range(6):
            for col in range(6):
                shadow[f"covariance_{row}{col}"] = 0.25 if row == col else 0.0
        poses = [
            {
                "t": stamp,
                "x": stamp,
                "y": 0.0,
                "z": 0.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.0,
                "qw": 1.0,
            }
            for stamp in (0.0, 1.0)
        ]
        audit, _, _, _, metadata = build_directional_audit(
            [dcreg],
            [basis],
            [shadow],
            poses,
            estimator_poses=poses,
            ground_truth_from_estimator_world_yaw=0.0,
            ground_truth_horizontal_sigma_m=0.1,
            ground_truth_vertical_sigma_m=0.2,
            max_gt_bracket_sec=2.0,
        )
        self.assertEqual(metadata["audit_spaces"], "translation_only")
        self.assertEqual(len(audit), 3)
        tx = next(row for row in audit if row["mode"] == 0)
        self.assertAlmostEqual(tx["projected_error"], 1.0)
        self.assertAlmostEqual(tx["reference_projected_variance"], 0.02)
        self.assertAlmostEqual(tx["reference_aware_variance"], 0.27)


if __name__ == "__main__":
    unittest.main()
