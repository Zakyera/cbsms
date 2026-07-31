#!/usr/bin/env python3

import json
import math
import tempfile
import unittest
from pathlib import Path

from glim_dcreg_calibration_protocol import (
    read_tum_pose_file,
    validate_manifest,
)


def write_poses(path: Path, quaternions: list[tuple[float, float, float, float]]) -> None:
    lines = []
    for index, quaternion in enumerate(quaternions):
        lines.append(
            f"{float(index):.6f} {float(index):.6f} 0 0 "
            f"{quaternion[0]} {quaternion[1]} {quaternion[2]} {quaternion[3]}"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


class CalibrationProtocolTest(unittest.TestCase):
    def test_pose_inventory_distinguishes_position_only_and_full_pose(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            fixed = root / "fixed.tum"
            turning = root / "turning.tum"
            write_poses(fixed, [(0.0, 0.0, 0.0, 1.0)] * 3)
            half_angle = 0.25 * math.pi
            write_poses(
                turning,
                [
                    (0.0, 0.0, 0.0, 1.0),
                    (0.0, 0.0, math.sin(half_angle), math.cos(half_angle)),
                ],
            )
            self.assertEqual(
                read_tum_pose_file(fixed)["observed_pose_content"],
                "position_only",
            )
            turning_report = read_tum_pose_file(turning)
            self.assertEqual(turning_report["observed_pose_content"], "full_pose")
            self.assertAlmostEqual(
                turning_report["orientation_span_deg"], 90.0, places=8
            )

    def test_fit_partition_requires_verified_extrinsic(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "sequence.bag").write_bytes(b"bag")
            write_poses(
                root / "truth.tum",
                [(0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.1, 0.995)],
            )
            manifest = {
                "schema_version": 1,
                "manifest_id": "test",
                "split_policy": {
                    "minimum_distinct_source_sequences": {
                        "calibration": 1,
                        "validation": 0,
                        "test": 0,
                    }
                },
                "sequences": [
                    {
                        "id": "sequence",
                        "split_group": "sequence",
                        "status": "calibration",
                        "host_bag": "sequence.bag",
                        "ground_truth": "truth.tum",
                        "ground_truth_kind": "full_pose",
                        "ground_truth_to_glim_pose_extrinsic": {
                            "status": "unverified"
                        },
                    }
                ],
            }
            report = validate_manifest(manifest, root)
            self.assertFalse(report["valid"])
            self.assertTrue(
                any("requires a verified" in message for message in report["errors"])
            )

    def test_overlapping_source_windows_cannot_cross_partitions(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            for name in ("a", "b"):
                (root / f"{name}.bag").write_bytes(b"bag")
                write_poses(
                    root / f"{name}.tum",
                    [(0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.1, 0.995)],
                )
            sequences = []
            for name, status in (("a", "calibration"), ("b", "test")):
                sequences.append(
                    {
                        "id": name,
                        "split_group": "same_recording",
                        "status": status,
                        "host_bag": f"{name}.bag",
                        "ground_truth": f"{name}.tum",
                        "ground_truth_kind": "full_pose",
                        "ground_truth_to_glim_pose_extrinsic": {
                            "status": "verified"
                        },
                    }
                )
            manifest = {
                "schema_version": 1,
                "manifest_id": "test",
                "split_policy": {
                    "minimum_distinct_source_sequences": {
                        "calibration": 1,
                        "validation": 0,
                        "test": 1,
                    }
                },
                "sequences": sequences,
            }
            report = validate_manifest(manifest, root)
            self.assertFalse(report["valid"])
            self.assertTrue(
                any(
                    "multiple fit partitions" in message
                    for message in report["errors"]
                )
            )


if __name__ == "__main__":
    unittest.main()
