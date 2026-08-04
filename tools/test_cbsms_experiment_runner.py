#!/usr/bin/env python3

import csv
import math
import tempfile
import unittest
from pathlib import Path

from cbsms_experiment import (
    dcreg_health_summary,
    expected_replay_wall_duration_sec,
    load_dcreg_health_csv,
)


class ExperimentRunnerTimingTest(unittest.TestCase):
    def test_slow_bag_rate_expands_wall_duration(self) -> None:
        self.assertAlmostEqual(
            expected_replay_wall_duration_sec(60.0, {"bag_rate": "0.25"}),
            240.0,
        )

    def test_fast_or_invalid_rate_never_shortens_sensor_duration(self) -> None:
        self.assertAlmostEqual(
            expected_replay_wall_duration_sec(60.0, {"bag_rate": "2.0"}),
            60.0,
        )
        self.assertAlmostEqual(
            expected_replay_wall_duration_sec(60.0, {"bag_rate": "invalid"}),
            60.0,
        )


class DcregHealthReportTest(unittest.TestCase):
    def test_dedicated_csv_is_loaded_and_summarized(self) -> None:
        fieldnames = [
            "schema_version",
            "sensor_identifier",
            "record_kind",
            "frame_id",
            "factor_id",
            "resolution",
            "valid",
            "reference_ready",
            "health_available",
            "reference_source",
            "offline_profile_status",
            "baseline_update_accepted",
            "baseline_update_reason",
            "rotation_health_smoothed_0",
            "rotation_health_smoothed_1",
            "rotation_health_smoothed_2",
            "translation_health_smoothed_0",
            "translation_health_smoothed_1",
            "translation_health_smoothed_2",
            "evaluation_time_ms",
            "inlier_count",
            "inlier_fraction",
            "hessian_trace",
        ]
        rows = [
            {
                "schema_version": 2,
                "sensor_identifier": "livox_mid360",
                "record_kind": "aggregate",
                "frame_id": 7,
                "factor_id": -1,
                "resolution": math.nan,
                "valid": 1,
                "reference_ready": 1,
                "health_available": 1,
                "reference_source": "hybrid_offline",
                "offline_profile_status": "offline_profile_loaded",
                "baseline_update_accepted": 0,
                "baseline_update_reason": "frozen_during_degradation",
                "rotation_health_smoothed_0": 1.0,
                "rotation_health_smoothed_1": 0.8,
                "rotation_health_smoothed_2": 0.6,
                "translation_health_smoothed_0": 0.4,
                "translation_health_smoothed_1": 0.2,
                "translation_health_smoothed_2": 0.1,
                "evaluation_time_ms": 0.25,
                "inlier_count": math.nan,
                "inlier_fraction": math.nan,
                "hessian_trace": 42.0,
            },
            {
                "schema_version": 2,
                "sensor_identifier": "livox_mid360",
                "record_kind": "factor",
                "frame_id": 7,
                "factor_id": 0,
                "resolution": 0.25,
                "valid": 1,
                "reference_ready": 1,
                "health_available": 1,
                "reference_source": "",
                "offline_profile_status": "",
                "baseline_update_accepted": 0,
                "baseline_update_reason": "",
                "rotation_health_smoothed_0": math.nan,
                "rotation_health_smoothed_1": math.nan,
                "rotation_health_smoothed_2": math.nan,
                "translation_health_smoothed_0": math.nan,
                "translation_health_smoothed_1": math.nan,
                "translation_health_smoothed_2": math.nan,
                "evaluation_time_ms": math.nan,
                "inlier_count": 120,
                "inlier_fraction": 0.75,
                "hessian_trace": 20.0,
            },
        ]

        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "glim_dcreg_health.csv"
            with path.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(rows)

            loaded = load_dcreg_health_csv(path)

        summary = dcreg_health_summary(loaded)
        self.assertEqual(summary["schema_versions"], [2])
        self.assertEqual(loaded[0]["sensor_identifier"], "livox_mid360")
        self.assertEqual(loaded[0]["reference_source"], "hybrid_offline")
        self.assertEqual(summary["aggregate_count"], 1)
        self.assertEqual(summary["factor_count"], 1)
        self.assertEqual(summary["reference_ready_count"], 1)
        self.assertAlmostEqual(summary["translation_health_min"], 0.1)
        self.assertAlmostEqual(summary["evaluation_time_ms_p50"], 0.25)
        self.assertEqual(summary["resolution_support"][0]["resolution"], 0.25)
        self.assertEqual(summary["resolution_support"][0]["inlier_count_p50"], 120)

    def test_missing_dedicated_csv_is_not_inferred_from_console(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            missing = Path(temp_dir) / "glim_dcreg_health.csv"
            self.assertEqual(load_dcreg_health_csv(missing), [])
        self.assertEqual(dcreg_health_summary([]), {})


if __name__ == "__main__":
    unittest.main()
