#!/usr/bin/env python3

import csv
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ANALYSIS = load("d2_c_analysis", ROOT / "geode_stage2c_c_analysis.py")
AUDIT = load("d2_reference_audit", ROOT / "geode_stage2c_d2_reference_audit.py")
PLAN = json.loads((ROOT.parent / "config" /
                   "geode_stage2c_c_analysis_plan.json").read_text())


FIELDS = [
    "schema_version", "configuration_fingerprint", "timestamp", "frame_id",
    "record_kind", "resolution", "valid", "factorization_ok",
    "registration_converged", "linear_solve_success", "reference_ready",
    "health_available", "source_point_count", "inlier_count", "inlier_fraction",
    "minimum_factor_inlier_count", "minimum_factor_inlier_fraction",
    "initial_cost", "final_cost", "rotation_condition_ratio_0",
    "rotation_condition_ratio_1", "rotation_condition_ratio_2",
    "translation_condition_ratio_0", "translation_condition_ratio_1",
    "translation_condition_ratio_2", "absolute_rotation_mask_0",
    "absolute_rotation_mask_1", "absolute_rotation_mask_2",
    "absolute_translation_mask_0", "absolute_translation_mask_1",
    "absolute_translation_mask_2", "baseline_update_accepted",
    "baseline_update_reason",
]


def row(frame, accepted=True, ready=False, timestamp=None):
    result = {field: "" for field in FIELDS}
    result.update({
        "schema_version": "2", "configuration_fingerprint": "gamma",
        "timestamp": str(frame * 0.1 if timestamp is None else timestamp),
        "frame_id": str(frame), "record_kind": "aggregate", "resolution": "nan",
        "valid": "1", "factorization_ok": "1", "registration_converged": "1",
        "linear_solve_success": "1", "reference_ready": str(int(ready)),
        "health_available": str(int(ready)), "source_point_count": "500",
        "inlier_count": "250", "inlier_fraction": "0.5",
        "minimum_factor_inlier_count": "250",
        "minimum_factor_inlier_fraction": "0.5", "initial_cost": "100",
        "final_cost": "50", "baseline_update_accepted": str(int(accepted)),
        "baseline_update_reason": ("bootstrap_sample_accepted" if accepted
                                   else "absolute_rotation_degeneracy"),
    })
    for index, value in enumerate((4.0, 3.0, 1.0)):
        result[f"rotation_condition_ratio_{index}"] = str(value)
        result[f"translation_condition_ratio_{index}"] = str(value)
        result[f"absolute_rotation_mask_{index}"] = "0"
        result[f"absolute_translation_mask_{index}"] = "0"
    return result


def write(path, count, gap=False, ready=True):
    rows = [row(index + 1, ready=(ready and index == count - 1))
            for index in range(count)]
    if gap and count >= 30:
        rows[-1]["timestamp"] = str(float(rows[-2]["timestamp"]) + 0.500001)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


class Stage2cD2ReferenceAuditTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def screen(self, count, gap=False, ready=True):
        path = self.root / f"{count}_{gap}.csv"
        write(path, count, gap, ready)
        return ANALYSIS.reference_screening(path, PLAN, "gamma")

    def test_exactly_29_accepted_samples_fail(self):
        self.assertFalse(self.screen(29)["qualified"])

    def test_exactly_30_continuous_samples_pass(self):
        self.assertTrue(self.screen(30)["qualified"])

    def test_thirty_samples_with_gap_above_half_second_fail(self):
        result = self.screen(30, gap=True)
        self.assertFalse(result["qualified"])
        self.assertFalse(result["qualification_checks"]["maximum_accepted_gap"])

    def test_online_readiness_is_distinct_from_offline_qualification(self):
        result = self.screen(29, ready=True)
        self.assertIsNotNone(result["first_reference_ready_frame"])
        self.assertFalse(result["qualified"])

    def test_samples_across_runs_cannot_be_combined(self):
        with self.assertRaisesRegex(ValueError, "may not be combined"):
            AUDIT.require_one_run([self.root / "a.csv", self.root / "b.csv"])

    def test_stairs_failure_cannot_authorize_profile_or_relative_health(self):
        path = self.root / "one.csv"
        write(path, 1, ready=False)
        screen = ANALYSIS.reference_screening(path, PLAN, "gamma")
        screen_path = self.root / "screen.json"
        screen_path.write_text(json.dumps(screen), encoding="utf-8")
        result = AUDIT.audit([path], screen_path)
        self.assertFalse(result["online_reference_ready"])
        self.assertFalse(result["offline_profile_qualified"])
        self.assertFalse(result["profile_export_authorized"])
        self.assertFalse(result["short_medium_hybrid_rerun_authorized"])
        self.assertFalse(result["ground_truth_used"])


if __name__ == "__main__":
    unittest.main()
