#!/usr/bin/env python3
"""Create passive Stage 2C-C Rerun recordings from finalized artifacts."""

import argparse
import csv
import hashlib
import json
import math
import sys
from decimal import Decimal
from pathlib import Path

import numpy as np
import rerun as rr


sys.path.insert(0, str(Path(__file__).resolve().parent))
import geode_stage2c_b_evaluator as evaluator  # noqa: E402


def rows(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def flag(row, name):
    return str(row.get(name, "")).lower() in ("1", "true", "yes")


def max_modes(row, stem):
    values = [float(row[f"{stem}_{index}"]) for index in range(3)
              if finite(row.get(f"{stem}_{index}"))]
    return max(values) if values else None


def any_modes(row, stem):
    return any(flag(row, f"{stem}_{index}") for index in range(3))


def log_scalar(path, value):
    if finite(value):
        rr.log(path, rr.Scalars(float(value)))


def log_trajectory_and_cloud(profile, glim_csv, pointcloud_npz):
    e_transform, _s_transform = evaluator.official_transforms(profile)
    poses = evaluator.read_glim_poses(glim_csv)
    trajectory = np.asarray([item["transform"][:3, 3] for item in poses])
    if len(trajectory) > 1:
        rr.log("glim_world/trajectory_imu", rr.LineStrips3D([trajectory]), static=True)
    if pointcloud_npz and Path(pointcloud_npz).is_file():
        archive = np.load(pointcloud_npz)
        for index, stamp_ns in enumerate(archive["stamp_ns"]):
            stamp = Decimal(int(stamp_ns)) / Decimal(1_000_000_000)
            nearest = min(poses, key=lambda item: abs(item["stamp"] - stamp))
            start, end = archive["offsets"][index:index + 2]
            cloud = archive["points"][start:end]
            homogeneous = np.column_stack((cloud, np.ones(len(cloud))))
            world_lidar = nearest["transform"] @ e_transform
            world = (world_lidar @ homogeneous.T).T[:, :3]
            rr.set_time("sensor_time", timestamp=float(stamp))
            rr.log("glim_world/lidar/points", rr.Points3D(world, radii=0.025))
    return len(poses)


def reference_recording(args, profile):
    health = rows(args.health_csv)
    aggregate = [row for row in health if row.get("record_kind") == "aggregate"]
    factors = [row for row in health if row.get("record_kind") == "factor"]
    factor_by_stamp = {}
    for row in factors:
        factor_by_stamp.setdefault(row["timestamp"], []).append(row)
    for row in aggregate:
        rr.set_time("sensor_time", timestamp=float(row["timestamp"]))
        log_scalar("dcreg/rotation_max_condition_ratio",
                   max_modes(row, "rotation_condition_ratio"))
        log_scalar("dcreg/translation_max_condition_ratio",
                   max_modes(row, "translation_condition_ratio"))
        rr.log("dcreg/absolute_rotation_degenerate", rr.Scalars(float(
            any_modes(row, "absolute_rotation_mask"))))
        rr.log("dcreg/absolute_translation_degenerate", rr.Scalars(float(
            any_modes(row, "absolute_translation_mask"))))
        rr.log("reference/candidate_accepted", rr.Scalars(float(
            flag(row, "baseline_update_accepted"))))
        rr.log("reference/ready", rr.Scalars(float(flag(row, "reference_ready"))))
        rr.log("registration/converged", rr.Scalars(float(
            flag(row, "registration_converged"))))
        for factor in factor_by_stamp.get(row["timestamp"], []):
            resolution = factor.get("resolution", "unknown").replace(".", "_")
            root = f"vgicp_resolution_{resolution}"
            for entity, field in (("support/source_points", "source_point_count"),
                                  ("support/inliers", "inlier_count"),
                                  ("support/inlier_fraction", "inlier_fraction"),
                                  ("matching/initial_cost", "initial_cost"),
                                  ("matching/final_cost", "final_cost")):
                log_scalar(f"{root}/{entity}", factor.get(field))
    return {
        "aggregate_health_sample_count": len(aggregate),
        "accepted_reference_candidate_count": sum(
            flag(row, "baseline_update_accepted") for row in aggregate),
        "reference_ready_sample_count": sum(flag(row, "reference_ready")
                                              for row in aggregate),
    }


def inland_recording(args, profile):
    analysis = rows(args.analysis_csv)
    gt, gt_summary = evaluator.load_ground_truth(args.ground_truth)
    gt_points = np.asarray([item.transform[:3, 3] for item in gt])
    if len(gt_points) > 1:
        rr.log("gt_world/trajectory_beta", rr.LineStrips3D([gt_points]), static=True)
    for row in analysis:
        rr.set_time("sensor_time", timestamp=float(row["to_stamp_sec"]))
        log_scalar("edge_error/rotation_rad", row.get("rotation_error_rad"))
        log_scalar("edge_error/translation_m", row.get("translation_error_m"))
        log_scalar("conditioning/rotation_max_ratio",
                   row.get("stage2c_c_max_rotation_condition_ratio"))
        log_scalar("conditioning/translation_max_ratio",
                   row.get("stage2c_c_max_translation_condition_ratio"))
        log_scalar("conditioning/rotation_absolute_occupancy",
                   row.get("stage2c_c_rotation_absolute_degenerate_fraction"))
        log_scalar("conditioning/translation_absolute_occupancy",
                   row.get("stage2c_c_translation_absolute_degenerate_fraction"))
        log_scalar("gt/max_interpolation_bracket_sec", max(
            float(row["gt_start_bracket_sec"]), float(row["gt_end_bracket_sec"])))
        for entity, field in (("support/source_points", "aggregate_source_point_count_median"),
                              ("support/inlier_fraction", "aggregate_inlier_fraction_median"),
                              ("matching/initial_cost", "aggregate_initial_cost_median"),
                              ("matching/final_cost", "aggregate_final_cost_median")):
            log_scalar(entity, row.get(field))
    return {"gt_valid_unique_edge_count": len(analysis), "ground_truth": gt_summary}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kind", choices=("reference", "inland"), required=True)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--glim-csv", type=Path, required=True)
    parser.add_argument("--pointcloud-npz", type=Path)
    parser.add_argument("--health-csv", type=Path)
    parser.add_argument("--analysis-csv", type=Path)
    parser.add_argument("--ground-truth", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    args = parser.parse_args()
    if args.kind == "reference" and not args.health_csv:
        parser.error("reference recording requires --health-csv")
    if args.kind == "inland" and (not args.analysis_csv or not args.ground_truth):
        parser.error("inland recording requires --analysis-csv and --ground-truth")

    profile = json.loads(args.profile.read_text(encoding="utf-8"))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rr.init("cbsms_geode_stage2c_c", recording_id=args.dataset)
    rr.save(str(args.output))
    rr.log("documentation", rr.TextDocument(
        f"# {args.dataset}\nPassive Stage 2C-C visualization. Condition ratios "
        "and health are LiDAR observability diagnostics, not covariance, "
        "probability, confidence, or metric-error calibration."), static=True)
    pose_count = log_trajectory_and_cloud(profile, args.glim_csv,
                                          args.pointcloud_npz)
    details = (reference_recording(args, profile) if args.kind == "reference"
               else inland_recording(args, profile))
    rr.disconnect()
    summary = {
        "schema_version": 1,
        "dataset": args.dataset,
        "kind": args.kind,
        "glim_pose_count": pose_count,
        "rrd": str(args.output.resolve()),
        "rrd_sha256": hashlib.sha256(args.output.read_bytes()).hexdigest(),
        **details,
    }
    args.summary.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n",
                            encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
