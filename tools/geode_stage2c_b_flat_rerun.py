#!/usr/bin/env python3
"""Create the passive Flat Smooth Gamma Rerun without a GT accuracy claim."""

import argparse
import csv
import hashlib
import json
import sys
from decimal import Decimal
from pathlib import Path

import numpy as np
import rerun as rr


sys.path.insert(0, str(Path(__file__).resolve().parent))
import geode_stage2c_b_evaluator as evaluator  # noqa: E402


def true_field(row, name):
    return row.get(name, "0") in ("1", "true", "True")


def any_mask(row, prefix):
    return any(true_field(row, f"{prefix}_{index}") for index in range(3))


def maximum_finite(row, prefix):
    values = [float(row[f"{prefix}_{index}"]) for index in range(3)
              if evaluator.finite(row.get(f"{prefix}_{index}"))]
    return max(values) if values else None


def minimum_finite(row, prefix):
    values = [float(row[f"{prefix}_{index}"]) for index in range(3)
              if evaluator.finite(row.get(f"{prefix}_{index}"))]
    return min(values) if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--glim-csv", type=Path, required=True)
    parser.add_argument("--belief-csv", type=Path, required=True)
    parser.add_argument("--health-csv", type=Path, required=True)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--pointcloud-npz", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    args = parser.parse_args()

    profile = json.loads(args.profile.read_text(encoding="utf-8"))
    e_transform, _s_transform = evaluator.official_transforms(profile)
    poses = evaluator.read_glim_poses(args.glim_csv)
    gt, gt_summary = evaluator.load_ground_truth(args.ground_truth)
    with args.health_csv.open(newline="", encoding="utf-8") as stream:
        health = [row for row in csv.DictReader(stream)
                  if row.get("record_kind") == "aggregate"]
    beliefs = evaluator.read_csv(args.belief_csv)
    stable, dedup = evaluator.select_last_stable_edges(beliefs)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    rr.init("cbsms_geode_stage2c_b_flat", recording_id="flat_smooth_gamma")
    rr.save(str(args.output))
    rr.log("documentation", rr.TextDocument(
        "# GEODE Flat Surfaces Smooth Gamma\n"
        "Passive GLIM/DCReg visualization. The Vicon target-to-Gamma-LiDAR "
        "extrinsic is unverified, so Vicon and GLIM trajectories are shown in "
        "separately named frames and no 6-DoF edge error is computed. Health is "
        "not covariance, probability, confidence, or metric error."), static=True)

    glim_points = np.asarray([item["transform"][:3, 3] for item in poses])
    vicon_points = np.asarray([item.transform[:3, 3] for item in gt])
    if len(glim_points) > 1:
        rr.log("glim_world/trajectory_imu", rr.LineStrips3D([glim_points]),
               static=True)
    if len(vicon_points) > 1:
        rr.log("vicon_world_unverified/trajectory_target",
               rr.LineStrips3D([vicon_points]), static=True)

    for row in health:
        stamp = float(row["timestamp"])
        rr.set_time("sensor_time", timestamp=stamp)
        rot_ratio = maximum_finite(row, "rotation_condition_ratio")
        trans_ratio = maximum_finite(row, "translation_condition_ratio")
        if rot_ratio is not None:
            rr.log("dcreg/rotation_max_condition_ratio", rr.Scalars(rot_ratio))
        if trans_ratio is not None:
            rr.log("dcreg/translation_max_condition_ratio", rr.Scalars(trans_ratio))
        rr.log("dcreg/absolute_rotation_degenerate", rr.Scalars(float(
            any_mask(row, "absolute_rotation_mask"))))
        rr.log("dcreg/absolute_translation_degenerate", rr.Scalars(float(
            any_mask(row, "absolute_translation_mask"))))
        rr.log("reference/ready", rr.Scalars(float(true_field(row, "reference_ready"))))
        if true_field(row, "health_available"):
            rot_health = minimum_finite(row, "rotation_health_smoothed")
            trans_health = minimum_finite(row, "translation_health_smoothed")
            if rot_health is not None:
                rr.log("health/rotation_min", rr.Scalars(rot_health))
            if trans_health is not None:
                rr.log("health/translation_min", rr.Scalars(trans_health))
        for entity, field in (
                ("support/source_points", "source_point_count"),
                ("support/inliers", "inlier_count"),
                ("support/inlier_fraction", "inlier_fraction"),
                ("matching/initial_cost", "initial_cost"),
                ("matching/final_cost", "final_cost")):
            if evaluator.finite(row.get(field)):
                rr.log(entity, rr.Scalars(float(row[field])))

    for edge in stable:
        rr.set_time("sensor_time", timestamp=float(edge["to_stamp_sec"]))
        rr.log("cbs/unique_edge_duration_sec", rr.Scalars(
            float(Decimal(edge["to_stamp_sec"]) - Decimal(edge["from_stamp_sec"]))))

    if args.pointcloud_npz and args.pointcloud_npz.is_file():
        archive = np.load(args.pointcloud_npz)
        for index, stamp_ns in enumerate(archive["stamp_ns"]):
            stamp = Decimal(int(stamp_ns)) / Decimal(1_000_000_000)
            nearest = min(poses, key=lambda item: abs(item["stamp"] - stamp))
            start, end = archive["offsets"][index:index + 2]
            cloud = archive["points"][start:end]
            homogeneous = np.column_stack((cloud, np.ones(len(cloud))))
            world_lidar = nearest["transform"] @ e_transform
            points = (world_lidar @ homogeneous.T).T[:, :3]
            rr.set_time("sensor_time", timestamp=float(stamp))
            rr.log("glim_world/lidar/points", rr.Points3D(points, radii=0.025))

    rr.disconnect()

    output_summary = {
        "schema_version": 1,
        "rrd": str(args.output.resolve()),
        "rrd_sha256": hashlib.sha256(args.output.read_bytes()).hexdigest(),
        "health_aggregate_count": len(health),
        "reference_ready_count": sum(true_field(row, "reference_ready")
                                     for row in health),
        "reference": evaluator.summarize_reference(args.health_csv),
        "absolute_rotation_degenerate_sample_count": sum(
            any_mask(row, "absolute_rotation_mask") for row in health),
        "absolute_translation_degenerate_sample_count": sum(
            any_mask(row, "absolute_translation_mask") for row in health),
        "unique_edge_count": len(stable),
        "deduplication": dedup,
        "vicon": gt_summary,
        "strict_edge_accuracy_eligible": False,
        "reason": "Vicon-target-to-Gamma-LiDAR extrinsic unverified",
    }
    args.summary.write_text(json.dumps(output_summary, indent=2, sort_keys=True) + "\n",
                            encoding="utf-8")
    print(json.dumps(output_summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
