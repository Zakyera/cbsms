#!/usr/bin/env python3
"""Cross-check GEODE's documented evo workflow against independent SE(3) math."""

import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
from evo.core import metrics, sync
from evo.tools import file_interface


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def pose_inverse(transform):
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -transform[:3, :3].T @ transform[:3, 3]
    return result


def so3_angle(rotation):
    cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.arccos(cosine))


def write_xy_svg(path, reference, estimate, title):
    reference_xy = np.asarray([pose[:2, 3] for pose in reference.poses_se3])
    estimate_xy = np.asarray([pose[:2, 3] for pose in estimate.poses_se3])
    combined = np.vstack((reference_xy, estimate_xy))
    minimum = np.min(combined, axis=0)
    maximum = np.max(combined, axis=0)
    span = np.maximum(maximum - minimum, 1e-12)
    width, height, margin = 900, 700, 55

    def polyline(values):
        points = []
        for x, y in values:
            px = margin + (x - minimum[0]) / span[0] * (width - 2 * margin)
            py = height - margin - (y - minimum[1]) / span[1] * (height - 2 * margin)
            points.append(f"{px:.3f},{py:.3f}")
        return " ".join(points)

    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">
<rect width="100%" height="100%" fill="white"/>
<text x="{width/2}" y="25" text-anchor="middle" font-family="sans-serif" font-size="16">{title}</text>
<polyline points="{polyline(reference_xy)}" fill="none" stroke="#2563eb" stroke-width="1.2"/>
<polyline points="{polyline(estimate_xy)}" fill="none" stroke="#dc2626" stroke-width="1.2"/>
<text x="{margin}" y="{height-16}" font-family="sans-serif" font-size="12" fill="#2563eb">GEODE-converted GLIM</text>
<text x="{margin+190}" y="{height-16}" font-family="sans-serif" font-size="12" fill="#dc2626">aligned released GT</text>
</svg>\n'''
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(svg, encoding="utf-8")


def run(args):
    # This order intentionally matches official GEODE rmse.py: method first,
    # released GT second.  evo therefore aligns the released GT to the method.
    reference = file_interface.read_tum_trajectory_file(args.estimate)
    estimate = file_interface.read_tum_trajectory_file(args.ground_truth)
    reference, estimate = sync.associate_trajectories(
        reference, estimate, max_diff=args.maximum_timestamp_difference,
        offset_2=args.time_offset, first_name="GEODE-converted GLIM",
        snd_name="released GT")
    estimate_aligned = copy.deepcopy(estimate)
    rotation, translation, scale = estimate_aligned.align(
        reference, correct_scale=False)
    if args.trajectory_svg:
        write_xy_svg(args.trajectory_svg, reference, estimate_aligned,
                     "Official-workflow aligned XY trajectories")

    ape = metrics.APE(metrics.PoseRelation.translation_part)
    ape.process_data((reference, estimate_aligned))
    rpe_translation = metrics.RPE(
        metrics.PoseRelation.translation_part, delta=1,
        delta_unit=metrics.Unit.frames, all_pairs=False)
    rpe_translation.process_data((reference, estimate_aligned))
    rpe_rotation = metrics.RPE(
        metrics.PoseRelation.rotation_angle_rad, delta=1,
        delta_unit=metrics.Unit.frames, all_pairs=False)
    rpe_rotation.process_data((reference, estimate_aligned))

    independent_ape = []
    independent_translation = []
    independent_rotation = []
    for first, second in zip(reference.poses_se3, estimate_aligned.poses_se3):
        independent_ape.append(float(np.linalg.norm(first[:3, 3] - second[:3, 3])))
    for index in range(reference.num_poses - 1):
        reference_delta = pose_inverse(reference.poses_se3[index]) @ reference.poses_se3[index + 1]
        estimate_delta = pose_inverse(estimate_aligned.poses_se3[index]) @ estimate_aligned.poses_se3[index + 1]
        error = pose_inverse(reference_delta) @ estimate_delta
        independent_translation.append(float(np.linalg.norm(error[:3, 3])))
        independent_rotation.append(so3_angle(error[:3, :3]))

    def comparison(evo_values, independent):
        evo_values = np.asarray(evo_values, dtype=float)
        independent = np.asarray(independent, dtype=float)
        return {
            "count": int(evo_values.size),
            "maximum_absolute_difference": float(np.max(np.abs(
                evo_values - independent))),
            "median_absolute_difference": float(np.median(np.abs(
                evo_values - independent))),
        }

    result = {
        "schema_version": 1,
        "official_argument_order": "converted_GLIM_reference_then_released_GT_estimate",
        "association": {
            "matched_pose_count": reference.num_poses,
            "maximum_timestamp_difference_sec": args.maximum_timestamp_difference,
            "time_offset_sec": args.time_offset,
        },
        "alignment": {
            "mode": "SE3_Umeyama_no_scale",
            "rotation": rotation.tolist(),
            "translation": translation.tolist(),
            "scale": float(scale),
        },
        "ape_translation_m": ape.get_all_statistics(),
        "rpe_translation_m": rpe_translation.get_all_statistics(),
        "rpe_rotation_rad": rpe_rotation.get_all_statistics(),
        "independent_math_comparison": {
            "ape_translation": comparison(ape.error, independent_ape),
            "rpe_translation": comparison(
                rpe_translation.error, independent_translation),
            "rpe_rotation": comparison(rpe_rotation.error, independent_rotation),
        },
        "inputs": {
            "estimate_sha256": sha256(args.estimate),
            "ground_truth_sha256": sha256(args.ground_truth),
        },
        "fitted_time_offset": False,
        "fitted_sensor_extrinsic": False,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return result


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--estimate", type=Path, required=True)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--maximum-timestamp-difference", type=float, default=0.1)
    parser.add_argument("--time-offset", type=float, default=0.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--trajectory-svg", type=Path)
    return parser.parse_args(argv)


def main(argv=None):
    run(parse_args(argv))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
