#!/usr/bin/env python3
"""Passive GEODE Gamma 6-DoF relative-edge evaluator.

This tool never writes estimator inputs. It evaluates already-recorded GLIM
beliefs against released GEODE ground truth and keeps rotation [rad] and
translation [m] separate.
"""

import argparse
import bisect
import csv
import hashlib
import json
import math
from collections import Counter, defaultdict
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from pathlib import Path

import numpy as np


NAN = float("nan")
PRIMARY_GT_BRACKET = Decimal("0.05")


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def skew(vector):
    x, y, z = np.asarray(vector, dtype=float)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def nearest_so3(matrix):
    u, _singular, vt = np.linalg.svd(np.asarray(matrix, dtype=float))
    rotation = u @ vt
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vt
    return rotation


def quaternion_normalize_xyzw(quaternion):
    result = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(result)
    if not np.isfinite(norm) or norm <= 0.0:
        raise ValueError("invalid zero/non-finite quaternion")
    return result / norm


def quaternion_to_matrix_xyzw(quaternion):
    x, y, z, w = quaternion_normalize_xyzw(quaternion)
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ])


def matrix_to_quaternion_xyzw(matrix):
    r = nearest_so3(matrix)
    trace = float(np.trace(r))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        q = np.array([(r[2, 1] - r[1, 2]) / scale,
                      (r[0, 2] - r[2, 0]) / scale,
                      (r[1, 0] - r[0, 1]) / scale,
                      0.25 * scale])
    else:
        index = int(np.argmax(np.diag(r)))
        if index == 0:
            scale = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
            q = np.array([0.25 * scale, (r[0, 1] + r[1, 0]) / scale,
                          (r[0, 2] + r[2, 0]) / scale,
                          (r[2, 1] - r[1, 2]) / scale])
        elif index == 1:
            scale = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
            q = np.array([(r[0, 1] + r[1, 0]) / scale, 0.25 * scale,
                          (r[1, 2] + r[2, 1]) / scale,
                          (r[0, 2] - r[2, 0]) / scale])
        else:
            scale = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
            q = np.array([(r[0, 2] + r[2, 0]) / scale,
                          (r[1, 2] + r[2, 1]) / scale, 0.25 * scale,
                          (r[1, 0] - r[0, 1]) / scale])
    return quaternion_normalize_xyzw(q)


def pose(rotation=None, translation=None):
    result = np.eye(4)
    if rotation is not None:
        result[:3, :3] = nearest_so3(rotation)
    if translation is not None:
        result[:3, 3] = np.asarray(translation, dtype=float)
    return result


def pose_from_tum(values):
    values = [float(value) for value in values]
    return pose(quaternion_to_matrix_xyzw(values[3:7]), values[:3])


def pose_inverse(transform):
    result = np.eye(4)
    rotation = transform[:3, :3]
    result[:3, :3] = rotation.T
    result[:3, 3] = -rotation.T @ transform[:3, 3]
    return result


def pose_between(first, second):
    return pose_inverse(first) @ second


def so3_exp(omega):
    omega = np.asarray(omega, dtype=float)
    theta = float(np.linalg.norm(omega))
    matrix = skew(omega)
    if theta < 1.0e-10:
        return np.eye(3) + matrix + 0.5 * matrix @ matrix
    a = math.sin(theta) / theta
    b = (1.0 - math.cos(theta)) / (theta * theta)
    return np.eye(3) + a * matrix + b * matrix @ matrix


def so3_log(rotation):
    rotation = nearest_so3(rotation)
    cosine = min(1.0, max(-1.0, (float(np.trace(rotation)) - 1.0) / 2.0))
    theta = math.acos(cosine)
    vee = np.array([rotation[2, 1] - rotation[1, 2],
                    rotation[0, 2] - rotation[2, 0],
                    rotation[1, 0] - rotation[0, 1]])
    if theta < 1.0e-10:
        return 0.5 * vee
    if math.pi - theta < 1.0e-7:
        quaternion = matrix_to_quaternion_xyzw(rotation)
        axis = quaternion[:3]
        norm = np.linalg.norm(axis)
        return theta * axis / norm
    return theta / (2.0 * math.sin(theta)) * vee


def se3_exp(tangent):
    tangent = np.asarray(tangent, dtype=float)
    omega, upsilon = tangent[:3], tangent[3:]
    theta = float(np.linalg.norm(omega))
    matrix = skew(omega)
    if theta < 1.0e-10:
        v_matrix = np.eye(3) + 0.5 * matrix + matrix @ matrix / 6.0
    else:
        v_matrix = (np.eye(3)
                    + (1.0 - math.cos(theta)) / theta**2 * matrix
                    + (theta - math.sin(theta)) / theta**3 * matrix @ matrix)
    return pose(so3_exp(omega), v_matrix @ upsilon)


def se3_log(transform):
    omega = so3_log(transform[:3, :3])
    theta = float(np.linalg.norm(omega))
    matrix = skew(omega)
    if theta < 1.0e-10:
        v_inverse = np.eye(3) - 0.5 * matrix + matrix @ matrix / 12.0
    else:
        coefficient = (1.0 / theta**2
                       - (1.0 + math.cos(theta))
                       / (2.0 * theta * math.sin(theta)))
        v_inverse = np.eye(3) - 0.5 * matrix + coefficient * matrix @ matrix
    return np.concatenate((omega, v_inverse @ transform[:3, 3]))


def official_transforms(profile):
    raw = np.asarray(profile["T_I_L_released"], dtype=float)
    e_transform = pose(nearest_so3(raw[:3, :3]), raw[:3, 3])
    released = profile["S_gamma_lidar_to_beta_gt"]
    qw, qx, qy, qz = released["quaternion_wxyz_released"]
    s_transform = pose(
        quaternion_to_matrix_xyzw([qx, qy, qz, qw]),
        released["translation_m"],
    )
    return e_transform, s_transform


def imu_pose_to_beta(world_imu, e_transform, s_transform):
    return world_imu @ e_transform @ pose_inverse(s_transform)


def imu_edge_to_beta(edge_imu, e_transform, s_transform):
    edge_lidar = pose_inverse(e_transform) @ edge_imu @ e_transform
    return s_transform @ edge_lidar @ pose_inverse(s_transform)


def quaternion_slerp(first, second, fraction):
    first = quaternion_normalize_xyzw(first)
    second = quaternion_normalize_xyzw(second)
    dot = float(first @ second)
    if dot < 0.0:
        second = -second
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return quaternion_normalize_xyzw(first + fraction * (second - first))
    theta = math.acos(dot)
    return ((math.sin((1.0 - fraction) * theta) * first
             + math.sin(fraction * theta) * second) / math.sin(theta))


@dataclass(frozen=True)
class GroundTruthRecord:
    stamp: Decimal
    transform: np.ndarray
    quaternion: np.ndarray
    original_index: int


def same_gt_pose(first, second, tolerance=1.0e-12):
    translation_equal = np.linalg.norm(
        first.transform[:3, 3] - second.transform[:3, 3]) <= tolerance
    quaternion_equal = abs(float(first.quaternion @ second.quaternion)) >= 1.0 - tolerance
    return translation_equal and quaternion_equal


def load_ground_truth(path):
    raw = []
    backward_steps = 0
    previous = None
    nonfinite = 0
    with Path(path).open(encoding="utf-8") as stream:
        for original_index, line in enumerate(stream):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            fields = stripped.split()
            if len(fields) < 8:
                raise ValueError(f"GT row {original_index} has fewer than 8 columns")
            try:
                stamp = Decimal(fields[0])
                numeric = [float(value) for value in fields[1:8]]
            except (InvalidOperation, ValueError) as error:
                raise ValueError(f"invalid GT row {original_index}: {error}") from error
            if not stamp.is_finite() or not all(math.isfinite(v) for v in numeric):
                nonfinite += 1
                continue
            if previous is not None and stamp < previous:
                backward_steps += 1
            previous = stamp
            quaternion = quaternion_normalize_xyzw(numeric[3:7])
            raw.append(GroundTruthRecord(
                stamp, pose(quaternion_to_matrix_xyzw(quaternion), numeric[:3]),
                quaternion, original_index))
    ordered = sorted(raw, key=lambda record: record.stamp)  # Python sort is stable.
    normalized = []
    harmless_duplicates = 0
    conflicting_duplicates = 0
    for record in ordered:
        if normalized and record.stamp == normalized[-1].stamp:
            if same_gt_pose(record, normalized[-1]):
                harmless_duplicates += 1
                continue
            conflicting_duplicates += 1
            raise ValueError(
                f"conflicting GT duplicate timestamp {record.stamp} at source rows "
                f"{normalized[-1].original_index} and {record.original_index}")
        normalized.append(record)
    gaps = np.asarray([
        float(normalized[index + 1].stamp - normalized[index].stamp)
        for index in range(len(normalized) - 1)
    ], dtype=float)
    return normalized, {
        "input_pose_count": len(raw),
        "normalized_pose_count": len(normalized),
        "source_backward_step_count": backward_steps,
        "nonfinite_rejected_count": nonfinite,
        "harmless_duplicate_count": harmless_duplicates,
        "conflicting_duplicate_count": conflicting_duplicates,
        "first_timestamp": str(normalized[0].stamp) if normalized else None,
        "last_timestamp": str(normalized[-1].stamp) if normalized else None,
        "stable_sort": True,
        "gap_seconds": {
            "count": int(gaps.size),
            "median": float(np.median(gaps)) if gaps.size else None,
            "p95": float(np.percentile(gaps, 95)) if gaps.size else None,
            "maximum": float(np.max(gaps)) if gaps.size else None,
            "greater_than_0_05_count": int(np.count_nonzero(gaps > 0.05)),
        },
    }


def interpolate_ground_truth(records, stamp, maximum_bracket=PRIMARY_GT_BRACKET):
    stamp = stamp if isinstance(stamp, Decimal) else Decimal(str(stamp))
    if not records or stamp < records[0].stamp or stamp > records[-1].stamp:
        return None, {"valid": False, "reason": "outside_coverage"}
    stamps = [record.stamp for record in records]
    position = bisect.bisect_left(stamps, stamp)
    if position < len(records) and records[position].stamp == stamp:
        record = records[position]
        return record.transform.copy(), {
            "valid": True, "status": "exact", "left_timestamp": str(stamp),
            "right_timestamp": str(stamp), "left_gap_sec": 0.0,
            "right_gap_sec": 0.0, "total_bracket_sec": 0.0,
            "fraction": 0.0, "original_left_index": record.original_index,
            "original_right_index": record.original_index,
        }
    if position == 0 or position >= len(records):
        return None, {"valid": False, "reason": "outside_coverage"}
    left, right = records[position - 1], records[position]
    bracket = right.stamp - left.stamp
    # Decimal comparison is exact: equality at precisely 0.05 seconds passes.
    if bracket > maximum_bracket:
        return None, {
            "valid": False, "reason": "bracket_exceeds_limit",
            "left_timestamp": str(left.stamp), "right_timestamp": str(right.stamp),
            "total_bracket_sec": float(bracket),
        }
    fraction_decimal = (stamp - left.stamp) / bracket
    fraction = float(fraction_decimal)
    translation = ((1.0 - fraction) * left.transform[:3, 3]
                   + fraction * right.transform[:3, 3])
    quaternion = quaternion_slerp(left.quaternion, right.quaternion, fraction)
    return pose(quaternion_to_matrix_xyzw(quaternion), translation), {
        "valid": True, "status": "interpolated",
        "left_timestamp": str(left.stamp), "right_timestamp": str(right.stamp),
        "left_gap_sec": float(stamp - left.stamp),
        "right_gap_sec": float(right.stamp - stamp),
        "total_bracket_sec": float(bracket), "fraction": fraction,
        "original_left_index": left.original_index,
        "original_right_index": right.original_index,
    }


def read_csv(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def read_glim_poses(path):
    poses = []
    for pose_index, row in enumerate(read_csv(path)):
        poses.append({
            "pose_index": pose_index,
            "stamp": Decimal(row["stamp_ns"]) / Decimal(1_000_000_000),
            "stamp_ns": int(row["stamp_ns"]),
            "transform": pose_from_tum([
                row["px"], row["py"], row["pz"], row["qx"], row["qy"],
                row["qz"], row["qw"],
            ]),
        })
    return poses


def belief_edge(row):
    tangent = [float(row[f"relative_mu_{index}"]) for index in range(6)]
    return se3_exp(tangent)


def stable_edge_key(row):
    return (int(row["from_pose_index"]), int(row["to_pose_index"]),
            row["from_stamp_sec"], row["to_stamp_sec"])


def select_last_stable_edges(rows):
    groups = defaultdict(list)
    for row in rows:
        if int(row.get("belief_index", -1)) < 0:
            continue
        groups[stable_edge_key(row)].append(row)
    selected = []
    for key, candidates in groups.items():
        candidates.sort(key=lambda row: (int(row["message_index"]),
                                         int(row["belief_index"])))
        selected.append(candidates[-1])
    selected.sort(key=lambda row: (Decimal(row["to_stamp_sec"]),
                                   int(row["to_pose_index"])))
    return selected, {
        "total_belief_publication_rows": sum(len(group) for group in groups.values()),
        "unique_stable_edge_count": len(selected),
        "republication_row_count": sum(max(0, len(group) - 1)
                                       for group in groups.values()),
        "selection_policy": "last_valid_publication_per_stable_edge_without_Kimera",
    }


def metadata_index(rows):
    return {
        (int(row["publication_sequence"]), int(row["belief_ordinal"])): row
        for row in rows
    }


def percentile(values, quantile):
    clean = np.asarray([float(value) for value in values if finite(value)], dtype=float)
    if not clean.size:
        return None
    return float(np.percentile(clean, quantile * 100.0))


def numeric_summary(values):
    clean = np.asarray([float(value) for value in values if finite(value)], dtype=float)
    if not clean.size:
        return {"count": 0, "minimum": None, "median": None, "mean": None,
                "p95": None, "maximum": None}
    return {"count": int(clean.size), "minimum": float(np.min(clean)),
            "median": float(np.median(clean)), "mean": float(np.mean(clean)),
            "p95": float(np.percentile(clean, 95)), "maximum": float(np.max(clean))}


def average_ranks(values):
    values = np.asarray(values, dtype=float)
    order = np.argsort(values, kind="mergesort")
    ranks = np.empty(len(values), dtype=float)
    start = 0
    while start < len(values):
        end = start + 1
        while end < len(values) and values[order[end]] == values[order[start]]:
            end += 1
        ranks[order[start:end]] = 0.5 * (start + 1 + end)
        start = end
    return ranks


def spearman_summary(rows, predictor, response):
    pairs = [(float(row[predictor]), float(row[response])) for row in rows
             if finite(row.get(predictor)) and finite(row.get(response))]
    if len(pairs) < 3:
        return {"count": len(pairs), "rho": None}
    x, y = np.asarray(pairs, dtype=float).T
    x_rank, y_rank = average_ranks(x), average_ranks(y)
    if np.std(x_rank) == 0.0 or np.std(y_rank) == 0.0:
        return {"count": len(pairs), "rho": None}
    return {"count": len(pairs),
            "rho": float(np.corrcoef(x_rank, y_rank)[0, 1])}


def summarize_reference(health_csv):
    if not health_csv or not Path(health_csv).is_file():
        return {"available": False}
    rows = [row for row in read_csv(health_csv) if row.get("record_kind") == "aggregate"]
    reasons = Counter(row.get("baseline_update_reason", "") or "none" for row in rows)
    ready = [row for row in rows if row.get("reference_ready") == "1"]
    accepted = [row for row in rows if row.get("baseline_update_accepted") == "1"]
    max_rot = [max(float(row[f"rotation_condition_ratio_{i}"]) for i in range(3))
               for row in rows if all(finite(row.get(f"rotation_condition_ratio_{i}"))
                                      for i in range(3))]
    max_trans = [max(float(row[f"translation_condition_ratio_{i}"]) for i in range(3))
                 for row in rows if all(finite(row.get(f"translation_condition_ratio_{i}"))
                                        for i in range(3))]
    return {
        "available": True, "aggregate_sample_count": len(rows),
        "accepted_reference_candidate_count": len(accepted),
        "reference_ready_sample_count": len(ready),
        "first_reference_ready_frame": int(ready[0]["frame_id"]) if ready else None,
        "reference_source_at_end": rows[-1].get("reference_source") if rows else None,
        "baseline_reason_counts": dict(reasons),
        "maximum_rotation_condition_ratio": numeric_summary(max_rot),
        "maximum_translation_condition_ratio": numeric_summary(max_trans),
    }


def bool_field(row, name):
    return bool(row) and str(row.get(name, "0")) in ("1", "true", "True")


def evaluate_edges(args, profile, gt_records):
    e_transform, s_transform = official_transforms(profile)
    poses = read_glim_poses(args.glim_csv)
    belief_rows = read_csv(args.belief_csv)
    selected, dedup = select_last_stable_edges(belief_rows)
    metadata_rows = read_csv(args.metadata_csv) if args.metadata_csv else []
    metadata = metadata_index(metadata_rows)
    results = []
    rejection_reasons = Counter()
    for belief in selected:
        publication_sequence = int(belief["message_index"]) + 1
        ordinal = int(belief["belief_index"])
        associated = metadata.get((publication_sequence, ordinal))
        if associated and (int(associated["from_pose_index"]) != int(belief["from_pose_index"])
                           or int(associated["to_pose_index"]) != int(belief["to_pose_index"])):
            associated = None
        start_stamp = Decimal(belief["from_stamp_sec"])
        end_stamp = Decimal(belief["to_stamp_sec"])
        gt_start, start_info = interpolate_ground_truth(
            gt_records, start_stamp, Decimal(str(args.maximum_gt_bracket_sec)))
        gt_end, end_info = interpolate_ground_truth(
            gt_records, end_stamp, Decimal(str(args.maximum_gt_bracket_sec)))
        if gt_start is None or gt_end is None:
            rejection_reasons[(start_info.get("reason") if gt_start is None
                               else end_info.get("reason", "unknown"))] += 1
            continue
        z_gt = pose_between(gt_start, gt_end)
        z_imu = belief_edge(belief)
        z_lidar = pose_inverse(e_transform) @ z_imu @ e_transform
        z_beta = s_transform @ z_lidar @ pose_inverse(s_transform)
        residual = se3_log(pose_inverse(z_gt) @ z_beta)
        z_gt_log = se3_log(z_gt)
        z_imu_log = se3_log(z_imu)
        z_lidar_log = se3_log(z_lidar)
        z_beta_log = se3_log(z_beta)
        row = {
            "message_index": int(belief["message_index"]),
            "publication_sequence": publication_sequence,
            "belief_ordinal": ordinal,
            "from_pose_index": int(belief["from_pose_index"]),
            "to_pose_index": int(belief["to_pose_index"]),
            "from_stamp_sec": float(start_stamp), "to_stamp_sec": float(end_stamp),
            "edge_duration_sec": float(end_stamp - start_stamp),
            "gt_start_status": start_info["status"],
            "gt_end_status": end_info["status"],
            "gt_start_bracket_sec": start_info["total_bracket_sec"],
            "gt_end_bracket_sec": end_info["total_bracket_sec"],
            "error_rotation_x_rad": residual[0], "error_rotation_y_rad": residual[1],
            "error_rotation_z_rad": residual[2], "error_translation_x_m": residual[3],
            "error_translation_y_m": residual[4], "error_translation_z_m": residual[5],
            "rotation_error_rad": float(np.linalg.norm(residual[:3])),
            "translation_error_m": float(np.linalg.norm(residual[3:])),
            "metadata_available": associated is not None,
        }
        for prefix, tangent in (("z_gt", z_gt_log), ("z_imu", z_imu_log),
                                ("z_lidar", z_lidar_log),
                                ("z_beta", z_beta_log)):
            for component, value in zip(("rx", "ry", "rz", "tx", "ty", "tz"),
                                        tangent):
                row[f"{prefix}_{component}"] = float(value)
        for name in (
            "association_valid", "association_complete", "reference_ready",
            "reference_consistent", "relative_health_valid",
            "rotational_health_minimum", "rotational_health_median",
            "translational_health_minimum", "translational_health_median",
            "rotational_degraded_count", "rotational_degraded_denominator",
            "translational_degraded_count", "translational_degraded_denominator",
            "rotational_absolute_degenerate_count",
            "rotational_absolute_degenerate_denominator",
            "translational_absolute_degenerate_count",
            "translational_absolute_degenerate_denominator",
            "rotational_absolute_degenerate_fraction",
            "translational_absolute_degenerate_fraction",
            "worst_rotational_condition_ratio",
            "worst_translational_condition_ratio",
            "longest_rotational_degraded_run", "longest_translational_degraded_run",
            "aggregate_source_point_count_median", "aggregate_inlier_fraction_median",
            "aggregate_initial_cost_median", "aggregate_final_cost_median",
            "clustered_rotation_fraction", "clustered_translation_fraction",
            "low_rotation_alignment_fraction", "low_translation_alignment_fraction",
        ):
            row[name] = associated.get(name, "") if associated else ""
        row["absolute_rotation_degenerate"] = bool(
            associated and int(associated["rotational_absolute_degenerate_count"]) > 0)
        row["absolute_translation_degenerate"] = bool(
            associated and int(associated["translational_absolute_degenerate_count"]) > 0)
        results.append(row)
    return results, poses, dedup, dict(rejection_reasons)


def transform_difference(first, second):
    return se3_log(pose_inverse(first) @ second)


def endpoint_edge_crosscheck(poses, e_transform, s_transform):
    if len(poses) < 2:
        return {"sample_count": 0}
    errors = []
    step = max(1, len(poses) // 100)
    for index in range(0, len(poses) - 1, step):
        first, second = poses[index]["transform"], poses[index + 1]["transform"]
        path_a = pose_between(imu_pose_to_beta(first, e_transform, s_transform),
                              imu_pose_to_beta(second, e_transform, s_transform))
        path_b = imu_edge_to_beta(pose_between(first, second), e_transform, s_transform)
        errors.append(float(np.linalg.norm(transform_difference(path_a, path_b))))
    return {"sample_count": len(errors), "maximum_se3_log_norm": max(errors),
            "median_se3_log_norm": float(np.median(errors))}


def stratified_summary(rows, predicate):
    selected = [row for row in rows if predicate(row)]
    return {"edge_count": len(selected),
            "rotation_error_rad": numeric_summary(row["rotation_error_rad"] for row in selected),
            "translation_error_m": numeric_summary(row["translation_error_m"] for row in selected)}


def write_csv(path, rows):
    if not rows:
        return
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_svg(path, rows, title):
    if not rows:
        Path(path).write_text("<svg xmlns='http://www.w3.org/2000/svg'/>\n")
        return
    width, height, margin = 1400, 850, 100
    t0, t1 = rows[0]["to_stamp_sec"], rows[-1]["to_stamp_sec"]
    if t1 <= t0:
        t1 = t0 + 1.0
    panels = [
        ("rotation error [rad]", "rotation_error_rad", "#d62728"),
        ("translation error [m]", "translation_error_m", "#1f77b4"),
        ("rotation health min", "rotational_health_minimum", "#9467bd"),
        ("translation health min", "translational_health_minimum", "#2ca02c"),
        ("GT max bracket [s]", "gt_bracket", "#ff7f0e"),
    ]
    panel_h = (height - 2 * margin) / len(panels)
    svg = [f"<svg xmlns='http://www.w3.org/2000/svg' width='{width}' height='{height}'>",
           "<rect width='100%' height='100%' fill='white'/>",
           f"<text x='{margin}' y='35' font-size='22'>{title}</text>"]
    for panel, (label, key, color) in enumerate(panels):
        top = margin + panel * panel_h
        bottom = top + panel_h - 22
        values = []
        for row in rows:
            value = (max(row["gt_start_bracket_sec"], row["gt_end_bracket_sec"])
                     if key == "gt_bracket" else row.get(key))
            if finite(value):
                values.append(float(value))
        maximum = max(values) if values else 1.0
        maximum = max(maximum, 1.0e-12)
        points = []
        for row in rows:
            value = (max(row["gt_start_bracket_sec"], row["gt_end_bracket_sec"])
                     if key == "gt_bracket" else row.get(key))
            if not finite(value):
                continue
            x = margin + (row["to_stamp_sec"] - t0) / (t1 - t0) * (width - 2 * margin)
            y = bottom - float(value) / maximum * (panel_h - 35)
            points.append(f"{x:.2f},{y:.2f}")
        svg.extend([
            f"<text x='8' y='{top + 15}' font-size='13'>{label}</text>",
            f"<line x1='{margin}' y1='{bottom}' x2='{width-margin}' y2='{bottom}' stroke='#aaa'/>",
        ])
        if points:
            svg.append(f"<polyline fill='none' stroke='{color}' stroke-width='1.5' points='{' '.join(points)}'/>")
    svg.append("</svg>")
    Path(path).write_text("\n".join(svg) + "\n", encoding="utf-8")


def write_rerun(path, rows, poses, gt_records, profile, pointcloud_npz, title):
    import rerun as rr
    e_transform, s_transform = official_transforms(profile)
    rr.init("cbsms_geode_stage2c_b", recording_id=title)
    rr.save(str(path))
    rr.log("documentation", rr.TextDocument(
        f"# {title}\nPassive GEODE Stage 2C-B. Health is not covariance, probability, or metric error."),
        static=True)
    valid_stamps = [Decimal(str(row["to_stamp_sec"])) for row in rows]
    if valid_stamps:
        first_stamp = valid_stamps[0]
        gt_first, _ = interpolate_ground_truth(gt_records, first_stamp, Decimal("0.20"))
        nearest_pose = min(poses, key=lambda item: abs(item["stamp"] - first_stamp))
        world_beta_first = imu_pose_to_beta(nearest_pose["transform"], e_transform, s_transform)
        visualization_alignment = (gt_first @ pose_inverse(world_beta_first)
                                   if gt_first is not None else np.eye(4))
    else:
        visualization_alignment = np.eye(4)
    glim_points = [(visualization_alignment @ imu_pose_to_beta(
        item["transform"], e_transform, s_transform))[:3, 3] for item in poses]
    gt_points = [record.transform[:3, 3] for record in gt_records]
    if len(glim_points) > 1:
        rr.log("world/trajectories/glim_beta_visual_alignment",
               rr.LineStrips3D([np.asarray(glim_points)]), static=True)
    if len(gt_points) > 1:
        rr.log("world/trajectories/ground_truth_beta",
               rr.LineStrips3D([np.asarray(gt_points)]), static=True)
    for row in rows:
        rr.set_time("sensor_time", timestamp=row["to_stamp_sec"])
        rr.log("edge_error/rotation_rad", rr.Scalars(row["rotation_error_rad"]))
        rr.log("edge_error/translation_m", rr.Scalars(row["translation_error_m"]))
        rr.log("gt/max_interpolation_bracket_sec", rr.Scalars(max(
            row["gt_start_bracket_sec"], row["gt_end_bracket_sec"])))
        for entity, key in (("health/rotation_min", "rotational_health_minimum"),
                            ("health/translation_min", "translational_health_minimum"),
                            ("support/inlier_fraction", "aggregate_inlier_fraction_median"),
                            ("matching/initial_cost", "aggregate_initial_cost_median"),
                            ("matching/final_cost", "aggregate_final_cost_median")):
            if finite(row.get(key)):
                rr.log(entity, rr.Scalars(float(row[key])))
        rr.log("absolute/rotation_degenerate", rr.Scalars(
            float(row["absolute_rotation_degenerate"])))
        rr.log("absolute/translation_degenerate", rr.Scalars(
            float(row["absolute_translation_degenerate"])))
        rr.log("reference/ready", rr.Scalars(float(bool_field(row, "reference_ready"))))
    if pointcloud_npz and Path(pointcloud_npz).is_file():
        archive = np.load(pointcloud_npz)
        cloud_stamps = archive["stamp_ns"]
        offsets = archive["offsets"]
        points = archive["points"]
        for index, stamp_ns in enumerate(cloud_stamps):
            stamp = Decimal(int(stamp_ns)) / Decimal(1_000_000_000)
            nearest_pose = min(poses, key=lambda item: abs(item["stamp"] - stamp))
            world_lidar = nearest_pose["transform"] @ e_transform
            cloud = points[offsets[index]:offsets[index + 1]]
            homogeneous = np.column_stack((cloud, np.ones(len(cloud))))
            world = (visualization_alignment @ world_lidar @ homogeneous.T).T[:, :3]
            rr.set_time("sensor_time", timestamp=float(stamp))
            rr.log("world/lidar/points", rr.Points3D(world, radii=0.025))
    rr.disconnect()
    return {
        "path": str(Path(path).resolve()),
        "sha256": hashlib.sha256(Path(path).read_bytes()).hexdigest(),
        "visualization_world_alignment": "first GT-valid endpoint only; not used for edge error",
    }


def run(args):
    profile = json.loads(args.profile.read_text(encoding="utf-8"))
    gt_records, gt_stats = load_ground_truth(args.ground_truth)
    rows, poses, dedup, rejections = evaluate_edges(args, profile, gt_records)
    e_transform, s_transform = official_transforms(profile)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "edge_accuracy_stable_edges.csv", rows)
    write_svg(args.output_dir / "edge_accuracy_timeline.svg", rows, args.dataset)
    reference = summarize_reference(args.health_csv)
    summary = {
        "schema_version": 1,
        "semantics": "passive GEODE released-calibration relative-edge evaluation",
        "dataset": args.dataset,
        "profile_sha256": hashlib.sha256(args.profile.read_bytes()).hexdigest(),
        "ground_truth": gt_stats,
        "primary_maximum_gt_bracket_sec": args.maximum_gt_bracket_sec,
        "gt_bracket_comparison": "exact Decimal total bracket <= limit; equality passes",
        "deduplication": dedup,
        "gt_rejection_reason_counts": rejections,
        "gt_valid_unique_edge_count": len(rows),
        "rotation_error_rad": numeric_summary(row["rotation_error_rad"] for row in rows),
        "translation_error_m": numeric_summary(row["translation_error_m"] for row in rows),
        "absolute_rotation_degenerate": stratified_summary(
            rows, lambda row: row["absolute_rotation_degenerate"]),
        "absolute_rotation_not_degenerate": stratified_summary(
            rows, lambda row: int(row.get(
                "rotational_absolute_degenerate_denominator") or 0) > 0
            and not row["absolute_rotation_degenerate"]),
        "absolute_rotation_unavailable": stratified_summary(
            rows, lambda row: int(row.get(
                "rotational_absolute_degenerate_denominator") or 0) == 0),
        "absolute_translation_degenerate": stratified_summary(
            rows, lambda row: row["absolute_translation_degenerate"]),
        "absolute_translation_not_degenerate": stratified_summary(
            rows, lambda row: int(row.get(
                "translational_absolute_degenerate_denominator") or 0) > 0
            and not row["absolute_translation_degenerate"]),
        "absolute_translation_unavailable": stratified_summary(
            rows, lambda row: int(row.get(
                "translational_absolute_degenerate_denominator") or 0) == 0),
        "relative_health_available_edge_count": sum(
            bool_field(row, "relative_health_valid") for row in rows),
        "preliminary_spearman_rank_associations": {
            "worst_rotation_condition_vs_rotation_error": spearman_summary(
                rows, "worst_rotational_condition_ratio", "rotation_error_rad"),
            "worst_translation_condition_vs_translation_error": spearman_summary(
                rows, "worst_translational_condition_ratio", "translation_error_m"),
            "source_points_vs_translation_error": spearman_summary(
                rows, "aggregate_source_point_count_median", "translation_error_m"),
            "final_matching_cost_vs_translation_error": spearman_summary(
                rows, "aggregate_final_cost_median", "translation_error_m"),
            "interpretation": (
                "exploratory monotonic association on deduplicated unique edges; "
                "not a health-to-error calibration and no independence/causality claim"
            ),
        },
        "reference": reference,
        "endpoint_vs_edge_conjugation": endpoint_edge_crosscheck(
            poses, e_transform, s_transform),
        "frame_chain": {
            "Z_L": "inverse(E) * Z_I * E",
            "Z_B": "S * Z_L * inverse(S)",
            "residual": "Logmap(inverse(Z_GT) * Z_B)",
            "rotation_and_translation_reported_separately": True,
        },
        "scientific_limits": [
            "Stage 1 health is scan-to-rolling-map observability, not edge error.",
            "No ratio-to-metric-error or health-to-error regression is fitted.",
            "No covariance, probability, trust weight, or control output is produced.",
            "One GEODE degraded sequence is a pilot, not a held-out healthy/degraded validation suite.",
        ],
    }
    if args.rerun_rrd:
        summary["rerun"] = write_rerun(
            args.rerun_rrd, rows, poses, gt_records, profile,
            args.pointcloud_npz, f"GEODE Stage 2C-B {args.dataset}")
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False))
    return summary


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--glim-csv", type=Path, required=True)
    parser.add_argument("--belief-csv", type=Path, required=True)
    parser.add_argument("--metadata-csv", type=Path)
    parser.add_argument("--health-csv", type=Path)
    parser.add_argument("--maximum-gt-bracket-sec", type=float, default=0.05)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--rerun-rrd", type=Path)
    parser.add_argument("--pointcloud-npz", type=Path)
    return parser.parse_args(argv)


def main(argv=None):
    run(parse_args(argv))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
