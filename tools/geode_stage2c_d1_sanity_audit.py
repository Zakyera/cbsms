#!/usr/bin/env python3
"""Offline GEODE Stage 2C-D1 edge-evaluator sanity audit.

This tool reads frozen Stage 2C-B/C artifacts.  It cannot configure or invoke
GLIM, cannot update a health reference, and never replaces the primary edge
error with a diagnostic alternative.
"""

import argparse
import csv
import hashlib
import json
import math
from decimal import Decimal
from pathlib import Path

import numpy as np

import geode_stage2c_b_evaluator as base


COMPONENTS = ("rx", "ry", "rz", "tx", "ty", "tz")


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def read_csv(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def numeric_summary(values):
    clean = np.asarray([float(value) for value in values if finite(value)], dtype=float)
    if clean.size == 0:
        return {"count": 0, "minimum": None, "p10": None, "median": None,
                "p90": None, "p95": None, "maximum": None}
    return {
        "count": int(clean.size),
        "minimum": float(np.min(clean)),
        "p10": float(np.percentile(clean, 10)),
        "median": float(np.median(clean)),
        "p90": float(np.percentile(clean, 90)),
        "p95": float(np.percentile(clean, 95)),
        "maximum": float(np.max(clean)),
    }


def tangent_from_row(row, prefix):
    return np.asarray([float(row[f"{prefix}_{name}"]) for name in COMPONENTS])


def residual_metrics(reference, estimate):
    residual = base.se3_log(base.pose_inverse(reference) @ estimate)
    return {
        "residual": residual,
        "rotation_error_rad": float(np.linalg.norm(residual[:3])),
        "translation_error_m": float(np.linalg.norm(residual[3:])),
        "raw_group_translation_residual_m": float(np.linalg.norm(
            (base.pose_inverse(reference) @ estimate)[:3, 3])),
    }


def vector_cosine(first, second):
    first = np.asarray(first, dtype=float)
    second = np.asarray(second, dtype=float)
    first_norm = float(np.linalg.norm(first))
    second_norm = float(np.linalg.norm(second))
    if first_norm <= 0.0 or second_norm <= 0.0:
        return float("nan")
    return float(first @ second / (first_norm * second_norm))


def constant_speed_time_offset_displacement(speed_mps, offset_sec):
    """Geometric displacement caused by a constant clock offset (diagnostic)."""
    return abs(float(speed_mps) * float(offset_sec))


def diagnostic_edge(row, epsilon, lever_arm_m):
    z_gt = base.se3_exp(tangent_from_row(row, "z_gt"))
    z_glim = base.se3_exp(tangent_from_row(row, "z_beta"))
    primary = residual_metrics(z_gt, z_glim)
    alternatives = {
        "A_primary": primary,
        "B_invert_glim": residual_metrics(z_gt, base.pose_inverse(z_glim)),
        "C_invert_gt": residual_metrics(base.pose_inverse(z_gt), z_glim),
        "D_invert_both": residual_metrics(
            base.pose_inverse(z_gt), base.pose_inverse(z_glim)),
    }
    duration = float(row["edge_duration_sec"])
    gt_translation = z_gt[:3, 3]
    glim_translation = z_glim[:3, 3]
    gt_displacement = float(np.linalg.norm(gt_translation))
    glim_displacement = float(np.linalg.norm(glim_translation))
    gt_rotation = float(np.linalg.norm(base.so3_log(z_gt[:3, :3])))
    glim_rotation = float(np.linalg.norm(base.so3_log(z_glim[:3, :3])))
    output = dict(row)
    output.update({
        "gt_displacement_m": gt_displacement,
        "glim_displacement_m": glim_displacement,
        "translation_residual_ratio": (
            primary["translation_error_m"] / max(gt_displacement, epsilon)),
        "translation_vector_dot_m2": float(gt_translation @ glim_translation),
        "translation_vector_cosine": vector_cosine(gt_translation, glim_translation),
        "gt_rotation_magnitude_rad": gt_rotation,
        "glim_rotation_magnitude_rad": glim_rotation,
        "gt_speed_mps": gt_displacement / duration if duration > 0.0 else float("nan"),
        "glim_speed_mps": glim_displacement / duration if duration > 0.0 else float("nan"),
        "lever_arm_translation_bound_m": (
            2.0 * lever_arm_m * math.sin(min(math.pi, gt_rotation) / 2.0)),
        "raw_group_translation_residual_m": (
            primary["raw_group_translation_residual_m"]),
    })
    for name, metrics in alternatives.items():
        output[f"{name}_rotation_error_rad"] = metrics["rotation_error_rad"]
        output[f"{name}_translation_error_m"] = metrics["translation_error_m"]
        output[f"{name}_raw_group_translation_residual_m"] = (
            metrics["raw_group_translation_residual_m"])
    return output


def add_gt_rotation_transpose_diagnostic(row, gt_records, maximum_bracket):
    """Test a mixed navigation convention: world position plus R_body_world.

    This is diagnostic only.  It is not one of the accepted primary evaluator
    conventions and cannot replace the released TUM interpretation.
    """
    start, _ = base.interpolate_ground_truth(
        gt_records, Decimal(row["from_stamp_sec"]), maximum_bracket)
    end, _ = base.interpolate_ground_truth(
        gt_records, Decimal(row["to_stamp_sec"]), maximum_bracket)
    if start is None or end is None:
        row["F_transpose_gt_rotation_translation_error_m"] = float("nan")
        row["F_transpose_gt_rotation_rotation_error_rad"] = float("nan")
        return row
    start_mixed = base.pose(start[:3, :3].T, start[:3, 3])
    end_mixed = base.pose(end[:3, :3].T, end[:3, 3])
    alternative_gt = base.pose_between(start_mixed, end_mixed)
    z_glim = base.se3_exp(tangent_from_row(row, "z_beta"))
    diagnostic = residual_metrics(alternative_gt, z_glim)
    row["F_transpose_gt_rotation_translation_error_m"] = diagnostic[
        "translation_error_m"]
    row["F_transpose_gt_rotation_rotation_error_rad"] = diagnostic[
        "rotation_error_rad"]
    return row


def alternative_summaries(rows, reduction_factor):
    result = {}
    for name in ("A_primary", "B_invert_glim", "C_invert_gt", "D_invert_both"):
        result[name] = {
            "rotation_error_rad": numeric_summary(
                row[f"{name}_rotation_error_rad"] for row in rows),
            "translation_error_m": numeric_summary(
                row[f"{name}_translation_error_m"] for row in rows),
            "raw_group_translation_residual_m": numeric_summary(
                row[f"{name}_raw_group_translation_residual_m"] for row in rows),
        }
    primary = result["A_primary"]["translation_error_m"]["median"]
    flagged = []
    if primary is not None:
        for name in ("B_invert_glim", "C_invert_gt", "D_invert_both"):
            value = result[name]["translation_error_m"]["median"]
            if value is not None and value > 0.0 and primary / value >= reduction_factor:
                flagged.append(name)
    return {
        "alternatives": result,
        "orders_of_magnitude_threshold_factor": reduction_factor,
        "flagged_alternatives": flagged,
        "primary_was_replaced": False,
    }


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


def spearman(first, second):
    pairs = [(float(a), float(b)) for a, b in zip(first, second)
             if finite(a) and finite(b)]
    if len(pairs) < 3:
        return None
    a, b = np.asarray(pairs, dtype=float).T
    a_rank, b_rank = average_ranks(a), average_ranks(b)
    if np.std(a_rank) == 0.0 or np.std(b_rank) == 0.0:
        return None
    return float(np.corrcoef(a_rank, b_rank)[0, 1])


def matrix_token(matrix):
    return ";".join(format(float(value), ".17g") for value in matrix.reshape(-1))


def pose_token(transform):
    quaternion = base.matrix_to_quaternion_xyzw(transform[:3, :3])
    values = list(transform[:3, 3]) + list(quaternion)
    return ";".join(format(float(value), ".17g") for value in values)


def manual_selections(rows, plan):
    count = int(plan["manual_edge_selection"]["count_per_group"])
    ordered = sorted(rows, key=lambda row: (
        float(row["to_stamp_sec"]), int(row["from_pose_index"]),
        int(row["to_pose_index"])))
    median = float(np.median([float(row["A_primary_translation_error_m"])
                              for row in rows]))
    p95 = float(np.percentile([float(row["A_primary_translation_error_m"])
                               for row in rows], 95))
    stable = lambda row: (int(row["from_pose_index"]), int(row["to_pose_index"]))
    groups = {
        "first": ordered[:count],
        "median_error": sorted(
            rows, key=lambda row: (abs(float(row["A_primary_translation_error_m"])
                                      - median), stable(row)))[:count],
        "p95_error": sorted(
            rows, key=lambda row: (abs(float(row["A_primary_translation_error_m"])
                                      - p95), stable(row)))[:count],
    }
    conditioned = [row for row in rows
                   if finite(row.get("worst_translational_condition_ratio"))]
    groups["low_conditioning"] = sorted(
        conditioned, key=lambda row: (
            float(row["worst_translational_condition_ratio"]), stable(row)))[:count]
    groups["high_conditioning"] = sorted(
        conditioned, key=lambda row: (
            -float(row["worst_translational_condition_ratio"]), stable(row)))[:count]
    return groups


def detailed_manual_rows(groups, glim_poses, gt_records, e_transform,
                         s_transform, maximum_bracket):
    result = []
    for group, rows in groups.items():
        for group_index, row in enumerate(rows):
            start_index = int(row["from_pose_index"])
            end_index = int(row["to_pose_index"])
            start_stamp = Decimal(row["from_stamp_sec"])
            end_stamp = Decimal(row["to_stamp_sec"])
            gt_start, start_info = base.interpolate_ground_truth(
                gt_records, start_stamp, maximum_bracket)
            gt_end, end_info = base.interpolate_ground_truth(
                gt_records, end_stamp, maximum_bracket)
            z_gt = base.se3_exp(tangent_from_row(row, "z_gt"))
            z_imu = base.se3_exp(tangent_from_row(row, "z_imu"))
            z_lidar = base.pose_inverse(e_transform) @ z_imu @ e_transform
            z_beta = s_transform @ z_lidar @ base.pose_inverse(s_transform)
            residual = base.se3_log(base.pose_inverse(z_gt) @ z_beta)
            output = {
                "selection_group": group,
                "selection_index": group_index,
                "from_pose_index": start_index,
                "to_pose_index": end_index,
                "from_stamp_sec": row["from_stamp_sec"],
                "to_stamp_sec": row["to_stamp_sec"],
                "edge_duration_sec": row["edge_duration_sec"],
                "gt_start_pose_tum": pose_token(gt_start),
                "gt_end_pose_tum": pose_token(gt_end),
                "gt_start_left_timestamp": start_info.get("left_timestamp", ""),
                "gt_start_right_timestamp": start_info.get("right_timestamp", ""),
                "gt_start_left_gap_sec": start_info.get("left_gap_sec", ""),
                "gt_start_right_gap_sec": start_info.get("right_gap_sec", ""),
                "gt_start_bracket_sec": start_info.get("total_bracket_sec", ""),
                "gt_end_left_timestamp": end_info.get("left_timestamp", ""),
                "gt_end_right_timestamp": end_info.get("right_timestamp", ""),
                "gt_end_left_gap_sec": end_info.get("left_gap_sec", ""),
                "gt_end_right_gap_sec": end_info.get("right_gap_sec", ""),
                "gt_end_bracket_sec": end_info.get("total_bracket_sec", ""),
                "z_gt_matrix": matrix_token(z_gt),
                "z_imu_matrix": matrix_token(z_imu),
                "z_lidar_matrix": matrix_token(z_lidar),
                "z_beta_matrix": matrix_token(z_beta),
                "residual_rx_ry_rz_tx_ty_tz": ";".join(
                    format(float(value), ".17g") for value in residual),
                "rotation_error_rad": float(np.linalg.norm(residual[:3])),
                "translation_error_m": float(np.linalg.norm(residual[3:])),
                "translation_vector_cosine": row["translation_vector_cosine"],
                "worst_translation_condition_ratio": row.get(
                    "worst_translational_condition_ratio", ""),
            }
            for label, index in (("from", start_index), ("to", end_index)):
                if index >= len(glim_poses):
                    output[f"glim_{label}_pose_available"] = False
                    continue
                world_imu = glim_poses[index]["transform"]
                world_lidar = world_imu @ e_transform
                world_beta = world_lidar @ base.pose_inverse(s_transform)
                output[f"glim_{label}_pose_available"] = True
                output[f"glim_{label}_world_imu_tum"] = pose_token(world_imu)
                output[f"glim_{label}_world_lidar_tum"] = pose_token(world_lidar)
                output[f"glim_{label}_world_beta_tum"] = pose_token(world_beta)
            result.append(output)
    return result


def rate_series(stamps, transforms):
    stamps = np.asarray(stamps, dtype=float)
    midpoint, translation, rotation = [], [], []
    for index in range(len(stamps) - 1):
        duration = stamps[index + 1] - stamps[index]
        if duration <= 0.0:
            continue
        relative = base.pose_between(transforms[index], transforms[index + 1])
        midpoint.append(0.5 * (stamps[index] + stamps[index + 1]))
        translation.append(float(np.linalg.norm(relative[:3, 3])) / duration)
        rotation.append(float(np.linalg.norm(base.so3_log(relative[:3, :3]))) / duration)
    return np.asarray(midpoint), np.asarray(translation), np.asarray(rotation)


def pearson(first, second):
    if len(first) < 3 or np.std(first) == 0.0 or np.std(second) == 0.0:
        return None
    return float(np.corrcoef(first, second)[0, 1])


def timing_curve(glim_stamps, glim_transforms, gt_records, plan):
    gt_stamps = [float(record.stamp) for record in gt_records]
    gt_transforms = [record.transform for record in gt_records]
    glim_mid, glim_speed, glim_angular = rate_series(glim_stamps, glim_transforms)
    gt_mid, gt_speed, gt_angular = rate_series(gt_stamps, gt_transforms)
    timing = plan["timing_diagnostic"]
    offsets = np.arange(float(timing["offset_min_sec"]),
                        float(timing["offset_max_sec"]) + 0.5 * float(
                            timing["offset_step_sec"]),
                        float(timing["offset_step_sec"]))
    rows = []
    for offset in offsets:
        start = max(float(glim_mid[0]), float(gt_mid[0]) - offset)
        end = min(float(glim_mid[-1]), float(gt_mid[-1]) - offset)
        grid = np.arange(start, end, float(timing["common_grid_step_sec"]))
        if grid.size < int(timing["minimum_overlap_samples"]):
            continue
        g_speed = np.interp(grid, glim_mid, glim_speed)
        t_speed = np.interp(grid + offset, gt_mid, gt_speed)
        g_angular = np.interp(grid, glim_mid, glim_angular)
        t_angular = np.interp(grid + offset, gt_mid, gt_angular)
        rows.append({
            "offset_sec": float(offset),
            "sample_count": int(grid.size),
            "translation_speed_pearson": pearson(g_speed, t_speed),
            "angular_speed_pearson": pearson(g_angular, t_angular),
        })
    def best(field):
        valid = [row for row in rows if finite(row[field])]
        return max(valid, key=lambda row: row[field]) if valid else None
    zero = min(rows, key=lambda row: abs(row["offset_sec"])) if rows else None
    return rows, {
        "offset_convention": timing["shift_convention"],
        "primary_offset_sec": 0.0,
        "zero_offset": zero,
        "best_translation_speed": best("translation_speed_pearson"),
        "best_angular_speed": best("angular_speed_pearson"),
        "diagnostic_only": True,
        "offset_was_applied_to_primary_evaluator": False,
    }


def pose_to_tum_line(stamp, transform):
    translation = transform[:3, 3]
    quaternion = base.matrix_to_quaternion_xyzw(transform[:3, :3])
    values = [float(stamp), *translation, *quaternion]
    return " ".join(format(float(value), ".17g") for value in values)


def export_trajectories(output_dir, glim_poses, gt_records, e_transform,
                        s_transform):
    output_dir = Path(output_dir)
    paths = {
        "glim_imu": output_dir / "glim_imu_tum.txt",
        "glim_lidar_official_input": output_dir / "glim_gamma_lidar_tum.txt",
        "glim_beta_local": output_dir / "glim_beta_gt_body_local_tum.txt",
        "ground_truth": output_dir / "ground_truth_normalized_tum.txt",
    }
    imu_lines, lidar_lines, beta_lines = [], [], []
    for row in glim_poses:
        world_imu = row["transform"]
        world_lidar = world_imu @ e_transform
        world_beta = world_lidar @ base.pose_inverse(s_transform)
        stamp = float(row["stamp"])
        imu_lines.append(pose_to_tum_line(stamp, world_imu))
        lidar_lines.append(pose_to_tum_line(stamp, world_lidar))
        beta_lines.append(pose_to_tum_line(stamp, world_beta))
    gt_lines = [pose_to_tum_line(float(row.stamp), row.transform)
                for row in gt_records]
    for path, lines in zip(paths.values(),
                           (imu_lines, lidar_lines, beta_lines, gt_lines)):
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return {name: {"path": str(path), "row_count": len(path.read_text(
        encoding="utf-8").splitlines()), "sha256": sha256(path)}
            for name, path in paths.items()}


def compare_tum_trajectories(first_path, second_path):
    def load(path):
        result = []
        for line in Path(path).read_text(encoding="utf-8").splitlines():
            fields = line.split()
            if len(fields) < 8:
                continue
            result.append((float(fields[0]), base.pose_from_tum(fields[1:8])))
        return result
    first, second = load(first_path), load(second_path)
    if len(first) != len(second):
        return {"valid": False, "reason": "row_count_mismatch",
                "first_count": len(first), "second_count": len(second)}
    timestamp, translation, rotation = [], [], []
    for (first_stamp, first_pose), (second_stamp, second_pose) in zip(first, second):
        timestamp.append(abs(first_stamp - second_stamp))
        difference = base.se3_log(base.pose_inverse(first_pose) @ second_pose)
        rotation.append(float(np.linalg.norm(difference[:3])))
        translation.append(float(np.linalg.norm(difference[3:])))
    return {
        "valid": True,
        "row_count": len(first),
        "maximum_timestamp_difference_sec": max(timestamp, default=0.0),
        "translation_difference_m": numeric_summary(translation),
        "rotation_difference_rad": numeric_summary(rotation),
        "note": "official script formats every field to six decimals",
    }


def official_formula_crosscheck(glim_poses, e_transform, s_transform):
    errors = []
    step = max(1, len(glim_poses) // 100)
    for row in glim_poses[::step]:
        world_imu = row["transform"]
        world_lidar = world_imu @ e_transform
        local = base.imu_pose_to_beta(world_imu, e_transform, s_transform)
        official = world_lidar @ np.linalg.inv(s_transform)
        errors.append(float(np.linalg.norm(base.se3_log(
            base.pose_inverse(official) @ local))))
    return {
        "sample_count": len(errors),
        "maximum_se3_log_norm": max(errors) if errors else None,
        "median_se3_log_norm": float(np.median(errors)) if errors else None,
        "formula": "official T_eval=T_device@inverse(S)",
    }


def simple_scatter_svg(path, rows, x_field, y_field, title):
    pairs = [(float(row[x_field]), float(row[y_field])) for row in rows
             if finite(row.get(x_field)) and finite(row.get(y_field))]
    if not pairs:
        return
    width, height, margin = 900, 500, 55
    x, y = np.asarray(pairs, dtype=float).T
    x_min, x_max = float(np.min(x)), float(np.max(x))
    y_min, y_max = float(np.min(y)), float(np.max(y))
    if x_min == x_max:
        x_max = x_min + 1.0
    if y_min == y_max:
        y_max = y_min + 1.0
    def point(a, b):
        px = margin + (a - x_min) / (x_max - x_min) * (width - 2 * margin)
        py = height - margin - (b - y_min) / (y_max - y_min) * (height - 2 * margin)
        return px, py
    circles = "\n".join(
        f'<circle cx="{point(a, b)[0]:.3f}" cy="{point(a, b)[1]:.3f}" '
        'r="1.2" fill="#2563eb" fill-opacity="0.35" />' for a, b in pairs)
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">
<rect width="100%" height="100%" fill="white"/>
<text x="{width/2}" y="25" text-anchor="middle" font-family="sans-serif" font-size="16">{title}</text>
<line x1="{margin}" y1="{height-margin}" x2="{width-margin}" y2="{height-margin}" stroke="black"/>
<line x1="{margin}" y1="{margin}" x2="{margin}" y2="{height-margin}" stroke="black"/>
{circles}
</svg>\n'''
    Path(path).write_text(svg, encoding="utf-8")


def simple_histogram_svg(path, rows, field, title, bin_count=40):
    values = np.asarray([float(row[field]) for row in rows
                         if finite(row.get(field))], dtype=float)
    if not values.size:
        return
    counts, boundaries = np.histogram(values, bins=bin_count)
    width, height, margin = 900, 500, 55
    plot_width = width - 2 * margin
    plot_height = height - 2 * margin
    maximum = max(1, int(np.max(counts)))
    bar_width = plot_width / len(counts)
    bars = []
    for index, count in enumerate(counts):
        bar_height = float(count) / maximum * plot_height
        bars.append(
            f'<rect x="{margin + index * bar_width:.3f}" '
            f'y="{height - margin - bar_height:.3f}" '
            f'width="{bar_width:.3f}" height="{bar_height:.3f}" '
            'fill="#2563eb" fill-opacity="0.65" />')
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">
<rect width="100%" height="100%" fill="white"/>
<text x="{width/2}" y="25" text-anchor="middle" font-family="sans-serif" font-size="16">{title}</text>
<text x="{width/2}" y="{height-12}" text-anchor="middle" font-family="sans-serif" font-size="12">range [{boundaries[0]:.6g}, {boundaries[-1]:.6g}]</text>
<line x1="{margin}" y1="{height-margin}" x2="{width-margin}" y2="{height-margin}" stroke="black"/>
<line x1="{margin}" y1="{margin}" x2="{margin}" y2="{height-margin}" stroke="black"/>
{chr(10).join(bars)}
</svg>\n'''
    Path(path).write_text(svg, encoding="utf-8")


def tunnel_reference_contract(screening, plan):
    contract = plan["tunnel4_reference_screen"]
    return bool(
        screening.get("reference_naturally_ready", False)
        and int(screening.get("accepted_candidate_count", 0)) >= int(
            contract["minimum_accepted_samples"])
        and float(screening.get("accepted_span_sec", 0.0)) >= float(
            contract["minimum_accepted_span_sec"])
        and float(screening.get("maximum_accepted_gap_sec", math.inf)) <= float(
            contract["maximum_accepted_gap_sec"])
        and float(screening.get("acceptance_fraction", 0.0)) >= float(
            contract["minimum_acceptance_fraction"]))


def run(args):
    plan = json.loads(args.plan.read_text(encoding="utf-8"))
    profile = json.loads(args.profile.read_text(encoding="utf-8"))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    edge_rows = read_csv(args.edge_csv)
    e_transform, s_transform = base.official_transforms(profile)
    lever_arm = float(np.linalg.norm(e_transform[:3, 3])
                      + np.linalg.norm(s_transform[:3, 3]))
    gt_records, gt_stats = base.load_ground_truth(args.ground_truth)
    maximum_bracket = Decimal(str(
        plan["primary"]["gt_interpolation_maximum_bracket_sec"]))
    rows = [add_gt_rotation_transpose_diagnostic(
        diagnostic_edge(
            row, float(plan["primary"]["displacement_ratio_epsilon_m"]),
            lever_arm), gt_records, maximum_bracket)
        for row in edge_rows]
    write_csv(args.output_dir / "edge_sanity_audit.csv", rows)
    glim_poses = base.read_glim_poses(args.glim_csv)
    manual = detailed_manual_rows(
        manual_selections(rows, plan), glim_poses, gt_records,
        e_transform, s_transform,
        Decimal(str(plan["primary"]["gt_interpolation_maximum_bracket_sec"])))
    write_csv(args.output_dir / "manual_edge_audit.csv", manual)
    glim_stamps = [float(row["stamp"]) for row in glim_poses]
    glim_beta = [base.imu_pose_to_beta(row["transform"], e_transform, s_transform)
                 for row in glim_poses]
    timing_rows, timing_summary = timing_curve(
        glim_stamps, glim_beta, gt_records, plan)
    write_csv(args.output_dir / "timing_cross_correlation.csv", timing_rows)
    trajectories = export_trajectories(
        args.output_dir, glim_poses, gt_records, e_transform, s_transform)
    for x_field, y_field, name, title in (
        ("gt_displacement_m", "glim_displacement_m",
         "glim_vs_gt_displacement.svg", "GLIM versus GT displacement"),
        ("gt_displacement_m", "A_primary_translation_error_m",
         "translation_error_vs_gt_displacement.svg", "Translation residual versus GT displacement"),
        ("edge_duration_sec", "A_primary_translation_error_m",
         "translation_error_vs_duration.svg", "Translation residual versus edge duration"),
        ("gt_speed_mps", "A_primary_translation_error_m",
         "translation_error_vs_speed.svg", "Translation residual versus GT speed"),
        ("gt_rotation_magnitude_rad", "A_primary_translation_error_m",
         "translation_error_vs_rotation.svg", "Translation residual versus edge rotation"),
    ):
        simple_scatter_svg(args.output_dir / name, rows, x_field, y_field, title)
    simple_histogram_svg(
        args.output_dir / "translation_vector_cosine_histogram.svg", rows,
        "translation_vector_cosine", "GT/GLIM relative-translation cosine")
    alternatives = alternative_summaries(
        rows, float(plan["edge_direction_diagnostics"][
            "orders_of_magnitude_reduction_factor"]))
    summary = {
        "schema_version": 1,
        "dataset": args.dataset,
        "plan_sha256": sha256(args.plan),
        "edge_csv_sha256": sha256(args.edge_csv),
        "glim_csv_sha256": sha256(args.glim_csv),
        "ground_truth_sha256": sha256(args.ground_truth),
        "edge_count": len(rows),
        "ground_truth": gt_stats,
        "edge_duration_sec": numeric_summary(row["edge_duration_sec"] for row in rows),
        "gt_displacement_m": numeric_summary(row["gt_displacement_m"] for row in rows),
        "glim_displacement_m": numeric_summary(row["glim_displacement_m"] for row in rows),
        "translation_error_m": numeric_summary(
            row["A_primary_translation_error_m"] for row in rows),
        "rotation_error_rad": numeric_summary(
            row["A_primary_rotation_error_rad"] for row in rows),
        "translation_residual_ratio": numeric_summary(
            row["translation_residual_ratio"] for row in rows),
        "translation_vector_cosine": numeric_summary(
            row["translation_vector_cosine"] for row in rows),
        "gt_rotation_magnitude_rad": numeric_summary(
            row["gt_rotation_magnitude_rad"] for row in rows),
        "glim_rotation_magnitude_rad": numeric_summary(
            row["glim_rotation_magnitude_rad"] for row in rows),
        "gt_speed_mps": numeric_summary(row["gt_speed_mps"] for row in rows),
        "lever_arm_translation_bound_m": numeric_summary(
            row["lever_arm_translation_bound_m"] for row in rows),
        "rank_diagnostics": {
            "translation_error_vs_gt_displacement": spearman(
                (row["A_primary_translation_error_m"] for row in rows),
                (row["gt_displacement_m"] for row in rows)),
            "translation_error_vs_edge_duration": spearman(
                (row["A_primary_translation_error_m"] for row in rows),
                (row["edge_duration_sec"] for row in rows)),
            "translation_error_vs_gt_speed": spearman(
                (row["A_primary_translation_error_m"] for row in rows),
                (row["gt_speed_mps"] for row in rows)),
            "translation_error_vs_gt_rotation": spearman(
                (row["A_primary_translation_error_m"] for row in rows),
                (row["gt_rotation_magnitude_rad"] for row in rows)),
        },
        "edge_direction_diagnostics": alternatives,
        "gt_rotation_transpose_diagnostic_not_primary": {
            "translation_error_m": numeric_summary(
                row["F_transpose_gt_rotation_translation_error_m"] for row in rows),
            "rotation_error_rad": numeric_summary(
                row["F_transpose_gt_rotation_rotation_error_rad"] for row in rows),
            "primary_was_replaced": False,
            "interpretation": (
                "tests world-position plus transposed-quaternion rotation; "
                "not an approved replacement for released TUM convention"),
        },
        "official_formula_crosscheck": official_formula_crosscheck(
            glim_poses, e_transform, s_transform),
        "timing_diagnostic": timing_summary,
        "manual_edge_audit_row_count": len(manual),
        "trajectories": trajectories,
        "primary_formula_changed": False,
        "estimator_or_reference_output_written": False,
    }
    if args.official_converted_tum:
        summary["official_script_output_comparison"] = compare_tum_trajectories(
            args.output_dir / "glim_beta_gt_body_local_tum.txt",
            args.official_converted_tum)
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False))
    return summary


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--glim-csv", type=Path, required=True)
    parser.add_argument("--edge-csv", type=Path, required=True)
    parser.add_argument("--official-converted-tum", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv=None):
    run(parse_args(argv))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
