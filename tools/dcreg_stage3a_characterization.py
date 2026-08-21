#!/usr/bin/env python3
"""Offline Stage 3A reference-free DCReg characterization.

This tool consumes frozen synthetic and GEODE artifacts. It never imports ROS,
publishes beliefs, or changes an estimator. NumPy is the only non-stdlib
dependency and is available in the accepted project container.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
from collections import Counter
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple
from xml.sax.saxutils import escape

import numpy as np


EPS_ABSOLUTE = 1.0e-12
EPS_RELATIVE = 1.0e-9
PINV_RELATIVE = 1.0e-8
DEFAULT_THRESHOLD = 10.0


def signed_margin(threshold: float, kappa: float) -> float:
    """Plot-only boundary margin; it has no estimator or CBS side effects."""
    return math.log(threshold / kappa)


def spectral_cluster_flags(eigenvalues: Sequence[float], relative_gap: float = 0.05) -> List[bool]:
    flags = [False, False, False]
    for first in range(3):
        for second in range(first + 1, 3):
            denominator = max(EPS_ABSOLUTE, abs(eigenvalues[first]), abs(eigenvalues[second]))
            if abs(eigenvalues[first] - eigenvalues[second]) / denominator <= relative_gap:
                flags[first] = True
                flags[second] = True
    return flags


def finite_float(value: object) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return float("nan")
    return number if math.isfinite(number) else float("nan")


def read_csv(path: Path) -> List[Dict[str, str]]:
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def percentile(values: Sequence[float], q: float) -> float:
    array = np.asarray([value for value in values if math.isfinite(value)], dtype=float)
    if not len(array):
        return float("nan")
    return float(np.percentile(array, q, interpolation="linear"))


def summarize(values: Sequence[float]) -> Dict[str, float]:
    clean = np.asarray([value for value in values if math.isfinite(value)], dtype=float)
    if not len(clean):
        return {key: float("nan") for key in ("count", "minimum", "p10", "median", "p90", "p95", "maximum")}
    return {
        "count": int(len(clean)),
        "minimum": float(np.min(clean)),
        "p10": percentile(clean, 10.0),
        "median": percentile(clean, 50.0),
        "p90": percentile(clean, 90.0),
        "p95": percentile(clean, 95.0),
        "maximum": float(np.max(clean)),
    }


def average_ranks(values: np.ndarray) -> np.ndarray:
    order = np.argsort(values, kind="mergesort")
    ranks = np.empty(len(values), dtype=float)
    cursor = 0
    while cursor < len(order):
        end = cursor + 1
        while end < len(order) and values[order[end]] == values[order[cursor]]:
            end += 1
        ranks[order[cursor:end]] = 0.5 * (cursor + end - 1) + 1.0
        cursor = end
    return ranks


def spearman(first: Sequence[float], second: Sequence[float]) -> float:
    pairs = [(x, y) for x, y in zip(first, second) if math.isfinite(x) and math.isfinite(y)]
    if len(pairs) < 3:
        return float("nan")
    x = np.asarray([pair[0] for pair in pairs], dtype=float)
    y = np.asarray([pair[1] for pair in pairs], dtype=float)
    rx = average_ranks(x)
    ry = average_ranks(y)
    if np.std(rx) == 0.0 or np.std(ry) == 0.0:
        return float("nan")
    return float(np.corrcoef(rx, ry)[0, 1])


def symmetric_pseudoinverse(matrix: np.ndarray) -> Tuple[np.ndarray, int]:
    symmetric = 0.5 * (matrix + matrix.T)
    eigenvalues, eigenvectors = np.linalg.eigh(symmetric)
    threshold = max(EPS_ABSOLUTE, PINV_RELATIVE * float(np.max(np.abs(eigenvalues))))
    inverse_values = np.asarray([1.0 / value if value > threshold else 0.0 for value in eigenvalues])
    return eigenvectors @ np.diag(inverse_values) @ eigenvectors.T, int(np.sum(eigenvalues > threshold))


def condition_ratios(eigenvalues: np.ndarray) -> np.ndarray:
    maximum = max(0.0, float(np.max(eigenvalues)))
    if maximum <= EPS_ABSOLUTE:
        return np.ones(3)
    floor = EPS_ABSOLUTE + EPS_RELATIVE * maximum
    return np.asarray([maximum / max(float(value), floor) for value in eigenvalues])


def numpy_schur_analysis(hessian: np.ndarray) -> Dict[str, object]:
    hessian = 0.5 * (hessian + hessian.T)
    rr = hessian[:3, :3]
    rt = hessian[:3, 3:]
    tr = hessian[3:, :3]
    tt = hessian[3:, 3:]
    tt_inverse, tt_rank = symmetric_pseudoinverse(tt)
    rr_inverse, rr_rank = symmetric_pseudoinverse(rr)
    rotation_schur = 0.5 * (rr - rt @ tt_inverse @ tr + (rr - rt @ tt_inverse @ tr).T)
    translation_schur = 0.5 * (tt - tr @ rr_inverse @ rt + (tt - tr @ rr_inverse @ rt).T)
    rotation_values, rotation_vectors = np.linalg.eigh(rotation_schur)
    translation_values, translation_vectors = np.linalg.eigh(translation_schur)
    rotation_ratios = condition_ratios(rotation_values)
    translation_ratios = condition_ratios(translation_values)
    full_values = np.linalg.eigvalsh(hessian)
    full_floor = max(EPS_ABSOLUTE, PINV_RELATIVE * float(np.max(np.abs(full_values))))
    return {
        "rotation_schur": rotation_schur,
        "translation_schur": translation_schur,
        "rotation_eigenvalues": rotation_values,
        "translation_eigenvalues": translation_values,
        "rotation_eigenvectors": rotation_vectors,
        "translation_eigenvectors": translation_vectors,
        "rotation_condition_ratios": rotation_ratios,
        "translation_condition_ratios": translation_ratios,
        "rotation_block_rank": rr_rank,
        "translation_block_rank": tt_rank,
        "full_eigenvalues": full_values,
        "full_rank": int(np.sum(full_values > full_floor)),
    }


def projector_overlap(expected: np.ndarray, detected: np.ndarray) -> float:
    dimension = int(round(float(np.trace(expected))))
    if dimension == 0:
        return float("nan")
    return float(np.trace(expected @ detected) / dimension)


def expected_projectors(scene_name: str) -> Tuple[np.ndarray, np.ndarray]:
    axes = np.eye(3)
    zero = np.zeros((3, 3))
    if scene_name in {"single_plane", "dense_degenerate_plane"}:
        return np.outer(axes[2], axes[2]), np.diag([1.0, 1.0, 0.0])
    if scene_name == "parallel_planes_corridor":
        return np.outer(axes[0], axes[0]), np.diag([0.0, 1.0, 1.0])
    if scene_name == "tunnel":
        return np.outer(axes[0], axes[0]), np.outer(axes[0], axes[0])
    if scene_name == "perpendicular_corner":
        return zero, np.outer(axes[2], axes[2])
    if scene_name == "tilted_offset_plane":
        normal = np.asarray([0.4, -0.3, 0.8660254], dtype=float)
        normal /= np.linalg.norm(normal)
        return np.outer(normal, normal), np.eye(3) - np.outer(normal, normal)
    return zero, zero


def synthetic_analysis(payload: Mapping[str, object]) -> Tuple[List[Dict[str, object]], Dict[str, object]]:
    rows: List[Dict[str, object]] = []
    maximum_crosscheck_error = 0.0
    maximum_crosscheck_relative_error = 0.0
    for scene in payload["scenes"]:  # type: ignore[index]
        hessian = np.asarray(scene["hessian"], dtype=float)
        analysis = numpy_schur_analysis(hessian)
        rotation_values = np.asarray(scene["rotation_schur_eigenvalues"], dtype=float)
        translation_values = np.asarray(scene["translation_schur_eigenvalues"], dtype=float)
        rotation_ratios = np.asarray(analysis["rotation_condition_ratios"], dtype=float)
        translation_ratios = np.asarray(analysis["translation_condition_ratios"], dtype=float)
        crosscheck_error = max(
            float(np.max(np.abs(rotation_values - analysis["rotation_eigenvalues"]))),
            float(np.max(np.abs(translation_values - analysis["translation_eigenvalues"]))),
            abs(float(scene["kappa_rotation"]) - float(np.max(rotation_ratios))),
            abs(float(scene["kappa_translation"]) - float(np.max(translation_ratios))),
        )
        eigenvalue_relative_errors = []
        for frozen, independent in (
            (rotation_values, np.asarray(analysis["rotation_eigenvalues"], dtype=float)),
            (translation_values, np.asarray(analysis["translation_eigenvalues"], dtype=float)),
        ):
            eigenvalue_relative_errors.extend(
                np.abs(frozen - independent) /
                np.maximum(1.0, np.maximum(np.abs(frozen), np.abs(independent)))
            )
        crosscheck_relative_error = max(
            [float(value) for value in eigenvalue_relative_errors]
            + [
                abs(float(scene["kappa_rotation"]) - float(np.max(rotation_ratios))) /
                max(1.0, abs(float(scene["kappa_rotation"]))),
                abs(float(scene["kappa_translation"]) - float(np.max(translation_ratios))) /
                max(1.0, abs(float(scene["kappa_translation"]))),
            ]
        )
        maximum_crosscheck_error = max(maximum_crosscheck_error, crosscheck_error)
        maximum_crosscheck_relative_error = max(maximum_crosscheck_relative_error, crosscheck_relative_error)
        expected_rotation, expected_translation = expected_projectors(str(scene["name"]))
        rotation_dimension = int(round(float(np.trace(expected_rotation))))
        translation_dimension = int(round(float(np.trace(expected_translation))))
        rotation_vectors = np.asarray(analysis["rotation_eigenvectors"], dtype=float)
        translation_vectors = np.asarray(analysis["translation_eigenvectors"], dtype=float)
        detected_rotation = rotation_vectors[:, :rotation_dimension] @ rotation_vectors[:, :rotation_dimension].T if rotation_dimension else np.zeros((3, 3))
        detected_translation = translation_vectors[:, :translation_dimension] @ translation_vectors[:, :translation_dimension].T if translation_dimension else np.zeros((3, 3))
        full_values = np.asarray(analysis["full_eigenvalues"], dtype=float)
        block_rotation = np.linalg.eigvalsh(hessian[:3, :3])
        block_translation = np.linalg.eigvalsh(hessian[3:, 3:])
        row = {
            "scene": scene["name"],
            "geometry_class": scene["geometry_class"],
            "source_points": scene["source_point_count"],
            "minimum_inliers": scene["minimum_inlier_count"],
            "minimum_inlier_fraction": scene["minimum_inlier_fraction"],
            "initial_cost": scene["initial_cost"],
            "final_cost": scene["final_cost"],
            "kappa_rotation": scene["kappa_rotation"],
            "kappa_translation": scene["kappa_translation"],
            "rotation_mask_tau10": int(float(scene["kappa_rotation"]) > 10.0),
            "translation_mask_tau10": int(float(scene["kappa_translation"]) > 10.0),
            "rotation_weak_projector_overlap": projector_overlap(expected_rotation, detected_rotation),
            "translation_weak_projector_overlap": projector_overlap(expected_translation, detected_translation),
            "full_hessian_condition": scene["full_hessian_condition"],
            "full_hessian_minimum_eigenvalue": scene["full_hessian_minimum_eigenvalue"],
            "full_hessian_rank": analysis["full_rank"],
            "rotation_block_condition": float(np.max(block_rotation) / max(float(np.min(block_rotation)), EPS_ABSOLUTE + EPS_RELATIVE * float(np.max(block_rotation)))),
            "translation_block_condition": float(np.max(block_translation) / max(float(np.min(block_translation)), EPS_ABSOLUTE + EPS_RELATIVE * float(np.max(block_translation)))),
            "rotation_block_minimum_eigenvalue": float(np.min(block_rotation)),
            "translation_block_minimum_eigenvalue": float(np.min(block_translation)),
            "python_frozen_crosscheck_max_abs": crosscheck_error,
            "python_frozen_crosscheck_max_relative": crosscheck_relative_error,
        }
        for threshold in (5.0, 10.0, 20.0):
            row[f"rotation_mask_tau{int(threshold)}"] = int(float(scene["kappa_rotation"]) > threshold)
            row[f"translation_mask_tau{int(threshold)}"] = int(float(scene["kappa_translation"]) > threshold)
            row[f"rotation_margin_tau{int(threshold)}"] = signed_margin(threshold, float(scene["kappa_rotation"]))
            row[f"translation_margin_tau{int(threshold)}"] = signed_margin(threshold, float(scene["kappa_translation"]))
        rows.append(row)
    return rows, {
        "scene_count": len(rows),
        "frozen_cpp_vs_numpy_max_absolute_difference": maximum_crosscheck_error,
        "frozen_cpp_vs_numpy_max_relative_difference": maximum_crosscheck_relative_error,
        "frozen_cpp_vs_numpy_relative_tolerance": 1.0e-10,
        "crosscheck_pass": maximum_crosscheck_relative_error <= 1.0e-10,
    }


def parse_scan_health_log(path: Path) -> Dict[int, Dict[str, float]]:
    marker = "GLIM_SCAN_HEALTH_ROW,"
    rows: Dict[int, Dict[str, float]] = {}
    with path.open(errors="replace") as stream:
        for line in stream:
            location = line.find(marker)
            if location < 0:
                continue
            values = next(csv.reader([line[location:]]))
            if len(values) < 28:
                continue
            frame = int(values[2])
            rows[frame] = {
                "stamp": finite_float(values[1]),
                "preprocessed_points": finite_float(values[4]),
                "initial_cost": finite_float(values[6]),
                "final_cost": finite_float(values[7]),
                "cost_ratio": finite_float(values[8]),
                "lm_iterations": finite_float(values[9]),
                "solve_success": finite_float(values[14]),
                "imu_pred_translation_m": finite_float(values[17]),
                "imu_pred_rotation_deg": finite_float(values[18]),
                "scan_translation_m": finite_float(values[19]),
                "scan_rotation_deg": finite_float(values[20]),
                "scan_minus_imu_translation_m": finite_float(values[21]),
                "scan_minus_imu_rotation_deg": finite_float(values[22]),
            }
    return rows


def merge_dcreg_health_csv(path: Path, rows: Dict[int, Dict[str, float]]) -> None:
    with path.open(newline="") as stream:
        for record in csv.DictReader(stream):
            if record.get("record_kind") != "aggregate":
                continue
            frame = int(record["frame_id"])
            target = rows.setdefault(frame, {})
            target.update({
                "dcreg_valid": finite_float(record.get("valid")),
                "registration_converged": finite_float(record.get("registration_converged")),
                "dcreg_linear_solve_success": finite_float(record.get("linear_solve_success")),
            })


def edge_scan_summary(row: Mapping[str, str], scan_rows: Mapping[int, Mapping[str, float]]) -> Dict[str, float]:
    start = int(row["from_pose_index"])
    end = int(row["to_pose_index"])
    selected = [scan_rows[key] for key in sorted(scan_rows) if start < key <= end]
    output: Dict[str, float] = {"scan_diagnostic_count": float(len(selected))}
    for field in (
        "lm_iterations",
        "solve_success",
        "cost_ratio",
        "scan_minus_imu_translation_m",
        "scan_minus_imu_rotation_deg",
    ):
        values = [finite_float(sample.get(field)) for sample in selected if math.isfinite(finite_float(sample.get(field)))]
        output[field] = percentile(values, 50.0)
    for field in ("dcreg_valid", "registration_converged", "dcreg_linear_solve_success"):
        values = [finite_float(sample.get(field)) for sample in selected if math.isfinite(finite_float(sample.get(field)))]
        output[field + "_fraction"] = float(sum(values) / len(values)) if values else float("nan")
    return output


def longest_true_run(values: Sequence[bool]) -> int:
    longest = 0
    current = 0
    for value in values:
        current = current + 1 if value else 0
        longest = max(longest, current)
    return longest


def count_transitions(values: Sequence[bool]) -> int:
    return sum(first != second for first, second in zip(values, values[1:]))


def longest_run_duration(states: Sequence[Optional[bool]], timestamps: Sequence[float]) -> float:
    longest = 0.0
    start: Optional[float] = None
    last: Optional[float] = None
    for state, timestamp in zip(states, timestamps):
        if state is True and math.isfinite(timestamp):
            if start is None:
                start = timestamp
            last = timestamp
            longest = max(longest, max(0.0, last - start))
        else:
            start = None
            last = None
    return longest


def condition_bin(value: float) -> str:
    if value < 5.0:
        return "below_5"
    if value < 10.0:
        return "5_to_10"
    if value < 20.0:
        return "10_to_20"
    if value < 100.0:
        return "20_to_100"
    return "100_or_more"


def support_stratum(value: float) -> str:
    if value < 0.25:
        return "low"
    if value < 0.50:
        return "moderate"
    return "high"


def sequence_analysis(
    name: str,
    edge_rows: List[Dict[str, str]],
    sanity_rows: List[Dict[str, str]],
    scan_rows: Mapping[int, Mapping[str, float]],
) -> Tuple[Dict[str, object], List[Dict[str, object]], List[Dict[str, object]]]:
    sanity_by_edge = {
        (row["from_pose_index"], row["to_pose_index"]): row for row in sanity_rows
    }
    enriched: List[Dict[str, object]] = []
    for row in sorted(edge_rows, key=lambda item: (finite_float(item["to_stamp_sec"]), int(item["to_pose_index"]))):
        rotation_kappa = finite_float(row["stage2c_c_max_rotation_condition_ratio"])
        translation_kappa = finite_float(row["stage2c_c_max_translation_condition_ratio"])
        inlier = finite_float(row["aggregate_inlier_fraction_median"])
        sanity = sanity_by_edge.get((row["from_pose_index"], row["to_pose_index"]), {})
        scan = edge_scan_summary(row, scan_rows)
        item: Dict[str, object] = {
            "dataset": name,
            "from_pose_index": int(row["from_pose_index"]),
            "to_pose_index": int(row["to_pose_index"]),
            "from_stamp_sec": finite_float(row["from_stamp_sec"]),
            "to_stamp_sec": finite_float(row["to_stamp_sec"]),
            "edge_duration_sec": finite_float(row["edge_duration_sec"]),
            "rotation_error_rad": finite_float(row["rotation_error_rad"]),
            "translation_error_m": finite_float(row["translation_error_m"]),
            "kappa_rotation": rotation_kappa,
            "kappa_translation": translation_kappa,
            "log_kappa_rotation": math.log(rotation_kappa) if rotation_kappa > 0 else float("nan"),
            "log_kappa_translation": math.log(translation_kappa) if translation_kappa > 0 else float("nan"),
            "rotation_condition_bin": condition_bin(rotation_kappa) if math.isfinite(rotation_kappa) else "unavailable",
            "translation_condition_bin": condition_bin(translation_kappa) if math.isfinite(translation_kappa) else "unavailable",
            "source_points": finite_float(row["aggregate_source_point_count_median"]),
            "inlier_fraction": inlier,
            "support_stratum": support_stratum(inlier) if math.isfinite(inlier) else "unavailable",
            "initial_cost": finite_float(row["aggregate_initial_cost_median"]),
            "final_cost": finite_float(row["aggregate_final_cost_median"]),
            "translation_vector_cosine": finite_float(sanity.get("translation_vector_cosine")),
            "gt_displacement_m": finite_float(sanity.get("gt_displacement_m")),
            "glim_displacement_m": finite_float(sanity.get("glim_displacement_m")),
            "gt_translation_x_m": finite_float(row["z_gt_tx"]),
            "gt_translation_y_m": finite_float(row["z_gt_ty"]),
            "gt_translation_z_m": finite_float(row["z_gt_tz"]),
            "glim_translation_x_m": finite_float(row["z_beta_tx"]),
            "glim_translation_y_m": finite_float(row["z_beta_ty"]),
            "glim_translation_z_m": finite_float(row["z_beta_tz"]),
            "rotation_absolute_degenerate_fraction": finite_float(row["stage2c_c_rotation_absolute_degenerate_fraction"]),
            "translation_absolute_degenerate_fraction": finite_float(row["stage2c_c_translation_absolute_degenerate_fraction"]),
            "clustered_rotation_fraction": finite_float(row["clustered_rotation_fraction"]),
            "clustered_translation_fraction": finite_float(row["clustered_translation_fraction"]),
            "low_rotation_alignment_fraction": finite_float(row["low_rotation_alignment_fraction"]),
            "low_translation_alignment_fraction": finite_float(row["low_translation_alignment_fraction"]),
        }
        item.update(scan)
        enriched.append(item)

    def values(field: str) -> List[float]:
        return [finite_float(row[field]) for row in enriched]

    threshold_results: Dict[str, object] = {}
    for threshold in (5.0, 10.0, 20.0):
        rotation_states: List[Optional[bool]] = [finite_float(row["kappa_rotation"]) > threshold if math.isfinite(finite_float(row["kappa_rotation"])) else None for row in enriched]
        translation_states: List[Optional[bool]] = [finite_float(row["kappa_translation"]) > threshold if math.isfinite(finite_float(row["kappa_translation"])) else None for row in enriched]
        rotation_mask = [value for value in rotation_states if value is not None]
        translation_mask = [value for value in translation_states if value is not None]
        timestamps = [finite_float(row["to_stamp_sec"]) for row in enriched]
        rotation_error_degenerate = [finite_float(row["rotation_error_rad"]) for row, state in zip(enriched, rotation_states) if state is True]
        rotation_error_not_degenerate = [finite_float(row["rotation_error_rad"]) for row, state in zip(enriched, rotation_states) if state is False]
        translation_error_degenerate = [finite_float(row["translation_error_m"]) for row, state in zip(enriched, translation_states) if state is True]
        translation_error_not_degenerate = [finite_float(row["translation_error_m"]) for row, state in zip(enriched, translation_states) if state is False]
        threshold_results[str(int(threshold))] = {
            "rotation_degenerate_count": int(sum(rotation_mask)),
            "rotation_denominator": len(rotation_mask),
            "rotation_occupancy": sum(rotation_mask) / len(rotation_mask) if rotation_mask else float("nan"),
            "rotation_transition_count": sum(first is not None and second is not None and first != second for first, second in zip(rotation_states, rotation_states[1:])),
            "rotation_longest_run_edges": longest_true_run([value is True for value in rotation_states]),
            "rotation_longest_run_duration_sec": longest_run_duration(rotation_states, timestamps),
            "rotation_error_when_degenerate": summarize(rotation_error_degenerate),
            "rotation_error_when_not_degenerate": summarize(rotation_error_not_degenerate),
            "translation_degenerate_count": int(sum(translation_mask)),
            "translation_denominator": len(translation_mask),
            "translation_occupancy": sum(translation_mask) / len(translation_mask) if translation_mask else float("nan"),
            "translation_transition_count": sum(first is not None and second is not None and first != second for first, second in zip(translation_states, translation_states[1:])),
            "translation_longest_run_edges": longest_true_run([value is True for value in translation_states]),
            "translation_longest_run_duration_sec": longest_run_duration(translation_states, timestamps),
            "translation_error_when_degenerate": summarize(translation_error_degenerate),
            "translation_error_when_not_degenerate": summarize(translation_error_not_degenerate),
        }

    conditioning_bins = {}
    for group, kappa_field, error_field in (
        ("rotation", "kappa_rotation", "rotation_error_rad"),
        ("translation", "kappa_translation", "translation_error_m"),
    ):
        conditioning_bins[group] = []
        for label in ("below_5", "5_to_10", "10_to_20", "20_to_100", "100_or_more"):
            selected = [row for row in enriched if row[f"{group}_condition_bin"] == label]
            conditioning_bins[group].append({
                "bin": label,
                "count": len(selected),
                "kappa": summarize([finite_float(row[kappa_field]) for row in selected]),
                "error": summarize([finite_float(row[error_field]) for row in selected]),
            })

    taxonomy = Counter()
    for row in enriched:
        translation_error_high = finite_float(row["translation_error_m"]) >= 0.50
        rotation_error_high = finite_float(row["rotation_error_rad"]) >= 0.02
        translation_kappa = finite_float(row["kappa_translation"])
        rotation_kappa = finite_float(row["kappa_rotation"])
        translation_poor = translation_kappa > 10.0
        rotation_poor = rotation_kappa > 10.0
        if math.isfinite(translation_kappa):
            taxonomy[f"translation_{'poor' if translation_poor else 'moderate_good'}_conditioning_{'high' if translation_error_high else 'low'}_error"] += 1
        else:
            taxonomy["translation_conditioning_unavailable"] += 1
        if math.isfinite(rotation_kappa):
            taxonomy[f"rotation_{'poor' if rotation_poor else 'moderate_good'}_conditioning_{'high' if rotation_error_high else 'low'}_error"] += 1
        else:
            taxonomy["rotation_conditioning_unavailable"] += 1
        if translation_error_high and finite_float(row["inlier_fraction"]) < 0.25:
            taxonomy["poor_support_high_translation_error"] += 1
        if translation_error_high and finite_float(row["inlier_fraction"]) >= 0.50:
            taxonomy["high_support_high_translation_error"] += 1
        if translation_error_high and math.isfinite(translation_kappa) and not translation_poor and finite_float(row["inlier_fraction"]) >= 0.50:
            taxonomy["apparently_successful_moderate_conditioning_high_error"] += 1
        if finite_float(row["dcreg_linear_solve_success_fraction"]) < 1.0:
            taxonomy["linear_solve_failure"] += 1
        if finite_float(row["registration_converged_fraction"]) < 1.0:
            taxonomy["registration_not_converged"] += 1

    wrong_candidates = []
    for row in enriched:
        gt_displacement = finite_float(row["gt_displacement_m"])
        ratio = finite_float(row["glim_displacement_m"]) / gt_displacement if gt_displacement > 0.0 else float("nan")
        if (
            finite_float(row["translation_error_m"]) >= 0.50
            and math.isfinite(finite_float(row["kappa_translation"]))
            and finite_float(row["translation_vector_cosine"]) <= -0.80
            and 0.80 <= ratio <= 1.25
            and finite_float(row["inlier_fraction"]) >= 0.50
            and finite_float(row["dcreg_linear_solve_success_fraction"]) >= 1.0
            and finite_float(row["registration_converged_fraction"]) >= 1.0
        ):
            candidate = dict(row)
            candidate["glim_to_gt_displacement_ratio"] = ratio
            wrong_candidates.append(candidate)
    selected_wrong: List[Dict[str, object]] = []
    if wrong_candidates:
        first_time = min(finite_float(row["to_stamp_sec"]) for row in wrong_candidates)
        last_time = max(finite_float(row["to_stamp_sec"]) for row in wrong_candidates)
        width = max((last_time - first_time) / 6.0, 1.0e-9)
        for bin_index in range(6):
            candidates = [
                row for row in wrong_candidates
                if min(5, int((finite_float(row["to_stamp_sec"]) - first_time) / width)) == bin_index
            ]
            if candidates:
                selected_wrong.append(min(candidates, key=lambda row: (finite_float(row["to_stamp_sec"]), int(row["to_pose_index"]))))
    taxonomy["wrong_basin_candidate"] = len(wrong_candidates)

    support_fields = {
        "source_points": values("source_points"),
        "inlier_fraction": values("inlier_fraction"),
        "initial_cost": values("initial_cost"),
        "final_cost": values("final_cost"),
        "scan_minus_imu_translation_m": values("scan_minus_imu_translation_m"),
        "scan_minus_imu_rotation_deg": values("scan_minus_imu_rotation_deg"),
    }
    translation_error = values("translation_error_m")
    rotation_error = values("rotation_error_rad")
    support_correlations = {
        field: {
            "translation_error_rho": spearman(field_values, translation_error),
            "rotation_error_rho": spearman(field_values, rotation_error),
        }
        for field, field_values in support_fields.items()
    }
    support_strata = {}
    for stratum in ("low", "moderate", "high"):
        rows = [row for row in enriched if row["support_stratum"] == stratum]
        support_strata[stratum] = {
            "count": len(rows),
            "translation_error": summarize([finite_float(row["translation_error_m"]) for row in rows]),
            "translation_kappa_error_rho": spearman(
                [finite_float(row["log_kappa_translation"]) for row in rows],
                [finite_float(row["translation_error_m"]) for row in rows],
            ),
        }

    summary = {
        "dataset": name,
        "unique_edge_count": len(enriched),
        "relative_health_available_count": 0,
        "kappa_rotation": summarize(values("kappa_rotation")),
        "kappa_translation": summarize(values("kappa_translation")),
        "log_kappa_rotation": summarize(values("log_kappa_rotation")),
        "log_kappa_translation": summarize(values("log_kappa_translation")),
        "rotation_error_rad": summarize(rotation_error),
        "translation_error_m": summarize(translation_error),
        "threshold_sensitivity": threshold_results,
        "conditioning_bins": conditioning_bins,
        "continuous_associations": {
            "rotation_kappa_vs_rotation_error_rho": spearman(values("log_kappa_rotation"), rotation_error),
            "translation_kappa_vs_translation_error_rho": spearman(values("log_kappa_translation"), translation_error),
        },
        "support_and_consistency_associations": support_correlations,
        "support_strata": support_strata,
        "taxonomy": dict(sorted(taxonomy.items())),
        "wrong_basin_candidate_count": len(wrong_candidates),
        "wrong_basin_selected_count": len(selected_wrong),
        "scan_imu_diagnostic": {
            "definition": "Log-like norms of inverse(pred_T_last_current) * scan_T_last_current, logged passively by frozen GLIM",
            "available_edge_count": sum(finite_float(row["scan_diagnostic_count"]) > 0 for row in enriched),
            "translation_disagreement_m": summarize(values("scan_minus_imu_translation_m")),
            "rotation_disagreement_deg": summarize(values("scan_minus_imu_rotation_deg")),
        },
    }
    return summary, enriched, selected_wrong


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    if not rows:
        path.write_text("")
        return
    fields: List[str] = []
    for row in rows:
        for field in row:
            if field not in fields:
                fields.append(field)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


class Svg:
    def __init__(self, width: int, height: int, title: str):
        self.width = width
        self.height = height
        self.items = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
            '<rect width="100%" height="100%" fill="white"/>',
            f'<text x="{width / 2}" y="28" text-anchor="middle" font-family="sans-serif" font-size="19" font-weight="bold">{escape(title)}</text>',
        ]

    def line(self, x1, y1, x2, y2, color="#333", width=1.5, opacity=1.0, dash=""):
        dash_attribute = f' stroke-dasharray="{dash}"' if dash else ""
        self.items.append(f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" stroke="{color}" stroke-width="{width}" opacity="{opacity}"{dash_attribute}/>')

    def rect(self, x, y, width, height, fill="none", stroke="#333", opacity=1.0, radius=0):
        self.items.append(f'<rect x="{x:.2f}" y="{y:.2f}" width="{width:.2f}" height="{height:.2f}" rx="{radius}" fill="{fill}" stroke="{stroke}" opacity="{opacity}"/>')

    def circle(self, x, y, radius=2, fill="#2c7fb8", opacity=0.6):
        self.items.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{radius}" fill="{fill}" opacity="{opacity}"/>')

    def text(self, x, y, value, size=11, anchor="start", color="#111", weight="normal"):
        self.items.append(f'<text x="{x:.2f}" y="{y:.2f}" text-anchor="{anchor}" font-family="sans-serif" font-size="{size}" fill="{color}" font-weight="{weight}">{escape(str(value))}</text>')

    def polyline(self, points: Sequence[Tuple[float, float]], color="#2c7fb8", width=1.5, opacity=1.0):
        encoded = " ".join(f"{x:.2f},{y:.2f}" for x, y in points)
        self.items.append(f'<polyline points="{encoded}" fill="none" stroke="{color}" stroke-width="{width}" opacity="{opacity}"/>')

    def save(self, path: Path):
        path.write_text("\n".join(self.items + ["</svg>"]) + "\n")


def map_range(value: float, minimum: float, maximum: float, low: float, high: float) -> float:
    if maximum <= minimum:
        return 0.5 * (low + high)
    return low + (value - minimum) / (maximum - minimum) * (high - low)


def figure_a(path: Path, synthetic_payload: Mapping[str, object], synthetic_rows: Sequence[Mapping[str, object]]) -> None:
    names = ["single_plane", "parallel_planes_corridor", "tunnel", "perpendicular_corner", "three_orthogonal_surfaces", "rich_irregular_3d"]
    by_name = {scene["name"]: scene for scene in synthetic_payload["scenes"]}  # type: ignore[index]
    result_by_name = {row["scene"]: row for row in synthetic_rows}
    svg = Svg(1320, 720, "A. Controlled VGICP geometry and reference-free DCReg characterization")
    for index, name in enumerate(names):
        column = index % 3
        row_index = index // 3
        x0 = 25 + column * 435
        y0 = 55 + row_index * 325
        svg.rect(x0, y0, 410, 300, fill="#fafafa", stroke="#bbb", radius=4)
        scene = by_name[name]
        points = np.asarray(scene["point_sample"], dtype=float)
        projected_x = points[:, 0] - 0.55 * points[:, 1]
        projected_y = 0.28 * points[:, 0] + 0.28 * points[:, 1] - points[:, 2]
        for px, py in zip(projected_x, projected_y):
            svg.circle(map_range(float(px), float(np.min(projected_x)), float(np.max(projected_x)), x0 + 25, x0 + 385), map_range(float(py), float(np.min(projected_y)), float(np.max(projected_y)), y0 + 220, y0 + 50), 1.2, "#2c7fb8", 0.38)
        result = result_by_name[name]
        svg.text(x0 + 12, y0 + 22, name.replace("_", " "), 13, weight="bold")
        svg.text(x0 + 12, y0 + 246, f"expected weak R: {scene['expected_rotation_weak_subspace']}; t: {scene['expected_translation_weak_subspace']}", 10)
        svg.text(x0 + 12, y0 + 264, f"detected kappa_R={float(result['kappa_rotation']):.2f}, kappa_t={float(result['kappa_translation']):.2f}", 11)
        svg.text(x0 + 12, y0 + 282, f"projector overlap R={finite_float(result['rotation_weak_projector_overlap']):.3f}, t={finite_float(result['translation_weak_projector_overlap']):.3f}", 10)
    svg.save(path)


def figure_b(path: Path, synthetic_payload: Mapping[str, object]) -> None:
    coupled = synthetic_payload["coupled_quadratic"]
    svg = Svg(980, 540, "B. Cross-coupling hidden by diagonal blocks and exposed by Schur complements")
    labels = ["R block", "R Schur", "t block", "t Schur"]
    groups = [
        coupled["rotation_block_eigenvalues"],
        coupled["rotation_schur_eigenvalues"],
        coupled["translation_block_eigenvalues"],
        coupled["translation_schur_eigenvalues"],
    ]
    colors = ["#a6cee3", "#1f78b4", "#b2df8a"]
    svg.line(70, 440, 930, 440, "#333")
    svg.line(70, 70, 70, 440, "#333")
    for tick, label in ((0.1, "0.1"), (1.0, "1"), (10.0, "10"), (100.0, "100")):
        y = map_range(math.log10(tick), -1.0, 2.1, 440, 70)
        svg.line(70, y, 930, y, "#ddd", 1)
        svg.text(60, y + 4, label, 10, "end")
    for group_index, (label, eigenvalues) in enumerate(zip(labels, groups)):
        x_center = 170 + group_index * 205
        for mode, value in enumerate(eigenvalues):
            height = map_range(math.log10(max(float(value), 0.1)), -1.0, 2.1, 0, 350)
            svg.rect(x_center - 55 + mode * 38, 440 - height, 30, height, colors[mode], "none")
        svg.text(x_center, 465, label, 12, "middle", weight="bold" if "Schur" in label else "normal")
    svg.text(500, 505, "Diagonal block condition = 5; Schur condition = 250.1 after allowing cross-variable compensation", 13, "middle")
    svg.save(path)


def timeline_panel(svg: Svg, rows: Sequence[Mapping[str, object]], x0: float, y0: float, width: float, height: float, field: str, label: str, log=False, color="#2c7fb8") -> None:
    sampled = rows[::max(1, len(rows) // 900)]
    times = [finite_float(row["to_stamp_sec"]) for row in sampled]
    values = [finite_float(row[field]) for row in sampled]
    pairs = [(t, math.log10(max(v, 1.0e-12)) if log else v) for t, v in zip(times, values) if math.isfinite(t) and math.isfinite(v)]
    if not pairs:
        return
    t_min, t_max = pairs[0][0], pairs[-1][0]
    v_values = [pair[1] for pair in pairs]
    v_min, v_max = percentile(v_values, 1.0), percentile(v_values, 99.0)
    points = [(map_range(t, t_min, t_max, x0, x0 + width), map_range(min(max(v, v_min), v_max), v_min, v_max, y0 + height, y0)) for t, v in pairs]
    svg.rect(x0, y0, width, height, fill="#fff", stroke="#ccc")
    svg.polyline(points, color, 1.1, 0.85)
    svg.text(x0 + 4, y0 + 13, label, 10, weight="bold")
    svg.text(x0 + width - 2, y0 + height - 3, f"{(t_max-t_min):.0f} s", 9, "end", "#555")


def figure_c(path: Path, datasets: Mapping[str, Sequence[Mapping[str, object]]]) -> None:
    svg = Svg(1320, 900, "C. Continuous conditioning, binary boundary, true edge error, and support")
    for column, name in enumerate(("Short", "Medium")):
        rows = datasets[name]
        x0 = 75 + column * 630
        svg.text(x0 + 285, 55, f"GEODE Inland {name}", 14, "middle", weight="bold")
        timeline_panel(svg, rows, x0, 75, 570, 165, "kappa_translation", "log10 kappa_t (continuous)", True, "#7b3294")
        mask_rows = [dict(row, mask=float(finite_float(row["kappa_translation"]) > 10.0)) for row in rows]
        timeline_panel(svg, mask_rows, x0, 265, 570, 140, "mask", "D_t = [kappa_t > 10]", False, "#d7191c")
        timeline_panel(svg, rows, x0, 430, 570, 165, "translation_error_m", "GT translation edge error [m]", False, "#1a9641")
        timeline_panel(svg, rows, x0, 620, 570, 165, "inlier_fraction", "median inlier fraction", False, "#2c7bb6")
    svg.text(660, 845, "Binary occupancy saturates while the continuous ratio retains within-degenerate severity; no calibration is implied.", 13, "middle")
    svg.save(path)


def figure_d(path: Path, datasets: Mapping[str, Sequence[Mapping[str, object]]]) -> None:
    svg = Svg(1220, 610, "D. Continuous translational conditioning versus verified GT edge error (sequences separate)")
    for column, name in enumerate(("Short", "Medium")):
        rows = datasets[name]
        x0 = 80 + column * 590
        y0 = 75
        width, height = 500, 430
        pairs = [(math.log10(finite_float(row["kappa_translation"])), finite_float(row["translation_error_m"])) for row in rows if finite_float(row["kappa_translation"]) > 0 and math.isfinite(finite_float(row["translation_error_m"]))]
        x_values = [pair[0] for pair in pairs]
        y_values = [pair[1] for pair in pairs]
        x_min, x_max = percentile(x_values, 1), percentile(x_values, 99)
        y_min, y_max = 0.0, percentile(y_values, 99)
        svg.rect(x0, y0, width, height, fill="#fff", stroke="#777")
        for x_value, y_value in pairs[::max(1, len(pairs) // 1400)]:
            svg.circle(map_range(min(max(x_value, x_min), x_max), x_min, x_max, x0, x0 + width), map_range(min(y_value, y_max), y_min, y_max, y0 + height, y0), 1.5, "#4d4d4d", 0.22)
        bins = [1.0, 5.0, 10.0, 20.0, 100.0, float("inf")]
        medians = []
        for lower, upper in zip(bins, bins[1:]):
            subset = [(x, y) for x, y in pairs if math.log10(lower) <= x < math.log10(upper) if math.isfinite(upper)] if math.isfinite(upper) else [(x, y) for x, y in pairs if x >= math.log10(lower)]
            if subset:
                x_center = percentile([item[0] for item in subset], 50)
                y_center = percentile([item[1] for item in subset], 50)
                medians.append((map_range(x_center, x_min, x_max, x0, x0 + width), map_range(min(y_center, y_max), y_min, y_max, y0 + height, y0)))
        svg.polyline(medians, "#d7301f", 3.0)
        for x, y in medians:
            svg.circle(x, y, 4, "#d7301f", 1.0)
        svg.text(x0 + width / 2, y0 - 18, f"Inland {name}", 14, "middle", weight="bold")
        svg.text(x0 + width / 2, y0 + height + 32, "log10(kappa_t)", 12, "middle")
        svg.text(x0 - 45, y0 + height / 2, "translation edge error [m]", 11, "middle")
    svg.text(610, 575, "Dots: unique stable edges; red: fixed-bin medians. Short and Medium are not pooled.", 12, "middle")
    svg.save(path)


def figure_e(
    path: Path,
    medium_wrong: Sequence[Mapping[str, object]],
    taxonomy: Mapping[str, int],
    case_clouds: Mapping[str, np.ndarray],
) -> None:
    svg = Svg(1260, 720, "E. Medium wrong-basin cases and descriptive failure taxonomy")
    svg.text(320, 58, "GT and GLIM relative-translation vectors", 14, "middle", weight="bold")
    for index, row in enumerate(medium_wrong[:6]):
        column = index % 3
        row_index = index // 3
        x0 = 35 + column * 205
        y0 = 90 + row_index * 255
        svg.rect(x0, y0, 185, 220, fill="#fafafa", stroke="#ccc")
        edge_key = f"{row['from_pose_index']}->{row['to_pose_index']}"
        cloud = case_clouds.get(edge_key)
        if cloud is not None and len(cloud):
            sampled = cloud[::max(1, len(cloud) // 180)]
            projected_x = sampled[:, 0] - 0.55 * sampled[:, 1]
            projected_y = 0.28 * sampled[:, 0] + 0.28 * sampled[:, 1] - sampled[:, 2]
            x_min, x_max = percentile(projected_x, 2.0), percentile(projected_x, 98.0)
            y_min, y_max = percentile(projected_y, 2.0), percentile(projected_y, 98.0)
            for px, py in zip(projected_x, projected_y):
                svg.circle(
                    map_range(min(max(float(px), x_min), x_max), x_min, x_max, x0 + 12, x0 + 173),
                    map_range(min(max(float(py), y_min), y_max), y_min, y_max, y0 + 105, y0 + 30),
                    0.9,
                    "#636363",
                    0.32,
                )
        gt = finite_float(row["gt_displacement_m"])
        glim = finite_float(row["glim_displacement_m"])
        cosine = finite_float(row["translation_vector_cosine"])
        angle = math.acos(max(-1.0, min(1.0, cosine)))
        scale = 120.0 / max(gt, glim, 1.0e-9)
        origin_x, origin_y = x0 + 92, y0 + 140
        svg.line(origin_x, origin_y, origin_x + gt * scale, origin_y, "#1a9641", 4)
        svg.line(origin_x, origin_y, origin_x + glim * scale * math.cos(angle), origin_y - glim * scale * math.sin(angle), "#d7191c", 4)
        svg.text(x0 + 8, y0 + 18, f"edge {row['from_pose_index']}->{row['to_pose_index']}", 10, weight="bold")
        svg.text(x0 + 8, y0 + 174, f"cos={cosine:.3f}; err={finite_float(row['translation_error_m']):.3f} m", 9)
        svg.text(x0 + 8, y0 + 190, f"kappa_t={finite_float(row['kappa_translation']):.1f}; inlier={finite_float(row['inlier_fraction']):.2f}", 9)
        svg.text(x0 + 8, y0 + 207, "gray cloud; green GT; red GLIM", 8, color="#555")
    keys = [
        "translation_poor_conditioning_high_error",
        "translation_poor_conditioning_low_error",
        "translation_moderate_good_conditioning_high_error",
        "translation_moderate_good_conditioning_low_error",
        "high_support_high_translation_error",
        "wrong_basin_candidate",
    ]
    x0, y0, width, height = 705, 95, 500, 500
    maximum = max([taxonomy.get(key, 0) for key in keys] + [1])
    for index, key in enumerate(keys):
        count = taxonomy.get(key, 0)
        y = y0 + index * 78
        bar_width = width * count / maximum
        svg.rect(x0, y, bar_width, 32, "#756bb1", "none")
        svg.text(x0, y - 7, key.replace("_", " "), 10)
        svg.text(x0 + bar_width + 7, y + 22, count, 11, weight="bold")
    svg.text(955, 635, "Counts overlap by design: conditioning, support, matching, and correctness remain separate evidence.", 11, "middle")
    svg.save(path)


def figure_f(path: Path) -> None:
    svg = Svg(1220, 520, "F. Covariance-neutral CBS diagnostic side channel")
    boxes = [
        (60, 110, 220, 82, "GLIM local VGICP", "unchanged optimization"),
        (360, 110, 220, 82, "Legacy G->K belief", "mean + covariance unchanged"),
        (680, 110, 220, 82, "Kimera CBS receiver", "unchanged factor insertion"),
        (360, 315, 220, 90, "Stage 2A metadata", "observability / support / matching"),
        (680, 315, 220, 90, "Stage 2B shadow", "optional disagreement only"),
        (980, 315, 180, 90, "Offline analysis", "no feedback"),
    ]
    for x, y, width, height, title, subtitle in boxes:
        svg.rect(x, y, width, height, fill="#f7fbff" if y < 250 else "#fff7ec", stroke="#3182bd" if y < 250 else "#e6550d", radius=8)
        svg.text(x + width / 2, y + 32, title, 13, "middle", weight="bold")
        svg.text(x + width / 2, y + 57, subtitle, 10, "middle")
    for x1, y1, x2, y2, color in ((280,151,360,151,"#3182bd"),(580,151,680,151,"#3182bd"),(470,192,470,315,"#e6550d"),(580,360,680,360,"#e6550d"),(900,360,980,360,"#e6550d")):
        svg.line(x1, y1, x2, y2, color, 3)
    svg.text(610, 470, "No metadata field changes a Gaussian belief, covariance, factor, gate, or optimizer.", 14, "middle", weight="bold")
    svg.save(path)


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--synthetic", type=Path, required=True)
    parser.add_argument("--short", type=Path, required=True)
    parser.add_argument("--medium", type=Path, required=True)
    parser.add_argument("--short-sanity", type=Path, required=True)
    parser.add_argument("--medium-sanity", type=Path, required=True)
    parser.add_argument("--short-log", type=Path, required=True)
    parser.add_argument("--medium-log", type=Path, required=True)
    parser.add_argument("--short-health", type=Path, required=True)
    parser.add_argument("--medium-health", type=Path, required=True)
    parser.add_argument("--short-established-summary", type=Path, required=True)
    parser.add_argument("--medium-established-summary", type=Path, required=True)
    parser.add_argument("--medium-wrong-clouds", type=Path, required=True)
    parser.add_argument("--medium-wrong-cloud-manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    arguments.output.mkdir(parents=True, exist_ok=True)
    figure_directory = arguments.output / "figures"
    table_directory = arguments.output / "tables"
    figure_directory.mkdir(exist_ok=True)
    table_directory.mkdir(exist_ok=True)

    plan = json.loads(arguments.plan.read_text())
    synthetic_payload = json.loads(arguments.synthetic.read_text())
    synthetic_rows, synthetic_summary = synthetic_analysis(synthetic_payload)
    short_rows = read_csv(arguments.short)
    medium_rows = read_csv(arguments.medium)
    short_sanity = read_csv(arguments.short_sanity)
    medium_sanity = read_csv(arguments.medium_sanity)
    short_scan = parse_scan_health_log(arguments.short_log)
    medium_scan = parse_scan_health_log(arguments.medium_log)
    merge_dcreg_health_csv(arguments.short_health, short_scan)
    merge_dcreg_health_csv(arguments.medium_health, medium_scan)
    short_summary, short_enriched, short_wrong = sequence_analysis("Short", short_rows, short_sanity, short_scan)
    medium_summary, medium_enriched, medium_wrong = sequence_analysis("Medium", medium_rows, medium_sanity, medium_scan)
    established_short = json.loads(arguments.short_established_summary.read_text())
    established_medium = json.loads(arguments.medium_established_summary.read_text())

    detector_rows = [
        {"detector": "DCReg Schur condition", "matrix": "S_R and S_t", "handles_rotation_translation_coupling": True, "scale_sensitive": False, "directional_mode": True, "reference_free": True},
        {"detector": "full Hessian condition", "matrix": "H 6x6", "handles_rotation_translation_coupling": "implicitly, but does not separate groups", "scale_sensitive": False, "directional_mode": "6D mixed", "reference_free": True},
        {"detector": "full Hessian minimum eigenvalue", "matrix": "H 6x6", "handles_rotation_translation_coupling": "implicitly", "scale_sensitive": True, "directional_mode": "6D mixed", "reference_free": True},
        {"detector": "fixed numerical rank", "matrix": "H 6x6", "handles_rotation_translation_coupling": "implicitly", "scale_sensitive": "threshold dependent", "directional_mode": False, "reference_free": True},
        {"detector": "diagonal-block condition", "matrix": "H_RR or H_tt", "handles_rotation_translation_coupling": False, "scale_sensitive": False, "directional_mode": True, "reference_free": True},
        {"detector": "diagonal-block minimum eigenvalue", "matrix": "H_RR or H_tt", "handles_rotation_translation_coupling": False, "scale_sensitive": True, "directional_mode": True, "reference_free": True},
        {"detector": "inlier fraction", "matrix": "none", "handles_rotation_translation_coupling": False, "scale_sensitive": False, "directional_mode": False, "reference_free": True},
        {"detector": "source/inlier count", "matrix": "none", "handles_rotation_translation_coupling": False, "scale_sensitive": True, "directional_mode": False, "reference_free": True},
    ]
    threshold_rows: List[Dict[str, object]] = []
    for dataset, summary in (("Short", short_summary), ("Medium", medium_summary)):
        for threshold, result in summary["threshold_sensitivity"].items():
            threshold_rows.append({"dataset": dataset, "threshold": threshold, **result})
    for row in synthetic_rows:
        for threshold in (5, 10, 20):
            threshold_rows.append({
                "dataset": f"synthetic:{row['scene']}",
                "threshold": threshold,
                "rotation_degenerate_count": row[f"rotation_mask_tau{threshold}"],
                "rotation_denominator": 1,
                "rotation_occupancy": row[f"rotation_mask_tau{threshold}"],
                "translation_degenerate_count": row[f"translation_mask_tau{threshold}"],
                "translation_denominator": 1,
                "translation_occupancy": row[f"translation_mask_tau{threshold}"],
            })

    write_csv(table_directory / "synthetic_scene_characterization.csv", synthetic_rows)
    write_csv(table_directory / "detector_comparison.csv", detector_rows)
    write_csv(table_directory / "threshold_sensitivity.csv", threshold_rows)
    write_csv(table_directory / "short_unique_edges_stage3a.csv", short_enriched)
    write_csv(table_directory / "medium_unique_edges_stage3a.csv", medium_enriched)
    write_csv(table_directory / "wrong_basin_cases.csv", medium_wrong)
    taxonomy_rows = []
    for dataset, summary in (("Short", short_summary), ("Medium", medium_summary)):
        for category, count in summary["taxonomy"].items():
            taxonomy_rows.append({"dataset": dataset, "category": category, "count": count})
    write_csv(table_directory / "failure_taxonomy.csv", taxonomy_rows)

    geode_findings_rows = []
    for dataset, summary, established in (
        ("Short", short_summary, established_short),
        ("Medium", medium_summary, established_medium),
    ):
        rotation_association = established["primary_block_bootstrap_associations"]["log_rotation_condition_vs_rotation_error"]
        translation_association = established["primary_block_bootstrap_associations"]["log_translation_condition_vs_translation_error"]
        geode_findings_rows.append({
            "dataset": dataset,
            "gt_valid_unique_edges": summary["unique_edge_count"],
            "rotation_kappa_available_edges": summary["threshold_sensitivity"]["10"]["rotation_denominator"],
            "translation_kappa_available_edges": summary["threshold_sensitivity"]["10"]["translation_denominator"],
            "rotation_degenerate_occupancy_tau10": summary["threshold_sensitivity"]["10"]["rotation_occupancy"],
            "translation_degenerate_occupancy_tau10": summary["threshold_sensitivity"]["10"]["translation_occupancy"],
            "rotation_kappa_error_rho": rotation_association["estimate"],
            "rotation_kappa_error_ci_lower": rotation_association["lower"],
            "rotation_kappa_error_ci_upper": rotation_association["upper"],
            "translation_kappa_error_rho": translation_association["estimate"],
            "translation_kappa_error_ci_lower": translation_association["lower"],
            "translation_kappa_error_ci_upper": translation_association["upper"],
            "inlier_fraction_translation_error_rho": summary["support_and_consistency_associations"]["inlier_fraction"]["translation_error_rho"],
            "scan_imu_translation_translation_error_rho": summary["support_and_consistency_associations"]["scan_minus_imu_translation_m"]["translation_error_rho"],
            "relative_health_available_edges": 0,
        })
    write_csv(table_directory / "geode_short_medium_findings.csv", geode_findings_rows)

    runtime_rows = [
        {"component": "Stage 1 health monitor", "metric": "median compute", "value": 0.01395, "unit": "ms/scan", "producer_waits": 0, "drops": 0, "source": "accepted Stage 1 closeout"},
        {"component": "Stage 1 health queue", "metric": "mean nonblocking publish", "value": 0.002094, "unit": "ms/scan", "producer_waits": 0, "drops": 0, "source": "accepted Stage 1 closeout"},
        {"component": "Stage 2A metadata producer", "metric": "mean descriptor copy lower", "value": 1.008, "unit": "us/array", "producer_waits": 0, "drops": 0, "source": "accepted Stage 2A report"},
        {"component": "Stage 2A metadata producer", "metric": "mean descriptor copy upper", "value": 1.039, "unit": "us/array", "producer_waits": 0, "drops": 0, "source": "accepted Stage 2A report"},
        {"component": "Stage 2A metadata worker", "metric": "mean association/hash/message upper", "value": 82.0, "unit": "us/array", "producer_waits": 0, "drops": 0, "source": "accepted Stage 2A report"},
        {"component": "Stage 2B active belief enqueue", "metric": "maximum", "value": 2.744, "unit": "us", "producer_waits": 0, "drops": 0, "source": "accepted Stage 2B isolated run"},
        {"component": "Stage 2B active receiver enqueue", "metric": "maximum", "value": 2.894, "unit": "us", "producer_waits": 0, "drops": 0, "source": "accepted Stage 2B isolated run"},
        {"component": "deterministic no-op", "metric": "maximum estimator/CBS difference", "value": 0.0, "unit": "at 1e-12 tolerance", "producer_waits": 0, "drops": 0, "source": "accepted deterministic A/B/C/D"},
    ]
    write_csv(table_directory / "runtime_passivity.csv", runtime_rows)

    figure_a(figure_directory / "figure_a_synthetic_geometry.svg", synthetic_payload, synthetic_rows)
    figure_b(figure_directory / "figure_b_schur_coupling.svg", synthetic_payload)
    figure_c(figure_directory / "figure_c_geode_timeline.svg", {"Short": short_enriched, "Medium": medium_enriched})
    figure_d(figure_directory / "figure_d_conditioning_vs_error.svg", {"Short": short_enriched, "Medium": medium_enriched})
    cloud_manifest = json.loads(arguments.medium_wrong_cloud_manifest.read_text())
    cloud_archive = np.load(arguments.medium_wrong_clouds)
    case_clouds = {
        entry["edge"]: cloud_archive[entry["array_key"]]
        for entry in cloud_manifest["entries"]
        if entry["endpoint"] == "end"
    }
    figure_e(
        figure_directory / "figure_e_failure_taxonomy.svg",
        medium_wrong,
        medium_summary["taxonomy"],
        case_clouds,
    )
    figure_f(figure_directory / "figure_f_cbs_architecture.svg")

    summary = {
        "schema_version": 1,
        "plan_sha256": file_sha256(arguments.plan),
        "synthetic_input_sha256": file_sha256(arguments.synthetic),
        "synthetic": synthetic_summary,
        "geode": {"Short": short_summary, "Medium": medium_summary},
        "established_stage2c_results_preserved": {
            "Short": established_short["primary_block_bootstrap_associations"],
            "Medium": established_medium["primary_block_bootstrap_associations"],
        },
        "wrong_basin_selection_rule": plan["wrong_basin_case_rule"],
        "wrong_basin_case_clouds": {
            "npz_path": str(arguments.medium_wrong_clouds),
            "npz_sha256": file_sha256(arguments.medium_wrong_clouds),
            "manifest_path": str(arguments.medium_wrong_cloud_manifest),
            "manifest_sha256": file_sha256(arguments.medium_wrong_cloud_manifest),
            "entry_count": len(cloud_manifest["entries"]),
            "maximum_timestamp_error_ns": max(entry["timestamp_error_ns"] for entry in cloud_manifest["entries"]),
        },
        "generated_figures": {},
        "generated_tables": {},
        "scientific_boundaries": [
            "Condition ratios characterize local registration observability; they are not covariance, confidence, probability, trust, accuracy, or metric error.",
            "Rotation radians and translation metres remain separate.",
            "Relative Gamma health is unavailable because no qualified compatible reference was found.",
            "No Stage 3A analysis output is fed back to GLIM, CBS, or Kimera.",
        ],
    }
    for path in sorted(figure_directory.glob("*.svg")):
        summary["generated_figures"][path.name] = {"path": str(path), "sha256": file_sha256(path), "bytes": path.stat().st_size}
    for path in sorted(table_directory.glob("*.csv")):
        summary["generated_tables"][path.name] = {"path": str(path), "sha256": file_sha256(path), "bytes": path.stat().st_size}
    output_path = arguments.output / "stage3a_analysis_summary.json"
    output_path.write_text(json.dumps(summary, indent=2, sort_keys=True, allow_nan=True) + "\n")
    print(json.dumps({"summary": str(output_path), "sha256": file_sha256(output_path), "synthetic_crosscheck": synthetic_summary}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
