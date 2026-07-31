#!/usr/bin/env python3
"""Read-only DCReg direction versus outgoing-belief consistency audit.

The audit joins three artifacts by GLIM pose index:

* the Schur eigenvalues/ratios from ``GLIM_SCAN_DCREG_ROW``;
* the corresponding local eigenvectors from ``GLIM_SCAN_DCREG_BASIS_ROW``;
* the exact relative mean and covariance actually published by GLIM from
  ``GLIM_DCREG_BELIEF_SHADOW_ROW``.

Ground-truth poses are interpolated at both endpoints.  The residual is

    Log(T_glim_from_to^-1 * T_gt_from_to)

in GTSAM Pose3 local tangent order ``[rx, ry, rz, tx, ty, tz]``.  Nothing in
this module changes an estimator, factor, covariance, or CBS message.
"""

from __future__ import annotations

import bisect
import math
from typing import Any, Dict, List, Optional, Sequence, Tuple


DEFAULT_THRESHOLDS = (3.0, 10.0, 30.0, 50.0, 100.0, 200.0)
Pose = Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _q_normalize(
    q: Sequence[float],
) -> Tuple[float, float, float, float]:
    norm = math.sqrt(sum(float(value) ** 2 for value in q))
    if not math.isfinite(norm) or norm <= 1.0e-15:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(float(value) / norm for value in q)  # type: ignore[return-value]


def _q_conjugate(
    q: Sequence[float],
) -> Tuple[float, float, float, float]:
    return (-q[0], -q[1], -q[2], q[3])


def _q_multiply(
    lhs: Sequence[float],
    rhs: Sequence[float],
) -> Tuple[float, float, float, float]:
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _q_rotate(
    q: Sequence[float],
    vector: Sequence[float],
) -> Tuple[float, float, float]:
    qn = _q_normalize(q)
    pure = (vector[0], vector[1], vector[2], 0.0)
    rotated = _q_multiply(_q_multiply(qn, pure), _q_conjugate(qn))
    return (rotated[0], rotated[1], rotated[2])


def _q_slerp(
    lhs: Sequence[float],
    rhs: Sequence[float],
    alpha: float,
) -> Tuple[float, float, float, float]:
    q0 = _q_normalize(lhs)
    q1 = _q_normalize(rhs)
    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0.0:
        q1 = tuple(-value for value in q1)
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        return _q_normalize(
            tuple((1.0 - alpha) * a + alpha * b for a, b in zip(q0, q1))
        )
    angle = math.acos(dot)
    sin_angle = math.sin(angle)
    if abs(sin_angle) <= 1.0e-15:
        return q0
    scale0 = math.sin((1.0 - alpha) * angle) / sin_angle
    scale1 = math.sin(alpha * angle) / sin_angle
    return _q_normalize(
        tuple(scale0 * a + scale1 * b for a, b in zip(q0, q1))
    )


def _pose_between(lhs: Pose, rhs: Pose) -> Pose:
    lhs_t, lhs_q = lhs
    rhs_t, rhs_q = rhs
    lhs_q_inv = _q_conjugate(_q_normalize(lhs_q))
    delta_world = tuple(rhs_t[i] - lhs_t[i] for i in range(3))
    return (
        _q_rotate(lhs_q_inv, delta_world),
        _q_normalize(_q_multiply(lhs_q_inv, rhs_q)),
    )


def _skew_times_vector(
    omega: Sequence[float],
    vector: Sequence[float],
) -> Tuple[float, float, float]:
    return (
        omega[1] * vector[2] - omega[2] * vector[1],
        omega[2] * vector[0] - omega[0] * vector[2],
        omega[0] * vector[1] - omega[1] * vector[0],
    )


def se3_log(pose: Pose) -> Tuple[float, float, float, float, float, float]:
    """GTSAM-compatible Pose3 Logmap for quaternion/translation input."""
    translation, quaternion = pose
    qx, qy, qz, qw = _q_normalize(quaternion)
    # q and -q represent the same rotation.  Select the shortest logarithm.
    if qw < 0.0:
        qx, qy, qz, qw = -qx, -qy, -qz, -qw
    vector_norm = math.sqrt(qx * qx + qy * qy + qz * qz)
    if vector_norm <= 1.0e-12:
        omega = (2.0 * qx, 2.0 * qy, 2.0 * qz)
    else:
        theta = 2.0 * math.atan2(vector_norm, max(0.0, qw))
        scale = theta / vector_norm
        omega = (scale * qx, scale * qy, scale * qz)

    theta = math.sqrt(sum(value * value for value in omega))
    omega_cross_t = _skew_times_vector(omega, translation)
    omega_cross_omega_cross_t = _skew_times_vector(omega, omega_cross_t)
    if theta <= 1.0e-6:
        # V^-1 = I - 1/2 W + 1/12 W^2 + O(theta^4)
        coefficient = 1.0 / 12.0
    else:
        coefficient = (
            1.0 / (theta * theta)
            - (1.0 + math.cos(theta))
            / (2.0 * theta * math.sin(theta))
        )
    tangent_translation = tuple(
        translation[i]
        - 0.5 * omega_cross_t[i]
        + coefficient * omega_cross_omega_cross_t[i]
        for i in range(3)
    )
    return (
        omega[0],
        omega[1],
        omega[2],
        tangent_translation[0],
        tangent_translation[1],
        tangent_translation[2],
    )


def directional_relative_residual(measured: Pose, truth: Pose) -> Tuple[float, ...]:
    """Return Local(measured, truth) = Log(measured^-1 * truth)."""
    return se3_log(_pose_between(measured, truth))


def interpolate_pose(
    poses: Sequence[Dict[str, float]],
    query_t: float,
    max_bracket_sec: float = 0.25,
) -> Tuple[Optional[Pose], float]:
    """Linear translation plus shortest-arc quaternion slerp."""
    if not poses:
        return None, math.inf
    times = [float(pose["t"]) for pose in poses]
    if query_t < times[0] or query_t > times[-1]:
        return None, math.inf
    index = bisect.bisect_left(times, query_t)
    if index < len(poses) and abs(times[index] - query_t) <= 1.0e-9:
        pose = poses[index]
        return (
            (
                (pose["x"], pose["y"], pose["z"]),
                (pose["qx"], pose["qy"], pose["qz"], pose["qw"]),
            ),
            0.0,
        )
    if index <= 0 or index >= len(poses):
        return None, math.inf
    before = poses[index - 1]
    after = poses[index]
    span = float(after["t"]) - float(before["t"])
    if span <= 0.0 or span > max_bracket_sec:
        return None, span
    alpha = (query_t - float(before["t"])) / span
    translation = tuple(
        float(before[axis]) + alpha * (float(after[axis]) - float(before[axis]))
        for axis in ("x", "y", "z")
    )
    quaternion = _q_slerp(
        (before["qx"], before["qy"], before["qz"], before["qw"]),
        (after["qx"], after["qy"], after["qz"], after["qw"]),
        alpha,
    )
    return (translation, quaternion), span


def _percentile(values: Sequence[float], quantile: float) -> float:
    finite = sorted(float(value) for value in values if _finite(value))
    if not finite:
        return math.nan
    position = max(0.0, min(1.0, quantile)) * (len(finite) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return finite[lower]
    alpha = position - lower
    return finite[lower] * (1.0 - alpha) + finite[upper] * alpha


def _mean(values: Sequence[float]) -> float:
    finite = [float(value) for value in values if _finite(value)]
    return sum(finite) / len(finite) if finite else math.nan


def _rms(values: Sequence[float]) -> float:
    finite = [float(value) for value in values if _finite(value)]
    return (
        math.sqrt(sum(value * value for value in finite) / len(finite))
        if finite
        else math.nan
    )


def _mode_vector(
    basis: Dict[str, Any],
    space: str,
    mode: int,
) -> Optional[Tuple[float, float, float]]:
    vector = tuple(
        float(basis.get(f"{space}_mode_{mode}_{axis}", math.nan))
        for axis in ("x", "y", "z")
    )
    if not all(_finite(value) for value in vector):
        return None
    norm = math.sqrt(sum(value * value for value in vector))
    if norm <= 1.0e-12:
        return None
    return tuple(value / norm for value in vector)


def _project_variance(
    covariance: Sequence[Sequence[float]],
    vector: Sequence[float],
    offset: int,
) -> float:
    return sum(
        vector[row]
        * 0.5
        * (covariance[offset + row][offset + col]
           + covariance[offset + col][offset + row])
        * vector[col]
        for row in range(3)
        for col in range(3)
    )


def _row_metrics(rows: Sequence[Dict[str, Any]]) -> Dict[str, float]:
    errors = [abs(float(row["projected_error"])) for row in rows]
    sigmas = [float(row["projected_sigma"]) for row in rows]
    normalized = [float(row["normalized_abs_error"]) for row in rows]
    nis = [float(row["directional_nis"]) for row in rows]
    reference_sigmas = [
        float(row.get("reference_projected_sigma", math.nan)) for row in rows
    ]
    combined_normalized = [
        float(row.get("reference_aware_normalized_abs_error", math.nan))
        for row in rows
        if _finite(row.get("reference_aware_normalized_abs_error", math.nan))
    ]
    combined_nis = [
        float(row.get("reference_aware_directional_nis", math.nan))
        for row in rows
        if _finite(row.get("reference_aware_directional_nis", math.nan))
    ]
    return {
        "count": float(len(rows)),
        "absolute_error_rms": _rms(errors),
        "absolute_error_p50": _percentile(errors, 0.50),
        "absolute_error_p95": _percentile(errors, 0.95),
        "projected_sigma_p50": _percentile(sigmas, 0.50),
        "projected_sigma_p95": _percentile(sigmas, 0.95),
        "normalized_abs_error_p50": _percentile(normalized, 0.50),
        "normalized_abs_error_p95": _percentile(normalized, 0.95),
        "directional_nis_mean": _mean(nis),
        "directional_nis_p50": _percentile(nis, 0.50),
        "directional_nis_p95": _percentile(nis, 0.95),
        "coverage_1sigma": (
            sum(value <= 1.0 for value in normalized) / len(normalized)
            if normalized
            else math.nan
        ),
        "coverage_1_96sigma": (
            sum(value <= 1.96 for value in normalized) / len(normalized)
            if normalized
            else math.nan
        ),
        "coverage_3sigma": (
            sum(value <= 3.0 for value in normalized) / len(normalized)
            if normalized
            else math.nan
        ),
        "reference_projected_sigma_p50": _percentile(
            reference_sigmas, 0.50
        ),
        "reference_aware_normalized_abs_error_p50": _percentile(
            combined_normalized, 0.50
        ),
        "reference_aware_normalized_abs_error_p95": _percentile(
            combined_normalized, 0.95
        ),
        "reference_aware_directional_nis_mean": _mean(combined_nis),
        "reference_aware_directional_nis_p95": _percentile(
            combined_nis, 0.95
        ),
        "reference_aware_coverage_1_96sigma": (
            sum(value <= 1.96 for value in combined_normalized)
            / len(combined_normalized)
            if combined_normalized
            else math.nan
        ),
    }


def _rank(values: Sequence[float]) -> List[float]:
    indexed = sorted(enumerate(values), key=lambda item: item[1])
    ranks = [0.0] * len(values)
    start = 0
    while start < len(indexed):
        end = start + 1
        while end < len(indexed) and indexed[end][1] == indexed[start][1]:
            end += 1
        average_rank = 0.5 * (start + end - 1) + 1.0
        for position in range(start, end):
            ranks[indexed[position][0]] = average_rank
        start = end
    return ranks


def _pearson(lhs: Sequence[float], rhs: Sequence[float]) -> float:
    if len(lhs) != len(rhs) or len(lhs) < 3:
        return math.nan
    lhs_mean = sum(lhs) / len(lhs)
    rhs_mean = sum(rhs) / len(rhs)
    lhs_centered = [value - lhs_mean for value in lhs]
    rhs_centered = [value - rhs_mean for value in rhs]
    denominator = math.sqrt(
        sum(value * value for value in lhs_centered)
        * sum(value * value for value in rhs_centered)
    )
    if denominator <= 0.0:
        return math.nan
    return sum(
        a * b for a, b in zip(lhs_centered, rhs_centered)
    ) / denominator


def _spearman(lhs: Sequence[float], rhs: Sequence[float]) -> float:
    pairs = [
        (float(a), float(b))
        for a, b in zip(lhs, rhs)
        if _finite(a) and _finite(b)
    ]
    if len(pairs) < 3:
        return math.nan
    left, right = zip(*pairs)
    return _pearson(_rank(left), _rank(right))


def summarize_directional_audit(
    audit_rows: Sequence[Dict[str, Any]],
    thresholds: Sequence[float] = DEFAULT_THRESHOLDS,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]], List[Dict[str, Any]]]:
    summary_rows: List[Dict[str, Any]] = []
    comparison_rows: List[Dict[str, Any]] = []
    correlation_rows: List[Dict[str, Any]] = []
    spaces_present = [
        space
        for space in ("rotation", "translation")
        if any(row["space"] == space for row in audit_rows)
    ]
    for space in spaces_present:
        space_rows = [row for row in audit_rows if row["space"] == space]
        ratios = [
            math.log10(max(float(row["spectral_ratio"]), 1.0e-15))
            for row in space_rows
        ]
        absolute_errors = [
            math.log10(max(abs(float(row["projected_error"])), 1.0e-15))
            for row in space_rows
        ]
        normalized_errors = [
            math.log10(max(float(row["normalized_abs_error"]), 1.0e-15))
            for row in space_rows
        ]
        correlation_rows.append(
            {
                "space": space,
                "count": len(space_rows),
                "spearman_log_ratio_vs_log_abs_error": _spearman(
                    ratios, absolute_errors
                ),
                "spearman_log_ratio_vs_log_normalized_abs_error": _spearman(
                    ratios, normalized_errors
                ),
            }
        )
        for threshold in thresholds:
            classified: Dict[str, List[Dict[str, Any]]] = {
                "weak": [],
                "strong": [],
            }
            for row in space_rows:
                classification = (
                    "weak"
                    if float(row["spectral_ratio"]) > float(threshold)
                    else "strong"
                )
                classified[classification].append(row)
            metrics_by_class: Dict[str, Dict[str, float]] = {}
            for classification in ("weak", "strong"):
                metrics = _row_metrics(classified[classification])
                metrics_by_class[classification] = metrics
                summary_rows.append(
                    {
                        "threshold": float(threshold),
                        "space": space,
                        "classification": classification,
                        **metrics,
                    }
                )
            weak = metrics_by_class["weak"]
            strong = metrics_by_class["strong"]

            def ratio(numerator: float, denominator: float) -> float:
                if not _finite(numerator) or not _finite(denominator):
                    return math.nan
                return numerator / denominator if denominator > 0.0 else math.nan

            comparison_rows.append(
                {
                    "threshold": float(threshold),
                    "space": space,
                    "weak_count": int(weak["count"]),
                    "strong_count": int(strong["count"]),
                    "weak_absolute_error_rms": weak["absolute_error_rms"],
                    "strong_absolute_error_rms": strong["absolute_error_rms"],
                    "weak_over_strong_error_rms": ratio(
                        weak["absolute_error_rms"],
                        strong["absolute_error_rms"],
                    ),
                    "weak_directional_nis_mean": weak["directional_nis_mean"],
                    "strong_directional_nis_mean": strong["directional_nis_mean"],
                    "weak_over_strong_nis_mean": ratio(
                        weak["directional_nis_mean"],
                        strong["directional_nis_mean"],
                    ),
                    "weak_normalized_abs_error_p95": weak[
                        "normalized_abs_error_p95"
                    ],
                    "strong_normalized_abs_error_p95": strong[
                        "normalized_abs_error_p95"
                    ],
                    "weak_coverage_1_96sigma": weak["coverage_1_96sigma"],
                    "strong_coverage_1_96sigma": strong["coverage_1_96sigma"],
                    "weak_reference_aware_nis_mean": weak[
                        "reference_aware_directional_nis_mean"
                    ],
                    "strong_reference_aware_nis_mean": strong[
                        "reference_aware_directional_nis_mean"
                    ],
                    "weak_reference_aware_coverage_1_96sigma": weak[
                        "reference_aware_coverage_1_96sigma"
                    ],
                    "strong_reference_aware_coverage_1_96sigma": strong[
                        "reference_aware_coverage_1_96sigma"
                    ],
                }
            )
    return summary_rows, comparison_rows, correlation_rows


def build_directional_audit(
    dcreg_rows: Sequence[Dict[str, Any]],
    basis_rows: Sequence[Dict[str, Any]],
    shadow_rows: Sequence[Dict[str, Any]],
    ground_truth_poses: Sequence[Dict[str, float]],
    estimator_poses: Sequence[Dict[str, float]] = (),
    ground_truth_from_estimator_world_yaw: Optional[float] = None,
    ground_truth_horizontal_sigma_m: float = 0.0,
    ground_truth_vertical_sigma_m: float = 0.0,
    ground_truth_relative_endpoint_variance_factor: float = 2.0,
    ground_truth_noise_model_label: str = "",
    max_gt_bracket_sec: float = 0.25,
    thresholds: Sequence[float] = DEFAULT_THRESHOLDS,
) -> Tuple[
    List[Dict[str, Any]],
    List[Dict[str, Any]],
    List[Dict[str, Any]],
    List[Dict[str, Any]],
    Dict[str, Any],
]:
    """Build per-mode audit rows and threshold/calibration summaries."""
    dcreg_by_frame = {
        int(row["frame_id"]): row
        for row in dcreg_rows
        if int(row.get("valid", 0)) == 1
    }
    basis_by_frame = {
        int(row["frame_id"]): row
        for row in basis_rows
    }
    latest_by_edge: Dict[Tuple[int, int], Dict[str, Any]] = {}
    for row in shadow_rows:
        edge = (int(row["from_index"]), int(row["to_index"]))
        prior = latest_by_edge.get(edge)
        if (
            prior is None
            or float(row.get("publish_stamp", -math.inf))
            >= float(prior.get("publish_stamp", -math.inf))
        ):
            latest_by_edge[edge] = row

    orientation_span_rad = 0.0
    if ground_truth_poses:
        first_q = (
            ground_truth_poses[0]["qx"],
            ground_truth_poses[0]["qy"],
            ground_truth_poses[0]["qz"],
            ground_truth_poses[0]["qw"],
        )
        first_q_inv = _q_conjugate(_q_normalize(first_q))
        for pose in ground_truth_poses:
            relative_q = _q_normalize(
                _q_multiply(
                    first_q_inv,
                    (pose["qx"], pose["qy"], pose["qz"], pose["qw"]),
                )
            )
            orientation_span_rad = max(
                orientation_span_rad,
                2.0 * math.acos(max(-1.0, min(1.0, abs(relative_q[3])))),
            )
    position_only_ground_truth = orientation_span_rad <= 1.0e-6
    if position_only_ground_truth:
        position_only_ground_truth = (
            bool(estimator_poses)
            and ground_truth_from_estimator_world_yaw is not None
            and math.isfinite(ground_truth_from_estimator_world_yaw)
        )

    metadata: Dict[str, Any] = {
        "raw_shadow_rows": len(shadow_rows),
        "unique_shadow_edges": len(latest_by_edge),
        "matched_edges": 0,
        "missing_dcreg_edges": 0,
        "missing_basis_edges": 0,
        "missing_ground_truth_edges": 0,
        "missing_estimator_pose_edges": 0,
        "invalid_covariance_modes": 0,
        "audit_rows": 0,
        "max_ground_truth_bracket_sec": max_gt_bracket_sec,
        "residual_convention": (
            "Log(T_glim_from_to^-1*T_gt_from_to), "
            "GTSAM Pose3 local [rx,ry,rz,tx,ty,tz]"
        ),
        "basis_convention": (
            "DCReg Schur eigenvector columns in target-pose local tangent"
        ),
        "edge_revision_policy": "latest published revision per (from_index,to_index)",
        "ground_truth_orientation_span_rad": orientation_span_rad,
        "ground_truth_orientation_available": orientation_span_rad > 1.0e-6,
        "audit_spaces": (
            "translation_only"
            if position_only_ground_truth
            else "rotation_and_translation"
        ),
        "position_only_translation_convention": (
            "SE2-align estimator world to GT world; rotate GT endpoint "
            "displacement into GLIM world and then GLIM from-pose local frame; "
            "reuse measured relative rotation so no rotation residual is claimed"
            if position_only_ground_truth
            else ""
        ),
        "ground_truth_from_estimator_world_yaw_rad": (
            ground_truth_from_estimator_world_yaw
            if ground_truth_from_estimator_world_yaw is not None
            else math.nan
        ),
        "ground_truth_horizontal_sigma_m": ground_truth_horizontal_sigma_m,
        "ground_truth_vertical_sigma_m": ground_truth_vertical_sigma_m,
        "ground_truth_relative_endpoint_variance_factor": (
            ground_truth_relative_endpoint_variance_factor
        ),
        "ground_truth_noise_model_label": ground_truth_noise_model_label,
    }
    audit_rows: List[Dict[str, Any]] = []
    for edge, shadow in sorted(latest_by_edge.items()):
        from_index, to_index = edge
        dcreg = dcreg_by_frame.get(to_index)
        basis = basis_by_frame.get(to_index)
        if dcreg is None:
            metadata["missing_dcreg_edges"] += 1
            continue
        if basis is None:
            metadata["missing_basis_edges"] += 1
            continue
        gt_from, gt_from_gap = interpolate_pose(
            ground_truth_poses,
            float(shadow["from_stamp"]),
            max_gt_bracket_sec,
        )
        gt_to, gt_to_gap = interpolate_pose(
            ground_truth_poses,
            float(shadow["to_stamp"]),
            max_gt_bracket_sec,
        )
        if gt_from is None or gt_to is None:
            metadata["missing_ground_truth_edges"] += 1
            continue
        measured: Pose = (
            (
                float(shadow["measured_tx"]),
                float(shadow["measured_ty"]),
                float(shadow["measured_tz"]),
            ),
            (
                float(shadow["measured_qx"]),
                float(shadow["measured_qy"]),
                float(shadow["measured_qz"]),
                float(shadow["measured_qw"]),
            ),
        )
        if position_only_ground_truth:
            estimator_from, _ = interpolate_pose(
                estimator_poses,
                float(shadow["from_stamp"]),
                max_gt_bracket_sec,
            )
            if estimator_from is None:
                metadata["missing_estimator_pose_edges"] += 1
                continue
            yaw = float(ground_truth_from_estimator_world_yaw)
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            gt_delta = tuple(
                gt_to[0][axis] - gt_from[0][axis] for axis in range(3)
            )
            # p_gt = R_gt_from_glim * p_glim + t, hence a GT displacement is
            # brought back to the GLIM world with R_gt_from_glim^T.
            glim_world_delta = (
                cos_yaw * gt_delta[0] + sin_yaw * gt_delta[1],
                -sin_yaw * gt_delta[0] + cos_yaw * gt_delta[1],
                gt_delta[2],
            )
            truth_translation_from_local = _q_rotate(
                _q_conjugate(_q_normalize(estimator_from[1])),
                glim_world_delta,
            )
            truth = (truth_translation_from_local, measured[1])
        else:
            truth = _pose_between(gt_from, gt_to)
        residual = directional_relative_residual(measured, truth)
        covariance = [
            [
                float(shadow[f"covariance_{row}{col}"])
                for col in range(6)
            ]
            for row in range(6)
        ]
        metadata["matched_edges"] += 1
        spaces = (
            (("translation", 3),)
            if position_only_ground_truth
            else (("rotation", 0), ("translation", 3))
        )
        for space, offset in spaces:
            residual_norm = math.sqrt(
                sum(residual[offset + axis] ** 2 for axis in range(3))
            )
            for mode in range(3):
                vector = _mode_vector(basis, space, mode)
                if vector is None:
                    metadata["invalid_covariance_modes"] += 1
                    continue
                variance = _project_variance(covariance, vector, offset)
                if not math.isfinite(variance) or variance <= 0.0:
                    metadata["invalid_covariance_modes"] += 1
                    continue
                projected_error = sum(
                    vector[axis] * residual[offset + axis]
                    for axis in range(3)
                )
                sigma = math.sqrt(variance)
                normalized_abs = abs(projected_error) / sigma
                reference_variance = 0.0
                if (
                    position_only_ground_truth
                    and ground_truth_horizontal_sigma_m > 0.0
                    and ground_truth_vertical_sigma_m > 0.0
                ):
                    # Move a target-local projection direction back through
                    # target->from, from->GLIM-world, then GLIM-world->GT-world.
                    direction_from = _q_rotate(measured[1], vector)
                    direction_glim_world = _q_rotate(
                        estimator_from[1], direction_from
                    )
                    direction_gt_world = (
                        cos_yaw * direction_glim_world[0]
                        - sin_yaw * direction_glim_world[1],
                        sin_yaw * direction_glim_world[0]
                        + cos_yaw * direction_glim_world[1],
                        direction_glim_world[2],
                    )
                    reference_variance = (
                        ground_truth_relative_endpoint_variance_factor
                        * (
                            ground_truth_horizontal_sigma_m ** 2
                            * (
                                direction_gt_world[0] ** 2
                                + direction_gt_world[1] ** 2
                            )
                            + ground_truth_vertical_sigma_m ** 2
                            * direction_gt_world[2] ** 2
                        )
                    )
                reference_sigma = math.sqrt(max(0.0, reference_variance))
                combined_variance = variance + reference_variance
                combined_sigma = math.sqrt(combined_variance)
                combined_normalized_abs = (
                    abs(projected_error) / combined_sigma
                )
                audit_rows.append(
                    {
                        "publish_stamp": float(shadow["publish_stamp"]),
                        "from_index": from_index,
                        "to_index": to_index,
                        "from_stamp": float(shadow["from_stamp"]),
                        "to_stamp": float(shadow["to_stamp"]),
                        "edge_duration_sec": (
                            float(shadow["to_stamp"])
                            - float(shadow["from_stamp"])
                        ),
                        "space": space,
                        "mode": mode,
                        "spectral_ratio": float(
                            dcreg[f"{space}_spectral_ratio_{mode}"]
                        ),
                        "eigenvalue": float(dcreg[f"{space}_eigenvalue_{mode}"]),
                        "weak_at_threshold_10": int(
                            float(dcreg[f"{space}_spectral_ratio_{mode}"]) > 10.0
                        ),
                        "basis_x": vector[0],
                        "basis_y": vector[1],
                        "basis_z": vector[2],
                        "projected_error": projected_error,
                        "projected_variance": variance,
                        "projected_sigma": sigma,
                        "normalized_abs_error": normalized_abs,
                        "directional_nis": normalized_abs * normalized_abs,
                        "reference_projected_variance": reference_variance,
                        "reference_projected_sigma": reference_sigma,
                        "reference_aware_variance": combined_variance,
                        "reference_aware_sigma": combined_sigma,
                        "reference_aware_normalized_abs_error": (
                            combined_normalized_abs
                        ),
                        "reference_aware_directional_nis": (
                            combined_normalized_abs * combined_normalized_abs
                        ),
                        "subspace_residual_norm": residual_norm,
                        "residual_rx": residual[0],
                        "residual_ry": residual[1],
                        "residual_rz": residual[2],
                        "residual_tx": residual[3],
                        "residual_ty": residual[4],
                        "residual_tz": residual[5],
                        "gt_from_bracket_sec": gt_from_gap,
                        "gt_to_bracket_sec": gt_to_gap,
                        "sender_health_alpha": shadow.get(
                            "sender_health_alpha", math.nan
                        ),
                        "covariance_mode": shadow.get("covariance_mode", ""),
                    }
                )
    metadata["audit_rows"] = len(audit_rows)
    summary_rows, comparison_rows, correlation_rows = summarize_directional_audit(
        audit_rows, thresholds
    )
    return (
        audit_rows,
        summary_rows,
        comparison_rows,
        correlation_rows,
        metadata,
    )
