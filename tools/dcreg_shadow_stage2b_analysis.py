#!/usr/bin/env python3
"""Offline Stage 2B analysis of passive DCReg shadow CSV records.

The online shadow analyzer is ground-truth independent.  This tool optionally
evaluates exact relative edges against a full-pose TUM trajectory only when a
verified fixed transform from the tracked GT body to Kimera's base frame is
provided.  It never fits a health-to-error mapping or a control threshold.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import random
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterable, Optional, Sequence


NAN = float("nan")
HEALTH_LABELS = {
    0: "INVALID",
    1: "HEALTHY",
    2: "DEGRADED",
    3: "ABSOLUTELY_DEGENERATE",
    4: "REFERENCE_UNAVAILABLE",
    5: "METADATA_UNAVAILABLE",
}


def finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def number(row: dict[str, str], key: str, default: float = NAN) -> float:
    value = row.get(key, "")
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def integer(row: dict[str, str], key: str, default: int = 0) -> int:
    value = row.get(key, "")
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def boolean(row: dict[str, str], key: str) -> bool:
    return row.get(key, "").strip().lower() in {"1", "true", "yes", "\x01"}


def median(values: Iterable[float]) -> float:
    clean = [float(v) for v in values if finite(v)]
    return statistics.median(clean) if clean else NAN


def mean(values: Iterable[float]) -> float:
    clean = [float(v) for v in values if finite(v)]
    return statistics.fmean(clean) if clean else NAN


def percentile(values: Iterable[float], q: float) -> float:
    clean = sorted(float(v) for v in values if finite(v))
    if not clean:
        return NAN
    if len(clean) == 1:
        return clean[0]
    position = q * (len(clean) - 1)
    low = int(math.floor(position))
    high = int(math.ceil(position))
    alpha = position - low
    return clean[low] * (1.0 - alpha) + clean[high] * alpha


def numeric_summary(values: Iterable[float]) -> dict[str, Any]:
    clean = [float(v) for v in values if finite(v)]
    if not clean:
        return {"count": 0, "mean": None, "median": None, "p95": None,
                "minimum": None, "maximum": None}
    return {
        "count": len(clean),
        "mean": statistics.fmean(clean),
        "median": statistics.median(clean),
        "p95": percentile(clean, 0.95),
        "minimum": min(clean),
        "maximum": max(clean),
    }


def average_ranks(values: Sequence[float]) -> list[float]:
    order = sorted(range(len(values)), key=lambda i: values[i])
    ranks = [0.0] * len(values)
    begin = 0
    while begin < len(order):
        end = begin + 1
        while end < len(order) and values[order[end]] == values[order[begin]]:
            end += 1
        rank = 0.5 * ((begin + 1) + end)
        for offset in range(begin, end):
            ranks[order[offset]] = rank
        begin = end
    return ranks


def pearson(x: Sequence[float], y: Sequence[float]) -> float:
    if len(x) < 2 or len(x) != len(y):
        return NAN
    mx, my = statistics.fmean(x), statistics.fmean(y)
    xx = sum((v - mx) ** 2 for v in x)
    yy = sum((v - my) ** 2 for v in y)
    if xx <= 0.0 or yy <= 0.0:
        return NAN
    return sum((a - mx) * (b - my) for a, b in zip(x, y)) / math.sqrt(xx * yy)


def spearman(x: Sequence[float], y: Sequence[float]) -> float:
    pairs = [(float(a), float(b)) for a, b in zip(x, y)
             if finite(a) and finite(b)]
    if len(pairs) < 3:
        return NAN
    return pearson(average_ranks([p[0] for p in pairs]),
                   average_ranks([p[1] for p in pairs]))


def bootstrap_spearman(
    x: Sequence[float], y: Sequence[float], samples: int, seed: int
) -> dict[str, Any]:
    pairs = [(float(a), float(b)) for a, b in zip(x, y)
             if finite(a) and finite(b)]
    rho = spearman([p[0] for p in pairs], [p[1] for p in pairs])
    if len(pairs) < 3 or samples <= 0 or not finite(rho):
        return {"n": len(pairs), "rho": None, "ci95": [None, None],
                "bootstrap_samples": 0, "seed": seed}
    rng = random.Random(seed)
    estimates: list[float] = []
    for _ in range(samples):
        draw = [pairs[rng.randrange(len(pairs))] for _ in pairs]
        value = spearman([p[0] for p in draw], [p[1] for p in draw])
        if finite(value):
            estimates.append(value)
    return {
        "n": len(pairs),
        "rho": rho,
        "ci95": [percentile(estimates, 0.025), percentile(estimates, 0.975)],
        "bootstrap_samples": len(estimates),
        "seed": seed,
    }


def eye4() -> list[list[float]]:
    return [[1.0 if r == c else 0.0 for c in range(4)] for r in range(4)]


def matmul(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> list[list[float]]:
    return [[sum(a[r][k] * b[k][c] for k in range(len(b)))
             for c in range(len(b[0]))] for r in range(len(a))]


def transpose3(r: Sequence[Sequence[float]]) -> list[list[float]]:
    return [[r[c][a] for c in range(3)] for a in range(3)]


def pose_inverse(t: Sequence[Sequence[float]]) -> list[list[float]]:
    out = eye4()
    rotation = [row[:3] for row in t[:3]]
    rt = transpose3(rotation)
    for r in range(3):
        for c in range(3):
            out[r][c] = rt[r][c]
        out[r][3] = -sum(rt[r][c] * t[c][3] for c in range(3))
    return out


def skew(v: Sequence[float]) -> list[list[float]]:
    return [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]]


def mat3_add(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> list[list[float]]:
    return [[a[r][c] + b[r][c] for c in range(3)] for r in range(3)]


def mat3_scale(a: Sequence[Sequence[float]], scale: float) -> list[list[float]]:
    return [[scale * a[r][c] for c in range(3)] for r in range(3)]


def mat3_mul(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> list[list[float]]:
    return [[sum(a[r][k] * b[k][c] for k in range(3)) for c in range(3)]
            for r in range(3)]


def mat3_vec(a: Sequence[Sequence[float]], v: Sequence[float]) -> list[float]:
    return [sum(a[r][c] * v[c] for c in range(3)) for r in range(3)]


def so3_exp(w: Sequence[float]) -> list[list[float]]:
    theta = math.sqrt(sum(v * v for v in w))
    wx = skew(w)
    wx2 = mat3_mul(wx, wx)
    identity = [[1.0 if r == c else 0.0 for c in range(3)] for r in range(3)]
    if theta < 1.0e-8:
        a = 1.0 - theta * theta / 6.0
        b = 0.5 - theta * theta / 24.0
    else:
        a = math.sin(theta) / theta
        b = (1.0 - math.cos(theta)) / (theta * theta)
    return mat3_add(identity, mat3_add(mat3_scale(wx, a), mat3_scale(wx2, b)))


def pose_exp(xi: Sequence[float]) -> list[list[float]]:
    w, v = list(xi[:3]), list(xi[3:6])
    theta = math.sqrt(sum(x * x for x in w))
    wx = skew(w)
    wx2 = mat3_mul(wx, wx)
    identity = [[1.0 if r == c else 0.0 for c in range(3)] for r in range(3)]
    if theta < 1.0e-8:
        b = 0.5 - theta * theta / 24.0
        c = 1.0 / 6.0 - theta * theta / 120.0
    else:
        b = (1.0 - math.cos(theta)) / (theta * theta)
        c = (theta - math.sin(theta)) / (theta ** 3)
    jacobian = mat3_add(identity, mat3_add(mat3_scale(wx, b), mat3_scale(wx2, c)))
    rotation = so3_exp(w)
    translation = mat3_vec(jacobian, v)
    out = eye4()
    for r in range(3):
        for col in range(3):
            out[r][col] = rotation[r][col]
        out[r][3] = translation[r]
    return out


def so3_log(rotation: Sequence[Sequence[float]]) -> list[float]:
    cosine = max(-1.0, min(1.0, 0.5 * (sum(rotation[i][i] for i in range(3)) - 1.0)))
    theta = math.acos(cosine)
    vee = [rotation[2][1] - rotation[1][2],
           rotation[0][2] - rotation[2][0],
           rotation[1][0] - rotation[0][1]]
    if theta < 1.0e-8:
        return [0.5 * value for value in vee]
    scale = theta / (2.0 * math.sin(theta))
    return [scale * value for value in vee]


def solve3(a: Sequence[Sequence[float]], b: Sequence[float]) -> list[float]:
    augmented = [list(a[r]) + [float(b[r])] for r in range(3)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(augmented[r][col]))
        if abs(augmented[pivot][col]) < 1.0e-15:
            raise ValueError("singular 3x3 matrix")
        augmented[col], augmented[pivot] = augmented[pivot], augmented[col]
        scale = augmented[col][col]
        augmented[col] = [value / scale for value in augmented[col]]
        for row in range(3):
            if row == col:
                continue
            factor = augmented[row][col]
            augmented[row] = [augmented[row][j] - factor * augmented[col][j]
                              for j in range(4)]
    return [augmented[r][3] for r in range(3)]


def pose_log(transform: Sequence[Sequence[float]]) -> list[float]:
    rotation = [list(row[:3]) for row in transform[:3]]
    w = so3_log(rotation)
    theta = math.sqrt(sum(x * x for x in w))
    wx = skew(w)
    wx2 = mat3_mul(wx, wx)
    identity = [[1.0 if r == c else 0.0 for c in range(3)] for r in range(3)]
    if theta < 1.0e-8:
        b = 0.5 - theta * theta / 24.0
        c = 1.0 / 6.0 - theta * theta / 120.0
    else:
        b = (1.0 - math.cos(theta)) / (theta * theta)
        c = (theta - math.sin(theta)) / (theta ** 3)
    jacobian = mat3_add(identity, mat3_add(mat3_scale(wx, b), mat3_scale(wx2, c)))
    v = solve3(jacobian, [transform[r][3] for r in range(3)])
    return w + v


def quaternion_matrix(qx: float, qy: float, qz: float, qw: float) -> list[list[float]]:
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm <= 0.0:
        raise ValueError("zero quaternion")
    x, y, z, w = qx/norm, qy/norm, qz/norm, qw/norm
    return [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]


def pose_from_xyz_quat(values: Sequence[float]) -> list[list[float]]:
    if len(values) != 7:
        raise ValueError("pose must contain x y z qx qy qz qw")
    out = eye4()
    rotation = quaternion_matrix(*values[3:7])
    for r in range(3):
        for c in range(3):
            out[r][c] = rotation[r][c]
        out[r][3] = values[r]
    return out


def quaternion_slerp(a: Sequence[float], b: Sequence[float], alpha: float) -> list[float]:
    qa, qb = list(a), list(b)
    dot = sum(x*y for x, y in zip(qa, qb))
    if dot < 0.0:
        qb = [-v for v in qb]
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        out = [(1-alpha)*x + alpha*y for x, y in zip(qa, qb)]
    else:
        theta = math.acos(dot)
        scale = math.sin(theta)
        out = [math.sin((1-alpha)*theta)/scale*x + math.sin(alpha*theta)/scale*y
               for x, y in zip(qa, qb)]
    norm = math.sqrt(sum(v*v for v in out))
    return [v/norm for v in out]


@dataclass(frozen=True)
class TimedPose:
    stamp: float
    xyz: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


def load_tum(path: Path) -> list[TimedPose]:
    poses: list[TimedPose] = []
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            fields = line.replace(",", " ").split()
            if not fields or fields[0].startswith("#") or len(fields) < 8:
                continue
            values = [float(value) for value in fields[:8]]
            if not all(finite(value) for value in values):
                continue
            poses.append(TimedPose(values[0], tuple(values[1:4]), tuple(values[4:8])))
    poses.sort(key=lambda pose: pose.stamp)
    return poses


def interpolate_pose(poses: Sequence[TimedPose], stamp: float) -> Optional[list[list[float]]]:
    if not poses or stamp < poses[0].stamp or stamp > poses[-1].stamp:
        return None
    times = [pose.stamp for pose in poses]
    index = bisect.bisect_left(times, stamp)
    if index < len(poses) and abs(poses[index].stamp - stamp) <= 1.0e-9:
        pose = poses[index]
        return pose_from_xyz_quat((*pose.xyz, *pose.quaternion_xyzw))
    if index == 0 or index == len(poses):
        return None
    before, after = poses[index-1], poses[index]
    if after.stamp <= before.stamp:
        return None
    alpha = (stamp-before.stamp)/(after.stamp-before.stamp)
    xyz = tuple((1-alpha)*a + alpha*b for a, b in zip(before.xyz, after.xyz))
    quat = quaternion_slerp(before.quaternion_xyzw, after.quaternion_xyzw, alpha)
    return pose_from_xyz_quat((*xyz, *quat))


def load_verified_extrinsic(path: Optional[Path]) -> tuple[Optional[list[list[float]]], str]:
    if path is None:
        return None, "no verified GT-body-to-Kimera-base extrinsic supplied"
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("status") != "verified":
        return None, "GT body extrinsic is not marked status=verified"
    values = payload.get("gt_body_T_kimera_base_xyz_xyzw")
    if not isinstance(values, list) or len(values) != 7:
        return None, "verified extrinsic lacks gt_body_T_kimera_base_xyz_xyzw"
    return pose_from_xyz_quat([float(v) for v in values]), "verified"


def read_shadow_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", errors="replace", newline="") as stream:
        return [dict(row) for row in csv.DictReader(stream)]


def six(row: dict[str, str], prefix_r: str, prefix_t: str) -> list[float]:
    return [number(row, f"{prefix_r}{i}") for i in range(3)] + [
        number(row, f"{prefix_t}{i}") for i in range(3)]


def enrich_with_ground_truth(
    rows: list[dict[str, Any]], gt: Sequence[TimedPose], gt_body_T_base: Sequence[Sequence[float]]
) -> int:
    matched = 0
    for row in rows:
        if (not row.get("metadata_match", True)
                or not row.get("receiver_match", True)
                or not all(finite(value) for value in row["sender_mu"])
                or not all(finite(value) for value in row["receiver_mu"])):
            row["gt_available"] = False
            continue
        first = interpolate_pose(gt, float(row["from_stamp_sec"]))
        second = interpolate_pose(gt, float(row["to_stamp_sec"]))
        if first is None or second is None:
            row["gt_available"] = False
            continue
        world_T_base_first = matmul(first, gt_body_T_base)
        world_T_base_second = matmul(second, gt_body_T_base)
        gt_relative = matmul(pose_inverse(world_T_base_first), world_T_base_second)
        sender = pose_exp(row["sender_mu"])
        receiver = pose_exp(row["receiver_mu"])
        sender_error = pose_log(matmul(pose_inverse(gt_relative), sender))
        receiver_error = pose_log(matmul(pose_inverse(gt_relative), receiver))
        row.update({
            "gt_available": True,
            "sender_rot_error_rad": math.sqrt(sum(v*v for v in sender_error[:3])),
            "sender_trans_error_m": math.sqrt(sum(v*v for v in sender_error[3:])),
            "receiver_rot_error_rad": math.sqrt(sum(v*v for v in receiver_error[:3])),
            "receiver_trans_error_m": math.sqrt(sum(v*v for v in receiver_error[3:])),
        })
        matched += 1
    return matched


def correlation(rows: Sequence[dict[str, Any]], x: str, y: str,
                samples: int, seed: int) -> dict[str, Any]:
    return bootstrap_spearman([row.get(x, NAN) for row in rows],
                              [row.get(y, NAN) for row in rows], samples, seed)


def correlation_suite(rows: Sequence[dict[str, Any]], ground_truth_available: bool,
                      samples: int, seed: int) -> dict[str, Any]:
    pairs = [
        ("rot_health_vs_rot_disagreement", "rot_health_min", "rot_disagreement_rad"),
        ("trans_health_vs_trans_disagreement", "trans_health_min", "trans_disagreement_m"),
        ("rot_degraded_occupancy_vs_rot_disagreement", "rot_degraded_fraction", "rot_disagreement_rad"),
        ("trans_degraded_occupancy_vs_trans_disagreement", "trans_degraded_fraction", "trans_disagreement_m"),
        ("rot_absolute_occupancy_vs_rot_disagreement", "rot_absolute_fraction", "rot_disagreement_rad"),
        ("trans_absolute_occupancy_vs_trans_disagreement", "trans_absolute_fraction", "trans_disagreement_m"),
        ("inlier_fraction_vs_trans_disagreement", "inlier_fraction_mean", "trans_disagreement_m"),
        ("cost_reduction_vs_trans_disagreement", "cost_reduction_mean", "trans_disagreement_m"),
    ]
    if ground_truth_available:
        pairs.extend([
            ("rot_health_vs_sender_rot_error", "rot_health_min", "sender_rot_error_rad"),
            ("trans_health_vs_sender_trans_error", "trans_health_min", "sender_trans_error_m"),
            ("rot_degraded_occupancy_vs_sender_rot_error", "rot_degraded_fraction", "sender_rot_error_rad"),
            ("trans_degraded_occupancy_vs_sender_trans_error", "trans_degraded_fraction", "sender_trans_error_m"),
            ("rot_absolute_occupancy_vs_sender_rot_error", "rot_absolute_fraction", "sender_rot_error_rad"),
            ("trans_absolute_occupancy_vs_sender_trans_error", "trans_absolute_fraction", "sender_trans_error_m"),
        ])
    return {
        label: correlation(rows, x, y, samples, seed + offset)
        for offset, (label, x, y) in enumerate(pairs)
    }


def svg_timeline(path: Path, rows: Sequence[dict[str, Any]]) -> None:
    points = [row for row in rows if finite(row.get("to_stamp_sec"))]
    width, height, margin = 1400, 720, 70
    if not points:
        path.write_text("<svg xmlns='http://www.w3.org/2000/svg'/>", encoding="utf-8")
        return
    t0, t1 = min(p["to_stamp_sec"] for p in points), max(p["to_stamp_sec"] for p in points)
    if t1 <= t0:
        t1 = t0 + 1.0
    series = [
        ("rot health", "rot_health_min", "#1f77b4", 0.0, 1.0),
        ("trans health", "trans_health_min", "#2ca02c", 0.0, 1.0),
        ("rot disagreement (rad)", "rot_disagreement_rad", "#d62728", 0.0, None),
        ("trans disagreement (m)", "trans_disagreement_m", "#9467bd", 0.0, None),
    ]
    chunks = [f"<svg xmlns='http://www.w3.org/2000/svg' width='{width}' height='{height}'>",
              "<rect width='100%' height='100%' fill='white'/>",
              "<style>text{font-family:sans-serif;font-size:13px}</style>"]
    panel_h = (height - 2*margin) / len(series)
    for panel, (label, key, color, ymin, fixed_max) in enumerate(series):
        values = [float(p[key]) for p in points if finite(p.get(key))]
        ymax = fixed_max if fixed_max is not None else (percentile(values, 0.99) if values else 1.0)
        ymax = max(ymin + 1.0e-12, ymax)
        top = margin + panel*panel_h
        bottom = top + panel_h - 20
        chunks.append(f"<line x1='{margin}' y1='{bottom}' x2='{width-margin}' y2='{bottom}' stroke='#aaa'/>")
        chunks.append(f"<text x='8' y='{top+15}'>{label}</text>")
        coordinates = []
        for p in points:
            value = p.get(key, NAN)
            if not finite(value):
                continue
            x = margin + (p["to_stamp_sec"]-t0)/(t1-t0)*(width-2*margin)
            clipped = max(ymin, min(ymax, float(value)))
            y = bottom - (clipped-ymin)/(ymax-ymin)*(panel_h-30)
            coordinates.append(f"{x:.2f},{y:.2f}")
        if coordinates:
            chunks.append(f"<polyline fill='none' stroke='{color}' stroke-width='1.5' points='{' '.join(coordinates)}'/>")
    chunks.append(f"<text x='{margin}' y='{height-15}'>sensor time {t0:.3f} to {t1:.3f} s</text></svg>")
    path.write_text("\n".join(chunks), encoding="utf-8")


def write_rerun(path: Path, rows: Sequence[dict[str, Any]], title: str) -> tuple[bool, str]:
    try:
        import rerun as rr  # type: ignore
    except ImportError as error:
        return False, f"rerun SDK unavailable: {error}"
    rr.init("cbsms_dcreg_stage2b_shadow", recording_id=title)
    rr.save(str(path))
    rr.log("summary/title", rr.TextDocument(f"# {title}\nPassive Stage 2B shadow analysis."), static=True)
    for row in rows:
        stamp = row.get("to_stamp_sec", NAN)
        if not finite(stamp):
            continue
        rr.set_time("sensor_time", timestamp=float(stamp))
        for entity, key in [
            ("health/rotation_min", "rot_health_min"),
            ("health/translation_min", "trans_health_min"),
            ("health/rotation_degraded_fraction", "rot_degraded_fraction"),
            ("health/translation_degraded_fraction", "trans_degraded_fraction"),
            ("absolute/rotation_fraction", "rot_absolute_fraction"),
            ("absolute/translation_fraction", "trans_absolute_fraction"),
            ("disagreement/rotation_rad", "rot_disagreement_rad"),
            ("disagreement/translation_m", "trans_disagreement_m"),
        ]:
            value = row.get(key, NAN)
            if finite(value):
                rr.log(entity, rr.Scalars(float(value)))
        if row.get("gt_available"):
            rr.log("ground_truth_error/sender_rotation_rad", rr.Scalars(row["sender_rot_error_rad"]))
            rr.log("ground_truth_error/sender_translation_m", rr.Scalars(row["sender_trans_error_m"]))
            rr.log("ground_truth_error/receiver_rotation_rad", rr.Scalars(row["receiver_rot_error_rad"]))
            rr.log("ground_truth_error/receiver_translation_m", rr.Scalars(row["receiver_trans_error_m"]))
    return True, str(path)


def analyze(args: argparse.Namespace) -> dict[str, Any]:
    raw = read_shadow_csv(args.shadow_csv)
    rows: list[dict[str, Any]] = []
    for source in raw:
        sender_mu = six(source, "sender_r", "sender_t")
        receiver_mu = six(source, "receiver_r", "receiver_t")
        row: dict[str, Any] = {
            "from_index": integer(source, "from_index"),
            "to_index": integer(source, "to_index"),
            "from_stamp_sec": number(source, "from_stamp_sec"),
            "to_stamp_sec": number(source, "to_stamp_sec"),
            "metadata_match": boolean(source, "metadata_match"),
            "receiver_match": boolean(source, "receiver_match"),
            "active_factor_accepted": boolean(source, "active_factor_accepted"),
            "health_state": integer(source, "health_state"),
            "health_state_label": HEALTH_LABELS.get(integer(source, "health_state"), "UNKNOWN"),
            "reference_ready": boolean(source, "reference_ready"),
            "relative_health_valid": boolean(source, "relative_health_valid"),
            "rot_health_min": number(source, "rot_health_min"),
            "trans_health_min": number(source, "trans_health_min"),
            "rot_degraded_fraction": number(source, "rot_degraded_fraction"),
            "trans_degraded_fraction": number(source, "trans_degraded_fraction"),
            "rot_absolute_fraction": number(source, "rot_absolute_fraction"),
            "trans_absolute_fraction": number(source, "trans_absolute_fraction"),
            "inlier_fraction_mean": number(source, "inlier_fraction_mean"),
            "cost_reduction_mean": number(source, "cost_reduction_mean"),
            "rot_disagreement_rad": number(source, "rot_disagreement_rad"),
            "trans_disagreement_m": number(source, "trans_disagreement_m"),
            "start_dt_sec": number(source, "start_dt_sec"),
            "end_dt_sec": number(source, "end_dt_sec"),
            "sender_mu": sender_mu,
            "receiver_mu": receiver_mu,
            "stable_edge_digest": source.get("stable_edge_digest", ""),
            "payload_digest": source.get("payload_digest", ""),
            "gt_available": False,
        }
        rows.append(row)

    gt_status = {"available": False, "reason": "not requested", "matched_edges": 0}
    if args.ground_truth:
        extrinsic, reason = load_verified_extrinsic(args.gt_body_extrinsic)
        if args.ground_truth_kind != "full_pose":
            gt_status["reason"] = "ground truth is not a verified full 6-DoF pose source"
        elif extrinsic is None:
            gt_status["reason"] = reason
        else:
            gt = load_tum(args.ground_truth)
            matched = enrich_with_ground_truth(rows, gt, extrinsic)
            gt_status = {"available": matched > 0, "reason": "verified",
                         "matched_edges": matched, "pose_count": len(gt)}

    valid_disagreement = [r for r in rows if r["metadata_match"] and r["receiver_match"]
                          and finite(r["rot_disagreement_rad"])
                          and finite(r["trans_disagreement_m"])]
    active_disagreement = [
        row for row in valid_disagreement if row["active_factor_accepted"]
    ]
    correlations = correlation_suite(
        active_disagreement,
        gt_status["available"],
        args.bootstrap_samples,
        args.bootstrap_seed,
    )
    all_matched_correlations = correlation_suite(
        valid_disagreement,
        gt_status["available"],
        args.bootstrap_samples,
        args.bootstrap_seed + 1000,
    )

    label_counts: dict[str, int] = {}
    active_label_counts: dict[str, int] = {}
    stratified: dict[str, Any] = {}
    for label in HEALTH_LABELS.values():
        selected = [r for r in rows if r["health_state_label"] == label]
        if not selected:
            continue
        label_counts[label] = len(selected)
        active_selected = [
            row for row in active_disagreement
            if row["health_state_label"] == label
        ]
        if active_selected:
            active_label_counts[label] = len(active_selected)
        stratified[label] = {
            "all_exact_receiver_matches": {
                "rotational_disagreement_rad": numeric_summary(r["rot_disagreement_rad"] for r in selected),
                "translational_disagreement_m": numeric_summary(r["trans_disagreement_m"] for r in selected),
            },
            "active_accepted_factors": {
                "count": len(active_selected),
                "rotational_disagreement_rad": numeric_summary(r["rot_disagreement_rad"] for r in active_selected),
                "translational_disagreement_m": numeric_summary(r["trans_disagreement_m"] for r in active_selected),
                "sender_rotational_edge_error_rad": numeric_summary(r.get("sender_rot_error_rad", NAN) for r in active_selected),
                "sender_translational_edge_error_m": numeric_summary(r.get("sender_trans_error_m", NAN) for r in active_selected),
                "receiver_rotational_edge_error_rad": numeric_summary(r.get("receiver_rot_error_rad", NAN) for r in active_selected),
                "receiver_translational_edge_error_m": numeric_summary(r.get("receiver_trans_error_m", NAN) for r in active_selected),
            },
        }

    summary: dict[str, Any] = {
        "schema_version": 1,
        "semantics": "passive descriptive Stage 2B; no control or calibration",
        "dataset": args.dataset,
        "source_csv": str(args.shadow_csv),
        "rows": len(rows),
        "metadata_matches": sum(r["metadata_match"] for r in rows),
        "receiver_matches": sum(r["receiver_match"] for r in rows),
        "active_factor_accepted": sum(r["active_factor_accepted"] for r in rows),
        "raw_disagreement_available": len(valid_disagreement),
        "active_accepted_disagreement_available": len(active_disagreement),
        "health_state_counts_all_belief_instances": label_counts,
        "health_state_counts_active_accepted_factors": active_label_counts,
        "endpoint_timestamp_error_sec": {
            "start": numeric_summary(r["start_dt_sec"] for r in rows),
            "end": numeric_summary(r["end_dt_sec"] for r in rows),
        },
        "rotational_disagreement_rad": numeric_summary(r["rot_disagreement_rad"] for r in rows),
        "translational_disagreement_m": numeric_summary(r["trans_disagreement_m"] for r in rows),
        "active_accepted_rotational_disagreement_rad": numeric_summary(
            r["rot_disagreement_rad"] for r in active_disagreement),
        "active_accepted_translational_disagreement_m": numeric_summary(
            r["trans_disagreement_m"] for r in active_disagreement),
        "ground_truth": gt_status,
        "primary_active_accepted_correlations": correlations,
        "supplementary_all_matched_correlations": all_matched_correlations,
        "stratified": stratified,
        "limitations": [
            "Spearman association is descriptive and does not establish causality.",
            "Primary correlations use only beliefs accepted into the active factor stream; rolling-array republications and superseded candidates are reported separately.",
            "Belief intervals remain temporally correlated, so row-bootstrap intervals are descriptive rather than independent-sample guarantees.",
            "Health is not covariance, probability, confidence, or metric error.",
            "No classification threshold or health-to-error mapping is fitted.",
            "Ground-truth edge errors are unavailable unless a verified fixed GT-body-to-Kimera-base transform is supplied.",
        ],
    }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    edge_path = args.output_dir / "shadow_edges_enriched.csv"
    fields = [key for key in rows[0] if key not in {"sender_mu", "receiver_mu"}] if rows else []
    with edge_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key) for key in fields})
    svg_timeline(args.output_dir / "shadow_timeline.svg", rows)
    if args.rerun_rrd:
        made, detail = write_rerun(args.rerun_rrd, rows, f"Stage 2B {args.dataset}")
        summary["rerun"] = {"written": made, "detail": detail}
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8")
    lines = [
        f"# Stage 2B shadow analysis: {args.dataset}", "",
        "This is passive descriptive analysis. It does not calibrate health or control CBS.", "",
        f"- Rows: {len(rows)}",
        f"- Exact belief–metadata matches: {summary['metadata_matches']}",
        f"- Exact receiver-state matches: {summary['receiver_matches']}",
        f"- Raw disagreements: {summary['raw_disagreement_available']}",
        f"- Active accepted-factor disagreements: {summary['active_accepted_disagreement_available']}",
        f"- Ground-truth edge evaluation: {gt_status['available']} ({gt_status['reason']})", "",
        "## Health-state counts", "",
    ]
    lines.extend(f"- {key}: {value}" for key, value in label_counts.items())
    lines.extend(["", "## Primary active-accepted Spearman correlations (95% fixed-seed row-bootstrap CI)", ""])
    for key, value in correlations.items():
        lines.append(f"- {key}: rho={value['rho']}, CI={value['ci95']}, n={value['n']}")
    lines.extend(["", "## Scientific boundary", "",
                  "No covariance scaling, trust weight, accept/reject decision, or metric-error mapping is produced."])
    (args.output_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return summary


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--shadow-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--ground-truth", type=Path)
    parser.add_argument("--ground-truth-kind", choices=("full_pose", "position_only"),
                        default="full_pose")
    parser.add_argument("--gt-body-extrinsic", type=Path)
    parser.add_argument("--bootstrap-samples", type=int, default=2000)
    parser.add_argument("--bootstrap-seed", type=int, default=20260805)
    parser.add_argument("--rerun-rrd", type=Path)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    analyze(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
