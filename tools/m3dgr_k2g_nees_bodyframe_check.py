#!/usr/bin/env python3
"""Check M3DGR K2G relative covariance consistency with GT body-frame handling.

This diagnostic is intentionally independent from GTSAM Python bindings.  It
uses the TUM trajectories and CBS covariance matrix dumps produced by
``cbsms_experiment.py``.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import re
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


PoseRow = Tuple[float, np.ndarray, np.ndarray]  # time, xyz, quaternion wxyz
RelRow = Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]


def quantile(values: Sequence[float], probability: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = (len(ordered) - 1) * probability
    lo = int(math.floor(index))
    hi = int(math.ceil(index))
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - index) + ordered[hi] * (index - lo)


def load_tum(path: Path) -> List[PoseRow]:
    rows: List[PoseRow] = []
    with path.open() as handle:
        for line in handle:
            if not line.strip():
                continue
            parts = line.split()
            xyz = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
            quat = np.array(
                [float(parts[7]), float(parts[4]), float(parts[5]), float(parts[6])]
            )
            rows.append((float(parts[0]), xyz, quat / np.linalg.norm(quat)))
    return rows


def load_raw_gt_times(path: Path) -> List[float]:
    times: List[float] = []
    with path.open() as handle:
        for line in handle:
            if line.strip():
                times.append(float(line.split()[0]))
    return sorted(times)


def quaternion_to_rotation(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = quat
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def rotation_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        major = int(np.argmax([rotation[0, 0], rotation[1, 1], rotation[2, 2]]))
        if major == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            w = (rotation[2, 1] - rotation[1, 2]) / scale
            x = 0.25 * scale
            y = (rotation[0, 1] + rotation[1, 0]) / scale
            z = (rotation[0, 2] + rotation[2, 0]) / scale
        elif major == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            w = (rotation[0, 2] - rotation[2, 0]) / scale
            x = (rotation[0, 1] + rotation[1, 0]) / scale
            y = 0.25 * scale
            z = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            w = (rotation[1, 0] - rotation[0, 1]) / scale
            x = (rotation[0, 2] + rotation[2, 0]) / scale
            y = (rotation[1, 2] + rotation[2, 1]) / scale
            z = 0.25 * scale
    quat = np.array([w, x, y, z])
    quat = quat / np.linalg.norm(quat)
    return -quat if quat[0] < 0.0 else quat


def left_quaternion_matrix(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = quat
    return np.array(
        [[w, -x, -y, -z], [x, w, -z, y], [y, z, w, -x], [z, -y, x, w]]
    )


def right_quaternion_matrix(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = quat
    return np.array(
        [[w, -x, -y, -z], [x, w, z, -y], [y, -z, w, x], [z, y, -x, w]]
    )


def relative_pose(first: PoseRow, second: PoseRow) -> Tuple[np.ndarray, np.ndarray]:
    rotation_a = quaternion_to_rotation(first[2])
    rotation_b = quaternion_to_rotation(second[2])
    return rotation_a.T @ rotation_b, rotation_a.T @ (second[1] - first[1])


def rotation_angle(rotation: np.ndarray) -> float:
    cosine = max(-1.0, min(1.0, float((np.trace(rotation) - 1.0) * 0.5)))
    return math.acos(cosine)


def estimate_body_transform(
    kimera: Sequence[PoseRow],
    gt: Sequence[PoseRow],
    *,
    min_index: int = 0,
    max_index: Optional[int] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    if max_index is None:
        max_index = min(len(kimera), len(gt)) - 1
    rels: List[RelRow] = []
    for separation in (1, 2, 4, 5, 10, 15, 20):
        step = max(1, separation // 2)
        last_start = min(len(kimera) - separation, max_index - separation + 1)
        for index in range(min_index, last_start, step):
            rotation_gt, translation_gt = relative_pose(gt[index], gt[index + separation])
            rotation_kimera, translation_kimera = relative_pose(
                kimera[index], kimera[index + separation]
            )
            moving = (
                np.linalg.norm(translation_gt) > 0.005
                or rotation_angle(rotation_gt) > math.radians(0.2)
            )
            if moving:
                rels.append((rotation_gt, translation_gt, rotation_kimera, translation_kimera))
    if len(rels) < 8:
        raise RuntimeError(
            f"not enough relative motions to estimate body transform "
            f"in index range [{min_index}, {max_index}]"
        )

    system_blocks = []
    for rotation_gt, _, rotation_kimera, _ in rels:
        quat_gt = rotation_to_quaternion(rotation_gt)
        quat_kimera = rotation_to_quaternion(rotation_kimera)
        system_blocks.append(left_quaternion_matrix(quat_gt) - right_quaternion_matrix(quat_kimera))
    _, _, vh = np.linalg.svd(np.vstack(system_blocks))
    quat_body = vh[-1]
    quat_body = quat_body / np.linalg.norm(quat_body)
    if quat_body[0] < 0.0:
        quat_body = -quat_body
    rotation_body = quaternion_to_rotation(quat_body)

    lhs_blocks = []
    rhs_blocks = []
    identity = np.eye(3)
    for rotation_gt, translation_gt, _, translation_kimera in rels:
        lhs_blocks.append(rotation_gt - identity)
        rhs_blocks.append(rotation_body @ translation_kimera - translation_gt)
    translation_body, _, _, _ = np.linalg.lstsq(
        np.vstack(lhs_blocks), np.concatenate(rhs_blocks), rcond=None
    )
    return rotation_body, translation_body


def transform_gt_relative(
    rotation_gt: np.ndarray,
    translation_gt: np.ndarray,
    rotation_body: np.ndarray,
    translation_body: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    rotation = rotation_body.T @ rotation_gt @ rotation_body
    translation = rotation_body.T @ (
        rotation_gt @ translation_body + translation_gt - translation_body
    )
    return rotation, translation


def log_so3(rotation: np.ndarray) -> np.ndarray:
    theta = rotation_angle(rotation)
    vee = np.array(
        [
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ]
    )
    if theta < 1e-9:
        return 0.5 * vee
    return theta / (2.0 * math.sin(theta)) * vee


def hat(vector: np.ndarray) -> np.ndarray:
    return np.array(
        [[0.0, -vector[2], vector[1]], [vector[2], 0.0, -vector[0]], [-vector[1], vector[0], 0.0]]
    )


def se3_log(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    omega = log_so3(rotation)
    theta = float(np.linalg.norm(omega))
    omega_hat = hat(omega)
    identity = np.eye(3)
    if theta < 1e-9:
        v_inv = identity - 0.5 * omega_hat + (omega_hat @ omega_hat) / 12.0
    else:
        coeff = 1.0 / (theta * theta) - (1.0 + math.cos(theta)) / (
            2.0 * theta * math.sin(theta)
        )
        v_inv = identity - 0.5 * omega_hat + coeff * (omega_hat @ omega_hat)
    return np.concatenate([omega, v_inv @ translation])


def parse_matrix(values: str) -> np.ndarray:
    return np.array([[float(value) for value in row.split(":")] for row in values.split(";")])


def pose_index(key: str) -> int:
    match = re.search(r"(\d+)$", key)
    if not match:
        raise ValueError(f"cannot parse pose index from key {key!r}")
    return int(match.group(1))


def latest_direction_covariance_rows(run_dir: Path, direction: str) -> List[Dict[str, str]]:
    path = run_dir / "parsed" / "cbs_odom_relative_covariance_matrices.csv"
    latest: Dict[Tuple[str, str], Tuple[int, Dict[str, str]]] = {}
    with path.open() as handle:
        for row in csv.DictReader(handle):
            if row["direction"] != direction:
                continue
            if row["matrix_label"] != "sigma_rel_schur_covariance_6x6":
                continue
            pair = (row["from_key"], row["to_key"])
            sample_index = int(row["sample_index"])
            if pair not in latest or sample_index >= latest[pair][0]:
                latest[pair] = (sample_index, row)
    return [value[1] for value in latest.values()]


def quadratic_form(covariance: np.ndarray, residual: np.ndarray) -> float:
    try:
        return float(residual.T @ np.linalg.solve(covariance, residual))
    except np.linalg.LinAlgError:
        return float(residual.T @ np.linalg.pinv(covariance) @ residual)


def residual_and_nees(
    rotation_kimera: np.ndarray,
    translation_kimera: np.ndarray,
    rotation_gt: np.ndarray,
    translation_gt: np.ndarray,
    covariance: np.ndarray,
) -> Tuple[float, float, float, float, float]:
    residual_rotation = rotation_kimera.T @ rotation_gt
    residual_translation = rotation_kimera.T @ (translation_gt - translation_kimera)
    residual = se3_log(residual_rotation, residual_translation)
    nees = quadratic_form(covariance, residual)
    rot_nees = quadratic_form(covariance[:3, :3], residual[:3])
    trans_nees = quadratic_form(covariance[3:, 3:], residual[3:])
    return (
        math.degrees(rotation_angle(residual_rotation)),
        float(np.linalg.norm(residual_translation)),
        nees,
        rot_nees,
        trans_nees,
    )


def summarize_rows(
    rows: Iterable[Dict[str, str]],
    kimera: Sequence[PoseRow],
    gt: Sequence[PoseRow],
    rotation_body: np.ndarray,
    translation_body: np.ndarray,
) -> Dict[str, float]:
    identity_nees: List[float] = []
    body_nees: List[float] = []
    body_rot_nees: List[float] = []
    body_trans_nees: List[float] = []
    body_rot_deg: List[float] = []
    body_trans_m: List[float] = []
    traces: List[float] = []
    rot_traces: List[float] = []
    trans_traces: List[float] = []
    one_axis_rot_sigmas_deg: List[float] = []
    one_axis_trans_sigmas_cm: List[float] = []
    rows_used = 0
    for row in rows:
        from_index = pose_index(row["from_key"])
        to_index = pose_index(row["to_key"])
        if max(from_index, to_index) >= min(len(kimera), len(gt)):
            continue
        covariance = parse_matrix(row["matrix_values"])
        trace_rot = float(np.trace(covariance[:3, :3]))
        trace_trans = float(np.trace(covariance[3:, 3:]))
        rotation_kimera, translation_kimera = relative_pose(kimera[from_index], kimera[to_index])
        rotation_gt, translation_gt = relative_pose(gt[from_index], gt[to_index])
        traces.append(float(np.trace(covariance)))
        rot_traces.append(trace_rot)
        trans_traces.append(trace_trans)
        one_axis_rot_sigmas_deg.append(math.degrees(math.sqrt(max(trace_rot, 0.0) / 3.0)))
        one_axis_trans_sigmas_cm.append(100.0 * math.sqrt(max(trace_trans, 0.0) / 3.0))

        _, _, nees_identity, _, _ = residual_and_nees(
            rotation_kimera, translation_kimera, rotation_gt, translation_gt, covariance
        )
        rotation_body_gt, translation_body_gt = transform_gt_relative(
            rotation_gt, translation_gt, rotation_body, translation_body
        )
        rot_deg, trans_m, nees_body, nees_rot, nees_trans = residual_and_nees(
            rotation_kimera,
            translation_kimera,
            rotation_body_gt,
            translation_body_gt,
            covariance,
        )
        identity_nees.append(nees_identity)
        body_nees.append(nees_body)
        body_rot_nees.append(nees_rot)
        body_trans_nees.append(nees_trans)
        body_rot_deg.append(rot_deg)
        body_trans_m.append(trans_m)
        rows_used += 1
    if rows_used == 0:
        return {"rows": 0}
    return {
        "rows": float(rows_used),
        "trace_p50": quantile(traces, 0.5),
        "trace_rot_p50": quantile(rot_traces, 0.5),
        "trace_trans_p50": quantile(trans_traces, 0.5),
        "one_axis_rot_sigma_p50_deg": quantile(one_axis_rot_sigmas_deg, 0.5),
        "one_axis_trans_sigma_p50_cm": quantile(one_axis_trans_sigmas_cm, 0.5),
        "identity_nees_p50": quantile(identity_nees, 0.5),
        "identity_nees_mean": float(np.mean(identity_nees)),
        "body_nees_p50": quantile(body_nees, 0.5),
        "body_nees_mean": float(np.mean(body_nees)),
        "body_rot_nees_p50": quantile(body_rot_nees, 0.5),
        "body_rot_nees_mean": float(np.mean(body_rot_nees)),
        "body_trans_nees_p50": quantile(body_trans_nees, 0.5),
        "body_trans_nees_mean": float(np.mean(body_trans_nees)),
        "body_scale_p50_to_6d": quantile(body_nees, 0.5) / 6.0,
        "body_rot_scale_p50_to_3d": quantile(body_rot_nees, 0.5) / 3.0,
        "body_trans_scale_p50_to_3d": quantile(body_trans_nees, 0.5) / 3.0,
        "body_rot_p50_deg": quantile(body_rot_deg, 0.5),
        "body_rot_p95_deg": quantile(body_rot_deg, 0.95),
        "body_trans_p50_cm": 100.0 * quantile(body_trans_m, 0.5),
        "body_trans_p95_cm": 100.0 * quantile(body_trans_m, 0.95),
    }


def timestamp_summary(raw_gt: Optional[Path], kimera: Sequence[PoseRow]) -> str:
    if raw_gt is None:
        return ""
    raw_times = load_raw_gt_times(raw_gt)
    gaps: List[float] = []
    for stamp, _, _ in kimera:
        index = bisect.bisect_left(raw_times, stamp)
        candidates = []
        if index > 0:
            candidates.append(abs(stamp - raw_times[index - 1]))
        if index < len(raw_times):
            candidates.append(abs(stamp - raw_times[index]))
        if candidates:
            gaps.append(min(candidates))
    return (
        f"nearest raw GT dt p50/p95/max ms: "
        f"{1000.0 * quantile(gaps, 0.5):.3f}/"
        f"{1000.0 * quantile(gaps, 0.95):.3f}/"
        f"{1000.0 * max(gaps):.3f}"
    )


def print_stats(prefix: str, stats: Dict[str, float]) -> None:
    if stats.get("rows", 0) <= 0:
        print(f"  {prefix}: no covariance rows")
        return
    print(
        f"  {prefix}: rows={int(stats['rows'])} "
        f"trace_p50={stats['trace_p50']:.3e} "
        f"trace_rot/trans_p50={stats['trace_rot_p50']:.3e}/{stats['trace_trans_p50']:.3e} "
        f"identity_NEES_p50/mean={stats['identity_nees_p50']:.1f}/{stats['identity_nees_mean']:.1f} "
        f"bodyfit_NEES_p50/mean={stats['body_nees_p50']:.1f}/{stats['body_nees_mean']:.1f}"
    )
    print(
        f"    bodyfit residual rot p50/p95 deg="
        f"{stats['body_rot_p50_deg']:.3f}/{stats['body_rot_p95_deg']:.3f}, "
        f"trans p50/p95 cm={stats['body_trans_p50_cm']:.2f}/"
        f"{stats['body_trans_p95_cm']:.2f}"
    )
    print(
        f"    bodyfit component NEES rot p50/mean="
        f"{stats['body_rot_nees_p50']:.1f}/{stats['body_rot_nees_mean']:.1f}, "
        f"trans p50/mean={stats['body_trans_nees_p50']:.1f}/"
        f"{stats['body_trans_nees_mean']:.1f}"
    )
    print(
        f"    one-axis sigma p50 rot/trans="
        f"{stats['one_axis_rot_sigma_p50_deg']:.4f} deg/"
        f"{stats['one_axis_trans_sigma_p50_cm']:.3f} cm, "
        f"rough scale full/rot/trans="
        f"{stats['body_scale_p50_to_6d']:.1f}/"
        f"{stats['body_rot_scale_p50_to_3d']:.1f}/"
        f"{stats['body_trans_scale_p50_to_3d']:.1f}"
    )


def load_body_transform(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    with path.open() as handle:
        data = json.load(handle)
    quat = np.array([float(value) for value in data["q_wxyz"]])
    translation = np.array([float(value) for value in data["t_xyz_m"]])
    return quaternion_to_rotation(quat / np.linalg.norm(quat)), translation


def save_body_transform(
    path: Path,
    *,
    run_dir: Path,
    rotation_body: np.ndarray,
    translation_body: np.ndarray,
    min_observed: int,
    max_observed: int,
    split_index: int,
    train_fraction: float,
    train_stats: Dict[str, float],
    validation_stats: Dict[str, float],
    all_stats: Dict[str, float],
    estimator: str,
    direction: str,
) -> None:
    quat_body = rotation_to_quaternion(rotation_body)
    payload = {
        "convention": (
            f"T_vrpnBody_{estimator}Frame, "
            f"T_W_{estimator} = T_W_vrpnBody * T_vrpnBody_{estimator}Frame"
        ),
        "warning": (
            "This is an empirical diagnostic alignment fitted from Kimera/GT relative "
            "motions. It is not an official M3DGR calibration unless independently "
            "validated against the sensor/marker rigid body definition."
        ),
        "source_run": str(run_dir),
        "estimator": estimator,
        "direction": direction,
        "observed_key_range": [int(min_observed), int(max_observed)],
        "train_split_key": int(split_index),
        "train_fraction": float(train_fraction),
        "q_wxyz": [float(value) for value in quat_body],
        "t_xyz_m": [float(value) for value in translation_body],
        "stats": {
            "train": train_stats,
            "validation": validation_stats,
            "all": all_stats,
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def analyze_run(
    run_dir: Path,
    raw_gt: Optional[Path],
    train_fraction: float,
    body_transform_path: Optional[Path],
    save_body_transform_path: Optional[Path],
    estimator: str,
    direction: str,
) -> None:
    trajectory = load_tum(run_dir / "trajectories" / "tum" / f"{estimator}.tum")
    gt = load_tum(run_dir / "trajectories" / "tum" / f"ground_truth_for_{estimator}.tum")
    if len(trajectory) != len(gt):
        raise RuntimeError(f"trajectory length mismatch in {run_dir}")
    max_stamp_diff = max(abs(lhs[0] - rhs[0]) for lhs, rhs in zip(trajectory, gt))
    rows = latest_direction_covariance_rows(run_dir, direction)
    observed_to_indices = [pose_index(row["to_key"]) for row in rows]
    if not observed_to_indices:
        raise RuntimeError(f"no {direction} covariance rows found in {run_dir}")
    min_observed = min(observed_to_indices)
    max_observed = max(observed_to_indices)
    split_index = int(round(min_observed + train_fraction * (max_observed - min_observed)))
    split_index = max(min_observed + 1, min(split_index, max_observed - 1))
    if body_transform_path is not None:
        rotation_body, translation_body = load_body_transform(body_transform_path)
        transform_source = f"loaded from {body_transform_path}"
    else:
        rotation_body, translation_body = estimate_body_transform(
            trajectory,
            gt,
            min_index=min_observed,
            max_index=split_index,
        )
        transform_source = "fitted from this run's training split"
    quat_body = rotation_to_quaternion(rotation_body)

    train_rows = [row for row in rows if pose_index(row["to_key"]) <= split_index]
    validation_rows = [row for row in rows if pose_index(row["from_key"]) > split_index]
    all_stats = summarize_rows(rows, trajectory, gt, rotation_body, translation_body)
    train_stats = summarize_rows(train_rows, trajectory, gt, rotation_body, translation_body)
    validation_stats = summarize_rows(validation_rows, trajectory, gt, rotation_body, translation_body)

    print(f"\n{run_dir.name}")
    print(f"  direction/estimator: {direction}/{estimator}")
    print(f"  observed key range: x{min_observed}..x{max_observed}, train split x{split_index}")
    print(f"  max exported GT/Kimera stamp diff: {max_stamp_diff:.3e} s")
    ts_summary = timestamp_summary(raw_gt, trajectory)
    if ts_summary:
        print(f"  {ts_summary}")
    print(
        f"  T_vrpnBody_{estimator}Frame q_wxyz: "
        + " ".join(f"{value:.9f}" for value in quat_body)
    )
    print(
        f"  T_vrpnBody_{estimator}Frame t_xyz_m: "
        + " ".join(f"{value:.6f}" for value in translation_body)
    )
    print(f"  transform source: {transform_source}")
    print_stats("train", train_stats)
    print_stats("validation", validation_stats)
    print_stats("all", all_stats)
    if save_body_transform_path is not None:
        save_body_transform(
            save_body_transform_path,
            run_dir=run_dir,
            rotation_body=rotation_body,
            translation_body=translation_body,
            min_observed=min_observed,
            max_observed=max_observed,
            split_index=split_index,
            train_fraction=train_fraction,
            train_stats=train_stats,
            validation_stats=validation_stats,
            all_stats=all_stats,
            estimator=estimator,
            direction=direction,
        )
        print(f"  saved transform: {save_body_transform_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dirs", nargs="+", type=Path)
    parser.add_argument("--raw-gt", type=Path, default=None)
    parser.add_argument("--train-fraction", type=float, default=0.6)
    parser.add_argument("--direction", choices=("K2G", "G2K"), default="K2G")
    parser.add_argument(
        "--estimator",
        choices=("kimera", "glim"),
        default=None,
        help="Estimator trajectory to compare against GT. Defaults to kimera for K2G and glim for G2K.",
    )
    parser.add_argument(
        "--body-transform",
        type=Path,
        default=None,
        help="Use a previously saved T_vrpnBody_cameraImu JSON instead of fitting one.",
    )
    parser.add_argument(
        "--save-body-transform",
        type=Path,
        default=None,
        help="Save the fitted/loaded T_vrpnBody_cameraImu JSON and evaluation stats.",
    )
    args = parser.parse_args()
    estimator = args.estimator
    if estimator is None:
        estimator = "kimera" if args.direction == "K2G" else "glim"
    if args.body_transform is not None and not args.body_transform.exists():
        raise FileNotFoundError(args.body_transform)
    if args.save_body_transform is not None and len(args.run_dirs) != 1:
        raise ValueError("--save-body-transform currently requires exactly one run directory")
    for run_dir in args.run_dirs:
        analyze_run(
            run_dir,
            args.raw_gt,
            args.train_fraction,
            args.body_transform,
            args.save_body_transform,
            estimator,
            args.direction,
        )


if __name__ == "__main__":
    main()
