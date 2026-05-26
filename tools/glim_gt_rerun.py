#!/usr/bin/env python3
"""Visualize a GLIM trajectory against S3E ground truth in Rerun.

Inputs are TUM trajectory text files:
  timestamp x y z qx qy qz qw

The GLIM trajectory is local, while S3E GT is in a global UTM-like frame. By
default this tool estimates an SE(3) alignment from timestamp-associated poses,
then subtracts the GT origin before logging so the scene is numerically small.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rerun as rr


@dataclass(frozen=True)
class Trajectory:
    times: np.ndarray
    positions: np.ndarray
    quaternions_xyzw: np.ndarray


def read_tum(path: Path) -> Trajectory:
    rows: list[list[float]] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            rows.append([float(x) for x in parts[:8]])

    if not rows:
        raise ValueError(f"No TUM poses found in {path}")

    data = np.asarray(rows, dtype=float)
    return Trajectory(
        times=data[:, 0],
        positions=data[:, 1:4],
        quaternions_xyzw=data[:, 4:8],
    )


def associate_by_time(
    source: Trajectory, target: Trajectory, max_diff_sec: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Associate each target pose to the nearest source pose.

    Returns source indices, target indices, and absolute timestamp differences.
    The S3E GT is about 1 Hz while GLIM is about 10 Hz, so associating GT poses
    into the denser GLIM timeline gives stable alignment pairs.
    """

    source_indices: list[int] = []
    target_indices: list[int] = []
    diffs: list[float] = []

    cursor = 0
    for j, t in enumerate(target.times):
        while cursor + 1 < len(source.times) and source.times[cursor + 1] <= t:
            cursor += 1

        candidates = [cursor]
        if cursor + 1 < len(source.times):
            candidates.append(cursor + 1)

        i = min(candidates, key=lambda idx: abs(source.times[idx] - t))
        diff = abs(source.times[i] - t)
        if diff <= max_diff_sec:
            source_indices.append(i)
            target_indices.append(j)
            diffs.append(diff)

    if len(source_indices) < 3:
        raise ValueError(
            f"Only {len(source_indices)} timestamp associations within "
            f"{max_diff_sec:.3f}s; need at least 3 for SE(3) alignment"
        )

    return (
        np.asarray(source_indices, dtype=int),
        np.asarray(target_indices, dtype=int),
        np.asarray(diffs, dtype=float),
    )


def estimate_se3(source_xyz: np.ndarray, target_xyz: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return R, t such that R @ source + t is closest to target."""

    source_mean = source_xyz.mean(axis=0)
    target_mean = target_xyz.mean(axis=0)
    source_centered = source_xyz - source_mean
    target_centered = target_xyz - target_mean

    covariance = source_centered.T @ target_centered
    u, _, vt = np.linalg.svd(covariance)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0.0:
        vt[-1, :] *= -1.0
        r = vt.T @ u.T

    t = target_mean - r @ source_mean
    return r, t


def transform_points(points: np.ndarray, r: np.ndarray, t: np.ndarray) -> np.ndarray:
    return (r @ points.T).T + t


def path_length(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def compute_translation_errors(
    aligned_source: np.ndarray,
    target: np.ndarray,
    source_indices: np.ndarray,
    target_indices: np.ndarray,
) -> np.ndarray:
    return np.linalg.norm(
        aligned_source[source_indices] - target[target_indices],
        axis=1,
    )


def log_static_scene(
    gt_xyz: np.ndarray,
    glim_xyz: np.ndarray,
    errors: np.ndarray,
    args: argparse.Namespace,
    association_count: int,
) -> None:
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    rr.log(
        "world/ground_truth/trajectory",
        rr.LineStrips3D([gt_xyz], colors=[0, 220, 80, 255], radii=0.05),
        static=True,
    )
    rr.log(
        "world/glim_aligned/trajectory",
        rr.LineStrips3D([glim_xyz], colors=[255, 170, 0, 255], radii=0.05),
        static=True,
    )
    rr.log(
        "world/ground_truth/samples",
        rr.Points3D(gt_xyz, colors=[0, 220, 80, 180], radii=0.10),
        static=True,
    )
    rr.log(
        "world/glim_aligned/samples",
        rr.Points3D(glim_xyz, colors=[255, 170, 0, 160], radii=0.07),
        static=True,
    )

    rmse = math.sqrt(float(np.mean(errors * errors))) if len(errors) else float("nan")
    mean = float(np.mean(errors)) if len(errors) else float("nan")
    median = float(np.median(errors)) if len(errors) else float("nan")
    max_error = float(np.max(errors)) if len(errors) else float("nan")

    summary = "\n".join(
        [
            "# GLIM vs S3E Ground Truth",
            "",
            f"GLIM poses: {len(glim_xyz)}",
            f"GT poses: {len(gt_xyz)}",
            f"Associations: {association_count}",
            f"GT path length: {path_length(gt_xyz):.3f} m",
            f"GLIM aligned path length: {path_length(glim_xyz):.3f} m",
            "",
            "Translation APE after SE(3) alignment:",
            f"- RMSE: {rmse:.3f} m",
            f"- mean: {mean:.3f} m",
            f"- median: {median:.3f} m",
            f"- max: {max_error:.3f} m",
            "",
            f"GT input: {args.gt}",
            f"GLIM input: {args.glim}",
        ]
    )
    rr.log("summary", rr.TextDocument(summary, media_type=rr.MediaType.MARKDOWN), static=True)


def log_time_series(
    times: np.ndarray,
    gt_xyz: np.ndarray,
    glim_xyz: np.ndarray,
    max_frames: int,
) -> None:
    if len(glim_xyz) == 0:
        return

    stride = max(1, math.ceil(len(glim_xyz) / max_frames))
    start_time = float(times[0])

    # GT is sparse; log it statically, then animate GLIM's current pose and path.
    for i in range(0, len(glim_xyz), stride):
        rr.set_time("time", duration=float(times[i] - start_time))
        rr.log(
            "world/glim_aligned/current_pose",
            rr.Points3D([glim_xyz[i]], colors=[255, 120, 0, 255], radii=0.35),
        )
        rr.log(
            "world/glim_aligned/path_so_far",
            rr.LineStrips3D([glim_xyz[: i + 1]], colors=[255, 170, 0, 255], radii=0.08),
        )

    rr.set_time("time", duration=float(times[-1] - start_time))
    rr.log(
        "world/glim_aligned/current_pose",
        rr.Points3D([glim_xyz[-1]], colors=[255, 120, 0, 255], radii=0.35),
    )
    rr.log(
        "world/glim_aligned/path_so_far",
        rr.LineStrips3D([glim_xyz], colors=[255, 170, 0, 255], radii=0.08),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stream aligned GLIM and S3E GT trajectories to Rerun."
    )
    parser.add_argument("--glim", required=True, type=Path, help="GLIM TUM trajectory")
    parser.add_argument("--gt", required=True, type=Path, help="S3E GT TUM trajectory")
    parser.add_argument(
        "--max-association-diff",
        type=float,
        default=0.1,
        help="Max timestamp difference for alignment/APE pairs [sec]",
    )
    parser.add_argument(
        "--alignment",
        choices=("se3", "none"),
        default="se3",
        help="Alignment applied to GLIM before visualization",
    )
    parser.add_argument(
        "--recording-id",
        default="glim_s3e_gt",
        help="Rerun recording id",
    )
    parser.add_argument(
        "--connect",
        default="rerun+http://127.0.0.1:9876/proxy",
        help="Rerun gRPC server URL. Use empty string with --save-only.",
    )
    parser.add_argument("--save", type=Path, help="Optional .rrd output path")
    parser.add_argument(
        "--max-frames",
        type=int,
        default=600,
        help="Max animated frames to log into Rerun",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    glim = read_tum(args.glim)
    gt_all = read_tum(args.gt)

    gt_mask = (gt_all.times >= glim.times[0] - args.max_association_diff) & (
        gt_all.times <= glim.times[-1] + args.max_association_diff
    )
    gt = Trajectory(
        times=gt_all.times[gt_mask],
        positions=gt_all.positions[gt_mask],
        quaternions_xyzw=gt_all.quaternions_xyzw[gt_mask],
    )
    if len(gt.times) == 0:
        raise ValueError("No GT poses overlap the GLIM trajectory time range")

    glim_indices, gt_indices, _ = associate_by_time(
        source=glim,
        target=gt,
        max_diff_sec=args.max_association_diff,
    )

    if args.alignment == "se3":
        r, t = estimate_se3(
            source_xyz=glim.positions[glim_indices],
            target_xyz=gt.positions[gt_indices],
        )
        glim_aligned = transform_points(glim.positions, r, t)
    else:
        glim_aligned = glim.positions.copy()

    # Center the large UTM-like coordinates around the first GT pose. This keeps
    # Rerun navigation stable while preserving the relative comparison.
    origin = gt.positions[0].copy()
    gt_view = gt.positions - origin
    glim_view = glim_aligned - origin

    errors = compute_translation_errors(
        aligned_source=glim_aligned,
        target=gt.positions,
        source_indices=glim_indices,
        target_indices=gt_indices,
    )

    rr.init("glim_s3e_gt", recording_id=args.recording_id)
    if args.save:
        rr.save(args.save)
    if args.connect:
        rr.connect_grpc(args.connect)

    log_static_scene(gt_view, glim_view, errors, args, len(glim_indices))
    log_time_series(glim.times, gt_view, glim_view, args.max_frames)


if __name__ == "__main__":
    main()
