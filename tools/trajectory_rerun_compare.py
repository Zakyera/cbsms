#!/usr/bin/env python3
"""Write a Rerun recording comparing TUM trajectories.

Each estimator is aligned to its own matched ground-truth TUM file using an
SE(3) transform without scale, matching the evo APE setup used by
cbsms_experiment.py.
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


@dataclass(frozen=True)
class EstimatorSpec:
    name: str
    tum: Path
    gt: Path
    color: tuple[int, int, int, int]


def read_tum(path: Path) -> Trajectory:
    rows: list[list[float]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            parts = stripped.split()
            if len(parts) < 8:
                continue
            rows.append([float(value) for value in parts[:8]])

    if not rows:
        raise ValueError(f"no TUM poses found in {path}")

    data = np.asarray(rows, dtype=float)
    return Trajectory(
        times=data[:, 0],
        positions=data[:, 1:4],
        quaternions_xyzw=data[:, 4:8],
    )


def estimate_se3(source_xyz: np.ndarray, target_xyz: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return R, t such that R @ source + t best matches target."""

    source_mean = source_xyz.mean(axis=0)
    target_mean = target_xyz.mean(axis=0)
    source_centered = source_xyz - source_mean
    target_centered = target_xyz - target_mean

    covariance = source_centered.T @ target_centered
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T

    translation = target_mean - rotation @ source_mean
    return rotation, translation


def transform_points(points: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    return (rotation @ points.T).T + translation


def path_length(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def rmse(errors: np.ndarray) -> float:
    if len(errors) == 0:
        return float("nan")
    return math.sqrt(float(np.mean(errors * errors)))


def parse_color(text: str) -> tuple[int, int, int, int]:
    parts = [int(part) for part in text.split(",")]
    if len(parts) == 3:
        parts.append(255)
    if len(parts) != 4 or any(part < 0 or part > 255 for part in parts):
        raise argparse.ArgumentTypeError("colors must be r,g,b or r,g,b,a with values 0..255")
    return tuple(parts)  # type: ignore[return-value]


def parse_estimator_spec(text: str) -> EstimatorSpec:
    parts = text.split(":")
    if len(parts) != 4:
        raise argparse.ArgumentTypeError(
            "estimator specs must be name:estimator.tum:matched_gt.tum:r,g,b[,a]"
        )
    name, tum, gt, color = parts
    return EstimatorSpec(name=name, tum=Path(tum), gt=Path(gt), color=parse_color(color))


def matching_time_range(trajectories: list[Trajectory]) -> tuple[float, float]:
    start = max(float(traj.times[0]) for traj in trajectories)
    end = min(float(traj.times[-1]) for traj in trajectories)
    if end <= start:
        start = min(float(traj.times[0]) for traj in trajectories)
        end = max(float(traj.times[-1]) for traj in trajectories)
    return start, end


def crop_by_time(traj: Trajectory, start: float, end: float) -> Trajectory:
    mask = (traj.times >= start) & (traj.times <= end)
    if not np.any(mask):
        return traj
    return Trajectory(
        times=traj.times[mask],
        positions=traj.positions[mask],
        quaternions_xyzw=traj.quaternions_xyzw[mask],
    )


def log_trajectory(
    path: str,
    xyz: np.ndarray,
    color: tuple[int, int, int, int],
    line_radius: float,
    point_radius: float,
    hide_samples: bool,
) -> None:
    rr.log(path, rr.LineStrips3D([xyz], colors=[color], radii=line_radius), static=True)
    if not hide_samples:
        rr.log(
            f"{path}/samples",
            rr.Points3D(xyz, colors=[color], radii=point_radius),
            static=True,
        )


def log_animation(
    name: str,
    times: np.ndarray,
    xyz: np.ndarray,
    color: tuple[int, int, int, int],
    max_frames: int,
) -> None:
    if len(xyz) == 0 or max_frames <= 0:
        return
    stride = max(1, math.ceil(len(xyz) / max_frames))
    start_time = float(times[0])
    for i in range(0, len(xyz), stride):
        rr.set_time("time", duration=float(times[i] - start_time))
        rr.log(f"world/{name}/current", rr.Points3D([xyz[i]], colors=[color], radii=0.25))
        rr.log(
            f"world/{name}/path_so_far",
            rr.LineStrips3D([xyz[: i + 1]], colors=[color], radii=0.06),
        )
    rr.set_time("time", duration=float(times[-1] - start_time))
    rr.log(f"world/{name}/current", rr.Points3D([xyz[-1]], colors=[color], radii=0.25))
    rr.log(f"world/{name}/path_so_far", rr.LineStrips3D([xyz], colors=[color], radii=0.06))


def build_summary(
    title: str,
    gt_path: Path,
    estimator_rows: list[dict[str, object]],
) -> str:
    lines = [
        f"# {title}",
        "",
        "All estimator trajectories are SE(3)-aligned to their matched ground-truth samples without scale.",
        "",
        "| trajectory | poses | path m | matched GT path m | APE RMSE m | mean m | max m |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in estimator_rows:
        lines.append(
            "| {name} | {poses} | {path:.3f} | {gt_path_len:.3f} | {rmse:.4f} | {mean:.4f} | {max:.4f} |".format(
                **row
            )
        )
    lines.extend(["", f"Ground truth input: `{gt_path}`"])
    for row in estimator_rows:
        lines.append(f"{row['name']} input: `{row['tum']}`")
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gt", required=True, type=Path, help="Full ground-truth TUM trajectory")
    parser.add_argument(
        "--estimator",
        action="append",
        required=True,
        type=parse_estimator_spec,
        help="name:estimator.tum:matched_gt.tum:r,g,b[,a]",
    )
    parser.add_argument("--save", required=True, type=Path, help="Output .rrd path")
    parser.add_argument("--recording-id", default="trajectory_compare")
    parser.add_argument("--title", default="Trajectory Comparison")
    parser.add_argument("--line-radius", type=float, default=0.05)
    parser.add_argument("--point-radius", type=float, default=0.08)
    parser.add_argument("--hide-samples", action="store_true")
    parser.add_argument("--max-frames", type=int, default=700)
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    full_gt = read_tum(args.gt)
    estimator_outputs: list[tuple[EstimatorSpec, Trajectory, Trajectory, np.ndarray, np.ndarray]] = []
    matched_gts: list[Trajectory] = []
    estimator_rows: list[dict[str, object]] = []

    for spec in args.estimator:
        est = read_tum(spec.tum)
        matched_gt = read_tum(spec.gt)
        if len(est.positions) != len(matched_gt.positions):
            count = min(len(est.positions), len(matched_gt.positions))
            est = Trajectory(est.times[:count], est.positions[:count], est.quaternions_xyzw[:count])
            matched_gt = Trajectory(
                matched_gt.times[:count],
                matched_gt.positions[:count],
                matched_gt.quaternions_xyzw[:count],
            )

        rotation, translation = estimate_se3(est.positions, matched_gt.positions)
        aligned = transform_points(est.positions, rotation, translation)
        errors = np.linalg.norm(aligned - matched_gt.positions, axis=1)

        estimator_rows.append(
            {
                "name": spec.name,
                "poses": len(est.positions),
                "path": path_length(aligned),
                "gt_path_len": path_length(matched_gt.positions),
                "rmse": rmse(errors),
                "mean": float(np.mean(errors)) if len(errors) else float("nan"),
                "max": float(np.max(errors)) if len(errors) else float("nan"),
                "tum": spec.tum,
            }
        )
        estimator_outputs.append((spec, est, matched_gt, aligned, errors))
        matched_gts.append(matched_gt)

    start, end = matching_time_range(matched_gts)
    gt_window = crop_by_time(full_gt, start, end)
    origin = gt_window.positions[0].copy()

    args.save.parent.mkdir(parents=True, exist_ok=True)
    rr.init(args.recording_id, recording_id=args.recording_id)
    rr.save(args.save)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    gt_xyz = gt_window.positions - origin
    log_trajectory(
        "world/ground_truth/trajectory",
        gt_xyz,
        (0, 210, 80, 255),
        args.line_radius,
        args.point_radius,
        args.hide_samples,
    )

    for spec, est, _matched_gt, aligned, _errors in estimator_outputs:
        view_xyz = aligned - origin
        log_trajectory(
            f"world/{spec.name}/trajectory",
            view_xyz,
            spec.color,
            args.line_radius,
            args.point_radius,
            args.hide_samples,
        )
        log_animation(spec.name, est.times, view_xyz, spec.color, args.max_frames)

    rr.log(
        "summary",
        rr.TextDocument(
            build_summary(args.title, args.gt, estimator_rows),
            media_type=rr.MediaType.MARKDOWN,
        ),
        static=True,
    )
    print(args.save)


if __name__ == "__main__":
    main()
