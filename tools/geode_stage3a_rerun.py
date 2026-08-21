#!/usr/bin/env python3
"""Create a passive GEODE Stage 3A Rerun inspection recording.

The recording combines already-finalized artifacts only. It does not rerun
GLIM and does not reconstruct unrecorded internal factors. The spatial graph
contains exact recorded G-to-K pose endpoints and unique outgoing belief
edges; it is deliberately labelled as an edge graph rather than GLIM's full
internal factor graph.
"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import rerun as rr
import rerun.blueprint as rrb


def read_rows(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def read_tum(path):
    values = []
    with Path(path).open(encoding="utf-8") as stream:
        for original_index, line in enumerate(stream):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            fields = stripped.split()
            if len(fields) < 8:
                raise ValueError(f"{path}:{original_index + 1}: expected TUM pose")
            row = [float(value) for value in fields[:8]]
            if not all(math.isfinite(value) for value in row):
                raise ValueError(f"{path}:{original_index + 1}: non-finite pose")
            values.append(row)
    if not values:
        raise ValueError(f"no poses in {path}")
    array = np.asarray(values, dtype=np.float64)
    if np.any(np.diff(array[:, 0]) <= 0.0):
        raise ValueError(f"timestamps are not strictly increasing in {path}")
    return array


def associate_nearest(source_times, target_times, maximum_difference):
    source_indices = []
    target_indices = []
    differences = []
    for target_index, stamp in enumerate(target_times):
        insertion = int(np.searchsorted(source_times, stamp))
        candidates = []
        if insertion < len(source_times):
            candidates.append(insertion)
        if insertion > 0:
            candidates.append(insertion - 1)
        source_index = min(candidates, key=lambda index: abs(source_times[index] - stamp))
        difference = abs(source_times[source_index] - stamp)
        if difference <= maximum_difference:
            source_indices.append(source_index)
            target_indices.append(target_index)
            differences.append(difference)
    if len(source_indices) < 3:
        raise ValueError("fewer than three trajectory timestamp associations")
    return (np.asarray(source_indices, dtype=int),
            np.asarray(target_indices, dtype=int),
            np.asarray(differences, dtype=np.float64))


def estimate_se3(source, target):
    """Return R,t with R @ source + t closest to target, without scale."""
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    covariance = (source - source_mean).T @ (target - target_mean)
    u, _singular, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = target_mean - rotation @ source_mean
    return rotation, translation


def interpolate_positions(trajectory, stamps):
    times = trajectory[:, 0]
    if np.any(stamps < times[0]) or np.any(stamps > times[-1]):
        raise ValueError("edge timestamp is outside trajectory coverage")
    return np.column_stack([
        np.interp(stamps, times, trajectory[:, axis]) for axis in (1, 2, 3)
    ])


def finite(row, field, default=math.nan):
    try:
        value = float(row.get(field, default))
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def log_scalar(path, value):
    if math.isfinite(float(value)):
        rr.log(path, rr.Scalars(float(value)))


def read_scan_diagnostics(health_path, glim_log_path=None):
    scans = {}
    with Path(health_path).open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            frame = int(row["frame_id"])
            scan = scans.setdefault(frame, {"factor_inlier_fractions": []})
            if row["record_kind"] == "aggregate":
                scan.update({
                    "kappa_rotation": max(
                        finite(row, f"rotation_condition_ratio_{index}")
                        for index in range(3)),
                    "kappa_translation": max(
                        finite(row, f"translation_condition_ratio_{index}")
                        for index in range(3)),
                    "source_points": finite(row, "source_point_count"),
                    "initial_cost": finite(row, "initial_cost"),
                    "final_cost": finite(row, "final_cost"),
                })
            elif row["record_kind"] == "factor":
                value = finite(row, "inlier_fraction")
                if math.isfinite(value):
                    scan["factor_inlier_fractions"].append(value)
    if glim_log_path:
        marker = "GLIM_SCAN_HEALTH_ROW,"
        with Path(glim_log_path).open(errors="replace") as stream:
            for line in stream:
                position = line.find(marker)
                if position < 0:
                    continue
                values = next(csv.reader([line[position:]]))
                if len(values) < 23:
                    continue
                scan = scans.setdefault(
                    int(values[2]), {"factor_inlier_fractions": []})
                scan.update({
                    "lm_iterations": float(values[9]),
                    "scan_minus_imu_translation_m": float(values[21]),
                    "scan_minus_imu_rotation_deg": float(values[22]),
                })
    return scans


def enrich_edges(edges, scans):
    def median(samples, field):
        values = [float(sample[field]) for sample in samples
                  if field in sample and math.isfinite(float(sample[field]))]
        return float(np.median(values)) if values else math.nan

    enriched = []
    for original in edges:
        row = dict(original)
        start = int(row["from_pose_index"])
        end = int(row["to_pose_index"])
        samples = [scans[key] for key in sorted(scans) if start < key <= end]
        for field in ("kappa_rotation", "kappa_translation"):
            values = [float(sample[field]) for sample in samples
                      if field in sample and math.isfinite(float(sample[field]))]
            if values and not math.isfinite(finite(row, field)):
                row[field] = max(values)
        for field in ("source_points", "initial_cost", "final_cost",
                      "lm_iterations", "scan_minus_imu_translation_m",
                      "scan_minus_imu_rotation_deg"):
            if not math.isfinite(finite(row, field)):
                row[field] = median(samples, field)
        inliers = []
        for sample in samples:
            inliers.extend(sample.get("factor_inlier_fractions", []))
        if inliers and not math.isfinite(finite(row, "inlier_fraction")):
            row["inlier_fraction"] = float(np.median(inliers))
        if not math.isfinite(finite(row, "translation_vector_cosine")):
            gt = np.asarray([finite(row, f"z_gt_t{axis}") for axis in "xyz"])
            glim = np.asarray([finite(row, f"z_beta_t{axis}") for axis in "xyz"])
            denominator = float(np.linalg.norm(gt) * np.linalg.norm(glim))
            if denominator > 0.0 and np.all(np.isfinite(gt)) and np.all(np.isfinite(glim)):
                row["translation_vector_cosine"] = float(np.dot(gt, glim) / denominator)
        enriched.append(row)
    return enriched


def make_blueprint(scene_center, scene_extent, cbs_label="CBS off",
                   include_covariance_audit=False, presentation_root="",
                   active_tab_index=None):
    presentation_root = presentation_root.strip("/")
    presentation_prefix = (
        f"/{presentation_root}" if presentation_root else ""
    )
    aligned_root = f"{presentation_prefix}/aligned"
    lidar_root = f"{presentation_prefix}/glim/current_scan"
    world = rrb.Spatial3DView(
        origin="/world",
        contents=["+ /world/**", "- /__properties/**"],
        name="GLIM vs verified GT + recorded G-to-K edge graph",
        background=[16, 18, 22],
        line_grid=rrb.LineGrid3D(
            visible=True,
            spacing=10.0,
            stroke_width=1.0,
            color=[90, 96, 105, 90],
        ),
        eye_controls=rrb.EyeControls3D(
            kind=rrb.Eye3DKind.Orbital,
            position=(float(scene_center[0]), float(scene_center[1]),
                      float(scene_center[2] + 1.4 * scene_extent)),
            look_target=tuple(float(value) for value in scene_center),
            eye_up=(0.0, 1.0, 0.0),
        ),
    )
    local_geometry = rrb.Spatial3DView(
        origin="/case_geometry",
        contents=["+ /case_geometry/**", "- /__properties/**"],
        name="Selected wrong-basin local LiDAR geometry",
        background=[16, 18, 22],
        eye_controls=rrb.EyeControls3D(
            kind=rrb.Eye3DKind.Orbital,
            position=(20.0, -120.0, 90.0),
            look_target=(20.0, 0.0, 0.0),
            eye_up=(0.0, 0.0, 1.0),
        ),
    )
    conditioning = rrb.TimeSeriesView(
        origin="/signals/conditioning",
        contents=["+ /signals/conditioning/**", "- /__properties/**"],
        name="DCReg continuous conditioning and tau=10 masks",
    )
    error = rrb.TimeSeriesView(
        origin="/signals/error",
        contents=["+ /signals/error/**", "- /__properties/**"],
        name="Verified short-edge error",
    )
    support = rrb.TimeSeriesView(
        origin="/signals/support",
        contents=["+ /signals/support/**", "- /__properties/**"],
        name="Support and matching",
    )
    consistency = rrb.TimeSeriesView(
        origin="/signals/consistency",
        contents=["+ /signals/consistency/**", "- /__properties/**"],
        name="Motion direction and scan-IMU consistency",
    )
    overview = rrb.Vertical(
        rrb.Horizontal(world, local_geometry, column_shares=[2.2, 1.0]),
        rrb.Horizontal(conditioning, error, support, consistency),
        row_shares=[2.2, 1.0],
        name="Observability and verified error",
    )

    factor_graph = rrb.Spatial3DView(
        origin="/glim/factor_graph_inspector/spatial",
        contents=[
            "+ /glim/factor_graph_inspector/spatial/**",
            "- /__properties/**",
        ],
        name="Actual GLIM fixed-lag factor graph",
        background=[16, 18, 22],
        line_grid=rrb.LineGrid3D(
            visible=True,
            spacing=1.0,
            stroke_width=1.0,
            color=[90, 96, 105, 90],
        ),
    )
    kimera_factor_graph = rrb.Spatial3DView(
        origin="/kimera/factor_graph_inspector/spatial",
        contents=[
            "+ /kimera/factor_graph_inspector/spatial/context_trajectory",
            "+ /kimera/factor_graph_inspector/spatial/states/**",
            "+ /kimera/factor_graph_inspector/spatial/factors/**",
            # Factor-marker entities carry long diagnostic labels and obscure
            # the active fixed-lag graph.  Keep them recorded for inspection,
            # but exclude them from the presentation view.  CBS merge overlays
            # likewise belong only in the dedicated CBS audit view.
            "- /kimera/factor_graph_inspector/spatial/factor_markers/**",
            "- /kimera/factor_graph_inspector/spatial/cbs_merge/**",
            "- /__properties/**",
        ],
        name="Actual Kimera fixed-lag factor graph + grey trajectory context",
        background=[16, 18, 22],
        line_grid=rrb.LineGrid3D(
            visible=True,
            spacing=1.0,
            stroke_width=1.0,
            color=[90, 96, 105, 90],
        ),
        eye_controls=rrb.EyeControls3D(
            kind=rrb.Eye3DKind.Orbital,
            position=(0.0, -12.0, 8.0),
            look_target=(0.0, 0.0, 0.0),
            eye_up=(0.0, 0.0, 1.0),
            tracking_entity=(
                "/kimera/factor_graph_inspector/spatial/states/latest_pose"
            ),
        ),
    )
    aligned_trajectories = rrb.Spatial3DView(
        origin=aligned_root,
        contents=[
            f"+ {aligned_root}/ground_truth/**",
            f"+ {aligned_root}/glim/**",
            f"+ {aligned_root}/kimera/**",
            f"- {aligned_root}/metadata/**",
            "- /__properties/**",
        ],
        name=f"Ground truth vs GLIM vs Kimera ({cbs_label})",
        background=[16, 18, 22],
        line_grid=rrb.LineGrid3D(
            visible=True,
            spacing=5.0,
            stroke_width=1.0,
            color=[90, 96, 105, 90],
        ),
    )
    lidar = rrb.Spatial3DView(
        origin=lidar_root,
        contents=[
            f"+ {lidar_root}/**",
            "- /__properties/**",
        ],
        name="Live GLIM PointCloud2 playback",
        background=[16, 18, 22],
    )
    left_video = rrb.Spatial2DView(
        origin="/video/left",
        contents=["+ /video/left/**", "- /__properties/**"],
        name="Original left camera",
    )
    right_video = rrb.Spatial2DView(
        origin="/video/right",
        contents=["+ /video/right/**", "- /__properties/**"],
        name="Original right camera",
    )
    graph_counts = rrb.TimeSeriesView(
        origin="/glim/factor_graph_inspector/counts",
        contents=[
            "+ /glim/factor_graph_inspector/counts/**",
            "- /__properties/**",
        ],
        name="Factor composition and active state",
    )
    live_dcreg_conditioning = rrb.TimeSeriesView(
        origin="/glim/dcreg_health",
        contents=[
            "+ /glim/dcreg_health/condition_ratio/**",
            "+ /glim/dcreg_health/condition/threshold",
            "- /__properties/**",
        ],
        name="Frozen DCReg condition ratios",
    )
    live_dcreg_absolute_masks = rrb.TimeSeriesView(
        origin="/glim/dcreg_health/absolute_mask",
        contents=[
            "+ /glim/dcreg_health/absolute_mask/**",
            "- /__properties/**",
        ],
        name="Frozen DCReg absolute masks (0/1)",
    )
    live_inspector = rrb.Vertical(
        rrb.Horizontal(
            factor_graph,
            kimera_factor_graph,
            aligned_trajectories,
            column_shares=[1.0, 1.0, 1.15],
        ),
        rrb.Horizontal(
            lidar,
            left_video,
            right_video,
            column_shares=[1.25, 1.0, 1.0],
        ),
        rrb.Horizontal(
            graph_counts,
            live_dcreg_conditioning,
            live_dcreg_absolute_masks,
            column_shares=[1.0, 1.0, 0.8],
        ),
        row_shares=[2.2, 1.3, 1.0],
        name=f"Live estimators, factor graphs and trajectories ({cbs_label})",
    )
    tabs = [overview, live_inspector]
    if include_covariance_audit:
        covariance_provenance = rrb.TimeSeriesView(
            origin="/covariance_audit/provenance",
            contents=["+ /covariance_audit/provenance/**", "- /__properties/**"],
            name="Covariance provenance difference norms",
        )
        covariance_values = rrb.TimeSeriesView(
            origin="/covariance_audit/covariance",
            contents=["+ /covariance_audit/covariance/**", "- /__properties/**"],
            name="Inserted covariance traces and diagonal",
        )
        residual = rrb.TimeSeriesView(
            origin="/covariance_audit/residual",
            contents=["+ /covariance_audit/residual/**", "- /__properties/**"],
            name="Pre-injection residual, whitening and NIS",
        )
        strength = rrb.TimeSeriesView(
            origin="/covariance_audit/strength",
            contents=["+ /covariance_audit/strength/**", "- /__properties/**"],
            name="External vs endpoint-reduced receiver information",
        )
        factor_groups = rrb.TimeSeriesView(
            origin="/covariance_audit/factor_groups",
            contents=["+ /covariance_audit/factor_groups/**", "- /__properties/**"],
            name="Receiver objective grouped by factor type",
        )
        audit = rrb.Vertical(
            rrb.Horizontal(covariance_provenance, covariance_values),
            rrb.Horizontal(residual, strength, factor_groups),
            name="CBS relative-covariance exchange audit",
        )
        tabs.append(audit)
    return rrb.Blueprint(
        rrb.Tabs(
            *tabs,
            active_tab=(
                active_tab_index
                if active_tab_index is not None
                else (2 if include_covariance_audit else 1)
            ),
        ),
        rrb.BlueprintPanel(state="collapsed"),
        rrb.SelectionPanel(state="collapsed"),
        rrb.TimePanel(state="expanded", play_state="paused", loop_mode="off"),
        auto_layout=False,
        auto_views=False,
    )


def log_static_scene(glim, gt, edges, aligned_positions):
    gt_in_coverage = gt[(gt[:, 0] >= glim[0, 0]) & (gt[:, 0] <= glim[-1, 0])]
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log(
        "world/ground_truth/trajectory",
        rr.LineStrips3D([gt_in_coverage[:, 1:4]], colors=[[0, 220, 80, 255]],
                        radii=0.10),
        static=True,
    )
    rr.log(
        "world/glim_visualization_aligned/trajectory",
        rr.LineStrips3D([aligned_positions], colors=[[255, 166, 0, 255]],
                        radii=0.10),
        static=True,
    )
    stride = max(1, len(aligned_positions) // 1000)
    rr.log(
        "world/recorded_edge_graph/pose_nodes",
        rr.Points3D(aligned_positions[::stride], colors=[255, 190, 70, 150],
                    radii=0.15),
        static=True,
    )

    strips = []
    colors = []
    for row in edges:
        start = int(row["from_pose_index"])
        end = int(row["to_pose_index"])
        if start < 0 or end >= len(aligned_positions):
            raise ValueError(f"edge {start}->{end} is outside GLIM pose array")
        strips.append(np.asarray([aligned_positions[start], aligned_positions[end]]))
        kappa = finite(row, "kappa_translation")
        colors.append([235, 70, 70, 120] if kappa > 10.0 else [70, 190, 255, 150])
    rr.log(
        "world/recorded_edge_graph/g_to_k_unique_edges",
        rr.LineStrips3D(strips, colors=colors, radii=0.025),
        static=True,
    )


def log_edge_timeline(edges, aligned_positions, gt):
    start_stamps = np.asarray([float(row["from_stamp_sec"]) for row in edges])
    end_stamps = np.asarray([float(row["to_stamp_sec"]) for row in edges])
    gt_starts = interpolate_positions(gt, start_stamps)
    gt_ends = interpolate_positions(gt, end_stamps)
    for index, row in enumerate(edges):
        stamp = end_stamps[index]
        start_index = int(row["from_pose_index"])
        end_index = int(row["to_pose_index"])
        glim_edge = np.asarray([aligned_positions[start_index], aligned_positions[end_index]])
        gt_edge = np.asarray([gt_starts[index], gt_ends[index]])
        # Match the frozen C++ Rerun visualizer's timestamp timeline so the
        # factor graph, cameras, bag points, and offline analysis share one
        # playback cursor.
        rr.set_time("time", timestamp=stamp)
        rr.log("world/current_edge/glim", rr.LineStrips3D(
            [glim_edge], colors=[[255, 166, 0, 255]], radii=0.25))
        rr.log("world/current_edge/ground_truth", rr.LineStrips3D(
            [gt_edge], colors=[[0, 240, 100, 255]], radii=0.25))
        rr.log("world/current_edge/endpoints", rr.Points3D(
            [glim_edge[-1], gt_edge[-1]],
            colors=[[255, 166, 0, 255], [0, 240, 100, 255]], radii=0.35))

        kappa_r = finite(row, "kappa_rotation")
        kappa_t = finite(row, "kappa_translation")
        log_scalar("signals/conditioning/log_kappa_rotation", math.log(kappa_r))
        log_scalar("signals/conditioning/log_kappa_translation", math.log(kappa_t))
        log_scalar("signals/conditioning/tau_log", math.log(10.0))
        log_scalar("signals/conditioning/rotation_degenerate", float(kappa_r > 10.0))
        log_scalar("signals/conditioning/translation_degenerate", float(kappa_t > 10.0))
        log_scalar("signals/conditioning/rotation_signed_margin", math.log(10.0 / kappa_r))
        log_scalar("signals/conditioning/translation_signed_margin", math.log(10.0 / kappa_t))

        log_scalar("signals/error/rotation_rad", finite(row, "rotation_error_rad"))
        log_scalar("signals/error/translation_m", finite(row, "translation_error_m"))
        log_scalar("signals/support/source_points", finite(row, "source_points"))
        log_scalar("signals/support/inlier_fraction", finite(row, "inlier_fraction"))
        log_scalar("signals/support/initial_cost", finite(row, "initial_cost"))
        log_scalar("signals/support/final_cost", finite(row, "final_cost"))
        log_scalar("signals/support/lm_iterations", finite(row, "lm_iterations"))
        log_scalar("signals/consistency/translation_vector_cosine",
                   finite(row, "translation_vector_cosine"))
        log_scalar("signals/consistency/scan_minus_imu_translation_m",
                   finite(row, "scan_minus_imu_translation_m"))
        log_scalar("signals/consistency/scan_minus_imu_rotation_deg",
                   finite(row, "scan_minus_imu_rotation_deg"))


def log_wrong_basin_geometry(case_rows, cloud_npz, manifest_path):
    if not case_rows or not cloud_npz or not manifest_path:
        return 0
    archive = np.load(cloud_npz)
    manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    entries = manifest["entries"]
    if len(entries) != 2 * len(case_rows):
        raise ValueError("wrong-basin cloud manifest does not match case table")
    for case_index, row in enumerate(case_rows):
        rr.set_time("time", timestamp=float(row["to_stamp_sec"]))
        start_entry = entries[2 * case_index]
        end_entry = entries[2 * case_index + 1]
        start_cloud = archive[start_entry["array_key"]]
        end_cloud = archive[end_entry["array_key"]]
        rr.log("case_geometry/start_cloud", rr.Points3D(
            start_cloud, colors=[60, 170, 255, 135], radii=0.04))
        rr.log("case_geometry/end_cloud", rr.Points3D(
            end_cloud, colors=[255, 155, 40, 135], radii=0.04))
        gt_vector = np.asarray([[finite(row, "gt_translation_x_m"),
                                 finite(row, "gt_translation_y_m"),
                                 finite(row, "gt_translation_z_m")]])
        glim_vector = np.asarray([[finite(row, "glim_translation_x_m"),
                                   finite(row, "glim_translation_y_m"),
                                   finite(row, "glim_translation_z_m")]])
        rr.log("case_geometry/translation/ground_truth", rr.Arrows3D(
            vectors=gt_vector, origins=[[0.0, 0.0, 0.0]],
            colors=[0, 240, 100, 255], radii=0.03))
        rr.log("case_geometry/translation/glim", rr.Arrows3D(
            vectors=glim_vector, origins=[[0.0, 0.0, 0.0]],
            colors=[255, 166, 0, 255], radii=0.03))
        rr.log("case_geometry/case", rr.TextDocument(
            f"# Wrong-basin edge {row['from_pose_index']}->{row['to_pose_index']}\n\n"
            f"- translation cosine: {finite(row, 'translation_vector_cosine'):.3f}\n"
            f"- translation error: {finite(row, 'translation_error_m'):.3f} m\n"
            f"- kappa_R: {finite(row, 'kappa_rotation'):.3f}\n"
            f"- kappa_t: {finite(row, 'kappa_translation'):.3f}\n"
            f"- inlier fraction: {finite(row, 'inlier_fraction'):.3f}\n\n"
            "Clouds are shown in their scan-local LiDAR coordinates. DCReg "
            "characterizes the local Hessian; it does not prove the selected "
            "correspondence basin is correct."))
    return len(case_rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", default="GEODE Inland Waterways Medium Gamma")
    parser.add_argument("--glim-tum", type=Path, required=True)
    parser.add_argument("--ground-truth-tum", type=Path, required=True)
    parser.add_argument("--edge-table", type=Path, required=True)
    parser.add_argument("--health-csv", type=Path)
    parser.add_argument("--glim-log", type=Path)
    parser.add_argument("--wrong-basin-table", type=Path)
    parser.add_argument("--wrong-basin-clouds", type=Path)
    parser.add_argument("--wrong-basin-manifest", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--application-id", default="cbsms_geode_stage3a")
    parser.add_argument("--recording-id",
                        default="medium_reference_free_observability")
    parser.add_argument("--maximum-alignment-time-difference", type=float, default=0.1)
    parser.add_argument("--internal-factor-graph-available", action="store_true")
    args = parser.parse_args()

    glim = read_tum(args.glim_tum)
    gt = read_tum(args.ground_truth_tum)
    edges = read_rows(args.edge_table)
    if args.health_csv:
        edges = enrich_edges(
            edges, read_scan_diagnostics(args.health_csv, args.glim_log))
    case_rows = read_rows(args.wrong_basin_table) if args.wrong_basin_table else []
    source_indices, target_indices, time_differences = associate_nearest(
        glim[:, 0], gt[:, 0], args.maximum_alignment_time_difference)
    rotation, translation = estimate_se3(
        glim[source_indices, 1:4], gt[target_indices, 1:4])
    aligned_positions = (rotation @ glim[:, 1:4].T).T + translation
    alignment_errors = np.linalg.norm(
        aligned_positions[source_indices] - gt[target_indices, 1:4], axis=1)
    visualization_origin = gt[target_indices[0], 1:4].copy()
    aligned_visual = aligned_positions - visualization_origin
    gt_visual = gt.copy()
    gt_visual[:, 1:4] -= visualization_origin

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    rr.init(args.application_id, recording_id=args.recording_id)
    rr.save(str(args.output))
    factor_graph_note = (
        "The second tab contains fresh internal GLIM fixed-lag snapshots. "
        if args.internal_factor_graph_available else
        "The internal GLIM inspector was unavailable in this recording. ")
    rr.log("documentation", rr.TextDocument(
        f"# {args.dataset}: Stage 3A passive inspector\n\n"
        "Orange: GLIM trajectory after visualization-only global SE(3) alignment. "
        "Green: released full-6DoF ground truth. Red/cyan graph edges are exact "
        "recorded unique G-to-K endpoints, colored by kappa_t > 10. The graph is "
        + factor_graph_note + "Condition ratios are observability descriptors, "
        "not confidence, covariance, probability, or metric error."), static=True)
    gt_coverage = gt_visual[
        (gt_visual[:, 0] >= glim[0, 0]) & (gt_visual[:, 0] <= glim[-1, 0]), 1:4
    ]
    scene_points = np.vstack([aligned_visual, gt_coverage])
    scene_minimum = scene_points.min(axis=0)
    scene_maximum = scene_points.max(axis=0)
    scene_center = 0.5 * (scene_minimum + scene_maximum)
    scene_extent = max(10.0, float(np.max(scene_maximum - scene_minimum)))
    rr.send_blueprint(make_blueprint(scene_center, scene_extent))
    log_static_scene(glim, gt_visual, edges, aligned_visual)
    log_edge_timeline(edges, aligned_visual, gt_visual)
    case_count = log_wrong_basin_geometry(
        case_rows, args.wrong_basin_clouds, args.wrong_basin_manifest)
    rr.disconnect()

    summary = {
        "schema_version": 1,
        "dataset": args.dataset,
        "recording": str(args.output.resolve()),
        "recording_sha256": hashlib.sha256(args.output.read_bytes()).hexdigest(),
        "recording_size_bytes": args.output.stat().st_size,
        "glim_pose_count": len(glim),
        "ground_truth_pose_count": len(gt),
        "unique_g_to_k_edge_count": len(edges),
        "wrong_basin_case_count": case_count,
        "visualization_alignment": {
            "type": "SE3 Umeyama without scale",
            "timestamp_pair_count": len(source_indices),
            "maximum_timestamp_difference_sec": float(time_differences.max()),
            "translation_ape_rmse_m": float(np.sqrt(np.mean(alignment_errors ** 2))),
            "translation_ape_median_m": float(np.median(alignment_errors)),
            "rotation_matrix": rotation.tolist(),
            "translation": translation.tolist(),
            "visualization_origin_subtracted_m": visualization_origin.tolist(),
        },
        "internal_factor_graph_available": args.internal_factor_graph_available,
        "internal_factor_graph_reason": (
            "fresh fixed-lag inspector captured at stride 5"
            if args.internal_factor_graph_available else
            "factor-graph inspector data not supplied"),
        "recorded_edge_graph_available": True,
    }
    args.summary.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n",
                            encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
