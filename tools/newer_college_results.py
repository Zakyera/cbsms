#!/usr/bin/env python3
"""Create, finalize, and aggregate official Newer College benchmark results."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import numpy as np


SCHEMA_VERSION = 1
DATASET_RELEASE = "2021-ouster-os0-128-alphasense"
SEQUENCES = {
    "quad-easy": ("collection1", "Quad-Easy"),
    "quad-medium": ("collection1", "Quad-Medium"),
    "quad-hard": ("collection1", "Quad-Hard"),
    "stairs": ("collection1", "Stairs"),
    "cloister": ("collection2", "Cloister"),
    "park": ("collection2", "Park"),
    "maths-easy": ("collection3", "Maths-Easy"),
    "maths-medium": ("collection3", "Maths-Medium"),
    "maths-hard": ("collection3", "Maths-Hard"),
}
DEFAULT_MODES = ("cbs_off", "cbs_on")
ESTIMATORS = ("glim", "kimera")
STATUS_VALUES = ("PENDING", "PILOT", "COMPLETE", "INCOMPLETE")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(8 * 1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def path_length(positions: np.ndarray) -> float:
    if len(positions) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())


def read_tum(path: Path) -> np.ndarray:
    rows = []
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            text = line.strip()
            if not text or text.startswith("#"):
                continue
            values = [float(value) for value in text.split()[:8]]
            if len(values) != 8 or not all(math.isfinite(value) for value in values):
                raise ValueError(f"invalid TUM row {line_number} in {path}")
            rows.append(values)
    result = np.asarray(rows, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != 8 or len(result) < 2:
        raise ValueError(f"TUM trajectory has fewer than two valid rows: {path}")
    if np.any(np.diff(result[:, 0]) <= 0.0):
        raise ValueError(f"TUM timestamps are not strictly increasing: {path}")
    return result


def interpolate_position(trajectory: np.ndarray, stamp: float) -> np.ndarray:
    times = trajectory[:, 0]
    if stamp < times[0] or stamp > times[-1]:
        raise ValueError(f"timestamp {stamp:.9f} lies outside ground-truth coverage")
    insertion = int(np.searchsorted(times, stamp))
    if insertion < len(times) and abs(times[insertion] - stamp) < 1.0e-9:
        return trajectory[insertion, 1:4].copy()
    if insertion == 0 or insertion == len(times):
        raise ValueError(f"cannot interpolate timestamp {stamp:.9f}")
    before = trajectory[insertion - 1]
    after = trajectory[insertion]
    alpha = float((stamp - before[0]) / (after[0] - before[0]))
    return (1.0 - alpha) * before[1:4] + alpha * after[1:4]


def ground_truth_window(
    trajectory: np.ndarray, start_stamp: float, end_stamp: float
) -> np.ndarray:
    if end_stamp <= start_stamp:
        raise ValueError("non-positive evaluation interval")
    interior = trajectory[
        (trajectory[:, 0] > start_stamp) & (trajectory[:, 0] < end_stamp), :4
    ]
    start = np.concatenate(([start_stamp], interpolate_position(trajectory, start_stamp)))
    end = np.concatenate(([end_stamp], interpolate_position(trajectory, end_stamp)))
    return np.vstack((start, interior, end))


def align_positions(trajectory: np.ndarray, rotation: Any, translation: Any) -> np.ndarray:
    rot = np.asarray(rotation, dtype=np.float64)
    trans = np.asarray(translation, dtype=np.float64)
    if rot.shape != (3, 3) or trans.shape != (3,):
        raise ValueError("invalid SE(3) alignment shape")
    return (rot @ trajectory[:, 1:4].T).T + trans


def associated_errors(
    reference: np.ndarray,
    estimate: np.ndarray,
    aligned_positions: np.ndarray,
    maximum_difference_sec: float,
) -> tuple[np.ndarray, np.ndarray]:
    times = []
    errors = []
    for index, stamp in enumerate(estimate[:, 0]):
        insertion = int(np.searchsorted(reference[:, 0], stamp))
        candidates = []
        if insertion < len(reference):
            candidates.append(insertion)
        if insertion > 0:
            candidates.append(insertion - 1)
        if not candidates:
            continue
        nearest = min(candidates, key=lambda item: abs(reference[item, 0] - stamp))
        if abs(reference[nearest, 0] - stamp) <= maximum_difference_sec:
            times.append(stamp)
            errors.append(float(np.linalg.norm(aligned_positions[index] - reference[nearest, 1:4])))
    if len(errors) < 2:
        raise ValueError("fewer than two timestamp-associated poses")
    return np.asarray(times), np.asarray(errors)


def write_position_csv(path: Path, timestamps: np.ndarray, positions: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("timestamp_sec", "x_m", "y_m", "z_m"))
        for stamp, position in zip(timestamps, positions):
            writer.writerow((f"{stamp:.9f}", *(f"{value:.12g}" for value in position)))


def select_glim_metric(metrics_path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    source = json.loads(metrics_path.read_text(encoding="utf-8"))
    rows = source.get("metrics")
    if not isinstance(rows, list):
        raise ValueError(f"missing metrics list in {metrics_path}")
    matches = [row for row in rows if row.get("name") == "glim_odom_base"]
    if len(matches) != 1:
        raise ValueError(f"expected exactly one glim_odom_base metric in {metrics_path}")
    return matches[0], source


def normalize_estimator_metrics(name: str, row: dict[str, Any]) -> dict[str, Any]:
    if name == "glim":
        return {
            "poses": int(row["samples"]),
            "start_stamp_sec": float(row["start_stamp_sec"]),
            "end_stamp_sec": float(row["end_stamp_sec"]),
            "evaluated_duration_sec": float(row["evaluated_duration_sec"]),
            "translation_ape_rmse_m": float(row["translation_ape_rmse_m"]),
            "translation_ape_mean_m": float(row["translation_ape_mean_m"]),
            "translation_ape_median_m": float(row["translation_ape_median_m"]),
            "translation_ape_std_m": float(row["translation_ape_std_m"]),
            "translation_ape_min_m": float(row["translation_ape_min_m"]),
            "translation_ape_max_m": float(row["translation_ape_max_m"]),
            "aligned_endpoint_error_m": float(row["aligned_endpoint_error_m"]),
            "estimate_distance_m": float(row["estimate_path_length_m"]),
            "reference_distance_m": float(row["reference_path_length_m"]),
            "alignment": row["alignment"],
            "alignment_rotation": row["alignment_rotation"],
            "alignment_translation_m": row["alignment_translation_m"],
            "maximum_association_difference_sec": float(
                row["maximum_association_difference_sec"]
            ),
        }
    ape = row["translation_ape"]
    return {
        "poses": int(row["poses"]),
        "start_stamp_sec": float(row["start_timestamp"]),
        "end_stamp_sec": float(row["end_timestamp"]),
        "evaluated_duration_sec": float(row["estimated_duration_sec"]),
        "translation_ape_rmse_m": float(ape["rmse_m"]),
        "translation_ape_mean_m": float(ape["mean_m"]),
        "translation_ape_median_m": float(ape["median_m"]),
        "translation_ape_std_m": float(ape["std_m"]),
        "translation_ape_min_m": float(ape["min_m"]),
        "translation_ape_max_m": float(ape["max_m"]),
        "aligned_height_rmse_m": float(row["aligned_height_error"]["rmse_m"]),
        "estimate_distance_m": float(row["estimated_path_length_m"]),
        "reference_distance_m": float(row["reference_path_length_m"]),
        "alignment": row["protocol"]["alignment"],
        "alignment_rotation": row["alignment_rotation"],
        "alignment_translation_m": row["alignment_translation_m"],
        "maximum_association_difference_sec": 0.001,
    }


def pending_result(sequence: str, mode: str) -> dict[str, Any]:
    collection, label = SEQUENCES[sequence]
    return {
        "schema_version": SCHEMA_VERSION,
        "status": "PENDING",
        "dataset": {
            "name": "Oxford Newer College Multi-Camera Dataset",
            "release": DATASET_RELEASE,
            "collection": collection,
            "sequence": sequence,
            "sequence_label": label,
        },
        "experiment": {"mode": mode},
        "updated_utc": utc_now(),
    }


def initialize_slot(slot: Path, sequence: str, mode: str) -> None:
    for directory in (
        "metrics",
        "trajectories/raw",
        "trajectories/aligned",
        "figures",
        "rerun",
        "covariances",
        "configs",
        "logs",
        "provenance",
    ):
        (slot / directory).mkdir(parents=True, exist_ok=True)
    result_path = slot / "result.json"
    if not result_path.exists():
        write_json(result_path, pending_result(sequence, mode))
    status_path = slot / "STATUS.md"
    if not status_path.exists():
        status_path.write_text(
            f"# {SEQUENCES[sequence][1]} — {mode}\n\n"
            "**PENDING — NO OFFICIAL RESULT RECORDED.**\n",
            encoding="utf-8",
        )


def init_benchmark(root: Path, modes: Iterable[str]) -> None:
    root.mkdir(parents=True, exist_ok=True)
    modes = tuple(modes)
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "created_utc": utc_now(),
        "dataset": "Oxford Newer College Multi-Camera Dataset",
        "release": DATASET_RELEASE,
        "sequences": list(SEQUENCES),
        "modes": list(modes),
        "evaluation": {
            "frame": "Oxford Multi-Camera Base",
            "metric": "translation APE RMSE [m]",
            "alignment": "Umeyama/Kabsch SE(3), scale fixed at 1",
            "time_offset_fitting": False,
            "sim3_scale_correction": False,
        },
    }
    manifest_path = root / "benchmark_manifest.json"
    if not manifest_path.exists():
        write_json(manifest_path, manifest)
    for sequence in SEQUENCES:
        for mode in modes:
            initialize_slot(root / "sequences" / sequence / mode, sequence, mode)
    (root / "aggregate").mkdir(exist_ok=True)
    (root / "pilots").mkdir(exist_ok=True)
    readme = root / "README.md"
    if not readme.exists():
        readme.write_text(
            "# Official Newer College CBS Benchmark Results\n\n"
            "Each `sequences/<sequence>/<mode>` directory is one official result "
            "slot. Only entries with `status: COMPLETE` are admitted to the paper "
            "table. Pilot and partial runs belong under `pilots/`.\n\n"
            "Run `src/cbsms/tools/newer_college_results.py collect --root <this-dir>` "
            "after finalizing any result.\n",
            encoding="utf-8",
        )


def copy_file(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def copy_tree(source: Path, destination: Path) -> None:
    if destination.exists():
        shutil.copytree(source, destination, dirs_exist_ok=True)
    else:
        shutil.copytree(source, destination)


def artifact_record(path: Path, verified: bool | None = None) -> dict[str, Any]:
    record: dict[str, Any] = {
        "path": str(path.resolve()),
        "size_bytes": path.stat().st_size,
        "sha256": sha256_file(path),
    }
    if verified is not None:
        record["rerun_verified"] = verified
    return record


def verify_rerun(path: Path) -> tuple[bool, str]:
    completed = subprocess.run(
        ["rerun", "rrd", "verify", str(path)],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    return completed.returncode == 0, completed.stdout.strip()


def make_figures(
    figures_dir: Path,
    gt_window: np.ndarray,
    glim_positions: np.ndarray,
    kimera_positions: np.ndarray,
    glim_error_times: np.ndarray,
    glim_errors: np.ndarray,
    kimera_error_times: np.ndarray,
    kimera_errors: np.ndarray,
    title: str,
) -> None:
    from PIL import Image, ImageDraw, ImageFont

    figures_dir.mkdir(parents=True, exist_ok=True)
    colors = {
        "background": (250, 251, 253),
        "grid": (218, 222, 228),
        "axis": (55, 60, 68),
        "gt": (65, 140, 255),
        "glim": (235, 162, 0),
        "kimera": (20, 185, 70),
    }
    font_path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    bold_font_path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf")
    if font_path.is_file() and bold_font_path.is_file():
        font = ImageFont.truetype(str(font_path), 22)
        title_font = ImageFont.truetype(str(bold_font_path), 28)
    else:
        font = ImageFont.load_default()
        title_font = font

    def scaled_polylines(
        series: list[tuple[str, np.ndarray]],
        width: int,
        height: int,
        equal_aspect: bool,
    ) -> list[tuple[str, list[tuple[int, int]]]]:
        all_points = np.vstack([points for _name, points in series])
        minimum = np.min(all_points, axis=0)
        maximum = np.max(all_points, axis=0)
        span = np.maximum(maximum - minimum, 1.0e-9)
        plot_left = 70.0
        plot_top = 90.0
        plot_width = width - 150
        plot_height = height - 165
        if equal_aspect:
            scale = min(plot_width / span[0], plot_height / span[1])
            scales = np.asarray([scale, scale])
        else:
            scales = np.asarray([plot_width / span[0], plot_height / span[1]])
        occupied = span * scales
        offset_x = plot_left + 0.5 * (plot_width - occupied[0])
        offset_y = plot_top + 0.5 * (plot_height - occupied[1])
        result = []
        for name, points in series:
            pixels = np.empty_like(points)
            pixels[:, 0] = offset_x + (points[:, 0] - minimum[0]) * scales[0]
            pixels[:, 1] = offset_y + (maximum[1] - points[:, 1]) * scales[1]
            result.append((name, [(int(x), int(y)) for x, y in pixels]))
        return result

    def save_plot(
        stem: str,
        plot_title: str,
        series: list[tuple[str, np.ndarray]],
        x_label: str,
        y_label: str,
        equal_aspect: bool,
        size: tuple[int, int] = (1500, 1100),
    ) -> None:
        width, height = size
        image = Image.new("RGB", size, colors["background"])
        draw = ImageDraw.Draw(image)
        draw.rectangle((65, 85, width - 70, height - 65), outline=colors["axis"], width=2)
        for fraction in np.linspace(0.0, 1.0, 11):
            x = int(65 + fraction * (width - 135))
            y = int(85 + fraction * (height - 150))
            draw.line((x, 85, x, height - 65), fill=colors["grid"], width=1)
            draw.line((65, y, width - 70, y), fill=colors["grid"], width=1)
        for name, pixels in scaled_polylines(series, width, height, equal_aspect):
            if len(pixels) >= 2:
                draw.line(pixels, fill=colors[name], width=5, joint="curve")
        all_points = np.vstack([points for _name, points in series])
        minimum = np.min(all_points, axis=0)
        maximum = np.max(all_points, axis=0)
        draw.text((70, 18), plot_title, fill=colors["axis"], font=title_font)
        draw.text((width // 2 - 55, height - 35), x_label, fill=colors["axis"], font=font)
        draw.text((8, height // 2), y_label, fill=colors["axis"], font=font)
        draw.text((65, height - 62), f"{minimum[0]:.1f}", fill=colors["axis"], font=font)
        draw.text((width - 125, height - 62), f"{maximum[0]:.1f}", fill=colors["axis"], font=font)
        draw.text((18, height - 90), f"{minimum[1]:.2f}", fill=colors["axis"], font=font)
        draw.text((18, 62), f"{maximum[1]:.2f}", fill=colors["axis"], font=font)
        legend_x = width - 245
        legend_y = 105
        labels = {"gt": "Oxford GT", "glim": "GLIM", "kimera": "Kimera"}
        for name, _points in series:
            draw.line((legend_x, legend_y + 6, legend_x + 45, legend_y + 6), fill=colors[name], width=5)
            draw.text((legend_x + 55, legend_y), labels.get(name, name), fill=colors["axis"], font=font)
            legend_y += 24
        image.save(figures_dir / f"{stem}.png", format="PNG", optimize=True)
        image.save(figures_dir / f"{stem}.pdf", format="PDF", resolution=180.0)

    origin = gt_window[0, 1:4]
    gt = gt_window[:, 1:4] - origin
    glim = glim_positions - origin
    kimera = kimera_positions - origin
    save_plot(
        "trajectories_xy",
        title + " — top view",
        [("gt", gt[:, :2]), ("glim", glim[:, :2]), ("kimera", kimera[:, :2])],
        "x [m]",
        "y [m]",
        True,
    )

    view = np.asarray(
        [[0.819152, -0.573576, 0.0], [-0.196175, -0.280166, 0.939693]],
        dtype=np.float64,
    )
    save_plot(
        "trajectories_3d",
        title + " — isometric 3D projection",
        [
            ("gt", (view @ gt.T).T),
            ("glim", (view @ glim.T).T),
            ("kimera", (view @ kimera.T).T),
        ],
        "isometric horizontal [m]",
        "isometric vertical [m]",
        True,
    )
    save_plot(
        "translation_ape",
        title + " — translation error",
        [
            (
                "glim",
                np.column_stack((glim_error_times - glim_error_times[0], glim_errors)),
            ),
            (
                "kimera",
                np.column_stack((kimera_error_times - kimera_error_times[0], kimera_errors)),
            ),
        ],
        "elapsed estimator time [s]",
        "translation APE [m]",
        False,
        size=(1500, 800),
    )


def markdown_metric(value: float | None, digits: int = 3) -> str:
    return "—" if value is None or not math.isfinite(value) else f"{value:.{digits}f}"


def write_result_report(result_dir: Path, result: dict[str, Any]) -> None:
    dataset = result["dataset"]
    coverage = result["coverage"]
    experiment = result["experiment"]
    lines = [
        f"# {dataset['sequence_label']} — {experiment['mode']}",
        "",
        f"**Status: {result['status']}**",
        "",
        f"- Collection: `{dataset['collection']}`",
        f"- Release: `{dataset['release']}`",
        f"- CBS direction: `{experiment['cbs_direction']}`",
        f"- Full logical sequence: `{str(coverage['full_sequence']).lower()}`",
        f"- Reported Oxford GT time coverage: `{coverage['time_travelled_sec']:.6f} s`",
        f"- Oxford GT distance over that interval: `{coverage['distance_travelled_m']:.6f} m`",
        "",
        "## Estimator metrics",
        "",
        "| Estimator | poses | duration [s] | estimate distance [m] | reference distance [m] | translation APE RMSE [m] |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for name in ESTIMATORS:
        metric = result["estimators"][name]
        lines.append(
            f"| {name.upper()} | {metric['poses']} | "
            f"{metric['evaluated_duration_sec']:.6f} | "
            f"{metric['estimate_distance_m']:.6f} | "
            f"{metric['reference_distance_m']:.6f} | "
            f"{metric['translation_ape_rmse_m']:.6f} |"
        )
    lines.extend(
        [
            "",
            "Evaluation uses Oxford Base, one rigid SE(3) alignment per estimator, "
            "and scale fixed at one. No Sim(3), time-offset fitting, or trajectory deformation is used.",
            "",
            "## Trajectory appearance",
            "",
            "![Top-down trajectories](figures/trajectories_xy.png)",
            "",
            "![3D trajectories](figures/trajectories_3d.png)",
            "",
            "![Translation APE](figures/translation_ape.png)",
            "",
        ]
    )
    covariance = result.get("cbs_factor_covariances")
    if covariance:
        lines.extend(
            [
                "## CBS inserted-factor covariances",
                "",
                f"- Audit status: `{covariance['status']}`",
                f"- G→K accepted factors: `{covariance['g2k_factor_count']}`",
                f"- K→G accepted factors: `{covariance['k2g_factor_count']}`",
                "- Full report: `covariances/REPORT.md`",
                "- Every sent and receiver-inserted 6×6 matrix: "
                "`covariances/accepted_factor_covariances.csv` and `.npz`",
                "",
            ]
        )
    lines.extend(
        [
            "## Source",
            "",
            f"- Source run: `{result['source']['run_dir']}`",
            f"- Recording ID: `{experiment['recording_id']}`",
            "- Exact artifact paths and hashes: `provenance/source_artifacts.tsv`",
            "- Package hashes: `SHA256SUMS`",
            "",
        ]
    )
    (result_dir / "REPORT.md").write_text("\n".join(lines), encoding="utf-8")
    (result_dir / "STATUS.md").write_text(
        f"# {dataset['sequence_label']} — {experiment['mode']}\n\n"
        f"**{result['status']}**\n\n"
        f"Updated: `{result['updated_utc']}`\n",
        encoding="utf-8",
    )


def write_package_hashes(result_dir: Path) -> None:
    paths = sorted(
        path
        for path in result_dir.rglob("*")
        if path.is_file() and path.name != "SHA256SUMS"
    )
    lines = [f"{sha256_file(path)}  {path.relative_to(result_dir)}" for path in paths]
    (result_dir / "SHA256SUMS").write_text("\n".join(lines) + "\n", encoding="utf-8")


def finalize_result(args: argparse.Namespace) -> None:
    result_dir = args.result_dir.resolve()
    sequence = args.sequence
    mode = args.mode
    collection, sequence_label = SEQUENCES[sequence]
    evaluation = args.evaluation_dir.resolve()
    required = {
        "GLIM metrics": evaluation / "metrics.json",
        "Kimera metrics": evaluation / "kimera_metrics_base.json",
        "Oxford ground truth": evaluation / "ground_truth_base.tum",
        "Kimera-associated ground truth": evaluation / "kimera_ground_truth_base.tum",
        "GLIM trajectory": evaluation / "glim_odom_base.tum",
        "Kimera trajectory": evaluation / "kimera_base.tum",
    }
    missing = [f"{name}: {path}" for name, path in required.items() if not path.is_file()]
    if missing:
        raise SystemExit("missing required evaluation artifacts:\n" + "\n".join(missing))
    if args.status == "COMPLETE":
        if not args.full_sequence:
            raise SystemExit("COMPLETE requires --full-sequence")
        if not args.rrd or not args.rbl:
            raise SystemExit("COMPLETE requires --rrd and --rbl")
        if not args.provenance_file:
            raise SystemExit("COMPLETE requires at least one --provenance-file")
        if args.cbs_activity_count is None:
            raise SystemExit("COMPLETE requires --cbs-activity-count")
        if mode == "cbs_on" and not args.covariance_report_dir:
            raise SystemExit("COMPLETE cbs_on requires --covariance-report-dir")
    if mode == "cbs_off" and args.cbs_direction != "none":
        raise SystemExit("cbs_off requires --cbs-direction none")
    if mode == "cbs_on" and args.cbs_direction == "none":
        raise SystemExit("cbs_on requires a non-none --cbs-direction")
    if mode == "cbs_off" and args.cbs_activity_count not in (None, 0):
        raise SystemExit("cbs_off requires --cbs-activity-count 0")
    if mode == "cbs_on" and args.cbs_activity_count is not None and args.cbs_activity_count <= 0:
        raise SystemExit("cbs_on requires a positive --cbs-activity-count")

    covariance_summary = None
    if args.covariance_report_dir:
        covariance_source = args.covariance_report_dir.resolve()
        covariance_summary_path = covariance_source / "summary.json"
        covariance_report_path = covariance_source / "REPORT.md"
        covariance_csv_path = covariance_source / "accepted_factor_covariances.csv"
        covariance_npz_path = covariance_source / "accepted_factor_covariances.npz"
        missing_covariance = [
            str(path)
            for path in (
                covariance_summary_path,
                covariance_report_path,
                covariance_csv_path,
                covariance_npz_path,
            )
            if not path.is_file()
        ]
        if missing_covariance:
            raise SystemExit(
                "missing required CBS covariance artifacts:\n"
                + "\n".join(missing_covariance)
            )
        covariance_summary = json.loads(
            covariance_summary_path.read_text(encoding="utf-8")
        )
        if covariance_summary.get("status") != "COMPLETE":
            raise SystemExit("CBS covariance report status is not COMPLETE")
        directions = covariance_summary.get("directions", {})
        try:
            covariance_activity_count = sum(
                int(directions[direction]["accepted_factor_count"])
                for direction in ("G2K", "K2G")
            )
        except (KeyError, TypeError, ValueError) as error:
            raise SystemExit("invalid CBS covariance direction counts") from error
        if (
            args.cbs_activity_count is not None
            and covariance_activity_count != args.cbs_activity_count
        ):
            raise SystemExit(
                "CBS covariance factor count differs from --cbs-activity-count: "
                f"{covariance_activity_count} versus {args.cbs_activity_count}"
            )
    elif mode == "cbs_off":
        covariance_source = None

    initialize_slot(result_dir, sequence, mode)
    glim_row, _glim_source = select_glim_metric(required["GLIM metrics"])
    kimera_source = json.loads(required["Kimera metrics"].read_text(encoding="utf-8"))
    metrics = {
        "glim": normalize_estimator_metrics("glim", glim_row),
        "kimera": normalize_estimator_metrics("kimera", kimera_source),
    }
    gt = read_tum(required["Oxford ground truth"])
    kimera_gt = read_tum(required["Kimera-associated ground truth"])
    glim = read_tum(required["GLIM trajectory"])
    kimera = read_tum(required["Kimera trajectory"])
    glim_aligned = align_positions(
        glim,
        metrics["glim"]["alignment_rotation"],
        metrics["glim"]["alignment_translation_m"],
    )
    kimera_aligned = align_positions(
        kimera,
        metrics["kimera"]["alignment_rotation"],
        metrics["kimera"]["alignment_translation_m"],
    )
    estimator_start = min(metrics[name]["start_stamp_sec"] for name in ESTIMATORS)
    estimator_end = max(metrics[name]["end_stamp_sec"] for name in ESTIMATORS)
    if args.coverage_start_sec is not None or args.coverage_end_sec is not None:
        if args.coverage_start_sec is None or args.coverage_end_sec is None:
            raise SystemExit("provide both --coverage-start-sec and --coverage-end-sec")
        coverage_start = args.coverage_start_sec
        coverage_end = args.coverage_end_sec
        coverage_definition = "explicit official ground-truth interval"
    elif args.full_sequence:
        coverage_start = float(gt[0, 0])
        coverage_end = float(gt[-1, 0])
        coverage_definition = "complete supplied Oxford ground-truth trajectory"
    else:
        coverage_start = estimator_start
        coverage_end = estimator_end
        coverage_definition = "common union of estimator output intervals"
    gt_window = ground_truth_window(gt, coverage_start, coverage_end)
    glim_error_times, glim_errors = associated_errors(
        gt,
        glim,
        glim_aligned,
        metrics["glim"]["maximum_association_difference_sec"] + 0.001,
    )
    kimera_error_times, kimera_errors = associated_errors(
        kimera_gt,
        kimera,
        kimera_aligned,
        metrics["kimera"]["maximum_association_difference_sec"] + 1.0e-9,
    )
    for name, errors in (("glim", glim_errors), ("kimera", kimera_errors)):
        crosscheck = float(np.sqrt(np.mean(errors * errors)))
        source_rmse = metrics[name]["translation_ape_rmse_m"]
        if abs(crosscheck - source_rmse) > 1.0e-5:
            raise SystemExit(
                f"{name} independent RMSE cross-check differs from source metric: "
                f"{crosscheck:.12g} versus {source_rmse:.12g}"
            )
        metrics[name]["independent_rmse_crosscheck_m"] = crosscheck

    raw_dir = result_dir / "trajectories/raw"
    aligned_dir = result_dir / "trajectories/aligned"
    for source_name, destination_name in (
        ("Oxford ground truth", "ground_truth_base.tum"),
        ("Kimera-associated ground truth", "kimera_ground_truth_base.tum"),
        ("GLIM trajectory", "glim_base.tum"),
        ("Kimera trajectory", "kimera_base.tum"),
    ):
        copy_file(required[source_name], raw_dir / destination_name)
    write_position_csv(aligned_dir / "glim_aligned_positions.csv", glim[:, 0], glim_aligned)
    write_position_csv(
        aligned_dir / "kimera_aligned_positions.csv", kimera[:, 0], kimera_aligned
    )
    write_position_csv(
        aligned_dir / "ground_truth_common_window.csv",
        gt_window[:, 0],
        gt_window[:, 1:4],
    )
    copy_file(required["GLIM metrics"], result_dir / "metrics/glim_metrics_source.json")
    copy_file(
        required["Kimera metrics"], result_dir / "metrics/kimera_metrics_source.json"
    )
    write_json(
        result_dir / "metrics/normalized_metrics.json",
        {"estimators": metrics},
    )

    for index, config_dir in enumerate(args.config_dir or []):
        source = config_dir.resolve()
        copy_tree(source, result_dir / "configs" / f"{index:02d}_{source.name}")
    for source in args.provenance_file or []:
        copy_file(source.resolve(), result_dir / "provenance" / source.name)
    if args.source_report:
        copy_file(args.source_report.resolve(), result_dir / "provenance/source_report.md")
    if args.covariance_report_dir:
        copy_tree(args.covariance_report_dir.resolve(), result_dir / "covariances")

    rerun_artifacts = {}
    artifact_rows = []
    for name, path in (("rrd", args.rrd), ("rbl", args.rbl)):
        if path is None:
            continue
        resolved = path.resolve()
        verified, verify_output = verify_rerun(resolved)
        if args.status == "COMPLETE" and not verified:
            raise SystemExit(f"Rerun verification failed for {resolved}:\n{verify_output}")
        record = artifact_record(resolved, verified)
        record["verify_output"] = verify_output
        rerun_artifacts[name] = record
        artifact_rows.append((name, record))

    source_paths = [("source_run", args.source_run.resolve())]
    source_paths.extend((f"config_{index}", path.resolve()) for index, path in enumerate(args.config_dir or []))
    source_paths.extend(("provenance", path.resolve()) for path in args.provenance_file or [])
    if args.source_report:
        source_paths.append(("source_report", args.source_report.resolve()))
    if args.covariance_report_dir:
        source_paths.extend(
            (
                ("cbs_covariance_report", args.covariance_report_dir.resolve() / "REPORT.md"),
                ("cbs_covariance_summary", args.covariance_report_dir.resolve() / "summary.json"),
            )
        )
    for name, path in source_paths:
        if path.is_file():
            artifact_rows.append((name, artifact_record(path)))

    with (result_dir / "provenance/source_artifacts.tsv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.writer(stream, delimiter="\t", lineterminator="\n")
        writer.writerow(("category", "path", "size_bytes", "sha256", "rerun_verified"))
        for category, record in artifact_rows:
            writer.writerow(
                (
                    category,
                    record["path"],
                    record["size_bytes"],
                    record["sha256"],
                    record.get("rerun_verified", "n/a"),
                )
            )

    result = {
        "schema_version": SCHEMA_VERSION,
        "status": args.status,
        "updated_utc": utc_now(),
        "dataset": {
            "name": "Oxford Newer College Multi-Camera Dataset",
            "release": DATASET_RELEASE,
            "collection": collection,
            "sequence": sequence,
            "sequence_label": sequence_label,
        },
        "experiment": {
            "mode": mode,
            "cbs_direction": args.cbs_direction,
            "cbs_activity_count": args.cbs_activity_count,
            "recording_id": args.recording_id,
            "requested_duration_sec": args.requested_duration_sec,
        },
        "coverage": {
            "full_sequence": bool(args.full_sequence),
            "start_stamp_sec": coverage_start,
            "end_stamp_sec": coverage_end,
            "time_travelled_sec": coverage_end - coverage_start,
            "distance_travelled_m": path_length(gt_window[:, 1:4]),
            "distance_source": "Oxford GT",
            "definition": coverage_definition,
            "estimator_union_start_stamp_sec": estimator_start,
            "estimator_union_end_stamp_sec": estimator_end,
        },
        "evaluation_protocol": {
            "frame": "Oxford Multi-Camera Base",
            "alignment": "Umeyama/Kabsch SE(3), scale fixed at 1",
            "metric": "translation APE RMSE [m]",
            "sim3_scale_correction": False,
            "time_offset_fitting": False,
            "trajectory_deformation": False,
        },
        "estimators": metrics,
        "source": {
            "run_dir": str(args.source_run.resolve()),
            "evaluation_dir": str(evaluation),
        },
        "rerun": rerun_artifacts,
        "notes": args.notes or "",
    }
    if covariance_summary:
        result["cbs_factor_covariances"] = {
            "status": covariance_summary["status"],
            "matrix_order": covariance_summary["matrix_order"],
            "g2k_factor_count": int(
                covariance_summary["directions"]["G2K"]["accepted_factor_count"]
            ),
            "k2g_factor_count": int(
                covariance_summary["directions"]["K2G"]["accepted_factor_count"]
            ),
            "maximum_logged_matrix_relative_error": float(
                covariance_summary["validation"][
                    "maximum_logged_matrix_relative_error"
                ]
            ),
            "report": "covariances/REPORT.md",
            "all_factor_matrices_csv": "covariances/accepted_factor_covariances.csv",
            "all_factor_matrices_npz": "covariances/accepted_factor_covariances.npz",
        }
    write_json(result_dir / "result.json", result)
    title = f"Newer College {sequence_label} — {mode.replace('_', ' ').upper()}"
    make_figures(
        result_dir / "figures",
        gt_window,
        glim_aligned,
        kimera_aligned,
        glim_error_times,
        glim_errors,
        kimera_error_times,
        kimera_errors,
        title,
    )
    write_result_report(result_dir, result)
    write_package_hashes(result_dir)
    print(result_dir)


def load_result(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if value.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"unsupported result schema in {path}")
    return value


def result_metric(
    results: dict[tuple[str, str], dict[str, Any]],
    sequence: str,
    mode: str,
    estimator: str,
) -> float | None:
    result = results.get((sequence, mode))
    if not result or result.get("status") != "COMPLETE":
        return None
    return float(result["estimators"][estimator]["translation_ape_rmse_m"])


def coverage_matches(first: dict[str, Any], second: dict[str, Any]) -> bool:
    first_coverage = first.get("coverage", {})
    second_coverage = second.get("coverage", {})
    required = ("start_stamp_sec", "end_stamp_sec", "distance_travelled_m")
    if not all(name in first_coverage and name in second_coverage for name in required):
        return False
    return (
        abs(first_coverage["start_stamp_sec"] - second_coverage["start_stamp_sec"])
        <= 0.001
        and abs(first_coverage["end_stamp_sec"] - second_coverage["end_stamp_sec"])
        <= 0.001
        and abs(first_coverage["distance_travelled_m"] - second_coverage["distance_travelled_m"])
        <= 0.01
    )


def latex_escape(text: str) -> str:
    return text.replace("_", "\\_").replace("&", "\\&")


def latex_metric(value: float | None, digits: int = 3) -> str:
    return "--" if value is None or not math.isfinite(value) else f"{value:.{digits}f}"


def collect_results(root: Path) -> None:
    root = root.resolve()
    aggregate = root / "aggregate"
    aggregate.mkdir(parents=True, exist_ok=True)
    results: dict[tuple[str, str], dict[str, Any]] = {}
    for path in sorted((root / "sequences").glob("*/*/result.json")):
        result = load_result(path)
        key = (result["dataset"]["sequence"], result["experiment"]["mode"])
        results[key] = result

    long_rows = []
    for sequence in SEQUENCES:
        collection, label = SEQUENCES[sequence]
        for mode in DEFAULT_MODES:
            result = results.get((sequence, mode), pending_result(sequence, mode))
            for estimator in ESTIMATORS:
                metric = result.get("estimators", {}).get(estimator, {})
                coverage = result.get("coverage", {})
                long_rows.append(
                    {
                        "collection": collection,
                        "sequence": sequence,
                        "sequence_label": label,
                        "mode": mode,
                        "estimator": estimator,
                        "status": result["status"],
                        "translation_ape_rmse_m": metric.get("translation_ape_rmse_m", ""),
                        "estimate_distance_m": metric.get("estimate_distance_m", ""),
                        "reference_distance_m": metric.get("reference_distance_m", ""),
                        "evaluated_duration_sec": metric.get("evaluated_duration_sec", ""),
                        "poses": metric.get("poses", ""),
                        "common_gt_distance_m": coverage.get("distance_travelled_m", ""),
                        "common_time_sec": coverage.get("time_travelled_sec", ""),
                    }
                )
    with (aggregate / "official_results_long.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(
            stream, fieldnames=list(long_rows[0]), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(long_rows)

    headers = (
        "Collection",
        "Sequence",
        "GT distance [m]",
        "Time [s]",
        "GLIM off RMSE [m]",
        "Kimera off RMSE [m]",
        "GLIM on RMSE [m]",
        "Kimera on RMSE [m]",
    )
    paper_rows = []
    complete_all = True
    admitted_sequences = set()
    complete_on_directions = {
        result.get("experiment", {}).get("cbs_direction")
        for (sequence, mode), result in results.items()
        if mode == "cbs_on" and result.get("status") == "COMPLETE"
    }
    complete_on_directions.discard(None)
    direction_consistent = len(complete_on_directions) <= 1
    for sequence, (collection, label) in SEQUENCES.items():
        off = results.get((sequence, "cbs_off"))
        on = results.get((sequence, "cbs_on"))
        complete_pair = bool(
            off
            and on
            and off.get("status") == "COMPLETE"
            and on.get("status") == "COMPLETE"
            and off.get("coverage", {}).get("full_sequence")
            and on.get("coverage", {}).get("full_sequence")
            and coverage_matches(off, on)
            and direction_consistent
        )
        complete_all = complete_all and complete_pair
        if complete_pair:
            admitted_sequences.add(sequence)
        coverage = off.get("coverage", {}) if complete_pair else {}
        paper_rows.append(
            (
                collection,
                label,
                coverage.get("distance_travelled_m"),
                coverage.get("time_travelled_sec"),
                result_metric(results, sequence, "cbs_off", "glim") if complete_pair else None,
                result_metric(results, sequence, "cbs_off", "kimera") if complete_pair else None,
                result_metric(results, sequence, "cbs_on", "glim") if complete_pair else None,
                result_metric(results, sequence, "cbs_on", "kimera") if complete_pair else None,
            )
        )

    markdown = [
        "# Official Newer College CBS Results",
        "",
        "Only `COMPLETE` full-sequence packages are admitted. PILOT, INCOMPLETE, and PENDING runs are excluded.",
        "All admitted CBS-on packages must use one consistent exchange direction.",
        f"Observed complete CBS-on directions: `{', '.join(sorted(complete_on_directions)) or 'none yet'}`.",
        "",
        "| " + " | ".join(headers) + " |",
        "|---|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in paper_rows:
        markdown.append(
            "| "
            + " | ".join(
                (str(row[0]), str(row[1]))
                + tuple(markdown_metric(value) for value in row[2:])
            )
            + " |"
        )
    if complete_all:
        means = []
        for mode, estimator in (
            ("cbs_off", "glim"),
            ("cbs_off", "kimera"),
            ("cbs_on", "glim"),
            ("cbs_on", "kimera"),
        ):
            values = [result_metric(results, sequence, mode, estimator) for sequence in SEQUENCES]
            means.append(float(np.mean(values)))
        markdown.append(
            "| — | **Average** | — | — | "
            + " | ".join(markdown_metric(value) for value in means)
            + " |"
        )
    markdown.extend(
        [
            "",
            f"Full 9-sequence off/on matrix complete: **{'YES' if complete_all else 'NO'}**.",
            "",
        ]
    )
    (aggregate / "paper_table.md").write_text("\n".join(markdown), encoding="utf-8")

    latex = [
        "\\begin{tabular}{llrrrrrr}",
        "\\toprule",
        "Collection & Sequence & GT dist. [m] & Time [s] & GLIM off & Kimera off & GLIM on & Kimera on \\\\",
        "\\midrule",
    ]
    for row in paper_rows:
        values = [latex_escape(str(row[0])), latex_escape(str(row[1]))]
        values.extend(latex_metric(value) for value in row[2:])
        latex.append(" & ".join(values) + " \\\\")
    latex.extend(("\\bottomrule", "\\end{tabular}", ""))
    (aggregate / "paper_table.tex").write_text("\n".join(latex), encoding="utf-8")

    completion = [
        "# Completion Matrix",
        "",
        "| Sequence | CBS off | CBS on | Paper-table pair |",
        "|---|---|---|---|",
    ]
    for sequence, (_collection, label) in SEQUENCES.items():
        statuses = []
        for mode in DEFAULT_MODES:
            result = results.get((sequence, mode))
            statuses.append(result.get("status", "MISSING") if result else "MISSING")
        pair_note = "admitted" if sequence in admitted_sequences else "not admitted"
        completion.append(
            f"| {label} | {statuses[0]} | {statuses[1]} | {pair_note} |"
        )
    (aggregate / "completion_matrix.md").write_text(
        "\n".join(completion) + "\n", encoding="utf-8"
    )

    figure_index = ["# Trajectory Figure Index", ""]
    for sequence, (_collection, label) in SEQUENCES.items():
        for mode in DEFAULT_MODES:
            result = results.get((sequence, mode))
            if result and result.get("status") == "COMPLETE":
                relative = Path("../sequences") / sequence / mode / "figures/trajectories_xy.png"
                figure_index.extend((f"## {label} — {mode}", "", f"![{label} {mode}]({relative})", ""))
    (aggregate / "trajectory_figures.md").write_text(
        "\n".join(figure_index), encoding="utf-8"
    )
    write_json(
        aggregate / "official_results.json",
        {
            "schema_version": SCHEMA_VERSION,
            "generated_utc": utc_now(),
            "full_matrix_complete": complete_all,
            "admitted_sequences": sorted(admitted_sequences),
            "complete_cbs_on_directions": sorted(complete_on_directions),
            "cbs_on_direction_consistent": direction_consistent,
            "results": {
                f"{sequence}/{mode}": result
                for (sequence, mode), result in sorted(results.items())
            },
        },
    )
    print(aggregate / "paper_table.md")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)

    init_parser = subparsers.add_parser("init", help="Create all official result slots.")
    init_parser.add_argument("--root", type=Path, required=True)
    init_parser.add_argument("--modes", nargs="+", default=list(DEFAULT_MODES))

    finalize = subparsers.add_parser("finalize", help="Finalize one result package.")
    finalize.add_argument("--result-dir", type=Path, required=True)
    finalize.add_argument("--sequence", choices=SEQUENCES, required=True)
    finalize.add_argument("--mode", choices=DEFAULT_MODES, required=True)
    finalize.add_argument("--status", choices=STATUS_VALUES[1:], required=True)
    finalize.add_argument("--source-run", type=Path, required=True)
    finalize.add_argument("--evaluation-dir", type=Path, required=True)
    finalize.add_argument("--recording-id", required=True)
    finalize.add_argument("--requested-duration-sec", type=float)
    finalize.add_argument("--full-sequence", action="store_true")
    finalize.add_argument("--coverage-start-sec", type=float)
    finalize.add_argument("--coverage-end-sec", type=float)
    finalize.add_argument(
        "--cbs-direction",
        choices=("none", "g2k", "k2g", "bidirectional"),
        default="none",
    )
    finalize.add_argument("--cbs-activity-count", type=int)
    finalize.add_argument("--rrd", type=Path)
    finalize.add_argument("--rbl", type=Path)
    finalize.add_argument("--source-report", type=Path)
    finalize.add_argument("--covariance-report-dir", type=Path)
    finalize.add_argument("--config-dir", type=Path, action="append")
    finalize.add_argument("--provenance-file", type=Path, action="append")
    finalize.add_argument("--notes")

    collect = subparsers.add_parser("collect", help="Regenerate aggregate tables.")
    collect.add_argument("--root", type=Path, required=True)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.command == "init":
        init_benchmark(args.root, args.modes)
    elif args.command == "finalize":
        finalize_result(args)
    elif args.command == "collect":
        collect_results(args.root)


if __name__ == "__main__":
    main()
