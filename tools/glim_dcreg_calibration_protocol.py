#!/usr/bin/env python3
"""Validate the GLIM DCReg multi-dataset calibration manifest.

This tool deliberately does not fit a calibration curve.  It inventories
ground-truth content, enforces sequence-level split rules, and makes blockers
such as an unverified ground-truth-to-MID360 transform explicit before any
model is allowed to learn from an experiment.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence, Tuple


DEFAULT_MANIFEST_RELATIVE = (
    "src/cbsms/config/glim_dcreg_calibration_manifest_v1.json"
)
VALID_STATUSES = {
    "calibration",
    "validation",
    "test",
    "collect_pending_extrinsic",
    "evidence_only",
    "excluded_sensor_profile",
}
FIT_PARTITIONS = {"calibration", "validation", "test"}


def _normalize_quaternion(
    q: Sequence[float],
) -> Tuple[float, float, float, float]:
    norm = math.sqrt(sum(float(value) ** 2 for value in q))
    if norm <= 1.0e-15 or not math.isfinite(norm):
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(float(value) / norm for value in q)  # type: ignore[return-value]


def _quat_multiply(
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


def _orientation_span_rad(
    quaternions: Sequence[Sequence[float]],
) -> float:
    if not quaternions:
        return 0.0
    first = _normalize_quaternion(quaternions[0])
    inverse = (-first[0], -first[1], -first[2], first[3])
    span = 0.0
    for quaternion in quaternions:
        relative = _normalize_quaternion(
            _quat_multiply(inverse, _normalize_quaternion(quaternion))
        )
        span = max(
            span,
            2.0
            * math.acos(max(-1.0, min(1.0, abs(relative[3])))),
        )
    return span


def read_tum_pose_file(path: Path) -> Dict[str, Any]:
    timestamps: List[float] = []
    positions: List[Tuple[float, float, float]] = []
    quaternions: List[Tuple[float, float, float, float]] = []
    malformed = 0
    quaternion_norm_errors: List[float] = []
    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.split()
            if len(fields) < 8:
                malformed += 1
                continue
            try:
                values = [float(value) for value in fields[:8]]
            except ValueError:
                malformed += 1
                continue
            if not all(math.isfinite(value) for value in values):
                malformed += 1
                continue
            timestamps.append(values[0])
            positions.append((values[1], values[2], values[3]))
            quaternion = (values[4], values[5], values[6], values[7])
            quaternions.append(quaternion)
            quaternion_norm_errors.append(
                abs(
                    math.sqrt(sum(value * value for value in quaternion))
                    - 1.0
                )
            )
    if not timestamps:
        raise ValueError(f"no valid TUM poses in {path}")
    backward_steps = [
        timestamps[index - 1] - timestamps[index]
        for index in range(1, len(timestamps))
        if timestamps[index] < timestamps[index - 1]
    ]
    duplicate_steps = sum(
        timestamps[index] == timestamps[index - 1]
        for index in range(1, len(timestamps))
    )
    minimum = tuple(min(position[axis] for position in positions) for axis in range(3))
    maximum = tuple(max(position[axis] for position in positions) for axis in range(3))
    position_span = math.sqrt(
        sum((maximum[axis] - minimum[axis]) ** 2 for axis in range(3))
    )
    orientation_span = _orientation_span_rad(quaternions)
    return {
        "pose_count": len(timestamps),
        "malformed_rows": malformed,
        "start_sec": timestamps[0],
        "end_sec": timestamps[-1],
        "duration_sec": timestamps[-1] - timestamps[0],
        "negative_timestamp_steps": len(backward_steps),
        "max_backward_timestamp_step_sec": (
            max(backward_steps) if backward_steps else 0.0
        ),
        "duplicate_timestamp_steps": duplicate_steps,
        "position_bbox_diagonal_m": position_span,
        "orientation_span_rad": orientation_span,
        "orientation_span_deg": math.degrees(orientation_span),
        "max_quaternion_norm_error": max(quaternion_norm_errors),
        "observed_pose_content": (
            "full_pose" if orientation_span > 1.0e-6 else "position_only"
        ),
    }


def _resolve(workspace: Path, value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else workspace / path


def load_manifest(path: Path) -> Dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("manifest root must be a JSON object")
    return data


def validate_manifest(
    manifest: Dict[str, Any],
    workspace: Path,
) -> Dict[str, Any]:
    errors: List[str] = []
    warnings: List[str] = []
    sequence_reports: List[Dict[str, Any]] = []
    if manifest.get("schema_version") != 1:
        errors.append("schema_version must be 1")
    sequences = manifest.get("sequences")
    if not isinstance(sequences, list) or not sequences:
        errors.append("sequences must be a non-empty list")
        sequences = []

    ids: set[str] = set()
    split_group_partitions: Dict[str, set[str]] = {}
    for index, sequence in enumerate(sequences):
        if not isinstance(sequence, dict):
            errors.append(f"sequence {index} is not an object")
            continue
        sequence_id = str(sequence.get("id", ""))
        status = str(sequence.get("status", ""))
        split_group = str(sequence.get("split_group", ""))
        prefix = sequence_id or f"sequence[{index}]"
        if not sequence_id:
            errors.append(f"sequence {index} has no id")
        elif sequence_id in ids:
            errors.append(f"duplicate sequence id: {sequence_id}")
        ids.add(sequence_id)
        if status not in VALID_STATUSES:
            errors.append(f"{prefix}: invalid status {status!r}")
        if not split_group:
            errors.append(f"{prefix}: split_group is required")
        if status in FIT_PARTITIONS:
            split_group_partitions.setdefault(split_group, set()).add(status)

        bag_value = str(sequence.get("host_bag", ""))
        gt_value = str(sequence.get("ground_truth", ""))
        bag_path = _resolve(workspace, bag_value) if bag_value else Path()
        gt_path = _resolve(workspace, gt_value) if gt_value else Path()
        report: Dict[str, Any] = {
            "id": sequence_id,
            "dataset_family": sequence.get("dataset_family", ""),
            "environment": sequence.get("environment", ""),
            "split_group": split_group,
            "status": status,
            "lidar_model": sequence.get("lidar_model", ""),
            "bag_exists": bool(bag_value and bag_path.is_file()),
            "ground_truth_exists": bool(gt_value and gt_path.is_file()),
            "bag_size_bytes": (
                bag_path.stat().st_size
                if bag_value and bag_path.is_file()
                else 0
            ),
            "calibration_ready": status in FIT_PARTITIONS,
        }
        if not report["bag_exists"]:
            errors.append(f"{prefix}: bag does not exist: {bag_path}")
        if not report["ground_truth_exists"]:
            errors.append(f"{prefix}: ground truth does not exist: {gt_path}")
        if report["ground_truth_exists"]:
            try:
                gt_report = read_tum_pose_file(gt_path)
                report.update(gt_report)
                declared = sequence.get("ground_truth_kind")
                observed = gt_report["observed_pose_content"]
                if declared != observed:
                    errors.append(
                        f"{prefix}: declared ground_truth_kind={declared!r} "
                        f"but observed {observed!r}"
                    )
                if gt_report["negative_timestamp_steps"]:
                    message = (
                        f"{prefix}: {gt_report['negative_timestamp_steps']} "
                        "ground-truth timestamp steps go backward; maximum "
                        f"{gt_report['max_backward_timestamp_step_sec']:.9g} s"
                    )
                    if gt_report["max_backward_timestamp_step_sec"] > 1.0e-3:
                        errors.append(message)
                    else:
                        warnings.append(message)
                if gt_report["malformed_rows"]:
                    warnings.append(
                        f"{prefix}: {gt_report['malformed_rows']} malformed GT rows"
                    )
                bag_start = sequence.get("bag_start_sec")
                bag_end = sequence.get("bag_end_sec")
                if bag_start is not None and bag_end is not None:
                    overlap = max(
                        0.0,
                        min(float(bag_end), gt_report["end_sec"])
                        - max(float(bag_start), gt_report["start_sec"]),
                    )
                    bag_duration = max(0.0, float(bag_end) - float(bag_start))
                    report["bag_gt_overlap_sec"] = overlap
                    report["bag_gt_overlap_fraction"] = (
                        overlap / bag_duration if bag_duration > 0.0 else 0.0
                    )
                    if overlap <= 0.0:
                        errors.append(f"{prefix}: bag and ground truth do not overlap")
            except (OSError, ValueError) as exc:
                errors.append(f"{prefix}: ground-truth parse failed: {exc}")

        extrinsic = sequence.get("ground_truth_to_glim_pose_extrinsic", {})
        extrinsic_status = (
            str(extrinsic.get("status", "unspecified"))
            if isinstance(extrinsic, dict)
            else "invalid"
        )
        report["extrinsic_status"] = extrinsic_status
        if status in FIT_PARTITIONS and extrinsic_status != "verified":
            errors.append(
                f"{prefix}: fit partition requires a verified "
                "ground-truth-to-GLIM pose transform"
            )
        if status == "collect_pending_extrinsic":
            report["calibration_ready"] = False
            warnings.append(
                f"{prefix}: collect only; do not fit until the tracked-body "
                "to MID360 transform is verified"
            )
        reference_value = str(sequence.get("reference_shadow_run", ""))
        if reference_value:
            reference_path = _resolve(workspace, reference_value)
            report["reference_shadow_run"] = str(reference_path)
            report["reference_shadow_run_exists"] = reference_path.is_dir()
            run_manifest_path = reference_path / "manifest.json"
            run_summary_path = reference_path / "summary.json"
            if not run_manifest_path.is_file() or not run_summary_path.is_file():
                errors.append(
                    f"{prefix}: reference shadow run lacks manifest.json "
                    f"or summary.json: {reference_path}"
                )
            else:
                run_manifest = json.loads(
                    run_manifest_path.read_text(encoding="utf-8")
                )
                run_summary = json.loads(
                    run_summary_path.read_text(encoding="utf-8")
                )
                report["reference_run_returncode"] = run_summary.get(
                    "roslaunch_returncode"
                )
                report["reference_run_duration_sec"] = run_manifest.get(
                    "duration_sec"
                )
                report["reference_run_bag_rate"] = run_manifest.get(
                    "launch_args", {}
                ).get("bag_rate")
                dcreg_rows = run_summary.get("glim_dcreg_summary", [])
                if dcreg_rows:
                    dcreg = dcreg_rows[0]
                    for key in (
                        "count",
                        "valid_count",
                        "invalid_count",
                        "rotation_condition_p50",
                        "rotation_condition_p95",
                        "rotation_condition_max",
                        "translation_condition_p50",
                        "translation_condition_p95",
                        "translation_condition_max",
                        "rotation_weak_scan_count",
                        "translation_weak_scan_count",
                    ):
                        report[f"reference_{key}"] = dcreg.get(key)
                else:
                    errors.append(
                        f"{prefix}: reference run has no GLIM DCReg summary"
                    )
                if report["reference_run_returncode"] != 0:
                    errors.append(
                        f"{prefix}: reference run return code is "
                        f"{report['reference_run_returncode']}"
                    )
                if int(report.get("reference_valid_count", 0)) < 500:
                    errors.append(
                        f"{prefix}: reference run has fewer than 500 valid "
                        "DCReg scans and is not a complete 60-second collection"
                    )
                recorded_sequence = (
                    run_manifest.get("dcreg_calibration", {})
                    .get("sequence", {})
                    .get("id")
                )
                if recorded_sequence and recorded_sequence != sequence_id:
                    errors.append(
                        f"{prefix}: reference run records DCReg sequence "
                        f"{recorded_sequence!r}"
                    )
        sequence_reports.append(report)

    for split_group, partitions in split_group_partitions.items():
        if len(partitions) > 1:
            errors.append(
                f"split_group {split_group!r} appears in multiple fit "
                f"partitions: {sorted(partitions)}"
            )

    partition_counts = {
        partition: sum(
            report["status"] == partition for report in sequence_reports
        )
        for partition in sorted(FIT_PARTITIONS)
    }
    required = (
        manifest.get("split_policy", {})
        .get("minimum_distinct_source_sequences", {})
    )
    split_frozen = all(
        partition_counts.get(partition, 0) >= int(required.get(partition, 0))
        for partition in FIT_PARTITIONS
    )
    if not split_frozen:
        warnings.append(
            "calibration split is not frozen or sufficiently populated; "
            "model fitting remains prohibited"
        )
    return {
        "manifest_id": manifest.get("manifest_id", ""),
        "schema_version": manifest.get("schema_version"),
        "scope": manifest.get("scope", {}),
        "sequence_count": len(sequence_reports),
        "errors": errors,
        "warnings": warnings,
        "valid": not errors,
        "split_frozen": split_frozen,
        "model_fitting_allowed": not errors and split_frozen,
        "partition_counts": partition_counts,
        "sequences": sequence_reports,
    }


def _markdown_table(headers: Sequence[str], rows: Iterable[Sequence[Any]]) -> str:
    header = "| " + " | ".join(headers) + " |"
    separator = "|" + "|".join("---" for _ in headers) + "|"
    body = [
        "| " + " | ".join(str(value) for value in row) + " |"
        for row in rows
    ]
    return "\n".join([header, separator, *body])


def render_markdown(report: Dict[str, Any]) -> str:
    lines = [
        f"# {report['manifest_id']} validation",
        "",
        f"- Valid inventory: `{report['valid']}`",
        f"- Split frozen: `{report['split_frozen']}`",
        f"- Model fitting allowed: `{report['model_fitting_allowed']}`",
        f"- Sequences: `{report['sequence_count']}`",
        "",
        "## Sequence inventory",
        "",
        _markdown_table(
            [
                "sequence",
                "sensor",
                "status",
                "GT",
                "poses",
                "orientation span deg",
                "extrinsic",
                "ready",
            ],
            (
                [
                    row["id"],
                    row["lidar_model"],
                    row["status"],
                    row.get("observed_pose_content", "unreadable"),
                    row.get("pose_count", 0),
                    f"{row.get('orientation_span_deg', math.nan):.3f}",
                    row.get("extrinsic_status", ""),
                    row["calibration_ready"],
                ]
                for row in report["sequences"]
            ),
        ),
        "",
        "## Partition counts",
        "",
        _markdown_table(
            ["partition", "source sequences"],
            (
                [partition, count]
                for partition, count in report["partition_counts"].items()
            ),
        ),
    ]
    reference_rows = [
        row for row in report["sequences"] if row.get("reference_shadow_run")
    ]
    if reference_rows:
        lines.extend(
            [
                "",
                "## Reference shadow collections",
                "",
                _markdown_table(
                    [
                        "sequence",
                        "valid scans",
                        "rotation condition p50/p95/max",
                        "translation condition p50/p95/max",
                        "weak scans R/T",
                        "return",
                    ],
                    (
                        [
                            row["id"],
                            row.get("reference_valid_count", 0),
                            (
                                f"{row.get('reference_rotation_condition_p50', math.nan):.3f} / "
                                f"{row.get('reference_rotation_condition_p95', math.nan):.3f} / "
                                f"{row.get('reference_rotation_condition_max', math.nan):.3f}"
                            ),
                            (
                                f"{row.get('reference_translation_condition_p50', math.nan):.3f} / "
                                f"{row.get('reference_translation_condition_p95', math.nan):.3f} / "
                                f"{row.get('reference_translation_condition_max', math.nan):.3f}"
                            ),
                            (
                                f"{row.get('reference_rotation_weak_scan_count', 0)} / "
                                f"{row.get('reference_translation_weak_scan_count', 0)}"
                            ),
                            row.get("reference_run_returncode", "n/a"),
                        ]
                        for row in reference_rows
                    ),
                ),
            ]
        )
    if report["errors"]:
        lines.extend(["", "## Errors", ""])
        lines.extend(f"- {message}" for message in report["errors"])
    if report["warnings"]:
        lines.extend(["", "## Warnings", ""])
        lines.extend(f"- {message}" for message in report["warnings"])
    lines.append("")
    return "\n".join(lines)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workspace",
        type=Path,
        default=Path(__file__).resolve().parents[3],
    )
    parser.add_argument("--manifest", type=Path, default=None)
    parser.add_argument("--json-output", type=Path, default=None)
    parser.add_argument("--markdown-output", type=Path, default=None)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    workspace = args.workspace.resolve()
    manifest_path = (
        args.manifest.resolve()
        if args.manifest
        else workspace / DEFAULT_MANIFEST_RELATIVE
    )
    manifest = load_manifest(manifest_path)
    report = validate_manifest(manifest, workspace)
    markdown = render_markdown(report)
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    if args.markdown_output:
        args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_output.write_text(markdown, encoding="utf-8")
    print(markdown)
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
