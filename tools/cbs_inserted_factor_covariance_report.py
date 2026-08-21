#!/usr/bin/env python3
"""Report every full 6x6 covariance used by accepted CBS odometry factors.

The ROS belief bag is the lossless matrix source.  Receiver log rows identify
which nested belief-window entries became factors and provide independent
trace/diagonal checks.  G2K matrices are transformed from GLIM's external body
frame into Kimera's base frame with the production Pose3 adjoint convention.
K2G messages are already published in GLIM's receiver frame.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np


ORDER = ("rot_x", "rot_y", "rot_z", "trans_x", "trans_y", "trans_z")
ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")


@dataclass
class FactorEvent:
    direction: str
    sender_from: int
    sender_to: int
    receiver_from: int
    receiver_to: int
    marker_trace: float
    marker_line: int
    action: str
    log_trace: float = math.nan
    log_rot_trace: float = math.nan
    log_trans_trace: float = math.nan
    log_diag: np.ndarray | None = None
    covariance_line: int = 0


@dataclass
class BagCandidate:
    direction: str
    sender_from: int
    sender_to: int
    from_stamp_sec: float
    to_stamp_sec: float
    bag_stamp_sec: float
    message_index: int
    topic_message_index: int
    belief_ordinal: int
    sent_covariance: np.ndarray
    inserted_covariance: np.ndarray


def fields_after(line: str, marker: str) -> list[str] | None:
    if marker not in line:
        return None
    payload = ANSI_RE.sub("", line.split(marker, 1)[1]).strip()
    return [field.strip() for field in payload.split(",")]


def pair_from_token(token: str) -> tuple[int, int]:
    values = [int(value) for value in re.findall(r"\d+", token)]
    if len(values) != 2:
        raise ValueError(f"cannot parse pose pair from {token!r}")
    return values[0], values[1]


def key_index(token: str) -> int:
    values = re.findall(r"\d+", token)
    if len(values) != 1:
        raise ValueError(f"cannot parse pose index from {token!r}")
    return int(values[0])


def parse_log(path: Path) -> tuple[list[FactorEvent], list[dict[str, Any]]]:
    events: list[FactorEvent] = []
    covariance_rows: list[dict[str, Any]] = []
    with path.open(encoding="utf-8", errors="replace") as stream:
        for line_number, line in enumerate(stream, 1):
            fields = fields_after(line, "CBS_BPSAM_ODOM_ADD_ROW_G2K,")
            if fields is not None and len(fields) >= 5:
                if fields[4] == "sender_odom_queued":
                    sender_from, sender_to = pair_from_token(fields[0])
                    receiver_from, receiver_to = pair_from_token(fields[1])
                    events.append(
                        FactorEvent(
                            "G2K",
                            sender_from,
                            sender_to,
                            receiver_from,
                            receiver_to,
                            float(fields[3]),
                            line_number,
                            fields[4],
                        )
                    )
                continue

            fields = fields_after(line, "GLIM_CBS_ODOM_INJECT_ROW,")
            if fields is not None and len(fields) >= 6:
                if fields[5] == "injected_persistent":
                    sender_from, sender_to = pair_from_token(fields[0])
                    receiver_from, receiver_to = pair_from_token(fields[1])
                    events.append(
                        FactorEvent(
                            "K2G",
                            sender_from,
                            sender_to,
                            receiver_from,
                            receiver_to,
                            float(fields[4]),
                            line_number,
                            fields[5],
                        )
                    )
                continue

            fields = fields_after(line, "CBS_ODOM_FACTOR_COVARIANCE_ROW,")
            if fields is None or len(fields) < 19 or fields[0] not in ("G2K", "K2G"):
                continue
            covariance_rows.append(
                {
                    "direction": fields[0],
                    "receiver_from": key_index(fields[3]),
                    "receiver_to": key_index(fields[4]),
                    "action": fields[5],
                    "trace": float(fields[6]),
                    "rot_trace": float(fields[7]),
                    "trans_trace": float(fields[8]),
                    "diag": np.asarray([float(value) for value in fields[9:15]]),
                    "line": line_number,
                }
            )

    event_groups: dict[tuple[str, int, int], list[FactorEvent]] = defaultdict(list)
    covariance_groups: dict[tuple[str, int, int], list[dict[str, Any]]] = defaultdict(list)
    for event in events:
        event_groups[(event.direction, event.receiver_from, event.receiver_to)].append(event)
    for row in covariance_rows:
        covariance_groups[
            (row["direction"], row["receiver_from"], row["receiver_to"])
        ].append(row)

    errors = []
    for key in sorted(set(event_groups) | set(covariance_groups)):
        grouped_events = event_groups.get(key, [])
        grouped_rows = covariance_groups.get(key, [])
        if len(grouped_events) != len(grouped_rows):
            errors.append(f"{key}: {len(grouped_events)} events vs {len(grouped_rows)} covariance rows")
            continue
        for event, row in zip(grouped_events, grouped_rows):
            event.log_trace = row["trace"]
            event.log_rot_trace = row["rot_trace"]
            event.log_trans_trace = row["trans_trace"]
            event.log_diag = row["diag"]
            event.covariance_line = row["line"]
    if errors:
        raise ValueError("receiver event/covariance association failed:\n" + "\n".join(errors[:20]))
    return events, covariance_rows


def quaternion_xyzw_to_rotation(values: Iterable[float]) -> np.ndarray:
    x, y, z, w = (float(value) for value in values)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("invalid quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def pose3_adjoint(translation: Iterable[float], quaternion_xyzw: Iterable[float]) -> np.ndarray:
    translation = np.asarray(tuple(translation), dtype=np.float64)
    rotation = quaternion_xyzw_to_rotation(quaternion_xyzw)
    skew = np.asarray(
        [
            [0.0, -translation[2], translation[1]],
            [translation[2], 0.0, -translation[0]],
            [-translation[1], translation[0], 0.0],
        ]
    )
    adjoint = np.zeros((6, 6), dtype=np.float64)
    adjoint[:3, :3] = rotation
    adjoint[3:, :3] = skew @ rotation
    adjoint[3:, 3:] = rotation
    return adjoint


def read_bag_candidates(
    path: Path,
    g2k_topic: str,
    k2g_topic: str,
    g2k_adjoint: np.ndarray,
    g2k_scale: float,
    k2g_scale: float,
) -> list[BagCandidate]:
    try:
        import rosbag  # type: ignore
    except ImportError as error:
        raise SystemExit(
            "rosbag is unavailable; run this tool in the ROS 1 experiment container"
        ) from error

    candidates: list[BagCandidate] = []
    topic_indices: dict[str, int] = defaultdict(int)
    with rosbag.Bag(str(path), "r") as bag:
        for message_index, (topic, message, bag_stamp) in enumerate(
            bag.read_messages(topics=[g2k_topic, k2g_topic])
        ):
            direction = "G2K" if topic == g2k_topic else "K2G"
            topic_message_index = topic_indices[topic]
            topic_indices[topic] += 1
            for belief_ordinal, belief in enumerate(message.beliefs):
                sent = np.asarray(belief.covariance, dtype=np.float64).reshape(6, 6)
                if direction == "G2K":
                    inserted = g2k_scale * (g2k_adjoint @ sent @ g2k_adjoint.T)
                else:
                    inserted = k2g_scale * sent
                candidates.append(
                    BagCandidate(
                        direction=direction,
                        sender_from=int(belief.from_pose_index),
                        sender_to=int(belief.to_pose_index),
                        from_stamp_sec=float(belief.from_stamp_sec),
                        to_stamp_sec=float(belief.to_stamp_sec),
                        bag_stamp_sec=float(bag_stamp.to_sec()),
                        message_index=message_index,
                        topic_message_index=topic_message_index,
                        belief_ordinal=belief_ordinal,
                        sent_covariance=sent,
                        inserted_covariance=inserted,
                    )
                )
    return candidates


def normalized_error(actual: np.ndarray, expected: np.ndarray) -> float:
    return float(np.linalg.norm(actual - expected) / max(np.linalg.norm(expected), 1.0e-18))


def match_events(
    events: list[FactorEvent], candidates: list[BagCandidate]
) -> list[tuple[FactorEvent, BagCandidate, float]]:
    candidate_groups: dict[tuple[str, int, int], list[BagCandidate]] = defaultdict(list)
    for candidate in candidates:
        candidate_groups[
            (candidate.direction, candidate.sender_from, candidate.sender_to)
        ].append(candidate)

    used: set[tuple[int, int]] = set()
    last_position = {"G2K": -1, "K2G": -1}
    matches: list[tuple[FactorEvent, BagCandidate, float]] = []
    for event in sorted(events, key=lambda item: item.marker_line):
        key = (event.direction, event.sender_from, event.sender_to)
        choices = candidate_groups.get(key, [])
        if not choices:
            raise ValueError(f"no bag covariance for accepted {key}")
        expected_diag = event.log_diag
        if expected_diag is None:
            raise ValueError(f"no receiver covariance diagnostic for event at line {event.marker_line}")

        scored = []
        for candidate in choices:
            identity = (candidate.message_index, candidate.belief_ordinal)
            if identity in used:
                continue
            covariance = candidate.inserted_covariance
            trace_error = abs(float(np.trace(covariance)) - event.log_trace) / max(
                abs(event.log_trace), 1.0e-18
            )
            diag_error = normalized_error(np.diag(covariance), expected_diag)
            order_penalty = 0.0 if candidate.message_index >= last_position[event.direction] else 1.0e3
            scored.append((trace_error + diag_error + order_penalty, trace_error + diag_error, candidate))
        if not scored:
            raise ValueError(f"all bag covariances already consumed for accepted {key}")
        _ordered_score, matrix_score, selected = min(
            scored,
            key=lambda item: (
                item[0],
                abs(item[2].message_index - last_position[event.direction]),
                item[2].message_index,
                item[2].belief_ordinal,
            ),
        )
        used.add((selected.message_index, selected.belief_ordinal))
        last_position[event.direction] = max(last_position[event.direction], selected.message_index)
        matches.append((event, selected, matrix_score))
    return matches


def matrix_stats(matrix: np.ndarray) -> dict[str, float | bool]:
    symmetric = 0.5 * (matrix + matrix.T)
    eigenvalues = np.linalg.eigvalsh(symmetric)
    minimum = float(np.min(eigenvalues))
    maximum = float(np.max(eigenvalues))
    return {
        "trace": float(np.trace(matrix)),
        "rot_trace": float(np.trace(matrix[:3, :3])),
        "trans_trace": float(np.trace(matrix[3:, 3:])),
        "min_eigenvalue": minimum,
        "max_eigenvalue": maximum,
        "condition_number": maximum / minimum if minimum > 0.0 else math.inf,
        "asymmetry_frobenius": float(np.linalg.norm(matrix - matrix.T)),
        "spd": bool(minimum > 0.0 and np.all(np.isfinite(matrix))),
    }


def percentile(values: np.ndarray, q: float) -> float:
    return float(np.percentile(values, q))


def matrix_markdown(matrix: np.ndarray) -> list[str]:
    lines = ["| | " + " | ".join(ORDER) + " |", "|---|" + "---:|" * 6]
    for label, row in zip(ORDER, matrix):
        lines.append("| " + label + " | " + " | ".join(f"{value:.8e}" for value in row) + " |")
    return lines


def write_matrix_csv(path: Path, matrix: np.ndarray) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("component", *ORDER))
        for label, row in zip(ORDER, matrix):
            writer.writerow((label, *(f"{value:.17g}" for value in row)))


def selected_sample_indices(count: int, requested: int) -> list[int]:
    if count <= requested:
        return list(range(count))
    return sorted(set(int(round(value)) for value in np.linspace(0, count - 1, requested)))


def write_outputs(
    output_dir: Path,
    matches: list[tuple[FactorEvent, BagCandidate, float]],
    args: argparse.Namespace,
) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    matrix_columns = [f"{prefix}_{row}_{column}" for prefix in ("sent", "inserted") for row in ORDER for column in ORDER]
    fieldnames = [
        "factor_index",
        "direction",
        "sender_from_index",
        "sender_to_index",
        "receiver_from_index",
        "receiver_to_index",
        "sender_from_stamp_sec",
        "sender_to_stamp_sec",
        "bag_stamp_sec",
        "bag_message_index",
        "topic_message_index",
        "belief_ordinal",
        "marker_log_line",
        "covariance_log_line",
        "action",
        "logged_trace",
        "inserted_trace",
        "trace_relative_error",
        "diagonal_relative_error",
        "candidate_match_score",
        "rotation_trace_rad2",
        "translation_trace_m2",
        "min_eigenvalue",
        "max_eigenvalue",
        "condition_number",
        "asymmetry_frobenius",
        "spd",
    ] + matrix_columns

    rows = []
    sent_matrices = []
    inserted_matrices = []
    for factor_index, (event, candidate, match_score) in enumerate(matches):
        inserted = candidate.inserted_covariance
        stats = matrix_stats(inserted)
        trace_error = abs(stats["trace"] - event.log_trace) / max(abs(event.log_trace), 1.0e-18)
        diag_error = normalized_error(np.diag(inserted), event.log_diag)
        row: dict[str, Any] = {
            "factor_index": factor_index,
            "direction": event.direction,
            "sender_from_index": event.sender_from,
            "sender_to_index": event.sender_to,
            "receiver_from_index": event.receiver_from,
            "receiver_to_index": event.receiver_to,
            "sender_from_stamp_sec": f"{candidate.from_stamp_sec:.9f}",
            "sender_to_stamp_sec": f"{candidate.to_stamp_sec:.9f}",
            "bag_stamp_sec": f"{candidate.bag_stamp_sec:.9f}",
            "bag_message_index": candidate.message_index,
            "topic_message_index": candidate.topic_message_index,
            "belief_ordinal": candidate.belief_ordinal,
            "marker_log_line": event.marker_line,
            "covariance_log_line": event.covariance_line,
            "action": event.action,
            "logged_trace": f"{event.log_trace:.17g}",
            "inserted_trace": f"{stats['trace']:.17g}",
            "trace_relative_error": f"{trace_error:.17g}",
            "diagonal_relative_error": f"{diag_error:.17g}",
            "candidate_match_score": f"{match_score:.17g}",
            "rotation_trace_rad2": f"{stats['rot_trace']:.17g}",
            "translation_trace_m2": f"{stats['trans_trace']:.17g}",
            "min_eigenvalue": f"{stats['min_eigenvalue']:.17g}",
            "max_eigenvalue": f"{stats['max_eigenvalue']:.17g}",
            "condition_number": f"{stats['condition_number']:.17g}",
            "asymmetry_frobenius": f"{stats['asymmetry_frobenius']:.17g}",
            "spd": int(stats["spd"]),
        }
        for prefix, matrix in (("sent", candidate.sent_covariance), ("inserted", inserted)):
            for row_index, row_label in enumerate(ORDER):
                for column_index, column_label in enumerate(ORDER):
                    row[f"{prefix}_{row_label}_{column_label}"] = f"{matrix[row_index, column_index]:.17g}"
        rows.append(row)
        sent_matrices.append(candidate.sent_covariance)
        inserted_matrices.append(inserted)

    with (output_dir / "accepted_factor_covariances.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)

    sent_array = np.stack(sent_matrices)
    inserted_array = np.stack(inserted_matrices)
    np.savez_compressed(
        output_dir / "accepted_factor_covariances.npz",
        order=np.asarray(ORDER),
        direction=np.asarray([event.direction for event, _candidate, _score in matches]),
        sender_from=np.asarray([event.sender_from for event, _candidate, _score in matches]),
        sender_to=np.asarray([event.sender_to for event, _candidate, _score in matches]),
        receiver_from=np.asarray([event.receiver_from for event, _candidate, _score in matches]),
        receiver_to=np.asarray([event.receiver_to for event, _candidate, _score in matches]),
        sent_covariance=sent_array,
        inserted_covariance=inserted_array,
    )

    summary: dict[str, Any] = {
        "status": "COMPLETE",
        "matrix_order": list(ORDER),
        "matrix_units": {
            "rotation_rotation": "rad^2",
            "translation_translation": "m^2",
            "cross_blocks": "rad*m",
        },
        "source": {
            "belief_bag": str(args.belief_bag.resolve()),
            "estimator_log": str(args.estimator_log.resolve()),
        },
        "reconstruction": {
            "g2k_topic": args.g2k_topic,
            "k2g_topic": args.k2g_topic,
            "g2k_base_T_external_translation": args.g2k_translation,
            "g2k_base_T_external_quaternion_xyzw": args.g2k_quaternion_xyzw,
            "g2k_covariance_scale": args.g2k_scale,
            "k2g_covariance_scale": args.k2g_scale,
            "health_aware_covariance_inflation": "disabled",
        },
        "directions": {},
    }
    stats_rows = []
    samples_lines = [
        "# Accepted CBS Factor Covariance Samples",
        "",
        "Matrices are the covariances actually supplied to each receiver's Gaussian between factor.",
        "The tangent order is `[rot_x, rot_y, rot_z, trans_x, trans_y, trans_z]`.",
        "",
    ]
    for direction in ("G2K", "K2G"):
        direction_indices = [index for index, row in enumerate(rows) if row["direction"] == direction]
        matrices = inserted_array[direction_indices]
        if len(matrices) == 0:
            raise ValueError(f"no accepted {direction} factors")
        traces = np.trace(matrices, axis1=1, axis2=2)
        rot_traces = np.trace(matrices[:, :3, :3], axis1=1, axis2=2)
        trans_traces = np.trace(matrices[:, 3:, 3:], axis1=1, axis2=2)
        min_eigenvalues = np.asarray([matrix_stats(matrix)["min_eigenvalue"] for matrix in matrices])
        conditions = np.asarray([matrix_stats(matrix)["condition_number"] for matrix in matrices])
        trace_errors = np.asarray([float(rows[index]["trace_relative_error"]) for index in direction_indices])
        diag_errors = np.asarray([float(rows[index]["diagonal_relative_error"]) for index in direction_indices])
        median_matrix = np.median(matrices, axis=0)
        p05_matrix = np.percentile(matrices, 5.0, axis=0)
        p95_matrix = np.percentile(matrices, 95.0, axis=0)
        for label, matrix in (("median", median_matrix), ("p05", p05_matrix), ("p95", p95_matrix)):
            write_matrix_csv(output_dir / f"{direction.lower()}_{label}_inserted_covariance.csv", matrix)
        direction_summary = {
            "accepted_factor_count": len(direction_indices),
            "spd_factor_count": int(np.count_nonzero(min_eigenvalues > 0.0)),
            "trace": {"p05": percentile(traces, 5), "median": percentile(traces, 50), "p95": percentile(traces, 95)},
            "rotation_trace_rad2": {"p05": percentile(rot_traces, 5), "median": percentile(rot_traces, 50), "p95": percentile(rot_traces, 95)},
            "translation_trace_m2": {"p05": percentile(trans_traces, 5), "median": percentile(trans_traces, 50), "p95": percentile(trans_traces, 95)},
            "minimum_eigenvalue": float(np.min(min_eigenvalues)),
            "condition_number": {"median": percentile(conditions, 50), "p95": percentile(conditions, 95), "maximum": float(np.max(conditions))},
            "validation": {
                "maximum_trace_relative_error": float(np.max(trace_errors)),
                "maximum_diagonal_relative_error": float(np.max(diag_errors)),
            },
        }
        summary["directions"][direction] = direction_summary
        stats_rows.append({"direction": direction, **direction_summary})

        samples_lines.extend((f"## {direction}", ""))
        for local_index in selected_sample_indices(len(direction_indices), args.sample_count):
            global_index = direction_indices[local_index]
            event, candidate, _score = matches[global_index]
            samples_lines.extend(
                (
                    f"### Factor {global_index}: sender {event.sender_from}→{event.sender_to}, receiver {event.receiver_from}→{event.receiver_to}",
                    "",
                    f"Bag time `{candidate.bag_stamp_sec:.9f}`, sender interval `{candidate.from_stamp_sec:.9f}`–`{candidate.to_stamp_sec:.9f}`.",
                    "",
                    *matrix_markdown(candidate.inserted_covariance),
                    "",
                )
            )

    maximum_error = max(
        max(value["validation"]["maximum_trace_relative_error"], value["validation"]["maximum_diagonal_relative_error"])
        for value in summary["directions"].values()
    )
    all_spd = all(
        value["accepted_factor_count"] == value["spd_factor_count"]
        for value in summary["directions"].values()
    )
    expected_ok = (
        (args.expected_g2k is None or summary["directions"]["G2K"]["accepted_factor_count"] == args.expected_g2k)
        and (args.expected_k2g is None or summary["directions"]["K2G"]["accepted_factor_count"] == args.expected_k2g)
    )
    summary["validation"] = {
        "maximum_logged_matrix_relative_error": maximum_error,
        "allowed_relative_error": args.max_relative_log_error,
        "all_covariances_spd": all_spd,
        "expected_counts_match": expected_ok,
    }
    if maximum_error > args.max_relative_log_error or not all_spd or not expected_ok:
        summary["status"] = "INCOMPLETE"
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (output_dir / "full_matrix_samples.md").write_text(
        "\n".join(samples_lines), encoding="utf-8"
    )

    report = [
        "# CBS Inserted-Factor Covariance Report",
        "",
        f"**Status: {summary['status']}**",
        "",
        "This report covers every accepted G→K and K→G odometry factor, not merely every ROS array message. Rolling belief windows contain repeated entries; receiver acceptance rows were matched to the corresponding full matrix by sender edge and the independently logged receiver trace/diagonal.",
        "",
        "## Contract",
        "",
        "- Matrix order: `[rot_x, rot_y, rot_z, trans_x, trans_y, trans_z]`.",
        "- G→K: the bag stores GLIM-frame Schur covariance; the reported inserted matrix applies `Ad(base_T_external) Σ Ad(base_T_external)ᵀ` and the configured receiver scale.",
        "- K→G: Kimera publishes the Schur covariance already transformed into GLIM's external receiver frame; the configured GLIM scale is then applied.",
        "- Health-aware covariance inflation was disabled for this run. A robust wrapper, if enabled, changes the loss but not this Gaussian covariance.",
        "",
        "## Direction summary",
        "",
        "| Direction | accepted factors | SPD | trace p05 | trace median | trace p95 | translation trace median [m²] | max log cross-check error |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for direction in ("G2K", "K2G"):
        value = summary["directions"][direction]
        report.append(
            f"| {direction} | {value['accepted_factor_count']} | {value['spd_factor_count']} | "
            f"{value['trace']['p05']:.8e} | {value['trace']['median']:.8e} | {value['trace']['p95']:.8e} | "
            f"{value['translation_trace_m2']['median']:.8e} | "
            f"{max(value['validation']['maximum_trace_relative_error'], value['validation']['maximum_diagonal_relative_error']):.3e} |"
        )
    report.extend(
        [
            "",
            "## Artifacts",
            "",
            "- `accepted_factor_covariances.csv`: every accepted factor, both sent and inserted 6×6 matrices, identities, timestamps, spectra, and validation errors.",
            "- `accepted_factor_covariances.npz`: lossless numerical arrays for later paper analysis.",
            "- `full_matrix_samples.md`: evenly spaced full-matrix examples in both directions.",
            "- `g2k_*_inserted_covariance.csv` and `k2g_*_inserted_covariance.csv`: component-wise p05, median, and p95 tables.",
            "- `summary.json`: machine-readable validation and distribution summary.",
            "",
            f"Maximum receiver-log cross-check error: `{maximum_error:.6e}` (limit `{args.max_relative_log_error:.6e}`).",
            "",
        ]
    )
    (output_dir / "REPORT.md").write_text("\n".join(report), encoding="utf-8")
    return summary


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--belief-bag", type=Path, required=True)
    parser.add_argument("--estimator-log", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--g2k-topic", default="/kimera/cbs/odom_belief_in")
    parser.add_argument("--k2g-topic", default="/kimera/cbs/odom_belief_out")
    parser.add_argument("--g2k-translation", type=float, nargs=3, default=(-0.051, -0.021, -0.041))
    parser.add_argument("--g2k-quaternion-xyzw", type=float, nargs=4, default=(1.0, 0.0, 0.0, 0.0))
    parser.add_argument("--g2k-scale", type=float, default=1.0)
    parser.add_argument("--k2g-scale", type=float, default=1.0)
    parser.add_argument("--expected-g2k", type=int)
    parser.add_argument("--expected-k2g", type=int)
    parser.add_argument("--sample-count", type=int, default=10)
    parser.add_argument("--max-relative-log-error", type=float, default=5.0e-5)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.g2k_scale <= 0.0 or args.k2g_scale <= 0.0:
        raise SystemExit("covariance scales must be positive")
    if args.sample_count <= 0:
        raise SystemExit("--sample-count must be positive")
    events, _covariance_rows = parse_log(args.estimator_log)
    adjoint = pose3_adjoint(args.g2k_translation, args.g2k_quaternion_xyzw)
    candidates = read_bag_candidates(
        args.belief_bag,
        args.g2k_topic,
        args.k2g_topic,
        adjoint,
        args.g2k_scale,
        args.k2g_scale,
    )
    matches = match_events(events, candidates)
    summary = write_outputs(args.output_dir, matches, args)
    print(args.output_dir.resolve())
    print(
        f"G2K={summary['directions']['G2K']['accepted_factor_count']} "
        f"K2G={summary['directions']['K2G']['accepted_factor_count']} "
        f"status={summary['status']}"
    )
    if summary["status"] != "COMPLETE":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
