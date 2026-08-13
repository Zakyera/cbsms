#!/usr/bin/env python3
"""Expand a frozen Stage 2C reference-screen result without changing it."""

import argparse
import csv
import hashlib
import json
import math
from collections import Counter, defaultdict
from pathlib import Path


def truth(value):
    return str(value).strip().lower() in ("1", "true", "yes")


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def maximum_modes(row, stem):
    values = [float(row[f"{stem}_{index}"]) for index in range(3)
              if finite(row.get(f"{stem}_{index}"))]
    return max(values) if values else math.nan


def any_modes(row, stem):
    return any(truth(row.get(f"{stem}_{index}")) for index in range(3))


def read_rows(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def require_one_run(paths):
    paths = [Path(path) for path in paths]
    if len(paths) != 1:
        raise ValueError("offline qualification accepts exactly one recording; "
                         "samples across runs may not be combined")
    return paths[0]


def audit(health_paths, frozen_screen_path):
    health_path = require_one_run(health_paths)
    rows = read_rows(health_path)
    screen = json.loads(Path(frozen_screen_path).read_text(encoding="utf-8"))
    aggregate = [row for row in rows if row.get("record_kind") == "aggregate"]
    factors = defaultdict(list)
    for row in rows:
        if row.get("record_kind") == "factor":
            factors[row.get("frame_id", "")].append(row)
    accepted = [row for row in aggregate if truth(row.get("baseline_update_accepted"))]
    accepted.sort(key=lambda row: (int(row["frame_id"]), float(row["timestamp"])))
    rejection_reasons = Counter(row.get("baseline_update_reason") or "missing_reason"
                                for row in aggregate
                                if not truth(row.get("baseline_update_accepted")))
    accepted_rows = []
    for row in accepted:
        factor_details = []
        for factor in sorted(factors[row["frame_id"]],
                             key=lambda item: float(item.get("resolution", "nan"))):
            factor_details.append({
                "resolution": float(factor["resolution"]),
                "source_point_count": int(float(factor["source_point_count"])),
                "inlier_count": int(float(factor["inlier_count"])),
                "inlier_fraction": float(factor["inlier_fraction"]),
                "initial_cost": float(factor["initial_cost"]),
                "final_cost": float(factor["final_cost"]),
                "registration_converged": truth(factor["registration_converged"]),
            })
        accepted_rows.append({
            "frame_id": int(row["frame_id"]),
            "timestamp": float(row["timestamp"]),
            "maximum_rotation_condition_ratio": maximum_modes(
                row, "rotation_condition_ratio"),
            "maximum_translation_condition_ratio": maximum_modes(
                row, "translation_condition_ratio"),
            "source_point_count": int(float(row["source_point_count"])),
            "minimum_factor_inlier_count": int(float(
                row["minimum_factor_inlier_count"])),
            "minimum_factor_inlier_fraction": float(
                row["minimum_factor_inlier_fraction"]),
            "baseline_update_reason": row["baseline_update_reason"],
            "per_resolution": factor_details,
        })
    timestamps = [row["timestamp"] for row in accepted_rows]
    gaps = [right - left for left, right in zip(timestamps, timestamps[1:])]
    combined_absolute = sum(
        any_modes(row, "absolute_rotation_mask") or
        any_modes(row, "absolute_translation_mask") for row in aggregate)
    adaptation_acceptances = sum(
        truth(row.get("baseline_update_accepted")) and
        row.get("baseline_update_reason") != "bootstrap_sample_accepted"
        for row in aggregate)
    if len(accepted_rows) != screen["accepted_reference_candidate_count"]:
        raise ValueError("accepted-row count disagrees with frozen screening result")
    return {
        "schema_version": 1,
        "scientific_scope": "passive frozen-gate reference screening only",
        "health_csv": str(health_path.resolve()),
        "health_csv_sha256": hashlib.sha256(health_path.read_bytes()).hexdigest(),
        "frozen_screen": str(Path(frozen_screen_path).resolve()),
        "online_reference_ready": screen["first_reference_ready_frame"] is not None,
        "offline_profile_qualified": bool(screen["qualified"]),
        "aggregate_sample_count": len(aggregate),
        "accepted_bootstrap_candidate_count": len(accepted_rows),
        "adaptation_acceptance_count": adaptation_acceptances,
        "accepted_samples": accepted_rows,
        "accepted_sample_maximum_gaps_sec": gaps,
        "maximum_accepted_gap_sec": max(gaps) if gaps else None,
        "accepted_interval_duration_sec": (
            timestamps[-1] - timestamps[0] if len(timestamps) >= 2 else 0.0),
        "rejection_reason_counts": dict(sorted(rejection_reasons.items())),
        "absolute_rotation_degenerate_count": screen["absolute_rotation_sample_count"],
        "absolute_translation_degenerate_count": screen["absolute_translation_sample_count"],
        "absolute_combined_degenerate_count": combined_absolute,
        "absolute_rotation_degenerate_fraction": (
            screen["absolute_rotation_sample_count"] / len(aggregate)),
        "absolute_translation_degenerate_fraction": (
            screen["absolute_translation_sample_count"] / len(aggregate)),
        "absolute_combined_degenerate_fraction": combined_absolute / len(aggregate),
        "qualification_checks": screen["qualification_checks"],
        "profile_export_authorized": bool(screen["qualified"]),
        "short_medium_hybrid_rerun_authorized": bool(screen["qualified"]),
        "ground_truth_used": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--health-csv", type=Path, action="append", required=True)
    parser.add_argument("--frozen-screen", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--accepted-csv", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.health_csv, args.frozen_screen)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    fields = ["frame_id", "timestamp", "maximum_rotation_condition_ratio",
              "maximum_translation_condition_ratio", "source_point_count",
              "minimum_factor_inlier_count", "minimum_factor_inlier_fraction",
              "baseline_update_reason"]
    with args.accepted_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in result["accepted_samples"]:
            writer.writerow({field: row[field] for field in fields})
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
