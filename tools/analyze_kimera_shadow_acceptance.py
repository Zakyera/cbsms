#!/usr/bin/env python3

import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path


ROW_MARKER = "KIMERA_SHADOW_ACCEPTANCE_ROW,"
FACTOR_MARKER = "KIMERA_SHADOW_ACCEPTANCE_FACTOR_ROW,"
POSE_PATH_MARKER = "KIMERA_BACKEND_POSE_PATH_ROW,"


def marker_fields(line, marker):
    index = line.find(marker)
    if index < 0:
        return None
    return line[index + len(marker) :].strip().split(",")


def percentile(values, fraction):
    if not values:
        return math.nan
    ordered = sorted(values)
    index = round(fraction * (len(ordered) - 1))
    return ordered[index]


def parse_bool(value):
    return value.lower() in {"1", "true"}


def parse_row(fields):
    if len(fields) < 30:
        return None
    names = [
        "timestamp",
        "latest_frame",
        "status",
        "normal_valid",
        "normal_objective_before",
        "normal_objective_after",
        "normal_objective_ratio",
        "normal_pose_count",
        "normal_translation_mean_m",
        "normal_translation_max_m",
        "normal_rotation_mean_deg",
        "normal_rotation_max_deg",
        "normal_variables_relinearized",
        "normal_variables_reeliminated",
        "shadow_valid",
        "shadow_objective_before",
        "shadow_objective_after",
        "shadow_objective_ratio",
        "shadow_pose_count",
        "shadow_translation_mean_m",
        "shadow_translation_max_m",
        "shadow_rotation_mean_deg",
        "shadow_rotation_max_deg",
        "shadow_variables_relinearized",
        "shadow_variables_reeliminated",
        "normal_vs_shadow_pose_count",
        "normal_vs_shadow_translation_mean_m",
        "normal_vs_shadow_translation_max_m",
        "normal_vs_shadow_rotation_mean_deg",
        "normal_vs_shadow_rotation_max_deg",
    ]
    row = dict(zip(names, fields[: len(names)]))
    row["timestamp"] = float(row["timestamp"])
    row["latest_frame"] = int(row["latest_frame"])
    row["normal_valid"] = parse_bool(row["normal_valid"])
    row["shadow_valid"] = parse_bool(row["shadow_valid"])
    integer_names = [
        "normal_pose_count",
        "normal_variables_relinearized",
        "normal_variables_reeliminated",
        "shadow_pose_count",
        "shadow_variables_relinearized",
        "shadow_variables_reeliminated",
        "normal_vs_shadow_pose_count",
    ]
    for name in integer_names:
        row[name] = int(row[name])
    for name in names:
        if name in {
            "latest_frame",
            "status",
            "normal_valid",
            "shadow_valid",
            *integer_names,
        }:
            continue
        row[name] = float(row[name])
    row["shadow_after_over_normal_after"] = (
        row["shadow_objective_after"] / row["normal_objective_after"]
        if row["normal_objective_after"] > 0.0
        else math.nan
    )
    return row


def parse_factor_row(fields):
    if len(fields) < 9:
        return None
    return {
        "timestamp": float(fields[0]),
        "latest_frame": int(fields[1]),
        "candidate": fields[2],
        "category": fields[3],
        "count": int(fields[4]),
        "before_valid": int(fields[5]),
        "after_valid": int(fields[6]),
        "before_error": float(fields[7]),
        "after_error": float(fields[8]),
    }


def parse_pose_path_rows(path):
    rows = {}
    with path.open("r", errors="replace") as stream:
        for line in stream:
            fields = marker_fields(line, POSE_PATH_MARKER)
            if not fields or len(fields) < 2:
                continue
            rows[int(fields[1])] = ",".join(fields)
    return rows


def write_event(stream, title, row, factors):
    stream.write(f"## {title}\n\n")
    stream.write(
        f"- Frame: {row['latest_frame']}\n"
        f"- Normal objective before / after / ratio: "
        f"{row['normal_objective_before']:.9g} / "
        f"{row['normal_objective_after']:.9g} / "
        f"{row['normal_objective_ratio']:.9g}\n"
        f"- Shadow objective before / after / ratio: "
        f"{row['shadow_objective_before']:.9g} / "
        f"{row['shadow_objective_after']:.9g} / "
        f"{row['shadow_objective_ratio']:.9g}\n"
        f"- Shadow-after / normal-after: "
        f"{row['shadow_after_over_normal_after']:.9g}\n"
        f"- Normal relinearized / reeliminated: "
        f"{row['normal_variables_relinearized']} / "
        f"{row['normal_variables_reeliminated']}\n"
        f"- Shadow relinearized / reeliminated: "
        f"{row['shadow_variables_relinearized']} / "
        f"{row['shadow_variables_reeliminated']}\n"
        f"- Normal mean / max pose movement: "
        f"{row['normal_translation_mean_m']:.6f} / "
        f"{row['normal_translation_max_m']:.6f} m\n"
        f"- Normal-vs-shadow mean / max pose difference: "
        f"{row['normal_vs_shadow_translation_mean_m']:.6f} / "
        f"{row['normal_vs_shadow_translation_max_m']:.6f} m\n\n"
    )

    event_factors = factors.get(row["latest_frame"], {})
    categories = sorted(
        {
            category
            for candidate in event_factors.values()
            for category in candidate
        }
    )
    if not categories:
        return
    stream.write(
        "| category | before | normal after | shadow after | "
        "normal delta | shadow delta |\n"
    )
    stream.write("|---|---:|---:|---:|---:|---:|\n")
    for category in categories:
        normal = event_factors.get("normal", {}).get(category)
        shadow = event_factors.get("shadow_forced_full", {}).get(category)
        if not normal or not shadow:
            continue
        stream.write(
            f"| {category} | {normal['before_error']:.9g} | "
            f"{normal['after_error']:.9g} | "
            f"{shadow['after_error']:.9g} | "
            f"{normal['after_error'] - normal['before_error']:.9g} | "
            f"{shadow['after_error'] - shadow['before_error']:.9g} |\n"
        )
    stream.write("\n")


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Summarize discarded forced-full-solve candidates against "
            "Kimera's accepted incremental fixed-lag updates."
        )
    )
    parser.add_argument("log", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--focus-frame", type=int, default=113)
    parser.add_argument("--control-log", type=Path)
    args = parser.parse_args()

    rows = []
    factor_rows = []
    factors = defaultdict(lambda: defaultdict(dict))
    with args.log.open("r", errors="replace") as stream:
        for line in stream:
            fields = marker_fields(line, ROW_MARKER)
            if fields:
                row = parse_row(fields)
                if row:
                    rows.append(row)
                continue
            fields = marker_fields(line, FACTOR_MARKER)
            if fields:
                factor_row = parse_factor_row(fields)
                if factor_row:
                    factor_rows.append(factor_row)
                    factors[factor_row["latest_frame"]][
                        factor_row["candidate"]
                    ][factor_row["category"]] = factor_row

    if not rows:
        raise SystemExit("No Kimera shadow-acceptance rows found")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    row_csv = args.output_dir / "kimera_shadow_acceptance.csv"
    with row_csv.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    factor_csv = args.output_dir / "kimera_shadow_acceptance_factors.csv"
    with factor_csv.open("w", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "timestamp",
                "latest_frame",
                "candidate",
                "category",
                "count",
                "before_valid",
                "after_valid",
                "before_error",
                "after_error",
            ],
        )
        writer.writeheader()
        writer.writerows(factor_rows)

    valid = [row for row in rows if row["normal_valid"] and row["shadow_valid"]]
    if not valid:
        raise SystemExit("No rows with valid normal and shadow objectives")
    normal_increases = [
        row for row in valid if row["normal_objective_ratio"] > 1.0
    ]
    shadow_increases = [
        row for row in valid if row["shadow_objective_ratio"] > 1.0
    ]
    shadow_better = [
        row
        for row in valid
        if row["shadow_objective_after"] < row["normal_objective_after"]
    ]
    finite_normal = [
        row for row in valid if math.isfinite(row["normal_objective_ratio"])
    ]
    finite_shadow = [
        row for row in valid if math.isfinite(row["shadow_objective_ratio"])
    ]
    normal_ratios = [row["normal_objective_ratio"] for row in finite_normal]
    shadow_ratios = [row["shadow_objective_ratio"] for row in finite_shadow]
    after_ratios = [
        row["shadow_after_over_normal_after"]
        for row in valid
        if math.isfinite(row["shadow_after_over_normal_after"])
    ]

    focus = min(
        valid,
        key=lambda row: abs(row["latest_frame"] - args.focus_frame),
    )
    worst_normal_ratio = max(
        finite_normal, key=lambda row: row["normal_objective_ratio"]
    )
    largest_candidate_difference = max(
        valid,
        key=lambda row: row["normal_vs_shadow_translation_mean_m"],
    )

    report = args.output_dir / "report.md"
    parity_summary = None
    if args.control_log:
        control_pose_rows = parse_pose_path_rows(args.control_log)
        audit_pose_rows = parse_pose_path_rows(args.log)
        common_frames = sorted(set(control_pose_rows) & set(audit_pose_rows))
        mismatches = [
            frame
            for frame in common_frames
            if control_pose_rows[frame] != audit_pose_rows[frame]
        ]
        parity_summary = {
            "control_count": len(control_pose_rows),
            "audit_count": len(audit_pose_rows),
            "common_count": len(common_frames),
            "mismatches": mismatches,
        }
    with report.open("w") as stream:
        stream.write("# Kimera Shadow Acceptance Audit\n\n")
        stream.write(
            f"- Parsed updates: {len(rows)}\n"
            f"- Valid normal/shadow comparisons: {len(valid)}\n"
            f"- Frame range: {min(row['latest_frame'] for row in rows)} to "
            f"{max(row['latest_frame'] for row in rows)}\n"
            f"- Accepted normal updates increasing audited nonlinear objective: "
            f"{len(normal_increases)} / {len(valid)}\n"
            f"- Forced-full shadow updates increasing audited nonlinear objective: "
            f"{len(shadow_increases)} / {len(valid)}\n"
            f"- Shadow ending below accepted normal objective: "
            f"{len(shadow_better)} / {len(valid)}\n"
            f"- Normal objective ratio p50 / p95 / max: "
            f"{statistics.median(normal_ratios):.6g} / "
            f"{percentile(normal_ratios, 0.95):.6g} / "
            f"{max(normal_ratios):.6g}\n"
            f"- Shadow objective ratio p50 / p95 / max: "
            f"{statistics.median(shadow_ratios):.6g} / "
            f"{percentile(shadow_ratios, 0.95):.6g} / "
            f"{max(shadow_ratios):.6g}\n"
            f"- Shadow-after / normal-after p50 / p95: "
            f"{statistics.median(after_ratios):.6g} / "
            f"{percentile(after_ratios, 0.95):.6g}\n"
            f"- Normal relinearized p50 / max: "
            f"{statistics.median(row['normal_variables_relinearized'] for row in valid):.3f} / "
            f"{max(row['normal_variables_relinearized'] for row in valid)}\n"
            f"- Shadow relinearized p50 / max: "
            f"{statistics.median(row['shadow_variables_relinearized'] for row in valid):.3f} / "
            f"{max(row['shadow_variables_relinearized'] for row in valid)}\n\n"
        )
        stream.write(
            "The audited objective is the sum of directly evaluable nonlinear "
            "factors. `LinearContainerFactor` marginal priors require "
            "delta-coordinate evaluation and therefore appear as zero in the "
            "factor-category tables.\n\n"
        )
        if parity_summary:
            stream.write(
                "- Production pose-path parity against control: "
                f"{parity_summary['common_count']} common frames, "
                f"{len(parity_summary['mismatches'])} mismatches "
                f"(control {parity_summary['control_count']} rows, "
                f"audit {parity_summary['audit_count']} rows)\n\n"
            )

        write_event(
            stream,
            f"Focus Event (requested frame {args.focus_frame})",
            focus,
            factors,
        )
        write_event(
            stream,
            "Worst Accepted Normal Objective Ratio",
            worst_normal_ratio,
            factors,
        )
        write_event(
            stream,
            "Largest Normal-vs-Shadow Pose Difference",
            largest_candidate_difference,
            factors,
        )

        stream.write("## Largest Accepted Objective Ratios\n\n")
        stream.write(
            "| frame | normal ratio | shadow ratio | shadow/normal after | "
            "normal mean move m | normal-vs-shadow mean m |\n"
        )
        stream.write("|---:|---:|---:|---:|---:|---:|\n")
        for row in sorted(
            finite_normal,
            key=lambda item: item["normal_objective_ratio"],
            reverse=True,
        )[:15]:
            stream.write(
                f"| {row['latest_frame']} | "
                f"{row['normal_objective_ratio']:.6g} | "
                f"{row['shadow_objective_ratio']:.6g} | "
                f"{row['shadow_after_over_normal_after']:.6g} | "
                f"{row['normal_translation_mean_m']:.6f} | "
                f"{row['normal_vs_shadow_translation_mean_m']:.6f} |\n"
            )

    print(row_csv)
    print(factor_csv)
    print(report)


if __name__ == "__main__":
    main()
