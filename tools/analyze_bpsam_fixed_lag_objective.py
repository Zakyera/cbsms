#!/usr/bin/env python3

import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path


STAGE_MARKER = "CBS_FIXED_LAG_STAGE_ROW,"
FACTOR_MARKER = "CBS_FIXED_LAG_FACTOR_ERROR_ROW,"
PRIOR_MARKER = "CBS_FIXED_LAG_PRIOR_UPDATE_ROW,"


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


def parse_frame(value):
    return int(round(float(value)))


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Reconstruct pre/post BPSAM fixed-lag objectives from diagnostic "
            "rows, including the Gaussian marginal prior in delta coordinates."
        )
    )
    parser.add_argument("log", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--smoother-id", default="k")
    args = parser.parse_args()

    stages = {}
    factors = defaultdict(lambda: defaultdict(dict))
    priors = defaultdict(lambda: {"pre": 0.0, "post": 0.0, "count": 0})

    with args.log.open("r", errors="replace") as stream:
        for line in stream:
            fields = marker_fields(line, STAGE_MARKER)
            if fields and len(fields) >= 22 and fields[0] == args.smoother_id:
                if fields[2] != "isam_update":
                    continue
                frame = parse_frame(fields[1])
                stages[frame] = {
                    "frame": frame,
                    "pose_count": int(fields[3]),
                    "mean_movement_m": float(fields[4]),
                    "max_movement_m": float(fields[5]),
                    "mean_rotation_deg": float(fields[6]),
                    "max_rotation_deg": float(fields[7]),
                    "first_key": fields[8],
                    "first_movement_m": float(fields[9]),
                    "last_key": fields[10],
                    "last_movement_m": float(fields[11]),
                    "common_world_tx_m": float(fields[12]),
                    "common_world_ty_m": float(fields[13]),
                    "common_world_tz_m": float(fields[14]),
                    "common_world_rx_deg": float(fields[15]),
                    "common_world_ry_deg": float(fields[16]),
                    "common_world_rz_deg": float(fields[17]),
                    "rigid_residual_translation_mean_m": float(fields[18]),
                    "rigid_residual_translation_max_m": float(fields[19]),
                    "rigid_residual_rotation_mean_deg": float(fields[20]),
                    "rigid_residual_rotation_max_deg": float(fields[21]),
                }
                continue

            fields = marker_fields(line, FACTOR_MARKER)
            if fields and len(fields) >= 9 and fields[0] == args.smoother_id:
                frame = parse_frame(fields[1])
                source = fields[2]
                category = fields[3]
                factors[frame][source][category] = {
                    "count": int(fields[4]),
                    "pre_valid": int(fields[5]),
                    "post_valid": int(fields[6]),
                    "pre": float(fields[7]),
                    "post": float(fields[8]),
                }
                continue

            fields = marker_fields(line, PRIOR_MARKER)
            if fields and len(fields) >= 13 and fields[0] == args.smoother_id:
                frame = parse_frame(fields[1])
                priors[frame]["pre"] += float(fields[11])
                priors[frame]["post"] += float(fields[12])
                priors[frame]["count"] += 1

    frames = sorted(set(stages) & set(factors))
    if not frames:
        raise SystemExit("No matching fixed-lag stage/factor rows found")

    categories = sorted(
        {
            category
            for frame in frames
            for source in factors[frame].values()
            for category in source
        }
        | {"marginal_prior"}
    )
    rows = []
    for frame in frames:
        existing = factors[frame].get("existing", {})
        new = factors[frame].get("new", {})
        row = dict(stages[frame])
        pre_total = 0.0
        post_total = 0.0
        for category in categories:
            existing_values = existing.get(
                category,
                {
                    "count": 0,
                    "pre_valid": 0,
                    "post_valid": 0,
                    "pre": 0.0,
                    "post": 0.0,
                },
            )
            new_values = new.get(category, {"count": 0, "pre": 0.0})
            if category == "marginal_prior":
                pre_error = priors[frame]["pre"]
                post_error = priors[frame]["post"]
            else:
                pre_error = existing_values["pre"] + new_values["pre"]
                post_error = existing_values["post"]
            row[f"{category}_preopt"] = pre_error
            row[f"{category}_accepted"] = post_error
            row[f"{category}_delta"] = post_error - pre_error
            row[f"{category}_count"] = existing_values["count"]
            pre_total += pre_error
            post_total += post_error

        row["preopt_error"] = pre_total
        row["accepted_error"] = post_total
        row["error_delta"] = post_total - pre_total
        row["error_ratio"] = (
            post_total / pre_total if pre_total > 0.0 else math.nan
        )
        row["marginal_prior_count"] = priors[frame]["count"]
        rows.append(row)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.output_dir / "bpsam_fixed_lag_objective_audit.csv"
    fieldnames = list(rows[0])
    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    finite_ratio_rows = [
        row for row in rows if math.isfinite(row["error_ratio"])
    ]
    ratios = [row["error_ratio"] for row in finite_ratio_rows]
    increases = [row for row in rows if row["error_delta"] > 0.0]
    severe_motion = [
        row for row in rows if row["mean_movement_m"] > 0.5
    ]
    severe_increase = [
        row for row in severe_motion if row["error_delta"] > 0.0
    ]
    worst_ratio = max(finite_ratio_rows, key=lambda row: row["error_ratio"])
    worst_delta = max(rows, key=lambda row: row["error_delta"])
    worst_motion = max(rows, key=lambda row: row["mean_movement_m"])
    positive_increase_by_category = {
        category: sum(
            max(0.0, row[f"{category}_delta"]) for row in increases
        )
        for category in categories
    }

    report_path = args.output_dir / "report.md"
    with report_path.open("w") as stream:
        stream.write("# Production BPSAM Fixed-Lag Objective Audit\n\n")
        stream.write(f"- Smoother ID: `{args.smoother_id}`\n")
        stream.write(f"- Updates: {len(rows)}\n")
        stream.write(
            f"- Accepted updates increasing exact error: {len(increases)}\n"
        )
        stream.write(
            "- Error ratio p50 / p95 / max: "
            f"{statistics.median(ratios):.6f} / "
            f"{percentile(ratios, 0.95):.6f} / "
            f"{max(ratios):.6f}\n"
        )
        stream.write(
            "- Updates with mean active-window movement > 0.5 m: "
            f"{len(severe_motion)}\n"
        )
        stream.write(
            "- Those severe-motion updates that increased exact error: "
            f"{len(severe_increase)}\n\n"
        )

        def write_event(title, row):
            stream.write(f"## {title}\n\n")
            stream.write(
                f"- Frame: {row['frame']}\n"
                f"- Poses: {row['pose_count']}\n"
                f"- Pre-opt / accepted error: "
                f"{row['preopt_error']:.9g} / "
                f"{row['accepted_error']:.9g}\n"
                f"- Delta / ratio: {row['error_delta']:.9g} / "
                f"{row['error_ratio']:.9g}\n"
                f"- Mean / max movement: "
                f"{row['mean_movement_m']:.6f} m / "
                f"{row['max_movement_m']:.6f} m\n"
                f"- First / last key movement: "
                f"{row['first_key']} {row['first_movement_m']:.6f} m / "
                f"{row['last_key']} {row['last_movement_m']:.6f} m\n"
                f"- Common world translation: "
                f"[{row['common_world_tx_m']:.6f}, "
                f"{row['common_world_ty_m']:.6f}, "
                f"{row['common_world_tz_m']:.6f}] m\n\n"
            )
            stream.write("| factor category | pre-opt | accepted | delta |\n")
            stream.write("|---|---:|---:|---:|\n")
            for category in categories:
                stream.write(
                    f"| {category} | "
                    f"{row[f'{category}_preopt']:.9g} | "
                    f"{row[f'{category}_accepted']:.9g} | "
                    f"{row[f'{category}_delta']:.9g} |\n"
                )
            stream.write("\n")

        write_event("Worst Error Ratio", worst_ratio)
        write_event("Largest Absolute Error Increase", worst_delta)
        write_event("Worst Window Movement", worst_motion)

        stream.write("## Positive Error Increase by Factor Category\n\n")
        for category, value in sorted(
            positive_increase_by_category.items(),
            key=lambda item: -item[1],
        ):
            stream.write(f"- {category}: {value:.9g}\n")

        stream.write("\n## Largest Accepted Window Moves\n\n")
        stream.write(
            "| frame | poses | mean move m | max move m | "
            "error delta | ratio |\n"
        )
        stream.write("|---:|---:|---:|---:|---:|---:|\n")
        for row in sorted(
            rows, key=lambda item: item["mean_movement_m"], reverse=True
        )[:15]:
            stream.write(
                f"| {row['frame']} | {row['pose_count']} | "
                f"{row['mean_movement_m']:.6f} | "
                f"{row['max_movement_m']:.6f} | "
                f"{row['error_delta']:.6g} | "
                f"{row['error_ratio']:.6f} |\n"
            )

    print(csv_path)
    print(report_path)


if __name__ == "__main__":
    main()
