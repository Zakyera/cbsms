#!/usr/bin/env python3
"""Preregistered passive GEODE Stage 2C-C analysis.

The tool operates only on previously recorded Stage 1/2A and Stage 2C-B
artifacts.  It never writes estimator configuration, never consumes ground
truth in reference construction, and never fits a health-to-error mapping.
"""

import argparse
import csv
import hashlib
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


NAN = float("nan")


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def truth(value):
    return str(value).strip().lower() in ("1", "true", "yes")


def read_csv(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def json_hash(value):
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def numeric_summary(values):
    clean = np.asarray([float(value) for value in values if finite(value)], dtype=float)
    if not clean.size:
        return {"count": 0, "minimum": None, "p10": None, "median": None,
                "mean": None, "p75": None, "p90": None, "p95": None,
                "maximum": None}
    return {
        "count": int(clean.size),
        "minimum": float(np.min(clean)),
        "p10": float(np.percentile(clean, 10)),
        "median": float(np.median(clean)),
        "mean": float(np.mean(clean)),
        "p75": float(np.percentile(clean, 75)),
        "p90": float(np.percentile(clean, 90)),
        "p95": float(np.percentile(clean, 95)),
        "maximum": float(np.max(clean)),
    }


def median_absolute_deviation(values):
    clean = np.asarray([float(value) for value in values if finite(value)], dtype=float)
    if not clean.size:
        return NAN
    median = np.median(clean)
    return float(np.median(np.abs(clean - median)))


def average_ranks(values):
    values = np.asarray(values, dtype=float)
    order = np.argsort(values, kind="mergesort")
    ranks = np.empty(len(values), dtype=float)
    start = 0
    while start < len(values):
        end = start + 1
        while end < len(values) and values[order[end]] == values[order[start]]:
            end += 1
        ranks[order[start:end]] = 0.5 * (start + 1 + end)
        start = end
    return ranks


def spearman(x_values, y_values):
    pairs = [(float(x), float(y)) for x, y in zip(x_values, y_values)
             if finite(x) and finite(y)]
    if len(pairs) < 3:
        return None
    x, y = np.asarray(pairs, dtype=float).T
    x_rank = average_ranks(x)
    y_rank = average_ranks(y)
    if np.std(x_rank) == 0.0 or np.std(y_rank) == 0.0:
        return None
    return float(np.corrcoef(x_rank, y_rank)[0, 1])


def health_rows(path):
    rows = [row for row in read_csv(path) if row.get("record_kind") == "aggregate"]
    indexed = {}
    exact_duplicates = 0
    conflicting_duplicates = 0
    for row in rows:
        key = int(row["frame_id"])
        previous = indexed.get(key)
        if previous is None:
            indexed[key] = row
        elif previous == row:
            exact_duplicates += 1
        else:
            conflicting_duplicates += 1
            indexed.pop(key, None)
    return indexed, {
        "aggregate_input_count": len(rows),
        "unique_frame_count": len(indexed),
        "exact_duplicate_count": exact_duplicates,
        "conflicting_duplicate_count": conflicting_duplicates,
    }


def per_resolution_support(path):
    """Summarize the documented support fields on factor rows only.

    Aggregate Stage 1 rows intentionally describe the summed LiDAR Hessian;
    their point/inlier/cost placeholders are not a second support population.
    Keeping resolution rows separate also avoids adding the two VGICP levels
    as though they contained independent source points.
    """
    grouped = defaultdict(list)
    for row in read_csv(path):
        if row.get("record_kind") != "factor" or not finite(row.get("resolution")):
            continue
        grouped[float(row["resolution"])].append(row)
    summaries = {}
    for resolution, rows in sorted(grouped.items()):
        key = format(resolution, ".17g")
        summaries[key] = {
            "factor_sample_count": len(rows),
            "valid_count": sum(truth(row.get("valid")) for row in rows),
            "registration_converged_count": sum(
                truth(row.get("registration_converged")) for row in rows),
            "linear_solve_success_count": sum(
                truth(row.get("linear_solve_success")) for row in rows),
            "source_point_count": numeric_summary(
                row.get("source_point_count") for row in rows),
            "inlier_count": numeric_summary(row.get("inlier_count") for row in rows),
            "inlier_fraction": numeric_summary(
                row.get("inlier_fraction") for row in rows),
            "initial_cost": numeric_summary(row.get("initial_cost") for row in rows),
            "final_cost": numeric_summary(row.get("final_cost") for row in rows),
            "cost_reduction": numeric_summary(
                float(row["initial_cost"]) - float(row["final_cost"])
                for row in rows
                if finite(row.get("initial_cost")) and finite(row.get("final_cost"))),
        }
    return summaries


def mode_maximum(row, prefix):
    values = [row.get(f"{prefix}_condition_ratio_{index}") for index in range(3)]
    if not all(finite(value) and float(value) > 0.0 for value in values):
        return NAN
    return max(float(value) for value in values)


def absolute_flag(row, prefix):
    fields = [row.get(f"absolute_{prefix}_mask_{index}") for index in range(3)]
    if len(fields) != 3 or any(value in (None, "") for value in fields):
        return None
    return any(truth(value) for value in fields)


def longest_true_run(values):
    longest = 0
    current = 0
    for value in values:
        if value is True:
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return longest


def edge_scan_summary(edge, indexed_health, expected_pose_keys):
    start = int(edge["from_pose_index"])
    end = int(edge["to_pose_index"])
    keys = [key for key in expected_pose_keys if start < key <= end]
    rotation_ratios = []
    translation_ratios = []
    rotation_flags = []
    translation_flags = []
    valid_count = 0
    missing_count = 0
    for key in keys:
        row = indexed_health.get(key)
        if row is None:
            missing_count += 1
            rotation_flags.append(None)
            translation_flags.append(None)
            continue
        rotation_ratio = mode_maximum(row, "rotation")
        translation_ratio = mode_maximum(row, "translation")
        rotation_flag = absolute_flag(row, "rotation")
        translation_flag = absolute_flag(row, "translation")
        row_valid = truth(row.get("valid")) and truth(row.get("factorization_ok"))
        if not row_valid or not finite(rotation_ratio) or not finite(translation_ratio):
            rotation_flags.append(None)
            translation_flags.append(None)
            continue
        valid_count += 1
        rotation_ratios.append(rotation_ratio)
        translation_ratios.append(translation_ratio)
        rotation_flags.append(rotation_flag)
        translation_flags.append(translation_flag)
    rotation_denominator = sum(value is not None for value in rotation_flags)
    translation_denominator = sum(value is not None for value in translation_flags)
    rotation_count = sum(value is True for value in rotation_flags)
    translation_count = sum(value is True for value in translation_flags)
    return {
        "expected_health_sample_count": len(keys),
        "observed_valid_health_sample_count": valid_count,
        "missing_health_sample_count": missing_count,
        "stage2c_c_max_rotation_condition_ratio": (
            max(rotation_ratios) if rotation_ratios else NAN),
        "stage2c_c_max_translation_condition_ratio": (
            max(translation_ratios) if translation_ratios else NAN),
        "stage2c_c_log_max_rotation_condition_ratio": (
            math.log(max(rotation_ratios)) if rotation_ratios else NAN),
        "stage2c_c_log_max_translation_condition_ratio": (
            math.log(max(translation_ratios)) if translation_ratios else NAN),
        "stage2c_c_rotation_absolute_degenerate_count": rotation_count,
        "stage2c_c_rotation_absolute_degenerate_denominator": rotation_denominator,
        "stage2c_c_rotation_absolute_degenerate_fraction": (
            rotation_count / rotation_denominator if rotation_denominator else NAN),
        "stage2c_c_translation_absolute_degenerate_count": translation_count,
        "stage2c_c_translation_absolute_degenerate_denominator": translation_denominator,
        "stage2c_c_translation_absolute_degenerate_fraction": (
            translation_count / translation_denominator if translation_denominator else NAN),
        "stage2c_c_longest_rotation_absolute_degenerate_run": longest_true_run(rotation_flags),
        "stage2c_c_longest_translation_absolute_degenerate_run": longest_true_run(
            translation_flags),
    }


def conditioning_bin(value, boundaries, labels):
    if not finite(value):
        return "unavailable"
    value = float(value)
    for index, boundary in enumerate(boundaries):
        if value < boundary:
            return labels[index]
    return labels[-1]


def block_groups(rows, duration):
    clean = [row for row in rows if finite(row.get("to_stamp_sec"))]
    if not clean:
        return {}, None
    origin = min(float(row["to_stamp_sec"]) for row in clean)
    groups = defaultdict(list)
    for row in clean:
        block = int(math.floor((float(row["to_stamp_sec"]) - origin) / duration))
        groups[block].append(row)
    return dict(sorted(groups.items())), origin


def bootstrap_statistic(rows, predictor, response, plan, statistic="spearman"):
    config = plan["temporal_block_bootstrap"]
    groups, origin = block_groups(rows, float(config["block_duration_sec"]))
    keys = list(groups)
    if len(keys) < 2:
        return {"estimate": None, "lower": None, "upper": None,
                "valid_replicates": 0, "block_count": len(keys), "origin": origin}

    def calculate(sample):
        if statistic == "spearman":
            return spearman([row.get(predictor) for row in sample],
                            [row.get(response) for row in sample])
        if statistic == "median_binary_difference":
            present = [float(row[response]) for row in sample
                       if truth(row.get(predictor)) and finite(row.get(response))]
            absent = [float(row[response]) for row in sample
                      if not truth(row.get(predictor)) and finite(row.get(response))]
            if not present or not absent:
                return None
            return float(np.median(present) - np.median(absent))
        raise ValueError(f"unsupported statistic: {statistic}")

    estimate = calculate(rows)
    rng = np.random.default_rng(int(config["random_seed"]) + sum(map(ord, predictor)))
    estimates = []
    for _iteration in range(int(config["replicates"])):
        selected = rng.choice(keys, size=len(keys), replace=True)
        sample = [row for key in selected for row in groups[int(key)]]
        value = calculate(sample)
        if value is not None and finite(value):
            estimates.append(float(value))
    alpha = 1.0 - float(config["confidence_level"])
    return {
        "estimate": estimate,
        "lower": float(np.percentile(estimates, 100.0 * alpha / 2.0))
        if estimates else None,
        "upper": float(np.percentile(estimates, 100.0 * (1.0 - alpha / 2.0)))
        if estimates else None,
        "valid_replicates": len(estimates),
        "requested_replicates": int(config["replicates"]),
        "block_count": len(keys),
        "block_duration_sec": float(config["block_duration_sec"]),
        "origin": origin,
    }


def bin_error_summaries(rows, condition_field, error_field, plan):
    labels = plan["conditioning_bins"]["labels"]
    result = []
    for label in labels:
        selected = [float(row[error_field]) for row in rows
                    if row.get(f"{condition_field}_bin") == label
                    and finite(row.get(error_field))]
        summary = numeric_summary(selected)
        result.append({"bin": label, **summary,
                       "adequate": len(selected) >= int(plan["conditioning_bins"]
                                                        ["minimum_edges_per_interpretable_bin"])})
    return result


def dynamic_range(rows, field, bin_field, plan):
    values = [float(row[field]) for row in rows if finite(row.get(field))]
    summary = numeric_summary(values)
    labels = Counter(row.get(bin_field) for row in rows
                     if row.get(bin_field) not in (None, "unavailable"))
    minimum_count = int(plan["conditioning_bins"]["minimum_edges_per_interpretable_bin"])
    adequate_bins = sum(count >= minimum_count for count in labels.values())
    ratio = (summary["p90"] / summary["p10"]
             if summary["p10"] is not None and summary["p10"] > 0.0 else None)
    informative = (ratio is not None
                   and ratio >= float(plan["dynamic_range"]["minimum_p90_to_p10_ratio"])
                   and adequate_bins >= int(plan["conditioning_bins"]
                                            ["minimum_interpretable_bin_count"]))
    return {"summary": summary, "p90_to_p10_ratio": ratio,
            "bin_counts": dict(labels), "adequate_bin_count": adequate_bins,
            "informative": informative}


def stage2a_consistency(rows, tolerance=1.0e-12):
    checks = {
        "rotation_maximum": ("stage2c_c_max_rotation_condition_ratio",
                             "worst_rotational_condition_ratio"),
        "translation_maximum": ("stage2c_c_max_translation_condition_ratio",
                                "worst_translational_condition_ratio"),
        "rotation_fraction": ("stage2c_c_rotation_absolute_degenerate_fraction",
                              "rotational_absolute_degenerate_fraction"),
        "translation_fraction": ("stage2c_c_translation_absolute_degenerate_fraction",
                                 "translational_absolute_degenerate_fraction"),
    }
    result = {}
    for label, fields in checks.items():
        differences = []
        for row in rows:
            first, second = row.get(fields[0]), row.get(fields[1])
            if finite(first) and finite(second):
                differences.append(abs(float(first) - float(second)))
        result[label] = {
            "compared_count": len(differences),
            "maximum_absolute_difference": max(differences) if differences else None,
            "within_tolerance": bool(differences) and max(differences) <= tolerance,
            "tolerance": tolerance,
        }
    return result


def write_scatter_svg(path, rows, x_field, y_field, title, x_label, y_label):
    pairs = [(float(row[x_field]), float(row[y_field])) for row in rows
             if finite(row.get(x_field)) and finite(row.get(y_field))]
    width, height, margin = 1000, 650, 90
    if not pairs:
        Path(path).write_text("<svg xmlns='http://www.w3.org/2000/svg'/>", encoding="utf-8")
        return
    x = np.asarray([pair[0] for pair in pairs])
    y = np.asarray([pair[1] for pair in pairs])
    x0, x1 = float(np.min(x)), float(np.max(x))
    y0, y1 = 0.0, float(np.percentile(y, 99.5))
    x1 = max(x1, x0 + 1.0e-12)
    y1 = max(y1, 1.0e-12)
    svg = [f"<svg xmlns='http://www.w3.org/2000/svg' width='{width}' height='{height}'>",
           "<rect width='100%' height='100%' fill='white'/>",
           f"<text x='{margin}' y='35' font-size='22'>{title}</text>",
           f"<text x='{width / 2}' y='{height - 20}' text-anchor='middle'>{x_label}</text>",
           f"<text x='20' y='{height / 2}' transform='rotate(-90 20,{height / 2})' text-anchor='middle'>{y_label}</text>",
           f"<line x1='{margin}' y1='{height-margin}' x2='{width-margin}' y2='{height-margin}' stroke='black'/>",
           f"<line x1='{margin}' y1='{margin}' x2='{margin}' y2='{height-margin}' stroke='black'/>"]
    for x_value, y_value in pairs:
        px = margin + (x_value - x0) / (x1 - x0) * (width - 2 * margin)
        py = height - margin - min(y_value, y1) / y1 * (height - 2 * margin)
        svg.append(f"<circle cx='{px:.2f}' cy='{py:.2f}' r='1.5' fill='#1f77b4' opacity='0.35'/>")
    svg.append("</svg>")
    Path(path).write_text("\n".join(svg) + "\n", encoding="utf-8")


def analyze_inland(args):
    plan = json.loads(args.plan.read_text(encoding="utf-8"))
    edges = read_csv(args.edge_csv)
    indexed, health_stats = health_rows(args.health_csv)
    if args.glim_csv:
        # The Stage 2C-B GLIM trajectory recorder emits one row per accepted
        # pose state in pose-key order.  Using its complete row count avoids
        # treating an absent diagnostic row as a pose state that never existed.
        expected_pose_keys = list(range(len(read_csv(args.glim_csv))))
    else:
        expected_pose_keys = sorted(indexed)
    boundaries = plan["conditioning_bins"]["boundaries"]
    labels = plan["conditioning_bins"]["labels"]
    enriched = []
    for edge in edges:
        row = dict(edge)
        row.update(edge_scan_summary(row, indexed, expected_pose_keys))
        row["rotation_condition_bin"] = conditioning_bin(
            row["stage2c_c_max_rotation_condition_ratio"], boundaries, labels)
        row["translation_condition_bin"] = conditioning_bin(
            row["stage2c_c_max_translation_condition_ratio"], boundaries, labels)
        row["stage2c_c_rotation_absolute_degenerate"] = str(
            int(row["stage2c_c_rotation_absolute_degenerate_count"] > 0))
        row["stage2c_c_translation_absolute_degenerate"] = str(
            int(row["stage2c_c_translation_absolute_degenerate_count"] > 0))
        enriched.append(row)

    rotation_field = "stage2c_c_max_rotation_condition_ratio"
    translation_field = "stage2c_c_max_translation_condition_ratio"
    rotation_log = "stage2c_c_log_max_rotation_condition_ratio"
    translation_log = "stage2c_c_log_max_translation_condition_ratio"
    associations = {
        "log_rotation_condition_vs_rotation_error": bootstrap_statistic(
            enriched, rotation_log, "rotation_error_rad", plan),
        "log_translation_condition_vs_translation_error": bootstrap_statistic(
            enriched, translation_log, "translation_error_m", plan),
        "rotation_absolute_occupancy_vs_rotation_error": bootstrap_statistic(
            enriched, "stage2c_c_rotation_absolute_degenerate_fraction",
            "rotation_error_rad", plan),
        "translation_absolute_occupancy_vs_translation_error": bootstrap_statistic(
            enriched, "stage2c_c_translation_absolute_degenerate_fraction",
            "translation_error_m", plan),
        "rotation_absolute_run_vs_rotation_error": bootstrap_statistic(
            enriched, "stage2c_c_longest_rotation_absolute_degenerate_run",
            "rotation_error_rad", plan),
        "translation_absolute_run_vs_translation_error": bootstrap_statistic(
            enriched, "stage2c_c_longest_translation_absolute_degenerate_run",
            "translation_error_m", plan),
    }
    secondary = {}
    for predictor in ("aggregate_source_point_count_median",
                      "aggregate_inlier_fraction_median",
                      "aggregate_initial_cost_median",
                      "aggregate_final_cost_median"):
        secondary[predictor] = {
            "rotation_error_rho": spearman(
                [row.get(predictor) for row in enriched],
                [row.get("rotation_error_rad") for row in enriched]),
            "translation_error_rho": spearman(
                [row.get(predictor) for row in enriched],
                [row.get("translation_error_m") for row in enriched]),
        }

    rotation_dynamic = dynamic_range(
        enriched, rotation_field, "rotation_condition_bin", plan)
    translation_dynamic = dynamic_range(
        enriched, translation_field, "translation_condition_bin", plan)
    summary = {
        "schema_version": 1,
        "semantics": "preregistered passive within-Inland conditioning/error association",
        "analysis_plan": str(args.plan.resolve()),
        "analysis_plan_sha256": sha256(args.plan),
        "gt_valid_unique_edge_count": len(enriched),
        "health_records": health_stats,
        "stage2a_reconstruction_consistency": stage2a_consistency(enriched),
        "conditioning_dynamic_range": {
            "rotation": rotation_dynamic,
            "translation": translation_dynamic,
        },
        "absolute_degeneracy": {
            "rotation_edge_count": sum(int(row["stage2c_c_rotation_absolute_degenerate_count"]) > 0 for row in enriched),
            "translation_edge_count": sum(int(row["stage2c_c_translation_absolute_degenerate_count"]) > 0 for row in enriched),
            "rotation_fraction": sum(int(row["stage2c_c_rotation_absolute_degenerate_count"]) > 0 for row in enriched) / len(enriched) if enriched else None,
            "translation_fraction": sum(int(row["stage2c_c_translation_absolute_degenerate_count"]) > 0 for row in enriched) / len(enriched) if enriched else None,
        },
        "conditioning_bins": {
            "rotation": bin_error_summaries(enriched, "rotation_condition", "rotation_error_rad", plan),
            "translation": bin_error_summaries(enriched, "translation_condition", "translation_error_m", plan),
        },
        "primary_block_bootstrap_associations": associations,
        "support_and_matching_spearman": secondary,
        "relative_health_available_edge_count": sum(
            truth(row.get("relative_health_valid")) for row in enriched),
        "decision": (
            "ABSOLUTE_ANALYSIS_INFORMATIVE"
            if rotation_dynamic["informative"] or translation_dynamic["informative"]
            else "ANALYSIS_UNINFORMATIVE_NARROW_CONDITIONING_RANGE"),
        "interpretation": [
            "Condition ratios and occupancy are descriptive LiDAR observability indicators, not metric-error predictors.",
            "Ground truth is used only by this offline evaluator and never enters Stage 1 reference construction.",
            "Rotation radians and translation metres remain separate.",
            "Confidence intervals resample fixed ten-second temporal blocks, not individual rows.",
        ],
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "inland_unique_edge_conditioning_analysis.csv", enriched)
    write_scatter_svg(
        args.output_dir / "rotation_condition_vs_gt_error.svg", enriched,
        rotation_log, "rotation_error_rad",
        "Inland: rotational conditioning versus GT edge error",
        "ln(max rotational condition ratio)", "rotation edge error [rad]")
    write_scatter_svg(
        args.output_dir / "translation_condition_vs_gt_error.svg", enriched,
        translation_log, "translation_error_m",
        "Inland: translational conditioning versus GT edge error",
        "ln(max translational condition ratio)", "translation edge error [m]")
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False))
    return summary


def reference_screening(path, plan, expected_fingerprint=None):
    indexed, duplicate_stats = health_rows(path)
    rows = [indexed[key] for key in sorted(indexed)]
    accepted = [row for row in rows if truth(row.get("baseline_update_accepted"))]
    ready = [row for row in rows if truth(row.get("reference_ready"))]
    first_ready_stamp = float(ready[0]["timestamp"]) if ready else None
    bootstrap_accepted = ([row for row in accepted
                           if float(row["timestamp"]) <= first_ready_stamp]
                          if first_ready_stamp is not None else accepted)
    stamps = [float(row["timestamp"]) for row in bootstrap_accepted]
    span = max(stamps) - min(stamps) if len(stamps) >= 2 else 0.0
    gaps = np.diff(sorted(stamps)) if len(stamps) >= 2 else np.asarray([])
    within_span = ([row for row in rows
                    if stamps and min(stamps) <= float(row["timestamp"]) <= max(stamps)]
                   if stamps else [])
    acceptance_fraction = (len(bootstrap_accepted) / len(within_span)
                           if within_span else 0.0)
    fingerprints = sorted(set(row.get("configuration_fingerprint", "") for row in rows))
    fingerprint_match = (expected_fingerprint is None
                         or fingerprints == [expected_fingerprint])
    criteria = plan["reference_screening"]
    qualification_checks = {
        "stage1_reference_ready": bool(ready),
        "minimum_accepted_samples": len(bootstrap_accepted) >= int(
            criteria["minimum_accepted_samples"]),
        "minimum_accepted_span": span >= float(criteria["minimum_accepted_span_sec"]),
        "maximum_accepted_gap": (bool(gaps.size) and float(np.max(gaps)) <= float(
            criteria["maximum_gap_between_accepted_samples_sec"])),
        "minimum_acceptance_fraction": acceptance_fraction >= float(
            criteria["minimum_acceptance_fraction_within_qualification_span"]),
        "configuration_fingerprint_match": fingerprint_match,
    }
    rot = [mode_maximum(row, "rotation") for row in rows]
    trans = [mode_maximum(row, "translation") for row in rows]
    valid = [row for row in rows if truth(row.get("valid"))
             and truth(row.get("factorization_ok"))]
    reasons = Counter((row.get("baseline_update_reason") or "none") for row in rows)
    return {
        "health_csv": str(Path(path).resolve()),
        "health_csv_sha256": sha256(path),
        "aggregate_sample_count": len(rows),
        "valid_hessian_sample_count": len(valid),
        "accepted_reference_candidate_count": len(accepted),
        "bootstrap_accepted_candidate_count": len(bootstrap_accepted),
        "reference_ready_sample_count": len(ready),
        "first_reference_ready_frame": int(ready[0]["frame_id"]) if ready else None,
        "first_reference_ready_timestamp": first_ready_stamp,
        "accepted_span_sec": span,
        "maximum_accepted_gap_sec": float(np.max(gaps)) if gaps.size else None,
        "acceptance_fraction_within_qualification_span": acceptance_fraction,
        "qualification_checks": qualification_checks,
        "qualified": all(qualification_checks.values()),
        "configuration_fingerprints": fingerprints,
        "baseline_rejection_reason_counts": dict(reasons),
        "maximum_rotation_condition_ratio": numeric_summary(rot),
        "maximum_translation_condition_ratio": numeric_summary(trans),
        "absolute_rotation_sample_count": sum(absolute_flag(row, "rotation") is True for row in valid),
        "absolute_translation_sample_count": sum(absolute_flag(row, "translation") is True for row in valid),
        "per_resolution_support_and_matching": per_resolution_support(path),
        "registration_converged_count": sum(truth(row.get("registration_converged")) for row in rows),
        "linear_solve_success_count": sum(truth(row.get("linear_solve_success")) for row in rows),
        "duplicate_stats": duplicate_stats,
    }


def screen_reference(args):
    plan = json.loads(args.plan.read_text(encoding="utf-8"))
    result = reference_screening(args.health_csv, plan, args.expected_fingerprint)
    result["dataset"] = args.dataset
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return result


def export_profile(args):
    plan = json.loads(args.plan.read_text(encoding="utf-8"))
    screening = reference_screening(args.health_csv, plan, args.expected_fingerprint)
    if not screening["qualified"]:
        raise ValueError("reference source did not satisfy frozen qualification criteria")
    indexed, _stats = health_rows(args.health_csv)
    rows = [indexed[key] for key in sorted(indexed)]
    ready = [row for row in rows if truth(row.get("reference_ready"))]
    first_ready_stamp = float(ready[0]["timestamp"])
    accepted = [row for row in rows if truth(row.get("baseline_update_accepted"))
                and float(row["timestamp"]) <= first_ready_stamp]
    rotation_logs = np.asarray([[math.log(float(row[f"rotation_condition_ratio_{index}"]))
                                 for index in range(3)] for row in accepted])
    translation_logs = np.asarray([[math.log(float(row[f"translation_condition_ratio_{index}"]))
                                    for index in range(3)] for row in accepted])
    metadata = ready[0]
    resolutions = sorted(set(float(row["resolution"]) for row in read_csv(args.health_csv)
                             if row.get("record_kind") == "factor"
                             and finite(row.get("resolution"))))
    core = {
        "schema_version": 2,
        "metadata": {
            "sensor_identifier": metadata["sensor_identifier"],
            "registration_type": metadata["registration_type"],
            "voxel_resolutions": resolutions,
            "pose_ordering": metadata["pose_ordering"],
            "tangent_convention": metadata["tangent_convention"],
            "configuration_fingerprint": metadata["configuration_fingerprint"],
        },
        "rotation_log_condition_ratios": np.median(rotation_logs, axis=0).tolist(),
        "translation_log_condition_ratios": np.median(translation_logs, axis=0).tolist(),
        "rotation_log_condition_mad": [median_absolute_deviation(rotation_logs[:, index])
                                       for index in range(3)],
        "translation_log_condition_mad": [median_absolute_deviation(
            translation_logs[:, index]) for index in range(3)],
    }
    provenance = {
        "stage2c_c_profile_provenance_schema_version": 1,
        "source_sequence": args.dataset,
        "source_health_csv": str(args.health_csv.resolve()),
        "source_health_csv_sha256": sha256(args.health_csv),
        "accepted_sample_count": len(accepted),
        "accepted_start_timestamp": min(float(row["timestamp"]) for row in accepted),
        "accepted_end_timestamp": max(float(row["timestamp"]) for row in accepted),
        "gamma_calibration_sha256": args.gamma_calibration_sha256,
        "adapter_manifest_sha256": args.adapter_manifest_sha256,
        "glim_config_manifest_sha256": args.glim_config_manifest_sha256,
        "ground_truth_used": False,
        "contains_error_calibration": False,
    }
    core["provenance"] = provenance
    core["reference_lineage_seed"] = json_hash({"core": core, "source": args.dataset})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(core, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    print(json.dumps(core, indent=2, sort_keys=True))
    return core


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    inland = subparsers.add_parser("analyze-inland")
    inland.add_argument("--plan", type=Path, required=True)
    inland.add_argument("--edge-csv", type=Path, required=True)
    inland.add_argument("--health-csv", type=Path, required=True)
    inland.add_argument("--glim-csv", type=Path)
    inland.add_argument("--output-dir", type=Path, required=True)
    screen = subparsers.add_parser("screen-reference")
    screen.add_argument("--plan", type=Path, required=True)
    screen.add_argument("--dataset", required=True)
    screen.add_argument("--health-csv", type=Path, required=True)
    screen.add_argument("--expected-fingerprint")
    screen.add_argument("--output", type=Path, required=True)
    export = subparsers.add_parser("export-profile")
    export.add_argument("--plan", type=Path, required=True)
    export.add_argument("--dataset", required=True)
    export.add_argument("--health-csv", type=Path, required=True)
    export.add_argument("--expected-fingerprint", required=True)
    export.add_argument("--gamma-calibration-sha256", required=True)
    export.add_argument("--adapter-manifest-sha256", required=True)
    export.add_argument("--glim-config-manifest-sha256", required=True)
    export.add_argument("--output", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.command == "analyze-inland":
        analyze_inland(args)
    elif args.command == "screen-reference":
        screen_reference(args)
    elif args.command == "export-profile":
        export_profile(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
