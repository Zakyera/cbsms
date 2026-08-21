#!/usr/bin/env python3
"""Build a matrix-level audit of a passive CBS relative-pose covariance run.

The tool joins the existing sender Schur dump, the sender belief-shadow row,
the receiver's exact inserted factor, and the receiver objective audit.  It is
strictly an offline reporter: it does not alter CBS messages or factors.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from collections import defaultdict
from pathlib import Path

import numpy as np


SENDER_MATRIX_MARKER = "CBS_ODOM_RELATIVE_COVARIANCE_MATRIX_ROW,"
SENDER_SUMMARY_MARKER = "CBS_ODOM_RELATIVE_COVARIANCE_ROW,"
SHADOW_MARKER = "GLIM_DCREG_BELIEF_SHADOW_ROW,"
RECEIVER_MATRIX_MARKER = "CBS_COVARIANCE_EXCHANGE_MATRIX_ROW,"
RECEIVER_SUMMARY_MARKER = "CBS_COVARIANCE_EXCHANGE_SUMMARY_ROW,"
RECEIVER_RELATIVE_MARKER = "CBS_COVARIANCE_EXCHANGE_RECEIVER_RELATIVE_ROW,"
FACTOR_GROUP_MARKER = "CBS_COVARIANCE_EXCHANGE_FACTOR_GROUP_ROW,"
GLOBAL_FACTOR_GROUP_MARKER = "CBS_COVARIANCE_EXCHANGE_GLOBAL_FACTOR_GROUP_ROW,"
ACTIVE_FACTOR_MARKER = "CBS_COVARIANCE_EXCHANGE_ACTIVE_FACTOR_ROW,"
MATCH_MARKER = "CBS_ODOM_MATCH_ROW_G2K,"


def fields_after(line: str, marker: str) -> list[str] | None:
    if marker not in line:
        return None
    payload = line.split(marker, 1)[1]
    payload = re.sub(r"\x1b\[[0-9;]*m", "", payload).strip()
    return payload.split(",")


def vector_token(token: str) -> np.ndarray:
    if not token:
        return np.empty((0,), dtype=np.float64)
    return np.asarray([float(value) for value in token.split(";")], dtype=np.float64)


def matrix_token(token: str, rows: int, cols: int) -> np.ndarray:
    parsed_rows = [[float(value) for value in row.split(":")]
                   for row in token.split(";")]
    matrix = np.asarray(parsed_rows, dtype=np.float64)
    if matrix.shape != (rows, cols):
        raise ValueError(f"matrix token has {matrix.shape}, expected {(rows, cols)}")
    return matrix


def json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {key: json_value(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_value(item) for item in value]
    return value


def sender_edge(receiver_key: str) -> str:
    match = re.fullmatch(r"([A-Za-z])p(\d+)", receiver_key)
    if not match:
        return receiver_key
    return f"{match.group(1)}{match.group(2)}"


def pose_index(key: str) -> int:
    match = re.search(r"(\d+)$", key)
    if not match:
        raise ValueError(f"pose key has no index: {key}")
    return int(match.group(1))


def parse_sender(log_path: Path):
    bundles = defaultdict(lambda: defaultdict(dict))
    summaries = defaultdict(list)
    shadows = defaultdict(list)
    with log_path.open(errors="replace") as stream:
        for line in stream:
            fields = fields_after(line, SENDER_MATRIX_MARKER)
            if fields and len(fields) >= 10:
                direction, sender, receiver, from_key, to_key = fields[:5]
                dump_index = int(fields[5])
                label = fields[6]
                matrix = matrix_token(fields[9], int(fields[7]), int(fields[8]))
                bundles[(direction, from_key, to_key)][dump_index][label] = matrix
                continue
            fields = fields_after(line, SENDER_SUMMARY_MARKER)
            if fields and len(fields) >= 25:
                summaries[(fields[0], fields[3], fields[4])].append({
                    "mode": fields[5],
                    "conditional_trace": float(fields[6]),
                    "schur_trace": float(fields[7]),
                    "trace_ratio": float(fields[8]),
                    "schur_direct_difference_norm": float(fields[19]),
                    "rotation_trace": float(fields[15]),
                    "translation_trace": float(fields[16]),
                    "rank": int(float(fields[17])),
                    "direct_covariance_available": bool(int(float(fields[18]))),
                    "used_jitter": bool(int(float(fields[20]))),
                    "jitter_added": float(fields[21]),
                    "information_eigenvalues": vector_token(fields[22]),
                    "covariance_eigenvalues": vector_token(fields[23]),
                    "status": fields[24],
                })
                continue
            fields = fields_after(line, SHADOW_MARKER)
            if fields and len(fields) >= 16:
                edge = ("G2K", f"g{int(fields[1])}", f"g{int(fields[2])}")
                shadows[edge].append({
                    "publication_stamp": float(fields[0]),
                    "from_index": int(fields[1]),
                    "to_index": int(fields[2]),
                    "from_stamp": float(fields[3]),
                    "to_stamp": float(fields[4]),
                    "relative_mean_log": vector_token(fields[5]),
                    "relative_pose_translation": np.asarray(
                        [float(fields[6]), float(fields[7]), float(fields[8])]),
                    "relative_pose_quaternion_xyzw": np.asarray(
                        [float(fields[9]), float(fields[10]), float(fields[11]),
                         float(fields[12])]),
                    "covariance": matrix_token(fields[13], 6, 6),
                    "sender_alpha": float(fields[14]),
                    "mode": fields[15],
                })
    return bundles, summaries, shadows


def parse_receiver(log_path: Path):
    samples = {}
    matches = defaultdict(list)
    with log_path.open(errors="replace") as stream:
        for line in stream:
            fields = fields_after(line, RECEIVER_SUMMARY_MARKER)
            if fields and len(fields) >= 33:
                index = int(fields[7])
                existing = samples.get(index, {})
                samples[index] = {
                    "direction": fields[0], "receiver": fields[1],
                    "source": fields[2], "sender_from": fields[3],
                    "sender_to": fields[4], "receiver_from": fields[5],
                    "receiver_to": fields[6], "sample_index": index,
                    "action": fields[8], "receiver_pose_source": fields[9],
                    "robust_noise_enabled": bool(int(fields[10])),
                    "information_valid": bool(int(fields[11])),
                    "raw_spd": bool(int(fields[12])),
                    "source_scaled_spd": bool(int(fields[13])),
                    "inserted_spd": bool(int(fields[14])),
                    "received_to_scaled_difference_norm": float(fields[15]),
                    "scaled_to_inserted_difference_norm": float(fields[16]),
                    "inserted_trace": float(fields[17]),
                    "inserted_rotation_trace": float(fields[18]),
                    "inserted_translation_trace": float(fields[19]),
                    "inserted_diagonal": vector_token(fields[20]),
                    "inserted_eigenvalues": vector_token(fields[21]),
                    "inserted_condition_number": float(fields[22]),
                    "measurement_log": vector_token(fields[23]),
                    "receiver_relative_log": vector_token(fields[24]),
                    "preinjection_residual": vector_token(fields[25]),
                    "unwhitened_residual": vector_token(fields[26]),
                    "whitened_residual": vector_token(fields[27]),
                    "nis": float(fields[28]), "factor_error": float(fields[29]),
                    "external_hessian_trace": float(fields[30]),
                    "external_hessian_frobenius": float(fields[31]),
                    "external_gradient_norm": float(fields[32]),
                    "matrices": existing.get("matrices", {}),
                    "receiver_relative": existing.get("receiver_relative", {}),
                    "factor_groups": existing.get("factor_groups", []),
                    "global_factor_groups": existing.get("global_factor_groups", []),
                    "active_factors": existing.get("active_factors", []),
                }
                continue
            fields = fields_after(line, RECEIVER_MATRIX_MARKER)
            if fields and len(fields) >= 13:
                index = int(fields[7])
                sample = samples.setdefault(index, {"sample_index": index,
                                                     "matrices": {},
                                                     "receiver_relative": {},
                                                     "factor_groups": [],
                                                     "global_factor_groups": [],
                                                     "active_factors": []})
                stage, label = fields[8], fields[9]
                sample["matrices"][f"{stage}/{label}"] = matrix_token(
                    fields[12], int(fields[10]), int(fields[11]))
                continue
            fields = fields_after(line, RECEIVER_RELATIVE_MARKER)
            if fields and len(fields) >= 16:
                index = int(fields[7])
                samples[index]["receiver_relative"][fields[8]] = {
                    "trace": float(fields[9]), "rotation_trace": float(fields[10]),
                    "translation_trace": float(fields[11]),
                    "schur_direct_difference_norm": float(fields[12]),
                    "used_jitter": bool(int(fields[13])),
                    "jitter_added": float(fields[14]), "status": fields[15],
                }
                continue
            fields = fields_after(line, GLOBAL_FACTOR_GROUP_MARKER)
            if fields and len(fields) >= 17:
                index = int(fields[7])
                samples[index]["global_factor_groups"].append({
                    "category": fields[8], "exact_type": fields[9],
                    "count": int(fields[10]), "error_sum": float(fields[11]),
                    "hessian_dimension": int(fields[12]),
                    "hessian_trace": float(fields[13]),
                    "hessian_frobenius": float(fields[14]),
                    "eta_norm": float(fields[15]),
                    "gradient_norm": float(fields[16]),
                })
                continue
            fields = fields_after(line, FACTOR_GROUP_MARKER)
            if fields and len(fields) >= 16:
                index = int(fields[7])
                samples[index]["factor_groups"].append({
                    "category": fields[8], "count": int(fields[9]),
                    "error_sum": float(fields[10]),
                    "hessian_dimension": int(fields[11]),
                    "hessian_trace": float(fields[12]),
                    "hessian_frobenius": float(fields[13]),
                    "eta_norm": float(fields[14]),
                    "gradient_norm": float(fields[15]),
                })
                continue
            fields = fields_after(line, ACTIVE_FACTOR_MARKER)
            if fields and len(fields) >= 16:
                index = int(fields[7])
                samples[index]["active_factors"].append({
                    "factor_index": int(fields[8]), "category": fields[9],
                    "touches_from": bool(int(fields[10])),
                    "touches_to": bool(int(fields[11])),
                    "keys": fields[12], "exact_type": fields[13],
                    "error": float(fields[14]),
                    "endpoint_hessian_frobenius": float(fields[15]),
                })
                continue
            fields = fields_after(line, MATCH_MARKER)
            if fields and len(fields) >= 16:
                source = fields[0]
                match = re.fullmatch(r"p:([A-Za-z]):(\d+)->p:\1:(\d+)", source)
                if not match:
                    continue
                receiver_match = re.fullmatch(
                    r"p:([A-Za-z]):(\d+)->p:\1:(\d+)", fields[3])
                edge = (f"{match.group(1)}p{match.group(2)}",
                        f"{match.group(1)}p{match.group(3)}")
                matches[edge].append({
                    "sender_from_stamp": float(fields[1]),
                    "sender_to_stamp": float(fields[2]),
                    "receiver_edge": fields[3],
                    "receiver_from_stamp": float(fields[4]),
                    "receiver_to_stamp": float(fields[5]),
                    "from_abs_dt": float(fields[6]), "to_abs_dt": float(fields[7]),
                    "status": fields[11], "sender_duration": float(fields[12]),
                    "receiver_duration": float(fields[13]),
                    "duration_error": float(fields[14]),
                    "duration_ratio": float(fields[15]),
                    "receiver_from_index": (int(receiver_match.group(2))
                                            if receiver_match else None),
                    "receiver_to_index": (int(receiver_match.group(3))
                                          if receiver_match else None),
                })
    return samples, matches


def closest_sender_bundle(edge, raw_covariance, bundles):
    candidates = bundles.get(edge, {})
    scored = []
    for dump_index, matrices in candidates.items():
        covariance = matrices.get("sigma_rel_schur_covariance_6x6")
        if covariance is not None:
            scored.append((float(np.linalg.norm(covariance - raw_covariance)),
                           dump_index, matrices))
    if not scored:
        raise ValueError(f"no sender Schur matrix bundle for {edge}")
    return min(scored, key=lambda item: item[0])


def closest_shadow(edge, raw_covariance, shadows):
    candidates = shadows.get(edge, [])
    if not candidates:
        return math.inf, None
    return min(((float(np.linalg.norm(row["covariance"] - raw_covariance)), row)
                for row in candidates), key=lambda item: item[0])


def generalized_information_ratio(external: np.ndarray, local: np.ndarray):
    symmetric_local = 0.5 * (local + local.T)
    values, vectors = np.linalg.eigh(symmetric_local)
    if np.min(values) <= 0.0:
        return np.full(6, np.nan)
    inverse_root = vectors @ np.diag(1.0 / np.sqrt(values)) @ vectors.T
    comparison = inverse_root @ (0.5 * (external + external.T)) @ inverse_root
    return np.linalg.eigvalsh(0.5 * (comparison + comparison.T))


def enrich(samples, matches, sender_bundles, sender_summaries, shadows):
    enriched = []
    for index in sorted(samples):
        sample = samples[index]
        raw = sample["matrices"]["receiver_after_frame_conversion/covariance"]
        edge = (sample["direction"], sender_edge(sample["sender_from"]),
                sender_edge(sample["sender_to"]))
        bundle_difference, dump_index, bundle = closest_sender_bundle(
            edge, raw, sender_bundles)
        shadow_difference, shadow = closest_shadow(edge, raw, shadows)
        match_rows = [row for row in matches.get(
            (sample["sender_from"], sample["sender_to"]), [])
                      if pose_index(sample["receiver_from"]) ==
                      row["receiver_from_index"] and
                      pose_index(sample["receiver_to"]) ==
                      row["receiver_to_index"]]
        match = next((row for row in match_rows if row["status"] == "sender_odom_queued"),
                     match_rows[-1] if match_rows else None)

        lambda_rel = bundle["lambda_rel_schur_information_6x6"]
        sigma_rel = bundle["sigma_rel_schur_covariance_6x6"]
        sigma_direct = bundle["sigma_rel_direct_full12x12_propagated_6x6"]
        recomputed_lambda = (bundle["schur_C_information_6x6"] -
                             bundle["schur_B_information_6x6"].T @
                             np.linalg.solve(bundle["schur_A_information_6x6"],
                                             bundle["schur_B_information_6x6"]))
        inserted = sample["matrices"]["receiver_inserted_factor/covariance"]
        omega = sample["matrices"]["receiver_inserted_factor/information_omega"]
        local_stage = "receiver_preinjection_excluding_explicit_external_factors"
        local_lambda = sample["matrices"].get(
            f"{local_stage}/lambda_rel_schur_information_6x6")
        local_sigma = sample["matrices"].get(
            f"{local_stage}/sigma_rel_schur_covariance_6x6")
        strength_eigenvalues = (generalized_information_ratio(omega, local_lambda)
                                if local_lambda is not None else np.full(6, np.nan))
        matched_summary = None
        if sender_summaries.get(edge):
            matched_summary = min(sender_summaries[edge],
                                  key=lambda row: abs(row["schur_trace"] -
                                                      np.trace(sigma_rel)))

        sample.update({
            "sender_edge": edge, "sender_dump_index": dump_index,
            "sender_matrices": bundle, "sender_summary": matched_summary,
            "sender_shadow": shadow, "timestamp_match": match,
            "sender_bundle_to_received_difference_norm": bundle_difference,
            "sender_shadow_to_received_difference_norm": shadow_difference,
            "schur_formula_difference_norm": float(np.linalg.norm(
                lambda_rel - recomputed_lambda)),
            "schur_inverse_difference_norm": float(np.linalg.norm(
                sigma_rel - np.linalg.inv(lambda_rel))),
            "schur_direct_difference_norm": float(np.linalg.norm(
                sigma_rel - sigma_direct)),
            "sender_raw_to_postprocessed_difference_norm": (
                float(np.linalg.norm(sigma_rel - shadow["covariance"]))
                if shadow is not None else math.nan),
            "sender_raw_to_inserted_difference_norm": float(np.linalg.norm(
                sigma_rel - inserted)),
            "inserted_information_inverse_difference_norm": float(np.linalg.norm(
                omega - np.linalg.inv(inserted))),
            "receiver_local_information": local_lambda,
            "receiver_local_covariance": local_sigma,
            "external_to_local_information_trace_ratio": (
                float(np.trace(omega) / np.trace(local_lambda))
                if local_lambda is not None and np.trace(local_lambda) != 0.0
                else math.nan),
            "external_to_local_information_generalized_eigenvalues":
                strength_eigenvalues,
        })
        enriched.append(sample)
    return enriched


def matrix_markdown(matrix: np.ndarray) -> str:
    rows = ["[" + ", ".join(f"{value:.17g}" for value in row) + "]"
            for row in np.atleast_2d(matrix)]
    return "```text\n" + "\n".join(rows) + "\n```\n"


def sample_time_metadata(sample):
    match = sample.get("timestamp_match") or {}
    shadow = sample.get("sender_shadow") or {}
    from_stamp = match.get("sender_from_stamp", shadow.get("from_stamp", math.nan))
    to_stamp = match.get("sender_to_stamp", shadow.get("to_stamp", math.nan))
    duration = match.get("sender_duration", to_stamp - from_stamp)
    return from_stamp, to_stamp, duration


def write_sample_markdown(sample, path: Path):
    from_stamp, to_stamp, duration = sample_time_metadata(sample)
    lines = [
        f"# Covariance audit sample {sample['sample_index']}", "",
        f"- Direction: `{sample['direction']}`",
        f"- Sender edge: `{sample['sender_from']} -> {sample['sender_to']}`",
        f"- Receiver edge: `{sample['receiver_from']} -> {sample['receiver_to']}`",
        f"- Sender timestamps: `{from_stamp:.17g}` -> `{to_stamp:.17g}`",
        f"- Edge duration: `{duration:.17g}` s",
        "- Tangent ordering: `[rot_x, rot_y, rot_z, trans_x, trans_y, trans_z]`",
        f"- Sender dump bundle: `{sample['sender_dump_index']}`",
        "", "## Relative transformation mean", "",
        f"Logmap: `{';'.join(f'{v:.17g}' for v in sample['measurement_log'])}`", "",
        matrix_markdown(sample["matrices"]["receiver_measurement/relative_pose_matrix"]),
        "## Sender Schur construction", "",
    ]
    required_sender = [
        "lambda_y_joint_information_12x12", "lambda_ff_information_6x6",
        "lambda_ft_information_6x6", "lambda_tf_information_6x6",
        "lambda_tt_information_6x6",
        "H_from_between_jacobian_6x6", "H_to_between_jacobian_6x6",
        "schur_A_information_6x6",
        "schur_B_information_6x6", "schur_C_information_6x6",
        "lambda_rel_schur_information_6x6", "sigma_rel_schur_covariance_6x6",
        "sigma_rel_direct_full12x12_propagated_6x6",
    ]
    for label in required_sender:
        lines.extend([f"### {label}", "", matrix_markdown(
            sample["sender_matrices"][label])])
    lines.extend([
        "## Sender validation and covariance provenance", "",
        f"- Recomputed Schur information difference norm: "
        f"`{sample['schur_formula_difference_norm']:.17g}`",
        f"- Schur inverse difference norm: "
        f"`{sample['schur_inverse_difference_norm']:.17g}`",
        f"- Direct 12x12 propagation difference norm: "
        f"`{sample['schur_direct_difference_norm']:.17g}`",
        f"- Raw sender -> sender processed difference norm: "
        f"`{sample['sender_raw_to_postprocessed_difference_norm']:.17g}`",
        f"- Sender processed -> received difference norm: "
        f"`{sample['sender_shadow_to_received_difference_norm']:.17g}`",
        f"- Received -> source-scaled difference norm: "
        f"`{sample['received_to_scaled_difference_norm']:.17g}`",
        f"- Source-scaled -> inserted difference norm: "
        f"`{sample['scaled_to_inserted_difference_norm']:.17g}`",
        f"- Raw sender -> inserted difference norm: "
        f"`{sample['sender_raw_to_inserted_difference_norm']:.17g}`",
        f"- SPD flags raw/scaled/inserted: `{sample['raw_spd']}/"
        f"{sample['source_scaled_spd']}/{sample['inserted_spd']}`",
        f"- Sender jitter used: `{bool((sample.get('sender_summary') or {}).get('used_jitter', False))}`",
        f"- Sender jitter: `{(sample.get('sender_summary') or {}).get('jitter_added', math.nan)}`",
        f"- Robust receiver noise wrapper: `{sample['robust_noise_enabled']}`",
        "", "### Raw sender Schur covariance", "",
        matrix_markdown(sample["sender_matrices"]["sigma_rel_schur_covariance_6x6"]),
    ])
    sender_shadow = sample.get("sender_shadow") or {}
    if sender_shadow.get("covariance") is not None:
        lines.extend([
            "### Covariance after sender processing", "",
            matrix_markdown(sender_shadow["covariance"]),
        ])
    lines.extend([
        "### Covariance received after receiver frame conversion", "",
        matrix_markdown(sample["matrices"]["receiver_after_frame_conversion/covariance"]),
        "### Covariance after configured receiver source scale", "",
        matrix_markdown(sample["matrices"]["receiver_after_source_scale/covariance"]),
        "### Inserted covariance", "",
        matrix_markdown(sample["matrices"]["receiver_inserted_factor/covariance"]),
        "### Inserted information Omega", "",
        matrix_markdown(sample["matrices"]["receiver_inserted_factor/information_omega"]),
        f"- Diagonal: `{';'.join(f'{v:.17g}' for v in sample['inserted_diagonal'])}`",
        f"- Rotation trace: `{sample['inserted_rotation_trace']:.17g}`",
        f"- Translation trace: `{sample['inserted_translation_trace']:.17g}`",
        f"- Eigenvalues: `{';'.join(f'{v:.17g}' for v in sample['inserted_eigenvalues'])}`",
        f"- Condition number: `{sample['inserted_condition_number']:.17g}`",
        "", "## Receiver factorization", "",
        f"- Pre-injection residual: `{';'.join(f'{v:.17g}' for v in sample['preinjection_residual'])}`",
        f"- Whitened residual: `{';'.join(f'{v:.17g}' for v in sample['whitened_residual'])}`",
        f"- NIS: `{sample['nis']:.17g}`",
        f"- Factor error: `{sample['factor_error']:.17g}` (NIS/2 for Gaussian noise)",
        "", "### Actual receiver BetweenFactor H_from", "",
        matrix_markdown(sample["matrices"]["receiver_factor_linearization/H_from"]),
        "### Actual receiver BetweenFactor H_to", "",
        matrix_markdown(sample["matrices"]["receiver_factor_linearization/H_to"]),
        "### External factor Hessian", "",
        matrix_markdown(sample["matrices"]["receiver_factor_linearization/external_factor_hessian"]),
        "### External factor gradient", "",
        matrix_markdown(sample["matrices"]["receiver_factor_linearization/external_factor_gradient_at_zero"]),
    ])
    if sample.get("receiver_local_information") is not None:
        lines.extend([
            "### Receiver local relative information (same endpoint reduction)", "",
            matrix_markdown(sample["receiver_local_information"]),
            "### Receiver local relative covariance", "",
            matrix_markdown(sample["receiver_local_covariance"]),
            f"- External/local information trace ratio: "
            f"`{sample['external_to_local_information_trace_ratio']:.17g}`",
            f"- Generalized external/local information eigenvalues: "
            f"`{';'.join(f'{v:.17g}' for v in sample['external_to_local_information_generalized_eigenvalues'])}`",
        ])
    lines.extend(["", "### Active receiver factors touching either endpoint", "",
                  "| index | category | exact type | keys | from | to | error | endpoint H frob |",
                  "|---:|---|---|---|---:|---:|---:|---:|"])
    for row in sample["active_factors"]:
        lines.append(f"| {row['factor_index']} | {row['category']} | `{row['exact_type']}` | "
                     f"`{row['keys']}` | {int(row['touches_from'])} | "
                     f"{int(row['touches_to'])} | {row['error']:.9g} | "
                     f"{row['endpoint_hessian_frobenius']:.9g} |")
    lines.extend(["", "### Receiver factors touching either audited endpoint, grouped by type", "",
                  "| category | count | error sum | H dimension | H trace | H frob | gradient norm |",
                  "|---|---:|---:|---:|---:|---:|---:|"])
    for row in sample["factor_groups"]:
        lines.append(f"| {row['category']} | {row['count']} | {row['error_sum']:.9g} | "
                     f"{row['hessian_dimension']} | {row['hessian_trace']:.9g} | "
                     f"{row['hessian_frobenius']:.9g} | {row['gradient_norm']:.9g} |")
    lines.extend([
        "", "### Complete active optimization update, grouped by factor type", "",
        "These whole-update totals provide composition context. They are not used in place of the exact same-endpoint receiver reduction above.", "",
        "| category | exact factor type | count | error sum | H dimension | H trace | H frob | gradient norm |",
        "|---|---|---:|---:|---:|---:|---:|---:|",
    ])
    for row in sample.get("global_factor_groups", []):
        lines.append(
            f"| {row['category']} | `{row['exact_type']}` | {row['count']} | {row['error_sum']:.9g} | "
            f"{row['hessian_dimension']} | {row['hessian_trace']:.9g} | "
            f"{row['hessian_frobenius']:.9g} | {row['gradient_norm']:.9g} |")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_outputs(samples, output: Path):
    output.mkdir(parents=True, exist_ok=True)
    sample_dir = output / "samples"
    sample_dir.mkdir(exist_ok=True)
    summary_fields = [
        "sample_index", "sender_from", "sender_to", "receiver_from", "receiver_to",
        "sender_from_stamp", "sender_to_stamp", "edge_duration_sec",
        "inserted_rotation_trace", "inserted_translation_trace",
        "inserted_condition_number", "schur_formula_difference_norm",
        "schur_direct_difference_norm", "sender_raw_to_postprocessed_difference_norm",
        "sender_shadow_to_received_difference_norm",
        "received_to_scaled_difference_norm", "scaled_to_inserted_difference_norm",
        "sender_raw_to_inserted_difference_norm", "nis", "factor_error",
        "external_to_local_information_trace_ratio", "sender_jitter_used",
        "sender_jitter_added", "raw_spd", "source_scaled_spd", "inserted_spd",
    ]
    rows = []
    for sample in samples:
        from_stamp, to_stamp, duration = sample_time_metadata(sample)
        sender_summary = sample.get("sender_summary") or {}
        row = {key: sample.get(key, math.nan) for key in summary_fields}
        row.update({
            "sender_from_stamp": from_stamp,
            "sender_to_stamp": to_stamp,
            "edge_duration_sec": duration,
            "sender_jitter_used": sender_summary.get("used_jitter", False),
            "sender_jitter_added": sender_summary.get("jitter_added", math.nan),
        })
        rows.append(row)
        (sample_dir / f"sample_{sample['sample_index']:02d}.json").write_text(
            json.dumps(json_value(sample), indent=2, sort_keys=True) + "\n",
            encoding="utf-8")
        write_sample_markdown(sample,
                              sample_dir / f"sample_{sample['sample_index']:02d}.md")
    with (output / "summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=summary_fields)
        writer.writeheader()
        writer.writerows(rows)

    maxima = {}
    for field in ("schur_formula_difference_norm", "schur_direct_difference_norm",
                  "sender_raw_to_postprocessed_difference_norm",
                  "sender_shadow_to_received_difference_norm",
                  "received_to_scaled_difference_norm",
                  "scaled_to_inserted_difference_norm",
                  "sender_raw_to_inserted_difference_norm"):
        finite = [abs(float(sample[field])) for sample in samples
                  if math.isfinite(float(sample[field]))]
        maxima[field] = max(finite) if finite else math.nan
    categories = sorted({factor["category"] for sample in samples
                         for factor in sample["active_factors"]})
    report = [
        "# CBS G→K relative-covariance audit", "",
        "This is a passive matrix-level audit. No covariance scale, CBS gate, factor, or optimizer behavior was changed.", "",
        f"- Distinct accepted G→K edges audited: **{len(samples)}**",
        "- Ordering: **[rot_x, rot_y, rot_z, trans_x, trans_y, trans_z]**",
        "- ROS covariance remapping: **none**",
        f"- Exact receiver factor categories observed: `{', '.join(categories)}`",
        "", "## Maximum numerical differences", "",
    ]
    report.extend(f"- `{key}`: `{value:.17g}`" for key, value in maxima.items())
    report.extend(["", "## Per-sample reports", ""])
    for sample in samples:
        report.append(f"- [Sample {sample['sample_index']:02d}](samples/sample_{sample['sample_index']:02d}.md): "
                      f"`{sample['sender_from']}→{sample['sender_to']}` / "
                      f"`{sample['receiver_from']}→{sample['receiver_to']}`")
    report.extend([
        "", "## Interpretation boundary", "",
        "The external/local information ratios are descriptive comparisons in the same GTSAM relative-error tangent. They are not tuning recommendations. Factor-group Hessian totals are also reported, but they are not substituted for the endpoint-reduced receiver information.",
    ])
    (output / "REPORT.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    return maxima


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--glim-log", type=Path, required=True)
    parser.add_argument("--kimera-log", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    sender_bundles, sender_summaries, shadows = parse_sender(args.glim_log)
    receiver_samples, matches = parse_receiver(args.kimera_log)
    samples = enrich(receiver_samples, matches, sender_bundles,
                     sender_summaries, shadows)
    if not samples:
        raise SystemExit("no covariance exchange audit samples found")
    maxima = write_outputs(samples, args.output)
    print(json.dumps({"samples": len(samples), "maxima": maxima}, indent=2))


if __name__ == "__main__":
    main()
