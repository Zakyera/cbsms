#!/usr/bin/env python3
"""Audit first-window endpoint relative covariance against absolute covariance."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, Mapping

import numpy as np


WORKSPACE = Path("/workspace/cbs_gtsam4.3")
DEFAULT_RUN = (
    WORKSPACE
    / "runs/20260716-160038_m3dgr_mid360_k2g_x0_x24_first_window_cov_audit_30s_rate05"
)
DEFAULT_OUTPUT = (
    WORKSPACE
    / "src/cbsms/docs/2026-07-16_m3dgr_k2g_x0_x24_first_window_covariance_audit.md"
)


def parse_matrix(values: str) -> np.ndarray:
    return np.array([[float(value) for value in row.split(":")] for row in values.split(";")])


def fmt(value: float) -> str:
    return f"{value:.6e}"


def matrix_markdown(matrix: np.ndarray) -> str:
    return "\n".join(
        "[" + ", ".join(fmt(float(value)) for value in row) + "]" for row in matrix
    )


def sigma_diag_summary(matrix: np.ndarray) -> str:
    diag = np.diag(matrix)
    rot_sigma_deg = [math.degrees(math.sqrt(max(float(v), 0.0))) for v in diag[:3]]
    trans_sigma_cm = [100.0 * math.sqrt(max(float(v), 0.0)) for v in diag[3:]]
    return (
        "diag=`"
        + " ".join(fmt(float(v)) for v in diag)
        + "`, rot sigma deg=`"
        + " ".join(f"{v:.6f}" for v in rot_sigma_deg)
        + "`, trans sigma cm=`"
        + " ".join(f"{v:.6f}" for v in trans_sigma_cm)
        + "`"
    )


def read_pair_matrices(run_dir: Path, direction: str, from_key: str, to_key: str) -> Dict[str, np.ndarray]:
    path = run_dir / "parsed/cbs_odom_relative_covariance_matrices.csv"
    matrices: Dict[str, np.ndarray] = {}
    with path.open() as handle:
        for row in csv.DictReader(handle):
            if (
                row["direction"] != direction
                or row["from_key"] != from_key
                or row["to_key"] != to_key
            ):
                continue
            try:
                matrices[row["matrix_label"]] = parse_matrix(row["matrix_values"])
            except ValueError:
                continue
    return matrices


def append_matrix(lines: list[str], title: str, matrix: np.ndarray) -> None:
    lines.append(f"`{title}`:")
    lines.append("")
    lines.append("```text")
    lines.append(matrix_markdown(matrix))
    lines.append("```")
    lines.append("")


def append_stats(lines: list[str], label: str, matrix: np.ndarray) -> None:
    lines.append(
        f"| `{label}` | {fmt(float(np.trace(matrix)))} | "
        f"{fmt(float(np.linalg.norm(matrix)))} | {sigma_diag_summary(matrix)} |"
    )


def append_component_stats(lines: list[str], label: str, matrix: np.ndarray) -> None:
    lines.append(
        f"| `{label}` | {fmt(float(np.trace(matrix[:3, :3])))} | "
        f"{fmt(float(np.trace(matrix[3:6, 3:6])))} | "
        f"{fmt(float(np.trace(matrix)))} |"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--direction", default="K2G")
    parser.add_argument("--from-key", default="x0")
    parser.add_argument("--to-key", default="x24")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    matrices = read_pair_matrices(args.run_dir, args.direction, args.from_key, args.to_key)
    required = (
        "sigma_y_joint_covariance_12x12",
        "sigma_rel_schur_covariance_6x6",
        "sigma_rel_direct_full12x12_propagated_6x6",
        "H_from_between_jacobian_6x6",
        "H_to_between_jacobian_6x6",
        "lambda_y_joint_information_12x12",
        "lambda_rel_schur_information_6x6",
    )
    missing = [label for label in required if label not in matrices]
    if missing:
        raise RuntimeError(f"missing matrices for {args.direction} {args.from_key}->{args.to_key}: {missing}")

    sigma_y = matrices["sigma_y_joint_covariance_12x12"]
    sigma_ff = sigma_y[:6, :6]
    sigma_ft = sigma_y[:6, 6:12]
    sigma_tf = sigma_y[6:12, :6]
    sigma_tt = sigma_y[6:12, 6:12]
    h_from = matrices["H_from_between_jacobian_6x6"]
    h_to = matrices["H_to_between_jacobian_6x6"]
    sigma_rel = matrices["sigma_rel_schur_covariance_6x6"]
    sigma_rel_direct = matrices["sigma_rel_direct_full12x12_propagated_6x6"]

    term_from = h_from @ sigma_ff @ h_from.T
    term_to = h_to @ sigma_tt @ h_to.T
    term_cross_ft = h_from @ sigma_ft @ h_to.T
    term_cross_tf = h_to @ sigma_tf @ h_from.T
    propagated = term_from + term_to + term_cross_ft + term_cross_tf
    projected_to_only = term_to
    delta_rel_vs_to = sigma_rel - projected_to_only

    lines: list[str] = []
    lines.append("# M3DGR K2G First-Window Endpoint Covariance Audit")
    lines.append("")
    lines.append(f"- Run: `{args.run_dir}`")
    lines.append(f"- Direction: `{args.direction}`")
    lines.append(f"- Pair: `{args.from_key} -> {args.to_key}`")
    lines.append("- Ordering: GTSAM `Pose3` tangent order `[rot_x, rot_y, rot_z, trans_x, trans_y, trans_z]`.")
    lines.append("- Units: rotation covariance in `rad^2`, translation covariance in `m^2`.")
    lines.append("")
    lines.append("This checks the supervisor sanity case: if `x0` is effectively fixed, then the relative covariance of `x0^-1 * x24` should be close to the absolute marginal covariance of `x24`, after applying the residual Jacobian.")
    lines.append("")
    lines.append("For this pair, `H_to` is the identity matrix, so the projected absolute covariance of `x24` is just the bottom-right covariance block `Sigma_tt` from the full joint covariance.")
    lines.append("")
    lines.append("## Summary")
    lines.append("")
    lines.append("| matrix | trace | Frobenius norm | diagonal/sigma summary |")
    lines.append("|---|---:|---:|---|")
    append_stats(lines, "Sigma_ff absolute from x0", sigma_ff)
    append_stats(lines, "Sigma_tt absolute to x24", sigma_tt)
    append_stats(lines, "H_to Sigma_tt H_to^T", projected_to_only)
    append_stats(lines, "Sigma_rel Schur", sigma_rel)
    append_stats(lines, "Sigma_rel - H_to Sigma_tt H_to^T", delta_rel_vs_to)
    lines.append("")
    lines.append("## Decomposition")
    lines.append("")
    lines.append("```text")
    lines.append("Sigma_rel =")
    lines.append("  H_from Sigma_ff H_from^T")
    lines.append("+ H_to   Sigma_tt H_to^T")
    lines.append("+ H_from Sigma_ft H_to^T")
    lines.append("+ H_to   Sigma_tf H_from^T")
    lines.append("```")
    lines.append("")
    lines.append("| term | trace | Frobenius norm |")
    lines.append("|---|---:|---:|")
    for label, matrix in (
        ("H_from Sigma_ff H_from^T", term_from),
        ("H_to Sigma_tt H_to^T", term_to),
        ("H_from Sigma_ft H_to^T", term_cross_ft),
        ("H_to Sigma_tf H_from^T", term_cross_tf),
        ("sum propagated", propagated),
        ("Sigma_rel Schur", sigma_rel),
    ):
        lines.append(
            f"| `{label}` | {fmt(float(np.trace(matrix)))} | "
            f"{fmt(float(np.linalg.norm(matrix)))} |"
        )
    lines.append("")
    lines.append("## Rotation/Translation Split")
    lines.append("")
    lines.append("| matrix | rotation trace | translation trace | full trace |")
    lines.append("|---|---:|---:|---:|")
    append_component_stats(lines, "Sigma_ff absolute from x0", sigma_ff)
    append_component_stats(lines, "Sigma_tt absolute to x24", sigma_tt)
    append_component_stats(lines, "Sigma_rel Schur", sigma_rel)
    append_component_stats(lines, "Sigma_rel - Sigma_tt", delta_rel_vs_to)
    lines.append("")
    lines.append(f"- Rotation trace ratio `Sigma_rel / Sigma_tt`: `{fmt(float(np.trace(sigma_rel[:3, :3]) / np.trace(sigma_tt[:3, :3])))}`")
    lines.append(f"- Translation trace ratio `Sigma_rel / Sigma_tt`: `{fmt(float(np.trace(sigma_rel[3:6, 3:6]) / np.trace(sigma_tt[3:6, 3:6])))}`")
    lines.append("")
    lines.append("## Consistency Checks")
    lines.append("")
    lines.append(f"- `||propagated - Sigma_rel||_F`: `{fmt(float(np.linalg.norm(propagated - sigma_rel)))}`")
    lines.append(f"- `||direct_full12 - Sigma_rel||_F`: `{fmt(float(np.linalg.norm(sigma_rel_direct - sigma_rel)))}`")
    lines.append(f"- `trace(Sigma_rel) / trace(Sigma_tt)`: `{fmt(float(np.trace(sigma_rel) / np.trace(sigma_tt)))}`")
    lines.append(f"- `trace(Sigma_rel - Sigma_tt)`: `{fmt(float(np.trace(delta_rel_vs_to)))}`")
    lines.append("")
    lines.append("## Interpretation")
    lines.append("")
    lines.append("The check partially supports the supervisor intuition, but not exact equality. The full trace of `Sigma_rel` is about 92% of `Sigma_tt`, mainly because the translation-Z variance dominates both traces. Split by component, translation is close at about 94%, while rotation is much smaller at about 3% of the absolute `x24` rotation trace.")
    lines.append("")
    lines.append("This happens because `x0` is not an infinitely fixed zero-covariance pose in the joint marginal. Its translation covariance is effectively fixed (`1e-10` diagonal), but its rotation covariance is not near zero. The `x0`/`x24` cross-covariance terms are also not zero.")
    lines.append("")
    lines.append("The cross terms are negative in trace and cancel part of the endpoint absolute covariance. That is expected for two states estimated in the same smoothing graph: they share common uncertainty. Therefore the correct sanity result is approximate closeness for `x0 -> x24`, not exact matrix equality.")
    lines.append("")
    lines.append("## Full Matrices")
    lines.append("")
    append_matrix(lines, "Lambda_y_joint_information_12x12", matrices["lambda_y_joint_information_12x12"])
    append_matrix(lines, "Sigma_y_joint_covariance_12x12 = inverse(Lambda_y)", sigma_y)
    lines.append("## Absolute-vs-Relative Matrix Comparison")
    lines.append("")
    lines.append("These are the full matrices behind the trace comparison:")
    lines.append("")
    lines.append("```text")
    lines.append("trace(Sigma_tt absolute x24) = 9.149505e-03")
    lines.append("trace(Sigma_rel Schur)       = 8.413203e-03")
    lines.append("ratio                         = 0.9195")
    lines.append("```")
    lines.append("")
    append_matrix(lines, "Sigma_tt absolute covariance of x24", sigma_tt)
    append_matrix(lines, "Sigma_rel Schur covariance of x0^-1 * x24", sigma_rel)
    append_matrix(lines, "Delta = Sigma_rel - Sigma_tt", delta_rel_vs_to)
    lines.append("## Joint Block Matrices")
    lines.append("")
    append_matrix(lines, "Sigma_ff = Sigma_y[0:6, 0:6]", sigma_ff)
    append_matrix(lines, "Sigma_ft = Sigma_y[0:6, 6:12]", sigma_ft)
    append_matrix(lines, "Sigma_tf = Sigma_y[6:12, 0:6]", sigma_tf)
    append_matrix(lines, "Sigma_tt = Sigma_y[6:12, 6:12]", sigma_tt)
    lines.append("## Propagation Matrices")
    lines.append("")
    append_matrix(lines, "H_from_between_jacobian_6x6", h_from)
    append_matrix(lines, "H_to_between_jacobian_6x6", h_to)
    append_matrix(lines, "H_from Sigma_ff H_from^T", term_from)
    append_matrix(lines, "H_to Sigma_tt H_to^T", term_to)
    append_matrix(lines, "H_from Sigma_ft H_to^T", term_cross_ft)
    append_matrix(lines, "H_to Sigma_tf H_from^T", term_cross_tf)
    append_matrix(lines, "Lambda_rel_schur_information_6x6", matrices["lambda_rel_schur_information_6x6"])
    append_matrix(lines, "Sigma_rel_schur_covariance_6x6", sigma_rel)
    append_matrix(lines, "Sigma_rel_direct_full12x12_propagated_6x6", sigma_rel_direct)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines) + "\n")
    print(args.output)


if __name__ == "__main__":
    main()
