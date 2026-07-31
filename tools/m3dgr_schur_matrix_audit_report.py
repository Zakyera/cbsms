#!/usr/bin/env python3
"""Generate full-matrix Schur audit samples for CBS relative covariance.

This report shows the actual matrices used to compute the Schur-relative
information and covariance for a small sample of odometry beliefs.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, List, Mapping, Sequence, Tuple

import numpy as np


WORKSPACE = Path("/workspace/cbs_gtsam4.3")
DEFAULT_RUN = (
    WORKSPACE
    / "runs/20260715-205548_m3dgr_mid360_k2g_nees_horizon02_rate05_matrix"
)
DEFAULT_OUTPUT = (
    WORKSPACE
    / "src/cbsms/docs/2026-07-16_m3dgr_k2g_schur_full_matrix_audit_10samples.md"
)

REQUIRED_LABELS = (
    "lambda_y_joint_information_12x12",
    "lambda_u_relative_information_12x12",
    "schur_A_information_6x6",
    "schur_B_information_6x6",
    "schur_C_information_6x6",
    "lambda_rel_schur_information_6x6",
    "sigma_rel_schur_covariance_6x6",
    "sigma_rel_direct_full12x12_propagated_6x6",
    "H_from_between_jacobian_6x6",
    "H_to_between_jacobian_6x6",
    "change_of_variables_M_12x12",
)


def parse_matrix(values: str) -> np.ndarray:
    return np.array([[float(value) for value in row.split(":")] for row in values.split(";")])


def fmt(value: float) -> str:
    return f"{value:.6e}"


def matrix_markdown(matrix: np.ndarray) -> str:
    return "\n".join(
        "[" + ", ".join(fmt(float(value)) for value in row) + "]" for row in matrix
    )


def evenly_spaced(items: Sequence[Mapping[str, object]], count: int) -> List[Mapping[str, object]]:
    if len(items) <= count:
        return list(items)
    indices = [round(index * (len(items) - 1) / (count - 1)) for index in range(count)]
    return [items[int(index)] for index in indices]


def load_groups(
    run_dir: Path, direction: str, *, keep_repeated_pairs: bool = False
) -> List[Dict[str, object]]:
    path = run_dir / "parsed/cbs_odom_relative_covariance_matrices.csv"
    groups_by_sample: Dict[int, Dict[str, object]] = {}
    latest_by_pair: Dict[Tuple[str, str], Tuple[int, Dict[str, object]]] = {}
    with path.open() as handle:
        for row in csv.DictReader(handle):
            if row["direction"] != direction or row["matrix_label"] not in REQUIRED_LABELS:
                continue
            try:
                matrix = parse_matrix(row["matrix_values"])
            except ValueError:
                continue
            sample_index = int(row["sample_index"])
            group = groups_by_sample.setdefault(
                sample_index,
                {
                    "sample_index": sample_index,
                    "from_key": row["from_key"],
                    "to_key": row["to_key"],
                    "matrices": {},
                },
            )
            group["matrices"][row["matrix_label"]] = matrix

    complete_groups: List[Dict[str, object]] = []
    for group in groups_by_sample.values():
        matrices = group["matrices"]
        if not all(label in matrices for label in REQUIRED_LABELS):
            continue
        complete_groups.append(group)
        pair = (str(group["from_key"]), str(group["to_key"]))
        sample_index = int(group["sample_index"])
        if pair not in latest_by_pair or sample_index >= latest_by_pair[pair][0]:
            latest_by_pair[pair] = (sample_index, group)

    if keep_repeated_pairs:
        return sorted(complete_groups, key=lambda group: int(group["sample_index"]))
    return [value[1] for value in sorted(latest_by_pair.values(), key=lambda item: item[0])]


def safe_inverse(matrix: np.ndarray) -> np.ndarray:
    try:
        return np.linalg.solve(matrix, np.eye(matrix.shape[0]))
    except np.linalg.LinAlgError:
        return np.linalg.pinv(matrix)


def append_matrix(lines: List[str], title: str, matrix: np.ndarray) -> None:
    lines.append(f"`{title}`:")
    lines.append("")
    lines.append("```text")
    lines.append(matrix_markdown(matrix))
    lines.append("```")
    lines.append("")


def append_sample(lines: List[str], group: Mapping[str, object], ordinal: int) -> None:
    matrices = group["matrices"]
    lambda_y = matrices["lambda_y_joint_information_12x12"]
    lambda_u = matrices["lambda_u_relative_information_12x12"]
    a = matrices["schur_A_information_6x6"]
    b = matrices["schur_B_information_6x6"]
    c = matrices["schur_C_information_6x6"]
    lambda_rel = matrices["lambda_rel_schur_information_6x6"]
    sigma_rel = matrices["sigma_rel_schur_covariance_6x6"]
    sigma_direct = matrices["sigma_rel_direct_full12x12_propagated_6x6"]
    m = matrices["change_of_variables_M_12x12"]
    h_from = matrices["H_from_between_jacobian_6x6"]
    h_to = matrices["H_to_between_jacobian_6x6"]

    computed_lambda_u = m.T @ lambda_y @ m
    computed_schur = c - b.T @ safe_inverse(a) @ b
    computed_sigma = safe_inverse(lambda_rel)

    lines.append(f"## Sample {ordinal}: `{group['from_key']} -> {group['to_key']}`")
    lines.append("")
    lines.append(f"- sample index: `{group['sample_index']}`")
    lines.append(f"- `trace(Lambda_rel)`: `{fmt(float(np.trace(lambda_rel)))}`")
    lines.append(f"- `trace(Sigma_rel)`: `{fmt(float(np.trace(sigma_rel)))}`")
    lines.append(
        "- `diag(Sigma_rel) [rot rad^2, trans m^2]`: `"
        + " ".join(fmt(float(value)) for value in np.diag(sigma_rel))
        + "`"
    )
    lines.append(f"- `||M^T Lambda_y M - Lambda_u||_F`: `{fmt(float(np.linalg.norm(computed_lambda_u - lambda_u)))}`")
    lines.append(f"- `||C - B^T A^-1 B - Lambda_rel||_F`: `{fmt(float(np.linalg.norm(computed_schur - lambda_rel)))}`")
    lines.append(f"- `||inv(Lambda_rel) - Sigma_rel||_F`: `{fmt(float(np.linalg.norm(computed_sigma - sigma_rel)))}`")
    lines.append(f"- `||Sigma_direct_full12 - Sigma_rel||_F`: `{fmt(float(np.linalg.norm(sigma_direct - sigma_rel)))}`")
    lines.append("")

    lines.append("The computation for this sample is:")
    lines.append("")
    lines.append("```text")
    lines.append("y = [delta_from; delta_to]")
    lines.append("u = [delta_from; delta_rel]")
    lines.append("y = M u")
    lines.append("Lambda_u = M^T Lambda_y M")
    lines.append("Lambda_u = [A, B; B^T, C]")
    lines.append("Lambda_rel = C - B^T A^-1 B")
    lines.append("Sigma_rel = Lambda_rel^-1")
    lines.append("```")
    lines.append("")

    append_matrix(lines, "Lambda_y_joint_information_12x12", lambda_y)
    append_matrix(lines, "change_of_variables_M_12x12", m)
    append_matrix(lines, "Lambda_u_relative_information_12x12 = M^T Lambda_y M", lambda_u)
    append_matrix(lines, "H_from_between_jacobian_6x6", h_from)
    append_matrix(lines, "H_to_between_jacobian_6x6", h_to)
    append_matrix(lines, "A = Lambda_u[0:6, 0:6]", a)
    append_matrix(lines, "B = Lambda_u[0:6, 6:12]", b)
    append_matrix(lines, "C = Lambda_u[6:12, 6:12]", c)
    append_matrix(lines, "Lambda_rel = C - B^T A^-1 B", lambda_rel)
    append_matrix(lines, "Sigma_rel = Lambda_rel^-1", sigma_rel)


def append_sample_summary(lines: List[str], samples: Sequence[Mapping[str, object]]) -> None:
    lines.append("## Sample Summary")
    lines.append("")
    lines.append("| # | edge | sample | trace Lambda_rel | trace Sigma_rel | diag Sigma_rel |")
    lines.append("|---:|---|---:|---:|---:|---|")
    for ordinal, group in enumerate(samples, start=1):
        matrices = group["matrices"]
        lambda_rel = matrices["lambda_rel_schur_information_6x6"]
        sigma_rel = matrices["sigma_rel_schur_covariance_6x6"]
        diag = " ".join(fmt(float(value)) for value in np.diag(sigma_rel))
        lines.append(
            f"| {ordinal} | `{group['from_key']} -> {group['to_key']}` | "
            f"{group['sample_index']} | {fmt(float(np.trace(lambda_rel)))} | "
            f"{fmt(float(np.trace(sigma_rel)))} | `{diag}` |"
        )
    lines.append("")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--direction", choices=("K2G", "G2K"), default="K2G")
    parser.add_argument("--sample-count", type=int, default=10)
    parser.add_argument(
        "--keep-repeated-pairs",
        action="store_true",
        help="Treat every complete matrix dump as a sample instead of retaining only the latest dump per endpoint pair.",
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    groups = load_groups(
        args.run_dir,
        args.direction,
        keep_repeated_pairs=args.keep_repeated_pairs,
    )
    if not groups:
        raise RuntimeError(f"no complete {args.direction} Schur matrix groups found in {args.run_dir}")
    samples = evenly_spaced(groups, max(1, args.sample_count))

    lines: List[str] = []
    lines.append("# M3DGR K2G Schur Relative Covariance Full-Matrix Audit")
    lines.append("")
    lines.append(f"- Run: `{args.run_dir}`")
    lines.append(f"- Direction: `{args.direction}`")
    group_label = "complete matrix dumps" if args.keep_repeated_pairs else "complete latest belief pairs"
    lines.append(f"- Samples shown: `{len(samples)}` of `{len(groups)}` {group_label}")
    lines.append("- Ordering: GTSAM `Pose3` tangent order `[rot_x, rot_y, rot_z, trans_x, trans_y, trans_z]`.")
    lines.append("- Units: rotation covariance in `rad^2`, translation covariance in `m^2`.")
    lines.append("")
    lines.append("This is the full-matrix check for the Schur relative covariance. The receiver factor uses the full `Sigma_rel` matrix, not only its trace.")
    lines.append("")
    lines.append("The key point is that the Schur complement is applied after changing variables from `[delta_from, delta_to]` to `[delta_from, delta_rel]`. Therefore the blocks `A`, `B`, `C` are blocks of `Lambda_u = M^T Lambda_y M`, not direct blocks of the original `Lambda_y`.")
    lines.append("")
    append_sample_summary(lines, samples)
    for ordinal, sample in enumerate(samples, start=1):
        append_sample(lines, sample, ordinal)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines) + "\n")
    print(args.output)


if __name__ == "__main__":
    main()
