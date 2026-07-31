#!/usr/bin/env python3
"""Generate a compact M3DGR Kimera K2G relative-covariance report.

The report is meant for reviewing why the exported relative covariance can be
around 1e-5.  It reads CBS matrix dumps and writes a markdown report plus a CSV
of sampled diagonal entries.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple

import numpy as np


WORKSPACE = Path("/workspace/cbs_gtsam4.3")
OUTPUT_DIR = WORKSPACE / "src/cbsms/docs"
DEFAULT_REPORT_PATH = OUTPUT_DIR / "2026-07-16_m3dgr_kimera_k2g_relative_covariance_report.md"
DEFAULT_SAMPLE_CSV_PATH = OUTPUT_DIR / "2026-07-16_m3dgr_kimera_k2g_relative_covariance_diagonal_samples.csv"

RUNS: Sequence[Tuple[str, Path, str]] = (
    (
        "0.2",
        WORKSPACE / "runs/20260715-205548_m3dgr_mid360_k2g_nees_horizon02_rate05_matrix",
        "horizon02 matrix dump",
    ),
    (
        "0.8",
        WORKSPACE / "runs/20260715-210030_m3dgr_mid360_k2g_nees_horizon08_rate05_matrix",
        "horizon08 matrix dump",
    ),
    (
        "1.0",
        WORKSPACE / "runs/20260715-210426_m3dgr_mid360_k2g_nees_horizon10_rate05_matrix",
        "horizon10 matrix dump",
    ),
    (
        "2.0",
        WORKSPACE / "runs/20260715-210825_m3dgr_mid360_k2g_nees_horizon20_rate05_matrix",
        "horizon20 matrix dump",
    ),
)

MATRIX_LABELS = (
    "sigma_rel_schur_covariance_6x6",
    "sigma_rel_direct_full12x12_propagated_6x6",
    "sigma_y_joint_covariance_12x12",
    "H_from_between_jacobian_6x6",
    "H_to_between_jacobian_6x6",
)


def parse_matrix(values: str) -> np.ndarray:
    return np.array([[float(value) for value in row.split(":")] for row in values.split(";")])


def quantile(values: Sequence[float], probability: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = (len(ordered) - 1) * probability
    lo = int(math.floor(index))
    hi = int(math.ceil(index))
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - index) + ordered[hi] * (index - lo)


def fmt(value: float) -> str:
    return f"{value:.3e}"


def fmt_fixed(value: float, precision: int = 4) -> str:
    return f"{value:.{precision}f}"


def matrix_markdown(matrix: np.ndarray) -> str:
    rows = []
    for row in matrix:
        rows.append("[" + ", ".join(fmt(float(value)) for value in row) + "]")
    return "\n".join(rows)


def evenly_spaced(items: Sequence[Mapping[str, object]], count: int) -> List[Mapping[str, object]]:
    if len(items) <= count:
        return list(items)
    indices = [round(index * (len(items) - 1) / (count - 1)) for index in range(count)]
    return [items[int(index)] for index in indices]


def load_latest_direction_groups(run_dir: Path, direction: str) -> List[Dict[str, object]]:
    path = run_dir / "parsed/cbs_odom_relative_covariance_matrices.csv"
    latest_by_pair: Dict[Tuple[str, str], Tuple[int, Dict[str, object]]] = {}
    groups_by_sample: Dict[int, Dict[str, object]] = {}
    with path.open() as handle:
        for row in csv.DictReader(handle):
            if row["direction"] != direction or row["matrix_label"] not in MATRIX_LABELS:
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
    for group in groups_by_sample.values():
        matrices = group["matrices"]
        if not all(label in matrices for label in MATRIX_LABELS):
            continue
        pair = (str(group["from_key"]), str(group["to_key"]))
        sample_index = int(group["sample_index"])
        if pair not in latest_by_pair or sample_index >= latest_by_pair[pair][0]:
            latest_by_pair[pair] = (sample_index, group)
    return [value[1] for value in sorted(latest_by_pair.values(), key=lambda item: item[0])]


def summarize(groups: Sequence[Mapping[str, object]]) -> Dict[str, float]:
    traces = []
    rot_traces = []
    trans_traces = []
    rot_sigmas_deg = []
    trans_sigmas_cm = []
    for group in groups:
        sigma = group["matrices"]["sigma_rel_schur_covariance_6x6"]
        rot_trace = float(np.trace(sigma[:3, :3]))
        trans_trace = float(np.trace(sigma[3:, 3:]))
        traces.append(float(np.trace(sigma)))
        rot_traces.append(rot_trace)
        trans_traces.append(trans_trace)
        rot_sigmas_deg.append(math.degrees(math.sqrt(max(rot_trace, 0.0) / 3.0)))
        trans_sigmas_cm.append(100.0 * math.sqrt(max(trans_trace, 0.0) / 3.0))
    return {
        "count": float(len(groups)),
        "trace_p50": quantile(traces, 0.5),
        "trace_p05": quantile(traces, 0.05),
        "trace_p95": quantile(traces, 0.95),
        "rot_trace_p50": quantile(rot_traces, 0.5),
        "trans_trace_p50": quantile(trans_traces, 0.5),
        "rot_sigma_deg_p50": quantile(rot_sigmas_deg, 0.5),
        "trans_sigma_cm_p50": quantile(trans_sigmas_cm, 0.5),
    }


def representative_group(groups: Sequence[Mapping[str, object]]) -> Mapping[str, object]:
    traces = [
        float(np.trace(group["matrices"]["sigma_rel_schur_covariance_6x6"]))
        for group in groups
    ]
    target = quantile(traces, 0.5)
    return min(
        groups,
        key=lambda group: abs(
            float(np.trace(group["matrices"]["sigma_rel_schur_covariance_6x6"])) - target
        ),
    )


def cancellation_terms(group: Mapping[str, object]) -> Dict[str, float]:
    matrices = group["matrices"]
    sigma_y = matrices["sigma_y_joint_covariance_12x12"]
    h_from = matrices["H_from_between_jacobian_6x6"]
    h_to = matrices["H_to_between_jacobian_6x6"]
    sigma_ff = sigma_y[:6, :6]
    sigma_ft = sigma_y[:6, 6:]
    sigma_tf = sigma_y[6:, :6]
    sigma_tt = sigma_y[6:, 6:]
    sigma_rel = matrices["sigma_rel_schur_covariance_6x6"]
    sigma_direct = matrices.get("sigma_rel_direct_full12x12_propagated_6x6", sigma_rel)
    from_trace = float(np.trace(h_from @ sigma_ff @ h_from.T))
    to_trace = float(np.trace(h_to @ sigma_tt @ h_to.T))
    cross_trace = float(
        np.trace(h_from @ sigma_ft @ h_to.T) + np.trace(h_to @ sigma_tf @ h_from.T)
    )
    return {
        "sigma_ff_trace": float(np.trace(sigma_ff)),
        "sigma_tt_trace": float(np.trace(sigma_tt)),
        "propagated_from_trace": from_trace,
        "propagated_to_trace": to_trace,
        "propagated_cross_trace": cross_trace,
        "propagated_sum_trace": from_trace + to_trace + cross_trace,
        "schur_trace": float(np.trace(sigma_rel)),
        "direct_trace": float(np.trace(sigma_direct)),
        "schur_direct_norm": float(np.linalg.norm(sigma_rel - sigma_direct)),
    }


def write_sample_csv(
    sample_csv_path: Path,
    direction: str,
    samples_by_horizon: Mapping[str, Sequence[Mapping[str, object]]],
) -> None:
    with sample_csv_path.open("w", newline="") as handle:
        fieldnames = [
            "direction",
            "horizon_sec",
            "from_key",
            "to_key",
            "sample_index",
            "var_rot_x_rad2",
            "var_rot_y_rad2",
            "var_rot_z_rad2",
            "var_trans_x_m2",
            "var_trans_y_m2",
            "var_trans_z_m2",
            "trace",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for horizon, samples in samples_by_horizon.items():
            for group in samples:
                sigma = group["matrices"]["sigma_rel_schur_covariance_6x6"]
                diag = np.diag(sigma)
                writer.writerow(
                    {
                        "direction": direction,
                        "horizon_sec": horizon,
                        "from_key": group["from_key"],
                        "to_key": group["to_key"],
                        "sample_index": group["sample_index"],
                        "var_rot_x_rad2": f"{diag[0]:.12e}",
                        "var_rot_y_rad2": f"{diag[1]:.12e}",
                        "var_rot_z_rad2": f"{diag[2]:.12e}",
                        "var_trans_x_m2": f"{diag[3]:.12e}",
                        "var_trans_y_m2": f"{diag[4]:.12e}",
                        "var_trans_z_m2": f"{diag[5]:.12e}",
                        "trace": f"{float(np.trace(sigma)):.12e}",
                    }
                )


def append_sample_table(
    lines: List[str],
    direction: str,
    horizon: str,
    samples: Sequence[Mapping[str, object]],
) -> None:
    lines.append(f"### Horizon {horizon} s: 10 {direction} Sigma_rel diagonal samples")
    lines.append("")
    lines.append(
        "| from->to | sample | var_rx | var_ry | var_rz | var_tx | var_ty | var_tz | trace |"
    )
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|")
    for group in samples:
        sigma = group["matrices"]["sigma_rel_schur_covariance_6x6"]
        diag = np.diag(sigma)
        lines.append(
            f"| {group['from_key']}->{group['to_key']} | {group['sample_index']} | "
            f"{fmt(diag[0])} | {fmt(diag[1])} | {fmt(diag[2])} | "
            f"{fmt(diag[3])} | {fmt(diag[4])} | {fmt(diag[5])} | "
            f"{fmt(float(np.trace(sigma)))} |"
        )
    lines.append("")


def append_representative(lines: List[str], horizon: str, group: Mapping[str, object]) -> None:
    matrices = group["matrices"]
    sigma_rel = matrices["sigma_rel_schur_covariance_6x6"]
    sigma_y = matrices["sigma_y_joint_covariance_12x12"]
    terms = cancellation_terms(group)
    lines.append(f"### Horizon {horizon} s: representative matrix near median trace")
    lines.append("")
    lines.append(f"- Pair: `{group['from_key']} -> {group['to_key']}`")
    lines.append(f"- Sample index: `{group['sample_index']}`")
    lines.append(f"- `trace(Sigma_rel)`: `{fmt(float(np.trace(sigma_rel)))}`")
    lines.append("")
    lines.append("| quantity | value |")
    lines.append("|---|---:|")
    for key in (
        "sigma_ff_trace",
        "sigma_tt_trace",
        "propagated_from_trace",
        "propagated_to_trace",
        "propagated_cross_trace",
        "propagated_sum_trace",
        "schur_trace",
        "direct_trace",
        "schur_direct_norm",
    ):
        lines.append(f"| `{key}` | {fmt(terms[key])} |")
    lines.append("")
    lines.append("`diag(Sigma_ff)`:")
    lines.append("")
    lines.append("```text")
    lines.append(" ".join(fmt(value) for value in np.diag(sigma_y[:6, :6])))
    lines.append("```")
    lines.append("")
    lines.append("`diag(Sigma_tt)`:")
    lines.append("")
    lines.append("```text")
    lines.append(" ".join(fmt(value) for value in np.diag(sigma_y[6:, 6:])))
    lines.append("```")
    lines.append("")
    lines.append("`Sigma_rel_schur_covariance_6x6`:")
    lines.append("")
    lines.append("```text")
    lines.append(matrix_markdown(sigma_rel))
    lines.append("```")
    lines.append("")
    lines.append("`Sigma_y_joint_covariance_12x12` over `[delta_from; delta_to]`:")
    lines.append("")
    lines.append("```text")
    lines.append(matrix_markdown(sigma_y))
    lines.append("```")
    lines.append("")


def default_output_paths(direction: str) -> Tuple[Path, Path]:
    if direction == "K2G":
        return DEFAULT_REPORT_PATH, DEFAULT_SAMPLE_CSV_PATH
    token = direction.lower()
    return (
        OUTPUT_DIR / f"2026-07-16_m3dgr_{token}_relative_covariance_report.md",
        OUTPUT_DIR / f"2026-07-16_m3dgr_{token}_relative_covariance_diagonal_samples.csv",
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--direction", choices=("K2G", "G2K"), default="K2G")
    parser.add_argument("--report-path", type=Path, default=None)
    parser.add_argument("--sample-csv-path", type=Path, default=None)
    args = parser.parse_args()
    report_path, sample_csv_path = default_output_paths(args.direction)
    if args.report_path is not None:
        report_path = args.report_path
    if args.sample_csv_path is not None:
        sample_csv_path = args.sample_csv_path

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    all_groups: Dict[str, List[Dict[str, object]]] = {}
    summaries: Dict[str, Dict[str, float]] = {}
    samples_by_horizon: Dict[str, List[Mapping[str, object]]] = {}
    representatives: Dict[str, Mapping[str, object]] = {}

    for horizon, run_dir, _ in RUNS:
        groups = load_latest_direction_groups(run_dir, args.direction)
        if not groups:
            raise RuntimeError(f"no {args.direction} covariance groups found in {run_dir}")
        all_groups[horizon] = groups
        summaries[horizon] = summarize(groups)
        samples_by_horizon[horizon] = evenly_spaced(groups, 10)
        representatives[horizon] = representative_group(groups)

    sample_csv_path.parent.mkdir(parents=True, exist_ok=True)
    write_sample_csv(sample_csv_path, args.direction, samples_by_horizon)

    lines: List[str] = []
    if args.direction == "K2G":
        title = "M3DGR Dynamic01 Kimera K2G Relative Covariance Check"
        direction_text = "Kimera outgoing relative beliefs that GLIM would receive"
        trace_text = "these K2G relative beliefs"
    else:
        title = "M3DGR Dynamic01 GLIM G2K Relative Covariance Check"
        direction_text = "GLIM outgoing relative beliefs that Kimera would receive"
        trace_text = "these G2K relative beliefs"
    other_direction = "G2K" if args.direction == "K2G" else "K2G"
    lines.append(f"# {title}")
    lines.append("")
    lines.append("This report is generated from CBS matrix dumps to show where the `~1e-5` relative covariance traces come from.")
    lines.append("")
    lines.append("Important conventions:")
    lines.append("")
    lines.append(f"- Direction shown here: `{args.direction}` only, i.e. {direction_text}.")
    lines.append("- Ordering is GTSAM `Pose3`: `[rot_x, rot_y, rot_z, trans_x, trans_y, trans_z]`.")
    lines.append("- Rotation covariance units are `rad^2`; translation covariance units are `m^2`.")
    lines.append("- `Sigma_rel` is the Schur relative covariance currently sent by CBS.")
    lines.append(f"- These files may also contain {other_direction} rows, but all tables below filter to {args.direction} rows only.")
    lines.append("")
    lines.append("## Horizon Summary")
    lines.append("")
    lines.append(f"| horizon s | {args.direction} pairs | trace p50 | trace p05 | trace p95 | rot trace p50 | trans trace p50 | one-axis rot sigma p50 | one-axis trans sigma p50 |")
    lines.append("|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for horizon, _, _ in RUNS:
        stats = summaries[horizon]
        lines.append(
            f"| {horizon} | {int(stats['count'])} | {fmt(stats['trace_p50'])} | "
            f"{fmt(stats['trace_p05'])} | {fmt(stats['trace_p95'])} | "
            f"{fmt(stats['rot_trace_p50'])} | {fmt(stats['trans_trace_p50'])} | "
            f"{fmt_fixed(stats['rot_sigma_deg_p50'], 4)} deg | "
            f"{fmt_fixed(stats['trans_sigma_cm_p50'], 3)} cm |"
        )
    lines.append("")
    lines.append(f"The trace is dominated by the translation block for {trace_text}. The rotation block is very small, but it is still visible in the rotation diagonal entries.")
    lines.append("")
    lines.append("## Diagonal Samples")
    lines.append("")
    lines.append(f"The same diagonal samples are also saved as CSV: `{sample_csv_path.relative_to(WORKSPACE)}`.")
    lines.append("")
    for horizon, _, _ in RUNS:
        append_sample_table(lines, args.direction, horizon, samples_by_horizon[horizon])
    lines.append("## Representative Full Matrices")
    lines.append("")
    lines.append(f"Each representative row is the {args.direction} sample whose `trace(Sigma_rel)` is closest to the median trace for that horizon.")
    lines.append("")
    lines.append("The cancellation table shows why the final relative trace can be tiny even when `Sigma_ff` and `Sigma_tt` look larger: the two endpoint poses are highly correlated, and the cross term cancels most shared uncertainty.")
    lines.append("")
    for horizon, _, _ in RUNS:
        append_representative(lines, horizon, representatives[horizon])

    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text("\n".join(lines) + "\n")
    print(report_path)
    print(sample_csv_path)


if __name__ == "__main__":
    main()
