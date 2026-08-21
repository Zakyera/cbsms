#!/usr/bin/env python3
"""Stream a completed CBS covariance audit into its existing Rerun recording."""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import rerun as rr


TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
from geode_stage3a_rerun import make_blueprint  # noqa: E402


def finite_scalar(path, value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return
    if math.isfinite(number):
        rr.log(path, rr.Scalars(number))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--audit-dir", type=Path, required=True)
    parser.add_argument("--recording-id", required=True)
    parser.add_argument("--connect", default="rerun+http://127.0.0.1:9876/proxy")
    parser.add_argument("--cbs-label", default="CBS G-to-K covariance audit")
    args = parser.parse_args()

    sample_paths = sorted((args.audit_dir / "samples").glob("sample_*.json"))
    if not sample_paths:
        raise SystemExit(f"no sample JSON files under {args.audit_dir}")
    rr.init("cbsms", recording_id=args.recording_id)
    rr.connect_grpc(args.connect)
    rr.send_blueprint(make_blueprint(np.zeros(3), 50.0,
                                     cbs_label=args.cbs_label,
                                     include_covariance_audit=True))
    rr.log("covariance_audit/documentation",
           rr.TextDocument((args.audit_dir / "REPORT.md").read_text()),
           static=True)

    for path in sample_paths:
        sample = json.loads(path.read_text())
        index = int(sample["sample_index"])
        match = sample.get("timestamp_match") or {}
        stamp = match.get("sender_to_stamp")
        if stamp is not None and math.isfinite(float(stamp)):
            rr.set_time("time", timestamp=float(stamp))
        rr.set_time("covariance_audit_sample", sequence=index)

        provenance = {
            "schur_formula": sample["schur_formula_difference_norm"],
            "schur_vs_direct": sample["schur_direct_difference_norm"],
            "raw_to_sender_processed": sample[
                "sender_raw_to_postprocessed_difference_norm"],
            "sender_processed_to_received": sample[
                "sender_shadow_to_received_difference_norm"],
            "received_to_source_scaled": sample[
                "received_to_scaled_difference_norm"],
            "source_scaled_to_inserted": sample[
                "scaled_to_inserted_difference_norm"],
            "raw_sender_to_inserted": sample[
                "sender_raw_to_inserted_difference_norm"],
        }
        for name, value in provenance.items():
            finite_scalar(f"covariance_audit/provenance/{name}", value)

        finite_scalar("covariance_audit/covariance/rotation_trace",
                      sample["inserted_rotation_trace"])
        finite_scalar("covariance_audit/covariance/translation_trace",
                      sample["inserted_translation_trace"])
        finite_scalar("covariance_audit/covariance/condition_number",
                      sample["inserted_condition_number"])
        for component, value in zip(
                ("rot_x", "rot_y", "rot_z", "trans_x", "trans_y", "trans_z"),
                sample["inserted_diagonal"]):
            finite_scalar(f"covariance_audit/covariance/diagonal/{component}", value)

        residual = np.asarray(sample["preinjection_residual"], dtype=float)
        finite_scalar("covariance_audit/residual/rotation_norm",
                      np.linalg.norm(residual[:3]))
        finite_scalar("covariance_audit/residual/translation_norm",
                      np.linalg.norm(residual[3:]))
        finite_scalar("covariance_audit/residual/nis", sample["nis"])
        finite_scalar("covariance_audit/residual/factor_error",
                      sample["factor_error"])

        finite_scalar("covariance_audit/strength/external_local_trace_ratio",
                      sample["external_to_local_information_trace_ratio"])
        generalized = np.asarray(
            sample["external_to_local_information_generalized_eigenvalues"],
            dtype=float)
        if generalized.size and np.any(np.isfinite(generalized)):
            finite_scalar("covariance_audit/strength/generalized_min",
                          np.nanmin(generalized))
            finite_scalar("covariance_audit/strength/generalized_median",
                          np.nanmedian(generalized))
            finite_scalar("covariance_audit/strength/generalized_max",
                          np.nanmax(generalized))

        for group in sample["factor_groups"]:
            category = group["category"]
            finite_scalar(f"covariance_audit/factor_groups/{category}/error_sum",
                          group["error_sum"])
            finite_scalar(f"covariance_audit/factor_groups/{category}/hessian_frobenius",
                          group["hessian_frobenius"])
            finite_scalar(f"covariance_audit/factor_groups/{category}/count",
                          group["count"])
    rr.disconnect()
    print(f"streamed {len(sample_paths)} covariance samples to {args.recording_id}")


if __name__ == "__main__":
    main()
