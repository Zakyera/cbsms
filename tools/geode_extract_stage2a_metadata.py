#!/usr/bin/env python3
"""Extract Stage 2A arrays by publication sequence, independent of bag order.

At high deterministic replay rates the asynchronous metadata callback can be
recorded before the corresponding legacy-belief callback.  Stage 2A's
publication sequence is the authoritative instance key; payload digests prove
the association.  This offline tool never changes either recorded stream.
"""

import argparse
import csv
import hashlib
import json
import struct
from pathlib import Path

import rosbag


BELIEF_TOPIC = "/kimera/cbs/odom_belief_in"
METADATA_TOPIC = "/glim/cbs/dcreg_edge_health_metadata"


def u32(value):
    return struct.pack(">I", int(value))


def u64(value):
    return struct.pack(">Q", int(value))


def f64(value):
    return struct.pack(">d", float(value))


def text(value):
    encoded = value.encode("utf-8")
    return u32(len(encoded)) + encoded


def digest128(payload):
    return hashlib.sha256(payload).digest()[:16]


def canonical_belief_payload(array_message, metadata_message, ordinal):
    belief = array_message.beliefs[ordinal]
    payload = bytearray(b"GLIM_CBS_BELIEF_PAYLOAD_V1")
    payload += u32(metadata_message.schema_version)
    payload += u32(array_message.header.stamp.secs)
    payload += u32(array_message.header.stamp.nsecs)
    payload += text(array_message.header.frame_id)
    payload += u32(metadata_message.source_agent)
    payload += u32(belief.from_pose_index)
    payload += u32(belief.to_pose_index)
    payload += f64(belief.from_stamp_sec)
    payload += f64(belief.to_stamp_sec)
    payload += b"".join(f64(value) for value in belief.relative_mu)
    payload += b"".join(f64(value) for value in belief.covariance)
    payload += f64(belief.relax_factor)
    payload += u32(ordinal)
    payload += u64(metadata_message.belief_publication_sequence)
    return bytes(payload)


def entry_row(message, entry, payload_ok):
    return {
        "publication_sequence": message.belief_publication_sequence,
        "belief_ordinal": entry.belief_ordinal,
        "from_pose_index": entry.from_pose_index,
        "to_pose_index": entry.to_pose_index,
        "from_stamp_sec": entry.from_stamp_sec,
        "to_stamp_sec": entry.to_stamp_sec,
        "payload_digest_ok": int(payload_ok),
        "association_valid": int(entry.association_valid),
        "association_complete": int(entry.association_complete),
        "expected_sample_count": entry.expected_sample_count,
        "observed_sample_count": entry.observed_sample_count,
        "valid_hessian_sample_count": entry.valid_hessian_sample_count,
        "health_available_sample_count": entry.health_available_sample_count,
        "unavailable_sample_count": entry.unavailable_sample_count,
        "reason_mask": entry.reason_mask,
        "reference_ready": int(entry.reference_ready),
        "reference_consistent": int(entry.reference_consistent),
        "reference_source": entry.reference_source,
        "relative_health_valid": int(entry.relative_health_valid),
        "rotational_health_minimum": entry.rotational_health_minimum,
        "rotational_health_median": entry.rotational_health_median,
        "translational_health_minimum": entry.translational_health_minimum,
        "translational_health_median": entry.translational_health_median,
        "rotational_degraded_count": entry.rotational_degraded_count,
        "rotational_degraded_denominator": entry.rotational_degraded_denominator,
        "translational_degraded_count": entry.translational_degraded_count,
        "translational_degraded_denominator": entry.translational_degraded_denominator,
        "rotational_absolute_degenerate_count":
            entry.rotational_absolute_degenerate_count,
        "rotational_absolute_degenerate_denominator":
            entry.rotational_absolute_degenerate_denominator,
        "translational_absolute_degenerate_count":
            entry.translational_absolute_degenerate_count,
        "translational_absolute_degenerate_denominator":
            entry.translational_absolute_degenerate_denominator,
        "rotational_absolute_degenerate_fraction":
            entry.rotational_absolute_degenerate_fraction,
        "translational_absolute_degenerate_fraction":
            entry.translational_absolute_degenerate_fraction,
        "worst_rotational_condition_ratio": entry.worst_rotational_condition_ratio,
        "worst_translational_condition_ratio": entry.worst_translational_condition_ratio,
        "longest_rotational_degraded_run": entry.longest_rotational_degraded_run,
        "longest_translational_degraded_run": entry.longest_translational_degraded_run,
        "clustered_rotation_fraction": entry.clustered_rotation_fraction,
        "clustered_translation_fraction": entry.clustered_translation_fraction,
        "low_rotation_alignment_fraction": entry.low_rotation_alignment_fraction,
        "low_translation_alignment_fraction": entry.low_translation_alignment_fraction,
        "aggregate_source_point_count_median":
            entry.aggregate_source_point_count.median,
        "aggregate_inlier_fraction_median": entry.aggregate_inlier_fraction.median,
        "aggregate_initial_cost_median": entry.aggregate_initial_cost.median,
        "aggregate_final_cost_median": entry.aggregate_final_cost.median,
    }


def extract(bag_path, csv_path, summary_path):
    beliefs = []
    metadata = []
    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, message, _record_time in bag.read_messages(
                topics=[BELIEF_TOPIC, METADATA_TOPIC]):
            if topic == BELIEF_TOPIC:
                beliefs.append(message)
            else:
                metadata.append(message)

    rows = []
    unsupported_sequence_count = 0
    entry_count_mismatch_count = 0
    payload_digest_mismatch_count = 0
    seen_sequences = set()
    duplicate_sequence_count = 0
    for message in metadata:
        sequence = int(message.belief_publication_sequence)
        if sequence in seen_sequences:
            duplicate_sequence_count += 1
        seen_sequences.add(sequence)
        if sequence < 1 or sequence > len(beliefs):
            unsupported_sequence_count += 1
            continue
        belief_array = beliefs[sequence - 1]
        if len(message.entries) != len(belief_array.beliefs):
            entry_count_mismatch_count += 1
        for entry in message.entries:
            ordinal = int(entry.belief_ordinal)
            if ordinal < 0 or ordinal >= len(belief_array.beliefs):
                entry_count_mismatch_count += 1
                continue
            expected = digest128(canonical_belief_payload(
                belief_array, message, ordinal))
            payload_ok = expected == bytes(entry.belief_payload_digest)
            payload_digest_mismatch_count += not payload_ok
            rows.append(entry_row(message, entry, payload_ok))

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    if rows:
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    summary = {
        "schema_version": 1,
        "association": (
            "metadata belief_publication_sequence indexes legacy belief arrays; "
            "128-bit payload digest is independently recomputed"
        ),
        "belief_array_count": len(beliefs),
        "metadata_array_count": len(metadata),
        "metadata_entry_count": len(rows),
        "unique_publication_sequence_count": len(seen_sequences),
        "unsupported_sequence_count": unsupported_sequence_count,
        "duplicate_sequence_count": duplicate_sequence_count,
        "entry_count_mismatch_count": entry_count_mismatch_count,
        "payload_digest_mismatch_count": payload_digest_mismatch_count,
        "all_payload_digests_valid": payload_digest_mismatch_count == 0,
    }
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n",
                            encoding="utf-8")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    args = parser.parse_args()
    result = extract(args.bag, args.csv, args.summary)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["all_payload_digests_valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
