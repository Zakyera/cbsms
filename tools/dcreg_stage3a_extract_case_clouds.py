#!/usr/bin/env python3
"""Extract bounded point samples for preregistered Stage 3A case-study edges."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import rosbag


POINT_DTYPE = np.dtype({
    "names": ["x", "y", "z", "t", "intensity", "ring"],
    "formats": ["<f4", "<f4", "<f4", "<u4", "u1", "u1"],
    "offsets": [0, 4, 8, 12, 16, 17],
    "itemsize": 18,
})


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("cases", type=Path)
    parser.add_argument("--topic", default="/geode/gamma/points")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--maximum-points", type=int, default=4000)
    arguments = parser.parse_args()
    if arguments.maximum_points < 1:
        raise SystemExit("maximum-points must be positive")

    with arguments.cases.open(newline="") as stream:
        cases = list(csv.DictReader(stream))
    desired = []
    for row in cases:
        edge = f"{row['from_pose_index']}->{row['to_pose_index']}"
        desired.extend([
            {"edge": edge, "endpoint": "start", "stamp_ns": int(round(float(row["from_stamp_sec"]) * 1.0e9))},
            {"edge": edge, "endpoint": "end", "stamp_ns": int(round(float(row["to_stamp_sec"]) * 1.0e9))},
        ])
    selected = [None] * len(desired)
    selected_error = [None] * len(desired)
    with rosbag.Bag(str(arguments.bag), "r") as bag:
        for _, message, _ in bag.read_messages(topics=[arguments.topic]):
            stamp_ns = message.header.stamp.to_nsec()
            for index, target in enumerate(desired):
                error = abs(stamp_ns - target["stamp_ns"])
                if selected_error[index] is None or error < selected_error[index]:
                    selected[index] = message
                    selected_error[index] = error

    arrays = {}
    manifest_entries = []
    for index, (target, message, error) in enumerate(zip(desired, selected, selected_error)):
        if message is None or error is None:
            raise RuntimeError(f"no PointCloud2 match for {target}")
        if message.point_step != 18:
            raise RuntimeError(f"unexpected point_step={message.point_step}")
        records = np.frombuffer(bytes(message.data), dtype=POINT_DTYPE)
        sample_indices = np.linspace(0, len(records) - 1, min(len(records), arguments.maximum_points), dtype=np.int64)
        points = np.column_stack((records["x"][sample_indices], records["y"][sample_indices], records["z"][sample_indices])).astype(np.float32, copy=False)
        key = f"cloud_{index:02d}"
        arrays[key] = points
        centered = points.astype(float) - np.mean(points.astype(float), axis=0)
        covariance_eigenvalues = np.linalg.eigvalsh(centered.T @ centered / max(1, len(points) - 1))
        manifest_entries.append({
            **target,
            "array_key": key,
            "matched_stamp_ns": message.header.stamp.to_nsec(),
            "timestamp_error_ns": int(error),
            "source_point_count": int(len(records)),
            "sampled_point_count": int(len(points)),
            "bounding_box_min": np.min(points, axis=0).astype(float).tolist(),
            "bounding_box_max": np.max(points, axis=0).astype(float).tolist(),
            "centroid": np.mean(points, axis=0).astype(float).tolist(),
            "point_position_covariance_eigenvalues": covariance_eigenvalues.astype(float).tolist(),
        })
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(arguments.output, **arrays)
    manifest = {
        "schema_version": 1,
        "bag": str(arguments.bag.resolve()),
        "bag_sha256": file_sha256(arguments.bag),
        "topic": arguments.topic,
        "case_table": str(arguments.cases.resolve()),
        "case_table_sha256": file_sha256(arguments.cases),
        "maximum_points_per_cloud": arguments.maximum_points,
        "entries": manifest_entries,
    }
    manifest_path = arguments.output.with_suffix(".manifest.json")
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(json.dumps({
        "output": str(arguments.output),
        "output_sha256": file_sha256(arguments.output),
        "manifest": str(manifest_path),
        "manifest_sha256": file_sha256(manifest_path),
        "entry_count": len(manifest_entries),
        "maximum_timestamp_error_ns": max(selected_error),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
