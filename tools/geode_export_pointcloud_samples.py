#!/usr/bin/env python3
"""Export bounded deterministic PointCloud2 samples for offline Rerun."""

import argparse
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


def export(bag_path, topic, output, stride, maximum_points):
    clouds = []
    stamps = []
    source_indices = []
    selected_hash = hashlib.sha256()
    total = 0
    with rosbag.Bag(str(bag_path), "r") as bag:
        for index, (_topic, message, _record_time) in enumerate(
                bag.read_messages(topics=[topic])):
            total += 1
            if index % stride:
                continue
            if message.point_step != 18:
                raise ValueError(f"unsupported point_step {message.point_step}; expected 18")
            records = np.frombuffer(bytes(message.data), dtype=POINT_DTYPE)
            if maximum_points > 0 and len(records) > maximum_points:
                sample_indices = np.linspace(
                    0, len(records) - 1, maximum_points, dtype=np.int64)
                records = records[sample_indices]
            xyz = np.column_stack((records["x"], records["y"], records["z"])).astype(
                np.float32, copy=False)
            clouds.append(xyz)
            stamps.append(message.header.stamp.to_nsec())
            source_indices.append(index)
            selected_hash.update(message.header.stamp.to_nsec().to_bytes(8, "big"))
            selected_hash.update(xyz.tobytes(order="C"))
    offsets = [0]
    for cloud in clouds:
        offsets.append(offsets[-1] + len(cloud))
    points = np.concatenate(clouds) if clouds else np.empty((0, 3), np.float32)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output, points=points, offsets=np.asarray(offsets, np.int64),
        stamp_ns=np.asarray(stamps, np.int64),
        source_scan_index=np.asarray(source_indices, np.int64))
    manifest = {
        "schema_version": 1, "bag": str(Path(bag_path).resolve()), "topic": topic,
        "source_scan_count": total, "selected_scan_count": len(clouds),
        "stride": stride, "maximum_points_per_scan": maximum_points,
        "selected_point_count": int(len(points)),
        "selected_content_sha256": selected_hash.hexdigest(),
        "npz_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
    }
    output.with_suffix(".manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--topic", default="/geode/gamma/points")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stride", type=int, default=10)
    parser.add_argument("--maximum-points", type=int, default=12000)
    args = parser.parse_args()
    if args.stride < 1 or args.maximum_points < 1:
        raise SystemExit("stride and maximum-points must be >= 1")
    print(json.dumps(export(args.bag, args.topic, args.output, args.stride,
                            args.maximum_points), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
