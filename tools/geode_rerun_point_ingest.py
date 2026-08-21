#!/usr/bin/env python3
"""Ingest bounded GEODE XYZ frames into a standalone Rerun stream."""

import argparse
import json
import struct
import sys

import numpy as np
import rerun as rr


MAGIC = b"GEODE_RERUN_POINT_STREAM_V1\n"
RECORD = struct.Struct("<QI")


def read_exact(stream, size):
    value = stream.read(size)
    if len(value) != size:
        raise EOFError(f"expected {size} bytes, received {len(value)}")
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--application-id", default="cbsms")
    parser.add_argument("--recording-id", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    source = sys.stdin.buffer
    if read_exact(source, len(MAGIC)) != MAGIC:
        raise ValueError("unsupported point stream")
    rr.init(args.application_id, recording_id=args.recording_id)
    rr.save(args.output)
    rr.log("bag/lidar", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("bag/lidar/documentation", rr.TextDocument(
        "Raw deterministic GEODE Gamma PointCloud2 input in its LiDAR frame; "
        "2 Hz visualization sampling with at most 12,000 ordered points per "
        "scan. This is not an optimized or world-frame cloud."), static=True)
    cloud_count = 0
    point_count = 0
    while True:
        header = source.read(RECORD.size)
        if not header:
            break
        if len(header) != RECORD.size:
            raise EOFError("truncated point record header")
        stamp_ns, count = RECORD.unpack(header)
        payload = read_exact(source, count * 3 * 4)
        points = np.frombuffer(payload, dtype="<f4").reshape(count, 3)
        rr.set_time("time", timestamp=stamp_ns / 1.0e9)
        rr.log("bag/lidar/current_scan", rr.Points3D(
            points, colors=[90, 180, 255, 170], radii=0.035))
        cloud_count += 1
        point_count += count
    rr.disconnect()
    print(json.dumps({
        "cloud_count": cloud_count,
        "logged_point_count": point_count,
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
