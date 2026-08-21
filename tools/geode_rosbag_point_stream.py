#!/usr/bin/env python3
"""Stream bounded XYZ samples from a deterministic GEODE PointCloud2 bag."""

import argparse
import json
import struct
import sys

import numpy as np
import rosbag
import rospy


MAGIC = b"GEODE_RERUN_POINT_STREAM_V1\n"
RECORD = struct.Struct("<QI")


def xyz_view(message):
    fields = {field.name: field for field in message.fields}
    required = [fields[name] for name in ("x", "y", "z")]
    if any(field.datatype != 7 or field.count != 1 for field in required):
        raise ValueError("x/y/z must be scalar PointField.FLOAT32 values")
    endian = ">" if message.is_bigendian else "<"
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": [endian + "f4", endian + "f4", endian + "f4"],
        "offsets": [field.offset for field in required],
        "itemsize": message.point_step,
    })
    count = int(message.width) * int(message.height)
    records = np.frombuffer(message.data, dtype=dtype, count=count)
    xyz = np.column_stack((records["x"], records["y"], records["z"]))
    return np.ascontiguousarray(xyz[np.isfinite(xyz).all(axis=1)], dtype="<f4")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--topic", default="/geode/gamma/points")
    parser.add_argument("--max-hz", type=float, default=2.0)
    parser.add_argument("--maximum-points", type=int, default=12000)
    parser.add_argument("--start-stamp-sec", type=float)
    parser.add_argument("--duration-sec", type=float)
    args = parser.parse_args()
    if args.duration_sec is not None and args.duration_sec <= 0.0:
        parser.error("--duration-sec must be positive")

    output = sys.stdout.buffer
    output.write(MAGIC)
    last_stamp = float("-inf")
    count = 0
    point_count = 0
    minimum_interval = 1.0 / args.max_hz if args.max_hz > 0.0 else 0.0
    with rosbag.Bag(args.bag, "r") as bag:
        start_time = (rospy.Time.from_sec(args.start_stamp_sec)
                      if args.start_stamp_sec is not None else None)
        end_time = (rospy.Time.from_sec(args.start_stamp_sec + args.duration_sec)
                    if args.start_stamp_sec is not None and
                    args.duration_sec is not None else None)
        for _topic, message, bag_stamp in bag.read_messages(
                topics=[args.topic], start_time=start_time, end_time=end_time):
            stamp = message.header.stamp
            stamp_sec = stamp.to_sec()
            if stamp_sec <= 0.0:
                stamp = bag_stamp
                stamp_sec = stamp.to_sec()
            if stamp_sec - last_stamp < minimum_interval - 1.0e-9:
                continue
            xyz = xyz_view(message)
            if len(xyz) > args.maximum_points:
                indices = np.linspace(
                    0, len(xyz) - 1, args.maximum_points, dtype=np.int64)
                xyz = np.ascontiguousarray(xyz[indices], dtype="<f4")
            stamp_ns = int(stamp.secs) * 1_000_000_000 + int(stamp.nsecs)
            output.write(RECORD.pack(stamp_ns, len(xyz)))
            output.write(xyz.tobytes(order="C"))
            last_stamp = stamp_sec
            count += 1
            point_count += len(xyz)
    output.flush()
    print(json.dumps({
        "cloud_count": count,
        "logged_point_count": point_count,
        "maximum_points_per_cloud": args.maximum_points,
        "start_stamp_sec": args.start_stamp_sec,
        "duration_sec": args.duration_sec,
    }, sort_keys=True), file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
