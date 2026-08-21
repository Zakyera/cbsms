#!/usr/bin/env python3
"""Stream timestamped GEODE compressed images from a ROS1 bag.

The binary stream is intentionally small and dependency-light so a ROS
environment can preserve the original JPEG payload while a separate Rerun
SDK environment performs the visualization logging.
"""

import argparse
import json
import struct
import sys

import rosbag
import rospy


MAGIC = b"GEODE_RERUN_IMAGE_STREAM_V1\n"
RECORD = struct.Struct("<BQI")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--left-topic", default="/left_camera/image/compressed")
    parser.add_argument("--right-topic", default="/right_camera/image/compressed")
    parser.add_argument("--left-max-hz", type=float, default=5.0)
    parser.add_argument("--right-max-hz", type=float, default=2.0)
    parser.add_argument("--start-stamp-sec", type=float)
    parser.add_argument("--duration-sec", type=float)
    args = parser.parse_args()
    if args.duration_sec is not None and args.duration_sec <= 0.0:
        parser.error("--duration-sec must be positive")

    topics = [args.left_topic]
    if args.right_topic:
        topics.append(args.right_topic)
    channel_for_topic = {args.left_topic: 0}
    if args.right_topic:
        channel_for_topic[args.right_topic] = 1
    max_hz = {0: args.left_max_hz, 1: args.right_max_hz}
    last_stamp = {0: float("-inf"), 1: float("-inf")}
    counts = {0: 0, 1: 0}
    bytes_written = {0: 0, 1: 0}

    output = sys.stdout.buffer
    output.write(MAGIC)
    with rosbag.Bag(args.bag, "r") as bag:
        start_time = (rospy.Time.from_sec(args.start_stamp_sec)
                      if args.start_stamp_sec is not None else None)
        end_time = (rospy.Time.from_sec(args.start_stamp_sec + args.duration_sec)
                    if args.start_stamp_sec is not None and
                    args.duration_sec is not None else None)
        for topic, message, bag_stamp in bag.read_messages(
                topics=topics, start_time=start_time, end_time=end_time):
            channel = channel_for_topic[topic]
            stamp = message.header.stamp
            stamp_sec = stamp.to_sec()
            if stamp_sec <= 0.0:
                stamp = bag_stamp
                stamp_sec = stamp.to_sec()
            rate = max_hz[channel]
            minimum_interval = 1.0 / rate if rate > 0.0 else 0.0
            if stamp_sec - last_stamp[channel] < minimum_interval - 1.0e-9:
                continue
            payload = bytes(message.data)
            stamp_ns = int(stamp.secs) * 1_000_000_000 + int(stamp.nsecs)
            output.write(RECORD.pack(channel, stamp_ns, len(payload)))
            output.write(payload)
            last_stamp[channel] = stamp_sec
            counts[channel] += 1
            bytes_written[channel] += len(payload)
    output.flush()
    print(json.dumps({
        "left_count": counts[0],
        "right_count": counts[1],
        "left_payload_bytes": bytes_written[0],
        "right_payload_bytes": bytes_written[1],
        "start_stamp_sec": args.start_stamp_sec,
        "duration_sec": args.duration_sec,
    }, sort_keys=True), file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
