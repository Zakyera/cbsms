#!/usr/bin/env python3
"""Audit GEODE CustomMsg->PointCloud2 and copied IMU semantics."""

import argparse
import hashlib
import io
import json
import math
import struct
from collections import Counter
from pathlib import Path

import rosbag


PACKER = struct.Struct("<fffIBB")


def serialize(message):
    stream = io.BytesIO()
    message.serialize(stream)
    return stream.getvalue()


def source_point_bytes(message):
    count = min(int(message.point_num), len(message.points)) if message.point_num else len(message.points)
    data = bytearray(count * PACKER.size)
    line_counts = Counter()
    for index, point in enumerate(message.points[:count]):
        PACKER.pack_into(data, index * PACKER.size, point.x, point.y, point.z,
                         point.offset_time, point.reflectivity, point.line)
        line_counts[int(point.line)] += 1
    return bytes(data), line_counts


def audit(source_path, converted_path, lidar_topic, imu_topic, points_topic,
          allowed_terminal_uncovered_scans=0):
    source_clouds = []
    source_imus = []
    with rosbag.Bag(str(source_path), "r") as bag:
        for topic, message, record_time in bag.read_messages(topics=[lidar_topic, imu_topic]):
            if topic == lidar_topic:
                source_clouds.append((message, record_time))
            else:
                source_imus.append((message, record_time))
    converted_clouds = []
    converted_imus = []
    with rosbag.Bag(str(converted_path), "r") as bag:
        for topic, message, record_time in bag.read_messages(topics=[points_topic, imu_topic]):
            if topic == points_topic:
                converted_clouds.append((message, record_time))
            else:
                converted_imus.append((message, record_time))
    if len(source_imus) != len(converted_imus):
        raise ValueError("IMU message counts differ")
    mismatches = Counter()
    scheduling_differences = Counter()
    source_lines = Counter()
    offset_min, offset_max = None, None
    point_count = 0
    ordered_hash = hashlib.sha256()
    source_by_stamp = {
        (message.header.stamp.secs, message.header.stamp.nsecs):
            (message, record_time)
        for message, record_time in source_clouds
    }
    converted_stamps = []
    for converted, converted_record in converted_clouds:
        stamp = (converted.header.stamp.secs, converted.header.stamp.nsecs)
        converted_stamps.append(stamp)
        if stamp not in source_by_stamp:
            mismatches["cloud_missing_from_source"] += 1
            continue
        source, source_record = source_by_stamp[stamp]
        packed, lines = source_point_bytes(source)
        source_lines.update(lines)
        point_count += len(source.points)
        if source.header.stamp != converted.header.stamp:
            mismatches["cloud_header_stamp"] += 1
        if source_record != converted_record:
            scheduling_differences["cloud_record_stamp"] += 1
        if packed != bytes(converted.data):
            mismatches["ordered_point_records"] += 1
        if converted.point_step != PACKER.size:
            mismatches["point_step"] += 1
        ordered_hash.update(bytes(converted.data))
        if source.points:
            local_min = min(int(point.offset_time) for point in source.points)
            local_max = max(int(point.offset_time) for point in source.points)
            offset_min = local_min if offset_min is None else min(offset_min, local_min)
            offset_max = local_max if offset_max is None else max(offset_max, local_max)
    imu_accel_norms = []
    imu_gyro_norms = []
    imu_hash = hashlib.sha256()
    for (source, source_record), (converted, converted_record) in zip(source_imus, converted_imus):
        source_bytes, converted_bytes = serialize(source), serialize(converted)
        if source_bytes != converted_bytes:
            mismatches["imu_serialized"] += 1
        if source_record != converted_record:
            scheduling_differences["imu_record_stamp"] += 1
        imu_hash.update(converted_bytes)
        acceleration = source.linear_acceleration
        angular = source.angular_velocity
        imu_accel_norms.append(math.sqrt(acceleration.x**2 + acceleration.y**2 + acceleration.z**2))
        imu_gyro_norms.append(math.sqrt(angular.x**2 + angular.y**2 + angular.z**2))
    source_stamps = [
        (message.header.stamp.secs, message.header.stamp.nsecs)
        for message, _record_time in source_clouds
    ]
    converted_stamp_set = set(converted_stamps)
    missing_stamps = [stamp for stamp in source_stamps
                      if stamp not in converted_stamp_set]
    missing_are_terminal = (
        not missing_stamps or
        missing_stamps == source_stamps[len(source_stamps) - len(missing_stamps):]
    )
    exclusion_valid = (
        len(missing_stamps) == allowed_terminal_uncovered_scans
        and missing_are_terminal
    )
    semantically_identical = not mismatches and exclusion_valid
    return {
        "schema_version": 1,
        "source_bag": str(Path(source_path).resolve()),
        "converted_bag": str(Path(converted_path).resolve()),
        "source_cloud_count": len(source_clouds),
        "converted_cloud_count": len(converted_clouds),
        "imu_count": len(source_imus),
        "excluded_terminal_uncovered_scan_count": len(missing_stamps),
        "excluded_terminal_uncovered_scan_stamps": [
            int(sec) * 1000000000 + int(nsec) for sec, nsec in missing_stamps
        ],
        "excluded_scans_are_terminal": missing_are_terminal,
        "allowed_terminal_uncovered_scan_count": allowed_terminal_uncovered_scans,
        "point_count": point_count,
        "point_fields": ["x", "y", "z", "t", "intensity", "ring"],
        "point_step": PACKER.size,
        "offset_time_min_ns": offset_min, "offset_time_max_ns": offset_max,
        "line_id_distribution": {str(key): value for key, value in sorted(source_lines.items())},
        "ordered_point_content_sha256": ordered_hash.hexdigest(),
        "imu_serialized_content_sha256": imu_hash.hexdigest(),
        "imu_acceleration_norm_min": min(imu_accel_norms),
        "imu_acceleration_norm_max": max(imu_accel_norms),
        "imu_angular_rate_norm_min": min(imu_gyro_norms),
        "imu_angular_rate_norm_max": max(imu_gyro_norms),
        "mismatch_counts": dict(mismatches),
        "scheduling_record_time_difference_counts": dict(scheduling_differences),
        "semantically_identical_for_processable_events": semantically_identical,
        "rosbag_record_times_are_test_dispatch_schedule_not_measurement_time": True,
        "tag_policy": "not represented; frozen GLIM converter never consumes Livox tag",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source_bag", type=Path)
    parser.add_argument("converted_bag", type=Path)
    parser.add_argument("--lidar-topic", default="/livox/lidar")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--points-topic", default="/geode/gamma/points")
    parser.add_argument("--allow-terminal-uncovered-scans", type=int, default=0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.source_bag, args.converted_bag, args.lidar_topic,
                   args.imu_topic, args.points_topic,
                   args.allow_terminal_uncovered_scans)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["semantically_identical_for_processable_events"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
