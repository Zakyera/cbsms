#!/usr/bin/env python3
"""Deterministic GEODE Gamma adapter with an explicit deskew barrier.

Message headers and payloads remain unchanged.  Only rosbag record times are
assigned to encode a deterministic single-thread dispatch order: emit a scan,
then emit IMU records through the first sample at or after that scan's maximum
point time, and only then emit the next scan.  This test-only scheduling policy
matches ``glim_rosbag``'s deterministic replay contract and does not replace
the production ROS pipeline.
"""

import argparse
import hashlib
import json
import sys
from collections import defaultdict
from pathlib import Path

import rosbag


M3DGR_SCRIPTS = Path(__file__).resolve().parents[2] / "m3dgr_tools" / "scripts"
sys.path.insert(0, str(M3DGR_SCRIPTS))
from livox_to_pointcloud2 import convert_custom_msg, custom_msg_type  # noqa: E402
from offline_convert_livox_bag import (  # noqa: E402
    CONVERSION_VERSION, empty_topic_summary, point_schema, point_semantics,
    serialized_bytes, sha256_file, stable_json_hash, time_nsec,
    update_topic_range,
)


ADAPTER_VERSION = "geode_gamma_deskew_barrier_v2"


def original_topic_orders(source, topics):
    orders = defaultdict(list)
    with rosbag.Bag(str(source), "r") as bag:
        for global_index, (topic, _message, _record_time) in enumerate(
                bag.read_messages(topics=topics)):
            orders[topic].append(global_index)
    return orders


def convert(source_bag, output_bag, manifest_path, message_manifest_path,
            lidar_topic, imu_topic, points_topic, frame_id, source_sha256=None):
    source_bag = Path(source_bag).resolve()
    output_bag = Path(output_bag).resolve()
    manifest_path = Path(manifest_path).resolve()
    message_manifest_path = Path(message_manifest_path).resolve()
    output_bag.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    orders = original_topic_orders(source_bag, [lidar_topic, imu_topic])
    configuration = {
        "adapter_version": ADAPTER_VERSION,
        "base_conversion_version": CONVERSION_VERSION,
        "driver": "livox_ros_driver", "lidar_topic": lidar_topic,
        "imu_topic": imu_topic, "points_topic": points_topic,
        "frame_id_override": frame_id,
        "ordering": (
            "scan_then_imu_through_first_sample_at_or_after_scan_max_point_time;"
            "source_record_order_breaks_equal_measurement_timestamps"
        ),
        "output_record_time": (
            "max(message_header_timestamp,previous_output_record_time_plus_1ns)"
        ),
    }
    config_hash = stable_json_hash(configuration)
    summaries = {
        points_topic: empty_topic_summary("sensor_msgs/PointCloud2"),
        imu_topic: empty_topic_summary("sensor_msgs/Imu"),
    }
    digests = {topic: hashlib.sha256() for topic in summaries}
    overall = hashlib.sha256()
    point_digest = hashlib.sha256()
    source_content = hashlib.sha256()
    uncovered_scan_count = 0
    uncovered_scan_end_ns = []
    source_handles = [rosbag.Bag(str(source_bag), "r"),
                      rosbag.Bag(str(source_bag), "r")]
    lidar_iterator = source_handles[0].read_messages(topics=[lidar_topic])
    imu_iterator = source_handles[1].read_messages(topics=[imu_topic])
    topic_local_indices = {lidar_topic: 0, imu_topic: 0}

    def next_source(iterator, topic):
        try:
            actual_topic, message, record_time = next(iterator)
        except StopIteration:
            return None
        if actual_topic != topic:
            raise RuntimeError("unexpected source topic {}".format(actual_topic))
        local_index = topic_local_indices[topic]
        topic_local_indices[topic] += 1
        return topic, message, record_time, orders[topic][local_index]

    pending_imu = next_source(imu_iterator, imu_topic)
    previous_output_record_ns = -1
    try:
        with rosbag.Bag(str(output_bag), "w", compression=rosbag.Compression.NONE) as destination, \
                message_manifest_path.open("w", encoding="utf-8") as message_manifest:
            event_index = 0

            def emit(source_event):
                nonlocal event_index, previous_output_record_ns
                topic, message, source_record_time, global_index = source_event
                source_payload = serialized_bytes(message)
                source_record = {
                    "source_filtered_record_index": global_index, "topic": topic,
                    "source_record_stamp_ns": time_nsec(source_record_time),
                    "header_stamp_ns": time_nsec(message.header.stamp),
                    "source_sequence": int(message.header.seq),
                    "serialized_sha256": hashlib.sha256(source_payload).hexdigest(),
                }
                source_content.update((json.dumps(source_record, sort_keys=True) + "\n").encode())
                if topic == lidar_topic:
                    output_topic = points_topic
                    output_message = convert_custom_msg(message, frame_id)
                    semantics = point_semantics(output_message)
                else:
                    output_topic = imu_topic
                    output_message = message
                    semantics = None
                header_ns = time_nsec(output_message.header.stamp)
                output_record_ns = max(header_ns, previous_output_record_ns + 1)
                output_record_time = source_record_time.__class__(
                    output_record_ns // 1000000000,
                    output_record_ns % 1000000000)
                previous_output_record_ns = output_record_ns
                destination.write(output_topic, output_message, output_record_time)
                payload = serialized_bytes(output_message)
                record = {
                    "event_index": event_index,
                    "source_filtered_record_index": global_index,
                    "topic": output_topic,
                    "source_record_stamp_ns": time_nsec(source_record_time),
                    "record_stamp_ns": time_nsec(output_record_time),
                    "header_stamp_ns": time_nsec(output_message.header.stamp),
                    "source_sequence": int(output_message.header.seq),
                    "serialized_sha256": hashlib.sha256(payload).hexdigest(),
                }
                if semantics is not None:
                    record["point_semantics"] = semantics
                    point_digest.update((json.dumps(semantics, sort_keys=True) + "\n").encode())
                encoded = (json.dumps(record, sort_keys=True) + "\n").encode()
                message_manifest.write(encoded.decode())
                digests[output_topic].update(encoded)
                overall.update(encoded)
                update_topic_range(summaries[output_topic], record["header_stamp_ns"],
                                   record["record_stamp_ns"])
                event_index += 1
                return semantics

            first_scan = True
            for scan_event in iter(lambda: next_source(lidar_iterator, lidar_topic), None):
                scan_stamp_ns = time_nsec(scan_event[1].header.stamp)
                if first_scan:
                    while (pending_imu is not None and
                           time_nsec(pending_imu[1].header.stamp) <= scan_stamp_ns):
                        emit(pending_imu)
                        pending_imu = next_source(imu_iterator, imu_topic)
                    first_scan = False
                maximum_offset_ns = max(
                    (int(point.offset_time) for point in scan_event[1].points),
                    default=0)
                scan_end_ns = scan_stamp_ns + maximum_offset_ns
                coverage_obtained = False
                coverage_events = []
                while pending_imu is not None:
                    imu_stamp_ns = time_nsec(pending_imu[1].header.stamp)
                    coverage_events.append(pending_imu)
                    pending_imu = next_source(imu_iterator, imu_topic)
                    if imu_stamp_ns >= scan_end_ns:
                        coverage_obtained = True
                        break
                if not coverage_obtained:
                    # A final source cloud without IMU through its maximum
                    # point time cannot be deskewed by GLIM and is therefore
                    # not a processable test event.  Preserve it through the
                    # source-bag hash and report the exclusion explicitly.
                    uncovered_scan_count += 1
                    uncovered_scan_end_ns.append(scan_end_ns)
                    for imu_event in coverage_events:
                        emit(imu_event)
                    continue
                emit(scan_event)
                for imu_event in coverage_events:
                    emit(imu_event)

            while pending_imu is not None:
                emit(pending_imu)
                pending_imu = next_source(imu_iterator, imu_topic)
    finally:
        for handle in source_handles:
            handle.close()
    for topic, digest in digests.items():
        summaries[topic]["serialized_sequence_sha256"] = digest.hexdigest()
    authoritative = {
        "configuration_sha256": config_hash, "event_count": event_index,
        "event_sequence_sha256": overall.hexdigest(),
        "point_semantics_sequence_sha256": point_digest.hexdigest(),
        "point_schema": point_schema(), "point_step": 18, "topics": summaries,
        "excluded_uncovered_source_scan_count": uncovered_scan_count,
        "excluded_uncovered_source_scan_end_ns": uncovered_scan_end_ns,
        "source_fields_not_represented": [{"name": "tag", "reason":
            "not present in frozen GLIM schema and not consumed by GLIM"}],
    }
    manifest = {
        "schema_version": 1, "conversion": configuration,
        "configuration_sha256": config_hash,
        "source": {"path": str(source_bag),
                   "bag_sha256": source_sha256 or sha256_file(source_bag),
                   "selected_content_sha256": source_content.hexdigest()},
        "output": {"path": str(output_bag), "bag_sha256": sha256_file(output_bag),
                   "message_manifest_path": str(message_manifest_path),
                   "message_manifest_sha256": sha256_file(message_manifest_path)},
        "authoritative_content": authoritative,
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n",
                             encoding="utf-8")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source_bag", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--message-manifest", type=Path, required=True)
    parser.add_argument("--lidar-topic", default="/livox/lidar")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--points-topic", default="/geode/gamma/points")
    parser.add_argument("--frame-id", default="geode_gamma_livox")
    parser.add_argument("--source-sha256")
    args = parser.parse_args()
    custom_msg_type("livox_ros_driver")
    result = convert(args.source_bag, args.output_bag, args.manifest,
                     args.message_manifest, args.lidar_topic, args.imu_topic,
                     args.points_topic, args.frame_id, args.source_sha256)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
