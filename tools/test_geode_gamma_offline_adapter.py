#!/usr/bin/env python3

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

import rosbag
import rospy
from livox_ros_driver.msg import CustomMsg, CustomPoint
from sensor_msgs.msg import Imu


SCRIPT = Path(__file__).resolve().parent / "geode_gamma_offline_adapter.py"
SPEC = importlib.util.spec_from_file_location("geode_gamma_offline_adapter", SCRIPT)
ADAPTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ADAPTER)


def imu(sequence, stamp_ns):
    message = Imu()
    message.header.seq = sequence
    message.header.stamp = rospy.Time(stamp_ns // 1_000_000_000,
                                      stamp_ns % 1_000_000_000)
    message.linear_acceleration.z = 9.81
    return message


def scan(sequence, stamp_ns, maximum_offset_ns=99_000_000):
    message = CustomMsg()
    message.header.seq = sequence
    message.header.stamp = rospy.Time(stamp_ns // 1_000_000_000,
                                      stamp_ns % 1_000_000_000)
    message.timebase = stamp_ns
    for index, offset in enumerate((0, maximum_offset_ns)):
        point = CustomPoint()
        point.offset_time = offset
        point.x, point.y, point.z = index + 1.0, -index, index * 0.5
        point.reflectivity = 20 + index
        point.line = index
        message.points.append(point)
    message.point_num = len(message.points)
    return message


class GeodeGammaOfflineAdapterTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.source = self.root / "source.bag"
        with rosbag.Bag(str(self.source), "w") as bag:
            bag.write("/imu/data", imu(0, 990_000_000), rospy.Time(0, 990_000_000))
            bag.write("/livox/lidar", scan(1, 1_000_000_000), rospy.Time(1, 0))
            for sequence, stamp_ns in enumerate(range(1_010_000_000,
                                                       1_101_000_000,
                                                       10_000_000), start=2):
                bag.write("/imu/data", imu(sequence, stamp_ns),
                          rospy.Time(stamp_ns // 1_000_000_000,
                                     stamp_ns % 1_000_000_000))
            bag.write("/livox/lidar", scan(20, 1_100_000_000), rospy.Time(1, 100_000_000))
            for sequence, stamp_ns in enumerate(range(1_110_000_000,
                                                       1_201_000_000,
                                                       10_000_000), start=21):
                bag.write("/imu/data", imu(sequence, stamp_ns),
                          rospy.Time(stamp_ns // 1_000_000_000,
                                     stamp_ns % 1_000_000_000))

    def tearDown(self):
        self.temporary.cleanup()

    def convert(self, suffix):
        output = self.root / f"output_{suffix}.bag"
        manifest_path = self.root / f"manifest_{suffix}.json"
        messages_path = self.root / f"messages_{suffix}.jsonl"
        manifest = ADAPTER.convert(
            self.source, output, manifest_path, messages_path,
            "/livox/lidar", "/imu/data", "/geode/gamma/points", "gamma")
        messages = [json.loads(line) for line in messages_path.read_text().splitlines()]
        return output, manifest, messages

    def test_repeated_conversion_is_byte_and_content_deterministic(self):
        _out_a, manifest_a, messages_a = self.convert("a")
        _out_b, manifest_b, messages_b = self.convert("b")
        self.assertEqual(manifest_a["authoritative_content"],
                         manifest_b["authoritative_content"])
        self.assertEqual(manifest_a["output"]["bag_sha256"],
                         manifest_b["output"]["bag_sha256"])
        self.assertEqual(messages_a, messages_b)

    def test_deskew_barrier_precedes_the_next_scan(self):
        _output, manifest, messages = self.convert("barrier")
        point_indices = [index for index, row in enumerate(messages)
                         if row["topic"] == "/geode/gamma/points"]
        self.assertEqual(len(point_indices), 2)
        first_coverage = messages[point_indices[1] - 1]
        self.assertEqual(first_coverage["topic"], "/imu/data")
        self.assertGreaterEqual(first_coverage["header_stamp_ns"], 1_099_000_000)
        second = messages[point_indices[1]]
        self.assertEqual(second["header_stamp_ns"], 1_100_000_000)
        self.assertGreater(second["record_stamp_ns"],
                           first_coverage["record_stamp_ns"])
        self.assertEqual(manifest["authoritative_content"]
                         ["excluded_uncovered_source_scan_count"], 0)

    def test_terminal_scan_without_imu_coverage_is_explicitly_excluded(self):
        with rosbag.Bag(str(self.source), "a") as bag:
            bag.write("/livox/lidar", scan(99, 1_300_000_000),
                      rospy.Time(1, 300_000_000))
        _output, manifest, messages = self.convert("uncovered")
        self.assertEqual(manifest["authoritative_content"]
                         ["excluded_uncovered_source_scan_count"], 1)
        self.assertNotIn(1_300_000_000,
                         [row["header_stamp_ns"] for row in messages
                          if row["topic"] == "/geode/gamma/points"])


if __name__ == "__main__":
    unittest.main()
