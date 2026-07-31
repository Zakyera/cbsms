#!/usr/bin/env python3

import unittest

from cbsms_experiment import expected_replay_wall_duration_sec


class ExperimentRunnerTimingTest(unittest.TestCase):
    def test_slow_bag_rate_expands_wall_duration(self) -> None:
        self.assertAlmostEqual(
            expected_replay_wall_duration_sec(60.0, {"bag_rate": "0.25"}),
            240.0,
        )

    def test_fast_or_invalid_rate_never_shortens_sensor_duration(self) -> None:
        self.assertAlmostEqual(
            expected_replay_wall_duration_sec(60.0, {"bag_rate": "2.0"}),
            60.0,
        )
        self.assertAlmostEqual(
            expected_replay_wall_duration_sec(60.0, {"bag_rate": "invalid"}),
            60.0,
        )


if __name__ == "__main__":
    unittest.main()
