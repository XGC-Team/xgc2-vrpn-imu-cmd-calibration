#!/usr/bin/env python3

import copy
import unittest
from pathlib import Path

import yaml

from vrpn_imu_cmd_calibration.profile import (
    estimate_nominal_duration,
    validate_profile,
)


class ProfileTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        profile_path = Path(__file__).resolve().parents[1] / "config" / "default.yaml"
        cls.profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))

    def test_default_profile_is_valid_and_short(self):
        validate_profile(self.profile)
        duration = estimate_nominal_duration(self.profile)
        self.assertGreaterEqual(duration, 60.0)
        self.assertLessEqual(duration, 180.0)

    def test_rejects_relative_motion_topic(self):
        profile = copy.deepcopy(self.profile)
        profile["topics"]["cmd"] = "cmd_vel"
        with self.assertRaisesRegex(ValueError, "absolute ROS name"):
            validate_profile(profile)

    def test_rejects_motion_that_does_not_fit_geofence(self):
        profile = copy.deepcopy(self.profile)
        profile["motion"]["line_distance_m"] = 3.0
        with self.assertRaises(ValueError):
            validate_profile(profile)

    def test_rejects_duration_without_watchdog_margin(self):
        profile = copy.deepcopy(self.profile)
        profile["safety"]["max_run_time_s"] = 100.0
        with self.assertRaisesRegex(ValueError, "safety margin"):
            validate_profile(profile)


if __name__ == "__main__":
    unittest.main()
