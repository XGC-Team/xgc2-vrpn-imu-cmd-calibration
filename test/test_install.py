#!/usr/bin/env python3

import unittest

from vrpn_imu_cmd_calibration.install import ESTIMATOR_KEYS, merge_allowed, validate_asset


class InstallTest(unittest.TestCase):
    def test_merge_changes_only_allowed_keys(self):
        target = {"keep": 7, **{key: "old" for key in ESTIMATOR_KEYS}}
        generated = {key: index for index, key in enumerate(ESTIMATOR_KEYS)}
        merged = merge_allowed(target, generated, ESTIMATOR_KEYS)
        self.assertEqual(merged["keep"], 7)
        for key in ESTIMATOR_KEYS:
            self.assertEqual(merged[key], generated[key])

    def test_install_rejects_failed_asset(self):
        with self.assertRaises(ValueError):
            validate_asset(
                {
                    "schema_version": "xgc2.vrpn_imu_cmd_calibration/result/v1",
                    "status": "FAIL",
                    "quality": {"runtime_yaml_ready": False},
                }
            )


if __name__ == "__main__":
    unittest.main()
