"""Validated merging of generated calibration YAML into runtime configs."""

from __future__ import annotations

import copy
from typing import Any, Dict, Mapping


ESTIMATOR_KEYS = (
    "imu_to_vrpn_marker_xyz",
    "imu_to_vrpn_marker_rpy",
    "extrinsic_verified",
    "estimate_extrinsic",
    "imu_noise_std_is_density",
    "accel_noise_std",
    "gyro_noise_std",
    "vrpn_position_noise_std",
    "vrpn_orientation_noise_std",
    "gyro_bias_random_walk_std",
    "accel_bias_random_walk_std",
)

CONTROLLER_KEYS = ("state_source", "state_estimate_topic")


def merge_allowed(
    target: Mapping[str, Any], generated: Mapping[str, Any], allowed_keys
) -> Dict[str, Any]:
    output = copy.deepcopy(dict(target))
    missing = [key for key in allowed_keys if key not in generated]
    if missing:
        raise ValueError("generated YAML is missing keys: %s" % ", ".join(missing))
    for key in allowed_keys:
        output[key] = copy.deepcopy(generated[key])
    return output


def validate_asset(asset: Mapping[str, Any]) -> None:
    if asset.get("schema_version") != "xgc2.vrpn_imu_cmd_calibration/result/v1":
        raise ValueError("unsupported calibration schema_version")
    if asset.get("status") != "PASS":
        raise ValueError("calibration status is not PASS")
    if not asset.get("quality", {}).get("runtime_yaml_ready"):
        raise ValueError("calibration is not marked runtime_yaml_ready")
