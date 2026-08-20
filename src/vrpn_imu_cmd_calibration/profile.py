"""Validation and duration estimates for cmd_vel calibration profiles."""

from __future__ import annotations

import math
from typing import Any, Mapping


SCHEMA_VERSION = "xgc2.vrpn_imu_cmd_calibration/profile/v1"


def _mapping(parent: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = parent.get(key)
    if not isinstance(value, Mapping):
        raise ValueError("profile.%s must be a mapping" % key)
    return value


def _number(
    parent: Mapping[str, Any], key: str, minimum: float, maximum: float
) -> float:
    value = parent.get(key)
    if isinstance(value, bool):
        raise ValueError("%s must be numeric" % key)
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise ValueError("%s must be numeric" % key)
    if not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError("%s must be in [%s, %s]" % (key, minimum, maximum))
    return value


def estimate_nominal_duration(profile: Mapping[str, Any]) -> float:
    motion = _mapping(profile, "motion")
    repeats = int(motion["line_repeats_per_heading"])
    nominal_line_s = (
        float(motion["line_distance_m"]) / float(motion["linear_speed_mps"]) + 1.0
    )
    average_probe_rate = 0.5 * (
        float(motion["slow_yaw_rate_rps"]) + float(motion["fast_yaw_rate_rps"])
    )
    nominal_turn_s = (
        4.0 * math.radians(float(motion["turn_probe_deg"])) / average_probe_rate
        + 2.0
        * math.radians(float(motion["heading_b_deg"]))
        / float(motion["slow_yaw_rate_rps"])
    )
    movement_count = 4 * repeats + 6
    return (
        float(motion["start_rest_s"])
        + float(motion["end_rest_s"])
        + 4 * repeats * nominal_line_s
        + nominal_turn_s
        + movement_count * float(motion["settle_s"])
    )


def validate_profile(profile: Mapping[str, Any]) -> None:
    """Reject incomplete or physically unreasonable profiles before ROS motion."""
    if not isinstance(profile, Mapping):
        raise ValueError("profile must contain a YAML mapping")
    if profile.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("unsupported profile schema_version")

    topics = _mapping(profile, "topics")
    required_topics = ("cmd", "imu", "pose", "phase")
    resolved_topics = []
    for key in required_topics:
        value = topics.get(key)
        if not isinstance(value, str) or not value.startswith("/") or any(
            character.isspace() for character in value
        ):
            raise ValueError("topics.%s must be an absolute ROS name" % key)
        resolved_topics.append(value)
    if len(set(resolved_topics)) != len(resolved_topics):
        raise ValueError("required ROS topics must be distinct")
    optional = topics.get("optional", [])
    if not isinstance(optional, list) or any(
        not isinstance(value, str) or not value.startswith("/") for value in optional
    ):
        raise ValueError("topics.optional must contain absolute ROS names")

    field = _mapping(profile, "field")
    width = _number(field, "width_m", 2.0, 20.0)
    height = _number(field, "height_m", 2.0, 20.0)
    margin = _number(field, "margin_m", 0.20, 0.5 * min(width, height) - 0.10)
    if field.get("center") != "initial_vrpn_pose":
        raise ValueError("field.center must be initial_vrpn_pose")
    if min(width, height) - 2.0 * margin < 1.0:
        raise ValueError("field margin leaves too little usable space")

    safety = _mapping(profile, "safety")
    _number(safety, "pose_timeout_s", 0.05, 1.0)
    _number(safety, "imu_timeout_s", 0.02, 1.0)
    _number(safety, "preflight_timeout_s", 2.0, 60.0)
    _number(safety, "stationary_check_s", 0.5, 10.0)
    _number(safety, "stationary_position_span_m", 0.001, 0.10)
    _number(safety, "stationary_yaw_span_deg", 0.1, 10.0)
    maximum_runtime = _number(safety, "max_run_time_s", 20.0, 180.0)
    _number(safety, "zero_publish_s", 0.3, 5.0)
    if not isinstance(safety.get("refuse_other_cmd_publishers"), bool):
        raise ValueError("safety.refuse_other_cmd_publishers must be boolean")

    motion = _mapping(profile, "motion")
    _number(motion, "publish_rate_hz", 10.0, 100.0)
    _number(motion, "start_rest_s", 3.0, 60.0)
    _number(motion, "end_rest_s", 3.0, 60.0)
    _number(motion, "settle_s", 0.2, 5.0)
    distance = _number(motion, "line_distance_m", 0.50, 2.50)
    repeats = motion.get("line_repeats_per_heading")
    if isinstance(repeats, bool) or not isinstance(repeats, int) or not 1 <= repeats <= 3:
        raise ValueError("motion.line_repeats_per_heading must be an integer in [1, 3]")
    _number(motion, "linear_speed_mps", 0.05, 0.60)
    _number(motion, "linear_ramp_s", 0.20, 3.0)
    slow = _number(motion, "slow_yaw_rate_rps", 0.10, 0.80)
    fast = _number(motion, "fast_yaw_rate_rps", 0.10, 0.80)
    if fast <= slow + 0.05:
        raise ValueError("fast_yaw_rate_rps must exceed slow_yaw_rate_rps by at least 0.05")
    _number(motion, "turn_probe_deg", 20.0, 90.0)
    _number(motion, "heading_b_deg", 30.0, 120.0)
    _number(motion, "rotation_tolerance_deg", 0.2, 5.0)
    usable_half_extent = 0.5 * min(width, height) - margin
    if distance > usable_half_extent - 0.20:
        raise ValueError("line distance does not fit inside the centered software geofence")
    nominal_duration = estimate_nominal_duration(profile)
    if nominal_duration > maximum_runtime - 3.0:
        raise ValueError(
            "nominal duration %.1f s leaves no safety margin below max_run_time_s"
            % nominal_duration
        )

    analysis = _mapping(profile, "analysis")
    for key in (
        "min_line_displacement_m",
        "min_heading_separation_deg",
        "max_t_cm_spread_deg",
        "min_imu_impulse_mps",
        "max_t_ci_spread_deg",
        "max_abs_t_ci_yaw_deg",
        "min_rotation_deg",
        "min_gyro_scale",
        "max_gyro_scale",
        "max_gyro_scale_spread",
    ):
        _number(analysis, key, 1.0e-6, 1000.0)
    if float(analysis["min_line_displacement_m"]) >= distance:
        raise ValueError("analysis.min_line_displacement_m must be below line_distance_m")
    if float(analysis["min_gyro_scale"]) >= float(analysis["max_gyro_scale"]):
        raise ValueError("analysis gyro scale interval is empty")

    priors = _mapping(profile, "engineering_priors")
    for key in (
        "accel_noise_std",
        "gyro_noise_std",
        "vrpn_position_noise_std",
        "vrpn_orientation_noise_std",
        "gyro_bias_random_walk_std",
        "accel_bias_random_walk_std",
    ):
        _number(priors, key, 1.0e-12, 1000.0)
