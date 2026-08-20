#!/usr/bin/env python3
"""Verify bag, safety envelope, result schema, and optional simulation truth."""

import argparse
import json
import math
from pathlib import Path

import rosbag
import yaml


def load_yaml(path):
    with open(str(path), "r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError("%s must be a YAML mapping" % path)
    return value


def wrap(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def verify_bag(bag_path, asset, manifest):
    topics = asset["metadata"]["topics"]
    commands = []
    poses = []
    phases = set()
    with rosbag.Bag(str(bag_path), "r") as bag:
        info = bag.get_type_and_topic_info().topics
        expected_types = {
            topics["cmd"]: "geometry_msgs/Twist",
            topics["imu"]: "sensor_msgs/Imu",
            topics["pose"]: "geometry_msgs/PoseStamped",
            topics["phase"]: "std_msgs/String",
        }
        for topic, expected_type in expected_types.items():
            if topic not in info or info[topic].msg_type != expected_type:
                raise ValueError("bag topic %s is missing or has the wrong type" % topic)
        for topic, message, _stamp in bag.read_messages(
            topics=[topics["cmd"], topics["pose"], topics["phase"]]
        ):
            if topic == topics["cmd"]:
                commands.append((float(message.linear.x), float(message.angular.z)))
            elif topic == topics["pose"]:
                poses.append((float(message.pose.position.x), float(message.pose.position.y)))
            else:
                phases.add(json.loads(message.data)["name"])
    if len(commands) < 100 or not any(abs(v) > 0.05 for v, _w in commands):
        raise ValueError("bag does not contain the automatic nonzero excitation")
    if any(abs(v) > 1.0e-6 or abs(w) > 1.0e-6 for v, w in commands[-10:]):
        raise ValueError("recorded command does not end with repeated zero velocity")
    if len(phases) < 18:
        raise ValueError("recorded phase coverage is incomplete")
    center = manifest.get("field_center_vrpn_xy")
    if not isinstance(center, list) or len(center) != 2:
        raise ValueError("manifest has no measured VRPN field center")
    profile = manifest["profile"]
    half_x = 0.5 * float(profile["field"]["width_m"]) - float(
        profile["field"]["margin_m"]
    )
    half_y = 0.5 * float(profile["field"]["height_m"]) - float(
        profile["field"]["margin_m"]
    )
    if any(abs(x - center[0]) > half_x or abs(y - center[1]) > half_y for x, y in poses):
        raise ValueError("recorded pose crossed the configured centered geofence")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir")
    parser.add_argument("--mode", choices=("simulation", "physical"), default="physical")
    args = parser.parse_args()
    run_dir = Path(args.run_dir).resolve()
    asset = load_yaml(run_dir / "analysis" / "calibration.yaml")
    estimator = load_yaml(run_dir / "analysis" / "estimator.yaml")
    controller = load_yaml(run_dir / "analysis" / "controller.yaml")
    manifest = load_yaml(run_dir / "manifest.yaml")
    if asset.get("schema_version") != "xgc2.vrpn_imu_cmd_calibration/result/v1":
        raise ValueError("unexpected result schema")
    if asset.get("status") != "PASS" or not asset.get("quality", {}).get(
        "runtime_yaml_ready"
    ):
        raise ValueError("calibration asset is not runtime-ready PASS")
    if manifest.get("status") != "PASS":
        raise ValueError("run manifest is not PASS")
    if not estimator.get("extrinsic_verified"):
        raise ValueError("generated estimator does not enable verified extrinsic")
    if not estimator.get("imu_noise_std_is_density"):
        raise ValueError("generated estimator does not mark IMU noise as density")
    if controller.get("state_source") != "state_estimator":
        raise ValueError("controller still permits direct VRPN bypass")
    if controller.get("state_estimate_topic") != "alg/state_estimator/state":
        raise ValueError("controller state-estimate linkage is wrong")
    verify_bag(run_dir / "calibration.bag", asset, manifest)
    if args.mode == "simulation":
        t_cm = float(asset["estimates"]["T_CM"]["rpy_rad"][2])
        t_ci = float(asset["estimates"]["T_CI"]["rpy_rad"][2])
        gyro_scale = float(asset["estimates"]["gyro_z"]["correction_scale"])
        if abs(wrap(t_cm - math.radians(93.0))) > math.radians(2.0):
            raise ValueError("T_CM.yaw misses simulation truth")
        if abs(wrap(t_ci - math.radians(5.0))) > math.radians(3.0):
            raise ValueError("T_CI.yaw misses simulation truth")
        if abs(gyro_scale - 1.02) > 0.035:
            raise ValueError("gyro correction scale misses simulation truth")
        center = manifest["field_center_vrpn_xy"]
        if math.hypot(center[0] - 12.5, center[1] + 7.3) > 0.02:
            raise ValueError("simulation did not prove nonzero mocap world origin")
    print("verified_%s_run=%s" % (args.mode, run_dir))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
