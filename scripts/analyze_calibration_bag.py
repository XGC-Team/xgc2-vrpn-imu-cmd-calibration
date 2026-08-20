#!/usr/bin/env python3
"""Analyze one vrpn_imu_cmd_calibration ROS1 bag and write YAML assets."""

import argparse
import hashlib
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import yaml

try:
    import rosbag
except ImportError as exc:
    raise SystemExit("rosbag is required; source the ROS1 Noetic environment") from exc

from vrpn_imu_cmd_calibration.analysis import analyze_samples, build_outputs


def quaternion_to_rpy(quaternion):
    x, y, z, w = (
        float(quaternion.x),
        float(quaternion.y),
        float(quaternion.z),
        float(quaternion.w),
    )
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def default_profile_path():
    try:
        import rospkg

        return os.path.join(rospkg.RosPack().get_path("vrpn_imu_cmd_calibration"), "config", "default.yaml")
    except Exception:
        return str(Path(__file__).resolve().parents[1] / "config" / "default.yaml")


def load_yaml(path):
    if not path:
        return {}
    with open(path, "r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError("%s must contain a YAML mapping" % path)
    return value


def phase_intervals(messages):
    intervals = []
    current = None
    for receive_time, payload in sorted(messages, key=lambda item: item[0]):
        sequence = int(payload.get("sequence", -1))
        if current is None or sequence != current["sequence"]:
            if current is not None:
                current["end"] = receive_time
                intervals.append(current)
            current = dict(payload)
            current["sequence"] = sequence
            current["start"] = receive_time
            current["end"] = receive_time
        else:
            current["end"] = receive_time
    if current is not None:
        intervals.append(current)
    return [phase for phase in intervals if phase["end"] - phase["start"] >= 0.05]


def read_bag(path, topics):
    imu_rows = []
    pose_rows = []
    cmd_rows = []
    phase_messages = []
    selected = [topics["imu"], topics["pose"], topics["cmd"], topics["phase"]]
    with rosbag.Bag(path, "r") as bag:
        for topic, message, receive_time in bag.read_messages(topics=selected):
            stamp = float(receive_time.to_sec())
            if topic == topics["imu"]:
                imu_rows.append(
                    [
                        stamp,
                        float(message.linear_acceleration.x),
                        float(message.linear_acceleration.y),
                        float(message.linear_acceleration.z),
                        float(message.angular_velocity.x),
                        float(message.angular_velocity.y),
                        float(message.angular_velocity.z),
                    ]
                )
            elif topic == topics["pose"]:
                roll, pitch, yaw = quaternion_to_rpy(message.pose.orientation)
                pose_rows.append(
                    [
                        stamp,
                        float(message.pose.position.x),
                        float(message.pose.position.y),
                        float(message.pose.position.z),
                        roll,
                        pitch,
                        yaw,
                    ]
                )
            elif topic == topics["cmd"]:
                cmd_rows.append([stamp, float(message.linear.x), float(message.angular.z)])
            elif topic == topics["phase"]:
                try:
                    payload = json.loads(message.data)
                except (TypeError, ValueError) as exc:
                    raise ValueError("invalid phase JSON at %.6f: %s" % (stamp, exc))
                phase_messages.append((stamp, payload))
    return {
        "imu": np.asarray(imu_rows, dtype=float),
        "pose": np.asarray(pose_rows, dtype=float),
        "cmd": np.asarray(cmd_rows, dtype=float),
        "phases": phase_intervals(phase_messages),
    }


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def atomic_text(path, text):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    os.replace(str(temporary), str(path))


def write_outputs(output_dir, asset, estimator, controller):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    atomic_text(
        output_dir / "calibration.yaml",
        yaml.safe_dump(asset, allow_unicode=True, sort_keys=False),
    )
    atomic_text(
        output_dir / "estimator.yaml",
        yaml.safe_dump(estimator, allow_unicode=True, sort_keys=False),
    )
    atomic_text(
        output_dir / "controller.yaml",
        yaml.safe_dump(controller, allow_unicode=True, sort_keys=False),
    )
    atomic_text(output_dir / "report.json", json.dumps(asset, ensure_ascii=False, indent=2) + "\n")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--profile", default=default_profile_path())
    parser.add_argument("--base-estimator-yaml")
    parser.add_argument("--output-dir")
    parser.add_argument("--imu-topic")
    parser.add_argument("--pose-topic")
    parser.add_argument("--cmd-topic")
    parser.add_argument("--phase-topic")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    try:
        profile = load_yaml(args.profile)
        topics = dict(profile.get("topics", {}))
        topics["imu"] = args.imu_topic or topics.get("imu", "/imu/data_raw")
        topics["pose"] = args.pose_topic or topics.get("pose", "/vrpn_client_node/pose_0/pose")
        topics["cmd"] = args.cmd_topic or topics.get("cmd", "/cmd_vel")
        topics["phase"] = args.phase_topic or topics.get(
            "phase", "/vrpn_imu_cmd_calibration/phase"
        )
        samples = read_bag(args.bag, topics)
        base = load_yaml(args.base_estimator_yaml)
        priors = dict(profile.get("engineering_priors", {}))
        priors.update({key: base[key] for key in priors if key in base})
        result = analyze_samples(samples, profile.get("analysis", {}), priors)
        bag_path = Path(args.bag).resolve()
        metadata = {
            "bag": str(bag_path),
            "bag_size_bytes": bag_path.stat().st_size,
            "bag_sha256": sha256_file(bag_path),
            "profile": str(Path(args.profile).resolve()),
            "base_estimator_yaml": str(Path(args.base_estimator_yaml).resolve())
            if args.base_estimator_yaml
            else None,
            "topics": topics,
        }
        asset, estimator, controller = build_outputs(result, base, metadata)
        output_dir = args.output_dir or str(bag_path.parent / (bag_path.stem + "-calibration"))
        write_outputs(output_dir, asset, estimator, controller)
    except (OSError, ValueError, KeyError, yaml.YAMLError) as exc:
        print("calibration analysis failed: %s" % exc, file=sys.stderr)
        return 2

    print("status=%s" % asset["status"])
    print("calibration_yaml=%s" % os.path.join(output_dir, "calibration.yaml"))
    print("estimator_yaml=%s" % os.path.join(output_dir, "estimator.yaml"))
    if asset["quality"]["failed_checks"]:
        print("failed_checks=%s" % ",".join(asset["quality"]["failed_checks"]))
    return 0 if asset["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
