#!/usr/bin/env python3
"""Safely excite a planar UGV, record one bag, then analyze priorities 1--6."""

import argparse
import collections
import datetime
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml

try:
    import rosgraph
    import rospy
    from geometry_msgs.msg import PoseStamped, Twist
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
except ImportError as exc:
    raise SystemExit("ROS1 Python modules are required; source Noetic first") from exc


def wrap(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def make_twist(linear, angular):
    message = Twist()
    message.linear.x = float(linear)
    message.angular.z = float(angular)
    return message


def default_profile_path():
    try:
        import rospkg

        return os.path.join(rospkg.RosPack().get_path("vrpn_imu_cmd_calibration"), "config", "default.yaml")
    except Exception:
        return str(Path(__file__).resolve().parents[1] / "config" / "default.yaml")


def load_profile(path):
    with open(path, "r", encoding="utf-8") as stream:
        profile = yaml.safe_load(stream)
    if not isinstance(profile, dict):
        raise ValueError("profile must contain a YAML mapping")
    return profile


def atomic_yaml(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        yaml.safe_dump(value, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    os.replace(str(temporary), str(path))


class CalibrationRunner:
    def __init__(self, profile, args):
        self.profile = profile
        self.args = args
        self.topics = dict(profile["topics"])
        self.field = dict(profile["field"])
        self.safety = dict(profile["safety"])
        self.motion = dict(profile["motion"])
        self.stop_requested = False
        self.abort_reason = None
        self.pose = None
        self.imu = None
        self.pose_wall = None
        self.imu_wall = None
        self.pose_history = collections.deque(maxlen=2000)
        self.imu_history = collections.deque(maxlen=4000)
        self.center = None
        self.sequence = 0
        self.cmd_pub = None
        self.phase_pub = None
        self.bag_process = None
        self.bag_log = None
        self.output_dir = None
        self.bag_path = None
        self.manifest = {}
        self.started_wall = None

    def handle_signal(self, signum, _frame):
        self.abort_reason = self.abort_reason or "signal_%d" % signum
        self.stop_requested = True

    def on_pose(self, message):
        values = (
            float(message.pose.position.x),
            float(message.pose.position.y),
            quaternion_yaw(message.pose.orientation),
        )
        if not all(math.isfinite(value) for value in values):
            self.abort_reason = "non_finite_vrpn_pose"
            self.stop_requested = True
            return
        now = time.monotonic()
        self.pose = values
        self.pose_wall = now
        self.pose_history.append((now,) + values)

    def on_imu(self, message):
        values = (
            float(message.linear_acceleration.x),
            float(message.linear_acceleration.y),
            float(message.linear_acceleration.z),
            float(message.angular_velocity.x),
            float(message.angular_velocity.y),
            float(message.angular_velocity.z),
        )
        if not all(math.isfinite(value) for value in values):
            self.abort_reason = "non_finite_imu"
            self.stop_requested = True
            return
        now = time.monotonic()
        self.imu = values
        self.imu_wall = now
        self.imu_history.append((now,) + values)

    def other_cmd_publishers(self):
        master = rosgraph.Master(rospy.get_name())
        publishers, _subscribers, _services = master.getSystemState()
        names = []
        for topic, nodes in publishers:
            if topic == rospy.resolve_name(self.topics["cmd"]):
                names.extend(node for node in nodes if node != rospy.get_name())
        return sorted(set(names))

    def wait_preflight(self):
        deadline = time.monotonic() + float(self.safety["preflight_timeout_s"])
        rate = rospy.Rate(float(self.motion["publish_rate_hz"]))
        stable_duration = float(self.safety["stationary_check_s"])
        while not rospy.is_shutdown() and not self.stop_requested:
            now = time.monotonic()
            if now > deadline:
                raise RuntimeError("preflight timeout waiting for fresh stable VRPN and IMU")
            if self.pose_wall is None or self.imu_wall is None:
                rate.sleep()
                continue
            if now - self.pose_wall > float(self.safety["pose_timeout_s"]):
                rate.sleep()
                continue
            if now - self.imu_wall > float(self.safety["imu_timeout_s"]):
                rate.sleep()
                continue
            recent_pose = [row for row in self.pose_history if now - row[0] <= stable_duration]
            recent_imu = [row for row in self.imu_history if now - row[0] <= stable_duration]
            if (
                len(recent_pose) < max(10, int(stable_duration * 8.0))
                or len(recent_imu) < max(20, int(stable_duration * 20.0))
                or recent_pose[-1][0] - recent_pose[0][0] < stable_duration * 0.85
            ):
                rate.sleep()
                continue
            xs = [row[1] for row in recent_pose]
            ys = [row[2] for row in recent_pose]
            yaws = [row[3] for row in recent_pose]
            yaw_reference = yaws[0]
            yaw_span = max(wrap(value - yaw_reference) for value in yaws) - min(
                wrap(value - yaw_reference) for value in yaws
            )
            position_span = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
            if position_span > float(self.safety["stationary_position_span_m"]):
                rate.sleep()
                continue
            if abs(yaw_span) > math.radians(float(self.safety["stationary_yaw_span_deg"])):
                rate.sleep()
                continue
            self.center = (sum(xs) / len(xs), sum(ys) / len(ys))
            return
        raise RuntimeError(self.abort_reason or "preflight interrupted")

    def safety_check(self):
        if self.stop_requested or rospy.is_shutdown():
            return False
        now = time.monotonic()
        if self.pose_wall is None or now - self.pose_wall > float(self.safety["pose_timeout_s"]):
            self.abort_reason = "vrpn_pose_stale"
            self.stop_requested = True
            return False
        if self.imu_wall is None or now - self.imu_wall > float(self.safety["imu_timeout_s"]):
            self.abort_reason = "imu_stale"
            self.stop_requested = True
            return False
        if self.started_wall is not None and now - self.started_wall > float(
            self.safety["max_run_time_s"]
        ):
            self.abort_reason = "maximum_run_time_exceeded"
            self.stop_requested = True
            return False
        half_width = 0.5 * float(self.field["width_m"]) - float(self.field["margin_m"])
        half_height = 0.5 * float(self.field["height_m"]) - float(self.field["margin_m"])
        dx, dy = self.pose[0] - self.center[0], self.pose[1] - self.center[1]
        if abs(dx) > half_width or abs(dy) > half_height:
            self.abort_reason = "geofence_crossed_dx_%.3f_dy_%.3f" % (dx, dy)
            self.stop_requested = True
            return False
        return True

    def publish_zero(self, duration=None):
        if self.cmd_pub is None:
            return
        duration = float(duration if duration is not None else self.safety["zero_publish_s"])
        deadline = time.monotonic() + duration
        rate = rospy.Rate(float(self.motion["publish_rate_hz"]))
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.cmd_pub.publish(make_twist(0.0, 0.0))
            rate.sleep()

    def phase_payload(self, name, kind, **values):
        payload = {"sequence": self.sequence, "name": name, "kind": kind}
        payload.update(values)
        return payload

    def publish_phase(self, payload):
        self.phase_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def run_rest(self, name, duration):
        self.sequence += 1
        payload = self.phase_payload(name, "rest", duration_s=float(duration))
        deadline = time.monotonic() + float(duration)
        rate = rospy.Rate(float(self.motion["publish_rate_hz"]))
        rospy.loginfo("calibration phase %s: rest %.1f s", name, duration)
        while self.safety_check() and time.monotonic() < deadline:
            self.publish_phase(payload)
            self.cmd_pub.publish(make_twist(0.0, 0.0))
            rate.sleep()
        if not self.safety_check():
            raise RuntimeError(self.abort_reason or "rest phase aborted")

    def run_line(self, name, direction):
        self.sequence += 1
        target = float(self.motion["line_distance_m"])
        speed_max = float(self.motion["linear_speed_mps"])
        ramp_s = float(self.motion["linear_ramp_s"])
        acceleration = speed_max / max(0.1, ramp_s)
        start_xy = self.pose[:2]
        payload = self.phase_payload(
            name,
            "line",
            direction=float(direction),
            target_distance_m=target,
            speed_mps=float(direction) * speed_max,
        )
        phase_start = time.monotonic()
        deadline = phase_start + max(8.0, target / max(0.05, speed_max) + 6.0)
        rate = rospy.Rate(float(self.motion["publish_rate_hz"]))
        rospy.loginfo("calibration phase %s: line %+.2f m", name, direction * target)
        while self.safety_check():
            now = time.monotonic()
            distance = math.hypot(self.pose[0] - start_xy[0], self.pose[1] - start_xy[1])
            remaining = max(0.0, target - distance)
            if remaining <= 0.015:
                break
            if now >= deadline:
                self.abort_reason = "%s_distance_timeout_%.3f" % (name, distance)
                self.stop_requested = True
                break
            ramp_limit = speed_max * min(1.0, max(0.0, (now - phase_start) / ramp_s))
            brake_limit = math.sqrt(max(0.0, 2.0 * acceleration * remaining))
            speed = max(0.06, min(speed_max, ramp_limit, brake_limit))
            self.publish_phase(payload)
            self.cmd_pub.publish(make_twist(direction * speed, 0.0))
            rate.sleep()
        self.publish_zero(0.4)
        if not self.safety_check():
            raise RuntimeError(self.abort_reason or "line phase aborted")

    def run_rotate(self, name, angle_deg, rate_rps):
        self.sequence += 1
        target = math.radians(abs(float(angle_deg)))
        direction = 1.0 if angle_deg >= 0.0 else -1.0
        commanded_rate = direction * abs(float(rate_rps))
        start_yaw = self.pose[2]
        tolerance = math.radians(float(self.motion["rotation_tolerance_deg"]))
        payload = self.phase_payload(
            name,
            "rotate",
            direction=direction,
            target_angle_rad=direction * target,
            rate_rps=commanded_rate,
        )
        deadline = time.monotonic() + max(8.0, target / max(0.05, abs(rate_rps)) + 5.0)
        angular_acceleration = max(0.25, abs(rate_rps) / 0.6)
        rate = rospy.Rate(float(self.motion["publish_rate_hz"]))
        rospy.loginfo("calibration phase %s: rotate %+.1f deg", name, angle_deg)
        while self.safety_check():
            progress = direction * wrap(self.pose[2] - start_yaw)
            remaining = target - progress
            if remaining <= tolerance:
                break
            if time.monotonic() >= deadline:
                self.abort_reason = "%s_rotation_timeout_%.3f" % (name, progress)
                self.stop_requested = True
                break
            brake_limit = math.sqrt(max(0.0, 2.0 * angular_acceleration * remaining))
            speed = max(0.08, min(abs(rate_rps), brake_limit))
            self.publish_phase(payload)
            self.cmd_pub.publish(make_twist(0.0, direction * speed))
            rate.sleep()
        self.publish_zero(0.4)
        if not self.safety_check():
            raise RuntimeError(self.abort_reason or "rotation phase aborted")

    def settle(self, suffix):
        self.run_rest("settle_" + suffix, float(self.motion["settle_s"]))

    def execute_profile(self):
        self.started_wall = time.monotonic()
        self.run_rest("rest_start", float(self.motion["start_rest_s"]))
        repeats = int(self.motion["line_repeats_per_heading"])
        for index in range(repeats):
            self.run_line("a_fwd_%d" % (index + 1), 1.0)
            self.settle("a_fwd_%d" % (index + 1))
            self.run_line("a_rev_%d" % (index + 1), -1.0)
            self.settle("a_rev_%d" % (index + 1))

        probe = float(self.motion["turn_probe_deg"])
        slow = float(self.motion["slow_yaw_rate_rps"])
        fast = float(self.motion["fast_yaw_rate_rps"])
        for name, angle, yaw_rate in (
            ("probe_slow_ccw", probe, slow),
            ("probe_slow_cw", -probe, slow),
            ("probe_fast_cw", -probe, fast),
            ("probe_fast_ccw", probe, fast),
        ):
            self.run_rotate(name, angle, yaw_rate)
            self.settle(name)

        heading_b = float(self.motion["heading_b_deg"])
        self.run_rotate("turn_to_b", heading_b, slow)
        self.settle("turn_to_b")
        for index in range(repeats):
            self.run_line("b_fwd_%d" % (index + 1), 1.0)
            self.settle("b_fwd_%d" % (index + 1))
            self.run_line("b_rev_%d" % (index + 1), -1.0)
            self.settle("b_rev_%d" % (index + 1))
        self.run_rotate("turn_home", -heading_b, slow)
        self.settle("turn_home")
        self.run_rest("rest_end", float(self.motion["end_rest_s"]))

    def start_bag(self):
        self.bag_path = self.output_dir / "calibration.bag"
        self.bag_log = open(self.output_dir / "rosbag-record.log", "w", encoding="utf-8")
        topics = [self.topics["imu"], self.topics["pose"], self.topics["cmd"], self.topics["phase"]]
        topics.extend(self.topics.get("optional", []))
        topics = list(dict.fromkeys(topics))
        command = [
            "rosbag",
            "record",
            "--lz4",
            "--buffsize=2048",
            "-O",
            str(self.bag_path),
        ] + topics
        self.bag_process = subprocess.Popen(command, stdout=self.bag_log, stderr=subprocess.STDOUT)
        time.sleep(1.0)
        if self.bag_process.poll() is not None:
            raise RuntimeError("rosbag record exited before motion")
        self.manifest["recorded_topics"] = topics

    def stop_bag(self):
        if self.bag_process is not None and self.bag_process.poll() is None:
            self.bag_process.send_signal(signal.SIGINT)
            try:
                self.bag_process.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                self.bag_process.terminate()
                self.bag_process.wait(timeout=3.0)
        if self.bag_log is not None:
            self.bag_log.close()
            self.bag_log = None

    def capture_environment(self):
        subprocess.run(
            ["rosparam", "dump", str(self.output_dir / "rosparam.yaml")],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        with open(self.output_dir / "topics.txt", "w", encoding="utf-8") as stream:
            subprocess.run(["rostopic", "list", "-v"], check=False, stdout=stream, stderr=subprocess.STDOUT)

    def write_manifest(self, status):
        now = datetime.datetime.now(datetime.timezone.utc).isoformat()
        self.manifest.update(
            {
                "schema_version": "xgc2.vrpn_imu_cmd_calibration/run/v1",
                "status": status,
                "abort_reason": self.abort_reason,
                "updated_at": now,
                "field_center_vrpn_xy": list(self.center) if self.center else None,
                "profile": self.profile,
                "base_estimator_yaml": self.args.base_estimator_yaml,
            }
        )
        atomic_yaml(self.output_dir / "manifest.yaml", self.manifest)

    def analyze(self):
        analyzer = str(Path(__file__).resolve().with_name("analyze_calibration_bag.py"))
        command = [
            sys.executable,
            analyzer,
            str(self.bag_path),
            "--profile",
            self.args.profile,
            "--output-dir",
            str(self.output_dir / "analysis"),
        ]
        if self.args.base_estimator_yaml:
            command.extend(["--base-estimator-yaml", self.args.base_estimator_yaml])
        return subprocess.run(command, check=False).returncode

    def run(self):
        rospy.init_node("vrpn_imu_cmd_calibration", anonymous=False, disable_signals=True)
        self.cmd_pub = rospy.Publisher(self.topics["cmd"], Twist, queue_size=1)
        self.phase_pub = rospy.Publisher(self.topics["phase"], String, queue_size=10, latch=True)
        rospy.Subscriber(self.topics["pose"], PoseStamped, self.on_pose, queue_size=10)
        rospy.Subscriber(self.topics["imu"], Imu, self.on_imu, queue_size=100)
        self.wait_preflight()
        conflicts = self.other_cmd_publishers()
        if conflicts and bool(self.safety.get("refuse_other_cmd_publishers", True)) and not self.args.allow_other_cmd_publishers:
            raise RuntimeError("other cmd_vel publishers are active: %s" % ", ".join(conflicts))
        timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        self.output_dir = Path(self.args.output_root).expanduser().resolve() / ("vrpn-imu-cmd-" + timestamp)
        self.output_dir.mkdir(parents=True, exist_ok=False)
        self.manifest = {
            "started_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "cmd_publishers_seen": conflicts,
            "ros_master_uri": os.environ.get("ROS_MASTER_URI"),
        }
        self.capture_environment()
        self.write_manifest("RECORDING")
        self.start_bag()
        self.execute_profile()
        self.publish_zero()
        self.stop_bag()
        self.write_manifest("RECORDED")
        analysis_code = self.analyze() if not self.args.no_analyze else 0
        self.write_manifest("PASS" if analysis_code == 0 else "ANALYSIS_FAILED")
        return analysis_code


def describe_profile(profile):
    motion = profile["motion"]
    repeats = int(motion["line_repeats_per_heading"])
    nominal_line_s = float(motion["line_distance_m"]) / float(motion["linear_speed_mps"]) + 1.0
    nominal_turn_s = (
        4.0 * math.radians(float(motion["turn_probe_deg"]))
        / ((float(motion["slow_yaw_rate_rps"]) + float(motion["fast_yaw_rate_rps"])) * 0.5)
        + 2.0 * math.radians(float(motion["heading_b_deg"])) / float(motion["slow_yaw_rate_rps"])
    )
    movement_count = 4 * repeats + 6
    nominal = (
        float(motion["start_rest_s"])
        + float(motion["end_rest_s"])
        + 4 * repeats * nominal_line_s
        + nominal_turn_s
        + movement_count * float(motion["settle_s"])
    )
    print("profile: 8x8 m, center=first stable VRPN pose")
    print("line legs: %d at %.2f m, headings separated by %.1f deg" % (4 * repeats, motion["line_distance_m"], motion["heading_b_deg"]))
    print("rotations: both signs at %.2f and %.2f rad/s" % (motion["slow_yaw_rate_rps"], motion["fast_yaw_rate_rps"]))
    print("nominal duration: %.0f s" % nominal)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", default=default_profile_path())
    parser.add_argument("--output-root", default="~/xgc2-calibration-runs")
    parser.add_argument("--base-estimator-yaml")
    parser.add_argument("--execute", action="store_true", help="required before publishing nonzero cmd_vel")
    parser.add_argument("--allow-other-cmd-publishers", action="store_true")
    parser.add_argument("--no-analyze", action="store_true")
    return parser.parse_args(rospy.myargv(argv=argv)[1:] if argv is not None else rospy.myargv()[1:])


def main(argv=None):
    args = parse_args(argv)
    try:
        profile = load_profile(args.profile)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print("invalid profile: %s" % exc, file=sys.stderr)
        return 2
    describe_profile(profile)
    if not args.execute:
        print("dry run only; add --execute after clearing the field and checking the physical E-stop")
        return 0

    runner = CalibrationRunner(profile, args)
    signal.signal(signal.SIGINT, runner.handle_signal)
    signal.signal(signal.SIGTERM, runner.handle_signal)
    try:
        return runner.run()
    except (OSError, RuntimeError, ValueError) as exc:
        runner.abort_reason = runner.abort_reason or str(exc)
        print("calibration run aborted: %s" % exc, file=sys.stderr)
        if runner.output_dir:
            runner.write_manifest("ABORTED")
        return 2
    finally:
        try:
            runner.publish_zero()
        except Exception:
            pass
        runner.stop_bag()


if __name__ == "__main__":
    sys.exit(main())
