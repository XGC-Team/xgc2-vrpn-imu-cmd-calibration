#!/usr/bin/env python3
"""Closed-loop planar sensor simulator for the cmd calibration ROS E2E."""

import argparse
import math
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


class PlanarCalibrationSimulator:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.command_v = 0.0
        self.command_w = 0.0
        self.x = float(args.start_x)
        self.y = float(args.start_y)
        self.yaw = float(args.start_yaw)
        self.previous_v = 0.0
        self.last_update = time.monotonic()
        self.started = self.last_update
        self.imu_sequence = 0
        self.pose_sequence = 0
        self.pose_pub = rospy.Publisher(args.pose_topic, PoseStamped, queue_size=10)
        self.imu_pub = rospy.Publisher(args.imu_topic, Imu, queue_size=100)
        rospy.Subscriber(args.cmd_topic, Twist, self.on_cmd, queue_size=10)

    def on_cmd(self, message):
        with self.lock:
            self.command_v = float(message.linear.x)
            self.command_w = float(message.angular.z)

    def step(self):
        now = time.monotonic()
        dt = min(0.03, max(1.0e-4, now - self.last_update))
        self.last_update = now
        with self.lock:
            velocity = self.command_v
            yaw_rate = self.command_w
        acceleration_c = (velocity - self.previous_v) / dt
        self.previous_v = velocity
        self.x += math.cos(self.yaw) * velocity * dt
        self.y += math.sin(self.yaw) * velocity * dt
        self.yaw += yaw_rate * dt
        elapsed = now - self.started
        self.publish_imu(acceleration_c, yaw_rate, elapsed)
        if self.imu_sequence % 3 == 0:
            self.publish_pose(elapsed)
        self.imu_sequence += 1

    def publish_imu(self, acceleration_c, yaw_rate, elapsed):
        c = math.cos(-float(self.args.t_ci_yaw))
        s = math.sin(-float(self.args.t_ci_yaw))
        message = Imu()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "e2e_imu"
        message.linear_acceleration.x = (
            c * acceleration_c + 0.020 + 0.010 * math.sin(17.0 * elapsed)
        )
        message.linear_acceleration.y = (
            s * acceleration_c - 0.010 + 0.009 * math.sin(19.0 * elapsed + 0.3)
        )
        message.linear_acceleration.z = 9.8366 + 0.012 * math.sin(23.0 * elapsed + 0.8)
        message.angular_velocity.x = 0.0012 * math.sin(29.0 * elapsed)
        message.angular_velocity.y = 0.0010 * math.sin(31.0 * elapsed + 0.2)
        message.angular_velocity.z = (
            yaw_rate / float(self.args.gyro_correction_scale)
            + 0.006
            + 0.0015 * math.sin(37.0 * elapsed + 0.5)
        )
        self.imu_pub.publish(message)

    def publish_pose(self, elapsed):
        message = PoseStamped()
        message.header.seq = self.pose_sequence
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "mocap_world"
        message.pose.position.x = self.x + 0.0008 * math.sin(11.0 * elapsed)
        message.pose.position.y = self.y + 0.0007 * math.sin(13.0 * elapsed + 0.4)
        message.pose.position.z = 0.001 * math.sin(7.0 * elapsed)
        marker_yaw = self.yaw + float(self.args.t_cm_yaw)
        x, y, z, w = quaternion_from_yaw(marker_yaw)
        message.pose.orientation.x = x
        message.pose.orientation.y = y
        message.pose.orientation.z = z
        message.pose.orientation.w = w
        self.pose_pub.publish(message)
        self.pose_sequence += 1


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--cmd-topic", default="/vrpn_imu_cmd_calibration/e2e/cmd_vel"
    )
    parser.add_argument(
        "--imu-topic", default="/vrpn_imu_cmd_calibration/e2e/imu/data_raw"
    )
    parser.add_argument(
        "--pose-topic", default="/vrpn_imu_cmd_calibration/e2e/vrpn/pose"
    )
    parser.add_argument("--start-x", type=float, default=12.5)
    parser.add_argument("--start-y", type=float, default=-7.3)
    parser.add_argument("--start-yaw", type=float, default=0.25)
    parser.add_argument("--t-cm-yaw", type=float, default=math.radians(93.0))
    parser.add_argument("--t-ci-yaw", type=float, default=math.radians(5.0))
    parser.add_argument("--gyro-correction-scale", type=float, default=1.02)
    return parser.parse_args(rospy.myargv()[1:])


def main():
    args = parse_args()
    rospy.init_node("vrpn_imu_cmd_calibration_e2e_simulator", anonymous=False)
    simulator = PlanarCalibrationSimulator(args)
    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        simulator.step()
        rate.sleep()


if __name__ == "__main__":
    main()
