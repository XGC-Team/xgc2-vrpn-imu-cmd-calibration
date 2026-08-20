#!/usr/bin/env python3
"""Write the deterministic synthetic run as a ROS1 bag for local CLI checks."""

import argparse
import json
import math

import rosbag
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from test_analysis import synthetic_run


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    args = parser.parse_args()
    samples, _t_cm, _scale = synthetic_run()
    events = []
    for row in samples["imu"]:
        message = Imu()
        message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = row[1:4]
        message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = row[4:7]
        events.append((row[0], "/imu/data_raw", message))
    for row in samples["pose"]:
        message = PoseStamped()
        message.pose.position.x, message.pose.position.y, message.pose.position.z = row[1:4]
        x, y, z, w = quaternion_from_rpy(row[4], row[5], row[6])
        message.pose.orientation.x = x
        message.pose.orientation.y = y
        message.pose.orientation.z = z
        message.pose.orientation.w = w
        events.append((row[0], "/vrpn_client_node/pose_0/pose", message))
    for row in samples["cmd"]:
        message = Twist()
        message.linear.x, message.angular.z = row[1:3]
        events.append((row[0], "/cmd_vel", message))
    for phase in samples["phases"]:
        payload = {key: value for key, value in phase.items() if key not in ("start", "end")}
        stamp = phase["start"]
        while stamp < phase["end"]:
            events.append((stamp, "/vrpn_imu_cmd_calibration/phase", String(data=json.dumps(payload))))
            stamp += 0.1
    with rosbag.Bag(args.bag, "w", compression=rosbag.Compression.LZ4) as bag:
        for stamp, topic, message in sorted(events, key=lambda item: item[0]):
            bag.write(topic, message, rospy.Time.from_sec(float(stamp)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
