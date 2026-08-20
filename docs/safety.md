# 8 m x 8 m field safety contract

The runner treats the first stable VRPN position as the center of an 8 m x 8 m
field. Absolute mocap coordinates are irrelevant. With the default 0.75 m
margin the allowed relative box is `[-3.25, +3.25] m` on both axes; the nominal
trajectory stays within about 1.3 m of its start.

Motion is allowed only when all checks hold:

1. VRPN pose and IMU are present, finite, fresh, and stationary at startup.
2. No other `/cmd_vel` publisher exists, unless the operator explicitly uses
   the audited override.
3. The robot starts at the physical field center, the field is empty, and a
   working physical emergency stop and operator are present.
4. During motion, stale VRPN, stale IMU, a geofence crossing, timeout, signal,
   ROS shutdown, or an exception immediately switches to repeated zero command.

The software geofence is a last line of defense, not collision avoidance. It
does not perceive people or obstacles and cannot know that the initial pose is
the physical center.
