#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
PREFIX="/opt/ros/${ROS_DISTRO}"

# shellcheck disable=SC1090
source "${PREFIX}/setup.bash"
dpkg -s ros-noetic-xgc2-vrpn-imu-cmd-calibration >/dev/null
test "$(rospack find vrpn_imu_cmd_calibration)" = "${PREFIX}/share/vrpn_imu_cmd_calibration"
for executable in \
  analyze_calibration_bag.py \
  install_calibration.py \
  run_calibration.py \
  run_vehicle_calibration_e2e.sh \
  simulate_calibration_sensors.py \
  verify_calibration_result.py; do
  test -x "${PREFIX}/lib/vrpn_imu_cmd_calibration/${executable}"
done
python3 -c 'from vrpn_imu_cmd_calibration.analysis import analyze_samples, build_outputs; from vrpn_imu_cmd_calibration.profile import validate_profile'
rosrun vrpn_imu_cmd_calibration run_calibration.py >/tmp/vrpn-imu-cmd-dry-run.log
grep -q 'dry run only' /tmp/vrpn-imu-cmd-dry-run.log
rosrun vrpn_imu_cmd_calibration analyze_calibration_bag.py --help >/dev/null
rosrun vrpn_imu_cmd_calibration run_vehicle_calibration_e2e.sh simulation

echo "Installed standalone VRPN IMU cmd calibration package passed"
