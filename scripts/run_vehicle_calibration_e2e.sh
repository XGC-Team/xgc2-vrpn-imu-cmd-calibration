#!/usr/bin/env bash
set -euo pipefail

mode="${1:-}"
if [[ "$mode" != "simulation" && "$mode" != "physical-preflight" && "$mode" != "physical" ]]; then
  echo "usage: $0 {simulation|physical-preflight|physical}" >&2
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if package_root="$(rospack find vrpn_imu_cmd_calibration 2>/dev/null)"; then
  :
else
  package_root="$(cd "$script_dir/.." && pwd)"
fi
export PYTHONPATH="$package_root/src${PYTHONPATH:+:$PYTHONPATH}"

runner="$script_dir/run_calibration.py"
simulator="$script_dir/simulate_calibration_sensors.py"
verifier="$script_dir/verify_calibration_result.py"
installer="$script_dir/install_calibration.py"
default_profile="$package_root/config/default.yaml"
simulation_profile="$package_root/config/e2e_simulation.yaml"

for executable in "$runner" "$verifier" "$installer"; do
  if [[ ! -x "$executable" ]]; then
    echo "required executable is missing: $executable" >&2
    exit 2
  fi
done

simulation_cleanup() {
  if [[ -n "${simulator_pid:-}" ]]; then
    kill -TERM "$simulator_pid" 2>/dev/null || true
    wait "$simulator_pid" 2>/dev/null || true
  fi
  if [[ -n "${roscore_pid:-}" ]]; then
    kill -TERM "$roscore_pid" 2>/dev/null || true
    wait "$roscore_pid" 2>/dev/null || true
  fi
}

wait_for_master() {
  local attempt
  for attempt in $(seq 1 50); do
    if rosparam list >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.1
  done
  echo "ROS master did not become ready" >&2
  return 1
}

latest_run_dir() {
  find "$1" -mindepth 1 -maxdepth 1 -type d -name 'vrpn-imu-cmd-*' -printf '%T@ %p\n' \
    | sort -n \
    | tail -1 \
    | cut -d' ' -f2-
}

dry_run_install() {
  local run_dir="$1"
  local estimator_target="$2"
  local controller_target="${3:-}"
  local arguments=(
    "$run_dir/analysis"
    --estimator-target "$estimator_target"
  )
  if [[ -n "$controller_target" ]]; then
    arguments+=(--controller-target "$controller_target")
  fi
  "$installer" "${arguments[@]}"
}

if [[ "$mode" == "simulation" ]]; then
  if [[ ! -x "$simulator" ]]; then
    echo "required simulator is missing: $simulator" >&2
    exit 2
  fi
  if [[ -n "${XGC_VRPN_IMU_CMD_E2E_ROOT:-}" ]]; then
    e2e_root="$XGC_VRPN_IMU_CMD_E2E_ROOT"
    mkdir -p "$e2e_root"
  else
    e2e_root="$(mktemp -d "${TMPDIR:-/tmp}/xgc2-vrpn-imu-cmd-e2e-XXXXXX")"
  fi
  if [[ -n "${XGC_VRPN_IMU_CMD_E2E_ROS_PORT:-}" ]]; then
    ros_port="$XGC_VRPN_IMU_CMD_E2E_ROS_PORT"
  else
    ros_port="$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1", 0)); print(s.getsockname()[1]); s.close()')"
  fi
  export ROS_HOME="$e2e_root/ros-home"
  mkdir -p "$ROS_HOME"
  export ROS_MASTER_URI="http://127.0.0.1:$ros_port"
  export ROS_IP=127.0.0.1
  export ROS_HOSTNAME=127.0.0.1
  trap simulation_cleanup EXIT INT TERM
  roscore -p "$ros_port" >"$e2e_root/roscore.log" 2>&1 &
  roscore_pid=$!
  wait_for_master
  "$simulator" >"$e2e_root/simulator.log" 2>&1 &
  simulator_pid=$!
  "$runner" \
    --profile "$simulation_profile" \
    --output-root "$e2e_root" \
    --execute
  run_dir="$(latest_run_dir "$e2e_root")"
  if [[ -z "$run_dir" ]]; then
    echo "simulation produced no run directory under $e2e_root" >&2
    exit 1
  fi
  "$verifier" "$run_dir" --mode simulation

  workspace_estimator="$package_root/../../estimator/rigid-state/estimator_vrpn_px4_rotor_state/config/vrpn_ugv_rigid_state_estimator.yaml"
  workspace_controller="$package_root/../../../controller/ugv-controller/unicycle_ugv_controller/config/unicycle_ugv_controller.yaml"
  if [[ -f "$workspace_estimator" && -f "$workspace_controller" ]]; then
    dry_run_install "$run_dir" "$workspace_estimator" "$workspace_controller"
  else
    dry_run_install "$run_dir" "$run_dir/analysis/estimator.yaml" "$run_dir/analysis/controller.yaml"
  fi
  echo "simulation_e2e=PASS"
  echo "artifacts=$run_dir"
  exit 0
fi

profile="${XGC_VRPN_IMU_CMD_PROFILE:-$default_profile}"
vehicle_id="${XGC_VRPN_IMU_CMD_VEHICLE_ID:-ugv1}"
documents_dir="${XDG_DOCUMENTS_DIR:-${HOME}/Documents}"
output_root="${XGC_VRPN_IMU_CMD_OUTPUT_ROOT:-$documents_dir/XGC/Calibration/vrpn-imu-cmd/$vehicle_id}"
base_estimator="${XGC_VRPN_IMU_CMD_BASE_ESTIMATOR_YAML:-}"
controller_target="${XGC_VRPN_IMU_CMD_CONTROLLER_YAML:-}"

echo "vehicle=$vehicle_id"
echo "profile=$profile"
echo "output_root=$output_root"
echo "This physical chain uses cmd_vel and requires an empty 8x8 m field, operator, and working E-stop."

preflight_arguments=(--profile "$profile" --output-root "$output_root" --preflight-only)
if [[ -n "$base_estimator" ]]; then
  preflight_arguments+=(--base-estimator-yaml "$base_estimator")
fi
"$runner" "${preflight_arguments[@]}"
echo "physical_preflight=PASS"

if [[ "$mode" == "physical-preflight" ]]; then
  echo "No nonzero cmd_vel was published."
  exit 0
fi

if [[ "${XGC_VRPN_IMU_CMD_PHYSICAL_CONFIRMED:-}" != "YES" ]]; then
  echo "refuse physical motion: set XGC_VRPN_IMU_CMD_PHYSICAL_CONFIRMED=YES only after clearing the field and checking the E-stop" >&2
  exit 2
fi
if [[ -z "$base_estimator" || ! -f "$base_estimator" ]]; then
  echo "refuse physical run: XGC_VRPN_IMU_CMD_BASE_ESTIMATOR_YAML must name the vehicle's existing estimator YAML" >&2
  exit 2
fi
mkdir -p "$output_root"
"$runner" \
  --profile "$profile" \
  --base-estimator-yaml "$base_estimator" \
  --output-root "$output_root" \
  --execute
run_dir="$(latest_run_dir "$output_root")"
if [[ -z "$run_dir" ]]; then
  echo "physical run produced no artifact directory" >&2
  exit 1
fi
"$verifier" "$run_dir" --mode physical
dry_run_install "$run_dir" "$base_estimator" "$controller_target"
echo "physical_e2e=PASS"
echo "artifacts=$run_dir"
echo "The generated estimator/controller merge was validated as a dry run; inspect it before using --apply."
