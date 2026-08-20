#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-noetic}"
INSTALL_ROOT=""
OUTPUT_DIR=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root) INSTALL_ROOT="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 1 ;;
  esac
done

if [[ -z "${INSTALL_ROOT}" || -z "${OUTPUT_DIR}" ]]; then
  echo "--install-root and --output-dir are required" >&2
  exit 1
fi

VERSION="${PACKAGE_VERSION:-$(awk '/^version:/ {print $2; exit}' "${REPO_ROOT}/.xgc2/product.yml")}"
if [[ -z "${VERSION}" ]]; then
  echo "package version is missing" >&2
  exit 1
fi

ARCH="$(dpkg --print-architecture)"
PREFIX="/opt/ros/${ROS_DISTRO}"
BUILD_ROOT="$(mktemp -d)"
trap 'rm -rf "${BUILD_ROOT}"' EXIT
PACKAGE_NAME="ros-noetic-xgc2-vrpn-imu-cmd-calibration"
PACKAGE_ROOT="${BUILD_ROOT}/${PACKAGE_NAME}"
mkdir -p "${PACKAGE_ROOT}" "${OUTPUT_DIR}"

copy_path() {
  local source="$1"
  if [[ ! -e "${source}" ]]; then return; fi
  local relative="${source#${INSTALL_ROOT}}"
  mkdir -p "${PACKAGE_ROOT}$(dirname "${relative}")"
  cp -a "${source}" "${PACKAGE_ROOT}${relative}"
}

copy_path "${INSTALL_ROOT}${PREFIX}/share/vrpn_imu_cmd_calibration"
copy_path "${INSTALL_ROOT}${PREFIX}/lib/vrpn_imu_cmd_calibration"
copy_path "${INSTALL_ROOT}${PREFIX}/lib/python3/dist-packages/vrpn_imu_cmd_calibration"

mkdir -p "${PACKAGE_ROOT}/DEBIAN" "${PACKAGE_ROOT}/usr/share/doc/${PACKAGE_NAME}"
cat >"${PACKAGE_ROOT}/DEBIAN/control" <<EOF
Package: ${PACKAGE_NAME}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <dev@xiaokang.ink>
Depends: python3-numpy, python3-rospkg, python3-yaml, ros-noetic-geometry-msgs, ros-noetic-rosbag, ros-noetic-rosbash, ros-noetic-rosgraph, ros-noetic-roslaunch, ros-noetic-rospack, ros-noetic-rospy, ros-noetic-sensor-msgs, ros-noetic-std-msgs
Description: Safe short-run VRPN and IMU parameter identification for ROS Noetic UGVs
EOF
install -m 0644 "${REPO_ROOT}/LICENSE" "${PACKAGE_ROOT}/usr/share/doc/${PACKAGE_NAME}/copyright"

test -f "${PACKAGE_ROOT}${PREFIX}/share/vrpn_imu_cmd_calibration/config/default.yaml"
test -f "${PACKAGE_ROOT}${PREFIX}/lib/python3/dist-packages/vrpn_imu_cmd_calibration/analysis.py"
test -x "${PACKAGE_ROOT}${PREFIX}/lib/vrpn_imu_cmd_calibration/run_calibration.py"
test -x "${PACKAGE_ROOT}${PREFIX}/lib/vrpn_imu_cmd_calibration/run_vehicle_calibration_e2e.sh"
find "${PACKAGE_ROOT}" -type d -name __pycache__ -prune -exec rm -rf {} +
find "${PACKAGE_ROOT}" -type d -exec chmod 0755 {} +
find "${PACKAGE_ROOT}" -type f -exec chmod 0644 {} +
find "${PACKAGE_ROOT}${PREFIX}/lib/vrpn_imu_cmd_calibration" -type f -exec chmod 0755 {} +
chmod 0755 "${PACKAGE_ROOT}/DEBIAN"
fakeroot dpkg-deb --build \
  "${PACKAGE_ROOT}" "${OUTPUT_DIR}/${PACKAGE_NAME}_${VERSION}_${ARCH}.deb" >/dev/null
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name "${PACKAGE_NAME}_${VERSION}_${ARCH}.deb" -print
