#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DOCKER_IMAGE="${DOCKER_IMAGE:-ghcr.io/xgc-team/xgc2-images/xgc2-build-focal-full-noetic:1.0.0}"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/docker}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image) DOCKER_IMAGE="$2"; shift 2 ;;
    --work-dir) WORK_DIR="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --skip-install-check) INSTALL_CHECK=false; shift ;;
    *) echo "unknown argument: $1" >&2; exit 1 ;;
  esac
done

mkdir -p "${WORK_DIR}" "${OUTPUT_DIR}"
PACKAGE_VERSION="${PACKAGE_VERSION:-$(awk '/^version:/ {print $2; exit}' "${REPO_ROOT}/.xgc2/product.yml")}"
if [[ -z "${PACKAGE_VERSION}" ]]; then
  echo "package version is missing" >&2
  exit 1
fi

docker pull "${DOCKER_IMAGE}"
docker run --rm \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -e PACKAGE_VERSION="${PACKAGE_VERSION}" \
  -v "${REPO_ROOT}:/workspace/repo:ro" \
  -v "${WORK_DIR}:/workspace/work" \
  -v "${OUTPUT_DIR}:/workspace/out" \
  "${DOCKER_IMAGE}" bash -lc '
    set -euo pipefail
    rm -rf /workspace/work/src /workspace/work/build /workspace/work/devel /workspace/work/install-root
    mkdir -p /workspace/work/src/vrpn_imu_cmd_calibration
    rsync -a --delete \
      --exclude .git --exclude .work --exclude debs \
      /workspace/repo/ /workspace/work/src/vrpn_imu_cmd_calibration/
    cd /workspace/work
    source /opt/ros/noetic/setup.bash
    catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo
    ROS_HOME=/workspace/work/ros-home ROS_LOG_DIR=/workspace/work/ros-log catkin_make run_tests
    catkin_test_results --verbose
    DESTDIR=/workspace/work/install-root catkin_make install \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic -DCATKIN_ENABLE_TESTING=OFF
    /workspace/repo/.xgc2/scripts/package_debs.sh \
      --install-root /workspace/work/install-root --output-dir /workspace/out
  '

if [[ "${INSTALL_CHECK}" == true ]]; then
  docker run --rm \
    -e DEBIAN_FRONTEND=noninteractive \
    -e PACKAGE_VERSION="${PACKAGE_VERSION}" \
    -v "${REPO_ROOT}:/workspace/repo:ro" \
    -v "${OUTPUT_DIR}:/workspace/out:ro" \
    "${DOCKER_IMAGE}" bash -lc '
      set -euo pipefail
      architecture="$(dpkg --print-architecture)"
      deb="/workspace/out/ros-noetic-xgc2-vrpn-imu-cmd-calibration_${PACKAGE_VERSION}_${architecture}.deb"
      test -f "${deb}"
      apt-get install -y "${deb}"
      /workspace/repo/.xgc2/scripts/check_installed_packages.sh
    '
fi

find "${OUTPUT_DIR}" -maxdepth 1 -type f \
  -name 'ros-noetic-xgc2-vrpn-imu-cmd-calibration_*.deb' -print | sort
