#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"

for script in .xgc2/scripts/*.sh scripts/*.sh; do
  bash -n "${script}"
done

PYTHONPYCACHEPREFIX="${TMPDIR:-/tmp}/xgc2-vrpn-imu-cmd-pycache" python3 -m py_compile \
  scripts/*.py \
  src/vrpn_imu_cmd_calibration/*.py \
  test/*.py \
  .xgc2/scripts/xgc2_artifact_manifest.py

python3 - <<'PY'
import pathlib
import xml.etree.ElementTree as ET

root = ET.parse("package.xml").getroot()
assert root.findtext("name") == "vrpn_imu_cmd_calibration"
assert root.findtext("version") == "0.2.0"
assert pathlib.Path("config/default.yaml").read_text(encoding="utf-8").startswith(
    "schema_version: xgc2.vrpn_imu_cmd_calibration/profile/v1"
)
PY

test "$(awk '/^version:/ {print $2; exit}' .xgc2/product.yml)" = "0.2.0-1"
grep -q '^id: xgc2-vrpn-imu-cmd-calibration-ros1$' .xgc2/product.yml
grep -q '^    focal: 0.2.0-1$' .xgc2/product.yml
grep -q '^  imu: /imu/data_raw$' config/default.yaml
grep -q 'install(PROGRAMS scripts/run_vehicle_calibration_e2e.sh' CMakeLists.txt
grep -q 'catkin_install_python(PROGRAMS' CMakeLists.txt
if grep -n 'catkin_install_python(PROGRAMS' -A8 CMakeLists.txt \
  | grep -q 'run_vehicle_calibration_e2e.sh'; then
  echo "shell E2E driver must not be installed through catkin_install_python" >&2
  exit 1
fi
if grep -R -n -E 'Mapping\[[^]]+\] \| None|Dict\[[^]]+\] \| None' \
  src scripts test >/dev/null; then
  echo "Python 3.10-only union syntax leaked into the Noetic package" >&2
  exit 1
fi
if grep -n -E '^  - (ros-noetic-estimator|ros-noetic-unicycle|ros-noetic-vrpn-.*estimator)' \
  .xgc2/product.yml >/dev/null; then
  echo "optional estimator/controller composition leaked into hard Debian Depends" >&2
  exit 1
fi
grep -q '^Depends: python3-numpy, python3-rospkg, python3-yaml, ros-noetic-geometry-msgs, ros-noetic-rosbag, ros-noetic-rosbash, ros-noetic-rosgraph, ros-noetic-roslaunch, ros-noetic-rospack, ros-noetic-rospy, ros-noetic-sensor-msgs, ros-noetic-std-msgs$' \
  .xgc2/scripts/package_debs.sh

MANIFEST_TEST_ROOT="$(mktemp -d)"
trap 'rm -rf "${MANIFEST_TEST_ROOT}"' EXIT
MANIFEST_TEST_ARCH="$(dpkg --print-architecture)"
mkdir -p \
  "${MANIFEST_TEST_ROOT}/package/DEBIAN" \
  "${MANIFEST_TEST_ROOT}/debs" \
  "${MANIFEST_TEST_ROOT}/manifests"
printf '%s\n' \
  'Package: xgc2-vrpn-imu-cmd-manifest-contract' \
  'Version: 0.2.0-1' \
  'Section: misc' \
  'Priority: optional' \
  "Architecture: ${MANIFEST_TEST_ARCH}" \
  'Maintainer: XGC2 <dev@xiaokang.ink>' \
  'Description: XGC2 VRPN IMU cmd calibration manifest contract test' \
  >"${MANIFEST_TEST_ROOT}/package/DEBIAN/control"
dpkg-deb --build \
  "${MANIFEST_TEST_ROOT}/package" \
  "${MANIFEST_TEST_ROOT}/debs/xgc2-vrpn-imu-cmd-manifest-contract_0.2.0-1_${MANIFEST_TEST_ARCH}.deb" \
  >/dev/null
python3 .xgc2/scripts/xgc2_artifact_manifest.py build \
  --deb-dir "${MANIFEST_TEST_ROOT}/debs" \
  --output-dir "${MANIFEST_TEST_ROOT}/manifests" \
  --product xgc2-vrpn-imu-cmd-calibration-ros1 \
  --product-version 0.2.0-1 \
  --distribution focal \
  --architecture "${MANIFEST_TEST_ARCH}" \
  --source-sha 0000000000000000000000000000000000000000 \
  --ci-run-id compliance \
  --ci-workflow ci \
  --ci-workflow-ref refs/heads/main

MANIFEST_TEST_ROOT="${MANIFEST_TEST_ROOT}" python3 - <<'PY'
import json
import os
from pathlib import Path

paths = list((Path(os.environ["MANIFEST_TEST_ROOT"]) / "manifests").glob("*.json"))
assert len(paths) == 1
manifest = json.loads(paths[0].read_text(encoding="utf-8"))
assert set(manifest) == {
    "schema", "product", "source_sha", "version", "distribution",
    "architecture", "ci", "created_at", "debs",
}
assert manifest["schema"] == "xgc2.build-artifact.v1"
assert manifest["product"] == "xgc2-vrpn-imu-cmd-calibration-ros1"
assert manifest["version"] == "0.2.0-1"
assert set(manifest["ci"]) == {"run_id", "workflow", "workflow_ref"}
assert len(manifest["debs"]) == 1
deb = manifest["debs"][0]
assert set(deb) == {
    "file", "package", "version", "architecture", "sha256", "size",
}
assert deb["package"] == "xgc2-vrpn-imu-cmd-manifest-contract"
assert deb["version"] == "0.2.0-1"
assert len(deb["sha256"]) == 64 and deb["size"] > 0
PY

echo "ROS1 VRPN IMU cmd calibration product compliance passed"
