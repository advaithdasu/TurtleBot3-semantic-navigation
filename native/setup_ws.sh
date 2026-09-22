#!/usr/bin/env bash
# Run after ./native/install_env.sh, and again any time source changes:
# fetches the YOLO weights if missing, then builds the workspace against
# the active RoboStack conda env instead of apt's /opt/ros/jazzy.
#
#   micromamba activate tb3
#   ./native/setup_ws.sh
set -euo pipefail

if [ -z "${CONDA_PREFIX:-}" ] || [ -z "${ROS_DISTRO:-}" ]; then
  echo "error: activate the env first: micromamba activate tb3" >&2
  exit 1
fi

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WEIGHTS="${WS_ROOT}/src/tb3_detector/models/yolov8n.pt"

if [ ! -f "${WEIGHTS}" ]; then
  echo "Downloading yolov8n.pt ..."
  curl -L -o "${WEIGHTS}" \
    https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt
fi

cd "${WS_ROOT}"
echo "=== Building ROS2 workspace (env: ${CONDA_PREFIX}) ==="
# Unlike build.sh (which forces /usr/bin/python3 to dodge a conda Python
# it assumes is a stray, apt-world problem), here the conda env's own
# Python IS the correct interpreter — it's the one ros-jazzy-cv-bridge and
# colcon-common-extensions were built against.
colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE="$(command -v python3)" "$@"

echo
echo "=== Build SUCCESS ==="
echo "Then: source install/setup.bash"
