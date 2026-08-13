#!/usr/bin/env bash
# Run inside the sim container: fetch YOLO weights if needed, then build.
#
# This is the shell equivalent of the notebook's build cell
# (tb3_nb.fetch_weights() + tb3_nb.build_workspace()) — use whichever fits
# how you are driving the stack.
set -e

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WEIGHTS="${WS_ROOT}/src/tb3_detector/models/yolov8n.pt"

if [ ! -f "${WEIGHTS}" ]; then
  echo "Downloading yolov8n.pt ..."
  curl -L -o "${WEIGHTS}" \
    https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt
fi

cd "${WS_ROOT}"
./build.sh
