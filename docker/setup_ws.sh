#!/usr/bin/env bash
# Run inside the sim container: fetch YOLO weights if needed, then build.
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
