#!/usr/bin/env bash
# Container entrypoint: put ROS 2 (and the built workspace, if there is one)
# on the environment, then exec whatever was asked for.
#
# This runs for every entry into the container — `docker compose up`,
# `docker compose exec sim bash`, and the JupyterLab server started by the
# default CMD. That last one matters most: notebook kernels inherit the
# server's environment, so sourcing here is what makes `import rclpy` and
# `ros2 launch` work inside a cell without any per-notebook sourcing.
set -e

source /opt/ros/jazzy/setup.bash

WS_ROOT=/home/ubuntu/ws
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  source "${WS_ROOT}/install/setup.bash"
else
  echo "[entrypoint] ${WS_ROOT}/install/setup.bash not found —" \
       "run ./docker/setup_ws.sh (or the notebook's build cell) first."
fi

# Best-view frames written by evidence_store_node. Created here so the
# first grounding query does not race the directory into existence.
mkdir -p "${HOME}/.tb3_semantic_nav/evidence"

exec "$@"
