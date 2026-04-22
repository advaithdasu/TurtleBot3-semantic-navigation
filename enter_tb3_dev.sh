#!/usr/bin/env bash
# Development environment for TurtleBot3 ROS2 workspace.
# Usage: source enter_tb3_dev.sh  (must be sourced, not executed)

# Resolve workspace root from the script's own location so this works
# regardless of where the standalone repo is cloned.
# BASH_SOURCE trick: the directory containing this script is the workspace root.
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. Source ROS2 Humble
source /opt/ros/humble/setup.bash

# 2. Source workspace install
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  source "${WS_ROOT}/install/setup.bash"
else
  echo "Warning: ${WS_ROOT}/install/setup.bash not found. Run ./build.sh first."
fi

# 3. Print environment info
echo "=== TurtleBot3 ROS2 dev environment ==="
echo "ROS_DISTRO:        ${ROS_DISTRO:-<not set>}"
echo "AMENT_PREFIX_PATH: ${AMENT_PREFIX_PATH:-<not set>}"
echo ""

# 4. Confirm workspace is active
echo "Workspace active: ${WS_ROOT}"
echo "Ready for development. Run nodes from this terminal."
echo ""
