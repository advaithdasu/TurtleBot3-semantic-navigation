#!/usr/bin/env bash
# Build tb3_frontier_exploration using system Python (has catkin_pkg from ROS),
# so that colcon/ament do not pick up Miniconda's Python which lacks catkin_pkg.
# Run from: ros2_ws (this directory)
#   ./build_tb3_frontier.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

# Force system Python first (must be before any other setup)
export PATH="/usr/bin:${PATH}"
if ! command -v python3 >/dev/null 2>&1; then
  echo "Error: python3 not found in /usr/bin. Install ROS Humble or set PATH."
  exit 1
fi
echo "Using python3: $(which python3)"

# Remove cached build so CMake does not reuse Miniconda's python path
rm -rf build/tb3_frontier_exploration install/tb3_frontier_exploration

source /opt/ros/humble/setup.bash
# Keep system Python first after ROS setup (ROS setup can change PATH)
export PATH="/usr/bin:${PATH}"

colcon build --packages-select tb3_frontier_exploration "$@"

echo ""
echo "Done. Then run:  source install/setup.bash"
