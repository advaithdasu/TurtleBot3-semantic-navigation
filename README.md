# TurtleBot3 Semantic Navigation (ROS 2)

Standalone extraction of the TB3 Semantic Navigation stack: YOLOv8 detection, LiDAR–camera
localization, persistent semantic landmarks on a SLAM map, and Nav2 goal execution driven
by simple text commands (e.g. `go to person`, `go to bench`).

**Stack:** Ubuntu 22.04 · ROS 2 Humble · Gazebo Classic · TurtleBot3 `waffle_pi` · YOLOv8n · Nav2 (+ SLAM via `nav2_bringup` with `slam:=True`).

## Quick build

```bash
cd ~/TurtleBot3-semantic-navigation
./build.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch tb3_coordinator full_semantic_nav.launch.py
```

Runtime debug overlay:

```bash
ros2 launch tb3_coordinator full_semantic_nav.launch.py use_runtime_debug:=true
```

Send a semantic command (second terminal, same workspace sourced):

```bash
ros2 topic pub --once /user_command std_msgs/String "data: 'go to person'"
```

## Packages

`tb3_detector` · `tb3_localizer` · `tb3_memory` · `tb3_coordinator` (+ `semantic_map_memory_node`) · `tb3_query` · `tb3_nav_adapter` · `tb3_frontier_exploration`.

Canonical semantic name mapping lives in
`src/tb3_frontier_exploration/config/semantic_targets.yaml`.

The YOLOv8n weights ship at
`src/tb3_detector/models/yolov8n.pt`.
