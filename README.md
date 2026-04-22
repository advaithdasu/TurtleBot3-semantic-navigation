# TurtleBot3 Semantic Navigation (ROS 2)

Standalone extraction of the TB3 Semantic Navigation stack: YOLOv8 detection, LiDAR–camera
localization, persistent semantic landmarks on a SLAM map, and Nav2 goal execution driven
by simple text commands (e.g. `go to person`, `go to bench`).

**Stack:** Ubuntu 22.04 · ROS 2 Humble · Gazebo Classic · TurtleBot3 `waffle_pi` · YOLOv8n · Nav2 (+ SLAM via `nav2_bringup` with `slam:=True`).

**Demo:** [screen recording (Google Drive)](https://drive.google.com/file/d/1xzl9NxP-YBjH4GugFu1Khj3FsQV0fiyI/view?usp=sharing)

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

### Send semantic commands (second terminal, same workspace sourced)

Commands are handled by a **small rule-based parser** in `tb3_query` (`parse_command` in `src/tb3_query/tb3_query/query_core.py`): it lowercases text, strips punctuation, drops a fixed **filler** word list (`go`, `to`, `the`, `please`, `navigate`, …), then resolves **canonical targets** loaded from `semantic_targets.yaml` (only entries with `enabled: true`). There is **no LLM**; extra words are ignored unless they appear before a recognized target token.

**Currently enabled semantic names:** `table`, `person` (see `src/tb3_frontier_exploration/config/semantic_targets.yaml`). Phrase alias: **`bench` → `table`** (so “bench” commands still navigate to the table landmark).

Examples that stay within this parser:

```bash
ros2 topic pub --once /user_command std_msgs/String "data: 'go to person'"
```

```bash
ros2 topic pub --once /user_command std_msgs/String "data: 'please could you navigate to the table'"
```

```bash
ros2 topic pub --once /user_command std_msgs/String "data: 'I need you to approach the bench'"
```

```bash
ros2 topic pub --once /user_command std_msgs/String "data: 'find a person for me'"
```

## Packages

`tb3_detector` · `tb3_localizer` · `tb3_memory` · `tb3_coordinator` (+ `semantic_map_memory_node`) · `tb3_query` · `tb3_nav_adapter` · `tb3_frontier_exploration`.

Canonical semantic name mapping lives in
`src/tb3_frontier_exploration/config/semantic_targets.yaml`.

The YOLOv8n weights ship at
`src/tb3_detector/models/yolov8n.pt`.
