# TurtleBot3 Semantic Navigation (ROS 2)

Standalone extraction of the TB3 Semantic Navigation stack: YOLOv8 detection, LiDAR–camera
localization, persistent semantic landmarks on a SLAM map, and Nav2 goal execution driven
by simple text commands (e.g. `go to person`, `go to bench`).

**Stack:** Ubuntu 22.04 · ROS 2 Humble · Gazebo Classic · TurtleBot3 `waffle_pi` · YOLOv8n · Nav2 (+ SLAM via `nav2_bringup` with `slam:=True`).

**Demo:** [screen recording (Google Drive)](https://drive.google.com/file/d/1xzl9NxP-YBjH4GugFu1Khj3FsQV0fiyI/view?usp=sharing)

## Quick build

> **Before building: download YOLOv8n weights (~6 MB).**
> The `*.pt` files are git-ignored (see `src/tb3_detector/models/.gitignore`),
> so you must fetch them locally **before `./build.sh`** — otherwise
> `detector_node` crashes on startup with `FileNotFoundError: yolov8n.pt`
> and the **"Detector Debug Image"** panel in RViz stays blank ("No Image").
>
> ```bash
> cd ~/TurtleBot3-semantic-navigation
> wget -O src/tb3_detector/models/yolov8n.pt \
>   https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt
> ```

### One-click start

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

### Choose a Gazebo world

The launch ships with **2 built-in worlds** under
`src/tb3_frontier_exploration/worlds/`. Pick one with `world:=<alias>` —
the matching default spawn pose is applied automatically:

| Alias                      | World file                          | Size      | Contents                                                                                              | Auto spawn       |
| -------------------------- | ----------------------------------- | --------- | ----------------------------------------------------------------------------------------------------- | ---------------- |
| `warehouse_models_person`  | `warehouse_models_person.world`     | 6 × 6 m   | **Default.** 5 `person` figures at the four corners + centre. Tuned for the "go to person N" workflow. | `(-1.5,  0.0)`   |
| `warehouse_models`         | `warehouse_semantic_models.world`   | 4 × 6 m   | One `table` (semantic alias `bench`) + one `person`. The original single-target test world.            | `(-1.2, -1.2)`   |

```bash
# Default — five-person room, spawn pose set automatically to (-1.5, 0)
ros2 launch tb3_coordinator full_semantic_nav.launch.py

# The original single-target world
ros2 launch tb3_coordinator full_semantic_nav.launch.py \
  world:=warehouse_models

# Override the spawn manually if you want
ros2 launch tb3_coordinator full_semantic_nav.launch.py \
  world:=warehouse_models_person x_pose:=0.0 y_pose:=-2.0

# Custom .world file (absolute path also accepted; spawn defaults to (-1.2, -1.2))
ros2 launch tb3_coordinator full_semantic_nav.launch.py \
  world:=/path/to/my_custom.world x_pose:=0.0 y_pose:=0.0
```

Bad aliases / missing files are caught at launch time:

```text
FileNotFoundError: [full_semantic_nav] world='foo' could not be resolved.
Tried as alias: .../worlds/foo.
Pass one of the built-in aliases (warehouse_models, warehouse_models_person)
or an absolute path to a .world file.
```

> **Why a 6 × 6 room for `warehouse_models_person`?** TurtleBot3's LDS-01
> LiDAR has a max range of 3.5 m. Sizing the room so the centre is exactly
> 3 m from every wall guarantees scan-matching always sees ≥ 3 walls plus
> visible corners — eliminating the "infinite corridor" ambiguity (and the
> resulting broken occupancy patches) that plagued the earlier 6 × 10 m
> layout.

### Send semantic commands (second terminal, same workspace sourced)

Commands are handled by a **small rule-based parser** in `tb3_query` (`parse_command` in `src/tb3_query/tb3_query/query_core.py`): it lowercases text, strips punctuation, drops a fixed **filler** word list (`go`, `to`, `the`, `please`, `navigate`, …), then resolves **canonical targets** loaded from `semantic_targets.yaml` (only entries with `enabled: true`). There is **no LLM**; extra words are ignored unless they appear before a recognized target token.

**Currently enabled semantic names:** `table`, `person` (see `src/tb3_frontier_exploration/config/semantic_targets.yaml`). Phrase alias: **`bench` → `table`** (so “bench” commands still navigate to the table landmark).

**Selection policy:**

- `go to person`  → the **nearest** observed person (no number means nearest).
- `go to person N` → the specific instance with `object_id == person_N` in semantic memory. Memory assigns ids `person_0, person_1, …` in **observation order** (per [`tb3_memory/memory_core.py`](src/tb3_memory/tb3_memory/memory_core.py)), so which physical figure ends up as which `person_N` depends on the path the bot took. Inspect with `ros2 topic echo /semantic_memory_node/objects` to see the live mapping.
- Recognised number forms: `person 3`, `person_3`, `person3`, `person number 3`, `person no 5`.
- Same rules apply to `table` (in worlds where multiple tables exist).

Examples in the **default world** `warehouse_models_person` (5 persons):

```bash
# Closest person (no number)
ros2 topic pub --once /user_command std_msgs/String "data: 'go to person'"
```

```bash
# Specific id "person_3" (the 4th-observed person — see selection policy)
ros2 topic pub --once /user_command std_msgs/String "data: 'go to person 3'"
```

```bash
# Underscore form
ros2 topic pub --once /user_command std_msgs/String "data: 'navigate to person_2'"
```

```bash
# Glued form (no separator)
ros2 topic pub --once /user_command std_msgs/String "data: 'go to person0'"
```

```bash
# "number" / "no" filler is allowed
ros2 topic pub --once /user_command std_msgs/String "data: 'find person number 4'"
```

Examples in the smaller `warehouse_models` world (1 table + 1 person):

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
