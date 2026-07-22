# `warehouse_semantic.world`

Minimal Gazebo Classic warehouse (≈8 m × 10 m interior) for TurtleBot3 + SLAM + Nav2 + fake semantic navigation.

## Coordinate layout

- **World frame:** Gazebo world origin at the **center** of the floor opening.
- **Axes:** +X east, +Y north, +Z up (Gazebo default).
- **Interior:** roughly **x ∈ [-4, 4]**, **y ∈ [-5, 5]** (8 m × 10 m clear floor inside the walls).
- **Semantic goals** in `config/semantic_goals.yaml` (map frame) are chosen to sit inside this box:
  - table `(2.0, 1.0)`
  - chair `(-1.5, 0.5)`
  - fridge `(3.0, -2.0)`

If **map** is aligned with **world** at startup (robot spawned near origin, SLAM map origin at robot), these goals remain valid.

## Recommended robot spawn

- **Pose:** `(0, 0, 0)` yaw `0` — center of the warehouse, facing +X.
- Matches typical TurtleBot3 empty-world spawn and keeps semantic coordinates easy to reason about.

## Launch with TurtleBot3 (ROS 2 Humble)

**Recommended — package launch** (uses `warehouse_semantic.world` + gzserver/gzclient + spawn):

```bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch tb3_frontier_exploration warehouse_semantic_sim.launch.py use_sim_time:=true x_pose:=0.0 y_pose:=0.0
```

Note: upstream `turtlebot3_gazebo/empty_world.launch.py` **hardcodes** its world path on Humble; it does not accept `world:=...`. Use the launch above or `gazebo_ros` manually:

```bash
WORLD=$(ros2 pkg prefix tb3_frontier_exploration)/share/tb3_frontier_exploration/worlds/warehouse_semantic.world
ros2 launch gazebo_ros gzserver.launch.py world:=$WORLD
# other terminal:
ros2 launch gazebo_ros gzclient.launch.py
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py use_sim_time:=true
ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py x_pose:=0.0 y_pose:=0.0
```

## Notes

- Walls are **static**; thickness **0.2 m**, height **2.5 m** — visible to lidar and safe for collision.
- **Optional later:** add boxes or shelves as separate `<model>` blocks; keep them commented or in a second world file to avoid cluttering v1.

---

# `warehouse_aws_semantic.world`

8 m (X) × 6 m (Y) enclosed room furnished with vendored AWS RoboMaker
warehouse/house models plus authored color-variant props (see
`../models/README.md`), for query-time grounding evaluation
("the blue box on the sofa"). Same wall conventions as the other worlds
(0.2 m thick, 2.5 m high, static, grey). The room is larger than the 6×6
person world to fit furniture; the furniture itself breaks scan-matching
ambiguity, so the LDS-01's 3.5 m range stays sufficient everywhere.

Launch alias: `world:=warehouse_aws` (spawns the robot at `(-3.0, 0.0)`).
Requires `GAZEBO_MODEL_PATH` to include
`share/tb3_frontier_exploration/models` — set automatically by this
package's ament environment hook after a `colcon build` + `source`.

## Object manifest (ground truth for the eval harness)

| name             | model                                  | x    | y     | z    | yaw     |
|------------------|----------------------------------------|------|-------|------|---------|
| shelf_north      | aws_robomaker_warehouse_ShelfD_01      |  0.0 |  2.56 | 0    |  0      |
| clutter_ne       | aws_robomaker_warehouse_ClutteringC_01 |  3.0 |  1.9  | 0    |  0      |
| pallet_jack_east | aws_robomaker_warehouse_PalletJackB_01 |  3.6 | -0.5  | 0    |  1.5708 |
| bucket_nw        | aws_robomaker_warehouse_Bucket_01      | -3.5 |  2.35 | 0    |  0      |
| trash_can_se     | aws_robomaker_warehouse_TrashCanC_01   |  3.2 | -2.5  | 0    |  0      |
| chair_blue       | chair_blue                             | -1.2 |  0.6  | 0    | -0.8    |
| chair_red        | chair_red                              |  1.2 | -0.8  | 0    |  2.4    |
| sofa_orange      | sofa_orange                            |  0.5 | -2.6  | 0    |  1.5708 |
| box_blue_sofa    | box_blue                               |  0.5 | -2.49 | 0.44 |  0      |
| box_blue_floor   | box_blue                               | -1.2 | -2.3  | 0    |  0      |
| box_red          | box_red                                |  1.6 | -2.5  | 0    |  0.4    |
| box_yellow       | box_yellow                             | -1.5 |  1.7  | 0    |  0.3    |
| person_sw        | person_standing                        | -3.0 | -2.2  | 0    |  0.63   |

Notes:

- `box_blue_sofa` sits **on** the orange sofa's seat (seat surface at
  z = 0.44); `box_blue_floor` is the floor-level distractor 1.73 m to
  the west — close enough to share the `sofa_zone` eval camera frame
  (grounding/eval/viewpoints.example.json). `box_yellow` sits in front
  of the shelf so it shares the `shelf_aisle` frame.
- The north-wall shelf is **ShelfD** (3.92 × 0.88 m footprint) — ShelfF
  is the 18 m full-length rack and cannot fit this room.
- Spawn zone `(-3.0, 0.0)` is kept clear: no furniture within 1.2 m.
