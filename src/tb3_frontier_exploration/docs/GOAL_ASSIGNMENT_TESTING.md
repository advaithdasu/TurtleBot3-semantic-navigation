# goal_assignment_node — 测试说明

## 依赖

- `/frontiers`（geometry_msgs/PoseArray，map 坐标系）由 `frontier_detection_node` 发布
- `/odometry/filtered`（nav_msgs/Odometry）由 robot_localization 或仿真提供
- TF：`map` → `base_link`（由 SLAM + odom → base_link 提供）
- Nav2 的 `navigate_to_pose` action server 已启动

## 构建

```bash
cd ~/TurtleBot3-semantic-navigation
source /opt/ros/humble/setup.bash
colcon build --packages-select tb3_frontier_exploration
source install/setup.bash
```

## 启动顺序（与 TB3_EXPLORATION_RUN_ORDER 一致）

1. 启动仿真或真机（TurtleBot3）
2. 启动 slam_toolbox（发布 `/map` 和 `map` → `odom`）
3. 启动 Nav2（提供 `navigate_to_pose`）
4. 启动 frontier_detection_node（发布 `/frontiers`）
5. 启动 goal_assignment_node（带参数文件）

```bash
ros2 run tb3_frontier_exploration goal_assignment_node --ros-args --params-file src/tb3_frontier_exploration/config/params.yaml
```

或通过 launch 一并加载 `params.yaml` 后只运行节点。

## 日志检查

- **Received frontiers: N** — 收到 N 个 frontier 质心
- **Selected goal [i]: (x, y) dist=d m** — 当前选中的最近 frontier 及距离
- **Action goal accepted** / **Action goal rejected** — 目标被 Nav2 接受或拒绝
- **Goal finished: SUCCEEDED** / **ABORTED** / **CANCELED** — 导航结果

## 简要验证

```bash
# 查看 /frontiers 是否有数据
ros2 topic echo /frontiers --once

# 确认 navigate_to_pose 存在
ros2 action list | grep navigate_to_pose

# 查看 goal_assignment 日志
ros2 run tb3_frontier_exploration goal_assignment_node --ros-args --params-file src/tb3_frontier_exploration/config/params.yaml
```

成功时：机器人会依次朝最近的 frontier 移动，到达后自动选择下一个目标并发送。
