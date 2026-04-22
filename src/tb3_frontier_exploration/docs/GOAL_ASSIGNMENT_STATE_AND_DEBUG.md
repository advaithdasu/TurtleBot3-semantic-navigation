# goal_assignment_node — 状态机与调试说明

## 状态机

```
                    +------------------+
                    |       IDLE      |
                    | (无目标进行中)   |
                    +--------+--------+
                             |
        有有效 frontier + 服务器就绪
        选最近且通过过滤的 frontier，发送目标
                             v
                    +------------------+
                    |    NAVIGATING    |
                    | (已发目标，等结果)|
                    +--------+--------+
                             |
        +---------------------+----------------------+
        |                     |                      |
   SUCCEEDED             ABORTED/CANCELED          timeout
   (清除 last_failed)    (记录失败点，避免再选)    (取消目标，记录失败点)
        |                     |                      |
        v                     v                      v
                    +------------------+
                    |       IDLE       |
                    +------------------+
```

- **IDLE**：当前没有正在执行的导航目标；每个周期可尝试选择并发送下一个目标。
- **NAVIGATING**：已向 Nav2 发送目标，等待结果或超时。
  - 若 `now - goal_sent_time >= goal_timeout`，则主动取消该目标、记录为失败点并回到 IDLE。
  - 收到 result（SUCCEEDED / ABORTED / CANCELED）后回到 IDLE；若为失败或取消，同时记录该目标位置为“上次失败目标”。

## 实现逻辑摘要

1. **过滤“太近”的 frontier**  
   与机器人距离 < `min_frontier_distance` 的质心不参与“最近”选择，避免发无用目标。

2. **避免重复发失败目标**  
   记录最近一次失败/取消/超时的目标坐标；选择时排除与“上次失败目标”距离 < `failed_goal_avoidance_radius` 的 frontier。  
   仅在一次目标 **SUCCEEDED** 后清除该失败点（可选策略：也可在发送新目标后清除）。

3. **超时处理**  
   在 NAVIGATING 下，若从 `goal_sent_time` 起经过 `goal_timeout` 秒仍未结束，则调用 `async_cancel_goal` 取消当前目标、记录为失败点并回到 IDLE，下次周期会重新选目标。

4. **无有效 frontier**  
   经过“太近”和“靠近失败点”过滤后，若没有任何候选，则视为当前无有效目标；  
   使用 `exploration_complete_log_interval` 限速打印“No valid frontiers — exploration complete”，避免刷屏。

5. **一次只发一个目标**  
   仅在 IDLE 时才可能发送新目标；NAVIGATING 期间只做超时检查或等待 result。

## 建议参数

| 参数 | 类型 | 建议值 | 说明 |
|------|------|--------|------|
| `min_frontier_distance` | double | 0.5 | 忽略与机器人距离小于此值的 frontier（米）。 |
| `failed_goal_avoidance_radius` | double | 1.0 | 不选择与“上次失败目标”距离小于此值的 frontier（米）。 |
| `goal_timeout` | double | 60.0 | 单目标最大执行时间，超时则取消并记为失败（秒）。 |
| `exploration_complete_log_interval` | double | 5.0 | “探索完成”类日志的最小间隔（秒）。 |
| `rate` | double | 1.0 | 主循环/定时器频率（Hz）。 |

## 调试日志建议

- **启动时**：已打印 `min_frontier_dist`、`failed_avoid_radius`、`goal_timeout`，便于确认配置。
- **日常运行**：  
  - 用 **INFO**：收到目标接受/拒绝、选中的目标索引与坐标、结果（SUCCEEDED/ABORTED/CANCELED）、超时取消、“exploration complete” 限速提示。  
  - 用 **DEBUG**：每次收到的 frontier 数量、被“太近”或“靠近失败点”过滤掉的候选、TF 失败、记录失败点坐标。  
- **排查“不选目标”**：  
  - 将 log level 设为 DEBUG：  
    `ros2 run tb3_frontier_exploration goal_assignment_node --ros-args --params-file ... --log-level goal_assignment_node:=DEBUG`  
  - 查看是否大量 “[select] Frontier [i] skipped: too close” 或 “near last failed goal”，以及 “[select] No valid frontier after filters”。
- **排查超时**：  
  - 确认 `goal_timeout` 是否过小或环境是否导致规划/执行很慢；  
  - 观察 “[timeout] Goal stalled for Xs (limit Ys), canceling” 是否频繁出现。

## 日志标签含义

- `[frontiers]` — 订阅到的 frontier 数据。
- `[TF]` — 机器人位姿查询。
- `[select]` — 候选过滤与最近选择。
- `[goal]` — 最终选中的目标。
- `[action]` — 目标被接受/拒绝。
- `[result]` — 导航结果。
- `[timeout]` — 超时取消。
- `[failed]` — 记录失败点（DEBUG）。
- `[exploration]` — 无有效 frontier 的限速提示。
- `[idle]` — 处于 IDLE 时的原因（DEBUG，如 action server 未就绪）。
