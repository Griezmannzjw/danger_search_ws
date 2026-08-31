# P1 多楼层比赛接口规范

版本：`v2.0-p1`；适用入口：`danger_search_bringup/competition.launch`。

## 1. 模式和输入边界

正式模式必须同时满足：

```text
competition_mode=true
multifloor_enabled=true
localization_backend=gicp
ENABLE_REFEREE_ODOM=0
ENABLE_GROUND_TRUTH=0
POINTCLOUD_USE_GROUND_TRUTH_ODOM=0
```

允许输入为官方激光、IMU、RGB-D 话题，`/call_elevator`、`/set_door_state` 和公开的
`team_scene_info.json`。禁止读取 `/Odometry_gazebo`、`/gazebo/link_states`、
`/ground_truth/*`、layout、building config、scene manifest、world 和危险源真值。
`gazebo_truth` 只能由 `simulation_truth.launch` 在 `competition_mode=false` 中使用，
且其结果永远不具备正式验收资格。

## 2. 模块所有权

| 模块 | 所有权 |
|---|---|
| localization | 唯一发布 `map -> odom -> base`；维护分层地图和 `map_epoch` |
| navigation | 唯一提供 `/move_base`、`make_plan` 和有界恢复；只输出 nav cmd |
| control | 唯一发布 `/cmd_vel`；仲裁 safety/elevator/navigation |
| exploration | WFD 选点、每层完成状态、served-floor 调度和 TransitFloor server |
| perception | 红球 RGB-D 三维候选与绑定楼层/epoch 的短时跟踪 |
| mission | 唯一任务终态、跨层返航、结果坐标转换和文件写入 |
| bringup/preflight | 统一参数并对正式 ROS graph 执行 fail-closed 检查 |

## 3. 定位与建图

`MappingStatus` 必须真实填写：

- `ready`、`stable`、`lost`、`current_floor`
- `transitioning`：换层时为 true，且此时 `stable=false`
- `map_epoch`：每次真实切图/重置单调增加
- `floor_z_m`：`floor_id * floor_height_m`
- `floor_maps[]`：各层地图版本与更新时间

localization 必须将同一份占据图同时发布为 `/map` 和锁存的
`/mapping/active_map: FloorOccupancyGrid`；后者必须填写完全匹配的
`floor_id/map_epoch/map_version`。过渡期不得重发旧层地图。
消费者对 `floor_id/map_epoch` 做严格相等检查；同 epoch 内，已核验快照的
`map_version` 允许因三条独立 ROS 连接短暂落后于 MappingStatus 最新版本，但必须为正、
单调且不得领先。不得用瞬时版本完全相等作为持续建图时的规划门禁。

`SwitchFloor.srv`：

```text
Request:  string transition_id, int32 target_floor
Response: bool success, uint64 map_epoch, string message
```

相同 transition ID 与目标的重试必须返回同一 epoch。切换时保存旧层地图，保持电梯井
平面 x/y/yaw，重建 GICP 扫描参考，切换/恢复目标层地图。目标层至少出现两个新地图版本、
定位健康且连续稳定后才可导航。

## 4. 导航和速度

普通目标只经 `/move_base`。所有 Action 结果、恢复事件和 plan generation 必须绑定 goal
ID/epoch，旧 goal 的迟到结果不能修改新 goal 的 attempt 或 blacklist。

`NavigationHealth` 同时发布 `current_floor/map_epoch/map_version/transitioning`。navigation
只在 `/map` 与 active-map 签名一致、MappingStatus 身份匹配，且当前 epoch 的
`/move_base/clear_costmaps` 成功后才设 `ready=true`。其中 `map_version` 是通过该门禁的
已提交快照版本，不是尚未匹配 envelope 的 MappingStatus 最新值。navigation 是换层
costmap reset 的唯一所有者。

control 固定优先级：

```text
safety stop > /danger_search/elevator_cmd_vel 短租约 > /danger_search/nav_cmd_vel
```

control 是 `/cmd_vel` 唯一发布者。DWA 与 control 的加速度联合合同为 x/y/yaw
`3.0/2.0/8.0`；每轴禁止单周期跨零反向。Unitree 恢复插件只在 move_base 内执行有限的
完整 footprint 扫掠，goal/cancel/safety epoch 改变会立即使旧恢复失效。

## 5. TransitFloor Action

```text
Goal:     int32 target_floor, bool exit_to_hall
Feedback: string phase, int32 current_floor, float32 progress, uint64 map_epoch
Result:   bool success, int32 reached_floor, uint64 map_epoch,
          string failure_code, string message
```

状态顺序为：

```text
TO_HALL -> OPEN_CURRENT -> VERIFY_HALL -> ENTER -> CLOSE_CURRENT
-> CALL_TARGET -> SWITCH_FLOOR -> EXIT -> WAIT_STABLE
```

厅导航时限由路径长度计算且不超过 180 s；门/电梯服务各 40 s；进、出轿厢各 20 s；
稳图 15 s。阻塞服务在工作线程执行，取消/超时后用 generation 丢弃迟到结果。任何失败都
取消普通 goal、停止电梯租约速度、确认最终输出归零，并只推进一次候选索引。

固定失败码：

```text
NO_HALL UNREACHABLE_HALL SERVICE_UNAVAILABLE SERVICE_REJECTED
SERVICE_TIMEOUT ENTER_FAILED FLOOR_MISMATCH MAP_NOT_STABLE EXIT_FAILED
CANCELED STALE_EPOCH
```

## 6. 探索和任务完成

前沿检测按 `map_epoch/map_version` 缓存，只遍历机器人可达已知区域。每层独立保存
frontier、失败 goal、trap blacklist、完成状态。地图稳定且连续 10 s 无可达 frontier 后
才完成该层。`/exploration/complete=true` 只在全部公开 served floors 完成后发布。

Mission 保存 `home_floor=0` 和起点位姿。覆盖完成、FinishMission、ReturnHome 或任务超时
都先停止探索；若不在 0 层，调用同一个 TransitFloor 回 0 层并出轿厢，再发送 home goal。
成功条件为位置误差 `<=0.5 m`、yaw `<=20 deg`、导航不活动且实际下发速度连续 2 s 为零。
只有此后才能冻结计时、写结果并进入 FINISHED；失败进入 ERROR。

## 7. 感知一致性

`DangerSource` 必须填写 `floor_id`、`map_epoch` 和
`localization_correction_version`。换层、地图不稳定、epoch 不匹配或定位修正版本改变时，
观测不得进入本轮轨迹；修正版本改变会清理旧的短时关联。Mission 只接受与当前
`MappingStatus` 同 floor/epoch 的确认红球。

## 8. 结果文件

Mission 是结果文件唯一写入方。任务开始原子写空 `RUNNING` 文档；任务结束原子写：

```json
{
  "exploration_time": 98.76,
  "coordinate_frame": "world",
  "mission_status": "FINISHED",
  "run_profile": "formal",
  "localization_backend": "gicp",
  "official_eligible": true,
  "detected_danger_sources": [
    {"position": [2.34, -1.56, 0.25]}
  ]
}
```

必要字段仍为 `exploration_time` 和 `detected_danger_sources[].position`。`auto` 帧策略只读
公开 scene contract：声明 world 时对起点相对点执行 `Rz(yaw0)*p+t0`；否则输出
`start_relative`。零目标必须是空数组，不能保留上次任务结果。
正式验收还必须通过 `validate_result.py --official`；真值 profile 会被强制拒绝。

## 9. 正式预检

预检至少验证：强类型服务可导入且 ROS 服务类型匹配、服务存在、传感器/状态话题类型、
`/mapping/active_map` 与 `/navigation/health` 的消息类型、`map -> base` TF、结果目录可写、
正式参数一致、没有禁止订阅/文件参数，以及 `/cmd_vel` 唯一发布者为 `/control`。失败时
required 节点退出，整个正式 launch 终止。
