# danger_search_exploration

P1 分层前沿探索和电梯换层执行模块。它只决定目标并调用 navigation/control，不直接发布
最终 `/cmd_vel`。

## 前沿规划

- 以 `map_epoch/map_version` 为缓存键，用 OpenCV 连通域/WFD 只遍历机器人所在的已知
  可达区域，避免每周期全图 Python BFS。
- 前沿簇的目标按“到当前簇”的观察距离放在已知区内侧；到任意未知边界和占用障碍的
  footprint 净空分别检查，避免相邻前沿错误地互相筛空，再调用 `/move_base/make_plan`。
- 目标按路径长度、路径最小净空和信息量排序；Action 终态按 goal epoch 隔离，失败采用
  显式 backoff，不允许迟到结果污染新目标。
- 恢复失败按原始 stuck pose 写入 trap blacklist，并在安全净空连续恢复后解除；不存在
  move_base 结束后再要求额外物理位移的自锁门控。
- 每层独立保存失败目标、trap blacklist、地图版本和完成状态。地图稳定且连续 10 秒没有
  可达 frontier 后才完成当前层。

`/exploration/complete` 仅在公开 `served_floors` 全部完成时发布。它不是任务完成；mission
还必须执行跨层返航、起点返航和静止验证。

## 楼层拓扑

只读取 `team_scene_info_v1` 的公开 `public_scene.elevators` 和 door IDs。以 served floors
建立楼层/电梯图，并选择到未探索楼层的最少换乘路径；不会使用 `current_floor+1`，也不会
把“不服务下一层”解释为建筑探索完成。

楼层身份以 `/call_elevator` 成功响应中的 `current_floor` 为准。正式 GICP 的高度不参与
身份判定；结果 z 使用配置的 `floor_height_m=2.6`。

## TransitFloor 状态机

本节点提供 `/danger_search/transit_floor` (`TransitFloorAction`)，供探索和 mission 返航
共同调用：

```text
TO_HALL -> OPEN_CURRENT -> VERIFY_HALL -> ENTER -> CLOSE_CURRENT
-> CALL_TARGET -> SWITCH_FLOOR -> EXIT -> WAIT_STABLE
```

`/danger_search/start` 后先执行一次 `INITIAL_HALL_DISCOVERY`。机器人保持静止并确认控制输出
连续为零，采集 5 帧初始开门全向扫描；随后通过公开 `SetDoorState` 关闭当前层电梯门，稳定
0.75 秒后再采集 5 帧。算法只使用几何一致的 360° 光束；为适配 Livox 稀疏投影，允许在
同一变化簇内桥接最多 5 个无返回 bin，但拟合仍只使用真实变化点。随后拟合关门后新增的
0.9–1.8 m 线段并经 TF 转到 map；采样期间位移超过 0.03 m、偏航超过 1°、变化簇有歧义或服务失败
都会恢复开门并转入被动回退。成功绑定标记为 `source=door_motion`、`validated=true`，并保持
门关闭直到实际换层。

- 厅导航超时按 `make_plan` 路径长度计算，上限 180 秒。
- `CallElevator`/`SetDoorState` 是强类型同步服务，每阶段超时 40 秒；调用在工作线程执行，
  取消、超时或新 action generation 会忽略迟到响应。
- 进入和退出轿厢各限 20 秒，默认门槛穿越速度为 `0.40 m/s`，使用局部激光避障，速度只发布到
  `/danger_search/elevator_cmd_vel`；control 以短租约仲裁。
- `/localization/switch_floor` 成功后要求 epoch 增加，并等待至少两个目标层
  新地图版本、active-map 原子身份、定位健康以及 navigation 完成当前 epoch
  的 costmap reset，再进行 15 秒稳定保持。探索不直接调用清图服务。
- `TO_HALL` 接受同一 floor/epoch 内已提交且新鲜的 active-map 快照；其正版本可以
  暂时落后最新 `MappingStatus.map_version`，但不能为零、超前或来自其他 floor/epoch。
  这与普通探索使用同一个 `map_context_is_committed` 合同，避免异步话题到达顺序造成误停。
  换层开始前仍要求 mapping ready/stable；厅导航开始后，恢复转向可能暂时使状态降级，
  此时只要未 lost/transitioning、位姿与地图新鲜且上下文仍已提交，就继续当前目标。
- 换层阶段的 recovery 事件不写入当前层的探索 trap blacklist；电梯厅候选由换层状态机
  自己有限重试，避免一个失败靠近点污染其余候选或下一次换层。
- 任何失败都取消普通导航、停止局部控制并返回固定失败码；候选索引只递增一次。
- 当前层完成事件只持久化和记录一次。换层失败冷却期间保持
  `WAITING/floor_transit_retry_backoff`；重试耗尽后稳定保持
  `FAILED/floor_transit_unavailable`，不会继续刷相同的 floor-complete 日志。

主动差分失败时，被动候选要求 4–12 m²、两边均为 1.8–3.6 m、三条非门边墙体支撑率
至少 70%、唯一且居中的 0.9–1.8 m 门洞、充足的内部已知自由空间，以及可规划的厅外
approach 点。候选按围合度 30%、尺寸 20%、门宽 20%、居中 10%、内部自由 10%、跨版本
稳定性 10% 计分；普通候选至少跨 3 个地图版本持续 2 秒且总分不低于 0.75。排序依次为
门运动验证、综合分、路径长度，机器人直线距离只作最终同分项。主动绑定在相同
floor/epoch/map-load identity 内允许地图版本增长；换层开门后直接 `ENTER`，不重复关门验证。

`/exploration/status` 的 JSON 还包含 `elevator_candidate_count`、`elevator_binding`
（坐标、来源、置信度和验证状态）以及 `initial_hall_discovery`。

固定失败码：

```text
NO_HALL UNREACHABLE_HALL SERVICE_UNAVAILABLE SERVICE_REJECTED
SERVICE_TIMEOUT ENTER_FAILED FLOOR_MISMATCH MAP_NOT_STABLE EXIT_FAILED
CANCELED STALE_EPOCH
```

## ROS 接口

订阅：`/map`（仅核对）、`/mapping/active_map`（规划权威）、`/localization/pose`、
`/mapping/status`、`/navigation/health`、`/navigation/recovery_event`、`/localization/scan`、
`/danger_search/safety_stop` 和 `/danger_search/cmd_vel_sent`。

调用：`/move_base`、`/move_base/make_plan`、
`/call_elevator`、`/set_door_state`、`/localization/switch_floor`。

发布：`/exploration/status`、`/exploration/complete`、观察目标/黑名单诊断话题、
`/localization/mapping_pause` 和 `/danger_search/elevator_cmd_vel`。

提供：`/danger_search/start_exploration`、`/danger_search/stop_exploration` 和
`/danger_search/transit_floor`。

参数见 `config/default.yaml`；正式启动由 `danger_search_bringup/competition.launch` 统一
传入 `competition_mode`、`multifloor_enabled`、`localization_backend` 和公开 scene 文件。

## 验证

```bash
catkin_make run_tests_danger_search_exploration -j4
catkin_test_results --all build/test_results/danger_search_exploration
```

测试覆盖 WFD 性能、扫描过滤默认合同、多楼层拓扑、连通墙/房间凹口/多候选、服务拒绝和
超时、迟到响应、候选重试及 WAIT_STABLE。真实电梯运动、Unitree 进出轿厢和 12-seed
比赛闭环仍需在 SimEnv 正式环境完成。
