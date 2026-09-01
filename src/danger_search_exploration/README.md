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
- 进入和退出轿厢各限 40 秒，默认门槛穿越最小距离为 `0.85 m @ 0.40 m/s`，使用局部激光避障，
  速度只发布到 `/danger_search/elevator_cmd_vel`；control 以短租约仲裁。进梯完成还要求机器人
  中心沿厅门 `into_yaw` 进入门内至少 `elevator_entry_cabin_side_margin_m=0.43 m`；累计位移
  达标但机身未完全进入时继续前进，提前遇障则返回 `ENTER_FAILED` 并保持门开启。
- 楼层切换后先用已确认厅门平面检查机器人中心位置；若已在目标厅侧至少 `0.05 m`，直接
  进入地图稳定门禁；离梯运动过程中也持续检查该条件，一旦到达厅侧就停止，不再要求达到
  固定总位移。
- `/localization/switch_floor` 成功后要求 epoch 增加，并等待至少两个目标层
  新地图版本、active-map 原子身份、定位健康以及 navigation 完成当前 epoch
  的 costmap reset，再进行 15 秒稳定保持。active map 或 navigation health 在刚切层时短暂
  stale 只会继续等待并节流告警，最终仍由总换层 deadline 返回 `MAP_NOT_STABLE`。探索不直接
  调用清图服务。
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
比赛闭环仍需在 `simenvnew` 正式环境完成。

### 2026-08-31 GUI=false 当前边界

当前三层场景尚未通过自动逐层探索。无 GUI 环境、A1、传感器和门/电梯服务可以启动，
纯逻辑测试为 `44+5+70` 通过；但公开输入下初始厅门差分进入 `FALLBACK`，0 层曾在仍有
前沿和覆盖债务时被记为完成，随后稳定为 `FAILED/floor_transit_unavailable`。测试专用的
厅外出生能够执行普通导航，但长时间运行后出现 `MAP_STALE` 和 costmap 传感器原点越界，
未到达第一次真实换层。完整复现、隔离条件和下一步验收顺序见
`docs/gui_false_multifloor_test_2026-08-31.md`。历史单次 `ELEVATOR_COMPLETE` 只证明状态机
骨架，不代表当前 S3/P3 或三层比赛闭环通过。

### 2026-09-01 复测进展

- 修复固定厅隔离测试中“机器人已位于 nominal approach 附近却被未知栅格拒绝”的问题：
  仅当 `fixed_elevator_hall_enabled=true` 且距离不超过 `plan_tolerance` 时跳过厅前
  `make_plan`，直接进入开门验证；正式候选不使用该旁路。
- seed 42 固定厅隔离中，门槛参数为 `0.85 m @ 0.40 m/s`。修复 `WAIT_STABLE` 瞬时 stale
  误失败后，`0 -> 1` Action 返回成功；另一运行完成到 2 层的真实呼梯/地图切换，并在加载
  动态厅侧判定后成功 `2 -> 0`，达到 `map_epoch=3`。这些运行仍包含固定厅、truth 定位、
  人工 Action 与厅前复位探针，不代表自主三层闭环。
- `GUI=true` 完整重启复测确认旧累计位移条件会在机身未完全进入时关门；改为门平面净空后，
  固定厅 `x=1.150 m` 条件下机器人中心到达 `x=1.670 m`、门内净空 `0.520 m` 才开始关门，
  随后 `0 -> 1` Action 成功达到 `current_floor=1,map_epoch=2`。
- mission 已把 exploration 的 `FAILED` 上卷为 `ERROR`，对应纯逻辑回归已覆盖。
- Unitree RL 姿态异常不必现，但低速、负向、门槛静止重启和原地转向的执行存在运行间差异。
  fixed stand 状态文字也可能与实际趴地不一致，应在切换 RL 前检查 base 高度。CPU 推理存在
  周期超时风险，但固定站立不经过 RL 模型，不能把站立失败归因于未使用 GPU。该 `simenvnew`
  运动可重复性阻塞解决前，不宣称正式 S3/P3 通过。

完整命令和证据见工作区根目录 `command_bringup_flow.md` 与
`docs/gui_false_multifloor_test_2026-08-31.md`。
