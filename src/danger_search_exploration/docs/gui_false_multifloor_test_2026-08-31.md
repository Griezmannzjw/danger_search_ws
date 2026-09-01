# GUI=false 多楼层能力边界测试（2026-08-31）

## 目标与结论

目标是在官方 `simenvnew`、`GUI=false` 下验证机器人能否从公开出生点自动进入建筑、完成每层探索、自动乘电梯换层并最终结束。

本轮结论为 **未通过**。`GUI=false` 环境、机器人生成、传感器、楼栋控制服务和算法栈可以启动；探索纯逻辑测试也全部通过。但当前端到端链路无法稳定到达第一次真实换层，更不能证明逐层完成：

1. 正常公开输入运行时，初始电梯厅门运动差分失败，进入 `FALLBACK`，之后无可用厅候选。
2. 从公开出生点启用自动入楼时，机器人向入口反方向运动并在建筑外被误判为 0 层完成。
3. 在仅限 `simulation_truth` 的真值隔离诊断中把机器人出生点放到电梯厅外后，普通探索可以持续下发导航目标，但机器人最终离开地图边界，地图转为 `MAP_STALE`，仍未触发换层。
4. exploration 进入稳定 `FAILED/floor_transit_unavailable` 后，mission 仍保持 `EXPLORING`，失败没有上卷为任务恢复或 `ERROR`。

因此，历史固定场景中曾达到 `ELEVATOR_COMPLETE` 只能证明状态机骨架存在，不能代表当前三层场景已满足 S3/P3 或无 GUI 自动逐层探索验收。

## 测试环境

- 日期：`2026-08-31`
- 官方环境：`/home/langan/simenvnew`
- 算法工作区：`/home/langan/danger_search_ws`
- 场景：当前已有 `team_scene_info_v1`，公开 `served_floors=[0,1,2]`
- 仿真：`GUI=false`、`ENABLE_GROUND_TRUTH=0`、`ENABLE_REFEREE_ODOM=0`
- 控制器：`UNITREE_RL_DEVICE=cpu`，输入 `2` 后等待 `15 s`，再输入 `6`
- 算法隔离配置：`simulation_truth` 只用于排除连续 GICP 对运动与换层状态机的干扰；不作为正式通过证据
- 结果文件：写入 `/tmp/gui_false_multifloor_20260831/detected_danger.simulation_truth.json`，未覆盖官方结果

登录 shell 的 `python3` 指向 Miniconda Python `3.14.6`，缺少 ROS OpenCV；测试必须显式使用 `/usr/bin/python3` `3.8.10`。catkin 安装后的节点 shebang 已固定为 `/usr/bin/python3`，ROS launch 不受该问题影响。

## 分层测试结果

### 1. 探索纯逻辑测试

```text
test_simple_frontier.py: 44 passed
test_entrance_boundary.py: 5 passed
test_multifloor.py: 70 passed
exploration_planner.py: py_compile passed
```

这证明 WFD、入口边界和 TransitFloor 状态机的 mock 路径通过，不证明 Gazebo 中的厅门识别、四足运动、地图边界和真实换层通过。

### 2. GUI=false 环境启动

- Gazebo server 在无 GUI 下启动。
- A1 在约 `6 s` 内成功生成。
- `/scan`、RealSense、`/set_door_state`、`/call_elevator` 可用。
- `junior_ctrl` 能加载 CPU 策略并进入 RL `/cmd_vel` 模式。

结论：本轮没有证据表明 `GUI=false` 会关闭或破坏换层所需服务；它不是当前首要阻塞。

### 3. 公开场景、正常初始厅发现

关键观测：

```text
sim t=70.650: initial elevator discovery fell back:
               door motion was absent or ambiguous
sim t=101.176: floor 0 complete, mode=bounded_unreachable,
               remaining_frontiers=4, coverage_debt=3
随后三次: floor transit failed [NO_HALL]: no elevator hall candidate
```

对应 `/exploration/status`：

```text
initial_hall_discovery=FALLBACK
elevator_candidate_count=0
completed_floors=[0]
known_grid_ratio≈0.231
state=FAILED
reason=floor_transit_unavailable
```

0 层在仍有前沿和覆盖债务时被记为完成，不能视为“成功完成该层探索”。

### 4. 自动入楼诊断

启用 `entry_enabled=true` 后，机器人由公开出生点 `x≈-3.0 m` 运动到 `x≈-6.83 m`，即远离入口。随后探索在建筑外得到 `remaining_frontier_count=0` 并把 0 层记为完成，固定厅候选因不可达被拒绝。

该结果首先指向 mission 入楼指令方向、控制器速度符号或入口朝向合同不一致。探索侧不应通过放宽结束条件掩盖此问题。

### 5. 电梯厅外出生的真值隔离诊断

为隔离入楼和厅发现，只在 `simulation_truth` 测试中使用离线场景几何，把机器人出生在 0 层电梯厅外，并启用固定厅候选。该配置严禁用于正式规划。

普通探索一度正常运行：

```text
map_revision=145
observation_goal_count=9
active move_base goals observed
```

约 `482 s` 后：

```text
robot pose≈(-5.09, 7.18)
remaining_frontier_count=18
coverage_debt_count=12
blacklisted_cell_count=226
exploration reason=input_stale
mapping status=MAP_STALE
move_base: sensor origin is out of map bounds
```

因此本轮仍未进入 `TO_HALL`，不能实证 `OPEN_CURRENT -> ENTER -> SWITCH_FLOOR -> EXIT -> WAIT_STABLE` 的 Gazebo 路径。

## 当前能力边界

已确认：

- `GUI=false` 官方环境和门/电梯服务可启动。
- 三层公开楼层图可被探索读取。
- 探索、多楼层状态机和服务超时的纯逻辑测试通过。
- 四足在保守切换时序下可保持直立并执行部分导航目标。

未确认或失败：

- 公开输入下可靠绑定初始电梯厅。
- 从公开出生点正确自动入楼。
- 单层覆盖的可靠收敛；当前存在建筑外或低覆盖误完成。
- 长时间导航时地图与机器人保持同一有效边界。
- 第一次真实换层及目标层地图恢复。
- exploration 失败向 mission 的恢复/异常传播。
- 正式 GICP 下三层闭环；在 `simulation_truth` 尚未通过前不应进入该验收。

## 下一步顺序

1. **先冻结运动方向合同**：从公开出生点执行入楼，验收机器人到入口的有符号距离连续下降；同时记录 `/danger_search/entry_cmd_vel`、`/cmd_vel` 和 Gazebo 位姿。责任模块为 mission/control/Unitree 控制接入。
2. **修复地图边界一致性**：导航期间机器人和传感器原点不得离开 active map/costmap；`MAP_STALE` 或连续越界必须取消目标并进入明确恢复。责任模块为 localization/navigation。
3. **阻止假楼层完成**：建筑外、`remaining_frontier_count>0`、覆盖债务未清或依赖 `MAP_STALE` 时不得写入 `completed_floors`。探索侧需要增加入口完成门控和持久不可达证据门控。
4. **补足厅发现诊断**：状态至少报告开/关门有效扫描数、变化 bin 比例、最大变化簇真实点数、拟合线段长度和唯一性拒绝原因，避免只留下 `absent or ambiguous`。
5. **传播不可恢复失败**：`FAILED/floor_transit_unavailable` 应让 mission 进入有限恢复或 `ERROR`，不能永久保持 `EXPLORING`。
6. **再做电梯阶段验收**：先在 `simulation_truth` 的测试专用固定厅配置中逐阶段通过 `TO_HALL` 到 `WAIT_STABLE`，再移除固定厅和真值出生点，验证公开输入的门运动绑定。
7. **最后切回正式 GICP**：只有自动入楼、单层可靠收敛和 simulation-truth 换层都通过后，才验证正式定位下的三层自动完成。

正式 S3/P3 通过条件必须是：从公开出生点启动、不使用固定厅或真值出生点、至少完成一次真实换层、每个 `served_floor` 都有独立且可信的完成证据，最终 mission 结束；单次 `ELEVATOR_COMPLETE`、单层误完成或测试旁路均不算通过。

## 2026-09-01 追加复测

本节追加新证据，不改变 2026-08-31 的历史结论。

### 启动与单层闭环修正

- 登录 shell 的 `python3` 实际指向 `/home/langan/miniconda3/bin/python3` 3.14，缺少 ROS
  OpenCV。联调前固定执行
  `export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH`，并确认
  `command -v python3` 为 `/usr/bin/python3`。
- `navigation.launch` 现将 `move_base/cmd_vel` remap 到
  `/danger_search/move_base_cmd_vel`，并启动 `navigation_command_mux.py` 统一输出
  `/danger_search/nav_cmd_vel`。navigation 的 112 项回归通过。
- exploration 稳定发布 `state=FAILED` 后，mission 现停止 exploration，并以
  `exploration_failed:<reason>` 进入 `ERROR`；纯逻辑测试覆盖了失败 reason 的解析。
- 自定义出生 `(-3.0,-0.8,pi)` 的正式 GICP 运行可完成短程入楼控制，但该朝向背离建筑，
  8 个有效前沿全部位于入口 guard 后方。该出生覆盖不再作为正式入楼证据；正式测试必须
  使用公开 `robot_start`。

### 正式公开出生阻塞

官方默认出生约为 `(0,-3.2,yaw=1.5708)`。输入 `2` 固定站立时 IMU 约为
`roll=-0.1°、pitch=3.0°`；输入 `6` 后稳定到约
`roll=11.5°、pitch=-19.4°`。正式 profile 的 15°恢复阈值因此不能解除启动安全门。

运行中执行 `8` reset 虽可短暂改变安全状态，但会破坏定位与任务参考；入口控制和
`/navigation/traverse_portal` 仍没有新增有效位移，因此 reset 不是合法流程或修复。

### 固定厅真实服务隔离链

只在 `simulation_truth` 使用 seed 42 的隔离出生
`(x=0.5,y=2.6,z=0.6,yaw=0)` 和固定厅
`(x=1.15,y=0.0,into_yaw=0)`。固定厅 nominal approach 为 `(0.35,0)`，距机器人约
0.35 m。此前该点因局部未知栅格被误判 `UNREACHABLE_HALL`；现仅对
`fixed_elevator_hall_enabled` 且距离不超过 `plan_tolerance` 的隔离候选跳过厅前
`make_plan`，正式候选仍执行全部可达性检查。

复测 Action：

```text
/danger_search/transit_floor target_floor=1 exit_to_hall=true
OPEN_CURRENT_START
CAPTURE_OPEN_SCAN
VALIDATE_CLOSE_START
CAPTURE_CLOSED_SCAN
REOPEN_CURRENT_START
ENTER
CLOSE_CURRENT_START
CALL_TARGET_START
SWITCH_FLOOR_START
EXIT
```

门开、关、重开和激光差分均通过，固定候选被标记为 `validated=true`。Action 最终返回：

```text
success=false
reached_floor=1
map_epoch=2
failure_code=EXIT_FAILED
message=elevator crossing timed out
```

失败后的状态快照为：

```text
safety_stop=false
posture_safety_reason=stable_recovery
mapping: floor=1 epoch=2 ready=true stable=true
floor maps: floor 0 and floor 1 both retained
navigation: floor=1 epoch=2 ready=true
```

ROS 图确认速度链存在：

```text
/exploration -> /danger_search/elevator_cmd_vel -> /control
/control -> /cmd_vel -> /unitree_gazebo_servo
```

门槛参数从 `1.40 m @ 0.40 m/s` 缩短为 `1.00 m @ 0.40 m/s` 后，最佳运行成功完成真实呼梯、
楼层切换和 0/1 层地图隔离。Unitree 不执行负向 `linear.x`，所以反向离梯超时。探索侧新增
厅门平面判定：目标层机器人中心若已在厅侧至少 `0.05 m`，直接进入 `WAIT_STABLE`，不再
强制倒车；该分支已有纯逻辑测试。重复端到端运行仍可能在入梯后触发
`excessive_tilt` 安全取消，说明门槛动力学未达到可重复验收。

### 追加结论

截至 2026-09-01：

- 单层探索相关纯逻辑、导航速度链和 mission 失败传播已修正并有回归覆盖。
- `GUI=false` 下真实门服务、开关门差分、呼梯、`SWITCH_FLOOR` 和独立楼层地图已实证。
- 固定厅旁路只属于 truth 隔离诊断，不能流入正式模式。
- 官方公开出生三层闭环仍被 Unitree RL 切换后的姿态和运动执行阻塞；`simenvnew` 未被修改。
- 最佳运行完成一次真实楼层切换，但没有可重复完成 `DONE`、二层恢复探索或继续到 2 层，
  因此仍不满足 S3/P3 正式验收。

后续复测应首先确认官方控制器在默认出生和 `GUI=false` 下能稳定执行正向 `/cmd_vel`；满足
后再按 `OPEN_CURRENT -> ENTER -> CALL_TARGET -> SWITCH_FLOOR -> EXIT -> WAIT_STABLE`
顺序采集每层 map epoch/version，并完成至少 `0->1->2` 和返回路径。标准命令已整理到
工作区根目录 `command_bringup_flow.md`。
