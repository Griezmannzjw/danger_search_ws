# 定位与建图模块审计报告

审计日期：2026-08-16
审计分支：`p0test_lrl`
审计基线：`1f46bc1` 及其上的未提交导航/控制速度调整
审计范围：`danger_search_localization` 及其在 `danger_search_bringup` 中的装配关系

## 1. 结论摘要

当前默认定位链路能够持续发布 `/localization/pose`、`/map` 和
`/mapping/status`，但其平移估计不满足闭环导航所需的观测独立性和连续性。
实测中机器人确实发生了物理移动，但公开位姿在运动期间严重低估，停车后又出现
米级跳变。该问题不是单纯调速度、放宽超时或修改导航规划参数能够解决的。

最严重的问题如下：

1. `/cmd_vel` 被积分成绝对位置，并以 `0.75` 权重直接写入本地里程计；
2. IMU 平移模块 ready 后，代码把 GICP 的平移增量直接清零；
3. 配置中声明的公共位姿跳变保护器没有接入运行节点，米级跳变可直接进入
   `/localization/pose` 和 TF；
4. Hector 当前工作在 `map_with_known_poses=true` 模式，建图位姿来自同一条本地
   里程计，因此不能作为独立全局观测纠正本地平移错误；
5. 现有 GICP 核心单元测试测试的是另一个未被运行节点使用的实现，无法覆盖上述
   命令积分、GICP 清零和停车跳变；
6. README/P0 指南描述的默认 FAST-LIO 链路与实际默认 GICP 链路不一致。

在修复之前，`mapping.ready=true`、`mapping.stable=true` 只能说明消息持续、地图已经
发布且内部失败计数未越界，不能证明位置准确，也不能证明地图与真实机器人位置一致。

## 2. 审计边界与方法

### 2.1 包含范围

- 原始 `/scan` 到 GICP 本地里程计；
- `/trunk_imu`、`/cmd_vel` 和可选腿里程计对平移/航向的约束；
- 本地里程计到 Hector 已知位姿建图；
- `odom -> base`、`map -> odom`、`/localization/pose` 和 `/map` 的发布；
- `/mapping/status`、`/localization/status` 的健康语义；
- 定位建图相关单元测试、启动文件和文档一致性。

### 2.2 不包含范围

- 导航动态障碍、A* 和速度跟踪实现；
- mission 的进门、探索、返航状态机；
- RL 步态控制器内部策略；
- 感知和危险源识别。

这些模块的异常可能暴露定位问题，但不在本文中给出代码修改方案。

### 2.3 运行条件

仿真使用以下约束：

```text
GUI=false
ENABLE_REFEREE_ODOM=0
ENABLE_GROUND_TRUTH=0
POINTCLOUD_USE_GROUND_TRUTH_ODOM=0
控制器先按 2 站立，再按 6 进入 RL /cmd_vel 模式
```

Gazebo `/gazebo/get_model_state` 仅作为测试观察值，用于判断机器人本体是否真实移动；
它没有接入定位、建图、导航或控制节点，也不是建议的生产输入。

## 3. 当前实际生效的数据链

`competition_lio.launch` 默认设置
`use_experimental_fast_lio=false`，从而选择 `local_odometry_source=gicp`：

```text
/scan + /trunk_imu + /cmd_vel
  -> lidar_odometry_node
  -> /localization/raw_pose (odom)
  -> known_pose_backend（只把 frame_id 改为 map）
  -> /localization/hector_pose

/scan
  -> scan_projector
  -> /localization/scan
  -> hector_mapping(map_with_known_poses=true)
  -> /localization/raw_map

/localization/raw_pose + /localization/hector_pose + /localization/raw_map
  -> localization_adapter
  -> /localization/pose + /map + TF + status
```

对应装配证据：

- [`competition_lio.launch`](../src/danger_search_bringup/launch/competition_lio.launch)
  默认进入 GICP 分支；
- [`localization.launch`](../src/danger_search_localization/launch/localization.launch)
  启动 `lidar_odometry_node`、`known_pose_backend` 和 Hector known-pose mapping；
- [`known_pose_backend.py`](../src/danger_search_localization/scripts/known_pose_backend.py)
  不进行定位计算，只复制位姿并把 `frame_id` 改为 `map`；
- [`adapter_node.py`](../src/danger_search_localization/src/danger_search_localization/adapter_node.py)
  将本地位姿直接送入 `HectorGicpFusion.update_local()`。

因此，当前所谓的“GICP + Hector 融合”并没有两个独立的平移观测源。Hector 的地图
插入位姿来自 GICP/IMU/cmd_vel 生成的同一条轨迹；在 known-pose 模式下，
`map -> odom` 被固定为单位修正。

## 4. 运行实测证据

### 4.1 0.40 m/s 完整链路

任务启动后同时采样 Gazebo 本体、`/localization/raw_pose`、
`/localization/pose`、实际发送速度和任务状态。

关键结果：

| 时刻/阶段 | Gazebo 本体位移 | 公开定位位移 | 现象 |
|---|---:|---:|---|
| 启动 | `0.000 m` | `0.000 m` | 基线正常 |
| 约 9.5 s | `2.188 m` | `1.192 m` | 定位明显低估 |
| 约 10.0 s | `2.352 m` | `1.196 m` | 仍在真实移动 |
| 停车过渡 | `约 2.405 m` | `1.052 -> 0.869 -> 0.689 m` | 定位倒退 |
| 静止后 | `约 2.405 m` | `2.699 m` | 公开位姿米级跳变 |

结论：机器人确实移动，`0.40 m/s` 下的位姿增长不是纯粹虚构；但当前定位轨迹既不
准确，也不连续。运动期间误差超过 `1 m`，停车后又一次性补偿到超过真实位移。

### 4.2 排除已知下游干扰后的复测

在不修改文件的情况下排除一个已知下游外参干扰后再次测试，定位仍表现为：

| 阶段 | Gazebo 本体位移 | 公开定位位移 |
|---|---:|---:|
| 运动末段 | `约 2.57 m` | `约 0.84 m` 后回落至 `0.39 m` |
| 静止后 | `约 2.57 m` | 跳变为 `约 2.64 m` |

这说明定位的运动低估和停车跳变不是下游规划单独造成的。

### 4.3 0.30 m/s 物理移动对照

不启动定位和导航，直接向已进入 RL 模式的控制器发送 `0.30 m/s`，Gazebo 本体在
约 5 秒内真实前进约 `1.0 m`。这证明 `0.30 m/s` 并非完全不能驱动机器人。

该结果同时说明：用“公开定位是否增长”判断底盘是否移动会得到错误结论。定位模块
必须使用物理传感器证据估计位移，不能把速度命令是否超过阈值当成位移是否存在的
决定条件。

## 5. 详细问题

### LOC-P0-01：命令速度被当作位置观测，且 GICP 平移被抑制

严重度：P0 / 阻断闭环导航正确性

配置文件 [`default.yaml`](../src/danger_search_localization/config/default.yaml) 当前启用：

```yaml
use_imu_translation_constraint: true
imu_translation_min_command_mps: 0.30
command_translation_weight: 0.75
use_cmd_vel_motion_constraints: true
use_leg_odom_translation_constraint: false
```

运行代码 [`lidar_odometry_node.cpp`](../src/danger_search_localization/src/lidar_odometry_node.cpp)
执行以下逻辑：

1. 新鲜命令速度达到 `0.30 m/s` 后，把运动段标记为 active；
2. 每个 IMU 周期直接累加 `latest_cmd_vel * dt` 到
   `imu_translation_command_position_odom_`；
3. `ApplyImuTranslation()` 用 `0.75` 权重把命令积分位置写入
   `world_from_base_`；
4. 一旦 `imu_translation_ready_` 为真，`ConstrainTranslation()` 直接执行
   `delta->translation().setZero()`，GICP 只剩旋转作用。

实际含义是：

```text
公开平移 = 25% IMU 双积分 + 75% cmd_vel 积分
GICP 平移 = 0（IMU translation ready 后）
腿里程计平移 = 禁用
```

这不是“用命令约束运动方向”，而是用控制输入代替位置观测。机器人被障碍挡住、
打滑、原地踏步或控制器未执行命令时，命令积分仍可增长。反过来，如果真实机器人
移动但实际命令略低于 `0.30` 门槛，命令积分不会启动。

另一个代码缺陷是：`commanded_translation` 的计算没有检查
`use_cmd_vel_motion_constraints_`。因此把 `use_cmd_vel_motion_constraints` 设置为
`false` 也不能关闭命令位置积分；该参数只影响 GICP 的转向/方向约束分支。

影响：

- 速度门槛两侧的定位行为不连续；
- 导航命令与定位进度形成正反馈；
- 位姿无法独立证明机器人是否执行了命令；
- 建图会使用同一条受命令驱动的轨迹，进一步放大地图与真实场景的不一致。

### LOC-P0-02：配置声明的公共位姿跳变保护器没有接入运行节点

严重度：P0 / 米级跳变可直接发布

配置中存在完整的位姿保护参数：

```yaml
pose_position_deadband_m: 0.015
pose_filter_time_constant_s: 0.15
pose_max_linear_speed_mps: 1.0
pose_jump_translation_margin_m: 0.25
pose_recovery_timeout_s: 3.0
```

[`pose_filter.py`](../src/danger_search_localization/src/danger_search_localization/pose_filter.py)
也实现了 `PoseStabilizer`，包括速度门、跳变拒绝和恢复逻辑。但
[`adapter_node.py`](../src/danger_search_localization/src/danger_search_localization/adapter_node.py)
没有导入或实例化 `PoseStabilizer`。

当前 `_gicp_pose_callback()` 只检查：

- frame 是否为 `odom`；
- 四元数是否可解析；
- 数值是否有限；
- 时间戳是否递增。

之后便通过 `HectorGicpFusion.update_local()` 接受位置。`update_local()` 不检查位移
速度、单帧跳变或运动连续性。因此实测中的 `约 0.39 m -> 2.64 m` 跳变能够直接进入：

- `/localization/pose`；
- `odom -> base` TF；
- Hector known-pose 建图位姿；
- 导航当前位姿。

这也意味着 `pose_filter.py` 的单元测试虽然通过，但并不能保护运行链路。

### LOC-P0-03：已知位姿建图与本地里程计形成循环依赖

严重度：P0 / 地图不能独立纠正轨迹

`known_pose_backend.py` 对 `/localization/raw_pose` 只执行深拷贝和 frame 重标记：

```python
mapped = copy.deepcopy(message)
mapped.header.frame_id = self.map_frame
```

Hector 使用该位姿插入扫描，并启用：

```xml
use_tf_pose_start_estimate=true
map_with_known_poses=true
```

适配器在 `mapping_with_known_poses=true` 时调用
`HectorGicpFusion.update_known_global()`，其中明确把 correction 设置为单位变换：

```python
self.correction = Pose2D(0.0, 0.0, 0.0)
```

所以当前链路是：

```text
本地里程计生成位姿
  -> Hector 按该位姿建图
  -> Hector 位姿回到适配器
  -> 适配器确认本地位姿本身正确
```

该结构可以避免第二套 scan matcher 把机器人突然拉走，但代价是失去独立全局修正。
如果本地里程计低估、超调或受命令积分污染，地图会沿错误轨迹建立，Hector 无法指出
该错误。

### LOC-P1-04：停车端点修正是批量修改，缺少连续性约束

严重度：P1 / 直接触发停车后倒退和跳变

运动结束且 IMU 连续静止达到 `stationary_hold_s` 后，代码执行：

```cpp
position.x -= 0.5 * velocity.x * duration;
position.y -= 0.5 * velocity.y * duration;
velocity.setZero();
```

这是对整个运动段累积误差的一次性端点修正。修正量与运动段时长和当前积分速度成
正比，没有单次最大修正、渐进应用或与 GICP/腿里程计的残差校验。

同时，静止分支再次调用 `ApplyImuTranslation()`，把批量修正后的 IMU 位置与仍然累积
保留的命令位置重新混合。运动期间、静止判定过渡期和静止后的输出可能分别来自不同
比例的内部状态，从而出现：

```text
运动时低估 -> 停车后倒退 -> 静止后跳到命令积分端点
```

实测轨迹与该风险一致。要确定每一段跳变分别由哪个内部量贡献，还需要增加内部诊断
话题或结构化日志；现有公共话题不足以拆分 IMU 积分位置和命令积分位置。

### LOC-P1-05：健康状态可在位置错误时继续报告 stable

严重度：P1 / 故障不可观测

GICP 节点在配准 accepted 时根据 fitness 发布较小 covariance；但 IMU translation ready
后，GICP 平移已经被清零，配准健康只说明点云配准的旋转/残差满足阈值，不能说明公开
平移正确。

适配器的 `mapping.ready/stable` 主要依赖：

- pose/map 消息新鲜；
- fusion 已初始化；
- map 已发布；
- covariance 派生的连续失败数未超阈值；
- Hector known-pose 回调持续被接受。

由于命令积分位置本身不会触发 GICP covariance 失败，且 known-pose Hector 与本地轨迹
同源，位置严重低估或停车跳变时仍可能保持：

```text
ready: true
stable: true
lost: false
```

因此现有状态只能表达数据链活性，不能表达定位可信度。

### LOC-P1-06：测试覆盖的 GICP Core 不是运行节点使用的实现

严重度：P1 / 测试产生错误信心

`test_lidar_odometry_core.cpp` 测试
[`lidar_odometry_core.hpp`](../src/danger_search_localization/include/danger_search_localization/lidar_odometry_core.hpp)
中的 `LidarOdometryCore`。该实现包含 submap、对应率、连续失败后重建和恢复确认。

但 CMake 实际编译的 `lidar_odometry_node` 只包含：

```cmake
add_executable(lidar_odometry_node src/lidar_odometry_node.cpp)
```

运行节点没有包含或实例化 `LidarOdometryCore`，而是在
`lidar_odometry_node.cpp` 中维护另一套内联 GICP、IMU、命令积分和状态逻辑。

现有测试没有覆盖：

- `/cmd_vel` 积分；
- `0.30 m/s` 门槛；
- `command_translation_weight`；
- IMU ready 后 GICP 平移清零；
- 运动结束端点修正；
- adapter 真实运行链路中的米级跳变；
- 错误位置下 `mapping.stable` 的语义。

### LOC-P2-07：文档、测试指南与实际默认链路不一致

严重度：P2 / 容易导致错误调试方向

`danger_search_localization/README.md` 仍描述 FAST-LIO 为推荐连续局部里程计，并将旧
GICP 描述为回退模式；`docs/P0_TEST_GUIDE.md` 也包含 FAST-LIO 默认链路和另一组仿真
环境要求。

但当前实际启动文件明确写明：

```xml
use_experimental_fast_lio=false
local_odometry_source=gicp
```

这会造成以下风险：

- 调参时修改 FAST-LIO 参数，但运行中实际是 GICP；
- 误以为公共位姿受 FAST-LIO 连续里程计约束；
- 测试启动环境与当前 RL 控制器/传感器链不一致；
- 无法从测试报告判断当时究竟使用了哪个 odometry source。

## 6. 根因关系

```text
cmd_vel 超过门槛
  -> 命令被积分为位置
  -> IMU translation ready
  -> GICP 平移被清零
  -> raw_pose 主要由命令积分/IMU 双积分决定
  -> 未接入 PoseStabilizer，跳变直接通过
  -> known-pose Hector 按同一错误位姿建图
  -> map/pose/status 相互自洽，但可能与真实机器人不一致
```

这个关系解释了为什么：

- 机器人真实移动时，定位仍可能严重低估；
- 停车后位姿可以一次性补偿；
- `/mapping/status` 看起来健康；
- 下游在运动期间获得不稳定的起点和地图边界。

## 7. 建议修改方案

### 7.1 修改边界

此前约束是不修改从
`/home/ruilinli/localization_stable_fix_v5_stand_safe` 导入的定位代码。本文只做审计，
没有修改该来源或当前定位实现。

但上述 P0 根因位于定位实现内部。仅修改 navigation、mission 或速度参数无法建立可信
位姿。正式修复需要二选一：

1. 明确授权修改当前导入的定位实现，并把修复回合并到其来源；
2. 保持导入代码只读，在工作区新增一个独立、可测试的本地里程计实现并替换运行装配。

不建议复制现有实现后只调阈值，因为命令积分替代物理观测是架构问题，不是参数问题。

### 7.2 第一阶段：恢复观测独立性

目标：公开位移只能由物理传感器证据推进。

要求：

- 删除命令积分对绝对位置的直接贡献；
- `/cmd_vel` 只可作为预测、方向先验或可观测子空间约束；
- `use_cmd_vel_motion_constraints=false` 必须彻底移除命令对估计结果的影响；
- GICP accepted 的平移不能因为 IMU ready 被整体清零；
- 如果使用 IMU 平移，只能作为高频预测，并由 GICP/可靠腿里程计持续校正；
- 障碍阻挡、打滑或原地踏步时，公开位置不得因存在命令而增长。

短期诊断可以关闭 IMU translation constraint，观察纯 GICP 平移，但这不是最终方案。
纯 GICP 在长走廊和稀疏 Mid-360 帧上可能退化，必须通过实测验证，不能直接作为正式
默认值。

### 7.3 第二阶段：统一运行实现和受测实现

选择一个唯一 GICP Core：

- 推荐让 `lidar_odometry_node` 组合并调用 `LidarOdometryCore`；
- IMU、命令先验和可选腿里程计通过明确输入接口进入 Core；
- 删除或停用未被运行节点使用的重复实现；
- 单元测试直接覆盖生产节点调用的状态机和数学核心。

### 7.4 第三阶段：接入连续性保护

- 在 raw local pose 进入 `HectorGicpFusion` 前接入实际使用的跳变门；
- 单帧不合理跳变必须拒绝并把状态降级，而不是静默吸收；
- 禁止超时后直接 `recover()` 到米级新位置；恢复应要求连续多帧物理观测一致；
- 停车端点修正应有单步上限并渐进应用；
- 修正期间 `/mapping/status.stable` 必须为 false；
- 增加诊断输出，分别报告 GICP 增量、IMU 预测、命令先验、腿里程计残差和最终融合量。

### 7.5 第四阶段：重新定义建图可信度

如果继续使用 known-pose Hector：

- 文档和状态必须明确它只是“按外部轨迹建图”，不提供独立全局定位；
- 地图可信度必须继承本地里程计可信度；
- 本地位姿跳变或融合残差异常时冻结新地图，并发布 degraded/lost；
- 不应把 known-pose 回调 accepted 解释成独立的全局校正成功。

若需要长期全局一致性，则必须引入独立的 scan-to-map、回环或其他全局观测，并对修正
设置连续性门和地图版本边界。

### 7.6 第五阶段：统一文档和启动入口

- 明确唯一正式默认：GICP 或 FAST-LIO；
- 启动日志必须打印最终 odometry source；
- P0 指南、定位 README、launch 默认值和测试命令保持一致；
- 测试报告必须记录 source、配置文件和关键融合参数。

## 8. 必需回归测试

### 8.1 单元测试

1. 机器人静止，持续输入 `0.29/0.31/0.40 m/s` 命令，输出位置不得持续增长；
2. 命令为零但点云存在合法平移，输出必须跟随物理点云移动；
3. GICP accepted 平移经过融合后仍保留，不得被 IMU ready 清零；
4. `use_cmd_vel_motion_constraints=false` 时改变命令不影响位置；
5. 运动停止前后输出连续，不允许批量端点跳变；
6. 米级 raw pose 跳变被运行节点实际拒绝，并报告 degraded；
7. 连续物理观测恢复后才重新进入 stable；
8. known-pose mapping 在本地里程计不健康时不得提升新地图版本。

### 8.2 仿真集成测试

Gazebo 本体位置可以继续作为测试 oracle，但不得接入生产节点。

建议至少覆盖：

| 场景 | 验收要求 |
|---|---|
| 静止站立 60 s | XY 漂移不超过 `0.05 m`，无单次跳变 |
| 直行 4 m | 公开前向误差不超过 `0.25 m`，轨迹连续 |
| 命令存在但前方阻挡 | 本体不动时公开位姿不增长 |
| 低于命令门槛的真实移动 | 仍能由物理观测正确估计 |
| 行走后停车 | 2 s 内收敛，最大回退/前跳不超过 `0.05 m` |
| 短时点云失败 | 位姿保持并 degraded，恢复后无补偿跳变 |
| 长走廊弱特征 | 明确降级或由其他物理里程计约束，不得用命令伪造进度 |

### 8.3 状态语义测试

- 注入 raw pose 跳变时 `stable=false`；
- GICP 平移不可用且没有其他物理平移源时 `ready/stable` 不得保持正常；
- 地图只在定位可信时推进版本；
- `/localization/pose`、`odom -> base` 和 Hector 建图位姿在同一时间戳下保持一致；
- status reason 能区分数据过期、配准失败、融合残差异常和恢复中。

## 9. 验收门槛

定位建图修复完成至少需要满足：

1. 速度命令不再直接产生绝对位移；
2. 机器人静止且持续收到非零命令时，公开位置保持不变；
3. 4 m 直行全过程中公开位姿连续，停车时无米级补偿；
4. 位姿误差或观测冲突能够让 `mapping/status` 降级；
5. 运行节点和单元测试使用同一个里程计核心；
6. 默认 odometry source 在 launch、README、P0 指南和运行日志中一致；
7. 生产链路不订阅 Gazebo/referee 真值，真值仅允许作为自动化测试 oracle。

在这些条件满足前，不应把修改导航速度、延长进门超时或放宽 A* 约束视为定位问题的
修复。
