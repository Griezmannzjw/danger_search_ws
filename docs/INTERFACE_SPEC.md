# P0 最小可运行接口规范
## 1. 文档范围
- 版本：`v1.1-p0`
- 日期：`2026-07-26`
- 目标：冻结单楼层最小可运行、可检测、可结束和可评分闭环。
```text
任务启动
  -> 当前层定位与占据地图
  -> 地图内自由候选和真实路径查询
  -> 导航执行与安全停车
  -> 基础危险源检测
  -> map 到 world 坐标转换
  -> 任务结束和结果落盘
  -> 官方 evaluator 可解析
```
本规范只要求下文标出的 P0 字段具有约定语义。未要求字段使用类型合法的默认值，P0 消费方不得依赖这些字段决策。
## 2. 模块边界
| 模块 | P0 职责 |
|------|---------|
| localization | 唯一发布当前 `map` 位姿、`map -> odom -> base` TF 和当前层二维占据地图 |
| exploration | 从地图生成合法候选，调用 `make_plan` 校验，并向 `/move_base` 发送或取消目标 |
| navigation | 唯一提供任务导航入口、路径查询和 `MoveBaseAction`，发布 `/danger_search/nav_cmd_vel`，不直接发布 `/cmd_vel` |
| control | 对导航速度进行安全仲裁、超时停车和加速度限制，唯一发布最终 `/cmd_vel` |
| perception | 从允许的 RGB/深度输入发布基础检测和检测器健康状态 |
| mission | 编排任务启停，汇总、过滤、去重和转换检测，唯一写入比赛结果文件 |
| bringup | 通过 `competition.launch` 拉起上述模块并加载统一配置 |
用户只通过 `/danger_search/start` 启动整个任务。`/danger_search/start_exploration` 和 `/danger_search/stop_exploration` 是 mission 编排及模块级控制接口，不是另一套任务入口。
## 3. P0 话题
### 3.1 定位与地图
| 话题 | 类型 | 发布者 | 最低要求 |
|------|------|--------|----------|
| `/tf` | `tf2_msgs/TFMessage` | localization | 唯一、连续提供 `map -> odom -> base`，建议不低于 `10 Hz` |
| `/localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | localization | 有效采集时间、`frame_id=map`、当前位置、合法四元数和数值合法的协方差，建议不低于 `10 Hz` |
| `/map` | `nav_msgs/OccupancyGrid` | localization | 当前单楼层地图，至少 `1 Hz`/变化时发布，或使用 latch 保证新订阅者取得最新地图 |
| `/mapping/status` | `danger_search_common/MappingStatus` | localization | 至少真实提供 `ready`、`stable`、`lost`、`current_floor` 和 `status_reason` |
`/map` 固定使用标准编码：`-1=UNKNOWN`、`0=FREE`、`100=OCCUPIED`。P0 中 `current_floor=0`。地图必须提供可用于选点的已知自由区域；全未知地图不满足该接口语义。
TF 链中 `odom -> base` 应平滑连续，`map -> odom` 承担 SLAM 坐标校正。localization 是该链的唯一发布方，其他导航或定位实现不得重复发布同一变换。P0 不要求真实在线协方差估计；若 `/localization/pose` 使用保守占位协方差，必须在 `/mapping/status.status_reason` 或配置中明确。
localization 只能使用第 7 章允许的传感器输入。`/danger_search/cmd_vel_sent` 或其他速度命令可以用于诊断，但不得作为正式里程计来源，因为命令速度不代表机器人实际运动。
exploration 只有在以下条件同时成立时才能发目标：
- 已收到有效地图和位姿。
- `mapping_status.ready=true`、`stable=true`、`lost=false`。
- 候选位于当前地图范围内且对应已知自由栅格。
- `/move_base/make_plan` 可用并为候选返回非空路径。
- `/move_base` Action server 可用。
### 3.2 导航与控制
| 话题 | 类型 | 发布者 | 最低要求 |
|------|------|--------|----------|
| `/navigation/health` | `danger_search_common/NavigationHealth` | navigation | 至少真实提供 `ready`、`controller_active`、`has_active_goal`、`stuck` 和 `failure_code` |
| `/danger_search/nav_cmd_vel` | `geometry_msgs/Twist` | navigation | 导航期望速度，只供 control 消费 |
| `/cmd_vel` | `geometry_msgs/Twist` | control | 发送给机器人控制器的最终速度，control 是唯一发布者 |
P0 沿用 `/danger_search/nav_cmd_vel` 和 `geometry_msgs/Twist`，不新增 `/navigation/cmd_vel`，也不改为 `TwistStamped`。只能有一套正式 navigation/control 链路对外提供 `/move_base` 和 `/cmd_vel`，禁止多个节点争抢速度输出或重复提供任务导航入口。
取消、超时、失败或 stop 后，navigation 必须停止旧目标的速度输出，control 必须在配置的命令超时内发布零速度。`/navigation/path` 和 `/danger_search/cmd_vel_sent` 可作为诊断接口，但不属于 P0 核心契约。
### 3.3 危险源感知
| 话题 | 类型 | 发布者 | 最低要求 |
|------|------|--------|----------|
| `/danger_detector/detections` | `danger_search_common/DangerSourceArray` | perception | 发布危险源和干扰源检测数组，数组字段为 `dangers` |
| `/danger_detector/status` | `danger_search_common/DetectionStatus` | perception | 至少真实提供 `ready`、`input_fresh` 和 `status_reason` |
P0 中单个 `DangerSource` 必须有效的字段：
| 字段 | P0 语义 |
|------|---------|
| `detection_id` | 单次检测唯一 ID |
| `class_id` | `0=UNKNOWN`、`1=DANGER_RED_SPHERE`、`2=DISTRACTOR_RED_CUBE`、`3=DISTRACTOR_GREEN_SPHERE` |
| `position` | 带有效时间戳和真实 `frame_id` 的三维位置 |
| `floor_id` | P0 固定为 `0` |
| `confidence` | `0..1` 的基础检测置信度 |
| `source_time` | 原始检测时间 |
`track_id`、位置协方差、确认、复核、疑似重复和定位修正版本字段可以保留默认值，P0 消费方忽略其后续阶段语义。`DangerSourceArray.dangers` 允许为空，表示本次消息没有检测结果；空数组不表示观察或探索完成。
### 3.4 任务状态
| 话题 | 类型 | 发布者 | 模式 | 最低要求 |
|------|------|--------|------|----------|
| `/mission/status` | `danger_search_common/MissionStatus` | mission | latch | 真实提供 `mission_state`、`current_floor`、`start_time`、`elapsed_time`、`scored_exploration_time` 和 `finish_reason` |
| `/mission/active` | `std_msgs/Bool` | mission | latch | 任务成功启动后为 true，完成或错误停止后为 false |
P0 的 `mission_state` 至少使用 `IDLE`、`EXPLORING`、`FINISHED` 和 `ERROR`。覆盖率、拓扑债务、房间可见域、剩余前沿和定位修正版本字段不是 P0 硬要求。
## 4. P0 Action 与服务
### 4.1 `/move_base`
- 类型：`move_base_msgs/MoveBaseAction`
- 提供方：navigation
- 消费方：exploration
- 用途：exploration 的唯一任务导航目标入口；不使用 `/danger_search/exploration_goal` 作为执行接口。
- 目标：有效时间戳、`frame_id=map`、地图内已验证自由位姿。
标准 `MoveBaseAction` 使用 actionlib `GoalStatus` 表达终态：
- `SUCCEEDED`：到达目标。
- `ABORTED`：执行失败。
- `PREEMPTED`：目标被取消。
`MoveBaseResult` 本身没有团队自定义结果码。细分原因通过 action status text 或 `/navigation/health.failure_code` 表达，并使用 `/navigation/health.active_goal_id` 与对应 Action GoalID 关联。P0 的 `failure_code` 固定为：
```text
NONE
SUCCEEDED
UNREACHABLE
CANCELED
TIMEOUT
CONTROL_FAILED
SAFETY_STOP
ROBOT_FALLEN
LOCALIZATION_LOST
```
exploration 必须处理成功、失败、取消和目标超时。失败候选不得立即无限重试；stop 时必须取消活动目标。
### 4.2 导航路径查询
| 服务 | 类型 | 提供方 | P0 语义 |
|------|------|--------|----------|
| `/move_base/make_plan` | `nav_msgs/GetPlan` | navigation | 使用与实际导航一致的地图、机器人尺寸和障碍配置判断可达性 |
请求的 start 和 goal 必须位于 `map` 坐标系。只有返回非空 `nav_msgs/Path` 的候选可以发送给 `/move_base`。服务不可用、超时或返回空路径时，exploration 跳过该候选并记录原因。
服务必须根据实际导航使用的地图和障碍判断可达性，不能无条件返回起点到终点的直线。`/move_base/clear_costmaps` 属于后续恢复能力，不是 P0 核心。
### 4.3 exploration 模块服务
| 服务 | 类型 | 提供方 | P0 语义 |
|------|------|--------|----------|
| `/danger_search/start_exploration` | `std_srvs/Trigger` | exploration | 进入探索；输入未就绪时进入等待，不盲目发目标 |
| `/danger_search/stop_exploration` | `std_srvs/Trigger` | exploration | 停止选点、取消活动目标并禁止旧回调继续发目标 |
重复 start/stop 必须返回可预测结果。start 前 exploration 不得发送目标。
### 4.4 mission 服务
| 服务 | 类型 | 提供方 | P0 语义 |
|------|------|--------|----------|
| `/danger_search/start` | `std_srvs/Trigger` | mission | 初始化状态和计时，调用模块级 start，成功后激活任务 |
| `/danger_search/finish` | `std_srvs/Trigger` | mission | 停止探索、冻结计分时间、写入结果并结束任务 |
start 必须遵循：
1. 拒绝重复启动或返回明确的幂等成功。
2. 初始化检测汇总、开始时间和任务状态。
3. 调用 `/danger_search/start_exploration`。
4. 只有模块级 start 成功后才发布 `EXPLORING` 和 `/mission/active=true`。
finish 必须遵循：
1. 调用 `/danger_search/stop_exploration` 并确保活动导航目标已取消。
2. 冻结 `exploration_time`。
3. 完成过滤、去重和 `world` 坐标转换。
4. 原子或等价可靠地写入结果文件。
5. 写入成功后发布 `FINISHED` 和 `/mission/active=false`。
6. 写入失败时返回失败并发布 `ERROR`，不能报告成功完成。
重复 finish 不得重复计时、重复追加检测或破坏已写结果。P0 允许人工调用 finish；自动收敛结束属于后续阶段。`600 s` 是评分阈值，不是自动 finish 的硬截止。
## 5. 坐标契约
### 5.1 机器人定位 TF
```text
map -> odom -> base
```
ROS 坐标约定为 X 向前、Y 向左、Z 向上。所有空间消息必须携带真实 `frame_id` 和有效时间戳。
### 5.2 `map -> world` 结果转换
`map` 是 SLAM 坐标系，不能默认等于 Gazebo `world`。最终结果必须使用三维 `world` 坐标。
若 `map` 与 `world` 不重合，采用以下一种正式方案并在 launch 中固定：
1. localization 发布 `world -> map` TF；或
2. mission 使用允许读取的 `generated_building/team_scene_info.json` 起点建立等价静态变换。
转换来源只能是公开起点和本队定位。不得读取 Gazebo 真值位姿、完整 world、布局元数据或危险源真值建立变换。mission 负责把检测时间对应的源坐标转换到 `world`；转换不可用时不得只修改 `frame_id` 或静默写入 map 坐标。
## 6. 检测汇总与结果文件
mission 是结果文件唯一写入方。P0 最低规则：
- 只有 `class_id=1` 的红色球体候选可以进入危险源结果。
- UNKNOWN、红方块和绿球不得写入危险源列表。
- 同一 `detection_id` 不得重复写入。
- 对空间距离明显重复的检测执行基础合并；阈值从配置读取。
- 只有成功转换为 `world` 的三维位置可以落盘。
结果文件格式：
```json
{
  "exploration_time": 98.76,
  "detected_danger_sources": [
    {"position": [2.34, -1.56, 0.25]}
  ]
}
```
结果路径必须由 launch/YAML 解析为唯一绝对路径，并指向 SimEnv 约定的 `results/detected_danger.json`，不能依赖 roslaunch 时的当前工作目录。
结果格式必须兼容 SimEnv 官方 evaluator。真值文件只能由独立 evaluator 读取，不能进入 localization、exploration、navigation、perception、control 或 mission。
## 7. 官方输入边界
P0 模块可按职责使用以下 SimEnv 输入：
| 输入 | 类型 |
|------|------|
| `/scan` | `sensor_msgs/PointCloud` |
| `/livox/Pointcloud2` | `sensor_msgs/PointCloud2` |
| `/livox/lidar2` | `unitree_guide/CustomMsg` |
| `/trunk_imu` | `sensor_msgs/Imu` |
| `/livox/imu` | `sensor_msgs/Imu` |
| `/real_sense/rgb/image_raw` | `sensor_msgs/Image` |
| `/real_sense/rgb/camera_info` | `sensor_msgs/CameraInfo` |
| `/real_sense/depth/image_raw` | `sensor_msgs/Image` |
| `/real_sense/depth/camera_info` | `sensor_msgs/CameraInfo` |
| `/real_sense/depth/points` | `sensor_msgs/PointCloud2` |
| `generated_building/team_scene_info.json` | 公开场景信息文件 |
正式节点禁止读取：
- `/Odometry_gazebo`
- `/ground_truth/*`
- `generated_building/layout_metadata.json`
- `generated_building/building_config.json`
- `generated_building/scene_manifest.json`
- `generated_building/competition_scene.world`
- `generated_building/danger_truth.json`
- `results/danger_truth.json`
## 8. 启动与参数
`danger_search_bringup/launch/competition.launch` 必须拉起 localization、perception、navigation、exploration、control 和 mission。话题、服务、Action、frame、超时和结果路径必须由 launch/YAML 参数或 remap 配置，不能在节点中另行硬编码同一接口。
P0 至少统一配置：
```yaml
map_topic: /map
pose_topic: /localization/pose
mapping_status_topic: /mapping/status
navigation_health_topic: /navigation/health
detections_topic: /danger_detector/detections
detection_status_topic: /danger_detector/status
move_base_action_name: /move_base
make_plan_service: /move_base/make_plan
start_service: /danger_search/start_exploration
stop_service: /danger_search/stop_exploration
map_frame: map
world_frame: world
result_file: /absolute/path/to/SimEnv/results/detected_danger.json
```
README、launch、YAML、代码读取位置和运行时 ROS graph 中的名称必须一致。
