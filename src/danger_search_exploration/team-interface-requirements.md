# 探索规划模块接口需求说明

本文用于探索规划负责人和接口架构、定位建图、导航控制、危险源感知及结果汇总负责人对齐接口。目标是在不读取裁判真值、不耦合具体算法实现的前提下，使探索模块完成走廊拓扑发现、房间可见性覆盖、多楼层切换、定位修正后的重验证、失败恢复和任务结束判断。

本文日期为 `2026-07-26`。本文是探索侧的分阶段能力需求，不是团队现有接口的替代规范。名称、已有消息类型和模块边界优先沿用框架负责人维护的 `danger_search_ws/docs/INTERFACE_SPEC.md` 及工作空间实际定义，并通过 ROS 参数或 remap 配置；尚缺能力优先在现有命名空间扩展。命名兼容不能替代必需功能，消息语义、时间戳、坐标系、状态枚举、失败原因和阶段验收仍需完整实现。

## 1. 分阶段交付路线

接口按“先形成可评分闭环，再逐步提高召回、降低虚警和支持多楼层”的顺序交付。后续阶段继承前一阶段接口；标记为后续阶段的接口不得阻塞早期节点启动。

### 1.1 P0：最小可运行与可评分闭环

目标是用 DFS、简单前沿或预设局部搜索跑通：

```text
定位与当前层地图
  -> 简单探索目标
  -> 导航执行
  -> 危险源检测与结果汇总
  -> results/detected_danger.json
  -> 官方 evaluator 可读取并给出分数
```

P0 不承诺完整探索、多楼层召回或低虚警，只证明团队框架能够启动、移动、接收检测、结束任务并生成格式正确的结果。即使只获得时间项或少量正确检测分数，也属于这一阶段的有效验收；不得把单场得分当作算法性能基线。

P0 必需接口：

| 接口 | 最小语义 | P0 可接受简化 |
|------|----------|---------------|
| `/tf` | 连续 `map -> odom -> base` | 暂不要求回环修正事件 |
| `/localization/pose` | 当前 `map` 位姿、时间戳 | 协方差可先给保守固定值并标注占位 |
| `/map` | 当前楼层 `OccupancyGrid`，`-1/0/100` | 只维护当前楼层 |
| `/mapping/status` | `danger_search_common/MappingStatus`：`ready/stable/lost/current_floor/floor_maps[]` | `current_floor=0`，单层只发布一项 `FloorMapInfo` |
| `/move_base` | 接收、取消目标并返回成功/失败 | 简单全局规划器可以接受，但必须避障并能停车 |
| `/move_base/make_plan` | 判断候选是否有路径 | 可只返回基础路径长度 |
| `/navigation/health` | `danger_search_common/NavigationHealth`：`ready/stuck/has_active_goal/failure_code` | 进度可暂不精确 |
| `/danger_detector/detections` | `danger_search_common/DangerSourceArray`：类别、位置、坐标系、时间戳、置信度 | 跟踪和复核可后补 |
| `/danger_detector/status` | `danger_search_common/DetectionStatus`：检测器是否就绪、输入是否新鲜 | 只需基础健康状态 |
| `/danger_search/start`、`/danger_search/finish` | `std_srvs/Trigger`，由 mission 提供的任务起止入口 | 可手动触发结束 |
| `/danger_search/start_exploration`、`/danger_search/stop_exploration` | `std_srvs/Trigger`，由 exploration 提供，供 mission 编排 | 不作为用户启动整个任务的入口 |
| `/mission/status`、`/mission/active` | `danger_search_common/MissionStatus` 与 latched `std_msgs/Bool` | 覆盖率摘要可为空 |
| 结果落盘 | 官方 JSON 格式和 `world` 坐标 | 可在 finish 时一次写入 |

P0 探索侧允许：DFS、最近可达前沿、固定旋转观察、有限失败重试和手动结束。P0 明确不依赖：结构障碍层、通行风险层、垂直通道、`surface_cloud`、房间可见域、异视角复核和 RL 策略接口。

P0 验收：

1. 一条启动命令可拉起框架，所有 P0 接口在约定超时内 ready。
2. 机器人至少完成一个非零距离目标，取消目标后停止旧速度。
3. 能接收零个或多个检测并调用 finish。
4. `results/detected_danger.json` 可被官方 evaluator 解析，探索时间非负，坐标为三维 `world` 坐标。
5. 不读取真值话题、布局文件或危险源真值。

### 1.2 P1：可靠单楼层简单探索

目标是在 P0 上把 DFS/简单前沿变成可自动收敛的单楼层基线，还不要求房间遮挡优化。

新增或强化：

- `/localization/status`：真实协方差、`TRACKING/DEGRADED/RELOCALIZING/LOST`、修正版本和跳变事件。
- `/mapping/status.floor_maps[]`：真实地图版本和更新时间。
- `/move_base/clear_costmaps`、完整导航结果码和目标超时。
- `/navigation/path`：当前路径或剩余路径，便于判断进展。
- 探索状态：剩余前沿、当前目标、冷却/黑名单和自动结束原因。
- 地图版本或定位修正变化后取消并重验证旧目标。

P1 验收以单层前沿清空、有限恢复、自动结束和结果落盘为准。走廊拓扑债务可以在探索模块内部产生，不要求相邻模块新增专用拓扑接口。

### 1.3 P2：房间可见性覆盖与低虚警复核

目标是从“地图被探索”升级为“危险源可能区域被有效观察”，提高召回并控制重复和干扰物虚警。

新增接口：

- `/mapping/obstacle_structure`：房间边界和内部遮挡的派生结构，引用源地图版本。
- `/danger_detector/observation_done`：关联观察任务的稳定检测完成事件。
- 感知能力参数或 `/danger_detector/capabilities`：有效 FOV、可靠距离、稳定时长和扫描角速度。
- 检测轨迹字段：稳定 `track_id`、位置协方差、确认状态、复核请求、疑似重复和定位修正版本。
- 任务状态增加房间可见域、复核债务和不可见死角摘要。

P2 才启用自适应房间视点、遮挡后方补扫、分段旋转和异视角复核。结构障碍接口缺失时允许保守多视点降级，但不得宣称达到确认的房间覆盖基线。

### 1.4 P3：多楼层与门/电梯闭环

目标是发现不完整的垂直通道，逐步确认并安全完成换层，而不是要求建图首次看见入口就给出完整楼梯。

新增接口：

- `/mapping/current_floor` 和可查询的分楼层地图/版本。
- `/mapping/vertical_connection_observations`：支持 `TENTATIVE/PARTIAL/CONFIRMED/INVALIDATED`、未知远端和逐步补全。
- `/set_door_state`、`/call_elevator` 的明确成功、失败、超时和幂等语义。
- 门、电梯运行状态或等价查询能力。
- 换层期间定位/地图 `stable=false`，换层后楼层、TF 和地图恢复一致。

P3 中探索模块负责假设融合、确认债务、跨层拓扑边、楼梯/电梯效用选择和执行经验；相邻模块只发布实际观测和执行状态。

### 1.5 P4：风险增强、效率优化与策略升级

目标是在已经可完整运行的基础上提高安全性、效率和随机场景稳定性。

可选或增强接口：

- `/mapping/traversability`：坡度、台阶、门槛、低净空和轿厢边缘等二维占据无法表达的风险。
- 更准确的路径进度、等待时间、通行耗时、失败概率和定位风险统计。
- RViz marker、统一诊断、rosbag/离线回放和结构化训练日志。
- `PolicyObservation/PolicyAction` 适配和 `RLTaskPolicy`；规则策略继续作为回退。
- `/mapping/surface_cloud` 仅用于结构提取调试、回放或显式降级，不提升为探索核心硬依赖。

P4 优化不得改变正式输入红线，也不得绕过导航、安全校验和确定性任务状态机。

### 1.6 评分导向

根据 `docs/evaluation.md`，重复输出会成为虚警，召回率 `<=60%` 时识别项为零分，虚警率 `<=10%` 才得满分，时间在 `600 s` 内均为满分。P0 只验证“能得分”；P1/P2 优先建立单层召回与低虚警能力；P3 才解决多楼层完整召回；P4 在不损害召回和虚警的前提下优化时间。

探索模块只负责高层目标选择与任务状态机，不实现 SLAM、危险源识别、四足底层控制或最终检测结果去重。

## 2. 总体接口原则

- 优先使用 ROP1 标准消息、`move_base` Action 和标准规划服务。
- 所有数据必须包含有效时间戳；所有空间数据必须显式提供 `frame_id`。
- 坐标关系通过 `/tf` 转换，不允许依赖隐含坐标约定。
- 楼层索引统一从 `0` 开始；跨楼层数据必须携带楼层编号或可靠高度。
- 话题和服务名称必须可通过 ROS 参数配置，不在算法核心中硬编码。
- 状态接口应提供数据版本或最近更新时间，便于识别停更和乱序数据。
- 地图坐标发生回环优化、重定位或其他离散修正时，必须显式发布修正版本和事件，不得静默移动历史目标。
- 服务、Action 和状态必须区分成功、失败、超时、取消与不可用。
- 正式接口不得使用 Gazebo 真值位姿、完整布局或危险源真值。
- 尚未完成的模块应提供同语义 mock，保证探索模块可独立联调。
- 现有框架已经冻结的接口优先复用：任务总控使用 `/danger_search/*` 服务和 `/mission/*` 状态，导航使用 `/move_base` 与 `/navigation/*`，感知使用 `/danger_detector/*`，定位建图使用 `/localization/*`、`/mapping/*` 和 `/map`。
- 沿用现有执行边界：navigation 发布 `/danger_search/nav_cmd_vel`，control 完成安全仲裁、超时停车和加速度限制，并作为 `/cmd_vel` 的唯一最终发布者；探索不直接发布任一速度话题。

### 2.1 数据分层与唯一职责

接口按“事实层—派生层—假设层—任务层”划分，避免多个话题重复承担地图职责：

1. **事实层 `/map`**：当前楼层截至当前时刻已观测到的二维占据事实，只表达 `UNKNOWN/FREE/OCCUPIED`。墙、楼梯踏步、门槛或轿厢边界若能被传感器观测，可按普通几何进入地图；`/map` 不编码“这里是楼梯”、连接楼层或尚未观测到的出口。
2. **派生层**：`/mapping/obstacle_structure` 只提供房间分割和视线遮挡所需的功能性结构；`/mapping/traversability` 只提供占据值无法表达的运动风险。两者必须引用生成它们的地图版本，不另建一套空间真值。
3. **假设层 `/mapping/vertical_connection_observations`**：发布从当前可见数据推断出的垂直通道局部观测，允许不完整、低置信和随后撤销。它不是完整建筑拓扑，也不要求首次发现入口时知道另一端位置或连接楼层。
4. **任务层**：探索模块融合历次假设，维护稳定 ID、拓扑债务、已确认连接、执行经验和不可达原因。建图模块不负责判断某个分支是否已服务，也不负责用未观测信息补全通道。

同一几何可以在不同层出现，但语义不能重复：例如楼梯踏步在 `/map` 中是占据/自由几何，在通行层中是风险，在假设层中才带 `STAIR` 类型和局部入口。消费者按 `source_map_version` 关联它们，不通过坐标近似猜测版本。

逻辑话题沿用 `/mapping/*` 只是为了兼容当前框架命名，不等于所有派生算法都由定位建图负责人实现：

| 产品 | 最终职责 | 可选生产者 |
|------|----------|------------|
| 位姿、TF、`/map`、地图版本 | 定位建图 | SLAM/建图模块 |
| 普通目标可达性和实际路径 | 导航 | `/move_base/make_plan` |
| 结构障碍派生层 | 环境结构适配器 | 建图后处理、独立节点或探索 ROS 壳层 |
| 通行风险派生层 | 导航/环境结构适配器 | 代价地图、地形分析节点；可缺省 |
| 垂直通道局部假设 | 环境结构适配器 | 点云/深度结构提取节点或探索 ROS 壳层 |
| 假设融合、拓扑债务、跨层边确认 | 探索规划 | `exploration_planner` |

框架负责人当前只需为派生产品保留消息、配置和 mock 插槽；具体生产者在模块联调时确定。不得因为话题位于 `/mapping` 命名空间，就默认要求 SLAM 模块实现房间语义或完整楼梯识别。

### 2.2 完整度、置信度与撤销

结构或垂直通道不得使用单个 `confidence` 暗示“已经完整”。至少分开表达：

```text
observation_state: TENTATIVE | PARTIAL | CONFIRMED | INVALIDATED
geometry_completeness: float32       # 0..1，只表示几何观测完整度
classification_confidence: float32   # 0..1，只表示类型判断置信度
traversability: UNKNOWN | CAUTION | OPEN | BLOCKED
```

- 首次看到疑似楼梯入口时可发布 `TENTATIVE`，只带当前层局部入口和已观测轮廓。
- 看到踏步或平台但未知出口时发布 `PARTIAL`，`connected_floor` 和远端入口保持未知，不得猜测。
- 实际通过、从两端观测或获得充分几何证据后才升级为 `CONFIRMED`。
- 新地图推翻旧判断时发布同一 `observation_id` 的 `INVALIDATED`，不能直接让记录消失。
- 探索模块对部分假设创建“继续观察/确认入口”债务，而不是立即执行跨层动作。

## 3. 接口阶段索引

下表用于快速确认某个接口最早在哪个阶段阻塞验收。详细字段以第 4～10 章为准。

| 接口或能力 | 首次必需阶段 | 缺失时影响 |
|------------|--------------|------------|
| TF、位姿、当前层 `/map`、基础地图状态 | P0 | 无法产生合法探索目标 |
| `move_base`、`make_plan`、基础导航健康 | P0 | 无法移动或判断目标可达 |
| 检测、任务起止、结果落盘和 `map/world` 坐标契约 | P0 | 无法形成可评分结果 |
| 定位修正、地图版本、完整失败码、恢复与自动结束 | P1 | 只能做人工监管的烟雾测试 |
| 结构障碍、观察完成、感知能力和检测复核语义 | P2 | 不能确认房间有效视觉覆盖和低虚警闭环 |
| 分层地图、垂直通道增量观测、门和电梯状态 | P3 | 不能完成可靠多楼层探索 |
| 通行风险层、精细代价、诊断、回放和策略训练接口 | P4 | 不阻塞基础任务，但限制安全性与效率优化 |
| `/mapping/surface_cloud` | P4 可选 | 仅影响结构提取调试或显式降级 |

## 4. 定位、地图与环境结构产品

### 4.1 接口产品

| 名称 | 类型 | 阶段 | 频率建议 | 关键语义 |
|------|------|------|----------|----------|
| `/tf` | `tf2_msgs/TFMessage` | P0 | 不低于 `10 Hz` | 连续提供 `map -> odom -> base`；`base` 与机器人配置一致 |
| `/localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | P0 | 不低于 `10 Hz` | 当前位姿、协方差、时间戳和 `frame_id` |
| `/localization/status` | `danger_search_common/LocalizationStatus` | P1 | `5 Hz` 以上及状态变化时 | 跟踪状态、漂移告警、位姿跳变、重定位事件和坐标修正版本 |
| `/map` | `nav_msgs/OccupancyGrid` | P0 | 不低于 `1 Hz` 或地图变化时 | 仅表达当前楼层；`-1=未知、0=自由、100=占用` |
| `/mapping/current_floor` | `std_msgs/Int32` | P3 | 变化时立即发布 | 当前楼层，索引从 `0` 开始；P0/P1 固定为 0 |
| `/mapping/status` | `danger_search_common/MappingStatus` | P0 基础、P1 完整 | `1 Hz` 以上及状态变化时 | `ready`、`stable`、`lost`、当前楼层、各楼层地图版本和最近更新时间 |
| `/mapping/obstacle_structure` | 团队结构化数组或矢量层 | P2 | 与源地图版本同步或结构变化时 | 仅表达建筑边界和视线遮挡的功能性分类，不重复占据状态 |
| `/mapping/traversability` | 团队栅格消息；能力不足时可暂缺 | P4 可选 | 与源地图版本同步 | 仅表达占据图无法表达的运动风险；普通自由/占用仍以 `/map` 为准 |
| `/mapping/vertical_connection_observations` | 团队结构化数组消息 | P3 | 新观测、状态变化或撤销时 | 楼梯、电梯和未知通道的增量局部假设；允许未知远端和不完整几何 |

`/mapping/surface_cloud` 不再列为探索规划必需接口。若定位建图或独立结构提取模块内部需要点云来生成上述派生层，应在模块内部消费；只有在 P4 调试、录包或明确降级方案中才作为可选接口暴露。

### 4.2 `/mapping/status` 最小字段

```text
header
ready: bool
stable: bool
lost: bool
current_floor: int32
floor_maps[]:
  floor_id: int32
  map_version: uint64
  last_update: time
status_reason: string
```

要求：

- 初始化未完成时 `ready=false`。
- 换层、地图重定位或地图坐标调整期间 `stable=false`。
- 定位不可用或持续异常时 `lost=true`，并填写原因。
- 不同楼层地图独立保存，不得把多个高度投影到同一张 `/map`。
- `map_version` 只在有效地图内容或坐标关系发生变化时递增。

### 4.3 `/localization/status` 最小字段与漂移语义

```text
header
tracking_state: INITIALIZING | TRACKING | DEGRADED | RELOCALIZING | LOST
pose_covariance_trace: float64
drift_warning: bool
drift_rate_linear: float32
drift_rate_angular: float32
pose_jump_detected: bool
last_correction_translation: float32
last_correction_rotation: float32
correction_version: uint64
relocalization_event_id: string
last_stable_time: time
status_reason: string
```

- 四足足滑、机体振动、长直走廊几何退化、楼梯运动和电梯轿厢运动都可能造成累计漂移；回环优化或重定位还可能造成离散位姿修正，因此探索模块不能只读取单帧位姿。
- 漂移率可由定位模块内部估计器、创新量、回环残差或协方差增长得到，不要求暴露具体 SLAM 实现。
- `correction_version` 在影响历史 `map` 坐标的修正发生时递增；正常连续 `odom` 变化不递增。
- `DEGRADED` 时探索模块增大安全裕量并避免启动楼梯、门槛等高风险任务；`RELOCALIZING` 或 `LOST` 时暂停发新目标并请求导航停止。
- 修正后定位/建图模块必须给出新的稳定时间和地图版本；探索模块会重新生成或重投影活动目标、拓扑入口、房间视点和未确认检测位置。

### 4.4 `/mapping/traversability` 语义

建议使用团队消息显式定义枚举；若首版使用 `OccupancyGrid`，需要双方确认数值编码。最少需要表达：

```text
UNKNOWN
TRAVERSABLE
CAUTION
NON_TRAVERSABLE
```

该层不复制 `/map` 的 `FREE/OCCUPIED` 结论。探索侧使用 `/map` 生成前沿，使用 `/move_base/make_plan` 作为普通导航可达性的最终查询；只有坡度、台阶、门槛、低净空、轿厢边缘等二维占据无法表达的风险才进入该层。

消息至少携带 `floor_id`、`source_map_version`、`layer_version` 和编码说明。优先与 `/map` 使用相同坐标系、原点、分辨率和尺寸；若不能完全同栅格，必须提供明确元数据，禁止消费者仅按数组索引对齐。建图侧暂时不能稳定产出时，可以缺省，由导航 `make_plan` 和垂直通道假设的风险字段降级承接；不能发布全图“可通行”的虚假占位。

### 4.5 `/mapping/obstacle_structure` 最小字段

每个障碍实体或分段至少包括：

```text
obstacle_id: string
floor_id: int32
class_id: STRUCTURAL_BOUNDARY | INTERIOR_OCCLUDER | TRANSIENT | UNKNOWN
footprint: geometry_msgs/PolygonStamped
min_height: float32
max_height: float32
height_known: bool
classification_confidence: float32
geometry_completeness: float32
persistence: float32
source_map_version: uint64
observation_state: TENTATIVE | PARTIAL | CONFIRMED | INVALIDATED
last_update: time
```

- `STRUCTURAL_BOUNDARY` 表示墙体、固定隔断或建筑外边界，用于房间/走廊分割和门洞提取。
- `INTERIOR_OCCLUDER` 表示位于房间内部、可能遮挡低矮危险源视线的长期障碍；不要求区分桌、椅、柜、沙发等家具类别。
- `TRANSIENT` 表示短时动态或低持久性障碍，不得据此永久切分房间或关闭拓扑分支。
- `UNKNOWN` 表示证据不足；探索模块在房间视线规划中按遮挡物保守处理，但允许后续观测更新类别。
- 高度范围用于近似从相机到球体中心高度的视线是否被阻挡。若只能给二维轮廓，应显式标记高度未知，不能默认“不遮挡”。
- `footprint` 是对占据几何的结构解释，不是第二份占据地图；冲突时以同版本 `/map` 的占据事实为准，并等待派生层更新。
- 该层应与 `/map` 使用同一楼层和坐标，并明确 `source_map_version`；短时分类变化不应造成实体 ID 无规律抖动。
- 允许建图内部通过平面连续性、与房间边界关系、高度、持久性和多视角观测分类；正式运行不得读取布局或模型真值。

### 4.6 `/mapping/vertical_connection_observations` 最小字段

每条消息表达一次可追踪的通道观测，而不是保证完整的跨层连接：

```text
observation_id: string
revision: uint64                       # 同一 observation_id 单调递增
track_hint: string                    # 可选，建图侧认为可能属于同一通道的局部轨迹
type: STAIR | ELEVATOR | UNKNOWN
observed_floor_id: int32
local_entry_pose: geometry_msgs/PoseStamped
observed_footprint: geometry_msgs/PolygonStamped
centerline_or_path: nav_msgs/Path       # 可为空或仅包含已观测部分
observed_height_min: float32
observed_height_max: float32
remote_floor_known: bool
remote_floor_id: int32                  # known=false 时忽略
remote_entry_pose: geometry_msgs/PoseStamped  # known=false 时忽略
remote_entry_known: bool
observation_state: TENTATIVE | PARTIAL | CONFIRMED | INVALIDATED
geometry_completeness: float32
classification_confidence: float32
traversability: OPEN | BLOCKED | CAUTION | UNKNOWN
width_known: bool
width: float32
slope_or_step_known: bool
slope_or_step_height: float32
clearance_known: bool
clearance: float32
source_map_version: uint64
source_correction_version: uint64
last_update: time
```

首次发现时只有入口点、局部轮廓甚至 `type=UNKNOWN` 都是合法状态。楼梯逐步观测后补充已见中心线、平台、宽度、坡度或台阶高度；不得为了满足字段而虚构出口。电梯逐步补充厅门、门槛、轿厢区域和当前可进入状态。探索模块基于 `observation_id/track_hint`、空间邻近、地图版本和实际通行结果合并为内部 `VerticalConnection`，并为缺失部分创建拓扑债务。

生产者对同一局部假设保持稳定 `observation_id`，更新时递增 `revision`；数组消息应声明是“当前快照”还是“增量事件”，首版统一采用当前快照并在撤销版本中保留 `INVALIDATED` 至少一个约定保持周期。探索侧消费规则固定为：

| 观测状态 | 允许的探索动作 |
|----------|----------------|
| `TENTATIVE` | 导航到安全观察位确认，不建立跨层边 |
| `PARTIAL` | 保留拓扑债务并补充观测；远端未知时不执行盲目换层 |
| `CONFIRMED` | 仍需通过安全、定位和导航校验后才生成换层候选 |
| `INVALIDATED` | 取消关联候选，保留失败/撤销记录并重新评估债务 |

### 4.7 垂直通道是否标在地图上

- `/map` 中只标实际观测到的自由、占用和未知几何，不增加“楼梯格”“电梯格”等非标准占据值，也不绘制未观测的远端入口。
- 语义位置通过 `/mapping/vertical_connection_observations` 发布；RViz 可额外显示 marker，但 marker 不是算法输入。
- `/mapping/traversability` 可把已观测楼梯或门槛标为 `CAUTION/BLOCKED`，这是运动风险，不代表连接关系。
- 探索模块内部拓扑图在假设确认后记录跨层边；该拓扑是任务记忆，不回写为占据地图真值。

### 4.8 `surface_cloud` 的边界

`surface_cloud` 是生成结构、通行风险或垂直通道假设的上游观测，不与 `/map` 并列成为规划空间状态。默认数据流为：

```text
允许的激光/深度点云
  -> 定位建图或结构提取
  -> /map + obstacle_structure + traversability + vertical_connection_observations
  -> 探索规划
```

只有派生模块尚未完成且双方明确启用降级适配器时，探索 ROS 壳层才可临时订阅局部点云；点云处理必须与算法核心隔离，且不能成为普通房间扫描或单楼层前沿探索的启动条件。

### 4.9 相邻模块明确不需要提供

- 不要求 `/mapping/occupied_voxels` 或完整全局占据体素。
- 不要求桌、椅、柜等家具类别语义，不要求建图侧直接计算家具阴影、候选视点、遮挡查询或逐体素可见性地图。
- 需要的是上述功能性结构/遮挡分类、二维轮廓和可用时的高度范围；探索模块负责射线可见域和房间覆盖规划。
- 不要求建图模块读取布局真值并绑定公开门、电梯 ID。
- 已有 OctoMap、TSDF、ESDF 或 `surface_cloud` 可作为建图/结构提取内部实现，但不是探索核心接口。
- 不要求第一次看到疑似楼梯或电梯时给出完整连接楼层、远端入口或最终通道 ID。

## 5. 导航控制接口

### 5.1 分阶段接口

| 名称 | 类型 | 阶段 | 用途 |
|------|------|------|------|
| `/move_base` | `move_base_msgs/MoveBaseAction` | P0 | 发送、取消并监控当前楼层导航目标 |
| `/move_base/make_plan` | `nav_msgs/GetPlan` | P0 | 判断候选可达性并计算实际路径长度 |
| `/move_base/clear_costmaps` | `std_srvs/Empty` | P1 | 受限次数的导航恢复 |
| `/navigation/path` | `nav_msgs/Path` | P1 | 当前执行路径和剩余路径评估 |
| `/navigation/health` | `danger_search_common/NavigationHealth` | P0 基础、P1 完整 | 导航、机器人及控制器健康状态 |

### 5.2 `/navigation/health` 最小字段

```text
header
ready: bool
controller_active: bool
stuck: bool
fallen: bool
has_active_goal: bool
active_goal_id: string
progress: float32
last_cmd_time: time
failure_code: string
failure_detail: string
```

标准 `move_base_msgs/MoveBaseAction` 通过 actionlib `GoalStatus` 表达 `SUCCEEDED/ABORTED/PREEMPTED` 等终态；团队框架在 action status text 或 `/navigation/health.failure_code` 中进一步区分：

```text
SUCCEEDED
UNREACHABLE
CANCELED
TIMEOUT
CONTROL_FAILED
ROBOT_FALLEN
LOCALIZATION_LOST
```

要求：

- 取消目标后必须停止旧目标产生的速度指令。
- navigation 只发布 `/danger_search/nav_cmd_vel`；control 负责安全仲裁并唯一发布最终 `/cmd_vel`，探索模块不直接发布速度。
- `make_plan` 应使用与实际导航一致的地图、机器人尺寸和障碍膨胀配置。
- 导航目标必须携带时间戳和坐标系；同楼层目标通常使用 `map`。
- 楼梯任务只能执行经过结构与可通行性校验的路径，不接受策略直接发送任意跨层目标。
- 正式接入时，障碍输入必须由允许传感器产生，不得依赖 Gazebo 真值。

## 6. 危险源感知接口

### 6.1 分阶段接口

| 名称 | 类型 | 阶段 | 用途 |
|------|------|------|------|
| `/danger_detector/detections` | `danger_search_common/DangerSourceArray` | P0 基础、P2 完整 | 上报危险源或干扰源检测结果；数组字段沿用 `dangers` |
| `/danger_detector/status` | `danger_search_common/DetectionStatus` | P0 | 检测器可用性和输入新鲜度 |
| `/danger_detector/observation_done` | 团队事件或状态消息 | P2 | 确认当前观察位姿已完成稳定检测 |
| 感知能力配置或 `/danger_detector/capabilities` | 参数或低频状态消息 | P2 | 有效视场、可靠距离、目标高度范围、稳定等待时间和最大扫描角速度 |

单个检测最少包含：

```text
detection_id: string
track_id: string
class_id: DANGER_RED_SPHERE | DISTRACTOR_RED_CUBE | DISTRACTOR_GREEN_SPHERE | UNKNOWN
position: geometry_msgs/PointStamped
position_covariance: float64[9]
floor_id: int32
confidence: float32
confirmed: bool
verification_required: bool
possible_duplicate_track_ids: string[]
localization_correction_version: uint64
source_time: time
```

`observation_done` 最少包含当前观察任务 ID、观察位姿 ID、完成状态、开始与结束时间，以及失败原因。检测为空与观察完成是两个独立语义：未检测到目标不能自动代表观察完成，观察完成也不能代表房间探索完成。

感知能力最少表达：

```text
horizontal_fov
vertical_fov
reliable_min_range
reliable_max_range
observable_height_min
observable_height_max
required_stable_duration
max_scan_angular_speed
capability_version
```

探索模块以“目标进入有效视场且满足稳定条件后能迅速发现”为规划假设，据此计算房间可见域和旋转步长。这里必须使用检测算法的有效能力，而不是直接把相机理论 FOV、裁剪距离当作可靠检测范围。能力变化时递增版本并使未执行扫描计划重新验证。

感知或结果汇总模块负责分类、跨帧/跨视角去重、定位修正后的坐标更新和最终结果写入；探索模块只消费检测事件、复核需求和观察完成状态。低置信红色目标、球/方块类别不稳定、位置协方差过大或疑似重复时，应以 `verification_required=true` 请求异视角复核，而不是直接输出多个检测点。

复核请求应至少给出 `track_id`、当前估计位置、复核原因和已有观测方向。探索模块根据可达性生成与已有方向有显著基线差异的观察位姿；感知模块在复核后确认、拒绝或合并轨迹。最终 `detected_danger_sources` 只允许包含已确认、已去重且使用最新定位修正版本转换到 `world` 的轨迹。

## 7. 门、电梯和多楼层协作

环境已有：

| 名称 | 类型 | 阶段 | 说明 |
|------|------|------|------|
| `/set_door_state` | `building_generator_interfaces/SetDoorState` | P3 | 打开或关闭公开 ID 对应的动态门 |
| `/call_elevator` | `building_generator_interfaces/CallElevator` | P3 | 呼叫电梯到目标楼层 |

探索模块负责门、电梯高层流程，并把公开服务 ID 与逐步发现的几何假设关联；关联状态必须允许 `UNASSOCIATED/CANDIDATE/CONFIRMED/REJECTED` 和置信度。建图模块只提供传感器实际支持的局部结构、通行风险与通道观测，不得使用公开 ID 反向补全未观测布局；导航模块负责执行门前、电梯厅、轿厢和出口目标。

当同一目标楼层同时可通过楼梯和电梯到达时，探索模块会比较入口实际路径、服务等待、通行时间、定位稳定性、安全风险、失败历史和目标楼层待探索收益，不接受固定优先某一种通道。接口因此需要：

- 电梯运行状态能区分所在楼层、运动中、门状态、可进入、服务失败和预计/已等待时间；几何入口仍来自增量观测。
- 楼梯观测按已知程度逐步提供入口、已见平台/可行走带、坡度/台阶与当前通行风险；未知出口保持未知。
- 定位状态能在轿厢运动、楼梯通行和出口重定位期间明确报告 `DEGRADED`、`RELOCALIZING` 或稳定恢复。
- 门、电梯和导航结果包含实际开始/结束时间，供探索模块更新通道经验代价，而不是长期使用固定常数。

建议补充可订阅的门、电梯运行状态，避免任务状态机只依赖最长约 `25 s` 的固定等待。若首版无法提供独立状态话题，至少保证服务调用：

- 动作完成后才返回成功。
- 超时或执行失败时返回明确错误。
- 重复请求具备可预测的幂等行为。
- 完成后可通过地图、可通行性或服务状态复检实际通路。

换层后，探索模块只有在定位恢复、`current_floor` 更新且 `/mapping/status.stable=true` 后才恢复普通拓扑或房间规划。

## 8. 任务与结果接口

沿用团队框架现有命名和职责：

| 名称 | 类型 | 阶段 | 用途 |
|------|------|------|------|
| `/danger_search/start` | `std_srvs/Trigger` | P0 | mission 提供；用户启动整个任务的唯一正式入口 |
| `/danger_search/finish` | `std_srvs/Trigger` | P0 | mission 提供；停止探索、冻结计分时间、最终落盘并结束任务 |
| `/danger_search/return_home` | `std_srvs/Trigger` | P0 可选 | mission 提供；结果冻结后的可选返航，不是完成前置条件 |
| `/danger_search/start_exploration` | `std_srvs/Trigger` | P0 | exploration 提供；供 mission 编排和模块独立测试 |
| `/danger_search/stop_exploration` | `std_srvs/Trigger` | P0 | exploration 提供；停止选点并取消活动导航目标 |
| `/mission/status` | `danger_search_common/MissionStatus` | P0 基础、后续扩展 | mission 发布任务阶段、楼层、进度和诊断，使用 latch |
| `/mission/active` | `std_msgs/Bool` | P0 | mission 发布任务是否激活，使用 latch |

`/mission/status` 沿用现有 `danger_search_common/MissionStatus` 字段：

```text
header
mission_state: string
current_floor: int32
start_time: time
elapsed_time: duration
scored_exploration_time: duration
active_goal_id: string
map_coverage_summary: string
topology_debt_summary: string
room_visibility_summary: string
remaining_frontier_count: uint32
localization_correction_version: uint64
finish_reason: string
```

任务状态至少覆盖等待输入、走廊拓扑探索、房间扫描、恢复、门操作、进入电梯、换层、离开电梯、可选返航、完成和异常结束。

调用 `/danger_search/start` 后，mission 只有在计时和状态初始化成功且 `/danger_search/start_exploration` 返回成功时才进入 `EXPLORING` 并发布 `/mission/active=true`；依赖未就绪、重复启动或模块级启动失败必须返回明确错误，不能形成假激活状态。探索收敛后调用现有 `/danger_search/finish`，不再定义另一套同义完成服务。

`/danger_search/finish` 必须具备幂等语义：停止 exploration、取消活动 `/move_base` 目标、冻结计分时间、写入结果并发布 `FINISHED`；写入失败时返回失败并进入明确错误状态。任务没有硬性总时限，也不存在基于“剩余时间”的强制返航接口。mission 应持续维护检测结果，并在 `/danger_search/finish` 后最终写入 `results/detected_danger.json`。`scored_exploration_time` 从任务正式开始计到探索、复核和最终结果冻结完成；结果冻结后执行的可选返航不应继续增加该字段，除非正式规则后续明确要求。返航是可配置行为，不是结果有效的前置条件。

任务状态可发布距离 `600 s` 及后续 `60 s` 扣分台阶的时间，供规划器比较剩余动作收益；这不是硬截止。不得为了保持 `600 s` 内满分而跳过高价值未扫描房间或必要复核，也不得以低新增可见域动作无目的跨越扣分台阶。

### 8.1 P0 `map -> world` 与结果责任

P0 就必须冻结结果坐标契约，不能把 `frame_id=map` 的点直接改名为 `world`：

- 所有检测位置保留真实 `frame_id` 和时间戳。
- 若 SLAM 的 `map` 与 Gazebo `world` 不重合，定位模块必须提供连接二者的 TF（推荐 `world` 为父、`map` 为子，使结果模块可查询 `map -> world`），或结果模块使用允许读取的 `team_scene_info.json` 起点信息建立等价静态变换。
- 变换来源必须是公开起点和本队定位，不得读取 `/ground_truth/*`、`/Odometry_gazebo`、world 文件或布局元数据。
- 结果汇总模块负责按检测时间和最新有效定位修正版本转换、去重并写入 `world`；转换不可用时不得静默写入 map 坐标。
- P0 集成测试至少用一个已知人工输入点验证数值变换和 JSON 三维坐标，而不把裁判真值反馈给规划器。

## 9. 探索模块对外输出

除导航 Action 和任务状态外，探索模块将提供：

- 当前楼层、规划状态和状态转换原因。
- 当前目标 ID、目标位姿、生成时地图版本和尝试次数。
- 地图探索进度、走廊拓扑图摘要、已发现房门/岔路和未服务拓扑债务。
- 房间目标支持区域、可见域覆盖、计划/已完成观察位姿和不可达死角。
- 剩余前沿数量、候选评分摘要、冷却和黑名单。
- 垂直通道候选的等待、风险、定位和经验代价摘要。
- 当前定位修正版本及因此失效、重投影或重生成的目标数量。
- 待复核检测数量、复核原因、关联 `track_id` 和当前确认/合并状态；不发布裁判真值相关指标。
- 目标取消、重规划和恢复原因。
- RViz 前沿、候选、当前目标、垂直通道和黑名单标记。
- 最终停止原因和不可恢复错误诊断。

具体话题名可在接口评审时确认；消息应保留时间戳、坐标系、楼层和地图版本。

## 10. 数据一致性与异常处理

- 定位、地图、结构障碍、可通行性和垂直通道观测必须能通过 TF 对齐，并携带明确的源地图版本；P4 可选点云只要求能被其直接消费模块对齐。
- 数据时间差超过双方约定阈值时，探索模块暂停发新目标并进入等待或恢复。
- 地图版本变化导致目标落入障碍、未知或不同楼层时，探索模块取消目标并重规划。
- `correction_version` 变化、位姿跳变或重定位时，探索模块暂停普通规划；稳定后复核活动目标、拓扑债务、房间覆盖、垂直通道入口和未确认检测坐标。
- 结构障碍层与占据地图源版本不一致时不得执行新的房间可见性计划；可继续安全退出房间或进入明确降级模式。
- 垂直通道观测变为 `INVALIDATED` 或源地图版本失效时，探索模块撤销相关执行候选但保留审计记录；不得因此删除无关的历史拓扑债务。
- 定位丢失、机器人摔倒、控制器失效时，导航必须立即停止正常目标执行并上报原因。
- 服务和 Action 必须有超时；探索模块会限制单目标、单恢复及门电梯操作的重试次数。
- 重启后应能通过 latched 状态、查询服务或重新发布恢复当前楼层、地图版本和模块健康状态。

## 11. 正式输入红线

正式算法不得订阅或读取：

- `/Odometry_gazebo`
- `/ground_truth/base_w`
- `/ground_truth/base_trunk`
- `/ground_truth/*_foot`
- `generated_building/layout_metadata.json`
- `generated_building/building_config.json`
- `generated_building/scene_manifest.json`
- `generated_building/danger_truth.json`
- `results/danger_truth.json`
- `generated_building/competition_scene.world` 或其他泄露完整布局的文件

上述数据仅允许在与正式算法隔离的 evaluator 中计算离线指标。

## 12. 联调与验收标准

### 12.1 P0 验收：可运行、可移动、可评分

- 所有 P0 消息可读取且时间戳更新，TF 无长期断链。
- 地图编码正确，简单 DFS/前沿候选能经 `make_plan` 检查后发送给导航。
- 机器人完成至少一个目标，并覆盖成功、取消或失败中的两种结果；取消后停止旧速度。
- 检测数组为空和非空时任务都能正常结束。
- 结果文件严格符合官方格式，使用三维 `world` 坐标，官方 evaluator 能输出评分结果。
- 只启动 P0 接口时，P2～P4 接口缺失不会导致节点崩溃或永久等待。

### 12.2 P1 验收：可靠单楼层自动闭环

- 探索模块能等待接口就绪后自动开始，并在前沿收敛后自动 finish。
- 导航失败后能取消、有限恢复、冷却并改选目标，不会无限重试。
- 地图版本变化会使失效目标重验证；定位跳变或重定位期间暂停普通规划。
- 模块停更、定位丢失、导航阻塞和恢复超时都能给出明确原因。
- 最终结果在自动结束时落盘，且没有活动目标继续输出速度。

### 12.3 P2 验收：房间覆盖与复核

- 结构障碍层能区分至少一个房间的边界和内部遮挡物，每项可追溯到源地图版本。
- 进入房间后按有效 FOV 生成观察位姿；有遮挡时补充不同侧视点，开阔房间不机械增加视点。
- `observation_done` 能结束关联观察动作；空检测不会被误作观察完成。
- 同一危险源的多帧、多视角和定位修正前后观测合并到同一轨迹。
- 红方块、绿色球、类别不稳定和低置信红球经过拒绝或异视角确认，不因单帧直接写入结果。
- 结构层缺失的降级结果与完整 P2 结果明确区分，不虚报房间覆盖完成。

### 12.4 P3 验收：多楼层闭环

- 同一疑似通道可依次发布 `TENTATIVE -> PARTIAL -> CONFIRMED`，也可发布 `INVALIDATED`；未知远端不填虚构坐标。
- 探索模块对 `TENTATIVE/PARTIAL` 创建确认债务，不直接建立可执行跨层边。
- 换层期间地图状态不稳定；换层后楼层编号、TF 和当前层地图一致。
- 各楼层地图、进度、拓扑债务和失败记录不会相互覆盖。
- 门、电梯失败能超时退出；楼梯或电梯至少有一种可完成真实换层并恢复探索。

### 12.5 P4 验收：风险与效率优化

- 通行风险层缺失时保持 P3 能力；存在时会改变高风险通道或候选的选择。
- 同时存在楼梯和电梯时，等待、风险、失败历史和楼层收益变化会改变选择。
- 多随机场景记录探索时间、路径长度、导航失败、房间可见域、待处理债务、确认检测和虚警代理指标。
- 策略插件失败、超时或给出非法动作时自动回退规则策略。

## 13. 接口评审待确认项

### 13.1 P0 必须先确认

1. 团队自定义消息所在 ROS package、节点启动顺序和命名空间。
2. `base`、`base_link`、`trunk` 等机器人基座 frame 的统一名称。
3. `/map` 是否与 `world` 重合；若不重合，`map -> world` 变换的发布方、公开数据来源和结果转换责任。
4. `/move_base`、`make_plan`、基础导航健康和取消后停车语义。
5. `/danger_search/start`、`/danger_search/finish` 与模块级 exploration 启停的调用时序、失败/幂等语义、结果路径，以及 `exploration_time` 起止时刻。

### 13.2 P1 确认

1. `/localization/status`、完整 `/mapping/status` 和 `/navigation/health` 消息定义。
2. 定位协方差、漂移率、位姿跳变和稳定恢复阈值，以及 `correction_version` 与 `map_version` 的关系。
3. 单层地图版本、前沿收敛、导航失败和自动结束的 mock 场景。

### 13.3 P2 确认

1. `/mapping/obstacle_structure` 的生产者、分类准则、最低置信度和实体 ID 策略。
2. 感知能力使用静态参数还是状态消息，以及 `observation_done` 的任务 ID、稳定时长和失败语义。
3. 检测 `track_id`、位置协方差、疑似重复关联和异视角复核消息。

### 13.4 P3 确认

1. 分楼层地图的缓存、切换和历史地图查询方式。
2. 垂直通道未知值编码、`observation_id/track_hint` 稳定策略和升级为 `CONFIRMED` 的证据。
3. 门、电梯采用状态话题还是查询服务，以及等待、运行、失败和幂等语义。

### 13.5 P4 确认

1. 可通行性派生层的生产者、数值编码、谨慎通行代价和缺失时降级契约。
2. rosbag、mock、RViz、训练日志和多随机场景接口回归的负责人及交付日期。

## 14. 建议交付顺序

1. **P0**：冻结位姿、当前层地图、基础导航、基础检测、任务起止、`map -> world` 和结果落盘；用 DFS/简单前沿跑通官方 evaluator。
2. **P1**：补齐定位修正、地图版本、导航恢复、进度和自动结束，形成可靠单楼层闭环。
3. **P2**：补齐结构障碍、感知能力、观察完成和检测复核，接入房间可见性覆盖。
4. **P3**：补齐分层地图、增量垂直通道观测、门和电梯状态，完成真实换层。
5. **P4**：按实验需要增加通行风险、诊断、回放、在线代价和策略插件。

每个阶段单独冻结接口并保留回归测试。后续阶段失败时，系统应尽可能降级到最近已通过验收的阶段，而不是让整个任务无法启动。
