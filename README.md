# 危险源自主搜索与识别系统

挑战杯 DG-2026 四足机器人危险源搜索方案，当前分支为
`exploration_p1_wxt` 的 P1 多楼层比赛闭环实现。

## 正式运行合同

正式入口固定使用：

- `competition_mode=true`
- `multifloor_enabled=true`
- `localization_backend=gicp`
- `ENABLE_REFEREE_ODOM=0`
- `ENABLE_GROUND_TRUTH=0`
- `POINTCLOUD_USE_GROUND_TRUTH_ODOM=0`

算法只读取官方传感器、`CallElevator`/`SetDoorState` 服务和公开的
`generated_building/team_scene_info.json`。`gazebo_truth` 仅用于隔离测试；正式预检发现
真值后端、禁止话题订阅、禁止文件参数、错误服务类型、缺失传感器/TF、结果目录不可写或
`/cmd_vel` 非唯一发布时会拒绝启动。

需要先验证探索、电梯、感知和返航而暂不评估连续定位时，使用独立入口：

```bash
roslaunch danger_search_bringup simulation_truth.launch autostart:=true
```

它固定 `run_profile=simulation_truth`、`competition_mode=false`、
`multifloor_enabled=true` 与 `localization_backend=gazebo_truth`。仅
`/gazebo_truth_odometry` 可读取 `/gazebo/link_states` 的 `a1_gazebo::base`；楼层身份仍只
来自电梯服务。真值模式输出到
`simenvnew/results/detected_danger.simulation_truth.json`，绝不可作为正式通过证据。

## 系统闭环

```text
mission
  ├─ ENTERING：从任务起点分段进入建筑
  ├─ EXPLORING：逐层 WFD 前沿探索与红球三维检测
  │    └─ TransitFloor：电梯厅 -> 轿厢 -> SwitchFloor -> 稳图 -> 楼层大厅
  └─ RETURNING：必要时先乘电梯回 0 层，再导航回起点
       └─ 位置/yaw 达标且连续静止 2 s -> 原子写结果 -> FINISHED

localization: GICP 平面连续定位 + 独立楼层地图 + map_epoch
navigation:   move_base/Navfn/DWA + Unitree 有界恢复
control:      safety > elevator lease > navigation，唯一发布 /cmd_vel
perception:   HSV + RGB-D 已知半径球拟合 + 分层/epoch 跟踪
```

`/exploration/complete` 只表示所有公开 served floors 的覆盖工作完成；只有 mission 可以
宣布任务 `FINISHED`。返航或结果写入失败进入 `ERROR`，不会伪报成功。

## 构建和启动

环境必须按 ROS、`simenvnew`、算法工作区的顺序加载，以解析官方强类型服务。登录 shell
若优先使用 Miniconda，先把系统 Python 放回 PATH，避免 ROS Python 误用 3.14：

```bash
source /opt/ros/noetic/setup.bash
export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH
test "$(command -v python3)" = /usr/bin/python3
source /home/langan/simenvnew/devel/setup.bash
cd /home/langan/danger_search_ws
catkin_make -j4
source devel/setup.bash
```

正式仿真：

```bash
cd /home/langan/simenvnew
GUI=false \
ENABLE_REFEREE_ODOM=0 \
ENABLE_GROUND_TRUTH=0 \
POINTCLOUD_USE_GROUND_TRUTH_ODOM=0 \
./auto.sh
```

Unitree 控制器输入 `2` 完成站立，等待至少 15 秒并确认 IMU 姿态可接受，再输入 `6`
进入 `/cmd_vel` 模式。之后另开终端启动算法；算法运行期间禁止输入 `8` 重置机器人：

```bash
source /opt/ros/noetic/setup.bash
export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH
source /home/langan/simenvnew/devel/setup.bash
source /home/langan/danger_search_ws/devel/setup.bash
roslaunch danger_search_bringup competition.launch autostart:=true
```

默认公开场景合同和结果路径分别为：

```text
/home/langan/simenvnew/generated_building/team_scene_info.json
/home/langan/simenvnew/results/detected_danger.json
```

不同目录布局可通过 `simenv_root`、`scene_info_file` 和 `result_file` 覆盖。正式运行不需要
人工选点、手动导航或调用 finish；`FinishMission`/`ReturnHome` 仅用于提前触发同一返航
流程。

## 关键公共接口

| 名称 | 类型 | 语义 |
|---|---|---|
| `/mapping/status` | `danger_search_common/MappingStatus` | 当前层、`transitioning`、`map_epoch`、`floor_z_m` 和稳定性 |
| `/mapping/active_map` | `danger_search_common/FloorOccupancyGrid` | 与 `/map` 相同栅格的原子 floor/epoch/version envelope；多楼层探索以此为权威输入 |
| `/navigation/health` | `danger_search_common/NavigationHealth` | 已核验的 floor/epoch/version 与 `transitioning` 门禁；未就绪时必须拒绝新 goal |
| `/localization/switch_floor` | `danger_search_common/SwitchFloor` | 以 `transition_id` 幂等切换楼层地图并重置 GICP 参考 |
| `/danger_search/transit_floor` | `danger_search_common/TransitFloorAction` | 复用的完整换层动作；探索和返航共同调用 |
| `/move_base` | `move_base_msgs/MoveBaseAction` | 普通二维导航目标 |
| `/danger_search/nav_cmd_vel` | `geometry_msgs/Twist` | move_base 输出到 control |
| `/danger_search/elevator_cmd_vel` | `geometry_msgs/Twist` | 进出轿厢的短租约输入 |
| `/cmd_vel` | `geometry_msgs/Twist` | control 唯一发布的最终速度 |
| `/danger_detector/detections` | `DangerSourceArray` | 检测绑定 floor、map epoch 和定位修正版本 |
| `/exploration/complete` | `std_msgs/Bool` | 全部 served floors 探索完成事件，不是任务终态 |
| `/mission/status` | `MissionStatus` | `IDLE/ENTERING/EXPLORING/RETURNING/FINISHED/ERROR` |

`TransitFloor` 固定失败码为：`NO_HALL`、`UNREACHABLE_HALL`、
`SERVICE_UNAVAILABLE`、`SERVICE_REJECTED`、`SERVICE_TIMEOUT`、`ENTER_FAILED`、
`FLOOR_MISMATCH`、`MAP_NOT_STABLE`、`EXIT_FAILED`、`CANCELED`、`STALE_EPOCH`。

完整字段见 [接口规范](docs/INTERFACE_SPEC.md)。

## 多楼层实现

- 楼层身份只来自公开拓扑和 `/call_elevator` 成功响应的 `current_floor`；GICP 不从
  Gazebo 高度推断正式楼层。
- 固定平台层高参数默认为 `2.6 m`，目标 z 为 `floor_id * 2.6 + RGB-D 局部球心高度`。
- 换层时取消普通导航、暂停长期建图、保留电梯 x/y/yaw、切换独立楼层图并递增
  `map_epoch`；清空 costmap 后需至少两个新地图版本并稳定 15 s 才恢复选点。
- 启动阶段保持静止，通过公开门服务取得开/关门各 5 帧全向激光差分，直接绑定未知场景中的
  电梯门中心和朝向；成功绑定跨同一 floor/epoch/map-load 的后续地图版本有效。差分失败时才
  使用严格的单门洞矩形井道几何和跨 3 个地图版本的累计评分，距离只作最终同分项。
- 电梯门槛穿越默认使用 `1.00 m @ 0.40 m/s`，主动门运动绑定在换层开门后直接进入，不重复执行
  “开—关—开”验证。拓扑按 served floors 选择最少换乘路线。
- 每层独立保存失败目标、trap blacklist、地图版本和完成状态。连续 10 s 无可达前沿且
  地图稳定后才标记当前层完成。

## 结果合同

任务开始时会原子覆盖为本轮空的 `RUNNING` 结果，避免旧检测残留；0 个危险源时输出空数组。
返航验证成功后才冻结 `exploration_time` 并写 `FINISHED`：

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

`result_coordinate_frame=auto` 时，公开合同声明 `world` 就使用其中的 `robot_start` 完成
完整 yaw/平移/高度变换；缺失或未知时回退为 `start_relative`。官方 evaluator 会忽略新增
字段并继续读取必要字段，详见 [evaluator 兼容说明](docs/EVALUATOR_COMPATIBILITY.md)。

## 测试门禁

```bash
source /opt/ros/noetic/setup.bash
source /home/langan/simenvnew/devel/setup.bash
source devel/setup.bash
catkin_make run_tests -j4
catkin_test_results --all
```

必须以最后一条命令为准。当前自动化覆盖扫描过滤合同、WFD、多楼层拓扑/状态机、
`0->1->2->0` 独立地图、epoch/迟到回调、DWA/控制联合参数、恢复插件、感知干扰物、
坐标变换、空结果和 mission 返航 smoke。

这些测试不等于正式比赛闭环实测。12 个固定 seed、覆盖率、600 s、召回/虚警/误差、
CPU/内存和重复稳定性仍必须在真实 `simenvnew` 上按验收矩阵运行并归档后，才能宣布比赛闭环
通过。2026-08-27 的首轮正式模式 smoke 已通过 preflight，但在入场导航阶段以
`entry_timeout` 结束；当前不得标记为比赛闭环通过，证据和整改顺序见
[seed 42001 正式 smoke 记录](docs/FORMAL_SMOKE_SEED42001.md)。

完整的 `GUI=false` 启动顺序、状态采集、隔离测试和停止步骤见
[`command_bringup_flow.md`](command_bringup_flow.md)。
