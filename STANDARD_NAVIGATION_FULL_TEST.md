# 标准 move_base + DWAPlannerROS 完整仿真测试

本文用于测试当前完整 `danger_search` 系统。测试链路为：

```text
SimEnv Gazebo + Unitree RL 控制器
    -> localization 真值位姿与建图
    -> 标准 move_base / NavfnROS / DWAPlannerROS
    -> /danger_search/nav_cmd_vel
    -> cmd_mux
    -> /cmd_vel
    -> Unitree RL 控制器
```

测试要求：

- 启动全部模块，不单独启动探索节点。
- 先按 `2` 站立，再按 `6` 进入 RL `/cmd_vel` 模式。
- 机器人直接出生在一层室内，跳过进门阶段。
- Gazebo 真值只用于 localization 测试后端，不允许 `state_from_gazebo` 重复发布 TF。
- `/cmd_vel` 必须只有 `danger_search_control` 一个发布者。

## 0. 首次测试前编译

代码修改后执行一次。已经成功编译过时可跳过本节。

```bash
cd /home/ruilinli/SimEnv
source /opt/ros/noetic/setup.bash
catkin_make --pkg unitree_guide -j2

cd /home/ruilinli/danger_search_ws
source /opt/ros/noetic/setup.bash
source /home/ruilinli/SimEnv/devel/setup.bash --extend
source /home/ruilinli/danger_search_ws/devel/setup.bash --extend
catkin_make -j2
```

## 1. 终端一：启动带 GUI 的 SimEnv

以下位置让机器人直接出生在一层室内大厅 `(0, 5)`：

```bash
cd /home/ruilinli/SimEnv
source /opt/ros/noetic/setup.bash

GUI=true \
SEED=42 \
ROBOT_X=0.0 \
ROBOT_Y=5.0 \
ROBOT_Z=0.6 \
ROBOT_YAW=1.5708 \
ENABLE_SENSORS=1 \
ENABLE_REFEREE_ODOM=0 \
ENABLE_GROUND_TRUTH=0 \
POINTCLOUD_USE_GROUND_TRUTH_ODOM=0 \
START_CONTROLLER=1 \
CONTROLLER_FOREGROUND=1 \
./auto.sh
```

这里必须保持：

- `ENABLE_REFEREE_ODOM=0`：不启动会重复发布 `map -> odom -> base` 的 `state_from_gazebo`。
- `simulation_truth.launch` 的固定真值后端直接读取 `/gazebo/link_states`，不依赖 `/Odometry_gazebo`。
- `CONTROLLER_FOREGROUND=1`：终端保留键盘输入能力。

等待 Gazebo 完全启动、机器人落地且终端出现 Unitree 控制提示后：

1. 在该终端按 `2`。
2. 等待至少 `4` 秒，让机器人完全站稳。
3. 按 `6`，进入 RL `/cmd_vel` 模式。

预期依次看到类似输出：

```text
Switched from passive to fixed stand
[INFO] Entered RL /cmd_vel mode.
```

如果已经处于 RL 状态，再按 `6` 也可能显示：

```text
[INFO] Switched RL command source to /cmd_vel.
```

终端一必须一直保持运行。不要按 `4`，模式 `4` 是键盘速度模式，不接收导航的 `/cmd_vel`。

## 2. 终端二：启动完整 danger_search 系统

等机器人完成 `2 -> 等待 -> 6` 后再执行：

```bash
cd /home/ruilinli/danger_search_ws
source /opt/ros/noetic/setup.bash
source /home/ruilinli/SimEnv/devel/setup.bash --extend
source /home/ruilinli/danger_search_ws/devel/setup.bash --extend

roslaunch danger_search_bringup simulation_truth.launch \
  autostart:=false \
  entry_enabled:=false \
  simenv_root:=/home/ruilinli/SimEnv
```

<<<<<<< Updated upstream
=======
只验证 Seed 42 已知电梯坐标下的进梯和换层链路时，改用：

```bash
roslaunch danger_search_bringup simulation_truth.launch \
  autostart:=false \
  entry_enabled:=false \
  simenv_root:=/home/ruilinli/SimEnv \
  fixed_elevator_hall_enabled:=true \
  fixed_elevator_hall_x:=-2.40 \
  fixed_elevator_hall_y:=-1.65 \
  fixed_elevator_hall_into_yaw:=-1.5707963
```

该开关仅允许用于 `simulation_truth`，固定门中心为 map 坐标
`(-2.40, -1.65)`、朝轿厢方向 `yaw=-1.5707963`。固定坐标测试路径到达门前后
直接请求开门，固定等待 `26 s`，再通过 `/danger_search/elevator_cmd_vel` 以
`0.40 m/s` 直行进入；它不执行门前精确对准和三分区开—关—开验证，但仍保留
激光 footprint、安全停车、穿门进度、呼梯和换层地图合同。在线电梯发现路径不变。
正式 competition 模式若误开该参数，探索节点必须拒绝启动。

>>>>>>> Stashed changes
该命令会同时启动：

- Gazebo 真值 localization 适配器。
- 激光投影与 OccupancyGrid 建图。
- 标准 `move_base`。
- `navfn/NavfnROS` 全局规划器。
- `dwa_local_planner/DWAPlannerROS` 局部规划器。
- 标准 costmap 和有限 recovery。
- 探索规划。
- `cmd_mux` 控制仲裁。
- 危险源感知。
- mission 任务总控。
- 门控制和兼容状态监控。

正确启动时应看到：

```text
[localization] TEST MODE: Gazebo truth ...
Created global_planner navfn/NavfnROS
Created local_planner dwa_local_planner/DWAPlannerROS
[navigation_monitor] standard move_base compatibility ready
```

在 `gazebo_truth` 模式下不应连续出现：

```text
GICP rejected
raw GICP covariance is unhealthy
```

终端二也必须一直保持运行。

## 3. 终端三：启动前检查

### 3.1 检查标准规划器确实被加载

```bash
cd /home/ruilinli/danger_search_ws
source /opt/ros/noetic/setup.bash
source /home/ruilinli/SimEnv/devel/setup.bash --extend
source /home/ruilinli/danger_search_ws/devel/setup.bash --extend

rosparam get /move_base/base_global_planner
rosparam get /move_base/base_local_planner
rosparam get /move_base/DWAPlannerROS/odom_topic
rosparam get /move_base/DWAPlannerROS/min_vel_x
rosparam get /move_base/DWAPlannerROS/max_vel_x
rosparam get /move_base/DWAPlannerROS/min_vel_trans
rosparam get /move_base/DWAPlannerROS/max_vel_trans
rosparam get /move_base/DWAPlannerROS/vx_samples
rosparam get /move_base/DWAPlannerROS/min_vel_theta
rosparam get /move_base/DWAPlannerROS/max_vel_theta
rosparam get /move_base/local_costmap/obstacles/observation_sources
rosparam get /move_base/global_costmap/obstacles/observation_sources
rostopic echo -n 1 /navigation/config_ready
timeout 5 rostopic hz /localization/depth_obstacle_scan
```

预期输出：

```text
navfn/NavfnROS
dwa_local_planner/DWAPlannerROS
/localization/odom
0.0
0.3
0.3
0.3
2
0.4
0.4
scan depth_scan
scan depth_scan
data: True
```

### 3.2 检查定位、地图、导航和感知 readiness

```bash
rostopic echo -n 1 /localization/odom
rostopic echo -n 1 /mapping/status
rostopic echo -n 1 /navigation/health
rostopic echo -n 1 /danger_detector/status
```

开始任务前至少应满足：

- `/localization/odom` 的 `header.frame_id` 为 `odom`，`child_frame_id` 为 `base`。
- `/mapping/status`：`ready: True`、`stable: True`、`lost: False`。
- 使用 `simulation_truth.launch` 时，正常 `status_reason` 应为
  `TRACKING_GAZEBO_TRUTH_WITH_LOCAL_OCCUPANCY_MAP`，不得显示为 GICP tracking。
- `/navigation/health`：`ready: True`。
- `/danger_detector/status`：`ready: True`。

若 mapping 还未稳定，等待数秒后重新执行检查，不要直接启动任务。

### 3.3 检查 TF 没有重复发布

```bash
rosnode list | grep state_from_gazebo || true
rosrun tf tf_echo map base
```

第一条命令正常情况下没有输出。第二条应持续输出平滑、有限的位姿；观察几秒后按 `Ctrl-C` 结束 `tf_echo`。

### 3.4 检查速度链路和唯一发布者

```bash
rostopic info /danger_search/nav_cmd_vel
rostopic info /cmd_vel
```

必须满足：

- `/danger_search/nav_cmd_vel` 的 Publisher 包含 `/move_base`。
- `/danger_search/nav_cmd_vel` 的 Subscriber 包含 `/control`。
- `/cmd_vel` 只有一个 Publisher，正常为 `/control`。
- `/cmd_vel` 的 Subscriber 包含 `/unitree_gazebo_servo`。

如果 `/cmd_vel` 没有 `/unitree_gazebo_servo` 订阅者，先不要启动任务，重新确认终端一中的 `2 -> 等待 -> 6` 和 Unitree 控制器状态。

## 4. 终端三：任务启动前运动预检

预检期间 mission 尚未启动，因此可以单独向标准 `move_base` 发送短距离目标，验证导航到 RL 步态的完整执行链。预检必须在出生点附近的开放区域进行；如果 RViz 中目标方向有障碍物，不要发送目标。

### 4.1 开放区直线预检

当前测试模式以出生位姿作为局部 `map` 原点。发送前方约 `1 m` 的目标：

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

在两个终端分别观察：

```bash
rostopic echo /danger_search/nav_cmd_vel
rostopic echo /cmd_vel
```

验收要求：

- 路径跟踪阶段 `/danger_search/nav_cmd_vel.linear.x` 应达到 `0.30 m/s`。
- `/cmd_vel.linear.x` 应受 cmd_mux 限加速度约束平滑上升，且不超过 `0.40 m/s`。
- Gazebo 中机器人应在 `5 s` 内产生明显前进，真值位移至少 `0.10 m`。
- 只统计 `/move_base/status` 为 `ACTIVE` 的控制区间：
  `/danger_search/nav_cmd_vel` 最大间隔应小于 `0.30 s`，P95 应不超过
  `0.15 s`。目标完成到下一目标发送之间的安全零速选点阶段不计入断流。

如果 `/cmd_vel.linear.x` 已达到 `0.30`，但机器人 `5 s` 内仍完全不动，取消目标并停止预检；此时问题属于 Unitree RL policy 或关节执行层，不要继续提高导航速度。

### 4.2 后方目标预检

直线预检完成后，向出生点附近发送一个位于机器人后方的目标。该目标专门验证
DWA 是否保留零线速度转向样本；若再次把 `min_vel_x` 收紧到 `0.30`，这里会在
首个控制周期开始持续报告 `DWA planner failed to produce path`。

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}}}"
```

终点朝向设为 `yaw=pi`，与返回出生点的行进方向一致，避免把“后方目标转向”
预检混入到点后再次回转 `180°` 的独立终点姿态测试。

验收要求：

- `/danger_search/nav_cmd_vel` 先出现 `linear.x=0`、`|angular.z|` 接近
  `0.40 rad/s` 的原地对准命令。
- 对准后只出现 `linear.x=0.30 m/s` 的持续前进模式；不得持续发布
  `0<linear.x<0.30 m/s` 的 DWA 目标速度。
- `/move_base/status` 最终为 `SUCCEEDED`，不得耗尽 recovery 后终止。

### 4.3 终点原地旋转预检

后方目标预检结束后，向当前位置发送约 `90°` 的最终朝向。由于
`xy_goal_tolerance=0.15`，局部规划器会进入标准终点旋转控制：

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7071068, w: 0.7071068}}}"
```

验收要求：

- `/danger_search/nav_cmd_vel` 应出现接近 `|angular.z|=0.40 rad/s` 的原地旋转命令。
- `/cmd_vel.angular.z` 平滑上升且不超过 `0.40 rad/s`。
- Gazebo 真值 yaw 应在 `3 s` 内变化至少 `0.20 rad`。
- 高速转向时长期建图可以暂停，但 `/localization/scan` 和局部 costmap 必须继续更新。

完成预检后取消所有测试目标，并确认 `/cmd_vel` 已归零：

```bash
rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID "{}"
rostopic echo -n 1 /cmd_vel
```

只有直线、后方目标和终点转向预检都通过后，才开始完整任务。

## 5. 终端三：通过 mission 启动完整任务

所有 readiness 检查通过后执行：

```bash
rosservice call /danger_search/start "{}"
```

预期返回：

```text
success: True
message: "Mission started"
```

因为使用了 `entry_enabled:=false`，任务应直接进入：

```text
[mission] EXPLORING; home pose captured in map
[exploration] Start exploration
```

随后先进入一次 `INITIAL_HALL_DISCOVERY`：机器人必须保持静止，依次采集 5 帧开门扫描、
关闭 `elevator_floor_0`、稳定 0.75 秒并采集 5 帧关门扫描。Seed 42 正常应在 35 秒内看到：

```bash
rostopic echo /exploration/status
rostopic echo /danger_search/cmd_vel_sent
```

`/exploration/status` 中应出现非空 `elevator_binding`，其 `source` 为 `door_motion`、
`confidence` 为 `1.0`、`validated` 为 `true`；首次普通导航目标只能在该状态结束后发出。
成功后 0 层电梯门保持关闭，直到 floor 0 完成并执行实际换层。若差分不可见、有歧义、
扫描几何改变或机器人位姿漂移超限，节点必须恢复开门并有限转入严格几何回退。
Livox 投影造成的稀疏无返回允许在同一线段内桥接最多 5 个 bin，拟合点本身仍必须是真实
变化点，门宽、直线 RMS 和多簇歧义阈值不因此放宽。

必须调用 `/danger_search/start`。不要直接调用 `/danger_search/start_exploration`，否则会绕过 mission 的任务生命周期、危险源确认和结果保存。

## 6. 终端四：启动 RViz

```bash
cd /home/ruilinli/danger_search_ws
source /opt/ros/noetic/setup.bash
source /home/ruilinli/SimEnv/devel/setup.bash --extend
source /home/ruilinli/danger_search_ws/devel/setup.bash --extend

rviz
```

将 RViz 的 `Fixed Frame` 设置为：

```text
map
```

建议添加以下显示项：

| RViz 显示类型 | Topic | 用途 |
|---|---|---|
| Map | `/map` | localization 输出的占据地图 |
| Map | `/move_base/global_costmap/costmap` | 全局 costmap |
| Map | `/move_base/local_costmap/costmap` | 局部 rolling costmap |
| PoseWithCovariance | `/localization/pose` | 当前 map 位姿 |
| LaserScan | `/localization/scan` | costmap 实际使用的二维激光 |
| LaserScan | `/localization/depth_obstacle_scan` | RealSense 地面过滤后的低矮近场障碍补盲 |
| PointCloud | `/scan` | Gazebo 原始点云 |
| PointCloud2 | `/livox/Pointcloud2` | 转换后的 Livox 点云 |
| Path | `/move_base/NavfnROS/plan` | Navfn 全局路径 |
| Path | `/move_base/DWAPlannerROS/global_plan` | 局部规划器接收的全局路径 |
| Path | `/move_base/DWAPlannerROS/local_plan` | DWAPlannerROS 当前局部轨迹 |
| Polygon | `/move_base/local_costmap/footprint` | 当前固定保守 footprint |
| TF | 无 | 检查 `map -> odom -> base` 和传感器 TF |

如果局部 costmap 在 `odom` 坐标系，而 RViz Fixed Frame 为 `map`，这是正常的；TF 会负责变换。

## 7. 终端五：运行期间监控与录包

### 7.1 查看导航命令频率

```bash
cd /home/ruilinli/danger_search_ws
source /opt/ros/noetic/setup.bash
source /home/ruilinli/SimEnv/devel/setup.bash --extend
source /home/ruilinli/danger_search_ws/devel/setup.bash --extend

rostopic hz /danger_search/nav_cmd_vel
```

活动控制阶段应接近 move_base 的 `10 Hz`。统计时只保留
`/move_base/status` 为 `ACTIVE` 的区间；这些区间内命令最大间隔应小于
`0.30 s`、P95 应不超过 `0.15 s`。目标成功、取消、recovery 切换和下一个
前沿选点期间本来就应安全输出零速，不算导航命令断流。

可在另一个终端查看最终命令：

```bash
rostopic hz /cmd_vel
```

`cmd_mux` 正常应接近 `50 Hz`。

### 7.2 查看导航健康、标准 recovery 和兼容 recovery

以下命令每次选择一个运行：

```bash
rostopic echo /navigation/health
rostopic echo /move_base/recovery_status
rostopic echo /navigation/recovery_event
rostopic echo /move_base/status
```

卡死或不可达场景中，`/move_base/recovery_status` 应出现有限序列：

```text
conservative_reset
aggressive_reset
```

恢复失败后 Action 应进入 `ABORTED`，不能无限清图、旋转或持续撞击。

### 7.3 查看探索和危险源识别

```bash
rostopic echo /exploration/status
rostopic echo /danger_detector/status
rostopic echo /danger_detector/detections
rostopic echo /mission/status
```

### 7.4 建议录制诊断 rosbag

开始任务前运行：

```bash
mkdir -p /home/ruilinli/danger_search_ws/test_bags

rosbag record \
  -O /home/ruilinli/danger_search_ws/test_bags/standard_navigation_full.bag \
  /tf /tf_static \
  /map \
  /mapping/status \
  /mapping/current_floor \
  /mapping/active_map \
  /mapping/floors/1/map \
  /localization/pose \
  /localization/odom \
  /localization/scan \
  /localization/depth_obstacle_scan \
  /move_base/status \
  /move_base/recovery_status \
  /move_base/NavfnROS/plan \
  /move_base/DWAPlannerROS/local_plan \
  /danger_search/nav_cmd_vel \
  /danger_search/elevator_cmd_vel \
  /danger_search/cmd_vel_sent \
  /cmd_vel \
  /navigation/health \
  /navigation/recovery_event \
  /exploration/status \
  /danger_detector/status \
  /danger_detector/detections \
  /mission/status
```

测试结束后在录包终端按 `Ctrl-C`，不要强制关闭后直接拔掉终端。

### 7.5 检查多楼层换层合同

floor 0 完成、系统开始前往电梯厅时，分别采样 mapping、active-map 和探索状态：

```bash
rostopic echo -n 1 /mapping/status
rostopic echo -n 1 --noarr /mapping/active_map
rostopic echo -n 1 /exploration/status
rostopic echo /danger_search/transit_floor/status
rostopic echo /danger_search/elevator_cmd_vel
```

第一个 `TO_HALL` 目标应使用启动阶段保存的 `source=door_motion` 绑定，不应导航到电梯
背面、普通房间或远端墙角。开门成功后应直接进入 `ENTER`，不再执行一次“开—关—开”
扫描验证；进入时 `/danger_search/elevator_cmd_vel.linear.x` 应达到 `0.40`，跨越门槛的
定位进度不少于 `0.80 m`。

`/mapping/status` 与 `/mapping/active_map` 必须属于相同的 floor 和 map epoch。active-map 的
正版本允许暂时小于最新 mapping 版本，因为两条 ROS 连接异步发布；只要地图新鲜，系统不应
因此取消 `TO_HALL`。不得出现目标刚发出便连续报告：

```text
floor transit failed [UNREACHABLE_HALL]: hall navigation active map context is not committed
floor_transit_unavailable
```

换层启动前仍必须看到 `ready=True`、`stable=True`。厅导航运行后，move_base recovery
转向期间 mapping 的 ready/stable 可以短暂降级；只要 `lost=False`、
`transitioning=False`、位姿和 active-map 新鲜且 floor/epoch 合同有效，目标不应因此被取消。
`/navigation/recovery_event` 在换层阶段也不应增加 `/exploration/trap_blacklist`。

成功进入下一层后必须同时满足：

```bash
rostopic echo -n 1 /mapping/current_floor
rostopic echo -n 1 /mapping/status
rostopic echo -n 1 /mapping/floors/1/map
rostopic echo -n 1 /exploration/status
```

- `/mapping/current_floor` 为 `1`。
- `/mapping/status` 的 `current_floor` 为 `1`、`map_epoch` 已增加、`stable=True` 且
  `transitioning=False`。
- `/mapping/floors/1/map` 已发布，探索状态已离开换层阶段并继续 floor 1 探索。

## 8. 场景与验收项目

让探索至少覆盖以下情况：

1. 开放区域连续直行。
2. 90 度墙角转弯。
3. 家具旁绕行。
4. 家具夹缝候选目标。
5. U 形凹区或不可达前沿。
6. 危险源进入 RGB-D 视野。

重点观察：

- 开放直线路段大部分命令应为 `linear.y = 0`。
- `/danger_search/nav_cmd_vel` 活动阶段连续输出。
- `/cmd_vel` 不发生第二发布者抢占。
- global/local costmap 中 footprint 相交区域不可通行。
- 机器人卡住后 recovery 次数有限，最终恢复或 `ABORTED`。
- 失败后探索不立即重复选择同一夹缝或墙角目标。
- localization 位姿与 Gazebo 中的实际运动方向一致。
- 探索期间感知、建图、导航和 mission 同时保持运行。

## 9. 检查危险源结果文件

任务运行或结束后执行：

```bash
python3 -m json.tool /home/ruilinli/SimEnv/results/detected_danger.simulation_truth.json
```

同时对照真值文件：

```bash
python3 -m json.tool /home/ruilinli/SimEnv/results/danger_truth.json
```

结果文件为空时，先检查：

```bash
rostopic echo -n 1 /danger_detector/status
rostopic echo -n 1 /danger_detector/detections
rostopic hz /real_sense/rgb/image_raw
rostopic hz /real_sense/depth/image_raw
```

## 10. 正确停止顺序

1. 录包终端按 `Ctrl-C`。
2. danger_search 的终端二按 `Ctrl-C`，等待所有节点退出。
3. RViz 终端按 `Ctrl-C`。
4. 最后在 SimEnv 终端一按 `Ctrl-C`。

不要在 Gazebo 尚运行时再次执行第二份 `auto.sh`，也不要并行启动旧的 `nav_controller.py`、Unitree 自带 move_base 或第二个 cmd_mux。

## 11. 常见失败定位

### `/danger_search/start` 返回 `navigation_not_ready`

依次检查：

```bash
rostopic echo -n 1 /mapping/status
rostopic echo -n 1 /navigation/health
rostopic echo -n 1 /localization/pose
rostopic echo -n 1 /localization/scan
timeout 5 rostopic hz /localization/depth_obstacle_scan
rostopic echo -n 1 /move_base/status
```

### Gazebo 中机器人站立但完全不走

```bash
rostopic info /cmd_vel
rostopic echo -n 5 /danger_search/nav_cmd_vel
rostopic echo -n 5 /cmd_vel
```

若导航命令存在但 `/cmd_vel` 为零，检查 cmd_mux 的安全门和看门狗。若 `/cmd_vel` 非零但机器人不动：

1. 确认终端一已经明确显示 `[INFO] Entered RL /cmd_vel mode.`。
2. 检查路径跟踪期间 `/cmd_vel.linear.x` 是否达到 `0.30 m/s`，不要只看低于步态下限的瞬时加速帧。
3. 如果命令已稳定达到 `0.30` 但 Gazebo 真值仍无位移，将问题归入 RL policy/关节执行层，不要通过继续提高导航速度掩盖。

### 路径存在但 move_base 持续报 costmap 不可用

```bash
rostopic hz /localization/scan
rostopic hz /localization/depth_obstacle_scan
rostopic hz /localization/odom
rosrun tf tf_echo odom base
rosrun tf tf_echo map odom
```

不得通过启动 `state_from_gazebo` 的 TF 来补链路；`map -> odom -> base` 必须由 localization adapter 独占发布。
