# P0 闭环测试启动指南

本文用于在本机同时启动官方 `SimEnv` 和 `danger_search_ws`，验证单楼层 P0 最小闭环：

```text
机器人站立并进入 /cmd_vel 模式
  -> bringup 确认入口门打开
  -> 任务启动前检查
  -> mission 分段进入楼内
  -> 自主探索并识别危险源
  -> 探索收敛
  -> 返回出生点
  -> 写入 detected_danger.json
```

首次联调推荐使用无 GUI、手动启动任务的方式。这样可以先确认各模块就绪，再开始计时和移动。

## 1. 目录约定

默认目录结构为：

```text
~/myProject/
├── SimEnv/
└── danger_search_ws/
```

`competition.launch` 会根据这个同级目录结构，将结果文件写到：

```text
~/myProject/SimEnv/results/detected_danger.json
```

如果两个工程不在同一个父目录，启动算法时必须通过 `simenv_root` 指定 SimEnv 的绝对路径。

## 2. 首次测试前编译

这部分在代码没有变化时只需执行一次。为避免 WSL 内存压力，建议使用两个并行编译任务。

### 2.1 编译 SimEnv

```bash
cd ~/myProject/SimEnv
source /opt/ros/noetic/setup.bash
catkin_make -j2
```

### 2.2 编译 danger_search_ws

```bash
cd ~/myProject/danger_search_ws
source /opt/ros/noetic/setup.bash
catkin_make -j2
```

编译完成后确认一键启动文件可以被 ROS 找到：

```bash
source ~/myProject/danger_search_ws/devel/setup.bash
rospack find danger_search_bringup
```

命令应返回 `danger_search_ws/src/danger_search_bringup` 的路径。

### 2.3 当前配置文件分工

正常启动 P0 不需要逐项修改 YAML。各模块默认参数集中在各包的
`config/default.yaml`，其中本次改动直接相关的是：

| 文件 | 当前用途 | 通常是否修改 |
|---|---|---|
| `danger_search_localization/config/default.yaml` | FAST-LIO2、静止约束、二维地图与位姿保护 | 否；只在实测点云或地图异常时调参 |
| `danger_search_mission/config/default.yaml` | 任务预检、分段进门、探索、返航和结果写入 | 否；入口距离与场景不符时才调整 |
| `danger_search_bringup/launch/competition.launch` | 装配全部节点，传入 SimEnv 与结果路径 | 不直接改；通过 launch 参数覆盖 |

定位配置中仍保留 Hector/GICP 的旧参数，供回退和对照使用；默认
`localization.launch` 不启动旧节点，修改这些旧参数不会改变当前 FAST-LIO2 P0 链路。
当前生效的定位参数及调节原则见
`src/danger_search_localization/README.md`，不要为了处理静止漂移重新启用 Hector/GICP。

## 3. 终端一：启动 SimEnv

打开第一个终端：

```bash
cd ~/myProject/SimEnv
source /opt/ros/noetic/setup.bash
source devel/setup.bash
GUI=false \
ENABLE_SENSOR_DATA=1 \
ENABLE_LIVOX=1 \
ENABLE_LIVOX_IMU=0 \
ENABLE_REALSENSE=1 \
ENABLE_POINTCLOUD_CONVERTER=0 \
POINTCLOUD_USE_GROUND_TRUTH_ODOM=0 \
ENABLE_REFEREE_ODOM=0 \
ENABLE_GROUND_TRUTH=0 \
./auto.sh
```

这组参数保留 P0 必需的 Livox 点云、机身 IMU 和 RealSense RGB-D，关闭算法不使用的
Livox IMU、真值位姿和点云真值转换。终端摘要中应显示 `Ground truth topics: false`、
`Referee odom: false`、`PointCloud2 converter: false`。

等待终端出现以下含义相同的信息：

```text
Controller startup handshake complete.
```

然后在当前终端依次操作：

1. 按一次 `2`，让机器人站立。
2. 等待约 3～5 秒，确认机器人已经站稳。
3. 按一次 `6`，进入基于模型的 `/cmd_vel` 控制模式。

这里的 `2` 和 `6` 直接输入到运行 `auto.sh` 的终端，不需要输入命令，也通常不需要按回车。终端应显示：

```text
Switched from passive to fixed stand
Switched from fixed stand to trotting
```

不要按 `4`。模式 `4` 是 RL 键盘行走模式，不接收 P0 控制模块输出的 `/cmd_vel`。

完成上述操作后保持该终端运行。

## 4. 终端二：启动全部 P0 节点

首次测试使用 `autostart:=false`：

```bash
cd ~/myProject/danger_search_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch danger_search_bringup competition.launch autostart:=false
```

该 launch 会启动以下模块：

- localization：FAST-LIO2 激光雷达/IMU 定位、TF 和二维占据栅格地图；
- perception：RGB-D 危险源检测与 map 坐标定位；
- navigation：`/move_base` 导航服务；
- exploration：前沿探索与完成判定；
- control：速度平滑、超时停车并输出 `/cmd_vel`；
- mission：进门、探索、返航、检测汇总和结果写入；
- entrance_door：调用 SimEnv 服务打开主入口门。

保持该终端运行并观察是否出现红色错误日志。

主入口在官方当前场景中默认为打开状态；`entrance_door` 仍会在 launch 启动后调用
`/set_door_state` 再确认一次。只有服务成功返回后才发布 `/entrance/ready=True`，此时
mission 的启动预检才可能通过。该动作发生在调用 `/danger_search/start` 之前。

## 5. 终端三：执行启动前检查

打开第三个终端并加载工作空间：

```bash
cd ~/myProject/danger_search_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

### 5.1 检查入口门

```bash
rostopic echo -n 1 /entrance/ready
```

预期结果：

```text
data: True
```

这表示官方开门服务已经成功执行，不只是节点已经启动。实测关闭状态下两块门板位于
`x=+0.50/-0.50 m`，自动开门后移动到 `x=+1.06/-1.06 m`。

### 5.2 检查定位

```bash
rostopic echo -n 1 /localization/pose
```

应能收到有效位姿，并且 `header.frame_id` 为 `map`。

### 5.3 检查建图状态

```bash
rostopic echo -n 1 /mapping/status
```

开始任务前至少应满足：

```text
ready: True
stable: True
lost: False
```

### 5.4 检查地图

```bash
rostopic echo -n 1 /map | head -n 20
```

地图的宽、高和分辨率应为有效值，地图不能始终处于全未知状态。

地图采用可逆射线更新。若门曾被扫描为障碍，开门后穿过原门板位置的激光射线会逐帧将
该处从占用更新为未知、再更新为自由。入口短目标因此允许等待地图更新后重试，不需要
手工清图。

### 5.5 检查导航

```bash
rostopic echo -n 1 /navigation/health
```

预期至少满足：

```text
ready: True
```

### 5.6 检查感知

```bash
rostopic echo -n 1 /danger_detector/status
```

预期至少满足：

```text
ready: True
```

### 5.7 检查任务初始状态

```bash
rostopic echo -n 1 /mission/status
```

手动启动模式下预期为：

```text
mission_state: "IDLE"
```

### 5.8 检查关键服务和 Action

```bash
rosservice list | grep danger_search
rostopic list | grep move_base
```

至少应存在：

```text
/danger_search/start
/danger_search/finish
/danger_search/return_home
/danger_search/start_exploration
/danger_search/stop_exploration
/move_base/goal
/move_base/status
```

### 5.9 入口策略配置

入口参数位于：

```text
src/danger_search_mission/config/default.yaml
```

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `entry_enabled` | `true` | 开始探索前先执行自动进门 |
| `entry_distance_m` | `4.2` | 从门外任务起点量取的总前向进入距离 |
| `entry_step_m` | `0.6` | 每次下发的滚动短目标增量 |
| `entry_completion_tolerance_m` | `0.25` | 达到总距离前允许的完成误差 |
| `entry_retry_delay_s` | `1.0` | 单段失败后等待地图更新的时间 |
| `entry_max_retries` | `8` | 连续无有效进展时最多重试次数 |
| `entry_min_progress_m` | `0.10` | 判定一次尝试确实取得进展的最小前向位移 |
| `entry_timeout_s` | `90.0` | 整个 `ENTERING` 阶段的总超时 |
| `require_entrance_ready` | `true` | 未确认门打开时禁止开始任务 |

正常联调不需要修改这些值。`entry_step_m` 不宜小于导航的到点容差，也不应直接增大为
完整 4.2 m；过小可能被导航判定已到达，过大则重新引入未知区域长目标问题。

如果只想临时覆盖工程目录或是否自动启动，不要改源文件，使用 launch 参数：

```bash
roslaunch danger_search_bringup competition.launch \
  simenv_root:=/absolute/path/to/SimEnv \
  autostart:=false \
  open_main_entrance:=true
```

`result_file` 默认由 `simenv_root` 自动得到；确有赛事输出路径要求时，也可增加
`result_file:=/absolute/path/to/detected_danger.json`。正式 P0 应保持
`open_main_entrance:=true` 和 mission 配置中的 `require_entrance_ready: true`。

## 6. 开始 P0 闭环测试

确认机器人已经进入模式 `6`，并且上述状态全部正常后，在终端三执行：

```bash
rosservice call /danger_search/start "{}"
```

成功时返回：

```text
success: True
```

任务状态应按以下顺序变化：

```text
IDLE -> ENTERING -> EXPLORING -> RETURNING -> FINISHED
```

`ENTERING` 阶段终端应依次出现类似日志：

```text
[mission] entry segment ... sent: 0.00 -> 0.60 m
[mission] entry segment ... sent: 0.60 -> 1.20 m
...
[mission] entrance crossed: forward=... lateral=...
[mission] EXPLORING
```

某一段暂时不可达时可能出现 `entry segment failed ... retry`，这表示系统正在等待新地图
并自动恢复，不需要人工发送速度。只有耗尽连续重试或超过入口总时限才进入 `ERROR`。

持续观察任务状态：

```bash
rostopic echo /mission/status
```

也可以分别观察导航速度、检测和探索状态：

```bash
rostopic echo /cmd_vel
```

```bash
rostopic echo /danger_detector/detections
```

```bash
rostopic echo /exploration/status
```

一次终端只运行一个持续输出命令。使用 `Ctrl+C` 只会退出当前观察命令，不会停止后台任务。

## 7. P0 成功判定

一次测试至少应满足以下条件：

1. `/mission/status.mission_state` 最终为 `FINISHED`；
2. 机器人能够从出生点进入建筑；
3. 机器人在无人遥控的情况下执行探索；
4. 探索过程中 `/danger_detector/detections` 能输出有效检测；
5. 探索完成后机器人返回任务开始时保存的出生点；
6. 结果文件成功生成；
7. 全流程没有节点崩溃，也没有人工发送移动指令。

查看结果文件：

```bash
cat ~/myProject/SimEnv/results/detected_danger.json
```

格式应类似：

```json
{
  "exploration_time": 98.76,
  "detected_danger_sources": [
    {"position": [2.34, -1.56, 0.25]}
  ]
}
```

其中危险源坐标以本次任务的机器人出生点为原点，而不是以机器人当前机身为原点。

## 8. 调试时提前返航

需要提前结束探索并测试返航时，可以调用：

```bash
rosservice call /danger_search/return_home "{}"
```

也可以使用：

```bash
rosservice call /danger_search/finish "{}"
```

这两个接口会请求任务进入返航流程，不应在正式完整测试中代替探索模块的自动完成事件。

600 秒只是比赛满分时间线，不是强制结束时间。当前默认配置不会在 600 秒时自动停止任务。

## 9. 后续一键自动测试

确认手动启动的完整闭环稳定后，可改为自动启动：

```bash
cd ~/myProject/danger_search_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch danger_search_bringup competition.launch autostart:=true
```

`mission` 会等待入口门、定位、地图、导航和感知全部就绪，然后自动开始进门与探索。机器人仍须事先在 SimEnv 终端按 `2` 站立、按 `6` 进入 `/cmd_vel` 模式。

如果 SimEnv 不在默认位置：

```bash
roslaunch danger_search_bringup competition.launch \
  simenv_root:=/absolute/path/to/SimEnv \
  autostart:=true
```

## 10. 常见问题

### 10.1 调用 start 返回 Preflight failed

根据返回原因检查对应接口：

| 原因 | 检查项 |
|------|--------|
| `entrance_not_ready` | `/entrance/ready` 和 `/set_door_state` |
| `pose_missing` 或 `pose_stale` | `/localization/pose` |
| `mapping_status_missing`、`mapping_not_ready` | `/mapping/status` 和 `/map` |
| `navigation_health_missing`、`navigation_not_ready` | `/navigation/health` |
| `detection_status_missing`、`perception_not_ready` | `/danger_detector/status` 和相机话题 |
| `move_base_unavailable` | `/move_base/status` |
| `exploration_services_unavailable` | start/stop exploration 服务 |

### 10.2 mission 长时间停在 ENTERING

依次检查：

```bash
rostopic echo -n 1 /entrance/ready
rostopic echo -n 1 /mapping/status
rostopic echo -n 1 /navigation/health
rostopic echo /mission/status
```

同时观察 launch 终端中的 `entry segment` 日志。偶发一次失败后自动重试属于正常恢复；
反复显示同一前向进度通常意味着门口仍被占用、地图尚未清除、机器人未进入模式 `6`，
或者运动控制没有产生实际位移。

### 10.3 `/cmd_vel` 有数据但机器人不移动

先查看 SimEnv 终端是否显示机器人已经切换到 trotting。最常见原因是没有按 `6`，或者误按了 `4`。

```bash
rostopic hz /cmd_vel
rostopic echo -n 1 /cmd_vel
```

### 10.4 没有生成结果文件

```bash
rostopic echo -n 1 /mission/status
ls -l ~/myProject/SimEnv/results/detected_danger.json
```

检查 `mission_state` 是否已经进入 `FINISHED` 或 `ERROR`，并查看 mission 终端中的 `result=` 和 `result write failed` 日志。

### 10.5 机器人摔倒或仿真明显卡顿

P0 首次测试保持 `GUI=false`。不要同时开启 Gazebo GUI、RViz 和多个点云可视化窗口。摔倒属于运动控制或 SimEnv 物理联调问题，不能通过手动遥控继续完成正式测试。

WSL 建议至少分配 12 GiB 内存（16 GiB 更稳妥）和 4 GiB swap。完整 P0 同时包含 Gazebo、
机器人控制器、RGB-D、FAST-LIO2 和规划节点；如果 `free -h` 显示总内存只有约 7.4 GiB，
即使各模块单独测试通过，也可能在完整联调时触发 swap 抖动或 OOM。修改 Windows 用户目录下
`.wslconfig` 后需执行 `wsl --shutdown` 才会生效。

## 11. 安全关闭

测试结束后：

1. 在运行 `competition.launch` 的终端按一次 `Ctrl+C`；
2. 在运行 `SimEnv/auto.sh` 的终端按 `Ctrl+C` 停止控制器；
3. 如果终端提示 Gazebo 仍保留供检查，再按一次 `Ctrl+C` 完全关闭。

确认没有遗留进程：

```bash
pgrep -af 'auto.sh|gzserver|gzclient|junior_ctrl|mission_manager|exploration_planner|nav_controller'
```

没有输出即表示相关进程已经关闭。
