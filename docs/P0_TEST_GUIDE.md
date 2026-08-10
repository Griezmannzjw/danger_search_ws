# P0 闭环测试启动指南

本文用于在本机同时启动官方 `SimEnv` 和 `danger_search_ws`，验证单楼层 P0 最小闭环：

```text
机器人站立并进入 /cmd_vel 模式
  -> 打开入口门
  -> 任务启动前检查
  -> 进入楼内
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

## 3. 终端一：启动 SimEnv

打开第一个终端：

```bash
cd ~/myProject/SimEnv
source /opt/ros/noetic/setup.bash
source devel/setup.bash
GUI=false ./auto.sh
```

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

- localization：定位、TF 和二维地图；
- perception：RGB-D 危险源检测与 map 坐标定位；
- navigation：`/move_base` 导航服务；
- exploration：前沿探索与完成判定；
- control：速度平滑、超时停车并输出 `/cmd_vel`；
- mission：进门、探索、返航、检测汇总和结果写入；
- entrance_door：调用 SimEnv 服务打开主入口门。

保持该终端运行并观察是否出现红色错误日志。

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
rostopic echo -n 1 /map/info
```

如果系统没有单独发布 `/map/info`，使用：

```bash
rostopic echo -n 1 /map | head -n 20
```

地图的宽、高和分辨率应为有效值，地图不能始终处于全未知状态。

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

### 10.2 `/cmd_vel` 有数据但机器人不移动

先查看 SimEnv 终端是否显示机器人已经切换到 trotting。最常见原因是没有按 `6`，或者误按了 `4`。

```bash
rostopic hz /cmd_vel
rostopic echo -n 1 /cmd_vel
```

### 10.3 没有生成结果文件

```bash
rostopic echo -n 1 /mission/status
ls -l ~/myProject/SimEnv/results/detected_danger.json
```

检查 `mission_state` 是否已经进入 `FINISHED` 或 `ERROR`，并查看 mission 终端中的 `result=` 和 `result write failed` 日志。

### 10.4 机器人摔倒或仿真明显卡顿

P0 首次测试保持 `GUI=false`。不要同时开启 Gazebo GUI、RViz 和多个点云可视化窗口。摔倒属于运动控制或 SimEnv 物理联调问题，不能通过手动遥控继续完成正式测试。

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
