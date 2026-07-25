# Danger Search System - 四足机器人危险源自主搜索系统

基于 ROS Noetic 的四足机器人危险源自主搜索与识别系统，面向「挑战杯」揭榜挂帅赛题 **DG-2026 基于四足机器人的危险源自主搜索与识别技术**。

## 项目简介

本项目在官方 SimEnv 仿真平台（Unitree A1 + Gazebo Classic）基础上，开发一套完整的自主探索与危险源识别系统，包含**定位建图、自主探索、导航控制、危险源感知、任务总控**五大核心模块。

**接口版本**：v1.0（对齐探索规划模块接口需求）

## 环境版本

| 组件 | 版本 | 说明 |
|------|------|------|
| Ubuntu | 20.04 LTS | 操作系统 |
| ROS | Noetic Ninjemys | ROS1 发行版 |
| Gazebo | Classic 11.x | 物理仿真器 |
| Python | 3.8.x | 脚本开发语言 |
| OpenCV | 4.2.x | 图像处理（apt 安装） |
| SimEnv | 官方最新版 | 比赛仿真环境 |
| Unitree A1 Controller | junior_ctrl | 官方 RL 控制器 |

## 模块划分

| 功能包 | 职责 | 负责人 |
|--------|------|--------|
| `danger_search_common` | 公共消息、服务定义、工具函数 | 框架 |
| `danger_search_perception` | 危险源视觉检测与三维定位 | 识别定位（2人） |
| `danger_search_localization` | 激光 SLAM 定位与建图 | 导航控制（2人） |
| `danger_search_navigation` | move_base Action 服务器 + 路径跟踪 | 导航控制（2人） |
| `danger_search_exploration` | 未知环境探索策略（move_base 客户端） | 探索规划（2人） |
| `danger_search_control` | 速度仲裁、安全监控、加速度限制 | 框架 |
| `danger_search_mission` | 任务状态机、危险源融合去重、结果输出 | 框架 |
| `danger_search_bringup` | 统一启动文件、参数配置、集成测试 | 框架 |

## 快速开始

### 1. 环境准备

先确保官方 SimEnv 环境已配置完成并可正常运行：
```bash
cd ~/SimEnv
source /opt/ros/noetic/setup.bash
catkin_make -j
source ./devel/setup.bash
```

### 2. 编译本项目

```bash
cd ~/danger_search_ws
source /opt/ros/noetic/setup.bash

# 给 Python 脚本加执行权限
cd src
find . -name "*.py" -exec chmod +x {} \;
cd ..

catkin_make -j
source devel/setup.bash
```

### 3. 启动仿真环境（SimEnv）

```bash
cd ~/SimEnv
./auto.sh
# 终端按 2 站立，按 6 切换到 /cmd_vel 模式
```

### 4. 启动算法框架

```bash
roslaunch danger_search_bringup competition.launch
```

### 5. 开始任务

```bash
rosservice call /danger_search/start "{}"
```

任务完成后结果自动写入 `results/detected_danger.json`。

## 坐标系约定

- `map`：世界坐标系，原点为机器人出发点（比赛基准）
- `odom`：里程计坐标系
- `base`：机器人基坐标系
- `trunk`：机器人躯干坐标系
- `laser_livox`：Livox Mid-360 雷达坐标系（base前上方0.2m，Y轴倾斜45°）
- `real_sense`：RealSense D415 相机坐标系（base最前端0.28m）

TF 树：`map → odom → base → {trunk, laser_livox, real_sense}`

坐标约定：X前、Y左、Z上；楼层索引从0开始。

## 接口总览（v1.0）

### Action 接口（P0）

| Action | 类型 | 提供方 | 说明 |
|--------|------|--------|------|
| `/move_base` | `move_base_msgs/MoveBaseAction` | navigation | 导航目标执行/取消/反馈 |

### 核心话题（P0）

| 话题 | 类型 | 发布者 | 说明 |
|------|------|--------|------|
| `/tf` | `tf2_msgs/TFMessage` | localization | map→odom→base 变换链 |
| `/localization/pose` | `PoseWithCovarianceStamped` | localization | 带协方差位姿 |
| `/localization/status` | `LocalizationStatus` | localization | 跟踪状态/漂移/修正版本 |
| `/map` | `nav_msgs/OccupancyGrid` | localization | 当前楼层栅格地图 |
| `/mapping/status` | `MappingStatus` | localization | 建图状态/楼层/地图版本 |
| `/mapping/current_floor` | `std_msgs/Int32` | localization | 当前楼层（从0开始） |
| `/navigation/path` | `nav_msgs/Path` | navigation | 当前执行路径 |
| `/navigation/health` | `NavigationHealth` | navigation | 导航健康/卡住/失败码 |
| `/danger_search/nav_cmd_vel` | `geometry_msgs/Twist` | navigation | 导航期望速度（给控制层） |
| `/danger_search/cmd_vel_sent` | `geometry_msgs/Twist` | control | 实际发送速度回显 |
| `/cmd_vel` | `geometry_msgs/Twist` | control | 最终输出给机器人 |
| `/danger_detector/detections` | `DangerSourceArray` | perception | 危险源/干扰源检测结果 |
| `/danger_detector/status` | `DetectionStatus` | perception | 检测器状态 |
| `/mission/status` | `MissionStatus` | mission | 任务状态（latch） |
| `/mission/active` | `std_msgs/Bool` | mission | 任务激活状态（latch） |

### 服务接口（P0）

| 服务 | 类型 | 提供方 | 说明 |
|------|------|--------|------|
| `/move_base/make_plan` | `nav_msgs/GetPlan` | navigation | 路径规划查询 |
| `/move_base/clear_costmaps` | `std_srvs/Empty` | navigation | 清除代价地图 |
| `/danger_search/start_exploration` | `std_srvs/Trigger` | exploration | 开始探索 |
| `/danger_search/stop_exploration` | `std_srvs/Trigger` | exploration | 停止探索 |
| `/danger_search/start` | `std_srvs/Trigger` | mission | 开始整个任务 |
| `/danger_search/finish` | `std_srvs/Trigger` | mission | 结束任务输出结果 |
| `/danger_search/return_home` | `std_srvs/Trigger` | mission | 触发返航 |

### 导航结果码

`SUCCEEDED` / `UNREACHABLE` / `CANCELED` / `TIMEOUT` / `CONTROL_FAILED` / `ROBOT_FALLEN` / `LOCALIZATION_LOST`

### 危险源分类

| class_id | 含义 |
|----------|------|
| 0 | UNKNOWN |
| 1 | DANGER_RED_SPHERE（红球=危险源） |
| 2 | DISTRACTOR_RED_CUBE（红方块=干扰） |
| 3 | DISTRACTOR_GREEN_SPHERE（绿球=干扰） |

## 数据流向

```
SimEnv 传感器
    │
    ├── Livox点云 ──► localization ──► /localization/pose + /map + TF
    ├── IMU ────────►                 /localization/status
    │                                /mapping/status
    └── RGB+Depth ──► perception ────► /danger_detector/detections
                                       /danger_detector/status
                                               │
                                               ▼
                                         mission (融合去重)
                                               │
exploration (决策"去哪")                        ▼
    │                                    /mission/status
    ├── 订阅所有状态话题
    └── 调用 /move_base Action
                    │
                    ▼
              navigation (算速度)
                    │
                    ▼
              /danger_search/nav_cmd_vel
                    │
                    ▼
              control (安全仲裁+超时停车+加速度限制)
                    │
                    ▼
                   /cmd_vel ──► SimEnv 机器人
```

## 目录结构

```
danger_search_ws/
├── CMakeLists.txt                  # 工作空间顶层
├── src/
│   ├── danger_search_common/       # 公共消息与服务
│   │   ├── msg/                    # 9个自定义消息
│   │   └── srv/                    # 3个服务定义
│   ├── danger_search_perception/   # 危险源感知
│   ├── danger_search_localization/ # 定位与建图
│   ├── danger_search_navigation/   # 导航控制（move_base Action）
│   ├── danger_search_exploration/  # 探索规划
│   ├── danger_search_control/      # 执行层与安全
│   ├── danger_search_mission/      # 任务总控
│   └── danger_search_bringup/      # 启动与集成
├── docs/
│   ├── INTERFACE_SPEC.md           # 详细接口规范
│   └── ENVIRONMENT.md              # 环境说明
├── results/                        # 结果输出目录
└── README.md
```

## 开发规范

1. 每个模块独立开发、独立测试，通过约定好的 topic/service/action 对接
2. 接口契约固定后不得随意修改，内部实现各模块自由发挥
3. Python 节点放在 `scripts/` 目录，C++ 节点放在 `src/` 目录
4. 参数统一通过 launch 文件加载，默认配置放在 `config/default.yaml`
5. 所有话题名可通过 ROS 参数配置，不得在算法核心中硬编码
6. 所有空间数据必须显式提供 `frame_id` 和有效时间戳
7. 禁止使用 `/ground_truth/*`、`/Odometry_gazebo` 等Gazebo真值话题
8. 代码必须有充分注释，关键算法附原理说明

## 比赛输出

最终结果文件：`results/detected_danger.json`（完全匹配官方评估格式）

```json
{
  "exploration_time": 98.76,
  "detected_danger_sources": [
    {"position": [2.34, -1.56, 0.25]}
  ]
}
```

**重要说明**：
- 600s 是评分阈值不是硬截止，不得为赶时间跳过高价值房间
- 返航是可配置行为，不是结果有效的前置条件
- `scored_exploration_time` 计到结果冻结，返航不计入

## 评估命令

```bash
python3 ~/SimEnv/src/building_obstacles/scripts/evaluate_danger.py \
  --truth-file ~/SimEnv/results/danger_truth.json \
  --detected-file ./results/detected_danger.json \
  --output-file ./results/evaluation_result.json
```
