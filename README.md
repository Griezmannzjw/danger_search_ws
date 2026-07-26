# 危险源自主搜索与识别系统

**挑战杯"揭榜挂帅"赛题 DG-2026 四足机器人危险源搜索**

当前版本：**v1.1-p0**（单楼层最小可运行闭环）

## 版本说明

v1.1-p0 是符合官方接口规范的最小可运行版本，实现完整的P0闭环：
- ✅ IMU积分航位推算 + 激光占据栅格建图（不用cmd_vel做里程计）
- ✅ 完整TF链：`world -> map -> odom -> base`
- ✅ 标准 `move_base` Action 服务器/客户端架构
- ✅ 基于实际地图的路径可达性检查（make_plan不返回直线）
- ✅ 探索选点严格前置条件检查（地图有效+定位稳定+自由区域+路径存在）
- ✅ 控制层安全仲裁 + 超时停车 + 加速度限制
- ✅ 任务状态机：IDLE → EXPLORING → FINISHED
- ✅ 危险源融合去重 + map→world坐标转换
- ✅ 原子写入官方格式结果文件
- ✅ 所有话题/服务/frame从参数读取，无硬编码

## 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                         Mission Manager                         │
│                   /danger_search/start /finish                  │
│              状态编排 | 检测融合 | 结果输出 | 坐标转换           │
└────────┬───────────────────────────────────────────┬────────────┘
         │                                           │
         ▼                                           ▼
┌─────────────────────┐                   ┌─────────────────────┐
│  Exploration        │                   │  Perception         │
│  - 自由区域选点     │                   │  - RGB/深度输入     │
│  - make_plan校验    │                   │  - 颜色检测         │
│  - move_base客户端  │                   │  - 发布检测结果     │
└────────┬────────────┘                   └──────────┬──────────┘
         │                                           │
         ▼                                           │
┌─────────────────────┐                             │
│  Navigation         │                             │
│  - move_base Action │                             │
│  - make_plan服务    │                             │
│  - P控制器跟踪      │                             │
│  - 发布nav_cmd_vel  │                             │
└────────┬────────────┘                             │
         │                                           │
         ▼                                           │
┌─────────────────────┐                             │
│  Control (cmd_mux)  │                             │
│  - 安全仲裁         │                             │
│  - 超时停车         │                             │
│  - 加速度限制       │                             │
│  - 唯一/cmd_vel     │                             │
└────────┬────────────┘                             │
         │                                           │
         ▼                                           │
┌─────────────────────┐                             │
│  Localization       │◄────────────────────────────┘
│  - IMU航位推算      │
│  - 激光占据栅格     │
│  - 唯一TF发布者     │
│  - 发布位姿和地图   │
└─────────────────────┘
```

## 包结构

```
danger_search_ws/
├── src/
│   ├── danger_search_common/       # 公共消息定义
│   ├── danger_search_localization/ # 定位与建图（P0实现）
│   ├── danger_search_navigation/   # 导航控制（P0实现）
│   ├── danger_search_exploration/  # 探索规划（P0实现）
│   ├── danger_search_control/      # 执行层与安全（完整实现）
│   ├── danger_search_perception/   # 危险源感知（骨架）
│   ├── danger_search_mission/      # 任务总控（P0实现）
│   └── danger_search_bringup/      # 启动与集成
├── docs/
│   └── INTERFACE_SPEC.md           # 接口规范 v1.1-p0
└── results/
```

## 快速开始

### 1. 编译

```bash
cd ~/danger_search_ws
source /opt/ros/noetic/setup.bash
catkin_make -j
source devel/setup.bash
```

### 2. 启动仿真

```bash
cd ~/SimEnv
source devel/setup.bash
./auto.sh
```

启动后：
- 按 **2** 站立
- 按 **6** 切换到 `/cmd_vel` 控制模式

### 3. 启动算法

```bash
cd ~/danger_search_ws
source devel/setup.bash
roslaunch danger_search_bringup competition.launch
```

### 4. 开始任务

```bash
# 开始探索
rosservice call /danger_search/start "{}"

# （探索完成后）结束任务，输出结果
rosservice call /danger_search/finish "{}"
```

结果文件会写入 `~/SimEnv/results/detected_danger.json`，格式兼容官方evaluator。

## P0 话题列表

| 话题 | 类型 | 发布者 | 说明 |
|------|------|--------|------|
| `/tf` | TFMessage | localization | map→odom→base 变换链 |
| `/localization/pose` | PoseWithCovarianceStamped | localization | 机器人位姿（map坐标系） |
| `/map` | OccupancyGrid | localization | 2D占据栅格地图 |
| `/mapping/status` | MappingStatus | localization | 建图状态 |
| `/navigation/health` | NavigationHealth | navigation | 导航健康状态 |
| `/danger_search/nav_cmd_vel` | Twist | navigation | 导航期望速度 |
| `/cmd_vel` | Twist | control | 最终速度指令（给机器人） |
| `/danger_detector/detections` | DangerSourceArray | perception | 危险源检测结果 |
| `/danger_detector/status` | DetectionStatus | perception | 检测器状态 |
| `/mission/status` | MissionStatus | mission | 任务状态（latch） |
| `/mission/active` | Bool | mission | 任务激活标志（latch） |

## P0 服务/Action

| 名称 | 类型 | 提供方 | 说明 |
|------|------|--------|------|
| `/move_base` | MoveBaseAction | navigation | 导航Action |
| `/move_base/make_plan` | GetPlan | navigation | 路径查询服务 |
| `/danger_search/start_exploration` | Trigger | exploration | 开始探索 |
| `/danger_search/stop_exploration` | Trigger | exploration | 停止探索 |
| `/danger_search/start` | Trigger | mission | 开始任务（用户调用） |
| `/danger_search/finish` | Trigger | mission | 结束任务（用户调用） |

## 结果文件格式

```json
{
  "exploration_time": 98.76,
  "detected_danger_sources": [
    {"position": [2.34, -1.56, 0.25]}
  ]
}
```

## 各模块后续升级方向

- **localization**: 升级为激光SLAM（如Cartographer/GMapping），加入回环检测
- **navigation**: 接入完整move_base（global planner + local planner + costmap_2d）
- **exploration**: 升级为前沿点算法（frontier exploration），支持多房间遍历
- **perception**: 接入YOLO/颜色+形状检测，结合深度图做3D定位
- **mission**: 加入自动返航、多楼层支持（电梯/楼梯）、自动结束判断
