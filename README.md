# 危险源自主搜索与识别系统

**挑战杯"揭榜挂帅"赛题 DG-2026 四足机器人危险源搜索**

当前版本：**v1.1-p0**（单楼层最小可运行闭环）

## ✅ 接口对齐检查清单

| 规范要求 | 实现状态 |
|---------|---------|
| localization是`/tf`唯一发布者，提供`map→odom→base` | ✅ 20Hz发布，链完整 |
| 地图有已知自由区域（非全未知） | ✅ 初始化2m半径自由区域 |
| 定位用允许的IMU/激光，不用`cmd_vel_sent`做里程计 | ✅ 默认纯 GICP 位移 + 同源占据地图；Hector 仅为可选受限修正 |
| `make_plan`基于实际地图判断可达性，不返回无条件直线 | ✅ 膨胀占据栅格 A* |
| 所有话题/服务/frame从ROS参数读取，无硬编码 | ✅ 全部参数化 |
| 结果文件路径可移植 | ✅ launch自动查找同级SimEnv，可用一个参数覆盖 |
| 完整任务状态机 | ✅ EXPLORING→RETURNING→FINISHED/ERROR |
| control是`/cmd_vel`唯一发布者 | ✅ 安全仲裁层唯一输出 |
| exploration发目标前检查所有前置条件 | ✅ 6项条件全部满足才发 |
| 失败不无限重试，stop时取消活动目标 | ✅ 最多3次重试，stop立即cancel |
| 只接收`class_id=1`红球，去重 | ✅ 置信度过滤+空间融合+至少三帧确认 |
| 任务起点坐标转换 | ✅ 记录home pose并转换为起点相对坐标 |
| 结果文件原子写入 | ✅ flush+fsync+os.replace |
| 所有服务统一用`std_srvs/Trigger` | ✅ 无自定义srv |
| 自动结束和超时 | ✅ 探索收敛后自动返航；600s仅作评分参考，默认不强制停止 |

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
│  - make_plan校验    │                   │  - 颜色+形状检测    │
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
│  - GICP激光里程计   │
│  - 位姿门控/地图冻结│
│  - 唯一TF发布者     │
│  - 发布位姿和地图   │
└─────────────────────┘
```

## 包结构

```
danger_search_ws/
├── CMakeLists.txt                    # 工作空间顶层
├── src/
│   ├── danger_search_common/         # 公共消息定义（9个msg）
│   │   └── msg/
│   │       ├── DangerSource.msg
│   │       ├── DangerSourceArray.msg
│   │       ├── MissionStatus.msg
│   │       ├── DetectionStatus.msg
│   │       ├── LocalizationStatus.msg
│   │       ├── FloorMapInfo.msg
│   │       ├── MappingStatus.msg
│   │       └── NavigationHealth.msg
│   ├── danger_search_localization/   # 定位与建图（P0实现）
│   ├── danger_search_navigation/     # 导航控制（P0实现）
│   ├── danger_search_exploration/    # 探索规划（P0实现）
│   ├── danger_search_control/        # 执行层与安全（完整实现）
│   ├── danger_search_perception/     # 危险源感知（P0骨架）
│   ├── danger_search_mission/        # 任务总控（P0实现）
│   └── danger_search_bringup/        # 启动与集成
│       ├── launch/competition.launch # 统一启动文件
│       └── config/global.yaml        # 全局参数配置
├── docs/
│   └── INTERFACE_SPEC.md             # 官方接口规范 v1.1-p0
└── README.md
```

## 快速开始

### 环境要求
- Ubuntu 20.04
- ROS Noetic（desktop-full安装）
- Unitree A1 仿真环境（SimEnv）
- Python依赖：numpy（ROS Noetic自带）

### 1. 获取代码

```bash
cd ~
git clone https://github.com/Griezmannzjw/danger_search_ws.git
cd danger_search_ws
```

### 2. 编译

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j$(nproc)
source devel/setup.bash
```

**编译成功标志**：没有error，最后显示`Built target danger_search_common_generate_messages`等信息。

### 3. 添加执行权限

```bash
cd ~/danger_search_ws/src
find . -name "*.py" -exec chmod +x {} \;
```

### 4. 目录布局

默认只要求 `SimEnv` 和 `danger_search_ws` 位于同一父目录，不需要修改 YAML。不同布局可在
启动时通过 `simenv_root:=/absolute/path/to/SimEnv` 覆盖一次。

### 5. 启动仿真

```bash
cd ~/myProject/SimEnv
GUI=false FLOOR_COUNT=1 ENABLE_REFEREE_ODOM=0 ENABLE_GROUND_TRUTH=1 POINTCLOUD_USE_GROUND_TRUTH_ODOM=0 ./auto.sh
```

Gazebo启动后，在终端按：
1. 按 **2** 让机器人站立
2. 按 **6** 切换到 `/cmd_vel` 控制模式（必须！否则/cmd_vel不生效）

### 6. 启动算法系统

**新开终端**：
```bash
cd ~/myProject/danger_search_ws
source devel/setup.bash
roslaunch danger_search_bringup competition.launch autostart:=true
```

正常启动后会看到各节点的日志输出，没有红色error。

### 7. 自动闭环

`autostart:=true` 时，mission 在定位、地图、导航和感知全部就绪后自动开始。探索收敛后
自动返航并输出结果，不需要人工调用 finish。使用 `autostart:=false` 时，只需调用一次
`rosservice call /danger_search/start "{}"`。

## 启动后验证（必做！）

启动后按顺序执行以下命令验证系统正常：

```bash
# 1. 检查全部运行节点
rosnode list
# 默认应看到scan projector、GICP、local occupancy mapper、localization adapter及5个功能节点

# 2. 检查TF链是否完整
rosrun tf view_frames && evince frames.pdf
# 应该看到完整链路：map -> odom -> base

# 3. 检查定位是否正常
rostopic echo /localization/pose -n 1
# 应该有pose数据，frame_id是"map"

# 4. 检查地图是否正常
rostopic echo /map -n 1 | grep -A 5 info
# 应该有resolution=0.05, width=1024, height=1024
# data数组里应该有0（自由区域），不是全-1

# 5. 检查建图状态
rostopic echo /mapping/status -n 1
# ready: True, stable: True, lost: False

# 6. 检查导航状态
rostopic echo /navigation/health -n 1
# ready: True, controller_active: False（还没发目标）

# 7. 检查任务状态
rostopic echo /mission/status -n 1
# mission_state: "IDLE"

# 8. 检查所有服务是否存在
rosservice list | grep danger_search
# 应该看到：
# /danger_search/start
# /danger_search/finish
# /danger_search/return_home
# /danger_search/start_exploration
# /danger_search/stop_exploration
```

## P0话题列表

| 话题 | 类型 | 发布者 | 频率 | 说明 |
|------|------|--------|------|------|
| `/tf` | TFMessage | localization | 20Hz | `world→map→odom→base` 变换链 |
| `/localization/pose` | PoseWithCovarianceStamped | localization | 20Hz | 机器人位姿（map坐标系） |
| `/map` | OccupancyGrid | localization | 1Hz | 2D占据栅格地图（latch） |
| `/mapping/status` | MappingStatus | localization | 2Hz | 建图状态 |
| `/navigation/health` | NavigationHealth | navigation | 5Hz | 导航健康状态 |
| `/danger_search/nav_cmd_vel` | Twist | navigation | 20Hz | 导航期望速度（给control） |
| `/cmd_vel` | Twist | control | 50Hz | 最终速度指令（给机器人） |
| `/danger_detector/detections` | DangerSourceArray | perception | 10Hz | 危险源检测结果 |
| `/danger_detector/status` | DetectionStatus | perception | 2Hz | 检测器状态 |
| `/mission/status` | MissionStatus | mission | 2Hz | 任务状态（latch） |
| `/mission/active` | Bool | mission | latch | 任务激活标志 |
| `/exploration/complete` | Bool | exploration | latch | 探索收敛事件 |

## P0服务/Action

| 名称 | 类型 | 提供方 | 调用方 | 说明 |
|------|------|--------|--------|------|
| `/move_base` | MoveBaseAction | navigation | exploration | 导航Action |
| `/move_base/make_plan` | GetPlan | navigation | exploration | 路径查询服务 |
| `/danger_search/start_exploration` | Trigger | exploration | mission | 开始探索 |
| `/danger_search/stop_exploration` | Trigger | exploration | mission | 停止探索 |
| `/danger_search/start` | Trigger | mission | 用户 | 开始任务 |
| `/danger_search/finish` | Trigger | mission | 用户 | 结束任务 |
| `/danger_search/return_home` | Trigger | mission | 用户 | 调试时提前返航 |

## 结果文件格式

任务结束后，结果会写入 `result_file` 指定的路径，格式完全兼容官方evaluator：
```json
{
  "exploration_time": 98.76,
  "detected_danger_sources": [
    {"position": [2.34, -1.56, 0.25]}
  ]
}
```
- `exploration_time`：从start到finish的秒数，保留2位小数
- `position`：以本次任务起点为原点的[x, y, z]坐标，保留2位小数
- 匹配阈值：1.0m欧氏距离，贪心一对一匹配

## 常见问题排查

### ❌ 问题1：机器人不动
**排查步骤**：
1. 确认仿真里按了**6**切换到/cmd_vel模式（按4是键盘模式，/cmd_vel不生效）
2. 检查`/navigation/health`的`ready`是否为`True`
3. 检查`/mapping/status`的`ready`和`stable`是否为`True`
4. 手动发速度测试：`rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.1}" -r 10`，看机器人是否动
5. 如果手动能动但自动不动，检查exploration日志是否在选点

### ❌ 问题2：不发导航目标
**原因**：exploration发目标需要同时满足所有条件：
- `mapping_status.ready = True`
- `mapping_status.stable = True`
- `mapping_status.lost = False`
- `navigation_health.ready = True`
- 地图中有自由区域（不是全-1）
- `make_plan`返回非空路径
查看exploration节点的throttle日志，会提示等待哪个条件。

### ❌ 问题3：结果文件没生成
**排查步骤**：
1. 确认 `/mission/status` 已进入 `FINISHED` 或带原因的 `ERROR`
2. 检查 mission 启动日志中的 `result=` 是否指向同级 SimEnv 的 results 目录
3. 检查mission节点日志是否有`Result written to ...`
4. 检查是否有红色error日志

### ❌ 问题4：编译报错找不到消息
**解决**：
```bash
cd ~/danger_search_ws
rm -rf build devel
source /opt/ros/noetic/setup.bash
catkin_make -j$(nproc)
source devel/setup.bash
```

### ❌ 问题5：roslaunch找不到包
**解决**：
```bash
source ~/danger_search_ws/devel/setup.bash
rospack find danger_search_bringup
# 应该返回包路径，如果找不到，重新source
```

## 各模块升级方向

| 模块 | P0实现 | 升级方向 | 负责人 |
|------|--------|---------|--------|
| localization | 默认 GICP连续里程计 + 同源2D栅格；可选Hector受限修正 | LIO/回环检测、多楼层地图 | 导航组 |
| navigation | P控制器+直线避障 | 完整move_base：global planner(Dijkstra/A*) + local planner(DWA/TEB) + costmap_2d | 导航组 |
| exploration | 最近可达前沿+自动收敛 | 信息增益、房间拓扑、多楼层电梯/楼梯 | 探索组 |
| perception | RGB-D球体识别和map定位 | YOLO/实例分割、跨视角复核 | 识别组 |
| mission | 自动结束、返航和结果输出 | 多楼层切换、自动电梯调用 | 框架组 |
| control | 速度平滑+超时停车 | 跌倒检测，紧急避障，步态切换 | 控制组 |

## 团队分工

| 方向 | 人数 | 模块 |
|------|------|------|
| ROS框架 | 1人 | common, mission, bringup, 接口联调 |
| 导航控制 | 2人 | localization升级, navigation升级 |
| 探索规划 | 2人 | exploration升级, 多楼层 |
| 识别定位 | 2人 | perception检测算法, 3D定位 |

## 提交信息

- 仓库：https://github.com/Griezmannzjw/danger_search_ws
- 提交截止：2026年9月15日
- 评分标准：环境探索效率15分（≤600s满分）、危险源搜索效率22分、系统鲁棒性13分
