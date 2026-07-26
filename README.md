# 危险源自主搜索与识别系统

**挑战杯"揭榜挂帅"赛题 DG-2026 四足机器人危险源搜索**

当前版本：**v1.1-p0**（单楼层最小可运行闭环）

## 版本说明

v1.1-p0 完全对齐官方接口规范，实现完整的P0最小可运行闭环：

| 模块 | P0实现状态 |
|------|-----------|
| ✅ localization | IMU积分航位推算 + Livox激光2D占据栅格建图<br>唯一发布`world→map→odom→base`完整TF链<br>初始化2m自由区域（保证可选点）<br>不使用`cmd_vel_sent`作为正式里程计 |
| ✅ navigation | 标准`/move_base` SimpleActionServer<br>`/move_base/make_plan`基于实际地图检查障碍（不返回直线）<br>P控制器：先转向再前进，线速度0.3m/s<br>取消/失败/超时后立即停止速度 |
| ✅ exploration | `/move_base` Action客户端<br>严格检查所有前置条件才发目标：<br>  - 地图有效 + 定位稳定 + 不丢失<br>  - 目标位于已知自由栅格<br>  - `make_plan`返回非空路径<br>失败最多重试3次，不无限重试<br>stop时取消活动目标 |
| ✅ control | 安全仲裁层，唯一发布`/cmd_vel`<br>指令超时0.5s自动停车<br>加速度限制平滑输出 |
| ✅ perception | P0状态骨架，发布检测状态<br>实际颜色/形状检测算法待识别组补充 |
| ✅ mission | 状态机：IDLE → EXPLORING → FINISHED<br>严格start/finish幂等流程<br>只接收`class_id=1`红球，0.8m空间去重<br>map→world坐标转换<br>原子写入官方格式结果文件 |
| ✅ bringup | 统一launch文件，全局参数集中配置<br>所有话题/服务/frame从参数读取，无硬编码 |

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
├── CMakeLists.txt                    # 工作空间顶层
├── src/
│   ├── danger_search_common/         # 公共消息定义（9个msg）
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
- ROS Noetic
- Unitree A1 仿真环境（SimEnv）

### 1. 获取代码

```bash
cd ~
git clone https://github.com/Griezmannzjw/danger_search_ws.git
cd danger_search_ws
```

### 2. 编译

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j
source devel/setup.bash
```

### 3. 添加执行权限

```bash
cd ~/danger_search_ws/src
find . -name "*.py" -exec chmod +x {} \;
```

### 4. 配置结果文件路径

编辑 `src/danger_search_bringup/config/global.yaml`，确认`result_file`指向你的SimEnv结果目录：
```yaml
# 默认配置（如果SimEnv在~/SimEnv）
result_file: /home/$USER/SimEnv/results/detected_danger.json
```

### 5. 启动仿真

```bash
cd ~/SimEnv
source devel/setup.bash
./auto.sh
```

Gazebo启动后：
1. 按 **2** 让机器人站立
2. 按 **6** 切换到 `/cmd_vel` 控制模式

### 6. 启动算法系统

新开终端：
```bash
cd ~/danger_search_ws
source devel/setup.bash
roslaunch danger_search_bringup competition.launch
```

### 7. 开始任务

新开终端：
```bash
# 开始自主探索
rosservice call /danger_search/start "{}"

# 探索完成后结束任务（输出结果）
rosservice call /danger_search/finish "{}"
```

## 验证检查清单

启动后可以用以下命令验证系统是否正常：

```bash
# 1. 检查所有节点是否启动
rosnode list

# 2. 检查TF链是否完整（应该有world->map->odom->base）
rosrun tf view_frames
evince frames.pdf

# 3. 检查话题是否发布
rostopic echo /localization/pose -n 1
rostopic echo /map -n 1
rostopic echo /mapping/status -n 1
rostopic echo /navigation/health -n 1
rostopic echo /mission/status -n 1

# 4. 检查服务是否存在
rosservice list | grep danger_search

# 5. 检查地图是否有自由区域（不是全-1）
rostopic echo /map -n 1 | grep data
```

## P0话题列表

| 话题 | 类型 | 发布者 | 说明 |
|------|------|--------|------|
| `/tf` | TFMessage | localization | `world→map→odom→base` 变换链 |
| `/localization/pose` | PoseWithCovarianceStamped | localization | 机器人位姿（map坐标系） |
| `/map` | OccupancyGrid | localization | 2D占据栅格地图（latch） |
| `/mapping/status` | MappingStatus | localization | 建图状态 |
| `/navigation/health` | NavigationHealth | navigation | 导航健康状态 |
| `/danger_search/nav_cmd_vel` | Twist | navigation | 导航期望速度（给control） |
| `/cmd_vel` | Twist | control | 最终速度指令（给机器人） |
| `/danger_detector/detections` | DangerSourceArray | perception | 危险源检测结果 |
| `/danger_detector/status` | DetectionStatus | perception | 检测器状态 |
| `/mission/status` | MissionStatus | mission | 任务状态（latch） |
| `/mission/active` | Bool | mission | 任务激活标志（latch） |

## P0服务/Action

| 名称 | 类型 | 提供方 | 说明 |
|------|------|--------|------|
| `/move_base` | MoveBaseAction | navigation | 导航Action |
| `/move_base/make_plan` | GetPlan | navigation | 路径查询服务 |
| `/danger_search/start_exploration` | Trigger | exploration | 开始探索（mission调用） |
| `/danger_search/stop_exploration` | Trigger | exploration | 停止探索（mission调用） |
| `/danger_search/start` | Trigger | mission | 开始任务（用户调用） |
| `/danger_search/finish` | Trigger | mission | 结束任务（用户调用） |

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

## 常见问题

### Q: 机器人不动？
A: 检查：
1. 是否按了6切换到/cmd_vel模式
2. /navigation/health的ready是否为true
3. /mapping/status的ready和stable是否为true
4. /cmd_vel是否有速度输出

### Q: 不发导航目标？
A: exploration发目标需要同时满足：
- mapping_status.ready=true, stable=true, lost=false
- navigation_health.ready=true
- 地图中有自由区域
- make_plan返回非空路径

### Q: 结果文件没生成？
A: 检查：
1. result_file路径是否正确，目录是否存在
2. 是否调用了/danger_search/finish服务
3. mission节点日志是否有错误

## 后续升级方向

| 模块 | 升级方向 | 负责人 |
|------|---------|--------|
| localization | 激光SLAM（Cartographer/GMapping），回环检测 | 导航组 |
| navigation | 完整move_base（global planner + local planner + costmap） | 导航组 |
| exploration | 前沿点算法（frontier exploration），多房间遍历 | 探索组 |
| perception | YOLO/颜色+形状检测，深度图3D定位 | 识别组 |
| mission | 自动返航、多楼层支持（电梯/楼梯）、自动结束 | 框架组 |
