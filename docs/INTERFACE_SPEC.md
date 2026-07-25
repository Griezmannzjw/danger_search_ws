# 接口总览文档

## 版本说明
- 版本：v1.0
- 对齐：探索规划模块接口需求
- 优先级：P0（单楼层闭环必需）接口全部定义，首版为骨架实现

---

## 一、话题接口（Topics）

### 1. 定位与建图输出（P0）

| 话题 | 类型 | 发布者 | 频率 | 说明 |
|------|------|--------|------|------|
| `/tf` | `tf2_msgs/TFMessage` | localization | ≥10Hz | map → odom → base 变换链 |
| `/localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | localization | ≥10Hz | 带协方差位姿 |
| `/localization/status` | `LocalizationStatus` | localization | ≥5Hz+变化时 | 跟踪状态/漂移告警/修正版本 |
| `/map` | `nav_msgs/OccupancyGrid` | localization | ≥1Hz或变化时 | 当前楼层栅格地图 (-1未知/0自由/100占用) |
| `/mapping/status` | `MappingStatus` | localization | ≥1Hz+变化时 | ready/stable/lost/楼层/地图版本 |
| `/mapping/current_floor` | `std_msgs/Int32` | localization | 变化时 | 当前楼层（从0开始） |

### 2. 导航控制（P0）

| 话题 | 类型 | 发布者 | 说明 |
|------|------|--------|------|
| `/navigation/path` | `nav_msgs/Path` | navigation | 当前执行路径（P2建议） |
| `/navigation/health` | `NavigationHealth` | navigation | 导航健康状态（卡住/摔倒/进度/失败码） |
| `/danger_search/nav_cmd_vel` | `geometry_msgs/Twist` | navigation | 导航期望速度（给控制层安全仲裁） |
| `/danger_search/cmd_vel_sent` | `geometry_msgs/Twist` | control | 实际发送速度回显（给定位用） |
| `/cmd_vel` | `geometry_msgs/Twist` | control | 最终输出给机器人的速度指令 |

### 3. 危险源感知（P0）

| 话题 | 类型 | 发布者 | 说明 |
|------|------|--------|------|
| `/danger_detector/detections` | `DangerSourceArray` | perception | 危险源/干扰源检测结果数组 |
| `/danger_detector/status` | `DetectionStatus` | perception | 检测器可用性和输入新鲜度 |

### 4. 任务状态（P0）

| 话题 | 类型 | 发布者 | 模式 | 说明 |
|------|------|--------|------|------|
| `/mission/status` | `MissionStatus` | mission | latch | 任务阶段/楼层/进度/诊断 |
| `/mission/active` | `std_msgs/Bool` | mission | latch | 任务是否激活 |

### 5. 官方环境输入（来自 SimEnv）

| 话题 | 类型 | 使用模块 | 说明 |
|------|------|----------|------|
| `/livox/Pointcloud2` | `sensor_msgs/PointCloud2` | localization, navigation | Livox标准点云 |
| `/scan` | `sensor_msgs/PointCloud` | localization | Livox原始点云 |
| `/trunk_imu` | `sensor_msgs/Imu` | localization | 机体IMU |
| `/livox/imu` | `sensor_msgs/Imu` | (预留) | 雷达IMU |
| `/real_sense/rgb/image_raw` | `sensor_msgs/Image` | perception | RGB图像 |
| `/real_sense/depth/image_raw` | `sensor_msgs/Image` | perception | 深度图像 |
| `/real_sense/rgb/camera_info` | `sensor_msgs/CameraInfo` | perception | 相机内参 |

---

## 二、Action 接口（P0）

### /move_base (MoveBaseAction)

- **类型**：`move_base_msgs/MoveBaseAction`
- **提供方**：danger_search_navigation
- **消费方**：danger_search_exploration, danger_search_mission
- **用途**：发送、取消、监控当前楼层导航目标

**终端状态码：**
- `SUCCEEDED` - 到达目标
- `ABORTED` - 失败（UNREACHABLE / TIMEOUT / CONTROL_FAILED / ROBOT_FALLEN / LOCALIZATION_LOST）
- `PREEMPTED` - 被取消

---

## 三、服务接口（Services）

### 1. 导航服务（P0）

| 服务名 | 类型 | 提供方 | 说明 |
|--------|------|--------|------|
| `/move_base/make_plan` | `nav_msgs/GetPlan` | navigation | 判断可达性、计算路径长度 |
| `/move_base/clear_costmaps` | `std_srvs/Empty` | navigation | 清除代价地图，导航恢复 |

### 2. 探索控制服务

| 服务名 | 类型 | 提供方 | 说明 |
|--------|------|--------|------|
| `/danger_search/start_exploration` | `std_srvs/Trigger` | exploration | 开始探索 |
| `/danger_search/stop_exploration` | `std_srvs/Trigger` | exploration | 停止探索 |

### 3. 任务控制服务

| 服务名 | 类型 | 提供方 | 说明 |
|--------|------|--------|------|
| `/danger_search/start` | `std_srvs/Trigger` | mission | 开始整个任务 |
| `/danger_search/finish` | `std_srvs/Trigger` | mission | 结束任务并输出结果 |
| `/danger_search/return_home` | `std_srvs/Trigger` | mission | 触发返航 |

### 4. 官方环境服务（P1多楼层）

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/set_door_state` | `building_generator_interfaces/SetDoorState` | 开关动态门 |
| `/call_elevator` | `building_generator_interfaces/CallElevator` | 呼叫电梯 |

---

## 四、坐标系（TF Frames）

### TF 树
```
map (世界坐标系，出发点为原点)
  └── odom (里程计坐标系)
       └── base (机器人基坐标系)
            ├── trunk → imu_link
            ├── laser_livox → livox_imu_link
            └── real_sense
```

### 坐标约定
- X轴：机器人前方
- Y轴：机器人左方
- Z轴：向上
- 楼层索引：从0开始（0=1楼）

---

## 五、自定义消息定义

### 1. DangerSource.msg - 单个检测结果

| 字段 | 类型 | 说明 |
|------|------|------|
| detection_id | string | 单次检测唯一ID |
| track_id | string | 跟踪轨迹ID |
| class_id | uint8 | 0未知/1红球/2红方块/3绿球 |
| position | PointStamped | map坐标系位置（带时间戳） |
| position_covariance | float64[9] | 3x3位置协方差 |
| floor_id | int32 | 所在楼层 |
| confidence | float32 | 置信度0~1 |
| confirmed | bool | 是否已确认入库 |
| verification_required | bool | 是否需要异视角复核 |
| possible_duplicate_track_ids | string[] | 疑似重复轨迹ID |
| localization_correction_version | uint64 | 对应定位修正版本 |
| source_time | time | 原始检测时间 |

### 2. LocalizationStatus.msg - 定位状态

| 字段 | 类型 | 说明 |
|------|------|------|
| tracking_state | uint8 | 0INIT/1TRACKING/2DEGRADED/3RELOCALIZING/4LOST |
| pose_covariance_trace | float64 | 协方差迹 |
| drift_warning | bool | 漂移告警 |
| drift_rate_linear | float32 | 线漂移率 |
| drift_rate_angular | float32 | 角漂移率 |
| pose_jump_detected | bool | 位姿跳变 |
| last_correction_translation | float32 | 上次修正平移量 |
| last_correction_rotation | float32 | 上次修正旋转量 |
| correction_version | uint64 | 修正版本号 |
| relocalization_event_id | string | 重定位事件ID |
| last_stable_time | time | 上次稳定时间 |
| status_reason | string | 状态原因 |

### 3. MappingStatus.msg - 建图状态

| 字段 | 类型 | 说明 |
|------|------|------|
| ready | bool | 初始化完成 |
| stable | bool | 地图稳定 |
| lost | bool | 定位丢失 |
| current_floor | int32 | 当前楼层 |
| floor_maps | FloorMapInfo[] | 各楼层地图版本 |
| status_reason | string | 状态原因 |

### 4. NavigationHealth.msg - 导航健康

| 字段 | 类型 | 说明 |
|------|------|------|
| ready | bool | 导航就绪 |
| controller_active | bool | 控制器活跃 |
| stuck | bool | 卡住 |
| fallen | bool | 摔倒 |
| has_active_goal | bool | 有活动目标 |
| active_goal_id | string | 当前目标ID |
| progress | float32 | 执行进度0~1 |
| last_cmd_time | time | 上次发指令时间 |
| failure_code | string | 失败码 |
| failure_detail | string | 失败详情 |

### 5. DetectionStatus.msg - 检测器状态

| 字段 | 类型 | 说明 |
|------|------|------|
| ready | bool | 检测器就绪 |
| input_fresh | bool | 输入新鲜 |
| input_latency_ms | float32 | 输入延迟 |
| total_detections | uint32 | 当前帧检测数 |
| confirmed_count | uint32 | 已确认数 |
| pending_verification | uint32 | 待复核数 |
| capability_version | uint32 | 能力参数版本 |
| status_reason | string | 状态原因 |

### 6. MissionStatus.msg - 任务状态

| 字段 | 类型 | 说明 |
|------|------|------|
| mission_state | string | IDLE/EXPLORING/RETURNING/FINISHED/ERROR |
| current_floor | int32 | 当前楼层 |
| start_time | time | 任务开始时间 |
| elapsed_time | duration | 已耗时 |
| scored_exploration_time | duration | 计分有效时间 |
| active_goal_id | string | 当前目标ID |
| map_coverage_summary | string | 覆盖率摘要 |
| topology_debt_summary | string | 拓扑债务摘要 |
| room_visibility_summary | string | 房间可见域摘要 |
| remaining_frontier_count | uint32 | 剩余前沿数 |
| localization_correction_version | uint64 | 定位修正版本 |
| finish_reason | string | 结束原因 |

---

## 六、数据流向图

```
SimEnv 传感器输入
    │
    ├── /livox/Pointcloud2 ──► localization ──► /localization/pose + /map + /tf
    │                              │              /localization/status
    │                              │              /mapping/status
    ├── /trunk_imu ────────────────┘
    │
    └── /real_sense/* ───────► perception ────► /danger_detector/detections
                                       │              /danger_detector/status
                                       ▼
                              mission (融合去重)
                                       │
                                       ▼
                              /mission/status

exploration (决策"去哪")
    │
    ├── 订阅: /map, /localization/pose, /localization/status, /mapping/status
    │         /navigation/health, /danger_detector/detections, /danger_detector/status
    │
    └── 调用: /move_base Action
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

---

## 七、结果输出格式

文件：`results/detected_danger.json`

```json
{
  "exploration_time": 98.76,
  "detected_danger_sources": [
    {"position": [2.34, -1.56, 0.25]},
    {"position": [5.67, 3.21, 0.30]}
  ]
}
```

完全匹配比赛官方评估格式。

---

## 八、首版实现状态

| 接口 | 状态 | 说明 |
|------|------|------|
| /localization/pose | ✅ 骨架 | 航位推算，协方差占位 |
| /localization/status | ✅ 骨架 | 固定TRACKING状态 |
| /map | ✅ 骨架 | 全未知占位地图 |
| /mapping/status | ✅ 骨架 | 单楼层固定版本 |
| /mapping/current_floor | ✅ 骨架 | 固定0层 |
| /move_base Action | ✅ 骨架 | P控制器，结果码完整 |
| /move_base/make_plan | ✅ 骨架 | 直线路径占位 |
| /move_base/clear_costmaps | ✅ 骨架 | no-op占位 |
| /navigation/path | ✅ 骨架 | 直线路径 |
| /navigation/health | ✅ 骨架 | 基础字段有效 |
| /danger_detector/detections | ⚠️ 空骨架 | 检测算法待实现 |
| /danger_detector/status | ✅ 骨架 | 基础状态 |
| /mission/status | ✅ 完整 | 状态机+去重+结果输出完整 |
| 所有服务 | ✅ 完整 | 接口全部可用 |

P0 接口全部定义完成，骨架可跑通闭环。各模块只需替换内部算法，接口契约已固定。
