# danger_search_navigation

导航控制模块。

## 职责

1. 接收上层（探索模块）下发的目标点
2. 基于地图进行全局路径规划
3. 结合传感器进行局部避障与路径跟踪
4. 输出速度指令给控制层

## 当前状态

**骨架版本**：简易 P 控制器 + 占位式避障，仅用于框架联调。
正式版本需要接入完整的导航栈。

## 节点

### nav_controller.py
导航控制器主节点。

#### 订阅话题
| 话题 | 类型 | 说明 |
|------|------|------|
| `/danger_search/exploration_goal` | `geometry_msgs/PoseStamped` | 探索目标点 |
| `/danger_search/odom` | `nav_msgs/Odometry` | 机器人里程计 |
| `/map` | `nav_msgs/OccupancyGrid` | 栅格地图 |
| `/livox/Pointcloud2` | `sensor_msgs/PointCloud2` | 激光点云（局部避障） |

#### 发布话题
| 话题 | 类型 | 说明 |
|------|------|------|
| `/danger_search/nav_cmd_vel` | `geometry_msgs/Twist` | 导航速度指令 |

## 配置参数

见 `config/default.yaml`，主要包括：
- 最大线速度 / 角速度限制
- 到达目标阈值
- 障碍物安全距离
- 控制频率

## 升级路线

推荐方案：

1. **move_base 架构**（ROS 标准）
   - global_planner: Dijkstra / A* 全局规划
   - local_planner: DWA / TEB 局部规划
   - costmap_2d: 全局 + 局部代价地图

2. **自定义实现**
   - 全局：A* / RRT* 路径规划
   - 局部：DWA 动态窗口法
   - 代价地图：膨胀障碍物

## 与探索模块的边界

- **探索模块**：决定"去哪里"（目标点决策）
- **导航模块**：决定"怎么去"（路径规划与跟踪）
- 接口：`/danger_search/exploration_goal` (PoseStamped)
