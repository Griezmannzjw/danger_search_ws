# danger_search_localization

定位与建图模块。

## 职责

1. 估计机器人在世界坐标系下的位姿
2. 构建环境占据栅格地图（Occupancy Grid Map）
3. 维护 TF 树：`map → odom → base`

## 当前状态

**骨架版本**：使用简易航位推算（速度指令积分），仅用于框架联调。
误差会快速累积，**正式版本必须接入激光 SLAM**。

## 节点

### pose_estimator.py
位姿估计主节点。

#### 订阅话题
| 话题 | 类型 | 说明 |
|------|------|------|
| `/trunk_imu` | `sensor_msgs/Imu` | 机体 IMU 数据 |
| `/danger_search/cmd_vel_sent` | `geometry_msgs/Twist` | 已发送的速度指令 |
| `/livox/Pointcloud2` | `sensor_msgs/PointCloud2` | 激光点云（SLAM 用） |

#### 发布话题
| 话题 | 类型 | 说明 |
|------|------|------|
| `/danger_search/odom` | `nav_msgs/Odometry` | 机器人里程计 |
| `/map` | `nav_msgs/OccupancyGrid` | 环境栅格地图（SLAM 接入后发布） |

#### 发布 TF
- `map → odom`：SLAM 校正后的漂移（当前为单位变换）
- `odom → base`：里程计推算的位姿

## 配置参数

见 `config/default.yaml`。

## SLAM 升级路线

推荐方案（按优先级）：

1. **FAST-LIO**：适配 Livox 激光雷达，紧耦合 LIO，精度高、速度快
2. **Cartographer**：Google 开源，稳定性好，支持 2D/3D
3. **GMapping**：经典 2D SLAM，粒子滤波，简单易上手

接入 SLAM 后：
- 由 SLAM 节点发布 `map → odom` TF 和 `/map` 话题
- `pose_estimator` 只负责 `odom → base` 的里程计部分
- 本包可增加 SLAM 配置文件和 launch 文件

## 坐标系约定

- `map`：世界坐标系（原点=机器人出发点，比赛基准坐标系）
- `odom`：里程计坐标系，平滑但有漂移
- `base`：机器人基坐标系
