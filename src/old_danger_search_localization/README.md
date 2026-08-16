# danger_search_localization

P0 定位与建图包。默认后端使用 FAST-LIO2 的 IMU 紧耦合 scan-to-map 状态估计，
不再以容易静止误匹配的 scan-to-scan GICP 或稀疏二维 Hector 作为定位源。旧实现仍保留
在源码中作为回退和对照，但默认 launch 不会启动它们。

## 数据链

```text
SimEnv /scan + TF -> sim_sensor_adapter -> /localization/lio/points
SimEnv /trunk_imu -----------------------> FAST-LIO2 (MARSIM mode)
                                           +-> /localization/lio/odometry
                                           +-> /localization/lio/cloud_registered
                                                       |
                              lio_occupancy_mapper -----+-> /map
                              lio_interface ----------------> 公共位姿、TF、状态
```

`sim_sensor_adapter` 使用机器人自身 TF 将原始雷达点统一到 `base`/IMU 坐标系，
FAST-LIO 因而使用单位外参，不需要手填容易写反的平移和旋转。MARSIM 模式把 SimEnv
同一仿真时刻生成的一帧点云视为瞬时扫描，不伪造逐点时间戳。地图节点从 LIO 校正后的
三维点云进行高度分层和射线更新，发布 navigation 使用的二维占据栅格。

占据栅格采用可逆的射线 log-odds 更新：障碍命中提高占用值，后续穿过该位置的自由射线
降低占用值。因此动态门打开后，原门板位置会随新点云逐步恢复为自由空间，而不是永久
保留为黑墙；navigation 应等待若干地图帧后再重试入口短目标。

本包不订阅 `/Odometry_gazebo`、裁判真值位姿或真值变换后的点云，也不要求修改
`SimEnv`。

## 公共输出（接口保持不变）

| 名称 | 类型 | 说明 |
|---|---|---|
| `/localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | 相对比赛起点的三维位姿 |
| `/map` | `nav_msgs/OccupancyGrid` | `-1` 未知、`0` 自由、`100` 占用 |
| `/tf` | TF | `map -> odom -> base` |
| `/localization/status` | `danger_search_common/LocalizationStatus` | 跟踪与跳变状态 |
| `/mapping/status` | `danger_search_common/MappingStatus` | 地图就绪、楼层与版本 |

首个有效 LIO 位姿会锚定为比赛起点 `(0,0,0)`。公共接口还有独立的物理速度门，
即使后端产生非有限值或不可能的单帧跳变，也不会直接传给 navigation。
后端同时根据每帧 IMU 的角速度、加速度模长和帧内变化检测真实静止状态；静止时
执行零速更新并锁定本帧位姿，避免连续的小幅错误匹配累积成米级漂移。检测到四足
行走产生的惯性变化后立即解除约束，恢复正常 LIO 跟踪。

## 编译与启动

```bash
cd ~/myProject/danger_search_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch danger_search_localization localization.launch
```

无 GUI 启动官方平台（在另一个终端）：

```bash
cd ~/myProject/SimEnv
source /opt/ros/noetic/setup.bash
source devel/setup.bash
GUI=false ENABLE_SENSOR_DATA=0 ENABLE_LIVOX=1 \
ENABLE_REALSENSE=0 ENABLE_LIVOX_IMU=0 \
ENABLE_REFEREE_ODOM=0 ENABLE_GROUND_TRUTH=0 \
POINTCLOUD_USE_GROUND_TRUTH_ODOM=0 ENABLE_POINTCLOUD_CONVERTER=0 \
./auto.sh
```

定位使用机器人机身已有的 `/trunk_imu`，因此额外的 Livox IMU 插件可关闭；完整 P0
需要相机识别危险源时，再把 `ENABLE_REALSENSE` 改回 `1`。上述命令只用于机器人不行走
的定位隔离测试；完整 P0 的模式 `6` 底层控制还需要设置 `ENABLE_GROUND_TRUTH=1`，但
本包不会订阅这些真值话题。

检查输出：

```bash
rostopic hz /localization/pose
rostopic echo -n 1 /localization/status
rostopic echo -n 1 /mapping/status
rostopic echo -n 1 /map --noarr
rosrun tf tf_echo map base
```

参数集中在 `config/default.yaml`。该文件为了回退和对照仍保留早期 Hector/GICP 参数，
但默认 `localization.launch` 只启动 `sim_sensor_adapter`、`fast_lio_mapping`、
`lio_occupancy_mapper` 和 `localization_adapter`，因此不要通过修改旧的 `lidar_odom_*`、
`fusion_*` 或 Hector 参数来调整当前 P0。

当前默认链路主要使用以下配置：

| 参数组 | 用途 | 调整时机 |
|---|---|---|
| `input_topic`、`min_range_m`、`max_range_m` | SimEnv 点云输入与距离过滤 | 点云范围明显不正确时 |
| `common`、`preprocess`、`mapping` | FAST-LIO2 传感器与状态估计 | 更换传感器或完成专项标定后 |
| `stationary_constraint` | 静止识别与零速约束 | 静止误漂或运动被误判静止时 |
| `map_*`、`map_resolution`、`map_size` | 二维占据地图 | 地面被标墙、矮障碍漏检或范围不足时 |
| `lio_guard_*` | 公共位姿的异常跳变保护 | 有明确日志证明正常运动被拒绝时 |

正常 P0 不需要修改这些默认值。尤其不要随意改变 `extrinsic_T`、`extrinsic_R`：传感器
适配节点已把点云统一到机身/IMU 坐标系，FAST-LIO 使用单位外参。FAST-LIO 核心源码及
原始许可证位于 `third_party/fast_lio`，来源和集成差异见其中 `README.vendor.md`。

MARSIM 时间链会在 IMU 初始化期间记录上一帧雷达时间。仿真负载造成合理传感器丢帧时，
较长 IMU 间隔会拆成不超过 10ms 的预测子步；负时间、非有限时间或超过 2s 的异常间隔
会回滚本帧状态和协方差。地图节点还会独立拒绝不符合物理速度/角速度约束的 LIO 位姿，
避免后端异常时继续污染 `/map`。

FAST-LIO 不再使用启动后的第一批 IMU 数据立即估计重力。只有陀螺、加速度幅值和加速度
波动同时满足阈值，并连续保持 `imu_initialization/stationary_hold_s` 后才完成初始化；若机器
人仍在下落、站立或晃动，累计数据会清空并重新等待。该检查只作用于初始化，完成后滤波器
仍估计完整 6DoF 位姿，不锁定水平、高度或姿态轴，因此可以继续扩展到楼梯和多楼层 P1。

## 当前范围

- P0 默认维护第 0 层的二维地图，同时输出真实 LIO `z/roll/pitch`；
- 多楼层地图切换与楼层编号仍属于后续 P1 功能；
- 当前没有回环检测，极长距离累计误差仍需在后续加入回环/重定位修正；
- `SimEnv` 的运动控制稳定性属于 control/仿真控制链问题，本包不会用真值掩盖它。
