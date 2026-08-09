# danger_search_localization

定位与建图包。P0 使用 GICP 提供连续局部里程计 `odom -> base`，Hector 提供二维
地图和受限的全局修正 `map -> odom`。两者融合后的 `map -> base` 作为 navigation
位姿。后端仍可在不改变公共接口的前提下替换为 FAST-LIO 或其他 LIO。

## 当前数据链

```text
/scan (官方原始 PointCloud, laser_livox)
  +-> scan_projector.py -> /localization/scan -> hector_mapping
  |    -> /localization/hector_pose（全局地图匹配）
  |    -> /localization/raw_map
  |
  +-> lidar_odometry_node（三维 GICP 局部里程计）
       -> /localization/raw_pose (odom)

pose_estimator.py 对 Hector 修正执行同步、静止漂移抑制、跳变拒绝和低通应用，融合
GICP 后发布 `/localization/pose`、`/map`、状态和 `map -> odom -> base` TF。错误的
Hector 修正不会瞬移机器人，相关地图更新也会冻结到下一次安全修正；错误的 GICP
帧发布高协方差保持位姿，短暂失败不会造成话题断流，持续失败则安全降级/丢失。
```

本包不订阅 `/Odometry_gazebo`，也不订阅 SimEnv 默认可能使用真值里程计转换过的
`/livox/Pointcloud2`。正式运行 SimEnv 时必须同时关闭 referee odom 和点云真值变换，
避免禁止的节点向 TF 树写入 `map -> odom -> base`：

```bash
GUI=false \
ENABLE_REFEREE_ODOM=0 \
ENABLE_GROUND_TRUTH=1 \
POINTCLOUD_USE_GROUND_TRUTH_ODOM=0 \
./auto.sh
```

`ENABLE_GROUND_TRUTH=1` 仅供 SimEnv 的 `junior_ctrl` 获取步态策略观测；本包不订阅
这些真值话题，referee 里程计和真值变换点云仍由另外两个选项禁用。

点云投影会排除机器人自身范围；同一角度的回波先按距离聚类，只接受具有足够
多点支持的最近表面并使用簇中位数，随后删除没有相邻连续表面支持的孤立命中。
这可以避免单个近距离 Livox 噪点在地图中形成放射状黑线。相关阈值位于
`config/default.yaml`；若窄障碍被过度过滤，可以减小 `min_returns_per_bin`、增大
`max_intra_bin_range_gap` 或 `max_neighbor_range_jump`，也可以临时关闭
`enable_isolated_hit_filter`。

默认不跨帧叠加 Livox 扫描，因为未做运动补偿的历史帧会在行走时制造重影和假墙。
单帧必须至少包含 40 个有效 0.5 度 bin，并有至少 0.35 rad（约 20 度）的连续覆盖；
机器人倾斜、旋转过快或雷达离地异常时直接丢弃该帧，避免污染 Hector 地图。运行时
可用以下命令确认数据链：

```bash
rostopic hz /scan
rostopic hz /localization/scan
rostopic echo -n 1 /mapping/status
```

正常情况下 `/scan` 约为 10 Hz，`/localization/scan` 会在缓存填充后持续发布；Hector
收到地图更新后，`/mapping/status` 应变为 `ready: True`、`stable: True`。

## 公共输出

| 名称 | 类型 | 说明 |
|---|---|---|
| `/tf`、`/tf_static` | TF | `map -> odom -> base` |
| `/localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | `map` 中的当前位姿 |
| `/map` | `nav_msgs/OccupancyGrid` | 当前单楼层二维占据地图 |
| `/mapping/status` | `danger_search_common/MappingStatus` | 地图就绪、稳定、丢失、楼层和版本 |
| `/localization/status` | `danger_search_common/LocalizationStatus` | 定位跟踪和协方差状态 |

`/localization/pose` 第一帧定义为比赛出发点附近 `(0,0,0)`。GICP 对静止微动使用
死区，并按时间间隔限制物理可达位移；一次异常配准会保持上一位姿、提高协方差并
重建参考帧，成功帧到来后自动恢复。Hector 只允许小幅、同步的全局修正；GICP 判定
静止时出现的 Hector 漂移和米级跳变不会传给 navigation。

### 探索模块实际收到的地图

`/map` 通过 ROS 发布为 `nav_msgs/OccupancyGrid`，不是截图或点云。消息包含地图坐标
系、分辨率、宽高、原点以及一维栅格数组 `data`。每个栅格的含义是：`-1` 未知、
`0` 自由、`1..100` 为递增的占用概率。探索模块将 `data` 按 `height x width`
还原成二维数组，并结合 `/localization/pose` 中的机器人坐标选择自由栅格目标；它
还会读取 `/mapping/status`，只有地图 `ready && stable && !lost` 时才允许规划。

正常状态原因为 `TRACKING_FUSED_GICP_ODOMETRY_WITH_BOUNDED_HECTOR_CORRECTION`。
反复 GICP 失败或 Hector 修正被拒绝时状态先变为 `DEGRADED`，navigation 会安全停车；
持续局部里程计失败才会进入 `LOST`，有效数据恢复后自动回到 `TRACKING`。

## 编译与启动

安装运行依赖后：

```bash
sudo apt install -y ros-noetic-hector-mapping ros-noetic-pcl-ros
cd ~/myProject/danger_search_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch danger_search_localization localization.launch
```

查看接口：

```bash
rostopic echo /mapping/status
rostopic echo /localization/pose
rostopic echo /map --noarr
rosrun tf tf_echo map base
```

## 目前边界与升级项

- navigation 当前使用可靠性优先的二维 `x/y/yaw`，`z=0`；未经验证的 IMU 双积分
  默认关闭；
- 当前只维护 `current_floor=0`，尚未实现换层检测和分楼层地图；
- 2D 投影不能保留楼梯、门槛和坡面的完整高度信息；
- GICP 仍是 scan-to-scan 局部里程计，长时间弱特征运动可能降级；Hector 只以受限
  `map -> odom` 修正长期漂移；
- Hector 被判定为异常时会冻结公共地图，保证不会给 navigation 同时提供错误地图和
  正常状态；连续异常需要停车等待恢复，而不是冒险继续探索；
- 下一阶段应接入 Livox + IMU 的 LIO，并增加表面点云、可通行性和楼层管理；
- 后端升级时保持本 README 中的公共输出不变，探索、导航和感知无需跟着改。
