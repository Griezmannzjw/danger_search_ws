# danger_search_localization

定位与建图包。P0 的 navigation 位姿和二维地图都由 Hector 激光匹配提供；GICP
保留为不阻断 P0 的三维点云诊断里程计。后端仍可在不改变公共接口的前提下替换为
FAST-LIO 或其他 LIO。

## 当前数据链

```text
/scan (官方原始 PointCloud, laser_livox)
  +-> scan_projector.py -> /localization/scan -> hector_mapping
  |    -> /localization/hector_pose -> pose_estimator.py
  |    -> /localization/raw_map -> /map + /mapping/status
  |
  +-> lidar_odometry_node（三维 GICP，诊断）
       -> /localization/raw_pose

pose_estimator.py 对 Hector 位姿执行起点归零、抖动抑制和跳变拒绝后，发布
`/localization/pose`、`/localization/status` 和 `map -> odom -> base` TF。
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

Livox 单帧的角度覆盖并不完整。投影器会在 `base` 坐标系中累计最近
`scan_accumulation_frames` 帧（默认 5 帧，约 0.5 秒），仅保留同一角度 bin 至少被
`scan_accumulation_min_samples_per_bin` 帧观测到的距离中位数。有效 bin 数和连续覆盖
门限作用于这个稳定扫描，而不是任意单帧。运行时可用以下命令确认数据链：

默认的累计扫描门限为至少 8 个有效 0.5 度 bin、连续覆盖至少 0.05 rad。它们仅排除
空扫描和退化的单点回波；实际地图质量仍由 Hector 的连续更新和 `/mapping/status` 判断。

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

`/localization/pose` 第一帧始终定义为比赛出发点 `(0,0,0)`。正常连续运动经过低通
处理；静止时小于阈值的扫描匹配抖动不会传给 navigation。若后端出现超过速度上限
的位置或航向跳变，适配器保留最后可信位姿，并在连续异常后将定位状态标为 LOST。
GICP 连续配准失败不会影响 P0 位姿；它会以当前扫描重新建立参考帧，避免陈旧点云
造成连锁跳变。GICP 输出仅用于诊断和后续 LIO 升级，不是 navigation 输入。

### 探索模块实际收到的地图

`/map` 通过 ROS 发布为 `nav_msgs/OccupancyGrid`，不是截图或点云。消息包含地图坐标
系、分辨率、宽高、原点以及一维栅格数组 `data`。每个栅格的含义是：`-1` 未知、
`0` 自由、`1..100` 为递增的占用概率。探索模块将 `data` 按 `height x width`
还原成二维数组，并结合 `/localization/pose` 中的机器人坐标选择自由栅格目标；它
还会读取 `/mapping/status`，只有地图 `ready && stable && !lost` 时才允许规划。

`status_reason=TRACKING_FILTERED_2D_POSE_FIXED_COVARIANCE_NO_LOOP_CLOSURE`
明确表示对外提供的是经过安全过滤的局部里程计位姿，使用保守协方差且没有回环。
它不是最终多楼层定位方案。

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
- GICP 是局部 scan-to-scan 诊断里程计，没有回环，长距离仍会累计漂移；
- P0 使用 Hector 同时提供地图和位姿；后续 LIO 替换时必须保持这两项输出的一致性；
- 下一阶段应接入 Livox + IMU 的 LIO，并增加表面点云、可通行性和楼层管理；
- 后端升级时保持本 README 中的公共输出不变，探索、导航和感知无需跟着改。
