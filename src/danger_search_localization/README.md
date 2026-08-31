# danger_search_localization

定位与建图包。P0 默认使用 GICP 提供连续局部里程计 `odom -> base`，并由同一份
经验证位姿构建二维占据地图。Hector 受限全局修正仅是显式开启的兼容模式，不属于
默认数据链。后端仍可在不改变公共接口的前提下替换为 FAST-LIO 或其他 LIO。

### SimEnv 真值定位测试模式

为隔离验证 navigation、exploration、perception 和 mission，可显式启用 Gazebo 真值
定位。该模式读取 `/gazebo/link_states` 中的 `a1_gazebo::base`，将首次有效位姿定义为
`(x,y,z,yaw)=(0,0,0,0)`，并只替代内部 `/localization/raw_pose` 来源。位姿守卫、传感器建图、公共
状态和 `map -> odom -> base` TF 仍使用正常数据链，adapter 仍是唯一 TF 发布者。
`LinkStates` 没有消息时间戳，因此节点用接收时的仿真时间保存最近 1000 个 truth
位姿，并复用扫描投影模块的 SE(2) 插值按 `/scan.header.stamp` 配对；ROS 回调乱序时
不再拿“最新 truth”冒充扫描时刻，也不会仅因最新 truth 提前超过 50 ms 丢弃扫描。

```bash
roslaunch danger_search_bringup competition.launch \
  localization_source:=gazebo_truth
```

这是 SimEnv 联调专用入口，不得用于正式比赛。无需开启 `ENABLE_REFEREE_ODOM` 或
ground-truth TF；同时开启 referee odom 会造成重复 TF 发布。默认
`localization_source:=gicp` 保持不变。

真值联调模式会自动开启分楼层地图。SimEnv 的楼板相对高度由
`floor_heights: [0.0, 2.6, 5.2]` 配置；机器人位于楼层附近时，扫描只更新该层地图，
位于电梯运行区间等层间高度时扫描直接丢弃。每个访问过的楼层都有独立的地图核心，
因此换层不会把墙体投影进上一层，返航到旧楼层时会恢复旧图并继续增加该层版本。
地图在一次任务进程内保存在内存中；新随机场景或显式调用
`/localization/reset_map` 时会清空所有楼层，避免跨场景复用旧图。

## 当前数据链

```text
/scan (官方原始 PointCloud, laser_livox)
  |
  +-> lidar_odometry_node（生产 LidarOdometryCore，三维 GICP）
       -> /localization/raw_pose (odom，仅诊断)
       -> pose_estimator.py（首帧归零、死区/低通、物理跳变门控）
            +-> /localization/validated_pose (odom，仅健康帧)
            |    +-> scan_projector.py -> /localization/scan（局部避障，持续）
            |          +-> /localization/mapping_scan（长期建图，旋转安全时）
            |               +-> local_occupancy_mapper
            |                    +-> /localization/raw_floor_map（楼层+版本+地图原子消息）
            |                    +-> /mapping/floors/<floor_id>/map（各层留存图）
            +-> /localization/pose、/map、状态和 map -> odom -> base TF

导航和建图都不直接消费 `/localization/raw_pose`；投影、长期建图和 TF 使用同一份
`/localization/validated_pose`。错误 GICP 帧会被保持在上一可信位姿，
不会发布到 `/localization/validated_pose`，因此地图同步冻结；连续异常使状态先降级再
进入 LOST。系统不订阅 `/cmd_vel` 或 `/danger_search/cmd_vel_sent` 来计算位置。

只有启动 `use_hector_correction:=true` 时，`/localization/mapping_scan` 才进入 Hector；Hector
只提供经过同步和幅度限制的 `map -> odom` 修正，不能替换 GICP 物理平移。
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
上述约束针对默认 GICP 正式模式；只有显式 `gazebo_truth` 测试模式会读取 Gazebo
`/gazebo/link_states`。

点云投影会排除机器人自身范围；同一角度的回波先按距离聚类，只接受具有足够
多点支持的最近表面并使用簇中位数，随后删除没有相邻连续表面支持的孤立命中。
这可以避免单个近距离 Livox 噪点在地图中形成放射状黑线。相关阈值位于
`config/default.yaml`；若窄障碍被过度过滤，可以减小 `min_returns_per_bin`、增大
`max_intra_bin_range_gap` 或 `max_neighbor_range_jump`，也可以临时关闭
`enable_isolated_hit_filter`。

投影使用短窗口叠加 Livox 扫描补足单帧的稀疏角度覆盖；窗口较短以限制未做运动补偿
造成的重影。原始 PointCloud 没有点级时间戳，因此 `|wz|>=0.35 rad/s` 时仅暂停
`/localization/mapping_scan` 的长期落图；`/localization/scan` 保持发布给局部 costmap。
角速度降至 `0.20 rad/s` 以下并稳定 `0.5 s`、累计 3 帧后恢复建图。机器人倾斜或雷达
离地异常时仍会拒绝对应扫描。运行时
可用以下命令确认数据链：

```bash
rostopic hz /scan
rostopic hz /localization/scan
rostopic echo -n 1 /mapping/status
```

正常情况下 `/scan` 约为 10 Hz，`/localization/scan` 会在缓存填充后持续发布；可信
GICP 位姿和地图更新建立后，`/mapping/status` 应变为 `ready: True`、`stable: True`。

## 公共输出

| 名称 | 类型 | 说明 |
|---|---|---|
| `/tf`、`/tf_static` | TF | `map -> odom -> base` |
| `/localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | `map` 中的当前位姿 |
| `/map` | `nav_msgs/OccupancyGrid` | 当前楼层二维占据地图；换层稳定前暂停更新 |
| `/mapping/current_floor` | `std_msgs/Int32` | 当前确认楼层，编号从 0 开始 |
| `/mapping/floors/<id>/map` | `nav_msgs/OccupancyGrid` | 已访问楼层的独立、latched 地图 |
| `/localization/reset_map` | `std_srvs/Empty` | 任务停止后清空所有分楼层占据图并发布全未知地图 |
| `/mapping/status` | `danger_search_common/MappingStatus` | 地图就绪、稳定、丢失、楼层和版本 |
| `/localization/status` | `danger_search_common/LocalizationStatus` | 定位跟踪和协方差状态 |

`/localization/pose` 第一帧定义为比赛出发点附近 `(0,0,0)`。GICP 对静止微动使用
死区，并按时间间隔限制物理可达位移；GICP 使用最近 5 个可信扫描构造有限局部子地图，
以匹配质量、有效对应比例和物理速度门共同拒绝错误局部最优。连续拒绝前两帧会保持
可信参考；达到阈值后才用当前帧重建参考，且必须连续 2 帧成功才恢复健康。异常配准
会保持上一位姿并停止污染地图。可选 Hector 模式只允许小幅、同步的全局修正；GICP
判定静止时出现的 Hector 漂移和米级跳变不会传给 navigation。

### 探索模块实际收到的地图

`/map` 通过 ROS 发布为 `nav_msgs/OccupancyGrid`，不是截图或点云。消息包含地图坐标
系、分辨率、宽高、原点以及一维栅格数组 `data`。每个栅格的含义是：`-1` 未知、
`0` 自由、`1..100` 为递增的占用概率。探索模块将 `data` 按 `height x width`
还原成二维数组，并结合 `/localization/pose` 中的机器人坐标选择自由栅格目标；它
还会读取 `/mapping/status`，只有地图 `ready && stable && !lost` 时才允许规划。

默认正常状态原因为 `TRACKING_GICP_ODOMETRY_WITH_LOCAL_OCCUPANCY_MAP`。只有可选
Hector 模式正常时才显示 `TRACKING_FUSED_GICP_ODOMETRY_WITH_BOUNDED_HECTOR_CORRECTION`。
反复 GICP 失败、位姿门控拒绝或 Hector 修正被拒绝时状态先变为 `DEGRADED`，navigation 会安全停车；
持续局部里程计失败才会进入 `LOST`，有效数据恢复后自动回到 `TRACKING`。

换层期间 `/mapping/status` 明确发布 `stable=false`，原因为
`FLOOR_TRANSITION_WAITING_FOR_CURRENT_MAP`。确认新楼层并积累至少
`min_map_updates_for_stable` 次新扫描后，`/map` 才切换到该层；
`floor_maps[]` 同时保留所有已访问楼层各自的版本与最后更新时间。

正式 GICP 后端当前只估计 SE(2)，其 `raw_pose.z` 固定为 `0`，不能单独确认电梯换层。电梯联调在 `/call_elevator` 响应已确认目标层后调用 `/localization/set_current_floor`（`danger_search_common/SetCurrentFloor`）；adapter 将该命令确认作为分层地图选择覆盖，并继续等待目标层地图刷新。该接口不读取 Gazebo 真值、世界文件或生成场景元数据。

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
rostopic echo /mapping/current_floor
rostopic echo /localization/pose
rostopic echo /map --noarr
rostopic echo /mapping/floors/0/map --noarr
rosrun tf tf_echo map base
# 仅在任务停止且导航目标已取消后调用
rosservice call /localization/reset_map "{}"
```

无需启动 SimEnv 的确定性分层回归测试：

```bash
rostest danger_search_localization multifloor_pipeline.test
```

该测试发布合成位姿和扫描，验证 `0 -> 1 -> 0` 换层、层间扫描拒绝、地图互不
污染以及返层恢复；它不替代后续真实电梯和完整 P1 闭环测试。

## 目前边界与升级项

- 默认正式 GICP 模式仍使用可靠性优先的二维 `x/y/yaw`，未经验证的 IMU 双积分
  默认关闭；当前分楼层能力只在显式 Gazebo 真值联调模式中自动启用；
- 已实现当前层 `/map`、所有已访问层地图留存、换层隔离和返层恢复；电梯调用、
  跨层目标调度和全楼层结束条件属于 mission/exploration 的后续工作；
- 2D 投影不能保留楼梯、门槛和坡面的完整高度信息；
- GICP 以最近可信扫描组成的有限局部子地图配准，长时间弱特征运动仍可能降级；可选
  Hector 只以受限 `map -> odom` 修正长期漂移；
- GICP 位姿门控或可选 Hector 被判定为异常时会冻结公共地图，保证不会给 navigation 同时提供错误地图和
  正常状态；连续异常需要停车等待恢复，而不是冒险继续探索；
- 下一阶段应让正式 LIO 提供可靠高度，再为导航/探索接入电梯状态机和跨层调度；
- 后端升级时保持本 README 中的公共输出不变，探索、导航和感知无需跟着改。
