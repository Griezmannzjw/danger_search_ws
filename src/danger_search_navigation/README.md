# danger_search_navigation

nav_controller.py 是本包唯一的 ROS 节点入口和唯一的 /move_base Action Server。它只发布 /danger_search/nav_cmd_vel，从不发布 /cmd_vel；最终 /cmd_vel 的仲裁和发布仍由 danger_search_control 负责。

## 对外接口

输入：

| 接口 | 类型 | 实际用途 |
|---|---|---|
| /localization/pose | geometry_msgs/PoseWithCovarianceStamped | map 坐标系中新鲜、数值有效的机器人位姿 |
| /map | nav_msgs/OccupancyGrid | 单楼层全局规划占据地图 |
| /mapping/status | danger_search_common/MappingStatus | ready && stable && !lost 的定位/建图门 |
| /scan | sensor_msgs/PointCloud | 仅作局部临时动态障碍保护，不参与定位 |
| /danger_search/safety_stop | std_msgs/Bool | 外部紧急停车门 |

输出：

| 接口 | 类型 | 实际语义 |
|---|---|---|
| /move_base | move_base_msgs/MoveBaseAction | 唯一导航 Action Server |
| /move_base/make_plan | nav_msgs/GetPlan | 与 Action 共用同一 A* 规划器的路径查询 |
| /move_base/clear_costmaps | std_srvs/Empty | 兼容服务；有活动目标时请求用当前地图/障碍数据重规划 |
| /danger_search/nav_cmd_vel | geometry_msgs/Twist | 给控制层的导航速度请求 |
| /navigation/health | danger_search_common/NavigationHealth | 实际 readiness、目标生命周期、进度、命令时刻和失败原因 |
| /navigation/recovery_event | danger_search_common/RecoveryEvent | 卡死触发、恢复动作、尝试次数、实际位移和最小净空 |
| /navigation/global_path | nav_msgs/Path | 当前经过 footprint 扫掠验证的全局路径（RViz 诊断） |
| /navigation/local_trajectory | nav_msgs/Path | 当前选中的局部采样轨迹（RViz 诊断） |
| /navigation/footprint | geometry_msgs/PolygonStamped | 当前腿部投影与最近 0.4 s 步态扫掠包络（RViz 诊断） |
| /navigation/trap_blacklist | nav_msgs/GridCells | 本任务内拒绝再次进入的卡死通道栅格（RViz 诊断） |

## 规划、跟踪与安全行为

- navigation_core.py 不依赖 ROS。make_plan、Action 启动、地图更新和动态障碍阻断后的重规划都调用同一个带净空代价和黑名单约束的 A* 入口。
- 地图内容未变化时只更新时间戳，不重复构建 planner；实际变化时使用 NumPy/OpenCV 圆形膨胀并原子替换规划器，避免 1024² 地图重建阻塞 /scan。
- /map 中 -1、达到 `occupied_threshold`（默认 65）的栅格、地图外部和长期黑名单不可通行；低于阈值的已观测概率格允许通行。A* 对低于 `0.05 m` 的剩余净空施加轻量递增代价，并硬拒绝 footprint 扫掠净空低于 `0.02 m` 的路径。静态栅格的基础膨胀为 `robot_radius = 0.30 m`，最终路径与局部轨迹仍使用当前完整 footprint 逐姿态复核。地图 origin.position 和二维 yaw 都参与 world/cell 转换，负坐标使用 floor 语义。
- 每个目标初始只规划一次；仅当前路径被阻断、偏离超过 `0.80 m`、到达投影终点、收到 clear_costmaps 请求或 recovery 成功时才重规划。初始规划和事件重规划期间，navigation 通过现有 20 Hz 控制定时器持续发布零速度心跳；规划短暂失败时按 `planning_failure_tolerance_s`（默认 `5.0 s`）安全重试，超时后才返回 `UNREACHABLE`。控制层 `cmd_timeout_s` 保持为独立的节点失联看门狗。
- 全局路径的直线剪枝只在整段 footprint 扫掠安全时生效。正常跟踪采用轻量 DWA 式采样，联合选择 `linear.x >= 0`、`linear.y` 和 `angular.z`，按路径偏差、目标收益、最小净空和速度平滑度评分；进入未知、障碍或黑名单的轨迹直接淘汰。
- `/robot_description` 中 trunk、hip、thigh、calf、foot 的 collision 几何通过腿部 TF 投影到 base 平面；导航使用当前凸包与最近 `0.4 s` 投影的并集并外扩 `0.02 m`。TF 短时不可用时使用保守矩形回退，持续失效则停车。/scan 自体过滤仍逐碰撞体判断，不会用整个凸包删除腿间家具回波；整帧点云使用 NumPy 批量变换和体积判断，避免逐点 Python 循环积压 10 Hz 扫描。
- /scan 根据消息 frame、时间戳和 TF 查询实际的 LiDAR-to-base 外参。最新原始帧立即参与前向 `0.45 m` 硬停车和全部局部/恢复轨迹碰撞检查；确认动态障碍再进入 A*。`obstacle_cloud_timeout` 是 /scan 新鲜度门，和障碍 TTL 是不同概念。require_obstacle_cloud 为 true 时，缺失、过期或 TF 无效的点云会立即停车并以 CONTROL_FAILED 结束活动目标。
- 有效运动命令 12 s 内真实位移不足 `0.03 m`，或所有正常轨迹连续 5 s 无进展时进入有限恢复。恢复优先闭环后退 `0.20-0.50 m`，后方不安全则选择净空持续改善的左右横移；新障碍、输入过期或 5 s 无位移立即停车。每个目标最多两次、每次最长 15 s，成功后带卡死区域黑名单重规划，失败则返回 `CONTROL_FAILED`。
- 原始目标不可达时，可在 `goal_projection_max_radius` 内选择安全跟踪终点。投影终点使用 `projection_tracking_tolerance`（默认 `0.10 m`）跟踪，但 Action 只有进入原始目标的 `goal_tolerance_xy` 后才成功，避免滚动短目标零位移完成。
- 当目标栅格被门框或临时障碍阻挡时，Action 和 `/move_base/make_plan` 会在目标 `0.35 m` 内按 `0.05 m` 网格寻找可达安全落点。落点仍须满足原始目标的 `0.45 m` XY 容差；如果没有候选点，才报告 `UNREACHABLE`。
- 位姿、地图和 MappingStatus 必须有正确 map 帧、合法数值/四元数及新鲜时间戳。MappingStatus 还必须 ready=true、stable=true、lost=false；不满足时 readiness 为 false，活动目标以 LOCALIZATION_LOST 安全结束。
- safety_stop=true 会立即向 /danger_search/nav_cmd_vel 发布零速度，并以 SAFETY_STOP 结束活动目标。取消、不可达、超时、卡住、地图失效和节点关闭也都会显式发布零速度。
- NavigationHealth 的 active_goal_id、progress 和 last_cmd_time 分别来自实际 Action GoalID、累计路径长度和实际导航命令发布时间，绝不用 health 发布时刻伪造。P0 没有摔倒传感器，fallen 始终为 false，不能解读为已实现摔倒检测。

## 参数与独立启动

所有参数均是节点私有参数，见 config/default.yaml。包内 launch 会在 nav_controller 节点内部加载 YAML；比赛 launch 也必须以同样方式加载，避免参数落到根命名空间。

    source /opt/ros/noetic/setup.bash
    source /home/ruilinli/SimEnv/danger_search_ws/devel/setup.bash
    roslaunch danger_search_navigation navigation.launch

该命令只启动导航节点。不要并行启动第二个 /move_base Server、第二套导航 TF 发布者，或绕过 control 直接向 /cmd_vel 发布。

## 静态测试

规划核心测试不依赖 ROS master、Gazebo 或真值数据，覆盖 A* 绕障、不可达、unknown/occupied/地图外阻断、窄通道膨胀、动态障碍膨胀、旋转原点与负坐标、取消停车语义、最终 yaw 和 health 状态字段：

    source /opt/ros/noetic/setup.bash
    cd /home/ruilinli/SimEnv/danger_search_ws
    catkin_make
    catkin_make run_tests_danger_search_navigation
    catkin_test_results build

构建前也可直接运行：

    python3 src/danger_search_navigation/test/test_navigation_core.py

## 真实仿真验证

仅在已有唯一 localization/TF 所有者、ROS master、传感器和控制层的仿真环境中运行。先确认接口与数值，再发正式目标：

    rosnode info /nav_controller
    rostopic echo -n 1 /navigation/health
    rostopic echo -n 1 /localization/pose
    rostopic echo -n 1 /map
    rosrun tf tf_echo map base
    rostopic info /cmd_vel

之后使用 /move_base/make_plan 验证路径，用 /move_base 验证可通行目标的成功、障碍目标的绕行或 UNREACHABLE；再验证取消、超时和 /danger_search/safety_stop=true 都使 /danger_search/nav_cmd_vel 立即为零。静态测试通过不等于真实仿真已验证，运行前仍须确认 pose、TF、地图数值有界且合理。
