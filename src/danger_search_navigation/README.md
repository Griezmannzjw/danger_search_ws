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

## 规划、跟踪与安全行为

- navigation_core.py 不依赖 ROS。make_plan、Action 启动、地图更新和动态障碍阻断后的重规划都调用同一个带膨胀的 A* 入口。
- /map 中 -1、100、任何非 0 栅格和地图外部均不可通行；所有非自由栅格按机器人半径加安全余量膨胀。地图 origin.position 和二维 yaw 都参与 world/cell 转换，负坐标使用 floor 语义。
- Action 跟踪 A* 路径的前视点，不会直接追最终目标。大偏航先原地旋转；进入 XY 容差后继续独立调整最终 yaw，只有 XY 和 yaw 都达标才 SUCCEEDED。
- /scan 依据 YAML 内的 LiDAR-to-base 外参筛选和投影成临时障碍，并以和静态障碍相同的策略膨胀。require_obstacle_cloud 为 true 时，缺失、过期或帧错误的点云会停止并以 CONTROL_FAILED 结束活动目标。
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
