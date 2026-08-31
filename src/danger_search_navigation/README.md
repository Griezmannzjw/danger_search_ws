# danger_search_navigation

除标准 `move_base` 外，本包包含 `navigation_command_mux.py`：普通导航速度先进入 `/danger_search/move_base_cmd_vel`，再由该节点统一输出 `/danger_search/nav_cmd_vel`。电梯门槛处二维 GICP 会受动态门和轿厢影响，因此 `/navigation/traverse_portal` 可临时覆盖 move_base，执行速度和时长均受配置硬上限约束的平移；`/navigation/cancel_portal` 会立即恢复零速度。最终 `/cmd_vel` 仍只由 `danger_search_control` 发布。

本包在完整系统中启动标准 ROS `move_base`，全局规划器为
`navfn/NavfnROS`，局部规划器为 Unitree 示例已经采用的
`base_local_planner/TrajectoryPlannerROS`。旧的 `nav_controller.py` 与
`navigation_core.py` 暂时保留用于历史对照和原有测试，但不再由 launch 启动。

## 数据链路

- global costmap：`map` 坐标系，StaticLayer 读取 `/map`。
- local costmap：`odom` 坐标系、4 m rolling window。
- 两套 costmap 的 ObstacleLayer 均读取 `/localization/scan`。
- `TrajectoryPlannerROS` 从 `/localization/odom` 读取 body-frame twist。
- move_base 的 `cmd_vel` 被重映射到 `/danger_search/nav_cmd_vel`。
- `danger_search_control/cmd_mux` 仍是最终 `/cmd_vel` 的唯一发布者。

## 对外接口

- `/move_base`：标准 `move_base_msgs/MoveBaseAction`。
- `/move_base/make_plan`：标准 `nav_msgs/GetPlan`。
- `/move_base/clear_costmaps`：标准清图服务。
- `/move_base/recovery_status`：标准恢复状态。
- `/navigation/health`：兼容 mission/exploration 的 `NavigationHealth`。
- `/navigation/recovery_event`：由标准 recovery 状态转换的兼容事件。

兼容监控节点只解释状态，不参与规划、恢复或速度输出。标准 recovery
按 `conservative_reset -> rotate_recovery -> aggressive_reset` 有限执行；清图
仅作用于名为 `obstacles` 的动态层，不清除静态地图。

## 启动

```bash
source /opt/ros/noetic/setup.bash
source /home/ruilinli/danger_search_ws/devel/setup.bash
roslaunch danger_search_navigation navigation.launch
```

单独启动导航时仍需提供 `map -> odom -> base` TF、`/map`、
`/localization/scan` 和 `/localization/odom`。完整系统使用：

```bash
roslaunch danger_search_bringup competition.launch
```

不要并行启动旧 `nav_controller.py`、Unitree 自带的另一套 move_base，或让
其他节点直接发布 `/cmd_vel`。

## 验证

```bash
catkin_make
catkin_make run_tests_danger_search_navigation run_tests_danger_search_localization
catkin_test_results build/test_results
```

RViz 可显示标准 global/local costmap、`NavfnROS/plan`、
`TrajectoryPlannerROS/global_plan` 与 `TrajectoryPlannerROS/local_plan`。
