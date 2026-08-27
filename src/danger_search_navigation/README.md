# danger_search_navigation

本包在完整系统中启动标准 ROS `move_base`，全局规划器为
`navfn/NavfnROS`，局部规划器为标准
`dwa_local_planner/DWAPlannerROS`。其速度域排除 Unitree RL 无法执行的
`0.10 m/s` 对角步态，并保留标准 DWA 的动态窗口与 critics。旧的 `nav_controller.py` 与
`navigation_core.py` 暂时保留用于历史对照和原有测试，但不再由 launch 启动。

## 数据链路

- global costmap：`map` 坐标系，StaticLayer 读取 `/map`。
- local costmap：`odom` 坐标系、4 m rolling window。
- 两套 costmap 的 ObstacleLayer 均读取 `/localization/scan`。
- `DWAPlannerROS` 从 `/localization/odom` 读取 body-frame twist。
- move_base 的 `cmd_vel` 被重映射到 `/danger_search/nav_cmd_vel`。
- `danger_search_control/cmd_mux` 仍是最终 `/cmd_vel` 的唯一发布者。

## 对外接口

- `/move_base`：标准 `move_base_msgs/MoveBaseAction`。
- `/move_base/make_plan`：标准 `nav_msgs/GetPlan`。
- `/move_base/clear_costmaps`：标准清图服务。
- `/move_base/recovery_status`：标准恢复状态。
- `/navigation/health`：兼容 mission/exploration 的 `NavigationHealth`。
- `/navigation/recovery_event`：由标准 recovery 状态转换的兼容事件。

兼容监控节点只解释状态，不参与规划、恢复或速度输出。recovery 按
`conservative_reset -> escape_recovery_1 -> rotate_recovery -> aggressive_reset
-> escape_recovery_2` 有限执行。`UnitreeEscapeRecovery` 是标准
`nav_core::RecoveryBehavior` 插件，使用 local costmap 的固定 padded footprint
检查完整后退/横移扫掠，并按定位闭环停止。清图仅作用于名为 `obstacles`
的动态层，不清除静态地图。

恢复执行、Action 终态、recovery event 和 plan generation 均绑定 goal ID/epoch。新目标、
取消或 safety stop 会让旧 epoch 立即失效，迟到回调不得更新新目标的尝试次数或失败状态。

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
`DWAPlannerROS/global_plan` 与 `DWAPlannerROS/local_plan`。
