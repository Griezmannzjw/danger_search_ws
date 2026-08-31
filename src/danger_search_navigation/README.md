# danger_search_navigation

除标准 `move_base` 外，本包包含 `navigation_command_mux.py`：普通导航速度先进入 `/danger_search/move_base_cmd_vel`，再由该节点统一输出 `/danger_search/nav_cmd_vel`。电梯门槛处二维 GICP 会受动态门和轿厢影响，因此 `/navigation/traverse_portal` 可临时覆盖 move_base，执行速度和时长均受配置硬上限约束的平移；`/navigation/cancel_portal` 会立即恢复零速度。最终 `/cmd_vel` 仍只由 `danger_search_control` 发布。

本包在完整系统中启动标准 ROS `move_base`，全局规划器为
`navfn/NavfnROS`，局部规划器为标准
`dwa_local_planner/DWAPlannerROS`。其速度域排除 Unitree RL 无法执行的
`0.10 m/s` 对角步态，并保留标准 DWA 的动态窗口与 critics。旧的 `nav_controller.py` 与
`navigation_core.py` 暂时保留用于历史对照和原有测试，但不再由 launch 启动。

普通导航保持非完整约束（`vy=0`）。真机策略对应的 Gazebo 响应标定显示：
纯转向 `|wz|<=0.30 rad/s` 基本停留在站立死区，`|wz|=0.40 rad/s`、
普通导航的 `vx=0.30 m/s` 步态以及恢复使用的
`vx=0.40 m/s, |wz|=0.40 rad/s` 组合弧线均能稳定执行。因此 DWA 的 yaw
域固定为 `[-0.40, 0.40] rad/s`，9 个对称样本仍覆盖零、`±0.10`、
`±0.20`、`±0.30` 与 `±0.40`。上游轨迹生成器只在平移速度低于
`min_vel_trans` 时应用 `min_vel_theta`，所以 `min_vel_theta=0.40` 会过滤
无物理响应的纯转向，却不会删除移动弧线的较小角速度样本。
平移域固定为两个离散模式：`min_vel_x=0`、`max_vel_x=0.30 m/s`、
`vx_samples=2`，且 `min_vel_trans=max_vel_trans=0.30 m/s`。ROS Noetic
原生轨迹生成器因此只保留 `vx=0, |wz|=0.40` 的有效原地转向，以及
`vx=0.30 m/s` 的前进/弧线轨迹，不生成 `0.01–0.29 m/s` 的站立或碎步候选。
这也允许普通路径在目标位于机器人后方时先进行 footprint-checked 原地对准。
`cmd_mux` 的 `0.80 rad/s` 仍只是所有生产者的最终硬上限。

不再装载标准 `rotate_recovery/RotateRecovery`：其上游实现会尝试完整一圈，
且运行循环没有目标抢占、安全停车或执行超时合同。目标朝向由 DWA 自带的
footprint-checked stop/rotate 完成；阻塞恢复只保留有 goal epoch、取消、
安全停车、无进展超时和完整 footprint 扫掠的 `UnitreeEscapeRecovery`。
其中优先尝试已在 Unitree Gazebo 标定通过的 `0.40 m/s + 0.40 rad/s`
左右短弧；短弧会改变朝向而不产生原地旋转的四角扫掠，若两侧均受阻才退回
直线后退。两个尝试仍共享 goal epoch 和方向排除预算。

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
`conservative_reset -> escape_recovery_1 -> aggressive_reset ->
escape_recovery_2` 有限执行。`UnitreeEscapeRecovery` 是标准
`nav_core::RecoveryBehavior` 插件，使用 local costmap 的固定 padded footprint
检查完整短弧/后退/横移扫掠，并按定位闭环停止。清图仅作用于名为 `obstacles`
的动态层，不清除静态地图。

恢复执行、Action 终态、recovery event 和 plan generation 均绑定 goal ID/epoch。新目标、
取消或 safety stop 会让旧 epoch 立即失效，迟到回调不得更新新目标的尝试次数或失败状态。

monitor 同时核对 `/map`、锁存的 `/mapping/active_map` 与 `MappingStatus`。换层、
floor/epoch 变化会立即取消旧 goal；新地图签名匹配后由 monitor 唯一调用
`/move_base/clear_costmaps`，仅成功回调能使当前 epoch 恢复 `ready=true`。
`NavigationHealth.map_version` 报告门禁已提交的快照；持续建图时它可以短暂落后于
MappingStatus，但 floor/epoch 永远不能放宽。

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
