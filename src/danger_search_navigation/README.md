# danger_search_navigation

`nav_controller.py` is the only navigation entry point and the only provider
of `/move_base`, `/move_base/make_plan`, and `/danger_search/nav_cmd_vel`.
It never publishes `/cmd_vel`; `danger_search_control` remains the only final
velocity publisher.

## Inputs

| Interface | Type | Purpose |
|---|---|---|
| `/localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Fresh robot pose in `map` |
| `/map` | `nav_msgs/OccupancyGrid` | Global planning map |
| `/mapping/status` | `danger_search_common/MappingStatus` | Localization readiness and loss gate |
| `/scan` | `sensor_msgs/PointCloud` | Raw local dynamic-obstacle check only |
| `/danger_search/safety_stop` | `std_msgs/Bool` | External safety stop gate |

`/scan` is consumed directly. This package does not subscribe to
`/Odometry_gazebo`, `/ground_truth/*`, `/livox/lidar2`, `/livox/Pointcloud2`,
or any truth/layout file.

## Outputs

| Interface | Type | Meaning |
|---|---|---|
| `/move_base` | `move_base_msgs/MoveBaseAction` | Navigation Action server |
| `/move_base/make_plan` | `nav_msgs/GetPlan` | Plan query using the same planner as Action execution |
| `/danger_search/nav_cmd_vel` | `geometry_msgs/Twist` | Navigation-only velocity request for control |
| `/navigation/health` | `danger_search_common/NavigationHealth` | Actual action and controller health |

`/move_base/clear_costmaps` is retained as an existing compatibility service.
It requests a route refresh from the newest map; it does not create a second
costmap or Action server.

## Planning And Control

The controller builds one conservative planning grid from `/map`:

1. Unknown cells, map exterior, all non-free cells, occupied cells, and
   occupied-cell inflation for the configured robot radius are non-traversable.
2. A* is used by both `make_plan` and Action execution. The Action follows
   the exact route computed by that shared planner instead of driving directly
   at its final goal.
3. The current raw `/scan` is transformed with YAML LiDAR-to-base extrinsics,
   checked in front of the robot, and overlaid as temporary inflated obstacles
   for replanning. A stale or invalid required cloud stops an active goal.
4. A non-holonomic pure-pursuit controller tracks a path lookahead point,
   rotates in place for large heading error, and separately satisfies final
   position and yaw tolerances.

Action requests must contain a fresh `map`-frame pose, a valid quaternion, a
free inflated goal cell, and an A* route. Cancellation, timeout, no route,
stuck progress, localization loss, and safety stop publish zero navigation
velocity and report only the frozen `NavigationHealth.failure_code` values.

## Configuration

All names, frames, thresholds, timeouts, and LiDAR extrinsics are private ROS
parameters in `config/default.yaml`. `competition.launch` already loads that
file inside the `navigation` node namespace. The package-local
`navigation.launch` now does the same, so standalone launches use exactly the
same private parameters:

```bash
roslaunch danger_search_navigation navigation.launch
```

## Static Tests

`test/test_navigation_core.py` is ROS-independent and covers A* detours,
robot-radius inflation, unreachable goals, dynamic obstacle inflation, Action
cancellation state, and the zero-velocity result. It can run with:

```bash
python3 test/test_navigation_core.py
```
