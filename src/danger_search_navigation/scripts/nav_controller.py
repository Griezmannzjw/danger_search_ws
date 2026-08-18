#!/usr/bin/env python3
"""导航控制节点。

提供 MoveBaseAction 与 make_plan 服务。两者通过同一个 occupancy-aware A*
规划入口使用同一份膨胀后占据栅格；Action 只跟踪该入口返回的路径。
"""

import math
import os
import sys
import time
import threading

import actionlib
import rospy
import tf.transformations
from danger_search_common.msg import MappingStatus, NavigationHealth
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback, MoveBaseResult
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan, GetPlanResponse
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyResponse

# catkin 的 devel-space 会为可执行 Python 脚本生成 relay。该 relay 不适合作为
# 被 import 的模块，因此先把本源码目录置于搜索路径最前，保证导入真实核心。
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
try:
    sys.path.remove(_SCRIPT_DIR)
except ValueError:
    pass
sys.path.insert(0, _SCRIPT_DIR)

from navigation_core import (
    GoalState,
    InflatedOccupancyGrid,
    goal_reached,
    normalize_angle,
    path_lengths,
    path_progress,
)


class NavController:
    """共享全局规划器和固定路径跟踪器的 ROS 适配层。"""

    def __init__(self):
        rospy.init_node("nav_controller", anonymous=False)

        # 所有名称均从节点私有参数读取。
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.nav_cmd_topic = rospy.get_param("~nav_cmd_topic", "/danger_search/nav_cmd_vel")
        self.pose_topic = rospy.get_param("~pose_topic", "/localization/pose")
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.mapping_status_topic = rospy.get_param("~mapping_status_topic", "/mapping/status")
        self.health_topic = rospy.get_param("~health_topic", "/navigation/health")
        self.obstacle_cloud_topic = rospy.get_param("~obstacle_cloud_topic", "/scan")
        self.safety_stop_topic = rospy.get_param("~safety_stop_topic", "/danger_search/safety_stop")
        self.move_base_action_name = rospy.get_param("~move_base_action_name", "/move_base")
        self.make_plan_service = rospy.get_param("~make_plan_service", "/move_base/make_plan")
        self.clear_costmaps_service = rospy.get_param(
            "~clear_costmaps_service", "/move_base/clear_costmaps"
        )

        # 所有规划、安全和控制阈值均从节点私有参数读取。
        self.occupied_threshold = int(rospy.get_param("~occupied_threshold", 65))
        self.robot_radius = float(rospy.get_param("~robot_radius", 0.30))
        self.inflation_padding = float(rospy.get_param("~inflation_padding", 0.10))
        self.allow_diagonal = bool(rospy.get_param("~allow_diagonal", True))
        self.max_expansions = int(rospy.get_param("~max_planner_expansions", 75000))
        self.pose_timeout = float(rospy.get_param("~pose_timeout", 0.50))
        self.map_timeout = float(rospy.get_param("~map_timeout", 2.00))
        self.mapping_status_timeout = float(rospy.get_param("~mapping_status_timeout", 1.50))
        self.obstacle_cloud_timeout = float(rospy.get_param("~obstacle_cloud_timeout", 1.00))
        self.goal_stamp_timeout = float(rospy.get_param("~goal_stamp_timeout", 2.00))
        self.max_future_stamp_skew = float(rospy.get_param("~max_future_stamp_skew", 0.05))
        self.quaternion_norm_tolerance = float(rospy.get_param("~quaternion_norm_tolerance", 0.05))
        self.require_obstacle_cloud = bool(rospy.get_param("~require_obstacle_cloud", True))
        self.lidar_frame = rospy.get_param("~lidar_frame", "laser_livox")
        self.obstacle_min_z = float(rospy.get_param("~obstacle_min_z", -0.30))
        self.obstacle_max_z = float(rospy.get_param("~obstacle_max_z", 0.80))
        self.obstacle_range_min = float(rospy.get_param("~obstacle_range_min", 0.15))
        self.obstacle_range_max = float(rospy.get_param("~obstacle_range_max", 8.00))
        self.zero_point_radius = float(rospy.get_param("~zero_point_radius", 0.05))
        self.obstacle_max_points = int(rospy.get_param("~obstacle_max_points", 2000))
        self.dynamic_stop_distance = float(rospy.get_param("~dynamic_stop_distance", 0.60))
        self.dynamic_front_half_angle = float(rospy.get_param("~dynamic_front_half_angle", 0.52))
        self.lookahead_distance = float(rospy.get_param("~lookahead_distance", 0.60))
        self.cruise_speed = float(rospy.get_param("~cruise_speed", 0.40))
        self.max_linear_speed = float(rospy.get_param("~max_linear_speed", 0.40))
        self.max_angular_speed = float(rospy.get_param("~max_angular_speed", 0.80))
        self.rotate_in_place_angle = float(rospy.get_param("~rotate_in_place_angle", 0.45))
        self.rotate_in_place_gain = float(rospy.get_param("~rotate_in_place_gain", 1.50))
        self.final_yaw_gain = float(rospy.get_param("~final_yaw_gain", 1.20))
        self.replan_period = float(rospy.get_param("~replan_period", 1.00))
        self.replan_min_interval = float(rospy.get_param("~replan_min_interval", 0.25))
        self.replan_deviation_distance = float(rospy.get_param("~replan_deviation_distance", 0.50))
        self.goal_tolerance_xy = float(rospy.get_param("~goal_tolerance_xy", 0.30))
        self.goal_tolerance_yaw = float(rospy.get_param("~goal_tolerance_yaw", 0.20))
        self.goal_timeout = float(rospy.get_param("~goal_timeout", 60.0))
        self.stuck_timeout = float(rospy.get_param("~stuck_timeout", 5.0))
        self.progress_distance = float(rospy.get_param("~progress_distance", 0.10))
        self.stuck_command_speed = float(rospy.get_param("~stuck_command_speed", 0.05))
        self.goal_projection_max_radius = float(
            rospy.get_param("~goal_projection_max_radius", 0.28)
        )
        self.goal_projection_step = float(
            rospy.get_param("~goal_projection_step", 0.05)
        )
        self.projection_tracking_tolerance = float(
            rospy.get_param("~projection_tracking_tolerance", 0.05)
        )
        self.control_rate = float(rospy.get_param("~control_rate", 20.0))
        self.health_rate = float(rospy.get_param("~health_rate", 5.0))
        self._validate_configuration()

        self.lock = threading.RLock()
        self.current_pose = None  # (x, y, yaw)，全部处于 map 坐标系。
        self.pose_stamp = rospy.Time(0)
        self.pose_valid = False
        self.map_data = None
        self.planner = None
        self.map_stamp = rospy.Time(0)
        self.map_valid = False
        self.map_generation = 0
        self.map_geometry = None
        self.map_unchanged_count = 0
        self.map_build_count = 0
        self.obstacle_tf_failure_count = 0
        self.obstacle_invalid_count = 0
        self.mapping_status = None
        self.mapping_status_stamp = rospy.Time(0)
        self.latest_obstacles_base = []
        self.obstacle_stamp = rospy.Time(0)
        self.obstacle_frame_valid = False
        self.safety_stop = False
        self.clear_costmaps_requested = False
        self.tf_listener = tf.TransformListener(cache_time=rospy.Duration(10.0))

        self.goal_state = GoalState()
        self.has_active_goal = False
        self.active_goal_id = ""
        self.active_path = []
        self.active_path_lengths = []
        self.waypoint_index = 0
        self.goal_start_time = None
        self.last_progress_time = rospy.Time.now()
        self.last_position = None
        self.is_stuck = False
        self.failure_code = "NONE"
        self.failure_detail = ""
        self.controller_active = False
        self.last_cmd_time = rospy.Time(0)
        self.active_tracking_target = None
        self.goal_projected = False

        self.cmd_pub = rospy.Publisher(self.nav_cmd_topic, Twist, queue_size=10)
        self.health_pub = rospy.Publisher(self.health_topic, NavigationHealth, queue_size=10)

        self.pose_sub = rospy.Subscriber(
            self.pose_topic, PoseWithCovarianceStamped, self.pose_callback
        )
        self.map_sub = rospy.Subscriber(
            self.map_topic, OccupancyGrid, self.map_callback, queue_size=1
        )
        self.mapping_status_sub = rospy.Subscriber(
            self.mapping_status_topic, MappingStatus, self.mapping_status_callback
        )
        self.obstacle_sub = rospy.Subscriber(
            self.obstacle_cloud_topic,
            PointCloud,
            self.obstacle_callback,
            queue_size=1,
        )
        self.safety_stop_sub = rospy.Subscriber(
            self.safety_stop_topic, Bool, self.safety_stop_callback
        )

        self.action_server = actionlib.SimpleActionServer(
            self.move_base_action_name,
            MoveBaseAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.action_server.register_preempt_callback(self.preempt_cb)
        self.action_server.start()

        self.make_plan_srv = rospy.Service(self.make_plan_service, GetPlan, self.make_plan_cb)
        self.clear_costmaps_srv = rospy.Service(
            self.clear_costmaps_service, Empty, self.clear_costmaps_cb
        )
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate), self.control_loop
        )
        self.health_timer = rospy.Timer(
            rospy.Duration(1.0 / self.health_rate), self.publish_health
        )
        rospy.on_shutdown(self._stop_robot)

        rospy.loginfo(
            "[navigation] 已启动共享 A* 导航节点，action: %s", self.move_base_action_name
        )

    def _validate_configuration(self):
        """在启动前拒绝会破坏安全语义的配置。"""
        positive_values = (
            self.max_expansions, self.pose_timeout, self.map_timeout,
            self.mapping_status_timeout, self.obstacle_cloud_timeout,
            self.goal_stamp_timeout, self.quaternion_norm_tolerance,
            self.obstacle_range_max, self.obstacle_max_points,
            self.dynamic_stop_distance, self.control_rate, self.health_rate,
            self.goal_timeout, self.goal_tolerance_xy, self.goal_tolerance_yaw,
            self.lookahead_distance, self.cruise_speed, self.max_linear_speed,
            self.max_angular_speed, self.rotate_in_place_angle,
            self.rotate_in_place_gain, self.final_yaw_gain, self.replan_period,
            self.replan_min_interval, self.replan_deviation_distance,
            self.stuck_timeout, self.progress_distance,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive_values):
            raise rospy.ROSInitException("导航正数参数无效")
        if (not math.isfinite(self.robot_radius) or not math.isfinite(self.inflation_padding)
                or self.robot_radius < 0.0 or self.inflation_padding < 0.0):
            raise rospy.ROSInitException("导航膨胀参数不能为负")
        if not math.isfinite(self.max_future_stamp_skew) or self.max_future_stamp_skew < 0.0:
            raise rospy.ROSInitException("消息未来时间容差无效")
        if self.obstacle_min_z > self.obstacle_max_z:
            raise rospy.ROSInitException("点云高度范围无效")
        if self.obstacle_range_min < 0.0 or self.obstacle_range_min >= self.obstacle_range_max:
            raise rospy.ROSInitException("点云距离范围无效")
        if (not math.isfinite(self.goal_projection_max_radius)
                or self.goal_projection_max_radius <= 0.0
                or not math.isfinite(self.goal_projection_step)
                or self.goal_projection_step <= 0.0
                or self.goal_projection_step > self.goal_projection_max_radius):
            raise rospy.ROSInitException("目标投影参数无效")
        if (not math.isfinite(self.projection_tracking_tolerance)
                or self.projection_tracking_tolerance <= 0.0
                or self.projection_tracking_tolerance >= self.goal_tolerance_xy):
            raise rospy.ROSInitException("投影跟踪容差必须位于 0 和目标容差之间")
        if not 1 <= self.occupied_threshold <= 100:
            raise rospy.ROSInitException("占据阈值必须位于 1..100")

    def pose_callback(self, msg):
        """仅保存时间、帧、数值和四元数均合法的 map 位姿。"""
        pose, valid = self._pose_from_message(msg)
        with self.lock:
            self.current_pose = pose
            self.pose_stamp = msg.header.stamp
            self.pose_valid = valid
            active = self.goal_state.active
        if not valid and active:
            self._stop_robot()

    def map_callback(self, msg):
        """校验地图并原子替换 planner，重复内容只更新时间戳。"""
        planner = None
        valid = False
        data = None
        geometry = None
        if msg.header.frame_id == self.map_frame and not msg.header.stamp.is_zero():
            origin = msg.info.origin
            yaw = self._quaternion_yaw(origin.orientation)
            if yaw is not None and self._finite(origin.position.x, origin.position.y):
                try:
                    # Keep an immutable snapshot so equality compares actual
                    # map contents without a cryptographic hash.
                    data = tuple(msg.data)
                    geometry = (
                        int(msg.info.width),
                        int(msg.info.height),
                        float(msg.info.resolution),
                        float(origin.position.x),
                        float(origin.position.y),
                        float(yaw),
                    )
                    with self.lock:
                        unchanged = (
                            self.map_valid
                            and self.planner is not None
                            and self.map_geometry == geometry
                            and self.map_data == data
                        )
                        if unchanged:
                            self.map_stamp = msg.header.stamp
                            self.map_unchanged_count += 1
                    if unchanged:
                        rospy.logdebug_throttle(
                            10.0,
                            "[navigation] 跳过未变化地图，累计 %d 帧",
                            self.map_unchanged_count,
                        )
                        return
                    build_started = time.monotonic()
                    planner = InflatedOccupancyGrid(
                        geometry[0], geometry[1], geometry[2],
                        geometry[3], geometry[4], geometry[5], data,
                        occupied_threshold=self.occupied_threshold,
                        robot_radius=self.robot_radius,
                        inflation_padding=self.inflation_padding,
                        allow_diagonal=self.allow_diagonal,
                        max_expansions=self.max_expansions,
                    )
                    valid = True
                    elapsed_ms = (time.monotonic() - build_started) * 1000.0
                    with self.lock:
                        self.map_build_count += 1
                        build_count = self.map_build_count
                    rospy.loginfo_throttle(
                        10.0,
                        "[navigation] planner 地图构建 %.1f ms，累计 %d 次",
                        elapsed_ms,
                        build_count,
                    )
                except (TypeError, ValueError) as exc:
                    rospy.logwarn_throttle(5.0, "[navigation] 地图无效，拒绝规划: %s", exc)
        if not valid:
            rospy.logwarn_throttle(5.0, "[navigation] 地图帧、时间戳、原点或几何无效")
        with self.lock:
            if valid:
                self.map_data = data
                self.map_geometry = geometry
                self.planner = planner
            else:
                self.map_data = None
                self.map_geometry = None
                self.planner = None
            self.map_stamp = msg.header.stamp
            self.map_valid = valid
            if valid:
                self.map_generation += 1
            active = self.goal_state.active
        if not valid and active:
            self._stop_robot()

    def mapping_status_callback(self, msg):
        """保存定位/建图模块明确声明的 ready、stable、lost 状态。"""
        with self.lock:
            self.mapping_status = msg
            self.mapping_status_stamp = msg.header.stamp
            active = self.goal_state.active
        if active and (msg.lost or not msg.ready or not msg.stable):
            self._stop_robot()

    def obstacle_callback(self, cloud):
        """把有效原始 /scan 转为 base 坐标下的临时障碍，不用作定位。"""
        points = []
        frame_valid = bool(cloud.header.frame_id) and not cloud.header.stamp.is_zero()
        if frame_valid:
            try:
                transform = self.tf_listener.lookupTransform(
                    self.base_frame, cloud.header.frame_id, cloud.header.stamp
                )
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
                with self.lock:
                    self.obstacle_tf_failure_count += 1
                    tf_failures = self.obstacle_tf_failure_count
                rospy.logwarn_throttle(
                    5.0, "[navigation] 点云 TF 不可用 %s <- %s: %s",
                    self.base_frame, cloud.header.frame_id, str(exc)
                )
                try:
                    rospy.logdebug_throttle(
                        10.0, "[navigation] 点云 TF 查询失败累计 %d 次", tf_failures
                    )
                except Exception:
                    pass
                frame_valid = False
            if frame_valid:
                count = len(cloud.points)
                step = max(1, int(math.ceil(float(count) / self.obstacle_max_points)))
                source_points = []
                for point in cloud.points[::step]:
                    if not self._finite(point.x, point.y, point.z):
                        continue
                    if point.x * point.x + point.y * point.y + point.z * point.z < self.zero_point_radius ** 2:
                        continue
                    source_points.append((point.x, point.y, point.z))
                for base_x, base_y, base_z in self._transform_points(
                    transform, source_points
                ):
                    distance = math.hypot(base_x, base_y)
                    if not self.obstacle_min_z <= base_z <= self.obstacle_max_z:
                        continue
                    if not self.obstacle_range_min <= distance <= self.obstacle_range_max:
                        continue
                    points.append((base_x, base_y, base_z))
        else:
            with self.lock:
                self.obstacle_invalid_count += 1
            rospy.logwarn_throttle(
                5.0, "[navigation] 点云帧或时间戳无效"
            )
        with self.lock:
            self.latest_obstacles_base = points
            self.obstacle_stamp = cloud.header.stamp
            self.obstacle_frame_valid = frame_valid
            active = self.goal_state.active
        if self.require_obstacle_cloud and not frame_valid and active:
            self._stop_robot()

    def safety_stop_callback(self, msg):
        """安全停车信号一到即停车；执行线程负责唯一的 Action 终态。"""
        with self.lock:
            self.safety_stop = bool(msg.data)
            active = self.goal_state.active
        if bool(msg.data) and active:
            self._stop_robot()

    @staticmethod
    def _transform_point(transform, x, y, z):
        translation, rotation = transform
        matrix = tf.transformations.quaternion_matrix(rotation)
        rotated = matrix.dot((x, y, z, 1.0))
        return (
            rotated[0] + translation[0],
            rotated[1] + translation[1],
            rotated[2] + translation[2],
        )

    @staticmethod
    def _transform_points(transform, points):
        """Transform a sampled cloud with one matrix construction per frame."""
        if not points:
            return ()
        translation, rotation = transform
        matrix = tf.transformations.quaternion_matrix(rotation)
        r00, r01, r02 = matrix[0, 0], matrix[0, 1], matrix[0, 2]
        r10, r11, r12 = matrix[1, 0], matrix[1, 1], matrix[1, 2]
        r20, r21, r22 = matrix[2, 0], matrix[2, 1], matrix[2, 2]
        tx, ty, tz = translation
        return tuple(
            (
                r00 * x + r01 * y + r02 * z + tx,
                r10 * x + r11 * y + r12 * z + ty,
                r20 * x + r21 * y + r22 * z + tz,
            )
            for x, y, z in points
        )

    def _tracking_tolerance(self, projected):
        return (
            self.projection_tracking_tolerance
            if projected else self.goal_tolerance_xy
        )

    def _requested_goal_reached(self, current, target):
        return goal_reached(
            current, target, self.goal_tolerance_xy, self.goal_tolerance_yaw
        )

    def _goal_candidates(self, goal_xy, goal_yaw, start_xy=None):
        """Generate nearby endpoints, preferring lateral doorway corrections."""
        radius = min(
            self.goal_projection_max_radius,
            self.goal_tolerance_xy - self.projection_tracking_tolerance,
        )
        if radius <= 0.0:
            return []
        normal = (-math.sin(goal_yaw), math.cos(goal_yaw))
        tangent = (math.cos(goal_yaw), math.sin(goal_yaw))
        candidates = []
        steps = int(math.floor(radius / self.goal_projection_step + 1e-9))
        for ring in range(1, steps + 1):
            distance = ring * self.goal_projection_step
            offsets = (
                (normal[0] * distance, normal[1] * distance),
                (-normal[0] * distance, -normal[1] * distance),
            )
            for lateral_x, lateral_y in offsets:
                candidates.append((distance, lateral_x, lateral_y))
            for tangent_sign in (-1.0, 1.0):
                for lateral_sign in (-1.0, 1.0):
                    offset_x = normal[0] * distance * lateral_sign + tangent[0] * distance * tangent_sign
                    offset_y = normal[1] * distance * lateral_sign + tangent[1] * distance * tangent_sign
                    offset_distance = math.hypot(offset_x, offset_y)
                    if offset_distance <= radius + 1e-9:
                        candidates.append((offset_distance, offset_x, offset_y))
        candidates.sort(key=lambda item: (item[0], abs(item[1] * tangent[0] + item[2] * tangent[1])))
        points = [(goal_xy[0] + dx, goal_xy[1] + dy) for _, dx, dy in candidates]
        if start_xy is None:
            return points
        # Do not select a projected endpoint materially behind the robot.
        tangent = (math.cos(goal_yaw), math.sin(goal_yaw))
        return [
            point for point in points
            if (point[0] - start_xy[0]) * tangent[0]
            + (point[1] - start_xy[1]) * tangent[1] >= -0.05
        ]

    def _plan_goal_path(self, start_xy, goal, dynamic_cells=()):
        """Plan to the requested goal or a safe endpoint within goal tolerance."""
        with self.lock:
            planner = self.planner
        if planner is None:
            return None, None, False
        goal_xy = goal[:2]
        route = planner.plan(start_xy, goal_xy, dynamic_cells)
        if route is not None:
            return route, goal, False
        projected_routes = []
        for candidate in self._goal_candidates(goal_xy, goal[2], start_xy):
            route = planner.plan(start_xy, candidate, dynamic_cells)
            if route is not None:
                route_length = path_lengths(route)[-1] if len(route) > 1 else 0.0
                goal_offset = math.hypot(
                    candidate[0] - goal_xy[0], candidate[1] - goal_xy[1]
                )
                projected_routes.append((goal_offset, route_length, candidate, route))
        if projected_routes:
            _, _, candidate, route = min(
                projected_routes, key=lambda item: (item[0], item[1])
            )
            return route, (candidate[0], candidate[1], goal[2]), True
        return None, None, False

    def _plan_path(self, start_xy, goal, dynamic_cells=()):
        """make_plan and Action share goal projection and A* planning."""
        route, _, _ = self._plan_goal_path(start_xy, goal, dynamic_cells)
        return route

    def _plan_action_path(self, start_xy, goal_xy, dynamic_cells=()):
        """Keep an Action on known free cells while approaching an unknown goal."""
        with self.lock:
            planner = self.planner
        if planner is None:
            return None
        route, _, _ = self._plan_goal_path(start_xy, goal_xy, dynamic_cells)
        if route is not None:
            return route
        return self._plan_unknown_fallback(start_xy, goal_xy, dynamic_cells)

    def _plan_unknown_fallback(self, start_xy, goal, dynamic_cells=()):
        with self.lock:
            planner = self.planner
        if planner is None:
            return None
        return planner.plan_toward_unknown(start_xy, goal[:2], dynamic_cells)

    def _pose_from_message(self, msg):
        if msg.header.frame_id != self.map_frame or msg.header.stamp.is_zero():
            return None, False
        position = msg.pose.pose.position
        yaw = self._quaternion_yaw(msg.pose.pose.orientation)
        if yaw is None or not self._finite(position.x, position.y, position.z):
            return None, False
        return (position.x, position.y, yaw), True

    def _pose_stamped_to_xy_yaw(self, pose, allow_zero_stamp):
        """验证 GetPlan 或 Action 的 map 位姿和时间要求。"""
        if pose.header.frame_id != self.map_frame:
            return None, False
        if pose.header.stamp.is_zero():
            if not allow_zero_stamp:
                return None, False
        elif not self._is_stamp_fresh(pose.header.stamp, self.goal_stamp_timeout, rospy.Time.now()):
            return None, False
        position = pose.pose.position
        yaw = self._quaternion_yaw(pose.pose.orientation)
        if yaw is None or not self._finite(position.x, position.y, position.z):
            return None, False
        return (position.x, position.y, yaw), True

    def _quaternion_yaw(self, quaternion):
        if not self._finite(quaternion.x, quaternion.y, quaternion.z, quaternion.w):
            return None
        norm = math.sqrt(
            quaternion.x * quaternion.x + quaternion.y * quaternion.y
            + quaternion.z * quaternion.z + quaternion.w * quaternion.w
        )
        if abs(norm - 1.0) > self.quaternion_norm_tolerance:
            return None
        try:
            return tf.transformations.euler_from_quaternion(
                [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
            )[2]
        except (TypeError, ValueError):
            return None

    def _navigation_readiness(self, now, require_obstacles):
        """统一给 Action、make_plan 和 health 使用的数据安全门。"""
        with self.lock:
            # Pose/map callbacks already reject zero stamps, wrong frames and
            # invalid geometry. Their source stamps describe the underlying
            # estimate/map content and need not advance on every adapter
            # publication. Localization owns source freshness and exposes it
            # through MappingStatus, whose own publication is checked below.
            pose_ready = self.pose_valid
            map_ready = self.map_valid and self.planner is not None
            mapping = self.mapping_status
            mapping_ready = mapping is not None and self._is_stamp_fresh(
                self.mapping_status_stamp, self.mapping_status_timeout, now
            )
            obstacle_ready = self.obstacle_frame_valid and self._is_stamp_fresh(
                self.obstacle_stamp, self.obstacle_cloud_timeout, now
            )
            safety_stop = self.safety_stop
            obstacle_stamp = self.obstacle_stamp
            obstacle_frame_valid = self.obstacle_frame_valid
            obstacle_tf_failures = getattr(self, "obstacle_tf_failure_count", 0)
            obstacle_invalid = getattr(self, "obstacle_invalid_count", 0)
        obstacle_age = (
            (now - obstacle_stamp).to_sec()
            if not obstacle_stamp.is_zero() else float("inf")
        )
        try:
            rospy.logdebug_throttle(
                10.0,
                "[navigation] safety gate pose=%s map=%s mapping=%s scan_frame=%s "
                "scan_age=%.3fs tf_failures=%d invalid=%d",
                str(pose_ready), str(map_ready), str(mapping_ready),
                str(obstacle_frame_valid), obstacle_age,
                obstacle_tf_failures, obstacle_invalid,
            )
        except Exception:
            # Keep ROS-free unit tests independent of rospy.init_node().
            pass
        if safety_stop:
            return False, "SAFETY_STOP", "外部 safety_stop 信号为真"
        if not pose_ready:
            return False, "LOCALIZATION_LOST", "map 位姿缺失、过期、帧错误或数值非法"
        if not map_ready:
            return False, "LOCALIZATION_LOST", "占据地图缺失、过期、帧错误或几何非法"
        if not mapping_ready:
            return False, "LOCALIZATION_LOST", "mapping/status 缺失或过期"
        if mapping.lost:
            return False, "LOCALIZATION_LOST", mapping.status_reason or "定位/建图报告已丢失"
        if not mapping.ready or not mapping.stable:
            return False, "LOCALIZATION_LOST", mapping.status_reason or "定位/建图尚未 ready 且 stable"
        if require_obstacles and self.require_obstacle_cloud and not obstacle_ready:
            return False, "CONTROL_FAILED", "要求的 /scan 缺失、过期或坐标帧无效"
        return True, "NONE", ""

    def _dynamic_obstacle_snapshot(self, now):
        """将当前点云临时障碍投到 map 栅格，并计算前方净空。"""
        with self.lock:
            planner = self.planner
            pose = self.current_pose
            obstacles = list(self.latest_obstacles_base)
            fresh = self.obstacle_frame_valid and self._is_stamp_fresh(
                self.obstacle_stamp, self.obstacle_cloud_timeout, now
            )
        if planner is None or pose is None or not fresh:
            return set(), float("inf"), fresh
        cos_yaw, sin_yaw = math.cos(pose[2]), math.sin(pose[2])
        dynamic_cells = set()
        front_clearance = float("inf")
        for base_x, base_y, _ in obstacles:
            if base_x > 0.0 and abs(math.atan2(base_y, base_x)) <= self.dynamic_front_half_angle:
                front_clearance = min(front_clearance, math.hypot(base_x, base_y))
            world_x = pose[0] + cos_yaw * base_x - sin_yaw * base_y
            world_y = pose[1] + sin_yaw * base_x + cos_yaw * base_y
            cell = planner.world_to_cell(world_x, world_y)
            if cell is not None:
                dynamic_cells.add(cell)
        return dynamic_cells, front_clearance, True

    def _apply_route_locked(self, route, now):
        self.active_path = list(route)
        self.active_path_lengths = path_lengths(self.active_path)
        self.waypoint_index = 0
        self.active_map_generation = self.map_generation
        self.last_replan_time = now

    def _route_snapshot(self):
        with self.lock:
            return list(self.active_path), self.active_map_generation, self.last_replan_time

    def _map_generation_snapshot(self):
        with self.lock:
            return self.map_generation

    def _current_pose_snapshot(self):
        with self.lock:
            return self.current_pose

    def _goal_start_time(self):
        with self.lock:
            return self.goal_start_time

    def _path_is_blocked(self, route, dynamic_cells):
        with self.lock:
            planner = self.planner
        return planner is None or not planner.path_is_traversable(route, dynamic_cells)

    def _current_goal_id(self):
        try:
            goal_id = self.action_server.current_goal.get_goal_id().id
            if goal_id:
                return goal_id
        except AttributeError:
            pass
        return "goal-%d" % rospy.Time.now().to_nsec()

    def _is_stamp_fresh(self, stamp, timeout, now):
        if stamp.is_zero():
            return False
        age = (now - stamp).to_sec()
        return -self.max_future_stamp_skew <= age <= timeout

    @staticmethod
    def _finite(*values):
        return all(math.isfinite(value) for value in values)

    def make_plan_cb(self, req):
        """使用共享 A* 返回可执行的全局路径。"""
        response = GetPlanResponse()
        # A zero-stamped start means "use the latest robot pose". Resolve it
        # before quaternion validation because the unused request pose is
        # commonly left as an all-zero message.
        if req.start.header.stamp.is_zero():
            start = self._current_pose_snapshot()
            start_valid = start is not None
        else:
            start, start_valid = self._pose_stamped_to_xy_yaw(
                req.start, allow_zero_stamp=False
            )
        goal, goal_valid = self._pose_stamped_to_xy_yaw(req.goal, allow_zero_stamp=True)
        if not start_valid or not goal_valid:
            return response
        ready, _, _ = self._navigation_readiness(rospy.Time.now(), require_obstacles=False)
        if not ready:
            return response
        # GetPlan 的常见零时间戳 start 请求使用最新已验证的定位位姿。
        dynamic_cells, _, obstacle_fresh = self._dynamic_obstacle_snapshot(rospy.Time.now())
        path_points = self._plan_path(
            start[:2], goal, dynamic_cells if obstacle_fresh else ()
        )
        if not path_points:
            return response

        response.plan.header.frame_id = self.map_frame
        response.plan.header.stamp = rospy.Time.now()
        for index, (x, y) in enumerate(path_points):
            point = PoseStamped()
            point.header = response.plan.header
            point.pose.position.x = x
            point.pose.position.y = y
            yaw = goal[2]
            if index + 1 < len(path_points):
                next_x, next_y = path_points[index + 1]
                yaw = math.atan2(next_y - y, next_x - x)
            quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
            point.pose.orientation.x = quaternion[0]
            point.pose.orientation.y = quaternion[1]
            point.pose.orientation.z = quaternion[2]
            point.pose.orientation.w = quaternion[3]
            response.plan.poses.append(point)
        return response

    def clear_costmaps_cb(self, _req):
        """无独立 costmap；请求当前地图/障碍数据立即重规划。"""
        with self.lock:
            self.clear_costmaps_requested = self.goal_state.active
        rospy.loginfo("[navigation] clear_costmaps：%s", "已请求当前路径重规划" if self.clear_costmaps_requested else "当前没有活动路径")
        return EmptyResponse()

    def execute_cb(self, goal):
        """使用共享 A* 建立路线并跟踪前视点，而非直冲最终目标。"""
        target, target_valid = self._pose_stamped_to_xy_yaw(
            goal.target_pose, allow_zero_stamp=False
        )
        if not target_valid:
            self._finish_goal("UNREACHABLE", "目标必须是时间有效、四元数合法的 map 位姿")
            return

        now = rospy.Time.now()
        ready, code, detail = self._navigation_readiness(now, require_obstacles=True)
        if not ready:
            self._finish_goal(code, detail)
            return
        current = self._current_pose_snapshot()
        if current is None:
            self._finish_goal("LOCALIZATION_LOST", "读取规划起点时定位位姿已失效")
            return
        dynamic_cells, _, obstacle_fresh = self._dynamic_obstacle_snapshot(now)
        route, tracking_target, projected = self._plan_goal_path(
            current[:2], target, dynamic_cells if obstacle_fresh else ()
        )
        if route is None:
            route = self._plan_unknown_fallback(
                current[:2], target, dynamic_cells if obstacle_fresh else ()
            )
            tracking_target = target if route is not None else None
            projected = False
        if not route:
            self._finish_goal(
                "UNREACHABLE",
                "原始目标及目标容差范围内的候选落点均不可达",
            )
            return

        with self.lock:
            self.goal_state.begin(self._current_goal_id())
            self.has_active_goal = True
            self.controller_active = True
            self.active_goal_id = self.goal_state.active_goal_id
            self.goal_start_time = now
            self.last_position = current[:2]
            self.last_progress_time = now
            self.active_tracking_target = tracking_target
            self.goal_projected = projected
            self._apply_route_locked(route, now)

        terminal_code = None
        terminal_detail = ""
        terminal_stuck = False
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if self.action_server.is_preempt_requested():
                terminal_code, terminal_detail = "CANCELED", "目标被客户端取消"
                break
            ready, code, detail = self._navigation_readiness(now, require_obstacles=True)
            if not ready:
                terminal_code, terminal_detail = code, detail
                break
            if (now - self._goal_start_time()).to_sec() > self.goal_timeout:
                terminal_code, terminal_detail = "TIMEOUT", "目标超过配置的执行时限"
                break

            current = self._current_pose_snapshot()
            if current is None:
                terminal_code, terminal_detail = "LOCALIZATION_LOST", "控制循环中定位位姿已失效"
                break
            dynamic_cells, front_clearance, obstacle_fresh = self._dynamic_obstacle_snapshot(now)
            route, route_generation, last_replan = self._route_snapshot()
            route_blocked = self._path_is_blocked(route, dynamic_cells if obstacle_fresh else ())
            deviation = self._path_deviation(current[:2], route)
            with self.lock:
                tracking_target = self.active_tracking_target or target
            route_endpoint_reached = (
                bool(route)
                and math.hypot(
                    current[0] - route[-1][0], current[1] - route[-1][1]
                ) <= self._tracking_tolerance(self.goal_projected)
                and not self._requested_goal_reached(current, target)
            )
            with self.lock:
                requested = self.clear_costmaps_requested
                self.clear_costmaps_requested = False
            replan_due = (
                route_generation != self._map_generation_snapshot()
                or route_blocked
                or route_endpoint_reached
                or requested
                or deviation > self.replan_deviation_distance
                or (now - last_replan).to_sec() >= self.replan_period
            )
            if replan_due and (now - last_replan).to_sec() >= self.replan_min_interval:
                route, tracking_target, projected = self._plan_goal_path(
                    current[:2], target, dynamic_cells if obstacle_fresh else ()
                )
                if route is None:
                    route = self._plan_unknown_fallback(
                        current[:2], target, dynamic_cells if obstacle_fresh else ()
                    )
                    tracking_target = target if route is not None else None
                    projected = False
                if not route:
                    terminal_code = "UNREACHABLE"
                    terminal_detail = "原始目标及目标容差范围内的候选落点均被最新地图或临时障碍阻断"
                    break
                with self.lock:
                    self.active_tracking_target = tracking_target
                    self.goal_projected = projected
                    self._apply_route_locked(route, now)

            if self._requested_goal_reached(current, target):
                terminal_code = "SUCCEEDED"
                terminal_detail = (
                    "已到达目标容差内的安全落点并满足最终朝向"
                    if self.goal_projected
                    else "已达到目标位置和最终朝向"
                )
                break
            distance_to_goal = math.hypot(
                current[0] - tracking_target[0], current[1] - tracking_target[1]
            )
            yaw_error = normalize_angle(tracking_target[2] - current[2])
            tracking_tolerance = self._tracking_tolerance(self.goal_projected)
            if distance_to_goal <= tracking_tolerance:
                self._publish_velocity(0.0, self.final_yaw_gain * yaw_error)
            else:
                linear_x, angular_z = self._path_tracking_command(current, self._route_snapshot()[0])
                if (route_blocked or front_clearance <= self.dynamic_stop_distance) and linear_x > 0.0:
                    linear_x = 0.0
                self._publish_velocity(linear_x, angular_z)
                if self._is_stuck(current[:2], linear_x, now):
                    terminal_code = "CONTROL_FAILED"
                    terminal_detail = "机器人在发布有效前进指令时持续无路径进展"
                    terminal_stuck = True
                    break
            self._publish_feedback(current)
            rate.sleep()

        if terminal_code is None:
            terminal_code, terminal_detail = "CANCELED", "节点关闭中断活动目标"
        self._finish_goal(terminal_code, terminal_detail, terminal_stuck)

    def _finish_goal(self, code, detail, stuck=False):
        """唯一设置 Action 终态的出口，所有终态先显式停车。"""
        # 取消与失败同时到达时，Action 语义优先保持为 PREEMPTED/CANCELED。
        if code != "CANCELED" and self.action_server.is_preempt_requested():
            code, detail, stuck = "CANCELED", "目标被客户端取消", False
        with self.lock:
            self.goal_state.finish(code, detail, stuck=stuck)
            self.has_active_goal = False
            self.controller_active = False
            self.active_goal_id = ""
            self.active_path = []
            self.active_path_lengths = []
            self.waypoint_index = 0
            self.active_tracking_target = None
            self.goal_projected = False
            self.failure_code = code
            self.failure_detail = detail
            self.is_stuck = bool(stuck)
        self._stop_robot()
        # Publish the semantic terminal code before completing the Action.
        # Mission's done callback otherwise races the 5 Hz health timer and can
        # observe the previous goal's NONE code.
        self.publish_health()
        result = MoveBaseResult()
        if code == "SUCCEEDED":
            self.action_server.set_succeeded(result, text=detail)
        elif code == "CANCELED":
            self.action_server.set_preempted(result, text=detail)
        else:
            self.action_server.set_aborted(result, text=detail)

    def preempt_cb(self):
        """取消到达时立即零速；execute_cb 统一发布 PREEMPTED。"""
        rospy.loginfo("[navigation] 收到目标取消请求，立即停车")
        self._stop_robot()

    def control_loop(self, _event):
        """没有活动目标时持续发送零速度，消除旧命令残留。"""
        with self.lock:
            active = self.goal_state.active
        if not active:
            self._stop_robot()

    def _is_stuck(self, current_xy, linear_x, now):
        """只在实际发布有效前进命令且无路径进展时标记卡住。"""
        if linear_x < self.stuck_command_speed:
            return False
        with self.lock:
            if self.last_position is None:
                self.last_position = current_xy
                self.last_progress_time = now
                return False
            moved = math.hypot(
                current_xy[0] - self.last_position[0], current_xy[1] - self.last_position[1]
            )
            if moved >= self.progress_distance:
                self.last_position = current_xy
                self.last_progress_time = now
                return False
            if (now - self.last_progress_time).to_sec() > self.stuck_timeout:
                self.goal_state.stuck = True
                self.is_stuck = True
                return True
        return False

    def _path_tracking_command(self, current, route):
        """跟踪路径前视点；大偏航时先原地旋转。"""
        if not route:
            return 0.0, 0.0
        current_x, current_y, current_yaw = current
        index = self._closest_path_index(current_x, current_y, route)
        target_x, target_y = route[-1]
        for point_x, point_y in route[index:]:
            if math.hypot(point_x - current_x, point_y - current_y) >= self.lookahead_distance:
                target_x, target_y = point_x, point_y
                break
        heading_error = normalize_angle(
            math.atan2(target_y - current_y, target_x - current_x) - current_yaw
        )
        if abs(heading_error) >= self.rotate_in_place_angle:
            return 0.0, self.rotate_in_place_gain * heading_error
        distance_to_goal = math.hypot(route[-1][0] - current_x, route[-1][1] - current_y)
        linear_x = min(self.cruise_speed, self.max_linear_speed, distance_to_goal)
        linear_x *= max(0.0, math.cos(heading_error))
        angular_z = linear_x * 2.0 * math.sin(heading_error) / self.lookahead_distance
        return linear_x, angular_z

    def _closest_path_index(self, current_x, current_y, route):
        with self.lock:
            start_index = min(self.waypoint_index, max(0, len(route) - 1))
        best_index = start_index
        best_distance = float("inf")
        for index in range(start_index, len(route)):
            distance = math.hypot(route[index][0] - current_x, route[index][1] - current_y)
            if distance < best_distance:
                best_index, best_distance = index, distance
        with self.lock:
            self.waypoint_index = max(self.waypoint_index, best_index)
        return best_index

    def _path_deviation(self, current_xy, route):
        if not route:
            return float("inf")
        return min(math.hypot(current_xy[0] - point[0], current_xy[1] - point[1]) for point in route)

    def _publish_velocity(self, linear_x, angular_z):
        """发布受速度上限约束的导航命令，并在发布前重新确认安全门。"""
        command = Twist()
        ready, _, _ = self._navigation_readiness(
            rospy.Time.now(), require_obstacles=True
        )
        if ready:
            command.linear.x = self._clamp(linear_x, -self.max_linear_speed, self.max_linear_speed)
            command.angular.z = self._clamp(angular_z, -self.max_angular_speed, self.max_angular_speed)
        with self.lock:
            stamp = rospy.Time.now()
            self.last_cmd_time = stamp
            self.goal_state.record_command(stamp)
        self.cmd_pub.publish(command)

    def _stop_robot(self):
        """立即发送导航零速度；navigation 从不直接发布 /cmd_vel。"""
        command = Twist()
        with self.lock:
            stamp = rospy.Time.now()
            self.last_cmd_time = stamp
            self.goal_state.record_command(stamp)
        self.cmd_pub.publish(command)

    def _publish_feedback(self, current):
        """根据真正已走过的累计路径长度发布反馈和 health 进度。"""
        route, _, _ = self._route_snapshot()
        if not route:
            return
        index = self._closest_path_index(current[0], current[1], route)
        with self.lock:
            self.goal_state.progress = path_progress(self.active_path_lengths, index)
        feedback = MoveBaseFeedback()
        feedback.base_position.header.stamp = rospy.Time.now()
        feedback.base_position.header.frame_id = self.map_frame
        feedback.base_position.pose.position.x = current[0]
        feedback.base_position.pose.position.y = current[1]
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, current[2])
        feedback.base_position.pose.orientation.x = quaternion[0]
        feedback.base_position.pose.orientation.y = quaternion[1]
        feedback.base_position.pose.orientation.z = quaternion[2]
        feedback.base_position.pose.orientation.w = quaternion[3]
        self.action_server.publish_feedback(feedback)

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))

    def publish_health(self, _event=None):
        """发布真实 readiness、Action 生命周期、命令时刻和失败详情。"""
        now = rospy.Time.now()
        ready, readiness_code, readiness_detail = self._navigation_readiness(
            now, require_obstacles=self.require_obstacle_cloud
        )
        with self.lock:
            state = self.goal_state
            msg = NavigationHealth()
            msg.header.stamp = now
            msg.ready = ready
            msg.controller_active = state.controller_active
            msg.stuck = state.stuck
            msg.fallen = False  # P0 没有摔倒传感器，不能伪造检测结果。
            msg.has_active_goal = state.active
            msg.active_goal_id = state.active_goal_id
            msg.progress = state.progress
            msg.last_cmd_time = self.last_cmd_time
            if not ready and state.failure_code == "NONE":
                msg.failure_code = readiness_code
                msg.failure_detail = readiness_detail
            else:
                msg.failure_code = state.failure_code
                msg.failure_detail = state.failure_detail
        self.health_pub.publish(msg)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        NavController().run()
    except rospy.ROSInterruptException:
        pass
