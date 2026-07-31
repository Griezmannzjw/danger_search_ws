#!/usr/bin/env python3
"""The only danger-search navigation Action server.

This node owns /move_base, /move_base/make_plan, and
/danger_search/nav_cmd_vel.  It never publishes /cmd_vel.
"""

import math
import threading

import actionlib
import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback, MoveBaseResult
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan, GetPlanResponse
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyResponse

from danger_search_common.msg import MappingStatus, NavigationHealth
from navigation_core import GoalState, InflatedOccupancyGrid


class NavController:
    def __init__(self):
        rospy.init_node("nav_controller", anonymous=False)

        self._load_private_parameters()
        self._validate_configuration()

        self.lock = threading.RLock()
        self.current_pose = None
        self.pose_stamp = rospy.Time(0)
        self.pose_valid = False
        self.grid = None
        self.map_stamp = rospy.Time(0)
        self.map_valid = False
        self.map_generation = 0
        self.mapping_status = None
        self.mapping_status_stamp = rospy.Time(0)
        self.latest_obstacles_base = []
        self.obstacle_stamp = rospy.Time(0)
        self.obstacle_frame_valid = False
        self.safety_stop = False
        self.safety_stop_stamp = rospy.Time(0)

        self.goal_state = GoalState()
        self.active_path = []
        self.active_path_lengths = []
        self.active_path_index = 0
        self.active_goal = None
        self.active_map_generation = 0
        self.goal_start_time = rospy.Time(0)
        self.last_replan_time = rospy.Time(0)
        self.last_progress_time = rospy.Time(0)
        self.last_progress_pose = None
        self.last_progress_value = 0.0
        self.last_linear_cmd = 0.0
        self.last_angular_cmd = 0.0
        self.last_cmd_time = rospy.Time(0)
        self.clear_costmaps_requested = False

        self.lidar_rotation = tf.transformations.euler_matrix(
            self.lidar_roll, self.lidar_pitch, self.lidar_yaw)[:3, :3]

        self.cmd_pub = rospy.Publisher(self.nav_cmd_topic, Twist, queue_size=10)
        self.health_pub = rospy.Publisher(self.health_topic, NavigationHealth, queue_size=10)

        self.pose_sub = rospy.Subscriber(
            self.pose_topic, PoseWithCovarianceStamped, self.pose_callback, queue_size=10)
        self.map_sub = rospy.Subscriber(
            self.map_topic, OccupancyGrid, self.map_callback, queue_size=2)
        self.mapping_status_sub = rospy.Subscriber(
            self.mapping_status_topic, MappingStatus, self.mapping_status_callback, queue_size=10)
        self.obstacle_sub = rospy.Subscriber(
            self.obstacle_cloud_topic, PointCloud, self.obstacle_callback, queue_size=2)
        self.safety_stop_sub = rospy.Subscriber(
            self.safety_stop_topic, Bool, self.safety_stop_callback, queue_size=2)

        self.action_server = actionlib.SimpleActionServer(
            self.move_base_action_name,
            MoveBaseAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.action_server.register_preempt_callback(self.preempt_cb)
        self.action_server.start()

        self.make_plan_srv = rospy.Service(
            self.make_plan_service, GetPlan, self.make_plan_cb)
        self.clear_costmaps_srv = rospy.Service(
            self.clear_costmaps_service, Empty, self.clear_costmaps_cb)

        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate), self.control_loop)
        self.health_timer = rospy.Timer(
            rospy.Duration(1.0 / self.health_rate), self.publish_health)

        rospy.loginfo(
            "[navigation] action=%s make_plan=%s cmd=%s",
            self.move_base_action_name,
            self.make_plan_service,
            self.nav_cmd_topic,
        )

    def _param(self, name):
        return rospy.get_param("~" + name)

    def _load_private_parameters(self):
        # All names, frames, thresholds, timeouts, and LiDAR extrinsics are
        # private parameters populated by config/default.yaml.
        self.map_frame = self._param("map_frame")
        self.base_frame = self._param("base_frame")
        self.lidar_frame = self._param("lidar_frame")

        self.nav_cmd_topic = self._param("nav_cmd_topic")
        self.pose_topic = self._param("pose_topic")
        self.map_topic = self._param("map_topic")
        self.mapping_status_topic = self._param("mapping_status_topic")
        self.health_topic = self._param("health_topic")
        self.obstacle_cloud_topic = self._param("obstacle_cloud_topic")
        self.safety_stop_topic = self._param("safety_stop_topic")
        self.move_base_action_name = self._param("move_base_action_name")
        self.make_plan_service = self._param("make_plan_service")
        self.clear_costmaps_service = self._param("clear_costmaps_service")

        self.occupied_threshold = int(self._param("occupied_threshold"))
        self.robot_radius = float(self._param("robot_radius"))
        self.inflation_padding = float(self._param("inflation_padding"))
        self.allow_diagonal = bool(self._param("allow_diagonal"))
        self.max_planner_expansions = int(self._param("max_planner_expansions"))

        self.pose_timeout = float(self._param("pose_timeout"))
        self.map_timeout = float(self._param("map_timeout"))
        self.mapping_status_timeout = float(self._param("mapping_status_timeout"))
        self.obstacle_cloud_timeout = float(self._param("obstacle_cloud_timeout"))
        self.goal_stamp_timeout = float(self._param("goal_stamp_timeout"))
        self.max_future_stamp_skew = float(self._param("max_future_stamp_skew"))
        self.quaternion_norm_tolerance = float(self._param("quaternion_norm_tolerance"))
        self.require_obstacle_cloud = bool(self._param("require_obstacle_cloud"))

        self.lidar_x = float(self._param("lidar_x"))
        self.lidar_y = float(self._param("lidar_y"))
        self.lidar_z = float(self._param("lidar_z"))
        self.lidar_roll = float(self._param("lidar_roll"))
        self.lidar_pitch = float(self._param("lidar_pitch"))
        self.lidar_yaw = float(self._param("lidar_yaw"))
        self.obstacle_min_z = float(self._param("obstacle_min_z"))
        self.obstacle_max_z = float(self._param("obstacle_max_z"))
        self.obstacle_range_min = float(self._param("obstacle_range_min"))
        self.obstacle_range_max = float(self._param("obstacle_range_max"))
        self.zero_point_radius = float(self._param("zero_point_radius"))
        self.obstacle_max_points = int(self._param("obstacle_max_points"))
        self.dynamic_stop_distance = float(self._param("dynamic_stop_distance"))
        self.dynamic_front_half_angle = float(self._param("dynamic_front_half_angle"))

        self.control_rate = float(self._param("control_rate"))
        self.health_rate = float(self._param("health_rate"))
        self.goal_timeout = float(self._param("goal_timeout"))
        self.goal_tolerance_xy = float(self._param("goal_tolerance_xy"))
        self.goal_tolerance_yaw = float(self._param("goal_tolerance_yaw"))
        self.lookahead_distance = float(self._param("lookahead_distance"))
        self.cruise_speed = float(self._param("cruise_speed"))
        self.max_linear_speed = float(self._param("max_linear_speed"))
        self.max_angular_speed = float(self._param("max_angular_speed"))
        self.max_linear_accel = float(self._param("max_linear_accel"))
        self.max_angular_accel = float(self._param("max_angular_accel"))
        self.max_decel = float(self._param("max_decel"))
        self.rotate_in_place_angle = float(self._param("rotate_in_place_angle"))
        self.rotate_in_place_gain = float(self._param("rotate_in_place_gain"))
        self.final_yaw_gain = float(self._param("final_yaw_gain"))
        self.replan_period = float(self._param("replan_period"))
        self.replan_min_interval = float(self._param("replan_min_interval"))
        self.replan_deviation_distance = float(self._param("replan_deviation_distance"))
        self.stuck_timeout = float(self._param("stuck_timeout"))
        self.progress_distance = float(self._param("progress_distance"))
        self.stuck_command_speed = float(self._param("stuck_command_speed"))

    def _validate_configuration(self):
        positive_values = (
            self.robot_radius,
            self.max_planner_expansions,
            self.pose_timeout,
            self.map_timeout,
            self.mapping_status_timeout,
            self.obstacle_cloud_timeout,
            self.goal_stamp_timeout,
            self.max_future_stamp_skew,
            self.quaternion_norm_tolerance,
            self.obstacle_range_max,
            self.obstacle_max_points,
            self.dynamic_stop_distance,
            self.dynamic_front_half_angle,
            self.control_rate,
            self.health_rate,
            self.goal_timeout,
            self.goal_tolerance_xy,
            self.goal_tolerance_yaw,
            self.lookahead_distance,
            self.cruise_speed,
            self.max_linear_speed,
            self.max_angular_speed,
            self.max_linear_accel,
            self.max_angular_accel,
            self.max_decel,
            self.rotate_in_place_angle,
            self.rotate_in_place_gain,
            self.final_yaw_gain,
            self.replan_period,
            self.replan_min_interval,
            self.replan_deviation_distance,
            self.stuck_timeout,
            self.progress_distance,
        )
        if any(value <= 0.0 for value in positive_values):
            raise rospy.ROSInitException("navigation numeric parameters must be positive")
        if self.inflation_padding < 0.0 or self.obstacle_range_min < 0.0:
            raise rospy.ROSInitException("navigation range parameters must be non-negative")
        if self.obstacle_min_z > self.obstacle_max_z:
            raise rospy.ROSInitException("obstacle_min_z must not exceed obstacle_max_z")
        if self.obstacle_range_min >= self.obstacle_range_max:
            raise rospy.ROSInitException("invalid obstacle range")
        if self.occupied_threshold < 1:
            raise rospy.ROSInitException("occupied_threshold must be positive")

    def pose_callback(self, msg):
        pose, valid = self._pose_from_message(msg)
        with self.lock:
            self.current_pose = pose
            self.pose_stamp = msg.header.stamp
            self.pose_valid = valid

    def map_callback(self, msg):
        grid = None
        valid = False
        if msg.header.frame_id == self.map_frame and not msg.header.stamp.is_zero():
            orientation = msg.info.origin.orientation
            yaw = self._quaternion_yaw(orientation)
            if yaw is not None:
                try:
                    grid = InflatedOccupancyGrid(
                        msg.info.width,
                        msg.info.height,
                        msg.info.resolution,
                        msg.info.origin.position.x,
                        msg.info.origin.position.y,
                        yaw,
                        msg.data,
                        self.occupied_threshold,
                        self.robot_radius,
                        self.inflation_padding,
                        self.allow_diagonal,
                        self.max_planner_expansions,
                    )
                    valid = True
                except ValueError as exc:
                    rospy.logwarn_throttle(5.0, "[navigation] rejected map: %s", exc)
        if not valid:
            rospy.logwarn_throttle(
                5.0,
                "[navigation] rejected map frame, stamp, origin orientation, or geometry",
            )
        with self.lock:
            self.grid = grid
            self.map_stamp = msg.header.stamp
            self.map_valid = valid
            if valid:
                self.map_generation += 1

    def mapping_status_callback(self, msg):
        with self.lock:
            self.mapping_status = msg
            self.mapping_status_stamp = msg.header.stamp

    def obstacle_callback(self, cloud):
        points = []
        frame_valid = cloud.header.frame_id == self.lidar_frame and not cloud.header.stamp.is_zero()
        if frame_valid:
            point_count = len(cloud.points)
            step = max(1, int(math.ceil(float(point_count) / self.obstacle_max_points)))
            for point in cloud.points[::step]:
                if not self._finite(point.x, point.y, point.z):
                    continue
                if point.x * point.x + point.y * point.y + point.z * point.z < self.zero_point_radius ** 2:
                    continue
                base_x, base_y, base_z = self._lidar_to_base(point.x, point.y, point.z)
                distance = math.hypot(base_x, base_y)
                if base_z < self.obstacle_min_z or base_z > self.obstacle_max_z:
                    continue
                if distance < self.obstacle_range_min or distance > self.obstacle_range_max:
                    continue
                points.append((base_x, base_y, base_z))
        else:
            rospy.logwarn_throttle(
                5.0,
                "[navigation] rejected obstacle cloud frame or zero stamp; expected frame=%s",
                self.lidar_frame,
            )
        with self.lock:
            self.latest_obstacles_base = points
            self.obstacle_stamp = cloud.header.stamp
            self.obstacle_frame_valid = frame_valid

    def safety_stop_callback(self, msg):
        with self.lock:
            self.safety_stop = bool(msg.data)
            self.safety_stop_stamp = rospy.Time.now()

    def make_plan_cb(self, req):
        response = GetPlanResponse()
        start, start_valid = self._pose_stamped_to_xy_yaw(req.start, check_stamp=True)
        goal, goal_valid = self._pose_stamped_to_xy_yaw(req.goal, check_stamp=True)
        if not start_valid or not goal_valid:
            return response

        ready, _, _ = self._navigation_readiness(rospy.Time.now(), require_obstacles=False)
        if not ready:
            return response

        dynamic_cells, _, obstacle_fresh = self._dynamic_obstacle_snapshot(rospy.Time.now())
        route = self._plan_route(start[:2], goal[:2], dynamic_cells if obstacle_fresh else set())
        if route is None:
            return response
        return self._route_response(route, goal[2])

    def clear_costmaps_cb(self, _req):
        # There is no independent costmap owner.  The next control iteration
        # recomputes the route from the latest inflated OccupancyGrid instead.
        with self.lock:
            self.clear_costmaps_requested = True
        return EmptyResponse()

    def execute_cb(self, goal):
        target, target_valid = self._pose_stamped_to_xy_yaw(goal.target_pose, check_stamp=True)
        if not target_valid:
            self._reject_goal("UNREACHABLE", "Goal must have a fresh map-frame pose and valid quaternion")
            return

        now = rospy.Time.now()
        ready, readiness_code, readiness_detail = self._navigation_readiness(now, require_obstacles=True)
        if not ready:
            self._reject_goal(readiness_code, readiness_detail)
            return

        current = self._current_pose_snapshot()
        dynamic_cells, _, obstacle_fresh = self._dynamic_obstacle_snapshot(now)
        if self.require_obstacle_cloud and not obstacle_fresh:
            self._reject_goal("CONTROL_FAILED", "Obstacle cloud is missing, stale, or in the wrong frame")
            return

        route = self._plan_route(current[:2], target[:2], dynamic_cells if obstacle_fresh else set())
        if route is None:
            self._reject_goal("UNREACHABLE", "Goal is outside free inflated map space or not reachable")
            return

        goal_id = self._current_goal_id()
        with self.lock:
            self.goal_state.begin(goal_id)
            self.active_goal = target
            self.goal_start_time = now
            self._apply_route_locked(route, now)
            self.last_progress_pose = current[:2]
            self.last_progress_value = 0.0
            self.last_progress_time = now

        terminal_code = None
        terminal_detail = ""
        terminal_stuck = False
        rate = rospy.Rate(self.control_rate)

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if self.action_server.is_preempt_requested():
                terminal_code = "CANCELED"
                terminal_detail = "Action goal canceled"
                break

            ready, readiness_code, readiness_detail = self._navigation_readiness(now, require_obstacles=True)
            if not ready:
                terminal_code = readiness_code
                terminal_detail = readiness_detail
                break

            with self.lock:
                if self.safety_stop:
                    terminal_code = "SAFETY_STOP"
                    terminal_detail = "Safety stop signal is active"
            if terminal_code is not None:
                break

            current = self._current_pose_snapshot()
            if (now - self._goal_start_time()).to_sec() > self.goal_timeout:
                terminal_code = "TIMEOUT"
                terminal_detail = "Goal exceeded configured timeout"
                break

            dynamic_cells, front_clearance, obstacle_fresh = self._dynamic_obstacle_snapshot(now)
            if self.require_obstacle_cloud and not obstacle_fresh:
                terminal_code = "CONTROL_FAILED"
                terminal_detail = "Obstacle cloud became stale or invalid during navigation"
                break

            route, path_generation, last_replan = self._route_snapshot()
            path_blocked = self._path_is_blocked(route, dynamic_cells if obstacle_fresh else set())
            deviation = self._path_deviation(current[:2], route)
            with self.lock:
                replan_requested = self.clear_costmaps_requested
                self.clear_costmaps_requested = False
            replan_due = (
                path_generation != self._map_generation_snapshot()
                or (now - last_replan).to_sec() >= self.replan_period
                or deviation > self.replan_deviation_distance
                or path_blocked
                or replan_requested
            )

            if replan_due and (now - last_replan).to_sec() >= self.replan_min_interval:
                replanned_route = self._plan_route(
                    current[:2], target[:2], dynamic_cells if obstacle_fresh else set())
                if replanned_route is None:
                    terminal_code = "UNREACHABLE"
                    terminal_detail = "No route remains on the current inflated map"
                    break
                with self.lock:
                    self._apply_route_locked(replanned_route, now)
                route, _, _ = self._route_snapshot()
                path_blocked = False

            distance_to_goal = math.hypot(current[0] - target[0], current[1] - target[1])
            if distance_to_goal <= self.goal_tolerance_xy:
                yaw_error = self._normalize_angle(target[2] - current[2])
                if abs(yaw_error) <= self.goal_tolerance_yaw:
                    terminal_code = "SUCCEEDED"
                    terminal_detail = "Goal position and orientation reached"
                    break
                self._publish_velocity(0.0, self.final_yaw_gain * yaw_error)
                self._publish_feedback(current)
                rate.sleep()
                continue

            linear_x, angular_z = self._path_tracking_command(current, route)
            force_linear_zero = False
            if front_clearance <= self.dynamic_stop_distance and linear_x > 0.0:
                # Rotation is safe and may face the first waypoint away from a
                # newly observed obstacle.  Forward motion remains stopped.
                linear_x = 0.0
                force_linear_zero = True
            if path_blocked and linear_x > 0.0:
                linear_x = 0.0
                force_linear_zero = True
            self._publish_velocity(linear_x, angular_z, force_linear_zero=force_linear_zero)
            self._publish_feedback(current)

            if self._is_stuck(current[:2], linear_x, now):
                terminal_code = "CONTROL_FAILED"
                terminal_detail = "Robot made no configured minimum path progress"
                terminal_stuck = True
                break
            rate.sleep()

        if terminal_code is None:
            terminal_code = "CANCELED"
            terminal_detail = "ROS shutdown interrupted the active goal"
        self._finish_goal(terminal_code, terminal_detail, terminal_stuck)

    def preempt_cb(self):
        with self.lock:
            active = self.goal_state.active
            if active:
                self.goal_state.cancel("Action cancel requested")
        if active:
            self._stop_robot()

    def control_loop(self, _event):
        with self.lock:
            active = self.goal_state.active
        if not active:
            self._stop_robot()

    def publish_health(self, _event=None):
        now = rospy.Time.now()
        ready, _, _ = self._navigation_readiness(now, require_obstacles=False)
        with self.lock:
            state = self.goal_state
            last_cmd_time = self.last_cmd_time
            safety_stop = self.safety_stop
            msg = NavigationHealth()
            msg.header.stamp = now
            msg.ready = ready and not safety_stop
            msg.controller_active = state.controller_active
            msg.stuck = state.stuck
            msg.fallen = False
            msg.has_active_goal = state.active
            msg.active_goal_id = state.active_goal_id
            msg.progress = state.progress
            msg.last_cmd_time = last_cmd_time
            msg.failure_code = state.failure_code
            msg.failure_detail = state.failure_detail
        self.health_pub.publish(msg)

    def _pose_from_message(self, msg):
        if msg.header.frame_id != self.map_frame or msg.header.stamp.is_zero():
            return None, False
        position = msg.pose.pose.position
        yaw = self._quaternion_yaw(msg.pose.pose.orientation)
        if yaw is None or not self._finite(position.x, position.y, position.z):
            return None, False
        return (position.x, position.y, yaw), True

    def _pose_stamped_to_xy_yaw(self, pose, check_stamp):
        if pose.header.frame_id != self.map_frame:
            return None, False
        if check_stamp and not self._is_stamp_fresh(
                pose.header.stamp, self.goal_stamp_timeout, rospy.Time.now()):
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
            quaternion.x * quaternion.x
            + quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
            + quaternion.w * quaternion.w)
        if abs(norm - 1.0) > self.quaternion_norm_tolerance:
            return None
        return tf.transformations.euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w])[2]

    def _navigation_readiness(self, now, require_obstacles):
        with self.lock:
            pose_valid = self.pose_valid and self._is_stamp_fresh(
                self.pose_stamp, self.pose_timeout, now)
            map_valid = self.map_valid and self._is_stamp_fresh(
                self.map_stamp, self.map_timeout, now)
            mapping = self.mapping_status
            mapping_fresh = mapping is not None and self._is_stamp_fresh(
                self.mapping_status_stamp, self.mapping_status_timeout, now)
            obstacle_ready = self.obstacle_frame_valid and self._is_stamp_fresh(
                self.obstacle_stamp, self.obstacle_cloud_timeout, now)
            safety_stop = self.safety_stop

        if safety_stop:
            return False, "SAFETY_STOP", "Safety stop signal is active"
        if not pose_valid:
            return False, "LOCALIZATION_LOST", "Map-frame pose is missing, stale, or invalid"
        if not map_valid:
            return False, "LOCALIZATION_LOST", "Inflated OccupancyGrid is missing, stale, or invalid"
        if not mapping_fresh:
            return False, "LOCALIZATION_LOST", "MappingStatus is missing or stale"
        if mapping.lost:
            return False, "LOCALIZATION_LOST", mapping.status_reason or "Localization reported lost"
        if not mapping.ready or not mapping.stable:
            return False, "LOCALIZATION_LOST", mapping.status_reason or "Localization is not ready and stable"
        if require_obstacles and self.require_obstacle_cloud and not obstacle_ready:
            return False, "CONTROL_FAILED", "Obstacle cloud is missing, stale, or in the wrong frame"
        return True, "NONE", ""

    def _dynamic_obstacle_snapshot(self, now):
        with self.lock:
            grid = self.grid
            pose = self.current_pose
            obstacles = list(self.latest_obstacles_base)
            fresh = self.obstacle_frame_valid and self._is_stamp_fresh(
                self.obstacle_stamp, self.obstacle_cloud_timeout, now)
        if grid is None or pose is None or not fresh:
            return set(), float("inf"), fresh

        pose_x, pose_y, pose_yaw = pose
        cos_yaw = math.cos(pose_yaw)
        sin_yaw = math.sin(pose_yaw)
        dynamic_cells = set()
        front_clearance = float("inf")
        for base_x, base_y, _ in obstacles:
            if base_x > 0.0 and abs(math.atan2(base_y, base_x)) <= self.dynamic_front_half_angle:
                front_clearance = min(front_clearance, math.hypot(base_x, base_y))
            world_x = pose_x + cos_yaw * base_x - sin_yaw * base_y
            world_y = pose_y + sin_yaw * base_x + cos_yaw * base_y
            cell = grid.world_to_cell(world_x, world_y)
            if cell is not None:
                dynamic_cells.add(cell)
        return dynamic_cells, front_clearance, True

    def _plan_route(self, start_xy, goal_xy, dynamic_cells):
        with self.lock:
            grid = self.grid
        if grid is None:
            return None
        return grid.plan(start_xy, goal_xy, dynamic_cells)

    def _route_response(self, route, goal_yaw):
        response = GetPlanResponse()
        response.plan.header.stamp = rospy.Time.now()
        response.plan.header.frame_id = self.map_frame
        for index, (x, y) in enumerate(route):
            pose = PoseStamped()
            pose.header = response.plan.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            yaw = goal_yaw
            if index + 1 < len(route):
                next_x, next_y = route[index + 1]
                yaw = math.atan2(next_y - y, next_x - x)
            quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            response.plan.poses.append(pose)
        return response

    def _apply_route_locked(self, route, now):
        self.active_path = list(route)
        self.active_path_lengths = self._path_lengths(self.active_path)
        self.active_path_index = 0
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
            grid = self.grid
        return grid is None or not grid.path_is_traversable(route, dynamic_cells)

    def _path_tracking_command(self, current, route):
        current_x, current_y, current_yaw = current
        index = self._closest_path_index(current_x, current_y, route)
        target_x, target_y = route[-1]
        for path_x, path_y in route[index:]:
            if math.hypot(path_x - current_x, path_y - current_y) >= self.lookahead_distance:
                target_x, target_y = path_x, path_y
                break

        heading_error = self._normalize_angle(
            math.atan2(target_y - current_y, target_x - current_x) - current_yaw)
        if abs(heading_error) >= self.rotate_in_place_angle:
            return 0.0, self.rotate_in_place_gain * heading_error

        distance_to_goal = math.hypot(route[-1][0] - current_x, route[-1][1] - current_y)
        braking_speed = math.sqrt(max(0.0, 2.0 * self.max_decel * distance_to_goal))
        linear_x = min(self.cruise_speed, self.max_linear_speed, braking_speed)
        linear_x *= max(0.0, math.cos(heading_error))
        curvature = 2.0 * math.sin(heading_error) / max(self.lookahead_distance, 1e-9)
        angular_z = linear_x * curvature
        return linear_x, angular_z

    def _closest_path_index(self, current_x, current_y, route):
        if not route:
            return 0
        with self.lock:
            start_index = min(self.active_path_index, len(route) - 1)
        best_index = start_index
        best_distance = float("inf")
        for index in range(start_index, len(route)):
            point_x, point_y = route[index]
            distance = math.hypot(point_x - current_x, point_y - current_y)
            if distance < best_distance:
                best_index = index
                best_distance = distance
        with self.lock:
            self.active_path_index = max(self.active_path_index, best_index)
        return best_index

    def _path_deviation(self, current_xy, route):
        if not route:
            return float("inf")
        return min(math.hypot(current_xy[0] - x, current_xy[1] - y) for x, y in route)

    def _is_stuck(self, current_xy, linear_x, now):
        if linear_x < self.stuck_command_speed:
            return False
        with self.lock:
            if self.last_progress_pose is None:
                self.last_progress_pose = current_xy
                self.last_progress_time = now
                return False
            distance = math.hypot(
                current_xy[0] - self.last_progress_pose[0],
                current_xy[1] - self.last_progress_pose[1])
            current_progress = self.goal_state.progress
            if distance >= self.progress_distance or current_progress > self.last_progress_value:
                self.last_progress_pose = current_xy
                self.last_progress_time = now
                self.last_progress_value = current_progress
                return False
            if (now - self.last_progress_time).to_sec() > self.stuck_timeout:
                self.goal_state.stuck = True
                return True
        return False

    def _publish_velocity(self, linear_x, angular_z, force_linear_zero=False):
        with self.lock:
            max_linear_step = self.max_linear_accel / self.control_rate
            max_angular_step = self.max_angular_accel / self.control_rate
            linear_x = self._clamp(linear_x, -self.max_linear_speed, self.max_linear_speed)
            angular_z = self._clamp(angular_z, -self.max_angular_speed, self.max_angular_speed)
            if force_linear_zero:
                linear_x = 0.0
            else:
                linear_x = self._clamp(
                    linear_x,
                    self.last_linear_cmd - max_linear_step,
                    self.last_linear_cmd + max_linear_step,
                )
            angular_z = self._clamp(
                angular_z,
                self.last_angular_cmd - max_angular_step,
                self.last_angular_cmd + max_angular_step,
            )
            self.last_linear_cmd = linear_x
            self.last_angular_cmd = angular_z
            self.last_cmd_time = rospy.Time.now()

        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        self.cmd_pub.publish(command)

    def _stop_robot(self):
        with self.lock:
            self.last_linear_cmd = 0.0
            self.last_angular_cmd = 0.0
            self.last_cmd_time = rospy.Time.now()
        self.cmd_pub.publish(Twist())

    def _publish_feedback(self, current):
        route, _, _ = self._route_snapshot()
        if not route:
            return
        index = self._closest_path_index(current[0], current[1], route)
        with self.lock:
            total = self.active_path_lengths[-1] if self.active_path_lengths else 0.0
            traversed = self.active_path_lengths[index] if self.active_path_lengths else 0.0
            self.goal_state.progress = 0.0 if total <= 0.0 else min(1.0, traversed / total)

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

    def _finish_goal(self, code, detail, stuck=False):
        with self.lock:
            self.goal_state.finish(code, detail, stuck=stuck)
            self.active_path = []
            self.active_path_lengths = []
            self.active_path_index = 0
            self.active_goal = None
        self._stop_robot()

        result = MoveBaseResult()
        if code == "SUCCEEDED":
            self.action_server.set_succeeded(result, text=detail)
        elif code == "CANCELED":
            self.action_server.set_preempted(result, text=detail)
        else:
            self.action_server.set_aborted(result, text=detail)

    def _reject_goal(self, code, detail):
        with self.lock:
            self.goal_state.finish(code, detail)
        self._stop_robot()
        self.action_server.set_aborted(MoveBaseResult(), text=detail)

    def _current_goal_id(self):
        try:
            goal_id = self.action_server.current_goal.get_goal_id().id
            if goal_id:
                return goal_id
        except AttributeError:
            pass
        return "goal-%d" % rospy.Time.now().to_nsec()

    def _lidar_to_base(self, x, y, z):
        rotated = self.lidar_rotation.dot((x, y, z))
        return (
            rotated[0] + self.lidar_x,
            rotated[1] + self.lidar_y,
            rotated[2] + self.lidar_z,
        )

    def _is_stamp_fresh(self, stamp, timeout, now):
        if stamp.is_zero():
            return False
        age = (now - stamp).to_sec()
        return -self.max_future_stamp_skew <= age <= timeout

    @staticmethod
    def _path_lengths(route):
        lengths = []
        total = 0.0
        for index, point in enumerate(route):
            if index:
                previous = route[index - 1]
                total += math.hypot(point[0] - previous[0], point[1] - previous[1])
            lengths.append(total)
        return lengths

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))

    @staticmethod
    def _finite(*values):
        return all(math.isfinite(value) for value in values)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        NavController().run()
    except rospy.ROSInterruptException:
        pass
