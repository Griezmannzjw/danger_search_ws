#!/usr/bin/env python3
"""
探索规划节点 - P0最小可运行版本
对齐接口规范 v1.1-p0

功能：
  - move_base Action客户端
  - 在地图自由区域随机选点
  - 发目标前严格检查所有条件
  - 失败不无限重试
  - stop时取消目标

P0要求：
  - 所有名称从参数读取
  - 只有满足所有条件才发目标
  - 目标必须在自由区域，make_plan返回非空路径
  - 失败候选不立即无限重试
  - stop时必须取消活动目标
"""

import rospy
import actionlib
import math
import json
import threading
from collections import deque
import cv2
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import GridCells, OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Bool, String
from nav_msgs.srv import GetPlan
from danger_search_common.msg import MappingStatus, NavigationHealth, RecoveryEvent
from danger_search_common.srv import SetCurrentFloor, TraversePortal

try:
    from building_generator_interfaces.srv import CallElevator, SetDoorState
except ImportError:
    CallElevator = None
    SetDoorState = None


class ExplorationPlanner:
    @staticmethod
    def _normalize_floor_ids(value):
        if not isinstance(value, (list, tuple)) or not value:
            raise rospy.ROSInitException("~elevator/served_floors must be a non-empty list")
        try:
            floors = sorted(set(int(item) for item in value))
        except (TypeError, ValueError):
            raise rospy.ROSInitException("~elevator/served_floors must contain integers")
        return floors

    @staticmethod
    def _parse_optional_pose_param(value):
        if value in (None, [], ()):
            return None
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise rospy.ROSInitException("~elevator/portal_pose must be [x, y, yaw]")
        pose = tuple(float(item) for item in value)
        if not all(math.isfinite(item) for item in pose):
            raise rospy.ROSInitException("~elevator/portal_pose must be finite")
        return pose

    @staticmethod
    def _derive_elevator_poses(portal_pose, lobby_offset, cabin_distance,
                               exit_distance):
        portal_x, portal_y, portal_yaw = portal_pose
        direction_x = math.cos(portal_yaw)
        direction_y = math.sin(portal_yaw)
        return {
            "portal": portal_pose,
            "lobby": (
                portal_x - lobby_offset * direction_x,
                portal_y - lobby_offset * direction_y,
                portal_yaw,
            ),
            "cabin": (
                portal_x + cabin_distance * direction_x,
                portal_y + cabin_distance * direction_y,
                portal_yaw,
            ),
            "exit": (
                portal_x - exit_distance * direction_x,
                portal_y - exit_distance * direction_y,
                math.atan2(math.sin(portal_yaw + math.pi),
                           math.cos(portal_yaw + math.pi)),
            ),
        }

    def __init__(self):
        rospy.init_node("exploration_planner", anonymous=False)

        # ========== 从参数读取所有名称 ==========
        self.map_frame = rospy.get_param("~map_frame", "map")

        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.pose_topic = rospy.get_param("~pose_topic", "/localization/pose")
        self.mapping_status_topic = rospy.get_param("~mapping_status_topic", "/mapping/status")
        self.navigation_health_topic = rospy.get_param("~navigation_health_topic", "/navigation/health")
        self.move_base_action_name = rospy.get_param("~move_base_action_name", "/move_base")
        self.make_plan_service = rospy.get_param("~make_plan_service", "/move_base/make_plan")
        self.start_service = rospy.get_param("~start_service", "/danger_search/start_exploration")
        self.stop_service = rospy.get_param("~stop_service", "/danger_search/stop_exploration")
        self.status_topic = rospy.get_param("~status_topic", "/exploration/status")
        self.complete_topic = rospy.get_param("~complete_topic", "/exploration/complete")
        self.recovery_event_topic = rospy.get_param(
            "~recovery_event_topic", "/navigation/recovery_event"
        )
        self.start_elevator_service = rospy.get_param(
            "~start_elevator_service", "/danger_search/start_elevator"
        )
        self.cancel_elevator_service = rospy.get_param(
            "~cancel_elevator_service", "/danger_search/cancel_elevator"
        )
        self.set_door_state_service = rospy.get_param(
            "~set_door_state_service", "/set_door_state"
        )
        self.call_elevator_service = rospy.get_param(
            "~call_elevator_service", "/call_elevator"
        )
        self.set_current_floor_service = rospy.get_param(
            "~set_current_floor_service", "/localization/set_current_floor"
        )
        self.traverse_portal_service = rospy.get_param(
            "~traverse_portal_service", "/navigation/traverse_portal"
        )
        self.cancel_portal_service = rospy.get_param(
            "~cancel_portal_service", "/navigation/cancel_portal"
        )

        # 探索参数
        self.goal_interval = rospy.get_param("~goal_interval", 2.0)
        self.max_retry = rospy.get_param("~max_retry", 3)
        self.min_frontier_length = rospy.get_param("~min_frontier_length", 0.6)
        self.free_threshold = int(rospy.get_param("~free_threshold", 25))
        if not 1 <= self.free_threshold <= 100:
            raise rospy.ROSInitException("~free_threshold must be within 1..100")
        self.connectivity_occupied_threshold = int(
            rospy.get_param("~connectivity_occupied_threshold", 65)
        )
        self.connectivity_clearance_radius = float(
            rospy.get_param("~connectivity_clearance_radius", 0.33)
        )
        if not 1 <= self.connectivity_occupied_threshold <= 100:
            raise rospy.ROSInitException(
                "~connectivity_occupied_threshold must be within 1..100"
            )
        if (not math.isfinite(self.connectivity_clearance_radius)
                or self.connectivity_clearance_radius < 0.0):
            raise rospy.ROSInitException(
                "~connectivity_clearance_radius must be finite and non-negative"
            )
        self.max_frontier_candidates = rospy.get_param("~max_frontier_candidates", 20)
        self.goal_timeout = rospy.get_param("~goal_timeout", 60.0)
        self.plan_tolerance = rospy.get_param("~plan_tolerance", 0.5)
        self.failed_goal_cooldown = rospy.get_param("~failed_goal_cooldown", 30.0)
        self.failed_goal_radius = rospy.get_param("~failed_goal_radius", 0.75)
        self.success_goal_cooldown = float(
            rospy.get_param("~success_goal_cooldown", 45.0)
        )
        self.success_goal_radius = float(
            rospy.get_param("~success_goal_radius", 0.50)
        )
        self.success_goal_clear_revisions = int(
            rospy.get_param("~success_goal_clear_revisions", 2)
        )
        self.dependency_check_timeout = rospy.get_param("~dependency_check_timeout", 0.1)
        self.input_timeout = rospy.get_param("~input_timeout", 3.0)
        self.no_frontier_cycles_required = rospy.get_param("~no_frontier_cycles_required", 5)
        self.map_stable_time = rospy.get_param("~map_stable_time", 8.0)
        self.map_change_cell_threshold = rospy.get_param("~map_change_cell_threshold", 5)
        self.observation_min_distance = float(
            rospy.get_param("~observation_min_distance", 0.40)
        )
        self.observation_max_distance = float(
            rospy.get_param("~observation_max_distance", 0.60)
        )
        self.observation_target_distance = float(
            rospy.get_param("~observation_target_distance", 0.50)
        )
        self.goal_clearance_margin = float(
            rospy.get_param("~goal_clearance_margin", 0.04)
        )
        self.path_length_weight = float(rospy.get_param("~path_length_weight", 1.0))
        self.frontier_gain_weight = float(rospy.get_param("~frontier_gain_weight", 0.30))
        self.path_clearance_weight = float(
            rospy.get_param("~path_clearance_weight", 0.50)
        )
        self.trap_blacklist_radius = float(
            rospy.get_param("~trap_blacklist_radius", 1.0)
        )
        self.trap_clearance_margin = float(
            rospy.get_param("~trap_clearance_margin", 0.15)
        )
        self.blacklist_clear_revisions = int(
            rospy.get_param("~blacklist_clear_revisions", 3)
        )
        if not (
                0.0 < self.observation_min_distance
                <= self.observation_target_distance
                <= self.observation_max_distance
                and self.goal_clearance_margin >= 0.0
                and self.trap_blacklist_radius > 0.0
                and self.blacklist_clear_revisions >= 1):
            raise rospy.ROSInitException("前沿观察位或长期黑名单参数无效")
        if (self.success_goal_cooldown < 0.0
                or self.success_goal_radius <= 0.0
                or self.success_goal_clear_revisions < 1):
            raise rospy.ROSInitException("成功目标冷却参数无效")

        self.elevator_enabled = bool(rospy.get_param("~elevator/enabled", False))
        self.elevator_id = rospy.get_param("~elevator/id", "elevator_main")
        self.elevator_target_floor = int(
            rospy.get_param("~elevator/target_floor", 1)
        )
        self.elevator_autonomous_when_floor_complete = bool(
            rospy.get_param("~elevator/autonomous_when_floor_complete", True)
        )
        self.elevator_served_floors = self._normalize_floor_ids(
            rospy.get_param("~elevator/served_floors", [0, 1])
        )
        self.elevator_max_autonomous_transition_failures = int(
            rospy.get_param("~elevator/max_autonomous_transition_failures", 1)
        )
        self.elevator_door_prefix = rospy.get_param(
            "~elevator/door_prefix", "elevator_floor_"
        )
        self.elevator_portal_pose = self._parse_optional_pose_param(
            rospy.get_param("~elevator/portal_pose", [])
        )
        self.elevator_lobby_offset = float(
            rospy.get_param("~elevator/lobby_offset", 0.70)
        )
        self.elevator_portal_near_distance = float(
            rospy.get_param("~elevator/portal_near_distance", 0.40)
        )
        self.elevator_cabin_distance = float(
            rospy.get_param("~elevator/cabin_distance", 0.60)
        )
        self.elevator_exit_distance = float(
            rospy.get_param("~elevator/exit_distance", 0.80)
        )
        self.elevator_nav_timeout = float(
            rospy.get_param("~elevator/navigation_timeout", 45.0)
        )
        self.elevator_service_timeout = float(
            rospy.get_param("~elevator/service_timeout", 10.0)
        )
        self.elevator_operation_timeout = float(
            rospy.get_param("~elevator/operation_timeout", 45.0)
        )
        self.elevator_floor_timeout = float(
            rospy.get_param("~elevator/floor_timeout", 45.0)
        )
        self.elevator_floor_stable_time = float(
            rospy.get_param("~elevator/floor_stable_time", 3.0)
        )
        self.elevator_close_target_door = bool(
            rospy.get_param("~elevator/close_target_door_after_exit", False)
        )
        self.elevator_portal_traversal_enabled = bool(
            rospy.get_param("~elevator/portal_traversal_enabled", True)
        )
        self.elevator_portal_speed = float(
            rospy.get_param("~elevator/portal_speed", 0.25)
        )
        self.elevator_portal_duration = float(
            rospy.get_param("~elevator/portal_duration", 3.0)
        )
        if self.elevator_enabled and (CallElevator is None or SetDoorState is None):
            raise rospy.ROSInitException(
                "elevator enabled but building_generator_interfaces is unavailable"
            )
        if (self.elevator_lobby_offset <= 0.0
                or self.elevator_portal_near_distance < 0.0
                or self.elevator_cabin_distance <= 0.0
                or self.elevator_exit_distance <= 0.0
                or min(self.elevator_nav_timeout, self.elevator_service_timeout,
                       self.elevator_operation_timeout, self.elevator_floor_timeout) <= 0.0
                or self.elevator_floor_stable_time < 0.0
                or self.elevator_portal_speed <= 0.0
                or self.elevator_portal_duration <= 0.0
                or self.elevator_max_autonomous_transition_failures < 1):
            raise rospy.ROSInitException("电梯流程参数无效")

        # ========== 状态 ==========
        self.exploring = False
        self.current_pose = None
        self.current_map = None
        self.map_info = None
        self.map_data = None
        self.mapping_ready = False
        self.mapping_stable = False
        self.mapping_lost = True
        self.current_floor = 0
        self.nav_ready = False
        self.nav_has_active_goal = False
        self.nav_stuck = False
        self.nav_failure_code = "NONE"
        self.nav_failure_detail = ""
        self.last_pose_time = rospy.Time(0)
        self.last_map_time = rospy.Time(0)
        self.last_mapping_status_time = rospy.Time(0)
        self.last_nav_health_time = rospy.Time(0)
        self.last_significant_map_change = rospy.Time(0)
        self.map_revision = 0
        self.no_reachable_frontier_cycles = 0
        self.remaining_frontier_count = 0
        self.exploration_state = "STOPPED"
        self.state_reason = "not_started"
        self.complete_published = False
        self.waiting_for_result = False
        self.retry_count = 0
        self.last_goal_time = rospy.Time(0)
        self.current_goal = None
        self.goal_context = "exploration"
        self.failed_goals = []
        self.successful_goals = []
        self.last_successful_goal = None
        self.trap_blacklist = {}
        self.last_recovery_event_id = 0
        self.last_recovery_trigger_id = 0
        self.last_recovery_stuck_pose = None
        self.last_checked_path_metrics = None
        self.observation_goal_cells = []
        self.session_id = 0
        self.goal_id = 0
        self.elevator_active = False
        self.elevator_state = "IDLE"
        self.elevator_reason = "disabled" if not self.elevator_enabled else "idle"
        self.elevator_source_floor = 0
        self.elevator_poses = None
        self.elevator_portal_body_command = (0.0, 0.0)
        self.elevator_goal_result = None
        self.elevator_goal_sent = False
        self.elevator_step_started = rospy.Time(0)
        self.elevator_floor_stable_since = rospy.Time(0)
        self.elevator_operation_token = 0
        self.elevator_operation_name = ""
        self.elevator_operation_result = None
        self.elevator_autonomous_transition = False
        self.next_autonomous_floor = None
        self.visited_floors = set()
        self.completed_floors = set()
        self.autonomous_transition_failures = {}
        self.state_lock = threading.RLock()

        # ========== Action客户端 ==========
        self.move_base_client = actionlib.SimpleActionClient(
            self.move_base_action_name, MoveBaseAction
        )

        # ========== 服务客户端 ==========
        self.make_plan_client = rospy.ServiceProxy(self.make_plan_service, GetPlan)
        self.set_door_client = None
        self.call_elevator_client = None
        self.set_current_floor_client = None
        self.traverse_portal_client = None
        self.cancel_portal_client = None
        if self.elevator_enabled:
            self.set_door_client = rospy.ServiceProxy(
                self.set_door_state_service, SetDoorState
            )
            self.call_elevator_client = rospy.ServiceProxy(
                self.call_elevator_service, CallElevator
            )
            self.set_current_floor_client = rospy.ServiceProxy(
                self.set_current_floor_service, SetCurrentFloor
            )
            self.traverse_portal_client = rospy.ServiceProxy(
                self.traverse_portal_service, TraversePortal
            )
            self.cancel_portal_client = rospy.ServiceProxy(
                self.cancel_portal_service, Trigger
            )

        # ========== 订阅者 ==========
        self.pose_sub = rospy.Subscriber(
            self.pose_topic, PoseWithCovarianceStamped, self.pose_callback
        )
        self.map_sub = rospy.Subscriber(
            self.map_topic, OccupancyGrid, self.map_callback
        )
        self.mapping_status_sub = rospy.Subscriber(
            self.mapping_status_topic, MappingStatus, self.mapping_status_callback
        )
        self.nav_health_sub = rospy.Subscriber(
            self.navigation_health_topic, NavigationHealth, self.nav_health_callback
        )
        self.recovery_sub = rospy.Subscriber(
            self.recovery_event_topic, RecoveryEvent, self.recovery_event_callback
        )

        # ========== 服务 ==========
        self.start_srv = rospy.Service(
            self.start_service, Trigger, self.start_exploration_cb
        )
        self.stop_srv = rospy.Service(
            self.stop_service, Trigger, self.stop_exploration_cb
        )
        self.start_elevator_srv = rospy.Service(
            self.start_elevator_service, Trigger, self.start_elevator_cb
        )
        self.cancel_elevator_srv = rospy.Service(
            self.cancel_elevator_service, Trigger, self.cancel_elevator_cb
        )
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10, latch=True)
        self.complete_pub = rospy.Publisher(self.complete_topic, Bool, queue_size=1, latch=True)
        self.observation_goals_pub = rospy.Publisher(
            "/exploration/observation_goals", PoseArray, queue_size=1
        )
        self.blacklist_pub = rospy.Publisher(
            "/exploration/trap_blacklist", GridCells, queue_size=1, latch=True
        )

        # ========== 主循环 ==========
        self.planner_timer = rospy.Timer(rospy.Duration(0.5), self.planner_loop)

        rospy.loginfo(f"[exploration] Planner started, action: {self.move_base_action_name}")

    def pose_callback(self, msg):
        if msg.header.frame_id != self.map_frame:
            rospy.logwarn_throttle(5, "[exploration] Ignoring pose outside map frame")
            return
        orientation = msg.pose.pose.orientation
        norm = math.sqrt(
            orientation.x ** 2 + orientation.y ** 2
            + orientation.z ** 2 + orientation.w ** 2
        )
        if not math.isfinite(norm) or norm < 1e-6:
            rospy.logwarn_throttle(5, "[exploration] Ignoring pose with invalid orientation")
            return
        self.current_pose = msg.pose.pose
        self.last_pose_time = rospy.Time.now()

    def map_callback(self, msg):
        expected_size = msg.info.width * msg.info.height
        if (msg.header.frame_id != self.map_frame or msg.info.resolution <= 0
                or expected_size == 0 or len(msg.data) != expected_size):
            rospy.logwarn_throttle(5, "[exploration] Ignoring invalid occupancy grid")
            return
        new_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )
        significant = (self.map_data is None or self.map_data.shape != new_data.shape
                       or np.count_nonzero(self.map_data != new_data)
                       >= self.map_change_cell_threshold)
        self.current_map = msg
        self.map_info = msg.info
        self.map_data = new_data
        now = rospy.Time.now()
        self.last_map_time = now
        if significant:
            self.map_revision += 1
            self.last_significant_map_change = now
            self.no_reachable_frontier_cycles = 0
            self._update_blacklist_validity()

    def mapping_status_callback(self, msg):
        self.mapping_ready = msg.ready
        self.mapping_stable = msg.stable
        self.mapping_lost = msg.lost
        self.current_floor = int(msg.current_floor)
        self.last_mapping_status_time = rospy.Time.now()

    def nav_health_callback(self, msg):
        self.nav_ready = msg.ready
        self.nav_has_active_goal = msg.has_active_goal
        self.nav_stuck = msg.stuck
        self.nav_failure_code = msg.failure_code
        self.nav_failure_detail = msg.failure_detail
        self.last_nav_health_time = rospy.Time.now()

    def recovery_event_callback(self, msg):
        if msg.header.frame_id != self.map_frame or self.map_data is None:
            return
        stuck_pose = (msg.stuck_pose.position.x, msg.stuck_pose.position.y)
        if msg.phase == RecoveryEvent.PHASE_TRIGGERED:
            # Triggered is diagnostic only.  A slow A1 gait may recover, so it
            # must not poison all nearby frontier candidates pre-emptively.
            self.last_recovery_stuck_pose = stuck_pose
            self.last_recovery_trigger_id = msg.event_id
            rospy.loginfo(
                "[exploration] recovery triggered at (%.2f, %.2f), "
                "attempt=%d maneuver=%d (diagnostic only)",
                stuck_pose[0], stuck_pose[1], msg.attempt, msg.maneuver,
            )
            return
        if (msg.phase != RecoveryEvent.PHASE_FAILED
                or msg.event_id <= self.last_recovery_event_id):
            return
        self.last_recovery_event_id = msg.event_id
        self.last_recovery_stuck_pose = stuck_pose
        self._remember_trap_region(*stuck_pose)

    def _remember_control_failure_if_needed(self, _event, event_id, stuck_pose):
        """Fallback if a CONTROL_FAILED Action has no FAILED event."""
        with self.state_lock:
            if event_id <= self.last_recovery_event_id:
                return
            self.last_recovery_event_id = event_id
            self._remember_trap_region(*stuck_pose)

    def _clearance_map(self):
        free = (self.map_data >= 0) & (self.map_data < self.free_threshold)
        return cv2.distanceTransform(
            free.astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE
        ).astype(np.float32) * self.map_info.resolution

    def _remember_trap_region(self, world_x, world_y):
        center_x, center_y = self._world_to_map(world_x, world_y)
        if (center_x < 0 or center_x >= self.map_info.width
                or center_y < 0 or center_y >= self.map_info.height):
            return
        clearance = self._clearance_map()
        threshold = self.connectivity_clearance_radius + self.trap_clearance_margin
        radius_cells = int(math.ceil(
            self.trap_blacklist_radius / self.map_info.resolution
        ))
        remembered = 0
        for offset_y in range(-radius_cells, radius_cells + 1):
            for offset_x in range(-radius_cells, radius_cells + 1):
                if (math.hypot(offset_x, offset_y) * self.map_info.resolution
                        > self.trap_blacklist_radius):
                    continue
                cell_x, cell_y = center_x + offset_x, center_y + offset_y
                if (cell_x < 0 or cell_x >= self.map_info.width
                        or cell_y < 0 or cell_y >= self.map_info.height):
                    continue
                if clearance[cell_y, cell_x] <= threshold:
                    self.trap_blacklist[(cell_x, cell_y)] = 0
                    remembered += 1
        self.trap_blacklist[(center_x, center_y)] = 0
        self._publish_blacklist()
        rospy.logwarn(
            "[exploration] recovery trap remembered at (%.2f, %.2f), cells=%d",
            world_x, world_y, remembered,
        )

    def _update_blacklist_validity(self):
        if not getattr(self, "trap_blacklist", {}) or self.map_data is None:
            return
        clearance = self._clearance_map()
        threshold = self.connectivity_clearance_radius + self.trap_clearance_margin
        updated = {}
        for cell, safe_revisions in self.trap_blacklist.items():
            cell_x, cell_y = cell
            if (cell_x >= self.map_info.width or cell_y >= self.map_info.height):
                continue
            safe_revisions = (
                safe_revisions + 1
                if clearance[cell_y, cell_x] > threshold else 0
            )
            if safe_revisions < self.blacklist_clear_revisions:
                updated[cell] = safe_revisions
        self.trap_blacklist = updated
        self._publish_blacklist()

    def _publish_blacklist(self):
        publisher = getattr(self, "blacklist_pub", None)
        if publisher is None or self.map_info is None:
            return
        message = GridCells()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.map_frame
        message.cell_width = self.map_info.resolution
        message.cell_height = self.map_info.resolution
        for cell_x, cell_y in self.trap_blacklist:
            world_x, world_y = self._map_to_world(cell_x, cell_y)
            message.cells.append(Point(x=world_x, y=world_y, z=0.03))
        publisher.publish(message)

    def _set_state(self, state, reason):
        self.exploration_state = state
        self.state_reason = reason
        self._publish_status()

    def _known_grid_ratio(self):
        """Known-cell ratio inside the observed bounding box, not fixed map bounds."""
        if self.map_data is None:
            return None
        known = self.map_data != -1
        cells = np.argwhere(known)
        if not len(cells):
            return 0.0
        y0, x0 = cells.min(axis=0)
        y1, x1 = cells.max(axis=0)
        observed_box = known[y0:y1 + 1, x0:x1 + 1]
        return float(np.count_nonzero(observed_box)) / float(observed_box.size)

    def _publish_status(self):
        coverage = self._known_grid_ratio()
        payload = {
            "state": self.exploration_state,
            "reason": self.state_reason,
            "complete": self.complete_published,
            "remaining_frontier_count": self.remaining_frontier_count,
            "known_grid_ratio": coverage,
            "known_grid_ratio_scope": "observed_known_bounding_box",
            "map_revision": self.map_revision,
            "has_active_goal": bool(self.waiting_for_result or self.nav_has_active_goal),
            "blacklisted_cell_count": len(getattr(self, "trap_blacklist", {})),
            "observation_goal_count": len(getattr(self, "observation_goal_cells", [])),
            "elevator_enabled": bool(getattr(self, "elevator_enabled", False)),
            "elevator_active": bool(getattr(self, "elevator_active", False)),
            "elevator_state": getattr(self, "elevator_state", "IDLE"),
            "elevator_reason": getattr(self, "elevator_reason", "idle"),
            "current_floor": int(getattr(self, "current_floor", 0)),
            "autonomous_floor_change_enabled": bool(
                getattr(self, "elevator_enabled", False)
                and getattr(self, "elevator_autonomous_when_floor_complete", False)
            ),
            "served_floors": list(getattr(self, "elevator_served_floors", [])),
            "visited_floors": sorted(getattr(self, "visited_floors", set())),
            "completed_floors": sorted(getattr(self, "completed_floors", set())),
            "next_autonomous_floor": getattr(self, "next_autonomous_floor", None),
            "autonomous_transition_failures": {
                f"{source}->{target}": count
                for (source, target), count in sorted(
                    getattr(self, "autonomous_transition_failures", {}).items()
                )
            },
        }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _inputs_health(self, now):
        if self.current_map is None:
            return False, "map_uninitialized"
        if self.current_pose is None:
            return False, "pose_uninitialized"
        stamps = (self.last_map_time, self.last_pose_time,
                  self.last_mapping_status_time, self.last_nav_health_time)
        if any(stamp == rospy.Time(0) or (now - stamp).to_sec() > self.input_timeout
               for stamp in stamps):
            return False, "input_stale"
        if self.mapping_lost:
            return False, "localization_lost"
        if not self.mapping_ready or not self.mapping_stable:
            return False, "mapping_not_ready"
        if not self.nav_ready:
            return False, "navigation_not_ready"
        return True, "healthy"

    def _world_to_map(self, x, y):
        origin = self.map_info.origin
        yaw = self._yaw_from_quaternion(origin.orientation)
        dx = x - origin.position.x
        dy = y - origin.position.y
        mx = int(math.floor((math.cos(yaw) * dx + math.sin(yaw) * dy)
                            / self.map_info.resolution))
        my = int(math.floor((-math.sin(yaw) * dx + math.cos(yaw) * dy)
                            / self.map_info.resolution))
        return mx, my

    @staticmethod
    def _yaw_from_quaternion(orientation):
        return math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )

    def _map_to_world(self, mx, my):
        origin = self.map_info.origin
        yaw = self._yaw_from_quaternion(origin.orientation)
        local_x = (mx + 0.5) * self.map_info.resolution
        local_y = (my + 0.5) * self.map_info.resolution
        return (
            origin.position.x + math.cos(yaw) * local_x - math.sin(yaw) * local_y,
            origin.position.y + math.sin(yaw) * local_x + math.cos(yaw) * local_y,
        )

    def _is_free(self, mx, my):
        if self.map_data is None:
            return False
        if mx < 0 or mx >= self.map_info.width or my < 0 or my >= self.map_info.height:
            return False
        value = int(self.map_data[my, mx])
        return 0 <= value < self.free_threshold

    def _check_path(self, start_x, start_y, goal_x, goal_y):
        """调用make_plan检查路径是否存在"""
        self.last_checked_path_metrics = None
        try:
            rospy.wait_for_service(
                self.make_plan_service, timeout=self.dependency_check_timeout
            )
            start = PoseStamped()
            start.header.frame_id = self.map_frame
            start.header.stamp = rospy.Time.now()
            start.pose.position.x = start_x
            start.pose.position.y = start_y
            start.pose.orientation.w = 1.0

            goal = PoseStamped()
            goal.header.frame_id = self.map_frame
            goal.header.stamp = start.header.stamp
            goal.pose.position.x = goal_x
            goal.pose.position.y = goal_y
            goal.pose.orientation.w = 1.0

            resp = self.make_plan_client(start, goal, self.plan_tolerance)
            if not resp.plan.poses:
                return "unreachable"
            points = [
                (pose.pose.position.x, pose.pose.position.y)
                for pose in resp.plan.poses
            ]
            path_length = sum(
                math.hypot(
                    points[index][0] - points[index - 1][0],
                    points[index][1] - points[index - 1][1],
                )
                for index in range(1, len(points))
            )
            clearance = self._clearance_map()
            min_clearance = float("inf")
            escaped_blacklist = False
            blacklist = getattr(self, "trap_blacklist", {})
            for index, (point_x, point_y) in enumerate(points):
                cell_x, cell_y = self._world_to_map(point_x, point_y)
                if (cell_x < 0 or cell_x >= self.map_info.width
                        or cell_y < 0 or cell_y >= self.map_info.height):
                    return "unreachable"
                inside_blacklist = (cell_x, cell_y) in blacklist
                if index == 0:
                    escaped_blacklist = not inside_blacklist
                elif escaped_blacklist and inside_blacklist:
                    return "unreachable"
                elif not inside_blacklist:
                    escaped_blacklist = True
                min_clearance = min(
                    min_clearance, float(clearance[cell_y, cell_x])
                )
            self.last_checked_path_metrics = {
                "goal": (goal_x, goal_y),
                "path_length": path_length,
                "min_clearance": min_clearance,
                "points": points,
            }
            return "reachable"
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logwarn_throttle(5, f"[exploration] make_plan failed: {e}")
            return "unavailable"

    def _goal_is_cooled_down(self, goal_x, goal_y):
        now = rospy.Time.now()
        self.failed_goals = [
            failure for failure in getattr(self, "failed_goals", [])
            if (now - failure[2]).to_sec()
            < getattr(self, "failed_goal_cooldown", 0.0)
        ]
        success_goal_cooldown = getattr(self, "success_goal_cooldown", 0.0)
        success_goal_clear_revisions = getattr(
            self, "success_goal_clear_revisions", 1
        )
        map_revision = getattr(self, "map_revision", 0)
        self.successful_goals = [
            success for success in getattr(self, "successful_goals", [])
            if ((now - success[3]).to_sec() < success_goal_cooldown
                or map_revision - success[2] < success_goal_clear_revisions)
        ]
        failed_nearby = any(
            math.hypot(goal_x - failed_x, goal_y - failed_y)
            < self.failed_goal_radius
            for failed_x, failed_y, _ in self.failed_goals
        )
        successful_nearby = any(
            math.hypot(goal_x - success_x, goal_y - success_y)
            < getattr(self, "success_goal_radius", 0.0)
            for success_x, success_y, _, _ in self.successful_goals
        )
        return failed_nearby or successful_nearby

    def _remember_failed_goal(self):
        if self.current_goal is not None:
            self.failed_goals.append(
                (self.current_goal[0], self.current_goal[1], rospy.Time.now())
            )

    def _remember_successful_goal(self):
        if self.current_goal is None:
            return
        goal = tuple(self.current_goal)
        self.last_successful_goal = goal
        self.successful_goals.append(
            (goal[0], goal[1], self.map_revision, rospy.Time.now())
        )

    def _frontier_mask(self):
        """返回与未知四邻接的已知自由栅格。"""
        free = (self.map_data >= 0) & (self.map_data < self.free_threshold)
        unknown = self.map_data == -1
        adjacent_unknown = np.zeros_like(unknown, dtype=bool)
        adjacent_unknown[1:, :] |= unknown[:-1, :]
        adjacent_unknown[:-1, :] |= unknown[1:, :]
        adjacent_unknown[:, 1:] |= unknown[:, :-1]
        adjacent_unknown[:, :-1] |= unknown[:, 1:]
        return free & adjacent_unknown

    def _frontier_representatives(self, frontier=None):
        """8邻域聚类，并为每个有效前沿选择靠近质心的自由栅格。"""
        representatives = []
        for cluster in self._frontier_clusters(frontier):
            centroid_x = sum(cell[0] for cell in cluster) / len(cluster)
            centroid_y = sum(cell[1] for cell in cluster) / len(cluster)
            representatives.append(min(
                cluster,
                key=lambda cell: ((cell[0] - centroid_x) ** 2
                                  + (cell[1] - centroid_y) ** 2),
            ))
        return representatives

    def _frontier_clusters(self, frontier=None):
        """Return valid 8-connected frontier clusters."""
        if frontier is None:
            frontier = self._frontier_mask()
        visited = np.zeros_like(frontier, dtype=bool)
        min_cells = max(
            1, int(math.ceil(self.min_frontier_length / self.map_info.resolution))
        )
        clusters = []

        for my, mx in np.argwhere(frontier):
            if visited[my, mx]:
                continue
            queue = deque([(mx, my)])
            visited[my, mx] = True
            cluster = []
            while queue:
                cell_x, cell_y = queue.popleft()
                cluster.append((cell_x, cell_y))
                for offset_y in (-1, 0, 1):
                    for offset_x in (-1, 0, 1):
                        if offset_x == 0 and offset_y == 0:
                            continue
                        next_x = cell_x + offset_x
                        next_y = cell_y + offset_y
                        if (next_x < 0 or next_x >= self.map_info.width
                                or next_y < 0 or next_y >= self.map_info.height
                                or visited[next_y, next_x]
                                or not frontier[next_y, next_x]):
                            continue
                        visited[next_y, next_x] = True
                        queue.append((next_x, next_y))

            if len(cluster) < min_cells:
                continue
            clusters.append(cluster)
        return clusters

    def _observation_goal_for_cluster(self, cluster, reachable, clearance):
        cluster_mask = np.zeros_like(reachable, dtype=np.uint8)
        for cell_x, cell_y in cluster:
            cluster_mask[cell_y, cell_x] = 1
        distance_to_frontier = cv2.distanceTransform(
            # DIST_MASK_PRECISE is required here: the 5x5 chamfer mask can
            # overestimate diagonal distance by enough to admit a goal only
            # 0.35 m from the frontier despite a configured 0.40 m minimum.
            (cluster_mask == 0).astype(np.uint8),
            cv2.DIST_L2,
            cv2.DIST_MASK_PRECISE,
        ) * self.map_info.resolution
        required_clearance = (
            self.connectivity_clearance_radius + self.goal_clearance_margin
        )
        candidate_mask = (
            reachable
            & (distance_to_frontier >= self.observation_min_distance - 1e-6)
            & (distance_to_frontier <= self.observation_max_distance + 1e-6)
            & (clearance >= required_clearance - 1e-6)
        )
        for cell in getattr(self, "trap_blacklist", {}):
            if 0 <= cell[0] < self.map_info.width and 0 <= cell[1] < self.map_info.height:
                candidate_mask[cell[1], cell[0]] = False
        candidates = np.argwhere(candidate_mask)
        if not len(candidates):
            return None
        centroid_x = sum(cell[0] for cell in cluster) / len(cluster)
        centroid_y = sum(cell[1] for cell in cluster) / len(cluster)
        best_y, best_x = min(
            candidates,
            key=lambda value: (
                abs(float(distance_to_frontier[value[0], value[1]])
                    - self.observation_target_distance),
                -float(clearance[value[0], value[1]]),
                (float(value[1]) - centroid_x) ** 2
                + (float(value[0]) - centroid_y) ** 2,
            ),
        )
        goal_x, goal_y = self._map_to_world(int(best_x), int(best_y))
        frontier_x, frontier_y = self._map_to_world(
            int(round(centroid_x)), int(round(centroid_y))
        )
        yaw = math.atan2(frontier_y - goal_y, frontier_x - goal_x)
        return {
            "x": goal_x,
            "y": goal_y,
            "yaw": yaw,
            "gain": len(cluster) * self.map_info.resolution,
            "clearance": float(clearance[best_y, best_x]),
            "cell": (int(best_x), int(best_y)),
        }

    def _observation_goals(self, frontier, reachable):
        clearance = self._clearance_map()
        goals = []
        for cluster in self._frontier_clusters(frontier & reachable):
            goal = self._observation_goal_for_cluster(cluster, reachable, clearance)
            if goal is not None:
                goals.append(goal)
        self.observation_goal_cells = [goal["cell"] for goal in goals]
        self._publish_observation_goals(goals)
        return goals

    def _publish_observation_goals(self, goals):
        publisher = getattr(self, "observation_goals_pub", None)
        if publisher is None:
            return
        message = PoseArray()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.map_frame
        for goal in goals:
            pose = Pose()
            pose.position.x = goal["x"]
            pose.position.y = goal["y"]
            pose.orientation.z = math.sin(0.5 * goal["yaw"])
            pose.orientation.w = math.cos(0.5 * goal["yaw"])
            message.poses.append(pose)
        publisher.publish(message)

    def _connectivity_traversable_mask(self):
        """Build the statically inflated free mask used for frontier reachability."""
        free = (self.map_data >= 0) & (self.map_data < self.free_threshold)
        occupied = (
            self.map_data >= self.connectivity_occupied_threshold
        ).astype(np.uint8)
        radius_cells = int(math.ceil(
            self.connectivity_clearance_radius / self.map_info.resolution
        ))
        yy, xx = np.ogrid[
            -radius_cells:radius_cells + 1,
            -radius_cells:radius_cells + 1,
        ]
        kernel = (
            (xx * xx + yy * yy) * self.map_info.resolution ** 2
            <= self.connectivity_clearance_radius ** 2 + 1e-9
        ).astype(np.uint8)
        inflated = cv2.dilate(occupied, kernel, iterations=1) != 0
        return free & ~inflated

    def _reachable_free_mask(self):
        """Return the 4-connected inflated-free component containing the robot."""
        traversable = self._connectivity_traversable_mask()
        start_x, start_y = self._world_to_map(
            self.current_pose.position.x, self.current_pose.position.y
        )
        if (start_x < 0 or start_x >= self.map_info.width
                or start_y < 0 or start_y >= self.map_info.height):
            return np.zeros_like(traversable, dtype=bool)

        seed = None
        if traversable[start_y, start_x]:
            seed = (start_x, start_y)
        else:
            radius_cells = int(math.ceil(
                self.connectivity_clearance_radius / self.map_info.resolution
            ))
            nearest_distance = None
            for offset_y in range(-radius_cells, radius_cells + 1):
                for offset_x in range(-radius_cells, radius_cells + 1):
                    distance = math.hypot(offset_x, offset_y) * self.map_info.resolution
                    if distance > self.connectivity_clearance_radius + 1e-9:
                        continue
                    cell_x = start_x + offset_x
                    cell_y = start_y + offset_y
                    if (cell_x < 0 or cell_x >= self.map_info.width
                            or cell_y < 0 or cell_y >= self.map_info.height
                            or not traversable[cell_y, cell_x]):
                        continue
                    if nearest_distance is None or distance < nearest_distance:
                        nearest_distance = distance
                        seed = (cell_x, cell_y)

        reachable = np.zeros_like(traversable, dtype=bool)
        if seed is None:
            return reachable

        queue = deque([seed])
        reachable[seed[1], seed[0]] = True
        while queue:
            cell_x, cell_y = queue.popleft()
            for next_x, next_y in (
                (cell_x - 1, cell_y), (cell_x + 1, cell_y),
                (cell_x, cell_y - 1), (cell_x, cell_y + 1),
            ):
                if (next_x < 0 or next_x >= self.map_info.width
                        or next_y < 0 or next_y >= self.map_info.height
                        or reachable[next_y, next_x]
                        or not traversable[next_y, next_x]):
                    continue
                reachable[next_y, next_x] = True
                queue.append((next_x, next_y))
        return reachable

    def _select_goal(self):
        """Return (goal, reason); dependency failure is not no-frontier."""
        if self.current_pose is None or self.map_data is None:
            return None, "input_missing"

        cx = self.current_pose.position.x
        cy = self.current_pose.position.y
        frontier = self._frontier_mask()
        all_representatives = self._frontier_representatives(frontier)
        self.remaining_frontier_count = len(all_representatives)
        if not all_representatives:
            return None, "no_frontier"

        reachable = self._reachable_free_mask()
        reachable_frontier = frontier & reachable
        use_observation_goals = hasattr(self, "observation_target_distance")
        if use_observation_goals:
            candidates = self._observation_goals(frontier, reachable)
        else:
            representatives = self._frontier_representatives(reachable_frontier)
            candidates = [
                {"x": goal[0], "y": goal[1], "yaw": 0.0,
                 "gain": 0.0, "clearance": 0.0}
                for goal in (self._map_to_world(mx, my) for mx, my in representatives)
            ]
        candidates.sort(
            key=lambda goal: math.hypot(goal["x"] - cx, goal["y"] - cy)
        )

        reachable_candidates = []
        service_unavailable = False
        for candidate in candidates[:self.max_frontier_candidates]:
            gx, gy = candidate["x"], candidate["y"]
            if self._goal_is_cooled_down(gx, gy):
                continue
            if (self._world_to_map(gx, gy) in getattr(self, "trap_blacklist", {})):
                continue
            path_state = self._check_path(cx, cy, gx, gy)
            if path_state == "reachable":
                metrics = getattr(self, "last_checked_path_metrics", None)
                path_length = (
                    metrics["path_length"]
                    if metrics is not None else math.hypot(gx - cx, gy - cy)
                )
                path_clearance = (
                    metrics["min_clearance"]
                    if metrics is not None else candidate["clearance"]
                )
                score = (
                    getattr(self, "path_length_weight", 1.0) * path_length
                    - getattr(self, "frontier_gain_weight", 0.0) * candidate["gain"]
                    + getattr(self, "path_clearance_weight", 0.0)
                    / max(path_clearance, 0.01)
                )
                reachable_candidates.append((score, candidate))
            if path_state == "unavailable":
                service_unavailable = True

        if reachable_candidates:
            _, selected = min(reachable_candidates, key=lambda item: item[0])
            goal = (
                (selected["x"], selected["y"], selected["yaw"])
                if use_observation_goals
                else (selected["x"], selected["y"])
            )
            return goal, "reachable_frontier"
        if service_unavailable:
            return None, "navigation_service_unavailable"

        return None, "all_frontiers_unreachable_or_blacklisted"

    def _send_goal(self, gx, gy, yaw=0.0, context="exploration"):
        """发送导航目标"""
        if not self.move_base_client.wait_for_server(
                rospy.Duration(self.dependency_check_timeout)):
            rospy.logwarn_throttle(5, "[exploration] move_base action server unavailable")
            return False

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = gx
        goal.target_pose.pose.position.y = gy
        goal.target_pose.pose.orientation.z = math.sin(0.5 * yaw)
        goal.target_pose.pose.orientation.w = math.cos(0.5 * yaw)

        self.goal_id += 1
        self.last_recovery_stuck_pose = None
        session_id = self.session_id
        goal_id = self.goal_id
        self.move_base_client.send_goal(
            goal,
            done_cb=lambda state, result: self.goal_done_cb(
                session_id, goal_id, state, result
            ),
        )
        self.waiting_for_result = True
        self.current_goal = (gx, gy, yaw)
        self.goal_context = context
        self.last_goal_time = rospy.Time.now()
        rospy.loginfo(
            "[exploration] Sent %s goal: (%.2f, %.2f, %.1fdeg)",
            context, gx, gy, math.degrees(yaw),
        )
        return True

    def goal_done_cb(self, session_id, goal_id, state, result):
        """目标完成回调"""
        with self.state_lock:
            if (session_id != self.session_id or goal_id != self.goal_id
                    or not self.exploring):
                return
            self.waiting_for_result = False
            status_text = ""
            try:
                status_text = self.move_base_client.get_goal_status_text()
            except AttributeError:
                pass
            if self.goal_context.startswith("elevator:"):
                succeeded = state == actionlib.GoalStatus.SUCCEEDED
                self.elevator_goal_result = (
                    succeeded,
                    status_text or ("goal_succeeded" if succeeded else f"state_{state}"),
                )
                rospy.loginfo(
                    "[exploration] Elevator navigation %s: %s",
                    "succeeded" if succeeded else "failed",
                    self.elevator_goal_result[1],
                )
                self.current_goal = None
                self.goal_context = "exploration"
                return
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("[exploration] Goal succeeded")
                self._remember_successful_goal()
                self.retry_count = 0
            else:
                rospy.loginfo(
                    "[exploration] Goal failed: state=%d action_text=%s "
                    "nav_failure=%s detail=%s",
                    state,
                    status_text or "<empty>",
                    getattr(self, "nav_failure_code", "NONE"),
                    getattr(self, "nav_failure_detail", ""),
                )
                if (getattr(self, "nav_failure_code", "") == "CONTROL_FAILED"
                        and self.last_recovery_stuck_pose is not None):
                    rospy.Timer(
                        rospy.Duration(0.25),
                        lambda event,
                               recovery_id=self.last_recovery_trigger_id,
                               stuck_pose=self.last_recovery_stuck_pose:
                            self._remember_control_failure_if_needed(
                                event, recovery_id, stuck_pose
                            ),
                        oneshot=True,
                    )
                self._remember_failed_goal()
                self.retry_count += 1
            self.current_goal = None

    def _transition_elevator(self, state, reason):
        self.elevator_state = state
        self.elevator_reason = reason
        self.elevator_goal_result = None
        self.elevator_goal_sent = False
        self.elevator_operation_name = ""
        self.elevator_operation_result = None
        self.elevator_step_started = rospy.Time.now()
        self.elevator_floor_stable_since = rospy.Time(0)
        self._set_state(state, reason)
        rospy.loginfo("[exploration] Elevator state -> %s: %s", state, reason)

    def _start_elevator_operation(self, name, operation):
        self.elevator_operation_token += 1
        token = self.elevator_operation_token
        self.elevator_operation_name = name
        self.elevator_operation_result = None
        self.elevator_step_started = rospy.Time.now()

        def worker():
            try:
                success, message = operation()
            except (rospy.ROSException, rospy.ServiceException) as error:
                success, message = False, str(error)
            except Exception as error:
                success, message = False, f"unexpected service error: {error}"
            with self.state_lock:
                if token != self.elevator_operation_token:
                    return
                self.elevator_operation_result = (bool(success), str(message))

        threading.Thread(target=worker, daemon=True).start()

    def _set_elevator_door(self, floor_id, open_state):
        rospy.wait_for_service(
            self.set_door_state_service, timeout=self.elevator_service_timeout
        )
        response = self.set_door_client(
            door_id=f"{self.elevator_door_prefix}{floor_id}", open=open_state
        )
        return response.accepted, response.message or response.state

    def _call_elevator(self, floor_id):
        rospy.wait_for_service(
            self.call_elevator_service, timeout=self.elevator_service_timeout
        )
        response = self.call_elevator_client(
            elevator_id=self.elevator_id,
            target_floor=floor_id,
            open_doors=False,
        )
        if response.accepted and int(response.current_floor) != int(floor_id):
            return False, (
                f"elevator reported floor {response.current_floor}, expected {floor_id}"
            )
        return response.accepted, response.message or response.state

    def _set_localization_floor(self, floor_id):
        rospy.wait_for_service(
            self.set_current_floor_service, timeout=self.elevator_service_timeout
        )
        response = self.set_current_floor_client(
            floor_id=floor_id,
            reason=f"elevator_{self.elevator_source_floor}_to_{floor_id}",
        )
        return response.accepted, response.message

    def _traverse_elevator_portal(self, direction):
        rospy.wait_for_service(
            self.traverse_portal_service, timeout=self.elevator_service_timeout
        )
        body_x, body_y = self.elevator_portal_body_command
        response = self.traverse_portal_client(
            linear_x=direction * body_x,
            linear_y=direction * body_y,
            angular_z=0.0,
            duration=self.elevator_portal_duration,
        )
        return response.accepted, response.message

    def _cancel_portal_traversal(self):
        if self.cancel_portal_client is None:
            return
        try:
            self.cancel_portal_client()
        except (rospy.ROSException, rospy.ServiceException):
            pass

    def _fail_elevator(self, reason):
        rospy.logerr("[exploration] Elevator flow failed: %s", reason)
        if self.elevator_autonomous_transition:
            transition = (self.elevator_source_floor, self.elevator_target_floor)
            self.autonomous_transition_failures[transition] = (
                self.autonomous_transition_failures.get(transition, 0) + 1
            )
        self._cancel_portal_traversal()
        self.goal_id += 1
        self.move_base_client.cancel_all_goals()
        self.waiting_for_result = False
        self.current_goal = None
        self.goal_context = "exploration"
        self.elevator_operation_token += 1
        self.elevator_active = False
        self.elevator_autonomous_transition = False
        self.next_autonomous_floor = None
        self.elevator_state = "ELEVATOR_ERROR"
        self.elevator_reason = reason
        self.last_goal_time = rospy.Time.now()
        self._set_state("RECOVERING", f"elevator_failed:{reason}")

    def _complete_elevator(self):
        rospy.loginfo(
            "[exploration] Elevator flow completed: floor %d -> %d",
            self.elevator_source_floor, self.elevator_target_floor,
        )
        self.elevator_active = False
        self.elevator_autonomous_transition = False
        self.next_autonomous_floor = None
        self.elevator_state = "ELEVATOR_COMPLETE"
        self.elevator_reason = "completed"
        now = rospy.Time.now()
        self.visited_floors.add(self.elevator_target_floor)
        self.no_reachable_frontier_cycles = 0
        self.retry_count = 0
        self.failed_goals = []
        self.successful_goals = []
        self.last_successful_goal = None
        self.trap_blacklist = {}
        self.observation_goal_cells = []
        self.last_significant_map_change = now
        self.last_goal_time = now
        self._set_state("WAITING", "elevator_complete")

    def _run_elevator_navigation_step(self, pose_name, next_state):
        if self.elevator_goal_result is not None:
            success, message = self.elevator_goal_result
            self.elevator_goal_result = None
            if not success:
                self._fail_elevator(f"{self.elevator_state}:{message}")
                return
            self._transition_elevator(next_state, f"{pose_name}_reached")
            return

        if self.elevator_goal_sent:
            if ((rospy.Time.now() - self.elevator_step_started).to_sec()
                    > self.elevator_nav_timeout):
                self.goal_id += 1
                self.move_base_client.cancel_goal()
                self.waiting_for_result = False
                self.current_goal = None
                self._fail_elevator(f"{self.elevator_state}:navigation_timeout")
            return

        pose = self.elevator_poses[pose_name]
        if not self._send_goal(
                pose[0], pose[1], pose[2], context=f"elevator:{pose_name}"):
            self._fail_elevator(f"{self.elevator_state}:move_base_unavailable")
            return
        self.elevator_goal_sent = True
        self.elevator_step_started = rospy.Time.now()

    def _run_elevator_operation_step(self, name, operation, next_state):
        if not self.elevator_operation_name:
            self._start_elevator_operation(name, operation)
            return
        if self.elevator_operation_result is not None:
            success, message = self.elevator_operation_result
            self.elevator_operation_result = None
            if not success:
                self._fail_elevator(f"{name}:{message}")
                return
            self._transition_elevator(next_state, f"{name}_completed")
            return
        if ((rospy.Time.now() - self.elevator_step_started).to_sec()
                > self.elevator_operation_timeout):
            self.elevator_operation_token += 1
            self._fail_elevator(f"{name}:operation_timeout")

    def _elevator_loop(self, now):
        state = self.elevator_state
        if state == "NAVIGATE_ELEVATOR_LOBBY":
            self._run_elevator_navigation_step("lobby", "CALL_ELEVATOR_CURRENT_FLOOR")
        elif state == "CALL_ELEVATOR_CURRENT_FLOOR":
            self._run_elevator_operation_step(
                "call_current_floor",
                lambda: self._call_elevator(self.elevator_source_floor),
                "OPEN_ELEVATOR_CURRENT_DOOR",
            )
        elif state == "OPEN_ELEVATOR_CURRENT_DOOR":
            self._run_elevator_operation_step(
                "open_current_door",
                lambda: self._set_elevator_door(self.elevator_source_floor, True),
                "ENTER_ELEVATOR",
            )
        elif state == "ENTER_ELEVATOR":
            if self.elevator_portal_traversal_enabled:
                self._run_elevator_operation_step(
                    "enter_elevator",
                    lambda: self._traverse_elevator_portal(1.0),
                    "CLOSE_ELEVATOR_CURRENT_DOOR",
                )
            else:
                self._run_elevator_navigation_step(
                    "cabin", "CLOSE_ELEVATOR_CURRENT_DOOR"
                )
        elif state == "CLOSE_ELEVATOR_CURRENT_DOOR":
            self._run_elevator_operation_step(
                "close_current_door",
                lambda: self._set_elevator_door(self.elevator_source_floor, False),
                "CHANGE_ELEVATOR_FLOOR",
            )
        elif state == "CHANGE_ELEVATOR_FLOOR":
            self._run_elevator_operation_step(
                "move_elevator",
                lambda: self._call_elevator(self.elevator_target_floor),
                "SET_LOCALIZATION_TARGET_FLOOR",
            )
        elif state == "SET_LOCALIZATION_TARGET_FLOOR":
            self._run_elevator_operation_step(
                "set_localization_floor",
                lambda: self._set_localization_floor(self.elevator_target_floor),
                "OPEN_ELEVATOR_TARGET_DOOR",
            )
        elif state == "OPEN_ELEVATOR_TARGET_DOOR":
            self._run_elevator_operation_step(
                "open_target_door",
                lambda: self._set_elevator_door(self.elevator_target_floor, True),
                "WAIT_ELEVATOR_TARGET_MAP",
            )
        elif state == "WAIT_ELEVATOR_TARGET_MAP":
            if self.current_floor == self.elevator_target_floor and self.mapping_stable:
                if self.elevator_floor_stable_since == rospy.Time(0):
                    self.elevator_floor_stable_since = now
                elif ((now - self.elevator_floor_stable_since).to_sec()
                      >= self.elevator_floor_stable_time):
                    self._transition_elevator("EXIT_ELEVATOR", "target_floor_stable")
            else:
                self.elevator_floor_stable_since = rospy.Time(0)
            if ((now - self.elevator_step_started).to_sec()
                    > self.elevator_floor_timeout):
                self._fail_elevator("target_floor_mapping_timeout")
        elif state == "EXIT_ELEVATOR":
            next_state = (
                "CLOSE_ELEVATOR_TARGET_DOOR"
                if self.elevator_close_target_door else "ELEVATOR_COMPLETE"
            )
            if self.elevator_portal_traversal_enabled:
                self._run_elevator_operation_step(
                    "exit_elevator",
                    lambda: self._traverse_elevator_portal(-1.0),
                    next_state,
                )
            else:
                self._run_elevator_navigation_step("exit", next_state)
        elif state == "CLOSE_ELEVATOR_TARGET_DOOR":
            self._run_elevator_operation_step(
                "close_target_door",
                lambda: self._set_elevator_door(self.elevator_target_floor, False),
                "ELEVATOR_COMPLETE",
            )
        elif state == "ELEVATOR_COMPLETE":
            self._complete_elevator()
        else:
            self._fail_elevator(f"invalid_state:{state}")

    def _start_elevator_locked(self, target_floor, portal_pose, autonomous=False):
        if not self.elevator_enabled:
            return False, "Elevator flow disabled"
        if not self.exploring:
            return False, "Exploration must be running"
        if self.elevator_active:
            return True, f"Elevator already active: {self.elevator_state}"
        if target_floor == self.current_floor:
            return False, "Target floor equals current floor"
        if portal_pose is None:
            return False, "No elevator portal pose"
        if self.current_pose is None:
            return False, "Current pose unavailable"

        self.goal_id += 1
        self.move_base_client.cancel_all_goals()
        self.waiting_for_result = False
        self.current_goal = None
        self.goal_context = "exploration"
        self.elevator_source_floor = self.current_floor
        self.elevator_target_floor = int(target_floor)
        self.elevator_autonomous_transition = bool(autonomous)
        self.next_autonomous_floor = int(target_floor) if autonomous else None
        self.elevator_poses = self._derive_elevator_poses(
            portal_pose,
            self.elevator_lobby_offset,
            self.elevator_cabin_distance,
            self.elevator_exit_distance,
        )
        robot_yaw = self._yaw_from_quaternion(self.current_pose.orientation)
        relative_yaw = portal_pose[2] - robot_yaw
        self.elevator_portal_body_command = (
            self.elevator_portal_speed * math.cos(relative_yaw),
            self.elevator_portal_speed * math.sin(relative_yaw),
        )
        self.elevator_operation_token += 1
        self.elevator_active = True
        portal_distance = math.hypot(
            self.current_pose.position.x - portal_pose[0],
            self.current_pose.position.y - portal_pose[1],
        )
        if portal_distance <= self.elevator_portal_near_distance:
            self._transition_elevator(
                "CALL_ELEVATOR_CURRENT_FLOOR",
                f"already_near_portal:{portal_distance:.2f}m",
            )
        else:
            self._transition_elevator(
                "NAVIGATE_ELEVATOR_LOBBY",
                f"floor_{self.elevator_source_floor}_to_{self.elevator_target_floor}",
            )
        return True, (f"Elevator flow started: floor {self.elevator_source_floor} "
                      f"-> {self.elevator_target_floor}")

    def start_elevator_cb(self, req):
        del req
        with self.state_lock:
            target_floor = int(rospy.get_param(
                "~elevator/target_floor", self.elevator_target_floor
            ))
            configured_portal = self._parse_optional_pose_param(
                rospy.get_param(
                    "~elevator/portal_pose",
                    list(self.elevator_portal_pose) if self.elevator_portal_pose else [],
                )
            )
            portal_pose = configured_portal or self.last_successful_goal
            success, message = self._start_elevator_locked(
                target_floor, portal_pose, autonomous=False
            )
            if not success and message == "No elevator portal pose":
                message = ("No elevator portal pose; configure elevator/portal_pose "
                           "or first reach the portal frontier")
            return TriggerResponse(success=success, message=message)

    def _select_next_autonomous_floor(self):
        candidates = []
        for floor_id in self.elevator_served_floors:
            transition = (self.current_floor, floor_id)
            if (floor_id == self.current_floor
                    or floor_id in self.completed_floors
                    or self.autonomous_transition_failures.get(transition, 0)
                    >= self.elevator_max_autonomous_transition_failures):
                continue
            candidates.append(floor_id)
        if not candidates:
            return None
        return min(candidates, key=lambda floor_id: (
            abs(floor_id - self.current_floor), floor_id
        ))

    def _publish_global_complete(self, reason):
        if not self.complete_published:
            self.complete_published = True
            self.complete_pub.publish(Bool(data=True))
            rospy.loginfo(
                "[exploration] Exploration converged on floors %s",
                sorted(self.completed_floors),
            )
        self._set_state("COMPLETE", reason)

    def _handle_converged_floor(self, selection_reason):
        self.visited_floors.add(self.current_floor)
        self.completed_floors.add(self.current_floor)
        if not (self.elevator_enabled
                and self.elevator_autonomous_when_floor_complete):
            self._publish_global_complete(selection_reason)
            return
        if set(self.elevator_served_floors).issubset(self.completed_floors):
            self._publish_global_complete("all_served_floors_complete")
            return

        target_floor = self._select_next_autonomous_floor()
        self.next_autonomous_floor = target_floor
        if target_floor is None:
            self._set_state("FAILED", "autonomous_floor_transition_unavailable")
            return

        configured_portal = self._parse_optional_pose_param(rospy.get_param(
            "~elevator/portal_pose",
            list(self.elevator_portal_pose) if self.elevator_portal_pose else [],
        ))
        if configured_portal is None:
            self._set_state("FAILED", "autonomous_elevator_portal_not_configured")
            return
        self.elevator_portal_pose = configured_portal
        success, message = self._start_elevator_locked(
            target_floor, configured_portal, autonomous=True
        )
        if not success:
            self.next_autonomous_floor = None
            self._set_state("FAILED", f"autonomous_elevator_start_failed:{message}")

    def _cancel_elevator_locked(self, reason):
        if not self.elevator_active:
            return False
        self.goal_id += 1
        self.move_base_client.cancel_all_goals()
        self._cancel_portal_traversal()
        self.waiting_for_result = False
        self.current_goal = None
        self.goal_context = "exploration"
        self.elevator_operation_token += 1
        self.elevator_active = False
        self.elevator_autonomous_transition = False
        self.next_autonomous_floor = None
        self.elevator_state = "ELEVATOR_CANCELED"
        self.elevator_reason = reason
        self.last_goal_time = rospy.Time.now()
        self._set_state("WAITING", f"elevator_canceled:{reason}")
        return True

    def cancel_elevator_cb(self, req):
        del req
        with self.state_lock:
            canceled = self._cancel_elevator_locked("service_request")
            return TriggerResponse(
                success=True,
                message="Elevator canceled" if canceled else "Elevator not active",
            )

    def start_exploration_cb(self, req):
        with self.state_lock:
            if self.exploring:
                return TriggerResponse(success=True, message="Exploration already running")
            rospy.loginfo("[exploration] Start exploration")
            self.session_id += 1
            self.exploring = True
            self.waiting_for_result = False
            self.current_goal = None
            self.retry_count = 0
            self.no_reachable_frontier_cycles = 0
            self.complete_published = False
            self.visited_floors = {self.current_floor}
            self.completed_floors = set()
            self.autonomous_transition_failures = {}
            self.next_autonomous_floor = None
            self.elevator_autonomous_transition = False
            self.complete_pub.publish(Bool(data=False))
            self._set_state("WAITING", "waiting_for_inputs")
            return TriggerResponse(success=True, message="Exploration started; waiting for inputs")

    def stop_exploration_cb(self, req):
        with self.state_lock:
            if not self.exploring:
                return TriggerResponse(success=True, message="Exploration already stopped")
            rospy.loginfo("[exploration] Stop exploration")
            self._cancel_elevator_locked("exploration_stopped")
            self.exploring = False
            self.session_id += 1
            self.goal_id += 1
            self.waiting_for_result = False
            self.current_goal = None
            self.move_base_client.cancel_all_goals()
            self._set_state("STOPPED", "stop_requested")
            return TriggerResponse(success=True, message="Exploration stopped")

    def planner_loop(self, event):
        """主规划循环"""
        if not self.exploring:
            return
        if self.complete_published:
            return

        now = rospy.Time.now()
        if self.elevator_active:
            if self.mapping_lost:
                self._fail_elevator("localization_lost")
                return
            self._elevator_loop(now)
            return

        healthy, reason = self._inputs_health(now)
        if not healthy:
            self.no_reachable_frontier_cycles = 0
            state = "FAILED" if reason == "localization_lost" else "WAITING"
            self._set_state(state, reason)
            return

        if self.waiting_for_result:
            self._set_state("NAVIGATING", "active_goal")
            # 检查目标是否超时
            elapsed = (rospy.Time.now() - self.last_goal_time).to_sec()
            if elapsed > self.goal_timeout:
                rospy.logwarn("[exploration] Goal timeout, canceling")
                self.goal_id += 1
                self.move_base_client.cancel_goal()
                self._remember_failed_goal()
                self.waiting_for_result = False
                self.current_goal = None
                self.retry_count += 1
                self._set_state("RECOVERING", "goal_timeout")
            return

        # 达到连续失败上限后退避，再尝试其他候选，避免永久停摆。
        if self.retry_count >= self.max_retry:
            rospy.logwarn(
                f"[exploration] Retry limit reached ({self.retry_count}); backing off"
            )
            self.retry_count = 0
            self.last_goal_time = rospy.Time.now()
            return

        # 间隔时间
        if (rospy.Time.now() - self.last_goal_time).to_sec() < self.goal_interval:
            return

        # 选择目标
        goal, selection_reason = self._select_goal()
        if goal is not None:
            self.no_reachable_frontier_cycles = 0
            if not self._send_goal(*goal):
                self.retry_count += 1
                self._set_state("WAITING", "move_base_unavailable")
            else:
                self._set_state("NAVIGATING", "goal_sent")
        else:
            if selection_reason == "navigation_service_unavailable":
                self.no_reachable_frontier_cycles = 0
                self._set_state("WAITING", selection_reason)
                return
            # Existing-but-unreachable frontiers are not exploration
            # convergence.  Counting them as completion caused a noisy or
            # temporarily disconnected map to finish the mission while many
            # frontiers were still present.
            if selection_reason != "no_frontier":
                self.no_reachable_frontier_cycles = 0
                self._set_state("WAITING", selection_reason)
                return
            self.no_reachable_frontier_cycles += 1
            map_stable = (now - self.last_significant_map_change).to_sec() >= self.map_stable_time
            no_active_goal = not self.waiting_for_result and not self.nav_has_active_goal
            if (self.no_reachable_frontier_cycles >= self.no_frontier_cycles_required
                    and map_stable and no_active_goal):
                self._handle_converged_floor(selection_reason)
            else:
                self._set_state("WAITING", selection_reason)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = ExplorationPlanner()
        node.run()
    except rospy.ROSInterruptException:
        pass
