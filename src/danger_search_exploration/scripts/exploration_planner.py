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
import concurrent.futures
import math
import json
import os
import threading
from collections import deque
import cv2
import numpy as np
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseArray,
    PoseStamped,
    PoseWithCovarianceStamped,
    Twist,
)
from nav_msgs.msg import GridCells, OccupancyGrid
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger, TriggerResponse
from std_srvs.srv import Empty
from std_msgs.msg import Bool, Header, Int32, String
from nav_msgs.srv import GetPlan
from danger_search_common.msg import (
    MappingStatus,
    NavigationHealth,
    RecoveryEvent,
    TransitFloorAction,
    TransitFloorFeedback,
    TransitFloorResult,
)
from danger_search_common.srv import SwitchFloor

try:
    from building_generator_interfaces.srv import (
        CallElevator,
        CallElevatorRequest,
        SetDoorState,
        SetDoorStateRequest,
    )
except ImportError:  # Unit tests may import the pure helpers without SimEnv sourced.
    CallElevator = None
    CallElevatorRequest = None
    SetDoorState = None
    SetDoorStateRequest = None


def classify_navigation_failure(state, status_text=""):
    """Classify an Action terminal state without stale health telemetry."""
    normalized = str(status_text or "").lower()
    if state == actionlib.GoalStatus.SUCCEEDED:
        return "SUCCEEDED"
    if state in (actionlib.GoalStatus.PREEMPTED, actionlib.GoalStatus.RECALLED):
        return "CANCELED"
    if state == actionlib.GoalStatus.REJECTED:
        return "UNREACHABLE"
    if state in (actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.LOST):
        planning_markers = (
            "valid plan",
            "planning",
            "unreachable",
            "failed to find a plan",
        )
        if any(marker in normalized for marker in planning_markers):
            return "UNREACHABLE"
        return "CONTROL_FAILED"
    return "NONE"


def load_public_scene_topology(path):
    """Read only organizer-approved topology from team_scene_info.json."""
    normalized = os.path.abspath(os.path.expanduser(os.path.expandvars(str(path))))
    if os.path.basename(normalized) != "team_scene_info.json":
        raise ValueError("scene info must be public team_scene_info.json")
    with open(normalized, encoding="utf-8") as stream:
        document = json.load(stream)
    if document.get("schema") != "team_scene_info_v1":
        raise ValueError("unsupported team scene info schema")
    public_scene = document.get("public_scene")
    if not isinstance(public_scene, dict):
        raise ValueError("team scene info is missing public_scene")
    door_by_floor = {}
    door_by_elevator_floor = {}
    for door in public_scene.get("door_ids", []):
        if door.get("kind") == "elevator":
            floor = int(door["floor_index"])
            door_id = str(door["id"])
            door_by_floor[floor] = door_id
            elevator_id = str(door.get("elevator_id", "")).strip()
            if elevator_id:
                door_by_elevator_floor[(elevator_id, floor)] = door_id
    elevators = []
    for elevator in public_scene.get("elevators", []):
        floors = sorted({int(value) for value in elevator.get("served_floors", [])})
        if not floors:
            continue
        elevators.append({"id": str(elevator["id"]), "served_floors": floors})
    if not elevators:
        raise ValueError("public scene contains no served elevator")
    return {
        "coordinate_frame": str(document.get("coordinate_frame", "")),
        "door_by_floor": door_by_floor,
        "door_by_elevator_floor": door_by_elevator_floor,
        "elevators": elevators,
        "served_floors": sorted({
            floor for elevator in elevators for floor in elevator["served_floors"]
        }),
    }


def shortest_floor_route(topology, start_floor, target_floor):
    """Return minimum-ride elevator legs as ``(floor, elevator_id)`` pairs.

    Each pair means "take elevator_id to floor".  A single elevator ride may
    skip intermediate served floors, so all floors served by one elevator are
    adjacent.  This helper intentionally consumes only the public topology.
    """
    start_floor = int(start_floor)
    target_floor = int(target_floor)
    if start_floor == target_floor:
        return []
    adjacency = {}
    for elevator in topology.get("elevators", []):
        elevator_id = str(elevator["id"])
        floors = sorted({int(value) for value in elevator.get("served_floors", [])})
        for source in floors:
            for destination in floors:
                if source != destination:
                    adjacency.setdefault(source, []).append(
                        (destination, elevator_id)
                    )
    queue = deque([start_floor])
    previous = {start_floor: None}
    incoming_elevator = {}
    while queue:
        floor = queue.popleft()
        for next_floor, elevator_id in sorted(
                adjacency.get(floor, []), key=lambda value: (value[0], value[1])):
            if next_floor in previous:
                continue
            previous[next_floor] = floor
            incoming_elevator[next_floor] = elevator_id
            if next_floor == target_floor:
                queue.clear()
                break
            queue.append(next_floor)
    if target_floor not in previous:
        return None
    legs = []
    floor = target_floor
    while floor != start_floor:
        legs.append((floor, incoming_elevator[floor]))
        floor = previous[floor]
    legs.reverse()
    return legs


def select_next_floor_transition(topology, current_floor, completed_floors):
    """Select the nearest incomplete public floor and the first ride to it."""
    completed = {int(value) for value in completed_floors}
    choices = []
    for target in topology.get("served_floors", []):
        target = int(target)
        if target in completed or target == int(current_floor):
            continue
        route = shortest_floor_route(topology, current_floor, target)
        if route:
            choices.append((len(route), target, route))
    if not choices:
        return None
    _rides, final_target, route = min(choices, key=lambda value: (value[0], value[1]))
    next_floor, elevator_id = route[0]
    return {
        "next_floor": next_floor,
        "elevator_id": elevator_id,
        "final_target": final_target,
        "route": route,
    }


def scan_door_changed(open_ranges, closed_ranges, change_threshold_m=0.20,
                      changed_fraction=0.08):
    """Detect the local range discontinuity caused by closing a real door."""
    opened = np.asarray(open_ranges, dtype=np.float32)
    closed = np.asarray(closed_ranges, dtype=np.float32)
    if opened.shape != closed.shape or opened.size < 5:
        return False
    finite = np.isfinite(opened) & np.isfinite(closed)
    if np.count_nonzero(finite) < 5:
        return False
    changed = np.abs(opened[finite] - closed[finite]) >= float(change_threshold_m)
    return float(np.count_nonzero(changed)) / float(changed.size) >= float(
        changed_fraction
    )


class ExplorationPlanner:
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
        self.competition_mode = bool(rospy.get_param(
            "~competition_mode", rospy.get_param("/competition_mode", True)
        ))
        self.localization_backend = str(rospy.get_param(
            "~localization_backend",
            rospy.get_param("/localization_backend", "gicp"),
        ))

        # 探索参数
        self.goal_interval = rospy.get_param("~goal_interval", 2.0)
        self.max_retry = rospy.get_param("~max_retry", 3)
        self.retry_backoff = float(rospy.get_param("~retry_backoff", 10.0))
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
        self.dependency_check_timeout = rospy.get_param("~dependency_check_timeout", 0.1)
        self.input_timeout = rospy.get_param("~input_timeout", 3.0)
        self.no_frontier_cycles_required = rospy.get_param("~no_frontier_cycles_required", 5)
        self.floor_no_frontier_hold_s = float(
            rospy.get_param("~floor_no_frontier_hold_s", 10.0)
        )
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

        # ========== 多楼层探索 ==========
        self.multifloor_enabled = bool(rospy.get_param(
            "~multifloor_enabled", rospy.get_param("/multifloor_enabled", False)
        ))
        self.scene_info_file = str(rospy.get_param("~scene_info_file", "")).strip()
        self.current_floor_topic = rospy.get_param(
            "~current_floor_topic", "/mapping/current_floor"
        )
        self.elevator_service = rospy.get_param("~elevator_service", "/call_elevator")
        self.door_service = rospy.get_param("~door_service", "/set_door_state")
        self.switch_floor_service = rospy.get_param(
            "~switch_floor_service", "/localization/switch_floor"
        )
        self.mapping_pause_topic = rospy.get_param(
            "~mapping_pause_topic", "/localization/mapping_pause"
        )
        self.clear_costmaps_service = rospy.get_param(
            "~clear_costmaps_service", "/move_base/clear_costmaps"
        )
        self.transit_floor_action_name = rospy.get_param(
            "~transit_floor_action_name", "/danger_search/transit_floor"
        )
        self.elevator_cmd_topic = rospy.get_param(
            "~elevator_cmd_topic", "/danger_search/elevator_cmd_vel"
        )
        self.scan_topic = rospy.get_param("~scan_topic", "/localization/scan")
        self.elevator_id = rospy.get_param("~elevator_id", "elevator_main")
        self.elevator_door_prefix = rospy.get_param(
            "~elevator_door_prefix", "elevator_floor"
        )
        self.shaft_min_area_m2 = float(rospy.get_param("~shaft_min_area_m2", 4.0))
        self.shaft_max_area_m2 = float(rospy.get_param("~shaft_max_area_m2", 50.0))
        self.door_gap_min_width_m = float(rospy.get_param("~door_gap_min_width_m", 0.8))
        self.door_gap_max_width_m = float(rospy.get_param("~door_gap_max_width_m", 2.5))
        self.elevator_hall_approach_m = float(
            rospy.get_param("~elevator_hall_approach_m", 0.8)
        )
        self.elevator_car_target_m = float(
            rospy.get_param("~elevator_car_target_m", 1.6)
        )
        self.elevator_service_timeout_s = float(
            rospy.get_param("~elevator_service_timeout_s", 40.0)
        )
        self.elevator_max_retries = int(rospy.get_param("~elevator_max_retries", 3))
        self.floor_change_timeout_s = float(
            rospy.get_param(
                "~elevator_floor_change_timeout_s",
                rospy.get_param("~floor_change_timeout_s", 480.0),
            )
        )
        self.floor_map_stable_time_s = float(
            rospy.get_param("~floor_map_stable_time_s", 15.0)
        )
        self.elevator_crossing_timeout_s = float(
            rospy.get_param("~elevator_crossing_timeout_s", 20.0)
        )
        self.elevator_crossing_speed_mps = float(
            rospy.get_param("~elevator_crossing_speed_mps", 0.20)
        )
        self.elevator_crossing_distance_m = float(
            rospy.get_param("~elevator_crossing_distance_m", 1.4)
        )
        self.elevator_crossing_min_progress_m = float(
            rospy.get_param("~elevator_crossing_min_progress_m", 0.8)
        )
        self.elevator_crossing_clearance_m = float(
            rospy.get_param("~elevator_crossing_clearance_m", 0.32)
        )
        self.elevator_hall_navigation_max_s = float(
            rospy.get_param("~elevator_hall_navigation_max_s", 180.0)
        )
        self.elevator_hall_nominal_speed_mps = float(
            rospy.get_param("~elevator_hall_nominal_speed_mps", 0.25)
        )
        self.elevator_scan_settle_s = float(
            rospy.get_param("~elevator_scan_settle_s", 0.75)
        )
        self.elevator_door_change_threshold_m = float(
            rospy.get_param("~elevator_door_change_threshold_m", 0.20)
        )
        self.elevator_door_changed_fraction = float(
            rospy.get_param("~elevator_door_changed_fraction", 0.08)
        )
        self.floor_min_new_map_versions = int(
            rospy.get_param("~floor_min_new_map_versions", 2)
        )
        self.floor_height_m = float(rospy.get_param("~floor_height_m", 2.6))
        self.hall_validation_required = bool(
            rospy.get_param("~hall_validation_required", True)
        )
        if not (
                self.shaft_min_area_m2 > 0.0
                and self.shaft_max_area_m2 >= self.shaft_min_area_m2
                and 0.0 < self.door_gap_min_width_m <= self.door_gap_max_width_m
                and self.elevator_hall_approach_m > 0.0
                and self.elevator_service_timeout_s > 0.0
                and self.elevator_max_retries >= 1
                and self.floor_change_timeout_s > 0.0
                and self.elevator_crossing_timeout_s > 0.0
                and self.elevator_crossing_speed_mps > 0.0
                and self.elevator_crossing_distance_m > 0.0
                and 0.0 < self.elevator_crossing_min_progress_m
                <= self.elevator_crossing_distance_m
                and self.elevator_crossing_clearance_m > 0.0
                and self.elevator_hall_navigation_max_s > 0.0
                and self.elevator_hall_nominal_speed_mps > 0.0
                and self.elevator_scan_settle_s >= 0.0
                and self.elevator_door_change_threshold_m > 0.0
                and 0.0 < self.elevator_door_changed_fraction <= 1.0
                and self.floor_min_new_map_versions >= 2
                and self.floor_no_frontier_hold_s >= 10.0
                and self.floor_height_m > 0.0
                and self.retry_backoff > 0.0):
            raise rospy.ROSInitException("多楼层电梯参数无效")

        self.scene_topology = None
        if self.competition_mode and (
                not self.multifloor_enabled or self.localization_backend != "gicp"):
            raise rospy.ROSInitException(
                "competition mode requires multifloor_enabled=true and "
                "localization_backend=gicp"
            )
        if self.multifloor_enabled:
            if CallElevator is None or SetDoorState is None:
                raise rospy.ROSInitException(
                    "building_generator_interfaces is unavailable; source SimEnv first"
                )
            try:
                self.scene_topology = load_public_scene_topology(self.scene_info_file)
            except (OSError, ValueError, KeyError, TypeError) as exc:
                raise rospy.ROSInitException("invalid public scene topology: %s" % exc)
            self.served_floors = set(self.scene_topology["served_floors"])
            self.elevator_door_ids = dict(self.scene_topology["door_by_floor"])
            self.elevator_id = self.scene_topology["elevators"][0]["id"]
        else:
            self.served_floors = {0}
            self.elevator_door_ids = {}

        if not (
                0.0 < self.observation_min_distance
                <= self.observation_target_distance
                <= self.observation_max_distance
                and self.goal_clearance_margin >= 0.0
                and self.trap_blacklist_radius > 0.0
                and self.blacklist_clear_revisions >= 1):
            raise rospy.ROSInitException("前沿观察位或长期黑名单参数无效")

        # ========== 状态 ==========
        self.exploring = False
        self.current_pose = None
        self.current_map = None
        self.map_info = None
        self.map_data = None
        self.mapping_ready = False
        self.mapping_stable = False
        self.mapping_lost = True
        self.nav_ready = False
        self.nav_has_active_goal = False
        self.nav_stuck = False
        self.nav_failure_code = "NONE"
        self.nav_failure_detail = ""
        self.nav_active_goal_id = ""
        self.last_pose_time = rospy.Time(0)
        self.last_map_time = rospy.Time(0)
        self.last_mapping_status_time = rospy.Time(0)
        self.last_nav_health_time = rospy.Time(0)
        self.last_significant_map_change = rospy.Time(0)
        self.map_revision = 0
        self.map_epoch = 0
        self.current_map_version = 0
        self.floor_map_versions = {}
        self.mapping_transitioning = False
        self._reachable_cache_key = None
        self._reachable_cache = None
        self._frontier_cache_key = None
        self._frontier_cache = None
        self._frontier_cluster_cache_key = None
        self._frontier_cluster_cache = None
        self._clearance_cache_key = None
        self._clearance_cache = None
        self.no_reachable_frontier_cycles = 0
        self.remaining_frontier_count = 0
        self.exploration_state = "STOPPED"
        self.state_reason = "not_started"
        self.complete_published = False
        self.waiting_for_result = False
        self.retry_count = 0
        self.backoff_until = rospy.Time(0)
        self.last_goal_time = rospy.Time(0)
        self.current_goal = None
        self.failed_goals = []
        self.trap_blacklist = {}
        self.last_recovery_event_id = 0
        self.last_recovery_trigger_id = 0
        self.last_recovery_stuck_pose = None
        self.last_checked_path_metrics = None
        self.observation_goal_cells = []
        self.session_id = 0
        self.goal_id = 0
        self.state_lock = threading.RLock()

        # ========== 多楼层状态 ==========
        self.current_floor = 0
        self.visited_floors = set()
        self.completed_floors = set()
        self.floor_runtime = {}
        self.coverage_debt_by_floor = {}
        self.floor_no_frontier_since = rospy.Time(0)
        self.elevator_halls = []          # 候选电梯厅世界坐标 (x, y)
        self.elevator_hall_index = 0      # 当前尝试的候选
        self.elevator_hall_found = None   # 确认的电梯厅 (x, y, 门缝朝向 yaw)
        self.floor_change_active = False
        # Upper-case TransitFloor phase; service stages use START/WAIT suffixes.
        self.floor_change_step = None
        self.floor_change_target = None   # 目标楼层
        self.floor_change_deadline = rospy.Time(0)
        self.floor_change_retries = 0
        self.floor_change_start_map_revision = 0
        self.floor_change_hall_point = None
        self.floor_change_car_point = None
        self.floor_change_stable_since = rospy.Time(0)
        self._floor_change_goal_succeeded = False
        self.floor_change_gave_up_count = 0
        self.floor_change_retry_after = rospy.Time(0)
        self.floor_change_transition_id = ""
        self.floor_change_expected_epoch = 0
        self.floor_change_result = None
        self.floor_change_external = False
        self.floor_change_exit_to_hall = True
        self.floor_change_final_target = None
        self.floor_change_route = []
        self.floor_change_start_floor = 0
        self.floor_change_start_epoch = 0
        self.floor_change_start_target_version = 0
        self.floor_change_hall_deadline = rospy.Time(0)
        self.floor_change_stage_deadline = rospy.Time(0)
        self.floor_change_crossing_start = None
        self.floor_change_crossing_direction = 0.0
        self.floor_change_crossing_target_m = 0.0
        self.floor_change_open_scan = None
        self.floor_change_open_scan_stamp = rospy.Time(0)
        self.floor_change_phase_started = rospy.Time(0)
        self.floor_change_error_code = ""
        self.floor_transit_fatal = False
        self.active_elevator_id = self.elevator_id
        self._service_future = None
        self._service_generation = 0
        self._service_future_generation = 0
        self._service_kind = ""
        self._service_executor = concurrent.futures.ThreadPoolExecutor(max_workers=4)
        self.latest_scan = None
        self.last_scan_time = rospy.Time(0)
        self.elevator_client = None
        self.door_client = None

        # ========== Action客户端 ==========
        self.move_base_client = actionlib.SimpleActionClient(
            self.move_base_action_name, MoveBaseAction
        )

        # ========== 服务客户端 ==========
        self.make_plan_client = rospy.ServiceProxy(self.make_plan_service, GetPlan)
        self.switch_floor_client = rospy.ServiceProxy(
            self.switch_floor_service, SwitchFloor
        )
        self.clear_costmaps_client = rospy.ServiceProxy(
            self.clear_costmaps_service, Empty
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
        self.scan_sub = rospy.Subscriber(
            self.scan_topic, LaserScan, self._scan_callback, queue_size=2
        )
        if self.multifloor_enabled:
            self.current_floor_sub = rospy.Subscriber(
                self.current_floor_topic, Int32, self._current_floor_callback, queue_size=2
            )

        # ========== 服务 ==========
        self.start_srv = rospy.Service(
            self.start_service, Trigger, self.start_exploration_cb
        )
        self.stop_srv = rospy.Service(
            self.stop_service, Trigger, self.stop_exploration_cb
        )
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10, latch=True)
        self.complete_pub = rospy.Publisher(self.complete_topic, Bool, queue_size=1, latch=True)
        self.observation_goals_pub = rospy.Publisher(
            "/exploration/observation_goals", PoseArray, queue_size=1
        )
        self.blacklist_pub = rospy.Publisher(
            "/exploration/trap_blacklist", GridCells, queue_size=1, latch=True
        )
        self.mapping_pause_pub = rospy.Publisher(
            self.mapping_pause_topic, Header, queue_size=10
        )
        self.elevator_cmd_pub = rospy.Publisher(
            self.elevator_cmd_topic, Twist, queue_size=2
        )

        self.transit_server = actionlib.SimpleActionServer(
            self.transit_floor_action_name,
            TransitFloorAction,
            execute_cb=self._execute_transit_action,
            auto_start=False,
        )
        self.transit_server.start()

        # ========== 主循环 ==========
        self.planner_timer = rospy.Timer(rospy.Duration(0.5), self.planner_loop)
        self.transit_timer = rospy.Timer(
            rospy.Duration(0.1), self._transit_timer_callback
        )

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
            self._reachable_cache_key = None
            self._reachable_cache = None
            self._component_cache_key = None
            self._frontier_cache_key = None
            self._frontier_cluster_cache_key = None
            self._clearance_cache_key = None
            self.last_significant_map_change = now
            self.no_reachable_frontier_cycles = 0
            self._update_blacklist_validity()

    def mapping_status_callback(self, msg):
        self.mapping_ready = msg.ready
        self.mapping_stable = msg.stable
        self.mapping_lost = msg.lost
        previous_epoch = self.map_epoch
        self.map_epoch = int(getattr(msg, "map_epoch", self.map_epoch))
        self.mapping_transitioning = bool(getattr(msg, "transitioning", False))
        self.current_floor = int(msg.current_floor)
        self.floor_map_versions = {
            int(item.floor_id): int(item.map_version)
            for item in getattr(msg, "floor_maps", [])
        }
        self.current_map_version = int(
            self.floor_map_versions.get(self.current_floor, 0)
        )
        self.visited_floors.add(self.current_floor)
        if self.map_epoch != previous_epoch:
            self._reachable_cache_key = None
            self._reachable_cache = None
            self._component_cache_key = None
            self._frontier_cache_key = None
            self._frontier_cluster_cache_key = None
            self._clearance_cache_key = None
        self.last_mapping_status_time = rospy.Time.now()

    def _scan_callback(self, message):
        self.latest_scan = message
        self.last_scan_time = rospy.Time.now()

    def nav_health_callback(self, msg):
        self.nav_ready = msg.ready
        self.nav_has_active_goal = msg.has_active_goal
        self.nav_stuck = msg.stuck
        self.nav_failure_code = msg.failure_code
        self.nav_failure_detail = msg.failure_detail
        self.nav_active_goal_id = msg.active_goal_id
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
        key = (
            int(getattr(self, "map_epoch", 0)),
            int(getattr(self, "map_revision", 0)),
            self.map_data.shape,
            int(self.free_threshold),
        )
        if key == getattr(self, "_clearance_cache_key", None):
            return self._clearance_cache
        free = (self.map_data >= 0) & (self.map_data < self.free_threshold)
        clearance = cv2.distanceTransform(
            free.astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE
        ).astype(np.float32) * self.map_info.resolution
        self._clearance_cache_key = key
        self._clearance_cache = clearance
        return clearance

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
        # This method is called only for a significant map revision. Force the
        # cached distance field to follow that revision even in isolated unit
        # harnesses which update map_data without invoking map_callback.
        self._clearance_cache_key = None
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
            "map_epoch": self.map_epoch,
            "current_floor": self.current_floor,
            "completed_floors": sorted(self.completed_floors),
            "served_floors": sorted(self.served_floors),
            "floor_transition_active": self.floor_change_active,
            "has_active_goal": bool(self.waiting_for_result or self.nav_has_active_goal),
            "blacklisted_cell_count": len(getattr(self, "trap_blacklist", {})),
            "observation_goal_count": len(getattr(self, "observation_goal_cells", [])),
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
        if self.mapping_transitioning:
            return False, "mapping_transitioning"
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
            failure for failure in self.failed_goals
            if (now - failure[2]).to_sec() < self.failed_goal_cooldown
        ]
        return any(
            math.hypot(goal_x - failed_x, goal_y - failed_y)
            < self.failed_goal_radius
            for failed_x, failed_y, _ in self.failed_goals
        )

    def _remember_failed_goal(self):
        if self.current_goal is not None:
            self.failed_goals.append(
                (self.current_goal[0], self.current_goal[1], rospy.Time.now())
            )

    def _frontier_mask(self):
        """返回与未知四邻接的已知自由栅格。"""
        key = (
            int(getattr(self, "map_epoch", 0)),
            int(getattr(self, "map_revision", 0)),
            self.map_data.shape,
            int(self.free_threshold),
        )
        if key == getattr(self, "_frontier_cache_key", None):
            return self._frontier_cache
        free = (self.map_data >= 0) & (self.map_data < self.free_threshold)
        unknown = self.map_data == -1
        adjacent_unknown = np.zeros_like(unknown, dtype=bool)
        adjacent_unknown[1:, :] |= unknown[:-1, :]
        adjacent_unknown[:-1, :] |= unknown[1:, :]
        adjacent_unknown[:, 1:] |= unknown[:, :-1]
        adjacent_unknown[:, :-1] |= unknown[:, 1:]
        frontier = free & adjacent_unknown
        self._frontier_cache_key = key
        self._frontier_cache = frontier
        return frontier

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

    def _frontier_clusters(self, frontier=None, cache_key=None):
        """Return valid 8-connected frontier clusters via OpenCV WFD."""
        if frontier is None:
            frontier = self._frontier_mask()
            cache_key = (
                int(getattr(self, "map_epoch", 0)),
                int(getattr(self, "map_revision", 0)),
                "all",
            )
        if (cache_key is not None
                and cache_key == getattr(self, "_frontier_cluster_cache_key", None)):
            return self._frontier_cluster_cache
        min_cells = max(
            1, int(math.ceil(self.min_frontier_length / self.map_info.resolution))
        )
        clusters = []
        count, labels, stats, _centroids = cv2.connectedComponentsWithStats(
            np.asarray(frontier, dtype=np.uint8), connectivity=8
        )
        for label in range(1, count):
            if int(stats[label, cv2.CC_STAT_AREA]) < min_cells:
                continue
            x = int(stats[label, cv2.CC_STAT_LEFT])
            y = int(stats[label, cv2.CC_STAT_TOP])
            width = int(stats[label, cv2.CC_STAT_WIDTH])
            height = int(stats[label, cv2.CC_STAT_HEIGHT])
            cells = np.argwhere(labels[y:y + height, x:x + width] == label)
            clusters.append([
                (x + int(mx), y + int(my)) for my, mx in cells
            ])
        if cache_key is not None:
            self._frontier_cluster_cache_key = cache_key
            self._frontier_cluster_cache = clusters
        return clusters

    def _observation_goal_for_cluster(
            self, cluster, reachable, clearance, frontier_distance=None):
        margin = int(math.ceil(
            self.observation_max_distance / self.map_info.resolution
        )) + 2
        cluster_x = [cell[0] for cell in cluster]
        cluster_y = [cell[1] for cell in cluster]
        x0 = max(0, min(cluster_x) - margin)
        x1 = min(self.map_info.width, max(cluster_x) + margin + 1)
        y0 = max(0, min(cluster_y) - margin)
        y1 = min(self.map_info.height, max(cluster_y) + margin + 1)
        if frontier_distance is None:
            cluster_mask = np.zeros((y1 - y0, x1 - x0), dtype=np.uint8)
            for cell_x, cell_y in cluster:
                cluster_mask[cell_y - y0, cell_x - x0] = 1
            distance_to_frontier = cv2.distanceTransform(
                (cluster_mask == 0).astype(np.uint8),
                cv2.DIST_L2,
                cv2.DIST_MASK_PRECISE,
            ) * self.map_info.resolution
        else:
            distance_to_frontier = frontier_distance[y0:y1, x0:x1]
        required_clearance = (
            self.connectivity_clearance_radius + self.goal_clearance_margin
        )
        candidate_mask = (
            reachable[y0:y1, x0:x1]
            & (distance_to_frontier >= (
                self.observation_min_distance
                + 0.5 * self.map_info.resolution - 1e-6
            ))
            & (distance_to_frontier <= self.observation_max_distance + 1e-6)
            & (clearance[y0:y1, x0:x1] >= required_clearance - 1e-6)
        )
        for cell in getattr(self, "trap_blacklist", {}):
            if x0 <= cell[0] < x1 and y0 <= cell[1] < y1:
                candidate_mask[cell[1] - y0, cell[0] - x0] = False
        candidate_y, candidate_x = np.nonzero(candidate_mask)
        if not len(candidate_y):
            return None
        centroid_x = sum(cell[0] for cell in cluster) / len(cluster)
        centroid_y = sum(cell[1] for cell in cluster) / len(cluster)
        target_error = np.abs(
            distance_to_frontier[candidate_y, candidate_x]
            - self.observation_target_distance
        )
        candidate_clearance = clearance[
            y0 + candidate_y, x0 + candidate_x
        ]
        centroid_distance = (
            (x0 + candidate_x - centroid_x) ** 2
            + (y0 + candidate_y - centroid_y) ** 2
        )
        best_index = int(np.lexsort((
            centroid_distance, -candidate_clearance, target_error
        ))[0])
        best_local_y = int(candidate_y[best_index])
        best_local_x = int(candidate_x[best_index])
        best_x = x0 + int(best_local_x)
        best_y = y0 + int(best_local_y)
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
        frontier_distance = cv2.distanceTransform(
            (frontier == 0).astype(np.uint8),
            cv2.DIST_L2,
            cv2.DIST_MASK_PRECISE,
        ) * self.map_info.resolution
        goals = []
        cluster_key = (
            int(getattr(self, "map_epoch", 0)),
            int(getattr(self, "map_revision", 0)),
            int(getattr(self, "_reachable_component_id", 0)),
        )
        for cluster in self._frontier_clusters(
                frontier & reachable, cache_key=cluster_key):
            goal = self._observation_goal_for_cluster(
                cluster, reachable, clearance, frontier_distance
            )
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
        component_key = (
            int(getattr(self, "map_epoch", 0)),
            int(getattr(self, "map_revision", 0)),
            self.map_data.shape,
            int(self.free_threshold),
            int(self.connectivity_occupied_threshold),
            float(self.connectivity_clearance_radius),
        )
        if component_key != getattr(self, "_component_cache_key", None):
            traversable = self._connectivity_traversable_mask()
            _count, labels = cv2.connectedComponents(
                traversable.astype(np.uint8), connectivity=4
            )
            self._component_cache_key = component_key
            self._component_traversable = traversable
            self._component_labels = labels
        else:
            traversable = self._component_traversable
        start_x, start_y = self._world_to_map(
            self.current_pose.position.x, self.current_pose.position.y
        )
        if (start_x < 0 or start_x >= self.map_info.width
                or start_y < 0 or start_y >= self.map_info.height):
            self._reachable_component_id = 0
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

        if seed is None:
            self._reachable_component_id = 0
            return np.zeros_like(traversable, dtype=bool)

        labels = self._component_labels
        component_id = int(labels[seed[1], seed[0]])
        self._reachable_component_id = component_id
        if component_id == 0:
            return np.zeros_like(traversable, dtype=bool)
        return labels == component_id

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

    def _send_goal(self, gx, gy, yaw=0.0):
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
        self.last_recovery_trigger_id = 0
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
        self.last_goal_time = rospy.Time.now()
        rospy.loginfo(f"[exploration] Sent goal: ({gx:.2f}, {gy:.2f})")
        return True

    def goal_done_cb(self, session_id, goal_id, state, result):
        """目标完成回调"""
        with self.state_lock:
            if (session_id != self.session_id or goal_id != self.goal_id
                    or (not self.exploring and not self.floor_change_active)):
                return
            self.waiting_for_result = False
            if self.floor_change_active:
                # 换层流程内的导航目标：只记录成功与否，交 _advance_floor_change 推进。
                self._floor_change_goal_succeeded_set(
                    state == actionlib.GoalStatus.SUCCEEDED
                )
                self.current_goal = None
                return
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("[exploration] Goal succeeded")
                self.retry_count = 0
                self.backoff_until = rospy.Time(0)
            else:
                try:
                    status_text = self.move_base_client.get_goal_status_text()
                except AttributeError:
                    status_text = ""
                failure_code = classify_navigation_failure(state, status_text)
                rospy.loginfo(
                    "[exploration] Goal failed: state=%d action_text=%s "
                    "classified=%s nav_failure=%s detail=%s",
                    state,
                    status_text or "<empty>",
                    failure_code,
                    getattr(self, "nav_failure_code", "NONE"),
                    getattr(self, "nav_failure_detail", ""),
                )
                if failure_code not in ("CANCELED", "NONE"):
                    self._remember_failed_goal()
                    self.retry_count += 1
            self.current_goal = None

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
            self.backoff_until = rospy.Time(0)
            self.no_reachable_frontier_cycles = 0
            self.floor_no_frontier_since = rospy.Time(0)
            self.complete_published = False
            self.complete_pub.publish(Bool(data=False))
            self.visited_floors = {self.current_floor}
            self.completed_floors = set()
            self.floor_runtime = {}
            self.coverage_debt_by_floor = {}
            self.floor_change_active = False
            self.floor_change_step = None
            self.floor_change_gave_up_count = 0
            self.floor_transit_fatal = False
            self.floor_change_retry_after = rospy.Time(0)
            self.elevator_halls = []
            self.elevator_hall_index = 0
            self.elevator_hall_found = None
            self._set_state("WAITING", "waiting_for_inputs")
            return TriggerResponse(success=True, message="Exploration started; waiting for inputs")

    def stop_exploration_cb(self, req):
        with self.state_lock:
            if not self.exploring:
                return TriggerResponse(success=True, message="Exploration already stopped")
            rospy.loginfo("[exploration] Stop exploration")
            if self.floor_change_active and not self.floor_change_external:
                self._cancel_floor_change(
                    "CANCELED", "exploration stopped during floor transit"
                )
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
        now = rospy.Time.now()
        # A dedicated 10 Hz timer advances transit so its leased velocity
        # input remains fresh; ordinary frontier planning pauses meanwhile.
        if self.floor_change_active:
            return
        if not self.exploring or self.complete_published:
            return
        if self.floor_transit_fatal:
            self._set_state("FAILED", "floor_transit_requires_operator_reset")
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
        if self.backoff_until != rospy.Time(0) and now < self.backoff_until:
            self._set_state("WAITING", "retry_backoff")
            return
        if self.retry_count >= self.max_retry:
            rospy.logwarn(
                "[exploration] Retry limit reached (%d); backing off %.1f s",
                self.retry_count,
                self.retry_backoff,
            )
            self.retry_count = 0
            self.backoff_until = now + rospy.Duration(self.retry_backoff)
            self._set_state("WAITING", "retry_backoff")
            return

        # 间隔时间
        if (rospy.Time.now() - self.last_goal_time).to_sec() < self.goal_interval:
            return

        # 选择目标
        goal, selection_reason = self._select_goal()
        if goal is not None:
            self.no_reachable_frontier_cycles = 0
            self.floor_no_frontier_since = rospy.Time(0)
            if not self._send_goal(*goal):
                self.retry_count += 1
                self._set_state("WAITING", "move_base_unavailable")
            else:
                self._set_state("NAVIGATING", "goal_sent")
        else:
            if selection_reason == "navigation_service_unavailable":
                self.no_reachable_frontier_cycles = 0
                self.floor_no_frontier_since = rospy.Time(0)
                self._set_state("WAITING", selection_reason)
                return
            # Existing-but-unreachable frontiers are not exploration
            # convergence.  Counting them as completion caused a noisy or
            # temporarily disconnected map to finish the mission while many
            # frontiers were still present.
            if selection_reason != "no_frontier":
                self.no_reachable_frontier_cycles = 0
                self.floor_no_frontier_since = rospy.Time(0)
                self._set_state("WAITING", selection_reason)
                return
            self.no_reachable_frontier_cycles += 1
            if self.floor_no_frontier_since == rospy.Time(0):
                self.floor_no_frontier_since = now
            map_stable = (now - self.last_significant_map_change).to_sec() >= self.map_stable_time
            no_active_goal = not self.waiting_for_result and not self.nav_has_active_goal
            no_frontier_held = (
                now - self.floor_no_frontier_since
            ).to_sec() >= self.floor_no_frontier_hold_s
            no_coverage_debt = not self.coverage_debt_by_floor.get(
                self.current_floor, set()
            )
            if (self.no_reachable_frontier_cycles >= self.no_frontier_cycles_required
                    and no_frontier_held and no_coverage_debt
                    and map_stable and no_active_goal):
                self._save_current_floor_runtime()
                self.completed_floors.add(self.current_floor)
                next_floor = self._select_next_floor()
                if not self.multifloor_enabled:
                    self._complete_exploration("all_served_floors_explored")
                elif self.completed_floors.issuperset(self.served_floors):
                    self._complete_exploration("all_served_floors_explored")
                elif next_floor is None:
                    self.floor_transit_fatal = True
                    self._set_state("FAILED", "served_floor_topology_unreachable")
                elif self.floor_change_gave_up_count >= self.elevator_max_retries:
                    self._set_state("FAILED", "floor_transit_unavailable")
                elif now > self.floor_change_retry_after:
                    self._begin_floor_change(next_floor)
            else:
                self._set_state("WAITING", selection_reason)

    # ========== 多楼层：电梯自主发现与换层 ==========

    def _current_floor_callback(self, message):
        """Compatibility mirror of the localization-owned discrete floor."""
        floor = int(message.data)
        if floor != self.current_floor:
            rospy.loginfo("[exploration] current floor -> %d", floor)
        self.current_floor = floor
        self.visited_floors.add(floor)

    def _elevator_door_id(self, floor):
        keyed = self.scene_topology.get("door_by_elevator_floor", {})
        return keyed.get(
            (self.active_elevator_id, int(floor)),
            self.elevator_door_ids.get(int(floor)),
        )

    def _select_next_floor(self):
        if not self.multifloor_enabled:
            return None
        selection = select_next_floor_transition(
            self.scene_topology, self.current_floor, self.completed_floors
        )
        return None if selection is None else selection["final_target"]

    def _save_current_floor_runtime(self):
        self.floor_runtime[int(self.current_floor)] = {
            "failed_goals": list(self.failed_goals),
            "trap_blacklist": dict(self.trap_blacklist),
            "map_epoch": int(self.map_epoch),
            "map_version": int(self.current_map_version),
        }

    def _restore_current_floor_runtime(self):
        runtime = self.floor_runtime.get(int(self.current_floor), {})
        self.failed_goals = list(runtime.get("failed_goals", []))
        self.trap_blacklist = dict(runtime.get("trap_blacklist", {}))
        self.no_reachable_frontier_cycles = 0
        self.floor_no_frontier_since = rospy.Time(0)
        self._reachable_cache_key = None
        self._reachable_cache = None
        self._component_cache_key = None
        self._frontier_cache_key = None
        self._frontier_cluster_cache_key = None
        self._clearance_cache_key = None
        self._publish_blacklist()

    def _ensure_service_clients(self):
        if CallElevator is None or SetDoorState is None:
            return False
        if self.elevator_client is None:
            self.elevator_client = rospy.ServiceProxy(
                self.elevator_service, CallElevator
            )
        if self.door_client is None:
            self.door_client = rospy.ServiceProxy(
                self.door_service, SetDoorState
            )
        return True

    def _call_elevator_request(self, target_floor, open_doors):
        if not self._ensure_service_clients():
            raise RuntimeError("typed elevator interfaces are unavailable")
        rospy.wait_for_service(
            self.elevator_service, timeout=self.elevator_service_timeout_s
        )
        request = CallElevatorRequest(
            elevator_id=self.active_elevator_id,
            target_floor=int(target_floor),
            open_doors=bool(open_doors),
        )
        return self.elevator_client(request)

    def _door_request(self, floor, open_state):
        if not self._ensure_service_clients():
            raise RuntimeError("typed door interface is unavailable")
        door_id = self._elevator_door_id(floor)
        if not door_id:
            raise RuntimeError("public scene has no elevator door for floor %d" % floor)
        rospy.wait_for_service(
            self.door_service, timeout=self.elevator_service_timeout_s
        )
        return self.door_client(SetDoorStateRequest(
            door_id=door_id, open=bool(open_state)
        ))

    def _switch_floor_request(self):
        rospy.wait_for_service(
            self.switch_floor_service, timeout=self.elevator_service_timeout_s
        )
        return self.switch_floor_client(
            transition_id=self.floor_change_transition_id,
            target_floor=int(self.floor_change_target),
        )

    def _clear_costmaps_request(self):
        rospy.wait_for_service(
            self.clear_costmaps_service, timeout=self.elevator_service_timeout_s
        )
        return self.clear_costmaps_client()

    def _submit_service(self, kind, callback):
        if self._service_future is not None:
            return False
        self._service_generation += 1
        self._service_future_generation = self._service_generation
        self._service_kind = str(kind)
        self._service_future = self._service_executor.submit(callback)
        self.floor_change_stage_deadline = rospy.Time.now() + rospy.Duration(
            self.elevator_service_timeout_s
        )
        return True

    def _invalidate_service(self):
        self._service_generation += 1
        self._service_future = None
        self._service_kind = ""
        self.floor_change_stage_deadline = rospy.Time(0)

    def _poll_service(self, now, expected_kind):
        if self._service_future is None or self._service_kind != expected_kind:
            return "missing", None
        if now > self.floor_change_stage_deadline:
            self._invalidate_service()
            return "timeout", None
        if not self._service_future.done():
            return "pending", None
        future = self._service_future
        generation = self._service_future_generation
        self._service_future = None
        self._service_kind = ""
        if generation != self._service_generation:
            return "stale", None
        try:
            return "done", future.result()
        except Exception as exc:  # ROSException and ServiceException included.
            return "error", exc

    def _publish_mapping_pause(self):
        message = Header()
        message.stamp = rospy.Time.now()
        message.frame_id = self.map_frame
        self.mapping_pause_pub.publish(message)

    def _stop_elevator_motion(self):
        self.elevator_cmd_pub.publish(Twist())

    def _scan_window(self, message, backward=False):
        if message is None or not message.ranges or message.angle_increment == 0.0:
            return np.array([], dtype=np.float32)
        values = []
        center = math.pi if backward else 0.0
        for index, value in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            delta = math.atan2(math.sin(angle - center), math.cos(angle - center))
            if abs(delta) > math.radians(40.0):
                continue
            if (not math.isfinite(value) or value < message.range_min
                    or value > message.range_max):
                values.append(float("nan"))
            else:
                values.append(float(value))
        return np.asarray(values, dtype=np.float32)

    def _perimeter_door_gaps(self, component):
        """在实心连通区域周界找门缝，返回 (外侧自由格x, 外侧格y, 朝向井道内的yaw)。"""
        ys, xs = np.where(component)
        if not len(xs):
            return []
        x0, x1 = int(xs.min()), int(xs.max())
        y0, y1 = int(ys.min()), int(ys.max())
        res = self.map_info.resolution
        min_cells = max(1, int(math.ceil(self.door_gap_min_width_m / res)))
        max_cells = int(math.ceil(self.door_gap_max_width_m / res))
        gaps = []
        edges = (
            # edge, 边界行/列取值, 扫描起点, 朝井道内yaw
            ("west",  lambda y: component[y, x0], y0, 0.0),
            ("east",  lambda y: component[y, x1], y0, math.pi),
            ("north", lambda x: component[y1, x], x0, -math.pi / 2.0),
            ("south", lambda x: component[y0, x], x0, math.pi / 2.0),
        )
        for edge, boundary, along_start, into_yaw in edges:
            cells = []
            if edge in ("west", "east"):
                cells = [not boundary(y) for y in range(y0, y1 + 1)]
            else:
                cells = [not boundary(x) for x in range(x0, x1 + 1)]
            i = 0
            while i < len(cells):
                if not cells[i]:
                    i += 1
                    continue
                j = i
                while j < len(cells) and cells[j]:
                    j += 1
                width = j - i
                if min_cells <= width <= max_cells:
                    mid = (i + j) // 2
                    if edge in ("west", "east"):
                        gx = x0 - 1 if edge == "west" else x1 + 1
                        gy = along_start + mid
                    else:
                        gx = along_start + mid
                        gy = y1 + 1 if edge == "north" else y0 - 1
                    if 0 <= gx < self.map_info.width and 0 <= gy < self.map_info.height:
                        gaps.append((gx, gy, into_yaw))
                i = j
        return gaps

    def _closed_space_hall_candidates(self, occupied, free):
        """Find rectangular shafts even when their walls join building walls.

        Closing only door-sized gaps turns a U-shaped shaft into an enclosed
        free-space component.  Inspecting the *original* boundary then recovers
        the door opening.  Runtime door-motion validation remains mandatory in
        competition mode, which is what separates a shaft from a small room.
        """
        resolution = self.map_info.resolution
        height, width = occupied.shape
        min_gap = max(1, int(math.ceil(self.door_gap_min_width_m / resolution)))
        max_gap = int(math.ceil(self.door_gap_max_width_m / resolution))
        kernel_lengths = sorted({
            min_gap + 2,
            (min_gap + max_gap) // 2 + 1,
            max_gap + 2,
        })
        closed_variants = []
        for length in kernel_lengths:
            closed_variants.append(cv2.morphologyEx(
                occupied,
                cv2.MORPH_CLOSE,
                np.ones((length, 1), dtype=np.uint8),
            ))
            closed_variants.append(cv2.morphologyEx(
                occupied,
                cv2.MORPH_CLOSE,
                np.ones((1, length), dtype=np.uint8),
            ))
        candidates = []

        def line_runs(values):
            runs = []
            index = 0
            while index < len(values):
                if values[index]:
                    index += 1
                    continue
                end = index
                while end < len(values) and not values[end]:
                    end += 1
                if min_gap <= end - index <= max_gap:
                    runs.append((index, end))
                index = end
            return runs

        for closed in closed_variants:
            count, _labels, stats, _centroids = cv2.connectedComponentsWithStats(
                (closed == 0).astype(np.uint8), connectivity=4
            )
            for label in range(1, count):
                x, y, w, h, area = (int(value) for value in stats[label])
                if x <= 1 or y <= 1 or x + w >= width - 2 or y + h >= height - 2:
                    continue
                physical_area = float(w * h) * resolution ** 2
                if not self.shaft_min_area_m2 <= physical_area <= self.shaft_max_area_m2:
                    continue
                aspect = float(w) / float(max(h, 1))
                fill = float(area) / float(max(w * h, 1))
                if not 0.40 <= aspect <= 2.50 or fill < 0.55:
                    continue

                west = occupied[y:y + h, x - 1] != 0
                east = occupied[y:y + h, x + w] != 0
                south = occupied[y - 1, x:x + w] != 0
                north = occupied[y + h, x:x + w] != 0
                boundary_lines = (west, east, north, south)
                if sum(float(np.mean(line)) for line in boundary_lines) < 2.1:
                    continue
                edge_specs = (
                    (west, "west", 0.0),
                    (east, "east", math.pi),
                    (north, "north", -math.pi / 2.0),
                    (south, "south", math.pi / 2.0),
                )
                for line, edge, into_yaw in edge_specs:
                    for start, end in line_runs(line):
                        middle = (start + end - 1) // 2
                        if edge == "west":
                            gx, gy = x - 2, y + middle
                        elif edge == "east":
                            gx, gy = x + w + 1, y + middle
                        elif edge == "north":
                            gx, gy = x + middle, y + h + 1
                        else:
                            gx, gy = x + middle, y - 2
                        if (0 <= gx < width and 0 <= gy < height
                                and bool(free[gy, gx])):
                            wx, wy = self._map_to_world(gx, gy)
                            candidate = (wx, wy, into_yaw)
                            if not any(
                                    math.hypot(wx - other[0], wy - other[1]) < 0.5
                                    and abs(math.atan2(
                                        math.sin(into_yaw - other[2]),
                                        math.cos(into_yaw - other[2]),
                                    )) < 0.35
                                    for other in candidates):
                                candidates.append(candidate)
        return candidates

    def _detect_elevator_halls(self):
        """从当前层地图检测电梯井门缝候选，返回 (world_x, world_y, 朝井道内yaw) 列表。

        电梯井/楼梯井是薄墙围成的"井道"：墙面积小但包围盒大（电梯井 ~2.4x2.7m）。
        用包围盒面积识别井道形状，再在周界找门缝。
        """
        if self.map_data is None:
            return []
        occupied = (self.map_data >= self.connectivity_occupied_threshold).astype(np.uint8)
        num, labels, stats, _ = cv2.connectedComponentsWithStats(occupied, connectivity=8)
        min_bbox_cells = self.shaft_min_area_m2 / (self.map_info.resolution ** 2)
        max_bbox_cells = self.shaft_max_area_m2 / (self.map_info.resolution ** 2)
        free = (self.map_data >= 0) & (self.map_data < self.free_threshold)
        width, height = self.map_info.width, self.map_info.height
        candidates = []
        for index in range(1, num):
            x, y, w, h, _area = stats[index]
            bbox_area = w * h
            if bbox_area < min_bbox_cells or bbox_area > max_bbox_cells:
                continue
            # 跳过贴地图边界的区域（密封楼外/世界墙等大面积伪影）
            if x <= 0 or y <= 0 or x + w >= width - 1 or y + h >= height - 1:
                continue
            component = (labels == index)
            for gx, gy, into_yaw in self._perimeter_door_gaps(component):
                if not (0 <= gy < height and 0 <= gx < width):
                    continue
                if not free[gy, gx]:
                    continue
                wx, wy = self._map_to_world(gx, gy)
                candidates.append((wx, wy, into_yaw))
        candidates.extend(self._closed_space_hall_candidates(occupied, free))
        deduplicated = []
        for candidate in candidates:
            if any(
                    math.hypot(candidate[0] - existing[0],
                               candidate[1] - existing[1]) < 0.50
                    and abs(math.atan2(
                        math.sin(candidate[2] - existing[2]),
                        math.cos(candidate[2] - existing[2]),
                    )) < 0.35
                    for existing in deduplicated):
                continue
            deduplicated.append(candidate)
        return deduplicated

    def _record_floor_change_result(self, success, failure_code, message):
        self.floor_change_result = {
            "success": bool(success),
            "reached_floor": int(self.current_floor),
            "map_epoch": int(self.map_epoch),
            "failure_code": str(failure_code),
            "message": str(message),
        }

    def _set_floor_change_phase(self, phase, reason=None):
        self.floor_change_step = str(phase)
        self.floor_change_phase_started = rospy.Time.now()
        self._set_state("FLOOR_CHANGE", reason or phase.lower())

    def _begin_floor_change(self, target_floor, external=False,
                            exit_to_hall=True):
        """Start one or more minimum-transfer rides to ``target_floor``."""
        requested_target = int(target_floor)
        self.floor_change_result = None
        if not self.multifloor_enabled:
            self._record_floor_change_result(
                False, "SERVICE_UNAVAILABLE", "multifloor mode is disabled"
            )
            return False
        if self.current_pose is None or self.map_data is None:
            self._record_floor_change_result(
                False, "NO_HALL", "pose or map is unavailable"
            )
            return False
        if requested_target == self.current_floor:
            self._record_floor_change_result(True, "", "already on target floor")
            return True
        route = shortest_floor_route(
            self.scene_topology, self.current_floor, requested_target
        )
        if not route:
            self._record_floor_change_result(
                False,
                "SERVICE_REJECTED",
                "target floor is not reachable through public elevator topology",
            )
            return False

        self._save_current_floor_runtime()
        self.floor_change_active = True
        self.floor_change_external = bool(external)
        self.floor_change_exit_to_hall = bool(exit_to_hall)
        self.floor_change_final_target = requested_target
        self.floor_change_route = list(route)
        self.floor_change_target, self.active_elevator_id = route[0]
        self.floor_change_start_floor = int(self.current_floor)
        self.floor_change_start_epoch = int(self.map_epoch)
        self.floor_change_start_target_version = int(
            self.floor_map_versions.get(self.floor_change_target, 0)
        )
        self.floor_change_transition_id = "transit-%d-%d-%d-%d-%d" % (
            self.session_id,
            self.goal_id,
            self.current_floor,
            self.floor_change_target,
            rospy.Time.now().to_nsec(),
        )
        now = rospy.Time.now()
        self.floor_change_deadline = now + rospy.Duration(
            self.floor_change_timeout_s
        )
        self.floor_change_retries = 0
        self.floor_change_expected_epoch = 0
        self.floor_change_stable_since = rospy.Time(0)
        self.floor_change_open_scan = None
        self._invalidate_service()
        self._stop_elevator_motion()

        # Retire the ordinary goal epoch before selecting an elevator goal.
        self.goal_id += 1
        self.move_base_client.cancel_all_goals()
        self.waiting_for_result = False
        self.current_goal = None
        self._floor_change_goal_succeeded = None

        self.elevator_halls = self._detect_elevator_halls()
        if not self.elevator_halls:
            self._floor_change_fail("NO_HALL", "no elevator hall candidate")
            return False
        cx = self.current_pose.position.x
        cy = self.current_pose.position.y
        self.elevator_halls.sort(
            key=lambda hall: math.hypot(hall[0] - cx, hall[1] - cy)
        )
        self.elevator_hall_index = 0
        self._set_floor_change_phase("TO_HALL", "select_elevator_hall")
        return self._pick_elevator_hall_and_send()

    def _pick_elevator_hall_and_send(self):
        """Plan to the 0.8 m hall stand-off; consume each candidate once."""
        cx = self.current_pose.position.x
        cy = self.current_pose.position.y
        had_candidate = self.elevator_hall_index < len(self.elevator_halls)
        while self.elevator_hall_index < len(self.elevator_halls):
            hx, hy, into_yaw = self.elevator_halls[self.elevator_hall_index]
            self.elevator_hall_index += 1
            approach_x = hx - self.elevator_hall_approach_m * math.cos(into_yaw)
            approach_y = hy - self.elevator_hall_approach_m * math.sin(into_yaw)
            map_x, map_y = self._world_to_map(approach_x, approach_y)
            if not self._is_free(map_x, map_y):
                continue
            path_state = self._check_path(cx, cy, approach_x, approach_y)
            if path_state != "reachable":
                continue
            metrics = self.last_checked_path_metrics or {}
            path_length = float(metrics.get(
                "path_length", math.hypot(approach_x - cx, approach_y - cy)
            ))
            navigation_timeout = min(
                self.elevator_hall_navigation_max_s,
                max(30.0, 20.0 + 2.5 * path_length
                    / self.elevator_hall_nominal_speed_mps),
            )
            self.floor_change_hall_point = (hx, hy, into_yaw)
            self.floor_change_car_point = (
                hx + self.elevator_car_target_m * math.cos(into_yaw),
                hy + self.elevator_car_target_m * math.sin(into_yaw),
            )
            self._floor_change_goal_succeeded = None
            if self._send_goal(approach_x, approach_y, into_yaw):
                self.floor_change_hall_deadline = (
                    rospy.Time.now() + rospy.Duration(navigation_timeout)
                )
                self._set_floor_change_phase(
                    "TO_HALL", "navigate_to_elevator_hall"
                )
                return True
        code = "UNREACHABLE_HALL" if had_candidate else "NO_HALL"
        self._floor_change_fail(code, "all elevator hall candidates are unreachable")
        return False

    def _floor_change_goal_succeeded_set(self, succeeded):
        self._floor_change_goal_succeeded = bool(succeeded)

    def _service_outcome(self, now, kind):
        status, response = self._poll_service(now, kind)
        if status in ("pending", "missing"):
            return "pending", None
        if status == "timeout":
            return "timeout", None
        if status in ("error", "stale"):
            return "unavailable", response
        if hasattr(response, "accepted") and not bool(response.accepted):
            return "rejected", response
        return "success", response

    def _retry_hall_or_fail(self, failure_code, message):
        self.goal_id += 1
        self.move_base_client.cancel_all_goals()
        self.waiting_for_result = False
        self.current_goal = None
        self._stop_elevator_motion()
        self._invalidate_service()
        if (self.floor_change_retries + 1 < self.elevator_max_retries
                and self.elevator_hall_index < len(self.elevator_halls)):
            self.floor_change_retries += 1
            self._set_floor_change_phase("TO_HALL", "retry_elevator_hall")
            self._pick_elevator_hall_and_send()
            return
        self._floor_change_fail(failure_code, message)

    def _start_crossing(self, direction):
        if self.current_pose is None:
            code = "ENTER_FAILED" if direction > 0.0 else "EXIT_FAILED"
            self._floor_change_fail(code, "pose unavailable for elevator crossing")
            return
        self.floor_change_crossing_start = (
            self.current_pose.position.x,
            self.current_pose.position.y,
        )
        self.floor_change_crossing_direction = 1.0 if direction > 0.0 else -1.0
        self.floor_change_crossing_target_m = self.elevator_crossing_distance_m
        phase = "ENTER" if direction > 0.0 else "EXIT"
        self._set_floor_change_phase(phase, phase.lower() + "_elevator")
        self.floor_change_stage_deadline = rospy.Time.now() + rospy.Duration(
            self.elevator_crossing_timeout_s
        )

    def _advance_crossing(self, now):
        entering = self.floor_change_crossing_direction > 0.0
        failure_code = "ENTER_FAILED" if entering else "EXIT_FAILED"
        if self.current_pose is None or now > self.floor_change_stage_deadline:
            self._stop_elevator_motion()
            self._floor_change_fail(failure_code, "elevator crossing timed out")
            return
        start_x, start_y = self.floor_change_crossing_start
        progress = math.hypot(
            self.current_pose.position.x - start_x,
            self.current_pose.position.y - start_y,
        )
        if progress >= self.floor_change_crossing_target_m:
            self._stop_elevator_motion()
            if entering:
                self._set_floor_change_phase("CLOSE_CURRENT_START")
            else:
                self._set_floor_change_phase("CLEAR_COSTMAP_START")
            return
        scan_fresh = (
            self.last_scan_time != rospy.Time(0)
            and (now - self.last_scan_time).to_sec() <= self.input_timeout
        )
        if not scan_fresh:
            self._stop_elevator_motion()
            self._floor_change_fail(failure_code, "laser scan is stale")
            return
        window = self._scan_window(self.latest_scan, backward=not entering)
        finite = window[np.isfinite(window)]
        clearance = float(np.min(finite)) if finite.size else float("inf")
        if clearance <= self.elevator_crossing_clearance_m:
            self._stop_elevator_motion()
            if progress < self.elevator_crossing_min_progress_m:
                self._floor_change_fail(
                    failure_code, "obstacle blocked elevator crossing"
                )
            elif entering:
                self._set_floor_change_phase("CLOSE_CURRENT_START")
            else:
                self._set_floor_change_phase("CLEAR_COSTMAP_START")
            return
        command = Twist()
        command.linear.x = (
            self.floor_change_crossing_direction
            * self.elevator_crossing_speed_mps
        )
        self.elevator_cmd_pub.publish(command)

    def _complete_exploration(self, reason):
        """发布探索收敛事件并进入 COMPLETE（供多楼层结束时使用）。"""
        if not self.complete_published:
            self.complete_published = True
            self.complete_pub.publish(Bool(data=True))
            rospy.loginfo("[exploration] Exploration complete: %s", reason)
        self._set_state("COMPLETE", reason)

    def _advance_floor_change(self, now):
        if now > self.floor_change_deadline:
            code = (
                "UNREACHABLE_HALL"
                if self.floor_change_step == "TO_HALL"
                else "MAP_NOT_STABLE"
                if self.floor_change_step == "WAIT_STABLE"
                else "SERVICE_TIMEOUT"
            )
            self._floor_change_fail(code, "overall floor transit deadline exceeded")
            return
        step = self.floor_change_step

        if step == "TO_HALL":
            if self.waiting_for_result:
                if now > self.floor_change_hall_deadline:
                    self.goal_id += 1
                    self.move_base_client.cancel_goal()
                    self.waiting_for_result = False
                    self.current_goal = None
                    self._retry_hall_or_fail(
                        "UNREACHABLE_HALL", "hall navigation timed out"
                    )
                return
            if self._floor_change_goal_succeeded is None:
                return
            if not self._floor_change_goal_succeeded:
                self._retry_hall_or_fail(
                    "UNREACHABLE_HALL", "hall navigation failed"
                )
                return
            self.elevator_hall_found = self.floor_change_hall_point
            self._set_floor_change_phase("OPEN_CURRENT_START")
            return

        if step == "OPEN_CURRENT_START":
            if self._submit_service(
                    "open_current",
                    lambda: self._door_request(self.floor_change_start_floor, True)):
                self._set_floor_change_phase("OPEN_CURRENT_WAIT")
            return
        if step == "OPEN_CURRENT_WAIT":
            outcome, response = self._service_outcome(now, "open_current")
            if outcome == "pending":
                return
            if outcome != "success":
                code = "SERVICE_TIMEOUT" if outcome == "timeout" else (
                    "SERVICE_REJECTED" if outcome == "rejected"
                    else "SERVICE_UNAVAILABLE"
                )
                detail = getattr(response, "message", str(response or outcome))
                self._floor_change_fail(code, "open current door: " + detail)
                return
            if self.hall_validation_required:
                self._set_floor_change_phase("CAPTURE_OPEN_SCAN")
            else:
                self._start_crossing(+1.0)
            return

        if step == "CAPTURE_OPEN_SCAN":
            if (now - self.floor_change_phase_started).to_sec() < self.elevator_scan_settle_s:
                return
            if (self.last_scan_time < self.floor_change_phase_started
                    or self.latest_scan is None):
                if (now - self.floor_change_phase_started).to_sec() > max(
                        2.0, self.elevator_scan_settle_s + 1.0):
                    self._retry_hall_or_fail("NO_HALL", "no fresh open-door scan")
                return
            self.floor_change_open_scan = self._scan_window(self.latest_scan)
            self.floor_change_open_scan_stamp = self.last_scan_time
            if self.floor_change_open_scan.size < 5:
                self._retry_hall_or_fail("NO_HALL", "open-door scan window is empty")
                return
            self._set_floor_change_phase("VALIDATE_CLOSE_START")
            return

        if step == "VALIDATE_CLOSE_START":
            if self._submit_service(
                    "validate_close",
                    lambda: self._door_request(self.floor_change_start_floor, False)):
                self._set_floor_change_phase("VALIDATE_CLOSE_WAIT")
            return
        if step == "VALIDATE_CLOSE_WAIT":
            outcome, response = self._service_outcome(now, "validate_close")
            if outcome == "pending":
                return
            if outcome != "success":
                code = "SERVICE_TIMEOUT" if outcome == "timeout" else (
                    "SERVICE_REJECTED" if outcome == "rejected"
                    else "SERVICE_UNAVAILABLE"
                )
                self._floor_change_fail(
                    code, "close door for hall validation: "
                    + getattr(response, "message", str(response or outcome))
                )
                return
            self._set_floor_change_phase("CAPTURE_CLOSED_SCAN")
            return

        if step == "CAPTURE_CLOSED_SCAN":
            if (now - self.floor_change_phase_started).to_sec() < self.elevator_scan_settle_s:
                return
            if self.last_scan_time <= self.floor_change_open_scan_stamp:
                if (now - self.floor_change_phase_started).to_sec() > max(
                        2.0, self.elevator_scan_settle_s + 1.0):
                    self._floor_change_fail(
                        "SERVICE_UNAVAILABLE", "no fresh closed-door scan"
                    )
                return
            closed_scan = self._scan_window(self.latest_scan)
            self._hall_validation_passed = scan_door_changed(
                self.floor_change_open_scan,
                closed_scan,
                self.elevator_door_change_threshold_m,
                self.elevator_door_changed_fraction,
            )
            self._set_floor_change_phase("REOPEN_CURRENT_START")
            return

        if step == "REOPEN_CURRENT_START":
            if self._submit_service(
                    "reopen_current",
                    lambda: self._door_request(self.floor_change_start_floor, True)):
                self._set_floor_change_phase("REOPEN_CURRENT_WAIT")
            return
        if step == "REOPEN_CURRENT_WAIT":
            outcome, response = self._service_outcome(now, "reopen_current")
            if outcome == "pending":
                return
            if outcome != "success":
                code = "SERVICE_TIMEOUT" if outcome == "timeout" else (
                    "SERVICE_REJECTED" if outcome == "rejected"
                    else "SERVICE_UNAVAILABLE"
                )
                self._floor_change_fail(
                    code, "reopen validated hall door: "
                    + getattr(response, "message", str(response or outcome))
                )
                return
            if not self._hall_validation_passed:
                self._retry_hall_or_fail(
                    "NO_HALL", "door motion did not change the local scan"
                )
                return
            self._start_crossing(+1.0)
            return

        if step in ("ENTER", "EXIT"):
            self._advance_crossing(now)
            return

        if step == "CLOSE_CURRENT_START":
            if self._submit_service(
                    "close_current",
                    lambda: self._door_request(self.floor_change_start_floor, False)):
                self._set_floor_change_phase("CLOSE_CURRENT_WAIT")
            return
        if step == "CLOSE_CURRENT_WAIT":
            outcome, response = self._service_outcome(now, "close_current")
            if outcome == "pending":
                return
            if outcome != "success":
                code = "SERVICE_TIMEOUT" if outcome == "timeout" else (
                    "SERVICE_REJECTED" if outcome == "rejected"
                    else "SERVICE_UNAVAILABLE"
                )
                self._floor_change_fail(
                    code, "close current floor door: "
                    + getattr(response, "message", str(response or outcome))
                )
                return
            self._set_floor_change_phase("CALL_TARGET_START")
            return

        if step == "CALL_TARGET_START":
            if self._submit_service(
                    "call_target",
                    lambda: self._call_elevator_request(
                        self.floor_change_target, True)):
                self._set_floor_change_phase("CALL_TARGET_WAIT")
            return
        if step == "CALL_TARGET_WAIT":
            outcome, response = self._service_outcome(now, "call_target")
            if outcome == "pending":
                return
            if outcome != "success":
                code = "SERVICE_TIMEOUT" if outcome == "timeout" else (
                    "SERVICE_REJECTED" if outcome == "rejected"
                    else "SERVICE_UNAVAILABLE"
                )
                self._floor_change_fail(
                    code, "call target floor: "
                    + getattr(response, "message", str(response or outcome))
                )
                return
            reached_floor = int(response.current_floor)
            if reached_floor != self.floor_change_target:
                self._floor_change_fail(
                    "FLOOR_MISMATCH",
                    "elevator reported floor %d, expected %d" % (
                        reached_floor, self.floor_change_target
                    ),
                )
                return
            self.elevator_reported_floor = reached_floor
            self._set_floor_change_phase("SWITCH_FLOOR_START")
            return

        if step == "SWITCH_FLOOR_START":
            if self._submit_service("switch_floor", self._switch_floor_request):
                self._set_floor_change_phase("SWITCH_FLOOR_WAIT")
            return
        if step == "SWITCH_FLOOR_WAIT":
            outcome, response = self._service_outcome(now, "switch_floor")
            if outcome == "pending":
                return
            if outcome != "success" or not bool(getattr(response, "success", False)):
                code = "SERVICE_TIMEOUT" if outcome == "timeout" else "MAP_NOT_STABLE"
                self._floor_change_fail(
                    code, "switch floor map: "
                    + getattr(response, "message", str(response or outcome))
                )
                return
            self.floor_change_expected_epoch = int(response.map_epoch)
            if self.floor_change_expected_epoch <= self.floor_change_start_epoch:
                self._floor_change_fail(
                    "STALE_EPOCH", "floor switch did not advance map epoch"
                )
                return
            if self.floor_change_exit_to_hall:
                self._start_crossing(-1.0)
            else:
                self._set_floor_change_phase("CLEAR_COSTMAP_START")
            return

        if step == "CLEAR_COSTMAP_START":
            self._stop_elevator_motion()
            if self._submit_service("clear_costmaps", self._clear_costmaps_request):
                self._set_floor_change_phase("CLEAR_COSTMAP_WAIT")
            return
        if step == "CLEAR_COSTMAP_WAIT":
            outcome, response = self._service_outcome(now, "clear_costmaps")
            if outcome == "pending":
                return
            if outcome != "success":
                code = "SERVICE_TIMEOUT" if outcome == "timeout" else "MAP_NOT_STABLE"
                self._floor_change_fail(
                    code, "costmap reset failed: " + str(response or outcome)
                )
                return
            self.floor_change_stable_since = rospy.Time(0)
            self._set_floor_change_phase("WAIT_STABLE", "wait_mapping_stable")
            return

        if step == "WAIT_STABLE":
            floor_ok = self.current_floor == self.floor_change_target
            epoch_ok = self.map_epoch >= self.floor_change_expected_epoch
            version_ok = self.current_map_version >= (
                self.floor_change_start_target_version
                + self.floor_min_new_map_versions
            )
            stable_now = (
                floor_ok and epoch_ok and version_ok
                and self.mapping_ready and self.mapping_stable
                and not self.mapping_transitioning and self.nav_ready
            )
            if not stable_now:
                self.floor_change_stable_since = rospy.Time(0)
                return
            if self.floor_change_stable_since == rospy.Time(0):
                self.floor_change_stable_since = now
                return
            if (now - self.floor_change_stable_since).to_sec() >= self.floor_map_stable_time_s:
                self._floor_change_done()
            return

    def _cancel_floor_change(self, failure_code="CANCELED", message="canceled"):
        self._floor_change_fail(failure_code, message)

    def _floor_change_fail(self, failure_code, message):
        rospy.logwarn(
            "[exploration] floor transit failed [%s]: %s",
            failure_code, message,
        )
        self.goal_id += 1
        self.move_base_client.cancel_all_goals()
        self.waiting_for_result = False
        self.current_goal = None
        self._stop_elevator_motion()
        self._invalidate_service()
        failed_step = self.floor_change_step
        self.floor_change_active = False
        self.floor_change_step = None
        self._record_floor_change_result(False, failure_code, message)
        self.floor_change_gave_up_count += 1
        self.floor_change_retry_after = rospy.Time.now() + rospy.Duration(
            self.map_stable_time
        )
        fatal_steps = {
            "ENTER", "CLOSE_CURRENT_START", "CLOSE_CURRENT_WAIT",
            "CALL_TARGET_START", "CALL_TARGET_WAIT", "SWITCH_FLOOR_START",
            "SWITCH_FLOOR_WAIT", "EXIT", "CLEAR_COSTMAP_START",
            "CLEAR_COSTMAP_WAIT", "WAIT_STABLE",
        }
        if not self.floor_change_external and failed_step in fatal_steps:
            self.floor_transit_fatal = True
            self._set_state("FAILED", "floor_change_failed:" + failure_code)
        else:
            self._set_state("WAITING", "floor_change_failed:" + failure_code)
        self.last_goal_time = rospy.Time.now()

    def _floor_change_done(self):
        rospy.loginfo(
            "[exploration] floor change to %d done; visited=%s",
            self.floor_change_target, sorted(self.visited_floors),
        )
        final_target = int(self.floor_change_final_target)
        external = bool(self.floor_change_external)
        exit_to_hall = bool(self.floor_change_exit_to_hall)
        self._restore_current_floor_runtime()
        self.elevator_halls = []
        self.elevator_hall_found = None
        self.no_reachable_frontier_cycles = 0
        self.floor_no_frontier_since = rospy.Time(0)
        self.last_significant_map_change = rospy.Time.now()
        self.retry_count = 0
        self.floor_change_gave_up_count = 0
        if self.current_floor != final_target:
            # A transfer floor is stable now; recompute the next public ride.
            self.floor_change_active = False
            self.floor_change_step = None
            if not self._begin_floor_change(
                    final_target, external=external,
                    exit_to_hall=exit_to_hall):
                if self.floor_change_result is None:
                    self._record_floor_change_result(
                        False, "SERVICE_REJECTED", "no transfer route"
                    )
            return
        self.floor_change_active = False
        self.floor_change_step = None
        self._record_floor_change_result(True, "", "target floor reached and stable")
        self._set_state("EXPLORE_FLOOR", "new_floor_reached")

    def _transit_timer_callback(self, _event):
        with self.state_lock:
            if not self.floor_change_active:
                return
            if self.floor_change_step in {
                    "ENTER", "CLOSE_CURRENT_START", "CLOSE_CURRENT_WAIT",
                    "CALL_TARGET_START", "CALL_TARGET_WAIT",
                    "SWITCH_FLOOR_START", "SWITCH_FLOOR_WAIT", "EXIT",
                    "CLEAR_COSTMAP_START", "CLEAR_COSTMAP_WAIT"}:
                self._publish_mapping_pause()
            try:
                self._advance_floor_change(rospy.Time.now())
            except Exception as exc:
                rospy.logerr("[exploration] transit state machine exception: %s", exc)
                self._floor_change_fail(
                    "SERVICE_UNAVAILABLE", "transit state exception: %s" % exc
                )

    def _execute_transit_action(self, goal):
        target_floor = int(goal.target_floor)
        with self.state_lock:
            if self.floor_change_active:
                result = TransitFloorResult(
                    success=False,
                    reached_floor=int(self.current_floor),
                    map_epoch=int(self.map_epoch),
                    failure_code="SERVICE_REJECTED",
                    message="another floor transit is active",
                )
                self.transit_server.set_aborted(result)
                return
            self._begin_floor_change(
                target_floor, external=True, exit_to_hall=bool(goal.exit_to_hall)
            )

        rate = rospy.Rate(10.0)
        progress_by_phase = {
            "TO_HALL": 0.10,
            "OPEN_CURRENT_START": 0.20,
            "OPEN_CURRENT_WAIT": 0.22,
            "CAPTURE_OPEN_SCAN": 0.25,
            "VALIDATE_CLOSE_START": 0.27,
            "VALIDATE_CLOSE_WAIT": 0.29,
            "CAPTURE_CLOSED_SCAN": 0.31,
            "REOPEN_CURRENT_START": 0.33,
            "REOPEN_CURRENT_WAIT": 0.35,
            "ENTER": 0.45,
            "CLOSE_CURRENT_START": 0.50,
            "CLOSE_CURRENT_WAIT": 0.52,
            "CALL_TARGET_START": 0.55,
            "CALL_TARGET_WAIT": 0.65,
            "SWITCH_FLOOR_START": 0.70,
            "SWITCH_FLOOR_WAIT": 0.75,
            "EXIT": 0.82,
            "CLEAR_COSTMAP_START": 0.86,
            "CLEAR_COSTMAP_WAIT": 0.88,
            "WAIT_STABLE": 0.92,
        }
        while not rospy.is_shutdown():
            with self.state_lock:
                if self.transit_server.is_preempt_requested():
                    self._cancel_floor_change("CANCELED", "transit action canceled")
                snapshot = dict(self.floor_change_result or {})
                phase = str(self.floor_change_step or "DONE")
                feedback = TransitFloorFeedback(
                    phase=phase,
                    current_floor=int(self.current_floor),
                    progress=float(progress_by_phase.get(phase, 1.0)),
                    map_epoch=int(self.map_epoch),
                )
            self.transit_server.publish_feedback(feedback)
            if snapshot:
                result = TransitFloorResult(**snapshot)
                if snapshot["success"]:
                    self.transit_server.set_succeeded(result)
                elif snapshot["failure_code"] == "CANCELED":
                    self.transit_server.set_preempted(result)
                else:
                    self.transit_server.set_aborted(result)
                return
            rate.sleep()

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = ExplorationPlanner()
        node.run()
    except rospy.ROSInterruptException:
        pass
