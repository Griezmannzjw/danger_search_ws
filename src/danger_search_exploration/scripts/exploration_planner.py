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
from dataclasses import dataclass
import cv2
import numpy as np
import tf2_ros
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
from std_msgs.msg import Bool, Header, Int32, String
from nav_msgs.srv import GetPlan
from danger_search_common.msg import (
    FloorOccupancyGrid,
    MappingStatus,
    NavigationHealth,
    RecoveryEvent,
    TransitFloorAction,
    TransitFloorFeedback,
    TransitFloorResult,
)
from danger_search_common.srv import SwitchFloor
from danger_search_common.short_range_safety import swept_footprint_obstacle

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


BOUNDED_FLOOR_EXHAUSTION_REASONS = frozenset((
    "all_frontiers_unreachable_or_blacklisted",
    "frontier_already_in_observation_range",
))


def validate_fixed_elevator_hall_mode(
        enabled, competition_mode, run_profile, x, y, into_yaw):
    """Validate the explicit Seed-42 hall override and return its values."""
    enabled = bool(enabled)
    values = tuple(float(value) for value in (x, y, into_yaw))
    if enabled and (
            bool(competition_mode) or str(run_profile) != "simulation_truth"):
        raise ValueError(
            "fixed elevator hall is restricted to the simulation_truth profile"
        )
    if enabled and not all(math.isfinite(value) for value in values):
        raise ValueError("fixed elevator hall coordinates must be finite")
    return enabled, values


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
    door_initial_open_by_floor = {}
    for door in public_scene.get("door_ids", []):
        if door.get("kind") == "elevator":
            floor = int(door["floor_index"])
            door_id = str(door["id"])
            door_by_floor[floor] = door_id
            door_initial_open_by_floor[floor] = bool(door.get("initial_open", False))
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
        "door_initial_open_by_floor": door_initial_open_by_floor,
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


@dataclass
class ElevatorHallCandidate:
    """One sensor-derived elevator entrance hypothesis in the map frame."""

    x: float
    y: float
    into_yaw: float
    score: float = 0.0
    source: str = "geometry"
    confidence: float = 0.0
    validated: bool = False
    path_length: float = float("inf")
    approach_x: float = float("nan")
    approach_y: float = float("nan")

    def __iter__(self):
        return iter((self.x, self.y, self.into_yaw))

    def __getitem__(self, index):
        return (self.x, self.y, self.into_yaw)[index]

    def __len__(self):
        return 3

    def hall(self):
        return (float(self.x), float(self.y), float(self.into_yaw))


def _circular_true_clusters(mask):
    """Return contiguous true-index clusters, merging a 360-degree seam."""
    mask = np.asarray(mask, dtype=bool)
    if not mask.size or not np.any(mask):
        return []
    transitions = np.flatnonzero(mask & ~np.roll(mask, 1))
    if not transitions.size:  # Every beam changed; not a localized door.
        return [np.arange(mask.size, dtype=np.int32)]
    clusters = []
    for start in transitions:
        values = []
        index = int(start)
        while mask[index]:
            values.append(index)
            index = (index + 1) % mask.size
            if index == start:
                break
        clusters.append(np.asarray(values, dtype=np.int32))
    return clusters


def _bridge_circular_gaps(mask, maximum_gap_beams):
    """Bridge short no-return gaps in a circular projected lidar scan."""
    bridged = np.asarray(mask, dtype=bool).copy()
    maximum_gap_beams = int(maximum_gap_beams)
    if maximum_gap_beams <= 0 or not np.any(bridged):
        return bridged
    for gap in _circular_true_clusters(~bridged):
        if gap.size <= maximum_gap_beams:
            bridged[gap] = True
    return bridged


def localize_actuated_door(open_scans, closed_scans, angle_min,
                           angle_increment, range_min, range_max,
                           scan_to_map, robot_xy,
                           change_threshold_m=0.20,
                           minimum_median_change_m=0.25,
                           minimum_beams=5,
                           minimum_width_m=0.9,
                           maximum_width_m=1.8,
                           maximum_line_rms_m=0.08,
                           ambiguity_ratio=1.5,
                           maximum_cluster_gap_beams=5):
    """Localize a door from median open/closed 360-degree lidar scans.

    Invalid open-door returns are represented by ``range_max`` only for beams
    where the closed scan has a valid hit.  This preserves the expected
    closing-door distance reduction without treating missing closed data as
    evidence.  ``scan_to_map`` is a planar ``(x, y, yaw)`` transform.
    """
    opened = np.asarray(open_scans, dtype=np.float32)
    closed = np.asarray(closed_scans, dtype=np.float32)
    if (opened.ndim != 2 or closed.ndim != 2
            or opened.shape != closed.shape or opened.shape[0] < 1
            or opened.shape[1] < int(minimum_beams)
            or not math.isfinite(float(angle_increment))
            or abs(float(angle_increment)) < 1e-9
            or abs(float(angle_increment)) * opened.shape[1]
            < 2.0 * math.pi - 2.0 * abs(float(angle_increment))):
        return None

    opened_median = np.ma.median(
        np.ma.masked_invalid(opened), axis=0
    ).filled(np.nan)
    closed_median = np.ma.median(
        np.ma.masked_invalid(closed), axis=0
    ).filled(np.nan)
    with np.errstate(invalid="ignore"):
        closed_valid = (
            np.isfinite(closed_median)
            & (closed_median >= float(range_min))
            & (closed_median <= float(range_max))
        )
        opened_valid = (
            np.isfinite(opened_median)
            & (opened_median >= float(range_min))
            & (opened_median <= float(range_max))
        )
    effective_open = np.where(opened_valid, opened_median, float(range_max))
    reductions = effective_open - closed_median
    with np.errstate(invalid="ignore"):
        changed = closed_valid & (reductions >= float(change_threshold_m))

    transform_x, transform_y, transform_yaw = (
        float(value) for value in scan_to_map
    )
    robot_x, robot_y = (float(value) for value in robot_xy)
    detections = []
    clustered = _bridge_circular_gaps(changed, maximum_cluster_gap_beams)
    for cluster_indices in _circular_true_clusters(clustered):
        indices = cluster_indices[changed[cluster_indices]]
        if indices.size < int(minimum_beams):
            continue
        angles = float(angle_min) + indices * float(angle_increment)
        ranges = closed_median[indices].astype(np.float64)
        scan_points = np.column_stack((ranges * np.cos(angles),
                                       ranges * np.sin(angles)))
        cosine = math.cos(transform_yaw)
        sine = math.sin(transform_yaw)
        points = np.column_stack((
            transform_x + cosine * scan_points[:, 0] - sine * scan_points[:, 1],
            transform_y + sine * scan_points[:, 0] + cosine * scan_points[:, 1],
        ))
        center = np.mean(points, axis=0)
        centered = points - center
        _singular, _values, vectors = np.linalg.svd(centered, full_matrices=False)
        tangent = vectors[0]
        projections = centered.dot(tangent)
        normal_offsets = centered.dot(np.array((-tangent[1], tangent[0])))
        width = float(np.max(projections) - np.min(projections))
        line_rms = float(np.sqrt(np.mean(normal_offsets ** 2)))
        median_change = float(np.median(reductions[indices]))
        if not (float(minimum_width_m) <= width <= float(maximum_width_m)):
            continue
        if line_rms > float(maximum_line_rms_m):
            continue
        if median_change < float(minimum_median_change_m):
            continue

        segment_midpoint = center + tangent * 0.5 * (
            float(np.min(projections)) + float(np.max(projections))
        )
        # The inward normal points away from the robot's hall-side pose.
        normal = np.array((-tangent[1], tangent[0]))
        away = segment_midpoint - np.array((robot_x, robot_y))
        if float(np.dot(normal, away)) < 0.0:
            normal = -normal
        into_yaw = math.atan2(float(normal[1]), float(normal[0]))
        evidence = float(indices.size) * median_change / max(
            1e-3, 1.0 + 10.0 * line_rms
        )
        detections.append((
            evidence,
            ElevatorHallCandidate(
                x=float(segment_midpoint[0]),
                y=float(segment_midpoint[1]),
                into_yaw=into_yaw,
                score=1.0,
                source="door_motion",
                confidence=1.0,
                validated=True,
            ),
        ))
    if not detections:
        return None
    detections.sort(key=lambda item: item[0], reverse=True)
    if (len(detections) > 1
            and detections[0][0] < float(ambiguity_ratio) * detections[1][0]):
        return None
    return detections[0][1]


def entrance_boundary_anchor_from_pose(pose, floor_id):
    """Return a start-pose anchor for the entrance-side frontier guard.

    The guard deliberately uses only the localization pose that is already
    available to exploration.  It never consumes a building layout or an
    entrance truth pose.  ``yaw`` defines the forward half-plane at the point
    exploration is started.
    """
    position = getattr(pose, "position", None)
    orientation = getattr(pose, "orientation", None)
    if position is None or orientation is None:
        raise ValueError("pose must contain position and orientation")
    values = (
        position.x, position.y,
        orientation.x, orientation.y, orientation.z, orientation.w,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("entrance boundary pose must be finite")
    qx, qy, qz, qw = (float(value) for value in values[2:])
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-6:
        raise ValueError("entrance boundary orientation is invalid")
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    return (float(position.x), float(position.y), yaw, int(floor_id))


def entrance_boundary_allows_goal(anchor, goal_x, goal_y, allowance_m):
    """Whether a goal is not behind an entrance start-pose boundary."""
    if anchor is None:
        return True
    if len(anchor) != 4:
        raise ValueError("entrance boundary anchor must have four fields")
    anchor_x, anchor_y, anchor_yaw, _floor_id = anchor
    values = (anchor_x, anchor_y, anchor_yaw, goal_x, goal_y, allowance_m)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("entrance boundary values must be finite")
    allowance_m = float(allowance_m)
    if allowance_m < 0.0:
        raise ValueError("entrance boundary allowance must be non-negative")
    forward_m = (
        math.cos(float(anchor_yaw)) * (float(goal_x) - float(anchor_x))
        + math.sin(float(anchor_yaw)) * (float(goal_y) - float(anchor_y))
    )
    return forward_m >= -allowance_m


def path_prefix_goal(points, maximum_length_m):
    """Return a pose no farther than ``maximum_length_m`` along a path.

    Long Navfn detours are split into bounded receding-horizon waypoints.  The
    next planning cycle can then use the newly observed map instead of holding
    one stale frontier action for the complete building-scale detour.
    """
    maximum_length_m = float(maximum_length_m)
    if not math.isfinite(maximum_length_m) or maximum_length_m <= 0.0:
        raise ValueError("maximum path prefix length must be positive")
    normalized = [tuple(float(value) for value in point[:2]) for point in points]
    if not normalized or not all(
            len(point) == 2 and all(math.isfinite(value) for value in point)
            for point in normalized):
        raise ValueError("path points must be finite x/y pairs")
    if len(normalized) == 1:
        return normalized[0][0], normalized[0][1], 0.0, 0.0, False

    traversed = 0.0
    last_yaw = 0.0
    for index in range(1, len(normalized)):
        start_x, start_y = normalized[index - 1]
        end_x, end_y = normalized[index]
        delta_x = end_x - start_x
        delta_y = end_y - start_y
        segment = math.hypot(delta_x, delta_y)
        if segment <= 1e-9:
            continue
        last_yaw = math.atan2(delta_y, delta_x)
        if traversed + segment >= maximum_length_m:
            ratio = (maximum_length_m - traversed) / segment
            return (
                start_x + ratio * delta_x,
                start_y + ratio * delta_y,
                last_yaw,
                maximum_length_m,
                True,
            )
        traversed += segment
    end_x, end_y = normalized[-1]
    return end_x, end_y, last_yaw, traversed, False


def bounded_navigation_timeout(
        path_length_m, maximum_timeout_s, base_timeout_s,
        seconds_per_meter, minimum_timeout_s):
    """Allocate a deterministic action timeout from the dispatched path."""
    values = tuple(float(value) for value in (
        path_length_m,
        maximum_timeout_s,
        base_timeout_s,
        seconds_per_meter,
        minimum_timeout_s,
    ))
    if (not all(math.isfinite(value) for value in values)
            or path_length_m < 0.0 or maximum_timeout_s <= 0.0
            or base_timeout_s < 0.0 or seconds_per_meter <= 0.0
            or minimum_timeout_s <= 0.0
            or minimum_timeout_s > maximum_timeout_s):
        raise ValueError("navigation timeout parameters are invalid")
    estimate = base_timeout_s + seconds_per_meter * path_length_m
    return min(maximum_timeout_s, max(minimum_timeout_s, estimate))


def map_context_is_committed(mapping_context, consumer_context):
    """Whether a same-epoch consumer snapshot is safe to use.

    MappingStatus, the active-map envelope and NavigationHealth travel over
    independent ROS connections.  Requiring their content versions to be
    equal at one instant can permanently starve planning while a live mapper
    keeps publishing.  Floor and epoch are coordinate-frame identity and must
    remain exact; a positive consumer version may lag the latest mapping
    status within that epoch, but may never lead it.
    """
    if mapping_context is None or consumer_context is None:
        return False
    try:
        mapping = tuple(int(value) for value in mapping_context)
        consumer = tuple(int(value) for value in consumer_context)
    except (TypeError, ValueError):
        return False
    if len(mapping) != 3 or len(consumer) != 3:
        return False
    return (
        mapping[:2] == consumer[:2]
        and mapping[2] >= 1
        and 1 <= consumer[2] <= mapping[2]
    )


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
        self.active_map_topic = rospy.get_param(
            "~active_map_topic", "/mapping/active_map"
        )
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
        self.run_profile = str(rospy.get_param(
            "~run_profile", rospy.get_param("/run_profile", "formal")
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
        # Straight-line order is only a cheap prefilter: the final ranking uses
        # Navfn path length.  Keep a sufficiently broad bounded pool so a
        # candidate just beyond a doorway/wall detour is not discarded before
        # its true path cost can be measured.
        self.max_frontier_candidates = int(
            rospy.get_param("~max_frontier_candidates", 64)
        )
        if self.max_frontier_candidates < 1:
            raise rospy.ROSInitException(
                "~max_frontier_candidates must be a positive integer"
            )
        self.goal_timeout = rospy.get_param("~goal_timeout", 60.0)
        self.max_frontier_goal_path_m = float(rospy.get_param(
            "~max_frontier_goal_path_m", 8.0
        ))
        self.goal_timeout_base_s = float(rospy.get_param(
            "~goal_timeout_base_s", 20.0
        ))
        self.goal_timeout_per_path_m = float(rospy.get_param(
            "~goal_timeout_per_path_m", 4.0
        ))
        self.goal_timeout_min_s = float(rospy.get_param(
            "~goal_timeout_min_s", 30.0
        ))
        timeout_values = (
            self.goal_timeout,
            self.max_frontier_goal_path_m,
            self.goal_timeout_base_s,
            self.goal_timeout_per_path_m,
            self.goal_timeout_min_s,
        )
        if (not all(math.isfinite(float(value)) for value in timeout_values)
                or self.goal_timeout <= 0.0
                or self.max_frontier_goal_path_m <= 0.0
                or self.goal_timeout_base_s < 0.0
                or self.goal_timeout_per_path_m <= 0.0
                or self.goal_timeout_min_s <= 0.0
                or self.goal_timeout_min_s > self.goal_timeout):
            raise rospy.ROSInitException(
                "frontier path horizon and timeout parameters are invalid"
            )
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
        self.min_goal_dispatch_distance_m = float(
            rospy.get_param("~min_goal_dispatch_distance_m", 0.45)
        )
        self.dependency_check_timeout = rospy.get_param("~dependency_check_timeout", 0.1)
        self.input_timeout = rospy.get_param("~input_timeout", 3.0)
        self.no_frontier_cycles_required = rospy.get_param("~no_frontier_cycles_required", 5)
        self.floor_no_frontier_hold_s = float(
            rospy.get_param("~floor_no_frontier_hold_s", 10.0)
        )
        self.floor_unreachable_hold_s = float(
            rospy.get_param("~floor_unreachable_hold_s", 30.0)
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
            rospy.get_param("~observation_target_distance", 0.30)
        )
        self.entrance_boundary_guard_enabled = bool(
            rospy.get_param("~entrance_boundary_guard_enabled", False)
        )
        self.entrance_boundary_allowance_m = float(
            rospy.get_param("~entrance_boundary_allowance_m", 1.0)
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
        self.transit_floor_action_name = rospy.get_param(
            "~transit_floor_action_name", "/danger_search/transit_floor"
        )
        self.elevator_cmd_topic = rospy.get_param(
            "~elevator_cmd_topic", "/danger_search/elevator_cmd_vel"
        )
        self.safety_stop_topic = rospy.get_param(
            "~safety_stop_topic", "/danger_search/safety_stop"
        )
        self.sent_cmd_topic = rospy.get_param(
            "~sent_cmd_topic", "/danger_search/cmd_vel_sent"
        )
        self.scan_topic = rospy.get_param("~scan_topic", "/localization/scan")
        self.elevator_id = rospy.get_param("~elevator_id", "elevator_main")
        self.elevator_door_prefix = rospy.get_param(
            "~elevator_door_prefix", "elevator_floor"
        )
        self.shaft_min_area_m2 = float(rospy.get_param("~shaft_min_area_m2", 4.0))
        self.shaft_max_area_m2 = float(rospy.get_param("~shaft_max_area_m2", 12.0))
        self.shaft_min_side_m = float(rospy.get_param("~shaft_min_side_m", 1.8))
        self.shaft_max_side_m = float(rospy.get_param("~shaft_max_side_m", 3.6))
        self.shaft_wall_support_min = float(rospy.get_param(
            "~shaft_wall_support_min", 0.70
        ))
        self.door_gap_min_width_m = float(rospy.get_param("~door_gap_min_width_m", 0.9))
        self.door_gap_max_width_m = float(rospy.get_param("~door_gap_max_width_m", 1.8))
        self.door_center_tolerance_fraction = float(rospy.get_param(
            "~door_center_tolerance_fraction", 0.25
        ))
        self.elevator_hall_min_score = float(rospy.get_param(
            "~elevator_hall_min_score", 0.75
        ))
        self.elevator_hall_min_versions = int(rospy.get_param(
            "~elevator_hall_min_versions", 3
        ))
        self.elevator_hall_min_duration_s = float(rospy.get_param(
            "~elevator_hall_min_duration_s", 2.0
        ))
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
            rospy.get_param("~elevator_crossing_timeout_s", 40.0)
        )
        self.elevator_crossing_speed_mps = float(
            rospy.get_param("~elevator_crossing_speed_mps", 0.40)
        )
        self.elevator_crossing_distance_m = float(
            rospy.get_param("~elevator_crossing_distance_m", 0.85)
        )
        self.elevator_crossing_min_progress_m = float(
            rospy.get_param("~elevator_crossing_min_progress_m", 0.8)
        )
        self.elevator_crossing_clearance_m = float(
            rospy.get_param("~elevator_crossing_clearance_m", 0.32)
        )
        self.elevator_exit_hall_side_margin_m = float(
            rospy.get_param("~elevator_exit_hall_side_margin_m", 0.05)
        )
        self.elevator_entry_cabin_side_margin_m = float(rospy.get_param(
            "~elevator_entry_cabin_side_margin_m", 0.43
        ))
        self.elevator_footprint_min_x = float(
            rospy.get_param("~elevator_footprint_min_x", -0.35)
        )
        self.elevator_footprint_max_x = float(
            rospy.get_param("~elevator_footprint_max_x", 0.30)
        )
        self.elevator_footprint_min_y = float(
            rospy.get_param("~elevator_footprint_min_y", -0.15)
        )
        self.elevator_footprint_max_y = float(
            rospy.get_param("~elevator_footprint_max_y", 0.15)
        )
        self.elevator_footprint_margin_m = float(
            rospy.get_param("~elevator_footprint_margin_m", 0.08)
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
        self.initial_hall_discovery_enabled = bool(rospy.get_param(
            "~initial_hall_discovery_enabled", True
        ))
        self.initial_hall_discovery_scan_count = int(rospy.get_param(
            "~initial_hall_discovery_scan_count", 5
        ))
        self.initial_hall_discovery_max_translation_m = float(rospy.get_param(
            "~initial_hall_discovery_max_translation_m", 0.03
        ))
        self.initial_hall_discovery_max_yaw_deg = float(rospy.get_param(
            "~initial_hall_discovery_max_yaw_deg", 1.0
        ))
        self.initial_hall_discovery_timeout_s = float(rospy.get_param(
            "~initial_hall_discovery_timeout_s", 60.0
        ))
        fixed_hall_enabled = bool(rospy.get_param(
            "~fixed_elevator_hall_enabled", False
        ))
        fixed_hall_values = (
            rospy.get_param("~fixed_elevator_hall_x", -2.40),
            rospy.get_param("~fixed_elevator_hall_y", -1.65),
            rospy.get_param("~fixed_elevator_hall_into_yaw", -math.pi / 2.0),
        )
        try:
            (self.fixed_elevator_hall_enabled,
             validated_fixed_hall) = validate_fixed_elevator_hall_mode(
                fixed_hall_enabled,
                self.competition_mode,
                self.run_profile,
                *fixed_hall_values,
            )
        except ValueError as exc:
            raise rospy.ROSInitException(str(exc))
        (self.fixed_elevator_hall_x,
         self.fixed_elevator_hall_y,
         self.fixed_elevator_hall_into_yaw) = validated_fixed_hall
        if not (
                self.shaft_min_area_m2 > 0.0
                and self.shaft_max_area_m2 >= self.shaft_min_area_m2
                and 0.0 < self.shaft_min_side_m <= self.shaft_max_side_m
                and 0.0 < self.shaft_wall_support_min <= 1.0
                and 0.0 < self.door_gap_min_width_m <= self.door_gap_max_width_m
                and 0.0 <= self.door_center_tolerance_fraction <= 0.5
                and 0.0 < self.elevator_hall_min_score <= 1.0
                and self.elevator_hall_min_versions >= 1
                and self.elevator_hall_min_duration_s >= 0.0
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
                and self.elevator_entry_cabin_side_margin_m > 0.0
                and self.elevator_footprint_min_x < self.elevator_footprint_max_x
                and self.elevator_footprint_min_y < self.elevator_footprint_max_y
                and self.elevator_footprint_margin_m >= 0.0
                and self.elevator_hall_navigation_max_s > 0.0
                and self.elevator_hall_nominal_speed_mps > 0.0
                and self.elevator_scan_settle_s >= 0.0
                and self.elevator_door_change_threshold_m > 0.0
                and 0.0 < self.elevator_door_changed_fraction <= 1.0
                and self.floor_min_new_map_versions >= 2
                and self.floor_no_frontier_hold_s >= 10.0
                and self.floor_unreachable_hold_s >= 10.0
                and self.floor_height_m > 0.0
                and self.initial_hall_discovery_scan_count >= 1
                and self.initial_hall_discovery_max_translation_m > 0.0
                and self.initial_hall_discovery_max_yaw_deg > 0.0
                and self.initial_hall_discovery_timeout_s > 0.0
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
            self.elevator_door_initial_open = dict(
                self.scene_topology["door_initial_open_by_floor"]
            )
            self.elevator_id = self.scene_topology["elevators"][0]["id"]
        else:
            self.served_floors = {0}
            self.elevator_door_ids = {}
            self.elevator_door_initial_open = {}

        if not (
                0.0 < self.observation_min_distance
                <= self.observation_target_distance
                <= self.observation_max_distance
                and math.isfinite(self.entrance_boundary_allowance_m)
                and self.entrance_boundary_allowance_m >= 0.0
                and self.min_goal_dispatch_distance_m > 0.0
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
        # A session-start anchor used only on its originating floor.  Other
        # floors must not be projected into this floor's world-XY half-plane.
        self.entrance_boundary_anchor = None
        self.current_map = None
        self.map_info = None
        self.map_data = None
        self.pending_active_map = None
        self.accepted_map_context = None
        self.accepted_map_load_identity = None
        self.last_legacy_map_time = rospy.Time(0)
        self.mapping_ready = False
        self.mapping_stable = False
        self.mapping_lost = True
        self.current_floor = 0
        self.nav_ready = False
        self.nav_has_active_goal = False
        self.nav_stuck = False
        self.nav_failure_code = "NONE"
        self.nav_failure_detail = ""
        self.nav_active_goal_id = ""
        self.nav_floor = 0
        self.nav_map_epoch = 0
        self.nav_map_version = 0
        self.nav_transitioning = True
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
        self.selected_goal_metrics = None
        self.active_goal_timeout_s = float(self.goal_timeout)
        self.failed_goals = []
        self.successful_goals = []
        self.last_successful_goal = None
        self.trap_blacklist = {}
        self.last_recovery_event_id = 0
        self.last_recovery_trigger_id = 0
        self.last_recovery_stuck_pose = None
        self.last_recovery_goal_id = ""
        self.navigation_goal_sent_at = rospy.Time(0)
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

        # ========== 多楼层状态 ==========
        self.current_floor = 0
        self.visited_floors = set()
        self.completed_floors = set()
        self.floor_runtime = {}
        self.coverage_debt_by_floor = {}
        self.elevator_hall_bindings = {}
        self.elevator_hall_tracks = {}
        self.elevator_hall_last_observed_version = None
        self.floor_no_frontier_since = rospy.Time(0)
        self.floor_unreachable_since = rospy.Time(0)
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
        self.floor_change_hall_candidate = None
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
        self.floor_change_stop_deadline = rospy.Time(0)
        self._pending_floor_failure = None
        self.floor_transit_fatal = False
        self.active_elevator_id = self.elevator_id
        self._service_future = None
        self._service_generation = 0
        self._service_future_generation = 0
        self._service_kind = ""
        self._service_executor = concurrent.futures.ThreadPoolExecutor(max_workers=4)
        self.latest_scan = None
        self.last_scan_time = rospy.Time(0)
        self.initial_hall_discovery_active = False
        self.initial_hall_discovery_step = "IDLE"
        self.initial_hall_discovery_started = rospy.Time(0)
        self.initial_hall_discovery_pose = None
        self.initial_hall_discovery_scan_geometry = None
        self.initial_hall_discovery_open_scans = []
        self.initial_hall_discovery_closed_scans = []
        self.initial_hall_discovery_last_scan_stamp = rospy.Time(0)
        self.initial_hall_discovery_settle_until = rospy.Time(0)
        self.initial_hall_discovery_restore_required = False
        self.initial_hall_discovery_failure = ""
        self.initial_hall_discovery_zero_since = rospy.Time(0)
        self.initial_hall_discovery_door_held_closed = False
        self.initial_hall_discovery_abort_deadline = rospy.Time(0)
        self.safety_stop_active = False
        self.last_sent_command = Twist()
        self.last_sent_command_time = rospy.Time(0)
        self.elevator_client = None
        self.door_client = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(60.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ========== Action客户端 ==========
        self.move_base_client = actionlib.SimpleActionClient(
            self.move_base_action_name, MoveBaseAction
        )

        # ========== 服务客户端 ==========
        self.make_plan_client = rospy.ServiceProxy(self.make_plan_service, GetPlan)
        self.switch_floor_client = rospy.ServiceProxy(
            self.switch_floor_service, SwitchFloor
        )
        # ========== 订阅者 ==========
        self.pose_sub = rospy.Subscriber(
            self.pose_topic, PoseWithCovarianceStamped, self.pose_callback
        )
        self.map_sub = rospy.Subscriber(
            self.map_topic, OccupancyGrid, self.map_callback
        )
        self.active_map_sub = rospy.Subscriber(
            self.active_map_topic,
            FloorOccupancyGrid,
            self.active_map_callback,
            queue_size=1,
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
        self.safety_stop_sub = rospy.Subscriber(
            self.safety_stop_topic, Bool, self._safety_stop_callback, queue_size=2
        )
        self.sent_cmd_sub = rospy.Subscriber(
            self.sent_cmd_topic, Twist, self._sent_cmd_callback, queue_size=10
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
        """Observe the legacy map; multi-floor planning uses active_map only."""
        expected_size = msg.info.width * msg.info.height
        if (msg.header.frame_id != self.map_frame or msg.info.resolution <= 0
                or expected_size == 0 or len(msg.data) != expected_size):
            rospy.logwarn_throttle(5, "[exploration] Ignoring invalid occupancy grid")
            return
        self.last_legacy_map_time = rospy.Time.now()
        if getattr(self, "multifloor_enabled", False):
            return
        self._apply_occupancy_grid(
            msg, (int(getattr(self, "current_floor", 0)),
                  int(getattr(self, "map_epoch", 0)),
                  int(getattr(self, "current_map_version", 0)))
        )

    @staticmethod
    def _map_load_identity(message):
        load = getattr(message.info, "map_load_time", None)
        origin = message.info.origin
        return (
            int(getattr(load, "secs", 0)), int(getattr(load, "nsecs", 0)),
            int(message.info.width),
            int(message.info.height), round(float(message.info.resolution), 9),
            round(float(origin.position.x), 6),
            round(float(origin.position.y), 6),
            round(float(getattr(origin.position, "z", 0.0)), 6),
            round(float(origin.orientation.x), 7),
            round(float(origin.orientation.y), 7),
            round(float(origin.orientation.z), 7),
            round(float(origin.orientation.w), 7),
        )

    def _invalidate_active_map(self, cancel_goal=False):
        self.current_map = None
        self.map_info = None
        self.map_data = None
        self.accepted_map_context = None
        self.accepted_map_load_identity = None
        self.last_map_time = rospy.Time(0)
        self._reachable_cache_key = None
        self._reachable_cache = None
        self._component_cache_key = None
        self._frontier_cache_key = None
        self._frontier_cluster_cache_key = None
        self._clearance_cache_key = None
        self._clearance_cache = None
        if cancel_goal and getattr(self, "waiting_for_result", False):
            self.goal_id += 1
            self.move_base_client.cancel_all_goals()
            self.waiting_for_result = False
            self.current_goal = None

    def active_map_callback(self, envelope):
        grid = envelope.occupancy_grid
        expected_size = grid.info.width * grid.info.height
        if (grid.header.frame_id != self.map_frame
                or envelope.header.frame_id != grid.header.frame_id
                or envelope.header.stamp != grid.header.stamp
                or grid.info.resolution <= 0.0 or expected_size == 0
                or len(grid.data) != expected_size):
            rospy.logwarn_throttle(
                2.0, "[exploration] ignoring malformed active map envelope"
            )
            return
        context = (
            int(envelope.floor_id), int(getattr(envelope, "map_epoch", 0)),
            int(envelope.map_version),
        )
        current = (
            int(self.current_floor), int(self.map_epoch),
            int(self.current_map_version),
        )
        if (context[1] < current[1]
                or (context[1] == current[1] and context[0] != current[0])
                or (self.accepted_map_context is not None
                    and context[:2] == self.accepted_map_context[:2]
                    and context[2] < self.accepted_map_context[2])):
            rospy.logwarn_throttle(
                2.0, "[exploration] ignoring stale active map context %s", context
            )
            return
        self.pending_active_map = envelope
        self._try_accept_pending_active_map()

    def _try_accept_pending_active_map(self):
        envelope = self.pending_active_map
        if envelope is None:
            return False
        context = (
            int(envelope.floor_id), int(getattr(envelope, "map_epoch", 0)),
            int(envelope.map_version),
        )
        expected = (
            int(self.current_floor), int(self.map_epoch),
            int(self.current_map_version),
        )
        if (not map_context_is_committed(expected, context)
                or self.mapping_transitioning
                or not self.mapping_ready or not self.mapping_stable):
            return False
        self._apply_occupancy_grid(envelope.occupancy_grid, context)
        self.pending_active_map = None
        return True

    def _apply_occupancy_grid(self, msg, context):
        new_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )
        load_identity = self._map_load_identity(msg)
        floor = int(context[0])
        if not hasattr(self, "floor_runtime"):
            self.floor_runtime = {}
        if not hasattr(self, "accepted_map_context"):
            self.accepted_map_context = None
        if not hasattr(self, "accepted_map_load_identity"):
            self.accepted_map_load_identity = None
        runtime = self.floor_runtime.get(floor, {})
        previous_identity = runtime.get("map_load_identity")
        previous_version = int(runtime.get("map_version", -1))
        if (self.accepted_map_context is not None
                and int(self.accepted_map_context[0]) == floor):
            previous_identity = (
                self.accepted_map_load_identity
                if previous_identity is None else previous_identity
            )
            previous_version = max(
                previous_version, int(self.accepted_map_context[2])
            )
        if ((previous_identity is not None and previous_identity != load_identity)
                or (previous_version >= 0 and int(context[2]) < previous_version)):
            self._clear_floor_runtime_for_map_reset(floor)
        significant = (self.map_data is None or self.map_data.shape != new_data.shape
                       or np.count_nonzero(self.map_data != new_data)
                       >= self.map_change_cell_threshold)
        self.current_map = msg
        self.map_info = msg.info
        self.map_data = new_data
        self.accepted_map_context = tuple(int(value) for value in context)
        self.accepted_map_load_identity = load_identity
        runtime = self.floor_runtime.setdefault(floor, {})
        runtime["map_load_identity"] = load_identity
        runtime["map_version"] = int(context[2])
        runtime["map_epoch"] = int(context[1])
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
        incoming_epoch = int(getattr(msg, "map_epoch", self.map_epoch))
        incoming_floor = int(msg.current_floor)
        incoming_versions = {
            int(item.floor_id): int(item.map_version)
            for item in getattr(msg, "floor_maps", [])
        }
        incoming_version = int(incoming_versions.get(incoming_floor, 0))
        if self.last_mapping_status_time != rospy.Time(0) and (
                incoming_epoch < self.map_epoch
                or (incoming_epoch == self.map_epoch
                    and incoming_floor != self.current_floor)
                or (incoming_epoch == self.map_epoch
                    and incoming_floor == self.current_floor
                    and incoming_version < self.current_map_version)):
            rospy.logwarn_throttle(
                2.0,
                "[exploration] ignoring regressive mapping status floor=%d epoch=%d version=%d",
                incoming_floor, incoming_epoch, incoming_version,
            )
            return
        previous_floor = self.current_floor
        previous_epoch = self.map_epoch
        self.mapping_ready = msg.ready
        self.mapping_stable = msg.stable
        self.mapping_lost = msg.lost
        self.map_epoch = incoming_epoch
        self.mapping_transitioning = bool(getattr(msg, "transitioning", False))
        self.current_floor = incoming_floor
        self.floor_map_versions = incoming_versions
        self.current_map_version = incoming_version
        self.visited_floors.add(self.current_floor)
        for key, binding in list(getattr(
                self, "elevator_hall_bindings", {}).items()):
            if (int(key[0]) == self.current_floor
                    and int(binding.get("epoch", -1)) != self.map_epoch):
                self.elevator_hall_bindings.pop(key, None)
        if (self.map_epoch != previous_epoch
                or self.current_floor != previous_floor
                or self.mapping_transitioning):
            self._invalidate_active_map(cancel_goal=True)
        self.last_mapping_status_time = rospy.Time.now()
        self._try_accept_pending_active_map()

    def _scan_callback(self, message):
        self.latest_scan = message
        self.last_scan_time = rospy.Time.now()

    def _safety_stop_callback(self, message):
        self.safety_stop_active = bool(message.data)
        if self.safety_stop_active and self.floor_change_active:
            self._stop_elevator_motion()

    def _sent_cmd_callback(self, message):
        self.last_sent_command = message
        self.last_sent_command_time = rospy.Time.now()

    def nav_health_callback(self, msg):
        incoming_context = (
            int(getattr(msg, "current_floor", self.current_floor)),
            int(getattr(msg, "map_epoch", self.map_epoch)),
            int(getattr(msg, "map_version", self.current_map_version)),
        )
        current_context = (
            int(getattr(self, "nav_floor", self.current_floor)),
            int(getattr(self, "nav_map_epoch", self.map_epoch)),
            int(getattr(self, "nav_map_version", 0)),
        )
        if self.last_nav_health_time != rospy.Time(0) and (
                incoming_context[1] < current_context[1]
                or (incoming_context[1] == current_context[1]
                    and incoming_context[0] != current_context[0])
                or (incoming_context[:2] == current_context[:2]
                    and incoming_context[2] < current_context[2])):
            rospy.logwarn_throttle(
                2.0,
                "[exploration] ignoring regressive navigation context %s",
                incoming_context,
            )
            return
        self.nav_ready = msg.ready
        self.nav_has_active_goal = msg.has_active_goal
        self.nav_stuck = msg.stuck
        self.nav_failure_code = msg.failure_code
        self.nav_failure_detail = msg.failure_detail
        self.nav_active_goal_id = msg.active_goal_id
        self.nav_floor, self.nav_map_epoch, self.nav_map_version = incoming_context
        self.nav_transitioning = bool(
            getattr(msg, "transitioning", self.mapping_transitioning)
        )
        self.last_nav_health_time = rospy.Time.now()

    def recovery_event_callback(self, msg):
        with self.state_lock:
            if (msg.header.frame_id != self.map_frame or self.map_data is None
                    or (not self.exploring and not self.floor_change_active)):
                return

            # Elevator-hall navigation has its own bounded candidate retry
            # policy.  A failed hall approach must not mutate the completed
            # floor's exploration blacklist: doing so can make every remaining
            # hall candidate appear unreachable on the next floor-change
            # attempt.  This also isolates late recovery events from the
            # ordinary goal canceled when floor transit starts.
            if self.floor_change_active:
                return

            # RecoveryEvent is latched and navigation is also used by Mission.
            # Only consume an event for the goal most recently handed to
            # move_base by this planner.  Clearing nav_active_goal_id when a
            # goal is sent forces a fresh NavigationHealth sample to establish
            # ownership, while the timestamp rejects a delayed latched event.
            event_goal_id = str(msg.active_goal_id or "")
            if (not event_goal_id or event_goal_id != self.nav_active_goal_id
                    or msg.header.stamp < self.navigation_goal_sent_at):
                return

            stuck_pose = (msg.stuck_pose.position.x, msg.stuck_pose.position.y)
            if msg.phase == RecoveryEvent.PHASE_TRIGGERED:
                # Triggered is diagnostic only.  A slow A1 gait may recover, so it
                # must not poison all nearby frontier candidates pre-emptively.
                self.last_recovery_goal_id = event_goal_id
                self.last_recovery_stuck_pose = stuck_pose
                self.last_recovery_trigger_id = msg.event_id
                rospy.loginfo(
                    "[exploration] recovery triggered at (%.2f, %.2f), "
                    "attempt=%d maneuver=%d goal=%s (diagnostic only)",
                    stuck_pose[0], stuck_pose[1], msg.attempt, msg.maneuver,
                    event_goal_id,
                )
                return
            if (msg.phase != RecoveryEvent.PHASE_FAILED
                    or msg.event_id <= self.last_recovery_event_id):
                return
            self.last_recovery_goal_id = event_goal_id
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

    def _map_cache_identity(self):
        """Identify every map-dependent cache across floor switches and loads."""
        return (
            int(getattr(self, "current_floor", 0)),
            int(getattr(self, "map_epoch", 0)),
            int(getattr(self, "current_map_version", 0)),
            int(getattr(self, "map_revision", 0)),
        )

    def _clearance_map(self):
        key = self._map_cache_identity() + (
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
        binding = getattr(self, "elevator_hall_bindings", {}).get((
            int(self.current_floor), str(getattr(self, "active_elevator_id", ""))
        ))
        binding_status = None
        if binding is not None:
            hall = tuple(binding.get("hall", ()))
            if len(hall) == 3:
                binding_status = {
                    "x": float(hall[0]),
                    "y": float(hall[1]),
                    "yaw": float(hall[2]),
                    "source": str(binding.get("source", "geometry")),
                    "confidence": float(binding.get("confidence", 0.0)),
                    "validated": bool(binding.get("validated", False)),
                }
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
            "coverage_debt_count": len(self.coverage_debt_by_floor.get(
                int(self.current_floor), set()
            )),
            "elevator_candidate_count": max(
                len(getattr(self, "elevator_halls", [])),
                len(getattr(self, "elevator_hall_tracks", {})),
            ),
            "elevator_binding": binding_status,
            "initial_hall_discovery": str(getattr(
                self, "initial_hall_discovery_step", "IDLE"
            )),
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
        expected_context = (
            int(self.current_floor), int(self.map_epoch),
            int(self.current_map_version),
        )
        if (self.multifloor_enabled and not map_context_is_committed(
                expected_context, self.accepted_map_context)):
            return False, "active_map_context_mismatch"
        if (not self.nav_ready or self.nav_transitioning
                or (self.multifloor_enabled and not map_context_is_committed(
                    expected_context,
                    (self.nav_floor, self.nav_map_epoch, self.nav_map_version),
                ))):
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
        key = self._map_cache_identity() + (
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
            cache_key = self._map_cache_identity() + ("all",)
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
            self, cluster, reachable, clearance, frontier_safe=None):
        """Choose a known-free viewpoint for one frontier cluster.

        The observation band is relative to *this* cluster.  Distance to the
        nearest frontier anywhere in the map is a separate footprint-safety
        constraint: using it as the observation distance makes every cluster
        impossible whenever two unknown boundaries are closer than twice the
        configured standoff.
        """
        margin = int(math.ceil(
            self.observation_max_distance / self.map_info.resolution
        )) + 2
        cluster_x = [cell[0] for cell in cluster]
        cluster_y = [cell[1] for cell in cluster]
        x0 = max(0, min(cluster_x) - margin)
        x1 = min(self.map_info.width, max(cluster_x) + margin + 1)
        y0 = max(0, min(cluster_y) - margin)
        y1 = min(self.map_info.height, max(cluster_y) + margin + 1)
        cluster_mask = np.zeros((y1 - y0, x1 - x0), dtype=np.uint8)
        for cell_x, cell_y in cluster:
            cluster_mask[cell_y - y0, cell_x - x0] = 1
        distance_to_cluster = cv2.distanceTransform(
            (cluster_mask == 0).astype(np.uint8),
            cv2.DIST_L2,
            cv2.DIST_MASK_PRECISE,
        ) * self.map_info.resolution
        required_clearance = (
            self.connectivity_clearance_radius + self.goal_clearance_margin
        )
        if frontier_safe is None:
            nearest_frontier_safe = np.ones_like(
                distance_to_cluster, dtype=bool
            )
        else:
            nearest_frontier_safe = frontier_safe[y0:y1, x0:x1]
        candidate_mask = (
            reachable[y0:y1, x0:x1]
            & (distance_to_cluster >= (
                self.observation_min_distance
                + 0.5 * self.map_info.resolution - 1e-6
            ))
            & (distance_to_cluster <= self.observation_max_distance + 1e-6)
            & nearest_frontier_safe
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
            distance_to_cluster[candidate_y, candidate_x]
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
        required_clearance = (
            self.connectivity_clearance_radius + self.goal_clearance_margin
        )
        radius_cells = int(math.ceil(
            required_clearance / self.map_info.resolution
        ))
        yy, xx = np.ogrid[
            -radius_cells:radius_cells + 1,
            -radius_cells:radius_cells + 1,
        ]
        # This boolean dilation is equivalent to testing whether the nearest
        # frontier centre is closer than required_clearance, while avoiding a
        # second full-map Euclidean distance transform on every planning pass.
        frontier_kernel = (
            (xx * xx + yy * yy) * self.map_info.resolution ** 2
            < required_clearance ** 2 - 1e-9
        ).astype(np.uint8)
        frontier_safe = cv2.dilate(
            np.asarray(frontier, dtype=np.uint8), frontier_kernel, iterations=1
        ) == 0
        goals = []
        cluster_key = self._map_cache_identity() + (
            int(getattr(self, "_reachable_component_id", 0)),
        )
        for cluster in self._frontier_clusters(
                frontier & reachable, cache_key=cluster_key):
            goal = self._observation_goal_for_cluster(
                cluster, reachable, clearance, frontier_safe
            )
            if (goal is not None
                    and self._entrance_boundary_allows_goal(goal["x"], goal["y"])):
                goals.append(goal)
        self.observation_goal_cells = [goal["cell"] for goal in goals]
        self._publish_observation_goals(goals)
        return goals

    def _capture_entrance_boundary_anchor(self):
        """Capture a fresh start-pose boundary for this exploration session."""
        if not getattr(self, "entrance_boundary_guard_enabled", False):
            self.entrance_boundary_anchor = None
            return True
        try:
            self.entrance_boundary_anchor = entrance_boundary_anchor_from_pose(
                self.current_pose, self.current_floor
            )
        except (AttributeError, TypeError, ValueError):
            self.entrance_boundary_anchor = None
            return False
        return True

    def _entrance_boundary_allows_goal(self, goal_x, goal_y):
        """Apply the virtual entrance boundary only on its anchor floor."""
        if not getattr(self, "entrance_boundary_guard_enabled", False):
            return True
        anchor = getattr(self, "entrance_boundary_anchor", None)
        if anchor is None:
            # Enabling the guard without a valid start pose is a startup
            # contract error; do not silently dispatch an unguarded goal.
            return False
        if int(getattr(self, "current_floor", 0)) != int(anchor[3]):
            return True
        try:
            return entrance_boundary_allows_goal(
                anchor,
                goal_x,
                goal_y,
                getattr(self, "entrance_boundary_allowance_m", 0.0),
            )
        except (TypeError, ValueError):
            return False

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
        component_key = self._map_cache_identity() + (
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
        self.selected_goal_metrics = None
        if self.current_pose is None or self.map_data is None:
            return None, "input_missing"

        cx = self.current_pose.position.x
        cy = self.current_pose.position.y
        frontier = self._frontier_mask()
        all_representatives = self._frontier_representatives(frontier)
        # The guarded entrance is a deliberate virtual boundary, not unpaid
        # exploration work. Excluded exterior frontiers must be removed from
        # convergence accounting or an indoor-only run can never finish.
        all_representatives = [
            cell for cell in all_representatives
            if self._entrance_boundary_allows_goal(*self._map_to_world(*cell))
        ]
        self.remaining_frontier_count = len(all_representatives)
        if not all_representatives:
            self.coverage_debt_by_floor.pop(int(self.current_floor), None)
            return None, "no_frontier"

        reachable = self._reachable_free_mask()
        reachable_frontier = frontier & reachable
        coverage_debt = set()
        for cell_x, cell_y in all_representatives:
            if not bool(reachable[cell_y, cell_x]):
                coverage_debt.add((int(cell_x), int(cell_y), "disconnected"))
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

        minimum_dispatch = float(
            getattr(self, "min_goal_dispatch_distance_m", 0.0)
        )
        dispatch_candidates = [
            candidate for candidate in candidates
            if math.hypot(candidate["x"] - cx, candidate["y"] - cy)
            >= minimum_dispatch
        ]
        skipped_near_candidates = len(dispatch_candidates) < len(candidates)

        reachable_candidates = []
        service_unavailable = False
        for candidate in dispatch_candidates[:self.max_frontier_candidates]:
            gx, gy = candidate["x"], candidate["y"]
            map_x, map_y = self._world_to_map(gx, gy)
            if self._goal_is_cooled_down(gx, gy):
                coverage_debt.add((int(map_x), int(map_y), "cooldown"))
                continue
            if ((map_x, map_y) in getattr(self, "trap_blacklist", {})):
                coverage_debt.add((int(map_x), int(map_y), "trap"))
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
                reachable_candidates.append((score, candidate, metrics))
            elif path_state == "unreachable":
                coverage_debt.add((int(map_x), int(map_y), "unreachable"))
            if path_state == "unavailable":
                service_unavailable = True

        if coverage_debt:
            self.coverage_debt_by_floor[int(self.current_floor)] = coverage_debt
        else:
            self.coverage_debt_by_floor.pop(int(self.current_floor), None)

        if reachable_candidates:
            _, selected, selected_metrics = min(
                reachable_candidates, key=lambda item: item[0]
            )
            selected_metrics = dict(selected_metrics or {})
            planned_path_length = float(selected_metrics.get(
                "path_length",
                math.hypot(selected["x"] - cx, selected["y"] - cy),
            ))
            points = selected_metrics.get("points") or ()
            path_horizon = float(getattr(
                self, "max_frontier_goal_path_m", float("inf")
            ))
            if planned_path_length > path_horizon and points:
                waypoint_x, waypoint_y, waypoint_yaw, dispatch_length, truncated = (
                    path_prefix_goal(points, path_horizon)
                )
                if truncated:
                    self.selected_goal_metrics = {
                        **selected_metrics,
                        "frontier_goal": (selected["x"], selected["y"]),
                        "dispatch_path_length": dispatch_length,
                        "truncated": True,
                    }
                    return (
                        (waypoint_x, waypoint_y, waypoint_yaw),
                        "reachable_frontier_waypoint",
                    )
            self.selected_goal_metrics = {
                **selected_metrics,
                "dispatch_path_length": planned_path_length,
                "truncated": False,
            }
            goal = (
                (selected["x"], selected["y"], selected["yaw"])
                if use_observation_goals
                else (selected["x"], selected["y"])
            )
            return goal, "reachable_frontier"
        if service_unavailable:
            return None, "navigation_service_unavailable"

        if skipped_near_candidates and not dispatch_candidates:
            return None, "frontier_already_in_observation_range"

        return None, "all_frontiers_unreachable_or_blacklisted"

    def _send_goal(self, gx, gy, yaw=0.0, planned_path_length=None,
                   context="exploration"):
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
        self.last_recovery_goal_id = ""
        self.nav_active_goal_id = ""
        self.navigation_goal_sent_at = rospy.Time.now()
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
        if planned_path_length is None:
            self.active_goal_timeout_s = float(self.goal_timeout)
        else:
            self.active_goal_timeout_s = bounded_navigation_timeout(
                planned_path_length,
                self.goal_timeout,
                self.goal_timeout_base_s,
                self.goal_timeout_per_path_m,
                self.goal_timeout_min_s,
            )
        rospy.loginfo(
            "[exploration] Sent goal: (%.2f, %.2f) path=%.2fm timeout=%.1fs%s",
            gx,
            gy,
            -1.0 if planned_path_length is None else planned_path_length,
            self.active_goal_timeout_s,
            " waypoint" if (planned_path_length is not None and (
                self.selected_goal_metrics
                and self.selected_goal_metrics.get("truncated")
            )) else "",
        )
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
                self._remember_successful_goal()
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
            if getattr(self, "initial_hall_discovery_active", False):
                return TriggerResponse(
                    success=False,
                    message="Initial elevator door restoration is still running",
                )
            if not self._capture_entrance_boundary_anchor():
                return TriggerResponse(
                    success=False,
                    message="entrance boundary guard requires a valid start pose",
                )
            rospy.loginfo("[exploration] Start exploration")
            self.session_id += 1
            self.exploring = True
            self.waiting_for_result = False
            self.current_goal = None
            self.last_recovery_goal_id = ""
            self.nav_active_goal_id = ""
            self.navigation_goal_sent_at = rospy.Time(0)
            self.retry_count = 0
            self.backoff_until = rospy.Time(0)
            self.no_reachable_frontier_cycles = 0
            self.floor_no_frontier_since = rospy.Time(0)
            self.floor_unreachable_since = rospy.Time(0)
            self.complete_published = False
            self.visited_floors = {self.current_floor}
            self.completed_floors = set()
            self.autonomous_transition_failures = {}
            self.next_autonomous_floor = None
            self.elevator_autonomous_transition = False
            self.complete_pub.publish(Bool(data=False))
            self.visited_floors = {self.current_floor}
            self.completed_floors = set()
            self.floor_runtime = {}
            self.coverage_debt_by_floor = {}
            self.elevator_hall_bindings = {}
            self.elevator_hall_tracks = {}
            self.elevator_hall_last_observed_version = None
            self.floor_change_active = False
            self.floor_change_step = None
            self.floor_change_gave_up_count = 0
            self.floor_transit_fatal = False
            self.floor_change_retry_after = rospy.Time(0)
            self.elevator_halls = []
            self.elevator_hall_index = 0
            self.elevator_hall_found = None
            self._begin_initial_hall_discovery()
            if self.initial_hall_discovery_active:
                self._set_state(
                    "INITIAL_HALL_DISCOVERY", "waiting_for_discovery_inputs"
                )
            else:
                self._set_state("WAITING", "waiting_for_inputs")
            return TriggerResponse(success=True, message="Exploration started; waiting for inputs")

    def stop_exploration_cb(self, req):
        with self.state_lock:
            if not self.exploring and not self.initial_hall_discovery_active:
                return TriggerResponse(success=True, message="Exploration already stopped")
            rospy.loginfo("[exploration] Stop exploration")
            if self.floor_change_active and not self.floor_change_external:
                self._cancel_floor_change(
                    "CANCELED", "exploration stopped during floor transit"
                )
            self.exploring = False
            if (self.initial_hall_discovery_active
                    or getattr(
                        self, "initial_hall_discovery_door_held_closed", False
                    )):
                if not self.initial_hall_discovery_active:
                    self.initial_hall_discovery_active = True
                self._begin_initial_discovery_restore(
                    "exploration stopped during initial hall discovery"
                )
            self.session_id += 1
            self.goal_id += 1
            self.waiting_for_result = False
            self.current_goal = None
            self.entrance_boundary_anchor = None
            self.last_recovery_goal_id = ""
            self.nav_active_goal_id = ""
            self.navigation_goal_sent_at = rospy.Time(0)
            self.move_base_client.cancel_all_goals()
            self._set_state("STOPPED", "stop_requested")
            return TriggerResponse(success=True, message="Exploration stopped")

    def _reset_floor_completion_evidence(self):
        self.no_reachable_frontier_cycles = 0
        self.floor_no_frontier_since = rospy.Time(0)
        self.floor_unreachable_since = rospy.Time(0)

    def _reset_bounded_floor_completion_evidence(self):
        self.floor_unreachable_since = rospy.Time(0)

    def _navigation_service_available(self):
        try:
            rospy.wait_for_service(
                self.make_plan_service, timeout=self.dependency_check_timeout
            )
            return True
        except rospy.ROSException:
            return False

    def _floor_completion_mode(self, selection_reason, now):
        """Accumulate strict or bounded floor-exhaustion evidence."""
        if selection_reason == "no_frontier":
            self.floor_unreachable_since = rospy.Time(0)
            self.no_reachable_frontier_cycles += 1
            if self.floor_no_frontier_since == rospy.Time(0):
                self.floor_no_frontier_since = now
        elif selection_reason in BOUNDED_FLOOR_EXHAUSTION_REASONS:
            self.no_reachable_frontier_cycles = 0
            self.floor_no_frontier_since = rospy.Time(0)
            if self.floor_unreachable_since == rospy.Time(0):
                self.floor_unreachable_since = now
        else:
            self._reset_floor_completion_evidence()
            return None, selection_reason

        map_stable = (
            now - self.last_significant_map_change
        ).to_sec() >= self.map_stable_time
        no_active_goal = (
            not self.waiting_for_result and not self.nav_has_active_goal
        )
        if selection_reason == "no_frontier":
            held = (
                now - self.floor_no_frontier_since
            ).to_sec() >= self.floor_no_frontier_hold_s
            no_coverage_debt = not self.coverage_debt_by_floor.get(
                self.current_floor, set()
            )
            ready = (
                self.no_reachable_frontier_cycles
                >= self.no_frontier_cycles_required
                and held
                and no_coverage_debt
                and map_stable
                and no_active_goal
            )
            return ("strict_no_frontier" if ready else None), selection_reason

        held = (
            now - self.floor_unreachable_since
        ).to_sec() >= self.floor_unreachable_hold_s
        if not (held and map_stable and no_active_goal):
            return None, selection_reason
        if not self._navigation_service_available():
            self._reset_floor_completion_evidence()
            return None, "navigation_service_unavailable"
        return "bounded_unreachable", selection_reason

    def _mark_current_floor_complete(self, completion_mode, selection_reason):
        """Persist and announce one floor-completion decision exactly once."""
        if self.current_floor in self.completed_floors:
            return False
        debt_count = len(self.coverage_debt_by_floor.get(
            self.current_floor, set()
        ))
        rospy.loginfo(
            "[exploration] floor %d complete: mode=%s reason=%s "
            "remaining_frontiers=%d coverage_debt=%d",
            self.current_floor,
            completion_mode,
            selection_reason,
            self.remaining_frontier_count,
            debt_count,
        )
        self._save_current_floor_runtime()
        self.completed_floors.add(self.current_floor)
        return True

    def _continue_completed_floor(self, now):
        """Advance or settle the transition for an already completed floor."""
        if self.floor_change_active:
            return
        if not self.multifloor_enabled:
            self._complete_exploration("all_served_floors_explored")
            return
        if self.completed_floors.issuperset(self.served_floors):
            self._complete_exploration("all_served_floors_explored")
            return

        next_floor = self._select_next_floor()
        if next_floor is None:
            self.floor_transit_fatal = True
            self._set_state("FAILED", "served_floor_topology_unreachable")
            return
        if self.floor_change_gave_up_count >= self.elevator_max_retries:
            if not (
                    self.exploration_state == "FAILED"
                    and self.state_reason == "floor_transit_unavailable"):
                self._set_state("FAILED", "floor_transit_unavailable")
            return
        if now <= self.floor_change_retry_after:
            if not (
                    self.exploration_state == "WAITING"
                    and self.state_reason == "floor_transit_retry_backoff"):
                self._set_state("WAITING", "floor_transit_retry_backoff")
            return
        self._begin_floor_change(next_floor)

    def planner_loop(self, event):
        """主规划循环"""
        now = rospy.Time.now()
        if getattr(self, "initial_hall_discovery_active", False):
            try:
                self._advance_initial_hall_discovery(now)
            except Exception as exc:
                rospy.logerr(
                    "[exploration] initial hall discovery exception: %s", exc
                )
                self._begin_initial_discovery_restore(
                    "initial hall discovery exception: %s" % exc
                )
            return
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
            # Preserve the legacy strict no-frontier hold semantics, while a
            # bounded-unreachable decision must observe 30 continuous seconds
            # of healthy inputs.
            self.no_reachable_frontier_cycles = 0
            self._reset_bounded_floor_completion_evidence()
            state = "FAILED" if reason == "localization_lost" else "WAITING"
            self._set_state(state, reason)
            return

        if (self.multifloor_enabled and hasattr(self, "map_epoch")
                and getattr(self, "map_data", None) is not None):
            self._observe_elevator_halls(now)

        # A floor-completion decision is durable across bounded transit retries.
        # Do not rediscover frontiers or emit the same completion log while the
        # state machine waits for its retry deadline.
        if self.current_floor in self.completed_floors:
            self._continue_completed_floor(now)
            return

        if self.waiting_for_result:
            self._set_state("NAVIGATING", "active_goal")
            # 检查目标是否超时
            elapsed = (rospy.Time.now() - self.last_goal_time).to_sec()
            if elapsed > self.active_goal_timeout_s:
                rospy.logwarn(
                    "[exploration] Goal timeout after %.1fs, canceling",
                    self.active_goal_timeout_s,
                )
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
            self._reset_floor_completion_evidence()
            selected_metrics = self.selected_goal_metrics or {}
            if not self._send_goal(
                    *goal,
                    planned_path_length=selected_metrics.get(
                        "dispatch_path_length"
                    )):
                self.retry_count += 1
                self._set_state("WAITING", "move_base_unavailable")
            else:
                self._set_state("NAVIGATING", "goal_sent")
        else:
            if selection_reason == "navigation_service_unavailable":
                self._reset_floor_completion_evidence()
                self._set_state("WAITING", selection_reason)
                return
            completion_mode, wait_reason = self._floor_completion_mode(
                selection_reason, now
            )
            if completion_mode is not None:
                self._mark_current_floor_complete(
                    completion_mode, selection_reason
                )
                self._continue_completed_floor(now)
            else:
                self._set_state("WAITING", wait_reason)

    # ========== 多楼层：电梯自主发现与换层 ==========

    def _current_floor_callback(self, message):
        """Compatibility mirror of the localization-owned discrete floor."""
        floor = int(message.data)
        if self.last_mapping_status_time != rospy.Time(0):
            if floor != self.current_floor:
                rospy.logwarn_throttle(
                    2.0,
                    "[exploration] ignoring current_floor=%d while status owns floor=%d",
                    floor,
                    self.current_floor,
                )
            return
        if floor != self.current_floor:
            rospy.loginfo("[exploration] bootstrap current floor -> %d", floor)
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
        prior = self.floor_runtime.get(int(self.current_floor), {})
        self.floor_runtime[int(self.current_floor)] = {
            "failed_goals": list(self.failed_goals),
            "trap_blacklist": dict(self.trap_blacklist),
            "retry_count": int(self.retry_count),
            "map_epoch": int(self.map_epoch),
            "map_version": int(self.current_map_version),
            "map_load_identity": (
                self.accepted_map_load_identity
                if self.accepted_map_load_identity is not None
                else prior.get("map_load_identity")
            ),
            "coverage_debt": set(self.coverage_debt_by_floor.get(
                int(self.current_floor), set()
            )),
        }

    def _restore_current_floor_runtime(self):
        runtime = self.floor_runtime.get(int(self.current_floor), {})
        self.failed_goals = list(runtime.get("failed_goals", []))
        self.trap_blacklist = dict(runtime.get("trap_blacklist", {}))
        self.retry_count = int(runtime.get("retry_count", 0))
        restored_debt = set(runtime.get("coverage_debt", set()))
        if restored_debt:
            self.coverage_debt_by_floor[int(self.current_floor)] = restored_debt
        else:
            self.coverage_debt_by_floor.pop(int(self.current_floor), None)
        self.backoff_until = rospy.Time(0)
        self.no_reachable_frontier_cycles = 0
        self.floor_no_frontier_since = rospy.Time(0)
        self.floor_unreachable_since = rospy.Time(0)
        self._reachable_cache_key = None
        self._reachable_cache = None
        self._component_cache_key = None
        self._frontier_cache_key = None
        self._frontier_cluster_cache_key = None
        self._clearance_cache_key = None
        self._publish_blacklist()

    def _clear_floor_runtime_for_map_reset(self, floor):
        """Discard coordinates tied to a map that was reset or reloaded."""
        floor = int(floor)
        self.floor_runtime.pop(floor, None)
        self.coverage_debt_by_floor.pop(floor, None)
        self.completed_floors.discard(floor)
        for key in list(self.elevator_hall_bindings):
            if int(key[0]) == floor:
                self.elevator_hall_bindings.pop(key, None)
        if floor != int(self.current_floor):
            return
        self.failed_goals = []
        self.trap_blacklist = {}
        self.retry_count = 0
        self.backoff_until = rospy.Time(0)
        self.no_reachable_frontier_cycles = 0
        self.floor_no_frontier_since = rospy.Time(0)
        self.floor_unreachable_since = rospy.Time(0)
        self.current_goal = None
        self.waiting_for_result = False
        self.last_recovery_goal_id = ""
        self.nav_active_goal_id = ""
        self.navigation_goal_sent_at = rospy.Time(0)
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

    def _control_output_is_zero(self, now, freshness_s=0.75):
        stamp = getattr(self, "last_sent_command_time", rospy.Time(0))
        command = getattr(self, "last_sent_command", None)
        if command is None or stamp == rospy.Time(0):
            return False
        if (now - stamp).to_sec() > float(freshness_s):
            return False
        values = (
            command.linear.x, command.linear.y, command.linear.z,
            command.angular.x, command.angular.y, command.angular.z,
        )
        return all(abs(float(value)) <= 1e-3 for value in values)

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

    @staticmethod
    def _scan_geometry(message):
        return (
            len(message.ranges),
            float(message.angle_min),
            float(message.angle_increment),
            float(message.range_min),
            float(message.range_max),
            str(message.header.frame_id),
        )

    @staticmethod
    def _full_scan_ranges(message):
        values = np.asarray(message.ranges, dtype=np.float32).copy()
        invalid = (
            ~np.isfinite(values)
            | (values < float(message.range_min))
            | (values > float(message.range_max))
        )
        values[invalid] = np.nan
        return values

    def _discovery_robot_pose(self):
        if self.current_pose is None:
            return None
        return (
            float(self.current_pose.position.x),
            float(self.current_pose.position.y),
            float(self._yaw_from_quaternion(self.current_pose.orientation)),
        )

    def _initial_discovery_robot_moved(self):
        start = self.initial_hall_discovery_pose
        current = self._discovery_robot_pose()
        if start is None or current is None:
            return True
        yaw_delta = abs(math.atan2(
            math.sin(current[2] - start[2]), math.cos(current[2] - start[2])
        ))
        return (
            math.hypot(current[0] - start[0], current[1] - start[1])
            > self.initial_hall_discovery_max_translation_m
            or yaw_delta > math.radians(self.initial_hall_discovery_max_yaw_deg)
        )

    def _set_initial_discovery_step(self, step, reason):
        self.initial_hall_discovery_step = str(step)
        self._set_state("INITIAL_HALL_DISCOVERY", str(reason))

    def _begin_initial_hall_discovery(self):
        enabled = (
            bool(getattr(self, "multifloor_enabled", False))
            and bool(getattr(self, "initial_hall_discovery_enabled", False))
            and not bool(getattr(self, "fixed_elevator_hall_enabled", False))
            and bool(getattr(self, "elevator_door_initial_open", {}).get(
                int(self.current_floor), False
            ))
            and bool(self._elevator_door_id(self.current_floor))
        )
        self.initial_hall_discovery_active = bool(enabled)
        if enabled:
            self.initial_hall_discovery_step = "WAIT_READY"
        elif bool(getattr(self, "fixed_elevator_hall_enabled", False)):
            self.initial_hall_discovery_step = "FIXED_OVERRIDE"
        else:
            self.initial_hall_discovery_step = "DISABLED"
        self.initial_hall_discovery_started = (
            rospy.Time.now() if enabled else rospy.Time(0)
        )
        self.initial_hall_discovery_pose = None
        self.initial_hall_discovery_scan_geometry = None
        self.initial_hall_discovery_open_scans = []
        self.initial_hall_discovery_closed_scans = []
        self.initial_hall_discovery_last_scan_stamp = rospy.Time(0)
        self.initial_hall_discovery_settle_until = rospy.Time(0)
        self.initial_hall_discovery_restore_required = False
        self.initial_hall_discovery_failure = ""
        self.initial_hall_discovery_zero_since = rospy.Time(0)
        self.initial_hall_discovery_abort_deadline = rospy.Time(0)
        self.initial_hall_discovery_closed_scan_message = None

    def _collect_initial_discovery_scan(self, destination):
        if self.latest_scan is None:
            return False
        if self.last_scan_time <= self.initial_hall_discovery_last_scan_stamp:
            return False
        geometry = self._scan_geometry(self.latest_scan)
        if self.initial_hall_discovery_scan_geometry is None:
            self.initial_hall_discovery_scan_geometry = geometry
        elif geometry != self.initial_hall_discovery_scan_geometry:
            self._begin_initial_discovery_restore("scan geometry changed")
            return False
        destination.append(self._full_scan_ranges(self.latest_scan))
        self.initial_hall_discovery_last_scan_stamp = self.last_scan_time
        if destination is self.initial_hall_discovery_closed_scans:
            self.initial_hall_discovery_closed_scan_message = self.latest_scan
        return len(destination) >= self.initial_hall_discovery_scan_count

    def _begin_initial_discovery_restore(self, reason):
        if not self.initial_hall_discovery_active:
            return
        self.initial_hall_discovery_restore_required = True
        self.initial_hall_discovery_failure = str(reason)
        if self._service_future is not None:
            self.initial_hall_discovery_abort_deadline = (
                rospy.Time.now() + rospy.Duration(self.elevator_service_timeout_s)
            )
            self._set_initial_discovery_step(
                "ABORTING", "restore_initial_elevator_door"
            )
        else:
            self._set_initial_discovery_step(
                "RESTORE_OPEN_START", "restore_initial_elevator_door"
            )

    def _finish_initial_hall_discovery(self, success):
        self.initial_hall_discovery_active = False
        self.initial_hall_discovery_step = "DONE" if success else "FALLBACK"
        if success:
            self._set_state("WAITING", "initial_hall_discovery_complete")
        elif self.exploring:
            rospy.logwarn(
                "[exploration] initial elevator discovery fell back: %s",
                self.initial_hall_discovery_failure or "no unambiguous door motion",
            )
            self._set_state("WAITING", "initial_hall_discovery_fallback")
        else:
            self._set_state("STOPPED", "stop_requested")

    def _scan_to_map_planar_transform(self, message):
        frame = str(message.header.frame_id)
        if frame == self.map_frame:
            return (0.0, 0.0, 0.0)
        transform = self.tf_buffer.lookup_transform(
            self.map_frame, frame, message.header.stamp, rospy.Duration(0.25)
        ).transform
        return (
            float(transform.translation.x),
            float(transform.translation.y),
            float(self._yaw_from_quaternion(transform.rotation)),
        )

    def _save_hall_binding(self, candidate, floor=None, epoch=None):
        floor = int(self.current_floor if floor is None else floor)
        epoch = int(self.map_epoch if epoch is None else epoch)
        key = (floor, str(self.active_elevator_id))
        self.elevator_hall_bindings[key] = {
            "hall": candidate.hall(),
            "floor": floor,
            "epoch": epoch,
            "map_version": int(self.current_map_version),
            "map_load_identity": self.accepted_map_load_identity,
            "source": str(candidate.source),
            "confidence": float(candidate.confidence),
            "score": float(candidate.score),
            "validated": bool(candidate.validated),
        }

    def _advance_initial_hall_discovery(self, now):
        step = self.initial_hall_discovery_step
        if (step not in ("ABORTING", "RESTORE_OPEN_START", "RESTORE_OPEN_WAIT")
                and (now - self.initial_hall_discovery_started).to_sec()
                > self.initial_hall_discovery_timeout_s):
            self._begin_initial_discovery_restore(
                "initial hall discovery timed out"
            )
            return
        if step == "WAIT_READY":
            healthy, _reason = self._inputs_health(now)
            output_zero = self._control_output_is_zero(now)
            if output_zero:
                if self.initial_hall_discovery_zero_since == rospy.Time(0):
                    self.initial_hall_discovery_zero_since = now
            else:
                self.initial_hall_discovery_zero_since = rospy.Time(0)
            stationary = (
                not self.waiting_for_result
                and not self.nav_has_active_goal
                and self.initial_hall_discovery_zero_since != rospy.Time(0)
                and (now - self.initial_hall_discovery_zero_since).to_sec() >= 0.75
            )
            if not healthy or not stationary or self.latest_scan is None:
                return
            self.initial_hall_discovery_pose = self._discovery_robot_pose()
            self.initial_hall_discovery_last_scan_stamp = rospy.Time(0)
            self._set_initial_discovery_step(
                "CAPTURE_OPEN", "capture_open_elevator_scans"
            )
            return

        if step not in ("ABORTING", "RESTORE_OPEN_START", "RESTORE_OPEN_WAIT"):
            if self._initial_discovery_robot_moved():
                self._begin_initial_discovery_restore("robot moved during scan pair")
                return

        if step == "CAPTURE_OPEN":
            if self._collect_initial_discovery_scan(
                    self.initial_hall_discovery_open_scans):
                self._set_initial_discovery_step(
                    "CLOSE_START", "close_initial_elevator_door"
                )
            return
        if step == "CLOSE_START":
            if self._submit_service(
                    "discovery_close",
                    lambda: self._door_request(self.current_floor, False)):
                self._set_initial_discovery_step(
                    "CLOSE_WAIT", "close_initial_elevator_door"
                )
            return
        if step == "CLOSE_WAIT":
            if (now > self.floor_change_stage_deadline
                    and self._service_future is not None
                    and not self._service_future.done()):
                self._begin_initial_discovery_restore(
                    "close service timed out"
                )
                return
            outcome, response = self._service_outcome(now, "discovery_close")
            if outcome == "pending":
                return
            if outcome != "success":
                self._begin_initial_discovery_restore(
                    "close service %s: %s" % (
                        outcome, getattr(response, "message", str(response or ""))
                    )
                )
                return
            self.initial_hall_discovery_settle_until = now + rospy.Duration(
                self.elevator_scan_settle_s
            )
            self._set_initial_discovery_step(
                "SETTLE_CLOSED", "settle_closed_elevator_door"
            )
            return
        if step == "SETTLE_CLOSED":
            if now < self.initial_hall_discovery_settle_until:
                return
            self.initial_hall_discovery_last_scan_stamp = self.last_scan_time
            self._set_initial_discovery_step(
                "CAPTURE_CLOSED", "capture_closed_elevator_scans"
            )
            return
        if step == "CAPTURE_CLOSED":
            if not self._collect_initial_discovery_scan(
                    self.initial_hall_discovery_closed_scans):
                return
            message = self.initial_hall_discovery_closed_scan_message
            try:
                scan_to_map = self._scan_to_map_planar_transform(message)
            except Exception as exc:
                self._begin_initial_discovery_restore(
                    "scan transform unavailable: %s" % exc
                )
                return
            geometry = self.initial_hall_discovery_scan_geometry
            candidate = localize_actuated_door(
                self.initial_hall_discovery_open_scans,
                self.initial_hall_discovery_closed_scans,
                geometry[1], geometry[2], geometry[3], geometry[4],
                scan_to_map,
                self.initial_hall_discovery_pose[:2],
                change_threshold_m=self.elevator_door_change_threshold_m,
            )
            if candidate is None:
                self._begin_initial_discovery_restore(
                    "door motion was absent or ambiguous"
                )
                return
            self._save_hall_binding(candidate)
            self.elevator_halls = [candidate]
            self.initial_hall_discovery_door_held_closed = True
            rospy.loginfo(
                "[exploration] initial elevator hall bound from door motion: "
                "floor=%d epoch=%d version=%d x=%.3f y=%.3f yaw=%.3f",
                self.current_floor, self.map_epoch, self.current_map_version,
                candidate.x, candidate.y, candidate.into_yaw,
            )
            # Successful discovery intentionally leaves the initial door closed.
            self._finish_initial_hall_discovery(True)
            return
        if step == "ABORTING":
            kind = str(self._service_kind)
            if not kind:
                self._set_initial_discovery_step(
                    "RESTORE_OPEN_START", "restore_initial_elevator_door"
                )
                return
            if (kind == "discovery_close"
                    and self._service_future is not None
                    and not self._service_future.done()
                    and now < self.initial_hall_discovery_abort_deadline):
                return
            if self._service_future is not None and self._service_future.done():
                try:
                    self._service_future.result()
                except Exception:
                    pass
                self._invalidate_service()
                self._set_initial_discovery_step(
                    "RESTORE_OPEN_START", "restore_initial_elevator_door"
                )
                return
            status, _response = self._poll_service(now, kind)
            if status == "pending":
                return
            self._invalidate_service()
            self._set_initial_discovery_step(
                "RESTORE_OPEN_START", "restore_initial_elevator_door"
            )
            return
        if step == "RESTORE_OPEN_START":
            if self._submit_service(
                    "discovery_restore_open",
                    lambda: self._door_request(self.current_floor, True)):
                self._set_initial_discovery_step(
                    "RESTORE_OPEN_WAIT", "restore_initial_elevator_door"
                )
            return
        if step == "RESTORE_OPEN_WAIT":
            outcome, response = self._service_outcome(
                now, "discovery_restore_open"
            )
            if outcome == "pending":
                return
            if outcome != "success":
                rospy.logerr(
                    "[exploration] failed to restore initial elevator door: %s",
                    getattr(response, "message", str(response or outcome)),
                )
            else:
                self.initial_hall_discovery_door_held_closed = False
            self._finish_initial_hall_discovery(False)

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
        """Find high-quality, single-opening rectangular shaft candidates."""
        resolution = self.map_info.resolution
        height, width = occupied.shape
        min_gap = max(1, int(math.ceil(self.door_gap_min_width_m / resolution)))
        max_gap = int(math.ceil(self.door_gap_max_width_m / resolution))
        min_side = float(getattr(self, "shaft_min_side_m", 1.8))
        max_side = float(getattr(self, "shaft_max_side_m", 3.6))
        minimum_support = float(getattr(self, "shaft_wall_support_min", 0.70))
        center_tolerance = float(getattr(
            self, "door_center_tolerance_fraction", 0.25
        ))
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
            minimum_structural_gap = max(2, int(math.ceil(0.15 / resolution)))
            index = 0
            while index < len(values):
                if values[index]:
                    index += 1
                    continue
                end = index
                while end < len(values) and not values[end]:
                    end += 1
                if end - index >= minimum_structural_gap:
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
                physical_width = float(w) * resolution
                physical_height = float(h) * resolution
                physical_area = physical_width * physical_height
                if not self.shaft_min_area_m2 <= physical_area <= self.shaft_max_area_m2:
                    continue
                if not (
                        min_side <= physical_width <= max_side
                        and min_side <= physical_height <= max_side):
                    continue
                fill = float(area) / float(max(w * h, 1))
                interior_free = float(np.mean(free[y:y + h, x:x + w]))
                interior_known = float(np.mean(
                    self.map_data[y:y + h, x:x + w] >= 0
                ))
                if fill < 0.55 or interior_free < 0.55 or interior_known < 0.70:
                    continue

                west = occupied[y:y + h, x - 1] != 0
                east = occupied[y:y + h, x + w] != 0
                south = occupied[y - 1, x:x + w] != 0
                north = occupied[y + h, x:x + w] != 0
                boundary_lines = (west, east, north, south)
                edge_specs = (
                    (0, west, "west", 0.0),
                    (1, east, "east", math.pi),
                    (2, north, "north", -math.pi / 2.0),
                    (3, south, "south", math.pi / 2.0),
                )
                for edge_index, line, edge, into_yaw in edge_specs:
                    gaps = line_runs(line)
                    if len(gaps) != 1:
                        continue
                    if not min_gap <= gaps[0][1] - gaps[0][0] <= max_gap:
                        continue
                    other_support = [
                        float(np.mean(other))
                        for index, other in enumerate(boundary_lines)
                        if index != edge_index
                    ]
                    if any(value < minimum_support for value in other_support):
                        continue
                    for start, end in gaps:
                        gap_cells = end - start
                        gap_width = gap_cells * resolution
                        edge_center = 0.5 * float(len(line))
                        gap_center = 0.5 * float(start + end)
                        center_offset = abs(gap_center - edge_center) / max(
                            1.0, float(len(line))
                        )
                        if center_offset > center_tolerance:
                            continue
                        non_gap = np.concatenate((line[:start], line[end:]))
                        if (not non_gap.size
                                or float(np.mean(non_gap)) < minimum_support):
                            continue
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
                            enclosure_score = float(np.mean(other_support))
                            target_side = 0.5 * (min_side + max_side)
                            half_side_range = max(1e-6, 0.5 * (max_side - min_side))
                            size_score = 0.5 * sum(
                                0.5 + 0.5 * max(
                                    0.0,
                                    1.0 - abs(side - target_side)
                                    / half_side_range,
                                ) for side in (physical_width, physical_height)
                            )
                            target_gap = 0.5 * (
                                self.door_gap_min_width_m
                                + self.door_gap_max_width_m
                            )
                            half_gap_range = max(
                                1e-6,
                                0.5 * (
                                    self.door_gap_max_width_m
                                    - self.door_gap_min_width_m
                                ),
                            )
                            width_score = 0.5 + 0.5 * max(
                                0.0,
                                1.0 - abs(gap_width - target_gap)
                                / half_gap_range,
                            )
                            center_score = max(
                                0.0, 1.0 - center_offset / max(
                                    1e-6, center_tolerance
                                )
                            )
                            base_score = (
                                0.30 * enclosure_score
                                + 0.20 * size_score
                                + 0.20 * width_score
                                + 0.10 * center_score
                                + 0.10 * interior_free
                            )
                            candidate = ElevatorHallCandidate(
                                x=wx,
                                y=wy,
                                into_yaw=into_yaw,
                                score=base_score,
                                source="geometry",
                                confidence=base_score,
                                validated=False,
                            )
                            if not any(
                                    math.hypot(wx - other[0], wy - other[1]) < 0.5
                                    and abs(math.atan2(
                                        math.sin(into_yaw - other[2]),
                                        math.cos(into_yaw - other[2]),
                                    )) < 0.35
                                    for other in candidates):
                                candidates.append(candidate)
        return candidates

    def _component_shaft_candidates(self, occupied, free):
        """Robust passive detection: find shaft wall components and their door gaps.

        The morphology-based _closed_space_hall_candidates can fail to fully enclose
        a shaft on real maps (closing leaves a non-wall boundary, so the strict
        three-wall support check rejects the real elevator).  This component-based
        fallback detects the shaft WALLS directly and looks for a single perimeter
        door gap, which is far more robust to lidar sparsity and small noise.
        """
        resolution = self.map_info.resolution
        height, width = occupied.shape
        num, labels, stats, _ = cv2.connectedComponentsWithStats(
            occupied, connectivity=8
        )
        min_bbox = self.shaft_min_area_m2 / (resolution ** 2)
        max_bbox = self.shaft_max_area_m2 / (resolution ** 2)
        minimum_support = float(getattr(self, "shaft_wall_support_min", 0.70))
        target_side = 0.5 * (self.shaft_min_side_m + self.shaft_max_side_m)
        half_side_range = max(1e-6, 0.5 * (self.shaft_max_side_m - self.shaft_min_side_m))
        candidates = []
        for index in range(1, num):
            x, y, w, h, _area = stats[index]
            if not (min_bbox <= w * h <= max_bbox):
                continue
            if x <= 0 or y <= 0 or x + w >= width - 2 or y + h >= height - 2:
                continue
            physical_width = float(w) * resolution
            physical_height = float(h) * resolution
            if not (self.shaft_min_side_m <= physical_width <= self.shaft_max_side_m
                    and self.shaft_min_side_m <= physical_height <= self.shaft_max_side_m):
                continue
            # 井道应该是"薄墙围成的空腔"：内部应主要是自由
            interior_free = float(np.mean(free[y:y + h, x:x + w]))
            if interior_free < 0.45:
                continue
            component = (labels == index)
            # 四条边界线的墙支撑（用于确认门缝所在的井道有其余三面墙）。
            # 井道墙就在组件包围盒的最外一行/列上（x / x+w-1），不是在包围盒外一格。
            boundaries = {
                "west": occupied[y:y + h, x] != 0,
                "east": occupied[y:y + h, x + w - 1] != 0,
                "south": occupied[y, x:x + w] != 0,
                "north": occupied[y + h - 1, x:x + w] != 0,
            }
            edge_yaws = {
                "west": 0.0, "east": math.pi,
                "north": -math.pi / 2.0, "south": math.pi / 2.0,
            }
            for gx, gy, into_yaw in self._perimeter_door_gaps(component):
                if not (0 <= gy < height and 0 <= gx < width):
                    continue
                if not free[gy, gx]:
                    continue
                # 门缝所在边的其余三面墙支撑必须足够，避免把房间/走廊门洞当井道
                other_support = [
                    float(np.mean(support))
                    for edge_name, support in boundaries.items()
                    if abs(math.atan2(
                        math.sin(into_yaw - edge_yaws[edge_name]),
                        math.cos(into_yaw - edge_yaws[edge_name]),
                    )) >= 0.10
                ]
                if any(value < minimum_support for value in other_support):
                    continue
                size_score = 0.5 * sum(
                    0.5 + 0.5 * max(
                        0.0,
                        1.0 - abs(side - target_side) / half_side_range,
                    ) for side in (physical_width, physical_height)
                )
                # 组件法已通过"墙+门缝+三墙支撑"验证，结构可靠，给较高置信
                base_score = 0.5 + 0.4 * size_score
                wx, wy = self._map_to_world(gx, gy)
                candidates.append(ElevatorHallCandidate(
                    x=wx,
                    y=wy,
                    into_yaw=into_yaw,
                    score=min(1.0, base_score),
                    source="geometry_component",
                    confidence=min(1.0, base_score),
                    validated=False,
                ))
        return candidates

    def _detect_elevator_halls(self):
        """Detect strict passive shaft candidates from the active map only."""
        if self.map_data is None:
            return []
        occupied = (self.map_data >= self.connectivity_occupied_threshold).astype(np.uint8)
        free = (self.map_data >= 0) & (self.map_data < self.free_threshold)
        candidates = self._closed_space_hall_candidates(occupied, free)
        # 闭运算法在真实地图上可能无法完全封闭井道，导致漏检；组件法兜底。
        candidates.extend(self._component_shaft_candidates(occupied, free))
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

    def _fixed_elevator_hall_candidate(self):
        """Return the explicit simulation-only hall without validating it."""
        if not bool(getattr(self, "fixed_elevator_hall_enabled", False)):
            return None
        return ElevatorHallCandidate(
            x=float(self.fixed_elevator_hall_x),
            y=float(self.fixed_elevator_hall_y),
            into_yaw=float(self.fixed_elevator_hall_into_yaw),
            score=1.0,
            source="fixed_test",
            confidence=1.0,
            validated=False,
        )

    def _hall_candidates_for_floor_change(self, now):
        """Select either the fixed test hall or the normal discovery results."""
        fixed_hall = self._fixed_elevator_hall_candidate()
        if fixed_hall is not None:
            # This test mode deliberately isolates the transit state machine
            # from both active discovery and passive geometry detection.  The
            # candidate remains unvalidated so the real door close/reopen scan
            # check still runs before ENTER.
            return [fixed_hall]

        cached_hall = self._cached_hall_candidate()
        self._observe_elevator_halls(now)
        halls = self._confirmed_elevator_halls(now)
        if cached_hall is not None and not any(
                math.hypot(cached_hall[0] - hall[0], cached_hall[1] - hall[1])
                < 0.35 for hall in halls):
            halls.insert(0, cached_hall)
        return halls

    def _observe_elevator_halls(self, now):
        """Accumulate passive evidence over distinct map versions."""
        active_context = getattr(self, "accepted_map_context", None)
        if (active_context is None
                or tuple(int(value) for value in active_context[:2]) != (
                    int(self.current_floor), int(self.map_epoch)
                )):
            return
        active_version = int(active_context[2])
        context = (
            int(self.current_floor), int(self.map_epoch),
            self.accepted_map_load_identity,
        )
        observation_key = context + (active_version,)
        if self.elevator_hall_last_observed_version == observation_key:
            return
        if getattr(self, "elevator_hall_track_context", None) != context:
            self.elevator_hall_tracks = {}
            self.elevator_hall_track_context = context
        self.elevator_hall_last_observed_version = observation_key
        for candidate in self._detect_elevator_halls():
            matching_key = None
            for key, track in self.elevator_hall_tracks.items():
                previous = track["candidate"]
                if (math.hypot(candidate.x - previous.x,
                               candidate.y - previous.y) < 0.50
                        and abs(math.atan2(
                            math.sin(candidate.into_yaw - previous.into_yaw),
                            math.cos(candidate.into_yaw - previous.into_yaw),
                        )) < 0.35):
                    matching_key = key
                    break
            if matching_key is None:
                matching_key = (
                    round(candidate.x / 0.25), round(candidate.y / 0.25),
                    round(candidate.into_yaw / 0.20),
                )
                self.elevator_hall_tracks[matching_key] = {
                    "candidate": candidate,
                    "first_seen": now,
                    "last_seen": now,
                    "versions": set(),
                }
            track = self.elevator_hall_tracks[matching_key]
            track["candidate"] = candidate
            track["last_seen"] = now
            track["versions"].add(active_version)

    def _confirmed_elevator_halls(self, now):
        candidates = []
        minimum_versions = int(getattr(self, "elevator_hall_min_versions", 3))
        minimum_duration = float(getattr(
            self, "elevator_hall_min_duration_s", 2.0
        ))
        minimum_score = float(getattr(self, "elevator_hall_min_score", 0.75))
        for track in getattr(self, "elevator_hall_tracks", {}).values():
            duration = (track["last_seen"] - track["first_seen"]).to_sec()
            if (len(track["versions"]) < minimum_versions
                    or duration < minimum_duration):
                continue
            candidate = track["candidate"]
            candidate.score = min(1.0, float(candidate.score) + 0.10)
            candidate.confidence = candidate.score
            if candidate.score >= minimum_score:
                candidates.append(candidate)
        return candidates

    def _cached_hall_candidate(self):
        """Return a still-valid hall binding for this floor/elevator pair."""
        key = (int(self.current_floor), str(self.active_elevator_id))
        binding = self.elevator_hall_bindings.get(key)
        if binding is None:
            return None
        if int(binding.get("floor", self.current_floor)) != int(self.current_floor):
            self.elevator_hall_bindings.pop(key, None)
            return None
        current_epoch = int(getattr(self, "map_epoch", 0))
        if int(binding.get("epoch", current_epoch)) != current_epoch:
            self.elevator_hall_bindings.pop(key, None)
            return None
        if binding.get("map_load_identity") != self.accepted_map_load_identity:
            self.elevator_hall_bindings.pop(key, None)
            return None
        hall = tuple(binding.get("hall", ()))
        if len(hall) != 3:
            self.elevator_hall_bindings.pop(key, None)
            return None
        hx, hy, into_yaw = hall
        approach_distance = float(getattr(self, "elevator_hall_approach_m", 0.8))
        approach_x = hx - approach_distance * math.cos(into_yaw)
        approach_y = hy - approach_distance * math.sin(into_yaw)
        map_x, map_y = self._world_to_map(approach_x, approach_y)
        if not self._is_free(map_x, map_y):
            self.elevator_hall_bindings.pop(key, None)
            return None
        path_state = self._check_path(
            self.current_pose.position.x,
            self.current_pose.position.y,
            approach_x,
            approach_y,
        )
        if path_state != "reachable":
            self.elevator_hall_bindings.pop(key, None)
            return None
        return ElevatorHallCandidate(
            x=float(hx),
            y=float(hy),
            into_yaw=float(into_yaw),
            score=float(binding.get("score", binding.get("confidence", 0.0))),
            source=str(binding.get("source", "geometry")),
            confidence=float(binding.get("confidence", 0.0)),
            validated=bool(binding.get("validated", False)),
            path_length=float((getattr(
                self, "last_checked_path_metrics", None
            ) or {}).get(
                "path_length", float("inf")
            )),
        )

    def _remember_validated_hall(self):
        if self.floor_change_hall_point is None:
            return
        candidate = self.floor_change_hall_candidate
        if candidate is None:
            candidate = ElevatorHallCandidate(
                *self.floor_change_hall_point,
                score=1.0,
                source="runtime_door_motion",
                confidence=1.0,
                validated=True,
            )
        else:
            candidate.validated = True
            candidate.confidence = 1.0
            candidate.score = 1.0
            if candidate.source not in ("door_motion", "fixed_test"):
                candidate.source = "runtime_door_motion"
        self._save_hall_binding(
            candidate,
            floor=self.floor_change_start_floor,
            epoch=self.floor_change_start_epoch,
        )

    def _discard_active_hall_binding(self):
        key = (int(self.floor_change_start_floor), str(self.active_elevator_id))
        self.elevator_hall_bindings.pop(key, None)

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
        healthy, reason = self._inputs_health(rospy.Time.now())
        if not healthy:
            self._record_floor_change_result(
                False, "UNREACHABLE_HALL",
                "floor transit input contract is not ready: " + reason,
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
        self.floor_change_hall_candidate = None
        self._invalidate_service()
        self._stop_elevator_motion()

        # Retire the ordinary goal epoch before selecting an elevator goal.
        self.goal_id += 1
        self.move_base_client.cancel_all_goals()
        self.waiting_for_result = False
        self.current_goal = None
        self.last_recovery_goal_id = ""
        self.nav_active_goal_id = ""
        self.navigation_goal_sent_at = rospy.Time(0)
        self._floor_change_goal_succeeded = None

        self.elevator_halls = self._hall_candidates_for_floor_change(now)
        if not self.elevator_halls:
            self._floor_change_fail("NO_HALL", "no elevator hall candidate")
            return False
        cx = self.current_pose.position.x
        cy = self.current_pose.position.y
        self.elevator_halls.sort(key=lambda hall: (
            0 if bool(getattr(hall, "validated", False)) else 1,
            -float(getattr(hall, "score", 0.0)),
            math.hypot(hall[0] - cx, hall[1] - cy),
        ))
        self.elevator_hall_index = 0
        self._set_floor_change_phase("TO_HALL", "select_elevator_hall")
        return self._pick_elevator_hall_and_send()

    def _pick_elevator_hall_and_send(self):
        """Plan to the 0.8 m hall stand-off; consume each candidate once."""
        cx = self.current_pose.position.x
        cy = self.current_pose.position.y
        had_candidate = self.elevator_hall_index < len(self.elevator_halls)
        if self.elevator_hall_index == 0:
            ranked = []
            for raw_candidate in self.elevator_halls:
                candidate = raw_candidate if isinstance(
                    raw_candidate, ElevatorHallCandidate
                ) else ElevatorHallCandidate(
                    *raw_candidate, score=1.0, confidence=1.0
                )
                if (not candidate.validated
                        and candidate.score < float(getattr(
                            self, "elevator_hall_min_score", 0.75
                        ))):
                    continue
                approach_x = candidate.x - self.elevator_hall_approach_m * math.cos(
                    candidate.into_yaw
                )
                approach_y = candidate.y - self.elevator_hall_approach_m * math.sin(
                    candidate.into_yaw
                )
                approach_distance = math.hypot(
                    approach_x - cx, approach_y - cy
                )
                if (bool(getattr(self, "fixed_elevator_hall_enabled", False))
                        and approach_distance <= self.plan_tolerance):
                    candidate.approach_x = float(cx)
                    candidate.approach_y = float(cy)
                    candidate.path_length = 0.0
                    ranked.append(candidate)
                    continue
                map_x, map_y = self._world_to_map(approach_x, approach_y)
                if not self._is_free(map_x, map_y):
                    continue
                if self._check_path(cx, cy, approach_x, approach_y) != "reachable":
                    continue
                metrics = self.last_checked_path_metrics or {}
                candidate.path_length = float(metrics.get(
                    "path_length", math.hypot(approach_x - cx, approach_y - cy)
                ))
                points = metrics.get("points") or []
                if points:
                    planned_x, planned_y = points[-1]
                    if (math.hypot(planned_x - approach_x, planned_y - approach_y)
                            <= self.plan_tolerance + 1e-6):
                        planned_map_x, planned_map_y = self._world_to_map(
                            planned_x, planned_y
                        )
                        if self._is_free(planned_map_x, planned_map_y):
                            candidate.approach_x = float(planned_x)
                            candidate.approach_y = float(planned_y)
                ranked.append(candidate)
            ranked.sort(key=lambda candidate: (
                0 if candidate.validated else 1,
                -candidate.score,
                candidate.path_length,
                math.hypot(candidate.x - cx, candidate.y - cy),
            ))
            self.elevator_halls = ranked
            # Preserve the distinction between "nothing was detected" and a
            # known hall whose stand-off or Navfn path is unreachable.
            had_candidate = had_candidate or bool(ranked)
        while self.elevator_hall_index < len(self.elevator_halls):
            candidate = self.elevator_halls[self.elevator_hall_index]
            hx, hy, into_yaw = candidate
            self.elevator_hall_index += 1
            nominal_approach_x = (
                hx - self.elevator_hall_approach_m * math.cos(into_yaw)
            )
            nominal_approach_y = (
                hy - self.elevator_hall_approach_m * math.sin(into_yaw)
            )
            approach_x = float(getattr(candidate, "approach_x", float("nan")))
            approach_y = float(getattr(candidate, "approach_y", float("nan")))
            if not (math.isfinite(approach_x) and math.isfinite(approach_y)):
                approach_x = nominal_approach_x
                approach_y = nominal_approach_y
            path_length = float(getattr(candidate, "path_length", float("inf")))
            if not math.isfinite(path_length):
                path_length = math.hypot(approach_x - cx, approach_y - cy)
            navigation_timeout = min(
                self.elevator_hall_navigation_max_s,
                max(30.0, 20.0 + 2.5 * path_length
                    / self.elevator_hall_nominal_speed_mps),
            )
            self.floor_change_hall_point = (hx, hy, into_yaw)
            self.floor_change_hall_candidate = candidate
            self.floor_change_car_point = (
                hx + self.elevator_car_target_m * math.cos(into_yaw),
                hy + self.elevator_car_target_m * math.sin(into_yaw),
            )
            self._floor_change_goal_succeeded = None
            if (bool(getattr(self, "fixed_elevator_hall_enabled", False))
                    and math.hypot(approach_x - cx, approach_y - cy)
                    <= self.plan_tolerance):
                self._floor_change_goal_succeeded = True
                self._set_floor_change_phase(
                    "OPEN_CURRENT_START", "already_at_fixed_hall_approach"
                )
                return True
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

    def _robot_is_on_hall_side(self):
        if self.current_pose is None or self.floor_change_hall_point is None:
            return False
        hall_x, hall_y, into_yaw = self.floor_change_hall_point
        signed_into_distance = (
            (self.current_pose.position.x - hall_x) * math.cos(into_yaw)
            + (self.current_pose.position.y - hall_y) * math.sin(into_yaw)
        )
        return signed_into_distance <= -self.elevator_exit_hall_side_margin_m

    def _robot_is_inside_cabin(self):
        if self.current_pose is None or self.floor_change_hall_point is None:
            return False
        hall_x, hall_y, into_yaw = self.floor_change_hall_point
        signed_into_distance = (
            (self.current_pose.position.x - hall_x) * math.cos(into_yaw)
            + (self.current_pose.position.y - hall_y) * math.sin(into_yaw)
        )
        return signed_into_distance >= self.elevator_entry_cabin_side_margin_m

    def _entry_distance_remaining(self):
        if self.current_pose is None or self.floor_change_hall_point is None:
            return self.floor_change_crossing_target_m
        hall_x, hall_y, into_yaw = self.floor_change_hall_point
        signed_into_distance = (
            (self.current_pose.position.x - hall_x) * math.cos(into_yaw)
            + (self.current_pose.position.y - hall_y) * math.sin(into_yaw)
        )
        return max(
            0.0,
            self.elevator_entry_cabin_side_margin_m - signed_into_distance,
        )

    def _advance_crossing(self, now):
        entering = self.floor_change_crossing_direction > 0.0
        failure_code = "ENTER_FAILED" if entering else "EXIT_FAILED"
        if self.current_pose is None or now > self.floor_change_stage_deadline:
            self._stop_elevator_motion()
            self._floor_change_fail(failure_code, "elevator crossing timed out")
            return
        if not entering and self._robot_is_on_hall_side():
            self._stop_elevator_motion()
            self.floor_change_stable_since = rospy.Time(0)
            self._set_floor_change_phase(
                "WAIT_STABLE", "reached_target_hall_side"
            )
            return
        if entering and self._robot_is_inside_cabin():
            self._stop_elevator_motion()
            self._set_floor_change_phase(
                "CLOSE_CURRENT_START", "fully_inside_elevator_cabin"
            )
            return
        start_x, start_y = self.floor_change_crossing_start
        progress = math.hypot(
            self.current_pose.position.x - start_x,
            self.current_pose.position.y - start_y,
        )
        if progress >= self.floor_change_crossing_target_m:
            if not entering:
                self._stop_elevator_motion()
                self.floor_change_stable_since = rospy.Time(0)
                self._set_floor_change_phase("WAIT_STABLE", "wait_navigation_epoch")
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
        remaining = max(0.0, self.floor_change_crossing_target_m - progress)
        if entering:
            remaining = max(remaining, self._entry_distance_remaining())
        swept_obstacle = swept_footprint_obstacle(
            self.latest_scan.ranges,
            self.latest_scan.angle_min,
            self.latest_scan.angle_increment,
            self.latest_scan.range_min,
            self.latest_scan.range_max,
            self.floor_change_crossing_direction,
            remaining,
            (
                self.elevator_footprint_min_x,
                self.elevator_footprint_max_x,
                self.elevator_footprint_min_y,
                self.elevator_footprint_max_y,
            ),
            self.elevator_footprint_margin_m,
        )
        if (clearance <= self.elevator_crossing_clearance_m
                or swept_obstacle is not None):
            self._stop_elevator_motion()
            if progress < self.elevator_crossing_min_progress_m:
                self._floor_change_fail(
                    failure_code,
                    "obstacle intersects elevator swept footprint",
                )
            elif entering:
                self._floor_change_fail(
                    failure_code,
                    "obstacle detected before full elevator cabin entry",
                )
            else:
                self.floor_change_stable_since = rospy.Time(0)
                self._set_floor_change_phase("WAIT_STABLE", "wait_navigation_epoch")
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

    @staticmethod
    def _stamp_is_fresh(stamp, now, timeout):
        return (stamp != rospy.Time(0)
                and (now - stamp).to_sec() <= float(timeout))

    def _transit_phase_health(self, now):
        """Apply only the dependencies that are valid for the active phase."""
        step = str(self.floor_change_step or "")
        if step == "STOPPING":
            return True, "", ""
        if self.safety_stop_active:
            return False, "CANCELED", "safety stop is active"

        if self.current_pose is None or not self._stamp_is_fresh(
                self.last_pose_time, now, self.input_timeout):
            code = "MAP_NOT_STABLE" if step == "WAIT_STABLE" else (
                "EXIT_FAILED" if step == "EXIT" else "UNREACHABLE_HALL"
                if step == "TO_HALL" else "ENTER_FAILED"
            )
            return False, code, "localized pose is stale"
        if not self._stamp_is_fresh(
                self.last_mapping_status_time, now, self.input_timeout):
            code = "MAP_NOT_STABLE" if step == "WAIT_STABLE" else (
                "UNREACHABLE_HALL" if step == "TO_HALL"
                else "SERVICE_UNAVAILABLE"
            )
            return False, code, "mapping status is stale"

        if step == "TO_HALL":
            expected = (
                int(self.current_floor), int(self.map_epoch),
                int(self.current_map_version),
            )
            if self.mapping_lost:
                return False, "UNREACHABLE_HALL", "hall navigation mapping is lost"
            if self.mapping_transitioning:
                return (
                    False, "UNREACHABLE_HALL",
                    "hall navigation mapping is transitioning",
                )
            if not map_context_is_committed(
                    expected, self.accepted_map_context):
                return (
                    False,
                    "UNREACHABLE_HALL",
                    "hall navigation active map context is not committed: "
                    "mapping=%r active=%r"
                    % (expected, self.accepted_map_context),
                )
            if not self._stamp_is_fresh(
                    self.last_map_time, now, self.input_timeout):
                return False, "UNREACHABLE_HALL", "hall navigation map is stale"
            if not self.mapping_ready or not self.mapping_stable:
                # The floor-change entry gate already requires ready/stable.
                # Once move_base starts, recovery turns can temporarily make
                # localization report degraded/stable=false even though the
                # pose, active map and floor/epoch contract remain healthy.
                # Keep observing the flags, but do not cancel a valid goal on
                # that transient diagnostic state.
                rospy.logwarn_throttle(
                    2.0,
                    "[exploration] hall navigation continuing with committed "
                    "map while mapping readiness is degraded: ready=%s "
                    "stable=%s mapping=%r active=%r",
                    self.mapping_ready,
                    self.mapping_stable,
                    expected,
                    self.accepted_map_context,
                )
            return True, "", ""

        if step == "WAIT_STABLE":
            if self.waiting_for_result or self.nav_has_active_goal:
                return False, "MAP_NOT_STABLE", "ordinary navigation goal survived transit"
            if not self._stamp_is_fresh(
                    self.last_map_time, now, self.input_timeout):
                rospy.logwarn_throttle(
                    2.0,
                    "[exploration] waiting for fresh active map after floor "
                    "switch",
                )
            if not self._stamp_is_fresh(
                    self.last_nav_health_time, now, self.input_timeout):
                rospy.logwarn_throttle(
                    2.0,
                    "[exploration] waiting for fresh navigation health after "
                    "floor switch",
                )
            return True, "", ""

        if self.waiting_for_result or self.nav_has_active_goal:
            code = "EXIT_FAILED" if step == "EXIT" else "ENTER_FAILED"
            return False, code, "ordinary move_base goal is active during transit"

        scan_required_steps = {
            "OPEN_CURRENT_START", "OPEN_CURRENT_WAIT", "CAPTURE_OPEN_SCAN",
            "VALIDATE_CLOSE_START", "VALIDATE_CLOSE_WAIT",
            "CAPTURE_CLOSED_SCAN", "REOPEN_CURRENT_START",
            "REOPEN_CURRENT_WAIT", "ENTER", "EXIT",
        }
        if step in scan_required_steps and (
                self.latest_scan is None or not self._stamp_is_fresh(
                    self.last_scan_time, now, self.input_timeout)):
            code = "EXIT_FAILED" if step == "EXIT" else "ENTER_FAILED"
            return False, code, "laser scan is stale during elevator transit"

        return True, "", ""

    def _advance_floor_change(self, now):
        if self.floor_change_step == "STOPPING":
            self._stop_elevator_motion()
            zero_confirmed = self._control_output_is_zero(now)
            if zero_confirmed or now >= self.floor_change_stop_deadline:
                self._finalize_floor_change_failure(zero_confirmed)
            return
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
            if self.nav_has_active_goal:
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
            self.initial_hall_discovery_door_held_closed = False
            if (self.hall_validation_required
                    and not bool(getattr(
                        self.floor_change_hall_candidate, "validated", False
                    ))):
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
                self._discard_active_hall_binding()
                self._retry_hall_or_fail(
                    "NO_HALL", "door motion did not change the local scan"
                )
                return
            self._remember_validated_hall()
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
                if self._robot_is_on_hall_side():
                    self.floor_change_stable_since = rospy.Time(0)
                    self._set_floor_change_phase(
                        "WAIT_STABLE", "already_on_target_hall_side"
                    )
                else:
                    self._start_crossing(-1.0)
            else:
                self.floor_change_stable_since = rospy.Time(0)
                self._set_floor_change_phase("WAIT_STABLE", "wait_navigation_epoch")
            return

        if step == "WAIT_STABLE":
            floor_ok = self.current_floor == self.floor_change_target
            epoch_ok = self.map_epoch == self.floor_change_expected_epoch
            minimum_version = (
                self.floor_change_start_target_version
                + self.floor_min_new_map_versions
            )
            version_ok = self.current_map_version >= minimum_version
            active_context_ok = (
                self.accepted_map_context is not None
                and self.accepted_map_context[:2] == (
                    int(self.floor_change_target),
                    int(self.floor_change_expected_epoch),
                )
                and int(self.accepted_map_context[2]) >= minimum_version
            )
            navigation_context_ok = (
                (self.nav_floor, self.nav_map_epoch) == (
                    int(self.floor_change_target),
                    int(self.floor_change_expected_epoch),
                )
                and self.nav_map_version >= minimum_version
            )
            stable_now = (
                floor_ok and epoch_ok and version_ok
                and self.mapping_ready and self.mapping_stable
                and not self.mapping_transitioning
                and active_context_ok
                and self.nav_ready and not self.nav_transitioning
                and navigation_context_ok
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
        if self.floor_change_step == "STOPPING":
            self._stop_elevator_motion()
            return
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
        fatal_steps = {
            "ENTER", "CLOSE_CURRENT_START", "CLOSE_CURRENT_WAIT",
            "CALL_TARGET_START", "CALL_TARGET_WAIT", "SWITCH_FLOOR_START",
            "SWITCH_FLOOR_WAIT", "EXIT", "WAIT_STABLE",
        }
        self._pending_floor_failure = {
            "failure_code": str(failure_code),
            "message": str(message),
            "fatal": failed_step in fatal_steps,
        }
        self.floor_change_step = "STOPPING"
        self.floor_change_stop_deadline = rospy.Time.now() + rospy.Duration(1.0)
        self._set_state("FLOOR_CHANGE", "confirm_control_stopped")

    def _finalize_floor_change_failure(self, zero_confirmed):
        pending = dict(self._pending_floor_failure or {})
        failure_code = str(pending.get("failure_code", "SERVICE_UNAVAILABLE"))
        message = str(pending.get("message", "floor transit failed"))
        if not zero_confirmed:
            message += "; control zero output was not confirmed within 1.0 s"
        self.floor_change_active = False
        self.floor_change_step = None
        self._pending_floor_failure = None
        self.last_recovery_goal_id = ""
        self.nav_active_goal_id = ""
        self.navigation_goal_sent_at = rospy.Time(0)
        self._record_floor_change_result(False, failure_code, message)
        self.floor_change_gave_up_count += 1
        self.floor_change_retry_after = rospy.Time.now() + rospy.Duration(
            self.map_stable_time
        )
        if not self.floor_change_external and bool(pending.get("fatal", False)):
            self.floor_transit_fatal = True
            self._set_state("FAILED", "floor_change_failed:" + failure_code)
        elif not self.exploring and not self.floor_change_external:
            self._set_state("STOPPED", "stop_requested")
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
        self.floor_unreachable_since = rospy.Time(0)
        self.last_significant_map_change = rospy.Time.now()
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
        self.last_recovery_goal_id = ""
        self.nav_active_goal_id = ""
        self.navigation_goal_sent_at = rospy.Time(0)
        self._record_floor_change_result(True, "", "target floor reached and stable")
        self._set_state("EXPLORE_FLOOR", "new_floor_reached")

    def _transit_timer_callback(self, _event):
        with self.state_lock:
            if not self.floor_change_active:
                return
            if self.floor_change_step in {
                    "ENTER", "CLOSE_CURRENT_START", "CLOSE_CURRENT_WAIT",
                    "CALL_TARGET_START", "CALL_TARGET_WAIT",
                    "SWITCH_FLOOR_START", "SWITCH_FLOOR_WAIT", "EXIT"}:
                self._publish_mapping_pause()
            try:
                now = rospy.Time.now()
                healthy, failure_code, detail = self._transit_phase_health(now)
                if not healthy:
                    self._floor_change_fail(failure_code, detail)
                    return
                self._advance_floor_change(now)
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
            "WAIT_STABLE": 0.92,
            "STOPPING": 0.99,
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
