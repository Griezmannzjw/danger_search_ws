#!/usr/bin/env python3
"""P0 mission manager: explore, return home, and atomically save results."""

import copy
import json
import math
import os
import threading

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from danger_search_common.msg import (
    DangerSource,
    DangerSourceArray,
    DetectionStatus,
    MappingStatus,
    MissionStatus,
    NavigationHealth,
    TransitFloorAction,
    TransitFloorGoal,
)
from danger_search_common.short_range_safety import (
    swept_arc_footprint_hit,
    swept_footprint_hit,
    swept_footprint_hits,
)
from danger_search_mission.entry_core import (
    CrossingReference,
    ProgressWatchdog,
    YawErrorFilter,
    command_is_zero,
    crossing_command,
    margin_avoidance_command,
    normalize_angle,
    quaternion_roll_pitch,
    rolling_sweep_distance,
)
from danger_search_mission.mission_core import (
    allocate_return_attempt_budget,
    build_result_document,
    DangerTrackStore,
    DEFAULT_ENTRY_COMPLETION_TOLERANCE_M,
    entry_progress,
    exploration_failure_reason,
    MissionLifecycle,
    next_entry_target,
    normalize_run_profile,
    normalize_result_file,
    parse_public_scene_contract,
    PostureSafetyGate,
    resolve_result_coordinate_frame,
)
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse


class MissionManager:
    """Own the task-level state machine and the return-home goal."""

    def __init__(self):
        rospy.init_node("mission_manager", anonymous=False)

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.pose_topic = rospy.get_param("~pose_topic", "/localization/pose")
        self.detections_topic = rospy.get_param(
            "~detections_topic", "/danger_detector/detections"
        )
        self.detection_status_topic = rospy.get_param(
            "~detection_status_topic", "/danger_detector/status"
        )
        self.mapping_status_topic = rospy.get_param(
            "~mapping_status_topic", "/mapping/status"
        )
        self.navigation_health_topic = rospy.get_param(
            "~navigation_health_topic", "/navigation/health"
        )
        self.exploration_status_topic = rospy.get_param(
            "~exploration_status_topic", "/exploration/status"
        )
        self.exploration_complete_topic = rospy.get_param(
            "~exploration_complete_topic", "/exploration/complete"
        )
        self.mission_status_topic = rospy.get_param(
            "~mission_status_topic", "/mission/status"
        )
        self.mission_active_topic = rospy.get_param(
            "~mission_active_topic", "/mission/active"
        )
        self.entrance_ready_topic = rospy.get_param(
            "~entrance_ready_topic", "/entrance/ready"
        )
        self.preflight_ready_topic = rospy.get_param(
            "~preflight_ready_topic", "/danger_search/preflight_ready"
        )
        self.sent_cmd_topic = rospy.get_param(
            "~sent_cmd_topic", "/danger_search/cmd_vel_sent"
        )
        self.entry_cmd_topic = rospy.get_param(
            "~entry_cmd_topic", "/danger_search/entry_cmd_vel"
        )
        self.scan_topic = rospy.get_param("~scan_topic", "/localization/scan")
        self.imu_topic = rospy.get_param("~imu_topic", "/trunk_imu")
        self.safety_stop_topic = rospy.get_param(
            "~safety_stop_topic", "/danger_search/safety_stop"
        )
        self.posture_fallen_topic = rospy.get_param(
            "~posture_fallen_topic", "/danger_search/posture_fallen"
        )
        self.posture_reason_topic = rospy.get_param(
            "~posture_reason_topic", "/danger_search/posture_safety_reason"
        )
        self.recoverable_safety_abort_s = self._positive_param(
            "~recoverable_safety_abort_s", 3.0
        )

        self.start_exploration_service = rospy.get_param(
            "~start_exploration_service", "/danger_search/start_exploration"
        )
        self.stop_exploration_service = rospy.get_param(
            "~stop_exploration_service", "/danger_search/stop_exploration"
        )
        self.start_mission_service = rospy.get_param(
            "~start_mission_service", "/danger_search/start"
        )
        self.finish_mission_service = rospy.get_param(
            "~finish_mission_service", "/danger_search/finish"
        )
        self.return_home_service = rospy.get_param(
            "~return_home_service", "/danger_search/return_home"
        )
        self.move_base_action_name = rospy.get_param(
            "~move_base_action_name", "/move_base"
        )
        self.transit_floor_action_name = rospy.get_param(
            "~transit_floor_action_name", "/danger_search/transit_floor"
        )

        try:
            self.run_profile = normalize_run_profile(rospy.get_param(
                "~run_profile", rospy.get_param("/run_profile", "formal")
            ))
            self.result_file = normalize_result_file(
                rospy.get_param("~result_file", ""), self.run_profile
            )
            self.tracker = DangerTrackStore(
                dedup_distance_m=float(rospy.get_param("~dedup_distance", 0.8)),
                min_detections=int(rospy.get_param("~min_detections", 3)),
                min_confidence=float(rospy.get_param("~min_confidence", 0.6)),
            )
        except ValueError as exc:
            raise rospy.ROSInitException(str(exc))

        self.preflight_wait_timeout_s = self._positive_param(
            "~preflight_wait_timeout_s", 20.0
        )
        self.return_timeout_s = self._positive_param("~return_timeout_s", 120.0)
        self.return_attempt_timeout_s = self._positive_param(
            "~return_attempt_timeout_s", 150.0
        )
        self.return_retry_reserve_s = self._nonnegative_param(
            "~return_retry_reserve_s", 15.0
        )
        self.return_terminal_reserve_s = self._positive_param(
            "~return_terminal_reserve_s", 5.0
        )
        self.return_min_attempt_timeout_s = self._positive_param(
            "~return_min_attempt_timeout_s", 5.0
        )
        if self.return_attempt_timeout_s > self.return_timeout_s:
            raise rospy.ROSInitException(
                "~return_attempt_timeout_s cannot exceed ~return_timeout_s"
            )
        self.return_position_tolerance_m = self._positive_param(
            "~return_position_tolerance_m", 0.5
        )
        self.return_yaw_tolerance_rad = self._positive_param(
            "~return_yaw_tolerance_rad", math.radians(20.0)
        )
        self.return_stationary_hold_s = self._positive_param(
            "~return_stationary_hold_s", 2.0
        )
        self.return_verify_timeout_s = self._positive_param(
            "~return_verify_timeout_s", 15.0
        )
        self.return_retry_delay_s = self._positive_param(
            "~return_retry_delay_s", 0.3
        )
        self.return_stationary_linear_mps = self._nonnegative_param(
            "~return_stationary_linear_mps", 0.02
        )
        self.return_stationary_angular_rps = self._nonnegative_param(
            "~return_stationary_angular_rps", 0.05
        )
        self.return_max_goal_attempts = int(
            rospy.get_param("~return_max_goal_attempts", 2)
        )
        if self.return_max_goal_attempts < 1:
            raise rospy.ROSInitException("~return_max_goal_attempts must be at least one")
        self.home_floor = int(rospy.get_param("~home_floor", 0))
        if self.home_floor < 0:
            raise rospy.ROSInitException("~home_floor cannot be negative")
        self.result_coordinate_frame = rospy.get_param(
            "~result_coordinate_frame", "auto"
        )
        self.scene_info_file = str(rospy.get_param("~scene_info_file", "")).strip()
        self.scene_contract = self._load_scene_contract(self.scene_info_file)
        self.output_coordinate_frame = resolve_result_coordinate_frame(
            self.result_coordinate_frame,
            self.scene_contract.get("coordinate_frame"),
        )
        self.entry_timeout_s = self._positive_param("~entry_timeout_s", 90.0)
        self.entry_distance_m = self._positive_param("~entry_distance_m", 4.2)
        self.entry_step_m = self._positive_param("~entry_step_m", 0.6)
        self.entry_retry_delay_s = self._positive_param(
            "~entry_retry_delay_s", 1.0
        )
        self.entry_map_retry_delay_s = self._positive_param(
            "~entry_map_retry_delay_s", 2.0
        )
        self.entry_health_settle_s = self._positive_param(
            "~entry_health_settle_s", 0.3
        )
        self.entry_completion_tolerance_m = self._nonnegative_param(
            "~entry_completion_tolerance_m", DEFAULT_ENTRY_COMPLETION_TOLERANCE_M
        )
        self.entry_min_progress_m = self._nonnegative_param(
            "~entry_min_progress_m", 0.10
        )
        self.entry_short_range_enabled = bool(rospy.get_param(
            "~entry_short_range_enabled", True
        ))
        self.entry_crossing_distance_m = self._positive_param(
            "~entry_crossing_distance_m", 3.75
        )
        self.entry_crossing_speed_mps = self._positive_param(
            "~entry_crossing_speed_mps", 0.40
        )
        self.entry_crossing_max_angular_speed_rps = self._nonnegative_param(
            "~entry_crossing_max_angular_speed_rps", 0.30
        )
        self.entry_crossing_yaw_gain = self._nonnegative_param(
            "~entry_crossing_yaw_gain", 1.50
        )
        self.entry_crossing_yaw_deadband_rad = self._nonnegative_param(
            "~entry_crossing_yaw_deadband_rad", 0.05
        )
        self.entry_crossing_yaw_lowpass_alpha = self._unit_interval_param(
            "~entry_crossing_yaw_lowpass_alpha", 0.25
        )
        self.entry_crossing_lateral_gain = self._nonnegative_param(
            "~entry_crossing_lateral_gain", 0.80
        )
        self.entry_crossing_max_lateral_error_m = self._positive_param(
            "~entry_crossing_max_lateral_error_m", 0.35
        )
        self.entry_crossing_max_yaw_error_rad = self._positive_param(
            "~entry_crossing_max_yaw_error_rad", 0.45
        )
        self.entry_crossing_max_tilt_rad = self._positive_param(
            "~entry_crossing_max_tilt_rad", 0.30
        )
        self.entry_crossing_timeout_s = self._positive_param(
            "~entry_crossing_timeout_s", 35.0
        )
        self.entry_crossing_input_timeout_s = self._positive_param(
            "~entry_crossing_input_timeout_s", 0.75
        )
        self.entry_crossing_prepare_zero_s = self._positive_param(
            "~entry_crossing_prepare_zero_s", 0.50
        )
        self.entry_crossing_handoff_zero_s = self._positive_param(
            "~entry_crossing_handoff_zero_s", 0.50
        )
        self.entry_crossing_release_s = self._positive_param(
            "~entry_crossing_release_s", 0.35
        )
        self.entry_crossing_stall_increment_m = self._positive_param(
            "~entry_crossing_stall_increment_m", 0.03
        )
        self.entry_crossing_stall_timeout_s = self._positive_param(
            "~entry_crossing_stall_timeout_s", 6.0
        )
        self.entry_crossing_sweep_enabled = bool(rospy.get_param(
            "~entry_crossing_sweep_enabled", True
        ))
        self.entry_crossing_sweep_lookahead_m = self._positive_param(
            "~entry_crossing_sweep_lookahead_m", 0.80
        )
        self.entry_footprint_min_x = float(rospy.get_param(
            "~entry_footprint_min_x", -0.35
        ))
        self.entry_footprint_max_x = float(rospy.get_param(
            "~entry_footprint_max_x", 0.30
        ))
        self.entry_footprint_min_y = float(rospy.get_param(
            "~entry_footprint_min_y", -0.15
        ))
        self.entry_footprint_max_y = float(rospy.get_param(
            "~entry_footprint_max_y", 0.15
        ))
        self.entry_footprint_margin_m = self._nonnegative_param(
            "~entry_footprint_margin_m", 0.05
        )
        self.entry_margin_avoidance_speed_mps = self._positive_param(
            "~entry_margin_avoidance_speed_mps", 0.25
        )
        self.entry_margin_avoidance_yaw_rps = self._positive_param(
            "~entry_margin_avoidance_yaw_rps", 0.08
        )
        self.entry_margin_avoidance_lookahead_m = self._positive_param(
            "~entry_margin_avoidance_lookahead_m", 0.30
        )
        self.entry_margin_min_side_clearance_m = self._positive_param(
            "~entry_margin_min_side_clearance_m", 0.01
        )
        self.entry_margin_release_hysteresis_m = self._nonnegative_param(
            "~entry_margin_release_hysteresis_m", 0.02
        )
        self.entry_arc_sweep_sample_spacing_m = self._positive_param(
            "~entry_arc_sweep_sample_spacing_m", 0.01
        )
        self.entry_max_retries = int(rospy.get_param("~entry_max_retries", 8))
        if self.entry_max_retries < 0:
            raise rospy.ROSInitException("~entry_max_retries must be non-negative")
        if self.entry_completion_tolerance_m >= self.entry_distance_m:
            raise rospy.ROSInitException(
                "~entry_completion_tolerance_m must be smaller than entry distance"
            )
        if self.entry_crossing_distance_m >= self.entry_distance_m:
            raise rospy.ROSInitException(
                "~entry_crossing_distance_m must be smaller than entry distance"
            )
        if (self.entry_short_range_enabled
                and self.entry_crossing_distance_m + 1e-9
                < self.entry_distance_m - self.entry_completion_tolerance_m):
            raise rospy.ROSInitException(
                "~entry_crossing_distance_m must reach the entry completion "
                "boundary before move_base handoff"
            )
        if self.entry_crossing_yaw_deadband_rad >= self.entry_crossing_max_yaw_error_rad:
            raise rospy.ROSInitException(
                "~entry_crossing_yaw_deadband_rad must be smaller than "
                "~entry_crossing_max_yaw_error_rad"
            )
        if self.entry_crossing_sweep_lookahead_m > self.entry_crossing_distance_m:
            raise rospy.ROSInitException(
                "~entry_crossing_sweep_lookahead_m must not exceed the "
                "crossing distance"
            )
        if self.entry_margin_avoidance_speed_mps > self.entry_crossing_speed_mps:
            raise rospy.ROSInitException(
                "~entry_margin_avoidance_speed_mps must not exceed crossing speed"
            )
        if (self.entry_margin_avoidance_yaw_rps
                > self.entry_crossing_max_angular_speed_rps):
            raise rospy.ROSInitException(
                "~entry_margin_avoidance_yaw_rps must not exceed crossing yaw limit"
            )
        if (self.entry_margin_avoidance_lookahead_m
                > self.entry_crossing_sweep_lookahead_m):
            raise rospy.ROSInitException(
                "~entry_margin_avoidance_lookahead_m must not exceed sweep lookahead"
            )
        if self.entry_arc_sweep_sample_spacing_m > 0.01:
            raise rospy.ROSInitException(
                "~entry_arc_sweep_sample_spacing_m must be at most 0.01m"
            )
        footprint = (
            self.entry_footprint_min_x,
            self.entry_footprint_max_x,
            self.entry_footprint_min_y,
            self.entry_footprint_max_y,
        )
        if (not all(math.isfinite(value) for value in footprint)
                or self.entry_footprint_min_x >= self.entry_footprint_max_x
                or self.entry_footprint_min_y >= self.entry_footprint_max_y):
            raise rospy.ROSInitException("entry footprint is invalid")
        self.mission_timeout_s = self._nonnegative_param("~mission_timeout_s", 0.0)
        self.input_timeout_s = self._positive_param("~input_timeout_s", 2.0)
        self.entry_enabled = bool(rospy.get_param("~entry_enabled", True))
        self.require_entrance_ready = bool(
            rospy.get_param("~require_entrance_ready", True)
        )
        self.competition_mode = bool(rospy.get_param(
            "~competition_mode", rospy.get_param("/competition_mode", True)
        ))
        self.multifloor_enabled = bool(rospy.get_param(
            "~multifloor_enabled", rospy.get_param("/multifloor_enabled", True)
        ))
        self.localization_backend = str(rospy.get_param(
            "~localization_backend", rospy.get_param("/localization_backend", "gicp")
        ))
        self.require_preflight_ready = bool(rospy.get_param(
            "~require_preflight_ready", self.competition_mode
        ))
        if self.run_profile == "formal" and (
                not self.competition_mode or not self.multifloor_enabled
                or self.localization_backend != "gicp"):
            raise rospy.ROSInitException(
                "formal mission requires competition_mode=true, multifloor GICP runtime"
            )
        if self.run_profile == "simulation_truth" and (
                self.competition_mode or not self.multifloor_enabled
                or self.localization_backend != "gazebo_truth"):
            raise rospy.ROSInitException(
                "simulation_truth mission requires competition_mode=false, "
                "multifloor gazebo_truth runtime"
            )
        if self.run_profile == "formal" and not self.scene_contract:
            raise rospy.ROSInitException(
                "competition mission requires public team_scene_info.json"
            )
        self.autostart = bool(rospy.get_param("~autostart", False))

        self.lock = threading.RLock()
        self.lifecycle = MissionLifecycle()
        self.start_time = None
        self.finish_time = None
        self.return_start_time = None
        self.entry_start_time = None
        self.home_pose = None
        self.finish_reason = ""
        self.current_floor = 0
        self.current_map_epoch = 0
        self.latest_pose = None
        self.last_pose_time = rospy.Time(0)
        self.mapping_status = None
        self.last_mapping_status_time = rospy.Time(0)
        self.navigation_health = None
        self.last_navigation_health_time = rospy.Time(0)
        self.detection_status = None
        self.last_detection_status_time = rospy.Time(0)
        self.remaining_frontier_count = 0
        self.map_coverage_summary = ""
        self.topology_debt_summary = ""
        self.return_goal_active = False
        self.transit_goal_active = False
        self.return_goal_attempts = 0
        self.return_goal_deadline = rospy.Time(0)
        self.return_epoch = 0
        self.return_retry_at = rospy.Time(0)
        self.return_retry_reason = ""
        self.return_trigger_reason = ""
        self.return_verify_since = rospy.Time(0)
        self.return_verify_deadline = rospy.Time(0)
        self.entry_goal_active = False
        self.entry_goal_sequence = 0
        self.entry_goal_progress_m = 0.0
        self.entry_attempt_start_progress_m = 0.0
        self.entry_retry_count = 0
        self.entry_retry_at = rospy.Time(0)
        self.entry_waiting_for_localization = False
        self.entry_short_range_phase = "INACTIVE"
        self.entry_short_range_epoch = 0
        self.entry_short_range_phase_time = rospy.Time(0)
        self.entry_short_range_started_at = rospy.Time(0)
        self.entry_progress_watchdog = ProgressWatchdog(
            self.entry_crossing_stall_increment_m,
            self.entry_crossing_stall_timeout_s,
        )
        self.entry_crossing_reference = None
        self.entry_margin_avoidance_side = 0.0
        self.entry_crossing_yaw_filter = YawErrorFilter(
            self.entry_crossing_yaw_deadband_rad,
            self.entry_crossing_yaw_lowpass_alpha,
        )
        self.latest_entry_scan = None
        self.last_entry_scan_time = rospy.Time(0)
        self.latest_entry_imu = None
        self.last_entry_imu_time = rospy.Time(0)
        self.safety_stop_active = False
        self.posture_safety = PostureSafetyGate(
            self.recoverable_safety_abort_s
        )
        self.safety_abort_started = False
        self.entrance_ready = not self.require_entrance_ready
        self.preflight_ready = not self.require_preflight_ready
        self.last_sent_cmd = Twist()
        self.last_sent_cmd_time = rospy.Time(0)
        self.exploration_completion_armed = False
        self.shutting_down = False
        self.finalized = False
        self.autostart_attempted = False

        self.status_pub = rospy.Publisher(
            self.mission_status_topic, MissionStatus, queue_size=10, latch=True
        )
        self.active_pub = rospy.Publisher(
            self.mission_active_topic, Bool, queue_size=10, latch=True
        )
        self.entry_cmd_pub = rospy.Publisher(
            self.entry_cmd_topic, Twist, queue_size=2
        )
        self.entrance_ready_sub = rospy.Subscriber(
            self.entrance_ready_topic, Bool, self._entrance_ready_callback, queue_size=2
        )
        self.preflight_ready_sub = rospy.Subscriber(
            self.preflight_ready_topic,
            Bool,
            self._preflight_ready_callback,
            queue_size=2,
        )
        self.sent_cmd_sub = rospy.Subscriber(
            self.sent_cmd_topic, Twist, self._sent_cmd_callback, queue_size=10
        )
        self.entry_scan_sub = rospy.Subscriber(
            self.scan_topic, LaserScan, self._entry_scan_callback, queue_size=2
        )
        self.entry_imu_sub = rospy.Subscriber(
            self.imu_topic, Imu, self._entry_imu_callback, queue_size=10
        )
        self.safety_stop_sub = rospy.Subscriber(
            self.safety_stop_topic, Bool, self._safety_stop_callback, queue_size=5
        )
        self.posture_fallen_sub = rospy.Subscriber(
            self.posture_fallen_topic,
            Bool,
            self._posture_fallen_callback,
            queue_size=5,
        )
        self.posture_reason_sub = rospy.Subscriber(
            self.posture_reason_topic,
            String,
            self._posture_reason_callback,
            queue_size=5,
        )

        self.start_explore_client = rospy.ServiceProxy(
            self.start_exploration_service, Trigger
        )
        self.stop_explore_client = rospy.ServiceProxy(
            self.stop_exploration_service, Trigger
        )
        self.move_base_client = actionlib.SimpleActionClient(
            self.move_base_action_name, MoveBaseAction
        )
        self.transit_floor_client = actionlib.SimpleActionClient(
            self.transit_floor_action_name, TransitFloorAction
        )

        self.pose_sub = rospy.Subscriber(
            self.pose_topic,
            PoseWithCovarianceStamped,
            self._pose_callback,
            queue_size=10,
        )
        self.mapping_sub = rospy.Subscriber(
            self.mapping_status_topic,
            MappingStatus,
            self._mapping_status_callback,
            queue_size=10,
        )
        self.navigation_sub = rospy.Subscriber(
            self.navigation_health_topic,
            NavigationHealth,
            self._navigation_health_callback,
            queue_size=10,
        )
        self.detection_status_sub = rospy.Subscriber(
            self.detection_status_topic,
            DetectionStatus,
            self._detection_status_callback,
            queue_size=10,
        )
        self.exploration_status_sub = rospy.Subscriber(
            self.exploration_status_topic,
            String,
            self._exploration_status_callback,
            queue_size=10,
        )
        self.exploration_complete_sub = rospy.Subscriber(
            self.exploration_complete_topic,
            Bool,
            self._exploration_complete_callback,
            queue_size=2,
        )
        self.detections_sub = rospy.Subscriber(
            self.detections_topic,
            DangerSourceArray,
            self._detections_callback,
            queue_size=20,
        )

        self.start_srv = rospy.Service(
            self.start_mission_service, Trigger, self._start_mission_callback
        )
        self.finish_srv = rospy.Service(
            self.finish_mission_service, Trigger, self._finish_mission_callback
        )
        self.return_srv = rospy.Service(
            self.return_home_service, Trigger, self._return_home_callback
        )

        self.status_timer = rospy.Timer(rospy.Duration(0.5), self._timer_callback)
        self.entry_control_timer = rospy.Timer(
            rospy.Duration(0.05), self._entry_control_timer_callback
        )
        rospy.on_shutdown(self._on_shutdown)
        self._publish_status()
        self.active_pub.publish(Bool(data=False))
        rospy.loginfo(
            "[mission] ready: result=%s autostart=%s",
            self.result_file,
            self.autostart,
        )

    @staticmethod
    def _load_scene_contract(path):
        if not path:
            return {}
        normalized = os.path.abspath(os.path.expanduser(os.path.expandvars(path)))
        if os.path.basename(normalized) != "team_scene_info.json":
            raise rospy.ROSInitException(
                "scene_info_file must reference public team_scene_info.json"
            )
        forbidden = {
            "layout_metadata.json",
            "building_config.json",
            "scene_manifest.json",
            "danger_truth.json",
        }
        if any(part in forbidden for part in normalized.split(os.sep)):
            raise rospy.ROSInitException("scene_info_file references forbidden data")
        try:
            with open(normalized, encoding="utf-8") as stream:
                return parse_public_scene_contract(json.load(stream))
        except (OSError, ValueError, KeyError, TypeError) as exc:
            raise rospy.ROSInitException("invalid public scene info: %s" % exc)

    @staticmethod
    def _positive_param(name, default):
        value = float(rospy.get_param(name, default))
        if not math.isfinite(value) or value <= 0.0:
            raise rospy.ROSInitException("%s must be positive and finite" % name)
        return value

    @staticmethod
    def _nonnegative_param(name, default):
        value = float(rospy.get_param(name, default))
        if not math.isfinite(value) or value < 0.0:
            raise rospy.ROSInitException("%s must be non-negative and finite" % name)
        return value

    @staticmethod
    def _unit_interval_param(name, default):
        value = float(rospy.get_param(name, default))
        if not math.isfinite(value) or not 0.0 < value <= 1.0:
            raise rospy.ROSInitException("%s must be in (0, 1]" % name)
        return value

    @property
    def mission_state(self):
        return self.lifecycle.state

    def _pose_callback(self, message):
        if message.header.frame_id != self.map_frame:
            return
        pose = message.pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        norm = math.sqrt(sum(float(value) ** 2 for value in values[3:]))
        if not all(math.isfinite(float(value)) for value in values) or norm < 1e-6:
            return
        with self.lock:
            self.latest_pose = copy.deepcopy(message)
            self.last_pose_time = rospy.Time.now()

    def _entrance_ready_callback(self, message):
        with self.lock:
            self.entrance_ready = bool(message.data)

    def _preflight_ready_callback(self, message):
        with self.lock:
            self.preflight_ready = bool(message.data)

    def _sent_cmd_callback(self, message):
        with self.lock:
            self.last_sent_cmd = copy.deepcopy(message)
            self.last_sent_cmd_time = rospy.Time.now()

    def _entry_scan_callback(self, message):
        values = (
            message.angle_min,
            message.angle_increment,
            message.range_min,
            message.range_max,
        )
        if (not message.ranges
                or not all(math.isfinite(float(value)) for value in values)
                or message.angle_increment == 0.0
                or message.range_min < 0.0
                or message.range_max <= message.range_min):
            return
        with self.lock:
            self.latest_entry_scan = copy.deepcopy(message)
            self.last_entry_scan_time = rospy.Time.now()

    def _entry_imu_callback(self, message):
        orientation = message.orientation
        try:
            quaternion_roll_pitch(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
        except ValueError:
            return
        with self.lock:
            self.latest_entry_imu = copy.deepcopy(message)
            self.last_entry_imu_time = rospy.Time.now()

    def _safety_stop_callback(self, message):
        now = rospy.Time.now()
        with self.lock:
            self.safety_stop_active = bool(message.data)
            self.posture_safety.update_stop(
                self.safety_stop_active, now.to_sec()
            )
        self._maybe_start_safety_abort(now)

    def _posture_fallen_callback(self, message):
        now = rospy.Time.now()
        with self.lock:
            self.posture_safety.update_fallen(message.data)
        self._maybe_start_safety_abort(now)

    def _posture_reason_callback(self, message):
        with self.lock:
            self.posture_safety.update_reason(message.data)

    def _maybe_start_safety_abort(self, now):
        start_abort = False
        with self.lock:
            mission_active = self.mission_state in (
                MissionLifecycle.ENTERING,
                MissionLifecycle.EXPLORING,
                MissionLifecycle.RETURNING,
            )
            detail = self.posture_safety.abort_detail(
                now.to_sec(), mission_active
            )
            abort_required = detail is not None and not self.finalized
            if abort_required and not self.safety_abort_started:
                self.safety_abort_started = True
                start_abort = True
        if start_abort:
            thread = threading.Thread(
                target=self._abort_for_safety_stop,
                args=(detail,),
                name="mission_safety_abort",
            )
            thread.daemon = True
            thread.start()
        return abort_required

    def _abort_for_safety_stop(self, detail):
        rospy.logerr(
            "[mission] terminal posture safety stop: %s", detail
        )
        self._stop_entry_crossing()
        self.move_base_client.cancel_all_goals()
        self.transit_floor_client.cancel_all_goals()
        try:
            self.stop_explore_client()
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("[mission] safety stop exploration failed: %s", str(exc))
        self._finalize("posture_safety_stop:" + detail, error=True)

    def _mapping_status_callback(self, message):
        with self.lock:
            self.mapping_status = copy.deepcopy(message)
            self.current_floor = int(message.current_floor)
            self.current_map_epoch = int(getattr(message, "map_epoch", 0))
            self.last_mapping_status_time = rospy.Time.now()

    def _navigation_health_callback(self, message):
        with self.lock:
            self.navigation_health = copy.deepcopy(message)
            self.last_navigation_health_time = rospy.Time.now()

    def _detection_status_callback(self, message):
        with self.lock:
            self.detection_status = copy.deepcopy(message)
            self.last_detection_status_time = rospy.Time.now()

    def _exploration_status_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            rospy.logwarn_throttle(5.0, "[mission] invalid exploration status JSON")
            return
        terminal_failure = exploration_failure_reason(payload)
        with self.lock:
            self.remaining_frontier_count = max(
                0, int(payload.get("remaining_frontier_count", 0))
            )
            ratio = payload.get("known_grid_ratio")
            self.map_coverage_summary = (
                "known_grid_ratio=%.3f" % float(ratio)
                if ratio is not None and math.isfinite(float(ratio))
                else ""
            )
            debt_count = max(0, int(payload.get("coverage_debt_count", 0)))
            self.topology_debt_summary = (
                "coverage_debt=%d" % debt_count if debt_count else ""
            )
            should_fail = (
                terminal_failure is not None
                and self.mission_state == MissionLifecycle.EXPLORING
                and not self.finalized
            )
        if not should_fail:
            return
        rospy.logerr(
            "[mission] exploration terminal failure: %s", terminal_failure
        )
        try:
            self.stop_explore_client()
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("[mission] failed to stop exploration: %s", str(exc))
        self._finalize("exploration_failed:" + terminal_failure, error=True)

    def _detections_callback(self, message):
        with self.lock:
            if self.mission_state != MissionLifecycle.EXPLORING:
                return
            current_floor = self.current_floor
            current_map_epoch = self.current_map_epoch
        for danger in message.dangers:
            if danger.class_id != DangerSource.CLASS_DANGER_RED_SPHERE:
                continue
            if danger.position.header.frame_id != self.map_frame:
                rospy.logwarn_throttle(
                    5.0,
                    "[mission] ignoring detection outside %s frame",
                    self.map_frame,
                )
                continue
            if (int(danger.floor_id) != current_floor
                    or int(getattr(danger, "map_epoch", 0)) != current_map_epoch):
                rospy.logwarn_throttle(
                    2.0,
                    "[mission] ignoring stale floor/map detection floor=%d epoch=%d",
                    int(danger.floor_id), int(getattr(danger, "map_epoch", 0)),
                )
                continue
            point = danger.position.point
            with self.lock:
                track = self.tracker.add(
                    danger.detection_id,
                    point.x,
                    point.y,
                    point.z,
                    danger.floor_id,
                    danger.confidence,
                )
                confirmed = (
                    track is not None
                    and track.count == self.tracker.min_detections
                )
            if confirmed:
                rospy.loginfo(
                    "[mission] confirmed danger floor=%d at (%.2f, %.2f, %.2f)",
                    track.floor_id,
                    track.x,
                    track.y,
                    track.z,
                )

    def _preflight_reason(self, now):
        with self.lock:
            pose = self.latest_pose
            pose_time = self.last_pose_time
            mapping = self.mapping_status
            mapping_time = self.last_mapping_status_time
            navigation = self.navigation_health
            navigation_time = self.last_navigation_health_time
            detection = self.detection_status
            detection_time = self.last_detection_status_time
            entrance_ready = self.entrance_ready
            preflight_ready = self.preflight_ready
            safety_stop = self.safety_stop_active
        if safety_stop:
            return "safety_stop_active"
        if self.require_preflight_ready and not preflight_ready:
            return "competition_preflight_not_ready"
        if self.require_entrance_ready and not entrance_ready:
            return "entrance_not_ready"
        inputs = (
            (pose, pose_time, "pose"),
            (mapping, mapping_time, "mapping_status"),
            (navigation, navigation_time, "navigation_health"),
            (detection, detection_time, "detection_status"),
        )
        for value, stamp, name in inputs:
            if value is None:
                return name + "_missing"
            if (now - stamp).to_sec() > self.input_timeout_s:
                return name + "_stale"
        if (
            not mapping.ready
            or not mapping.stable
            or mapping.lost
            or bool(getattr(mapping, "transitioning", False))
        ):
            return "mapping_not_ready"
        if int(mapping.current_floor) != self.home_floor:
            return "mission_start_not_on_home_floor"
        if not navigation.ready:
            return "navigation_not_ready"
        if not detection.ready:
            return "perception_not_ready"
        if not self.move_base_client.wait_for_server(rospy.Duration(0.05)):
            return "move_base_unavailable"
        if (self.multifloor_enabled
                and not self.transit_floor_client.wait_for_server(rospy.Duration(0.05))):
            return "transit_floor_unavailable"
        try:
            rospy.wait_for_service(self.start_exploration_service, timeout=0.05)
            rospy.wait_for_service(self.stop_exploration_service, timeout=0.05)
        except rospy.ROSException:
            return "exploration_services_unavailable"
        return "ready"

    def _wait_for_preflight(self):
        deadline = rospy.Time.now() + rospy.Duration(self.preflight_wait_timeout_s)
        rate = rospy.Rate(10)
        reason = "waiting"
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            reason = self._preflight_reason(rospy.Time.now())
            if reason == "ready":
                return True, reason
            rate.sleep()
        return False, reason

    def _start_mission_callback(self, _request):
        return self._start_mission(wait_for_ready=True)

    def _start_mission(self, wait_for_ready):
        with self.lock:
            if self.mission_state not in (
                MissionLifecycle.IDLE,
                MissionLifecycle.FINISHED,
                MissionLifecycle.ERROR,
            ):
                return TriggerResponse(False, "Mission already running")
        if wait_for_ready:
            ready, reason = self._wait_for_preflight()
        else:
            reason = self._preflight_reason(rospy.Time.now())
            ready = reason == "ready"
        if not ready:
            return TriggerResponse(False, "Preflight failed: " + reason)

        with self.lock:
            home_pose = copy.deepcopy(self.latest_pose)
            self.lifecycle = MissionLifecycle()
            self.lifecycle.start()
            self.start_time = rospy.Time.now()
            self.finish_time = None
            self.return_start_time = None
            self.entry_start_time = rospy.Time.now()
            self.home_pose = home_pose
            self.finish_reason = ""
            self.tracker.reset()
            self.return_goal_active = False
            self.transit_goal_active = False
            self.return_goal_attempts = 0
            self.return_goal_deadline = rospy.Time(0)
            self.return_retry_at = rospy.Time(0)
            self.return_retry_reason = ""
            self.return_trigger_reason = ""
            self.return_epoch += 1
            self.return_verify_since = rospy.Time(0)
            self.return_verify_deadline = rospy.Time(0)
            self.entry_goal_active = False
            self.entry_goal_sequence += 1
            self.entry_goal_progress_m = 0.0
            self.entry_attempt_start_progress_m = 0.0
            self.entry_retry_count = 0
            self.entry_retry_at = rospy.Time(0)
            self.entry_waiting_for_localization = False
            self.entry_short_range_phase = "INACTIVE"
            self.entry_short_range_epoch += 1
            self.entry_short_range_phase_time = rospy.Time(0)
            self.entry_short_range_started_at = rospy.Time(0)
            self.entry_crossing_reference = None
            self.entry_margin_avoidance_side = 0.0
            self.entry_crossing_yaw_filter.reset()
            self.exploration_completion_armed = False
            self.finalized = False
            self.safety_abort_started = False
            self.remaining_frontier_count = 0
            self.map_coverage_summary = ""
            self.topology_debt_summary = ""
        try:
            self._write_result_file(
                mission_status="RUNNING", finish_override=self.start_time
            )
        except (OSError, ValueError) as exc:
            self._finalize("initial_result_write_failed:" + str(exc), error=True)
            return TriggerResponse(False, "Could not initialize result file")
        self.active_pub.publish(Bool(data=True))
        self._publish_status()
        if self.entry_enabled:
            if self.entry_short_range_enabled:
                success, message = self._start_entry_crossing()
            else:
                success, message = self._advance_entry()
            if not success:
                self._finalize("entry_start_failed:" + message, error=True)
                return TriggerResponse(False, message)
            rospy.loginfo(
                "[mission] ENTERING; guarded crossing then rolling target %.2f m ahead",
                self.entry_distance_m,
            )
            return TriggerResponse(True, "Mission started; entering building")

        success, message = self._start_exploration()
        if not success:
            self._finalize("start_exploration_error:" + message, error=True)
            return TriggerResponse(False, message)
        return TriggerResponse(True, "Mission started")

    @staticmethod
    def _pose_yaw(pose):
        orientation = pose.pose.pose.orientation
        return math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )

    def _entry_progress(self, pose, home_pose):
        current = pose.pose.pose.position
        home = home_pose.pose.pose.position
        return entry_progress(
            current.x, current.y, home.x, home.y, self._pose_yaw(home_pose)
        )

    def _publish_entry_command(self, linear_x=0.0, angular_z=0.0):
        command = Twist()
        command.linear.x = float(linear_x)
        command.angular.z = float(angular_z)
        self.entry_cmd_pub.publish(command)

    def _start_entry_crossing(self):
        with self.lock:
            if self.mission_state != MissionLifecycle.ENTERING or self.finalized:
                return False, "Mission is not entering"
            if self.home_pose is None or self.latest_pose is None:
                return False, "Entry pose unavailable"
            self.entry_short_range_epoch += 1
            self.entry_short_range_phase = "PREPARE"
            self.entry_short_range_phase_time = rospy.Time.now()
            self.entry_short_range_started_at = rospy.Time(0)
            self.entry_crossing_reference = None
            self.entry_margin_avoidance_side = 0.0
            self.entry_crossing_yaw_filter.reset()
            self.entry_retry_at = rospy.Time(0)
            self.entry_goal_active = False
            self.entry_goal_sequence += 1
        # The short-range lease and move_base must never own motion together.
        self.move_base_client.cancel_all_goals()
        self._publish_entry_command()
        rospy.loginfo(
            "[mission] entry short-range PREPARE: target=%.2fm vx<=%.2f wz<=%.2f",
            self.entry_crossing_distance_m,
            self.entry_crossing_speed_mps,
            self.entry_crossing_max_angular_speed_rps,
        )
        return True, "Entry crossing preparing"

    def _stop_entry_crossing(self):
        self._publish_entry_command()
        with self.lock:
            if self.entry_short_range_phase in ("PREPARE", "CROSS", "HANDOFF"):
                self.entry_short_range_epoch += 1
                self.entry_short_range_phase = "INACTIVE"
                self.entry_short_range_phase_time = rospy.Time.now()
                self.entry_crossing_reference = None
                self.entry_margin_avoidance_side = 0.0
                self.entry_crossing_yaw_filter.reset()

    def _fail_entry_crossing(self, code, detail):
        self.move_base_client.cancel_all_goals()
        self._publish_entry_command()
        with self.lock:
            if self.entry_short_range_phase not in ("PREPARE", "CROSS", "HANDOFF"):
                return
            self.entry_short_range_phase = "FAILED"
            self.entry_short_range_epoch += 1
            self.entry_short_range_phase_time = rospy.Time.now()
            self.entry_crossing_reference = None
            self.entry_margin_avoidance_side = 0.0
            self.entry_crossing_yaw_filter.reset()
        rospy.logerr("[mission] entry short-range failed: %s: %s", code, detail)
        self._finalize("entry_short_range_%s:%s" % (code, detail), error=True)

    def _entry_control_timer_callback(self, _event=None):
        now = rospy.Time.now()
        with self.lock:
            phase = self.entry_short_range_phase
            state = self.mission_state
            finalized = self.finalized
            phase_time = self.entry_short_range_phase_time
            started_at = self.entry_short_range_started_at
            home_pose = copy.deepcopy(self.home_pose)
            current_pose = copy.deepcopy(self.latest_pose)
            crossing_reference = self.entry_crossing_reference
            avoidance_side = self.entry_margin_avoidance_side
            pose_time = self.last_pose_time
            scan = copy.deepcopy(self.latest_entry_scan)
            scan_time = self.last_entry_scan_time
            imu = copy.deepcopy(self.latest_entry_imu)
            imu_time = self.last_entry_imu_time
            navigation = copy.deepcopy(self.navigation_health)
            navigation_time = self.last_navigation_health_time
            last_command = copy.deepcopy(self.last_sent_cmd)
            last_command_time = self.last_sent_cmd_time
            safety_stop = self.safety_stop_active
        if phase not in ("PREPARE", "CROSS", "HANDOFF"):
            return
        if state != MissionLifecycle.ENTERING or finalized:
            self._stop_entry_crossing()
            return

        if phase in ("PREPARE", "HANDOFF"):
            self._publish_entry_command()
        if safety_stop:
            # cmd_mux enforces the same zero-motion stop globally.  Keep the
            # entry phase alive so a recoverable IMU dropout can resume rather
            # than turning a transient sensor fault into a terminal mission.
            if phase == "CROSS":
                self.entry_progress_watchdog.last_progress_time_s = now.to_sec()
            self._publish_entry_command()
            return
        inputs = (
            ("pose", current_pose, pose_time),
            ("scan", scan, scan_time),
            ("imu", imu, imu_time),
        )
        for name, value, stamp in inputs:
            if (value is None or stamp == rospy.Time(0)
                    or (now - stamp).to_sec() > self.entry_crossing_input_timeout_s):
                self._fail_entry_crossing("STALE_INPUT", name + " is stale")
                return
        if home_pose is None:
            self._fail_entry_crossing("NO_HOME", "captured home pose is unavailable")
            return
        orientation = imu.orientation
        try:
            roll, pitch = quaternion_roll_pitch(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
        except ValueError as exc:
            self._fail_entry_crossing("INVALID_IMU", str(exc))
            return
        if max(abs(roll), abs(pitch)) > self.entry_crossing_max_tilt_rad:
            self._fail_entry_crossing(
                "TILT",
                "roll=%.3f pitch=%.3f" % (roll, pitch),
            )
            return

        if phase == "PREPARE":
            forward, lateral = self._entry_progress(current_pose, home_pose)
            yaw_error = normalize_angle(
                self._pose_yaw(current_pose) - self._pose_yaw(home_pose)
            )
            navigation_fresh = (
                navigation is not None
                and navigation_time != rospy.Time(0)
                and (now - navigation_time).to_sec()
                <= self.entry_crossing_input_timeout_s
            )
            command_zero = (
                last_command_time != rospy.Time(0)
                and (now - last_command_time).to_sec()
                <= self.entry_crossing_input_timeout_s
                and command_is_zero(last_command)
            )
            nav_idle = (
                navigation_fresh
                and not navigation.has_active_goal
                and not navigation.controller_active
            )
            if not nav_idle or not command_zero:
                return
            if (now - phase_time).to_sec() < self.entry_crossing_prepare_zero_s:
                return
            if abs(lateral) > 0.10 or abs(yaw_error) > 0.20:
                self._fail_entry_crossing(
                    "NOT_ALIGNED",
                    "lateral=%.3f yaw=%.3f" % (lateral, yaw_error),
                )
                return
            current = current_pose.pose.pose.position
            try:
                reference = CrossingReference(
                    current.x,
                    current.y,
                    self._pose_yaw(current_pose),
                )
            except ValueError as exc:
                self._fail_entry_crossing("INVALID_POSE", str(exc))
                return
            with self.lock:
                if self.entry_short_range_phase != "PREPARE":
                    return
                self.entry_short_range_phase = "CROSS"
                self.entry_short_range_phase_time = now
                self.entry_short_range_started_at = now
                # This is deliberately distinct from home_pose: home remains
                # the immutable return/output origin for the entire mission.
                self.entry_crossing_reference = reference
                self.entry_margin_avoidance_side = 0.0
                self.entry_crossing_yaw_filter.reset()
                self.entry_progress_watchdog.reset(0.0, now.to_sec())
            rospy.loginfo(
                "[mission] entry short-range CROSS started at local reference "
                "(%.2f, %.2f, %.2f)",
                reference.x_m,
                reference.y_m,
                reference.yaw_rad,
            )
            return

        if crossing_reference is None:
            self._fail_entry_crossing(
                "INTERNAL", "crossing reference is unset"
            )
            return
        current = current_pose.pose.pose.position
        try:
            forward, lateral, yaw_error = crossing_reference.errors(
                current.x,
                current.y,
                self._pose_yaw(current_pose),
            )
        except ValueError as exc:
            self._fail_entry_crossing("INVALID_POSE", str(exc))
            return

        if phase == "HANDOFF":
            command_zero = (
                last_command_time != rospy.Time(0)
                and (now - last_command_time).to_sec()
                <= self.entry_crossing_input_timeout_s
                and command_is_zero(last_command)
            )
            if (not command_zero
                    or (now - phase_time).to_sec()
                    < self.entry_crossing_handoff_zero_s):
                return
            with self.lock:
                if self.entry_short_range_phase != "HANDOFF":
                    return
                self.entry_short_range_phase = "COMPLETE"
                self.entry_short_range_phase_time = now
                self.entry_retry_at = now + rospy.Duration(
                    self.entry_crossing_release_s
                )
            rospy.loginfo(
                "[mission] entry short-range handoff complete at %.2fm", forward
            )
            return

        if started_at == rospy.Time(0):
            self._fail_entry_crossing("INTERNAL", "crossing start time is unset")
            return
        if (now - started_at).to_sec() > self.entry_crossing_timeout_s:
            self._fail_entry_crossing("TIMEOUT", "crossing deadline exceeded")
            return
        if abs(lateral) > self.entry_crossing_max_lateral_error_m:
            self._fail_entry_crossing(
                "LATERAL_ERROR", "lateral=%.3f" % lateral
            )
            return
        if abs(yaw_error) > self.entry_crossing_max_yaw_error_rad:
            self._fail_entry_crossing("YAW_ERROR", "yaw=%.3f" % yaw_error)
            return
        if forward >= self.entry_crossing_distance_m:
            with self.lock:
                if self.entry_short_range_phase != "CROSS":
                    return
                self.entry_short_range_phase = "HANDOFF"
                self.entry_short_range_phase_time = now
            self._publish_entry_command()
            rospy.loginfo(
                "[mission] entry short-range target reached: forward=%.2f lateral=%.2f",
                forward,
                lateral,
            )
            return
        if not self.entry_progress_watchdog.update(forward, now.to_sec()):
            self._fail_entry_crossing(
                "STALLED", "forward progress stopped at %.3fm" % forward
            )
            return
        margin_side = None
        soft_hits = ()
        if self.entry_crossing_sweep_enabled:
            remaining = max(0.0, self.entry_crossing_distance_m - forward)
            try:
                sweep_distance = rolling_sweep_distance(
                    remaining,
                    self.entry_crossing_sweep_lookahead_m,
                )
                scan_geometry = (
                    scan.ranges,
                    scan.angle_min,
                    scan.angle_increment,
                    scan.range_min,
                    scan.range_max,
                    1.0,
                    sweep_distance,
                    (
                        self.entry_footprint_min_x,
                        self.entry_footprint_max_x,
                        self.entry_footprint_min_y,
                        self.entry_footprint_max_y,
                    ),
                )
                hard_hit = swept_footprint_hit(*scan_geometry, margin=0.0)
                if hard_hit is None and self.entry_footprint_margin_m > 0.0:
                    soft_distance = min(
                        remaining,
                        self.entry_margin_avoidance_lookahead_m,
                    )
                    soft_margin = self.entry_footprint_margin_m
                    if abs(avoidance_side) > 0.5:
                        soft_margin += self.entry_margin_release_hysteresis_m
                    soft_hits = swept_footprint_hits(
                        scan.ranges,
                        scan.angle_min,
                        scan.angle_increment,
                        scan.range_min,
                        scan.range_max,
                        1.0,
                        soft_distance,
                        (
                            self.entry_footprint_min_x,
                            self.entry_footprint_max_x,
                            self.entry_footprint_min_y,
                            self.entry_footprint_max_y,
                        ),
                        margin=soft_margin,
                    )
            except ValueError as exc:
                self._fail_entry_crossing("INVALID_SCAN", str(exc))
                return
            if hard_hit is not None:
                self._fail_entry_crossing(
                    "OBSTACLE",
                    "physical footprint hit at %.3fm" % hard_hit.distance_m,
                )
                return
            forward_soft_hits = tuple(hit for hit in soft_hits if hit.x_m > 0.0)
            unsafe_soft_hits = tuple(
                hit for hit in forward_soft_hits
                if (self.entry_footprint_min_y
                    - self.entry_margin_min_side_clearance_m
                    < hit.y_m
                    < self.entry_footprint_max_y
                    + self.entry_margin_min_side_clearance_m)
            )
            if unsafe_soft_hits:
                nearest = min(unsafe_soft_hits, key=lambda hit: hit.distance_m)
                # A point in the center corridor, or with less than the
                # explicit 1cm lateral clearance, has no safe turn direction.
                self._fail_entry_crossing(
                    "OBSTACLE",
                    "center corridor safety margin hit at %.3fm"
                    % nearest.distance_m,
                )
                return
            left_hits = tuple(
                hit for hit in forward_soft_hits
                if hit.y_m >= (
                    self.entry_footprint_max_y
                    + self.entry_margin_min_side_clearance_m
                )
            )
            right_hits = tuple(
                hit for hit in forward_soft_hits
                if hit.y_m <= (
                    self.entry_footprint_min_y
                    - self.entry_margin_min_side_clearance_m
                )
            )
            if left_hits and right_hits:
                margin_side = 0.0
            elif left_hits:
                margin_side = 1.0
            elif right_hits:
                margin_side = -1.0
        try:
            with self.lock:
                if (self.entry_short_range_phase != "CROSS"
                        or self.entry_crossing_reference is not crossing_reference):
                    return
                filtered_yaw_error = self.entry_crossing_yaw_filter.update(
                    yaw_error
                )
            linear, angular = crossing_command(
                lateral,
                filtered_yaw_error,
                self.entry_crossing_speed_mps,
                self.entry_crossing_yaw_gain,
                self.entry_crossing_lateral_gain,
                self.entry_crossing_max_angular_speed_rps,
            )
            if margin_side is not None:
                linear, angular = margin_avoidance_command(
                    linear,
                    angular,
                    margin_side,
                    self.entry_margin_avoidance_speed_mps,
                    self.entry_margin_avoidance_yaw_rps,
                    self.entry_crossing_max_angular_speed_rps,
                )
            if self.entry_crossing_sweep_enabled:
                arc_distance = min(
                    max(0.0, self.entry_crossing_distance_m - forward),
                    self.entry_margin_avoidance_lookahead_m,
                )
                curvature = angular / linear
                arc_hit = swept_arc_footprint_hit(
                    scan.ranges,
                    scan.angle_min,
                    scan.angle_increment,
                    scan.range_min,
                    scan.range_max,
                    1.0,
                    arc_distance,
                    curvature,
                    (
                        self.entry_footprint_min_x,
                        self.entry_footprint_max_x,
                        self.entry_footprint_min_y,
                        self.entry_footprint_max_y,
                    ),
                    margin=0.0,
                    sample_spacing_m=self.entry_arc_sweep_sample_spacing_m,
                )
                if arc_hit is not None:
                    self._fail_entry_crossing(
                        "OBSTACLE",
                        "commanded arc footprint hit at %.3fm"
                        % arc_hit.distance_m,
                    )
                    return
        except ValueError as exc:
            self._fail_entry_crossing("INVALID_COMMAND", str(exc))
            return
        with self.lock:
            if (self.entry_short_range_phase != "CROSS"
                    or self.entry_crossing_reference is not crossing_reference):
                return
            self.entry_margin_avoidance_side = (
                0.0 if margin_side is None else margin_side
            )
        if margin_side is not None:
            rospy.logwarn_throttle(
                1.0,
                "[mission] entry lateral margin avoidance: side=%+.0f "
                "vx=%.2f wz=%.2f",
                margin_side,
                linear,
                angular,
            )
        self._publish_entry_command(linear, angular)

    def _advance_entry(self):
        with self.lock:
            if self.mission_state != MissionLifecycle.ENTERING or self.finalized:
                return False, "Mission is not entering"
            if self.entry_goal_active:
                return True, "Entry goal already active"
            if (self.entry_short_range_enabled
                    and self.entry_short_range_phase != "COMPLETE"):
                return False, "Entry short-range crossing is not complete"
            home_pose = copy.deepcopy(self.home_pose)
            current_pose = copy.deepcopy(self.latest_pose)
        if home_pose is None or current_pose is None:
            return False, "Entry pose unavailable"

        forward, lateral = self._entry_progress(current_pose, home_pose)
        completion = self.entry_distance_m - self.entry_completion_tolerance_m
        if forward >= completion:
            rospy.loginfo(
                "[mission] entrance crossed: forward=%.2f m lateral=%.2f m",
                forward,
                lateral,
            )
            return self._start_exploration()

        home = home_pose.pose.pose.position
        target_x, target_y, target_progress = next_entry_target(
            home.x,
            home.y,
            self._pose_yaw(home_pose),
            forward,
            self.entry_distance_m,
            self.entry_step_m,
        )
        return self._send_entry_goal(
            home_pose, target_x, target_y, target_progress, forward
        )

    def _send_entry_goal(
        self, home_pose, target_x, target_y, target_progress, start_progress
    ):
        if not self.move_base_client.wait_for_server(rospy.Duration(1.0)):
            return False, "move_base unavailable for entry"
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.pose = copy.deepcopy(home_pose.pose.pose)
        goal.target_pose.pose.position.x = target_x
        goal.target_pose.pose.position.y = target_y
        with self.lock:
            self.entry_goal_sequence += 1
            sequence = self.entry_goal_sequence
            self.entry_goal_active = True
            self.entry_goal_progress_m = target_progress
            self.entry_attempt_start_progress_m = start_progress
            self.entry_retry_at = rospy.Time(0)
        self.move_base_client.send_goal(
            goal,
            done_cb=lambda state, result: self._entry_done_callback(
                sequence, state, result
            ),
        )
        rospy.loginfo(
            "[mission] entry segment %d sent: %.2f -> %.2f m",
            sequence,
            start_progress,
            target_progress,
        )
        return True, "Entry goal sent"

    def _entry_done_callback(self, sequence, state, _result):
        with self.lock:
            if (
                self.mission_state != MissionLifecycle.ENTERING
                or self.finalized
                or sequence != self.entry_goal_sequence
            ):
                return
            self.entry_goal_active = False
        if state == GoalStatus.SUCCEEDED:
            with self.lock:
                self.entry_retry_count = 0
                self.entry_waiting_for_localization = False
                # Do not send a new goal from inside SimpleActionClient's done
                # callback.  Let the mission timer advance after actionlib has
                # fully returned to its idle state.
                self.entry_retry_at = rospy.Time.now()
            return

        # Navigation publishes its terminal health immediately before setting
        # the Action result, but those messages use separate ROS connections.
        # Classify after a short settling interval instead of racing a stale
        # failure_code from the previous goal.
        rospy.Timer(
            rospy.Duration(self.entry_health_settle_s),
            lambda _event: self._classify_entry_failure(sequence, state),
            oneshot=True,
        )

    def _classify_entry_failure(self, sequence, state):
        with self.lock:
            if (
                self.mission_state != MissionLifecycle.ENTERING
                or self.finalized
                or sequence != self.entry_goal_sequence
            ):
                return
            home_pose = copy.deepcopy(self.home_pose)
            current_pose = copy.deepcopy(self.latest_pose)
            attempt_start = self.entry_attempt_start_progress_m

        forward = attempt_start
        if home_pose is not None and current_pose is not None:
            forward, _ = self._entry_progress(current_pose, home_pose)
        made_progress = forward >= attempt_start + self.entry_min_progress_m
        with self.lock:
            navigation_failure = (
                self.navigation_health.failure_code
                if self.navigation_health is not None
                else ""
            )
            localization_lost = navigation_failure == "LOCALIZATION_LOST"
            transient_failure = navigation_failure in (
                "",
                "NONE",
                "LOCALIZATION_LOST",
                "SAFETY_STOP",
                "UNREACHABLE",
            )
            self.entry_waiting_for_localization = localization_lost
            if transient_failure:
                # Localization outages and rolling-map reachability misses are
                # readiness conditions, not failed robot motion attempts. The
                # complete entrance remains bounded by entry_timeout_s.
                self.entry_retry_count = 0 if made_progress else self.entry_retry_count
            else:
                self.entry_retry_count = 0 if made_progress else self.entry_retry_count + 1
            retries = self.entry_retry_count
            if transient_failure or retries <= self.entry_max_retries:
                retry_delay = (
                    self.entry_map_retry_delay_s
                    if navigation_failure == "UNREACHABLE"
                    else self.entry_retry_delay_s
                )
                self.entry_retry_at = rospy.Time.now() + rospy.Duration(
                    retry_delay
                )
        if not transient_failure and retries > self.entry_max_retries:
            self._finalize(
                "entry_failed_action_state_%d_retries_exhausted" % state,
                error=True,
            )
            return
        rospy.logwarn(
            "[mission] entry segment failed state=%d code=%s progress=%.2f m; retry %d/%d",
            state,
            navigation_failure or "UNKNOWN",
            forward,
            retries,
            self.entry_max_retries,
        )

    def _start_exploration(self):
        self._publish_entry_command()
        try:
            response = self.start_explore_client()
        except rospy.ServiceException as exc:
            return False, str(exc)
        if not response.success:
            return False, response.message
        with self.lock:
            if not self.lifecycle.begin_exploration():
                return False, "Mission is not entering"
            self.entry_start_time = None
            self.entry_retry_at = rospy.Time(0)
            self.entry_waiting_for_localization = False
            self.entry_short_range_phase = "DONE"
            # A successful start service response establishes a new exploration
            # session even when its latched false marker arrives asynchronously.
            self.exploration_completion_armed = True
        self._publish_status()
        rospy.loginfo("[mission] EXPLORING; home pose captured in %s", self.map_frame)
        return True, "Exploration started"

    def _exploration_complete_callback(self, message):
        with self.lock:
            if self.mission_state != MissionLifecycle.EXPLORING:
                return
            if not message.data:
                # Exploration publishes false at the beginning of every session.
                # Requiring it prevents an old latched true from ending a new run.
                self.exploration_completion_armed = True
                return
            armed = self.exploration_completion_armed
        if not armed:
            rospy.logwarn_throttle(
                2.0, "[mission] ignoring stale exploration completion"
            )
            return
        self._begin_return("exploration_complete")

    def _finish_mission_callback(self, _request):
        with self.lock:
            state = self.mission_state
        if state == MissionLifecycle.FINISHED:
            return TriggerResponse(True, "Mission already finished")
        if state == MissionLifecycle.RETURNING:
            return TriggerResponse(True, "Return already in progress")
        if state not in (MissionLifecycle.ENTERING, MissionLifecycle.EXPLORING):
            return TriggerResponse(False, "No active mission")
        success, message = self._begin_return("manual_finish_requested")
        return TriggerResponse(success, message)

    def _return_home_callback(self, _request):
        with self.lock:
            state = self.mission_state
        if state == MissionLifecycle.RETURNING:
            return TriggerResponse(True, "Return already in progress")
        if state not in (MissionLifecycle.ENTERING, MissionLifecycle.EXPLORING):
            return TriggerResponse(False, "No active mission")
        success, message = self._begin_return("return_home_requested")
        return TriggerResponse(success, message)

    def _begin_return(self, reason):
        with self.lock:
            if self.mission_state == MissionLifecycle.RETURNING:
                return True, "Return already in progress"
            if not self.lifecycle.begin_return():
                return False, "Mission is not active"
            home_pose = copy.deepcopy(self.home_pose)
            self.return_start_time = rospy.Time.now()
            self.finish_reason = reason
            self.return_trigger_reason = str(reason)
            self.return_goal_active = False
            self.transit_goal_active = False
            self.return_goal_attempts = 0
            self.return_goal_deadline = rospy.Time(0)
            self.return_retry_at = rospy.Time(0)
            self.return_retry_reason = ""
            self.return_epoch += 1
            return_epoch = self.return_epoch
            self.return_verify_since = rospy.Time(0)
            self.return_verify_deadline = rospy.Time(0)
            self.entry_goal_active = False
            self.entry_goal_sequence += 1
            self.entry_retry_at = rospy.Time(0)
            self.entry_waiting_for_localization = False
            self.exploration_completion_armed = False
        self._stop_entry_crossing()
        self.move_base_client.cancel_all_goals()
        try:
            self.stop_explore_client()
        except rospy.ServiceException as exc:
            rospy.logwarn("[mission] stop exploration failed: %s", str(exc))

        if home_pose is None:
            self._finalize("home_pose_missing", error=True)
            return False, "Home pose missing"
        with self.lock:
            current_floor = self.current_floor
        if current_floor != self.home_floor:
            if not self.transit_floor_client.wait_for_server(rospy.Duration(1.0)):
                self._finalize("transit_floor_unavailable_for_return", error=True)
                return False, "floor transit unavailable"
            goal = TransitFloorGoal()
            goal.target_floor = self.home_floor
            goal.exit_to_hall = True
            self.transit_floor_client.send_goal(
                goal,
                done_cb=lambda state, result: self._return_transit_done_callback(
                    return_epoch, state, result
                ),
            )
            with self.lock:
                self.transit_goal_active = True
            self._publish_status()
            rospy.loginfo(
                "[mission] RETURNING via floor transit %d -> %d: %s",
                current_floor,
                self.home_floor,
                reason,
            )
            return True, "Return floor transit started"
        return self._send_home_goal(reason)

    def _return_transit_done_callback(self, return_epoch, state, result):
        with self.lock:
            if (self.mission_state != MissionLifecycle.RETURNING or self.finalized
                    or return_epoch != self.return_epoch):
                return
            self.transit_goal_active = False
        if (
            state != GoalStatus.SUCCEEDED
            or result is None
            or not result.success
            or int(result.reached_floor) != self.home_floor
        ):
            failure = getattr(result, "failure_code", "ACTION_FAILED")
            self._finalize("return_floor_transit_failed:" + str(failure), error=True)
            return
        self._send_home_goal("home_floor_reached")

    def _send_home_goal(self, reason):
        with self.lock:
            if self.mission_state != MissionLifecycle.RETURNING or self.finalized:
                return False, "Mission is not returning"
            home_pose = copy.deepcopy(self.home_pose)
            if self.return_goal_attempts >= self.return_max_goal_attempts:
                self._finalize("return_goal_attempts_exhausted", error=True)
                return False, "Return attempts exhausted"
            self.return_goal_attempts += 1
            now = rospy.Time.now()
            return_deadline = (
                self.return_start_time + rospy.Duration(self.return_timeout_s)
            )
            remaining_s = max(0.0, (return_deadline - now).to_sec())
            attempts_left = max(
                0, self.return_max_goal_attempts - self.return_goal_attempts
            )
            attempt_budget_s = allocate_return_attempt_budget(
                remaining_s,
                self.return_attempt_timeout_s,
                self.return_retry_reserve_s,
                self.return_terminal_reserve_s,
                attempts_left,
            )
            if attempt_budget_s < self.return_min_attempt_timeout_s:
                self._finalize("return_budget_exhausted", error=True)
                return False, "Return budget exhausted"
            return_epoch = self.return_epoch
            self.return_retry_at = rospy.Time(0)
            self.return_retry_reason = ""
            self.return_verify_since = rospy.Time(0)
            self.return_verify_deadline = rospy.Time(0)
            self.return_goal_deadline = now + rospy.Duration(attempt_budget_s)
        if not self.move_base_client.wait_for_server(rospy.Duration(1.0)):
            self._finalize("move_base_unavailable_for_return", error=True)
            return False, "move_base unavailable"
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.pose = copy.deepcopy(home_pose.pose.pose)
        self.move_base_client.send_goal(
            goal,
            done_cb=lambda state, result: self._return_done_callback(
                return_epoch, state, result
            ),
        )
        with self.lock:
            self.return_goal_active = True
        self._publish_status()
        rospy.loginfo(
            "[mission] RETURNING to home pose attempt=%d budget=%.1fs: %s",
            self.return_goal_attempts,
            attempt_budget_s,
            reason,
        )
        return True, "Return home goal sent"

    def _return_done_callback(self, return_epoch, state, _result):
        with self.lock:
            if (self.mission_state != MissionLifecycle.RETURNING or self.finalized
                    or return_epoch != self.return_epoch):
                return
            self.return_goal_active = False
            self.return_goal_deadline = rospy.Time(0)
            safety_stop = self.safety_stop_active
            if state != GoalStatus.SUCCEEDED and safety_stop:
                # The controller correctly terminates its Action while the
                # hard stop is active.  Retry only after recovery and do not
                # spend the bounded return-attempt budget on that safety event.
                self.return_goal_attempts = max(
                    0, self.return_goal_attempts - 1
                )
                self.return_retry_reason = "safety_recovery"
                self.return_retry_at = (
                    rospy.Time.now()
                    + rospy.Duration(self.return_retry_delay_s)
                )
                return
        if state == GoalStatus.SUCCEEDED:
            with self.lock:
                self.return_verify_deadline = (
                    rospy.Time.now() + rospy.Duration(self.return_verify_timeout_s)
                )
        else:
            with self.lock:
                attempts = self.return_goal_attempts
            if attempts < self.return_max_goal_attempts:
                # SimpleActionClient is still unwinding this callback.  Sending
                # another goal here races its DONE transition and produced
                # "ACTIVE when in simple state DONE" in Gazebo.  The mission
                # timer owns retry dispatch after a short settling interval.
                with self.lock:
                    self.return_retry_reason = "retry_action_state_%d" % state
                    self.return_retry_at = (
                        rospy.Time.now()
                        + rospy.Duration(self.return_retry_delay_s)
                    )
            else:
                self._finalize("return_failed_action_state_%d" % state, error=True)

    def _return_pose_within_tolerance(self):
        with self.lock:
            home = copy.deepcopy(self.home_pose)
            current = copy.deepcopy(self.latest_pose)
            navigation = copy.deepcopy(self.navigation_health)
            floor = self.current_floor
            command = copy.deepcopy(self.last_sent_cmd)
            command_time = self.last_sent_cmd_time
        if home is None or current is None or floor != self.home_floor:
            return False
        home_position = home.pose.pose.position
        current_position = current.pose.pose.position
        distance = math.hypot(
            current_position.x - home_position.x,
            current_position.y - home_position.y,
        )
        yaw_error = abs(
            math.atan2(
                math.sin(self._pose_yaw(current) - self._pose_yaw(home)),
                math.cos(self._pose_yaw(current) - self._pose_yaw(home)),
            )
        )
        stationary = (
            navigation is not None
            and not navigation.controller_active
            and not navigation.has_active_goal
            and command_time != rospy.Time(0)
            and (rospy.Time.now() - command_time).to_sec() <= self.input_timeout_s
            and math.hypot(command.linear.x, command.linear.y)
            <= self.return_stationary_linear_mps
            and abs(command.angular.z) <= self.return_stationary_angular_rps
        )
        return (
            distance <= self.return_position_tolerance_m
            and yaw_error <= self.return_yaw_tolerance_rad
            and stationary
        )

    def _timer_callback(self, _event=None):
        now = rospy.Time.now()
        with self.lock:
            state = self.mission_state
            start_time = self.start_time
            return_start_time = self.return_start_time
            entry_start_time = self.entry_start_time
            entry_retry_at = self.entry_retry_at
            entry_goal_active = self.entry_goal_active
            entry_waiting_for_localization = self.entry_waiting_for_localization
            return_goal_active = self.return_goal_active
            return_goal_deadline = self.return_goal_deadline
            return_retry_at = self.return_retry_at
            return_retry_reason = self.return_retry_reason
            return_verify_deadline = self.return_verify_deadline
            should_autostart = (
                self.autostart
                and not self.autostart_attempted
                and state == MissionLifecycle.IDLE
            )
        if self._maybe_start_safety_abort(now):
            return
        with self.lock:
            safety_stop = self.safety_stop_active
        if safety_stop:
            return
        if should_autostart and self._preflight_reason(now) == "ready":
            with self.lock:
                self.autostart_attempted = True
            response = self._start_mission(wait_for_ready=False)
            if not response.success:
                rospy.logerr("[mission] autostart failed: %s", response.message)
        elif (
            state == MissionLifecycle.ENTERING
            and entry_start_time is not None
            and (now - entry_start_time).to_sec() >= self.entry_timeout_s
        ):
            if entry_goal_active:
                self.move_base_client.cancel_goal()
            self._stop_entry_crossing()
            self._finalize("entry_timeout", error=True)
        elif (
            state == MissionLifecycle.ENTERING
            and not entry_goal_active
            and not entry_retry_at.is_zero()
            and now >= entry_retry_at
        ):
            if entry_waiting_for_localization and not self._entry_localization_ready(now):
                with self.lock:
                    self.entry_retry_at = now + rospy.Duration(self.entry_retry_delay_s)
                rospy.logwarn_throttle(
                    2.0, "[mission] entry paused until localization/map recover"
                )
            else:
                with self.lock:
                    self.entry_retry_at = rospy.Time(0)
                    self.entry_waiting_for_localization = False
                success, message = self._advance_entry()
                if not success:
                    self._finalize("entry_retry_failed:" + message, error=True)
        elif (
            state == MissionLifecycle.EXPLORING
            and self.mission_timeout_s > 0.0
            and start_time is not None
            and (now - start_time).to_sec() >= self.mission_timeout_s
        ):
            self._begin_return("mission_timeout")
        elif (
            state == MissionLifecycle.RETURNING
            and return_start_time is not None
            and (now - return_start_time).to_sec() >= self.return_timeout_s
        ):
            self.move_base_client.cancel_goal()
            self.transit_floor_client.cancel_goal()
            self._finalize("return_timeout", error=True)
        elif (
            state == MissionLifecycle.RETURNING
            and return_goal_active
            and not return_goal_deadline.is_zero()
            and now >= return_goal_deadline
        ):
            # Cancel once.  The action done callback records a delayed retry
            # after SimpleActionClient has fully unwound its DONE transition.
            with self.lock:
                self.return_goal_deadline = rospy.Time(0)
            rospy.logwarn(
                "[mission] home goal attempt %d exhausted its action budget",
                self.return_goal_attempts,
            )
            self.move_base_client.cancel_goal()
        elif (
            state == MissionLifecycle.RETURNING
            and not return_goal_active
            and not return_retry_at.is_zero()
            and now >= return_retry_at
        ):
            with self.lock:
                # Clear before dispatch so a synchronous failure cannot cause
                # the same retry epoch to be sent twice.
                self.return_retry_at = rospy.Time(0)
                self.return_retry_reason = ""
            self._send_home_goal(return_retry_reason or "action_retry")
        elif (
            state == MissionLifecycle.RETURNING
            and not return_verify_deadline.is_zero()
        ):
            if self._return_pose_within_tolerance():
                with self.lock:
                    if self.return_verify_since.is_zero():
                        self.return_verify_since = now
                    held_for = (now - self.return_verify_since).to_sec()
                if held_for >= self.return_stationary_hold_s:
                    with self.lock:
                        trigger = self.return_trigger_reason
                    self._finalize(
                        "completed:" + (trigger or "return"), error=False
                    )
            else:
                with self.lock:
                    self.return_verify_since = rospy.Time(0)
                if now >= return_verify_deadline:
                    with self.lock:
                        attempts = self.return_goal_attempts
                        self.return_verify_deadline = rospy.Time(0)
                    if attempts < self.return_max_goal_attempts:
                        self._send_home_goal("pose_verification_retry")
                    else:
                        self._finalize("return_pose_verification_failed", error=True)
        self._publish_status()

    def _entry_localization_ready(self, now):
        with self.lock:
            pose = self.latest_pose
            pose_time = self.last_pose_time
            mapping = self.mapping_status
            mapping_time = self.last_mapping_status_time
            navigation = self.navigation_health
            navigation_time = self.last_navigation_health_time
        inputs = (
            (pose, pose_time),
            (mapping, mapping_time),
            (navigation, navigation_time),
        )
        if any(value is None for value, _stamp in inputs):
            return False
        if any((now - stamp).to_sec() > self.input_timeout_s for _value, stamp in inputs):
            return False
        return (
            mapping.ready
            and mapping.stable
            and not mapping.lost
            and navigation.ready
        )

    def _finalize(self, reason, error):
        self._stop_entry_crossing()
        self.move_base_client.cancel_all_goals()
        self.transit_floor_client.cancel_all_goals()
        with self.lock:
            if self.finalized:
                return
            self.finalized = True
            self.finish_time = rospy.Time.now()
            self.finish_reason = reason
            if error:
                self.lifecycle.fail()
            else:
                self.lifecycle.finish()
        try:
            self._write_result_file()
        except (OSError, ValueError) as exc:
            with self.lock:
                self.finish_reason = "result_write_failed:" + str(exc)
                self.lifecycle.fail()
            rospy.logerr("[mission] result write failed: %s", str(exc))
        self.active_pub.publish(Bool(data=False))
        self._publish_status()
        rospy.loginfo(
            "[mission] %s: reason=%s confirmed=%d result=%s",
            self.mission_state,
            self.finish_reason,
            len(self.tracker.confirmed_tracks()),
            self.result_file,
        )

    def _write_result_file(self, mission_status=None, finish_override=None):
        with self.lock:
            home = copy.deepcopy(self.home_pose)
            tracks = list(self.tracker.confirmed_tracks())
            start_time = self.start_time
            finish_time = finish_override or self.finish_time or rospy.Time.now()
            status = mission_status or self.mission_state
            finish_reason = self.finish_reason
        if home is None or start_time is None:
            raise ValueError("mission start pose/time unavailable")
        orientation = home.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )
        home_position = home.pose.pose.position
        result = build_result_document(
            tracks,
            (home_position.x, home_position.y, home_position.z, yaw),
            (finish_time - start_time).to_sec(),
            coordinate_frame=self.output_coordinate_frame,
            robot_start=self.scene_contract.get("robot_start"),
            mission_status=status,
            run_profile=self.run_profile,
            localization_backend=self.localization_backend,
            finish_reason=finish_reason,
        )
        directory = os.path.dirname(self.result_file)
        os.makedirs(directory, exist_ok=True)
        temporary = self.result_file + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(result, stream, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.result_file)

    def _publish_status(self):
        if self.shutting_down or rospy.is_shutdown():
            return
        now = rospy.Time.now()
        with self.lock:
            state = self.mission_state
            start_time = self.start_time
            finish_time = self.finish_time
            navigation = self.navigation_health
            current_floor = self.current_floor
            remaining = self.remaining_frontier_count
            coverage = self.map_coverage_summary
            topology_debt = self.topology_debt_summary
            finish_reason = self.finish_reason
        message = MissionStatus()
        message.header.stamp = now
        message.header.frame_id = self.map_frame
        message.mission_state = state
        message.current_floor = current_floor
        if start_time is not None:
            message.start_time = start_time
            end = finish_time if finish_time is not None else now
            elapsed = end - start_time
            message.elapsed_time = elapsed
            message.scored_exploration_time = elapsed
        if navigation is not None:
            message.active_goal_id = navigation.active_goal_id
        message.map_coverage_summary = coverage
        message.topology_debt_summary = topology_debt
        message.remaining_frontier_count = remaining
        message.finish_reason = finish_reason
        try:
            self.status_pub.publish(message)
        except rospy.ROSException:
            # A timer can cross rospy's publisher teardown during Ctrl-C.
            if not (self.shutting_down or rospy.is_shutdown()):
                raise

    def _on_shutdown(self):
        with self.lock:
            self.shutting_down = True
            active = self.mission_state in (
                MissionLifecycle.EXPLORING,
                MissionLifecycle.ENTERING,
                MissionLifecycle.RETURNING,
            )
        if active:
            try:
                self._stop_entry_crossing()
            except rospy.ROSException:
                pass
            self.move_base_client.cancel_all_goals()
            self.transit_floor_client.cancel_all_goals()

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        MissionManager().run()
    except rospy.ROSInterruptException:
        pass
