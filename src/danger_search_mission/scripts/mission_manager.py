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
)
from danger_search_mission.mission_core import (
    build_result_document,
    DangerTrackStore,
    entry_progress,
    MissionLifecycle,
    next_entry_target,
    normalize_result_file,
)
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
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

        try:
            self.result_file = normalize_result_file(
                rospy.get_param("~result_file", "")
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
            "~entry_completion_tolerance_m", 0.25
        )
        self.entry_min_progress_m = self._nonnegative_param(
            "~entry_min_progress_m", 0.10
        )
        self.entry_max_retries = int(rospy.get_param("~entry_max_retries", 8))
        if self.entry_max_retries < 0:
            raise rospy.ROSInitException("~entry_max_retries must be non-negative")
        if self.entry_completion_tolerance_m >= self.entry_distance_m:
            raise rospy.ROSInitException(
                "~entry_completion_tolerance_m must be smaller than entry distance"
            )
        self.mission_timeout_s = self._nonnegative_param("~mission_timeout_s", 0.0)
        self.input_timeout_s = self._positive_param("~input_timeout_s", 2.0)
        self.entry_enabled = bool(rospy.get_param("~entry_enabled", True))
        self.require_entrance_ready = bool(
            rospy.get_param("~require_entrance_ready", True)
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
        self.return_goal_active = False
        self.entry_goal_active = False
        self.entry_goal_sequence = 0
        self.entry_goal_progress_m = 0.0
        self.entry_attempt_start_progress_m = 0.0
        self.entry_retry_count = 0
        self.entry_retry_at = rospy.Time(0)
        self.entry_waiting_for_localization = False
        self.entrance_ready = not self.require_entrance_ready
        self.exploration_completion_armed = False
        self.finalized = False
        self.autostart_attempted = False

        self.status_pub = rospy.Publisher(
            self.mission_status_topic, MissionStatus, queue_size=10, latch=True
        )
        self.active_pub = rospy.Publisher(
            self.mission_active_topic, Bool, queue_size=10, latch=True
        )
        self.entrance_ready_sub = rospy.Subscriber(
            self.entrance_ready_topic, Bool, self._entrance_ready_callback, queue_size=2
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
        rospy.on_shutdown(self._on_shutdown)
        self._publish_status()
        self.active_pub.publish(Bool(data=False))
        rospy.loginfo(
            "[mission] ready: result=%s autostart=%s",
            self.result_file,
            self.autostart,
        )

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

    def _mapping_status_callback(self, message):
        with self.lock:
            self.mapping_status = copy.deepcopy(message)
            self.current_floor = int(message.current_floor)
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

    def _detections_callback(self, message):
        with self.lock:
            if self.mission_state != MissionLifecycle.EXPLORING:
                return
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
        if not mapping.ready or not mapping.stable or mapping.lost:
            return "mapping_not_ready"
        if not navigation.ready:
            return "navigation_not_ready"
        if not detection.ready:
            return "perception_not_ready"
        if not self.move_base_client.wait_for_server(rospy.Duration(0.05)):
            return "move_base_unavailable"
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
            self.entry_goal_active = False
            self.entry_goal_sequence += 1
            self.entry_goal_progress_m = 0.0
            self.entry_attempt_start_progress_m = 0.0
            self.entry_retry_count = 0
            self.entry_retry_at = rospy.Time(0)
            self.entry_waiting_for_localization = False
            self.exploration_completion_armed = False
            self.finalized = False
            self.remaining_frontier_count = 0
            self.map_coverage_summary = ""
        self.active_pub.publish(Bool(data=True))
        self._publish_status()
        if self.entry_enabled:
            success, message = self._advance_entry()
            if not success:
                self._finalize("entry_start_failed:" + message, error=True)
                return TriggerResponse(False, message)
            rospy.loginfo(
                "[mission] ENTERING; home captured, rolling target %.2f m ahead",
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

    def _advance_entry(self):
        with self.lock:
            if self.mission_state != MissionLifecycle.ENTERING or self.finalized:
                return False, "Mission is not entering"
            if self.entry_goal_active:
                return True, "Entry goal already active"
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
            self.return_goal_active = False
            self.entry_goal_active = False
            self.entry_goal_sequence += 1
            self.entry_retry_at = rospy.Time(0)
            self.entry_waiting_for_localization = False
            self.exploration_completion_armed = False
        try:
            self.stop_explore_client()
        except rospy.ServiceException as exc:
            rospy.logwarn("[mission] stop exploration failed: %s", str(exc))

        if home_pose is None:
            self._finalize("home_pose_missing", error=True)
            return False, "Home pose missing"
        if not self.move_base_client.wait_for_server(rospy.Duration(1.0)):
            self._finalize("move_base_unavailable_for_return", error=True)
            return False, "move_base unavailable"

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.pose = copy.deepcopy(home_pose.pose.pose)
        self.move_base_client.send_goal(goal, done_cb=self._return_done_callback)
        with self.lock:
            self.return_goal_active = True
        self._publish_status()
        rospy.loginfo("[mission] RETURNING to captured home pose: %s", reason)
        return True, "Return started"

    def _return_done_callback(self, state, _result):
        with self.lock:
            if self.mission_state != MissionLifecycle.RETURNING or self.finalized:
                return
            self.return_goal_active = False
        if state == GoalStatus.SUCCEEDED:
            self._finalize("completed", error=False)
        else:
            self._finalize("return_failed_action_state_%d" % state, error=True)

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
            should_autostart = (
                self.autostart
                and not self.autostart_attempted
                and state == MissionLifecycle.IDLE
            )
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
            self._finalize("return_timeout", error=True)
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

    def _write_result_file(self):
        with self.lock:
            home = copy.deepcopy(self.home_pose)
            tracks = list(self.tracker.confirmed_tracks())
            start_time = self.start_time
            finish_time = self.finish_time or rospy.Time.now()
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
        now = rospy.Time.now()
        with self.lock:
            state = self.mission_state
            start_time = self.start_time
            finish_time = self.finish_time
            navigation = self.navigation_health
            current_floor = self.current_floor
            remaining = self.remaining_frontier_count
            coverage = self.map_coverage_summary
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
        message.remaining_frontier_count = remaining
        message.finish_reason = finish_reason
        self.status_pub.publish(message)

    def _on_shutdown(self):
        with self.lock:
            active = self.mission_state in (
                MissionLifecycle.EXPLORING,
                MissionLifecycle.ENTERING,
                MissionLifecycle.RETURNING,
            )
        if active:
            self.move_base_client.cancel_all_goals()

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        MissionManager().run()
    except rospy.ROSInterruptException:
        pass
