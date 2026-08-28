#!/usr/bin/env python3
"""Translate standard move_base state into danger-search compatibility topics."""

import copy
import math
import os
import sys
import threading

import rospy
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from danger_search_common.msg import (
    FloorOccupancyGrid,
    MappingStatus,
    NavigationHealth,
    RecoveryEvent,
)
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import (
    MoveBaseActionGoal,
    MoveBaseActionResult,
    RecoveryStatus,
)
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Empty

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from navigation_monitor_core import (
    GoalEpochTracker,
    classify_terminal_status,
    maneuver_from_command,
    polyline_progress,
    recovery_has_translation_progress,
    recovery_maneuver,
)


class NavigationMonitor:
    def __init__(self):
        rospy.init_node("navigation_monitor", anonymous=False)
        self.lock = threading.RLock()
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.health_rate = self._positive("~health_rate", 5.0)
        self.input_timeout = self._positive("~input_timeout", 2.0)
        self.scan_timeout = self._positive("~scan_timeout", 1.0)
        self.command_timeout = self._positive("~command_timeout", 0.50)
        self.config_timeout = self._positive("~config_timeout", 2.0)
        self.recovery_success_distance = self._positive(
            "~recovery_success_distance", 0.05
        )
        self.clear_costmaps_timeout = self._positive(
            "~clear_costmaps_timeout_s", 2.0
        )
        self.clear_costmaps_service_name = rospy.get_param(
            "~clear_costmaps_service", "/move_base/clear_costmaps"
        )

        self.pose = None
        self.goal_pose = None
        self.path = []
        self.active_goal_id = ""
        self.goal_tracker = GoalEpochTracker()
        self.has_active_goal = False
        self.mapping_ready = False
        self.mapping_stable = False
        self.mapping_lost = True
        self.mapping_transitioning = True
        self.mapping_floor = None
        self.mapping_epoch = None
        self.mapping_version = None
        self.raw_map_signature = None
        self.active_map_signature = None
        self.active_map_identity = None
        self.last_active_map = rospy.Time(0)
        self.map_gate_ready = False
        self.map_gate_identity = None
        self.costmaps_cleared_epoch = None
        self.clear_generation = 0
        self.clear_inflight = False
        self.clear_failure = ""
        self.active_goal_binding = None
        self.active_goal_accepted_stamp = rospy.Time(0)
        self.path_goal_epoch = None
        self.last_path_stamp = rospy.Time(0)
        self.invalidated_goal_ids = set()
        self.config_ready = False
        self.last_pose = rospy.Time(0)
        self.last_map = rospy.Time(0)
        self.last_scan = rospy.Time(0)
        self.last_mapping = rospy.Time(0)
        self.last_status = rospy.Time(0)
        self.last_nav_cmd = rospy.Time(0)
        self.last_sent_cmd = rospy.Time(0)
        self.last_config_ready = rospy.Time(0)
        self.last_posture_fallen = rospy.Time(0)
        self.posture_fallen = False
        self.last_sent_moving = False
        self.last_sent_translation = False
        self.failure_code = "NONE"
        self.failure_detail = ""
        self.recovery_event_id = 0
        self.recovery = None
        self.escape_attempt_count = 0
        self.plan_generation = 0

        self.health_pub = rospy.Publisher(
            rospy.get_param("~health_topic", "/navigation/health"),
            NavigationHealth,
            queue_size=10,
            latch=True,
        )
        self.recovery_pub = rospy.Publisher(
            rospy.get_param(
                "~recovery_event_topic", "/navigation/recovery_event"
            ),
            RecoveryEvent,
            queue_size=10,
            latch=True,
        )
        self.cancel_pub = rospy.Publisher(
            rospy.get_param("~move_base_cancel_topic", "/move_base/cancel"),
            GoalID,
            queue_size=10,
        )
        self.clear_costmaps = rospy.ServiceProxy(
            self.clear_costmaps_service_name, Empty
        )

        rospy.Subscriber(
            rospy.get_param("~pose_topic", "/localization/pose"),
            PoseWithCovarianceStamped,
            self._pose_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~map_topic", "/map"),
            OccupancyGrid,
            self._map_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~active_map_topic", "/mapping/active_map"),
            FloorOccupancyGrid,
            self._active_map_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~scan_topic", "/localization/scan"),
            LaserScan,
            self._scan_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~mapping_status_topic", "/mapping/status"),
            MappingStatus,
            self._mapping_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~nav_cmd_topic", "/danger_search/nav_cmd_vel"),
            Twist,
            self._nav_cmd_callback,
            queue_size=20,
        )
        rospy.Subscriber(
            rospy.get_param("~sent_cmd_topic", "/danger_search/cmd_vel_sent"),
            Twist,
            self._sent_cmd_callback,
            queue_size=20,
        )
        rospy.Subscriber(
            rospy.get_param("~config_ready_topic", "/navigation/config_ready"),
            Bool,
            self._config_ready_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~posture_fallen_topic", "/danger_search/posture_fallen"),
            Bool,
            self._posture_fallen_callback,
            queue_size=2,
        )
        rospy.Subscriber(
            rospy.get_param("~move_base_status_topic", "/move_base/status"),
            GoalStatusArray,
            self._status_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~move_base_goal_topic", "/move_base/goal"),
            MoveBaseActionGoal,
            self._goal_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~move_base_result_topic", "/move_base/result"),
            MoveBaseActionResult,
            self._result_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~move_base_recovery_topic", "/move_base/recovery_status"
            ),
            RecoveryStatus,
            self._recovery_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~global_plan_topic", "/move_base/NavfnROS/plan"),
            Path,
            self._path_callback,
            queue_size=2,
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.health_rate), self._timer_callback
        )
        rospy.loginfo("[navigation_monitor] standard move_base compatibility ready")

    @staticmethod
    def _positive(name, default):
        value = float(rospy.get_param(name, default))
        if not math.isfinite(value) or value <= 0.0:
            raise rospy.ROSInitException("%s must be positive and finite" % name)
        return value

    @staticmethod
    def _pose_tuple(pose):
        orientation = pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y
                         + orientation.z * orientation.z),
        )
        return (pose.position.x, pose.position.y, yaw)

    def _pose_callback(self, message):
        if message.header.frame_id != self.map_frame:
            return
        values = self._pose_tuple(message.pose.pose)
        if not all(math.isfinite(value) for value in values):
            return
        with self.lock:
            self.pose = values
            self.last_pose = rospy.Time.now()

    @staticmethod
    def _grid_signature(message):
        """Return a cheap identity for one complete OccupancyGrid payload."""
        if message.info.width * message.info.height != len(message.data):
            return None
        origin = message.info.origin
        return (
            int(message.header.stamp.secs),
            int(message.header.stamp.nsecs),
            int(message.info.map_load_time.secs),
            int(message.info.map_load_time.nsecs),
            int(message.info.width),
            int(message.info.height),
            float(message.info.resolution),
            float(origin.position.x),
            float(origin.position.y),
            float(origin.orientation.z),
            float(origin.orientation.w),
            len(message.data),
        )

    def _map_callback(self, message):
        signature = self._grid_signature(message)
        if signature is None or message.header.frame_id != self.map_frame:
            return
        clear_request = None
        with self.lock:
            self.last_map = rospy.Time.now()
            self.raw_map_signature = signature
            clear_request = self._refresh_map_gate_locked()
        self._start_clear_costmaps(*clear_request) if clear_request else None

    def _active_map_callback(self, envelope):
        message = envelope.occupancy_grid
        signature = self._grid_signature(message)
        if signature is None or message.header.frame_id != self.map_frame:
            return
        identity = (
            int(envelope.floor_id),
            int(envelope.map_epoch),
            int(envelope.map_version),
        )
        if identity[1] < 1 or identity[2] < 1:
            return
        clear_request = None
        with self.lock:
            self.last_active_map = rospy.Time.now()
            self.active_map_signature = signature
            self.active_map_identity = identity
            clear_request = self._refresh_map_gate_locked()
        self._start_clear_costmaps(*clear_request) if clear_request else None

    def _scan_callback(self, _message):
        with self.lock:
            self.last_scan = rospy.Time.now()

    def _mapping_callback(self, message):
        previous = None
        cancel = None
        clear_request = None
        with self.lock:
            old_floor_epoch = (self.mapping_floor, self.mapping_epoch)
            current_floor = int(message.current_floor)
            current_epoch = int(message.map_epoch)
            current_version = None
            for item in message.floor_maps:
                if int(item.floor_id) == current_floor:
                    current_version = int(item.map_version)
                    break
            changed_floor_epoch = old_floor_epoch != (
                current_floor, current_epoch
            )
            entering_transition = (
                bool(message.transitioning) and not self.mapping_transitioning
            )
            self.mapping_ready = message.ready
            self.mapping_stable = message.stable
            self.mapping_lost = message.lost
            self.mapping_transitioning = bool(message.transitioning)
            self.mapping_floor = current_floor
            self.mapping_epoch = current_epoch
            self.mapping_version = current_version
            self.last_mapping = rospy.Time.now()
            if changed_floor_epoch:
                self.costmaps_cleared_epoch = None
                self.clear_failure = ""
            if changed_floor_epoch or entering_transition:
                previous, cancel = self._invalidate_goal_for_map_locked(
                    "MAP_EPOCH_CHANGED" if changed_floor_epoch else "MAP_TRANSITIONING",
                    "floor/map epoch changed before navigation completed",
                )
            clear_request = self._refresh_map_gate_locked()
        if previous is not None:
            self.recovery_pub.publish(previous)
        if cancel is not None:
            self.cancel_pub.publish(cancel)
        self._start_clear_costmaps(*clear_request) if clear_request else None

    def _matching_map_identity_locked(self):
        if (
            self.mapping_floor is None
            or self.mapping_epoch is None
            or self.mapping_version is None
            or self.active_map_identity is None
            or self.raw_map_signature is None
            or self.raw_map_signature != self.active_map_signature
        ):
            return None
        expected = (
            self.mapping_floor, self.mapping_epoch, self.mapping_version
        )
        if self.active_map_identity != expected:
            return None
        return expected

    def _map_gate_candidate_locked(self):
        identity = self._matching_map_identity_locked()
        return (
            identity
            if (
                identity is not None
                and self.mapping_ready
                and self.mapping_stable
                and not self.mapping_lost
                and not self.mapping_transitioning
            )
            else None
        )

    def _map_epoch_healthy_locked(self):
        return (
            self.mapping_floor is not None
            and self.mapping_epoch is not None
            and self.mapping_ready
            and self.mapping_stable
            and not self.mapping_lost
            and not self.mapping_transitioning
        )

    def _map_gate_is_ready_locked(self):
        identity = self.map_gate_identity
        return (
            identity is not None
            and self._map_epoch_healthy_locked()
            and identity[:2] == (self.mapping_floor, self.mapping_epoch)
            and self.costmaps_cleared_epoch == identity[:2]
            and self.map_gate_ready
        )

    def _reported_map_identity_locked(self):
        """Report the map snapshot actually committed by the epoch gate.

        MappingStatus can advance before the matching `/map` and active-map
        envelope arrive.  Advertising that uncommitted version made
        downstream consumers chase a moving value even though navigation was
        ready on the preceding same-epoch snapshot.
        """
        if (
            self.map_gate_identity is not None
            and self.mapping_floor is not None
            and self.mapping_epoch is not None
            and self.map_gate_identity[:2] == (
                self.mapping_floor, self.mapping_epoch
            )
        ):
            return self.map_gate_identity
        return (
            0 if self.mapping_floor is None else int(self.mapping_floor),
            0 if self.mapping_epoch is None else int(self.mapping_epoch),
            0,
        )

    def _refresh_map_gate_locked(self):
        """Advance the epoch gate without flickering on map-version delivery.

        `/map`, `/mapping/active_map` and MappingStatus use independent ROS
        connections.  A normal publication therefore has a short interval in
        which their versions/signatures differ.  Once an epoch has been
        verified and its costmaps cleared, retain that trust across such
        same-epoch intervals; a version is diagnostic and goal-binding state,
        not a new navigation coordinate frame.
        """
        identity = self._map_gate_candidate_locked()
        current_epoch = (
            (self.mapping_floor, self.mapping_epoch)
            if self.mapping_floor is not None and self.mapping_epoch is not None
            else None
        )

        if not self._map_epoch_healthy_locked():
            self.map_gate_ready = False
            return None

        if (
            self.map_gate_identity is not None
            and self.map_gate_identity[:2] == current_epoch
            and self.costmaps_cleared_epoch == current_epoch
        ):
            if identity is not None:
                self.map_gate_identity = identity
            self.map_gate_ready = True
            self.clear_failure = ""
            return None

        self.map_gate_ready = False
        if self.map_gate_identity is not None and self.map_gate_identity[:2] != current_epoch:
            self.map_gate_identity = None
        if identity is None:
            return None
        self.map_gate_identity = identity
        if self.costmaps_cleared_epoch == identity[:2]:
            self.map_gate_ready = True
            self.clear_failure = ""
            return None
        if self.clear_inflight:
            return None
        # Do not tight-loop a failed ROS service. A later map version or a
        # floor epoch transition supplies a fresh bounded retry opportunity.
        if self.clear_failure == identity:
            return None
        self.clear_generation += 1
        self.clear_inflight = True
        return (identity, self.clear_generation)

    def _start_clear_costmaps(self, identity, generation):
        """Call move_base reset outside callbacks and ignore stale replies."""
        def worker():
            success = False
            detail = ""
            completed = threading.Event()

            def invoke_service():
                nonlocal success, detail
                try:
                    rospy.wait_for_service(
                        self.clear_costmaps_service_name,
                        timeout=self.clear_costmaps_timeout,
                    )
                    self.clear_costmaps()
                    success = True
                except (rospy.ROSException, rospy.ServiceException) as exc:
                    detail = str(exc)
                finally:
                    completed.set()

            invocation = threading.Thread(
                target=invoke_service, name="clear_costmaps_call"
            )
            invocation.daemon = True
            invocation.start()
            # rospy.ServiceProxy has no per-call deadline. Bound the monitor's
            # state transition here; a late daemon reply cannot re-enable an
            # unrelated map epoch.
            if not completed.wait(self.clear_costmaps_timeout):
                detail = "clear_costmaps response timed out"
            next_request, applied = self._complete_clear_costmaps(
                identity, generation, success, detail
            )
            if applied:
                self._publish_health()
            if next_request is not None:
                self._start_clear_costmaps(*next_request)

        thread = threading.Thread(target=worker, name="clear_costmaps")
        thread.daemon = True
        thread.start()

    def _complete_clear_costmaps(
        self, identity, generation, success, detail=""
    ):
        """Commit one bounded clear result iff it still owns the map epoch.

        Keeping this state transition separate from the service thread makes
        success, timeout and stale-generation behavior deterministic to test.
        """
        with self.lock:
            if generation != self.clear_generation:
                return None, False
            self.clear_inflight = False
            current_epoch = (
                (self.mapping_floor, self.mapping_epoch)
                if self.mapping_floor is not None and self.mapping_epoch is not None
                else None
            )
            if current_epoch != identity[:2] or self.mapping_transitioning:
                # A version may advance while the service is running, but a
                # different floor/epoch owns different navigation state.
                return self._refresh_map_gate_locked(), False
            if success:
                self.costmaps_cleared_epoch = identity[:2]
                self.clear_failure = ""
                current_identity = self._map_gate_candidate_locked()
                self.map_gate_identity = (
                    current_identity
                    if current_identity is not None
                    and current_identity[:2] == identity[:2]
                    else identity
                )
                self.map_gate_ready = self._map_epoch_healthy_locked()
            else:
                self.clear_failure = identity
                self.map_gate_ready = False
                self.failure_code = "CONTROL_FAILED"
                self.failure_detail = (
                    "clear_costmaps failed for map epoch: " + str(detail)
                )
            return None, True

    def _nav_cmd_callback(self, message):
        update = None
        with self.lock:
            self.last_nav_cmd = rospy.Time.now()
            if self._recovery_is_current_locked():
                inferred = maneuver_from_command(
                    message.linear.x,
                    message.linear.y,
                    message.angular.z,
                    self.recovery["maneuver"],
                )
                if inferred != self.recovery["maneuver"]:
                    self.recovery["maneuver"] = inferred
                    self.recovery["requested_distance"] = (
                        0.35 if inferred == "BACKUP"
                        else 0.30 if inferred.startswith("STRAFE")
                        else 0.0
                    )
                    update = self._recovery_message_locked(
                        RecoveryEvent.PHASE_TRIGGERED, 0.0
                    )
        if update is not None:
            self.recovery_pub.publish(update)

    def _sent_cmd_callback(self, message):
        translation = math.hypot(message.linear.x, message.linear.y) > 0.02
        moving = (
            translation
            or abs(message.angular.z) > 0.05
        )
        with self.lock:
            self.last_sent_cmd = rospy.Time.now()
            self.last_sent_moving = moving
            self.last_sent_translation = translation
            if self._recovery_is_current_locked() and translation:
                self.recovery["had_translation"] = True

    def _config_ready_callback(self, message):
        with self.lock:
            self.config_ready = bool(message.data)
            self.last_config_ready = rospy.Time.now()

    def _posture_fallen_callback(self, message):
        with self.lock:
            self.posture_fallen = bool(message.data)
            self.last_posture_fallen = rospy.Time.now()

    def _cancel_message(self, goal_id):
        message = GoalID()
        message.stamp = rospy.Time.now()
        message.id = str(goal_id)
        return message

    def _invalidate_goal_for_map_locked(self, code, detail):
        """Close the tracked action before map transition callbacks race in."""
        if not self.goal_tracker.active:
            self.path = []
            self.active_goal_binding = None
            return None, None
        goal_id = self.goal_tracker.active_goal_id
        previous = (
            self._finish_recovery_locked(False)
            if self._recovery_is_current_locked()
            else None
        )
        self.recovery = None
        self.goal_tracker.close_goal(goal_id)
        self.invalidated_goal_ids.add(goal_id)
        self.has_active_goal = False
        self.active_goal_id = goal_id
        self.active_goal_binding = None
        self.active_goal_accepted_stamp = rospy.Time(0)
        self.path_goal_epoch = None
        self.last_path_stamp = rospy.Time(0)
        self.goal_pose = None
        self.path = []
        self.failure_code = code
        self.failure_detail = detail
        return previous, self._cancel_message(goal_id)

    def _accept_goal_locked(self, goal_id, goal_pose=None, accepted_stamp=None):
        """Bind an observed move_base goal to the currently verified map."""
        goal_id = str(goal_id or "")
        if not goal_id:
            return None, None
        is_new_goal = not self.goal_tracker.matches(goal_id)
        previous = None
        if is_new_goal and self.recovery is not None:
            previous = self._finish_recovery_locked(False)
        self.goal_tracker.accept_goal(goal_id)
        self.invalidated_goal_ids.discard(goal_id)
        self.active_goal_id = goal_id
        if goal_pose is not None:
            self.goal_pose = copy.deepcopy(goal_pose)
        self.has_active_goal = True
        if is_new_goal:
            self.failure_code = "NONE"
            self.failure_detail = ""
            self.path = []
            self.path_goal_epoch = None
            self.last_path_stamp = rospy.Time(0)
            self.escape_attempt_count = 0
        accepted_stamp = (
            accepted_stamp if accepted_stamp is not None else rospy.Time(0)
        )
        self.active_goal_accepted_stamp = accepted_stamp
        if self._map_gate_is_ready_locked():
            self.active_goal_binding = {
                "goal_id": goal_id,
                "goal_epoch": self.goal_tracker.epoch,
                "floor_id": self.map_gate_identity[0],
                "map_epoch": self.map_gate_identity[1],
                "map_version": self.map_gate_identity[2],
                "accepted_stamp": accepted_stamp,
            }
            return previous, None
        invalid_previous, cancel = self._invalidate_goal_for_map_locked(
            "MAP_EPOCH_NOT_READY",
            "navigation goal rejected until matching map and costmap reset are ready",
        )
        return previous or invalid_previous, cancel

    def _goal_callback(self, message):
        previous = None
        cancel = None
        with self.lock:
            goal_id = message.goal_id.id
            if not goal_id:
                rospy.logwarn_throttle(
                    2.0, "[navigation_monitor] ignored goal with empty id"
                )
                return
            previous, cancel = self._accept_goal_locked(
                goal_id,
                message.goal.target_pose.pose,
                message.goal_id.stamp,
            )
        if previous is not None:
            self.recovery_pub.publish(previous)
        if cancel is not None:
            self.cancel_pub.publish(cancel)

    def _status_callback(self, message):
        active_states = {
            GoalStatus.PENDING,
            GoalStatus.ACTIVE,
            GoalStatus.PREEMPTING,
            GoalStatus.RECALLING,
        }
        active = [status for status in message.status_list
                  if status.status in active_states]
        previous = None
        cancel = None
        with self.lock:
            self.last_status = rospy.Time.now()
            matching = [
                status for status in active
                if self.goal_tracker.matches(status.goal_id.id)
            ]
            if matching:
                self.has_active_goal = True
            elif active and not self.goal_tracker.active:
                selected = active[-1]
                if selected.goal_id.id in self.invalidated_goal_ids:
                    return
                try:
                    previous, cancel = self._accept_goal_locked(
                        selected.goal_id.id
                    )
                except ValueError:
                    return
        if previous is not None:
            self.recovery_pub.publish(previous)
        if cancel is not None:
            self.cancel_pub.publish(cancel)

    def _path_callback(self, message):
        points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in message.poses
            if math.isfinite(pose.pose.position.x)
            and math.isfinite(pose.pose.position.y)
        ]
        with self.lock:
            binding = self.active_goal_binding
            if (
                not self.has_active_goal
                or not self.goal_tracker.active
                or binding is None
                or not self.goal_tracker.matches(
                    binding["goal_id"], binding["goal_epoch"]
                )
            ):
                return
            stamp = message.header.stamp
            accepted_stamp = binding["accepted_stamp"]
            if (
                stamp == rospy.Time(0)
                or stamp < accepted_stamp
                or (
                    self.last_path_stamp != rospy.Time(0)
                    and stamp <= self.last_path_stamp
                )
            ):
                return
            self.path = points
            self.path_goal_epoch = binding["goal_epoch"]
            self.last_path_stamp = stamp
            if len(points) >= 2:
                self.plan_generation += 1

    def _result_callback(self, message):
        event = None
        ignored = False
        with self.lock:
            result_goal_id = message.status.goal_id.id
            if not self.goal_tracker.matches(result_goal_id):
                ignored = True
            else:
                code = classify_terminal_status(
                    message.status.status, message.status.text
                )
                self.failure_code = code
                self.failure_detail = message.status.text or code
                self.has_active_goal = False
                self.active_goal_id = result_goal_id
                if self._recovery_is_current_locked():
                    event = self._finish_recovery_locked(
                        message.status.status == GoalStatus.SUCCEEDED
                    )
                self.goal_tracker.close_goal(result_goal_id)
                self.active_goal_binding = None
                self.active_goal_accepted_stamp = rospy.Time(0)
                self.goal_pose = None
                self.path = []
                self.path_goal_epoch = None
                self.last_path_stamp = rospy.Time(0)
                self.recovery = None
        if ignored:
            rospy.logwarn_throttle(
                2.0,
                "[navigation_monitor] ignored terminal result for stale goal %s",
                message.status.goal_id.id or "<empty>",
            )
            return
        if event is not None:
            self.recovery_pub.publish(event)
        self._publish_health()

    def _recovery_callback(self, message):
        previous = None
        with self.lock:
            if not self.has_active_goal or not self.goal_tracker.active:
                rospy.logwarn_throttle(
                    2.0,
                    "[navigation_monitor] ignored recovery without active goal",
                )
                return
            if self._recovery_is_current_locked():
                previous = self._finish_recovery_locked(False)
            elif self.recovery is not None:
                self.recovery = None
            self.recovery_event_id += 1
            stuck_pose = self._pose_tuple(message.pose_stamped.pose)
            if not all(math.isfinite(value) for value in stuck_pose):
                stuck_pose = self.pose or (0.0, 0.0, 0.0)
            behavior_name = message.recovery_behavior_name
            if "escape_recovery" in behavior_name.lower():
                self.escape_attempt_count += 1
                attempt = self.escape_attempt_count
            else:
                attempt = int(message.current_recovery_number) + 1
            self.recovery = {
                "event_id": self.recovery_event_id,
                "goal_id": self.goal_tracker.active_goal_id,
                "goal_epoch": self.goal_tracker.epoch,
                "behavior": behavior_name,
                "maneuver": recovery_maneuver(behavior_name),
                "attempt": attempt,
                "requested_distance": 0.0,
                "had_translation": False,
                "stuck_pose": stuck_pose,
                "goal_pose": copy.deepcopy(self.goal_pose),
                "plan_generation": self.plan_generation,
            }
            triggered = self._recovery_message_locked(
                RecoveryEvent.PHASE_TRIGGERED, 0.0
            )
        if previous is not None:
            self.recovery_pub.publish(previous)
        self.recovery_pub.publish(triggered)

    def _timer_callback(self, _event):
        event = None
        with self.lock:
            if (self._recovery_is_current_locked() and self.pose is not None
                    and self._fresh(
                        rospy.Time.now(), self.last_sent_cmd, self.command_timeout
                    )):
                stuck = self.recovery["stuck_pose"]
                achieved = math.hypot(
                    self.pose[0] - stuck[0], self.pose[1] - stuck[1]
                )
                translating_escape = (
                    "escape_recovery" in self.recovery["behavior"].lower()
                    and self.recovery["had_translation"]
                    and achieved >= self.recovery_success_distance
                    and self.plan_generation > self.recovery["plan_generation"]
                )
                if (translating_escape or recovery_has_translation_progress(
                        achieved, self.recovery_success_distance,
                        self.plan_generation,
                        self.recovery["plan_generation"],
                        self.last_sent_translation)):
                    event = self._finish_recovery_locked(True)
        if event is not None:
            self.recovery_pub.publish(event)
        self._publish_health()

    def _finish_recovery_locked(self, success):
        if self.recovery is None:
            return None
        stuck = self.recovery["stuck_pose"]
        achieved = 0.0 if self.pose is None else math.hypot(
            self.pose[0] - stuck[0], self.pose[1] - stuck[1]
        )
        phase = (
            RecoveryEvent.PHASE_SUCCEEDED
            if success else RecoveryEvent.PHASE_FAILED
        )
        message = self._recovery_message_locked(phase, achieved)
        self.recovery = None
        return message

    def _recovery_is_current_locked(self):
        return (
            self.recovery is not None
            and self.goal_tracker.matches(
                self.recovery["goal_id"], self.recovery["goal_epoch"]
            )
        )

    def _recovery_message_locked(self, phase, achieved):
        state = self.recovery
        message = RecoveryEvent()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.map_frame
        message.event_id = state["event_id"]
        message.active_goal_id = state["goal_id"]
        message.phase = phase
        maneuver_values = {
            "BACKUP": RecoveryEvent.MANEUVER_BACKUP,
            "STRAFE_LEFT": RecoveryEvent.MANEUVER_STRAFE_LEFT,
            "STRAFE_RIGHT": RecoveryEvent.MANEUVER_STRAFE_RIGHT,
            "ROTATE": RecoveryEvent.MANEUVER_ROTATE,
        }
        message.maneuver = maneuver_values.get(
            state["maneuver"], RecoveryEvent.MANEUVER_NONE
        )
        message.stuck_pose.position.x = state["stuck_pose"][0]
        message.stuck_pose.position.y = state["stuck_pose"][1]
        message.stuck_pose.orientation.z = math.sin(
            0.5 * state["stuck_pose"][2]
        )
        message.stuck_pose.orientation.w = math.cos(
            0.5 * state["stuck_pose"][2]
        )
        if state["goal_pose"] is not None:
            message.goal_pose = copy.deepcopy(state["goal_pose"])
        else:
            message.goal_pose.orientation.w = 1.0
        message.attempt = state["attempt"]
        message.requested_distance = state.get("requested_distance", 0.0)
        message.achieved_distance = float(achieved)
        message.min_clearance = 0.0
        return message

    @staticmethod
    def _fresh(now, stamp, timeout):
        return (
            stamp != rospy.Time(0)
            and 0.0 <= (now - stamp).to_sec() <= timeout
        )

    def _publish_health(self):
        now = rospy.Time.now()
        with self.lock:
            map_gate_ready = self._map_gate_is_ready_locked()
            reported_map_identity = self._reported_map_identity_locked()
            inputs_ready = (
                self._fresh(now, self.last_pose, self.input_timeout)
                and self._fresh(now, self.last_map, self.input_timeout)
                and self._fresh(now, self.last_active_map, self.input_timeout)
                and self._fresh(now, self.last_scan, self.scan_timeout)
                and self._fresh(now, self.last_mapping, self.input_timeout)
                and self._fresh(now, self.last_status, self.input_timeout)
                and self._fresh(now, self.last_config_ready, self.config_timeout)
                and self._fresh(now, self.last_posture_fallen, self.input_timeout)
                and self.config_ready
                and self.mapping_ready
                and self.mapping_stable
                and not self.mapping_lost
                and map_gate_ready
            )
            message = NavigationHealth()
            message.header.stamp = now
            message.header.frame_id = self.map_frame
            message.ready = inputs_ready
            message.has_active_goal = self.has_active_goal
            message.active_goal_id = self.active_goal_id
            message.current_floor = reported_map_identity[0]
            message.map_epoch = reported_map_identity[1]
            message.map_version = reported_map_identity[2]
            message.transitioning = bool(
                self.mapping_transitioning or not map_gate_ready
            )
            message.controller_active = (
                self.has_active_goal
                and self._fresh(now, self.last_nav_cmd, self.command_timeout)
            )
            message.stuck = self.recovery is not None
            message.fallen = bool(
                self.posture_fallen
                and self._fresh(now, self.last_posture_fallen, self.input_timeout)
            )
            message.progress = (
                polyline_progress(self.path, self.pose[:2])
                if self.pose is not None else 0.0
            )
            message.last_cmd_time = self.last_nav_cmd
            if (self.has_active_goal and not inputs_ready
                    and (self.mapping_lost
                         or not self._fresh(
                             now, self.last_pose, self.input_timeout
                         ))):
                message.failure_code = "LOCALIZATION_LOST"
                message.failure_detail = "standard move_base input is stale"
            elif (self.has_active_goal and not self.config_ready):
                message.failure_code = "CONTROL_FAILED"
                message.failure_detail = "selected local planner configuration is not verified"
            elif self.has_active_goal and not map_gate_ready:
                message.failure_code = "MAP_EPOCH_NOT_READY"
                message.failure_detail = (
                    "matching active map and costmap reset are required"
                )
            else:
                message.failure_code = self.failure_code
                message.failure_detail = self.failure_detail
        self.health_pub.publish(message)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        NavigationMonitor().run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
