#!/usr/bin/env python3
"""Translate standard move_base state into danger-search compatibility topics."""

import copy
import math
import os
import sys
import threading

import rospy
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from danger_search_common.msg import MappingStatus, NavigationHealth, RecoveryEvent
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import (
    MoveBaseActionGoal,
    MoveBaseActionResult,
    RecoveryStatus,
)
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from navigation_monitor_core import (
    classify_terminal_status,
    polyline_progress,
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
        self.recovery_success_distance = self._positive(
            "~recovery_success_distance", 0.05
        )

        self.pose = None
        self.goal_pose = None
        self.path = []
        self.active_goal_id = ""
        self.has_active_goal = False
        self.mapping_ready = False
        self.mapping_stable = False
        self.mapping_lost = True
        self.last_pose = rospy.Time(0)
        self.last_map = rospy.Time(0)
        self.last_scan = rospy.Time(0)
        self.last_mapping = rospy.Time(0)
        self.last_status = rospy.Time(0)
        self.last_nav_cmd = rospy.Time(0)
        self.last_sent_cmd = rospy.Time(0)
        self.last_sent_moving = False
        self.failure_code = "NONE"
        self.failure_detail = ""
        self.recovery_event_id = 0
        self.recovery = None

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

    def _map_callback(self, message):
        if (message.header.frame_id != self.map_frame
                or message.info.width * message.info.height != len(message.data)):
            return
        with self.lock:
            self.last_map = rospy.Time.now()

    def _scan_callback(self, _message):
        with self.lock:
            self.last_scan = rospy.Time.now()

    def _mapping_callback(self, message):
        with self.lock:
            self.mapping_ready = message.ready
            self.mapping_stable = message.stable
            self.mapping_lost = message.lost
            self.last_mapping = rospy.Time.now()

    def _nav_cmd_callback(self, _message):
        with self.lock:
            self.last_nav_cmd = rospy.Time.now()

    def _sent_cmd_callback(self, message):
        moving = (
            math.hypot(message.linear.x, message.linear.y) > 0.02
            or abs(message.angular.z) > 0.05
        )
        with self.lock:
            self.last_sent_cmd = rospy.Time.now()
            self.last_sent_moving = moving

    def _goal_callback(self, message):
        with self.lock:
            self.active_goal_id = message.goal_id.id
            self.goal_pose = copy.deepcopy(message.goal.target_pose.pose)
            self.has_active_goal = True
            self.failure_code = "NONE"
            self.failure_detail = ""
            self.path = []

    def _status_callback(self, message):
        active_states = {
            GoalStatus.PENDING,
            GoalStatus.ACTIVE,
            GoalStatus.PREEMPTING,
            GoalStatus.RECALLING,
        }
        active = [status for status in message.status_list
                  if status.status in active_states]
        with self.lock:
            self.last_status = rospy.Time.now()
            if active:
                selected = active[-1]
                self.active_goal_id = selected.goal_id.id
                self.has_active_goal = True

    def _path_callback(self, message):
        points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in message.poses
            if math.isfinite(pose.pose.position.x)
            and math.isfinite(pose.pose.position.y)
        ]
        with self.lock:
            self.path = points

    def _result_callback(self, message):
        event = None
        with self.lock:
            code = classify_terminal_status(
                message.status.status, message.status.text
            )
            self.failure_code = code
            self.failure_detail = message.status.text or code
            self.has_active_goal = False
            if message.status.goal_id.id:
                self.active_goal_id = message.status.goal_id.id
            if self.recovery is not None:
                event = self._finish_recovery_locked(
                    message.status.status == GoalStatus.SUCCEEDED
                )
        if event is not None:
            self.recovery_pub.publish(event)
        self._publish_health()

    def _recovery_callback(self, message):
        previous = None
        with self.lock:
            if self.recovery is not None:
                previous = self._finish_recovery_locked(False)
            self.recovery_event_id += 1
            stuck_pose = self._pose_tuple(message.pose_stamped.pose)
            if not all(math.isfinite(value) for value in stuck_pose):
                stuck_pose = self.pose or (0.0, 0.0, 0.0)
            self.recovery = {
                "event_id": self.recovery_event_id,
                "behavior": message.recovery_behavior_name,
                "maneuver": recovery_maneuver(message.recovery_behavior_name),
                "attempt": int(message.current_recovery_number) + 1,
                "stuck_pose": stuck_pose,
                "goal_pose": copy.deepcopy(self.goal_pose),
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
            if (self.recovery is not None and self.pose is not None
                    and self.last_sent_moving
                    and self._fresh(
                        rospy.Time.now(), self.last_sent_cmd, self.command_timeout
                    )):
                stuck = self.recovery["stuck_pose"]
                achieved = math.hypot(
                    self.pose[0] - stuck[0], self.pose[1] - stuck[1]
                )
                if achieved >= self.recovery_success_distance:
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

    def _recovery_message_locked(self, phase, achieved):
        state = self.recovery
        message = RecoveryEvent()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = self.map_frame
        message.event_id = state["event_id"]
        message.active_goal_id = self.active_goal_id
        message.phase = phase
        message.maneuver = (
            RecoveryEvent.MANEUVER_ROTATE
            if state["maneuver"] == "ROTATE"
            else RecoveryEvent.MANEUVER_NONE
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
        message.requested_distance = 0.0
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
            inputs_ready = (
                self._fresh(now, self.last_pose, self.input_timeout)
                and self._fresh(now, self.last_map, self.input_timeout)
                and self._fresh(now, self.last_scan, self.scan_timeout)
                and self._fresh(now, self.last_mapping, self.input_timeout)
                and self._fresh(now, self.last_status, self.input_timeout)
                and self.mapping_ready
                and self.mapping_stable
                and not self.mapping_lost
            )
            message = NavigationHealth()
            message.header.stamp = now
            message.header.frame_id = self.map_frame
            message.ready = inputs_ready
            message.has_active_goal = self.has_active_goal
            message.active_goal_id = self.active_goal_id
            message.controller_active = (
                self.has_active_goal
                and self._fresh(now, self.last_nav_cmd, self.command_timeout)
            )
            message.stuck = self.recovery is not None
            message.fallen = False
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

