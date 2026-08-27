#!/usr/bin/env python3

import os
import sys
import threading
import unittest
from unittest.mock import patch

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult

SCRIPT_DIR = os.path.join(os.path.dirname(__file__), "..", "scripts")
sys.path.insert(0, os.path.abspath(SCRIPT_DIR))

import navigation_monitor as monitor_module
from navigation_monitor import NavigationMonitor
from navigation_monitor_core import GoalEpochTracker


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_goal(goal_id):
    message = MoveBaseActionGoal()
    message.goal_id.id = goal_id
    message.goal.target_pose.pose.orientation.w = 1.0
    return message


def make_result(goal_id, status, text=""):
    message = MoveBaseActionResult()
    message.status.goal_id.id = goal_id
    message.status.status = status
    message.status.text = text
    return message


def make_monitor():
    monitor = NavigationMonitor.__new__(NavigationMonitor)
    monitor.lock = threading.RLock()
    monitor.goal_tracker = GoalEpochTracker()
    monitor.active_goal_id = ""
    monitor.goal_pose = None
    monitor.has_active_goal = False
    monitor.failure_code = "NONE"
    monitor.failure_detail = ""
    monitor.path = []
    monitor.escape_attempt_count = 0
    monitor.recovery = None
    monitor.recovery_pub = FakePublisher()
    monitor.pose = (0.0, 0.0, 0.0)
    monitor.map_frame = "map"
    monitor.health_publish_count = 0
    monitor._publish_health = lambda: setattr(
        monitor, "health_publish_count", monitor.health_publish_count + 1
    )
    return monitor


class NavigationMonitorCallbackTest(unittest.TestCase):
    def test_late_result_cannot_clear_or_fail_new_goal(self):
        monitor = make_monitor()
        monitor._goal_callback(make_goal("goal-a"))
        with patch.object(
            monitor_module.rospy.Time,
            "now",
            return_value=monitor_module.rospy.Time(0),
        ):
            monitor._goal_callback(make_goal("goal-b"))

        with patch.object(monitor_module.rospy, "logwarn_throttle"):
            monitor._result_callback(
                make_result(
                    "goal-a", GoalStatus.ABORTED, "Failed to find a valid control"
                )
            )

        self.assertEqual(monitor.active_goal_id, "goal-b")
        self.assertTrue(monitor.has_active_goal)
        self.assertEqual(monitor.failure_code, "NONE")
        self.assertTrue(monitor.goal_tracker.matches("goal-b"))
        self.assertEqual(monitor.health_publish_count, 0)

        monitor._result_callback(
            make_result("goal-b", GoalStatus.SUCCEEDED, "Goal reached")
        )
        self.assertFalse(monitor.has_active_goal)
        self.assertEqual(monitor.failure_code, "SUCCEEDED")
        self.assertFalse(monitor.goal_tracker.matches("goal-b"))
        self.assertEqual(monitor.health_publish_count, 1)

    def test_new_goal_finishes_old_recovery_with_old_goal_id(self):
        monitor = make_monitor()
        monitor._goal_callback(make_goal("goal-a"))
        epoch_a = monitor.goal_tracker.epoch
        monitor.recovery = {
            "event_id": 7,
            "goal_id": "goal-a",
            "goal_epoch": epoch_a,
            "behavior": "escape_recovery_1",
            "maneuver": "BACKUP",
            "attempt": 1,
            "requested_distance": 0.35,
            "had_translation": True,
            "stuck_pose": (0.0, 0.0, 0.0),
            "goal_pose": monitor.goal_pose,
            "plan_generation": 0,
        }

        with patch.object(
            monitor_module.rospy.Time,
            "now",
            return_value=monitor_module.rospy.Time(0),
        ):
            monitor._goal_callback(make_goal("goal-b"))

        self.assertEqual(len(monitor.recovery_pub.messages), 1)
        event = monitor.recovery_pub.messages[0]
        self.assertEqual(event.active_goal_id, "goal-a")
        self.assertEqual(event.phase, event.PHASE_FAILED)
        self.assertIsNone(monitor.recovery)
        self.assertTrue(monitor.goal_tracker.matches("goal-b"))


if __name__ == "__main__":
    unittest.main()
