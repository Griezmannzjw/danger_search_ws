#!/usr/bin/env python3

import os
import sys
import threading
import unittest
from unittest.mock import patch

from actionlib_msgs.msg import GoalStatus
from danger_search_common.msg import MappingStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from nav_msgs.msg import Path
from std_msgs.msg import Bool

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


def make_goal(goal_id, stamp=0.0):
    message = MoveBaseActionGoal()
    message.goal_id.id = goal_id
    message.goal_id.stamp = monitor_module.rospy.Time.from_sec(stamp)
    message.goal.target_pose.pose.orientation.w = 1.0
    return message


def make_result(goal_id, status, text=""):
    message = MoveBaseActionResult()
    message.status.goal_id.id = goal_id
    message.status.status = status
    message.status.text = text
    return message


def make_path(stamp):
    message = Path()
    message.header.stamp = monitor_module.rospy.Time.from_sec(stamp)
    for x in (0.0, 1.0):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
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
    monitor.plan_generation = 0
    monitor.path_goal_epoch = None
    monitor.last_path_stamp = monitor_module.rospy.Time(0)
    monitor.active_goal_accepted_stamp = monitor_module.rospy.Time(0)
    monitor.escape_attempt_count = 0
    monitor.recovery = None
    monitor.recovery_pub = FakePublisher()
    monitor.cancel_pub = FakePublisher()
    monitor.pose = (0.0, 0.0, 0.0)
    monitor.map_frame = "map"
    monitor.health_publish_count = 0
    monitor._publish_health = lambda: setattr(
        monitor, "health_publish_count", monitor.health_publish_count + 1
    )
    monitor.mapping_ready = True
    monitor.mapping_stable = True
    monitor.mapping_lost = False
    monitor.mapping_transitioning = False
    monitor.mapping_floor = 0
    monitor.mapping_epoch = 1
    monitor.mapping_version = 3
    monitor.raw_map_signature = ("same",)
    monitor.active_map_signature = ("same",)
    monitor.active_map_identity = (0, 1, 3)
    monitor.map_gate_ready = True
    monitor.map_gate_identity = (0, 1, 3)
    monitor.costmaps_cleared_epoch = (0, 1)
    monitor.clear_inflight = False
    monitor.clear_failure = ""
    monitor.clear_generation = 0
    monitor.invalidated_goal_ids = set()
    monitor.active_goal_binding = None
    return monitor


class NavigationMonitorCallbackTest(unittest.TestCase):
    def test_posture_fallen_callback_tracks_explicit_fallen_signal(self):
        monitor = make_monitor()
        monitor.posture_fallen = False
        monitor.last_posture_fallen = monitor_module.rospy.Time(0)
        with patch.object(
            monitor_module.rospy.Time, "now",
            return_value=monitor_module.rospy.Time.from_sec(5.0),
        ):
            monitor._posture_fallen_callback(Bool(data=True))
        self.assertTrue(monitor.posture_fallen)
        self.assertEqual(monitor.last_posture_fallen, monitor_module.rospy.Time.from_sec(5.0))

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

    def test_map_epoch_change_cancels_goal_and_ignores_late_result(self):
        monitor = make_monitor()
        monitor._goal_callback(make_goal("goal-a"))
        self.assertTrue(monitor.goal_tracker.matches("goal-a"))

        status = MappingStatus()
        status.current_floor = 1
        status.map_epoch = 2
        status.ready = False
        status.stable = False
        status.transitioning = True
        status.lost = False
        with patch.object(
            monitor_module.rospy.Time,
            "now",
            return_value=monitor_module.rospy.Time(0),
        ):
            monitor._mapping_callback(status)

        self.assertFalse(monitor.goal_tracker.matches("goal-a"))
        self.assertFalse(monitor.has_active_goal)
        self.assertEqual(monitor.failure_code, "MAP_EPOCH_CHANGED")
        self.assertEqual(len(monitor.cancel_pub.messages), 1)
        self.assertEqual(monitor.cancel_pub.messages[0].id, "goal-a")
        with patch.object(monitor_module.rospy, "logwarn_throttle"):
            monitor._result_callback(
                make_result("goal-a", GoalStatus.SUCCEEDED, "late")
            )
        self.assertEqual(monitor.failure_code, "MAP_EPOCH_CHANGED")

    def test_goal_is_immediately_canceled_until_map_gate_is_ready(self):
        monitor = make_monitor()
        monitor.mapping_transitioning = True
        monitor.map_gate_ready = False

        with patch.object(
            monitor_module.rospy.Time,
            "now",
            return_value=monitor_module.rospy.Time(0),
        ):
            monitor._goal_callback(make_goal("goal-before-map"))

        self.assertFalse(monitor.has_active_goal)
        self.assertFalse(monitor.goal_tracker.matches("goal-before-map"))
        self.assertEqual(monitor.failure_code, "MAP_EPOCH_NOT_READY")
        self.assertEqual(len(monitor.cancel_pub.messages), 1)

    def test_clear_costmaps_success_is_required_for_ready_gate(self):
        monitor = make_monitor()
        identity = (0, 1, 3)
        monitor.costmaps_cleared_epoch = None
        monitor.map_gate_ready = False
        monitor.clear_generation = 4
        monitor.clear_inflight = True

        next_request, applied = monitor._complete_clear_costmaps(
            identity, 4, True
        )

        self.assertTrue(applied)
        self.assertIsNone(next_request)
        self.assertTrue(monitor.map_gate_ready)
        self.assertEqual(monitor.costmaps_cleared_epoch, (0, 1))
        self.assertTrue(monitor._map_gate_is_ready_locked())

    def test_same_epoch_version_delivery_does_not_flicker_ready_gate(self):
        monitor = make_monitor()

        # `/map` for version 4 arrives before its envelope and status.
        monitor.raw_map_signature = ("new",)
        self.assertIsNone(monitor._refresh_map_gate_locked())
        self.assertTrue(monitor._map_gate_is_ready_locked())
        self.assertEqual(monitor.map_gate_identity, (0, 1, 3))

        # The envelope arrives next, while MappingStatus still says version 3.
        monitor.active_map_signature = ("new",)
        monitor.active_map_identity = (0, 1, 4)
        self.assertIsNone(monitor._refresh_map_gate_locked())
        self.assertTrue(monitor._map_gate_is_ready_locked())

        # Full agreement advances only the diagnostic/goal-binding version.
        monitor.mapping_version = 4
        self.assertIsNone(monitor._refresh_map_gate_locked())
        self.assertTrue(monitor._map_gate_is_ready_locked())
        self.assertEqual(monitor.map_gate_identity, (0, 1, 4))

    def test_health_identity_reports_committed_gate_snapshot(self):
        monitor = make_monitor()
        monitor.mapping_version = 8
        monitor.map_gate_identity = (0, 1, 6)

        self.assertEqual(
            monitor._reported_map_identity_locked(), (0, 1, 6)
        )

    def test_unready_health_identity_reports_transition_target(self):
        monitor = make_monitor()
        monitor.mapping_floor = 2
        monitor.mapping_epoch = 9
        monitor.mapping_version = 4
        monitor.map_gate_ready = False

        self.assertEqual(
            monitor._reported_map_identity_locked(), (2, 9, 0)
        )

    def test_unready_health_identity_keeps_same_epoch_gate_candidate(self):
        monitor = make_monitor()
        monitor.map_gate_identity = (0, 1, 5)
        monitor.mapping_version = 7
        monitor.map_gate_ready = False

        self.assertEqual(
            monitor._reported_map_identity_locked(), (0, 1, 5)
        )

    def test_status_first_same_epoch_version_does_not_flicker_ready_gate(self):
        monitor = make_monitor()
        monitor.mapping_version = 4

        self.assertIsNone(monitor._refresh_map_gate_locked())
        self.assertTrue(monitor._map_gate_is_ready_locked())
        monitor.raw_map_signature = ("new",)
        self.assertIsNone(monitor._refresh_map_gate_locked())
        self.assertTrue(monitor._map_gate_is_ready_locked())
        monitor.active_map_signature = ("new",)
        monitor.active_map_identity = (0, 1, 4)
        self.assertIsNone(monitor._refresh_map_gate_locked())
        self.assertEqual(monitor.map_gate_identity, (0, 1, 4))

    def test_clear_success_accepts_newer_version_in_same_epoch(self):
        monitor = make_monitor()
        dispatched = (0, 1, 3)
        monitor.costmaps_cleared_epoch = None
        monitor.map_gate_ready = False
        monitor.mapping_version = 4
        monitor.active_map_identity = (0, 1, 4)
        monitor.clear_generation = 6
        monitor.clear_inflight = True

        next_request, applied = monitor._complete_clear_costmaps(
            dispatched, 6, True
        )

        self.assertTrue(applied)
        self.assertIsNone(next_request)
        self.assertEqual(monitor.costmaps_cleared_epoch, (0, 1))
        self.assertEqual(monitor.map_gate_identity, (0, 1, 4))
        self.assertTrue(monitor._map_gate_is_ready_locked())

    def test_clear_success_from_old_epoch_cannot_reenable_gate(self):
        monitor = make_monitor()
        dispatched = (0, 1, 3)
        monitor.costmaps_cleared_epoch = None
        monitor.map_gate_ready = False
        monitor.mapping_floor = 1
        monitor.mapping_epoch = 2
        monitor.mapping_version = 1
        monitor.active_map_identity = (1, 2, 1)
        monitor.clear_generation = 7
        monitor.clear_inflight = True

        next_request, applied = monitor._complete_clear_costmaps(
            dispatched, 7, True
        )

        self.assertFalse(applied)
        self.assertIsNotNone(next_request)
        self.assertEqual(next_request[0], (1, 2, 1))
        self.assertIsNone(monitor.costmaps_cleared_epoch)
        self.assertFalse(monitor.map_gate_ready)

    def test_clear_costmaps_timeout_keeps_gate_not_ready(self):
        monitor = make_monitor()
        identity = (0, 1, 3)
        monitor.costmaps_cleared_epoch = None
        monitor.map_gate_ready = False
        monitor.clear_generation = 5
        monitor.clear_inflight = True

        next_request, applied = monitor._complete_clear_costmaps(
            identity, 5, False, "clear_costmaps response timed out"
        )

        self.assertTrue(applied)
        self.assertIsNone(next_request)
        self.assertFalse(monitor.map_gate_ready)
        self.assertIsNone(monitor.costmaps_cleared_epoch)
        self.assertEqual(monitor.clear_failure, identity)
        self.assertEqual(monitor.failure_code, "CONTROL_FAILED")

    def test_stale_clear_generation_cannot_reenable_gate(self):
        monitor = make_monitor()
        identity = (0, 1, 3)
        monitor.costmaps_cleared_epoch = None
        monitor.map_gate_ready = False
        monitor.clear_generation = 9
        monitor.clear_inflight = True

        next_request, applied = monitor._complete_clear_costmaps(
            identity, 8, True
        )

        self.assertFalse(applied)
        self.assertIsNone(next_request)
        self.assertFalse(monitor.map_gate_ready)
        self.assertIsNone(monitor.costmaps_cleared_epoch)
        self.assertTrue(monitor.clear_inflight)

    def test_terminal_result_clears_goal_scoped_runtime(self):
        monitor = make_monitor()
        monitor._goal_callback(make_goal("goal-a", stamp=10.0))
        monitor._path_callback(make_path(11.0))
        monitor.recovery = {
            "event_id": 7,
            "goal_id": "goal-a",
            "goal_epoch": monitor.goal_tracker.epoch,
            "behavior": "rotate_recovery",
            "maneuver": "ROTATE",
            "attempt": 1,
            "requested_distance": 0.0,
            "had_translation": False,
            "stuck_pose": (0.0, 0.0, 0.0),
            "goal_pose": monitor.goal_pose,
            "plan_generation": monitor.plan_generation,
        }

        with patch.object(
            monitor_module.rospy.Time,
            "now",
            return_value=monitor_module.rospy.Time.from_sec(12.0),
        ):
            monitor._result_callback(
                make_result("goal-a", GoalStatus.SUCCEEDED, "done")
            )

        self.assertIsNone(monitor.active_goal_binding)
        self.assertEqual(monitor.path, [])
        self.assertIsNone(monitor.path_goal_epoch)
        self.assertIsNone(monitor.recovery)
        self.assertIsNone(monitor.goal_pose)

    def test_paths_are_isolated_by_goal_epoch_and_timestamp(self):
        monitor = make_monitor()
        monitor._goal_callback(make_goal("goal-a", stamp=10.0))
        epoch_a = monitor.goal_tracker.epoch

        monitor._path_callback(make_path(9.0))
        self.assertEqual(monitor.plan_generation, 0)
        monitor._path_callback(make_path(11.0))
        self.assertEqual(monitor.plan_generation, 1)
        self.assertEqual(monitor.path_goal_epoch, epoch_a)
        monitor._path_callback(make_path(11.0))
        self.assertEqual(monitor.plan_generation, 1)

        monitor._goal_callback(make_goal("goal-b", stamp=20.0))
        epoch_b = monitor.goal_tracker.epoch
        self.assertGreater(epoch_b, epoch_a)
        monitor._path_callback(make_path(12.0))
        self.assertEqual(monitor.plan_generation, 1)
        self.assertEqual(monitor.path, [])
        monitor._path_callback(make_path(21.0))
        self.assertEqual(monitor.plan_generation, 2)
        self.assertEqual(monitor.path_goal_epoch, epoch_b)


if __name__ == "__main__":
    unittest.main()
