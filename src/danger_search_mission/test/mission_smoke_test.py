#!/usr/bin/env python3

import json
import os
import threading
import time
import unittest

import actionlib
import rospy
import rostest
from danger_search_common.msg import (
    DangerSource,
    DangerSourceArray,
    DetectionStatus,
    MappingStatus,
    MissionStatus,
    NavigationHealth,
)
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse


RESULT_FILE = "/tmp/danger_search_mission_smoke/detected_danger.json"


class MissionSmokeTest(unittest.TestCase):
    def setUp(self):
        if os.path.exists(RESULT_FILE):
            os.remove(RESULT_FILE)
        self.latest_status = None
        self.move_base_goals = []
        self.exploration_started = False
        self.stop_called = False
        self.complete_pub = rospy.Publisher(
            "/exploration/complete", Bool, queue_size=2, latch=True
        )
        self.entrance_ready_pub = rospy.Publisher(
            "/entrance/ready", Bool, queue_size=1, latch=True
        )
        self.entrance_ready_pub.publish(Bool(data=True))
        self.exploration_status_pub = rospy.Publisher(
            "/exploration/status", String, queue_size=2, latch=True
        )
        self.pose_pub = rospy.Publisher(
            "/localization/pose", PoseWithCovarianceStamped, queue_size=5
        )
        self.mapping_pub = rospy.Publisher(
            "/mapping/status", MappingStatus, queue_size=5
        )
        self.navigation_pub = rospy.Publisher(
            "/navigation/health", NavigationHealth, queue_size=5
        )
        self.detection_status_pub = rospy.Publisher(
            "/danger_detector/status", DetectionStatus, queue_size=5
        )
        self.detections_pub = rospy.Publisher(
            "/danger_detector/detections", DangerSourceArray, queue_size=5
        )
        self.status_sub = rospy.Subscriber(
            "/mission/status", MissionStatus, self._status_callback
        )
        self.start_exploration_service = rospy.Service(
            "/danger_search/start_exploration", Trigger, self._start_exploration
        )
        self.stop_exploration_service = rospy.Service(
            "/danger_search/stop_exploration", Trigger, self._stop_exploration
        )
        self.move_base_server = actionlib.SimpleActionServer(
            "/move_base",
            MoveBaseAction,
            execute_cb=self._execute_goal,
            auto_start=False,
        )
        self.move_base_server.start()
        self.health_timer = rospy.Timer(rospy.Duration(0.05), self._publish_health)

    def tearDown(self):
        self.health_timer.shutdown()
        self.start_exploration_service.shutdown()
        self.stop_exploration_service.shutdown()

    def _status_callback(self, message):
        self.latest_status = message

    def _start_exploration(self, _request):
        self.exploration_started = True
        self.complete_pub.publish(Bool(data=False))
        return TriggerResponse(True, "started")

    def _stop_exploration(self, _request):
        self.stop_called = True
        return TriggerResponse(True, "stopped")

    def _execute_goal(self, goal):
        self.move_base_goals.append(goal)
        rospy.sleep(0.05)
        self.move_base_server.set_succeeded(MoveBaseResult())

    def _publish_health(self, _event):
        now = rospy.Time.now()
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = now
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = 1.0
        pose.pose.pose.position.y = 2.0
        pose.pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        mapping = MappingStatus()
        mapping.header.stamp = now
        mapping.header.frame_id = "map"
        mapping.ready = True
        mapping.stable = True
        mapping.lost = False
        mapping.current_floor = 0
        self.mapping_pub.publish(mapping)

        navigation = NavigationHealth()
        navigation.header.stamp = now
        navigation.ready = True
        self.navigation_pub.publish(navigation)

        detection = DetectionStatus()
        detection.header.stamp = now
        detection.header.frame_id = "map"
        detection.ready = True
        detection.input_fresh = True
        self.detection_status_pub.publish(detection)

        self.exploration_status_pub.publish(String(data=json.dumps({
            "remaining_frontier_count": 0,
            "known_grid_ratio": 0.9,
        })))

    def _publish_detection(self, detection_id):
        now = rospy.Time.now()
        danger = DangerSource()
        danger.detection_id = detection_id
        danger.class_id = DangerSource.CLASS_DANGER_RED_SPHERE
        danger.position.header.stamp = now
        danger.position.header.frame_id = "map"
        danger.position.point.x = 2.0
        danger.position.point.y = 2.0
        danger.position.point.z = 0.15
        danger.floor_id = 0
        danger.confidence = 0.9
        array = DangerSourceArray()
        array.header.stamp = now
        array.header.frame_id = "map"
        array.dangers.append(danger)
        self.detections_pub.publish(array)

    @staticmethod
    def _wait_for(predicate, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                return True
            rospy.sleep(0.02)
        return False

    def test_complete_signal_returns_home_and_writes_result(self):
        rospy.wait_for_service("/danger_search/start", timeout=5.0)
        start = rospy.ServiceProxy("/danger_search/start", Trigger)
        response = start()
        self.assertTrue(response.success, response.message)
        self.assertTrue(self._wait_for(lambda: self.exploration_started, 2.0))
        self.assertTrue(self._wait_for(
            lambda: self.latest_status is not None
            and self.latest_status.mission_state == "EXPLORING",
            2.0,
        ))
        # Allow the latched false session marker to reach mission before true.
        rospy.sleep(0.1)

        self._publish_detection("frame-1")
        rospy.sleep(0.05)
        self._publish_detection("frame-2")
        rospy.sleep(0.05)
        self.complete_pub.publish(Bool(data=True))

        self.assertTrue(self._wait_for(
            lambda: self.latest_status is not None
            and self.latest_status.mission_state == "FINISHED",
            5.0,
        ))
        self.assertTrue(self.stop_called)
        self.assertEqual(len(self.move_base_goals), 10)
        entry_goals = self.move_base_goals[:-1]
        return_goal = self.move_base_goals[-1]
        self.assertAlmostEqual(entry_goals[0].target_pose.pose.position.x, 1.5)
        self.assertAlmostEqual(entry_goals[-1].target_pose.pose.position.x, 5.2)
        self.assertTrue(all(
            goal.target_pose.pose.position.y == 2.0 for goal in entry_goals
        ))
        self.assertAlmostEqual(return_goal.target_pose.pose.position.x, 1.0)
        self.assertAlmostEqual(return_goal.target_pose.pose.position.y, 2.0)
        self.assertTrue(os.path.isfile(RESULT_FILE))
        with open(RESULT_FILE, encoding="utf-8") as stream:
            result = json.load(stream)
        self.assertEqual(
            result["detected_danger_sources"],
            [{"position": [1.0, 0.0, 0.15]}],
        )
        self.assertGreaterEqual(result["exploration_time"], 0.0)


if __name__ == "__main__":
    rospy.init_node("mission_smoke_test")
    rostest.rosrun("danger_search_mission", "mission_smoke_test", MissionSmokeTest)
