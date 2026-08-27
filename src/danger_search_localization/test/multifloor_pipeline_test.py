#!/usr/bin/env python3

import copy
import math
import threading
import time
import unittest

import rospy
import rostest
from danger_search_common.msg import MappingStatus
from danger_search_common.srv import SwitchFloor
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32


class MultiFloorPipelineTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.RLock()
        self.current_floor = None
        self.mapping_status = None
        self.current_map = None
        self.floor_maps = {}
        self.raw_pose_pub = rospy.Publisher(
            "/localization/raw_pose",
            PoseWithCovarianceStamped,
            queue_size=20,
        )
        self.mapping_scan_pub = rospy.Publisher(
            "/localization/mapping_scan", LaserScan, queue_size=20
        )
        self.switch_floor = rospy.ServiceProxy(
            "/localization/switch_floor", SwitchFloor
        )
        self.subscribers = [
            rospy.Subscriber(
                "/mapping/current_floor", Int32, self._floor_callback
            ),
            rospy.Subscriber(
                "/mapping/status", MappingStatus, self._status_callback
            ),
            rospy.Subscriber("/map", OccupancyGrid, self._map_callback),
            rospy.Subscriber(
                "/mapping/floors/0/map",
                OccupancyGrid,
                self._archive_callback,
                callback_args=0,
            ),
            rospy.Subscriber(
                "/mapping/floors/1/map",
                OccupancyGrid,
                self._archive_callback,
                callback_args=1,
            ),
            rospy.Subscriber(
                "/mapping/floors/2/map",
                OccupancyGrid,
                self._archive_callback,
                callback_args=2,
            ),
        ]

    def _floor_callback(self, message):
        with self.lock:
            self.current_floor = int(message.data)

    def _status_callback(self, message):
        with self.lock:
            self.mapping_status = copy.deepcopy(message)

    def _map_callback(self, message):
        with self.lock:
            self.current_map = copy.deepcopy(message)

    def _archive_callback(self, message, floor_id):
        with self.lock:
            self.floor_maps[int(floor_id)] = copy.deepcopy(message)

    @staticmethod
    def _wait_for(predicate, timeout_s=8.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                return True
            rospy.sleep(0.02)
        return False

    def _wait_for_connections(self):
        return self._wait_for(
            lambda: self.raw_pose_pub.get_num_connections() >= 1
            and self.mapping_scan_pub.get_num_connections() >= 1,
            timeout_s=5.0,
        )

    def _publish_sample(self, z, angle):
        stamp = rospy.Time.now()
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "odom"
        pose.pose.pose.position.z = float(z)
        pose.pose.pose.orientation.w = 1.0
        for index in (0, 7, 14, 21, 28, 35):
            pose.pose.covariance[index] = 1e-4

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "base"
        scan.angle_min = float(angle)
        scan.angle_max = float(angle + 0.2)
        scan.angle_increment = 0.1
        scan.range_min = 0.2
        scan.range_max = 10.0
        scan.ranges = [1.5, 1.6, 1.7]

        self.raw_pose_pub.publish(pose)
        self.mapping_scan_pub.publish(scan)
        rospy.sleep(0.06)

    def _publish_floor_samples(self, z, angle, count=4):
        for _ in range(count):
            self._publish_sample(z, angle)

    def _status_is(self, floor_id, stable):
        with self.lock:
            return (
                self.mapping_status is not None
                and self.mapping_status.current_floor == floor_id
                and self.mapping_status.ready == stable
                and self.mapping_status.stable == stable
            )

    def _floor_versions(self):
        with self.lock:
            if self.mapping_status is None:
                return {}
            return {
                item.floor_id: int(item.map_version)
                for item in self.mapping_status.floor_maps
            }

    def _wait_for_stable_floor_version(
        self, floor_id, timeout_s=5.0, quiet_s=0.35
    ):
        deadline = time.monotonic() + timeout_s
        last_version = None
        unchanged_since = time.monotonic()
        while time.monotonic() < deadline and not rospy.is_shutdown():
            version = self._floor_versions().get(floor_id)
            now = time.monotonic()
            if version is None or version <= 0:
                last_version = None
                unchanged_since = now
            elif version != last_version:
                last_version = version
                unchanged_since = now
            elif now - unchanged_since >= quiet_s:
                return version
            rospy.sleep(0.02)
        return None

    def test_switch_and_return_preserve_independent_maps(self):
        self.assertTrue(self._wait_for_connections())
        rospy.wait_for_service("/localization/switch_floor", timeout=5.0)

        self._publish_floor_samples(0.0, 0.0)
        self.assertTrue(self._wait_for(lambda: self._status_is(0, True)))
        with self.lock:
            self.assertEqual(self.mapping_status.map_epoch, 1)
            self.assertFalse(self.mapping_status.transitioning)
            self.assertAlmostEqual(self.mapping_status.floor_z_m, 0.0)
        self.assertTrue(self._wait_for(lambda: 0 in self.floor_maps))
        floor_zero_version = self._wait_for_stable_floor_version(0)
        self.assertIsNotNone(floor_zero_version)
        with self.lock:
            floor_zero_before = copy.deepcopy(self.floor_maps[0])
        versions_before = self._floor_versions()
        self.assertEqual(versions_before.get(0), floor_zero_version)
        self.assertGreaterEqual(versions_before.get(0, 0), 2)

        # A sample halfway between configured landings must not enter either
        # floor map, and the public mapping state must become unstable.
        self._publish_sample(1.3, math.pi / 4.0)
        self.assertTrue(self._wait_for(
            lambda: self.mapping_status is not None
            and not self.mapping_status.stable
            and self.mapping_status.transitioning
            and self.mapping_status.status_reason
            == "FLOOR_TRANSITION_WAITING_FOR_CURRENT_MAP"
        ))

        switch_one = self.switch_floor(
            transition_id="elevator-run-1", target_floor=1
        )
        switch_one_replay = self.switch_floor(
            transition_id="elevator-run-1", target_floor=1
        )
        self.assertTrue(switch_one.success)
        self.assertEqual(switch_one.map_epoch, 2)
        self.assertEqual(switch_one_replay.map_epoch, 2)

        self._publish_floor_samples(2.6, math.pi / 2.0)
        self.assertTrue(self._wait_for(lambda: self._status_is(1, True)))
        with self.lock:
            self.assertEqual(self.mapping_status.map_epoch, 2)
            self.assertAlmostEqual(self.mapping_status.floor_z_m, 2.6)
        self.assertTrue(self._wait_for(lambda: 1 in self.floor_maps))
        floor_one_version = self._wait_for_stable_floor_version(1)
        self.assertIsNotNone(floor_one_version)
        versions_floor_one = self._floor_versions()
        self.assertEqual(versions_floor_one.get(1), floor_one_version)
        self.assertIn(0, versions_floor_one)
        self.assertIn(1, versions_floor_one)
        with self.lock:
            floor_one = copy.deepcopy(self.floor_maps[1])
            current_on_one = copy.deepcopy(self.current_map)
        self.assertEqual(current_on_one.data, floor_one.data)
        self.assertNotEqual(floor_zero_before.data, floor_one.data)

        switch_two = self.switch_floor(
            transition_id="elevator-run-2", target_floor=2
        )
        self.assertTrue(switch_two.success)
        self.assertEqual(switch_two.map_epoch, 3)
        self._publish_floor_samples(5.2, math.pi)
        self.assertTrue(self._wait_for(lambda: self._status_is(2, True)))
        with self.lock:
            self.assertEqual(self.mapping_status.map_epoch, 3)
            self.assertAlmostEqual(self.mapping_status.floor_z_m, 5.2)
        self.assertTrue(self._wait_for(lambda: 2 in self.floor_maps))
        floor_two_version = self._wait_for_stable_floor_version(2)
        self.assertIsNotNone(floor_two_version)
        versions_floor_two = self._floor_versions()
        self.assertEqual(versions_floor_two.get(2), floor_two_version)
        self.assertEqual(versions_floor_two.get(0), versions_floor_one.get(0))
        self.assertEqual(versions_floor_two.get(1), versions_floor_one.get(1))
        with self.lock:
            floor_two = copy.deepcopy(self.floor_maps[2])
            current_on_two = copy.deepcopy(self.current_map)
        self.assertEqual(current_on_two.data, floor_two.data)
        self.assertNotEqual(floor_one.data, floor_two.data)

        switch_home = self.switch_floor(
            transition_id="elevator-run-3", target_floor=0
        )
        self.assertTrue(switch_home.success)
        self.assertEqual(switch_home.map_epoch, 4)
        self._publish_floor_samples(0.0, 0.0)
        self.assertTrue(self._wait_for(lambda: self._status_is(0, True)))
        with self.lock:
            self.assertEqual(self.mapping_status.map_epoch, 4)
        returned_floor_zero_version = self._wait_for_stable_floor_version(0)
        self.assertIsNotNone(returned_floor_zero_version)
        versions_after_return = self._floor_versions()
        self.assertEqual(
            versions_after_return.get(0), returned_floor_zero_version
        )
        self.assertGreater(
            versions_after_return[0], versions_before[0]
        )
        self.assertEqual(
            versions_after_return[1], versions_floor_one[1]
        )
        self.assertEqual(
            versions_after_return[2], versions_floor_two[2]
        )
        with self.lock:
            floor_zero_after = copy.deepcopy(self.floor_maps[0])
            current_after_return = copy.deepcopy(self.current_map)
        self.assertEqual(
            floor_zero_after.info.map_load_time,
            floor_zero_before.info.map_load_time,
        )
        self.assertEqual(current_after_return.data, floor_zero_after.data)


if __name__ == "__main__":
    rospy.init_node("multifloor_pipeline_test")
    rostest.rosrun(
        "danger_search_localization",
        "multifloor_pipeline_test",
        MultiFloorPipelineTest,
    )
