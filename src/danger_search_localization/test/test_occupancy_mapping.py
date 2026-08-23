#!/usr/bin/env python3

import math
import struct
import threading
import unittest
from types import SimpleNamespace
from unittest import mock

import rospy

from danger_search_localization.occupancy_mapping import (
    OccupancyMapperCore,
    OccupancyMappingConfig,
)
from danger_search_localization.occupancy_mapper_node import OccupancyMapperNode


class TestOccupancyMapping(unittest.TestCase):
    def setUp(self):
        self.core = OccupancyMapperCore(
            OccupancyMappingConfig(
                resolution=0.1,
                size=100,
                start_x=0.5,
                start_y=0.5,
                clear_radius_m=0.2,
            )
        )

    @staticmethod
    def scan(ranges, angle_min=0.0, angle_increment=0.1):
        return SimpleNamespace(
            ranges=ranges,
            angle_min=angle_min,
            angle_increment=angle_increment,
            range_min=0.2,
            range_max=10.0,
        )

    def test_ray_marks_free_space_and_occupied_endpoint(self):
        self.assertTrue(self.core.update((0.0, 0.0, 0.0), self.scan([2.0])))
        data = self.core.occupancy_data()
        start = self.core.world_to_cell(0.0, 0.0)
        middle = self.core.world_to_cell(1.0, 0.0)
        endpoint = self.core.world_to_cell(2.0, 0.0)
        width = self.core.config.size
        self.assertEqual(data[start[1] * width + start[0]], 0)
        self.assertEqual(data[middle[1] * width + middle[0]], 0)
        self.assertEqual(data[endpoint[1] * width + endpoint[0]], 100)

    def test_pose_is_used_to_place_endpoint_in_map(self):
        self.core.update(
            (1.0, 1.0, math.pi / 2.0), self.scan([1.0])
        )
        endpoint = self.core.world_to_cell(1.0, 2.0)
        data = self.core.occupancy_data()
        self.assertEqual(
            data[endpoint[1] * self.core.config.size + endpoint[0]], 100
        )

    def test_empty_scan_does_not_create_map_update(self):
        self.assertFalse(self.core.update((0.0, 0.0, 0.0), self.scan([math.inf])))
        self.assertEqual(self.core.update_count, 0)

    def test_origin_cell_survives_occupancy_grid_float32_resolution(self):
        internal = self.core.world_to_cell(0.0, 0.0)
        wire_resolution = struct.unpack(
            "f", struct.pack("f", self.core.config.resolution)
        )[0]
        external = (
            int(math.floor((0.0 - self.core.origin_x) / wire_resolution)),
            int(math.floor((0.0 - self.core.origin_y) / wire_resolution)),
        )

        self.assertEqual(internal, external)
        self.assertEqual(internal, (50, 50))

    def test_reset_service_replaces_latched_map_with_unknown_grid(self):
        node = OccupancyMapperNode.__new__(OccupancyMapperNode)
        node.config = OccupancyMappingConfig(size=16)
        node.core = OccupancyMapperCore(node.config)
        node.core.scores[:, :] = node.config.max_score
        node.core.observed[:, :] = True
        node.lock = threading.RLock()
        node.pose_cache = {(1, 0): (0.0, 0.0, 0.0)}
        node.scan_cache = {(1, 0): object()}
        node.map_frame = "map"
        node.last_scan_stamp = rospy.Time(0)
        node.map_load_time = rospy.Time.from_sec(1.0)
        node.map_dirty = True
        node.publisher = mock.Mock()

        with mock.patch.object(
            rospy.Time, "now", return_value=rospy.Time.from_sec(2.0)
        ):
            node._reset_map_callback(None)

        message = node.publisher.publish.call_args.args[0]
        self.assertTrue(all(value == -1 for value in message.data))
        self.assertEqual(message.info.map_load_time, rospy.Time.from_sec(2.0))
        self.assertFalse(node.pose_cache)
        self.assertFalse(node.scan_cache)
