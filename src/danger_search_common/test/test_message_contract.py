#!/usr/bin/env python3
"""Static guards for the multi-floor ROS message wire contracts."""

import pathlib
import unittest


PACKAGE = pathlib.Path(__file__).parents[1]


def message_fields(name):
    lines = (PACKAGE / "msg" / name).read_text(encoding="utf-8").splitlines()
    return [
        line.split("#", 1)[0].strip() for line in lines
        if line.split("#", 1)[0].strip()
    ]


class MultiFloorMessageContractTest(unittest.TestCase):
    def test_active_map_envelope_has_an_atomic_floor_epoch_version_identity(self):
        self.assertEqual(message_fields("FloorOccupancyGrid.msg"), [
            "std_msgs/Header header",
            "int32 floor_id",
            "uint64 map_epoch",
            "uint64 map_version",
            "nav_msgs/OccupancyGrid occupancy_grid",
        ])

    def test_navigation_health_carries_the_same_map_identity_and_transition_gate(self):
        fields = message_fields("NavigationHealth.msg")
        for field in (
                "int32 current_floor",
                "uint64 map_epoch",
                "uint64 map_version",
                "bool transitioning"):
            self.assertIn(field, fields)


if __name__ == "__main__":
    unittest.main()
