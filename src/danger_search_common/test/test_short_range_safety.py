#!/usr/bin/env python3

import math
import unittest

import numpy as np

from danger_search_common.short_range_safety import (
    swept_arc_footprint_hit,
    swept_footprint_hit,
    swept_footprint_hits,
    swept_footprint_obstacle,
)


FOOTPRINT = (-0.35, 0.30, -0.15, 0.15)


def scan_with_points(*points):
    count = 1441
    angle_min = -math.pi
    increment = 2.0 * math.pi / float(count - 1)
    ranges = np.full(count, float("inf"), dtype=np.float64)
    for x, y in points:
        angle = math.atan2(y, x)
        index = int(round((angle - angle_min) / increment))
        ranges[index] = math.hypot(x, y)
    return ranges, angle_min, increment


def checked(*points, direction=1.0, travel=1.2, margin=0.08):
    ranges, angle_min, increment = scan_with_points(*points)
    return swept_footprint_obstacle(
        ranges, angle_min, increment, 0.05, 10.0, direction, travel,
        FOOTPRINT, margin,
    )


class SweptFootprintObstacleTest(unittest.TestCase):
    def test_forward_backward_and_margin_are_directional(self):
        self.assertIsNotNone(checked((0.8, 0.0)))
        self.assertIsNotNone(checked((0.8, 0.20)))
        self.assertIsNone(checked((-0.8, 0.0)))
        self.assertIsNotNone(checked((-0.8, 0.0), direction=-1.0))
        self.assertIsNone(checked((0.8, 0.35)))

    def test_body_return_is_ignored_but_swept_endpoint_is_not(self):
        self.assertIsNone(checked((0.0, 0.0)))
        self.assertIsNotNone(checked((1.50, 0.0)))

    def test_nearest_valid_swept_obstacle_is_reported(self):
        obstacle = checked((1.10, 0.0), (0.65, 0.05))
        self.assertAlmostEqual(obstacle, math.hypot(0.65, 0.05), places=6)

    def test_hit_reports_point_for_lateral_margin_avoidance(self):
        ranges, angle_min, increment = scan_with_points(
            (0.95, 0.185), (0.70, -0.19)
        )
        hit = swept_footprint_hit(
            ranges, angle_min, increment, 0.05, 10.0, 1.0, 1.2,
            FOOTPRINT, 0.05,
        )
        self.assertIsNotNone(hit)
        self.assertAlmostEqual(hit.distance_m, math.hypot(0.70, -0.19), places=2)
        self.assertAlmostEqual(hit.x_m, 0.70, places=2)
        self.assertAlmostEqual(hit.y_m, -0.19, places=2)
        self.assertIsNone(swept_footprint_hit(
            ranges, angle_min, increment, 0.05, 10.0, 1.0, 1.2,
            FOOTPRINT, 0.0,
        ))

    def test_all_hits_are_range_sorted_for_two_sided_margin_decision(self):
        ranges, angle_min, increment = scan_with_points(
            (0.75, 0.19), (0.60, -0.19), (1.0, 0.0)
        )
        hits = swept_footprint_hits(
            ranges, angle_min, increment, 0.05, 10.0, 1.0, 1.2,
            FOOTPRINT, 0.05,
        )
        self.assertEqual(len(hits), 3)
        self.assertLess(hits[0].distance_m, hits[1].distance_m)
        self.assertLess(hits[1].distance_m, hits[2].distance_m)

    def test_arc_sweep_catches_turning_inner_corner_missed_by_straight_path(self):
        # Left turn: a point is outside the straight physical half-width but
        # inside the left edge of the footprint after following the arc.
        ranges, angle_min, increment = scan_with_points((0.53, 0.18))
        straight = swept_footprint_hit(
            ranges, angle_min, increment, 0.05, 10.0, 1.0, 0.30,
            FOOTPRINT, 0.0,
        )
        curved = swept_arc_footprint_hit(
            ranges, angle_min, increment, 0.05, 10.0, 1.0, 0.30,
            0.32, FOOTPRINT, 0.0, 0.01,
        )
        self.assertIsNone(straight)
        self.assertIsNotNone(curved)

    def test_arc_sweep_matches_straight_at_zero_curvature(self):
        ranges, angle_min, increment = scan_with_points((0.50, 0.0))
        hit = swept_arc_footprint_hit(
            ranges, angle_min, increment, 0.05, 10.0, 1.0, 0.30,
            0.0, FOOTPRINT, 0.0, 0.01,
        )
        self.assertIsNotNone(hit)

    def test_invalid_and_nonfinite_inputs_are_rejected_or_ignored(self):
        ranges, angle_min, increment = scan_with_points((0.8, 0.0))
        ranges.fill(float("nan"))
        self.assertIsNone(swept_footprint_obstacle(
            ranges, angle_min, increment, 0.05, 10.0, 1.0, 1.2,
            FOOTPRINT, 0.08,
        ))
        ranges, angle_min, increment = scan_with_points((0.8, 0.0))
        with self.assertRaises(ValueError):
            swept_footprint_obstacle(
                ranges, angle_min, 0.0, 0.05, 10.0, 1.0, 1.2,
                FOOTPRINT, 0.08,
            )
        with self.assertRaises(ValueError):
            swept_footprint_obstacle(
                ranges, angle_min, increment, 0.05, 10.0, 0.0, 1.2,
                FOOTPRINT, 0.08,
            )


if __name__ == "__main__":
    unittest.main()
