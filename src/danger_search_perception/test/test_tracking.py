#!/usr/bin/env python3

import unittest

from danger_search_perception.config import TrackingConfig
from danger_search_perception.tracking import (
    DetectionObservation,
    MultiFrameDangerTracker,
)


def observation(detection_id, x, y, z, floor_id, stamp_s, confidence=0.9):
    return DetectionObservation(
        detection_id=detection_id,
        floor_id=floor_id,
        position=(x, y, z),
        confidence=confidence,
        stamp_s=stamp_s,
    )


class TestMultiFrameDangerTracker(unittest.TestCase):
    def setUp(self):
        self.tracker = MultiFrameDangerTracker(
            TrackingConfig(
                association_distance_m=0.40,
                confirmation_hits=3,
                tentative_timeout_s=1.0,
                confirmed_timeout_s=0.0,
            )
        )

    def test_nearby_observations_share_track_and_confirm(self):
        first = self.tracker.update([
            observation("frame-1", 1.00, 2.00, 0.15, 0, 0.0)
        ])[0]
        second = self.tracker.update([
            observation("frame-2", 1.06, 1.98, 0.16, 0, 0.1)
        ])[0]
        third = self.tracker.update([
            observation("frame-3", 0.97, 2.02, 0.14, 0, 0.2)
        ])[0]

        self.assertEqual(first.track_id, second.track_id)
        self.assertEqual(second.track_id, third.track_id)
        self.assertFalse(first.confirmed)
        self.assertFalse(second.confirmed)
        self.assertTrue(third.confirmed)
        self.assertEqual(third.hit_count, 3)
        self.assertAlmostEqual(third.position[0], 1.01, places=6)
        self.assertGreater(third.position_covariance[0], 0.0)
        self.assertEqual(third.position_covariance[1], 0.0)

    def test_same_xyz_on_different_floors_never_merges(self):
        floor_zero = self.tracker.update([
            observation("floor-0", 1.0, 1.0, 0.2, 0, 0.0)
        ])[0]
        floor_one = self.tracker.update([
            observation("floor-1", 1.0, 1.0, 2.8, 1, 0.1)
        ])[0]

        self.assertNotEqual(floor_zero.track_id, floor_one.track_id)
        self.assertTrue(floor_zero.track_id.startswith("danger-f0-"))
        self.assertTrue(floor_one.track_id.startswith("danger-f1-"))

    def test_far_observation_creates_new_track(self):
        first = self.tracker.update([
            observation("near", 0.0, 0.0, 0.0, 0, 0.0)
        ])[0]
        second = self.tracker.update([
            observation("far", 1.0, 0.0, 0.0, 0, 0.1)
        ])[0]
        self.assertNotEqual(first.track_id, second.track_id)

    def test_tentative_track_expires(self):
        first = self.tracker.update([
            observation("old", 0.0, 0.0, 0.0, 0, 0.0)
        ])[0]
        replacement = self.tracker.update([
            observation("new", 0.0, 0.0, 0.0, 0, 2.0)
        ])[0]
        self.assertNotEqual(first.track_id, replacement.track_id)

    def test_confirmed_track_is_retained_for_return_to_floor(self):
        tracker = MultiFrameDangerTracker(
            TrackingConfig(confirmation_hits=2, confirmed_timeout_s=0.0)
        )
        tracker.update([
            observation("a", 2.0, 0.0, 0.2, 0, 0.0)
        ])
        confirmed = tracker.update([
            observation("b", 2.02, 0.0, 0.2, 0, 0.1)
        ])[0]
        self.assertTrue(confirmed.confirmed)
        self.assertEqual(tracker.counts(1000.0), (1, 0))

        revisited = tracker.update([
            observation("c", 1.99, 0.0, 0.2, 0, 1000.0)
        ])[0]
        self.assertEqual(revisited.track_id, confirmed.track_id)

    def test_one_track_accepts_at_most_one_observation_per_frame(self):
        existing = self.tracker.update([
            observation("seed", 0.0, 0.0, 0.0, 0, 0.0)
        ])[0]
        assignments = self.tracker.update([
            observation("left", -0.02, 0.0, 0.0, 0, 0.1),
            observation("right", 0.03, 0.0, 0.0, 0, 0.1),
        ])
        matching = [
            item for item in assignments if item.track_id == existing.track_id
        ]
        self.assertEqual(len(matching), 1)
        self.assertNotEqual(assignments[0].track_id, assignments[1].track_id)


if __name__ == "__main__":
    unittest.main()
