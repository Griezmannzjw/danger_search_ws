#!/usr/bin/env python3

import math
import os
import tempfile
import unittest

from danger_search_mission.mission_core import (
    build_result_document,
    DangerTrack,
    DangerTrackStore,
    entry_progress,
    MissionLifecycle,
    next_entry_target,
    normalize_result_file,
    task_relative_position,
)


class MissionLifecycleTest(unittest.TestCase):
    def test_happy_path_requires_return(self):
        lifecycle = MissionLifecycle()
        self.assertTrue(lifecycle.start())
        self.assertEqual(lifecycle.state, MissionLifecycle.ENTERING)
        self.assertTrue(lifecycle.begin_exploration())
        self.assertEqual(lifecycle.state, MissionLifecycle.EXPLORING)
        self.assertTrue(lifecycle.begin_return())
        self.assertEqual(lifecycle.state, MissionLifecycle.RETURNING)
        self.assertTrue(lifecycle.finish())
        self.assertEqual(lifecycle.state, MissionLifecycle.FINISHED)

    def test_invalid_transitions_are_rejected(self):
        lifecycle = MissionLifecycle()
        self.assertFalse(lifecycle.begin_return())
        self.assertFalse(lifecycle.finish())
        self.assertTrue(lifecycle.start())
        self.assertFalse(lifecycle.start())
        self.assertTrue(lifecycle.begin_exploration())
        self.assertFalse(lifecycle.begin_exploration())

    def test_failure_can_override_a_late_write_failure(self):
        lifecycle = MissionLifecycle()
        lifecycle.start()
        lifecycle.begin_exploration()
        lifecycle.begin_return()
        lifecycle.finish()
        lifecycle.fail()
        self.assertEqual(lifecycle.state, MissionLifecycle.ERROR)


class DangerTrackStoreTest(unittest.TestCase):
    def setUp(self):
        self.store = DangerTrackStore(0.8, 3, 0.6)

    def test_requires_multiple_unique_frames(self):
        first = self.store.add("a", 1.0, 2.0, 0.1, 0, 0.9)
        self.store.add("b", 1.1, 2.0, 0.1, 0, 0.8)
        self.assertEqual(first.count, 2)
        self.assertEqual(self.store.confirmed_tracks(), [])
        self.store.add("c", 0.9, 2.0, 0.1, 0, 0.7)
        self.assertEqual(len(self.store.confirmed_tracks()), 1)
        self.assertAlmostEqual(self.store.confirmed_tracks()[0].x, 1.0)

    def test_duplicate_detection_id_is_ignored(self):
        track = self.store.add("same", 1.0, 0.0, 0.0, 0, 0.9)
        self.assertIsNone(self.store.add("same", 1.1, 0.0, 0.0, 0, 0.9))
        self.assertEqual(track.count, 1)

    def test_weak_or_nonfinite_observations_are_rejected(self):
        self.assertIsNone(self.store.add("weak", 1, 2, 3, 0, 0.59))
        self.assertIsNone(self.store.add("nan", math.nan, 2, 3, 0, 0.9))
        self.assertEqual(self.store.tracks, [])

    def test_different_floors_do_not_merge(self):
        self.store.add("floor0", 1, 2, 0.1, 0, 0.9)
        self.store.add("floor1", 1, 2, 0.1, 1, 0.9)
        self.assertEqual(len(self.store.tracks), 2)


class ResultContractTest(unittest.TestCase):
    def test_task_frame_subtracts_and_rotates_home(self):
        result = task_relative_position(
            10.0, 7.0, 0.2,
            10.0, 5.0, 0.0,
            math.pi / 2.0,
        )
        self.assertAlmostEqual(result[0], 2.0)
        self.assertAlmostEqual(result[1], 0.0)
        self.assertAlmostEqual(result[2], 0.2)

    def test_evaluator_document_has_exact_required_shape(self):
        track = DangerTrack(2.345, -1.234, 0.156, 0, count=3, max_confidence=0.9)
        result = build_result_document([track], (0.0, 0.0, 0.0, 0.0), 12.345)
        self.assertEqual(result["exploration_time"], 12.35)
        self.assertEqual(
            result["detected_danger_sources"],
            [{"position": [2.35, -1.23, 0.16]}],
        )

    def test_result_path_expands_and_normalizes(self):
        with tempfile.TemporaryDirectory() as directory:
            raw = os.path.join(directory, "sub", "..", "detected_danger.json")
            normalized = normalize_result_file(raw)
            self.assertTrue(os.path.isabs(normalized))
            self.assertEqual(os.path.basename(normalized), "detected_danger.json")

    def test_wrong_result_filename_is_rejected(self):
        with self.assertRaises(ValueError):
            normalize_result_file("/tmp/result.json")

    def test_relative_result_path_is_rejected(self):
        with self.assertRaises(ValueError):
            normalize_result_file("results/detected_danger.json")


class EntryStrategyTest(unittest.TestCase):
    def test_progress_uses_captured_heading(self):
        forward, lateral = entry_progress(
            current_x=9.0,
            current_y=7.0,
            home_x=10.0,
            home_y=5.0,
            home_yaw=math.pi / 2.0,
        )
        self.assertAlmostEqual(forward, 2.0)
        self.assertAlmostEqual(lateral, 1.0)

    def test_next_target_advances_in_bounded_segments(self):
        target = next_entry_target(
            home_x=1.0,
            home_y=2.0,
            home_yaw=0.0,
            current_progress=1.1,
            distance=4.2,
            step=0.6,
        )
        self.assertAlmostEqual(target[0], 2.7)
        self.assertAlmostEqual(target[1], 2.0)
        self.assertAlmostEqual(target[2], 1.7)

    def test_final_segment_is_clamped_to_entry_distance(self):
        target = next_entry_target(1.0, 2.0, 0.0, 4.0, 4.2, 0.6)
        self.assertAlmostEqual(target[0], 5.2)
        self.assertAlmostEqual(target[2], 4.2)


if __name__ == "__main__":
    unittest.main()
