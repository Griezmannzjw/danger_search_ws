#!/usr/bin/env python3

import unittest
from types import SimpleNamespace

from danger_search_localization.floor_mapping import (
    FloorHeightClassifier,
    FloorSwitchState,
    MultiFloorOccupancyStore,
)
from danger_search_localization.occupancy_mapping import OccupancyMappingConfig


class TestFloorHeightClassifier(unittest.TestCase):
    def setUp(self):
        self.classifier = FloorHeightClassifier(
            [0.0, 2.6, 5.2], assignment_tolerance_m=0.45
        )

    def test_assigns_only_near_configured_landings(self):
        self.assertEqual(self.classifier.classify(-0.25).floor_id, 0)
        self.assertEqual(self.classifier.classify(2.35).floor_id, 1)
        self.assertEqual(self.classifier.classify(5.45).floor_id, 2)

        self.assertIsNone(self.classifier.classify(1.3))
        self.assertIsNone(self.classifier.classify(3.9))

    def test_invalid_floor_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            FloorHeightClassifier([])
        with self.assertRaises(ValueError):
            FloorHeightClassifier([0.0, 0.0])
        with self.assertRaises(ValueError):
            FloorHeightClassifier([0.0, 2.6], assignment_tolerance_m=0.0)
        with self.assertRaises(ValueError):
            FloorHeightClassifier([0.0, 1.0], assignment_tolerance_m=0.5)


class TestMultiFloorOccupancyStore(unittest.TestCase):
    @staticmethod
    def scan(distance, angle=0.0):
        return SimpleNamespace(
            ranges=[distance],
            angle_min=angle,
            angle_increment=0.1,
            range_min=0.2,
            range_max=10.0,
        )

    def setUp(self):
        config = OccupancyMappingConfig(
            resolution=0.1,
            size=100,
            clear_radius_m=0.2,
        )
        self.store = MultiFloorOccupancyStore(
            config, valid_floor_ids=range(3), initial_floor=0
        )

    def test_floor_maps_are_independent_and_restored(self):
        floor_zero = self.store.core(0)
        self.assertTrue(
            self.store.update(0, (0.0, 0.0, 0.0), self.scan(2.0))
        )
        floor_zero_snapshot = floor_zero.occupancy_data()

        self.assertTrue(
            self.store.update(1, (0.0, 0.0, 0.0), self.scan(1.5, 1.57))
        )

        self.assertIs(self.store.core(0), floor_zero)
        self.assertEqual(self.store.core(0).occupancy_data(), floor_zero_snapshot)
        self.assertNotEqual(
            self.store.core(1).occupancy_data(), floor_zero_snapshot
        )
        self.assertEqual(self.store.version(0), 1)
        self.assertEqual(self.store.version(1), 1)

        self.store.update(0, (0.1, 0.0, 0.0), self.scan(1.8))
        self.assertEqual(self.store.version(0), 2)
        self.assertEqual(self.store.version(1), 1)

    def test_reset_clears_every_visited_floor(self):
        self.store.update(0, (0.0, 0.0, 0.0), self.scan(1.0))
        self.store.update(2, (0.0, 0.0, 0.0), self.scan(1.0))

        self.store.reset_all()

        self.assertEqual(self.store.floor_ids, (0, 2))
        self.assertEqual(self.store.version(0), 0)
        self.assertEqual(self.store.version(2), 0)
        self.assertTrue(
            all(value == -1 for value in self.store.core(2).occupancy_data())
        )


class TestFloorSwitchState(unittest.TestCase):
    def setUp(self):
        self.state = FloorSwitchState([0.0, 2.6, 5.2], initial_floor=0)

    def test_switch_is_idempotent_by_transition_id(self):
        first = self.state.request("elevator-run-1", 1)
        replay = self.state.request("elevator-run-1", 1)

        self.assertTrue(first.success)
        self.assertTrue(first.changed)
        self.assertEqual(first.map_epoch, 2)
        self.assertEqual(replay, first)
        self.assertEqual(self.state.map_epoch, 2)
        self.assertEqual(self.state.floor_z_m, 2.6)

    def test_transition_id_cannot_be_reused_for_another_floor(self):
        self.state.request("elevator-run-1", 1)

        conflict = self.state.request("elevator-run-1", 2)

        self.assertFalse(conflict.success)
        self.assertEqual(conflict.current_floor, 1)
        self.assertEqual(conflict.map_epoch, 2)

    def test_same_floor_is_success_without_new_epoch(self):
        decision = self.state.request("already-home", 0)

        self.assertTrue(decision.success)
        self.assertFalse(decision.changed)
        self.assertEqual(decision.map_epoch, 1)
        self.assertFalse(self.state.transitioning)

    def test_map_reset_invalidates_transition_replay_epoch(self):
        self.state.request("elevator-run-1", 1)
        self.state.mark_stable()
        self.state.reset_map()

        after_reset = self.state.request("elevator-run-1", 1)

        self.assertTrue(after_reset.success)
        self.assertFalse(after_reset.changed)
        self.assertEqual(after_reset.map_epoch, 3)


if __name__ == "__main__":
    unittest.main()
