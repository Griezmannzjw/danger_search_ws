#!/usr/bin/env python3

import unittest

from danger_search_perception.floor_context import (
    FloorHeightClassifier,
    LocalizationCorrectionGate,
    MappingGate,
)


class TestMappingGate(unittest.TestCase):
    def setUp(self):
        self.gate = MappingGate(status_timeout_s=1.5, fallback_floor_id=0)

    def test_requires_fresh_stable_mapping(self):
        self.assertEqual(
            self.gate.snapshot(1.0).reason,
            "WAITING_FOR_MAPPING_STATUS",
        )

        self.gate.update(True, False, False, 0, received_s=1.0)
        self.assertEqual(self.gate.snapshot(1.1).reason, "MAPPING_UNSTABLE")

        self.gate.update(True, True, False, 0, received_s=1.2)
        snapshot = self.gate.snapshot(1.3)
        self.assertTrue(snapshot.allowed)
        self.assertEqual(snapshot.floor_id, 0)
        self.assertEqual(self.gate.snapshot(3.0).reason, "MAPPING_STATUS_STALE")

    def test_lost_and_invalid_floor_are_blocked(self):
        self.gate.update(True, True, True, 0, received_s=2.0)
        self.assertEqual(self.gate.snapshot(2.1).reason, "MAPPING_LOST")

        self.gate.update(True, True, False, -1, received_s=2.2)
        self.assertEqual(
            self.gate.snapshot(2.3).reason, "MAPPING_FLOOR_INVALID"
        )

    def test_semantic_change_invalidates_sensor_snapshot(self):
        self.gate.update(True, True, False, 0, received_s=1.0)
        floor_zero = self.gate.snapshot(1.1)
        self.assertTrue(self.gate.is_current(floor_zero, 1.2))

        self.gate.update(True, False, False, 1, received_s=1.3)
        self.assertFalse(self.gate.is_current(floor_zero, 1.4))

    def test_disabled_gate_uses_configured_fallback(self):
        snapshot = self.gate.snapshot(1.0, required=False)
        self.assertTrue(snapshot.allowed)
        self.assertEqual(snapshot.floor_id, 0)

    def test_external_map_epoch_and_transition_are_authoritative(self):
        self.gate.update(
            True, False, False, 1, 2.0, transitioning=True, map_epoch=7
        )
        snapshot = self.gate.snapshot(2.1)
        self.assertEqual(snapshot.epoch, 7)
        self.assertEqual(snapshot.reason, "MAPPING_TRANSITIONING")
        self.gate.update(
            True, True, False, 1, 2.2, transitioning=False, map_epoch=7
        )
        self.assertTrue(self.gate.snapshot(2.3).allowed)

    def test_new_map_epoch_invalidates_same_floor_sensor_snapshot(self):
        self.gate.update(
            True, True, False, 0, received_s=1.0, map_epoch=10
        )
        captured = self.gate.snapshot(1.1)
        self.assertTrue(captured.allowed)

        # A reset/load can keep every boolean and the floor id unchanged;
        # map_epoch is still a new coordinate context.
        self.gate.update(
            True, True, False, 0, received_s=1.2, map_epoch=11
        )
        self.assertFalse(self.gate.is_current(captured, 1.3))


class TestFloorHeightClassifier(unittest.TestCase):
    def setUp(self):
        self.classifier = FloorHeightClassifier(
            [0.0, 2.6, 5.2], tolerance_m=0.45
        )

    def test_classifies_landings_and_rejects_elevator_transit(self):
        self.assertEqual(self.classifier.classify(0.05), 0)
        self.assertEqual(self.classifier.classify(2.62), 1)
        self.assertEqual(self.classifier.classify(5.15), 2)
        self.assertIsNone(self.classifier.classify(1.30))

    def test_floor_match_is_explicit(self):
        self.assertTrue(self.classifier.matches(1, 2.60))
        self.assertFalse(self.classifier.matches(0, 2.60))

    def test_overlapping_floor_tolerance_is_rejected(self):
        with self.assertRaises(ValueError):
            FloorHeightClassifier([0.0, 1.0], tolerance_m=0.5)


class TestLocalizationCorrectionGate(unittest.TestCase):
    def test_requires_fresh_status_and_rejects_mid_frame_version_change(self):
        gate = LocalizationCorrectionGate(status_timeout_s=1.0)
        self.assertEqual(
            gate.snapshot(1.0).reason, "WAITING_FOR_LOCALIZATION_STATUS"
        )
        self.assertFalse(gate.update(4, 1.0))
        captured = gate.snapshot(1.1)
        self.assertTrue(captured.allowed)
        self.assertEqual(captured.version, 4)
        self.assertTrue(gate.is_current(captured, 1.2))
        self.assertTrue(gate.update(5, 1.3))
        self.assertFalse(gate.is_current(captured, 1.4))
        self.assertEqual(gate.snapshot(2.4).reason, "LOCALIZATION_STATUS_STALE")

    def test_optional_status_uses_zero_version(self):
        snapshot = LocalizationCorrectionGate(1.0).snapshot(
            1.0, required=False
        )
        self.assertTrue(snapshot.allowed)
        self.assertEqual(snapshot.version, 0)


if __name__ == "__main__":
    unittest.main()
