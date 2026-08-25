#!/usr/bin/env python3

import os
import sys
import unittest

SCRIPT_DIR = os.path.join(os.path.dirname(__file__), "..", "scripts")
sys.path.insert(0, os.path.abspath(SCRIPT_DIR))

from navigation_monitor_core import (
    classify_terminal_status,
    polyline_progress,
    recovery_has_translation_progress,
    recovery_maneuver,
)


class NavigationMonitorCoreTest(unittest.TestCase):
    def test_terminal_status_mapping_uses_move_base_failure_text(self):
        self.assertEqual(classify_terminal_status(3, "Goal reached"), "SUCCEEDED")
        self.assertEqual(classify_terminal_status(2, "cancelled"), "CANCELED")
        self.assertEqual(
            classify_terminal_status(4, "Failed to find a valid plan"),
            "UNREACHABLE",
        )
        self.assertEqual(
            classify_terminal_status(4, "Failed to find a valid control"),
            "CONTROL_FAILED",
        )

    def test_polyline_progress_projects_continuously(self):
        path = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
        self.assertAlmostEqual(polyline_progress(path, (0.5, 0.2)), 0.25)
        self.assertAlmostEqual(polyline_progress(path, (1.1, 0.5)), 0.75)

    def test_only_rotate_recovery_maps_to_rotate_maneuver(self):
        self.assertEqual(recovery_maneuver("rotate_recovery"), "ROTATE")
        self.assertEqual(recovery_maneuver("conservative_reset"), "NONE")

    def test_recovery_requires_new_plan_translation_and_distance(self):
        self.assertFalse(recovery_has_translation_progress(0.20, 0.10, 4, 4, True))
        self.assertFalse(recovery_has_translation_progress(0.20, 0.10, 5, 4, False))
        self.assertFalse(recovery_has_translation_progress(0.05, 0.10, 5, 4, True))
        self.assertTrue(recovery_has_translation_progress(0.10, 0.10, 5, 4, True))


if __name__ == "__main__":
    unittest.main()
