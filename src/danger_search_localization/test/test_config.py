#!/usr/bin/env python3

import unittest

from danger_search_localization.config import (
    AdapterConfig,
    ScanProjectionConfig,
)


class TestLocalizationConfig(unittest.TestCase):
    def test_defaults_are_valid(self):
        ScanProjectionConfig()
        AdapterConfig()

    def test_invalid_height_range_is_rejected(self):
        with self.assertRaises(ValueError):
            ScanProjectionConfig(min_height=1.0, max_height=0.5)

    def test_pose_rate_below_interface_minimum_is_rejected(self):
        with self.assertRaises(ValueError):
            AdapterConfig(pose_publish_rate_hz=5.0)
