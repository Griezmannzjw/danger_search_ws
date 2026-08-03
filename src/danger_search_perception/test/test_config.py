#!/usr/bin/env python3

import unittest

from danger_search_perception.config import (
    ColorDetectionConfig,
    GeometryConfig,
    PipelineConfig,
)


class TestPerceptionConfig(unittest.TestCase):
    def test_default_configs_are_valid(self):
        ColorDetectionConfig()
        GeometryConfig()
        PipelineConfig()

    def test_reversed_depth_range_is_rejected(self):
        with self.assertRaises(ValueError):
            GeometryConfig(min_depth_m=5.0, max_depth_m=1.0)

    def test_invalid_confidence_is_rejected(self):
        with self.assertRaises(ValueError):
            PipelineConfig(confidence_threshold=1.1)

    def test_reliable_range_is_applied(self):
        config = PipelineConfig(
            reliable_min_range=0.5, reliable_max_range=3.0
        )
        self.assertFalse(config.is_reliable_range((0.1, 0.0, 0.0)))
        self.assertTrue(config.is_reliable_range((1.0, 2.0, 0.0)))
        self.assertFalse(config.is_reliable_range((3.1, 0.0, 0.0)))


if __name__ == "__main__":
    unittest.main()
