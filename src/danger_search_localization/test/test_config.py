#!/usr/bin/env python3

import unittest
from pathlib import Path
from xml.etree import ElementTree

import yaml

from danger_search_localization.config import (
    AdapterConfig,
    ScanProjectionConfig,
)


class TestLocalizationConfig(unittest.TestCase):
    package_dir = Path(__file__).resolve().parents[1]

    def test_defaults_are_valid(self):
        scan_config = ScanProjectionConfig()
        self.assertEqual(scan_config.scan_accumulation_frames, 3)
        self.assertEqual(scan_config.min_valid_scan_bins, 8)
        self.assertEqual(scan_config.min_angular_coverage_rad, 0.05)
        AdapterConfig()

    def test_gicp_rebaseline_threshold_is_positive(self):
        config_path = self.package_dir / "config" / "default.yaml"
        config = yaml.safe_load(config_path.read_text())

        self.assertGreaterEqual(config["lidar_odom_rebaseline_after_failures"], 1)
        self.assertGreater(config["lidar_odom_max_reference_age_s"], 0.5)
        self.assertEqual(config["lidar_odom_submap_scans"], 5)
        self.assertEqual(config["lidar_odom_submap_max_points"], 1200)
        self.assertEqual(config["gicp_recovery_consecutive_accepts"], 2)
        self.assertAlmostEqual(config["lidar_odom_min_correspondence_ratio"], 0.35)

    def test_gicp_health_timeouts_are_ordered(self):
        with self.assertRaises(ValueError):
            AdapterConfig(
                gicp_healthy_fresh_timeout_s=2.0,
                gicp_healthy_lost_timeout_s=1.0,
            )

        config = AdapterConfig()
        self.assertLess(
            config.gicp_healthy_fresh_timeout_s,
            config.gicp_healthy_lost_timeout_s,
        )
        self.assertEqual(config.fusion_max_pose_pair_age_s, 0.80)
        self.assertEqual(config.pose_fresh_timeout_s, 2.0)
        self.assertEqual(config.tf_publish_future_tolerance_s, 0.5)

    def test_invalid_height_range_is_rejected(self):
        with self.assertRaises(ValueError):
            ScanProjectionConfig(min_height=1.0, max_height=0.5)

    def test_invalid_scan_accumulation_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            ScanProjectionConfig(scan_accumulation_frames=0)
        with self.assertRaises(ValueError):
            ScanProjectionConfig(
                scan_accumulation_frames=2,
                scan_accumulation_min_samples_per_bin=3,
            )

    def test_pose_rate_below_interface_minimum_is_rejected(self):
        with self.assertRaises(ValueError):
            AdapterConfig(pose_publish_rate_hz=5.0)

    def test_default_launch_uses_lio_without_legacy_backends(self):
        config_path = self.package_dir / "config" / "default.yaml"
        config = yaml.safe_load(config_path.read_text())

        self.assertEqual(config["common"]["lid_topic"], "/localization/lio/points")
        self.assertEqual(config["common"]["imu_topic"], "/trunk_imu")
        self.assertEqual(config["preprocess"]["lidar_type"], 4)
        self.assertFalse(config["mapping"]["extrinsic_est_en"])

        launch_path = self.package_dir / "launch" / "localization.launch"
        root = ElementTree.parse(launch_path).getroot()
        names = {node.attrib["name"] for node in root.findall("node")}
        self.assertIn("fast_lio_mapping", names)
        self.assertIn("sim_sensor_adapter", names)
        self.assertIn("lio_occupancy_mapper", names)
        self.assertIn("localization_adapter", names)
        self.assertNotIn("hector_mapping", names)
        self.assertNotIn("lidar_odometry", names)
        lio_node = next(node for node in root.findall("node")
                        if node.attrib["name"] == "fast_lio_mapping")
        remaps = {
            remap.attrib["from"]: remap.attrib["to"]
            for remap in lio_node.findall("remap")
        }
        self.assertEqual(remaps["/Odometry"], "/localization/lio/odometry")
