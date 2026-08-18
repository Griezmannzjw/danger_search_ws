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
        self.assertEqual(scan_config.min_returns_per_bin, 1)
        self.assertFalse(scan_config.enable_isolated_hit_filter)
        self.assertFalse(scan_config.enable_ground_clearance_gate)
        self.assertEqual(scan_config.min_angular_coverage_rad, 0.03)
        AdapterConfig()

    def test_gicp_rebaseline_threshold_is_positive(self):
        config_path = self.package_dir / "config" / "default.yaml"
        config = yaml.safe_load(config_path.read_text())

        self.assertGreaterEqual(config["lidar_odom_rebaseline_after_failures"], 1)
        self.assertGreater(config["lidar_odom_max_reference_age_s"], 0.5)
        self.assertEqual(config["lidar_odom_submap_scans"], 5)
        self.assertEqual(config["lidar_odom_submap_max_points"], 1200)
        self.assertEqual(config["lidar_odom_registration_max_points"], 100)
        self.assertEqual(config["lidar_odom_observation_scans"], 1)
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

    def test_default_pose_wiring_separates_hector_and_gicp(self):
        config_path = self.package_dir / "config" / "default.yaml"
        config = yaml.safe_load(config_path.read_text())

        self.assertEqual(config["backend_pose_topic"], "/localization/hector_pose")
        self.assertEqual(config["gicp_pose_topic"], "/localization/raw_pose")
        self.assertEqual(
            config["validated_gicp_pose_topic"],
            "/localization/validated_pose",
        )
        self.assertNotEqual(
            config["backend_pose_topic"], config["gicp_pose_topic"]
        )
        self.assertNotEqual(
            config["gicp_pose_topic"], config["validated_gicp_pose_topic"]
        )

        launch_path = self.package_dir / "launch" / "localization.launch"
        root = ElementTree.parse(launch_path).getroot()
        nodes = root.findall(".//node")
        hector_node = next(
            node for node in nodes if node.attrib["name"] == "hector_mapping"
        )
        remaps = {
            remap.attrib["from"]: remap.attrib["to"]
            for remap in hector_node.findall("remap")
        }
        self.assertEqual(remaps["poseupdate"], config["backend_pose_topic"])
        parameters = {
            parameter.attrib["name"]: parameter.attrib["value"]
            for parameter in hector_node.findall("param")
        }
        self.assertEqual(parameters["use_tf_pose_start_estimate"], "false")
        mapper_node = next(
            node for node in nodes if node.attrib["name"] == "local_occupancy_mapper"
        )
        self.assertEqual(mapper_node.attrib["type"], "occupancy_mapper.py")
        self.assertFalse(config["use_hector_correction"])

    def test_gazebo_truth_source_is_explicit_and_exclusive(self):
        launch_path = self.package_dir / "launch" / "localization.launch"
        root = ElementTree.parse(launch_path).getroot()
        arguments = {
            argument.attrib["name"]: argument.attrib.get("default")
            for argument in root.findall("arg")
        }
        self.assertEqual(arguments["localization_source"], "gicp")
        self.assertEqual(arguments["gazebo_base_link"], "a1_gazebo::base")

        source = launch_path.read_text()
        self.assertIn("localization_source') == 'gicp'", source)
        self.assertIn("localization_source') == 'gazebo_truth'", source)
        self.assertIn("tf_publish_future_tolerance_s", source)
        self.assertIn("localization_source') == 'gazebo_truth' else 0.5", source)
        self.assertEqual(source.count('name="lidar_odometry"'), 1)
        self.assertEqual(source.count('name="gazebo_truth_odometry"'), 1)

        truth_script = (
            self.package_dir / "scripts" / "gazebo_truth_odometry.py"
        ).read_text()
        self.assertIn('"~gicp_pose_topic"', truth_script)
        self.assertNotIn("TransformBroadcaster", truth_script)
        self.assertNotIn("/localization/pose", truth_script)

    def test_pose_guard_is_wired_before_public_pose_and_mapping(self):
        config_path = self.package_dir / "config" / "default.yaml"
        config = yaml.safe_load(config_path.read_text())
        adapter_source = (
            self.package_dir
            / "src"
            / "danger_search_localization"
            / "adapter_node.py"
        ).read_text()
        mapper_source = (
            self.package_dir
            / "src"
            / "danger_search_localization"
            / "occupancy_mapper_node.py"
        ).read_text()

        self.assertIn("PoseStabilizer", adapter_source)
        self.assertIn("self.pose_stabilizer.update", adapter_source)
        self.assertIn("self.validated_pose_pub.publish", adapter_source)
        self.assertIn("~validated_gicp_pose_topic", mapper_source)
        self.assertAlmostEqual(config["pose_jump_translation_margin_m"], 0.08)
        self.assertAlmostEqual(config["pose_jump_yaw_margin_rad"], 0.10)
        self.assertAlmostEqual(config["pose_gate_max_dt_s"], 0.50)
        self.assertEqual(config["pose_rejections_before_lost"], 3)
        self.assertNotIn("pose_recovery_timeout_s", config)

    def test_no_command_velocity_position_fallback_is_configured(self):
        config_path = self.package_dir / "config" / "default.yaml"
        config = yaml.safe_load(config_path.read_text())
        forbidden = {
            "command_translation_weight",
            "use_imu_translation_constraint",
            "use_cmd_vel_motion_constraints",
        }
        self.assertTrue(forbidden.isdisjoint(config))
