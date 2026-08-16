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
        self.assertEqual(scan_config.scan_accumulation_frames, 1)
        self.assertEqual(scan_config.min_valid_scan_bins, 40)
        self.assertEqual(scan_config.min_angular_coverage_rad, 0.35)
        AdapterConfig()

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

    def test_gicp_failure_threshold_defaults_match_runtime_yaml(self):
        config = AdapterConfig()
        self.assertEqual(config.gicp_failures_before_degraded, 3)
        self.assertEqual(config.gicp_failures_before_lost, 20)

    def test_invalid_gicp_failure_thresholds_are_rejected(self):
        with self.assertRaises(ValueError):
            AdapterConfig(gicp_failures_before_degraded=0)
        with self.assertRaises(ValueError):
            AdapterConfig(
                gicp_failures_before_degraded=4,
                gicp_failures_before_lost=3,
            )

    def test_default_pose_wiring_separates_hector_and_gicp(self):
        config_path = self.package_dir / "config" / "default.yaml"
        config = yaml.safe_load(config_path.read_text())

        self.assertEqual(config["backend_pose_topic"], "/localization/hector_pose")
        self.assertEqual(config["gicp_pose_topic"], "/localization/raw_pose")
        self.assertNotEqual(
            config["backend_pose_topic"], config["gicp_pose_topic"]
        )

        launch_path = self.package_dir / "launch" / "localization.launch"
        root = ElementTree.parse(launch_path).getroot()
        hector_node = next(
            node for node in root.findall("node") if node.attrib["name"] == "hector_mapping"
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
        self.assertEqual(parameters["use_tf_pose_start_estimate"], "true")
        self.assertEqual(parameters["map_with_known_poses"], "true")

        bridge = next(
            node for node in root.findall("node")
            if node.attrib["name"] == "known_pose_backend"
        )
        self.assertEqual(bridge.attrib["type"], "known_pose_backend.py")
