#!/usr/bin/env python3

import pathlib
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE = pathlib.Path(__file__).parents[1]


class StandardNavigationConfigTest(unittest.TestCase):
    def _yaml(self, name):
        with (PACKAGE / "config" / name).open() as stream:
            return yaml.safe_load(stream)

    @staticmethod
    def _control_config():
        path = PACKAGE.parent / "danger_search_control" / "config" / "default.yaml"
        with path.open() as stream:
            return yaml.safe_load(stream)

    def test_launch_uses_standard_move_base_and_cmd_mux_input(self):
        root = ET.parse(PACKAGE / "launch" / "navigation.launch").getroot()
        move_base = next(
            node for node in root.findall("node")
            if node.attrib.get("name") == "move_base"
        )
        self.assertEqual(move_base.attrib["pkg"], "move_base")
        self.assertEqual(move_base.attrib["type"], "move_base")
        remaps = {
            remap.attrib["from"]: remap.attrib["to"]
            for remap in move_base.findall("remap")
        }
        self.assertEqual(remaps["cmd_vel"], "/danger_search/nav_cmd_vel")

    def test_plugins_and_make_plan_safety_are_explicit(self):
        config = self._yaml("standard_move_base.yaml")
        self.assertEqual(config["base_global_planner"], "navfn/NavfnROS")
        self.assertEqual(
            config["base_local_planner"],
            "dwa_local_planner/DWAPlannerROS",
        )
        self.assertFalse(config["make_plan_clear_costmap"])
        self.assertFalse(config["make_plan_add_unreachable_goal"])
        self.assertFalse(config["clearing_rotation_allowed"])
        self.assertEqual(
            [behavior["name"] for behavior in config["recovery_behaviors"]],
            ["conservative_reset", "escape_recovery_1",
             "aggressive_reset", "escape_recovery_2"],
        )
        for name in ("escape_recovery_1", "escape_recovery_2"):
            self.assertEqual(config[name]["max_attempts_per_goal"], 2)
            self.assertEqual(config[name]["simulation_step"], 0.025)
            self.assertTrue(config[name]["enable_arc"])
            self.assertEqual(config[name]["arc_distance"], 0.45)
            self.assertEqual(config[name]["arc_linear_speed"], 0.40)
            self.assertEqual(config[name]["arc_angular_speed"], 0.40)
            self.assertFalse(config[name]["enable_strafe"])
        self.assertNotIn("rotate_recovery", config)
        self.assertNotIn("TrajectoryPlannerROS", config)

    def test_costmaps_fuse_ground_filtered_depth_obstacles(self):
        common = self._yaml("costmap_common.yaml")
        self.assertGreaterEqual(common["max_obstacle_height"], 6.4)
        for filename in ("local_costmap.yaml", "global_costmap.yaml"):
            obstacles = self._yaml(filename)["obstacles"]
            self.assertEqual(
                obstacles["observation_sources"], "scan depth_scan"
            )
            depth = obstacles["depth_scan"]
            self.assertEqual(depth["data_type"], "LaserScan")
            self.assertEqual(
                depth["topic"], "/localization/depth_obstacle_scan"
            )
            self.assertTrue(depth["marking"])
            self.assertFalse(depth["clearing"])
            self.assertEqual(depth["expected_update_rate"], 0.0)
            self.assertLessEqual(depth["obstacle_range"], 3.0)

    def test_unitree_dwa_velocity_domain_and_supported_parameters(self):
        config = self._yaml("dwa_planner.yaml")["DWAPlannerROS"]
        self.assertEqual(config["odom_topic"], "/localization/odom")
        self.assertEqual(config["min_vel_trans"], 0.30)
        self.assertEqual(config["max_vel_trans"], 0.40)
        self.assertEqual(config["min_vel_x"], 0.30)
        self.assertEqual(config["max_vel_x"], 0.40)
        self.assertEqual(config["min_vel_y"], 0.0)
        self.assertEqual(config["max_vel_y"], 0.0)
        self.assertEqual(config["max_vel_theta"], 0.40)
        self.assertEqual(config["min_vel_theta"], 0.40)
        self.assertEqual((config["vx_samples"], config["vy_samples"],
                          config["vth_samples"]), (5, 1, 9))
        self.assertTrue(config["use_dwa"])
        self.assertEqual(config["path_distance_bias"], 32.0)
        self.assertEqual(config["goal_distance_bias"], 24.0)
        self.assertEqual(config["occdist_scale"], 0.02)
        self.assertEqual(config["twirling_scale"], 0.30)
        self.assertEqual(config["xy_goal_tolerance"], 0.15)

        self.assertEqual(config["vy_samples"], 1)
        samples = [
            -config["max_vel_theta"] + index * (
                2.0 * config["max_vel_theta"]
            ) / (config["vth_samples"] - 1)
            for index in range(config["vth_samples"])
        ]
        for actual, expected in zip(
                samples,
                [-0.40, -0.30, -0.20, -0.10, 0.0,
                 0.10, 0.20, 0.30, 0.40]):
            self.assertAlmostEqual(actual, expected, places=9)

    def test_dwa_speed_and_acceleration_contract_matches_cmd_mux(self):
        planner = self._yaml("dwa_planner.yaml")["DWAPlannerROS"]
        mux = self._control_config()
        self.assertLessEqual(planner["max_vel_x"], mux["max_linear_speed"])
        self.assertGreaterEqual(planner["min_vel_x"], planner["min_vel_trans"])
        self.assertLessEqual(planner["max_vel_y"], mux["max_lateral_speed"])
        safety_limit = 0.40
        self.assertLessEqual(planner["max_vel_theta"], safety_limit)
        self.assertLessEqual(planner["min_vel_theta"], safety_limit)
        self.assertLessEqual(planner["max_vel_theta"], mux["max_angular_speed"])
        self.assertLessEqual(
            planner["min_vel_theta"], mux["max_angular_speed"]
        )
        self.assertEqual(planner["acc_lim_x"], mux["max_linear_accel"])
        self.assertEqual(planner["acc_lim_y"], mux["max_lateral_accel"])
        self.assertEqual(planner["acc_lim_theta"], mux["max_angular_accel"])

        frequency = self._yaml("standard_move_base.yaml")["controller_frequency"]
        self.assertGreaterEqual(
            planner["acc_lim_x"] / frequency, planner["min_vel_trans"]
        )
        self.assertGreaterEqual(
            planner["acc_lim_y"] / frequency, planner["max_vel_y"]
        )
        self.assertGreaterEqual(
            planner["acc_lim_theta"] / frequency, planner["max_vel_theta"]
        )
        self.assertEqual(planner["min_vel_theta"], safety_limit)

    def test_launch_loads_dwa_and_guard_checks_its_namespace(self):
        root = ET.parse(PACKAGE / "launch" / "navigation.launch").getroot()
        move_base = next(node for node in root.findall("node")
                         if node.attrib.get("name") == "move_base")
        files = [item.attrib.get("file", "") for item in move_base.findall("rosparam")]
        self.assertTrue(any("dwa_planner.yaml" in path for path in files))
        self.assertFalse(any("trajectory_planner.yaml" in path for path in files))
        guard = next(node for node in root.findall("node")
                     if node.attrib.get("name") == "navigation_config_guard")
        params = {item.attrib["name"]: item.attrib["value"]
                  for item in guard.findall("param")}
        self.assertEqual(params["planner_name"], "/move_base/DWAPlannerROS")
        self.assertEqual(params["planner_config_key"], "DWAPlannerROS")
        self.assertEqual(params["safe_max_angular_speed_rps"], "0.40")
        self.assertEqual(
            params["effective_min_in_place_angular_speed_rps"], "0.40"
        )

    def test_escape_recovery_is_a_nav_core_plugin(self):
        root = ET.parse(PACKAGE / "recovery_plugin.xml").getroot()
        plugin = root.find("class")
        self.assertIsNotNone(plugin)
        self.assertEqual(
            plugin.attrib["name"],
            "danger_search_navigation/UnitreeEscapeRecovery",
        )
        self.assertEqual(plugin.attrib["base_class_type"],
                         "nav_core::RecoveryBehavior")
        package_xml = ET.parse(PACKAGE / "package.xml").getroot()
        export = package_xml.find("export/nav_core")
        self.assertIsNotNone(export)
        self.assertEqual(export.attrib["plugin"],
                         "${prefix}/recovery_plugin.xml")

    def test_recovery_declares_direct_tf2_dependencies_and_tests(self):
        package_xml = ET.parse(PACKAGE / "package.xml").getroot()
        for tag in ("build_depend", "build_export_depend", "exec_depend"):
            dependencies = {item.text for item in package_xml.findall(tag)}
            self.assertIn("tf2", dependencies, msg=tag)
            self.assertIn("tf2_ros", dependencies, msg=tag)

        cmake = (PACKAGE / "CMakeLists.txt").read_text()
        self.assertIn("catkin_add_gtest(test_escape_recovery", cmake)
        self.assertIn(
            "catkin_add_nosetests(test/test_navigation_monitor_callbacks.py)",
            cmake,
        )

    def test_costmaps_use_fixed_padded_footprint_and_scan(self):
        common = self._yaml("costmap_common.yaml")
        self.assertEqual(common["footprint_padding"], 0.04)
        self.assertEqual(
            common["footprint"],
            [[0.30, 0.15], [0.30, -0.15],
             [-0.35, -0.15], [-0.35, 0.15]],
        )
        for name in ("global_costmap.yaml", "local_costmap.yaml"):
            config = self._yaml(name)
            self.assertEqual(
                config["obstacles"]["scan"]["topic"],
                "/localization/scan",
            )
            self.assertEqual(config["static"]["lethal_cost_threshold"], 65)
            self.assertEqual(config["inflation"]["inflation_radius"], 0.55)


if __name__ == "__main__":
    unittest.main()
