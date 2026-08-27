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
        self.assertTrue(config["clearing_rotation_allowed"])
        self.assertEqual(
            [behavior["name"] for behavior in config["recovery_behaviors"]],
            ["conservative_reset", "escape_recovery_1", "rotate_recovery",
             "aggressive_reset", "escape_recovery_2"],
        )
        for name in ("escape_recovery_1", "escape_recovery_2"):
            self.assertEqual(config[name]["max_attempts_per_goal"], 2)
            self.assertEqual(config[name]["simulation_step"], 0.025)
            self.assertFalse(config[name]["enable_strafe"])

    def test_unitree_dwa_velocity_domain_and_supported_parameters(self):
        config = self._yaml("dwa_planner.yaml")["DWAPlannerROS"]
        self.assertEqual(config["odom_topic"], "/localization/odom")
        self.assertEqual(config["min_vel_trans"], 0.30)
        self.assertEqual(config["max_vel_trans"], 0.40)
        self.assertEqual(config["min_vel_x"], 0.0)
        self.assertEqual(config["max_vel_x"], 0.40)
        self.assertEqual(config["min_vel_y"], 0.0)
        self.assertEqual(config["max_vel_y"], 0.0)
        self.assertEqual(config["max_vel_theta"], 0.80)
        self.assertEqual(config["min_vel_theta"], 0.80)
        self.assertEqual((config["vx_samples"], config["vy_samples"],
                          config["vth_samples"]), (5, 1, 5))
        self.assertTrue(config["use_dwa"])
        self.assertEqual(config["path_distance_bias"], 32.0)
        self.assertEqual(config["goal_distance_bias"], 24.0)
        self.assertEqual(config["occdist_scale"], 0.02)
        self.assertEqual(config["twirling_scale"], 0.30)

        self.assertEqual(config["vy_samples"], 1)

    def test_dwa_speed_and_acceleration_contract_matches_cmd_mux(self):
        planner = self._yaml("dwa_planner.yaml")["DWAPlannerROS"]
        mux = self._control_config()
        self.assertLessEqual(planner["max_vel_x"], mux["max_linear_speed"])
        self.assertLessEqual(planner["max_vel_y"], mux["max_lateral_speed"])
        self.assertEqual(planner["max_vel_theta"], mux["max_angular_speed"])
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
            planner["acc_lim_theta"] / frequency, planner["min_vel_theta"]
        )

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
