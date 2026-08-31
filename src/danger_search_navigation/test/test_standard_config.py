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
        self.assertEqual(
            remaps["cmd_vel"], "/danger_search/move_base_cmd_vel"
        )
        mux = root.find("node[@name='navigation_command_mux']")
        self.assertIsNotNone(mux)
        self.assertEqual(mux.attrib["type"], "navigation_command_mux.py")

    def test_plugins_and_make_plan_safety_are_explicit(self):
        config = self._yaml("standard_move_base.yaml")
        self.assertEqual(config["base_global_planner"], "navfn/NavfnROS")
        self.assertEqual(
            config["base_local_planner"],
            "base_local_planner/TrajectoryPlannerROS",
        )
        self.assertFalse(config["make_plan_clear_costmap"])
        self.assertFalse(config["make_plan_add_unreachable_goal"])
        self.assertFalse(config["clearing_rotation_allowed"])
        self.assertEqual(len(config["recovery_behaviors"]), 2)
        self.assertNotIn("rotate_recovery", {
            behavior["name"] for behavior in config["recovery_behaviors"]
        })

    def test_unitree_velocity_floor_and_supported_parameters(self):
        config = self._yaml("trajectory_planner.yaml")["TrajectoryPlannerROS"]
        self.assertEqual(config["odom_topic"], "/localization/odom")
        self.assertEqual(config["min_vel_x"], 0.30)
        self.assertEqual(config["escape_vel"], -0.30)
        self.assertEqual(config["max_rotational_vel"], 0.80)
        self.assertEqual(config["max_vel_theta"], 0.80)
        self.assertEqual(config["min_vel_theta"], -0.80)
        self.assertEqual(config["min_in_place_vel_theta"], 0.80)
        self.assertEqual(config["acc_lim_theta"], 0.80)
        self.assertEqual(config["path_distance_bias"], 5.0)
        self.assertEqual(config["goal_distance_bias"], 5.0)
        self.assertIsInstance(config["y_vels"], str)
        self.assertIn("0.0", config["y_vels"])
        self.assertTrue(config["dwa"])
        self.assertTrue(config["holonomic_robot"])
        self.assertNotIn("min_vel_y", config)
        self.assertNotIn("max_vel_y", config)
        self.assertLessEqual(config["heading_scoring_timestep"], 1.0)

    def test_rotation_limits_do_not_exceed_cmd_mux_hard_cap(self):
        planner = self._yaml("trajectory_planner.yaml")["TrajectoryPlannerROS"]
        mux = self._control_config()
        limit = mux["max_angular_speed"]
        self.assertEqual(planner["max_rotational_vel"], limit)
        self.assertEqual(planner["max_vel_theta"], limit)
        self.assertEqual(planner["min_in_place_vel_theta"], limit)
        self.assertGreaterEqual(planner["min_vel_theta"], -limit)

    def test_translation_floor_matches_unitree_and_cmd_mux_cap(self):
        planner = self._yaml("trajectory_planner.yaml")["TrajectoryPlannerROS"]
        mux = self._control_config()
        self.assertEqual(planner["min_vel_x"], 0.30)
        self.assertLessEqual(planner["min_vel_x"], planner["max_vel_x"])
        self.assertLessEqual(planner["max_vel_x"], mux["max_linear_speed"])

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
