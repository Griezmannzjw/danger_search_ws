#!/usr/bin/env python3

import pathlib
import unittest
import xml.etree.ElementTree as ET


PACKAGE = pathlib.Path(__file__).parents[1]


class CompetitionLaunchTest(unittest.TestCase):
    def setUp(self):
        self.root = ET.parse(PACKAGE / "launch" / "competition.launch").getroot()

    def test_portable_simenv_and_result_arguments_exist(self):
        arguments = {element.attrib["name"]: element for element in self.root.findall("arg")}
        self.assertIn("simenv_root", arguments)
        self.assertIn("result_file", arguments)
        self.assertIn("scene_info_file", arguments)
        self.assertIn("result_coordinate_frame", arguments)
        self.assertIn("autostart", arguments)
        self.assertIn("open_main_entrance", arguments)
        self.assertIn("competition_mode", arguments)
        self.assertIn("multifloor_enabled", arguments)
        self.assertIn("localization_backend", arguments)
        self.assertEqual(arguments["competition_mode"].attrib["default"], "true")
        self.assertEqual(arguments["multifloor_enabled"].attrib["default"], "true")
        self.assertEqual(arguments["localization_backend"].attrib["default"], "gicp")
        self.assertIn("$(find danger_search_bringup)", arguments["simenv_root"].attrib["default"])
        self.assertEqual(
            arguments["result_file"].attrib["default"],
            "$(arg simenv_root)/results/detected_danger.json",
        )
        self.assertEqual(
            arguments["scene_info_file"].attrib["default"],
            "$(arg simenv_root)/generated_building/team_scene_info.json",
        )

    def test_mission_launch_parameters_override_yaml(self):
        mission = next(
            node for node in self.root.findall("node")
            if node.attrib.get("name") == "mission"
        )
        children = list(mission)
        result_index = next(
            index for index, child in enumerate(children)
            if child.tag == "param" and child.attrib.get("name") == "result_file"
        )
        last_yaml_index = max(
            index for index, child in enumerate(children) if child.tag == "rosparam"
        )
        self.assertGreater(result_index, last_yaml_index)
        self.assertEqual(children[result_index].attrib["value"], "$(arg result_file)")

    def test_all_runtime_nodes_are_present_once(self):
        nodes = [node.attrib.get("name") for node in self.root.findall("node")]
        self.assertEqual(
            nodes,
            ["entrance_door", "perception", "exploration", "control", "mission",
             "competition_preflight"],
        )
        includes = self.root.findall("include")
        self.assertEqual(len(includes), 2)
        include_files = [element.attrib["file"] for element in includes]
        self.assertTrue(any(
            "danger_search_localization" in path for path in include_files
        ))
        self.assertTrue(any(
            "danger_search_navigation" in path for path in include_files
        ))

    def test_localization_receives_canonical_runtime_mode_arguments(self):
        localization = next(
            include for include in self.root.findall("include")
            if "danger_search_localization" in include.attrib["file"]
        )
        arguments = {
            child.attrib["name"]: child.attrib["value"]
            for child in localization.findall("arg")
        }
        self.assertEqual(arguments["competition_mode"], "$(arg competition_mode)")
        self.assertEqual(arguments["multifloor_enabled"], "$(arg multifloor_enabled)")
        self.assertEqual(arguments["localization_backend"], "$(arg localization_backend)")
        self.assertNotIn("localization_source", arguments)
        self.assertNotIn("enable_multifloor_maps", arguments)

    def test_preflight_is_required_and_formal_truth_is_disabled(self):
        preflight = next(
            node for node in self.root.findall("node")
            if node.attrib.get("name") == "competition_preflight"
        )
        self.assertEqual(preflight.attrib.get("required"), "true")
        environment = {
            item.attrib["name"]: item.attrib["value"]
            for item in self.root.findall("env")
        }
        self.assertEqual(environment["ENABLE_REFEREE_ODOM"], "0")
        self.assertEqual(environment["ENABLE_GROUND_TRUTH"], "0")
        self.assertEqual(environment["POINTCLOUD_USE_GROUND_TRUTH_ODOM"], "0")


if __name__ == "__main__":
    unittest.main()
