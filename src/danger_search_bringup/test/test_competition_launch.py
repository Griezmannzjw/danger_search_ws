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
        self.assertIn("autostart", arguments)
        self.assertIn("open_main_entrance", arguments)
        self.assertIn("$(find danger_search_bringup)", arguments["simenv_root"].attrib["default"])
        self.assertEqual(
            arguments["result_file"].attrib["default"],
            "$(arg simenv_root)/results/detected_danger.json",
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
            ["entrance_door", "perception", "navigation", "exploration", "control", "mission"],
        )
        includes = self.root.findall("include")
        self.assertEqual(len(includes), 1)
        self.assertIn("danger_search_localization", includes[0].attrib["file"])


if __name__ == "__main__":
    unittest.main()
