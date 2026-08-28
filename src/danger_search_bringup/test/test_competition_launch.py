#!/usr/bin/env python3

import pathlib
import unittest
import xml.etree.ElementTree as ET


PACKAGE = pathlib.Path(__file__).parents[1]


def launch_arguments(root):
    return {element.attrib["name"]: element for element in root.findall("arg")}


def include_arguments(root):
    include = root.find("include")
    return {
        child.attrib["name"]: child.attrib["value"]
        for child in include.findall("arg")
    }


class CompetitionLaunchTest(unittest.TestCase):
    def setUp(self):
        self.formal = ET.parse(PACKAGE / "launch" / "competition.launch").getroot()
        self.truth = ET.parse(
            PACKAGE / "launch" / "simulation_truth.launch"
        ).getroot()
        self.system = ET.parse(PACKAGE / "launch" / "system.launch").getroot()

    def test_public_wrappers_expose_only_safe_operational_arguments(self):
        formal_arguments = launch_arguments(self.formal)
        truth_arguments = launch_arguments(self.truth)
        for arguments in (formal_arguments, truth_arguments):
            self.assertIn("simenv_root", arguments)
            self.assertIn("result_file", arguments)
            self.assertIn("scene_info_file", arguments)
            self.assertIn("autostart", arguments)
            self.assertNotIn("competition_mode", arguments)
            self.assertNotIn("multifloor_enabled", arguments)
            self.assertNotIn("localization_backend", arguments)
            self.assertNotIn("gazebo_base_link", arguments)
            self.assertNotIn("use_hector_correction", arguments)

    def test_formal_wrapper_hard_codes_the_official_runtime_contract(self):
        arguments = include_arguments(self.formal)
        self.assertEqual(arguments["run_profile"], "formal")
        self.assertEqual(arguments["competition_mode"], "true")
        self.assertEqual(arguments["multifloor_enabled"], "true")
        self.assertEqual(arguments["localization_backend"], "gicp")
        self.assertEqual(arguments["use_hector_correction"], "false")
        self.assertEqual(
            launch_arguments(self.formal)["result_file"].attrib["default"],
            "$(arg simenv_root)/results/detected_danger.json",
        )

    def test_truth_wrapper_hard_codes_the_isolated_runtime_contract(self):
        arguments = include_arguments(self.truth)
        self.assertEqual(arguments["run_profile"], "simulation_truth")
        self.assertEqual(arguments["competition_mode"], "false")
        self.assertEqual(arguments["multifloor_enabled"], "true")
        self.assertEqual(arguments["localization_backend"], "gazebo_truth")
        self.assertEqual(
            launch_arguments(self.truth)["result_file"].attrib["default"],
            "$(arg simenv_root)/results/detected_danger.simulation_truth.json",
        )
        self.assertEqual(arguments["gazebo_base_link"], "a1_gazebo::base")

    def test_system_owns_all_nodes_and_forwards_runtime_contract(self):
        nodes = [node.attrib.get("name") for node in self.system.findall("node")]
        self.assertEqual(
            nodes,
            ["entrance_door", "perception", "exploration", "control",
             "posture_safety_monitor", "mission",
             "competition_preflight"],
        )
        includes = self.system.findall("include")
        self.assertEqual(len(includes), 2)
        localization = next(
            include for include in includes
            if "danger_search_localization" in include.attrib["file"]
        )
        localization_arguments = {
            child.attrib["name"]: child.attrib["value"]
            for child in localization.findall("arg")
        }
        self.assertEqual(
            localization_arguments["competition_mode"], "$(arg competition_mode)"
        )
        self.assertEqual(
            localization_arguments["multifloor_enabled"], "$(arg multifloor_enabled)"
        )
        self.assertEqual(
            localization_arguments["localization_backend"], "$(arg localization_backend)"
        )

        mission = next(
            node for node in self.system.findall("node")
            if node.attrib.get("name") == "mission"
        )
        mission_parameters = {
            child.attrib["name"]: child.attrib["value"]
            for child in mission.findall("param")
        }
        self.assertEqual(mission_parameters["run_profile"], "$(arg run_profile)")
        self.assertEqual(mission_parameters["result_file"], "$(arg result_file)")
        self.assertEqual(mission_parameters["require_preflight_ready"], "true")

        exploration = next(
            node for node in self.system.findall("node")
            if node.attrib.get("name") == "exploration"
        )
        exploration_parameters = {
            child.attrib["name"]: child.attrib["value"]
            for child in exploration.findall("param")
        }
        self.assertEqual(
            exploration_parameters["entrance_boundary_guard_enabled"],
            "$(arg entry_enabled)",
        )

    def test_system_preflight_is_required_and_disables_other_truth_sources(self):
        preflight = next(
            node for node in self.system.findall("node")
            if node.attrib.get("name") == "competition_preflight"
        )
        self.assertEqual(preflight.attrib.get("required"), "true")
        parameters = {
            child.attrib["name"]: child.attrib["value"]
            for child in preflight.findall("param")
        }
        self.assertEqual(parameters["run_profile"], "$(arg run_profile)")
        self.assertEqual(parameters["gazebo_base_link"], "$(arg gazebo_base_link)")
        environment = {
            item.attrib["name"]: item.attrib["value"]
            for item in self.system.findall("env")
        }
        self.assertEqual(environment["ENABLE_REFEREE_ODOM"], "0")
        self.assertEqual(environment["ENABLE_GROUND_TRUTH"], "0")
        self.assertEqual(environment["POINTCLOUD_USE_GROUND_TRUTH_ODOM"], "0")


if __name__ == "__main__":
    unittest.main()
