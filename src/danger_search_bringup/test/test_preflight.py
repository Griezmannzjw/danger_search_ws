#!/usr/bin/env python3

import importlib.util
import json
import pathlib
import tempfile
import unittest


SCRIPT = pathlib.Path(__file__).parents[1] / "scripts" / "competition_preflight.py"
SPEC = importlib.util.spec_from_file_location("competition_preflight", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class CompetitionPreflightPureTest(unittest.TestCase):
    def test_formal_runtime_requires_gicp_multifloor_and_truth_off(self):
        errors = MODULE.validate_runtime_contract(
            True,
            False,
            "gazebo_truth",
            {"ENABLE_GROUND_TRUTH": "1"},
        )
        self.assertTrue(any("multifloor_enabled" in item for item in errors))
        self.assertTrue(any("localization_backend" in item for item in errors))
        self.assertTrue(any("ENABLE_GROUND_TRUTH" in item for item in errors))

    def test_simulation_truth_runtime_requires_the_explicit_truth_contract(self):
        self.assertEqual(
            MODULE.validate_runtime_contract(
                False,
                True,
                "gazebo_truth",
                {"ENABLE_GROUND_TRUTH": "1"},
                "simulation_truth",
            ),
            [],
        )
        errors = MODULE.validate_runtime_contract(
            False, True, "gicp", {}, "simulation_truth"
        )
        self.assertTrue(any("gazebo_truth" in item for item in errors))
        errors = MODULE.validate_runtime_contract(
            True, True, "gazebo_truth", {}, "simulation_truth"
        )
        self.assertTrue(any("competition_mode" in item for item in errors))

    def test_unknown_run_profile_is_rejected(self):
        errors = MODULE.validate_runtime_contract(
            True, True, "gicp", {}, "prototype"
        )
        self.assertEqual(errors, ["run_profile must be formal or simulation_truth"])

    def test_simulation_truth_rejects_any_non_competition_base_link(self):
        self.assertEqual(MODULE.validate_truth_base_link(
            "simulation_truth", "a1_gazebo::base"
        ), [])
        errors = MODULE.validate_truth_base_link(
            "simulation_truth", "some_other_model::base"
        )
        self.assertEqual(errors, [
            "simulation_truth profile requires gazebo_base_link=a1_gazebo::base"
        ])
        self.assertEqual(MODULE.validate_truth_base_link(
            "formal", "some_other_model::base"
        ), [])

    def test_result_filename_is_profile_specific(self):
        self.assertEqual(
            MODULE.expected_result_filename("formal"), "detected_danger.json"
        )
        self.assertEqual(
            MODULE.expected_result_filename("simulation_truth"),
            "detected_danger.simulation_truth.json",
        )

    def test_preflight_requires_the_active_map_and_navigation_envelopes(self):
        self.assertEqual(
            MODULE.REQUIRED_TOPIC_TYPES["/mapping/active_map"],
            "danger_search_common/FloorOccupancyGrid",
        )
        self.assertEqual(
            MODULE.REQUIRED_TOPIC_TYPES["/navigation/health"],
            "danger_search_common/NavigationHealth",
        )
        self.assertEqual(
            MODULE.REQUIRED_TOPIC_TYPES["/localization/depth_obstacle_scan"],
            "sensor_msgs/LaserScan",
        )
        self.assertIn("/depth_obstacle_projector", MODULE.ALGORITHM_NODES)

    def test_only_public_scene_contract_filename_is_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            public_path = pathlib.Path(directory) / "team_scene_info.json"
            public_path.write_text(json.dumps({
                "schema": "team_scene_info_v1",
                "public_scene": {
                    "elevators": [{"id": "main", "served_floors": [0, 1]}]
                },
            }), encoding="utf-8")
            normalized, document = MODULE.load_public_scene_contract(public_path)
            self.assertEqual(pathlib.Path(normalized), public_path)
            self.assertEqual(document["schema"], "team_scene_info_v1")

            forbidden = pathlib.Path(directory) / "building_config.json"
            forbidden.write_text("{}", encoding="utf-8")
            with self.assertRaises(ValueError):
                MODULE.load_public_scene_contract(forbidden)

    def test_public_forbidden_topics_extend_builtins_without_redeclaring_them(self):
        topics = MODULE.effective_forbidden_topics({
            "referee_only": {"forbidden_topics": ["/private/referee_pose"]}
        })
        self.assertIn("/gazebo/link_states", topics)
        self.assertIn("/private/referee_pose", topics)

        builtins_only = MODULE.effective_forbidden_topics({
            "referee_only": {"forbidden_topics": []}
        })
        self.assertEqual(builtins_only, MODULE.FORBIDDEN_TOPICS)

        truth_topics = MODULE.effective_forbidden_topics(
            {"referee_only": {"forbidden_topics": ["/gazebo/link_states"]}},
            "simulation_truth",
        )
        self.assertNotIn("/gazebo/link_states", truth_topics)
        self.assertIn("/Odometry_gazebo", truth_topics)

    def test_forbidden_topic_and_file_parameters_are_reported(self):
        errors = MODULE.forbidden_parameter_references({
            "localization": {"pose_topic": "/Odometry_gazebo"},
            "scene": "/tmp/generated_building/danger_truth.json",
        })
        self.assertEqual(len(errors), 2)

    def test_cmd_vel_has_exactly_one_expected_owner(self):
        self.assertEqual(MODULE.command_owner_error(["/control"], "/control"), "")
        self.assertTrue(MODULE.command_owner_error(
            ["/control", "/exploration"], "/control"
        ))

    def test_system_state_accepts_rosgraph_and_raw_xmlrpc_shapes(self):
        state = [
            [["/cmd_vel", ["/control"]]],
            [["/scan", ["/localizer"]]],
            [["/call_elevator", ["/building"]]],
        ]
        expected = (
            {"/cmd_vel": ["/control"]},
            {"/scan": ["/localizer"]},
            {"/call_elevator": ["/building"]},
        )
        self.assertEqual(MODULE.normalize_system_state(state), expected)
        self.assertEqual(
            MODULE.normalize_system_state([1, "ok", state]), expected
        )

    def test_forbidden_subscription_check_ignores_platform_nodes(self):
        subscribers = {
            "/ground_truth/base_w": ["/unitree_gazebo_servo"],
            "/gazebo/link_states": ["/localization_adapter"],
        }
        violations = MODULE.forbidden_algorithm_subscriptions(
            subscribers, MODULE.FORBIDDEN_TOPICS
        )
        self.assertEqual(len(violations), 1)
        self.assertIn("/localization_adapter", violations[0])

    def test_truth_profile_allows_only_the_dedicated_node_subscription(self):
        self.assertEqual(MODULE.truth_subscription_violations({
            "/gazebo/link_states": ["/gazebo_truth_odometry", "/unitree_gazebo_servo"]
        }), [])
        violations = MODULE.truth_subscription_violations({
            "/gazebo/link_states": ["/gazebo_truth_odometry", "/exploration"]
        })
        self.assertEqual(len(violations), 1)
        self.assertIn("/exploration", violations[0])
        missing = MODULE.truth_subscription_violations({
            "/gazebo/link_states": ["/unitree_gazebo_servo"]
        })
        self.assertEqual(len(missing), 1)
        self.assertIn("must subscribe", missing[0])

    def test_truth_profile_only_allows_the_private_truth_parameter(self):
        tree = {
            "gazebo_truth_odometry": {
                "gazebo_link_states_topic": "/gazebo/link_states",
            },
            "localization_adapter": {"pose_topic": "/localization/raw_pose"},
        }
        self.assertEqual(MODULE.truth_parameter_reference_violations(tree), [])
        tree["exploration"] = {"debug_topic": "/gazebo/link_states"}
        violations = MODULE.truth_parameter_reference_violations(tree)
        self.assertEqual(len(violations), 1)
        self.assertIn("/exploration/debug_topic", violations[0])

    def test_runtime_identity_requires_one_profile_on_every_algorithm_node(self):
        nodes = set(MODULE.RUNTIME_IDENTITY_NODES) | {
            MODULE.TRUTH_ODOMETRY_NODE
        }
        tree = {
            node.lstrip("/"): {
                "run_profile": "simulation_truth",
                "competition_mode": False,
                "multifloor_enabled": True,
                "localization_backend": "gazebo_truth",
            }
            for node in nodes
        }
        self.assertEqual(MODULE.runtime_identity_parameter_violations(
            tree, "simulation_truth", False, True, "gazebo_truth"
        ), [])

        tree["local_occupancy_mapper"]["competition_mode"] = True
        violations = MODULE.runtime_identity_parameter_violations(
            tree, "simulation_truth", False, True, "gazebo_truth"
        )
        self.assertEqual(len(violations), 1)
        self.assertIn("/local_occupancy_mapper/competition_mode", violations[0])


if __name__ == "__main__":
    unittest.main()
