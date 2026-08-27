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

    def test_test_mode_allows_truth_backend(self):
        self.assertEqual(
            MODULE.validate_runtime_contract(
                False, False, "gazebo_truth", {"ENABLE_GROUND_TRUTH": "1"}
            ),
            [],
        )

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


if __name__ == "__main__":
    unittest.main()
