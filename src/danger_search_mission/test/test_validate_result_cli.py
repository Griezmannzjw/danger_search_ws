#!/usr/bin/env python3

import contextlib
import importlib.util
import io
import json
import pathlib
import tempfile
import unittest


SCRIPT = pathlib.Path(__file__).parents[1] / "scripts" / "validate_result.py"
SPEC = importlib.util.spec_from_file_location("validate_result", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class ValidateResultCliTest(unittest.TestCase):
    @staticmethod
    def _run(document, official=False):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "result.json"
            path.write_text(json.dumps(document), encoding="utf-8")
            arguments = [str(path)]
            if official:
                arguments.append("--official")
            output = io.StringIO()
            errors = io.StringIO()
            with contextlib.redirect_stdout(output), contextlib.redirect_stderr(errors):
                code = MODULE.main(arguments)
            return code, output.getvalue(), errors.getvalue()

    def test_completed_formal_gicp_result_passes_official_gate(self):
        code, output, errors = self._run({
            "mission_status": "FINISHED",
            "run_profile": "formal",
            "localization_backend": "gicp",
            "official_eligible": True,
        }, official=True)
        self.assertEqual(code, 0)
        self.assertIn("VALID official result", output)
        self.assertEqual(errors, "")

    def test_simulation_truth_is_rejected_by_official_gate(self):
        code, _output, errors = self._run({
            "mission_status": "FINISHED",
            "run_profile": "simulation_truth",
            "localization_backend": "gazebo_truth",
            "official_eligible": False,
        }, official=True)
        self.assertEqual(code, 2)
        self.assertIn("run_profile=formal", errors)
        self.assertIn("localization_backend=gicp", errors)
        self.assertIn("official_eligible=true", errors)

    def test_unfinished_or_forged_result_is_rejected(self):
        code, _output, errors = self._run({
            "mission_status": "ERROR",
            "run_profile": "simulation_truth",
            "localization_backend": "gicp",
            "official_eligible": True,
        }, official=True)
        self.assertEqual(code, 2)
        self.assertIn("mission_status=FINISHED", errors)
        self.assertIn("inconsistent", errors)

    def test_non_official_mode_still_checks_profile_consistency(self):
        code, _output, errors = self._run({
            "mission_status": "FINISHED",
            "run_profile": "simulation_truth",
            "localization_backend": "gazebo_truth",
            "official_eligible": True,
        })
        self.assertEqual(code, 2)
        self.assertIn("inconsistent", errors)


if __name__ == "__main__":
    unittest.main()
