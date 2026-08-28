#!/usr/bin/env python3
"""Fail-closed profile gate to run before an official evaluator."""

import argparse
import json
import sys

from danger_search_mission.mission_core import result_profile_errors


def build_parser():
    parser = argparse.ArgumentParser(
        description="Validate danger-search result profile metadata."
    )
    parser.add_argument("result_file", help="Result JSON written by mission")
    parser.add_argument(
        "--official",
        action="store_true",
        help="Require a completed formal GICP run eligible for official scoring",
    )
    return parser


def main(argv=None):
    arguments = build_parser().parse_args(argv)
    try:
        with open(arguments.result_file, encoding="utf-8") as stream:
            document = json.load(stream)
    except (OSError, json.JSONDecodeError) as exc:
        print("INVALID result: %s" % exc, file=sys.stderr)
        return 2

    errors = result_profile_errors(document, official=arguments.official)
    if errors:
        print("INVALID result: %s" % "; ".join(errors), file=sys.stderr)
        return 2
    mode = "official" if arguments.official else "profile"
    print("VALID %s result: %s" % (mode, arguments.result_file))
    return 0


if __name__ == "__main__":
    sys.exit(main())
