#!/usr/bin/env python3
"""Entry point for the canonical localization interface adapter."""

import rospy

from danger_search_localization.adapter_node import LocalizationAdapterNode


if __name__ == "__main__":
    try:
        LocalizationAdapterNode().run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
