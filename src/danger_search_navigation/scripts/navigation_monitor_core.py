#!/usr/bin/env python3
"""ROS-independent helpers for the standard move_base compatibility monitor."""

import math


def classify_terminal_status(status, text=""):
    """Map actionlib terminal states and move_base text to the legacy code."""
    normalized = (text or "").lower()
    if status == 3:  # GoalStatus.SUCCEEDED
        return "SUCCEEDED"
    if status in (2, 8):  # PREEMPTED / RECALLED
        return "CANCELED"
    if status in (5,):  # REJECTED
        return "UNREACHABLE"
    if status in (4, 9):  # ABORTED / LOST
        if ("valid plan" in normalized or "unreachable" in normalized
                or "planning" in normalized):
            return "UNREACHABLE"
        return "CONTROL_FAILED"
    return "NONE"


def recovery_maneuver(behavior_name):
    name = (behavior_name or "").lower()
    if "rotate" in name:
        return "ROTATE"
    return "NONE"


def polyline_progress(points, position):
    """Return normalized progress of the nearest projection on a polyline."""
    if not points:
        return 0.0
    if len(points) == 1:
        return 1.0 if math.hypot(
            position[0] - points[0][0], position[1] - points[0][1]
        ) < 1e-6 else 0.0

    lengths = [0.0]
    for index in range(1, len(points)):
        lengths.append(lengths[-1] + math.hypot(
            points[index][0] - points[index - 1][0],
            points[index][1] - points[index - 1][1],
        ))
    total = lengths[-1]
    if total <= 1e-9:
        return 0.0

    best_distance = float("inf")
    best_progress = 0.0
    px, py = position
    for index in range(1, len(points)):
        ax, ay = points[index - 1]
        bx, by = points[index]
        dx, dy = bx - ax, by - ay
        squared = dx * dx + dy * dy
        ratio = 0.0 if squared <= 1e-12 else max(
            0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / squared)
        )
        projected_x = ax + ratio * dx
        projected_y = ay + ratio * dy
        distance = math.hypot(px - projected_x, py - projected_y)
        progress = lengths[index - 1] + ratio * math.sqrt(squared)
        if distance < best_distance:
            best_distance = distance
            best_progress = progress
    return max(0.0, min(1.0, best_progress / total))

