#!/usr/bin/env python3
"""多楼层探索单元测试：电梯井自主检测（门缝发现）。

只测 ROS 无关的 _detect_elevator_halls / _perimeter_door_gaps 逻辑。
"""

import importlib.util
import math
import pathlib
import unittest
from types import SimpleNamespace

import numpy as np


SCRIPT = pathlib.Path(__file__).parents[1] / "scripts" / "exploration_planner.py"
SPEC = importlib.util.spec_from_file_location("exploration_planner", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def make_planner(grid, resolution=0.10):
    planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
    size = grid.shape[0]
    planner.map_data = np.array(grid, dtype=np.int8)
    planner.map_info = SimpleNamespace(
        width=size,
        height=size,
        resolution=resolution,
        origin=SimpleNamespace(
            position=SimpleNamespace(x=-size * resolution / 2.0,
                                     y=-size * resolution / 2.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )
    planner.connectivity_occupied_threshold = 65
    planner.shaft_min_area_m2 = 4.0
    planner.shaft_max_area_m2 = 50.0
    planner.door_gap_min_width_m = 0.8
    planner.door_gap_max_width_m = 2.5
    planner.free_threshold = 40
    return planner


def build_shaft_map(size_cells, shaft=(1.5, 4.0, 1.0, 4.0), door=(2.0, 3.0)):
    """自由地图 + 一个矩形井道，西墙带门缝。所有值：0=自由, 100=占用。"""
    res = 0.10
    half = size_cells * res / 2.0
    grid = np.zeros((size_cells, size_cells), dtype=np.int8)

    def setrect(x1, y1, x2, y2, val):
        for cx in range(int((x1 + half) / res), int((x2 + half) / res) + 1):
            for cy in range(int((y1 + half) / res), int((y2 + half) / res) + 1):
                if 0 <= cx < size_cells and 0 <= cy < size_cells:
                    grid[cy, cx] = val

    x0, x1, y0, y1 = shaft
    d0, d1 = door
    setrect(x0, y0, x0 + 0.2, d0, 100)        # 西墙下段
    setrect(x0, d1, x0 + 0.2, y1, 100)        # 西墙上段
    setrect(x0, y0, x1, y0 + 0.2, 100)        # 南墙
    setrect(x0, y1 - 0.2, x1, y1, 100)        # 北墙
    setrect(x1 - 0.2, y0, x1, y1, 100)        # 东墙
    return grid


class ElevatorDetectionTest(unittest.TestCase):
    def test_shaft_with_west_door_is_detected(self):
        planner = make_planner(build_shaft_map(400))
        halls = planner._detect_elevator_halls()
        self.assertTrue(halls, "应当检测到至少一个电梯厅候选")
        # 电梯门缝中心约 (1.5, 2.5)，外侧应落在其附近
        self.assertTrue(
            any(math.hypot(hx - 1.5, hy - 2.5) < 1.0 for hx, hy, _ in halls),
            "应在西墙门缝附近找到候选",
        )

    def test_door_yaw_points_into_shaft(self):
        planner = make_planner(build_shaft_map(400))
        halls = planner._detect_elevator_halls()
        west = [yaw for hx, hy, yaw in halls
                if abs(hx - 1.5) < 1.0 and abs(hy - 2.5) < 1.0]
        self.assertTrue(west, "应找到西墙门缝候选")
        # 西墙门缝朝井道内 = +x = yaw 0
        self.assertLess(abs(west[0]), 0.2)

    def test_no_shaft_no_candidates(self):
        planner = make_planner(np.zeros((200, 200), dtype=np.int8))
        self.assertEqual(planner._detect_elevator_halls(), [])

    def test_sealed_shaft_without_door_is_not_candidate(self):
        grid = np.zeros((400, 400), dtype=np.int8)
        half = 20.0
        res = 0.10
        # 封闭的实心矩形，无门缝
        for cx in range(int((1.5 + half) / res), int((4.0 + half) / res) + 1):
            for cy in range(int((1.0 + half) / res), int((4.0 + half) / res) + 1):
                grid[cy, cx] = 100
        planner = make_planner(grid)
        self.assertEqual(planner._detect_elevator_halls(), [])


if __name__ == "__main__":
    unittest.main()
