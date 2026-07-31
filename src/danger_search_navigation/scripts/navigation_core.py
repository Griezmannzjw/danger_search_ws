#!/usr/bin/env python3
"""Pure planning primitives used by the navigation ROS node.

Keeping this module free of rospy makes the collision policy and A* behavior
directly unit-testable.  The ROS entry point remains nav_controller.py.
"""

from dataclasses import dataclass
import heapq
import math


VALID_FAILURE_CODES = frozenset((
    "NONE",
    "SUCCEEDED",
    "UNREACHABLE",
    "CANCELED",
    "TIMEOUT",
    "CONTROL_FAILED",
    "SAFETY_STOP",
    "ROBOT_FALLEN",
    "LOCALIZATION_LOST",
))


def zero_velocity():
    """Return the non-holonomic linear/angular zero command."""
    return 0.0, 0.0


@dataclass
class GoalState:
    """Mutable action state shared with the ROS health publisher."""

    active_goal_id: str = ""
    active: bool = False
    controller_active: bool = False
    stuck: bool = False
    failure_code: str = "NONE"
    failure_detail: str = ""
    progress: float = 0.0

    def begin(self, goal_id):
        self.active_goal_id = goal_id
        self.active = True
        self.controller_active = True
        self.stuck = False
        self.failure_code = "NONE"
        self.failure_detail = ""
        self.progress = 0.0

    def finish(self, failure_code, detail="", stuck=False):
        if failure_code not in VALID_FAILURE_CODES:
            raise ValueError("Unsupported failure code: %s" % failure_code)
        self.active_goal_id = ""
        self.active = False
        self.controller_active = False
        self.stuck = bool(stuck)
        self.failure_code = failure_code
        self.failure_detail = detail
        if failure_code == "SUCCEEDED":
            self.progress = 1.0

    def cancel(self, detail="Action goal canceled"):
        self.finish("CANCELED", detail)
        return zero_velocity()


class InflatedOccupancyGrid:
    """A conservative, immutable occupancy grid used by all plan requests."""

    def __init__(self, width, height, resolution, origin_x, origin_y, origin_yaw,
                 data, occupied_threshold, robot_radius, inflation_padding,
                 allow_diagonal, max_expansions):
        if width <= 0 or height <= 0 or resolution <= 0.0:
            raise ValueError("Invalid OccupancyGrid geometry")
        if len(data) != width * height:
            raise ValueError("OccupancyGrid data length does not match geometry")
        if occupied_threshold < 1 or robot_radius < 0.0 or inflation_padding < 0.0:
            raise ValueError("Invalid occupancy or inflation configuration")
        if max_expansions < 1:
            raise ValueError("max_expansions must be positive")

        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.origin_yaw = float(origin_yaw)
        self.cos_origin_yaw = math.cos(self.origin_yaw)
        self.sin_origin_yaw = math.sin(self.origin_yaw)
        self.allow_diagonal = bool(allow_diagonal)
        self.max_expansions = int(max_expansions)
        self.inflation_radius = float(robot_radius) + float(inflation_padding)

        # P0 only defines -1, 0, and 100.  Treat every non-free value as
        # blocked so unknown/probabilistic cells never become traversable.
        self.base_blocked = [value != 0 for value in data]
        occupied_cells = [
            (index % self.width, index // self.width)
            for index, value in enumerate(data)
            if value >= occupied_threshold
        ]
        self.inflated_blocked = list(self.base_blocked)
        inflation_cells = self._disk_offsets(self.inflation_radius)
        for cell_x, cell_y in occupied_cells:
            for dx, dy in inflation_cells:
                nx = cell_x + dx
                ny = cell_y + dy
                if self.in_bounds(nx, ny):
                    self.inflated_blocked[self.index(nx, ny)] = True

    def index(self, cell_x, cell_y):
        return cell_y * self.width + cell_x

    def in_bounds(self, cell_x, cell_y):
        return 0 <= cell_x < self.width and 0 <= cell_y < self.height

    def _disk_offsets(self, radius_m):
        radius_cells = int(math.ceil(radius_m / self.resolution))
        offsets = []
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if math.hypot(dx * self.resolution, dy * self.resolution) <= radius_m + 1e-9:
                    offsets.append((dx, dy))
        return offsets

    def expanded_cells(self, cells):
        """Inflate dynamic obstacle cells with the same robot footprint."""
        expanded = set()
        for cell_x, cell_y in cells:
            for dx, dy in self._disk_offsets(self.inflation_radius):
                nx = cell_x + dx
                ny = cell_y + dy
                if self.in_bounds(nx, ny):
                    expanded.add((nx, ny))
        return expanded

    def world_to_cell(self, x, y):
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        dx = x - self.origin_x
        dy = y - self.origin_y
        local_x = self.cos_origin_yaw * dx + self.sin_origin_yaw * dy
        local_y = -self.sin_origin_yaw * dx + self.cos_origin_yaw * dy
        cell_x = int(math.floor(local_x / self.resolution))
        cell_y = int(math.floor(local_y / self.resolution))
        if not self.in_bounds(cell_x, cell_y):
            return None
        return cell_x, cell_y

    def cell_to_world(self, cell_x, cell_y):
        local_x = (cell_x + 0.5) * self.resolution
        local_y = (cell_y + 0.5) * self.resolution
        return (
            self.origin_x + self.cos_origin_yaw * local_x - self.sin_origin_yaw * local_y,
            self.origin_y + self.sin_origin_yaw * local_x + self.cos_origin_yaw * local_y,
        )

    def traversable(self, cell, dynamic_blocked=None):
        if cell is None:
            return False
        cell_x, cell_y = cell
        if not self.in_bounds(cell_x, cell_y):
            return False
        if self.inflated_blocked[self.index(cell_x, cell_y)]:
            return False
        return dynamic_blocked is None or cell not in dynamic_blocked

    def path_is_traversable(self, path, dynamic_blocked=None):
        dynamic_blocked = self.expanded_cells(dynamic_blocked or ())
        for x, y in path:
            if not self.traversable(self.world_to_cell(x, y), dynamic_blocked):
                return False
        return True

    def plan(self, start_world, goal_world, dynamic_cells=None):
        """Plan one collision-free A* route on the inflated grid."""
        start = self.world_to_cell(start_world[0], start_world[1])
        goal = self.world_to_cell(goal_world[0], goal_world[1])
        dynamic_blocked = self.expanded_cells(dynamic_cells or ())
        if not self.traversable(start, dynamic_blocked) or not self.traversable(goal, dynamic_blocked):
            return None
        if start == goal:
            return [start_world, goal_world]

        frontier = []
        heapq.heappush(frontier, (0.0, 0.0, start))
        came_from = {}
        g_score = {start: 0.0}
        expansions = 0

        while frontier:
            _, current_cost, current = heapq.heappop(frontier)
            if current_cost != g_score.get(current):
                continue
            if current == goal:
                return self._reconstruct_path(came_from, start, goal, start_world, goal_world)

            expansions += 1
            if expansions > self.max_expansions:
                return None

            for neighbor, step_cost in self._neighbors(current, dynamic_blocked):
                tentative_cost = current_cost + step_cost
                if tentative_cost >= g_score.get(neighbor, float("inf")):
                    continue
                came_from[neighbor] = current
                g_score[neighbor] = tentative_cost
                priority = tentative_cost + self._heuristic(neighbor, goal)
                heapq.heappush(frontier, (priority, tentative_cost, neighbor))

        return None

    def _neighbors(self, current, dynamic_blocked):
        cell_x, cell_y = current
        candidates = ((1, 0), (-1, 0), (0, 1), (0, -1))
        if self.allow_diagonal:
            candidates += ((1, 1), (1, -1), (-1, 1), (-1, -1))

        for dx, dy in candidates:
            neighbor = (cell_x + dx, cell_y + dy)
            if not self.traversable(neighbor, dynamic_blocked):
                continue
            if dx and dy:
                # Never allow a diagonal path to cut through two blocked cells.
                if not self.traversable((cell_x + dx, cell_y), dynamic_blocked):
                    continue
                if not self.traversable((cell_x, cell_y + dy), dynamic_blocked):
                    continue
                yield neighbor, math.sqrt(2.0)
            else:
                yield neighbor, 1.0

    @staticmethod
    def _heuristic(current, goal):
        dx = abs(current[0] - goal[0])
        dy = abs(current[1] - goal[1])
        return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)

    def _reconstruct_path(self, came_from, start, goal, start_world, goal_world):
        cells = [goal]
        current = goal
        while current != start:
            current = came_from[current]
            cells.append(current)
        cells.reverse()

        path = [start_world]
        path.extend(self.cell_to_world(cell_x, cell_y) for cell_x, cell_y in cells[1:-1])
        path.append(goal_world)
        return self._remove_duplicate_points(path)

    @staticmethod
    def _remove_duplicate_points(path):
        result = []
        for point in path:
            if not result or math.hypot(point[0] - result[-1][0], point[1] - result[-1][1]) > 1e-9:
                result.append(point)
        return result
