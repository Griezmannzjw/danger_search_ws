"""Small deterministic occupancy mapper driven by trusted planar odometry."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class OccupancyMappingConfig:
    resolution: float = 0.05
    size: int = 1024
    start_x: float = 0.5
    start_y: float = 0.5
    max_rays: int = 360
    free_update: int = 1
    occupied_update: int = 4
    min_score: int = -20
    max_score: int = 20
    occupied_score: int = 2
    clear_radius_m: float = 0.35

    def __post_init__(self):
        if not math.isfinite(self.resolution) or self.resolution <= 0.0:
            raise ValueError("map resolution must be positive")
        if self.size < 16:
            raise ValueError("map size is too small")
        if not 0.0 <= self.start_x <= 1.0 or not 0.0 <= self.start_y <= 1.0:
            raise ValueError("map start fractions must be within [0, 1]")
        if self.max_rays < 1 or self.free_update < 1 or self.occupied_update < 1:
            raise ValueError("map update parameters must be positive")
        if not self.min_score < 0 < self.max_score:
            raise ValueError("map score limits must straddle zero")
        if not 0 < self.occupied_score <= self.max_score:
            raise ValueError("occupied score is invalid")
        if self.clear_radius_m < 0.0:
            raise ValueError("clear radius cannot be negative")


class OccupancyMapperCore:
    """Raytrace sparse scans into a fixed single-floor P0 map."""

    def __init__(self, config=None):
        self.config = config or OccupancyMappingConfig()
        shape = (self.config.size, self.config.size)
        self.scores = np.zeros(shape, dtype=np.int16)
        self.observed = np.zeros(shape, dtype=bool)
        # Put the task origin at a cell center. OccupancyGrid.resolution is
        # serialized as float32; placing (0, 0) exactly on a cell boundary can
        # otherwise make the mapper and a consumer choose adjacent cells.
        self.origin_x = -(
            self.config.start_x * self.config.size + 0.5
        ) * self.config.resolution
        self.origin_y = -(
            self.config.start_y * self.config.size + 0.5
        ) * self.config.resolution
        self.update_count = 0

    def update(self, pose, scan):
        x, y, yaw = (float(value) for value in pose)
        values = (x, y, yaw, scan.angle_min, scan.angle_increment)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("map pose or scan geometry is invalid")
        start = self.world_to_cell(x, y)
        if start is None:
            raise ValueError("robot pose is outside the configured map")

        ranges = np.asarray(scan.ranges, dtype=np.float64)
        valid = np.flatnonzero(
            np.isfinite(ranges)
            & (ranges >= float(scan.range_min))
            & (ranges <= float(scan.range_max))
        )
        if valid.size == 0:
            return False
        if valid.size > self.config.max_rays:
            stride = valid.size / float(self.config.max_rays)
            valid = np.asarray(
                [valid[int(index * stride)] for index in range(self.config.max_rays)],
                dtype=np.int64,
            )

        for index in valid:
            angle = yaw + float(scan.angle_min) + index * float(scan.angle_increment)
            distance = float(ranges[index])
            endpoint = self.world_to_cell(
                x + distance * math.cos(angle),
                y + distance * math.sin(angle),
            )
            if endpoint is None:
                continue
            cells = self._line_cells(start[0], start[1], endpoint[0], endpoint[1])
            if len(cells) > 1:
                for cell_x, cell_y in cells[:-1]:
                    self._add_score(cell_x, cell_y, -self.config.free_update)
            self._add_score(endpoint[0], endpoint[1], self.config.occupied_update)

        self._clear_robot_footprint(start)
        self.update_count += 1
        return True

    def occupancy_data(self):
        output = np.full(self.scores.shape, -1, dtype=np.int8)
        output[self.observed & (self.scores < self.config.occupied_score)] = 0
        output[self.observed & (self.scores >= self.config.occupied_score)] = 100
        return output.reshape(-1).tolist()

    def world_to_cell(self, x, y):
        cell_x = int(math.floor((float(x) - self.origin_x) / self.config.resolution))
        cell_y = int(math.floor((float(y) - self.origin_y) / self.config.resolution))
        if 0 <= cell_x < self.config.size and 0 <= cell_y < self.config.size:
            return cell_x, cell_y
        return None

    def _add_score(self, cell_x, cell_y, delta):
        value = int(self.scores[cell_y, cell_x]) + int(delta)
        self.scores[cell_y, cell_x] = min(
            self.config.max_score, max(self.config.min_score, value)
        )
        self.observed[cell_y, cell_x] = True

    def _clear_robot_footprint(self, center):
        radius = int(math.ceil(self.config.clear_radius_m / self.config.resolution))
        radius_squared = radius * radius
        for offset_y in range(-radius, radius + 1):
            for offset_x in range(-radius, radius + 1):
                if offset_x * offset_x + offset_y * offset_y > radius_squared:
                    continue
                cell_x = center[0] + offset_x
                cell_y = center[1] + offset_y
                if 0 <= cell_x < self.config.size and 0 <= cell_y < self.config.size:
                    self.scores[cell_y, cell_x] = self.config.min_score
                    self.observed[cell_y, cell_x] = True

    @staticmethod
    def _line_cells(start_x, start_y, end_x, end_y):
        """Integer Bresenham line including both endpoints."""
        cells = []
        x, y = int(start_x), int(start_y)
        dx = abs(int(end_x) - x)
        dy = -abs(int(end_y) - y)
        step_x = 1 if x < int(end_x) else -1
        step_y = 1 if y < int(end_y) else -1
        error = dx + dy
        while True:
            cells.append((x, y))
            if x == int(end_x) and y == int(end_y):
                return cells
            doubled = 2 * error
            if doubled >= dy:
                error += dy
                x += step_x
            if doubled <= dx:
                error += dx
                y += step_y
