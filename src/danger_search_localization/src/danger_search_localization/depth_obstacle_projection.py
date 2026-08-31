"""Ground-relative depth-camera projection for near-field navigation obstacles."""

from dataclasses import dataclass
import math

import numpy as np

from .config import _require


@dataclass(frozen=True)
class DepthObstacleProjectionConfig:
    image_stride: int = 4
    min_depth_m: float = 0.40
    max_depth_m: float = 3.00
    floor_search_min_z_m: float = -0.65
    floor_search_max_z_m: float = -0.08
    floor_histogram_bin_m: float = 0.02
    floor_min_support_points: int = 40
    floor_max_jump_m: float = 0.08
    floor_cache_timeout_s: float = 2.0
    fallback_floor_z_m: float = -0.30
    min_obstacle_height_m: float = 0.06
    max_obstacle_height_m: float = 1.20

    def __post_init__(self):
        _require(self.image_stride >= 1, "depth image stride must be positive")
        _require(
            0.0 < self.min_depth_m < self.max_depth_m,
            "depth obstacle range is invalid",
        )
        _require(
            self.floor_search_min_z_m < self.floor_search_max_z_m < 0.0,
            "floor search height range is invalid",
        )
        _require(
            self.floor_histogram_bin_m > 0.0,
            "floor histogram bin must be positive",
        )
        _require(
            self.floor_min_support_points >= 3,
            "floor support count is too small",
        )
        _require(self.floor_max_jump_m > 0.0, "floor jump limit must be positive")
        _require(
            self.floor_cache_timeout_s > 0.0,
            "floor cache timeout must be positive",
        )
        _require(
            self.floor_search_min_z_m
            <= self.fallback_floor_z_m
            <= self.floor_search_max_z_m,
            "fallback floor height is outside the search range",
        )
        _require(
            0.0 < self.min_obstacle_height_m < self.max_obstacle_height_m,
            "ground-relative obstacle height range is invalid",
        )


def depth_image_to_optical_points(depth_image, intrinsics, config):
    """Convert a strided metric depth image into ROS optical-frame XYZ points."""
    depth = np.asarray(depth_image, dtype=np.float64)
    if depth.ndim != 2:
        raise ValueError("depth image must be two-dimensional")
    fx, fy, cx, cy = (float(value) for value in intrinsics)
    if not all(math.isfinite(value) for value in (fx, fy, cx, cy)):
        raise ValueError("camera intrinsics contain non-finite values")
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera focal length must be positive")

    rows = np.arange(0, depth.shape[0], config.image_stride, dtype=np.int64)
    columns = np.arange(0, depth.shape[1], config.image_stride, dtype=np.int64)
    vv, uu = np.meshgrid(rows, columns, indexing="ij")
    sampled = depth[vv, uu]
    valid = (
        np.isfinite(sampled)
        & (sampled >= config.min_depth_m)
        & (sampled <= config.max_depth_m)
    )
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float64)

    z = sampled[valid]
    x = (uu[valid].astype(np.float64) - cx) * z / fx
    y = (vv[valid].astype(np.float64) - cy) * z / fy
    return np.column_stack((x, y, z))


def estimate_floor_z(points_base, config, previous_floor_z=None):
    """Estimate the horizontal floor mode in gravity-levelled base coordinates."""
    points = np.asarray(points_base, dtype=np.float64).reshape((-1, 3))
    if points.size == 0:
        return None
    planar_range = np.hypot(points[:, 0], points[:, 1])
    valid = (
        np.isfinite(points).all(axis=1)
        & (planar_range >= config.min_depth_m)
        & (planar_range <= config.max_depth_m)
        & (points[:, 2] >= config.floor_search_min_z_m)
        & (points[:, 2] <= config.floor_search_max_z_m)
    )
    candidates = points[valid, 2]
    if candidates.size < config.floor_min_support_points:
        return None

    edges = np.arange(
        config.floor_search_min_z_m,
        config.floor_search_max_z_m + config.floor_histogram_bin_m,
        config.floor_histogram_bin_m,
    )
    if edges.size < 2:
        return None
    counts, edges = np.histogram(candidates, bins=edges)
    winner = int(np.argmax(counts))
    if int(counts[winner]) < config.floor_min_support_points:
        return None
    lower = edges[winner]
    upper = edges[winner + 1]
    support = candidates[(candidates >= lower) & (candidates <= upper)]
    estimate = float(np.median(support))
    if not math.isfinite(estimate):
        return None
    if (
        previous_floor_z is not None
        and math.isfinite(float(previous_floor_z))
        and abs(estimate - float(previous_floor_z)) > config.floor_max_jump_m
    ):
        return None
    return estimate


def select_ground_relative_obstacles(points_base, floor_z, config):
    """Keep points within the robot-collision height band above the floor."""
    points = np.asarray(points_base, dtype=np.float64).reshape((-1, 3))
    floor_z = float(floor_z)
    if not math.isfinite(floor_z):
        raise ValueError("floor height must be finite")
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    planar_range = np.hypot(points[:, 0], points[:, 1])
    relative_height = points[:, 2] - floor_z
    valid = (
        np.isfinite(points).all(axis=1)
        & (planar_range >= config.min_depth_m)
        & (planar_range <= config.max_depth_m)
        & (relative_height >= config.min_obstacle_height_m)
        & (relative_height <= config.max_obstacle_height_m)
    )
    return points[valid]

