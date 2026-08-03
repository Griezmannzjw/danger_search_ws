"""Convert the official local Livox PointCloud into a planar LaserScan."""

import math

import numpy as np

from .vertical_estimation import quaternion_to_rpy


def transform_points(points, translation, quaternion):
    """Apply a rigid transform to an N-by-3 array."""
    points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    qx, qy, qz, qw = (float(value) for value in quaternion)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        raise ValueError("transform quaternion has zero norm")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    rotation = np.array(
        [
            [
                1 - 2 * (qy * qy + qz * qz),
                2 * (qx * qy - qz * qw),
                2 * (qx * qz + qy * qw),
            ],
            [
                2 * (qx * qy + qz * qw),
                1 - 2 * (qx * qx + qz * qz),
                2 * (qy * qz - qx * qw),
            ],
            [
                2 * (qx * qz - qy * qw),
                2 * (qy * qz + qx * qw),
                1 - 2 * (qx * qx + qy * qy),
            ],
        ],
        dtype=np.float64,
    )
    return points.dot(rotation.T) + np.asarray(translation, dtype=np.float64)


def gravity_level_points(points_base, world_from_base_quaternion):
    """Rotate body-frame points into a gravity-aligned frame preserving yaw."""
    points = np.asarray(points_base, dtype=np.float64).reshape((-1, 3))
    roll, pitch, yaw = quaternion_to_rpy(world_from_base_quaternion)
    points_world = transform_points(
        points, (0.0, 0.0, 0.0), world_from_base_quaternion
    )
    cosine, sine = math.cos(yaw), math.sin(yaw)
    world_to_heading = np.array(
        [[cosine, sine, 0.0], [-sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    return points_world.dot(world_to_heading.T), roll, pitch


def estimate_ground_clearance(points_level, config):
    """Estimate base-to-ground clearance from low nearby lidar returns.

    Returns ``None`` when the scan does not contain enough supporting points.
    The low percentile tolerates some wall returns while still detecting a
    collapsed robot whose lidar plane is nearly touching the floor.
    """
    points = np.asarray(points_level, dtype=np.float64).reshape((-1, 3))
    if points.size == 0:
        return None
    planar_range = np.hypot(points[:, 0], points[:, 1])
    candidates = points[
        np.isfinite(points).all(axis=1)
        & (points[:, 2] <= config.ground_candidate_max_z)
        & (planar_range >= config.ground_candidate_min_range)
        & (planar_range <= config.ground_candidate_max_range)
    ]
    if candidates.shape[0] < config.min_ground_candidate_points:
        return None
    return max(0.0, -float(np.percentile(candidates[:, 2], 10.0)))


def quaternion_multiply(left, right):
    lx, ly, lz, lw = (float(value) for value in left)
    rx, ry, rz, rw = (float(value) for value in right)
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_inverse(quaternion):
    qx, qy, qz, qw = (float(value) for value in quaternion)
    norm_squared = qx * qx + qy * qy + qz * qz + qw * qw
    if norm_squared <= 1e-12:
        raise ValueError("cannot invert zero quaternion")
    return (
        -qx / norm_squared,
        -qy / norm_squared,
        -qz / norm_squared,
        qw / norm_squared,
    )


def project_planar_scan(points_base, config):
    """Return nearest range in every angular bin; empty bins are +inf."""
    points = np.asarray(points_base, dtype=np.float64).reshape((-1, 3))
    ranges = np.full(config.bin_count, np.inf, dtype=np.float32)
    if points.size == 0:
        return ranges

    planar_range = np.hypot(points[:, 0], points[:, 1])
    angles = np.arctan2(points[:, 1], points[:, 0])
    inside_robot = (
        (points[:, 0] >= config.self_exclusion_min_x)
        & (points[:, 0] <= config.self_exclusion_max_x)
        & (np.abs(points[:, 1]) <= config.self_exclusion_half_width_y)
    )
    valid = (
        np.isfinite(points).all(axis=1)
        & ~inside_robot
        & (points[:, 2] >= config.min_height)
        & (points[:, 2] <= config.max_height)
        & (planar_range >= config.range_min)
        & (planar_range <= config.range_max)
        & (angles >= config.angle_min)
        & (angles <= config.angle_max)
    )
    if not np.any(valid):
        return ranges

    indices = np.floor(
        (angles[valid] - config.angle_min) / config.angle_increment
    ).astype(np.int64)
    indices = np.clip(indices, 0, config.bin_count - 1)
    valid_ranges = planar_range[valid].astype(np.float32)
    ranges = _nearest_supported_surface(
        indices,
        valid_ranges,
        config.bin_count,
        config.min_returns_per_bin,
        config.max_intra_bin_range_gap,
    )
    if config.enable_isolated_hit_filter:
        ranges = _reject_isolated_hits(ranges, config)
    return ranges


def _nearest_supported_surface(
    indices, point_ranges, bin_count, min_returns, max_range_gap
):
    """Select the nearest range cluster with enough support in each bin.

    A plain minimum is very sensitive to one short-range Livox outlier.  Points
    are sorted by angular bin and range, split into range-continuous clusters,
    and the median of the nearest sufficiently supported cluster is returned.
    """
    output = np.full(bin_count, np.inf, dtype=np.float32)
    if len(indices) == 0:
        return output

    order = np.lexsort((point_ranges, indices))
    sorted_indices = np.asarray(indices, dtype=np.int64)[order]
    sorted_ranges = np.asarray(point_ranges, dtype=np.float32)[order]
    group_start = 0
    while group_start < sorted_indices.size:
        group_end = group_start + 1
        bin_index = sorted_indices[group_start]
        while (
            group_end < sorted_indices.size
            and sorted_indices[group_end] == bin_index
        ):
            group_end += 1

        values = sorted_ranges[group_start:group_end]
        cluster_start = 0
        while cluster_start < values.size:
            cluster_end = cluster_start + 1
            while (
                cluster_end < values.size
                and values[cluster_end] - values[cluster_end - 1]
                <= max_range_gap
            ):
                cluster_end += 1
            if cluster_end - cluster_start >= min_returns:
                output[bin_index] = np.median(
                    values[cluster_start:cluster_end]
                )
                break
            cluster_start = cluster_end

        group_start = group_end
    return output


def _reject_isolated_hits(ranges, config):
    """Remove hits without a range-continuous neighbor in a local window."""
    filtered = np.asarray(ranges, dtype=np.float32).copy()
    finite = np.isfinite(filtered)
    support = np.zeros(filtered.size, dtype=np.int16)
    for offset in range(1, config.neighbor_window_bins + 1):
        left = filtered[:-offset]
        right = filtered[offset:]
        with np.errstate(invalid="ignore"):
            compatible = (
                np.isfinite(left)
                & np.isfinite(right)
                & (np.abs(left - right) <= config.max_neighbor_range_jump)
            )
        support[:-offset] += compatible
        support[offset:] += compatible
    filtered[finite & (support < config.min_neighbor_support)] = np.inf
    return filtered
