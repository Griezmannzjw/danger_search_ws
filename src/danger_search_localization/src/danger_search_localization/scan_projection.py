"""Convert the official local Livox PointCloud into a planar LaserScan."""

from collections import deque
import math

import numpy as np

from .vertical_estimation import quaternion_to_rpy


class TimedScanAccumulator:
    """Bound sparse scan history by frame count and monotonic sensor time."""

    def __init__(self, max_frames, max_age_s):
        if int(max_frames) < 1:
            raise ValueError("maximum scan history length must be positive")
        if not math.isfinite(float(max_age_s)) or float(max_age_s) <= 0.0:
            raise ValueError("maximum scan history age must be positive")
        self.max_age_s = float(max_age_s)
        self._history = deque(maxlen=int(max_frames))
        self._last_stamp_s = None

    def add(self, stamp_s, ranges):
        stamp_s = float(stamp_s)
        scan = np.asarray(ranges, dtype=np.float32)
        if not math.isfinite(stamp_s):
            raise ValueError("scan timestamp must be finite")
        if scan.ndim != 1:
            raise ValueError("projected scan must be one-dimensional")

        reset = (
            self._last_stamp_s is not None
            and (
                stamp_s <= self._last_stamp_s
                or stamp_s - self._last_stamp_s > self.max_age_s
            )
        )
        if reset:
            self._history.clear()
        elif self._history and scan.shape != self._history[-1][1].shape:
            self._history.clear()
            reset = True

        self._history.append((stamp_s, scan.copy()))
        self._last_stamp_s = stamp_s
        while (
            self._history
            and stamp_s - self._history[0][0] > self.max_age_s
        ):
            self._history.popleft()
        return reset

    @property
    def scans(self):
        return tuple(scan for _, scan in self._history)

    def __len__(self):
        return len(self._history)


class PoseCompensatedPointAccumulator:
    """Keep a short point window and express it in the newest base frame."""

    def __init__(self, max_frames, max_age_s):
        if int(max_frames) < 1:
            raise ValueError("maximum point history length must be positive")
        if not math.isfinite(float(max_age_s)) or float(max_age_s) <= 0.0:
            raise ValueError("maximum point history age must be positive")
        self.max_frames = int(max_frames)
        self.max_age_s = float(max_age_s)
        self._history = deque()
        self._last_stamp_s = None

    def add(self, stamp_s, pose, points):
        stamp_s = float(stamp_s)
        pose = tuple(float(value) for value in pose)
        points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        if not math.isfinite(stamp_s) or not all(math.isfinite(v) for v in pose):
            raise ValueError("point observation timestamp and pose must be finite")
        if len(pose) != 3 or not np.isfinite(points).all():
            raise ValueError("point observation is invalid")
        reset = (
            self._last_stamp_s is not None
            and (
                stamp_s <= self._last_stamp_s
                or stamp_s - self._last_stamp_s > self.max_age_s
            )
        )
        if reset:
            self._history.clear()
        self._history.append((stamp_s, pose, points.copy()))
        self._last_stamp_s = stamp_s
        while len(self._history) > self.max_frames:
            self._history.popleft()
        while self._history and stamp_s - self._history[0][0] > self.max_age_s:
            self._history.popleft()
        return reset

    def points_in_latest_frame(self):
        if not self._history:
            return np.empty((0, 3), dtype=np.float64)
        target_pose = self._history[-1][1]
        transformed = [
            transform_points_between_planar_poses(points, pose, target_pose)
            for _, pose, points in self._history
        ]
        return np.concatenate(transformed, axis=0)

    def __len__(self):
        return len(self._history)


def interpolate_planar_pose(start, end, ratio):
    """Interpolate an odometry pose while respecting yaw wrapping."""
    ratio = min(1.0, max(0.0, float(ratio)))
    sx, sy, syaw = (float(value) for value in start)
    ex, ey, eyaw = (float(value) for value in end)
    yaw_delta = math.atan2(math.sin(eyaw - syaw), math.cos(eyaw - syaw))
    return (
        sx + ratio * (ex - sx),
        sy + ratio * (ey - sy),
        math.atan2(
            math.sin(syaw + ratio * yaw_delta),
            math.cos(syaw + ratio * yaw_delta),
        ),
    )


def transform_points_between_planar_poses(points, source_pose, target_pose):
    """Transform source-base points into target-base coordinates via odom."""
    points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    sx, sy, syaw = (float(value) for value in source_pose)
    tx, ty, tyaw = (float(value) for value in target_pose)
    source_cosine, source_sine = math.cos(syaw), math.sin(syaw)
    target_cosine, target_sine = math.cos(tyaw), math.sin(tyaw)
    odom_x = sx + source_cosine * points[:, 0] - source_sine * points[:, 1]
    odom_y = sy + source_sine * points[:, 0] + source_cosine * points[:, 1]
    delta_x = odom_x - tx
    delta_y = odom_y - ty
    output = points.copy()
    output[:, 0] = target_cosine * delta_x + target_sine * delta_y
    output[:, 1] = -target_sine * delta_x + target_cosine * delta_y
    return output


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


def merge_scan_history(range_history, min_samples_per_bin):
    """Merge sparse projected scans with a per-bin median.

    A Livox frame does not cover every direction. A range is retained only
    when enough frames observed that direction, so an isolated one-frame hit
    cannot become a map obstacle.
    """
    if min_samples_per_bin < 1:
        raise ValueError("min_samples_per_bin must be at least one")
    if len(range_history) == 0:
        return np.empty(0, dtype=np.float32)

    history = np.asarray(range_history, dtype=np.float32)
    if history.ndim != 2:
        raise ValueError("range history must contain one-dimensional scans")
    if min_samples_per_bin > history.shape[0]:
        return np.full(history.shape[1], np.inf, dtype=np.float32)

    output = np.full(history.shape[1], np.inf, dtype=np.float32)
    finite = np.isfinite(history)
    supported_indices = np.flatnonzero(
        np.sum(finite, axis=0) >= min_samples_per_bin
    )
    if supported_indices.size:
        supported = history[:, supported_indices]
        supported_finite = finite[:, supported_indices]
        supported = np.where(supported_finite, supported, np.nan)
        output[supported_indices] = np.nanmedian(supported, axis=0)
    return output


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
    full_circle = (
        config.angle_max - config.angle_min
        >= 2.0 * math.pi - 1.5 * config.angle_increment
    )
    for offset in range(1, config.neighbor_window_bins + 1):
        if full_circle:
            neighbor = np.roll(filtered, offset)
            with np.errstate(invalid="ignore"):
                compatible = (
                    finite
                    & np.isfinite(neighbor)
                    & (
                        np.abs(filtered - neighbor)
                        <= config.max_neighbor_range_jump
                    )
                )
            support += compatible
            support += np.roll(compatible, -offset)
        else:
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
