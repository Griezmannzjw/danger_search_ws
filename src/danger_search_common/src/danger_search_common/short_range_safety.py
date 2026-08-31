"""Geometry checks shared by short, straight safety-critical traversals."""

import math
from typing import NamedTuple

import numpy as np


class SweptFootprintHit(NamedTuple):
    """Nearest valid laser return inside a swept footprint."""

    distance_m: float
    x_m: float
    y_m: float


def _scan_points(ranges, angle_min, angle_increment, range_min, range_max):
    """Return finite scan points and their ranges in the current base frame."""
    samples = np.asarray(ranges, dtype=np.float64)
    if samples.ndim != 1 or samples.size == 0:
        return (
            np.empty(0, dtype=np.float64),
            np.empty(0, dtype=np.float64),
            np.empty(0, dtype=np.float64),
        )
    values = tuple(float(value) for value in (
        angle_min, angle_increment, range_min, range_max,
    ))
    if (not all(math.isfinite(value) for value in values)
            or angle_increment == 0.0 or range_min < 0.0
            or range_max <= range_min):
        raise ValueError("invalid scan geometry")
    indices = np.arange(samples.size, dtype=np.float64)
    angles = float(angle_min) + indices * float(angle_increment)
    finite = np.isfinite(samples)
    valid_samples = np.where(finite, samples, 0.0)
    finite &= (
        (valid_samples >= float(range_min))
        & (valid_samples <= float(range_max))
    )
    distances = samples[finite]
    return (
        distances,
        distances * np.cos(angles[finite]),
        distances * np.sin(angles[finite]),
    )


def _validate_footprint(footprint, margin):
    values = tuple(float(value) for value in (*footprint, margin))
    if not all(math.isfinite(value) for value in values) or margin < 0.0:
        raise ValueError("invalid swept-footprint geometry")
    min_x, max_x, min_y, max_y = values[:4]
    if min_x >= max_x or min_y >= max_y:
        raise ValueError("invalid footprint")
    return min_x, max_x, min_y, max_y


def _hits_from_mask(distances, xs, ys, mask):
    indices = np.flatnonzero(mask)
    if not indices.size:
        return ()
    ordered = indices[np.argsort(distances[indices])]
    return tuple(
        SweptFootprintHit(
            float(distances[index]), float(xs[index]), float(ys[index])
        )
        for index in ordered
    )


def swept_footprint_hits(
        ranges, angle_min, angle_increment, range_min, range_max,
        direction, travel_distance, footprint, margin=0.0):
    """Return all scan hits inside a straight swept footprint, nearest first.

    ``footprint`` is ``(min_x, max_x, min_y, max_y)`` in the robot base
    frame.  This assumes a zero-yaw, straight traversal: the swept volume is
    the conservative axis-aligned union of the footprint at its current pose
    and after the requested x translation.  Scan points inside the uninflated
    current body are ignored as residual self returns.
    """
    direction = float(direction)
    travel_distance = float(travel_distance)
    if (not math.isfinite(direction) or not math.isfinite(travel_distance)
            or travel_distance < 0.0 or direction == 0.0):
        raise ValueError("invalid swept-footprint geometry")
    min_x, max_x, min_y, max_y = _validate_footprint(footprint, margin)
    distances, xs, ys = _scan_points(
        ranges, angle_min, angle_increment, range_min, range_max
    )
    if not distances.size:
        return ()

    translation = math.copysign(travel_distance, direction)
    sweep_min_x = min(min_x, min_x + translation) - margin
    sweep_max_x = max(max_x, max_x + translation) + margin
    sweep_min_y = min_y - margin
    sweep_max_y = max_y + margin
    in_sweep = (
        (xs >= sweep_min_x) & (xs <= sweep_max_x)
        & (ys >= sweep_min_y) & (ys <= sweep_max_y)
    )
    in_body = (
        (xs >= min_x) & (xs <= max_x)
        & (ys >= min_y) & (ys <= max_y)
    )
    return _hits_from_mask(distances, xs, ys, in_sweep & ~in_body)


def swept_footprint_hit(
        ranges, angle_min, angle_increment, range_min, range_max,
        direction, travel_distance, footprint, margin=0.0):
    """Return the nearest scan hit inside a straight swept footprint."""
    hits = swept_footprint_hits(
        ranges, angle_min, angle_increment, range_min, range_max,
        direction, travel_distance, footprint, margin,
    )
    return None if not hits else hits[0]


def swept_arc_footprint_hit(
        ranges, angle_min, angle_increment, range_min, range_max,
        direction, travel_distance, curvature_rad_per_m, footprint,
        margin=0.0, sample_spacing_m=0.01):
    """Return the nearest hit swept by a constant-curvature footprint path.

    ``curvature_rad_per_m`` is angular velocity divided by signed linear
    velocity.  The footprint is sampled at no more than
    ``sample_spacing_m`` along the commanded arc, including both endpoints.
    A fresh laser scan is evaluated every control tick; this check covers the
    actual turning command that a straight axis-aligned union cannot model.
    """
    direction = float(direction)
    travel_distance = float(travel_distance)
    curvature = float(curvature_rad_per_m)
    sample_spacing = float(sample_spacing_m)
    if (not all(math.isfinite(value) for value in (
            direction, travel_distance, curvature, sample_spacing))
            or direction == 0.0 or travel_distance < 0.0
            or sample_spacing <= 0.0):
        raise ValueError("invalid arc swept-footprint geometry")
    min_x, max_x, min_y, max_y = _validate_footprint(footprint, margin)
    distances, xs, ys = _scan_points(
        ranges, angle_min, angle_increment, range_min, range_max
    )
    if not distances.size:
        return None
    in_body = (
        (xs >= min_x) & (xs <= max_x)
        & (ys >= min_y) & (ys <= max_y)
    )
    hit_mask = np.zeros(distances.size, dtype=bool)
    sample_count = max(1, int(math.ceil(travel_distance / sample_spacing)))
    signed_end = math.copysign(travel_distance, direction)
    for signed_distance in np.linspace(0.0, signed_end, sample_count + 1):
        yaw = curvature * signed_distance
        if abs(curvature) < 1e-9:
            center_x = signed_distance
            center_y = 0.0
        else:
            center_x = math.sin(yaw) / curvature
            center_y = (1.0 - math.cos(yaw)) / curvature
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        dx = xs - center_x
        dy = ys - center_y
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        hit_mask |= (
            (local_x >= min_x - margin)
            & (local_x <= max_x + margin)
            & (local_y >= min_y - margin)
            & (local_y <= max_y + margin)
        )
    hits = _hits_from_mask(distances, xs, ys, hit_mask & ~in_body)
    return None if not hits else hits[0]


def swept_footprint_obstacle(
        ranges, angle_min, angle_increment, range_min, range_max,
        direction, travel_distance, footprint, margin=0.0):
    """Return only the nearest range for the legacy safety-check API."""
    hit = swept_footprint_hit(
        ranges,
        angle_min,
        angle_increment,
        range_min,
        range_max,
        direction,
        travel_distance,
        footprint,
        margin,
    )
    return None if hit is None else hit.distance_m
