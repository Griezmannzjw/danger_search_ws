"""Confidence scoring for accepted danger source observations."""

import numpy as np

from .config import GeometryConfig
from .models import GeometryResult, ImageCandidate


def observation_confidence(
    candidate: ImageCandidate,
    geometry: GeometryResult,
    config: GeometryConfig,
) -> float:
    shape_score = np.clip(
        0.5 * candidate.circularity + 0.5 * candidate.circle_fill_ratio,
        0.0,
        1.0,
    )
    depth_score = np.clip(geometry.valid_depth_ratio, 0.0, 1.0)
    residual_score = np.clip(
        1.0
        - geometry.sphere_residual_m
        / max(config.max_sphere_residual_m, 1e-6),
        0.0,
        1.0,
    )
    radius_score = np.clip(
        1.0
        - geometry.radius_error_m / max(config.radius_tolerance_m, 1e-6),
        0.0,
        1.0,
    )
    return float(
        0.25 * shape_score
        + 0.15 * depth_score
        + 0.30 * residual_score
        + 0.20 * geometry.sphere_inlier_ratio
        + 0.10 * radius_score
    )
