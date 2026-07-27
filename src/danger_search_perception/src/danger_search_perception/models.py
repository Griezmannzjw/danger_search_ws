"""Data models shared by the perception algorithm components."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ImageCandidate:
    contour: np.ndarray
    center_u: float
    center_v: float
    enclosing_radius_px: float
    circularity: float
    circle_fill_ratio: float
    aspect_ratio: float


@dataclass(frozen=True)
class GeometryResult:
    center_camera: np.ndarray
    radius_estimate_m: float
    valid_depth_ratio: float
    sphere_residual_m: float
    sphere_inlier_ratio: float
    plane_residual_m: float
    radius_error_m: float
