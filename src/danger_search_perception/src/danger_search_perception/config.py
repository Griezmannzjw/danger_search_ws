"""Typed configuration objects for the perception pipeline."""

from dataclasses import dataclass


@dataclass(frozen=True)
class ColorDetectionConfig:
    red_h_low_1: int = 0
    red_h_high_1: int = 10
    red_h_low_2: int = 170
    red_h_high_2: int = 180
    red_s_low: int = 120
    red_v_low: int = 80
    morph_open_ksize: int = 3
    morph_close_ksize: int = 5
    min_blob_area: float = 50.0
    max_blob_area: float = 100000.0
    min_circularity: float = 0.65
    min_circle_fill_ratio: float = 0.72
    min_aspect_ratio: float = 0.65
    max_aspect_ratio: float = 1.50
    reject_border_candidates: bool = True
    border_margin_px: int = 2


@dataclass(frozen=True)
class GeometryConfig:
    min_depth_m: float = 0.4
    max_depth_m: float = 8.0
    min_valid_depth_ratio: float = 0.45
    depth_mad_scale: float = 3.5
    max_points_per_candidate: int = 1500
    expected_radius_m: float = 0.15
    radius_tolerance_m: float = 0.07
    sphere_inlier_threshold_m: float = 0.035
    max_sphere_residual_m: float = 0.035
    min_sphere_inlier_ratio: float = 0.55
    enable_plane_rejection: bool = True
    max_plane_residual_m: float = 0.006
    plane_vs_sphere_ratio: float = 0.45


@dataclass(frozen=True)
class PipelineConfig:
    confidence_threshold: float = 0.60
    tf_timeout_s: float = 0.10
    publish_empty_array: bool = True
