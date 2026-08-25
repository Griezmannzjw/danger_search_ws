"""Typed configuration objects for the perception pipeline."""

from dataclasses import dataclass
import math


def _require(condition, message):
    if not condition:
        raise ValueError(message)


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

    def __post_init__(self):
        for name in (
            "red_h_low_1", "red_h_high_1", "red_h_low_2", "red_h_high_2"
        ):
            _require(
                0 <= getattr(self, name) <= 180,
                name + " must be in [0, 180]",
            )
        _require(
            self.red_h_low_1 <= self.red_h_high_1,
            "first hue range is reversed",
        )
        _require(
            self.red_h_low_2 <= self.red_h_high_2,
            "second hue range is reversed",
        )
        _require(0 <= self.red_s_low <= 255, "red_s_low must be in [0, 255]")
        _require(0 <= self.red_v_low <= 255, "red_v_low must be in [0, 255]")
        _require(self.morph_open_ksize > 0, "morph_open_ksize must be positive")
        _require(self.morph_close_ksize > 0, "morph_close_ksize must be positive")
        _require(
            0.0 <= self.min_blob_area < self.max_blob_area,
            "blob area range is invalid",
        )
        _require(
            0.0 <= self.min_circularity <= 1.0,
            "min_circularity must be in [0, 1]",
        )
        _require(
            0.0 <= self.min_circle_fill_ratio <= 1.0,
            "min_circle_fill_ratio must be in [0, 1]",
        )
        _require(
            0.0 < self.min_aspect_ratio <= self.max_aspect_ratio,
            "aspect ratio range is invalid",
        )
        _require(self.border_margin_px >= 0, "border_margin_px cannot be negative")


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

    def __post_init__(self):
        _require(
            0.0 < self.min_depth_m < self.max_depth_m,
            "depth range is invalid",
        )
        _require(
            0.0 <= self.min_valid_depth_ratio <= 1.0,
            "min_valid_depth_ratio must be in [0, 1]",
        )
        _require(self.depth_mad_scale > 0.0, "depth_mad_scale must be positive")
        _require(
            self.max_points_per_candidate >= 12,
            "max_points_per_candidate must be at least 12",
        )
        _require(self.expected_radius_m > 0.0, "expected_radius_m must be positive")
        _require(self.radius_tolerance_m >= 0.0, "radius_tolerance_m cannot be negative")
        _require(
            self.sphere_inlier_threshold_m > 0.0,
            "sphere_inlier_threshold_m must be positive",
        )
        _require(
            self.max_sphere_residual_m > 0.0,
            "max_sphere_residual_m must be positive",
        )
        _require(
            0.0 <= self.min_sphere_inlier_ratio <= 1.0,
            "min_sphere_inlier_ratio must be in [0, 1]",
        )
        _require(self.max_plane_residual_m >= 0.0, "max_plane_residual_m cannot be negative")
        _require(self.plane_vs_sphere_ratio >= 0.0, "plane_vs_sphere_ratio cannot be negative")


@dataclass(frozen=True)
class PipelineConfig:
    confidence_threshold: float = 0.60
    tf_timeout_s: float = 0.10
    publish_empty_array: bool = True
    reliable_min_range: float = 0.4
    reliable_max_range: float = 5.0

    def __post_init__(self):
        _require(
            0.0 <= self.confidence_threshold <= 1.0,
            "confidence_threshold must be in [0, 1]",
        )
        _require(self.tf_timeout_s >= 0.0, "tf_timeout_s cannot be negative")
        _require(
            0.0 <= self.reliable_min_range < self.reliable_max_range,
            "reliable range is invalid",
        )

    def is_reliable_range(self, center_camera):
        squared_range = sum(float(value) ** 2 for value in center_camera)
        return (
            self.reliable_min_range ** 2
            <= squared_range
            <= self.reliable_max_range ** 2
        )


@dataclass(frozen=True)
class TrackingConfig:
    """Association and lifetime policy for static danger-source tracks."""

    association_distance_m: float = 0.40
    confirmation_hits: int = 3
    tentative_timeout_s: float = 1.0
    # Zero keeps confirmed tracks for the lifetime of the detector process.
    confirmed_timeout_s: float = 0.0
    initial_position_variance_m2: float = 0.0064
    minimum_position_variance_m2: float = 0.0004

    def __post_init__(self):
        _require(
            math.isfinite(float(self.association_distance_m))
            and self.association_distance_m > 0.0,
            "association_distance_m must be positive and finite",
        )
        _require(
            isinstance(self.confirmation_hits, int)
            and not isinstance(self.confirmation_hits, bool)
            and self.confirmation_hits >= 1,
            "confirmation_hits must be a positive integer",
        )
        _require(
            math.isfinite(float(self.tentative_timeout_s))
            and self.tentative_timeout_s > 0.0,
            "tentative_timeout_s must be positive and finite",
        )
        _require(
            math.isfinite(float(self.confirmed_timeout_s))
            and self.confirmed_timeout_s >= 0.0,
            "confirmed_timeout_s must be finite and cannot be negative",
        )
        _require(
            math.isfinite(float(self.initial_position_variance_m2))
            and self.initial_position_variance_m2 > 0.0,
            "initial_position_variance_m2 must be positive and finite",
        )
        _require(
            math.isfinite(float(self.minimum_position_variance_m2))
            and 0.0 < self.minimum_position_variance_m2
            <= self.initial_position_variance_m2,
            "minimum_position_variance_m2 must be positive and no greater "
            "than initial_position_variance_m2",
        )
