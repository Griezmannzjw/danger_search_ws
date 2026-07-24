"""Depth processing, sphere fitting and red-plane rejection."""

from typing import Optional

import numpy as np

from .config import GeometryConfig
from .models import GeometryResult, ImageCandidate


class DepthGeometryValidator:
    """Estimate a known-radius sphere center from a candidate depth ROI."""

    def __init__(self, config: GeometryConfig):
        self.config = config

    @staticmethod
    def depth_to_metres(depth: np.ndarray, encoding: str) -> np.ndarray:
        depth = np.asarray(depth)
        encoding_upper = (encoding or "").upper()
        if encoding_upper in ("16UC1", "MONO16"):
            return depth.astype(np.float32) * 0.001
        if encoding_upper == "32FC1":
            return depth.astype(np.float32)
        if np.issubdtype(depth.dtype, np.uint16):
            return depth.astype(np.float32) * 0.001
        if np.issubdtype(depth.dtype, np.floating):
            return depth.astype(np.float32)
        raise ValueError("unsupported depth encoding: {}".format(encoding))

    def validate(
        self,
        candidate: ImageCandidate,
        inner_mask: np.ndarray,
        depth_m: np.ndarray,
        camera_model,
    ) -> Optional[GeometryResult]:
        rows, cols = np.nonzero(inner_mask)
        if rows.size == 0:
            return None

        depths = depth_m[rows, cols]
        cfg = self.config
        valid = (
            np.isfinite(depths)
            & (depths >= cfg.min_depth_m)
            & (depths <= cfg.max_depth_m)
        )
        valid_ratio = float(np.count_nonzero(valid)) / float(depths.size)
        if valid_ratio < cfg.min_valid_depth_ratio:
            return None

        rows, cols, depths = rows[valid], cols[valid], depths[valid]
        rows, cols, depths = self._remove_depth_outliers(rows, cols, depths)
        if depths.size < 12:
            return None

        rows, cols, depths = self._subsample(rows, cols, depths)
        points = self._pixels_to_points(cols, rows, depths, camera_model)
        sphere_fit = self._fit_known_radius_sphere(
            points, candidate.center_u, candidate.center_v, camera_model
        )
        if sphere_fit is None:
            return None

        center, sphere_residual, sphere_inlier_ratio = sphere_fit
        if sphere_residual > cfg.max_sphere_residual_m:
            return None
        if sphere_inlier_ratio < cfg.min_sphere_inlier_ratio:
            return None

        plane_residual = self._fit_plane_residual(points)
        if self._looks_like_plane(plane_residual, sphere_residual):
            return None

        focal_mean = 0.5 * (camera_model.fx() + camera_model.fy())
        radius_estimate = (
            candidate.enclosing_radius_px
            * float(center[2])
            / max(focal_mean, 1e-6)
        )
        radius_error = abs(radius_estimate - cfg.expected_radius_m)
        if radius_error > cfg.radius_tolerance_m:
            return None

        return GeometryResult(
            center_camera=center,
            radius_estimate_m=float(radius_estimate),
            valid_depth_ratio=valid_ratio,
            sphere_residual_m=sphere_residual,
            sphere_inlier_ratio=sphere_inlier_ratio,
            plane_residual_m=plane_residual,
            radius_error_m=float(radius_error),
        )
    def _remove_depth_outliers(self, rows, cols, depths):
        median = float(np.median(depths))
        mad = float(np.median(np.abs(depths - median)))
        tolerance = max(
            0.015, self.config.depth_mad_scale * 1.4826 * mad
        )
        keep = np.abs(depths - median) <= tolerance
        return rows[keep], cols[keep], depths[keep]

    def _subsample(self, rows, cols, depths):
        maximum = self.config.max_points_per_candidate
        if depths.size <= maximum:
            return rows, cols, depths
        indices = np.linspace(0, depths.size - 1, maximum).astype(np.int64)
        return rows[indices], cols[indices], depths[indices]

    @staticmethod
    def _pixels_to_points(cols, rows, depths, camera_model):
        fx, fy = float(camera_model.fx()), float(camera_model.fy())
        cx, cy = float(camera_model.cx()), float(camera_model.cy())
        x = (cols.astype(np.float64) - cx) * depths / fx
        y = (rows.astype(np.float64) - cy) * depths / fy
        return np.column_stack((x, y, depths)).astype(np.float64)

    def _fit_known_radius_sphere(
        self, points, center_u, center_v, camera_model
    ):
        ray = np.asarray(
            camera_model.projectPixelTo3dRay((center_u, center_v)),
            dtype=np.float64,
        )
        ray_norm = np.linalg.norm(ray)
        if ray_norm <= 1e-9:
            return None
        ray /= ray_norm

        projections = points.dot(ray)
        base = float(np.median(projections))
        radius = self.config.expected_radius_m
        low = max(self.config.min_depth_m, base - 0.15 * radius)
        high = min(self.config.max_depth_m + radius, base + 1.8 * radius)
        if high <= low:
            return None

        best_t, best_loss = None, float("inf")
        for sample_count in (81, 61):
            for distance in np.linspace(low, high, sample_count):
                center = float(distance) * ray
                errors = np.abs(
                    np.linalg.norm(points - center, axis=1) - radius
                )
                loss = float(np.median(errors))
                if loss < best_loss:
                    best_t, best_loss = float(distance), loss
            step = (high - low) / max(sample_count - 1, 1)
            low = max(self.config.min_depth_m, best_t - 2.0 * step)
            high = min(
                self.config.max_depth_m + radius, best_t + 2.0 * step
            )

        center = best_t * ray
        errors = np.abs(np.linalg.norm(points - center, axis=1) - radius)
        residual = float(np.median(errors))
        inlier_ratio = float(
            np.mean(errors <= self.config.sphere_inlier_threshold_m)
        )
        return center, residual, inlier_ratio

    @staticmethod
    def _fit_plane_residual(points):
        centered = points - np.mean(points, axis=0)
        try:
            _, _, vh = np.linalg.svd(centered, full_matrices=False)
        except np.linalg.LinAlgError:
            return float("inf")
        distances = np.abs(centered.dot(vh[-1]))
        return float(np.median(distances))

    def _looks_like_plane(self, plane_residual, sphere_residual):
        cfg = self.config
        return (
            cfg.enable_plane_rejection
            and plane_residual < cfg.max_plane_residual_m
            and plane_residual
            < cfg.plane_vs_sphere_ratio * max(sphere_residual, 1e-5)
        )
