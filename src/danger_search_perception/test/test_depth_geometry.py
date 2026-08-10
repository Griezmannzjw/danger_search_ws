#!/usr/bin/env python3

import unittest

import cv2
import numpy as np

from danger_search_perception.color_detector import RedCandidateDetector
from danger_search_perception.config import (
    ColorDetectionConfig,
    GeometryConfig,
)
from danger_search_perception.depth_geometry import DepthGeometryValidator


class FakeCameraModel:
    def __init__(self, fx=400.0, fy=400.0, cx=320.0, cy=240.0):
        self._fx, self._fy = fx, fy
        self._cx, self._cy = cx, cy

    def fx(self):
        return self._fx

    def fy(self):
        return self._fy

    def cx(self):
        return self._cx

    def cy(self):
        return self._cy

    def projectPixelTo3dRay(self, pixel):
        u, v = pixel
        return (
            (u - self._cx) / self._fx,
            (v - self._cy) / self._fy,
            1.0,
        )


class TestDepthGeometryValidator(unittest.TestCase):
    def setUp(self):
        self.camera = FakeCameraModel()
        self.color_detector = RedCandidateDetector(ColorDetectionConfig())
        self.validator = DepthGeometryValidator(GeometryConfig())

    def test_depth_unit_conversion(self):
        depth_mm = np.array([[1000, 2500]], dtype=np.uint16)
        result = self.validator.depth_to_metres(depth_mm, "16UC1")
        np.testing.assert_allclose(result, [[1.0, 2.5]])

    def test_known_sphere_center_is_recovered(self):
        center_z = 2.0
        radius = 0.15
        image, depth = self._synthetic_sphere(center_z, radius)
        _, candidates = self.color_detector.detect(image)
        self.assertEqual(len(candidates), 1)

        candidate = candidates[0]
        inner_mask = self.color_detector.make_inner_mask(
            candidate, image.shape
        )
        result = self.validator.validate(
            candidate, inner_mask, depth, self.camera
        )

        self.assertIsNotNone(result)
        self.assertAlmostEqual(result.center_camera[0], 0.0, delta=0.02)
        self.assertAlmostEqual(result.center_camera[1], 0.0, delta=0.02)
        self.assertAlmostEqual(result.center_camera[2], center_z, delta=0.04)

    def test_missing_depth_is_rejected(self):
        image, depth = self._synthetic_sphere(2.0, 0.15)
        depth.fill(np.nan)

        self.assertIsNone(self._validate_first_candidate(image, depth))

    def test_flat_red_circle_is_rejected_as_plane(self):
        image, depth = self._synthetic_sphere(2.0, 0.15)
        depth[np.isfinite(depth)] = 2.0

        self.assertIsNone(self._validate_first_candidate(image, depth))

    def test_wrong_radius_sphere_is_rejected(self):
        image, depth = self._synthetic_sphere(2.0, 0.30)

        self.assertIsNone(self._validate_first_candidate(image, depth))

    def test_invalid_camera_intrinsics_are_rejected(self):
        image, depth = self._synthetic_sphere(2.0, 0.15)
        self.camera = FakeCameraModel(fx=0.0)

        self.assertIsNone(self._validate_first_candidate(image, depth))

    def _validate_first_candidate(self, image, depth):
        _, candidates = self.color_detector.detect(image)
        self.assertEqual(len(candidates), 1)
        inner_mask = self.color_detector.make_inner_mask(
            candidates[0], image.shape
        )
        return self.validator.validate(
            candidates[0], inner_mask, depth, self.camera
        )

    def _synthetic_sphere(self, center_z, radius):
        height, width = 480, 640
        image = np.zeros((height, width, 3), dtype=np.uint8)
        depth = np.full((height, width), np.nan, dtype=np.float32)
        projected_radius = int(
            round(self.camera.fx() * radius / center_z)
        )
        cv2.circle(
            image,
            (int(self.camera.cx()), int(self.camera.cy())),
            projected_radius,
            (0, 0, 255),
            thickness=-1,
        )

        rows, cols = np.indices((height, width))
        x_norm = (cols - self.camera.cx()) / self.camera.fx()
        y_norm = (rows - self.camera.cy()) / self.camera.fy()
        a = x_norm * x_norm + y_norm * y_norm + 1.0
        discriminant = center_z * center_z - a * (
            center_z * center_z - radius * radius
        )
        visible = discriminant >= 0.0
        depth[visible] = (
            center_z - np.sqrt(discriminant[visible])
        ) / a[visible]
        return image, depth


if __name__ == "__main__":
    unittest.main()
