"""OpenCV-based red circular candidate extraction."""

import math
from typing import List, Tuple

import cv2
import numpy as np

from .config import ColorDetectionConfig
from .models import ImageCandidate


class RedCandidateDetector:
    """Extract red, approximately circular regions from a BGR image."""

    def __init__(self, config: ColorDetectionConfig):
        self.config = config
        self.open_kernel = self._ellipse_kernel(config.morph_open_ksize)
        self.close_kernel = self._ellipse_kernel(config.morph_close_ksize)

    def detect(self, bgr: np.ndarray) -> Tuple[np.ndarray, List[ImageCandidate]]:
        red_mask = self._red_mask(bgr)
        contours, _ = cv2.findContours(
            red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        candidates = []
        for contour in contours:
            candidate = self._make_candidate(contour, bgr.shape)
            if candidate is not None:
                candidates.append(candidate)
        return red_mask, candidates

    def make_inner_mask(
        self, candidate: ImageCandidate, image_shape: Tuple[int, int]
    ) -> np.ndarray:
        mask = np.zeros(image_shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, [candidate.contour], -1, 255, thickness=-1)

        erosion_radius = max(
            1, int(round(candidate.enclosing_radius_px * 0.08))
        )
        erosion_radius = min(erosion_radius, 5)
        kernel_size = erosion_radius * 2 + 1
        inner_mask = cv2.erode(mask, self._ellipse_kernel(kernel_size))
        return inner_mask if cv2.countNonZero(inner_mask) else mask

    def _red_mask(self, bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        cfg = self.config
        mask_low = cv2.inRange(
            hsv,
            np.array(
                [cfg.red_h_low_1, cfg.red_s_low, cfg.red_v_low],
                dtype=np.uint8,
            ),
            np.array([cfg.red_h_high_1, 255, 255], dtype=np.uint8),
        )
        mask_high = cv2.inRange(
            hsv,
            np.array(
                [cfg.red_h_low_2, cfg.red_s_low, cfg.red_v_low],
                dtype=np.uint8,
            ),
            np.array([cfg.red_h_high_2, 255, 255], dtype=np.uint8),
        )
        mask = cv2.bitwise_or(mask_low, mask_high)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.open_kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.close_kernel)

    def _make_candidate(self, contour, image_shape):
        cfg = self.config
        area = float(cv2.contourArea(contour))
        if area < cfg.min_blob_area or area > cfg.max_blob_area:
            return None

        perimeter = float(cv2.arcLength(contour, True))
        if perimeter <= 1e-6:
            return None
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        if circularity < cfg.min_circularity:
            return None

        x, y, width, height = cv2.boundingRect(contour)
        if height <= 0:
            return None
        aspect_ratio = float(width) / float(height)
        if not cfg.min_aspect_ratio <= aspect_ratio <= cfg.max_aspect_ratio:
            return None

        (center_u, center_v), radius_px = cv2.minEnclosingCircle(contour)
        enclosing_area = math.pi * radius_px * radius_px
        if enclosing_area <= 1e-6:
            return None
        fill_ratio = area / enclosing_area
        if fill_ratio < cfg.min_circle_fill_ratio:
            return None

        image_height, image_width = image_shape[:2]
        margin = cfg.border_margin_px
        touches_border = (
            x <= margin
            or y <= margin
            or x + width >= image_width - margin
            or y + height >= image_height - margin
        )
        if cfg.reject_border_candidates and touches_border:
            return None

        return ImageCandidate(
            contour=contour,
            center_u=float(center_u),
            center_v=float(center_v),
            enclosing_radius_px=float(radius_px),
            circularity=float(circularity),
            circle_fill_ratio=float(fill_ratio),
            aspect_ratio=float(aspect_ratio),
        )

    @staticmethod
    def _ellipse_kernel(size: int) -> np.ndarray:
        size = max(1, int(size))
        if size % 2 == 0:
            size += 1
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
