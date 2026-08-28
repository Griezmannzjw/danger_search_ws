#!/usr/bin/env python3

import unittest

import cv2
import numpy as np

from danger_search_perception.color_detector import RedCandidateDetector
from danger_search_perception.config import ColorDetectionConfig


class TestRedCandidateDetector(unittest.TestCase):
    def setUp(self):
        self.detector = RedCandidateDetector(ColorDetectionConfig())

    def test_red_circle_is_candidate(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.circle(image, (320, 240), 35, (0, 0, 255), thickness=-1)

        _, candidates = self.detector.detect(image)

        self.assertEqual(len(candidates), 1)
        self.assertGreater(candidates[0].circularity, 0.8)

    def test_red_square_is_rejected(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(image, (280, 200), (360, 280), (0, 0, 255), -1)

        _, candidates = self.detector.detect(image)

        self.assertEqual(candidates, [])

    def test_green_circle_is_rejected(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.circle(image, (320, 240), 35, (0, 255, 0), thickness=-1)

        _, candidates = self.detector.detect(image)

        self.assertEqual(candidates, [])

    def test_border_circle_is_rejected(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.circle(image, (10, 240), 35, (0, 0, 255), thickness=-1)

        _, candidates = self.detector.detect(image)

        self.assertEqual(candidates, [])

    def test_empty_scene_has_no_candidate(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)

        _, candidates = self.detector.detect(image)

        self.assertEqual(candidates, [])

    def test_lightly_occluded_circle_remains_candidate(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.circle(image, (320, 240), 35, (0, 0, 255), thickness=-1)
        # Simulate a small foreground edge at the far side of the target.
        # This remains inside the acceptance envelope, while a heavily
        # occluded shape is still rejected by the circularity checks.
        cv2.rectangle(image, (340, 200), (370, 280), (0, 0, 0), -1)

        _, candidates = self.detector.detect(image)

        self.assertEqual(len(candidates), 1)

    def test_moderately_motion_blurred_circle_remains_candidate(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.circle(image, (320, 240), 35, (0, 0, 255), thickness=-1)
        image = cv2.GaussianBlur(image, (15, 15), 0)

        _, candidates = self.detector.detect(image)

        self.assertEqual(len(candidates), 1)


if __name__ == "__main__":
    unittest.main()
