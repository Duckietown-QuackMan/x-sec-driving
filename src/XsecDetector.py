#!/usr/bin/env python3

import cv2
import numpy as np

from numpy.typing import NDArray


class XsecDetector:

    def detect_red_lines(
        self, input_bgr_image: NDArray[np.uint8]
    ) -> NDArray[np.bool_]:
        """
        Detect the red lines in the input image.

        Args:
            input_bgr_image (NDArray[np.uint8]): 3-channels input image in BGR format

        Returns:
            NDArray[np.bool_]: 1-channel binary mask of the red lines
        """

        # Convert the BGR image to HSV color space
        lab_image = cv2.cvtColor(input_bgr_image, cv2.COLOR_BGR2LAB)

        # Define HSV range for red color (this might need fine-tuning)
        lower_red = np.array([180, 125, 120], dtype=np.uint8)   # Low saturation and high brightness
        upper_red = np.array([245, 170, 170], dtype=np.uint8)

        # Create a binary mask for red color
        red_lines_mask = cv2.inRange(lab_image, lower_red, upper_red).astype(np.bool_)

        # ---

        return red_lines_mask


