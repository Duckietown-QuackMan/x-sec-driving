import cv2
import numpy as np
import rospy

HSV_RANGES_RED_1 = [
    (0, 140, 100),
    (15, 255, 255),
]
HSV_RANGES_RED_2 = [
    (165, 70, 50),
    (180, 255, 255),
]


def redline_detection(
    img: np.ndarray,
    threshold: float = 400000,
    ) -> np.array:
    """
    Parameters:
        img (np.ndarray): The input image to be cropped and converted into red mask.
        threshold (int): Threshold, defining number of red pixels detected in image.

    Returns:
        bool: flag of redline detection in image
        hsv_image: returns img in hsv colorspace, used for evaluation
    """
    #cropping the image
    top = 350
    bottom = 0
    #corp left due to discrepancy
    left = 50 
    right = 0
    
    #crop the image
    cropped_image = img[top:(img.shape[0] - bottom), left:(img.shape[1]- right)]
    # Convert the BGR image to HSV color space
    hsv_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)
    # Create red masks (two ranges for red)
    red_mask1 = cv2.inRange(hsv_image, HSV_RANGES_RED_1[0], HSV_RANGES_RED_1[1])
    red_mask2 = cv2.inRange(hsv_image, HSV_RANGES_RED_2[0], HSV_RANGES_RED_2[1])
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    # count number of red pixels
    if np.sum(red_mask) > threshold:
        rospy.loginfo(f"RED detection {np.sum(red_mask)}")
        return True, hsv_image
    
    return False , hsv_image
