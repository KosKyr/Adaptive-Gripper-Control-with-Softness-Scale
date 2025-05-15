### File: configDepth.py
import cv2
import numpy as np

def calculate_average_depth(depth_region):
    """
    Calculate the average depth from a cropped region of the depth frame.
    """
    valid_depth = depth_region[depth_region > 0]
    if valid_depth.size == 0:
        return 1  # fallback value in meters
    return np.median(valid_depth)

def normalize_depth_frame(depth_frame):
    depth_vis = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    return cv2.cvtColor(depth_vis, cv2.COLOR_GRAY2BGR)
