import cv2
import numpy as np

def depth_costmap(depth_frame, height_threshold=1000, dilation_size=5):

    """
    Converts a depth image into a binary costmap.
    
    :param depth_frame: The raw depth image (unit: millimeters).
    :param height_threshold: Depth values below this (closer to camera) are treated as obstacles.
    :param dilation_size: Size of the kernel to expand obstacles for safety.
    :return: A binary costmap (0 for free space, 255 for obstacles).
    """
    # 1. Create a binary mask where '0' is free space and '255' is an obstacle
    # In your notebook, values closer than the threshold indicate an object.
    _, costmap = cv2.threshold(depth_frame, height_threshold, 255, cv2.THRESH_BINARY_INV)
    
    # 2. Convert to uint8 for OpenCV operations if it isn't already
    costmap = costmap.astype(np.uint8)

    # 3. Clean up noise using Morphological transformations
    kernel = np.ones((dilation_size, dilation_size), np.uint8)
    
    # Remove small noise (opening) and expand obstacles (dilation)
    costmap = cv2.morphologyEx(costmap, cv2.MORPH_OPEN, kernel)
    costmap = cv2.dilate(costmap, kernel, iterations=1)

    return costmap


def lane_costmap(image, low_hsv=[18, 94, 140], high_hsv=[48, 255, 255]):

    """
    Converts a color image into a binary lane costmap using HSV thresholding.
    
    :param image: Input BGR image.
    :param low_hsv: Lower bound for HSV threshold (default tuned for yellow).
    :param high_hsv: Upper bound for HSV threshold.
    :return: Binary costmap where lanes are 255 (obstacles) and road is 0.
    """
    # 1. Convert BGR to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # 2. Define range and threshold the HSV image
    lower_limit = np.array(low_hsv)
    upper_limit = np.array(high_hsv)
    mask = cv2.inRange(hsv, lower_limit, upper_limit)
    
    # 3. Bitwise-AND mask and original image to isolate lanes
    res = cv2.bitwise_and(image, image, mask=mask)
    
    # 4. Convert isolated result to grayscale and threshold to get binary map
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    _, lane_costmap = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
    
    return lane_costmap

def combine_costmaps(lane_map, depth_map):
    """
    Combines a lane-based costmap and a depth-based costmap into a single master map.
    
    :param lane_map: Binary image representing lane boundaries.
    :param depth_map: Binary image representing physical obstacles.
    :return: A single binary costmap containing all obstacles.
    """
    # Use a bitwise OR operation to combine the white pixels (obstacles) from both maps.
    # If a pixel is an obstacle in either map, it will be an obstacle in the combined map.
    combined_map = cv2.bitwise_or(lane_map, depth_map)
    
    return combined_map