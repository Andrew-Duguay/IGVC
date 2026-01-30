import cv2
import numpy as np

def birdseye(image, top_width_feet=12, bot_width_feet=8):
    """
    Applies a perspective transform to simulate a bird's-eye view.
    
    :param image: Input image (numpy array).
    :param top_width_feet: Physical width visible at the top of the frame.
    :param bot_width_feet: Physical width visible at the bottom of the frame.
    :return: Transformed bird's-eye view image.
    """
    height, width = image.shape[:2]
    
    # Calculate how much to pad/stretch based on perspective geometry
    top_width_goal = width * top_width_feet / bot_width_feet
    append_amount = int(top_width_goal - width)
    pad_width = append_amount // 2

    # 1. Pad the image to make room for the "fan out" effect
    padded_image = cv2.copyMakeBorder(
        image,
        top=0,
        bottom=0,
        left=pad_width,
        right=pad_width,
        borderType=cv2.BORDER_CONSTANT,
        value=[0, 0, 0]
    )

    # 2. Define source points (the corners of the original image within the padded frame)
    src_pts = np.float32([
        [pad_width, 0], 
        [pad_width + width, 0], 
        [pad_width + width, height], 
        [pad_width, height]
    ])

    # 3. Define destination points (stretching the top corners to the edges of the padding)
    dst_pts = np.float32([
        [0, 0], 
        [padded_image.shape[1], 0], 
        [pad_width + width, height], 
        [pad_width, height]
    ])

    # 4. Perform the transform
    matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
    birdseye_image = cv2.warpPerspective(
        padded_image, 
        matrix, 
        (padded_image.shape[1], height)
    )

    return birdseye_image