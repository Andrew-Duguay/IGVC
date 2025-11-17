import matplotlib.pyplot as plt
import numpy as np
import cv2

def get_averaged_line_coords(image, lines):
    # 1. Collect all individual points from all lines
    x_points = []
    y_points = []   
    if lines is None or len(lines) == 0:
        return None
    for x1, y1, x2, y2 in lines:
        x_points.append(x1)
        x_points.append(x2)
        y_points.append(y1)
        y_points.append(y2)

    # 2. Find the line of best fit (1st-degree polynomial) returns [slope, intercept]
    line_fit = np.polyfit(y_points, x_points, 1)
    
    # 3. Generate the coordinates for our final line
    y_bottom = image.shape[0]          # At the bottom
    y_top = int(image.shape[0] * 0.6)  # 60% from the top (closer to camera and more reliable)
    
    # 4. Calculate the x values using our line model (x = m*y + b)
    # line_fit[0] is m (slope), line_fit[1] is b (intercept)
    x_bottom = int(line_fit[0] * y_bottom + line_fit[1])
    x_top = int(line_fit[0] * y_top + line_fit[1])   
    return [x_bottom, y_bottom, x_top, y_top]
    
def draw_lines_on_picture(picture):
    # Convert to grayscale
    picture = cv2.cvtColor(picture, cv2.COLOR_BGR2GRAY)
    # Convert datatype of pixels from float (0.0 to 1.0) to int8(0 to 255)
    picture = cv2.normalize(picture, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8U)
    og_pic = cv2.cvtColor(picture, cv2.COLOR_GRAY2BGR)
    
    #ADJUSTABLE VARIABLES
    # Define Region of Interest
    height, width = picture.shape[:2]
    ROI = np.array([[(0,height),(width/8,height/2), (width*7/8, height/2), (width,height)]], dtype=np.int32)
    image_midpoint = width / 2

    # Gaussian blur. More iterations to reduce noise.
    for i in range(2):
        picture = cv2.GaussianBlur(picture, (7,7), 0)
    # Canny edge detection
    threshold_low = 10
    threshold_high = 200
    picture = cv2.Canny(picture, threshold_low, threshold_high)
    # MASK REGION OF INTEREST
    mask = np.zeros_like(picture)   
    cv2.fillPoly(mask, ROI, 255)
    picture = cv2.bitwise_and(picture, mask)
    #DETECT LINES USING HOUGH LINE DETECTION
    rho = 2             # distance resolution in pixels 
    theta = np.pi/180   # angular resolution in radians 
    threshold = 40      # minimum number of votes 
    min_line_len = 100  # minimum number of pixels making up a line
    max_line_gap = 50   # maximum gap in pixels between connectable line segments    
    lines = cv2.HoughLinesP(picture, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    left_lines = []
    right_lines = []

    # Loop through all the lines
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:           
                # Only parse lines entirely on left or right of midpoint
                if x1 < image_midpoint and x2 < image_midpoint:
                    left_lines.append([x1, y1, x2, y2])
                elif x1 > image_midpoint and x2 > image_midpoint:
                    right_lines.append([x1, y1, x2, y2])

    # Get averaged left and right lines
    left_lane = get_averaged_line_coords(picture, left_lines)
    right_lane = get_averaged_line_coords(picture, right_lines)

    right_x_bottom = 0
    right_x_top = 0
    left_x_top = 0
    left_x_bottom = 0
    #get lane center at bottom
    if left_lane is not None:
        left_x_bottom = left_lane[0]
        left_x_top = left_lane[2]       
    if right_lane is not None:
        right_x_bottom = right_lane[0]
        right_x_top = right_lane[2]        
    
    lane_center_bottom = (left_x_bottom + right_x_bottom) / 2
    lane_center_top = (left_x_top + right_x_top) / 2

    # Draw the lines
    if left_lane is not None:
        x1, y1, x2, y2 = left_lane
        cv2.line(og_pic, (x1, y1), (x2, y2), (0, 255, 0), 15) # Green
    if right_lane is not None:
        x1, y1, x2, y2 = right_lane
        cv2.line(og_pic, (x1, y1), (x2, y2), (0, 0, 255), 15) # Blue
    # Draw the LANE CENTER (Red)
    # We can get its top coordinate the same way as the bottom
    cv2.line(og_pic, (int(lane_center_bottom), height), (int(lane_center_top), int(height * 0.6)), (255, 0, 0), 5, lineType=cv2.LINE_AA)

    # Draw the CAR CENTER (Yellow, Dashed)
    cv2.line(og_pic, (int(image_midpoint), height), (int(image_midpoint), int(height * 0.6)), (255, 255, 0), 5, lineType=cv2.LINE_AA)
    return og_pic

def decide_steer(offset_t: int, offset_b: int) -> float:

    # Define key figures to prevent over correction/ oscillations
    gain = 1             #defines the scale of adjustment
    upper_tolerance = 0  #number of pixels to ignore
    lower_tolerance = 0  
    if abs(offset_t) < upper_tolerance:
        offset_t = 0
    if abs(offset_b) < lower_tolerance:
        offset_b = 0

    # Decide steering
    if offset_b > 0: 
        # Current position is right of lane center, need to steer left
        steering_direction = -1
    elif offset_b < 0:
        # Current position is left of lane center, need to steer right
        steering_output = 1
    elif offset_b > 0:
        # camera is centered. check for trajectory
        if offset_t > 0:
            # road veers left, steer left
            steering_direction = -1
        elif offset_t < 0:
            # road veers right, steer right
            steering_direction = 1
        else:
            #On course. Do nothing
            steering_direction = 0

    # If both offsets are pos or both are neg then current position and trajectory are on same side
    # So then we need a big adjustment bc we're off course and going in wrong dir
    # Otherwise we are off course and going in the right direction
    steering_magnitude = abs(offset_b + offset_t) * gain

    return steering_direction*steering_magnitude

