import math
import heapq
import cv2
import numpy as np
import matplotlib.pyplot as plt

# Define the Cell class
class Cell:
    def __init__(self):
        self.parent_y = 0  # Parent cell's row index
        self.parent_x = 0  # Parent cell's column index
        self.f = float('inf')  # Total cost of the cell (g + h)
        self.g = float('inf')  # Cost from start to this cell
        self.h = 0  # Heuristic cost from this cell to destination


# Check if a cell is unblocked
def is_unblocked(image, x, y):
    return image[y][x] == 0     #image access coords swapped

# Check if a cell is the destination
def is_destination(x, y, dest):
    return x == dest[0] and y == dest[1]

# Calculate the heuristic value of a cell (Euclidean distance to destination)
def calculate_h_value(x, y, dest):
    return ((x - dest[0]) ** 2 + (y - dest[1]) ** 2) ** 0.5

def is_valid(x, y, width, height):
    return (x >= 0) and (x < width) and (y >= 0) and (y < height)


def trace_path(cell_details, dest):
    print("The Path is ")
    path = []
    x = dest[0]
    y = dest[1]

    # Trace the path from destination to source using parent cells
    while not (cell_details[y][x].parent_x == x and cell_details[y][x].parent_y == y):
        path.append((x, y))
        temp_row = cell_details[y][x].parent_x
        temp_col = cell_details[y][x].parent_y
        x = temp_row
        y = temp_col

    # Add the source cell to the path
    path.append((x, y))
    # Reverse the path to get the path from source to destination
    path.reverse()

    # Print the path
    for i in path:
        print(i)
    return path

# Implement the A* search algorithm. Returns a dictionary of points to follow
def a_star_search(image, src, dest):
    height, width = image.shape
    # Check if the source and destination are valid
    if not is_valid(src[0], src[1], width, height) or not is_valid(dest[0], dest[1], width, height):
        raise ValueError("Source or destination is invalid")

    # Check if the source and destination are unblocked
    if not is_unblocked(image, src[0], src[1]) or not is_unblocked(image, dest[0], dest[1]):
        raise ValueError("Source or the destination is blocked")

    # Check if we are already at the destination
    if is_destination(src[0], src[1], dest):
        raise ValueError("We are already at the destination")

    # Initialize the closed list (visited cells)
    closed_list = [[False for _ in range(width)] for _ in range(height)]
    # Initialize the details of each cell
    cell_details = [[Cell() for _ in range(width)] for _ in range(height)]

    # Initialize the start cell details
    x = src[0]
    y = src[1]
    cell_details[y][x].f = 0
    cell_details[y][x].g = 0
    cell_details[y][x].h = 0
    cell_details[y][x].parent_x = x
    cell_details[y][x].parent_y = y

    # Initialize the open list (cells to be visited) with the start cell
    open_list = []
    heapq.heappush(open_list, (0.0, x, y))

    # Initialize the flag for whether destination is found
    found_dest = False

    # Main loop of A* search algorithm
    while len(open_list) > 0:
        # Pop the cell with the smallest f value from the open list
        p = heapq.heappop(open_list)

        # Mark the cell as visited
        x = p[1]
        y = p[2]
        closed_list[y][x] = True

        # For each direction, check the successors
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        for dir in directions:
            new_x = x + dir[0]
            new_y = y + dir[1]

            # If the successor is valid, unblocked, and not visited
            if is_valid(new_x, new_y, width, height) and is_unblocked(image, new_x, new_y) and not closed_list[new_y][new_x]:
                # If the successor is the destination
                if is_destination(new_x, new_y, dest):
                    # Set the parent of the destination cell
                    cell_details[new_y][new_x].parent_x = x
                    cell_details[new_y][new_x].parent_y = y
                    print("The destination cell is found")
                    # Trace and print the path from source to destination
                    rv = trace_path(cell_details, dest)
                    found_dest = True
                    return rv
                else:
                    # Calculate the new f, g, and h values
                    g_new = cell_details[y][x].g + 1.0
                    h_new = calculate_h_value(new_x, new_y, dest)
                    f_new = g_new + h_new

                    # If the cell is not in the open list or the new f value is smaller
                    if cell_details[new_y][new_x].f == float('inf') or cell_details[new_y][new_x].f > f_new:
                        # Add the cell to the open list
                        heapq.heappush(open_list, (f_new, new_x, new_y))
                        # Update the cell details
                        cell_details[new_y][new_x].f = f_new
                        cell_details[new_y][new_x].g = g_new
                        cell_details[new_y][new_x].h = h_new
                        cell_details[new_y][new_x].parent_x = x
                        cell_details[new_y][new_x].parent_y = y
    # If the destination is not found after visiting all cells
    if not found_dest:
        raise ValueError("Failed to find the destination cell")    
    return

def a_star_trace(image_path, start_node, end_node, robot_radius=40, show_plot=True):
    """
    Loads a costmap, inflates obstacles based on robot size, 
    calculates the path using A*, and visualizes the result.
    """
    # 1. Load and prepare images
    disp_img = cv2.imread(image_path)
    if disp_img is None:
        raise FileNotFoundError(f"Could not load image at {image_path}")
        
    # Convert to grayscale for processing
    img_gray = cv2.cvtColor(disp_img, cv2.COLOR_BGR2GRAY)

    # 2. Inflate Map (Minkowski Sum approximation)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (robot_radius * 2, robot_radius * 2))
    inflated_map = cv2.dilate(img_gray, kernel, iterations=1)

    # 3. Run A* Search
    # Note: We pass the inflated map to the search algorithm
    path_coords = a_star_search(inflated_map, start_node, end_node)

    if path_coords:
        # 4. Prepare path for drawing
        path_arr = np.array(path_coords, dtype=np.int32)
        pts = path_arr.reshape((-1, 1, 2))

        # 5. Draw Visualization on the original color image
        # Draw start/end points
        cv2.circle(disp_img, tuple(start_node), radius=5, color=(0, 255, 0), thickness=-1) # Green
        cv2.circle(disp_img, tuple(end_node), radius=5, color=(0, 0, 255), thickness=-1)   # Red
        
        # Draw the path
        cv2.polylines(disp_img, [pts], isClosed=False, color=(255, 0, 0), thickness=2) # Blue

        if show_plot:
            plt.figure(figsize=(10, 8))
            plt.imshow(cv2.cvtColor(disp_img, cv2.COLOR_BGR2RGB))
            plt.title(f"A* Path with {robot_radius}px Inflation")
            plt.show()
            
        return path_coords, disp_img
    else:
        print("No path found!")
        return None, disp_img


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

