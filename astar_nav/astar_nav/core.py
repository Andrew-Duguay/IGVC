import heapq
import math
import numpy as np

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

# def heuristic(x, y, dest):
#     return math.hypot(x - dest[0], y - dest[1])

def trace_path(cell_details, dest):
    # print("The Path is ")
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
    # for i in path:
    #     print(i)
    ## no print in libiraries 
    return path

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
