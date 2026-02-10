# IGVC
Guidelines to launching simulation found in ros2_ws/launch_guide.txt

# Goals
These are broad, bird's eye tasks to do. If existing code exists in gitLab, port it over.


### 5. Node combines lines and objects into single image
  * Subscribes to /object_box and lane_map
  * fills object boxes with white pixels
  * Widens white space on cost map (config file to edit robot width)
  * Publishes to /cost_map
### 6. Node runs A* algorithm to find path
  * Subscribe to /cost_map
  * Subscribe to /way_point
  * Run algorithm and publish steering and throttle (cmd_vel?)
### 7. Make config file/node for
  * camera gap
  * robot width
  * resolution of cameras
### 10. Figure out what to do about waypoint
### 11. Docker For all Python code
### 12. Node or logic to calculate the waypoint

## Ideas
* Define different states ("Obstacle Avoidance Mode": Slow down and make sharper turns)
* Potentially convert to 3D rolling cost map (Use lidar instead of depth image)
  - Maybe priority node
