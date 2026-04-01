# Little Blue - IGVC Autonomous Ground Vehicle

ROS2 autonomous navigation system for the Intelligent Ground Vehicle Competition (IGVC). Features camera-based lane detection, LiDAR obstacle avoidance, GPS waypoint navigation, and hardware bridge for the Little Blue robot platform.

## System Architecture

```
                         Gazebo Sim / Real Hardware
                                   |
                    +--------------+--------------+
                    |              |              |
              Left Camera    Right Camera    GPU LiDAR
              (640x480)      (640x480)      (270 deg)
                    |              |              |
                    v              v              v
          lane_candidate    lane_candidate    lidar_obstacle
          (HSV detect)      (HSV detect)      (clustering)
                    |              |              |
                    +------+-------+              |
                           |                      |
                    lane_accumulator              |
                    (snapshot buffer)              |
                           |                      |
                /candidate/lane_points    /obstacle_points
                           |                      |
                           +----------+-----------+
                                      |
                               lane_follower_node
                              (A* path planning)
                                      |
                                  /cmd_vel
                                      |
                              cmd_vel_to_joy_node
                                      |
                                    /joy
                                      |
                            Pi Motor Controller
                            (existing hardware)
```

## Packages

### `littleblue_autonomy`
Core autonomy stack: lane following, obstacle avoidance, GPS waypoint navigation.

**Nodes:**
| Node | Description |
|------|-------------|
| `lane_follower_node` | A* path planner with RANSAC lane reconstruction, obstacle avoidance, NML waypoint pursuit |
| `safety_monitor_node` | Read-only observer: collision detection, boundary violations, lap timing, JSONL event log |
| `cmd_vel_to_joy_node` | Bridge: converts `/cmd_vel` (Twist) to `/joy` (sensor_msgs/Joy) for Little Blue hardware |
| `data_recorder_node` | Optional data recording for post-run analysis |

### `littleblue_vision`
Sensor processing: lane detection, obstacle detection, point cloud accumulation.

**Nodes:**
| Node | Description |
|------|-------------|
| `lane_candidate_node` | Camera-based lane detection with 10 selectable methods (HSV, brightness, Canny, etc.) |
| `lane_accumulator_node` | Merges left+right camera detections with snapshot-based persistence |
| `lane_benchmark_node` | Scores candidate detectors against ground truth |
| `lidar_obstacle_node` | LiDAR scan clustering for obstacle detection |
| `lane_detector_node` | Original HSV lane detector (legacy) |
| `obstacle_projector_node` | YOLO + depth obstacle projection (legacy) |

### `littleblue_sim`
Gazebo simulation environment for the IGVC AutoNav course.

**Worlds:**
- `igvc_autonav2.sdf` - Current competition course with obstacles, waypoints, NML zone, ramp
- `igvc_autonav.sdf` - Original autonav course
- `igvc_course.sdf` - Simple oval course

**Robot Model (URDF/Xacro):**
- Differential drive chassis (0.8m x 0.6m)
- Dual RGB cameras (640x480, 85 deg FOV, 15Hz) with configurable pitch/yaw
- GPU LiDAR (270 deg, 10Hz, rear 90 deg blocked by electronics box)
- IMU (100Hz), GPS/NavSat (1Hz)

### `littleblue_nav`
Nav2 integration (currently unused).

## Lane Detection Pipeline

### Camera Processing
1. Dual cameras capture images at 15Hz
2. `lane_candidate_node` runs HSV white-line detection:
   - Convert to HSV, threshold V>230, S<25
   - Morphological open/close to clean noise
   - Connected component blob filter (rejects large compact blobs like barrel surfaces)
   - Ground-plane projection to body frame with configurable camera pitch/yaw compensation
3. Each camera publishes to `/candidate/left_lane_points` and `/candidate/right_lane_points`

### Point Cloud Accumulation
The `lane_accumulator_node` provides persistence through occlusions:
- Stores raw body-frame snapshots with their odom poses (rolling buffer of 50 frames)
- At publish time, re-projects all snapshots to current body frame
- Distance-based retention: keeps all points within 8m of robot
- No grid quantization: preserves exact point positions
- Publishes body-frame on `/candidate/lane_points` (for planner) and odom-frame on `/candidate/lane_points_viz` (for RViz)

### Lane Reconstruction (RANSAC)
The `lane_follower_node` reconstructs continuous lane lines from accumulated points:
1. Grid-based spatial clustering (0.5m cells) to identify separate lane lines
2. Takes the two largest clusters as left and right lane lines
3. RANSAC line fitting (Y = aX + b) for each cluster, robust to outliers
4. Generates dense wall points (every 0.2m) along each fitted line
5. Lane walls are injected into the A* obstacle list as impassable barriers

## Obstacle Detection

### LiDAR Clustering
The `lidar_obstacle_node` processes 2D laser scans:
- Filters returns by range (0.5-8.0m)
- Clusters consecutive returns with <0.2m gap
- Rejects clusters with fewer than 3 points
- Estimates radius from cluster spread
- Publishes surface points around each cluster center

### Obstacle Persistence
The `lane_follower_node` accumulates obstacles in odom frame:
- EMA position merge for repeated detections (alpha weighted by range)
- 30-second TTL with distance-based pruning
- Line-of-sight occlusion in `fake_lanes_autonav.py` (benchmark mode)

## Path Planning (A*)

### Waypoint Graph Construction
1. Extract lane boundaries per X-bin from RANSAC-fitted lane lines
2. Find drivable gaps between lane edges and inflated obstacles
3. Place waypoints within each gap
4. Build adjacency graph with segment-clear checks
5. A* search with heuristics: distance + center weight + path hysteresis + side commitment

### Path Smoothing
1. Densify to 0.15m spacing
2. Iterative averaging (configurable passes) with obstacle + lane clamping
3. Double final obstacle clamping pass to handle cascading

### Pure Pursuit
- Adaptive lookahead: `L = lookahead_min + speed_gain * |vx|`
- Curve slowdown based on curvature
- Steering EMA smoothing with rate limiting
- Obstacle repulsion nudge for nearby obstacles

## GPS Waypoint Navigation (NML)

### Multi-Waypoint Sequence
When the robot enters the No Man's Land (NML) zone (no lane lines):
1. GPS/world_pose detects proximity to entry waypoint (2.0m activate radius)
2. Robot navigates sequentially through waypoint list
3. Each waypoint reached within 1.0m triggers advancement to next
4. Final waypoint exits NML mode, resumes lane following

### NML Path Planning
- Potential field approach: goal attraction + obstacle repulsion
- Forward-only constraint: path always curves forward, never reverses
- Attraction scales down when repulsion is strong (prioritizes obstacle avoidance)
- Heading recovery disabled in NML (would fight with diagonal navigation)

**Default waypoint sequence:**
```
Entry (10, 8.75) -> Waypoint 3 (0, 6) -> Exit (10, 5)
```

## Hardware Bridge (cmd_vel_to_joy)

Converts autonomy velocity commands to Xbox controller format for Little Blue's existing motor controller.

### Signal Flow
```
/cmd_vel (Twist)
    linear.x -> throttle (0-1)     -> Joy axes[5]  (right trigger)
    angular.z -> steering (-1 to 1) -> Joy axes[0]  (left stick X)
    linear.x < 0 -> reverse         -> Joy buttons[1] (B button)
```

### Motor Mixing (on Pi, unchanged)
```
a = throttle (0-1)
b = -steering
M0 = (-(a/4)*(b + sqrt(b^2))*b) + ((a+1)/2)   -> left motor (0-100%)
M1 = (-(a/4)*(b - sqrt(b^2))*b) + ((a+1)/2)   -> right motor (0-100%)
```

### Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_linear_speed` | 1.0 m/s | cmd_vel speed at 100% throttle |
| `max_angular_speed` | 2.0 rad/s | cmd_vel turn rate at full steering |
| `publish_rate` | 20 Hz | Joy message rate |

### Safety
- 0.5s timeout: zeros output if no cmd_vel received
- RViz dashboard shows real-time controller state, motor duty cycles

## Simulation

### Physics Tuning
| Parameter | Value | Notes |
|-----------|-------|-------|
| `max_step_size` | 0.006 | Increased from 0.002 to prevent sensor render stalls |
| `real_time_factor` | 1.0 | Reduced from 3.0 for smooth RViz |
| `odom_publish_frequency` | 100 Hz | Increased from 30 for smooth visualization |

### Camera Configuration
- Dual cameras at front corners (y_offset = +/- 0.28m)
- Configurable pitch/yaw for aiming at lane lines
- 85 deg horizontal FOV (Basler wide-angle lens equivalent)
- Depth cameras disabled (saves GPU render passes)

### RViz Visualization
All markers use `stamp=0` (latest TF) to avoid sim-time extrapolation errors.

**Displays:**
| Display | Topic | Color | Description |
|---------|-------|-------|-------------|
| LanePoints | `/lane_points` | Blue | Ground truth lane lines (benchmark) |
| CandidateAccumulated | `/candidate/lane_points_viz` | Orange | Accumulated camera detections |
| CandidateLeft/Right | `/candidate/left_lane_points` | White | Raw per-frame detections |
| ObstaclePoints | `/obstacle_points` | Red spheres | LiDAR obstacle detections |
| ObstacleZones | `/autonomy/obstacles` | Red cylinders | Obstacle clearance zones |
| PlannedPath | `/autonomy/path` | Green line | A* planned path |
| Lookahead | `/autonomy/lookahead` | Yellow | Pure pursuit target |
| CameraDetection | `/candidate/debug_image` | - | Annotated camera feed |
| ControllerDashboard | `/joy_dashboard` | - | Joy state + motor duty |

## Quick Start

### Launch Everything (Simulation)
```bash
source install/setup.bash
bash scripts/launch_all.sh
```

### Build After Code Changes
```bash
source /opt/ros/jazzy/setup.bash    # or humble on Jetson
colcon build
# NTFS/WSL workaround: manually copy Python files
for pkg in littleblue_autonomy littleblue_vision; do
  for pyver in python3.10 python3.12; do
    dir="install/$pkg/lib/$pyver/site-packages/$pkg"
    [ -d "$dir" ] && cp src/$pkg/$pkg/*.py "$dir/"
  done
done
```

### Save Gazebo World Changes
The Gazebo GUI save times out on WSL. Use the service directly:
```bash
gz service -s /world/igvc_autonav/generate_world_sdf \
  --reqtype gz.msgs.SdfGeneratorConfig \
  --reptype gz.msgs.StringMsg \
  --timeout 30000 -r "" | \
  sed 's/^data: "//;s/"$//' | sed "s/\\\\n/\n/g" | sed "s/\\\\'/'/g" \
  > src/littleblue_sim/worlds/igvc_autonav2.sdf
```

### GPU Permissions (WSL, non-persistent)
```bash
sudo chmod 666 /dev/dri/renderD128
```

## Benchmark Framework

### Lane Detection Benchmark
Tests 10 detection methods against ground truth:
```bash
bash scripts/test_lane_detectors.sh all        # all methods
bash scripts/test_lane_detectors.sh brightness  # single method
python3 scripts/analyze_lane_benchmark.py       # results table
```

**Methods:** hsv_threshold, brightness, canny_hough, adaptive, lab_lightness, saturation_filter, sobel_gradient, combined_vote, morpho_skeleton, clahe_enhanced

### Navigation Diagnostic Logs
Written to `/tmp/autonomy_diag/`:
- `cycle_log_*.csv` - Per-cycle planner data (20Hz)
- `obstacle_log_*.csv` - Per-obstacle detail when within 5m
- `safety_events_*.jsonl` - Collisions, boundary violations, laps, stuck events

## Key Parameters (`autonomy_params.yaml`)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `control_rate` | 20 Hz | Main control loop rate |
| `cruise_speed` | 0.35 m/s | Forward speed |
| `lane_width` | 3.0 m | Expected lane width |
| `robot_half_width` | 0.35 m | Robot collision radius |
| `safety_margin` | 0.25 m | Extra clearance around obstacles |
| `obstacle_persist_time` | 30 s | Obstacle memory duration |
| `min_gap_width` | 0.80 m | Minimum gap A* will route through |
| `path_hysteresis` | 15.0 | Penalty for switching paths |
| `nml_activate_radius` | 2.0 m | GPS distance to trigger NML |
| `nml_exit_radius` | 1.0 m | GPS distance to advance/exit NML waypoint |

## Hardware Platform

### Little Blue Robot
- **Chassis:** 0.8m x 0.6m differential drive
- **Compute:** Jetson (GPU for inference) + Raspberry Pi (motor control)
- **Cameras:** Dual Basler wide-angle RGB
- **LiDAR:** 2D, 270 deg scan (rear blocked by electronics)
- **IMU:** 100Hz
- **GPS:** NavSat for waypoint navigation
- **Motors:** Arduino-controlled via serial from Pi (`<motor, M0, M1>`)
- **Controller:** Xbox gamepad via ROS2 `joy` node

### Communication
```
Jetson (ROS2) --/joy--> Pi (ROS2) --serial--> Arduino --PWM--> Motors
```
The `cmd_vel_to_joy_node` on the Jetson publishes Joy messages that the Pi's existing subscriber processes unchanged.

## Development Notes

- **WSL/NTFS breaks symlink-install** - Always manually copy .py files after build
- **Gazebo ogre2 required** - ogre1 causes frozen camera frames
- **Stale FastDDS shared memory** - After crashes: `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*`
- **Camera startup in WSL** - Software rendering needs extra time
- **Odom drifts** - Use `/world_pose` for ground truth, `/odom` only for TF/visualization
