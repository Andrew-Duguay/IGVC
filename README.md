# LittleBlue real-robot deployment

This package (`startup_robot`) is the single entry point for running the
autonomy stack on the physical robot. Everything sim-specific lives in
`littleblue_sim` and is **not** used on deploy.

## One-time setup on the robot

0. **Unpack the ship bundle**

   The zip expands into this layout:

   ```
   README.md
   ros2_ws/src/littleblue_sim/
   ros2_ws/src/workbench/littleblue_autonomy/
   ros2_ws/src/workbench/littleblue_vision/
   ros2_ws/src/workbench/startup_robot/
   ```

   Merge `ros2_ws/src/` into the existing real-robot workspace's
   `ros2_ws/src/` — alongside the packages already there
   (`sllidar_ros2`, `game_controller`, `data_interface`). Don't
   overwrite those.

   If you don't already have `sllidar_ros2`, `game_controller`, or
   `data_interface` in that workspace, copy them in now from the
   existing real-robot repo — steps 4 and 5 below tell you where
   they live.

1. **Install system deps**

   ```bash
   cd ros2_ws
   source /opt/ros/iron/setup.bash
   colcon build
   source install/setup.bash
   bash src/workbench/startup_robot/install_deps.sh
   ```

   This pulls in `robot_localization`, `usb_cam`, `nmea_navsat_driver`,
   `joy`, and the other runtime deps, and installs the udev rules into
   `/etc/udev/rules.d/`.

2. **Fill in the udev rules**

   Edit `/etc/udev/rules.d/99-littleblue.rules` — every `REPLACE_*`
   value needs the real VID/PID/serial/port for this robot's hardware.
   Find them with:

   ```bash
   udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct|serial"
   udevadm info -a -n /dev/video0  | grep -E "idVendor|idProduct|KERNELS"
   ```

   After editing, reload:

   ```bash
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

   Verify each symlink appears:

   ```bash
   ls -l /dev/lidar /dev/gps /dev/imu /dev/motor /dev/camera_left /dev/camera_right
   ```

3. **Choose and install the IMU + GPS drivers**

   The deploy launch leaves these as commented-out placeholders because
   the choice depends on hardware. You must pick, install, and launch
   drivers that meet the contract below. The rest of the stack reads
   only these topics — any driver that produces them is fine.

   ### IMU driver contract

   | Property       | Required value                            |
   | -------------- | ----------------------------------------- |
   | Topic          | `/imu/data`                               |
   | Message type   | `sensor_msgs/msg/Imu`                     |
   | `frame_id`     | `imu_link`                                |
   | Rate           | ≥ 50 Hz                                   |
   | Orientation    | Populated (covariance[0] ≥ 0)             |
   | Angular vel    | Populated (covariance[0] ≥ 0)             |
   | Linear accel   | Populated (covariance[0] ≥ 0)             |

   Typical packages: `ros-iron-microstrain-inertial-driver` (Microstrain
   3DM-GX*), `ros-iron-xsens-mti-driver` (XSens MTi), `ros-iron-bno055`
   (Bosch BNO055 over I²C/UART).

   After installing, add an `IncludeLaunchDescription(...)` block for
   the driver's launch file in `deploy.launch.py` (next to the commented
   `# imu_launch = ...` placeholder) and append `imu_launch` to the
   `LaunchDescription([...])` list at the bottom.

   Verify with:

   ```bash
   ros2 topic echo --once /imu/data
   ros2 topic hz /imu/data           # expect 50+ Hz
   ```

   ### GPS driver contract

   | Property       | Required value                            |
   | -------------- | ----------------------------------------- |
   | Topic          | `/gps/fix`                                |
   | Message type   | `sensor_msgs/msg/NavSatFix`               |
   | `frame_id`     | `gps_link`                                |
   | Rate           | ≥ 1 Hz                                    |
   | `status.status`| ≥ 0 (NO_FIX = -1 is unusable)             |

   Typical packages: `ros-iron-nmea-navsat-driver` (any NMEA-0183 GPS
   over USB-serial), `ros-iron-ublox-gps` (u-blox F9P and similar).

   Integration is the same pattern as IMU: include the driver's launch
   file and append its handle to the `LaunchDescription([...])` list.

   Verify with:

   ```bash
   ros2 topic echo --once /gps/fix   # check latitude/longitude look right
   ros2 topic hz /gps/fix            # expect 1-10 Hz
   ```

   ### What breaks if these are missing or wrong

   - No `/imu/data` → both EKFs stall; `/odometry/filtered_map` never
     publishes; `/world_pose` stays empty; autonomy stops at the
     "Waiting for world pose..." log.
   - No `/gps/fix` → the global EKF + `navsat_transform_node` never
     establish a datum; `/world_pose` has no real world anchor.
   - Wrong `frame_id` → the EKF can't place the sensor on the robot;
     output will drift or jump. Double-check that `imu_link` and
     `gps_link` exist in the TF tree (`ros2 run tf2_tools view_frames`)
     and match where the sensors are physically mounted.

4. **Build `sllidar_ros2`**

   Pulled from the existing real-stack workspace
   (`Dev/ROS2/lidar_ws/src/sllidar_ros2`). Copy it into
   `ros2_ws/src/` and `colcon build`.

5. **Build `game_controller` + `data_interface`**

   From `src/motorControl/rpi/controller_ws/src/` in the existing
   real-stack repo. Copy both packages into `ros2_ws/src/` and rebuild.

## YOLO TensorRT (Jetson)

The deploy launch starts two YOLO instances (one per camera) + two
`obstacle_projector` nodes. YOLO detections get fused with lidar on
`/obstacle_points` and feed the A* planner's obstacle list.

### What you need to provide

1. **A compiled TensorRT engine file** at
   `$HOME/yolo_weights/<name>.engine` (or override the path with
   `YOLO_ENGINE=...` before launching).

   Build one on the Jetson with Ultralytics:

   ```bash
   # Start from a PyTorch checkpoint (generic YOLOv11n for starters)
   yolo export model=yolo11n.pt format=engine imgsz=640 device=0
   # Move the result to the expected location
   mkdir -p ~/yolo_weights
   mv yolo11n.engine ~/yolo_weights/
   ```

   **Compile on the target machine.** TensorRT engines are tied to the
   exact GPU + CUDA + TensorRT version they were built against. An
   `.engine` built on a dev laptop will not load on the Jetson.

2. **Depth cameras** producing `/left_camera/depth/image_raw` and
   `/right_camera/depth/image_raw`. `install_deps.sh` does not install
   a depth-camera driver — pick one for your hardware (RealSense,
   ZED, Astra, etc.) and wire the launch include in
   `deploy.launch.py` next to the `# left_depth_launch = ...` comment.

### Verify YOLO is running

```bash
ros2 topic hz /left_yolo/detections    # expect ~10–15 Hz
ros2 topic hz /right_yolo/detections
ros2 topic hz /obstacle_points         # fires from lidar AND projector
```

If `/obstacle_points` only fires from the lidar rate, depth topics
aren't synchronising with the detection stream — check
`/left_camera/depth/image_raw` and `/right_camera/depth/image_raw`
are publishing and their timestamps are close to the RGB stream.

### CPU fallback

Set `device='cpu'` in the YOLO parameter block in `deploy.launch.py`.
Ultralytics will fall back to PyTorch CPU inference — works but is
far too slow for real-time control. Only useful for bring-up on a
non-Jetson machine.

## Per-course config (day-of)

Edit `src/workbench/startup_robot/config/course.yaml`:

- `nml_waypoints`: today's NML segment in the world frame. Format is
  `[entry_x, entry_y, wp1_x, wp1_y, ..., exit_x, exit_y]`.
- `camera_*`: only change if the cameras are re-mounted.

No other day-of edits should be needed.

## Running

```bash
ros2 launch startup_robot deploy.launch.py
```

Or with RViz (for debugging on a connected laptop):

```bash
ros2 launch startup_robot deploy.launch.py rviz:=true
```

## What the launch does

1. **`robot_state_publisher`** loads `urdf/littleblue_deploy.urdf.xacro`
   and emits the TF tree.
2. **Sensor drivers**: `usb_cam` ×2, `sllidar_ros2`, IMU, GPS.
3. **`robot_localization` EKF stack**:
   - `ekf_local_node` fuses wheel odom + IMU → `odom → base_footprint`
   - `ekf_global_node` fuses the above + GPS → `map → odom`
   - `navsat_transform_node` auto-captures the world-frame datum from
     the first GPS fix (no manual datum entry).
4. **`world_pose_from_ekf.py`** republishes the map-frame pose as
   `/world_pose` (`PoseStamped`), which the autonomy expects.
5. **Motor path**: `cmd_vel_to_joy_node` → `joy` → `game_controller/listener`
   → serial to the Arduino on `/dev/motor`.
6. **Autonomy**: `lane_follower_node`, two `lane_candidate_node`s,
   `lane_accumulator_node`, `lidar_obstacle_node`.

## World-frame contract

`/world_pose` is `geometry_msgs/PoseStamped` with `frame_id='world'`.
The pose is the robot's position/orientation in a local ENU frame
anchored at the first GPS fix. **All autonomy internals are in this
frame** (lane cell accumulation, NML waypoints, stuck detection, etc.).

If the autonomy behaves like it's in the wrong place, check:

- `/gps/fix` is publishing and has a valid fix (`status.status >= 0`)
- `/odometry/filtered_map` is publishing (global EKF healthy)
- `ros2 topic echo /world_pose` shows reasonable values near 0,0 at start

## Known not-in-scope

- Autonomy behaviour in tight barrel clusters is still being tuned in
  sim; outdoor behaviour may differ. Treat the first outdoor run as an
  operating test, not a correctness test.
- HSV thresholds are locked for white-paint-on-concrete under normal
  daylight. Extreme lighting (night, heavy shadow) may need retuning.
