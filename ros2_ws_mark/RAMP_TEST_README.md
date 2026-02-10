# Little Blue IGVC — Ramp Test System

Full-stack autonomous lane following with ramp traversal on the IGVC oval course.

## Quick Start

```bash
cd ~/ros2_ws
colcon build --packages-select littleblue_sim littleblue_autonomy
./run_ramp_test.sh          # headless (default)
./run_ramp_test.sh gui      # with Gazebo GUI
```

Ctrl-C stops everything.

---

## System Architecture

```
                    Ignition Fortress (Sim)
                    ┌──────────────────────────────────────────┐
                    │  igvc_course world                       │
                    │  ┌─────────┐  ┌─────┐  ┌────────────┐   │
                    │  │littleblue│  │ramp │  │ obstacles   │   │
                    │  │(URDF)   │  │model│  │ (cones,     │   │
                    │  │DiffDrive│  │     │  │  barrels)   │   │
                    │  │LiDAR    │  │     │  │             │   │
                    │  │Camera   │  │     │  │             │   │
                    │  └────┬────┘  └─────┘  └─────────────┘   │
                    │       │                                   │
                    │  dynamic_pose/info (Ign transport)        │
                    └───────┼───────────────────────────────────┘
                            │
              ros_gz_bridge  │  (10 topics bridged)
         ┌──────────────────┼──────────────────────────┐
         │                  │                          │
    /cmd_vel           /odom, /scan              /clock, /tf
    (Twist)            /image_raw, etc.          /joint_states
         │                  │                          │
         │                  ▼                          │
         │    ┌─────────────────────────┐              │
         │    │   fake_lanes.py         │              │
         │    │   (fake_lane_publisher) │              │
         │    │                         │              │
         │    │ Reads: Ign ground truth │              │
         │    │ Pubs:  /lane_points     │──────┐       │
         │    │        /world_pose      │──┐   │       │
         │    └─────────────────────────┘  │   │       │
         │                                 │   │       │
         │    ┌────────────────────────────┼───┼───────┘
         │    │  lane_follower_node        │   │
         │    │  (littleblue_autonomy)     │   │
         │    │                            │   │
         │    │  Subs: /lane_points ◄──────┼───┘
         │    │        /scan               │
         │    │        /odom               │
         │    │        /world_pose ◄───────┘
         │    │                            │
         │    │  Pubs: /cmd_vel ───────────┼───► (to sim)
         │    │        /autonomy/state     │
         │    │        /autonomy/centerline│
         │    │        /autonomy/lookahead │
         │    └────────────────────────────┘
         │
         │    ┌────────────────────────────┐
         │    │  static_transform_publisher│
         │    │  lidar_link ↔ gpu_lidar    │
         │    └────────────────────────────┘
         │
         │    ┌────────────────────────────┐
         │    │  robot_state_publisher     │
         │    │  (from sim.launch.py)      │
         │    │  Publishes URDF TF tree    │
         │    └────────────────────────────┘
```

---

## Nodes in Detail

### 1. Ignition Fortress Simulation (`sim.launch.py`)

**What it does:** Runs the physics simulation, renders sensors, and bridges data to ROS2.

**Launched by:** `ros2 launch littleblue_sim sim.launch.py`

**Sub-components:**

| Component | Role |
|-----------|------|
| `ign gazebo` | Physics engine (DART), renders the world |
| `robot_state_publisher` | Publishes URDF TF tree (base_footprint → wheels, sensors) |
| `ros_gz_sim create` | Spawns the robot URDF into the Ignition world |
| `ros_gz_bridge` | Bridges 10 topics between Ignition and ROS2 |

**World:** `igvc_course.sdf` — stadium-shaped oval track:
- 12m straights, 5m-radius semicircles, 3m lane width
- White lane lines (emissive material for visibility with ogre renderer)
- 7 obstacles (cones + barrels) placed around the track
- Ramp at position (2, 0) on the bottom straight
- Physics: DART solver, 0.002s step size, 3x real-time factor

**Robot:** `littleblue.urdf.xacro` — differential drive robot:
- 0.8m x 0.6m x 0.15m chassis, 20kg
- Two drive wheels (0.2m radius, 0.55m track), front caster
- Sensors: RGB camera, depth camera, 2D LiDAR (360 samples), IMU, GPS
- Spawns at world position (-4, 0, 0.15) — 3.75m before the ramp approach funnel

**Bridge topics** (`config/bridge.yaml`):

| Topic | Direction | Type | Purpose |
|-------|-----------|------|---------|
| `/cmd_vel` | ROS→Ign | Twist | Drive commands |
| `/odom` | Ign→ROS | Odometry | Wheel odometry |
| `/scan` | Ign→ROS | LaserScan | 2D LiDAR |
| `/image_raw` | Ign→ROS | Image | RGB camera |
| `/depth/image_raw` | Ign→ROS | Image | Depth camera |
| `/imu/data` | Ign→ROS | Imu | IMU data |
| `/gps/fix` | Ign→ROS | NavSatFix | GPS position |
| `/joint_states` | Ign→ROS | JointState | Wheel positions |
| `/tf` | Ign→ROS | TFMessage | odom→base_footprint |
| `/clock` | Ign→ROS | Clock | Sim time (required for use_sim_time) |

---

### 2. Ramp Model (`models/ramp/model.sdf`)

**What it is:** A static 3-segment ramp placed on the bottom straight of the oval track.

**Geometry:**
- 3 segments: up slope, flat top, down slope
- Each segment: 1.5m long x 1.5m wide x 0.05m thick
- Up slope pitch: -0.1963 rad (-11.25 deg), peaks at z=0.3m
- Down slope pitch: +0.1963 rad, descends back to ground
- Total length: 4.5m (from x=-2.25 to x=+2.25 in model frame)
- Placed at world (2, 0) so it spans world x=-0.25 to x=4.25

**Lane lines on ramp:**
- 6 visual-only links (2 per segment, left at y=+0.75, right at y=-0.75)
- White emissive material (same as ground lane lines)
- Match the tilt of their parent segment
- No collision — visual only, won't affect physics

**Why 1.5m wide:** The robot is 0.66m wide, giving 0.42m clearance per side on the 1.5m ramp.

---

### 3. Fake Lane Publisher (`/tmp/fake_lanes.py`)

**Node name:** `fake_lane_publisher`

**What it does:** Replaces the real vision pipeline (camera → HSV detection → lane points) with synthetic lane points generated from known track geometry. Uses Ignition ground truth pose instead of drifting odometry for accurate world-frame positioning.

**Why it exists:** The real `lane_detector_node` processes camera images, but in sim the ogre renderer produces flat lighting that makes lane detection unreliable. This node provides perfect lane data so we can test the autonomy logic independently of vision.

**How it works:**

1. **Track geometry generation** (`generate_track_lines()`):
   - Generates ~604 inner + 604 outer lane line points for the full oval
   - Bottom straight has 5 sections for ramp funneling:

   | Section | World X range | Inner Y | Outer Y | Lane width |
   |---------|--------------|---------|---------|------------|
   | Normal | -6.0 to -1.25 | +1.5 | -1.5 | 3.0m |
   | Approach funnel | -1.25 to -0.25 | 1.5→0.75 | -1.5→-0.75 | 3.0→1.5m |
   | Ramp | -0.25 to 4.25 | +0.75 | -0.75 | 1.5m |
   | Exit funnel | 4.25 to 5.25 | 0.75→1.5 | -0.75→-1.5 | 1.5→3.0m |
   | Normal | 5.25 to 6.0 | +1.5 | -1.5 | 3.0m |

   - Semicircles: inner radius 3.5m, outer radius 6.5m, centered at (6, 5) and (-6, 5)
   - Top straight: inner y=8.5, outer y=11.5

2. **Ground truth pose** (`IgnPoseReader` thread):
   - Runs `ign topic -e -t /world/igvc_course/dynamic_pose/info` as a subprocess
   - Parses the protobuf text output line-by-line looking for the `littleblue` model
   - Extracts position (x, y) and orientation quaternion (qz, qw) → computes yaw
   - Thread-safe updates via mutex lock

3. **Publish loop** (10 Hz via ROS timer):
   - Gets ground truth pose (rx, ry, ryaw)
   - Publishes `/world_pose` (PoseStamped in `world` frame) — used by lane_follower for ramp detection
   - Transforms all track points from world frame to robot frame (base_footprint)
   - Filters to camera-like visible range: 0.3m ≤ forward ≤ 6.0m, lateral ≤ 6.0m
   - Adds small Gaussian noise (sigma=0.02m) to simulate sensor imprecision
   - Publishes visible points as PointCloud2 on `/lane_points` in `base_footprint` frame

**Subscribes to:** nothing (reads Ignition transport directly via subprocess)

**Publishes:**
| Topic | Type | Frame | Rate | Purpose |
|-------|------|-------|------|---------|
| `/lane_points` | PointCloud2 | base_footprint | 10 Hz | Synthetic lane detection output |
| `/world_pose` | PoseStamped | world | 10 Hz | Ground truth position for ramp detection |

---

### 4. Lane Follower Node (`lane_follower_node.py`)

**Node name:** `lane_follower_node`
**Package:** `littleblue_autonomy`
**Launched by:** `ros2 launch littleblue_autonomy autonomy.launch.py`
**Config:** `config/autonomy_params.yaml`

**What it does:** The main autonomy brain. Follows lane lines using pure pursuit, reactively avoids obstacles by offsetting the centerline, and suppresses obstacle detection near the ramp.

#### State Machine

```
    ┌──────────┐  lane points detected   ┌────────────────┐
    │ STOPPED  │ ──────────────────────► │ LANE_FOLLOWING  │
    └──────────┘                         └───────┬─────────┘
         ▲                                       │
         │ no lane timeout                       │ obstacle detected
         │                                       │ (±35°, 0.8-3.0m)
         │                                       │ (and not near ramp)
         │                                       ▼
         │                               ┌──────────────┐
         └────────────────────────────── │   AVOIDING    │◄─┐
                   no lane timeout        └──────┬───────┘  │
                                                 │          │
                                                 │          │ close obstacle
                                                 │          │ re-evaluation
                                                 │          │ (±70°, 0.15m)
                                                 │          │
                                                 ├──────────┘
                                                 │
                                                 │ traveled ≥ 5.0m
                                                 │ + no obstacle (±70°, 0.15m)
                                                 │
                                         back to LANE_FOLLOWING
```

- **STOPPED:** No lane data available. Waits for `/lane_points` to arrive. Transitions to LANE_FOLLOWING when enough centerline points are computed.
- **LANE_FOLLOWING:** Normal operation. Computes centerline from lane points, runs pure pursuit to follow it. Checks for obstacles via LiDAR. Transitions to AVOIDING if an obstacle is detected (unless near the ramp).
- **AVOIDING:** Obstacle evasion. Uses gap-finding to pick the widest clear direction (with side-clearance fallback), shifts the centerline laterally, and runs pure pursuit on the offset path at reduced speed. Uses wide-angle (±70°) close-range (0.15m) detection to track obstacles during evasion. Re-evaluates evasion direction when an obstacle enters the close-range dead zone. Returns to LANE_FOLLOWING after traveling a minimum distance AND no obstacles remain in the wide detection cone.

#### Centerline Computation (`_compute_centerline()`)

Converts raw lane points (in base_footprint frame) into a driveable centerline:

1. **Bin by X:** Divides the forward range (0.5m to 5.0m) into 0.5m bins
2. **Cluster into lanes:** For each bin, sorts Y values and looks for the largest gap:
   - If gap > 0.5m and enough points on each side → **two-line mode**: centerline = midpoint of the two line means
   - Otherwise → **single-line mode**: assumes centerline is `lane_width/2` toward the track center
3. **Output:** List of (x, y) centerline points in base_footprint frame, sorted by distance

#### Pure Pursuit (`_pure_pursuit()`)

Standard pure pursuit controller for differential drive:

1. **Adaptive lookahead:** `L = lookahead_min + speed_gain * |current_vx|`, clamped to [1.0, 3.0]m
2. **Find lookahead point:** First centerline point at distance ≥ L from robot origin
3. **Compute curvature:** `κ = 2 * gy / L²` (where gy is lateral offset of lookahead point)
4. **Compute velocities:**
   - `linear = cruise_speed * speed_factor`
   - `angular = linear * κ`, clamped to ±max_angular_vel
   - If curvature > threshold → apply curve slowdown factor
5. **Publish:** Twist on `/cmd_vel`, lookahead marker on `/autonomy/lookahead`

#### Obstacle Detection (`_check_obstacle()`)

LiDAR-based forward corridor scan with configurable overrides:

1. Scans `/scan` ranges in a forward cone (default ±35°, overridable via `half_angle_deg_override`)
2. Counts hits in range [`min_range`, `detect_range`] (default [0.8m, 3.0m], `min_range` overridable)
3. Returns True if hits ≥ 3 (noise-robust threshold)

**Two detection modes:**
- **LANE_FOLLOWING:** Standard ±35° cone, 0.8-3.0m range — avoids false positives from robot geometry
- **AVOIDING:** Wide ±70° cone, 0.15-3.0m range — prevents losing sight of obstacles that are offset laterally or very close

#### Gap Finding (`_find_best_gap()`)

Scans the forward hemisphere (±90°) to find the safest evasion direction:

1. Divides ±90° into 18 sectors (10° each), records minimum range per sector
2. Finds the widest contiguous run of "clear" sectors (min range > `gap_clearance_threshold`)
3. Converts the center angle of the widest gap to a lateral offset at lookahead distance
4. Returns `(offset, angle_deg)` or `None` if no clear gap exists

#### Evasion Direction (`_pick_evasion_offset()`)

Combines gap-finding with a side-clearance fallback:

1. Tries `_find_best_gap()` first — uses the widest clear angular gap
2. If no gap found (clustered obstacles), falls back to `_measure_side_clearance()` — measures average range on left (30-90°) vs right (-90 to -30°) and picks the side with more room
3. Clamps offset to ±`max_evasion_offset` (1.5m)

#### Reactive Evasion (`_handle_avoiding()`)

When an obstacle is detected:

1. **Choose direction:** `_pick_evasion_offset()` — gap-finding with side-clearance fallback
2. **Offset centerline:** Shifts all centerline Y values by the computed evasion offset (up to ±1.5m)
3. **Slow down:** Runs pure pursuit at 50% speed (`evasion_speed_factor`)
4. **Wide-angle tracking:** Uses ±70° close-range (0.15m) detection to keep obstacles visible even when laterally offset
5. **Close-range re-evaluation:** When an obstacle enters the dead zone (detected at ±70°/0.15m but NOT at ±35°/0.8m), re-computes gap direction and resets the distance counter — handles clustered obstacles and wrong-side evasion
6. **Completion:** Returns to LANE_FOLLOWING after traveling `evasion_min_distance` (5.0m) AND no obstacles remain in the wide ±70° detection cone

#### Ramp Suppression (`_near_ramp()`)

Prevents the ramp's slope from being treated as an obstacle:

1. Reads `ramp_waypoints` parameter — flat list of [x1, y1, x2, y2, ...] in **world frame**
2. Configured waypoints: ramp entry (-0.25, 0) and exit (4.25, 0)
3. Uses world-frame position from `/world_pose` (ground truth, not odom — odom drifts too much)
4. Returns True if the robot is within `ramp_suppression_radius` (3.0m) of any waypoint
5. When True, `_handle_lane_following()` skips the `_check_obstacle()` call entirely

**Subscribes:**
| Topic | Type | Purpose |
|-------|------|---------|
| `/lane_points` | PointCloud2 | Lane detection input |
| `/scan` | LaserScan | Obstacle detection |
| `/odom` | Odometry | Current velocity + distance integration |
| `/world_pose` | PoseStamped | World-frame position for ramp detection |

**Publishes:**
| Topic | Type | Purpose |
|-------|------|---------|
| `/cmd_vel` | Twist | Drive commands |
| `/autonomy/state` | String | Current state (STOPPED/LANE_FOLLOWING/AVOIDING) |
| `/autonomy/centerline` | Path | Computed centerline (for visualization) |
| `/autonomy/lookahead` | PointStamped | Pure pursuit lookahead point |

#### Parameters (`autonomy_params.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `control_rate` | 20.0 Hz | Main loop frequency |
| `min_x` / `max_x` | 0.5 / 5.0 m | Forward range for centerline computation |
| `bin_width` | 0.5 m | X-axis bin size for lane clustering |
| `min_points_per_bin` | 3 | Minimum points to consider a bin valid |
| `lane_width` | 3.0 m | Expected lane width (for single-line centering) |
| `min_centerline_points` | 2 | Min points to run pure pursuit |
| `cruise_speed` | 0.5 m/s | Normal driving speed |
| `lookahead_min` / `max` | 1.0 / 3.0 m | Pure pursuit lookahead range |
| `speed_gain` | 0.8 | Lookahead speed scaling factor |
| `max_angular_vel` | 1.0 rad/s | Angular velocity clamp |
| `curve_slowdown_curvature` | 0.5 | Curvature threshold for slowdown |
| `curve_slowdown_factor` | 0.6 | Speed multiplier in curves |
| `obstacle_half_angle` | 35.0 deg | Forward detection cone half-width (LANE_FOLLOWING; AVOIDING uses 70°) |
| `obstacle_min_range` | 0.8 m | Ignore closer than this (LANE_FOLLOWING; AVOIDING uses 0.15m) |
| `obstacle_detect_range` | 3.0 m | Ignore farther than this |
| `obstacle_min_hits` | 3 | Hit count threshold |
| `max_evasion_offset` | 1.5 m | Max lateral centerline shift during evasion |
| `gap_clearance_threshold` | 3.0 m | Min sector range to consider "clear" in gap-finding |
| `evasion_min_distance` | 5.0 m | Min distance before clearing evasion |
| `evasion_speed_factor` | 0.5 | Speed multiplier during evasion |
| `centerline_spread_threshold` | 1.0 m | Y-spread above which bin is treated as two-line |
| `no_lane_timeout` | 3.0 s | Time without lane data before stopping |
| `ramp_waypoints` | [-0.25, 0.0, 4.25, 0.0] | Ramp entry/exit in world frame |
| `ramp_suppression_radius` | 3.0 m | Distance to suppress obstacle detection |

---

### 5. Static Transform Publisher (lidar frame bridge)

**What it does:** Publishes an identity transform between `lidar_link` and `littleblue/base_footprint/gpu_lidar`.

**Why it exists:** Ignition Fortress publishes LiDAR data with frame_id `littleblue/base_footprint/gpu_lidar` (the Ignition-internal sensor frame name). The rest of the ROS TF tree uses `lidar_link` (from the URDF). This static transform bridges them so `/scan` data can be resolved in the robot's TF tree.

**Command:**
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 lidar_link littleblue/base_footprint/gpu_lidar
```

---

### 6. Robot State Publisher (from sim.launch.py)

**What it does:** Reads the URDF robot description and publishes the static TF tree for all robot links (base_footprint → chassis → wheels, camera, lidar, IMU, GPS links).

**Why it exists:** Other nodes need TF lookups between sensor frames and the robot base. The DiffDrive plugin publishes odom→base_footprint, and robot_state_publisher fills in the rest of the tree.

---

## Topic Flow for Ramp Traversal

1. **Robot approaches ramp** (world x ≈ -2 to -0.25):
   - `fake_lanes.py` emits narrowing lane points (funnel: 3.0m → 1.5m)
   - `lane_follower_node` computes centerline from narrowing lane → steers toward center
   - Pure pursuit naturally centers the robot on the narrowing path

2. **Robot enters ramp suppression zone** (within 3.0m of waypoint at x=-0.25):
   - `_near_ramp()` returns True
   - `_check_obstacle()` is skipped — LiDAR sees the ramp slope but doesn't trigger evasion
   - Robot continues in LANE_FOLLOWING state

3. **Robot drives over ramp** (world x ≈ -0.25 to 4.25):
   - Lane points are constant 1.5m wide → robot stays centered
   - Obstacle detection remains suppressed

4. **Robot exits ramp** (world x ≈ 4.25 to 5.25):
   - Lane points widen back to 3.0m (exit funnel)
   - Robot leaves the suppression radius of the exit waypoint (x=4.25)
   - Obstacle detection re-enables
   - Normal lane following continues

---

## File Listing

```
~/ros2_ws/
├── run_ramp_test.sh                              # Launch script
├── src/
│   ├── littleblue_sim/
│   │   ├── launch/sim.launch.py                  # Sim + bridge launch
│   │   ├── config/bridge.yaml                    # 10-topic bridge config
│   │   ├── worlds/igvc_course.sdf                # Track + obstacles + ramp
│   │   ├── models/ramp/model.sdf                 # 3-segment ramp with lane lines
│   │   ├── models/cone/model.sdf                 # Cone obstacle
│   │   ├── models/barrel/model.sdf               # Barrel obstacle
│   │   └── urdf/littleblue.urdf.xacro            # Robot description
│   └── littleblue_autonomy/
│       ├── launch/autonomy.launch.py             # Autonomy launch
│       ├── config/autonomy_params.yaml           # All tunable parameters
│       └── littleblue_autonomy/
│           └── lane_follower_node.py             # Main autonomy node
└── /tmp/
    └── fake_lanes.py                             # Synthetic lane publisher
```
