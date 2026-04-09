#!/usr/bin/env python3
"""Lane-following with A* waypoint graph for obstacle avoidance.

Architecture:
1. Extract lane boundaries from /lane_points (per-bin left/right)
2. At each X-step, find drivable gaps between lane edges and inflated obstacles
3. Build a sparse waypoint graph: nodes = gap waypoints, edges = obstacle-free connections
4. A* finds the optimal path through the graph
5. Smooth the A* path while respecting obstacle clearance
6. Pure pursuit follows the smoothed path

Obstacle handling:
- Incoming surface points are clustered into obstacle centers each frame
- Centers are accumulated in odom frame with timestamps for persistence
- Each cycle, odom-frame centers are projected to body frame for A* planning
- Obstacles are pruned by distance (>7m) AND staleness (>15s behind robot)

Approach modes (selectable via approach_mode parameter):
- 'baseline': Original fixed clearance from obstacle center
- 'A': Size-aware clearance (est_radius + robot_half_width + safety)
- 'B': Side commitment (A* cost penalty for switching sides)
- 'C': Distance-graduated clearance (plan wide far, tight near)
- 'D': All combined (A + B + C)
"""

import csv
import math
import heapq
import os
import time
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, PointField, NavSatFix, Imu
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


# ─── Known obstacle positions (world frame) for diagnostic matching ───
# Must match safety_monitor_node.py's OBSTACLES list
KNOWN_OBSTACLES = [
    (10, 7.98, 0.15, 'cone_1'),
    (-6, 22, 0.15, 'cone_2'),
    (3.11, -1, 0.15, 'cone_3'),
    (-10, 15, 0.25, 'barrel_1'),
    (7, 24, 0.25, 'barrel_2'),
    (-8, 1, 0.25, 'barrel_minspeed'),
    (0, 24.5, 0.30, 'tire_1'),
    (10, 17, 0.30, 'tire_2'),
    (-3, 0.5, 0.20, 'trash_can_1'),
    (5, -0.5, 0.50, 'barricade_1'),
    (10.5, 6, 0.25, 'barrel_nml_1'),
    (9.5, 4.5, 0.15, 'cone_nml_1'),
    (0.91, 1.17, 0.15, 'cone_3_1'),
    (2.94, -0.40, 0.25, 'barrel_nml_1_1'),
    (2.73, 0.28, 0.25, 'barrel_nml_1_2'),
    (1.5, 7.0, 0.15, 'cone_wp3_1'),
    (-1.0, 5.5, 0.15, 'cone_wp3_2'),
    (0.5, 5.0, 0.25, 'barrel_wp3_1'),
    (1.10, 0.52, 0.25, 'barrel_wp3_1_1'),
    (0.14, -1.06, 0.25, 'barrel_nml_1_2_1'),
    (-0.24, -0.37, 0.20, 'trash_can_1_1'),
]


class DiagnosticLogger:
    """Structured CSV diagnostic logger for lane_follower_node.

    Writes two files:
    - cycle_log.csv: per-cycle summary (pose, segment, nearest obstacle, cmd)
    - obstacle_log.csv: detailed per-obstacle data when robot is within proximity_threshold
    """

    CYCLE_FIELDS = [
        'time', 'cycle', 'world_x', 'world_y', 'world_yaw_deg',
        'segment', 'centerline_dist', 'heading_err_deg', 'track_idx',
        'n_lane_pts', 'n_obstacles', 'n_world_obstacles',
        'in_nml', 'in_recovery', 'on_ramp',
        'nearest_obs_name', 'nearest_obs_dist',
        'path_len', 'replanned',
        'cmd_vx', 'cmd_az', 'repulsive_az', 'curve_factor',
        'goal_wx', 'goal_wy',
    ]

    OBSTACLE_FIELDS = [
        'time', 'cycle', 'obs_name', 'obs_wx', 'obs_wy', 'obs_radius',
        'robot_wx', 'robot_wy',
        'world_dist', 'body_x', 'body_y',
        'clearance_threshold', 'actual_clearance',
        'detected', 'det_bx', 'det_by', 'det_radius', 'det_match_dist',
        'path_closest_dist', 'path_closest_x', 'path_closest_y',
        'dodge_side', 'lane_left_at_obs', 'lane_right_at_obs',
    ]

    def __init__(self, log_dir='/tmp/autonomy_diag', proximity_threshold=5.0):
        self.log_dir = log_dir
        self.proximity_threshold = proximity_threshold
        os.makedirs(log_dir, exist_ok=True)

        ts = time.strftime('%Y%m%d_%H%M%S')
        self._cycle_file = open(os.path.join(log_dir, f'cycle_log_{ts}.csv'), 'w', newline='')
        self._obs_file = open(os.path.join(log_dir, f'obstacle_log_{ts}.csv'), 'w', newline='')

        self._cycle_writer = csv.DictWriter(self._cycle_file, fieldnames=self.CYCLE_FIELDS)
        self._obs_writer = csv.DictWriter(self._obs_file, fieldnames=self.OBSTACLE_FIELDS)
        self._cycle_writer.writeheader()
        self._obs_writer.writeheader()

        self._cycle_file.flush()
        self._obs_file.flush()
        self._flush_counter = 0

    def log_cycle(self, data):
        """Write one row to cycle_log.csv."""
        self._cycle_writer.writerow(data)
        self._flush_counter += 1
        if self._flush_counter % 20 == 0:
            self._cycle_file.flush()

    def log_obstacle(self, data):
        """Write one row to obstacle_log.csv."""
        self._obs_writer.writerow(data)

    def flush(self):
        self._cycle_file.flush()
        self._obs_file.flush()

    def close(self):
        self._cycle_file.close()
        self._obs_file.close()


# ─── Track centerline for goal waypoint system ───────────────────────
# Disabled: the precomputed AutoNav-course centerline doesn't match arbitrary
# worlds (e.g. full_course). Returning an empty list makes the goal-pursuit,
# preavoid bias, and centerline clamp code paths no-op so the planner relies
# purely on sensor-detected lane boundaries and obstacles.

def _build_track_centerline():
    """Disabled — return empty list. See module note above."""
    return []


class LaneFollowerNode(Node):
    def __init__(self):
        super().__init__('lane_follower_node')

        # --- Parameters ---
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('min_x', 0.5)
        self.declare_parameter('max_x', 6.0)
        self.declare_parameter('bin_width', 0.25)       # ARCH FIX: finer bins (was 0.5)
        self.declare_parameter('min_points_per_bin', 2)  # ARCH FIX: lower threshold for finer bins
        self.declare_parameter('lane_width', 3.0)
        self.declare_parameter('cruise_speed', 0.35)
        self.declare_parameter('lookahead_min', 1.5)
        self.declare_parameter('lookahead_max', 4.0)
        self.declare_parameter('speed_gain', 0.8)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('curve_slowdown_curvature', 0.5)
        self.declare_parameter('curve_slowdown_factor', 0.5)
        self.declare_parameter('no_lane_timeout', 3.0)
        self.declare_parameter('obstacle_clearance', 0.6)
        self.declare_parameter('min_gap_width', 0.5)
        self.declare_parameter('center_weight', 0.3)
        self.declare_parameter('steering_alpha', 0.50)
        self.declare_parameter('max_angular_accel', 2.0)
        self.declare_parameter('path_hysteresis', 0.5)
        self.declare_parameter('waypoints_per_gap', 3)   # ARCH FIX: multiple waypoints per gap
        self.declare_parameter('path_smooth_passes', 3)  # ARCH FIX: post-A* smoothing iterations
        self.declare_parameter('obstacle_stale_time', 15.0)  # ARCH FIX: obstacle lifetime (seconds)
        self.declare_parameter('lane_persist_time', 2.0)      # seconds to keep lane points in world frame
        self.declare_parameter('obstacle_persist_time', 10.0)  # seconds to keep obstacles in world frame
        self.declare_parameter('persist_max_behind', 3.0)   # max meters behind robot to keep points

        # GPS / No Man's Land parameters
        self.declare_parameter('gps_ref_lat', 42.674501350902)
        self.declare_parameter('gps_ref_lon', -83.218697147978)
        self.declare_parameter('nml_activate_radius', 2.0)
        self.declare_parameter('nml_exit_radius', 1.0)
        self.declare_parameter('nml_corridor_width', 3.0)
        # NML waypoint sequence: [entry_x, entry_y, wp1_x, wp1_y, ..., exit_x, exit_y]
        # Robot activates NML near entry, navigates to each waypoint in order, exits at last
        # Sequence: entry(10,9.75) → waypoint_3(0,6) → nml_exit(10,5)
        # Robot enters from ramp side heading south, goes to wp3, returns to exit
        self.declare_parameter('nml_waypoints', [10.0, 8.75, 0.0, 6.0, 10.0, 5.0])
        self.declare_parameter('nml_min_lane_points', 8)

        # Approach mode parameters
        self.declare_parameter('approach_mode', 'baseline')
        self.declare_parameter('robot_half_width', 0.325)
        self.declare_parameter('safety_margin', 0.1)
        # min/max_obstacle_radius removed — real course has unknown obstacle sizes
        self.declare_parameter('clearance_far_scale', 1.3)
        self.declare_parameter('clearance_near_scale', 0.8)
        self.declare_parameter('clearance_far_threshold', 3.0)
        self.declare_parameter('clearance_near_threshold', 1.5)
        self.declare_parameter('side_lock_weight', 2.0)
        self.declare_parameter('side_lock_decay', 0.95)

        # Track goal / recovery parameters
        self.declare_parameter('goal_ahead_distance', 10.0)
        self.declare_parameter('recovery_heading_threshold', 1.0)  # radians (~57°)
        self.declare_parameter('recovery_speed_factor', 0.5)
        self.declare_parameter('lane_centering_gain', 0.0)
        self.declare_parameter('lane_centering_threshold', 0.3)

        # known_obstacles_world removed — real course has unknown obstacle positions

        # Ramp detection via IMU pitch
        self.declare_parameter('ramp_pitch_threshold', 0.10)  # rad (~5.7°) to enter ramp mode
        self.declare_parameter('ramp_pitch_exit', 0.05)       # rad to exit ramp mode
        self.declare_parameter('ramp_speed', 0.25)            # reduced speed on ramp
        self.declare_parameter('ramp_exit_hold', 1.0)         # seconds to hold straight after pitch drops

        # --- State ---
        self.lane_points = None
        self.lane_stamp = None
        self.current_vx = 0.0
        self.obstacle_points = []       # body-frame obstacles for A*: [(bx, by, radius), ...]
        self.obstacle_points_viz = []   # all nearby body-frame obstacles for RViz viz
        self.last_lane_time = self.get_clock().now()
        self._lane_batch_start = None  # start time of current lane merge batch
        self.last_obstacle_time = self.get_clock().now()

        # Persistent world-frame clouds — accumulated from sensor data,
        # projected to body frame each cycle for planning.
        # Lane: grid-thinned {(gx,gy): timestamp} — prevents density explosion
        self._lane_world = {}
        self._lane_grid_res = 0.20  # meters per grid cell
        self._lane_viz_ttl = 30.0  # seconds to keep lane points for RViz
        # Obstacles: [(wx, wy, est_radius, timestamp), ...]
        self._obstacle_world = []
        self._log_counter = 0
        self._prev_angular = 0.0
        self._prev_path = None
        self._lane_wall_obstacles = []

        # Odom state for obstacle tracking
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        # Orbit detection: suppress obstacles when stuck
        self._orbit_check_time = self._now_sec()
        self._orbit_check_x = 0.0
        self._orbit_check_y = 0.0
        self._orbit_suppress_until = 0.0

        # World pose state (ground truth from /world_pose)
        self.world_x = 0.0
        self.world_y = 0.0
        self.world_yaw = 0.0
        self._world_pose_valid = False

        # GPS / NML state
        self.gps_x = 0.0
        self.gps_y = 0.0
        self.gps_valid = False
        self.in_nml = False
        self.nml_target = None
        self._nml_wp_index = 0         # current waypoint index in sequence
        self._nml_wp_list = []          # parsed [(x,y), ...] waypoint sequence
        self._nml_corridor_world = []
        self._nml_path_world = []
        self.gps_x_smooth = 0.0
        self.gps_y_smooth = 0.0
        self.real_lane_count = 0

        # Side commitment state (Approach B)
        self._obstacle_side_lock = {}

        # Track centerline — built incrementally as the robot drives by
        # accumulating world-frame midpoints from detected lane boundaries.
        # Open polyline (not a closed loop), grows from spawn forward.
        self._track_centerline = _build_track_centerline()  # stub returns []
        self._track_centerline_grid = set()                  # O(1) dedup keys
        self._track_centerline_grid_res = 0.20               # 20cm dedup cells
        self._track_idx = 0
        self._track_dist = 0.0
        self._goal_wx = 0.0
        self._goal_wy = 0.0
        self._track_heading = 0.0
        self._in_recovery = False
        self._recovery_enabled_time = None  # Set after first 10s of movement

        # Ramp detection state
        self._imu_pitch = 0.0
        self._on_ramp = False
        self._ramp_yaw_lock = None  # heading to hold while on ramp
        self._ramp_exit_time = None  # time when pitch dropped below exit threshold

        # --- Diagnostic logger ---
        self._diag = DiagnosticLogger(log_dir='/tmp/autonomy_diag', proximity_threshold=5.0)
        self._diag_last_cmd = {'vx': 0.0, 'az': 0.0, 'repulsive_az': 0.0, 'curve_factor': 1.0}
        self._diag_replanned = False

        # --- Subscribers ---
        self.create_subscription(PointCloud2, '/candidate/lane_points', self._lane_cb, 10)
        self.create_subscription(PointCloud2, '/obstacle_points', self._obstacle_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self._gps_cb, 10)
        self.create_subscription(PoseStamped, '/world_pose', self._world_pose_cb, 10)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)

        # --- Publishers ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/autonomy/path', 10)
        self.lookahead_pub = self.create_publisher(PointStamped, '/autonomy/lookahead', 10)
        self.obstacle_marker_pub = self.create_publisher(MarkerArray, '/autonomy/obstacles', 10)
        self.persistent_lane_pub = self.create_publisher(PointCloud2, '/autonomy/lane_cloud', 10)

        # --- Control timer ---
        rate = self.get_parameter('control_rate').value
        self.create_timer(1.0 / rate, self._control_loop)

        mode = self._p('approach_mode')
        self.get_logger().info(f'Lane follower started (A* waypoint graph, approach={mode})')

    def _p(self, name):
        return self.get_parameter(name).value

    def _now_sec(self):
        """Current time in seconds (float)."""
        return self.get_clock().now().nanoseconds / 1e9

    # ─── callbacks ───────────────────────────────────────────────────
    def _imu_cb(self, msg):
        """Extract pitch from IMU quaternion for ramp detection."""
        q = msg.orientation
        # Pitch from quaternion (rotation about Y axis)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            self._imu_pitch = math.copysign(math.pi / 2, sinp)
        else:
            self._imu_pitch = math.asin(sinp)

    def _lane_cb(self, msg):
        pts = []
        for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            pts.append((p[0], p[1]))
        if not pts:
            return
        self.lane_stamp = msg.header.stamp
        self.last_lane_time = self.get_clock().now()
        self.real_lane_count = len(pts)

        # Always populate body-frame lane_points so the control loop's wait
        # check (which runs before _project_to_body) sees data on the first
        # callback. _project_to_body() will overwrite this with the merged
        # world-frame buffer projected to body each cycle.
        self.lane_points = pts

        if not self._world_pose_valid:
            return

        # Accumulate in world frame for persistence across slow camera frames.
        cos_y = math.cos(self.world_yaw)
        sin_y = math.sin(self.world_yaw)
        wx0, wy0 = self.world_x, self.world_y
        now_sec = self._now_sec()
        grid_res = self._lane_grid_res
        for bx, by in pts:
            wx = wx0 + bx * cos_y - by * sin_y
            wy = wy0 + bx * sin_y + by * cos_y
            self._lane_world[(int(wx / grid_res), int(wy / grid_res))] = (wx, wy, now_sec)

    def _obstacle_cb(self, msg):
        raw_pts = []
        for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            if p[0] > 0:
                raw_pts.append((p[0], p[1]))
        self.last_obstacle_time = self.get_clock().now()

        if not raw_pts:
            return

        # Cluster body-frame points into obstacle centers
        centers = self._cluster_points(raw_pts, cluster_radius=0.4)

        if not self._world_pose_valid:
            # Before world pose arrives, use body-frame directly
            self.obstacle_points = [(cx, cy, sp) for cx, cy, sp in centers]
            self.obstacle_points_viz = list(self.obstacle_points)
            return

        # Transform to world frame and merge with persistent buffer.
        # Update existing obstacles if within merge radius, else add new.
        now_sec = self._now_sec()
        cos_y = math.cos(self.world_yaw)
        sin_y = math.sin(self.world_yaw)
        wx, wy = self.world_x, self.world_y
        merge_r2 = 0.36  # 0.6m merge radius (squared)

        for bx, by, spread in centers:
            world_ox = wx + bx * cos_y - by * sin_y
            world_oy = wy + bx * sin_y + by * cos_y
            # Find closest existing obstacle
            best_i = -1
            best_d2 = merge_r2
            for i, (ewx, ewy, er, _) in enumerate(self._obstacle_world):
                d2 = (world_ox - ewx) ** 2 + (world_oy - ewy) ** 2
                if d2 < best_d2:
                    best_d2 = d2
                    best_i = i
            if best_i >= 0:
                ewx, ewy, er, _ = self._obstacle_world[best_i]
                # Closer detections are more accurate — weight by inverse range
                alpha = min(0.5, 0.3 + 0.2 / max(bx, 1.0))
                self._obstacle_world[best_i] = (
                    (1 - alpha) * ewx + alpha * world_ox,
                    (1 - alpha) * ewy + alpha * world_oy,
                    (1 - alpha) * er + alpha * spread,
                    now_sec)
            else:
                self._obstacle_world.append((world_ox, world_oy, spread, now_sec))

    def _cluster_points(self, pts, cluster_radius=0.8):
        """Cluster nearby points into (center_x, center_y, spread)."""
        centers = []
        used = [False] * len(pts)
        for i in range(len(pts)):
            if used[i]:
                continue
            cluster_x = [pts[i][0]]
            cluster_y = [pts[i][1]]
            used[i] = True
            for j in range(i + 1, len(pts)):
                if used[j]:
                    continue
                if ((pts[j][0] - pts[i][0]) ** 2 +
                        (pts[j][1] - pts[i][1]) ** 2 < cluster_radius ** 2):
                    cluster_x.append(pts[j][0])
                    cluster_y.append(pts[j][1])
                    used[j] = True
            cx = sum(cluster_x) / len(cluster_x)
            cy = sum(cluster_y) / len(cluster_y)
            spread = 0.0
            for px, py in zip(cluster_x, cluster_y):
                d = math.sqrt((px - cx) ** 2 + (py - cy) ** 2)
                if d > spread:
                    spread = d
            centers.append((cx, cy, spread))
        return centers

    def _odom_cb(self, msg):
        self.current_vx = msg.twist.twist.linear.x
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_yaw = math.atan2(siny, cosy)

    def _world_pose_cb(self, msg):
        self.world_x = msg.pose.position.x
        self.world_y = msg.pose.position.y
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.world_yaw = math.atan2(siny, cosy)
        self._world_pose_valid = True


    # ─── GPS / No Man's Land ─────────────────────────────────────────
    EARTH_R = 6371000.0

    def _gps_cb(self, msg):
        if msg.status.status < 0:
            return
        raw_x, raw_y = self._gps_to_xy(msg.latitude, msg.longitude)
        if not self.gps_valid:
            self.gps_x_smooth = raw_x
            self.gps_y_smooth = raw_y
            self.get_logger().info('GPS first fix: world (%.1f, %.1f) from lat=%.6f lon=%.6f' %
                                  (raw_x, raw_y, msg.latitude, msg.longitude))
        else:
            a = 0.3
            self.gps_x_smooth = a * raw_x + (1.0 - a) * self.gps_x_smooth
            self.gps_y_smooth = a * raw_y + (1.0 - a) * self.gps_y_smooth
        self.gps_x = raw_x
        self.gps_y = raw_y
        self.gps_valid = True

    def _gps_to_xy(self, lat, lon):
        ref_lat = self._p('gps_ref_lat')
        ref_lon = self._p('gps_ref_lon')
        x = (lon - ref_lon) * math.pi / 180.0 * self.EARTH_R * math.cos(ref_lat * math.pi / 180.0)
        y = (lat - ref_lat) * math.pi / 180.0 * self.EARTH_R
        return x, y

    def _check_nml_entry(self):
        """Check if robot is near the NML entry waypoint (first in sequence)."""
        wps = self._p('nml_waypoints')
        if len(wps) < 4:
            return
        # Parse waypoint list: [entry_x, entry_y, wp1_x, wp1_y, ..., exit_x, exit_y]
        wp_list = [(wps[i], wps[i+1]) for i in range(0, len(wps) - 1, 2)]
        if len(wp_list) < 2:
            return

        entry_x, entry_y = wp_list[0]
        dist = math.sqrt((self.world_x - entry_x) ** 2 + (self.world_y - entry_y) ** 2)
        if dist < self._p('nml_activate_radius'):
            self.in_nml = True
            self._nml_wp_list = wp_list
            self._nml_wp_index = 1  # start navigating to first target (skip entry)
            self.nml_target = wp_list[1]
            self._nml_entry_wp = wp_list[0]
            self._nml_corridor_world = self._build_nml_corridor_world()
            self._nml_path_world = self._build_nml_path_world()
            self.get_logger().info(
                'NML ACTIVATED — %d waypoints, first target (%.1f, %.1f), GPS (%.1f, %.1f)' %
                (len(wp_list), self.nml_target[0], self.nml_target[1],
                 self.gps_x, self.gps_y))

    def _check_nml_exit(self):
        """Check if robot reached current NML waypoint; advance or exit."""
        if self.nml_target is None:
            return
        tx, ty = self.nml_target
        # Use world_pose (ground truth) instead of smoothed GPS for reliable detection
        dist = math.sqrt((self.world_x - tx) ** 2 + (self.world_y - ty) ** 2)
        if self._log_counter % 20 == 0:
            self.get_logger().info(
                'NML nav: target=(%.1f,%.1f) pos=(%.1f,%.1f) dist=%.1f wp=%d/%d' %
                (tx, ty, self.world_x, self.world_y, dist,
                 self._nml_wp_index, len(self._nml_wp_list)))
        if dist < self._p('nml_exit_radius'):
            self._nml_wp_index += 1
            if self._nml_wp_index < len(self._nml_wp_list):
                # Advance to next waypoint
                self.nml_target = self._nml_wp_list[self._nml_wp_index]
                # Rebuild corridor for new segment
                self._nml_entry_wp = self._nml_wp_list[self._nml_wp_index - 1]
                self._nml_corridor_world = self._build_nml_corridor_world()
                self.get_logger().info(
                    'NML WAYPOINT %d/%d reached — next target (%.1f, %.1f)' %
                    (self._nml_wp_index, len(self._nml_wp_list),
                     self.nml_target[0], self.nml_target[1]))
            else:
                # All waypoints reached — exit NML
                self.get_logger().info(
                    'NML COMPLETE — all %d waypoints reached, resuming lane following' %
                    len(self._nml_wp_list))
                self.in_nml = False
                self.nml_target = None
                self._nml_wp_list = []
                self._nml_wp_index = 0
                self._nml_corridor_world = []
                self._nml_path_world = []

    def _build_nml_corridor_world(self):
        """Generate NML corridor in world frame (called ONCE at NML activation).

        Creates boundary points from entry waypoint to exit waypoint.
        Uses waypoint coordinates (not robot GPS) for a straight corridor
        aligned with the track. These persist and are projected to body frame
        each cycle, giving heading-independent stability.
        """
        if self.nml_target is None:
            return []
        ex, ey = self.nml_target
        sx, sy = self._nml_entry_wp
        dx, dy = ex - sx, ey - sy
        length = math.sqrt(dx * dx + dy * dy)
        if length < 0.1:
            return []
        # Unit direction and normal
        ux, uy = dx / length, dy / length
        nx, ny = -uy, ux  # perpendicular
        half_w = self._p('nml_corridor_width') / 2.0
        # Extend corridor a bit behind entry and past exit
        pts = []
        d = -2.0
        while d <= length + 2.0:
            px = sx + d * ux
            py = sy + d * uy
            pts.append((px + half_w * nx, py + half_w * ny))
            pts.append((px - half_w * nx, py - half_w * ny))
            d += 0.15
        return pts

    def _project_nml_corridor(self):
        """Project world-frame NML corridor to body frame."""
        cos_y = math.cos(self.world_yaw)
        sin_y = math.sin(self.world_yaw)
        wx, wy = self.world_x, self.world_y
        pts = []
        for px, py in self._nml_corridor_world:
            dx, dy = px - wx, py - wy
            bx = dx * cos_y + dy * sin_y
            by = -dx * sin_y + dy * cos_y
            pts.append((bx, by))
        return pts

    def _build_nml_path_world(self):
        """No-op: corridor + A* with sensor-detected obstacles works better."""
        return []

    def _project_nml_path(self):
        """Project world-frame NML path to body frame, keeping only ahead points."""
        cos_y = math.cos(self.world_yaw)
        sin_y = math.sin(self.world_yaw)
        wx, wy = self.world_x, self.world_y
        pts = []
        for px, py in self._nml_path_world:
            dx, dy = px - wx, py - wy
            bx = dx * cos_y + dy * sin_y
            by = -dx * sin_y + dy * cos_y
            if bx > 0.2:  # only ahead of robot
                pts.append((bx, by))
        return pts

    def _pure_pursuit_nml(self, path):
        """Pure pursuit for NML with longer lookahead for smooth tracking."""
        if len(path) < 2:
            return None

        # Longer lookahead (2.0m) for smooth S-curve following
        L = 2.0 + self._p('speed_gain') * abs(self.current_vx)
        L = max(2.0, min(L, self._p('lookahead_max')))

        lookahead_pt = path[-1]
        for i in range(1, len(path)):
            px, py = path[i]
            d = math.sqrt(px * px + py * py)
            if d >= L:
                lookahead_pt = (px, py)
                break

        gx, gy = lookahead_pt
        L_actual = math.sqrt(gx * gx + gy * gy)
        if L_actual < 0.01:
            return None

        pt_msg = PointStamped()
        pt_msg.header.stamp = self.get_clock().now().to_msg()
        pt_msg.header.frame_id = 'base_footprint'
        pt_msg.point.x = float(gx)
        pt_msg.point.y = float(gy)
        self.lookahead_pub.publish(pt_msg)

        curvature = 2.0 * gy / (L_actual * L_actual)
        linear = self._p('cruise_speed')
        angular = linear * curvature
        angular = max(-self._p('max_angular_vel'),
                      min(angular, self._p('max_angular_vel')))

        cmd = Twist()
        cmd.linear.x = linear  # no curve slowdown in NML (gentle S-curves)
        cmd.angular.z = angular
        return cmd, 1.0

# ─── clearance computation ─────────────────────────────────────
    def _compute_obstacle_clearance(self, est_radius, obs_x=None):
        mode = self._p('approach_mode')

        if mode == 'baseline':
            return self._p('obstacle_clearance')

        if mode in ('A', 'D'):
            base = est_radius + self._p('robot_half_width') + self._p('safety_margin')
        else:
            base = self._p('obstacle_clearance')

        if mode in ('C', 'D') and obs_x is not None:
            far_thresh = self._p('clearance_far_threshold')
            near_thresh = self._p('clearance_near_threshold')
            far_scale = self._p('clearance_far_scale')
            near_scale = self._p('clearance_near_scale')
            if obs_x >= far_thresh:
                scale = far_scale
            elif obs_x <= near_thresh:
                scale = near_scale
            else:
                t = (obs_x - near_thresh) / (far_thresh - near_thresh)
                scale = near_scale + t * (far_scale - near_scale)
            base *= scale

        return base

    # ─── world→body projection + lifetime pruning ─────────────────────
    def _project_to_body(self):
        """Project persistent world-frame clouds to body frame.

        Prunes stale data by age and distance behind the robot.
        """
        now_sec = self._now_sec()
        cos_y = math.cos(self.world_yaw)
        sin_y = math.sin(self.world_yaw)
        wx, wy = self.world_x, self.world_y
        max_behind = self._p('persist_max_behind')

        # ── Lanes ──
        # Project the persistent world-frame lane cloud back to body frame.
        # NOTE: TTL pruning + behind-robot pruning are intentionally disabled.
        # Lane points accumulate for the lifetime of the run and are only
        # deduplicated by the 20cm grid in _lane_cb. The dict size is bounded
        # by the area covered (~62k entries for a 50x50m course).
        # The `lane_persist_time` parameter and `persist_max_behind` are still
        # declared but unused for lanes here — they still apply to obstacles.
        body_lanes = []
        for lwx, lwy, _ in self._lane_world.values():
            dx, dy = lwx - wx, lwy - wy
            bx = dx * cos_y + dy * sin_y
            if bx > 0:  # only points ahead of robot feed the planner
                by = -dx * sin_y + dy * cos_y
                body_lanes.append((bx, by))
        self.lane_points = body_lanes

        # ── Obstacles ──
        obs_ttl = self._p('obstacle_persist_time')
        new_obs_world = []
        body_obs = []
        body_obs_viz = []
        for owx, owy, er, t in self._obstacle_world:
            if now_sec - t > obs_ttl:
                continue
            dx, dy = owx - wx, owy - wy
            bx = dx * cos_y + dy * sin_y
            by = -dx * sin_y + dy * cos_y
            if bx < -max_behind:
                continue
            new_obs_world.append((owx, owy, er, t))
            if bx > 0:
                body_obs.append((bx, by, er))
            body_obs_viz.append((bx, by, er))
        self._obstacle_world = new_obs_world
        self.obstacle_points = body_obs
        self.obstacle_points_viz = body_obs_viz


    # ─── track centerline accumulation ───────────────────────────────
    def _accumulate_centerline(self, lane_boundaries, min_x, bin_w):
        """Append midpoints from current lane boundaries to the track centerline.

        Each control cycle, takes the per-bin (left, right) lane boundaries from
        _extract_lane_boundaries(), computes their midpoint in body frame,
        projects to world frame, and appends to self._track_centerline if not
        already represented in the dedup grid. Builds an open polyline
        incrementally as the robot drives forward.
        """
        if not lane_boundaries:
            return
        cos_y = math.cos(self.world_yaw)
        sin_y = math.sin(self.world_yaw)
        wx0, wy0 = self.world_x, self.world_y
        grid_res = self._track_centerline_grid_res

        # Sort by bin index so points are appended in forward-X order. The
        # polyline order matters for goal-walking and `(idx + N)` lookups.
        for bi in sorted(lane_boundaries.keys()):
            left_b, right_b = lane_boundaries[bi]
            mid_y_body = (left_b + right_b) / 2.0
            x_body = min_x + (bi + 0.5) * bin_w
            # Body → world
            wx = wx0 + x_body * cos_y - mid_y_body * sin_y
            wy = wy0 + x_body * sin_y + mid_y_body * cos_y
            grid_key = (int(wx / grid_res), int(wy / grid_res))
            if grid_key not in self._track_centerline_grid:
                self._track_centerline_grid.add(grid_key)
                self._track_centerline.append((wx, wy))

    # ─── track goal waypoint system ──────────────────────────────────
    def _update_track_goal(self):
        """Find closest centerline point, set goal ahead, compute track heading."""
        cl = self._track_centerline
        if not cl:
            return
        wx, wy = self.world_x, self.world_y
        n = len(cl)

        # Find closest centerline point (search near previous index first).
        # Open polyline: clamp the search window instead of wrapping.
        best_d2 = float('inf')
        best_idx = self._track_idx
        search_range = 50
        lo = max(0, self._track_idx - search_range)
        hi = min(n, self._track_idx + search_range + 1)
        for i in range(lo, hi):
            d2 = (wx - cl[i][0]) ** 2 + (wy - cl[i][1]) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_idx = i
        # If not close enough, do full scan (e.g., at startup)
        if best_d2 > 4.0:
            for i in range(n):
                d2 = (wx - cl[i][0]) ** 2 + (wy - cl[i][1]) ** 2
                if d2 < best_d2:
                    best_d2 = d2
                    best_idx = i
        self._track_idx = best_idx
        self._track_dist = math.sqrt(best_d2)

        # Track heading: use ~2m baseline for stability at arc/straight transitions
        lookahead_pts = 8  # 8 * 0.25m = 2m baseline
        # Open polyline — clamp at end instead of wrapping
        i_ahead = min(best_idx + lookahead_pts, n - 1)
        if i_ahead == best_idx and best_idx > 0:
            i_ahead = max(best_idx - lookahead_pts, 0)
        dx = cl[i_ahead][0] - cl[best_idx][0]
        dy = cl[i_ahead][1] - cl[best_idx][1]
        # Sign-flip if we had to walk backward to get a tangent
        if i_ahead < best_idx:
            dx, dy = -dx, -dy
        self._track_heading = math.atan2(dy, dx)

        # Walk forward along centerline to find goal — stop at end (no wrap)
        goal_dist = self._p('goal_ahead_distance')
        cum = 0.0
        gi = best_idx
        while cum < goal_dist and gi < n - 1:
            gi_next = gi + 1
            sdx = cl[gi_next][0] - cl[gi][0]
            sdy = cl[gi_next][1] - cl[gi][1]
            seg_len = math.sqrt(sdx * sdx + sdy * sdy)
            if seg_len < 0.001:
                gi = gi_next
                continue
            remain = goal_dist - cum
            if remain <= seg_len:
                frac = remain / seg_len
                self._goal_wx = cl[gi][0] + frac * sdx
                self._goal_wy = cl[gi][1] + frac * sdy
                return
            cum += seg_len
            gi = gi_next
        self._goal_wx = cl[gi][0]
        self._goal_wy = cl[gi][1]

    def _get_heading_error(self):
        """Signed heading error between robot yaw and track direction."""
        err = self._track_heading - self.world_yaw
        # Normalize to [-pi, pi]
        err = (err + math.pi) % (2 * math.pi) - math.pi
        return err

    def _goal_pursuit(self, obstacles):
        """Drive toward the track goal waypoint with obstacle avoidance.

        Used when heading is far from track direction (e.g., after a big dodge).
        Pure pursuit toward the world-frame goal projected to body frame,
        with obstacle repulsion to avoid collisions.
        """
        cos_y = math.cos(self.world_yaw)
        sin_y = math.sin(self.world_yaw)
        dx = self._goal_wx - self.world_x
        dy = self._goal_wy - self.world_y
        goal_bx = dx * cos_y + dy * sin_y
        goal_by = -dx * sin_y + dy * cos_y

        L = math.sqrt(goal_bx * goal_bx + goal_by * goal_by)
        if L < 0.3:
            return None

        # Proportional heading toward goal (more aggressive than pure pursuit)
        desired_heading = math.atan2(goal_by, goal_bx)
        speed = self._p('cruise_speed') * self._p('recovery_speed_factor')
        angular = 1.5 * desired_heading  # proportional gain
        angular = max(-self._p('max_angular_vel'), min(angular, self._p('max_angular_vel')))

        # Obstacle repulsion: push away from nearby obstacles
        for ox, oy, _ in obstacles:
            dist = math.sqrt(ox * ox + oy * oy)
            if dist < 3.0 and ox > 0.0:
                push = (-oy) / max(dist * dist, 0.5)
                angular += max(-0.3, min(0.3, push))

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = max(-self._p('max_angular_vel'),
                            min(angular, self._p('max_angular_vel')))

        # Publish goal for RViz
        pt_msg = PointStamped()
        pt_msg.header.stamp = self.get_clock().now().to_msg()
        pt_msg.header.frame_id = 'base_footprint'
        pt_msg.point.x = float(goal_bx)
        pt_msg.point.y = float(goal_by)
        self.lookahead_pub.publish(pt_msg)

        return cmd, goal_bx, goal_by

    # ─── side lock helpers (Approach B) ───────────────────────────────
    def _get_side_lock_key(self, odom_x, odom_y):
        return (round(odom_x * 2) / 2, round(odom_y * 2) / 2)

    def _update_side_locks(self, path, obstacles_body):
        mode = self._p('approach_mode')
        if mode not in ('A', 'B', 'D'):
            return
        for bx, by, est_r in obstacles_body:
            path_y_at_obs = None
            for i in range(1, len(path)):
                x0, y0 = path[i - 1]
                x1, y1 = path[i]
                if x0 <= bx <= x1 and x1 > x0:
                    t = (bx - x0) / (x1 - x0)
                    path_y_at_obs = y0 + t * (y1 - y0)
                    break
            if path_y_at_obs is None:
                continue
            side = 'left' if path_y_at_obs > by else 'right'
            cos_y = math.cos(self.odom_yaw)
            sin_y = math.sin(self.odom_yaw)
            ox = self.odom_x + bx * cos_y - by * sin_y
            oy = self.odom_y + bx * sin_y + by * cos_y
            key = self._get_side_lock_key(ox, oy)
            if key in self._obstacle_side_lock:
                old_side, old_conf = self._obstacle_side_lock[key]
                if old_side == side:
                    new_conf = min(old_conf + 0.1, 1.0)
                else:
                    new_conf = old_conf - 0.2
                    if new_conf <= 0:
                        side = 'left' if side == 'right' else 'right'
                        new_conf = 0.1
                    else:
                        side = old_side
                self._obstacle_side_lock[key] = (side, new_conf)
            else:
                self._obstacle_side_lock[key] = (side, 0.5)

    def _decay_side_locks(self):
        mode = self._p('approach_mode')
        if mode not in ('A', 'B', 'D'):
            return
        decay = self._p('side_lock_decay')
        to_remove = []
        for key, (side, conf) in self._obstacle_side_lock.items():
            new_conf = conf * decay
            if new_conf < 0.05:
                to_remove.append(key)
            else:
                self._obstacle_side_lock[key] = (side, new_conf)
        for key in to_remove:
            del self._obstacle_side_lock[key]

    def _get_side_lock_penalty(self, from_y, to_y, obs_body):
        mode = self._p('approach_mode')
        if mode not in ('A', 'B', 'D'):
            return 0.0
        penalty = 0.0
        weight = self._p('side_lock_weight')
        cos_y = math.cos(self.odom_yaw)
        sin_y = math.sin(self.odom_yaw)
        for bx, by, est_r in obs_body:
            ox = self.odom_x + bx * cos_y - by * sin_y
            oy = self.odom_y + bx * sin_y + by * cos_y
            key = self._get_side_lock_key(ox, oy)
            if key not in self._obstacle_side_lock:
                continue
            locked_side, conf = self._obstacle_side_lock[key]
            mid_y = (from_y + to_y) / 2.0
            actual_side = 'left' if mid_y > by else 'right'
            if actual_side != locked_side:
                penalty += weight * conf
        return penalty

    # ─── lane reconstruction + boundary extraction ─────────────────
    def _reconstruct_lane_walls(self):
        """Cluster lane points, RANSAC fit, generate wall obstacle points.

        Returns list of (x, y, radius) pseudo-obstacles along each fitted
        lane line. These are fed to the A* as impassable obstacles so the
        planner treats lane lines as physical walls.
        """
        if not self.lane_points or len(self.lane_points) < 10:
            return []

        min_x = self._p('min_x')
        max_x = self._p('max_x')

        pts = [(x, y) for x, y in self.lane_points if min_x <= x < max_x]
        if len(pts) < 10:
            return []

        # Grid-based clustering (0.5m cells)
        cell_size = 0.5
        grid = {}
        for x, y in pts:
            gx, gy = int(x / cell_size), int(y / cell_size)
            grid.setdefault((gx, gy), []).append((x, y))

        visited = set()
        clusters = []
        for key in grid:
            if key in visited:
                continue
            cluster_pts = []
            queue = [key]
            visited.add(key)
            while queue:
                gx, gy = queue.pop(0)
                cluster_pts.extend(grid.get((gx, gy), []))
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        nk = (gx + dx, gy + dy)
                        if nk in grid and nk not in visited:
                            visited.add(nk)
                            queue.append(nk)
            if len(cluster_pts) >= 8:
                clusters.append(cluster_pts)

        # Take top 2 clusters as lane lines
        clusters.sort(key=len, reverse=True)
        clusters = clusters[:2]

        if not clusters:
            return []

        # RANSAC fit each cluster
        def ransac_fit(points, n_iter=50, thresh=0.15):
            if len(points) < 4:
                return None
            pts_arr = np.array(points)
            px, py = pts_arr[:, 0], pts_arr[:, 1]
            best_inliers = 0
            best_ab = None
            for _ in range(n_iter):
                idx = np.random.choice(len(pts_arr), 2, replace=False)
                x1, y1 = px[idx[0]], py[idx[0]]
                x2, y2 = px[idx[1]], py[idx[1]]
                ddx = x2 - x1
                if abs(ddx) < 0.01:
                    continue
                a = (y2 - y1) / ddx
                b = y1 - a * x1
                residuals = np.abs(py - (a * px + b))
                inliers = np.sum(residuals < thresh)
                if inliers > best_inliers:
                    best_inliers = inliers
                    best_ab = (a, b)
            if best_ab is None or best_inliers < 4:
                return None
            a, b = best_ab
            inlier_mask = np.abs(py - (a * px + b)) < thresh
            if np.sum(inlier_mask) >= 4:
                a, b = np.polyfit(px[inlier_mask], py[inlier_mask], 1)
            return (a, b)

        fits = [ransac_fit(c) for c in clusters]
        fits = [f for f in fits if f is not None]

        if not fits:
            return []

        # Generate dense wall points along each fitted line
        # These act as impassable obstacles for the A*
        wall_radius = 0.05  # thin wall
        wall_spacing = 0.2  # point every 0.2m
        wall_points = []

        for a, b in fits:
            x = min_x
            while x <= max_x:
                y = a * x + b
                wall_points.append((x, y, wall_radius))
                x += wall_spacing

        return wall_points

    def _extract_lane_boundaries(self):
        """Extract lane boundaries from reconstructed lane walls.

        Uses RANSAC-fitted lane lines to define boundaries per bin.
        The fitted lines are also converted to wall obstacles that the
        A* cannot cross.
        """
        min_x = self._p('min_x')
        max_x = self._p('max_x')
        bin_w = self._p('bin_width')
        min_pts = self._p('min_points_per_bin')
        half_lane = self._p('lane_width') / 2.0
        n_bins = int((max_x - min_x) / bin_w)
        lane_w = self._p('lane_width')

        # Reconstruct lane walls
        self._lane_wall_obstacles = self._reconstruct_lane_walls()

        if not self._lane_wall_obstacles:
            return {}, n_bins

        # Group wall points by X-bin to extract boundaries
        wall_bins = [[] for _ in range(n_bins)]
        for wx, wy, wr in self._lane_wall_obstacles:
            if wx < min_x or wx >= max_x:
                continue
            idx = min(int((wx - min_x) / bin_w), n_bins - 1)
            wall_bins[idx].append(wy)

        # Per bin: distinguish "two distinct lines" from "many points on one
        # line." When _reconstruct_lane_walls fits a single RANSAC line, it
        # generates dense wall points along that one line, so a bin can have
        # several Y values that are all clustered together (≈ same line_y).
        # If we naively treated that as "two lines", the synthesized boundary
        # would be degenerate (left ≈ right) and the bin would be skipped as
        # "lane too narrow", which collapses single-lane behavior entirely.
        # Use a Y-spread threshold to disambiguate: real lanes are ~3m apart,
        # same-line clutter is well under 0.5m.
        SINGLE_LINE_SPREAD_THRESH = 0.5
        lane_boundaries = {}
        for i in range(n_bins):
            ys = sorted(wall_bins[i])
            if len(ys) >= 2 and (ys[-1] - ys[0]) > SINGLE_LINE_SPREAD_THRESH:
                # Two distinct lines present in this bin
                lane_boundaries[i] = (ys[-1], ys[0])
            elif ys:
                # Single line (or multiple same-line points) — synthesize
                # phantom boundary at lane_w distance on the opposite side.
                line_y = (ys[0] + ys[-1]) / 2.0  # average for stability
                if line_y > 0:
                    # Visible line is to the left of robot — phantom on the right
                    lane_boundaries[i] = (line_y, line_y - lane_w)
                else:
                    # Visible line is to the right of robot — phantom on the left
                    lane_boundaries[i] = (line_y + lane_w, line_y)

        # Fill gaps
        for i in range(n_bins):
            if i not in lane_boundaries:
                for offset in (1, -1, 2, -2, 3, -3):
                    ni = i + offset
                    if ni in lane_boundaries:
                        lane_boundaries[i] = lane_boundaries[ni]
                        break

        return lane_boundaries, n_bins

    # ─── edge collision check ────────────────────────────────────────
    def _segment_clear(self, x1, y1, x2, y2, obstacles, curve_scale=1.0):
        """True if line segment doesn't pass within clearance of any obstacle.
        curve_scale: multiply clearance to account for lane curve angle."""
        dx = x2 - x1
        dy = y2 - y1
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-6:
            return True
        for ox, oy, est_r in obstacles:
            t = ((ox - x1) * dx + (oy - y1) * dy) / seg_len_sq
            t = max(0.0, min(1.0, t))
            cx = x1 + t * dx
            cy = y1 + t * dy
            dist_sq = (ox - cx) ** 2 + (oy - cy) ** 2
            clr = self._compute_obstacle_clearance(est_r, obs_x=ox) * curve_scale
            if dist_sq < clr * clr:
                return False
        return True

    def _adaptive_nav_margin(self, left_b, right_b):
        """Nav margin that scales with lane width."""
        span = abs(left_b - right_b)
        if span > 2.5:
            return 0.25  # wide lanes
        elif span < 1.8:
            return 0.15  # narrow lanes (ramp funnel)
        else:
            t = (span - 1.8) / (2.5 - 1.8)
            return 0.15 + t * 0.10

    # ─── post-A* path smoothing ──────────────────────────────────────
    def _clamp_to_lane(self, sx, sy, lane_boundaries, min_x, bin_w):
        """Clamp a point's Y to stay within lane boundaries for its bin.

        With the hardcoded track centerline disabled, the centerline fallback
        is gone — when no bin data is available, the point is left unclamped.
        """
        if not lane_boundaries:
            return sy
        bi = int((sx - min_x) / bin_w)
        # Check exact bin and neighbors
        for offset in (0, 1, -1, 2, -2, 3, -3):
            nbi = bi + offset
            if nbi in lane_boundaries:
                left_b, right_b = lane_boundaries[nbi]
                if left_b < right_b:
                    left_b, right_b = right_b, left_b
                nav_margin = self._adaptive_nav_margin(left_b, right_b)
                return max(right_b + nav_margin, min(left_b - nav_margin, sy))
        # No bin data at all — leave unclamped (centerline fallback removed)
        return sy

    def _clamp_to_centerline(self, bx, by):
        """Clamp body-frame point to track centerline ± half lane width.

        No-op when the centerline is empty or too short to compute a tangent.
        """
        cl = self._track_centerline
        if len(cl) < 5:
            return by
        # Project body-frame point to world frame
        cos_y = math.cos(self.world_yaw)
        sin_y = math.sin(self.world_yaw)
        wpx = self.world_x + bx * cos_y - by * sin_y
        wpy = self.world_y + bx * sin_y + by * cos_y
        # Find nearest centerline point
        best_d2 = float('inf')
        best_ci = 0
        for ci in range(len(cl)):
            d2 = (wpx - cl[ci][0]) ** 2 + (wpy - cl[ci][1]) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_ci = ci
        # Compute local tangent — clamp to end (open polyline, no wrap)
        ci_next = min(best_ci + 4, len(cl) - 1)
        if ci_next == best_ci:
            ci_next = max(best_ci - 4, 0)
            if ci_next == best_ci:
                return by
        tdx = cl[ci_next][0] - cl[best_ci][0]
        tdy = cl[ci_next][1] - cl[best_ci][1]
        tlen = math.sqrt(tdx * tdx + tdy * tdy)
        if tlen < 0.01:
            return by
        odx = wpx - cl[best_ci][0]
        ody = wpy - cl[best_ci][1]
        lateral = (odx * tdy - ody * tdx) / tlen
        # Clamp lateral to ± (half_lane - nav_margin)
        half_lane = self._p('lane_width') / 2.0
        max_lat = half_lane - 0.25
        if abs(lateral) <= max_lat:
            return by  # already within lane
        # Push back toward centerline
        excess = abs(lateral) - max_lat
        # Direction: if lateral > 0 (left of center), push right (negative by shift)
        # Need to convert world-frame correction to body-frame Y
        if lateral > 0:
            return by - excess
        else:
            return by + excess

    def _smooth_path(self, path, obstacles, lane_boundaries=None, min_x=0.5, bin_w=0.25):
        """Smooth the A* path with averaging + obstacle/lane clamping.

        1. Densify: interpolate to ~0.15m spacing, clamping each point to lane
        2. Iterative averaging with obstacle clamping and lane boundary clamping:
           smooth but never pull the path outside the lane or into obstacles.
        """
        if len(path) < 3:
            return path

        # Densify: linear interpolation to ~0.15m spacing
        dense = [path[0]]
        for i in range(1, len(path)):
            x0, y0 = dense[-1]
            x1, y1 = path[i]
            seg_len = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
            n_steps = max(1, int(seg_len / 0.15))
            for s in range(1, n_steps + 1):
                t = s / n_steps
                px = x0 + t * (x1 - x0)
                py = y0 + t * (y1 - y0)
                dense.append((px, py))

        n_passes = self._p('path_smooth_passes')
        for _ in range(n_passes):
            smoothed = [dense[0]]
            for i in range(1, len(dense) - 1):
                sx = 0.15 * dense[i - 1][0] + 0.7 * dense[i][0] + 0.15 * dense[i + 1][0]
                sy = 0.15 * dense[i - 1][1] + 0.7 * dense[i][1] + 0.15 * dense[i + 1][1]
                # Lane boundary clamping first (soft constraint)
                if lane_boundaries:
                    sy = self._clamp_to_lane(sx, sy, lane_boundaries, min_x, bin_w)
                # Obstacle clamping last (hard constraint — always wins over lane)
                for ox, oy, er in obstacles:
                    clr = self._compute_obstacle_clearance(er, obs_x=ox)
                    ddx = sx - ox
                    ddy = sy - oy
                    dist = math.sqrt(ddx * ddx + ddy * ddy)
                    if dist < clr and dist > 0.01:
                        sx = ox + ddx / dist * clr
                        sy = oy + ddy / dist * clr
                smoothed.append((sx, sy))
            smoothed.append(dense[-1])
            dense = smoothed

        # Final obstacle-only clamping passes (no averaging).
        # Two passes to handle cascading — pushing away from one obstacle
        # can push into another; second pass fixes that.
        for _pass in range(2):
            for i in range(1, len(dense) - 1):
                sx, sy = dense[i]
                for ox, oy, er in obstacles:
                    clr = self._compute_obstacle_clearance(er, obs_x=ox)
                    ddx = sx - ox
                    ddy = sy - oy
                    dist = math.sqrt(ddx * ddx + ddy * ddy)
                    if dist < clr and dist > 0.01:
                        sx = ox + ddx / dist * clr
                        sy = oy + ddy / dist * clr
                dense[i] = (sx, sy)

        return dense

    # ─── centerline path planning with obstacle deviation ────────────
    def _plan_path(self):
        """Plan path by following the track centerline, deviating for obstacles.

        1. Project track centerline to body frame (next ~8m)
        2. For each obstacle, if it blocks the path, shift path points to clear it
        3. Lock the deviation side per obstacle (world-frame, prevents oscillation)
        4. Smooth the result
        """
        max_range = self._p('max_x')
        half_lane = self._p('lane_width') / 2.0

        # Step 1: Project centerline ahead into body frame
        cos_y = math.cos(self.world_yaw)
        sin_y = math.sin(self.world_yaw)
        cl = self._track_centerline
        n = len(cl)
        idx = self._track_idx

        path = [(0.0, 0.0)]  # start at robot
        for step in range(1, int(max_range / 0.2) + 1):
            # Walk forward along centerline from current track position
            ci = (idx + step * 1) % n  # ~0.25m per centerline point, sample every 1
            wx, wy = cl[ci]
            dx, dy = wx - self.world_x, wy - self.world_y
            bx = dx * cos_y + dy * sin_y
            by = -dx * sin_y + dy * cos_y
            if bx > max_range:
                break
            if bx > 0.2:
                path.append((bx, by))

        if len(path) < 3:
            return None

        # Step 2: Deviate around obstacles (stateless — side determined by geometry)
        obstacles = [(ox, oy, er) for ox, oy, er in self.obstacle_points if ox > 0.3]

        for ox, oy, er in obstacles:
            clr = self._compute_obstacle_clearance(er, obs_x=ox)

            # Find the centerline Y at the obstacle's X position
            center_y = 0.0
            for i in range(len(path) - 1):
                if path[i][0] <= ox <= path[i+1][0]:
                    t = (ox - path[i][0]) / max(path[i+1][0] - path[i][0], 0.01)
                    center_y = path[i][1] + t * (path[i+1][1] - path[i][1])
                    break

            # Deterministic side: dodge away from obstacle relative to centerline
            obstacle_offset = oy - center_y
            if obstacle_offset >= 0:
                target_y = oy - clr  # obstacle is left/center → dodge right
            else:
                target_y = oy + clr  # obstacle is right → dodge left

            # Clamp target to lane bounds
            target_y = max(-half_lane, min(half_lane, target_y))

            # Shift path points in the obstacle's influence zone
            influence = clr + 1.0
            for i in range(len(path)):
                px, py = path[i]
                x_dist = abs(px - ox)
                if x_dist < influence:
                    blend = max(0.0, 1.0 - x_dist / influence)
                    path[i] = (px, py + blend * (target_y - py))

        # Step 3: Smooth
        for _ in range(3):
            smoothed = [path[0]]
            for i in range(1, len(path) - 1):
                sx = 0.15 * path[i-1][0] + 0.7 * path[i][0] + 0.15 * path[i+1][0]
                sy = 0.15 * path[i-1][1] + 0.7 * path[i][1] + 0.15 * path[i+1][1]
                smoothed.append((sx, sy))
            smoothed.append(path[-1])
            path = smoothed

        # Final obstacle clamping
        for i in range(1, len(path) - 1):
            sx, sy = path[i]
            for ox, oy, er in obstacles:
                clr = self._compute_obstacle_clearance(er, obs_x=ox)
                ddx = sx - ox
                ddy = sy - oy
                dist = math.sqrt(ddx * ddx + ddy * ddy)
                if dist < clr and dist > 0.01:
                    sx = ox + ddx / dist * clr
                    sy = oy + ddy / dist * clr
            path[i] = (sx, sy)

        self._prev_path = path
        return path

    def _plan_path_legacy(self):
        lane_boundaries, n_bins = self._extract_lane_boundaries()
        if not lane_boundaries:
            return None

        min_x = self._p('min_x')
        max_x = self._p('max_x')
        bin_w = self._p('bin_width')

        # Grow the world-frame centerline polyline from this frame's lanes.
        # Replaces the old hardcoded AutoNav-course centerline.
        self._accumulate_centerline(lane_boundaries, min_x, bin_w)
        min_gap = self._p('min_gap_width')
        center_w = self._p('center_weight')
        hyst_w = self._p('path_hysteresis')
        if self.in_nml:
            hyst_w *= 5.0  # Strong path commitment in NML to prevent oscillation
        n_waypoints = self._p('waypoints_per_gap')

        obstacles = list(self.obstacle_points)

        # Build previous path lookup by bin index for hysteresis
        prev_y_by_bin = {}
        if self._prev_path and hyst_w > 0.0:
            for ppx, ppy in self._prev_path:
                if ppx >= min_x and ppx < max_x:
                    bi = int((ppx - min_x) / bin_w)
                    if bi not in prev_y_by_bin:
                        prev_y_by_bin[bi] = ppy

        # ── Build waypoint layers ──
        layers = []
        layer_indices = []

        # Max clearance for nearby filter
        max_clr = self._p('obstacle_clearance')
        mode = self._p('approach_mode')
        if mode in ('A', 'D'):
            max_sensor_r = max((er for _, _, er in obstacles), default=0.3)
            max_clr = (max_sensor_r +
                       self._p('robot_half_width') +
                       self._p('safety_margin'))
            if mode in ('C', 'D'):
                max_clr *= self._p('clearance_far_scale')
        elif mode in ('C',):
            max_clr *= self._p('clearance_far_scale')

        # Pre-compute lane center per bin for direction estimation
        bin_centers = {}
        for i in lane_boundaries:
            lb, rb = lane_boundaries[i]
            if lb < rb:
                lb, rb = rb, lb
            bin_centers[i] = (lb + rb) / 2.0

        # Compute lane direction correction per bin.
        # At curves, body-frame Y gaps overestimate the true perpendicular
        # gap. We scale obstacle clearance by 1/cos(angle) to compensate.
        lane_angle_scale = {}
        sorted_bins = sorted(bin_centers.keys())
        for idx, i in enumerate(sorted_bins):
            # Compute lane direction from neighboring bins
            if idx > 0 and idx < len(sorted_bins) - 1:
                i_prev, i_next = sorted_bins[idx - 1], sorted_bins[idx + 1]
                dy = bin_centers[i_next] - bin_centers[i_prev]
                dx = (i_next - i_prev) * bin_w
            elif idx == 0 and len(sorted_bins) > 1:
                i_next = sorted_bins[1]
                dy = bin_centers[i_next] - bin_centers[i]
                dx = (i_next - i) * bin_w
            elif idx == len(sorted_bins) - 1 and len(sorted_bins) > 1:
                i_prev = sorted_bins[-2]
                dy = bin_centers[i] - bin_centers[i_prev]
                dx = (i - i_prev) * bin_w
            else:
                dy, dx = 0.0, 1.0
            length = math.sqrt(dx * dx + dy * dy)
            if length > 0.01:
                # cos(angle) = dx/length — how aligned the lane is with body X
                cos_a = abs(dx) / length
                # Clamp to prevent extreme scaling (max 2x at >60° curves)
                lane_angle_scale[i] = 1.0 / max(0.5, cos_a)
            else:
                lane_angle_scale[i] = 1.0

        # Gap consistency: for each obstacle, check if north/south gap is
        # viable at the bin closest to the obstacle (tightest point).
        # If not, block that side at ALL bins — prevents the A* from routing
        # through gaps that widen due to chicane but close at the obstacle.
        obs_block_north = set()  # indices into obstacles list
        obs_block_south = set()
        for oi, (ox, oy, er) in enumerate(obstacles):
            obs_bin = max(0, min(n_bins - 1, int((ox - min_x) / bin_w)))
            if obs_bin not in lane_boundaries:
                continue
            lb, rb = lane_boundaries[obs_bin]
            if lb < rb:
                lb, rb = rb, lb
            nm = self._adaptive_nav_margin(lb, rb)
            nl = lb - nm
            nr = rb + nm
            cs = lane_angle_scale.get(obs_bin, 1.0)
            clr = self._compute_obstacle_clearance(er, obs_x=ox) * cs
            if nl - (oy + clr) < min_gap:
                obs_block_north.add(oi)
            if (oy - clr) - nr < min_gap:
                obs_block_south.add(oi)

        for i in range(n_bins):
            x_center = min_x + (i + 0.5) * bin_w
            if i not in lane_boundaries:
                continue

            left_b, right_b = lane_boundaries[i]
            if left_b < right_b:
                left_b, right_b = right_b, left_b

            lane_center = (left_b + right_b) / 2.0

            # Navigable zone: keep robot center slightly inside lane lines
            nav_margin = self._adaptive_nav_margin(left_b, right_b)
            nav_left = left_b - nav_margin
            nav_right = right_b + nav_margin
            if nav_left <= nav_right:
                continue  # Lane too narrow for robot

            # Lane curve correction: scale clearance to account for
            # body-frame Y overestimating true perpendicular gap at curves
            curve_scale = lane_angle_scale.get(i, 1.0)

            nearby = [(oi, ox, oy, er) for oi, (ox, oy, er) in enumerate(obstacles)
                      if abs(ox - x_center) < (max_clr * curve_scale) + bin_w]

            # Build blocked intervals (clamped to navigable zone)
            blocked = []
            for oi, ox, oy, er in nearby:
                clr = self._compute_obstacle_clearance(er, obs_x=ox) * curve_scale
                lo = max(nav_right, oy - clr)
                hi = min(nav_left, oy + clr)
                # Gap consistency: if north/south gap is blocked at tightest
                # bin, extend the blocked interval to close that side here too
                if oi in obs_block_north:
                    hi = nav_left
                if oi in obs_block_south:
                    lo = nav_right
                if lo < hi:
                    blocked.append((lo, hi))

            blocked.sort()
            merged = []
            for lo, hi in blocked:
                if merged and lo <= merged[-1][1]:
                    merged[-1] = (merged[-1][0], max(merged[-1][1], hi))
                else:
                    merged.append((lo, hi))

            # Find free gaps with MULTIPLE waypoints per gap
            waypoints = []
            prev = nav_right
            for lo, hi in merged:
                gap_w = lo - prev
                if gap_w >= min_gap:
                    self._add_gap_waypoints(waypoints, x_center, prev, lo,
                                            lane_center, n_waypoints)
                prev = hi
            gap_w = nav_left - prev
            if gap_w >= min_gap:
                self._add_gap_waypoints(waypoints, x_center, prev, nav_left,
                                        lane_center, n_waypoints)

            if not waypoints:
                continue

            # Filter waypoints on the blocked side of gap-consistency obstacles.
            # This prevents A* from routing through a gap that looks open at
            # far-away bins but is too narrow at the obstacle's actual position.
            if obs_block_north or obs_block_south:
                filtered = []
                for (wx, wy, lc) in waypoints:
                    remove = False
                    for oi in obs_block_north:
                        if oi < len(obstacles):
                            _, oy, _ = obstacles[oi]
                            if wy > oy:  # north (+y) side blocked
                                remove = True
                                break
                    if not remove:
                        for oi in obs_block_south:
                            if oi < len(obstacles):
                                _, oy, _ = obstacles[oi]
                                if wy < oy:  # south (-y) side blocked
                                    remove = True
                                    break
                    if not remove:
                        filtered.append((wx, wy, lc))
                if filtered:
                    waypoints = filtered

            layers.append(waypoints)
            layer_indices.append(i)
            # Store curve scale per layer for segment checks
            # (not defined yet — will be set below)

        if len(layers) < 2:
            return None

        # ── Pre-avoidance bias from distant world-frame obstacles ──
        # For obstacles ahead, use the TRACK CENTERLINE (which knows chicane
        # geometry) to determine which side of the lane the obstacle is on,
        # then bias the A* path to the opposite side.
        # SKIPPED when track centerline is disabled (empty) — the planner
        # falls back to pure A* through detected lane gaps.
        preavoid_bias_y = None  # target Y in body frame (None = no bias)
        preavoid_strength = 0.0
        cl = self._track_centerline
        if cl:
            cos_y = math.cos(self.world_yaw)
            sin_y = math.sin(self.world_yaw)
            for owx, owy, er, _ in self._obstacle_world:
                dx, dy = owx - self.world_x, owy - self.world_y
                bx = dx * cos_y + dy * sin_y
                by = -dx * sin_y + dy * cos_y
                if bx < 3.0 or bx > 20.0:
                    continue
                # Find obstacle's lateral offset from track centerline (world frame).
                # This is robust to chicane because the centerline follows it.
                best_d2 = float('inf')
                best_ci = 0
                for ci in range(len(cl)):
                    d2 = (owx - cl[ci][0]) ** 2 + (owy - cl[ci][1]) ** 2
                    if d2 < best_d2:
                        best_d2 = d2
                        best_ci = ci
                # Compute local tangent — open polyline, clamp to end
                ci_next = min(best_ci + 4, len(cl) - 1)
                if ci_next == best_ci:
                    ci_next = max(best_ci - 4, 0)
                    if ci_next == best_ci:
                        continue
                tdx = cl[ci_next][0] - cl[best_ci][0]
                tdy = cl[ci_next][1] - cl[best_ci][1]
                tlen = math.sqrt(tdx * tdx + tdy * tdy)
                if tlen < 0.01:
                    continue
                # Cross product gives signed lateral offset
                # Positive = obstacle is to the left of track direction
                odx = owx - cl[best_ci][0]
                ody = owy - cl[best_ci][1]
                lateral = (tdx * ody - tdy * odx) / tlen
                # Bias to the opposite side: if obstacle is left (+), bias right (-)
                # For centered obstacles, bias toward the side the robot is already on
                half_lane = self._p('lane_width') / 2.0
                if abs(lateral) < 0.15:
                    # Obstacle is on centerline — bias toward robot's current side
                    # Use the robot's lateral offset from the obstacle to pick a side
                    if by > 0:
                        lateral = -0.3  # robot is left of obstacle → treat as right-of-center → bias left
                    else:
                        lateral = 0.3   # robot is right of obstacle → treat as left-of-center → bias right
                # World-frame bias direction: perpendicular to track, away from obstacle
                nx, ny = -tdy / tlen, tdx / tlen  # left-pointing normal
                if lateral > 0:
                    # Obstacle is left of center → bias right
                    bias_wx = cl[best_ci][0] - nx * (half_lane * 0.4)
                    bias_wy = cl[best_ci][1] - ny * (half_lane * 0.4)
                else:
                    # Obstacle is right of center → bias left
                    bias_wx = cl[best_ci][0] + nx * (half_lane * 0.4)
                    bias_wy = cl[best_ci][1] + ny * (half_lane * 0.4)
                # Project bias point to body frame
                bdx, bdy = bias_wx - self.world_x, bias_wy - self.world_y
                bias_by = -bdx * sin_y + bdy * cos_y
                # Strength ramps up as obstacle gets closer
                strength = min(3.0, 8.0 / max(bx, 1.0))
                if strength > preavoid_strength:
                    preavoid_bias_y = bias_by
                    preavoid_strength = strength

        # Curve scale per layer (for segment_clear calls)
        layer_curve_scales = [lane_angle_scale.get(bi, 1.0) for bi in layer_indices]

        # ── Build graph nodes ──
        nodes = {0: (0.0, 0.0)}
        node_centers = {0: 0.0}
        nid = 1
        layer_nids = []

        for layer in layers:
            ids = []
            for (wx, wy, lc) in layer:
                nodes[nid] = (wx, wy)
                node_centers[nid] = lc
                ids.append(nid)
                nid += 1
            layer_nids.append(ids)

        goal_ids = set(layer_nids[-1])
        goal_x = layers[-1][0][0]

        # ── Build adjacency list ──
        adj = {n: [] for n in nodes}

        # Start → first layer
        cs0 = layer_curve_scales[0] if layer_curve_scales else 1.0
        for to_id in layer_nids[0]:
            tx, ty = nodes[to_id]
            if self._segment_clear(0.0, 0.0, tx, ty, obstacles, cs0):
                dist = math.sqrt(tx * tx + ty * ty)
                center_pen = center_w * abs(ty - node_centers[to_id])
                hyst_pen = 0.0
                to_bin = int((tx - min_x) / bin_w) if tx >= min_x and tx < max_x else -1
                if to_bin in prev_y_by_bin:
                    hyst_pen = hyst_w * abs(ty - prev_y_by_bin[to_bin])
                side_pen = self._get_side_lock_penalty(0.0, ty, obstacles)
                pa_pen = preavoid_strength * abs(ty - preavoid_bias_y) if preavoid_bias_y is not None else 0.0
                adj[0].append((to_id, dist + center_pen + hyst_pen + side_pen + pa_pen))

        if not adj[0]:
            for to_id in layer_nids[0]:
                tx, ty = nodes[to_id]
                dist = math.sqrt(tx * tx + ty * ty)
                adj[0].append((to_id, dist + 5.0))

        # Layer i → layer i+1
        for li in range(len(layers) - 1):
            # Use max curve scale of the two layers for segment check
            cs = max(layer_curve_scales[li], layer_curve_scales[li + 1])
            for from_id in layer_nids[li]:
                fx, fy = nodes[from_id]
                for to_id in layer_nids[li + 1]:
                    tx, ty = nodes[to_id]
                    if self._segment_clear(fx, fy, tx, ty, obstacles, cs):
                        dist = math.sqrt((tx - fx) ** 2 + (ty - fy) ** 2)
                        center_pen = center_w * abs(ty - node_centers[to_id])
                        hyst_pen = 0.0
                        to_bin = int((tx - min_x) / bin_w) if tx >= min_x and tx < max_x else -1
                        if to_bin in prev_y_by_bin:
                            hyst_pen = hyst_w * abs(ty - prev_y_by_bin[to_bin])
                        side_pen = self._get_side_lock_penalty(fy, ty, obstacles)
                        pa_pen = preavoid_strength * abs(ty - preavoid_bias_y) if preavoid_bias_y is not None else 0.0
                        adj[from_id].append((to_id, dist + center_pen + hyst_pen + side_pen + pa_pen))

            for from_id in layer_nids[li]:
                if not adj[from_id]:
                    fx, fy = nodes[from_id]
                    for to_id in layer_nids[li + 1]:
                        tx, ty = nodes[to_id]
                        dist = math.sqrt((tx - fx) ** 2 + (ty - fy) ** 2)
                        adj[from_id].append((to_id, dist + 5.0))

        # ── A* search ──
        open_set = [(0.0, 0.0, 0, -1)]
        g_scores = {0: 0.0}
        came_from = {}

        while open_set:
            f, g, current, parent = heapq.heappop(open_set)

            if current in came_from:
                continue
            came_from[current] = parent

            if current in goal_ids:
                path = []
                n = current
                while n != -1:
                    path.append(nodes[n])
                    n = came_from[n]
                path.reverse()

                # ARCH FIX: smooth the A* path while respecting obstacle + lane bounds
                path = self._smooth_path(path, obstacles, lane_boundaries, min_x, bin_w)

                self._prev_path = path
                self._update_side_locks(path, obstacles)
                return path

            for neighbor, edge_cost in adj[current]:
                new_g = g + edge_cost
                if new_g < g_scores.get(neighbor, float('inf')):
                    g_scores[neighbor] = new_g
                    nx, ny = nodes[neighbor]
                    h = abs(goal_x - nx)
                    heapq.heappush(open_set, (new_g + h, new_g, neighbor, current))

        return None

    def _add_gap_waypoints(self, waypoints, x_center, gap_lo, gap_hi,
                           lane_center, n_waypoints):
        """Add multiple evenly-spaced waypoints within a gap.

        For n_waypoints=3: places at 25%, 50%, 75% of gap width.
        For n_waypoints=1: places at center (original behavior).
        """
        if n_waypoints <= 1:
            wy = (gap_lo + gap_hi) / 2.0
            waypoints.append((x_center, wy, lane_center))
            return

        gap_w = gap_hi - gap_lo
        for k in range(n_waypoints):
            frac = (k + 1.0) / (n_waypoints + 1.0)
            wy = gap_lo + frac * gap_w
            waypoints.append((x_center, wy, lane_center))

    # ─── pure pursuit on path ────────────────────────────────────────
    def _pure_pursuit(self, path):
        if len(path) < 2:
            return None

        L = self._p('lookahead_min') + self._p('speed_gain') * abs(self.current_vx)
        L = max(self._p('lookahead_min'), min(L, self._p('lookahead_max')))

        # Euclidean lookahead on path
        lookahead_pt = path[-1]
        for i in range(1, len(path)):
            px, py = path[i]
            d = math.sqrt(px * px + py * py)
            if d >= L:
                lookahead_pt = (px, py)
                break

        gx, gy = lookahead_pt
        L_actual = math.sqrt(gx * gx + gy * gy)
        if L_actual < 0.01:
            return None

        pt_msg = PointStamped()
        pt_msg.header.stamp = self.get_clock().now().to_msg()
        pt_msg.header.frame_id = 'base_footprint'
        pt_msg.point.x = float(gx)
        pt_msg.point.y = float(gy)
        self.lookahead_pub.publish(pt_msg)

        curvature = 2.0 * gy / (L_actual * L_actual)
        linear = self._p('cruise_speed')
        angular = linear * curvature
        angular = max(-self._p('max_angular_vel'), min(angular, self._p('max_angular_vel')))

        curve_factor = 1.0
        if abs(curvature) > self._p('curve_slowdown_curvature'):
            curve_factor = self._p('curve_slowdown_factor')

        cmd = Twist()
        cmd.linear.x = linear * curve_factor
        cmd.angular.z = angular
        return cmd, curve_factor

    # ─── RViz visualization ──────────────────────────────────────────
    def _publish_viz(self, path, obstacles):
        from builtin_interfaces.msg import Time
        now = Time()  # stamp=0 → RViz uses latest TF (avoids extrapolation errors)

        path_msg = Path()
        path_msg.header.stamp = now
        path_msg.header.frame_id = 'base_footprint'
        for px, py in path:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(px)
            ps.pose.position.y = float(py)
            path_msg.poses.append(ps)
        self.path_pub.publish(path_msg)

        self._publish_obstacle_markers(obstacles)

    def _publish_obstacle_markers(self, obstacles):
        """Publish obstacle markers — called every cycle for stable viz."""
        from builtin_interfaces.msg import Time
        now = Time()  # stamp=0 → RViz uses latest TF
        markers = MarkerArray()

        for i, (ox, oy, est_r) in enumerate(obstacles):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = 'base_footprint'
            m.ns = 'obstacles'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(ox)
            m.pose.position.y = float(oy)
            m.pose.position.z = 0.25
            m.scale.x = max(est_r * 2.0, 0.1)
            m.scale.y = max(est_r * 2.0, 0.1)
            m.scale.z = 0.5
            m.color.r = 1.0
            m.color.a = 0.9
            markers.markers.append(m)

        # Delete excess markers from previous frame
        n_markers = len(obstacles)
        if hasattr(self, '_prev_n_obs_markers'):
            for i in range(n_markers, self._prev_n_obs_markers):
                m = Marker()
                m.header.stamp = now
                m.header.frame_id = 'base_footprint'
                m.ns = 'obstacles'
                m.id = i
                m.action = Marker.DELETE
                markers.markers.append(m)
        self._prev_n_obs_markers = n_markers

        self.obstacle_marker_pub.publish(markers)

    # ─── diagnostics ────────────────────────────────────────────────
    def _identify_segment(self, wx, wy):
        """Identify which track segment the robot is on."""
        corners = [(6.0, 4.0, 'BR'), (6.0, 20.0, 'TR'),
                   (-6.0, 20.0, 'TL'), (-6.0, 4.0, 'BL')]
        for cx, cy, name in corners:
            if math.sqrt((wx - cx)**2 + (wy - cy)**2) < 5.0:
                return name
        if wy < 4.0:
            return 'bottom'
        elif wy > 20.0:
            return 'top'
        elif wx > 6.0:
            return 'right'
        elif wx < -6.0:
            return 'left'
        return 'unknown'

    def _log_diagnostics(self, path, obstacles, replanned):
        """Write per-cycle and per-obstacle diagnostic data."""
        now_sec = self._now_sec()
        cycle = self._log_counter
        wx, wy, wyaw = self.world_x, self.world_y, self.world_yaw
        cos_y, sin_y = math.cos(wyaw), math.sin(wyaw)
        segment = self._identify_segment(wx, wy)

        # Find nearest known obstacle (world frame)
        nearest_name = ''
        nearest_dist = float('inf')
        for kox, koy, kor, kname in KNOWN_OBSTACLES:
            d = math.sqrt((wx - kox)**2 + (wy - koy)**2)
            if d < nearest_dist:
                nearest_dist = d
                nearest_name = kname

        # Cycle-level log
        cmd = self._diag_last_cmd
        self._diag.log_cycle({
            'time': f'{now_sec:.3f}',
            'cycle': cycle,
            'world_x': f'{wx:.3f}',
            'world_y': f'{wy:.3f}',
            'world_yaw_deg': f'{math.degrees(wyaw):.1f}',
            'segment': segment,
            'centerline_dist': f'{self._track_dist:.3f}',
            'heading_err_deg': f'{math.degrees(abs(self._get_heading_error())):.1f}',
            'track_idx': self._track_idx,
            'n_lane_pts': len(self.lane_points) if self.lane_points else 0,
            'n_obstacles': len(obstacles),
            'n_world_obstacles': len(self._obstacle_world),
            'in_nml': self.in_nml,
            'in_recovery': self._in_recovery,
            'on_ramp': self._on_ramp,
            'nearest_obs_name': nearest_name,
            'nearest_obs_dist': f'{nearest_dist:.3f}',
            'path_len': len(path) if path else 0,
            'replanned': replanned,
            'cmd_vx': f'{cmd["vx"]:.4f}',
            'cmd_az': f'{cmd["az"]:.4f}',
            'repulsive_az': f'{cmd["repulsive_az"]:.4f}',
            'curve_factor': f'{cmd["curve_factor"]:.3f}',
            'goal_wx': f'{self._goal_wx:.2f}',
            'goal_wy': f'{self._goal_wy:.2f}',
        })

        # Per-obstacle detail for all known obstacles within proximity threshold
        for kox, koy, kor, kname in KNOWN_OBSTACLES:
            world_dist = math.sqrt((wx - kox)**2 + (wy - koy)**2)
            if world_dist > self._diag.proximity_threshold:
                continue

            # Project known obstacle to body frame
            dx, dy = kox - wx, koy - wy
            bx = dx * cos_y + dy * sin_y
            by = -dx * sin_y + dy * cos_y

            # Find closest detected obstacle to this known obstacle's body position
            det_bx, det_by, det_r, det_match_d = 0.0, 0.0, 0.0, 999.0
            detected = False
            for ox, oy, er in obstacles:
                d = math.sqrt((ox - bx)**2 + (oy - by)**2)
                if d < det_match_d:
                    det_match_d = d
                    det_bx, det_by, det_r = ox, oy, er
                    detected = d < 2.0  # within 2m = matched

            # Clearance calculation
            clr_threshold = self._compute_obstacle_clearance(kor, obs_x=bx) if bx > 0 else 0.0
            actual_clearance = world_dist - kor - self._p('robot_half_width')

            # Path closest approach
            path_closest_d, path_closest_x, path_closest_y = 999.0, 0.0, 0.0
            if path:
                for px, py in path:
                    pd = math.sqrt((px - bx)**2 + (py - by)**2)
                    if pd < path_closest_d:
                        path_closest_d = pd
                        path_closest_x, path_closest_y = px, py

            # Determine dodge side from path
            dodge_side = ''
            if path and bx > 0:
                # Find path Y at obstacle's body X
                for i in range(len(path) - 1):
                    if path[i][0] <= bx <= path[i+1][0] and path[i+1][0] > path[i][0]:
                        t = (bx - path[i][0]) / (path[i+1][0] - path[i][0])
                        path_y = path[i][1] + t * (path[i+1][1] - path[i][1])
                        dodge_side = 'left' if path_y > by else 'right'
                        break

            # Lane boundaries at obstacle X (from lane_boundaries if available)
            lane_left, lane_right = '', ''

            self._diag.log_obstacle({
                'time': f'{now_sec:.3f}',
                'cycle': cycle,
                'obs_name': kname,
                'obs_wx': f'{kox:.2f}',
                'obs_wy': f'{koy:.2f}',
                'obs_radius': f'{kor:.2f}',
                'robot_wx': f'{wx:.3f}',
                'robot_wy': f'{wy:.3f}',
                'world_dist': f'{world_dist:.3f}',
                'body_x': f'{bx:.3f}',
                'body_y': f'{by:.3f}',
                'clearance_threshold': f'{clr_threshold:.3f}',
                'actual_clearance': f'{actual_clearance:.3f}',
                'detected': detected,
                'det_bx': f'{det_bx:.3f}',
                'det_by': f'{det_by:.3f}',
                'det_radius': f'{det_r:.3f}',
                'det_match_dist': f'{det_match_d:.3f}',
                'path_closest_dist': f'{path_closest_d:.3f}',
                'path_closest_x': f'{path_closest_x:.3f}',
                'path_closest_y': f'{path_closest_y:.3f}',
                'dodge_side': dodge_side,
                'lane_left_at_obs': lane_left,
                'lane_right_at_obs': lane_right,
            })

        # Flush every 100 cycles (~5s at 20Hz)
        if cycle % 100 == 0:
            self._diag.flush()

    # ─── helpers ─────────────────────────────────────────────────────
    def _smooth_angular(self, angular_z):
        """Apply EMA smoothing + rate limiting to angular velocity."""
        alpha = self._p('steering_alpha')
        smoothed = alpha * angular_z + (1.0 - alpha) * self._prev_angular
        # Rate limit: cap change per cycle
        max_delta = self._p('max_angular_accel') / self._p('control_rate')
        delta = smoothed - self._prev_angular
        if abs(delta) > max_delta:
            smoothed = self._prev_angular + max_delta * (1.0 if delta > 0 else -1.0)
        self._prev_angular = smoothed
        return smoothed

    def _stop_robot(self):
        self.cmd_pub.publish(Twist())

    def _predict_collision(self, vx, az, obstacles):
        """Forward-simulate the arc trajectory and check for obstacle collision.

        Returns (will_collide, closest_dist) where will_collide is True
        if the commanded velocity would physically contact an obstacle.
        Uses a tight physical threshold (est_r + robot_hw + 0.10m) rather
        than the full planning clearance, to avoid false positives that
        cause oscillation/deadlock near obstacles the A* path correctly avoids.
        """
        if abs(vx) < 0.01:
            return False, 0.0

        dt = 0.05  # 50ms steps
        horizon = 1.0  # look 1 second ahead (short to avoid false positives)
        n_steps = int(horizon / dt)
        rhw = self._p('robot_half_width')

        # Simulate arc in body frame: start at origin heading forward
        x, y, theta = 0.0, 0.0, 0.0
        for _ in range(n_steps):
            theta += az * dt
            x += vx * math.cos(theta) * dt
            y += vx * math.sin(theta) * dt

            for ox, oy, est_r in obstacles:
                dist_sq = (x - ox) ** 2 + (y - oy) ** 2
                # Physical collision threshold: obstacle radius + robot half-width + small margin
                threshold = est_r + rhw + 0.10
                if dist_sq < threshold * threshold:
                    dist = math.sqrt(dist_sq)
                    return True, dist
        return False, 0.0

    # ─── main control loop ───────────────────────────────────────────
    def _control_loop(self):
        self._log_counter += 1

        # Startup gate: don't move until we have valid world pose AND
        # stable lane data (check world-frame buffer, not body-frame which
        # requires _project_to_body to run first).
        if not self._world_pose_valid:
            if self._log_counter % 20 == 0:
                self.get_logger().info('Waiting for world pose...')
            self._stop_robot()
            return
        lane_age = (self.get_clock().now() - self.last_lane_time).nanoseconds / 1e9
        n_lane = len(self.lane_points) if self.lane_points else 0
        if n_lane < 100 and lane_age < 5.0:
            if self._log_counter % 20 == 0:
                self.get_logger().info('Waiting for lane data (n=%d, age=%.1fs)...' % (n_lane, lane_age))
            self._stop_robot()
            return

        # Startup debug: log first 5 cycles
        if self._log_counter <= 5:
            self.get_logger().info(
                'STARTUP[%d]: world=(%.1f,%.1f,yaw=%.1f°) lanes=%s obs=%d' % (
                    self._log_counter, self.world_x, self.world_y,
                    math.degrees(self.world_yaw),
                    len(self.lane_points) if self.lane_points else 0,
                    len(self.obstacle_points)))

        self._decay_side_locks()

        # Ramp mode disabled — normal lane following handles ramps fine.
        # The A* planner sees lane lines through the ramp and drives normally.

        if self.gps_valid:
            if self._log_counter % 40 == 0:
                self.get_logger().info('GPS pos (%.1f, %.1f) wyaw=%.2f real_lanes=%d nml=%s odom_obs=%d' %
                                       (self.gps_x, self.gps_y, self.world_yaw, self.real_lane_count,
                                        self.in_nml, len(self.obstacle_points)))
            if not self.in_nml:
                self._check_nml_entry()
            else:
                self._check_nml_exit()

        # Project persistent world-frame clouds to body frame
        self._project_to_body()

        # In NML, replace lane points with virtual corridor
        if self.in_nml:
            corridor_pts = self._project_nml_corridor()
            if corridor_pts:
                self.lane_points = corridor_pts
                self.last_lane_time = self.get_clock().now()

        elapsed = (self.get_clock().now() - self.last_lane_time).nanoseconds / 1e9
        if elapsed > self._p('no_lane_timeout'):
            self.get_logger().warn('No lane data for %.1fs, stopping' % elapsed)
            self._stop_robot()
            return

        self._publish_obstacle_markers(list(self.obstacle_points_viz))

        # Stuck detection: if WORLD position hasn't moved >2m in 10s, reverse briefly.
        # Uses world_pose (ground truth) since odom drifts during orbits/stuck.
        now = self._now_sec()
        dx = self.world_x - self._orbit_check_x
        dy = self.world_y - self._orbit_check_y
        moved = math.sqrt(dx * dx + dy * dy)
        if now - self._orbit_check_time > 10.0:
            if moved < 0.5 and len(self.obstacle_points) > 0:
                self._orbit_suppress_until = now + 3.0
                self.get_logger().warn(
                    'Stuck detected (moved %.1fm in 10s at world %.1f,%.1f), '
                    'reversing for 3s' % (moved, self.world_x, self.world_y))
            self._orbit_check_time = now
            self._orbit_check_x = self.world_x
            self._orbit_check_y = self.world_y

        if now < self._orbit_suppress_until:
            # Reverse to back away from physical obstacles
            cmd = Twist()
            cmd.linear.x = -0.2
            self.cmd_pub.publish(cmd)
            return

        # ── Track goal + heading recovery ──
        # Centerline is clockwise (matching travel direction).
        # Recovery activates when heading deviates > threshold from track direction.
        # DISABLED in NML — NML has its own waypoint pursuit that may go off-track.
        self._update_track_goal()

        # Heading recovery DISABLED — it uses hardcoded track centerline which
        # doesn't match unknown competition tracks. The A* planner + pure pursuit
        # handles heading corrections using only sensor-detected lane boundaries.

        obstacles = list(self.obstacle_points)
        obstacles_viz = list(self.obstacle_points_viz)

        # Add reconstructed lane walls as impassable obstacles
        lane_walls = getattr(self, '_lane_wall_obstacles', [])
        obstacles.extend(lane_walls)

        # ── NML: waypoint pursuit with obstacle avoidance ──
        # Uses a local potential field: goal attraction + obstacle repulsion
        # to build a short path, then pure pursuit follows it.
        if self.in_nml and self.nml_target is not None:
            cos_y = math.cos(self.world_yaw)
            sin_y = math.sin(self.world_yaw)
            tx, ty = self.nml_target
            dx, dy = tx - self.world_x, ty - self.world_y
            goal_bx = dx * cos_y + dy * sin_y
            goal_by = -dx * sin_y + dy * cos_y
            goal_dist = math.sqrt(goal_bx * goal_bx + goal_by * goal_by)

            # Build path via potential field: step toward goal while avoiding obstacles
            # The path always goes FORWARD — if the goal is behind, the path curves
            # ahead and around to reach it. The robot never reverses or spins.
            path = [(0.0, 0.0)]
            px, py = 0.0, 0.0
            step_size = 0.3
            max_steps = int(min(goal_dist, self._p('max_x')) / step_size) + 1
            rhw = self._p('robot_half_width')
            sm = self._p('safety_margin')

            for step in range(max_steps):
                # Repulsion from obstacles (computed first, dominates when close)
                rx, ry = 0.0, 0.0
                repulsion_strength = 0.0
                for ox, oy, er in obstacles:
                    odx = px - ox
                    ody = py - oy
                    od = math.sqrt(odx * odx + ody * ody)
                    clr = er + rhw + sm
                    influence = clr + 1.0
                    if od < influence and od > 0.01:
                        strength = ((influence - od) / influence) ** 2
                        repulsion_strength = max(repulsion_strength, strength)
                        rx += odx / od * strength * step_size * 1.25
                        ry += ody / od * strength * step_size * 1.25
                        # Hard push out of clearance zone
                        if od < clr:
                            push = clr - od + 0.1
                            rx += odx / od * push
                            ry += ody / od * push

                # Attraction toward goal (always moves forward, curves toward goal)
                gdx = goal_bx - px
                gdy = goal_by - py
                gd = math.sqrt(gdx * gdx + gdy * gdy)
                if gd < 0.5:
                    path.append((goal_bx, goal_by))
                    break
                attract_scale = max(0.1, 1.0 - repulsion_strength * 2.0)
                # Direction toward goal
                ax_raw = gdx / max(gd, 0.01)
                ay_raw = gdy / max(gd, 0.01)
                # Enforce forward motion but ease off when goal is close
                min_fwd = 0.3 if gd > 2.0 else 0.1
                ax = max(ax_raw, min_fwd) * step_size * attract_scale
                ay = ay_raw * step_size * attract_scale

                px += ax + rx
                py += ay + ry
                path.append((px, py))

            if len(path) < 2:
                path = [(0.0, 0.0), (goal_bx, goal_by)]

            self._publish_viz(path, obstacles_viz)
            result = self._pure_pursuit_nml(path)
            if result is not None:
                cmd, curve_factor = result
                cmd.angular.z = self._smooth_angular(cmd.angular.z)
                self._diag_last_cmd = {
                    'vx': cmd.linear.x, 'az': cmd.angular.z,
                    'repulsive_az': 0.0, 'curve_factor': curve_factor,
                }
                self.cmd_pub.publish(cmd)
                if self._log_counter % 20 == 0:
                    self.get_logger().info(
                        'NML nav: target=(%.1f,%.1f) body=(%.1f,%.1f) dist=%.1f obs=%d path=%d' %
                        (tx, ty, goal_bx, goal_by, goal_dist,
                         len(obstacles), len(path)))
                return
            self._stop_robot()
            return

        # ── Normal lane following with A* ──
        # Replan throttle: only replan every N cycles to prevent A* oscillation.
        # Between replans, reuse the previous path. Force replan if no valid path.
        self._replan_counter = getattr(self, '_replan_counter', 0) + 1
        replan_interval = 5  # replan every ~0.25s at 20Hz
        replanned = False
        if (self._replan_counter >= replan_interval or
                self._prev_path is None or len(self._prev_path) < 3):
            path = self._plan_path_legacy()
            self._replan_counter = 0
            replanned = True
        else:
            path = self._prev_path

        if path and len(path) >= 2:
            self._publish_viz(path, obstacles_viz)
            result = self._pure_pursuit(path)
            if result is not None:
                cmd, curve_factor = result
                # Obstacle repulsion: gentle nudge away from nearby obstacles.
                repulsive_az = 0.0
                if obstacles:
                    min_dist = 999.0
                    for ox, oy, _ in obstacles:
                        dist = math.sqrt(ox * ox + oy * oy)
                        if dist < min_dist:
                            min_dist = dist
                        if dist < 4.0 and ox > 0.3 and abs(oy) < 1.0:
                            repulsive_az += (-oy) / max(dist, 1.0)

                    repulsive_az = max(-0.08, min(0.08, repulsive_az))
                    cmd.angular.z += repulsive_az

                cmd.angular.z = self._smooth_angular(cmd.angular.z)

                # Record cmd for diagnostics before publishing
                self._diag_last_cmd = {
                    'vx': cmd.linear.x, 'az': cmd.angular.z,
                    'repulsive_az': repulsive_az, 'curve_factor': curve_factor,
                }

                self.cmd_pub.publish(cmd)

                # Diagnostic logging (every 5th cycle = 4Hz)
                if self._log_counter % 5 == 0:
                    self._log_diagnostics(path, obstacles, replanned)

                if self._log_counter % 40 == 0:
                    path_str = ' → '.join('(%.1f,%.1f)' % (px, py) for px, py in path[:5])
                    nml_str = ' [NML→(%.1f,%.1f)]' % self.nml_target if self.in_nml else ''
                    self.get_logger().info(
                        'Path: %d pts, %d obs, vx=%.2f az=%.2f%s | %s' % (
                            len(path), len(obstacles),
                            cmd.linear.x, cmd.angular.z, nml_str, path_str))
                return

        # No valid path — log and stop
        self._diag_last_cmd = {'vx': 0.0, 'az': 0.0, 'repulsive_az': 0.0, 'curve_factor': 0.0}
        if self._log_counter % 5 == 0:
            self._log_diagnostics(path, obstacles, replanned)
        self._stop_robot()


    def destroy_node(self):
        self.get_logger().info('Closing diagnostic logger...')
        self._diag.flush()
        self._diag.close()
        self.get_logger().info(f'Diagnostic logs saved to {self._diag.log_dir}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
