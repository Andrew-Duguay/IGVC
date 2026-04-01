#!/usr/bin/env python3
"""Fake lane publisher for IGVC AutoNav course.

Reads ground truth pose from Gazebo via `gz topic` and publishes:
  /lane_points     (PointCloud2 in base_footprint)
  /world_pose      (PoseStamped in world frame)
  /obstacle_points (PointCloud2 in base_footprint)

Replaces real vision pipeline for testing autonomy logic independently.
"""

import math
import re
import subprocess
import threading
import time
import struct
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


# ── Track geometry ─────────────────────────────────────────────────────

# Constants matching generate_autonav_sdf.py
CORNER_RADIUS = 4.0
HALF_LANE = 1.5
BOTTOM_Y = 0.0
RIGHT_X = 10.0
TOP_Y = 24.0
NML_Y_START = 4.0   # No man's land start (right straight)
NML_Y_END = 9.75    # No man's land end (at ramp entrance)
LEFT_X = -10.0
CHICANE_X_START = 3.0
CHICANE_X_END = -3.0
CHICANE_AMPLITUDE = 1.1
RAMP_CENTER_Y = 12.0
RAMP_HALF_LEN = 2.25
RAMP_FUNNEL_LEN = 2.5


def chicane_offset(x):
    """S-curve lateral offset on top straight."""
    if x > CHICANE_X_START or x < CHICANE_X_END:
        return 0.0
    t = (x - CHICANE_X_START) / (CHICANE_X_END - CHICANE_X_START)
    return CHICANE_AMPLITUDE * math.sin(t * 2 * math.pi)


def ramp_funnel_half_width(y):
    """Lane half-width on right straight near the ramp.
    Curved (cosine) transitions for smooth entrance/exit."""
    ramp_y_start = RAMP_CENTER_Y - RAMP_HALF_LEN
    ramp_y_end = RAMP_CENTER_Y + RAMP_HALF_LEN
    funnel_start = ramp_y_start - RAMP_FUNNEL_LEN
    funnel_end = ramp_y_end + RAMP_FUNNEL_LEN

    if y < funnel_start or y > funnel_end:
        return HALF_LANE
    elif y < ramp_y_start:
        t = (y - funnel_start) / RAMP_FUNNEL_LEN
        return 0.75 + (HALF_LANE - 0.75) * (1.0 + math.cos(t * math.pi)) / 2.0
    elif y <= ramp_y_end:
        return 0.75
    else:
        t = (y - ramp_y_end) / RAMP_FUNNEL_LEN
        return 0.75 + (HALF_LANE - 0.75) * (1.0 - math.cos(t * math.pi)) / 2.0


def generate_track_lines():
    """Generate inner and outer lane line points for the autonav course.

    Returns (inner_pts, outer_pts) — lists of (x, y) in world frame.
    Counterclockwise travel: inner = left side, outer = right side.
    """
    inner = []
    outer = []
    step = 0.1

    def add_pair(cx, cy, nx, ny, inner_off, outer_off):
        """Add inner/outer points offset from centerline along normal."""
        inner.append((cx + nx * inner_off, cy + ny * inner_off))
        outer.append((cx - nx * outer_off, cy - ny * outer_off))

    # 1. Bottom straight: x from -6 to 6, y=0, travel direction = +x
    # Normal points in +y direction (left of travel)
    x = -6.0
    while x <= 6.0:
        inner.append((x, BOTTOM_Y + HALF_LANE))
        outer.append((x, BOTTOM_Y - HALF_LANE))
        x += step

    # 2. Bottom-right corner: center (6, 4), from -pi/2 to 0
    cx, cy = 6.0, 4.0
    n_arc = 40
    for i in range(n_arc + 1):
        theta = -math.pi / 2 + (math.pi / 2) * i / n_arc
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        # Inner (smaller radius, toward center)
        inner.append((cx + (CORNER_RADIUS - HALF_LANE) * cos_t,
                       cy + (CORNER_RADIUS - HALF_LANE) * sin_t))
        # Outer (larger radius, away from center)
        outer.append((cx + (CORNER_RADIUS + HALF_LANE) * cos_t,
                       cy + (CORNER_RADIUS + HALF_LANE) * sin_t))

    # 3. Right straight: y from 4 to 20, x=10, travel direction = +y
    # Normal points in -x direction (left of travel when going +y)
    # Skip no man's land zone (NML_Y_START to NML_Y_END) — no lane lines there
    y = 4.0
    while y <= 20.0:
        if NML_Y_START <= y <= NML_Y_END:
            y += step
            continue
        hw = ramp_funnel_half_width(y)
        inner.append((RIGHT_X - hw, y))
        outer.append((RIGHT_X + hw, y))
        y += step

    # 4. Top-right corner: center (6, 20), from 0 to pi/2
    cx, cy = 6.0, 20.0
    for i in range(n_arc + 1):
        theta = 0 + (math.pi / 2) * i / n_arc
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        inner.append((cx + (CORNER_RADIUS - HALF_LANE) * cos_t,
                       cy + (CORNER_RADIUS - HALF_LANE) * sin_t))
        outer.append((cx + (CORNER_RADIUS + HALF_LANE) * cos_t,
                       cy + (CORNER_RADIUS + HALF_LANE) * sin_t))

    # 5. Top straight: x from 6 to -6, y=24, travel direction = -x
    # Normal points in -y direction (left of travel when going -x)
    # Chicane shifts centerline in normal direction (-y), so subtract offset from y
    x = 6.0
    while x >= -6.0:
        ch = chicane_offset(x)
        inner.append((x, TOP_Y - HALF_LANE - ch))
        outer.append((x, TOP_Y + HALF_LANE - ch))
        x -= step

    # 6. Top-left corner: center (-6, 20), from pi/2 to pi
    cx, cy = -6.0, 20.0
    for i in range(n_arc + 1):
        theta = math.pi / 2 + (math.pi / 2) * i / n_arc
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        inner.append((cx + (CORNER_RADIUS - HALF_LANE) * cos_t,
                       cy + (CORNER_RADIUS - HALF_LANE) * sin_t))
        outer.append((cx + (CORNER_RADIUS + HALF_LANE) * cos_t,
                       cy + (CORNER_RADIUS + HALF_LANE) * sin_t))

    # 7. Left straight: y from 20 to 4, x=-10, travel direction = -y
    # Normal points in +x direction (left of travel when going -y)
    y = 20.0
    while y >= 4.0:
        inner.append((LEFT_X + HALF_LANE, y))
        outer.append((LEFT_X - HALF_LANE, y))
        y -= step

    # 8. Bottom-left corner: center (-6, 4), from pi to 3pi/2
    cx, cy = -6.0, 4.0
    for i in range(n_arc + 1):
        theta = math.pi + (math.pi / 2) * i / n_arc
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        inner.append((cx + (CORNER_RADIUS - HALF_LANE) * cos_t,
                       cy + (CORNER_RADIUS - HALF_LANE) * sin_t))
        outer.append((cx + (CORNER_RADIUS + HALF_LANE) * cos_t,
                       cy + (CORNER_RADIUS + HALF_LANE) * sin_t))

    return inner, outer


# ── Ground truth pose reader ───────────────────────────────────────────

class GzPoseReader:
    """Read ground truth pose from Gazebo Harmonic via `gz topic`."""

    def __init__(self, world_name='igvc_autonav', model_name='littleblue'):
        self.model_name = model_name
        self.topic = f'/world/{world_name}/dynamic_pose/info'
        self.lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while True:
            try:
                proc = subprocess.Popen(
                    ['gz', 'topic', '-e', '-t', self.topic],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    text=True
                )
                self._parse_stream(proc.stdout)
            except Exception as e:
                print(f'[GzPoseReader] Error: {e}, retrying in 2s...')
                time.sleep(2)

    def _parse_stream(self, stream):
        in_target = False
        in_position = False
        in_orientation = False
        brace_depth = 0
        px, py, pz = 0.0, 0.0, 0.0
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

        for line in stream:
            line = line.strip()

            if not in_target:
                if f'name: "{self.model_name}"' in line:
                    in_target = True
                    brace_depth = 1
                continue

            brace_depth += line.count('{') - line.count('}')

            if 'position {' in line or 'position{' in line:
                in_position = True
                in_orientation = False
            elif 'orientation {' in line or 'orientation{' in line:
                in_orientation = True
                in_position = False

            m = re.search(r'x:\s*([-\d.eE+]+)', line)
            if m:
                val = float(m.group(1))
                if in_position:
                    px = val
                elif in_orientation:
                    qx = val

            m = re.search(r'y:\s*([-\d.eE+]+)', line)
            if m:
                val = float(m.group(1))
                if in_position:
                    py = val
                elif in_orientation:
                    qy = val

            m = re.search(r'z:\s*([-\d.eE+]+)', line)
            if m:
                val = float(m.group(1))
                if in_position:
                    pz = val
                elif in_orientation:
                    qz = val

            m = re.search(r'w:\s*([-\d.eE+]+)', line)
            if m:
                val = float(m.group(1))
                if in_orientation:
                    qw = val

            if brace_depth <= 0:
                yaw = math.atan2(
                    2.0 * (qw * qz + qx * qy),
                    1.0 - 2.0 * (qy * qy + qz * qz)
                )
                with self.lock:
                    self.x = px
                    self.y = py
                    self.z = pz
                    self.yaw = yaw
                in_target = False
                in_position = False
                in_orientation = False

    def get_pose(self):
        with self.lock:
            return self.x, self.y, self.z, self.yaw


# ── ROS2 Node ──────────────────────────────────────────────────────────

class FakeLanePublisher(Node):
    def __init__(self):
        super().__init__('fake_lane_publisher')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.get_logger().info('Starting fake lane publisher (autonav course)...')

        self.inner_pts, self.outer_pts = generate_track_lines()
        self.get_logger().info(
            f'Track: {len(self.inner_pts)} inner + {len(self.outer_pts)} outer points')

        self.pose_reader = GzPoseReader()

        # Obstacle positions matching autonav SDF
        # (x, y, radius) — radii from actual SDF model geometry
        self.obstacles = [
            (10, 7.98, 0.15),    # cone_1
            (-6, 22, 0.15),      # cone_2
            (3.11, -1, 0.15),    # cone_3
            (0.91, 1.17, 0.15),  # cone_3_1
            (-10, 15, 0.25),     # barrel_1
            (7, 24, 0.25),       # barrel_2
            (-8, 1, 0.25),       # barrel_minspeed
            (0, 24.5, 0.30),     # tire_1
            (10, 17, 0.30),      # tire_2
            (-3, 0.5, 0.20),     # trash_can_1
            (5, -0.5, 0.50),     # barricade_1
            (10.5, 6, 0.25),     # barrel_nml_1
            (2.94, -0.40, 0.25), # barrel_nml_1_1
            (2.73, 0.28, 0.25),  # barrel_nml_1_2
            (9.5, 4.5, 0.15),   # cone_nml_1
            (1.5, 7.0, 0.15),   # cone_wp3_1
            (-1.0, 5.5, 0.15),  # cone_wp3_2
            (0.5, 5.0, 0.25),   # barrel_wp3_1
            (1.10, 0.52, 0.25), # barrel_wp3_1_1
            (0.14, -1.06, 0.25), # barrel_nml_1_2_1
            (-0.24, -0.37, 0.20), # trash_can_1_1
        ]

        self.lane_pub = self.create_publisher(PointCloud2, '/lane_points', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/world_pose', 10)
        # Benchmark ground-truth topics (for scoring candidate detectors)
        # Obstacles moved to /benchmark/obstacle_points only (real detection via LiDAR now)
        self.bench_lane_pub = self.create_publisher(PointCloud2, '/benchmark/lane_points', 10)
        self.bench_obs_pub = self.create_publisher(PointCloud2, '/benchmark/obstacle_points', 10)

        self.create_timer(0.1, self._publish)
        self.get_logger().info('Fake lane publisher ready (10 Hz)')

    def _publish(self):
        rx, ry, rz, ryaw = self.pose_reader.get_pose()

        # Publish world pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = rx
        pose_msg.pose.position.y = ry
        pose_msg.pose.position.z = rz
        pose_msg.pose.orientation.z = math.sin(ryaw / 2.0)
        pose_msg.pose.orientation.w = math.cos(ryaw / 2.0)
        self.pose_pub.publish(pose_msg)

        cos_yaw = math.cos(ryaw)
        sin_yaw = math.sin(ryaw)

        # Transform lane points to body frame
        visible = []
        for pts_list in (self.inner_pts, self.outer_pts):
            for wx, wy in pts_list:
                dx = wx - rx
                dy = wy - ry
                bx = dx * cos_yaw + dy * sin_yaw
                by = -dx * sin_yaw + dy * cos_yaw

                # Extended visibility for larger course: forward 0.3-8.0m, lateral ±8m
                if 0.3 <= bx <= 8.0 and abs(by) <= 8.0:
                    bx += random.gauss(0, 0.02)
                    by += random.gauss(0, 0.02)
                    visible.append((bx, by, 0.0))

        if not visible:
            return

        now_stamp = self.get_clock().now().to_msg()

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        point_step = 12
        data = bytearray()
        for x, y, z in visible:
            data.extend(struct.pack('fff', x, y, z))

        # Normal topic: stamp=0 for RViz compatibility (uses latest TF)
        header = Header()
        header.frame_id = 'base_footprint'
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(visible)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = point_step
        cloud.row_step = point_step * len(visible)
        cloud.data = bytes(data)
        cloud.is_dense = True
        self.lane_pub.publish(cloud)

        # Benchmark topic: real timestamp for sync with candidate
        bench_header = Header()
        bench_header.stamp = now_stamp
        bench_header.frame_id = 'base_footprint'
        bench_cloud = PointCloud2()
        bench_cloud.header = bench_header
        bench_cloud.height = 1
        bench_cloud.width = len(visible)
        bench_cloud.fields = fields
        bench_cloud.is_bigendian = False
        bench_cloud.point_step = point_step
        bench_cloud.row_step = point_step * len(visible)
        bench_cloud.data = bytes(data)
        bench_cloud.is_dense = True
        self.bench_lane_pub.publish(bench_cloud)

        # Publish obstacle surface points (with line-of-sight occlusion)
        # First pass: compute body-frame positions for all obstacles
        obs_body = []
        for wx, wy, radius in self.obstacles:
            dx = wx - rx
            dy = wy - ry
            bx = dx * cos_yaw + dy * sin_yaw
            by = -dx * sin_yaw + dy * cos_yaw
            obs_body.append((bx, by, radius))

        obs_visible = []
        n_surface = 8
        for i, (bx, by, radius) in enumerate(obs_body):
            if not (0.5 <= bx <= 10.0 and abs(by) <= 6.0):
                continue
            # Check if any closer obstacle occludes this one
            dist = math.sqrt(bx * bx + by * by)
            occluded = False
            for j, (obx, oby, orad) in enumerate(obs_body):
                if i == j:
                    continue
                odist = math.sqrt(obx * obx + oby * oby)
                if odist >= dist:
                    continue  # farther away, can't occlude
                # Check if obstacle j blocks the ray from robot to obstacle i
                # Project obstacle j onto the ray toward obstacle i
                ray_len = dist
                ray_dx, ray_dy = bx / ray_len, by / ray_len
                # Perpendicular distance from obstacle j center to the ray
                cross = obx * ray_dy - oby * ray_dx
                if abs(cross) < orad + 0.1:  # ray passes within obstacle radius
                    # And obstacle j is between robot and obstacle i
                    dot = obx * ray_dx + oby * ray_dy
                    if 0.3 < dot < dist:
                        occluded = True
                        break
            if occluded:
                continue
            for k in range(n_surface):
                angle = 2.0 * math.pi * k / n_surface
                sx = bx + radius * math.cos(angle)
                sy = by + radius * math.sin(angle)
                obs_visible.append((sx, sy, 0.0))

        # Always publish obstacle cloud (even empty) to clear stale data
        obs_data = bytearray()
        for x, y, z in obs_visible:
            obs_data.extend(struct.pack('fff', x, y, z))

        # Benchmark topic only (real obstacle detection via LiDAR now)
        bench_obs_header = Header()
        bench_obs_header.stamp = now_stamp
        bench_obs_header.frame_id = 'base_footprint'
        bench_obs_cloud = PointCloud2()
        bench_obs_cloud.header = bench_obs_header
        bench_obs_cloud.height = 1
        bench_obs_cloud.width = len(obs_visible)
        bench_obs_cloud.fields = fields
        bench_obs_cloud.is_bigendian = False
        bench_obs_cloud.point_step = point_step
        bench_obs_cloud.row_step = point_step * max(1, len(obs_visible))
        bench_obs_cloud.data = bytes(obs_data)
        bench_obs_cloud.is_dense = True
        self.bench_obs_pub.publish(bench_obs_cloud)


def main():
    rclpy.init()
    node = FakeLanePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
