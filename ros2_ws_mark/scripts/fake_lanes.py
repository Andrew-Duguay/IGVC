#!/usr/bin/env python3
"""Fake lane publisher — synthetic lane points from known track geometry.

Reads ground truth pose from Gazebo via `gz topic` and publishes:
  /lane_points  (PointCloud2 in base_footprint)
  /world_pose   (PoseStamped in world frame)

Replaces real vision pipeline for testing autonomy logic independently.
"""

import math
import re
import subprocess
import threading
import time
import struct

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import random


# ── Track geometry ─────────────────────────────────────────────────────

def generate_track_lines():
    """Generate inner and outer lane line points for the full oval track.

    Returns (inner_pts, outer_pts) — lists of (x, y) in world frame.
    """
    inner = []
    outer = []
    step = 0.1  # meters between points

    # Bottom straight: x from -6 to 6 (with ramp funnel sections)
    # 5 sections:
    #   Normal:          x = -6.0  to -1.25  inner y=+1.5, outer y=-1.5
    #   Approach funnel: x = -1.25 to -0.25  inner 1.5→0.75, outer -1.5→-0.75
    #   Ramp:            x = -0.25 to  4.25  inner y=+0.75, outer y=-0.75
    #   Exit funnel:     x =  4.25 to  5.25  inner 0.75→1.5, outer -0.75→-1.5
    #   Normal:          x =  5.25 to  6.0   inner y=+1.5, outer y=-1.5

    x = -6.0
    while x <= 6.0:
        if x <= -1.25:
            iy, oy = 1.5, -1.5
        elif x <= -0.25:
            t = (x - (-1.25)) / (-0.25 - (-1.25))
            iy = 1.5 + t * (0.75 - 1.5)
            oy = -1.5 + t * (-0.75 - (-1.5))
        elif x <= 4.25:
            iy, oy = 0.75, -0.75
        elif x <= 5.25:
            t = (x - 4.25) / (5.25 - 4.25)
            iy = 0.75 + t * (1.5 - 0.75)
            oy = -0.75 + t * (-1.5 - (-0.75))
        else:
            iy, oy = 1.5, -1.5

        inner.append((x, iy))
        outer.append((x, oy))
        x += step

    # Right semicircle: center (6, 5), inner r=3.5, outer r=6.5
    # θ from -π/2 (bottom) to π/2 (top)
    n_arc = 60
    for i in range(n_arc + 1):
        theta = -math.pi / 2 + math.pi * i / n_arc
        inner.append((6.0 + 3.5 * math.cos(theta), 5.0 + 3.5 * math.sin(theta)))
        outer.append((6.0 + 6.5 * math.cos(theta), 5.0 + 6.5 * math.sin(theta)))

    # Top straight: x from 6 to -6, inner y=8.5, outer y=11.5
    x = 6.0
    while x >= -6.0:
        inner.append((x, 8.5))
        outer.append((x, 11.5))
        x -= step

    # Left semicircle: center (-6, 5), inner r=3.5, outer r=6.5
    # θ from π/2 (top) through π to 3π/2 == -π/2 (bottom)
    for i in range(n_arc + 1):
        theta = math.pi / 2 + math.pi * i / n_arc
        inner.append((-6.0 + 3.5 * math.cos(theta), 5.0 + 3.5 * math.sin(theta)))
        outer.append((-6.0 + 6.5 * math.cos(theta), 5.0 + 6.5 * math.sin(theta)))

    return inner, outer


# ── Ground truth pose reader ───────────────────────────────────────────

class GzPoseReader:
    """Read ground truth pose from Gazebo Harmonic via `gz topic`."""

    def __init__(self, world_name='igvc_course', model_name='littleblue'):
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
        """Continuously read gz topic output and parse pose."""
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
        """Parse protobuf text format from gz topic -e.

        Looks for blocks like:
          pose {
            name: "littleblue"
            ...
            position { x: ... y: ... z: ... }
            orientation { x: ... y: ... z: ... w: ... }
          }
        """
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
                    brace_depth = 1  # we're inside the pose block
                continue

            # Count braces to track nesting
            brace_depth += line.count('{') - line.count('}')

            if 'position {' in line or 'position{' in line:
                in_position = True
                in_orientation = False
            elif 'orientation {' in line or 'orientation{' in line:
                in_orientation = True
                in_position = False

            # Parse values
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

            # End of this pose block
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
        # Use sim time so timestamps match the lane_follower_node
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.get_logger().info('Starting fake lane publisher...')

        # Generate track geometry
        self.inner_pts, self.outer_pts = generate_track_lines()
        self.get_logger().info(
            f'Track: {len(self.inner_pts)} inner + {len(self.outer_pts)} outer points')

        # Start gz pose reader
        self.pose_reader = GzPoseReader()

        # Obstacle positions in world frame (from SDF) with physical radii
        # (x, y, radius) — cones ~0.15m, barrels ~0.30m
        self.obstacles = [
            (5.5, 0.5, 0.15),     # cone_1
            (-5.5, -0.3, 0.30),   # barrel_1
            (-3.0, 10.5, 0.15),   # cone_2
            (2.0, 9.5, 0.15),     # cone_3
            (5.0, 10.0, 0.30),    # barrel_2
            (8.5, 3.0, 0.15),     # cone_4
            (-8.5, 7.0, 0.30),    # barrel_3
        ]

        # Publishers
        self.lane_pub = self.create_publisher(PointCloud2, '/lane_points', 10)
        self.obstacle_pub = self.create_publisher(PointCloud2, '/obstacle_points', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/world_pose', 10)

        # 10 Hz publish loop
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
        # quaternion from yaw
        pose_msg.pose.orientation.z = math.sin(ryaw / 2.0)
        pose_msg.pose.orientation.w = math.cos(ryaw / 2.0)
        self.pose_pub.publish(pose_msg)

        # Transform track points to body frame, filter visible ones
        cos_yaw = math.cos(ryaw)
        sin_yaw = math.sin(ryaw)

        visible = []
        for pts_list in (self.inner_pts, self.outer_pts):
            for wx, wy in pts_list:
                # World to body frame
                dx = wx - rx
                dy = wy - ry
                bx = dx * cos_yaw + dy * sin_yaw
                by = -dx * sin_yaw + dy * cos_yaw

                # Camera-like visibility: forward 0.3-6.0m, lateral ±6m
                if 0.3 <= bx <= 6.0 and abs(by) <= 6.0:
                    # Add small noise
                    bx += random.gauss(0, 0.02)
                    by += random.gauss(0, 0.02)
                    visible.append((bx, by, 0.0))

        if not visible:
            return

        # Build PointCloud2 message
        # stamp=0 tells RViz to use the latest available TF (avoids timing mismatch)
        header = Header()
        header.stamp.sec = 0
        header.stamp.nanosec = 0
        header.frame_id = 'base_footprint'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        point_step = 12  # 3 floats * 4 bytes
        data = bytearray()
        for x, y, z in visible:
            data.extend(struct.pack('fff', x, y, z))

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

        # Publish obstacle SURFACE points in body frame
        # Instead of one center point, publish points around the obstacle
        # perimeter so the autonomy node sees actual obstacle extent.
        obs_visible = []
        n_surface = 8  # points around each obstacle
        for wx, wy, radius in self.obstacles:
            dx = wx - rx
            dy = wy - ry
            bx = dx * cos_yaw + dy * sin_yaw
            by = -dx * sin_yaw + dy * cos_yaw
            # Only publish obstacles ahead and within range
            if 0.5 <= bx <= 8.0 and abs(by) <= 4.0:
                for k in range(n_surface):
                    angle = 2.0 * math.pi * k / n_surface
                    sx = bx + radius * math.cos(angle)
                    sy = by + radius * math.sin(angle)
                    obs_visible.append((sx, sy, 0.0))

        if obs_visible:
            obs_header = Header()
            obs_header.stamp.sec = 0
            obs_header.stamp.nanosec = 0
            obs_header.frame_id = 'base_footprint'

            obs_data = bytearray()
            for x, y, z in obs_visible:
                obs_data.extend(struct.pack('fff', x, y, z))

            obs_cloud = PointCloud2()
            obs_cloud.header = obs_header
            obs_cloud.height = 1
            obs_cloud.width = len(obs_visible)
            obs_cloud.fields = fields
            obs_cloud.is_bigendian = False
            obs_cloud.point_step = point_step
            obs_cloud.row_step = point_step * len(obs_visible)
            obs_cloud.data = bytes(obs_data)
            obs_cloud.is_dense = True
            self.obstacle_pub.publish(obs_cloud)


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
