#!/usr/bin/env python3
"""Data recorder node for IGVC obstacle navigation A/B testing.

Subscribes to robot state topics and logs per-cycle metrics to CSV at 10Hz.
Uses hardcoded SDF obstacle positions for ground-truth distance computation.
"""

import math
import os
import time
import csv

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import MarkerArray


# Hardcoded SDF obstacle positions and approximate radii
# From igvc_course.sdf (verified in MEMORY.md)
SDF_OBSTACLES = [
    # (x, y, radius, name)
    (5.5, 0.5, 0.15, 'cone_1'),
    (-5.5, -0.3, 0.30, 'barrel_1'),
    (-3.0, 10.5, 0.15, 'cone_2'),
    (2.0, 9.5, 0.15, 'cone_3'),
    (5.0, 10.0, 0.30, 'barrel_2'),
    (8.5, 3.0, 0.15, 'cone_4'),
    (-8.5, 7.0, 0.30, 'barrel_3'),
]

ROBOT_HALF_WIDTH = 0.325


class DataRecorderNode(Node):
    def __init__(self):
        super().__init__('data_recorder_node')

        self.declare_parameter('approach_label', 'baseline')
        self.declare_parameter('record_rate', 10.0)
        self.declare_parameter('output_dir', '/tmp/igvc_data')

        self.approach_label = self.get_parameter('approach_label').value
        output_dir = self.get_parameter('output_dir').value
        os.makedirs(output_dir, exist_ok=True)

        timestamp_str = time.strftime('%Y%m%d_%H%M%S')
        filename = f'{self.approach_label}_{timestamp_str}.csv'
        self.csv_path = os.path.join(output_dir, filename)

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'world_x', 'world_y', 'world_yaw',
            'cmd_vx', 'cmd_az', 'path_length', 'path_y_at_2m',
            'num_obstacles', 'min_obstacle_dist', 'min_obstacle_clearance',
            'is_stopped', 'approach_label',
        ])

        # State from subscriptions
        self.world_x = 0.0
        self.world_y = 0.0
        self.world_yaw = 0.0
        self.world_pose_valid = False
        self.cmd_vx = 0.0
        self.cmd_az = 0.0
        self.path_length = 0.0
        self.path_y_at_2m = 0.0
        self.num_obstacles = 0

        # Subscribers
        self.create_subscription(PoseStamped, '/world_pose', self._world_pose_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(Path, '/autonomy/path', self._path_cb, 10)
        self.create_subscription(MarkerArray, '/autonomy/obstacles', self._obstacles_cb, 10)

        # Record timer
        rate = self.get_parameter('record_rate').value
        self.create_timer(1.0 / rate, self._record)

        self.get_logger().info(
            f'Data recorder started: approach={self.approach_label}, file={self.csv_path}')

    def _world_pose_cb(self, msg):
        self.world_x = msg.pose.position.x
        self.world_y = msg.pose.position.y
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.world_yaw = math.atan2(siny, cosy)
        self.world_pose_valid = True

    def _cmd_vel_cb(self, msg):
        self.cmd_vx = msg.linear.x
        self.cmd_az = msg.angular.z

    def _path_cb(self, msg):
        if len(msg.poses) < 2:
            self.path_length = 0.0
            self.path_y_at_2m = 0.0
            return
        total = 0.0
        for i in range(1, len(msg.poses)):
            dx = msg.poses[i].pose.position.x - msg.poses[i - 1].pose.position.x
            dy = msg.poses[i].pose.position.y - msg.poses[i - 1].pose.position.y
            total += math.sqrt(dx * dx + dy * dy)
        self.path_length = total

        # Interpolate path y at x=2m
        self.path_y_at_2m = 0.0
        for i in range(1, len(msg.poses)):
            x0 = msg.poses[i - 1].pose.position.x
            x1 = msg.poses[i].pose.position.x
            if x0 <= 2.0 <= x1 and x1 > x0:
                t = (2.0 - x0) / (x1 - x0)
                y0 = msg.poses[i - 1].pose.position.y
                y1 = msg.poses[i].pose.position.y
                self.path_y_at_2m = y0 + t * (y1 - y0)
                break

    def _obstacles_cb(self, msg):
        # Count non-DELETEALL markers
        count = 0
        for m in msg.markers:
            if m.action != m.DELETEALL:
                count += 1
        self.num_obstacles = count

    def _record(self):
        if not self.world_pose_valid:
            return

        # Compute ground-truth distance to closest SDF obstacle
        min_dist = float('inf')
        min_clearance = float('inf')
        for ox, oy, orad, _ in SDF_OBSTACLES:
            d = math.sqrt((self.world_x - ox) ** 2 + (self.world_y - oy) ** 2)
            clearance = d - orad - ROBOT_HALF_WIDTH
            if d < min_dist:
                min_dist = d
            if clearance < min_clearance:
                min_clearance = clearance

        is_stopped = 1 if abs(self.cmd_vx) < 0.01 else 0

        now = self.get_clock().now()
        t = now.nanoseconds / 1e9

        self.csv_writer.writerow([
            f'{t:.3f}',
            f'{self.world_x:.4f}', f'{self.world_y:.4f}', f'{self.world_yaw:.4f}',
            f'{self.cmd_vx:.4f}', f'{self.cmd_az:.4f}',
            f'{self.path_length:.3f}', f'{self.path_y_at_2m:.4f}',
            self.num_obstacles,
            f'{min_dist:.4f}', f'{min_clearance:.4f}',
            is_stopped, self.approach_label,
        ])
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info(f'Data recorder saved to {self.csv_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
