#!/usr/bin/env python3
"""LiDAR-based obstacle detection.

Subscribes to /scan (LaserScan), clusters returns into obstacle centers,
publishes /obstacle_points (PointCloud2 in base_footprint).

Filtering:
- Ignores returns < min_range (ground/chassis noise)
- Ignores returns > max_range (irrelevant)
- Clusters nearby returns into obstacle centers with estimated radius
- Publishes surface points around each cluster center
"""

import math
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header


class LidarObstacleNode(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_node')

        self.declare_parameter('min_range', 0.5)       # ignore close returns (chassis noise)
        self.declare_parameter('max_range', 8.0)        # max detection range
        self.declare_parameter('cluster_gap', 0.4)      # max gap between consecutive returns in a cluster (m)
        self.declare_parameter('min_cluster_points', 3)  # minimum returns to count as obstacle
        self.declare_parameter('n_surface_points', 8)    # surface points per obstacle for A* planner

        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.pub = self.create_publisher(PointCloud2, '/obstacle_points', 10)

        self._log_count = 0
        self.get_logger().info('LiDAR obstacle detector started')

    def _scan_cb(self, msg):
        min_r = self.get_parameter('min_range').value
        max_r = self.get_parameter('max_range').value
        cluster_gap = self.get_parameter('cluster_gap').value
        min_pts = self.get_parameter('min_cluster_points').value
        n_surface = self.get_parameter('n_surface_points').value

        # Convert scan to (x, y) in body frame, filtering by range
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if min_r < r < max_r and math.isfinite(r):
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append((x, y, r, angle))
            angle += msg.angle_increment

        if not points:
            self._publish_empty(msg.header)
            return

        # Cluster consecutive scan points that are close together
        # Two points are in the same cluster if the distance between them is < cluster_gap
        clusters = []
        current = [points[0]]
        for i in range(1, len(points)):
            px, py = points[i][0], points[i][1]
            cx, cy = current[-1][0], current[-1][1]
            dist = math.sqrt((px - cx)**2 + (py - cy)**2)
            if dist < cluster_gap:
                current.append(points[i])
            else:
                clusters.append(current)
                current = [points[i]]
        clusters.append(current)

        # Convert clusters to obstacle surface points
        obs_pts = []
        for cluster in clusters:
            if len(cluster) < min_pts:
                continue

            # Cluster center and radius
            xs = [p[0] for p in cluster]
            ys = [p[1] for p in cluster]
            cx = sum(xs) / len(xs)
            cy = sum(ys) / len(ys)

            # Estimate radius from spread
            max_spread = 0.0
            for p in cluster:
                d = math.sqrt((p[0] - cx)**2 + (p[1] - cy)**2)
                if d > max_spread:
                    max_spread = d
            radius = max(max_spread, 0.1)

            # Generate surface points around the cluster center
            for k in range(n_surface):
                angle = 2.0 * math.pi * k / n_surface
                sx = cx + radius * math.cos(angle)
                sy = cy + radius * math.sin(angle)
                obs_pts.append((sx, sy, 0.0))

        # Publish
        header = Header()
        header.frame_id = 'base_footprint'
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        data = bytearray()
        for x, y, z in obs_pts:
            data.extend(struct.pack('fff', x, y, z))

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(obs_pts)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * max(1, len(obs_pts))
        cloud.data = bytes(data)
        cloud.is_dense = True
        self.pub.publish(cloud)

        self._log_count += 1
        if self._log_count % 50 == 0:
            self.get_logger().info(
                f'LiDAR: {len(points)} returns, {len(clusters)} clusters, '
                f'{sum(1 for c in clusters if len(c) >= min_pts)} obstacles')

    def _publish_empty(self, scan_header):
        header = Header()
        header.frame_id = 'base_footprint'
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = 0
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 0
        cloud.data = bytes()
        cloud.is_dense = True
        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
