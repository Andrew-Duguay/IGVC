#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs_py import point_cloud2
import numpy as np

class LaneCenterlineNode(Node):
    def __init__(self):
        super().__init__('lane_centerline_node')

        # --- Parameters (Only the geometry ones) ---
        self.declare_parameter('min_x', 0.5)
        self.declare_parameter('max_x', 5.0)
        self.declare_parameter('bin_width', 0.5)
        self.declare_parameter('min_points_per_bin', 3)
        self.declare_parameter('lane_width', 3.0)
        self.declare_parameter('centerline_spread_threshold', 1.0)
        self.declare_parameter('exp_avg_alpha', 0.4)

        # Input: Raw Lane Points
        self.create_subscription(PointCloud2, '/lane_points', self._lane_cb, 10)
        
        # Output: Clean Path
        self.centerline_pub = self.create_publisher(Path, '/autonomy/centerline', 10)

        self.lane_points = []
        self.prev_centerline = []
        self.get_logger().info('Centerline Processor started')

    def _p(self, name):
        return self.get_parameter(name).value

    def _lane_cb(self, msg: PointCloud2):
        # 1. Unpack Cloud       
        self.lane_points = get_points_from_cloud(msg)

        # 2. Compute Centerline
        centerline = self._compute_centerline()

        # 3. Publish Path
        if centerline:
            self._publish_path(centerline, msg.header)

    def get_points_from_cloud(cloud: PointCloud2):
        pts = []
        for p in point_cloud2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True):
            pts.append((p[0], p[1]))
        return pts

    def _compute_centerline(self):
        """Bin lane points by X, cluster into two lines, compute center."""
        if not self.lane_points:
            return []

        min_pts = self._p('min_points_per_bin')
        half_lane = self._p('lane_width') / 2.0
        spread_thresh = self._p('centerline_spread_threshold')

        bins = sort_points_into_bins()

        centerline = []
        for i in range(n_bins):
            x_center = min_x + (i + 0.5) * bin_width
            ys = sorted(bins[i])

            if len(ys) < min_pts:
                continue

            two_lines = False
            # ... (Gap logic omitted for brevity, exact same as original) ...
            # Simplification: Calculate midpoint
            y_center = np.mean(ys)
            
            # Refined gap logic (from original)
            if len(ys) >= 2 * min_pts:
                max_gap = 0.0
                split_idx = len(ys) // 2
                for j in range(min_pts - 1, len(ys) - min_pts):
                    gap = ys[j + 1] - ys[j]
                    if gap > max_gap:
                        max_gap = gap
                        split_idx = j + 1
                if max_gap > 0.3:
                    y_center = (np.mean(ys[:split_idx]) + np.mean(ys[split_idx:])) / 2.0
                    two_lines = True
            
            if not two_lines:
                # Single line logic
                mean_y = np.mean(ys)
                # If points are widely spread, assume we see both sides
                if (ys[-1] - ys[0]) > spread_thresh:
                    y_center = (ys[0] + ys[-1]) / 2.0
                else:
                    # Offset logic (assume left or right based on sign)
                    y_center = mean_y - half_lane if mean_y > 0 else mean_y + half_lane

            centerline.append((x_center, y_center))

        smoothed = smooth_centerline(centerline)
        self.prev_centerline = smoothed
        return smoothed

    def _publish_path(self, centerline, header):
        path = Path()
        path.header = header
        path.header.frame_id = 'base_footprint'
        for cx, cy in centerline:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(cx)
            pose.pose.position.y = float(cy)
            path.poses.append(pose)
        self.centerline_pub.publish(path)

    def smooth_centerline(self, centerline):
        # Smoothing (Exponential Moving Average)
        if self.prev_centerline and len(centerline) == len(self.prev_centerline):
            alpha = self._p('exp_avg_alpha')
            centerline = [
                (x, alpha * y + (1.0 - alpha) * py)
                for (x, y), (_, py) in zip(centerline, self.prev_centerline)
            ]
        return centerline

    def sort_points_into_bins(self):
        min_x = self._p('min_x')
        max_x = self._p('max_x')
        bin_width = self._p('bin_width')
        n_bins = int((max_x - min_x) / bin_width)
        bins = [[] for _ in range(n_bins)]

        for x, y in self.lane_points:
            if (x < min_x) or (x >= max_x):
                continue
            idx = int((x - min_x) / bin_width)
            idx = min(idx, n_bins - 1)
            bins[idx].append(y)

        return bins

def main(args=None):
    rclpy.init(args=args)
    node = LaneCenterlineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()