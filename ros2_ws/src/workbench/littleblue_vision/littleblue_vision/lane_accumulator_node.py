#!/usr/bin/env python3
"""Lane point accumulator — merges left+right camera detections with persistence.

Keeps a rolling buffer of raw body-frame snapshots with their odom poses.
At publish time, re-projects all snapshots to current body frame using
consistent odom transforms. No grid quantization, no pose mismatch.

Points expire by distance from the robot (not TTL).
"""

import math
import struct
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class LaneAccumulatorNode(Node):
    def __init__(self):
        super().__init__('lane_accumulator_node')

        self.declare_parameter('mode', 'accumulate')
        self.declare_parameter('max_range', 8.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('max_snapshots', 50)  # rolling buffer size

        self._mode = self.get_parameter('mode').value
        self._max_range = self.get_parameter('max_range').value
        self._max_snapshots = self.get_parameter('max_snapshots').value

        # Rolling buffer: list of (odom_x, odom_y, odom_yaw, [(bx, by), ...])
        self._snapshots = []

        # Latest raw points for merge mode
        self._left_pts = []
        self._right_pts = []

        # Current odom
        self._wx = 0.0
        self._wy = 0.0
        self._wyaw = 0.0
        self._pose_valid = False

        # Subscribers
        self.create_subscription(PointCloud2, '/candidate/left_lane_points', self._left_cb, 10)
        self.create_subscription(PointCloud2, '/candidate/right_lane_points', self._right_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # Publishers
        self.pub = self.create_publisher(PointCloud2, '/candidate/lane_points', 10)
        self.viz_pub = self.create_publisher(PointCloud2, '/candidate/lane_points_viz', 10)

        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(f'Lane accumulator started (mode={self._mode})')

    def _read_points(self, msg):
        return [(p[0], p[1]) for p in
                point_cloud2.read_points(msg, field_names=('x', 'y'), skip_nans=True)]

    def _left_cb(self, msg):
        pts = self._read_points(msg)
        self._left_pts = pts
        if self._mode == 'accumulate' and self._pose_valid:
            self._store_snapshot(pts)

    def _right_cb(self, msg):
        pts = self._read_points(msg)
        self._right_pts = pts
        if self._mode == 'accumulate' and self._pose_valid:
            self._store_snapshot(pts)

    def _odom_cb(self, msg):
        self._wx = msg.pose.pose.position.x
        self._wy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._wyaw = math.atan2(siny, cosy)
        self._pose_valid = True

    def _store_snapshot(self, body_pts):
        """Store a snapshot of body-frame points with the current odom pose."""
        if not body_pts:
            return
        self._snapshots.append((self._wx, self._wy, self._wyaw, body_pts))
        # Trim to max size
        if len(self._snapshots) > self._max_snapshots:
            self._snapshots = self._snapshots[-self._max_snapshots:]

    def _publish(self):
        if self._mode == 'merge':
            self._publish_merge()
        else:
            self._publish_accumulated()

    def _publish_merge(self):
        combined = self._left_pts + self._right_pts
        if not combined:
            return
        self._publish_cloud_body(combined)

    def _publish_accumulated(self):
        if not self._pose_valid or not self._snapshots:
            return

        max_r2 = self._max_range ** 2
        cur_wx, cur_wy, cur_yaw = self._wx, self._wy, self._wyaw
        cur_cos = math.cos(cur_yaw)
        cur_sin = math.sin(cur_yaw)

        body_pts = []
        odom_pts = []
        keep = []

        for snap_wx, snap_wy, snap_yaw, snap_pts in self._snapshots:
            # Check if snapshot origin is within range
            dx = snap_wx - cur_wx
            dy = snap_wy - cur_wy
            if dx * dx + dy * dy > (self._max_range + 10.0) ** 2:
                continue  # snapshot too far, discard

            snap_cos = math.cos(snap_yaw)
            snap_sin = math.sin(snap_yaw)
            has_close_pts = False

            for bx, by in snap_pts:
                # Snapshot body → odom
                ox = snap_wx + bx * snap_cos - by * snap_sin
                oy = snap_wy + bx * snap_sin + by * snap_cos

                # Distance from current robot in odom frame
                ddx = ox - cur_wx
                ddy = oy - cur_wy
                if ddx * ddx + ddy * ddy > max_r2:
                    continue

                # Odom → current body
                cbx = ddx * cur_cos + ddy * cur_sin
                cby = -ddx * cur_sin + ddy * cur_cos

                if cbx > -2.0:
                    body_pts.append((cbx, cby))
                    odom_pts.append((ox, oy))
                    has_close_pts = True

            if has_close_pts:
                keep.append((snap_wx, snap_wy, snap_yaw, snap_pts))

        self._snapshots = keep

        if body_pts:
            self._publish_cloud_body(body_pts)
        if odom_pts:
            self._publish_cloud_odom(odom_pts)

    def _publish_cloud_body(self, pts):
        header = Header()
        header.frame_id = 'base_footprint'
        self._publish_cloud(self.pub, header, pts)

    def _publish_cloud_odom(self, pts):
        header = Header()
        header.frame_id = 'odom'
        self._publish_cloud(self.viz_pub, header, pts)

    def _publish_cloud(self, publisher, header, pts):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        data = bytearray()
        for x, y in pts:
            data.extend(struct.pack('fff', x, y, 0.0))

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(pts)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * len(pts)
        cloud.data = bytes(data)
        cloud.is_dense = True
        publisher.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = LaneAccumulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
