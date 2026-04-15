#!/usr/bin/env python3
"""Lane point accumulator — merges left+right camera detections with persistence.

Stores detected lane points in a world-frame grid dict (keyed by 20cm
cell) so duplicates from overlapping frames collapse into a single entry.
Memory is bounded by area covered, not by time. Points expire by distance
from the robot, not TTL.
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
        self.declare_parameter('grid_res', 0.05)        # metres per cell
        self.declare_parameter('retention_range', 25.0) # distance past which cells are forgotten

        self._mode = self.get_parameter('mode').value
        self._max_range = self.get_parameter('max_range').value
        self._grid_res = self.get_parameter('grid_res').value
        self._retention_range = self.get_parameter('retention_range').value

        # World-frame grid: {(ix, iy): (wx, wy)} where ix=int(wx/grid_res)
        self._lane_world: dict[tuple[int, int], tuple[float, float]] = {}

        # Latest raw points for merge mode
        self._left_pts = []
        self._right_pts = []

        # Current odom pose
        self._wx = 0.0
        self._wy = 0.0
        self._wyaw = 0.0
        self._pose_valid = False

        self.create_subscription(PointCloud2, '/candidate/left_lane_points', self._left_cb, 10)
        self.create_subscription(PointCloud2, '/candidate/right_lane_points', self._right_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        self.pub = self.create_publisher(PointCloud2, '/candidate/lane_points', 10)
        self.viz_pub = self.create_publisher(PointCloud2, '/candidate/lane_points_viz', 10)

        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f'Lane accumulator started (mode={self._mode}, grid={self._grid_res}m)')

    def _read_points(self, msg):
        return [(p[0], p[1]) for p in
                point_cloud2.read_points(msg, field_names=('x', 'y'), skip_nans=True)]

    def _left_cb(self, msg):
        pts = self._read_points(msg)
        self._left_pts = pts
        if self._mode == 'accumulate' and self._pose_valid:
            self._ingest(pts)

    def _right_cb(self, msg):
        pts = self._read_points(msg)
        self._right_pts = pts
        if self._mode == 'accumulate' and self._pose_valid:
            self._ingest(pts)

    def _odom_cb(self, msg):
        self._wx = msg.pose.pose.position.x
        self._wy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._wyaw = math.atan2(siny, cosy)
        self._pose_valid = True

    def _ingest(self, body_pts):
        if not body_pts:
            return
        cos_y = math.cos(self._wyaw)
        sin_y = math.sin(self._wyaw)
        wx0, wy0 = self._wx, self._wy
        res = self._grid_res
        grid = self._lane_world
        for bx, by in body_pts:
            wx = wx0 + bx * cos_y - by * sin_y
            wy = wy0 + bx * sin_y + by * cos_y
            grid[(int(wx / res), int(wy / res))] = (wx, wy)

    def _publish(self):
        if self._mode == 'merge':
            self._publish_merge()
        else:
            self._publish_accumulated()

    def _publish_merge(self):
        combined = self._left_pts + self._right_pts
        if combined:
            self._publish_cloud_body(combined)

    def _publish_accumulated(self):
        if not self._pose_valid or not self._lane_world:
            return

        max_r2 = self._max_range ** 2
        keep_r2 = self._retention_range ** 2
        cur_wx, cur_wy, cur_yaw = self._wx, self._wy, self._wyaw
        cur_cos = math.cos(cur_yaw)
        cur_sin = math.sin(cur_yaw)

        body_pts = []     # forward-biased, for the planner
        odom_pts = []     # world-frame, for the viz map
        stale_keys = []

        for key, (wx, wy) in self._lane_world.items():
            ddx = wx - cur_wx
            ddy = wy - cur_wy
            d2 = ddx * ddx + ddy * ddy
            if d2 > keep_r2:
                stale_keys.append(key)
                continue
            if d2 > max_r2:
                continue

            odom_pts.append((wx, wy))

            cbx = ddx * cur_cos + ddy * cur_sin
            cby = -ddx * cur_sin + ddy * cur_cos
            if cbx > -2.0:
                body_pts.append((cbx, cby))

        for key in stale_keys:
            del self._lane_world[key]

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
