#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class LaneProjectorNode(Node):
    def __init__(self):
        super().__init__('lane_projector_node')

        # Declare parameters
        self.declare_parameter('roi_top_row', 245)
        self.declare_parameter('subsample_stride', 4)
        self.declare_parameter('rgb_fx', 462.17)
        self.declare_parameter('rgb_fy', 462.17)
        self.declare_parameter('rgb_cx', 320.0)
        self.declare_parameter('rgb_cy', 240.0)
        self.declare_parameter('camera_height', 0.27)
        self.declare_parameter('camera_forward_offset', 0.38)
        self.declare_parameter('min_range', 0.4)
        self.declare_parameter('max_range', 6.0)
        self.declare_parameter('frame_skip', 2)

        self.bridge = CvBridge()
        self.frame_count = 0

        self.image_sub = self.create_subscription(
            Image, '/filtered_lanes', self.image_callback, 10)
        self.pc_pub = self.create_publisher(PointCloud2, '/lane_points', 10)

        self.get_logger().info('Lane detector node started')

    def _get_params(self):
        p = self.get_parameter
        return {
            'roi_top': p('roi_top_row').value,
            'stride': p('subsample_stride').value,
            'fx': p('rgb_fx').value,
            'fy': p('rgb_fy').value,
            'cx': p('rgb_cx').value,
            'cy': p('rgb_cy').value,
            'cam_h': p('camera_height').value,
            'cam_fwd': p('camera_forward_offset').value,
            'min_r': p('min_range').value,
            'max_r': p('max_range').value,
        }

    def image_callback(self, msg: Image):
        params = self._get_params()
        # Rate limit: process every Nth frame
        self.frame_count += 1
        frame_skip = self.get_parameter('frame_skip').value
        if self.frame_count % frame_skip != 0:
            return

        # Convert ROS image to OpenCV 
        filtered_lanes = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

        vs, us = extract_lane_pixels(filtered_lanes, params)
        if vs is None:
            return

        X, Y, Z = project_pixels_to_ground(vs, us, params)
        if X is None:
            return

        x_filt, y_filt, z_filt = filter_points_by_range(X, Y, Z, params)
        if x_filt is None:
            return

        pc_msg = build_point_cloud(x_filt, y_filt, z_filt, msg.header)
        self.pc_pub.publish(pc_msg)

    def extract_lane_pixels(self, pic, params):
        roi_top = params['roi_top']
        roi_mask = filtered_lanes[roi_top:, :]
        rows, cols = np.where(roi_mask > 0)
        if len(rows) == 0:
            return None, None
        # Offset v back to full image coordinates
        rows = rows + roi_top

        # Subsample
        stride = params['stride']
        rows = rows[::stride]
        cols = cols[::stride]

        return rows.astype(np.float32), cols.astype(np.float32)
    
    def project_pixels_to_ground(self, v, u, params):
        fx, fy = params['fx'], params['fy']
        cx, cy = params['cx'], params['cy']
        cam_h = params['cam_h']
        cam_fwd = params['cam_fwd']

        # 1. Project to Optical Coordinates (Normalized Pinhole Model)
        # Optical Frame: x_opt is Right, y_opt is Down
        y_norm = (v - cy) / fy
        x_norm = (u - cx) / fx

        # 2. Filter: Horizon Check
        # y_norm corresponds to the "down" angle.
        # Values <= 0.01 are at or above the horizon (sky/infinity).
        valid_indices = y_norm > 0.01
        y_norm = y_norm[valid_indices]
        x_norm = x_norm[valid_indices]

        if len(y_norm) == 0:
            return None, None, None

        # 3. Ground Plane Intersection
        # t is the depth (Z_optical) along the ray to hit the ground at distance 'cam_h'
        t = cam_h / y_norm

        # 4. Transform to Robot Frame (base_footprint)
        # Optical Frame:  Z = Forward, X = Right,    Y = Down
        # Robot Frame:    X = Forward, Y = Left,     Z = Up
       
        # Robot X (Forward) = Optical Depth (t) + Camera Offset
        X_robot = t + cam_fwd
        
        # Robot Y (Left) = -Optical X (Right) * scale
        Y_robot = -x_norm * t
        
        # Robot Z (Up) = 0.0 (Projected to ground)
        Z_robot = np.zeros_like(X_robot)

        return X_robot, Y_robot, Z_robot

    def filter_points_by_range(self, x, y, z, params):
        """
        Filters 3D points based on min and max distance from the robot origin.
        """
        distances = np.sqrt(x**2 + y**2)
        range_mask = (distances >= params['min_r']) & (distances <= params['max_r'])

        x_filtered = x[range_mask]
        y_filtered = y[range_mask]
        z_filtered = z[range_mask]

        if len(x_filtered) == 0:
            return None, None, None
            
        return x_filtered, y_filtered, z_filtered

    def build_point_cloud(self, x, y, z, header):
        # Stack coordinates: Shape (N, 3)
        points = np.stack([x, y, z], axis=-1)

        # Override frame_id to be the robot base, since we projected to ground
        header.frame_id = 'base_footprint'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        pointCloud = point_cloud2.create_cloud(header, fields, points)
        return pointCloud


def main(args=None):
    rclpy.init(args=args)
    node = LaneProjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
