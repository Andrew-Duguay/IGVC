#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector_node')

        # Declare parameters
        self.declare_parameter('hsv_low_h', 0)
        self.declare_parameter('hsv_high_h', 180)
        self.declare_parameter('hsv_low_s', 0)
        self.declare_parameter('hsv_high_s', 50)
        self.declare_parameter('hsv_low_v', 180)
        self.declare_parameter('hsv_high_v', 255)
        self.declare_parameter('morph_kernel_size', 3)
        self.declare_parameter('roi_top_row', 245)
        self.declare_parameter('subsample_stride', 4)
        self.declare_parameter('rgb_fx', 462.17)
        self.declare_parameter('rgb_fy', 462.17)
        self.declare_parameter('rgb_cx', 320.0)
        self.declare_parameter('rgb_cy', 240.0)
        self.declare_parameter('camera_height', 0.27)
        self.declare_parameter('camera_forward_offset', 0.38)
        self.declare_parameter('camera_lateral_offset', 0.0)
        self.declare_parameter('min_range', 0.4)
        self.declare_parameter('max_range', 6.0)
        self.declare_parameter('frame_skip', 2)
        self.declare_parameter('image_topic', '/image_raw')

        self.bridge = CvBridge()
        self.frame_count = 0

        image_topic = self.get_parameter('image_topic').value
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        self.pc_pub = self.create_publisher(PointCloud2, '/lane_points', 10)
        self.mask_pub = self.create_publisher(Image, '/lane_mask', 10)

        self.get_logger().info('Lane detector node started')

    def _get_params(self):
        p = self.get_parameter
        return {
            'hsv_low': np.array([
                p('hsv_low_h').value, p('hsv_low_s').value, p('hsv_low_v').value
            ]),
            'hsv_high': np.array([
                p('hsv_high_h').value, p('hsv_high_s').value, p('hsv_high_v').value
            ]),
            'morph_k': p('morph_kernel_size').value,
            'roi_top': p('roi_top_row').value,
            'stride': p('subsample_stride').value,
            'fx': p('rgb_fx').value,
            'fy': p('rgb_fy').value,
            'cx': p('rgb_cx').value,
            'cy': p('rgb_cy').value,
            'cam_h': p('camera_height').value,
            'cam_fwd': p('camera_forward_offset').value,
            'cam_lat': p('camera_lateral_offset').value,
            'min_r': p('min_range').value,
            'max_r': p('max_range').value,
        }

    def image_callback(self, msg: Image):
        # Rate limit: process every Nth frame
        self.frame_count += 1
        frame_skip = self.get_parameter('frame_skip').value
        if self.frame_count % frame_skip != 0:
            return

        params = self._get_params()

        # Convert to OpenCV BGR then HSV
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # White line detection via HSV threshold
        mask = cv2.inRange(hsv, params['hsv_low'], params['hsv_high'])

        # Morphological open then close to clean noise
        k = params['morph_k']
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Publish debug mask
        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

        # Extract white pixel coordinates from bottom half only (ROI)
        roi_top = params['roi_top']
        roi_mask = mask[roi_top:, :]

        # Get pixel coordinates of white pixels
        vs, us = np.where(roi_mask > 0)
        if len(vs) == 0:
            return

        # Offset v back to full image coordinates
        vs = vs + roi_top

        # Subsample
        stride = params['stride']
        vs = vs[::stride]
        us = us[::stride]

        if len(vs) == 0:
            return

        # Vectorized ground-plane projection
        fx = params['fx']
        fy = params['fy']
        cx = params['cx']
        cy = params['cy']
        cam_h = params['cam_h']
        cam_fwd = params['cam_fwd']
        cam_lat = params['cam_lat']

        # Normalized image coordinates (optical frame)
        y_norm = (vs.astype(np.float32) - cy) / fy
        x_norm = (us.astype(np.float32) - cx) / fx

        # Only keep pixels below the horizon (y_norm > 0 means below optical center)
        valid = y_norm > 0.01
        y_norm = y_norm[valid]
        x_norm = x_norm[valid]

        if len(y_norm) == 0:
            return

        # Flat ground intersection: t = camera_height / y_norm
        t = cam_h / y_norm

        # Project to base_footprint frame
        # X = forward (depth along ground + camera offset)
        # Y = left (negative of rightward optical x)
        # Z = 0 (ground plane)
        X = t + cam_fwd
        Y = -x_norm * t + cam_lat
        Z = np.zeros_like(X)

        # Range filter
        ranges = np.sqrt(X * X + Y * Y)
        in_range = (ranges >= params['min_r']) & (ranges <= params['max_r'])
        X = X[in_range]
        Y = Y[in_range]
        Z = Z[in_range]

        if len(X) == 0:
            return

        # Build and publish PointCloud2
        points = np.stack([X, Y, Z], axis=-1).tolist()
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = 'base_footprint'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pc_msg = point_cloud2.create_cloud(header, fields, points)
        self.pc_pub.publish(pc_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
