#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class ObstacleProjectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_projector_node')

        # Declare parameters
        self.declare_parameter('rgb_fx', 462.17)
        self.declare_parameter('rgb_fy', 462.17)
        self.declare_parameter('rgb_cx', 320.0)
        self.declare_parameter('rgb_cy', 240.0)
        self.declare_parameter('depth_fx', 337.22)
        self.declare_parameter('depth_fy', 337.22)
        self.declare_parameter('depth_cx', 320.0)
        self.declare_parameter('depth_cy', 240.0)
        self.declare_parameter('camera_height', 0.27)
        self.declare_parameter('camera_forward_offset', 0.38)
        self.declare_parameter('depth_patch_size', 5)
        self.declare_parameter('camera_lateral_offset', 0.0)
        self.declare_parameter('min_range', 0.3)
        self.declare_parameter('max_range', 8.0)
        self.declare_parameter('sync_slop', 0.1)
        self.declare_parameter('detection_topic', '/yolo/detections')
        self.declare_parameter('depth_topic', '/depth/image_raw')

        self.bridge = CvBridge()

        # Synchronized subscribers
        detection_topic = self.get_parameter('detection_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        det_sub = Subscriber(self, Detection2DArray, detection_topic)
        depth_sub = Subscriber(self, Image, depth_topic)

        slop = self.get_parameter('sync_slop').value
        self.sync = ApproximateTimeSynchronizer(
            [det_sub, depth_sub], queue_size=10, slop=slop)
        self.sync.registerCallback(self.sync_callback)

        self.pc_pub = self.create_publisher(PointCloud2, '/obstacle_points', 10)

        self.get_logger().info('Obstacle projector node started')

    def sync_callback(self, det_msg: Detection2DArray, depth_msg: Image):
        if len(det_msg.detections) == 0:
            return

        p = self.get_parameter
        rgb_fx = p('rgb_fx').value
        rgb_fy = p('rgb_fy').value
        rgb_cx = p('rgb_cx').value
        rgb_cy = p('rgb_cy').value
        depth_fx = p('depth_fx').value
        depth_fy = p('depth_fy').value
        depth_cx = p('depth_cx').value
        depth_cy = p('depth_cy').value
        cam_h = p('camera_height').value
        cam_fwd = p('camera_forward_offset').value
        cam_lat = p('camera_lateral_offset').value
        patch_size = p('depth_patch_size').value
        min_r = p('min_range').value
        max_r = p('max_range').value

        # Convert depth image (float32 meters)
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        img_h, img_w = depth_img.shape
        half_patch = patch_size // 2

        points = []

        for det in det_msg.detections:
            # Get bbox center in RGB pixel coords
            u_rgb = det.bbox.center.position.x
            v_rgb = det.bbox.center.position.y

            # Map RGB pixel to normalized ray
            x_norm = (u_rgb - rgb_cx) / rgb_fx
            y_norm = (v_rgb - rgb_cy) / rgb_fy

            # Reproject normalized ray to depth pixel coords
            u_depth = int(x_norm * depth_fx + depth_cx)
            v_depth = int(y_norm * depth_fy + depth_cy)

            # Bounds check
            if (u_depth < half_patch or u_depth >= img_w - half_patch or
                    v_depth < half_patch or v_depth >= img_h - half_patch):
                continue

            # Median depth from patch, excluding invalid values
            patch = depth_img[
                v_depth - half_patch:v_depth + half_patch + 1,
                u_depth - half_patch:u_depth + half_patch + 1
            ]
            valid = patch[(patch > 0) & np.isfinite(patch)]
            if len(valid) == 0:
                continue
            depth = float(np.median(valid))

            # Project to base_footprint frame
            X = depth + cam_fwd
            Y = -x_norm * depth + cam_lat
            Z = cam_h - y_norm * depth

            # Range filter
            r = np.sqrt(X * X + Y * Y)
            if r < min_r or r > max_r:
                continue

            points.append([X, Y, Z])

        if len(points) == 0:
            return

        # Build and publish PointCloud2
        header = Header()
        header.stamp = det_msg.header.stamp
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
    node = ObstacleProjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
