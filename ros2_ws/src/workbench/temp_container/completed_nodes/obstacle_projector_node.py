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
        self.declare_parameter('min_range', 0.3)
        self.declare_parameter('max_range', 8.0)
        self.declare_parameter('sync_slop', 0.1)

        self.bridge = CvBridge()

        # Synchronized subscribers
        yolo_sub = Subscriber(self, Detection2DArray, '/yolo/detections')
        depth_sub = Subscriber(self, Image, '/depth/image_raw')

        slop = self.get_parameter('sync_slop').value
        self.sync = ApproximateTimeSynchronizer(
            [yolo_sub, depth_sub], 
            queue_size=10, 
            slop=slop
        )
        self.sync.registerCallback(self.synchronized_callback)

        self.pc_pub = self.create_publisher(PointCloud2, '/obstacle_points', 10)

        self.get_logger().info('Obstacle projector node started')

    def synchronized_callback(self, yolo_msg: Detection2DArray, depth_msg: Image):
        if len(yolo_msg.detections) == 0:
            return

        params = self._get_params()

        # 1. Convert depth image (float32 meters)
        try:
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # 2. Process detections into 3D points
        points_3d = self.extract_obstacle_points(yolo_msg.detections, depth_img, params)
        if not points_3d:
            return

        # 3. Publish Point Cloud
        pc_msg = self.build_point_cloud(points_3d, yolo_msg.header)
        self.pc_pub.publish(pc_msg)
    
    def _get_params(self):
        """Fetch all parameters at once."""
        p = self.get_parameter
        return {
            'rgb_fx': p('rgb_fx').value,
            'rgb_fy': p('rgb_fy').value,
            'rgb_cx': p('rgb_cx').value,
            'rgb_cy': p('rgb_cy').value,
            'depth_fx': p('depth_fx').value,
            'depth_fy': p('depth_fy').value,
            'depth_cx': p('depth_cx').value,
            'depth_cy': p('depth_cy').value,
            'cam_h': p('camera_height').value,
            'cam_fwd': p('camera_forward_offset').value,
            'patch_size': p('depth_patch_size').value,
            'min_r': p('min_range').value,
            'max_r': p('max_range').value,
        }
    
    def extract_obstacle_points(self, detections, depth_img, params):
        """
        Iterates over YOLO detections and calculates 3D coordinates for each.
        """
        valid_points = []
        img_h, img_w = depth_img.shape

        for det in detections:
            # Calculate single point for this detection
            point = self.compute_3d_position(det, depth_img, img_w, img_h, params)
            if point is not None:
                valid_points.append(point)
        
        return valid_points

    def compute_3d_position(self, det, depth_img, img_w, img_h, params):
        """
        Projects a single YOLO detection to 3D space.
        """
        # 1. Get bbox center in RGB pixel coords
        u_rgb = det.bbox.center.position.x
        v_rgb = det.bbox.center.position.y

        # 2. Map RGB pixel to normalized ray (Optical Frame)
        x_norm = (u_rgb - params['rgb_cx']) / params['rgb_fx']
        y_norm = (v_rgb - params['rgb_cy']) / params['rgb_fy']

        # 3. Reproject normalized ray to Depth Pixel Coords
        # (Handles case where RGB and Depth have different resolutions/intrinsics)
        u_depth = int(x_norm * params['depth_fx'] + params['depth_cx'])
        v_depth = int(y_norm * params['depth_fy'] + params['depth_cy'])

        # 4. Extract Depth (Median Filter)
        half_patch = params['patch_size'] // 2
        
        # Bounds check
        if (u_depth < half_patch or u_depth >= img_w - half_patch or
            v_depth < half_patch or v_depth >= img_h - half_patch):
            return None

        patch = depth_img[
            v_depth - half_patch : v_depth + half_patch + 1,
            u_depth - half_patch : u_depth + half_patch + 1
        ]
        
        # Filter valid depth values
        valid_depths = patch[(patch > 0) & np.isfinite(patch)]
        if len(valid_depths) == 0:
            return None
            
        depth = float(np.median(valid_depths))

        # 5. Project to Robot Frame (base_footprint)
        # Assumes camera is facing forward
        X = depth + params['cam_fwd']
        Y = -x_norm * depth
        Z = params['cam_h'] - y_norm * depth

        # 6. Range Filter
        r = np.sqrt(X**2 + Y**2)
        if r < params['min_r'] or r > params['max_r']:
            return None

        return [X, Y, Z]

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
