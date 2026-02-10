import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import message_filters
import numpy as np

class StereoDepthNode(Node):
    def __init__(self):
        super().__init__('stereo_depth_node')
        # Declare parameters so you can tune them without changing code
        self.declare_parameter('baseline', 0.05) # Distance between cameras (meters)
        self.declare_parameter('focal_length', 488.37) # Focal length (pixels)

        # FIXED: Typo 'self.get._parameter' -> 'self.get_parameter'
        self.baseline = self.get_parameter('baseline').get_parameter_value().double_value
        self.focal_length = self.get_parameter('focal_length').get_parameter_value().double_value

        # FIXED: Typo 'get_looger' -> 'get_logger'
        self.get_logger().info(f"Node Started. Baseline={self.baseline} Focal Length={self.focal_length}")

        # Stereo SGBM Tuning
        # These parameters control the quality vs speed of the depth map
        min_disparity = 0 
        num_disparities = 16 * 6 # Must be divisible by 16
        block_size = 5
        
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=min_disparity,
            numDisparities=num_disparities, 
            blockSize=block_size,
            P1=8 * 3 * block_size**2,
            P2=32 * 3 * block_size**2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32
        )
        
        # Subscribe to both cameras using message_filters
        self.sub_left = message_filters.Subscriber(self, Image, '/left_camera_raw')
        self.sub_right = message_filters.Subscriber(self, Image, '/right_camera_raw')
        
        # Synchronize the topics
        # 'slop' is the max time delay allowed between left/right frames
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_left, self.sub_right], 
            queue_size=10, 
            slop=0.05
        )
        self.ts.registerCallback(self.sync_callback)

        self.pub_depth = self.create_publisher(Image, '/depth_map', 10)
        self.bridge = CvBridge()

    def sync_callback(self, msg_left, msg_right):
        try:
            # SGBM works on grayscale images ('mono8')
            img_l = self.bridge.imgmsg_to_cv2(msg_left, desired_encoding='mono8')
            img_r = self.bridge.imgmsg_to_cv2(msg_right, desired_encoding='mono8')
            
            # 1. Compute Disparity
            disparity = self.stereo.compute(img_l, img_r)
            
            # SGBM output is fixed-point (multiplied by 16). We divide to get pixels.
            disparity = disparity.astype(np.float32) / 16.0

            # Prevent divide-by-zero errors
            disparity[disparity <= 0.0] = 0.1

            # 2. Compute Depth: Z = (f * b) / d
            depth = (self.focal_length * self.baseline) / disparity

            # 3. Convert to Millimeters (standard for uint16 depth images)
            depth_mm = (depth * 1000).astype(np.uint16)

            # 4. Publish
            # encoding='16UC1' means 16-bit Unsigned Single-Channel
            out_msg = self.bridge.cv2_to_imgmsg(depth_mm, encoding='16UC1')
            out_msg.header = msg_left.header # Keep original timestamp/frame_id

            self.pub_depth.publish(out_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing stereo pair: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()