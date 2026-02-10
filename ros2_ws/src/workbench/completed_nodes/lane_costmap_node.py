import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from cv_bridge import CvBridge
import cv2
import numpy as np


class LaneCostmapNode(Node):
    def __init__(self):
        super().__init__('lane_costmap_node')
        
        self.get_logger().info("Lane Costmap Node Started.")

        # Updated topic name placeholder
        self.subscription = self.create_subscription(
            Image, 
            '/right_camera/image_raw', 
            self.image_callback, 
            10
        )

        # Matched variable names to usage in callbacks
        self.costmap_pub = self.create_publisher(Image, '/lane_costmap', 10)
        
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width = frame.shape[:2]

            # 1. Generate Costmap
            lane_mask = get_lane_costmap(frame)
            
            out_msg = self.bridge.cv2_to_imgmsg(lane_mask, encoding='mono8')
            out_msg.header = msg.header
            self.costmap_pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")


def get_lane_costmap(image, low_hsv=[18, 94, 140], high_hsv=[48, 255, 255]):

    """
    Converts a color image into a binary lane costmap using HSV thresholding.
    
    :param image: Input BGR image.
    :param low_hsv: Lower bound for HSV threshold (default tuned for yellow).
    :param high_hsv: Upper bound for HSV threshold.
    :return: Binary costmap where lanes are 255 (obstacles) and road is 0.
    """
    # 1. Convert BGR to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # 2. Define range and threshold the HSV image
    lower_limit = np.array(low_hsv)
    upper_limit = np.array(high_hsv)
    lane_costmap = cv2.inRange(hsv, lower_limit, upper_limit)
    
    return lane_costmap

def main(args=None):
    rclpy.init(args=args)
    node = LaneCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()