import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from cv_bridge import CvBridge
import cv2
import numpy as np

morph_kernel_size = 3
low_hsv_threshold=[18, 94, 140]
high_hsv_threshold=[48, 255, 255]

class LaneFilterNode(Node):
    def __init__(self):
        super().__init__('lane_filter_node')

        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)

        self.filter_pub = self.create_publisher(Image, '/filtered_lanes', 10)  

        self.bridge = CvBridge()

        self.get_logger().info("Lane Filter Node Started.")

    def image_callback(self, msg: Image):
        try:
            # 1. Convert ROS image to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 2. mask all but lanes
            lane_mask = filter_lanes(frame)

            # 3. Morphological open then close to clean noise
            morphed_mask = morphImage(lane_mask)
            
            # 4. Publish
            out_msg = self.bridge.cv2_to_imgmsg(lane_mask, encoding='mono8')
            out_msg.header = msg.header
            self.filter_pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")

def morph_image(picture, k=morph_kernel_size):
    """
    Performs open then close morphological transform to clean noise.
    
    :param image: single-channel image.
    :param k: kernel size for transform.
    :return: single-channel image.
    """
    # Morphological open then close to clean noise
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
    morphed = cv2.morphologyEx(picture, cv2.MORPH_OPEN, kernel)
    unmorphed = cv2.morphologyEx(morphed, cv2.MORPH_CLOSE, kernel)
    return unmorphed
    
def filter_lanes(picture: Image, low_hsv=low_hsv_threshold, high_hsv=high_hsv_theshold):

    """
    Converts a color image into a binary lane costmap using HSV thresholding.
    
    :param image: Input BGR image.
    :param low_hsv: Lower bound for HSV threshold.
    :param high_hsv: Upper bound for HSV threshold.
    :return: Single-channel image where lanes are 255 otherwise 0.
    """
    # Convert BGR to HSV
    hsv = cv2.cvtColor(picture, cv2.COLOR_BGR2HSV)

    # White line detection via HSV threshold
    low_hsv = np.array(low_hsv)
    high_hsv = np.array(high_hsv)
    masked = cv2.inRange(hsv, low_hsv, high_hsv)

    return masked

def main(args=None):
    rclpy.init(args=args)
    node = LaneFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()