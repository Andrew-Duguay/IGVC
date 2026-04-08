import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped # Needed for Path messages
from nav_msgs.msg import Path
from cv_bridge import CvBridge
import cv2
import numpy as np
# Ensure this module exists in your workspace
import astar_nav as ast 

class LaneFilterNode(Node):
    def __init__(self):
        super().__init__('lane_filter_node')
        
        self.get_logger().info("Lane Filter Node Started.")

        # Updated topic name placeholder
        self.subscription = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            10
        )

        # Matched variable names to usage in callbacks
        self.lane_pub = self.create_publisher(Image, '/lane_map', 10)
        self.path_pub = self.create_publisher(Path, '/path_direction', 10)
        
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width = frame.shape[:2]

            # 1. Generate Costmap
            lane_mask = ast.lane_costmap(frame)

            # 2. Define Start and Destination for A*
            # Start at the bottom center of the image
            start = (int(width / 2), height - 1)
            # End at the top center (or a lookahead point)
            destination = (int(width / 2), 0)

            # 3. Calculate Path
            path = ast.a_star_search(lane_mask, start, destination)
            
            if path:
                self.publish_path(path, msg.header)
            
            # 4. Publish Debug Image (Lane Mask)
            out_msg = self.bridge.cv2_to_imgmsg(lane_mask, encoding='mono8')
            out_msg.header = msg.header
            self.lane_pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")

    def publish_path(self, path_coords, header):
        path_msg = Path()
        path_msg.header = header
        # Ensure the frame_id matches your TF tree (e.g., 'camera_link' or 'map')
        path_msg.header.frame_id = header.frame_id 

        for (x, y) in path_coords:
            pose = PoseStamped()
            pose.header = header
            
            # Assuming path_coords are in pixel coordinates (x, y)
            # You might need to convert these to world coordinates if this 
            # path is intended for navigation in the real world.
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            
            # Identity orientation
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0 
            
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

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