import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import Path
import cv2
import message_filters
import numpy as np
import astar_nav as ast

class LaneFilterNode(Node):
    def __init__(self):
        super().__init__('lane_filter_node')
        

        

        self.get_looger().info(f"lane filter Node Started. ")

       
        
        self.subscribtion = self.create_subscription(Image,'/IDK_camera_raw',self.image_callback,10)


        self.pub_lane = self.create_publisher(Image, '/lane_map', 10)
        self.pub_road = self.create_publisher(Path , '/path_direction', 10)
        self.bridge = CvBridge()

    def image_callback(self,msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width = frame.shape[:2]

            lane_mask = ast.lane_costmap(frame)

            path = ast.a_star_search(lane_mask,start,destination) # how to get start and distination 
            if path:
                    self.publish_path(path, msg.header)
            out_msg = self.bridge.cv2_to_imgmsg(lane_mask, encoding='mono8')
            out_msg.header = msg.header
            self.lane_pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Error is {e}")

    def publish_path(self, path_coords, header):
        path_msg = Path()
        path_msg.header = header
        path_msg.header.frame_id = "map" 

        for (x, y) in path_coords:
            pose = PoseStamped()
            pose.header = header
            
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
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
