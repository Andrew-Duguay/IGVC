import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('stereo_camera_publisher')

        self.pub_left = self.create_publisher(Image, '/left_camera_raw', 10)
        self.pub_right = self.create_publisher(Image, '/right_camera_raw', 10)

        self.bridge = CvBridge()
        self.cap_left = cv2.VideoCapture(0) # connect left then right
        self.cap_right = cv2.VideoCapture(1) 

        if not self.cap_left.isOpened():
            self.get_logger().error("Left Camera Rrror ")
        if not self.cap_right.isOpened():
            self.get_logger().error("Right Camera Error")

        timer_period = 0.033  #FPS

        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info("Stereo Camera Node has been started.")

    def timer_callback(self):
        ret_l, frame_l = self.cap_left.read()
        ret_r, frame_r = self.cap_right.read()

        current_time = self.get_clock().now().to_msg()

        if ret_l:
            try:
                msg_l = self.bridge.cv2_to_imgmsg(frame_l, encoding="bgr8")
                msg_l.header.stamp = current_time
                msg_l.header.frame_id = "left_camera_frame"
                self.pub_left.publish(msg_l)
            except Exception as e:
                self.get_logger().error(f"Error publishing left image: {e}")

        if ret_r:
            try:
                msg_r = self.bridge.cv2_to_imgmsg(frame_r, encoding="bgr8")
                msg_r.header.stamp = current_time
                msg_r.header.frame_id = "right_camera_frame"
                self.pub_right.publish(msg_r)
            except Exception as e:
                self.get_logger().error(f"Error publishing right image: {e}")

    def destroy_node(self):
        self.cap_left.release()
        self.cap_right.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraPublisher()

    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()