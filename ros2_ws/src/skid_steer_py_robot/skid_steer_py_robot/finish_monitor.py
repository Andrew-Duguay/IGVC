import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class FinishMonitor(Node):

    def __init__(self):
        super().__init__('finish_monitor')

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Subscribe to movement commands to start timer
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.timer_started = False
        self.finished = False
        self.start_time = None

        # Define finish line (adjust as needed)
        self.finish_x = 4.0

        self.get_logger().info("Waiting for movement to start timer...")

    def cmd_callback(self, msg):
        if not self.timer_started:
            if abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001:
                self.start_time = self.get_clock().now()
                self.timer_started = True
                self.get_logger().info("⏱ Timer started.")

    def odom_callback(self, msg):
        if self.timer_started and not self.finished:

            x = msg.pose.pose.position.x

            if x >= self.finish_x:
                end_time = self.get_clock().now()
                duration = (end_time - self.start_time).nanoseconds / 1e9

                self.get_logger().info("🏁 COURSE FINISHED!")
                self.get_logger().info(f"⏱ Total time: {duration:.3f} seconds")

                self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = FinishMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
