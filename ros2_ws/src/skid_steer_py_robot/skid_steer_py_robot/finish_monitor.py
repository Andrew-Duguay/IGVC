import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ContactsState


class FinishMonitor(Node):

    def __init__(self):
        super().__init__('finish_monitor')

        self.subscription = self.create_subscription(
            ContactsState,
            '/course/bumper_states',
            self.listener_callback,
            10
        )

        self.finished = False

    def listener_callback(self, msg):
        if not self.finished and len(msg.states) > 0:
            self.get_logger().info("🏁 COURSE FINISHED!")
            self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = FinishMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
