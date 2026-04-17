#!/usr/bin/env python3
# Bridge from robot_localization to the `/world_pose` contract that the
# autonomy stack (lane_follower_node, etc.) expects.
#
# The global EKF publishes nav_msgs/Odometry on /odometry/filtered_map
# with pose in the `map` frame. Our autonomy wants a PoseStamped with
# frame_id='world' on /world_pose. This node re-emits exactly that,
# with the header stamp passed through.
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class WorldPoseFromEKF(Node):
    def __init__(self):
        super().__init__('world_pose_from_ekf')
        self.declare_parameter('input_topic', '/odometry/filtered_map')
        self.declare_parameter('output_topic', '/world_pose')
        self.declare_parameter('frame_id', 'world')
        self._frame_id = self.get_parameter('frame_id').value
        self.create_subscription(
            Odometry, self.get_parameter('input_topic').value, self._cb, 10)
        self._pub = self.create_publisher(
            PoseStamped, self.get_parameter('output_topic').value, 10)

    def _cb(self, msg: Odometry):
        ps = PoseStamped()
        ps.header = msg.header
        ps.header.frame_id = self._frame_id
        ps.pose = msg.pose.pose
        self._pub.publish(ps)


def main():
    rclpy.init()
    try:
        rclpy.spin(WorldPoseFromEKF())
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
