#!/usr/bin/env python3
# Republish the robot's ground-truth world odometry (nav_msgs/Odometry
# published by the OdometryPublisher plugin with odom_frame=world) as a
# PoseStamped on /world_pose.
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class WorldPoseRelay(Node):
    def __init__(self):
        super().__init__('world_pose_relay')
        self.declare_parameter('input_topic', '/world_odom')
        self.declare_parameter('output_topic', '/world_pose')
        self.declare_parameter('frame_id', 'world')
        self.frame_id = self.get_parameter('frame_id').value
        self.create_subscription(
            Odometry, self.get_parameter('input_topic').value, self._cb, 10)
        self.pub = self.create_publisher(
            PoseStamped, self.get_parameter('output_topic').value, 10)

    def _cb(self, msg: Odometry):
        ps = PoseStamped()
        ps.header = msg.header
        ps.header.frame_id = self.frame_id
        ps.pose = msg.pose.pose
        self.pub.publish(ps)


def main():
    rclpy.init()
    try:
        rclpy.spin(WorldPoseRelay())
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
