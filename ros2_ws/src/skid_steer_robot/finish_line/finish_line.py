#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class Point:
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)

    def set_point(self, x, y):
        self.x = float(x)
        self.y = float(y)

class Line:
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2

    def ccw(self, A, B, C):
        return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x)

    def has_intersection(self, other_line):
        A = self.p1
        B = self.p2
        C = other_line.p1
        D = other_line.p2
        
        straddles_AB = self.ccw(A, B, C) != self.ccw(A, B, D)
        straddles_CD = self.ccw(C, D, A) != self.ccw(C, D, B)
        return straddles_AB and straddles_CD    # If both straddle each other, they intersect

class FinishLine(Node):

    def __init__(self):
        super().__init__('finish_line_node')

        # 1. Declare parameters with default values
        self.declare_parameter('x1', -10.0)
        self.declare_parameter('y1', -10.0)
        self.declare_parameter('x2', 10.0)
        self.declare_parameter('y2', -10.0)
        
        # 2. Get the values from the launch file
        x1 = self.get_parameter('x1').get_parameter_value().double_value
        y1 = self.get_parameter('y1').get_parameter_value().double_value
        x2 = self.get_parameter('x2').get_parameter_value().double_value
        y2 = self.get_parameter('y2').get_parameter_value().double_value

        self.finish_line = Line(Point(x1, y1), Point(x2, y2))
        self.current_position = Point(-1000, -1000)
        self.last_position = Point(-1000, -1000)

        # odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # needs movement to start timer
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.timer_started = False
        self.finished = False
        self.start_time = None

        

        self.get_logger().info("Finish line node started.")

    def cmd_callback(self, msg):
        if not self.timer_started:
            if abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001:
                self.start_time = self.get_clock().now()
                self.timer_started = True
                self.get_logger().info("⏱ Timer started.")

    def odom_callback(self, msg):
        if self.timer_started and not self.finished:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.update_current_position(x, y)
            if self.finish_line_crossed():
                end_time = self.get_clock().now()
                duration = (end_time - self.start_time).nanoseconds / 1e9
                self.get_logger().info(f"[SUCCESS] Course completed.")
                self.finished = True
    
    def finish_line_crossed(self):        
        p1 = self.last_position
        p2 = self.current_position
        traversal_line = Line(p1,p2)
        finish = self.finish_line
        return finish.has_intersection(traversal_line)

    def update_current_position(self, x, y):
        if(self.current_position.x == -1000 or self.current_position.y == -1000):
            self.current_position.set_point(x,y)
            self.last_position.set_point(x,y)
        else:
            self.last_position.set_point(self.current_position.x, self.current_position.y)
            self.current_position.set_point(x, y)

def main(args=None):
    rclpy.init(args=args)
    node = FinishLine()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()