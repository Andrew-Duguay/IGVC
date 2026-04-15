#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import Contacts 
from rclpy.qos import qos_profile_sensor_data 
import math

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
        return straddles_AB and straddles_CD

class PassOrFailNode(Node):
    def __init__(self):
        super().__init__('pass_or_fail_node')

        self.declare_parameter('x1', -10.0)
        self.declare_parameter('y1', -10.0)
        self.declare_parameter('x2', 10.0)
        self.declare_parameter('y2', -10.0)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('timeout', 30.0)
        
        x1 = self.get_parameter('x1').get_parameter_value().double_value
        y1 = self.get_parameter('y1').get_parameter_value().double_value
        x2 = self.get_parameter('x2').get_parameter_value().double_value
        y2 = self.get_parameter('y2').get_parameter_value().double_value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.start_yaw = self.get_parameter('start_yaw').value
        self.timeout = self.get_parameter('timeout').value
        self.pass_or_fail = Line(Point(x1, y1), Point(x2, y2))
        self.current_position = Point(-1000, -1000)
        self.last_position = Point(-1000, -1000)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_sensor_data 
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # Subscriptions to contact sensors
        self.chassis_sub = self.create_subscription(
            Contacts,
            '/chassis_sensor', 
            self.collision_callback,
            qos_profile_sensor_data
        )
        self.left_wheel_sub = self.create_subscription(
            Contacts,
            '/left_wheel_sensor', 
            self.collision_callback,
            qos_profile_sensor_data
        )
        self.right_wheel_sub = self.create_subscription(
            Contacts,
            '/right_wheel_sensor', 
            self.collision_callback,
            qos_profile_sensor_data
        )
        self.caster_sub = self.create_subscription(
            Contacts,
            '/caster_sensor', 
            self.collision_callback,
            qos_profile_sensor_data
        )

        self.finished = False
        self.started = False
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("[INFO] pass_or_fail node for testing of robot started. Waiting for robot movement.")

    def cmd_callback(self, msg):
        if not self.started:
            if abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001:
                self.started = True
                self.get_logger().info("[INFO] Test started.")

    def timer_callback(self):
        if self.finished:
            return
        current_time = self.get_clock().now()
        if current_time.nanoseconds == 0:
            return
        if self.start_time is None:
            self.start_time = current_time
            self.get_logger().info("Simulation clock received! Referee has started.")
            return
        elapsed_duration = current_time - self.start_time
        elapsed_seconds = elapsed_duration.nanoseconds / 1e9
        if elapsed_seconds > self.timeout:
            self.get_logger().error(f"[FAILURE] Timeout at {elapsed_seconds}s")
            self.finished = True

    def collision_callback(self, msg):
        # Ignore collisions before the robot starts moving, or after the test is over
        if self.finished or not self.started:
            return

        if not msg.contacts:
            return

        # Get the collision name
        hit_object = msg.contacts[0].collision2.name.lower()
        
        # Ignore these collisions
        if "ramp" in hit_object or "floor" in hit_object:
            return
            
        # Failure condition
        self.get_logger().error(f"[FAILURE] Collision type: {hit_object}")
        self.finished = True

    def odom_callback(self, msg):
        if self.started and not self.finished:
            local_x = msg.pose.pose.position.x
            local_y = msg.pose.pose.position.y
            
            # Convert to Absolute World Coordinates using 2D Rotation Matrix
            abs_x = self.start_x + (local_x * math.cos(self.start_yaw) - local_y * math.sin(self.start_yaw))
            abs_y = self.start_y + (local_x * math.sin(self.start_yaw) + local_y * math.cos(self.start_yaw))

            self.update_current_position(abs_x, abs_y)
            if self.pass_or_fail_crossed():
                self.get_logger().info(f"[SUCCESS] Course completed!")
                self.finished = True
    
    def pass_or_fail_crossed(self):        
        p1 = self.last_position
        p2 = self.current_position
        traversal_line = Line(p1,p2)
        return self.pass_or_fail.has_intersection(traversal_line)

    def update_current_position(self, x, y):
        if(self.current_position.x == -1000 or self.current_position.y == -1000):
            self.current_position.set_point(x,y)
            self.last_position.set_point(x,y)
        else:
            self.last_position.set_point(self.current_position.x, self.current_position.y)
            self.current_position.set_point(x, y)

def main(args=None):
    rclpy.init(args=args)
    node = PassOrFailNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()