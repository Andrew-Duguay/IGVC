#!/usr/bin/env python3
"""Bridge node: converts /cmd_vel (Twist) to /joy (sensor_msgs/Joy).

Translates autonomy velocity commands into the Joy message format that
Little Blue's existing motor controller subscriber expects.

Also publishes RViz markers showing joystick state:
- Green bar: throttle level
- Blue arrow: steering direction
- Red bar: reverse indicator
- Text overlay: M0/M1 duty cycles

The Pi-side subscriber uses:
  axes[0] = steering (left stick X, -1 to 1, negative = left)
  axes[5] = throttle (right trigger, 1.0 = released, -1.0 = fully pressed)
             The Pi computes: a = -(axes[5] - 1.0)/2 -> 0 to 1
  buttons[1] = reversing flag (B button, 0 or 1)

The Pi's mixing equations:
  a = throttle (0-1), b = -steering
  M0 = (-(a/4)*(b + sqrt(b*b))*b) + ((a+1)/2)
  M1 = (-(a/4)*(b - sqrt(b*b))*b) + ((a+1)/2)
"""

import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, Image
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge


class CmdVelToJoyNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_joy_node')

        # Parameters
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('joy_topic', '/joy')

        self._max_lin = self.get_parameter('max_linear_speed').value
        self._max_ang = self.get_parameter('max_angular_speed').value

        # Latest cmd_vel
        self._linear_x = 0.0
        self._angular_z = 0.0
        self._last_cmd_time = self.get_clock().now()

        # Subscribe to cmd_vel
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.create_subscription(Twist, cmd_topic, self._cmd_vel_cb, 10)

        self._bridge = CvBridge()

        # Publishers
        joy_topic = self.get_parameter('joy_topic').value
        self.joy_pub = self.create_publisher(Joy, joy_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/joy_viz', 10)
        self.dashboard_pub = self.create_publisher(Image, '/joy_dashboard', 10)

        # Timer
        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f'CmdVel->Joy bridge started (max_lin={self._max_lin}, max_ang={self._max_ang})')

    def _cmd_vel_cb(self, msg):
        self._linear_x = msg.linear.x
        self._angular_z = msg.angular.z
        self._last_cmd_time = self.get_clock().now()

    def _compute_motors(self, throttle, steering):
        """Replicate Pi's mixing equation to show predicted motor duty cycles."""
        a = throttle
        b = steering  # already negated in _publish
        m0 = (-(a / 4.0) * (b + math.sqrt(b * b)) * b) + ((a + 1.0) / 2.0)
        m1 = (-(a / 4.0) * (b - math.sqrt(b * b)) * b) + ((a + 1.0) / 2.0)
        return int(m0 * 100), int(m1 * 100)

    def _publish(self):
        # Safety timeout
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if age > 0.5:
            self._linear_x = 0.0
            self._angular_z = 0.0

        speed = self._linear_x
        turn = self._angular_z

        reversing = 0
        if speed < 0:
            reversing = 1
            speed = -speed

        throttle = min(speed / self._max_lin, 1.0)
        steering = max(-1.0, min(1.0, -turn / self._max_ang))

        # Joy axes format
        axes_5 = 1.0 - 2.0 * throttle

        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        joy.header.frame_id = 'joy'
        joy.axes = [0.0] * 8
        joy.buttons = [0] * 21
        joy.axes[0] = steering
        joy.axes[5] = axes_5
        joy.buttons[1] = reversing
        self.joy_pub.publish(joy)

        # Compute predicted motor outputs
        b = -steering  # Pi negates axes[0]
        m0, m1 = self._compute_motors(throttle, b)

        # Publish RViz markers + dashboard image
        self._publish_viz(throttle, steering, reversing, m0, m1)
        self._publish_dashboard(throttle, steering, reversing, m0, m1)

    def _publish_viz(self, throttle, steering, reversing, m0, m1):
        markers = MarkerArray()
        # Position the HUD above and behind the robot
        hud_x = -0.5
        hud_z = 0.8

        # Throttle bar (green, height = throttle level)
        m = Marker()
        m.header.frame_id = 'base_footprint'
        m.ns = 'joy_hud'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        bar_h = max(throttle * 0.5, 0.01)
        m.pose.position.x = float(hud_x)
        m.pose.position.y = 0.3
        m.pose.position.z = float(hud_z + bar_h / 2.0)
        m.scale.x = 0.05
        m.scale.y = 0.1
        m.scale.z = float(bar_h)
        if reversing:
            m.color.r = 1.0
            m.color.g = 0.2
        else:
            m.color.g = 1.0
        m.color.a = 0.8
        m.lifetime.sec = 0
        m.lifetime.nanosec = 200000000
        markers.markers.append(m)

        # Steering arrow (blue, points left/right)
        m2 = Marker()
        m2.header.frame_id = 'base_footprint'
        m2.ns = 'joy_hud'
        m2.id = 1
        m2.type = Marker.ARROW
        m2.action = Marker.ADD
        m2.pose.position.x = float(hud_x)
        m2.pose.position.y = 0.0
        m2.pose.position.z = float(hud_z)
        # Arrow points in the joystick stick direction (what's actually sent)
        yaw = math.pi / 2.0 * steering
        m2.pose.orientation.z = math.sin(yaw / 2.0)
        m2.pose.orientation.w = math.cos(yaw / 2.0)
        arrow_len = 0.1 + abs(steering) * 0.3
        m2.scale.x = float(arrow_len)  # length
        m2.scale.y = 0.04              # width
        m2.scale.z = 0.04              # height
        m2.color.b = 1.0
        m2.color.a = 0.8
        m2.lifetime.sec = 0
        m2.lifetime.nanosec = 200000000
        markers.markers.append(m2)

        # Full controller state text
        m3 = Marker()
        m3.header.frame_id = 'base_footprint'
        m3.ns = 'joy_hud'
        m3.id = 2
        m3.type = Marker.TEXT_VIEW_FACING
        m3.action = Marker.ADD
        m3.pose.position.x = float(hud_x)
        m3.pose.position.y = -0.3
        m3.pose.position.z = float(hud_z + 0.25)
        m3.scale.z = 0.06  # text height

        # Build full state display
        btn_names = ['A', 'B', 'X', 'Y', '?', '?', '?', '?', '?',
                     'LB', 'RB', 'Up', 'Down', 'Left', 'Right',
                     '?', '?', '?', '?', '?', '?']
        # Get the joy message we just published
        joy_axes = [steering, 0.0, 0.0, 0.0, 0.0, 1.0 - 2.0 * throttle, 0.0, 0.0]
        joy_buttons = [0] * 21
        joy_buttons[1] = reversing

        pressed = [btn_names[i] for i in range(min(len(btn_names), 21)) if joy_buttons[i]]
        btn_str = ', '.join(pressed) if pressed else 'none'

        lines = [
            f'--- CONTROLLER ---',
            f'M0={m0}%  M1={m1}%',
            f'Throttle: {throttle:.0%}',
            f'Steering: {steering:+.2f}',
            f'Reverse:  {"YES" if reversing else "no"}',
            f'Buttons:  [{btn_str}]',
            f'LStick X: {steering:+.2f}',
            f'RTrigger: {throttle:.2f}',
        ]
        m3.text = '\n'.join(lines)
        m3.color.r = 1.0
        m3.color.g = 1.0
        m3.color.b = 1.0
        m3.color.a = 0.9
        m3.lifetime.sec = 0
        m3.lifetime.nanosec = 200000000
        markers.markers.append(m3)

        self.marker_pub.publish(markers)


    def _publish_dashboard(self, throttle, steering, reversing, m0, m1):
        """Render controller state as an image for RViz Image panel."""
        W, H = 320, 240
        img = np.zeros((H, W, 3), dtype=np.uint8)
        img[:] = (40, 40, 40)  # dark background

        # Colors
        GREEN = (0, 200, 0)
        RED = (0, 0, 200)
        BLUE = (200, 100, 0)
        WHITE = (220, 220, 220)
        GRAY = (100, 100, 100)
        YELLOW = (0, 200, 200)

        font = cv2.FONT_HERSHEY_SIMPLEX

        # Title
        cv2.putText(img, 'CONTROLLER', (90, 20), font, 0.5, WHITE, 1)

        # ── Left stick (steering) ──
        stick_cx, stick_cy = 70, 80
        stick_r = 35
        cv2.circle(img, (stick_cx, stick_cy), stick_r, GRAY, 1)
        cv2.line(img, (stick_cx, stick_cy - stick_r), (stick_cx, stick_cy + stick_r), GRAY, 1)
        cv2.line(img, (stick_cx - stick_r, stick_cy), (stick_cx + stick_r, stick_cy), GRAY, 1)
        # Stick position
        sx = int(stick_cx + steering * stick_r)
        cv2.circle(img, (sx, stick_cy), 8, BLUE, -1)
        cv2.putText(img, 'STEER', (45, 125), font, 0.35, WHITE, 1)
        cv2.putText(img, f'{steering:+.2f}', (45, 140), font, 0.35, YELLOW, 1)

        # ── Throttle bar ──
        bar_x, bar_y = 150, 40
        bar_w, bar_h = 30, 80
        cv2.rectangle(img, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), GRAY, 1)
        fill_h = int(throttle * bar_h)
        color = RED if reversing else GREEN
        if fill_h > 0:
            cv2.rectangle(img, (bar_x + 1, bar_y + bar_h - fill_h),
                          (bar_x + bar_w - 1, bar_y + bar_h - 1), color, -1)
        cv2.putText(img, 'THROT', (145, 135), font, 0.35, WHITE, 1)
        cv2.putText(img, f'{throttle:.0%}', (150, 150), font, 0.35, YELLOW, 1)

        # ── Buttons ──
        btn_info = [
            ('A', 0, (0, 180, 0)),
            ('B', 1, (0, 0, 200)),
            ('X', 2, (200, 100, 0)),
            ('Y', 3, (0, 200, 200)),
            ('LB', 9, (150, 150, 150)),
            ('RB', 10, (150, 150, 150)),
        ]
        bx_start = 210
        by_start = 45
        joy_buttons = [0] * 21
        joy_buttons[1] = reversing
        for i, (name, idx, color_on) in enumerate(btn_info):
            bx = bx_start + (i % 2) * 50
            by = by_start + (i // 2) * 30
            pressed = joy_buttons[idx] if idx < len(joy_buttons) else 0
            c = color_on if pressed else GRAY
            cv2.circle(img, (bx, by), 10, c, -1 if pressed else 1)
            cv2.putText(img, name, (bx - 6, by + 4), font, 0.3, WHITE, 1)

        # ── Reverse indicator ──
        if reversing:
            cv2.putText(img, 'REVERSE', (220, 140), font, 0.4, RED, 1)

        # ── Motor outputs ──
        cv2.putText(img, 'MOTORS', (20, 175), font, 0.4, WHITE, 1)
        # M0 bar
        m0_w = int(min(m0, 100) / 100.0 * 120)
        cv2.rectangle(img, (20, 185), (140, 200), GRAY, 1)
        if m0_w > 0:
            cv2.rectangle(img, (20, 185), (20 + m0_w, 200), GREEN, -1)
        cv2.putText(img, f'M0={m0}%', (145, 197), font, 0.35, WHITE, 1)
        # M1 bar
        m1_w = int(min(m1, 100) / 100.0 * 120)
        cv2.rectangle(img, (20, 205), (140, 220), GRAY, 1)
        if m1_w > 0:
            cv2.rectangle(img, (20, 205), (20 + m1_w, 220), GREEN, -1)
        cv2.putText(img, f'M1={m1}%', (145, 217), font, 0.35, WHITE, 1)

        # Publish
        msg = self._bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.dashboard_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
