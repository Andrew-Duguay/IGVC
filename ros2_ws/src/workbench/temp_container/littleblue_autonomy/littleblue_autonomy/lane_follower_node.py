#!/usr/bin/env python3
"""Lane-following autonomy node with reactive obstacle avoidance."""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from sensor_msgs_py import point_cloud2


class LaneFollowerNode(Node):
    # States
    STOPPED = 'STOPPED'
    LANE_FOLLOWING = 'LANE_FOLLOWING'
    AVOIDING = 'AVOIDING'

    def __init__(self):
        super().__init__('lane_follower_node')

        # --- Parameters ---
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('min_x', 0.5)
        self.declare_parameter('max_x', 5.0)
        self.declare_parameter('bin_width', 0.5)
        self.declare_parameter('min_points_per_bin', 3)
        self.declare_parameter('lane_width', 3.0)
        self.declare_parameter('min_centerline_points', 2)
        self.declare_parameter('cruise_speed', 0.5)
        self.declare_parameter('lookahead_min', 1.0)
        self.declare_parameter('lookahead_max', 3.0)
        self.declare_parameter('speed_gain', 0.8)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('curve_slowdown_curvature', 0.5)
        self.declare_parameter('curve_slowdown_factor', 0.6)
        self.declare_parameter('obstacle_half_angle', 35.0)
        self.declare_parameter('obstacle_min_range', 0.8)
        self.declare_parameter('obstacle_detect_range', 2.5)
        self.declare_parameter('obstacle_min_hits', 3)
        self.declare_parameter('no_lane_timeout', 3.0)
        # Centerline smoothing
        self.declare_parameter('centerline_spread_threshold', 1.0)
        # Reactive avoidance params
        self.declare_parameter('max_evasion_offset', 1.5)
        self.declare_parameter('gap_clearance_threshold', 3.0)
        self.declare_parameter('evasion_min_distance', 4.0)
        self.declare_parameter('evasion_speed_factor', 0.6)
        # Ramp obstacle suppression
        self.declare_parameter('ramp_waypoints', [0.0])
        self.declare_parameter('ramp_suppression_radius', 3.0)

        # --- State ---
        self.state = self.STOPPED
        self.lane_points = None
        self.lane_stamp = None
        self.current_vx = 0.0
        self.last_scan = None
        self.last_lane_time = self.get_clock().now()
        self.centerline = []

        # Centerline smoothing state
        self.prev_centerline = []

        # Reactive evasion state
        self.evasion_offset = 0.0       # lateral offset (+left, -right)
        self.evasion_distance = 0.0     # distance traveled since evasion start
        self.evasion_min_dist = 0.0     # minimum distance to travel before clearing

        # Robot world-frame position (from /world_pose ground truth)
        self.world_x = 0.0
        self.world_y = 0.0

        # Distance tracking via odom integration
        self.cumulative_distance = 0.0
        self.last_odom_time = None

        # --- Subscribers ---
        self.create_subscription(
            PointCloud2, '/lane_points', self._lane_cb, 10)
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(
            PoseStamped, '/world_pose', self._world_pose_cb, 10)

        # --- Publishers ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.centerline_pub = self.create_publisher(Path, '/autonomy/centerline', 10)
        self.lookahead_pub = self.create_publisher(PointStamped, '/autonomy/lookahead', 10)
        self.state_pub = self.create_publisher(String, '/autonomy/state', 10)

        # --- Control timer ---
        rate = self.get_parameter('control_rate').value
        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info('Lane follower node started')

    # ─── parameter helpers ───────────────────────────────────────────
    def _p(self, name):
        return self.get_parameter(name).value

    # ─── callbacks ───────────────────────────────────────────────────
    def _lane_cb(self, msg: PointCloud2):
        pts = []
        for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            pts.append((p[0], p[1]))
        self.lane_points = pts
        self.lane_stamp = msg.header.stamp
        self.last_lane_time = self.get_clock().now()

    def _scan_cb(self, msg: LaserScan):
        self.last_scan = msg

    def _world_pose_cb(self, msg: PoseStamped):
        self.world_x = msg.pose.position.x
        self.world_y = msg.pose.position.y

    def _odom_cb(self, msg: Odometry):
        self.current_vx = msg.twist.twist.linear.x
        # Integrate distance
        now = self.get_clock().now()
        if self.last_odom_time is not None:
            dt = (now - self.last_odom_time).nanoseconds / 1e9
            self.cumulative_distance += abs(self.current_vx) * dt
        self.last_odom_time = now

    # ─── centerline computation ──────────────────────────────────────
    def _compute_centerline(self):
        """Bin lane points by X, cluster into two lines, compute center."""
        if not self.lane_points:
            return []

        min_x = self._p('min_x')
        max_x = self._p('max_x')
        bin_w = self._p('bin_width')
        min_pts = self._p('min_points_per_bin')
        half_lane = self._p('lane_width') / 2.0
        spread_thresh = self._p('centerline_spread_threshold')

        n_bins = int((max_x - min_x) / bin_w)
        bins = [[] for _ in range(n_bins)]

        for x, y in self.lane_points:
            if x < min_x or x >= max_x:
                continue
            idx = int((x - min_x) / bin_w)
            idx = min(idx, n_bins - 1)
            bins[idx].append(y)

        centerline = []
        for i in range(n_bins):
            x_center = min_x + (i + 0.5) * bin_w
            ys = sorted(bins[i])

            if len(ys) < min_pts:
                continue

            two_lines = False
            if len(ys) >= 2 * min_pts:
                max_gap = 0.0
                split_idx = len(ys) // 2
                for j in range(min_pts - 1, len(ys) - min_pts):
                    gap = ys[j + 1] - ys[j]
                    if gap > max_gap:
                        max_gap = gap
                        split_idx = j + 1

                if max_gap > 0.3:
                    line_a = ys[:split_idx]
                    line_b = ys[split_idx:]
                    y_center = (np.mean(line_a) + np.mean(line_b)) / 2.0
                    two_lines = True

            if not two_lines:
                spread = ys[-1] - ys[0]
                if spread > spread_thresh:
                    # Both lines present but gap detection failed — use range midpoint
                    y_center = (ys[0] + ys[-1]) / 2.0
                else:
                    # Truly single line — offset toward track center
                    mean_y = np.mean(ys)
                    if mean_y > 0:
                        y_center = mean_y - half_lane
                    else:
                        y_center = mean_y + half_lane

            centerline.append((x_center, y_center))

        # Smooth with previous centerline (exponential moving average)
        if self.prev_centerline and len(centerline) == len(self.prev_centerline):
            alpha = 0.4  # weight for new data
            centerline = [
                (x, alpha * y + (1.0 - alpha) * py)
                for (x, y), (_, py) in zip(centerline, self.prev_centerline)
            ]
        self.prev_centerline = centerline

        return centerline

    # ─── pure pursuit ────────────────────────────────────────────────
    def _pure_pursuit(self, centerline, speed_factor=1.0):
        """Compute cmd_vel from centerline using pure pursuit."""
        if len(centerline) < self._p('min_centerline_points'):
            return None

        L = self._p('lookahead_min') + self._p('speed_gain') * abs(self.current_vx)
        L = max(self._p('lookahead_min'), min(L, self._p('lookahead_max')))

        lookahead_pt = None
        for cx, cy in centerline:
            dist = math.sqrt(cx * cx + cy * cy)
            if dist >= L:
                lookahead_pt = (cx, cy)
                break

        if lookahead_pt is None:
            lookahead_pt = centerline[-1]

        gx, gy = lookahead_pt
        L_actual = math.sqrt(gx * gx + gy * gy)

        if L_actual < 0.01:
            return None

        # Publish lookahead marker
        pt_msg = PointStamped()
        pt_msg.header.stamp = self.get_clock().now().to_msg()
        pt_msg.header.frame_id = 'base_footprint'
        pt_msg.point.x = gx
        pt_msg.point.y = gy
        self.lookahead_pub.publish(pt_msg)

        curvature = 2.0 * gy / (L_actual * L_actual)

        linear = self._p('cruise_speed') * speed_factor
        angular = linear * curvature
        angular = max(-self._p('max_angular_vel'),
                      min(angular, self._p('max_angular_vel')))

        if abs(curvature) > self._p('curve_slowdown_curvature'):
            linear *= self._p('curve_slowdown_factor')

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        return cmd

    # ─── obstacle detection ──────────────────────────────────────────
    def _check_obstacle(self, min_range_override=None, half_angle_deg_override=None):
        """Check for obstacles in a forward corridor using /scan."""
        if self.last_scan is None:
            return False

        scan = self.last_scan
        half_angle = math.radians(half_angle_deg_override if half_angle_deg_override is not None else self._p('obstacle_half_angle'))
        min_range = min_range_override if min_range_override is not None else self._p('obstacle_min_range')
        detect_range = self._p('obstacle_detect_range')
        min_hits = self._p('obstacle_min_hits')

        hits = 0
        for i, r in enumerate(scan.ranges):
            if math.isinf(r) or math.isnan(r) or r < scan.range_min:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            if abs(angle) <= half_angle and min_range <= r <= detect_range:
                hits += 1

        return hits >= min_hits

    def _measure_side_clearance(self, angle_min_deg, angle_max_deg):
        """Measure average clearance in a scan angular range."""
        if self.last_scan is None:
            return float('inf')

        scan = self.last_scan
        a_min = math.radians(angle_min_deg)
        a_max = math.radians(angle_max_deg)
        total = 0.0
        count = 0

        for i, r in enumerate(scan.ranges):
            if math.isinf(r) or math.isnan(r) or r < scan.range_min:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            if a_min <= angle <= a_max:
                total += r
                count += 1

        return total / count if count > 0 else float('inf')

    def _find_best_gap(self):
        """Find the widest clear angular gap in the forward hemisphere.

        Returns (offset, angle_deg) or None if no clear gap found.
        """
        if self.last_scan is None:
            return None

        scan = self.last_scan
        sector_size = math.radians(10)  # 10° per sector
        n_sectors = 18  # -90° to +90°
        sector_min = [float('inf')] * n_sectors

        # Build sector map — minimum range per 10° slice
        for i, r in enumerate(scan.ranges):
            if math.isinf(r) or math.isnan(r) or r < scan.range_min:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            if abs(angle) > math.pi / 2:
                continue
            idx = int((angle + math.pi / 2) / sector_size)
            idx = min(max(idx, 0), n_sectors - 1)
            sector_min[idx] = min(sector_min[idx], r)

        # Find widest contiguous clear gap
        threshold = self._p('gap_clearance_threshold')
        best_start, best_len = 0, 0
        cur_start, cur_len = 0, 0

        for i in range(n_sectors):
            if sector_min[i] > threshold:
                if cur_len == 0:
                    cur_start = i
                cur_len += 1
            else:
                if cur_len > best_len:
                    best_start, best_len = cur_start, cur_len
                cur_len = 0
        if cur_len > best_len:
            best_start, best_len = cur_start, cur_len

        if best_len == 0:
            return None

        # Center angle of the widest gap
        center_idx = best_start + best_len / 2.0
        center_angle = -math.pi / 2 + center_idx * sector_size

        # Convert to lateral offset at lookahead distance
        lookahead = self._p('lookahead_min')
        offset = lookahead * math.tan(center_angle)
        max_off = self._p('max_evasion_offset')
        offset = max(-max_off, min(offset, max_off))

        return offset, math.degrees(center_angle)

    def _pick_evasion_offset(self, reason):
        """Compute evasion offset from gap-finding with side-clearance fallback.

        Returns the chosen offset. Also logs the decision.
        """
        gap = self._find_best_gap()
        if gap is not None:
            offset, angle = gap
            offset = max(-self._p('max_evasion_offset'),
                         min(offset, self._p('max_evasion_offset')))
            direction = 'left' if offset > 0 else 'right'
            self.get_logger().info(
                f'{reason} Evading {direction} '
                f'(gap at {angle:.0f}\u00b0, offset {offset:.2f}m)')
            return offset

        # No clear gap — pick side with more clearance
        left_clear = self._measure_side_clearance(30, 90)
        right_clear = self._measure_side_clearance(-90, -30)
        if left_clear >= right_clear:
            offset = self._p('max_evasion_offset')
        else:
            offset = -self._p('max_evasion_offset')
        direction = 'left' if offset > 0 else 'right'
        self.get_logger().info(
            f'{reason} No clear gap, evading {direction} '
            f'(clearance L:{left_clear:.1f} R:{right_clear:.1f})')
        return offset

    # ─── ramp suppression ──────────────────────────────────────────
    def _near_ramp(self):
        """Return True if robot is within suppression radius of any ramp waypoint."""
        waypoints = self._p('ramp_waypoints')
        if not waypoints or len(waypoints) < 2:
            return False
        radius = self._p('ramp_suppression_radius')
        for i in range(0, len(waypoints) - 1, 2):
            wx, wy = waypoints[i], waypoints[i + 1]
            dist = math.sqrt((self.world_x - wx) ** 2 + (self.world_y - wy) ** 2)
            if dist <= radius:
                return True
        return False

    # ─── state machine ───────────────────────────────────────────────
    def _transition(self, new_state):
        if new_state != self.state:
            self.get_logger().info(f'State: {self.state} -> {new_state}')
            self.state = new_state

    def _publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def _publish_centerline(self, centerline):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'base_footprint'
        for cx, cy in centerline:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = cx
            pose.pose.position.y = cy
            path.poses.append(pose)
        self.centerline_pub.publish(path)

    def _stop_robot(self):
        self.cmd_pub.publish(Twist())

    # ─── main control loop ───────────────────────────────────────────
    def _control_loop(self):
        self._publish_state()

        self.centerline = self._compute_centerline()
        if self.centerline:
            self._publish_centerline(self.centerline)

        if self.state == self.STOPPED:
            self._handle_stopped()
        elif self.state == self.LANE_FOLLOWING:
            self._handle_lane_following()
        elif self.state == self.AVOIDING:
            self._handle_avoiding()

    def _handle_stopped(self):
        if self.centerline and len(self.centerline) >= self._p('min_centerline_points'):
            self._transition(self.LANE_FOLLOWING)

    def _handle_lane_following(self):
        # Check for no-lane timeout
        elapsed = (self.get_clock().now() - self.last_lane_time).nanoseconds / 1e9
        if elapsed > self._p('no_lane_timeout'):
            self.get_logger().warn('No lane data, stopping')
            self._transition(self.STOPPED)
            self._stop_robot()
            return

        # Check for obstacles (suppressed near ramp waypoints)
        if not self._near_ramp() and self._check_obstacle():
            self.evasion_offset = self._pick_evasion_offset('Obstacle detected!')
            self.evasion_distance = self.cumulative_distance
            self.evasion_min_dist = self._p('evasion_min_distance')
            self._transition(self.AVOIDING)
            return

        # Normal pure pursuit
        cmd = self._pure_pursuit(self.centerline)
        if cmd is not None:
            self.cmd_pub.publish(cmd)
        else:
            self._stop_robot()

    def _handle_avoiding(self):
        # Check for no-lane timeout
        elapsed = (self.get_clock().now() - self.last_lane_time).nanoseconds / 1e9
        if elapsed > self._p('no_lane_timeout'):
            self.get_logger().warn('No lane data during avoidance, stopping')
            self.evasion_offset = 0.0
            self._transition(self.STOPPED)
            self._stop_robot()
            return

        dist_traveled = self.cumulative_distance - self.evasion_distance

        # Use wide-angle (±70°) close-range (0.15m) detection during evasion
        # so we don't lose sight of obstacles that are offset or very close
        obstacle_ahead = self._check_obstacle(
            min_range_override=0.15, half_angle_deg_override=70.0)

        # Evasion complete: traveled far enough and no obstacles in wide cone
        if dist_traveled >= self.evasion_min_dist and not obstacle_ahead:
            self.get_logger().info(
                f'Evasion complete after {dist_traveled:.1f}m')
            self.evasion_offset = 0.0
            self._transition(self.LANE_FOLLOWING)
            return

        # Obstacle in the close-range dead zone (0.15-0.8m at ±70°):
        # re-evaluate evasion direction since current offset may be wrong
        if obstacle_ahead and not self._check_obstacle():
            self.evasion_offset = self._pick_evasion_offset(
                'Close obstacle!')
            self.evasion_distance = self.cumulative_distance

        # Apply evasion offset to centerline and do pure pursuit
        offset_centerline = [
            (x, y + self.evasion_offset) for x, y in self.centerline
        ]

        speed_factor = self._p('evasion_speed_factor')
        cmd = self._pure_pursuit(offset_centerline, speed_factor)
        if cmd is not None:
            self.cmd_pub.publish(cmd)
        else:
            self._stop_robot()


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
