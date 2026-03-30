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