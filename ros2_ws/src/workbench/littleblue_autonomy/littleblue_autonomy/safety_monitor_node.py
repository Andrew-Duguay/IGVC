#!/usr/bin/env python3
"""Safety monitor for IGVC AutoNav course.

Read-only observer that uses /world_pose ground truth to detect:
- Collisions with obstacles
- Lane boundary violations
- Lap completions
- Stuck events (position unchanged >10s)

Logs events and prints per-lap summaries.
Writes JSON-lines diagnostic log and JSON results file.
"""

import json
import math
import os
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


# ── Track geometry constants (matching fake_lanes_autonav.py) ─────────

CORNER_RADIUS = 4.0
HALF_LANE = 1.5
BOTTOM_Y = 0.0
RIGHT_X = 10.0
TOP_Y = 24.0
LEFT_X = -10.0
CHICANE_X_START = 3.0
CHICANE_X_END = -3.0
CHICANE_AMPLITUDE = 1.1
RAMP_CENTER_Y = 12.0
RAMP_HALF_LEN = 2.25
RAMP_FUNNEL_LEN = 2.5

# Chassis is 0.8m x 0.6m — front half-length = 0.40m, side half-width = 0.30m
# Use front half-length as effective radius (side-passing is the common case)
ROBOT_HALF_WIDTH = 0.35

# Obstacles: (x, y, radius, name)
OBSTACLES = [
    (10, 7.98, 0.15, 'cone_1'),
    (-6, 22, 0.15, 'cone_2'),
    (3.11, -1, 0.15, 'cone_3'),
    (-10, 15, 0.25, 'barrel_1'),
    (7, 24, 0.25, 'barrel_2'),
    (-8, 1, 0.25, 'barrel_minspeed'),
    (0, 24.5, 0.30, 'tire_1'),
    (10, 17, 0.30, 'tire_2'),
    (-3, 0.5, 0.20, 'trash_can_1'),
    (5, -0.5, 0.50, 'barricade_1'),
    (10.5, 6, 0.25, 'barrel_nml_1'),
    (9.5, 4.5, 0.15, 'cone_nml_1'),
    (0.91, 1.17, 0.15, 'cone_3_1'),
    (2.94, -0.40, 0.25, 'barrel_nml_1_1'),
    (2.73, 0.28, 0.25, 'barrel_nml_1_2'),
    (1.5, 7.0, 0.15, 'cone_wp3_1'),
    (-1.0, 5.5, 0.15, 'cone_wp3_2'),
    (0.5, 5.0, 0.25, 'barrel_wp3_1'),
    (1.10, 0.52, 0.25, 'barrel_wp3_1_1'),
    (0.14, -1.06, 0.25, 'barrel_nml_1_2_1'),
    (-0.24, -0.37, 0.20, 'trash_can_1_1'),
]


def chicane_offset(x):
    """S-curve lateral offset on top straight."""
    if x > CHICANE_X_START or x < CHICANE_X_END:
        return 0.0
    t = (x - CHICANE_X_START) / (CHICANE_X_END - CHICANE_X_START)
    return CHICANE_AMPLITUDE * math.sin(t * 2 * math.pi)


def ramp_funnel_half_width(y):
    """Lane half-width on right straight near the ramp."""
    ramp_y_start = RAMP_CENTER_Y - RAMP_HALF_LEN
    ramp_y_end = RAMP_CENTER_Y + RAMP_HALF_LEN
    funnel_start = ramp_y_start - RAMP_FUNNEL_LEN
    funnel_end = ramp_y_end + RAMP_FUNNEL_LEN

    if y < funnel_start or y > funnel_end:
        return HALF_LANE
    elif y < ramp_y_start:
        t = (y - funnel_start) / RAMP_FUNNEL_LEN
        return 0.75 + (HALF_LANE - 0.75) * (1.0 + math.cos(t * math.pi)) / 2.0
    elif y <= ramp_y_end:
        return 0.75
    else:
        t = (y - ramp_y_end) / RAMP_FUNNEL_LEN
        return 0.75 + (HALF_LANE - 0.75) * (1.0 - math.cos(t * math.pi)) / 2.0


def build_centerline():
    """Build centerline polyline with per-point half_width.

    Returns list of (x, y, half_width) tuples at ~0.05m spacing.
    """
    pts = []
    step = 0.05
    n_arc = 80  # points per quarter-circle

    # 1. Bottom straight: y=0, x from -6 to 6
    x = -6.0
    while x <= 6.0:
        pts.append((x, BOTTOM_Y, HALF_LANE))
        x += step

    # 2. Bottom-right corner: center (6, 4), from -pi/2 to 0
    cx, cy = 6.0, 4.0
    for i in range(n_arc + 1):
        theta = -math.pi / 2 + (math.pi / 2) * i / n_arc
        pts.append((cx + CORNER_RADIUS * math.cos(theta),
                     cy + CORNER_RADIUS * math.sin(theta),
                     HALF_LANE))

    # 3. Right straight: x=10, y from 4 to 20
    y = 4.0
    while y <= 20.0:
        hw = ramp_funnel_half_width(y)
        pts.append((RIGHT_X, y, hw))
        y += step

    # 4. Top-right corner: center (6, 20), from 0 to pi/2
    cx, cy = 6.0, 20.0
    for i in range(n_arc + 1):
        theta = 0 + (math.pi / 2) * i / n_arc
        pts.append((cx + CORNER_RADIUS * math.cos(theta),
                     cy + CORNER_RADIUS * math.sin(theta),
                     HALF_LANE))

    # 5. Top straight: y=24, x from 6 to -6 (travel direction -x)
    # Chicane shifts centerline
    x = 6.0
    while x >= -6.0:
        ch = chicane_offset(x)
        pts.append((x, TOP_Y - ch, HALF_LANE))
        x -= step

    # 6. Top-left corner: center (-6, 20), from pi/2 to pi
    cx, cy = -6.0, 20.0
    for i in range(n_arc + 1):
        theta = math.pi / 2 + (math.pi / 2) * i / n_arc
        pts.append((cx + CORNER_RADIUS * math.cos(theta),
                     cy + CORNER_RADIUS * math.sin(theta),
                     HALF_LANE))

    # 7. Left straight: x=-10, y from 20 to 4
    y = 20.0
    while y >= 4.0:
        pts.append((LEFT_X, y, HALF_LANE))
        y -= step

    # 8. Bottom-left corner: center (-6, 4), from pi to 3pi/2
    cx, cy = -6.0, 4.0
    for i in range(n_arc + 1):
        theta = math.pi + (math.pi / 2) * i / n_arc
        pts.append((cx + CORNER_RADIUS * math.cos(theta),
                     cy + CORNER_RADIUS * math.sin(theta),
                     HALF_LANE))

    return pts


class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__('safety_monitor_node')

        self.centerline = build_centerline()
        self.get_logger().info(f'Centerline: {len(self.centerline)} points')

        # State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.got_pose = False

        # Lap detection: track y crossing thresholds
        self.was_above_20 = False
        self.lap_count = 0

        # Per-lap stats
        self.lap_collisions = 0
        self.lap_violations = 0
        self.lap_min_obs_clearance = float('inf')
        self.lap_min_lane_margin = float('inf')
        self.lap_violations_by_seg = {}  # segment_name -> count

        # Lap timing
        self._lap_start_time = None
        self._lap_times = []

        # Total stats
        self.total_collisions = 0
        self.total_violations = 0
        self.collision_log = []
        self.violation_log = []

        # Collision cooldown: avoid spamming for same obstacle
        self._collision_cooldown = {}  # name -> last_time

        # Status timer counter
        self._tick = 0

        # Stuck detection
        self._stuck_check_time = time.monotonic()
        self._stuck_check_x = 0.0
        self._stuck_check_y = 0.0
        self._stuck_events = 0

        # Per-obstacle tracking: min clearance ever seen
        self._obs_min_clearance = {name: float('inf') for _, _, _, name in OBSTACLES}

        # Track segment corners
        self._corners = [
            (6.0, 4.0, 'BR_corner'),
            (6.0, 20.0, 'TR_corner'),
            (-6.0, 20.0, 'TL_corner'),
            (-6.0, 4.0, 'BL_corner'),
        ]

        # JSON-lines diagnostic log
        self._diag_dir = '/tmp/autonomy_diag'
        os.makedirs(self._diag_dir, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        self._event_log_path = os.path.join(self._diag_dir, f'safety_events_{ts}.jsonl')
        self._event_log = open(self._event_log_path, 'w')

        # Subscribe to ground truth pose
        self.create_subscription(PoseStamped, '/world_pose', self._pose_cb, 10)

        # 10Hz check timer
        self.create_timer(0.1, self._check)

        self.get_logger().info('Safety monitor started (with diagnostics)')

    def _log_event(self, event_type, data):
        """Write a JSON-lines event to the diagnostic log."""
        entry = {
            'time': time.monotonic(),
            'tick': self._tick,
            'type': event_type,
            'robot_x': round(self.robot_x, 3),
            'robot_y': round(self.robot_y, 3),
            'segment': self._segment_name(self.robot_x, self.robot_y),
            'lap': self.lap_count,
        }
        entry.update(data)
        self._event_log.write(json.dumps(entry) + '\n')
        if self._tick % 50 == 0:
            self._event_log.flush()

    def _segment_name(self, x, y):
        """Identify which track segment the robot is on."""
        # Check corners first (within CORNER_RADIUS of corner center)
        for cx, cy, name in self._corners:
            if math.sqrt((x - cx)**2 + (y - cy)**2) < CORNER_RADIUS + 1.0:
                return name
        # Straights
        if y < 4.0:
            return 'bottom'
        elif y > 20.0:
            return 'top'
        elif x > 6.0:
            ramp_start = RAMP_CENTER_Y - RAMP_HALF_LEN - RAMP_FUNNEL_LEN
            ramp_end = RAMP_CENTER_Y + RAMP_HALF_LEN + RAMP_FUNNEL_LEN
            if ramp_start <= y <= ramp_end:
                return 'right_ramp'
            return 'right'
        elif x < -6.0:
            return 'left'
        return 'unknown'

    def _pose_cb(self, msg):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        self.got_pose = True
        if self._lap_start_time is None:
            self._lap_start_time = time.monotonic()

    def _check(self):
        if not self.got_pose:
            return

        rx, ry = self.robot_x, self.robot_y
        self._tick += 1

        # ── Stuck detection (10s window) ──
        now_mono = time.monotonic()
        if now_mono - self._stuck_check_time > 10.0:
            sdx = rx - self._stuck_check_x
            sdy = ry - self._stuck_check_y
            moved = math.sqrt(sdx * sdx + sdy * sdy)
            if moved < 0.5 and self.got_pose:
                self._stuck_events += 1
                self.get_logger().warn(
                    f'[STUCK] Position unchanged >10s at ({rx:.1f},{ry:.1f}), '
                    f'moved={moved:.2f}m (event #{self._stuck_events})')
                self._log_event('stuck', {'moved': round(moved, 3),
                                          'event_num': self._stuck_events})
            self._stuck_check_time = now_mono
            self._stuck_check_x = rx
            self._stuck_check_y = ry

        # ── Collision check ──
        for ox, oy, orad, name in OBSTACLES:
            dist = math.sqrt((rx - ox) ** 2 + (ry - oy) ** 2)
            threshold = orad + ROBOT_HALF_WIDTH
            clearance = dist - threshold

            # Track min obstacle clearance (per-obstacle)
            if clearance < self._obs_min_clearance[name]:
                self._obs_min_clearance[name] = clearance

            # Track global min
            if clearance < self.lap_min_obs_clearance:
                self.lap_min_obs_clearance = clearance

            if clearance < 0:
                now = self.get_clock().now().nanoseconds / 1e9
                last = self._collision_cooldown.get(name, 0.0)
                if now - last > 3.0:  # 3s cooldown per obstacle
                    penetration = -clearance
                    self.get_logger().error(
                        f'[COLLISION] Hit {name} at ({rx:.1f},{ry:.1f}), '
                        f'penetration={penetration:.2f}m')
                    self.collision_log.append((name, rx, ry, penetration))
                    self.lap_collisions += 1
                    self.total_collisions += 1
                    self._collision_cooldown[name] = now
                    self._log_event('collision', {
                        'obstacle': name, 'obs_x': ox, 'obs_y': oy,
                        'obs_radius': orad, 'penetration': round(penetration, 4),
                        'distance': round(dist, 4),
                    })
            elif clearance < 0.15:
                # Near-miss: log when clearance is very tight
                if self._tick % 5 == 0:  # every 0.5s to avoid spam
                    self._log_event('near_miss', {
                        'obstacle': name, 'clearance': round(clearance, 4),
                        'distance': round(dist, 4),
                    })

        # ── Lane boundary check ──
        min_dist_to_center = float('inf')
        nearest_hw = HALF_LANE
        for cx, cy, hw in self.centerline:
            d = math.sqrt((rx - cx) ** 2 + (ry - cy) ** 2)
            if d < min_dist_to_center:
                min_dist_to_center = d
                nearest_hw = hw

        # Use side half-width (0.30m) for boundary check — robot edge must stay in lane
        side_hw = 0.30
        lane_margin = nearest_hw - min_dist_to_center - side_hw
        if lane_margin < self.lap_min_lane_margin:
            self.lap_min_lane_margin = lane_margin

        if min_dist_to_center + side_hw > nearest_hw:
            overshoot = min_dist_to_center + side_hw - nearest_hw
            # Only log if significant overshoot (>0.05m avoids noise)
            if overshoot > 0.05:
                seg = self._segment_name(rx, ry)
                self.get_logger().warn(
                    f'[BOUNDARY] Crossed at ({rx:.1f},{ry:.1f}) [{seg}], '
                    f'overshoot={overshoot:.2f}m hw={nearest_hw:.2f}m')
                self.violation_log.append((rx, ry, overshoot))
                self.lap_violations += 1
                self.total_violations += 1
                self.lap_violations_by_seg[seg] = self.lap_violations_by_seg.get(seg, 0) + 1
                self._log_event('boundary', {
                    'overshoot': round(overshoot, 4),
                    'half_width': round(nearest_hw, 3),
                    'dist_to_center': round(min_dist_to_center, 4),
                })

        # ── Lap detection ──
        if ry > 20.0:
            self.was_above_20 = True
        if self.was_above_20 and ry < 4.0:
            self.was_above_20 = False
            self.lap_count += 1

            # Lap timing
            lap_time = 0.0
            if self._lap_start_time is not None:
                lap_time = time.monotonic() - self._lap_start_time
                self._lap_times.append(lap_time)
                self._lap_start_time = time.monotonic()

            seg_str = ' '.join(f'{k}={v}' for k, v in sorted(self.lap_violations_by_seg.items()))
            self.get_logger().info(
                f'[LAP {self.lap_count}] time={lap_time:.1f}s '
                f'collisions={self.lap_collisions} '
                f'violations={self.lap_violations} '
                f'min_obs_clearance={self.lap_min_obs_clearance:.2f}m '
                f'min_lane_margin={self.lap_min_lane_margin:.2f}m '
                f'segments: {seg_str}')
            self._log_event('lap', {
                'lap_num': self.lap_count,
                'lap_time': round(lap_time, 1),
                'collisions': self.lap_collisions,
                'violations': self.lap_violations,
                'min_obs_clearance': round(self.lap_min_obs_clearance, 4),
                'min_lane_margin': round(self.lap_min_lane_margin, 4),
                'violations_by_seg': dict(self.lap_violations_by_seg),
            })
            # Reset per-lap stats
            self.lap_collisions = 0
            self.lap_violations = 0
            self.lap_min_obs_clearance = float('inf')
            self.lap_min_lane_margin = float('inf')
            self.lap_violations_by_seg = {}

        # ── Periodic status (every 2s = every 20 ticks at 10Hz) ──
        if self._tick % 20 == 0:
            # Find nearest obstacle
            nearest_name = ''
            nearest_dist = float('inf')
            for ox, oy, orad, name in OBSTACLES:
                d = math.sqrt((rx - ox) ** 2 + (ry - oy) ** 2)
                if d < nearest_dist:
                    nearest_dist = d
                    nearest_name = name

            self.get_logger().info(
                f'[SAFETY] pos=({rx:.1f},{ry:.1f}) '
                f'margin={lane_margin:.2f}m '
                f'nearest={nearest_name}@{nearest_dist:.1f}m '
                f'laps={self.lap_count} '
                f'col={self.total_collisions} bnd={self.total_violations}')

            # Log periodic status to JSONL
            self._log_event('status', {
                'lane_margin': round(lane_margin, 4),
                'nearest_obs': nearest_name,
                'nearest_dist': round(nearest_dist, 3),
            })

    def destroy_node(self):
        # Print final summary
        self.get_logger().info('='*60)
        self.get_logger().info('SAFETY MONITOR FINAL SUMMARY')
        self.get_logger().info(f'  Laps completed: {self.lap_count}')
        self.get_logger().info(f'  Lap times: {[f"{t:.1f}s" for t in self._lap_times]}')
        self.get_logger().info(f'  Total collisions: {self.total_collisions}')
        self.get_logger().info(f'  Total boundary violations: {self.total_violations}')
        self.get_logger().info(f'  Stuck events: {self._stuck_events}')
        if self.collision_log:
            self.get_logger().info('  Collision details:')
            for name, x, y, pen in self.collision_log:
                self.get_logger().info(f'    {name} at ({x:.1f},{y:.1f}) pen={pen:.2f}m')
        if self.violation_log:
            self.get_logger().info(f'  Boundary violations: {len(self.violation_log)} events')
        # Per-obstacle min clearance
        self.get_logger().info('  Per-obstacle min clearance:')
        for name, clr in sorted(self._obs_min_clearance.items(), key=lambda x: x[1]):
            if clr < float('inf'):
                self.get_logger().info(f'    {name}: {clr:.3f}m')
        self.get_logger().info('='*60)

        # Write summary event and close log
        self._log_event('summary', {
            'laps': self.lap_count,
            'lap_times': [round(t, 1) for t in self._lap_times],
            'total_collisions': self.total_collisions,
            'total_violations': self.total_violations,
            'stuck_events': self._stuck_events,
            'per_obstacle_min_clearance': {
                name: round(clr, 4) for name, clr in self._obs_min_clearance.items()
                if clr < float('inf')
            },
            'collision_details': [
                {'name': n, 'x': round(x, 2), 'y': round(y, 2), 'pen': round(p, 4)}
                for n, x, y, p in self.collision_log
            ],
        })
        self._event_log.flush()
        self._event_log.close()
        self.get_logger().info(f'Event log saved to {self._event_log_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
