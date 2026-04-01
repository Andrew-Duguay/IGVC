#!/usr/bin/env python3
"""Generate IGVC AutoNav course SDF.

Produces src/littleblue_sim/worlds/igvc_autonav.sdf with:
- Rounded rectangle track with lane lines
- Chicane on top straight
- Ramp on right straight with funnel
- Obstacles (cones, barrels, tires, trash cans, barricade)
- Waypoint ground markers
"""

import math
import os

# ── Track geometry constants ──────────────────────────────────────────

# Corner centers of the rounded rectangle
# Bottom-right: (6, 4), Top-right: (6, 20), Top-left: (-6, 20), Bottom-left: (-6, 4)
CORNER_RADIUS = 4.0
LANE_WIDTH = 3.0  # total width; ±1.5 from centerline
HALF_LANE = LANE_WIDTH / 2.0

# Straight centerlines
BOTTOM_Y = 0.0
RIGHT_X = 10.0
TOP_Y = 24.0
LEFT_X = -10.0

# Corner centers
CORNERS = [
    (6.0, 4.0),    # bottom-right
    (6.0, 20.0),   # top-right
    (-6.0, 20.0),  # top-left
    (-6.0, 4.0),   # bottom-left
]

# Chicane parameters (on top straight, y≈24)
CHICANE_X_START = 3.0
CHICANE_X_END = -3.0
CHICANE_AMPLITUDE = 1.1  # lateral deviation (was 1.5)

# No man's land on right straight (no lane lines, waypoint-only navigation)
NML_Y_START = 4.0   # start of gap (bottom)
NML_Y_END = 9.75    # end of gap (top, at ramp entrance)

# Ramp on right straight
RAMP_CENTER_Y = 12.0
RAMP_HALF_LEN = 2.25  # approach+ramp+exit extent along y
RAMP_FUNNEL_LEN = 2.5  # funnel transition length

# Lane line segment properties
SEG_LEN = 0.3
SEG_WIDTH = 0.1
SEG_HEIGHT = 0.01
SEG_Z = 0.01

# ── Lane line generation ─────────────────────────────────────────────

def generate_centerline_points(step=0.15):
    """Generate centerline points going counterclockwise around the track.

    Order: bottom straight (left to right), bottom-right corner, right straight
    (bottom to top), top-right corner, top straight (right to left) with chicane,
    top-left corner, left straight (top to bottom), bottom-left corner.
    """
    pts = []

    # 1. Bottom straight: x from -6 to 6, y=0
    x = -6.0
    while x <= 6.0:
        pts.append((x, BOTTOM_Y))
        x += step

    # 2. Bottom-right corner: center (6, 4), from -pi/2 to 0
    cx, cy = 6.0, 4.0
    n_arc = int(math.pi / 2 / step * CORNER_RADIUS)
    for i in range(n_arc + 1):
        theta = -math.pi / 2 + (math.pi / 2) * i / n_arc
        pts.append((cx + CORNER_RADIUS * math.cos(theta),
                     cy + CORNER_RADIUS * math.sin(theta)))

    # 3. Right straight: y from 4 to 20, x=10
    y = 4.0
    while y <= 20.0:
        pts.append((RIGHT_X, y))
        y += step

    # 4. Top-right corner: center (6, 20), from 0 to pi/2
    cx, cy = 6.0, 20.0
    for i in range(n_arc + 1):
        theta = 0 + (math.pi / 2) * i / n_arc
        pts.append((cx + CORNER_RADIUS * math.cos(theta),
                     cy + CORNER_RADIUS * math.sin(theta)))

    # 5. Top straight: x from 6 to -6, y=24 (with chicane)
    x = 6.0
    while x >= -6.0:
        pts.append((x, TOP_Y))
        x -= step

    # 6. Top-left corner: center (-6, 20), from pi/2 to pi
    cx, cy = -6.0, 20.0
    for i in range(n_arc + 1):
        theta = math.pi / 2 + (math.pi / 2) * i / n_arc
        pts.append((cx + CORNER_RADIUS * math.cos(theta),
                     cy + CORNER_RADIUS * math.sin(theta)))

    # 7. Left straight: y from 20 to 4, x=-10
    y = 20.0
    while y >= 4.0:
        pts.append((LEFT_X, y))
        y -= step

    # 8. Bottom-left corner: center (-6, 4), from pi to 3pi/2
    cx, cy = -6.0, 4.0
    for i in range(n_arc + 1):
        theta = math.pi + (math.pi / 2) * i / n_arc
        pts.append((cx + CORNER_RADIUS * math.cos(theta),
                     cy + CORNER_RADIUS * math.sin(theta)))

    return pts


def offset_point(cx, cy, nx, ny, offset):
    """Offset a point by `offset` along normal (nx, ny)."""
    return (cx + nx * offset, cy + ny * offset)


def compute_normal(pts, i):
    """Compute outward normal at point i (pointing left of travel direction)."""
    if i == 0:
        dx = pts[1][0] - pts[0][0]
        dy = pts[1][1] - pts[0][1]
    elif i == len(pts) - 1:
        dx = pts[-1][0] - pts[-2][0]
        dy = pts[-1][1] - pts[-2][1]
    else:
        dx = pts[i + 1][0] - pts[i - 1][0]
        dy = pts[i + 1][1] - pts[i - 1][1]
    length = math.sqrt(dx * dx + dy * dy)
    if length < 1e-9:
        return (0.0, 1.0)
    # Normal: rotate tangent 90° left (counterclockwise)
    nx = -dy / length
    ny = dx / length
    return (nx, ny)


def chicane_offset(x):
    """Lateral offset for chicane on top straight.

    S-curve from x=3 to x=-3, sinusoidal, amplitude ±1.5m.
    """
    if x > CHICANE_X_START or x < CHICANE_X_END:
        return 0.0
    t = (x - CHICANE_X_START) / (CHICANE_X_END - CHICANE_X_START)  # 0→1
    return CHICANE_AMPLITUDE * math.sin(t * 2 * math.pi)


def ramp_funnel_half_width(y):
    """Half-width of lane on right straight near the ramp.

    Normal lane is 1.5m half-width. Ramp is 0.75m wide on each side.
    Curved (cosine) transitions for smooth entrance/exit.
    """
    ramp_y_start = RAMP_CENTER_Y - RAMP_HALF_LEN
    ramp_y_end = RAMP_CENTER_Y + RAMP_HALF_LEN
    funnel_start = ramp_y_start - RAMP_FUNNEL_LEN
    funnel_end = ramp_y_end + RAMP_FUNNEL_LEN

    if y < funnel_start or y > funnel_end:
        return HALF_LANE
    elif y < ramp_y_start:
        # Smooth cosine curve from HALF_LANE to 0.75
        t = (y - funnel_start) / RAMP_FUNNEL_LEN  # 0→1
        return 0.75 + (HALF_LANE - 0.75) * (1.0 + math.cos(t * math.pi)) / 2.0
    elif y <= ramp_y_end:
        return 0.75
    else:
        # Smooth cosine curve from 0.75 to HALF_LANE
        t = (y - ramp_y_end) / RAMP_FUNNEL_LEN  # 0→1
        return 0.75 + (HALF_LANE - 0.75) * (1.0 - math.cos(t * math.pi)) / 2.0


def recompute_yaws(segments):
    """Recompute yaw for each segment from actual positions (not centerline)."""
    result = []
    for i, (x, y, _) in enumerate(segments):
        if i < len(segments) - 1:
            dx = segments[i + 1][0] - x
            dy = segments[i + 1][1] - y
        else:
            dx = x - segments[i - 1][0]
            dy = y - segments[i - 1][1]
        yaw = math.atan2(dy, dx)
        result.append((x, y, yaw))
    return result


def generate_lane_points():
    """Generate inner and outer lane line points.

    Returns list of (x, y, yaw) for each lane segment center.
    """
    center_pts = generate_centerline_points(step=0.15)

    inner_segments = []
    outer_segments = []

    for i, (cx, cy) in enumerate(center_pts):
        nx, ny = compute_normal(center_pts, i)

        # Determine offsets based on position
        inner_off = HALF_LANE
        outer_off = HALF_LANE

        # Chicane: shift both lines on top straight
        is_top_straight = (23.0 < cy < 25.0 and -7.0 < cx < 7.0)
        if is_top_straight:
            ch_off = chicane_offset(cx)
            # Shift centerline laterally (in the normal direction)
            cx += nx * ch_off
            cy += ny * ch_off

        # Ramp funnel: narrow lane on right straight
        is_right_straight = (9.0 < cx < 11.0 and 4.0 < cy < 20.0)
        if is_right_straight:
            hw = ramp_funnel_half_width(cy)
            inner_off = hw
            outer_off = hw

        # Placeholder yaw — will be recomputed below
        yaw = 0.0

        # Inner line (left of travel = positive normal direction)
        ix, iy = offset_point(cx, cy, nx, ny, inner_off)
        inner_segments.append((ix, iy, yaw))

        # Outer line (right of travel = negative normal direction)
        ox, oy = offset_point(cx, cy, nx, ny, -outer_off)
        outer_segments.append((ox, oy, yaw))

    # Recompute yaw from actual lane line positions so segments
    # follow the curve direction (important for ramp funnel + chicane)
    inner_segments = recompute_yaws(inner_segments)
    outer_segments = recompute_yaws(outer_segments)

    return inner_segments, outer_segments


# ── SDF generation ────────────────────────────────────────────────────

def lane_segment_sdf(name, x, y, yaw):
    """Generate SDF for one lane line segment."""
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {SEG_Z} 0 0 {yaw:.4f}</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{SEG_LEN} {SEG_WIDTH} {SEG_HEIGHT}</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <emissive>1 1 1 1</emissive>
          </material>
        </visual>
      </link>
    </model>"""


def waypoint_marker_sdf(name, x, y):
    """Generate a + shaped ground marker for navigation waypoints."""
    return f"""    <!-- Waypoint: {name} -->
    <model name="{name}">
      <static>true</static>
      <link name="link">
        <visual name="arm_x">
          <pose>{x} {y} 0.005 0 0 0</pose>
          <geometry><box><size>1.0 0.15 0.01</size></box></geometry>
          <material>
            <ambient>1 0.8 0 1</ambient>
            <diffuse>1 0.8 0 1</diffuse>
            <emissive>0.5 0.4 0 1</emissive>
          </material>
        </visual>
        <visual name="arm_y">
          <pose>{x} {y} 0.005 0 0 1.5708</pose>
          <geometry><box><size>1.0 0.15 0.01</size></box></geometry>
          <material>
            <ambient>1 0.8 0 1</ambient>
            <diffuse>1 0.8 0 1</diffuse>
            <emissive>0.5 0.4 0 1</emissive>
          </material>
        </visual>
      </link>
    </model>"""


def obstacle_include_sdf(model_type, name, x, y, z=0, yaw=0):
    """Generate <include> element for an obstacle."""
    return f"""    <include>
      <uri>model://{model_type}</uri>
      <name>{name}</name>
      <pose>{x} {y} {z} 0 0 {yaw}</pose>
    </include>"""


def generate_sdf():
    """Generate the complete SDF file content."""
    inner_segs, outer_segs = generate_lane_points()

    parts = []

    # Header
    parts.append("""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="igvc_autonav">

    <!-- Physics -->
    <physics name="dart_physics" type="dart">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>3.0</real_time_factor>
    </physics>

    <!-- Spherical coordinates: Oakland University, MI (IGVC venue) -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>42.6745</latitude_deg>
      <longitude_deg>-83.2187</longitude_deg>
      <elevation>282.0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <!-- System plugins -->
    <plugin filename="ignition-gazebo-physics-system"
            name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-sensors-system"
            name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-imu-system"
            name="ignition::gazebo::systems::Imu"/>
    <plugin filename="ignition-gazebo-navsat-system"
            name="ignition::gazebo::systems::NavSat"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system"
            name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-user-commands-system"
            name="ignition::gazebo::systems::UserCommands"/>

    <!-- Sun -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.5 0.3 -0.9</direction>
    </light>

    <!-- Ground plane (dark gray asphalt) -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>60 60</size></plane>
          </geometry>
          <surface>
            <friction>
              <ode><mu>0.8</mu><mu2>0.8</mu2></ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>60 60</size></plane>
          </geometry>
          <material>
            <ambient>0.25 0.25 0.25 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>""")

    # Lane lines
    parts.append("\n    <!-- ===== WHITE LANE LINES ===== -->")
    parts.append("    <!-- Rounded rectangle with chicane and ramp funnel -->")

    # Subsample: place a segment every ~SEG_LEN along the path
    def subsample_segments(segs, spacing=0.3):
        """Pick segments spaced approximately `spacing` apart."""
        result = [segs[0]]
        last_x, last_y = segs[0][0], segs[0][1]
        for x, y, yaw in segs[1:]:
            dist = math.sqrt((x - last_x) ** 2 + (y - last_y) ** 2)
            if dist >= spacing:
                result.append((x, y, yaw))
                last_x, last_y = x, y
        return result

    def in_no_mans_land(x, y):
        """Check if a segment is in the no man's land zone (right straight, y=4..12)."""
        return 8.0 < x < 12.0 and NML_Y_START <= y <= NML_Y_END

    inner_sub = subsample_segments(inner_segs, SEG_LEN)
    outer_sub = subsample_segments(outer_segs, SEG_LEN)

    # Filter out segments in the no man's land
    inner_sub = [s for s in inner_sub if not in_no_mans_land(s[0], s[1])]
    outer_sub = [s for s in outer_sub if not in_no_mans_land(s[0], s[1])]

    parts.append(f"\n    <!-- Inner lane line: {len(inner_sub)} segments -->")
    for i, (x, y, yaw) in enumerate(inner_sub):
        parts.append(lane_segment_sdf(f"lane_inner_{i}", x, y, yaw))

    parts.append(f"\n    <!-- Outer lane line: {len(outer_sub)} segments -->")
    for i, (x, y, yaw) in enumerate(outer_sub):
        parts.append(lane_segment_sdf(f"lane_outer_{i}", x, y, yaw))

    # Obstacles
    parts.append("\n    <!-- ===== OBSTACLES ===== -->")

    obstacles = [
        ("cone", "cone_1", 10, 8, 0, 0),
        ("cone", "cone_2", -6, 22, 0, 0),
        ("cone", "cone_3", 3, -1, 0, 0),
        ("barrel", "barrel_1", -10, 15, 0, 0),
        ("barrel", "barrel_2", 7, 24, 0, 0),
        ("barrel", "barrel_minspeed", -8, 1, 0, 0),
        ("tire", "tire_1", 0, 24.5, 0, 0),
        ("tire", "tire_2", 10, 17, 0, 0),
        ("trash_can", "trash_can_1", -3, 0.5, 0, 0),
        ("barricade", "barricade_1", 5, -0.5, 0, 0),
        ("barrel", "barrel_nml_1", 10.5, 6, 0, 0),
        ("cone", "cone_nml_1", 9.5, 4.5, 0, 0),
    ]

    for model_type, name, x, y, z, yaw in obstacles:
        parts.append(obstacle_include_sdf(model_type, name, x, y, z, yaw))

    # Ramp on right straight, rotated 90° so it faces along Y axis
    parts.append("\n    <!-- ===== RAMP ===== -->")
    parts.append(obstacle_include_sdf("ramp", "ramp_1", 10, 12, 0, 1.5708))

    # Waypoint markers
    parts.append("\n    <!-- ===== WAYPOINT MARKERS (visual only) ===== -->")
    waypoints = [
        ("waypoint_1", -4, 12),
        ("waypoint_2", 4, 12),
        ("waypoint_3", 0, 6),
        ("waypoint_4", 0, 18),
        ("nml_entry", 10, NML_Y_START),   # No man's land entry
        ("nml_exit", 10, NML_Y_END),      # No man's land exit
    ]
    for name, x, y in waypoints:
        parts.append(waypoint_marker_sdf(name, x, y))

    # Footer
    parts.append("""
  </world>
</sdf>""")

    return "\n".join(parts)


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    ws_dir = os.path.dirname(script_dir)
    output_path = os.path.join(ws_dir, "src", "littleblue_sim", "worlds", "igvc_autonav.sdf")

    sdf_content = generate_sdf()

    with open(output_path, 'w') as f:
        f.write(sdf_content)

    # Count segments
    inner_count = sdf_content.count('lane_inner_')
    outer_count = sdf_content.count('lane_outer_')
    print(f"Generated {output_path}")
    print(f"  Inner lane segments: {inner_count}")
    print(f"  Outer lane segments: {outer_count}")
    print(f"  Total models: {inner_count + outer_count + 10 + 1 + 4}")  # lanes + obstacles + ramp + waypoints


if __name__ == '__main__':
    main()
