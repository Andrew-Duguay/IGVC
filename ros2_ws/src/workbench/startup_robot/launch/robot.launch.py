import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # ── Launch arguments ──
    approach_arg = DeclareLaunchArgument(
        'approach', default_value='A',
        description='Autonomy approach mode'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz with the autonomy config'
    )

    # ── Mark's Autonomy Stack ──

    # Lane follower (A* path planner + obstacle avoidance)
    autonomy_pkg = get_package_share_directory('littleblue_autonomy')
    autonomy_params = os.path.join(autonomy_pkg, 'config', 'autonomy_params.yaml')

    lane_follower = Node(
        package='littleblue_autonomy',
        executable='lane_follower_node',
        name='lane_follower_node',
        parameters=[autonomy_params, {'approach_mode': LaunchConfiguration('approach')}],
        output='screen',
    )

    # Safety monitor REMOVED from launch — its centerline, OBSTACLES list, lap
    # detection, and segment naming are all hardcoded for the AutoNav course
    # geometry. Re-enable only when running that specific world.

    # cmd_vel to Joy bridge (for Pi motor controller)
    cmd_vel_to_joy = Node(
        package='littleblue_autonomy',
        executable='cmd_vel_to_joy_node',
        name='cmd_vel_to_joy_node',
        parameters=[{
            'use_sim_time': True,
            'max_linear_speed': 1.0,
            'max_angular_speed': 2.0,
            'publish_rate': 20.0,
        }],
        output='screen',
    )

    # Left camera lane detector
    # HSV thresholds loosened from source defaults (hsv_low_v=230, hsv_high_s=25)
    # to catch dimmer/slightly-tinted lane lines in full_course.
    # Camera mounting (raised, tilted down, angled outward) mirrors sensors.xacro.
    left_lane_detector = Node(
        package='littleblue_vision',
        executable='lane_candidate_node',
        name='left_lane_candidate',
        parameters=[{
            'use_sim_time': True,
            'method': 'hsv_threshold',
            'image_topic': '/left_camera/image_raw',
            'output_topic': '/candidate/left_lane_points',
            'debug_topic': '/candidate/left_debug',
            'hsv_low_v': 180,
            'hsv_high_s': 60,
            'camera_height': 0.32,
            'camera_lateral_offset': 0.28,
            'camera_forward_offset': 0.38,
            'camera_pitch': 0.13,
            'camera_yaw': 0.32,
            'frame_skip': 2,
            'subsample_stride': 8,
        }],
        output='screen',
    )

    # Right camera lane detector
    right_lane_detector = Node(
        package='littleblue_vision',
        executable='lane_candidate_node',
        name='right_lane_candidate',
        parameters=[{
            'use_sim_time': True,
            'method': 'hsv_threshold',
            'image_topic': '/right_camera/image_raw',
            'output_topic': '/candidate/right_lane_points',
            'debug_topic': '/candidate/right_debug',
            'hsv_low_v': 180,
            'hsv_high_s': 60,
            'camera_height': 0.32,
            'camera_lateral_offset': -0.28,
            'camera_forward_offset': 0.38,
            'camera_pitch': 0.13,
            'camera_yaw': -0.32,
            'frame_skip': 2,
            'subsample_stride': 8,
        }],
        output='screen',
    )

    # Lane accumulator (merges left+right with snapshot persistence)
    lane_accumulator = Node(
        package='littleblue_vision',
        executable='lane_accumulator_node',
        name='lane_accumulator_node',
        parameters=[{
            'use_sim_time': True,
            'mode': 'accumulate',
            'max_range': 8.0,
            'publish_rate': 15.0,
            'max_snapshots': 50,
        }],
        output='screen',
    )

    # LiDAR obstacle detector
    lidar_obstacle = Node(
        package='littleblue_vision',
        executable='lidar_obstacle_node',
        name='lidar_obstacle_node',
        parameters=[{
            'use_sim_time': True,
            'min_range': 0.5,
            'max_range': 8.0,
            'cluster_gap': 0.2,
            'min_cluster_points': 3,
        }],
        output='screen',
    )

    # ── RViz with autonomy config ──
    rviz_config = os.path.join(autonomy_pkg, 'config', 'autonomy_rviz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # ── Other team subsystems (uncomment as ready) ──

    # Vision:
    """
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('vis_pkg'), 'launch', 'vis.launch.py')
        )
    )
    """

    # GPS:
    """
    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gps_pkg'), 'launch', 'gps.launch.py')
        )
    )
    """

    # Launch everything
    return LaunchDescription([
        approach_arg,
        rviz_arg,
        # Mark's autonomy stack
        lane_follower,
        cmd_vel_to_joy,
        left_lane_detector,
        right_lane_detector,
        lane_accumulator,
        lidar_obstacle,
        rviz_node,
        # Other subsystems
        #vision_launch,
        #gps_launch,
    ])
