import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('littleblue_vision')
    params_file = os.path.join(pkg_share, 'config', 'vision_params.yaml')

    left_lane_detector = Node(
        package='littleblue_vision',
        executable='lane_detector_node',
        name='left_lane_detector',
        parameters=[params_file, {
            'image_topic': '/left_camera/image_raw',
            'camera_lateral_offset': 0.28,
        }],
        output='screen',
    )

    right_lane_detector = Node(
        package='littleblue_vision',
        executable='lane_detector_node',
        name='right_lane_detector',
        parameters=[params_file, {
            'image_topic': '/right_camera/image_raw',
            'camera_lateral_offset': -0.28,
        }],
        output='screen',
    )

    left_obstacle_projector = Node(
        package='littleblue_vision',
        executable='obstacle_projector_node',
        name='left_obstacle_projector',
        parameters=[params_file, {
            'depth_topic': '/left_camera/depth/image_raw',
            'camera_lateral_offset': 0.28,
        }],
        output='screen',
    )

    right_obstacle_projector = Node(
        package='littleblue_vision',
        executable='obstacle_projector_node',
        name='right_obstacle_projector',
        parameters=[params_file, {
            'depth_topic': '/right_camera/depth/image_raw',
            'camera_lateral_offset': -0.28,
        }],
        output='screen',
    )

    return LaunchDescription([
        left_lane_detector,
        right_lane_detector,
        left_obstacle_projector,
        right_obstacle_projector,
    ])
