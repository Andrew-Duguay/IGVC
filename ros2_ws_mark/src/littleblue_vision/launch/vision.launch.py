import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('littleblue_vision')
    params_file = os.path.join(pkg_share, 'config', 'vision_params.yaml')

    lane_detector = Node(
        package='littleblue_vision',
        executable='lane_detector_node',
        name='lane_detector_node',
        parameters=[params_file],
        output='screen',
    )

    obstacle_projector = Node(
        package='littleblue_vision',
        executable='obstacle_projector_node',
        name='obstacle_projector_node',
        parameters=[params_file],
        output='screen',
    )

    return LaunchDescription([
        lane_detector,
        obstacle_projector,
    ])
