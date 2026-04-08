import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('littleblue_vision')
    params_file = os.path.join(pkg_share, 'config', 'vision_params.yaml')

    left_lane = Node(
        package='littleblue_vision',
        executable='lane_candidate_node',
        name='left_lane_candidate',
        parameters=[params_file, {'image_topic': '/left/left_camera/image_raw', 'camera_lateral_offset': 0.28, 'subsample_stride': 10, 'output_topic': '/lane_points'}],
        output='screen'
    )
    
    right_lane = Node(
        package='littleblue_vision',
        executable='lane_candidate_node',
        name='right_lane_candidate',
        parameters=[params_file, {'image_topic': '/right/right_camera/image_raw', 'camera_lateral_offset': -0.28, 'subsample_stride': 10, 'output_topic': '/lane_points'}],
        output='screen'
    )

    return LaunchDescription([left_lane, right_lane])
