import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('littleblue_vision')
    params_file = os.path.join(pkg_share, 'config', 'vision_params.yaml')

    left_lane_candidate = Node(
        package='littleblue_vision',
        executable='lane_candidate_node',
        name='left_lane_candidate',
        parameters=[params_file, {
            'image_topic': '/left/left_camera/image_raw',
            'camera_lateral_offset': 0.28,
            'subsample_stride': 10,  # تخفيف النقط
        }],
        remappings=[
            ('/candidate/lane_points', '/candidate/left_lane_points'), # رجعنا السلك للمجمع
            ('/candidate/lane_mask', '/candidate/left_lane_mask'),
            ('/candidate/debug_image', '/candidate/left_debug_image')
        ],
        output='screen',
    )

    right_lane_candidate = Node(
        package='littleblue_vision',
        executable='lane_candidate_node',
        name='right_lane_candidate',
        parameters=[params_file, {
            'image_topic': '/right/right_camera/image_raw',
            'camera_lateral_offset': -0.28,
            'subsample_stride': 10,
        }],
        remappings=[
            ('/candidate/lane_points', '/candidate/right_lane_points'),
            ('/candidate/lane_mask', '/candidate/right_lane_mask'),
            ('/candidate/debug_image', '/candidate/right_debug_image')
        ],
        output='screen',
    )

    lane_accumulator = Node(
        package='littleblue_vision',
        executable='lane_accumulator_node',
        name='lane_accumulator',
        parameters=[params_file],
        remappings=[
            ('/candidate/lane_points', '/lane_points')  # المجمع بيبعت للعقل
        ],
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
        left_lane_candidate,
        right_lane_candidate,
        lane_accumulator,
        left_obstacle_projector,
        right_obstacle_projector,
    ])
