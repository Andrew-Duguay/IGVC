import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('littleblue_autonomy')
    params_file = os.path.join(pkg_share, 'config', 'autonomy_params.yaml')
    rviz_config_file = os.path.join(pkg_share, 'config', 'autonomy_rviz.rviz')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz'
    )

    lane_follower = Node(
        package='littleblue_autonomy',
        executable='lane_follower_node',
        name='lane_follower_node',
        parameters=[params_file],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg,
        lane_follower,
        rviz_node,
    ])
