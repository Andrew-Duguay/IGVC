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

    approach_arg = DeclareLaunchArgument(
        'approach', default_value='A',
        description='Approach mode: baseline, A, B, C, D'
    )

    record_arg = DeclareLaunchArgument(
        'record', default_value='false',
        description='Launch data recorder node'
    )

    lane_follower = Node(
        package='littleblue_autonomy',
        executable='lane_follower_node',
        name='lane_follower_node',
        parameters=[params_file, {'approach_mode': LaunchConfiguration('approach')}],
        output='screen',
    )

    data_recorder = Node(
        package='littleblue_autonomy',
        executable='data_recorder_node',
        name='data_recorder_node',
        parameters=[{'use_sim_time': True,
                     'approach_label': LaunchConfiguration('approach')}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record')),
    )

    safety_monitor = Node(
        package='littleblue_autonomy',
        executable='safety_monitor_node',
        name='safety_monitor_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

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
        approach_arg,
        record_arg,
        lane_follower,
        data_recorder,
        safety_monitor,
        cmd_vel_to_joy,
        rviz_node,
    ])
