import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_dir = get_package_share_directory('littleblue_sim')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'littleblue.urdf.xacro')
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz_config.rviz')

    robot_description = xacro.process_file(urdf_file).toxml()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                      'use_sim_time': True}],
        output='screen',
    )

    joint_state_pub = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        robot_state_pub,
        joint_state_pub,
        rviz,
    ])
