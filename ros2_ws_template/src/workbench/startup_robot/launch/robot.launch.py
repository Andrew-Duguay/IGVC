import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Here you define each package to launch. You make a launch command for each of your robot's packages. 

    example_node = Node(
        package='example_pkg',
        executable='example_pkg',
        name='example_pkg',
        output='screen',
    )

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
        example_node,
        # Other subsystems
        #vision_launch,
        #gps_launch,
    ])
