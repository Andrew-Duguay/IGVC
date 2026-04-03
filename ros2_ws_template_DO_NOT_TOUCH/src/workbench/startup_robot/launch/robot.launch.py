import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    launch_example = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('example_pkg'), 'launch', 'example.launch.py'
                )
        )
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
        launch_example
        #vision_launch,
        #gps_launch,
    ])
