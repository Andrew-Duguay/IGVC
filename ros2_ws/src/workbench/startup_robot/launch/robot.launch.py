import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Launch file used for example:
    example_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('example_pkg'), 'launch', 'example.launch.py')
        )
    )

    # Launch file 1:
    """
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('vis_pkg'), 'launch', 'vis.launch.py')
        )
    )
    """

    # Launch file 2:
    """
    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gps_pkg'), 'launch', 'gps.launch.py')
        )
    )
    """

    # Launch file 3:
    """
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav_pkg'), 'launch', 'nav.launch.py')
        )
    )
    """

    # Launch them all together
    return LaunchDescription([
        example_launch,
        #vision_launch,
        #gps_launch,
        #nav_launch
    ])