import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_pkg = get_package_share_directory('littleblue_sim')
    vision_pkg = get_package_share_directory('littleblue_vision')
    nav_pkg = get_package_share_directory('littleblue_nav')
    autonomy_pkg = get_package_share_directory('littleblue_autonomy')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz'
    )

    # Simulation (Ignition + robot + bridge)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, 'launch', 'sim.launch.py')
        ),
    )

    # Vision (lane detector + obstacle projector)
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vision_pkg, 'launch', 'vision.launch.py')
        ),
    )

    # Nav2 (controller, planner, behaviors, BT — with RViz disabled)
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'rviz': 'false'}.items(),
    )

    # Autonomy (lane follower node + RViz)
    autonomy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(autonomy_pkg, 'launch', 'autonomy.launch.py')
        ),
        launch_arguments={'rviz': LaunchConfiguration('rviz')}.items(),
    )

    return LaunchDescription([
        rviz_arg,
        sim_launch,
        vision_launch,
        nav_launch,
        autonomy_launch,
    ])
