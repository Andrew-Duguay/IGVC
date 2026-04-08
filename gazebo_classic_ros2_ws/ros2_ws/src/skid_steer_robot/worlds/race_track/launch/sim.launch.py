import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_dir = get_package_share_directory('littleblue_sim')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    # Paths
    urdf_file = os.path.join(pkg_dir, 'urdf', 'littleblue.urdf.xacro')
    world_file = os.path.join(pkg_dir, 'worlds', 'igvc_course.sdf')
    bridge_config = os.path.join(pkg_dir, 'config', 'bridge.yaml')
    models_dir = os.path.join(pkg_dir, 'models')
    worlds_dir = os.path.join(pkg_dir, 'worlds')

    # Process xacro
    robot_description = xacro.process_file(urdf_file).toxml()

    # Launch arguments
    headless = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Ignition headless (no GUI)'
    )

    # Environment variables
    resource_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        ':'.join([models_dir, worlds_dir,
                  os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')])
    )

    # Use OGRE 1.x — ogre2 fails without DISPLAY on headless Jetson
    render_engine = SetEnvironmentVariable(
        'IGN_RENDERING_ENGINE', 'ogre'
    )

    # Ignition Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
        }.items(),
    )

    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                      'use_sim_time': True}],
        output='screen',
    )

    # Spawn robot into Ignition
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'littleblue',
            '-topic', '/robot_description',
            '-x', '-4', '-y', '0', '-z', '0.15',
        ],
        output='screen',
    )

    # ros_gz_bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config,
                      'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        headless,
        resource_path,
        render_engine,
        gz_sim,
        robot_state_pub,
        spawn_robot,
        bridge,
    ])
