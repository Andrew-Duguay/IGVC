import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_setup(context):
    """Resolve course selection and return launch actions."""
    pkg_dir = get_package_share_directory('littleblue_sim')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    course = LaunchConfiguration('course').perform(context)

    # Select world file and spawn position based on course
    if course == 'oval':
        world_file = os.path.join(pkg_dir, 'worlds', 'igvc_course.sdf')
        spawn_x, spawn_y, spawn_z, spawn_yaw = '-4', '0', '0.15', '0'
    elif course == 'autonav':
        world_file = os.path.join(pkg_dir, 'worlds', 'igvc_autonav.sdf')
        spawn_x, spawn_y, spawn_z, spawn_yaw = '-10', '2', '0.15', '1.5708'
    else:  # autonav2 (default)
        world_file = os.path.join(pkg_dir, 'worlds', 'igvc_autonav2.sdf')
        spawn_x, spawn_y, spawn_z, spawn_yaw = '-10', '2', '0.15', '1.5708'

    # Paths
    urdf_file = os.path.join(pkg_dir, 'urdf', 'littleblue.urdf.xacro')
    bridge_config = os.path.join(pkg_dir, 'config', 'bridge.yaml')
    models_dir = os.path.join(pkg_dir, 'models')
    worlds_dir = os.path.join(pkg_dir, 'worlds')

    # Process xacro
    robot_description = xacro.process_file(urdf_file).toxml()

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
            '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
            '-Y', spawn_yaw,
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

    return [gz_sim, robot_state_pub, spawn_robot, bridge]


def generate_launch_description():
    pkg_dir = get_package_share_directory('littleblue_sim')
    models_dir = os.path.join(pkg_dir, 'models')
    worlds_dir = os.path.join(pkg_dir, 'worlds')

    # Launch arguments
    course_arg = DeclareLaunchArgument(
        'course', default_value='autonav',
        description='Course to load: oval or autonav'
    )

    headless = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Ignition headless (no GUI)'
    )

    # Environment variables — set both IGN_ (Fortress compat) and GZ_ (Harmonic)
    resource_path_ign = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        ':'.join([models_dir, worlds_dir,
                  os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')])
    )
    resource_path_gz = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        ':'.join([models_dir, worlds_dir,
                  os.environ.get('GZ_SIM_RESOURCE_PATH', '')])
    )

    return LaunchDescription([
        course_arg,
        headless,
        resource_path_ign,
        resource_path_gz,
        OpaqueFunction(function=launch_setup),
    ])
