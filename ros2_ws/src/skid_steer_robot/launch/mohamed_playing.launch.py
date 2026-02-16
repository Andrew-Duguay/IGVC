#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Define the package
    pkg = get_package_share_directory('skid_steer_robot')
    # Point to world file and custom models for world. UPDATE IF CHANGING WORLD
    world_file = os.path.join(pkg, 'worlds', 'mohamed_playing', 'mohamed_playing.world')
    model_path = os.path.join(pkg, 'worlds', 'mohamed_playing')
    # Update GAZEBO_MODEL_PATH to see the new models
    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path += os.pathsep + os.environ['GAZEBO_MODEL_PATH']
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=model_path
    )
    # Define files
    xacro_file = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={"scale": os.getenv("SCALE", "1.0")})
    robot_description = {"robot_description": doc.toxml()}

    return LaunchDescription([
        # Set new path first
        set_gazebo_model_path,
        # Start state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': True}]
        ),
        # Start Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file}.items()
        ),
        # spawn Robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description', '-entity', 'skid_steer_robot',
                '-x', '0.02',
                '-y', '-9',
                '-z', '0.1',
                '-R', '0.0',  # Roll
                '-P', '0.0',  # Pitch
                '-Y', '1.06'
            ],
            output='screen'
        ),
    ])
