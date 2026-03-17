#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pkg = get_package_share_directory('skid_steer_robot')

    # LOAD SECOND FILE'S WORLD
    world_file = os.path.join(pkg, 'worlds', 'new_world', 'new_world copy.world')
    model_path = os.path.join(pkg, 'worlds', 'new_world', 'models')
    

    # UPDATE GAZEBO MODEL PATH
    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path += os.pathsep + os.environ['GAZEBO_MODEL_PATH']

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=model_path
    )

    # ROBOT DESCRIPTION
    xacro_file = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={"scale": os.getenv("SCALE", "1.0")})
    robot_description = {"robot_description": doc.toxml()}

    # SPAWN ROBOT
    robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'skid_steer_robot',
            '-x', '-6.18',
            '-y', '10.3',
            '-z', '0.1',
            '-R', '0',
            '-P', '0',
            '-Y', '0'
        ],
        output='screen'
    )

    # STATE PUBLISHER
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': True}
        ]
    )

    # FINISH LINE NODE
    x1_arg = LaunchConfiguration('x1', default='-10.0')
    y1_arg = LaunchConfiguration('y1', default='-10.0')
    x2_arg = LaunchConfiguration('x2', default='10.0')
    y2_arg = LaunchConfiguration('y2', default='-10.0')

    finish_line_node = Node(
        package='skid_steer_robot',
        executable='finish_line.py',
        name='finish_line_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'x1': x1_arg},
            {'y1': y1_arg},
            {'x2': x2_arg},
            {'y2': y2_arg}
        ]
    )

    # GAZEBO GUI ARGUMENT
    gui_arg = LaunchConfiguration('gui')

    gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Run Gazebo with GUI'
    )

    # GAZEBO LAUNCH
    launch_gazebo_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_file,
            'gui': gui_arg
        }.items()
    )

    return LaunchDescription([
        gui_cmd,
        set_gazebo_model_path,
        robot_state_publisher_node,
        launch_gazebo_action,
        robot_node,
        finish_line_node
    ])