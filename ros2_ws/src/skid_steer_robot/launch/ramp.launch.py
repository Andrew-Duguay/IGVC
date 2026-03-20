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
    world_name = "ramp"
    robot_position = {
        'x': '0.5', 
        'y': '4.3',
        'z': '0.1',
        'R': '0.0',
        'P': '0.0',
        'Y': '-1.558'
    }
    finish_line_points = {
        'x1': '3.0',
        'y1': '-3.0',
        'x2': '-1.0',
        'y2': '-3.0'
    }
    pkg = get_package_share_directory('skid_steer_robot')

    # GET PATHS TO WORLD
    world_file = os.path.join(pkg, 'worlds', world_name, f"{world_name}.world")
    model_path = os.path.join(pkg, 'worlds', world_name, 'custom_models')

    # Update GAZEBO_MODEL_PATH TO SEE CUSTOM MODELS
    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path += os.pathsep + os.environ['GAZEBO_MODEL_PATH']
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=model_path
    )

    # DEFINE THE ROBOT NODE
    xacro_file = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={"scale": os.getenv("SCALE", "1.0")})
    robot_description = {"robot_description": doc.toxml()}
    robot_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description', '-entity', 'skid_steer_robot',
                '-x', robot_position['x'],
                '-y', robot_position['y'],
                '-z', robot_position['z'],
                '-R', robot_position['R'],
                '-P', robot_position['P'],
                '-Y', robot_position['Y']
            ],
            output='screen'
        )

    # DEFINE THE SIMULATION NODE
    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                robot_description, 
                {'use_sim_time': True}
            ]
        )
    
    # DEFINE THE FINISH LINE TRACKER NODE (OPENS IN NEW TERMINAL)
    # PUT YOUR FINISH LINE HERE
    x1_arg = LaunchConfiguration('x1', default=finish_line_points['x1'])
    y1_arg = LaunchConfiguration('y1', default=finish_line_points['y1'])
    x2_arg = LaunchConfiguration('x2', default=finish_line_points['x2'])
    y2_arg = LaunchConfiguration('y2', default=finish_line_points['y2'])
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

    # DEFINE THE GAZEBO LAUNCH
    gui_arg = LaunchConfiguration('gui')    # grab value from CLI
    gui_cmd = DeclareLaunchArgument(        # Declare the arguments so the system knows they exist
            'gui',
            default_value='true',
            description='Run the simulation with GUI (true) or without GUI (false)'
        )
    launch_gazebo_action = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': world_file,
                'gui': gui_arg
            }.items()
        )

    return LaunchDescription([
        gui_cmd,
        # Set new path first
        set_gazebo_model_path,
        # Start state publisher
        robot_state_publisher_node,
        # Start Gazebo
        launch_gazebo_action,
        # spawn Robot
        robot_node,
        #launch Finish Line
        finish_line_node
    ])