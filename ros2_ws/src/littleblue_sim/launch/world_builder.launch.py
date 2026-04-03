#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
import xacro
                
def generate_launch_description():
    pkg = get_package_share_directory('littleblue_sim')

    # 1. DECLARE THE ARGUMENT (Default is just the name, not the path)
    world_arg_decl = DeclareLaunchArgument(
        'world',
        default_value='template',
        description='Name of the simulation world folder and file'
    )
    world_name = LaunchConfiguration('world')

    # 2. BUILD THE WORLD FILE PATH
    # PathJoin Substitution builds the directories with slashes: /pkg/worlds/ramp/ramp
    # Putting it in a list with '.sdf' concatenates them directly: ramp.sdf
    world_file = [
        PathJoinSubstitution([pkg, 'worlds', world_name, world_name]),
        '.sdf'
    ]

    #### SET ENVIRONMENT VARIABLE ####
    # GET PATH TO MODELS (os.path.join is safe here because there is no world_name)
    model_path = os.path.join(pkg, 'models') 
    
    # BUILD DYNAMIC FLOOR PATH (Must use PathJoinSubstitution)
    floor_path = PathJoinSubstitution([pkg, 'worlds', world_name, 'custom_models'])   
    
    # UPDATE ENVIRONMENT VARIABLE FOR IGNITION
    # The 'value' array dynamically stitches the strings together at execution time
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            ':', model_path,
            ':', floor_path
        ]
    )

    # 5. UPDATE GAZEBO LAUNCH ACTION FOR IGNITION
    launch_gazebo_action = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                # Pass the '-r' flag to run immediately, followed by the world file
                'gz_args': [
                    '-r ', 
                    PathJoinSubstitution([pkg, 'worlds', world_name, world_name]), 
                    '.sdf'
                ]
            }.items()
        )

    return LaunchDescription([
        # Must return the argument declaration so the CLI knows it exists
        world_arg_decl,
        # Set new path first
        set_gazebo_model_path,
        # Start Gazebo
        launch_gazebo_action
    ])