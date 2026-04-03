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
    world_name = "chicane1"
    robot_position = {
        'x': '3.8', 
        'y': '-4.0', 
        'z': '0.1', 
        'R': '0.0', 
        'P': '0.0', 
        'Y': '2.43'
    }
    finish_line_points = {
        'x1': '-3.6', 
        'y1': '-0.7',
        'x2': '-0.6' , 
        'y2': '2.0'
    }
    pkg = get_package_share_directory('littleblue_sim')

### DEFINE THE WORLD FILE USED ### 
    # GET PATH TO WORLD
    default_world_file = os.path.join(pkg, 'worlds', world_name, f"{world_name}.world")
    # DECLARE WORLD ARGUMENT (IN CASE ALT WORLD FILE SPECIFIED)
    world_arg_decl = DeclareLaunchArgument(
        'world',
        default_value=default_world_file,
        description='Full path to the world file'
    )
    world_file_config = LaunchConfiguration('world')

#### SET ENVIRNOMENT VARIABLE ####
    # GET PATH TO MODELS
    model_path = os.path.join(pkg, 'models') 
    floor_path = os.path.join(pkg, 'worlds', world_name, 'custom_models')   
    # COMBINE THEM
    combined_model_path = f"{model_path}:{floor_path}"
    # UPDATE ENVIRONMENT VARIABLE FOR IGNITION
    current_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if current_resource_path:
        final_resource_path = f"{current_resource_path}:{combined_model_path}"
    else:
        final_resource_path = combined_model_path
    # TELL ROS TO USE IT
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=final_resource_path
    )

    # DEFINE THE ROBOT NODE
    xacro_file = os.path.join(pkg, 'urdf', 'littleblue.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={"scale": os.getenv("SCALE", "1.0")})
    robot_description = {"robot_description": doc.toxml()}
    
    # 3. UPDATE ROBOT SPAWNER FOR IGNITION
    # gazebo_ros 'spawn_entity.py' becomes ros_gz_sim 'create'
    robot_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description', 
                '-name', 'littleblue_sim',
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
    
    # DEFINE THE pass_or_fail TRACKER NODE
    x1_arg = LaunchConfiguration('x1', default=finish_line_points['x1'])
    y1_arg = LaunchConfiguration('y1', default=finish_line_points['y1'])
    x2_arg = LaunchConfiguration('x2', default=finish_line_points['x2'])
    y2_arg = LaunchConfiguration('y2', default=finish_line_points['y2'])
    pass_or_fail_node = Node(
        package='littleblue_sim',
        executable='pass_or_fail.py',
        name='pass_or_fail_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'x1': x1_arg},
            {'y1': y1_arg},
            {'x2': x2_arg},
            {'y2': y2_arg},
            {'start_x': float(robot_position['x'])},
            {'start_y': float(robot_position['y'])},
            {'start_yaw': float(robot_position['Y'])}
        ]
    )

    # 4. ADD THE CLOCK BRIDGE (MANDATORY)
    # Because you use 'use_sim_time': True, ROS 2 needs Gazebo's clock.
    # Without this bridge, your nodes will wait forever and do nothing!
    gz_chassis_topic = f"/world/{world_name}/model/littleblue_sim/link/base_footprint/sensor/chassis_sensor/contact"
    gz_caster_topic = f"/world/{world_name}/model/littleblue_sim/link/caster_link/sensor/caster_sensor/contact"
    gz_left_wheel_topic = f"/world/{world_name}/model/littleblue_sim/link/left_wheel_link/sensor/left_wheel_sensor/contact"
    gz_right_wheel_topic = f"/world/{world_name}/model/littleblue_sim/link/right_wheel_link/sensor/right_wheel_sensor/contact"

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            
            # Bridge the contact sensors using f-strings
            f"{gz_chassis_topic}@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",
            f"{gz_caster_topic}@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",
            f"{gz_left_wheel_topic}@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",
            f"{gz_right_wheel_topic}@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts"
        ],
        # REMAP TO YOUR CLEAN PYTHON TOPICS
        remappings=[
            (gz_chassis_topic, '/chassis_sensor'),
            (gz_caster_topic, '/caster_sensor'),
            (gz_left_wheel_topic, '/left_wheel_sensor'),
            (gz_right_wheel_topic, '/right_wheel_sensor')
        ],
        output='screen'
    )

    # 5. UPDATE GAZEBO LAUNCH ACTION FOR IGNITION
    launch_gazebo_action = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                # Pass the world file and the '-r' flag to run immediately--render-engine ogre
                'gz_args': [world_file_config, ' -r '] 
            }.items()
        )

    return LaunchDescription([
        world_arg_decl,
        # Set new path first
        set_gazebo_model_path,
        # Start state publisher
        robot_state_publisher_node,
        # Start Gazebo
        launch_gazebo_action,
        # Start Bridge
        ros_gz_bridge,
        # spawn Robot
        robot_node,
        # launch pass_or_fail
        pass_or_fail_node
    ])