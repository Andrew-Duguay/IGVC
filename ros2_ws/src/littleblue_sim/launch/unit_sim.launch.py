#!/usr/bin/env python3
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable, 
                            DeclareLaunchArgument, OpaqueFunction,
                            ExecuteProcess, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def launch_setup(context, *args, **kwargs):
    # 1. GATHER ALL PATHS NEEDED
    pkg_path = get_package_share_directory('littleblue_sim')

    world_name = LaunchConfiguration('world').perform(context)
    world_dir_path = os.path.join(pkg_path, 'worlds', 'unit_sims', world_name)
    world_file_path = os.path.join(world_dir_path, f"{world_name}.world")

    config_path = os.path.join(world_dir_path, 'config.yaml')

    robot_file_path = os.path.join(pkg_path, 'urdf', 'littleblue.urdf.xacro')

    shared_models_path = os.path.join(pkg_path, 'models') 
    world_floor_path = os.path.join(world_dir_path, 'custom_models') 
    
    # 2. PARSE FILES FOR LAUNCH DATA
    try:
        with open(config_path, 'r') as file:
            config_data = yaml.safe_load(file)
    except FileNotFoundError:
        print(f"\n[WARNING] No config.yaml found at {config_path}! Defaulting to 0.0.\n")
        config_data = {
            'spawn': {'x': '0.0', 'y': '0.0', 'z': '0.1', 'R': '0.0', 'P': '0.0', 'Y': '0.0'},
            'finish': {'x1': '0.0', 'y1': '0.0', 'x2': '0.0', 'y2': '0.0'}
        }
    robot_position = config_data.get('spawn', {})
    finish_line_points = config_data.get('finish', {})

    # 3. SET ENVIRONMENT VARIABLES   
    gz_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}:{shared_models_path}:{world_floor_path}"
    )  
    ign_env = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=f"{os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')}:{shared_models_path}:{world_floor_path}"
    )

    # 4. LAUNCH GAZEBO
    gui = LaunchConfiguration('gui').perform(context).lower() 
    if gui in ["true", "True", 1]:
        gui_arg = ""  
    else:
        gui_arg = "-s "

    speed = LaunchConfiguration('speed').perform(context)
    try:
        speed_factor = float(speed)
        target_hz = int(speed_factor * 1000)
    except ValueError:
        print(f"\n[WARNING] Invalid speed '{speed_str}'. Defaulting to 1000 Hz.\n")
        target_hz = 1000

    gz_arguments = f"{world_file_path} -r " + f"{gui_arg}" + f"-z {target_hz}"           
    launch_gazebo_action = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': gz_arguments
            }.items()
        )

    step_size_str = LaunchConfiguration('step_size').perform(context)
    rtf_override_action = TimerAction(
        period=2.0, 
        actions=[
            ExecuteProcess(
                cmd=[
                    'ign', 'service', '-s', f'/world/{world_name}/set_physics',
                    '--reqtype', 'ignition.msgs.Physics',
                    '--reptype', 'ignition.msgs.Boolean',
                    # Inject BOTH the step size and the RTF override here
                    '--req', f'max_step_size: {step_size_str} real_time_factor: {speed_factor}',
                    '--timeout', '3000'
                ],
                output='screen'
            )
        ]
    )

    # 5. LAUNCH THE ROBOT STATE PUBLISHER (BLUEPRINT OF ROBOT)
    doc = xacro.process_file(robot_file_path, mappings={"scale": os.getenv("SCALE", "1.0")})
    robot_description = {"robot_description": doc.toxml()}
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description, 
            {'use_sim_time': True}
        ]
    )

    # 6. SPAWN ROBOT IN GAZEBO
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

    # 7. CREATE BRIDGE BETWEEN ROS AND GAZEBO
    gz_chassis_topic = f"/world/{world_name}/model/littleblue_sim/link/base_footprint/sensor/chassis_sensor/contact"
    gz_caster_topic = f"/world/{world_name}/model/littleblue_sim/link/caster_link/sensor/caster_sensor/contact"
    gz_left_wheel_topic = f"/world/{world_name}/model/littleblue_sim/link/left_wheel_link/sensor/left_wheel_sensor/contact"
    gz_right_wheel_topic = f"/world/{world_name}/model/littleblue_sim/link/right_wheel_link/sensor/right_wheel_sensor/contact"
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            # Timing and Transforms
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            # Drive and Odometry
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            # Cameras
            '/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/depth/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            # Environment Sensors
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/gps/fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
            # Contact Sensors
            f"{gz_chassis_topic}@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",
            f"{gz_caster_topic}@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",
            f"{gz_left_wheel_topic}@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",
            f"{gz_right_wheel_topic}@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts"
        ],       
        # Contact Sensors: Remap for cleanliness
        remappings=[
            (gz_chassis_topic, '/chassis_sensor'),
            (gz_caster_topic, '/caster_sensor'),
            (gz_left_wheel_topic, '/left_wheel_sensor'),
            (gz_right_wheel_topic, '/right_wheel_sensor')
        ],
        output='screen'
    )

    # 8. LAUNCH "REFEREE". MONITORS SIMULATION FOR COURSE COMPLETION, LINE-CROSSING, AND OBSTACLE COLLISION
    timeout_str = LaunchConfiguration('timeout').perform(context)
    try:
        timeout_duration = float(timeout_str)
    except ValueError:
        print(f"\n[WARNING] Invalid timeout '{timeout_str}'. Defaulting to 30.0s.\n")
        timeout_duration = 30.0
    pass_or_fail_node = Node(
        package='littleblue_sim',
        executable='pass_or_fail.py',
        name='pass_or_fail_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'x1':        float(finish_line_points['x1'])},
            {'y1':        float(finish_line_points['y1'])},
            {'x2':        float(finish_line_points['x2'])},
            {'y2':        float(finish_line_points['y2'])},
            {'start_x':   float(robot_position['x'])},
            {'start_y':   float(robot_position['y'])},
            {'start_yaw': float(robot_position['Y'])},
            {'timeout_limit': timeout_duration}
        ]
    )

    nodes_to_start = [
        gz_env,
        ign_env,
        launch_gazebo_action,
        rtf_override_action
    ]
    world_building_arg = LaunchConfiguration('world_building_mode').perform(context).lower()
    is_world_building = world_building_arg in ["true", "True", "1"]
    if not is_world_building:
        nodes_to_start.extend([
            robot_state_publisher_node, 
            robot_node, 
            ros_gz_bridge, 
            pass_or_fail_node
        ])
    return nodes_to_start
                   
def generate_launch_description():
    
    # 1. DECLARE THE ARGUMENT (Default is just the name, not the path)
    world_arg_decl = DeclareLaunchArgument(
        'world',
        default_value='template',
        description='Name of the simulation world'
    )

    speed_arg_decl = DeclareLaunchArgument(
        'speed',
        default_value='1.0',
        description='Simulation speed multiplier (e.g., 2.0 for double speed, 0 for max unthrottled)'
    )

    gui_arg_decl = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to false to run the simulation without the Gazebo GUI'
    )

    world_building_arg_decl = DeclareLaunchArgument(
        'world_building_mode',
        default_value='false',
        description='Set to true to run the simulation without the robot. This allows easy editing of objects in the simulation'
    )    

    timeout_arg_decl = DeclareLaunchArgument(
        'timeout',
        default_value='30.0',
        description='Simulation time in seconds before failure condition'
    )   

    step_size_arg_decl = DeclareLaunchArgument(
        'step_size',
        default_value='0.001', # Default to standard high-fidelity physics
        description='Physics max step size. Increase (e.g., 0.003) for faster batch simulations.'
    )

    return LaunchDescription([
        world_arg_decl,
        speed_arg_decl,
        gui_arg_decl,
        timeout_arg_decl,
        world_building_arg_decl,
        step_size_arg_decl,
        OpaqueFunction(function=launch_setup)
    ])