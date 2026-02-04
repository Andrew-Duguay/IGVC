import os
import sys
from launch import LaunchDescription
# --- IMPORT FIX ---
# Ensure ExecuteProcess is imported here!
from launch.actions import ExecuteProcess, SetEnvironmentVariable, LogInfo, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
# ------------------
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Configuration
    home_dir = os.environ.get('HOME')
    if not home_dir:
        print("ERROR: $HOME environment variable is not set.")
        sys.exit(1)

    # Paths
    project_root = os.path.join(home_dir, 'gazebo-sim')
    world_file = os.path.join(project_root, 'lane_sim', 'world.sdf')
    
    # Debug Prints (Python side)
    print(f"\n--- DEBUG CONFIGURATION ---")
    print(f"Project Root: {project_root}")
    print(f"World File:   {world_file}")
    
    # Verify file exists
    if not os.path.exists(world_file):
        print(f"CRITICAL ERROR: World file not found at: {world_file}")
        print("Please check your file structure.")
        sys.exit(1)
    else:
        print(f"Status: World file found.")
    print(f"---------------------------\n")

    # 2. Environment Variable
    # This sets the variable ONLY for processes launched below this line
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=project_root
    )

    # 3. DEBUG VERIFICATION
    # This executes 'echo' inside the launch environment to prove the variable is set
    debug_print = ExecuteProcess(
        cmd=['echo', '--- PROOF: GZ_SIM_RESOURCE_PATH inside launch is:', '$GZ_SIM_RESOURCE_PATH'],
        shell=True,
        output='screen'
    )

    # 4. Launch Gazebo (DIRECT MODE)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_file],
        output='screen'
    )

    # 5. ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        shell=True,
        output='screen'
    )

    # 6. AUTO-SHUTDOWN HANDLER
    # This tells ROS: "When gz_sim exits, shut down the entire launch file"
    shutdown_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_sim,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )

    return LaunchDescription([
        set_resource_path,
        debug_print, # Run the proof
        gz_sim,
        bridge,
        shutdown_handler, # Register the handler
    ])