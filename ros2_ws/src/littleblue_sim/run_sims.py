#!/usr/bin/env python3
import argparse
import subprocess
import time
import os
import signal
import sys
import select
import re
import tempfile
from ament_index_python.packages import get_package_share_directory

# Parse the speed multiplier
parser = argparse.ArgumentParser(description="Run batch simulations")
parser.add_argument('--speed', type=float, default=1.0,
                    help="Simulation speed multiplier (default=1.0)")
args = parser.parse_args()
new_rate = int(1000 * args.speed)

# -----------------------------
# Dynamic Paths & Safe Temp File
# -----------------------------
# 1. Get path dynamically
pkg_share = get_package_share_directory('littleblue_sim')

# -----------------------------
# Simulation Config
# -----------------------------
simulations = [
    {
        'id': 1,
        'name': 'ramp',
        'TIMEOUT': 30,        
    },
    {
        'id': 2,
        'name': 'empty_straight_lane',
        'TIMEOUT': 30,        
    },
    {
        'id': 3,
        'name': 'straight_lane_obstacles1',
        'TIMEOUT': 30,        
    },
    {
        'id': 4,
        'name': 'straight_lane_obstacles2',
        'TIMEOUT': 30,        
    },
    {
        'id': 5,
        'name': 'straight_lane_obstacles3',
        'TIMEOUT': 30,        
    },
    {
        'id': 6,
        'name': 'straight_lane_obstacles4',
        'TIMEOUT': 30,        
    },
]

SUCCESS_PHRASE = "[SUCCESS] Course completed."

print("="*40)
print("🚀🚀🚀 BATCH SIMULATIONS STARTED 🚀🚀🚀")
print("="*40)

for sim in simulations:
    status = ""
    # 1. Get world file path
    world_path = os.path.join(pkg_share, 'worlds', sim['name'], f"{sim['name']}.world")

    # 2. Read the original src world file in buffer
    with open(world_path, "r") as f:
        world_content = f.read()

    # 3. Replace time update rate in buffer
    world_content = re.sub(
        r"<real_time_update_rate>\d+</real_time_update_rate>",
        f"<real_time_update_rate>{new_rate}</real_time_update_rate>",
        world_content
    )

    # 4. Create a temporary world file with updated buffer value
    temp_world = tempfile.NamedTemporaryFile(delete=False, suffix='.world', mode='w')
    temp_world.write(world_content)
    temp_world.close()

    # 5. Launch sim using temp world file, not original
    sim_cmd = [
            'ros2', 'launch', 'skid_steer_robot', f"{sim['name']}.launch.py",
            'gui:=false',
            f"world:={temp_world.name}"  # Pass the temp file to the launch file
        ]

    # 6. Launch the robot
    robot_cmd = [
            'ros2', 'launch', 'startup_robot', "robot.launch.py"
        ]

    print(f"🔄 Simulation {sim['id']} Running...", end='', flush=True)
    start_time = time.time()

    sim_process = subprocess.Popen(
        sim_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
        env=os.environ.copy() 
    )
    time.sleep(3)
    robot_process = subprocess.Popen(
        robot_cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
        env=os.environ.copy() 
    )

    try:
        # 6. Non-Blocking Read Logic
        while True:
            # Check timeout
            if (time.time() - start_time) > sim['TIMEOUT']:
                print(f"\r❌ Simulation {sim['id']} Failed to complete in {sim['TIMEOUT']}s\033[K")
                break

            # Wait 0.5s for new terminal output
            ready, _, _ = select.select([sim_process.stdout], [], [], 0.5)

            if ready:
                line = sim_process.stdout.readline()
                
                # If readline returns empty, Gazebo crashed or closed
                if not line:
                    print(f"\r💥 Simulation {sim['id']} crashed after {time.time() - start_time}s\033[K")
                    break

                if SUCCESS_PHRASE in line:
                    print(f"\r✅ Simulation {sim['id']} completed in {time.time() - start_time}s\033[K")
                    break

    except KeyboardInterrupt:
        print("\nPipeline manually aborted by user.")
        os.killpg(os.getpgid(sim_process.pid), signal.SIGINT)
        os.killpg(os.getpgid(robot_process.pid), signal.SIGINT)
        os.remove(temp_world.name) # Clean up temp file
        sys.exit(1)
    
    finally:
    # complete teardown
        try:
            os.killpg(os.getpgid(sim_process.pid), signal.SIGINT)
        except ProcessLookupError:
            pass # Already dead
        
        try:
            os.killpg(os.getpgid(robot_process.pid), signal.SIGINT)
        except ProcessLookupError:
            pass # Already dead
        sim_process.wait()
        robot_process.wait()
        time.sleep(3)
    # 7. Clean up the temp file
    os.remove(temp_world.name)

        

print("="*40)
print("🏆🏆🏆 ALL SIMULATIONS COMPLETE 🏆🏆🏆")
print("="*40)