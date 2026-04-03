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

parser = argparse.ArgumentParser(description="Run single simulation")
parser.add_argument('--world', type=str, default="template",
                    help="Name of the simulation to launch")
parser.add_argument('--gui', type=str, default="true",
                    help="true or false")
parser.add_argument('--speed', type=float, default=1.0,
                    help="Simulation speed multiplier (default=1.0)")
args = parser.parse_args()
new_rate = int(1000 * args.speed)
world_name = args.world
# 1. Get path dynamically (No hardcoded paths!)
pkg_share = get_package_share_directory('littleblue_sim')

# 1. Get world file path
world_path = os.path.join(pkg_share, 'worlds', world_name, f"{world_name}.world")
if args.speed != 1.0:
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
    world_path = temp_world.name

    # 5. Launch sim using temp world file, not original
sim_cmd = [
    'ros2', 'launch', 'littleblue_sim', f"{world_name}.launch.py",
    f"gui:={args.gui}",
    f"world:={world_path}" 
    ]

# 6. Launch the robot
robot_cmd = [
        'ros2', 'launch', 'startup_robot', "robot.launch.py"
    ]

print(f"[INFO] Simulation launched")

sim_process = subprocess.Popen(
    sim_cmd,
    # stdout=subprocess.STDOUT,
    # stderr=subprocess.STDOUT,
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
    print("Press ctrl+C to abort simulation")
    start_time = time.time()
    while True:
        if(time.time() - start_time) > 3000:
            break

except KeyboardInterrupt:
    print("\nSimulation manually aborted by user.")
    os.killpg(os.getpgid(sim_process.pid), signal.SIGINT)
    os.killpg(os.getpgid(robot_process.pid), signal.SIGINT)
    if args.speed != 1.0:
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
if args.speed != 1.0:
    os.remove(temp_world.name)