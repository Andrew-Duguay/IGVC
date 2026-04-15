#!/usr/bin/env python3
import argparse
import subprocess
import time
import os
import signal
import sys
import select

parser = argparse.ArgumentParser(description="Run single simulation")
parser.add_argument('--world', type=str, default="template",
                    help="Name of the simulation to launch")
parser.add_argument('--gui', type=str, default="true",
                    help="Show gui during batch simulation (default=true)")
parser.add_argument('--speed', type=float, default=1.0,
                    help="Simulation speed multiplier (default=1.0)")
parser.add_argument('--timeout', type=float, default=30.0,
                    help="Timeout for failure condition (default=30)")
parser.add_argument('--step_size', type=float, default=0.001,
                    help="Step size of simulation time (default=0.001)")
args = parser.parse_args()

sim_cmd = [
        'ros2', 'launch', 'littleblue_sim', 'unit_sim.launch.py',
        f"gui:={args.gui}",
        f"world:={args.world}",
        f"speed:={args.speed}",
        f"timeout:={args.timeout}",
        f"step_size:={args.step_size}",
    ]

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