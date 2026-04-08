#!/usr/bin/env python3
import argparse
import subprocess
import time
import os
import signal
import sys
import select

parser = argparse.ArgumentParser(description="Run batch simulations")
parser.add_argument('--speed', type=float, default=1.0,
                    help="Simulation speed multiplier (default=1.0)")
parser.add_argument('--gui', type=str, default="false",
                    help="Show gui during batch simulation (default=false)")
parser.add_argument('--step_size', type=str, default=0.001,
                    help="Sets the interval between simulation time calculations (default=0.001)")
args = parser.parse_args()

simulations = [
    {'name': 'ramp', 'timeout': 30.0},
    {'name': 'empty_straight_lane', 'timeout': 30.0},
    {'name': 'straight_lane_obstacles1', 'timeout': 30.0},
    {'name': 'straight_lane_obstacles2', 'timeout': 30.0},
    {'name': 'straight_lane_obstacles3', 'timeout': 30.0},
    {'name': 'straight_lane_obstacles4', 'timeout': 30.0},
    {'name': 'chicane1', 'timeout': 45.0},
]

SUCCESS_PHRASE = "[SUCCESS]"
FAILURE_PHRASE = "[FAILURE]"

print("="*40)
print("🚀🚀🚀 BATCH SIMULATIONS STARTED 🚀🚀🚀")
print("="*40)

for sim in simulations:
    sim_name = sim['name']
    sim_timeout = sim['timeout']

    sim_cmd = [
            'ros2', 'launch', 'littleblue_sim', 'unit_sim.launch.py',
            f"gui:={args.gui}",
            f"world:={sim_name}",
            f"speed:={args.speed}",
            f"timeout:={sim_timeout}",
            f"step-size:={args.step_size}",
        ]

    robot_cmd = [
            'ros2', 'launch', 'startup_robot', "robot.launch.py"
        ]

    print(f"🔄 Simulating {sim_name}", end='', flush=True)

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
            # Wait 0.5s for new terminal output
            ready, _, _ = select.select([sim_process.stdout], [], [], 0.5)
            if ready:
                line = sim_process.stdout.readline()                
                # If readline returns empty, Gazebo crashed or closed
                if not line:
                    print(f"\r💥 Simulation {sim_name} crashed\033[K")
                    break
                if SUCCESS_PHRASE in line:
                    print(f"\r✅ Simulation {sim_name} passed\033[K")
                    break
                if FAILURE_PHRASE in line:
                    clean_line = line.strip()
                    print(f"\r❌ {clean_line}\033[K")
                    break
    except KeyboardInterrupt:
        print("\nPipeline manually aborted by user.")
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

        

print("="*40)
print("🏆🏆🏆 ALL SIMULATIONS COMPLETE 🏆🏆🏆")
print("="*40)