#!/usr/bin/env python3
import argparse
import subprocess
import time
import os
import signal
import sys
import select

parser = argparse.ArgumentParser(description="Run batch simulations")
parser.add_argument('--speed', type=float, default=1.0, help="Simulation speed multiplier")
args = parser.parse_args()

os.environ['SIM_SPEED'] = str(args.speed)

simulations = [
    {
        'id': 1, 
        'cmd': ['ros2', 'launch', 'skid_steer_robot', 'mohamed_playing.launch.py', 'gui:=true']
    },
    {
        'id': 2, 
        'cmd': ['ros2', 'launch', 'skid_steer_robot', 'custom_world_template.launch.py', 'gui:=true']
    },
    {
        'id': 3, 
        'cmd': ['ros2', 'launch', 'skid_steer_robot', 'race_track.launch.py', 'gui:=true']
    },
    {
        'id': 4, 
        'cmd': ['ros2', 'launch', 'skid_steer_robot', 'small_course.launch.py', 'gui:=true']
    },
]

TIMEOUT_SECONDS = 60
SUCCESS_PHRASE = "[SUCCESS] Course completed"

print("="*40)
print(f"BATCH SIMULATION RUNNER STARTED (SPEED: {args.speed}x)")
print("="*40)

for sim in simulations:
    print(f"Starting simulation {sim['id']}...")
    
    start_time = time.time()
    status = "[FAILURE] CRASHED OR EXITED EARLY"
    elapsed = 0
    
    process = subprocess.Popen(
        sim['cmd'],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid
    )
    
    try:
        print("Waiting for simulation to complete...\n")
        print("--- ROS 2 OUTPUT ---")
        
        while True:
            current_elapsed = time.time() - start_time
            if current_elapsed > TIMEOUT_SECONDS:
                status = "[FAILURE] TIMEOUT"
                elapsed = int(current_elapsed)
                break
                
            ready, _, _ = select.select([process.stdout], [], [], 0.5)
            if ready:
                line = process.stdout.readline()
                if not line:
                    elapsed = int(current_elapsed)
                    break
                
                print(line, end='')
                    
                if SUCCESS_PHRASE in line:
                    status = "[SUCCESS]"
                    elapsed = int(current_elapsed)
                    break

            if process.poll() is not None:
                elapsed = int(current_elapsed)
                break
                
    except KeyboardInterrupt:
        print("\nPipeline manually aborted by user.")
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        sys.exit(1)
        
    finally:
        print("\n--------------------")
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        process.wait()
        
        print(f"{status} IN {elapsed}s (Real-world time)")
        time.sleep(3)

print("="*40)
print("ALL SIMULATIONS COMPLETE")
print("="*40)