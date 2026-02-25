#!/usr/bin/env python3
import subprocess
import time
import os
import signal
import sys


# Define the simulations you want to run
simulations = [
    {
        'id': 1, 
        'cmd': ['ros2', 'launch', 'skid_steer_robot', 'mohamed_playing.launch.py', 'gui:=false']
    },
    # {
    #     'id': 2, 
    #     'cmd': ['ros2', 'launch', 'skid_steer_robot', 'world2.launch.py', 'gui:=false']
    # },

]

TIMEOUT_SECONDS = 2
SUCCESS_PHRASE = "[SUCCESS] Course completed."

print("="*40)
print("🤖 BATCH SIMULATION RUNNER STARTED")
print("="*40)

for sim in simulations:
    print(f"Starting simulation {sim['id']}...", end='', flush=True)
    
    start_time = time.time()
    status = "[FAILURE] TIMEOUT"
    elapsed = TIMEOUT_SECONDS
    
    # Spawn the ROS 2 launch process in its own process group
    process = subprocess.Popen(
        sim['cmd'],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid  # Groups the launch file and Gazebo together
    )
    
    try:
        # Read the terminal output line by line in real-time
        print("start\n")
        for line in process.stdout:
            # 1. Check for success
            if SUCCESS_PHRASE in line:
                status = "[SUCCESS]"
                elapsed = int(time.time() - start_time)
                break
            
            # 2. Check for timeout
            if (time.time() - start_time) > TIMEOUT_SECONDS:
                break
                
    except KeyboardInterrupt:
        print("\nPipeline manually aborted by user.")
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        sys.exit(1)
        
    finally:
        # Cleanly kill the entire process group (Ctrl+C equivalent)
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait() # Wait for Gazebo to fully close
        
        # Print the exact formatted string you requested
        print(f"{status} IN {elapsed}s")
        
        # Give ROS a moment to free up ports before starting the next sim
        time.sleep(3)

print("="*40)
print("✅ ALL SIMULATIONS COMPLETE")
print("="*40)