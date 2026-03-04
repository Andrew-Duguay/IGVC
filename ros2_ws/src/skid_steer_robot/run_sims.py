#!/usr/bin/env python3
import argparse
import subprocess
import time
import os
import signal
import sys
import select
import re
import shutil

# -----------------------------
# Argument Parser
# -----------------------------
parser = argparse.ArgumentParser(description="Run batch simulations")
parser.add_argument('--speed', type=float, default=1.0,
                    help="Simulation speed multiplier (default=1.0)")
args = parser.parse_args()

# -----------------------------
# Paths
# -----------------------------
WORLD_PATH = os.path.expanduser(
    "~/ros_ws/IGVC/ros2_ws/install/skid_steer_robot/share/skid_steer_robot/worlds/mohamed_playing/mohamed_playing.world"
)
BACKUP_PATH = WORLD_PATH + ".backup"

# -----------------------------
# Backup Original World
# -----------------------------
shutil.copy(WORLD_PATH, BACKUP_PATH)

# -----------------------------
# Modify Update Rate
# -----------------------------
with open(WORLD_PATH, "r") as f:
    world_content = f.read()

new_rate = int(1000 * args.speed)

world_content = re.sub(
    r"<real_time_update_rate>\d+</real_time_update_rate>",
    f"<real_time_update_rate>{new_rate}</real_time_update_rate>",
    world_content
)

with open(WORLD_PATH, "w") as f:
    f.write(world_content)

print(f"\nUsing update rate: {new_rate}")

# -----------------------------
# Simulation Config
# -----------------------------
simulations = [
    {
        'id': 1,
        'cmd': [
            'ros2',
            'launch',
            'skid_steer_robot',
            'mohamed_playing.launch.py',
            'gui:=True'
        ]
    },
]

TIMEOUT_SECONDS = 60
SUCCESS_PHRASE = "[SUCCESS] Course completed."

print("="*40)
print("🤖 BATCH SIMULATION RUNNER STARTED")
print("="*40)

for sim in simulations:
    print(f"Starting simulation {sim['id']}...", end='', flush=True)

    start_time = time.time()
    status = "[FAILURE] TIMEOUT"
    elapsed = TIMEOUT_SECONDS

    process = subprocess.Popen(
        sim['cmd'],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid
    )

    try:
        print("start\n")

        while True:
            if (time.time() - start_time) > TIMEOUT_SECONDS:
                break

            ready, _, _ = select.select([process.stdout], [], [], 0.5)

            if ready:
                line = process.stdout.readline()
                if not line:
                    break

                if SUCCESS_PHRASE in line:
                    status = "[SUCCESS]"
                    elapsed = int(time.time() - start_time)
                    break

            if process.poll() is not None:
                break

    except KeyboardInterrupt:
        print("\nPipeline manually aborted by user.")
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        sys.exit(1)

    finally:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait()

        print(f"{status} IN {elapsed}s")
        time.sleep(3)

# -----------------------------
# Restore Original World
# -----------------------------
shutil.move(BACKUP_PATH, WORLD_PATH)

print("="*40)
print("✅ ALL SIMULATIONS COMPLETE")
print("="*40)