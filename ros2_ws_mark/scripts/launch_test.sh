#!/bin/bash
# Launch full autonav test: sim + fake_lanes + autonomy
# Usage: bash scripts/launch_test.sh
# Kill:  pkill -9 -f "gz|ros2|ruby|fake_lanes"

source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/ulate/Desktop/ros2_ws_mark/install/setup.bash

# Clean stale DDS shared memory
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null

echo "=== Starting sim + RViz ==="
ros2 launch littleblue_sim sim.launch.py &
sleep 15

echo "=== Starting fake_lanes ==="
python3 /mnt/c/Users/ulate/Desktop/ros2_ws_mark/scripts/fake_lanes_autonav.py &
sleep 3

echo "=== Starting autonomy ==="
ros2 launch littleblue_autonomy autonomy.launch.py rviz:=true approach:=A &

echo "=== All launched. Press Ctrl+C to stop. ==="
wait
