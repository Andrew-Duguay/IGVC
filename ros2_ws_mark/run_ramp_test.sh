#!/bin/bash
# Launch the full ramp test stack: sim + fake_lanes + TF + autonomy
# Usage: ./run_ramp_test.sh [headless]
#   headless (default): no Gazebo GUI
#   gui:               with Gazebo GUI

set -e

HEADLESS="${1:-headless}"
if [ "$HEADLESS" = "gui" ]; then
    HEADLESS_ARG="false"
else
    HEADLESS_ARG="true"
fi

# Source ROS and workspace
source /opt/ros/humble/setup.bash
source /home/mark/ros2_ws/install/setup.bash

# Kill any existing instances
echo "Cleaning up old processes..."
pkill -9 -f "ign gazebo" 2>/dev/null || true
pkill -9 -f "ign-transport" 2>/dev/null || true
pkill -9 -f "fake_lanes" 2>/dev/null || true
pkill -9 -f "lane_follower" 2>/dev/null || true
pkill -9 -f "static_transform_publisher.*lidar_link" 2>/dev/null || true
pkill -9 -f "parameter_bridge" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pkill -9 -f "ros2 launch littleblue" 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
sleep 2

# Trap Ctrl-C to kill all children
cleanup() {
    echo ""
    echo "Shutting down..."
    kill 0 2>/dev/null
    wait 2>/dev/null
    echo "Done."
    exit 0
}
trap cleanup SIGINT SIGTERM

# 1. Launch sim
echo "Starting simulation (headless=$HEADLESS_ARG)..."
ros2 launch littleblue_sim sim.launch.py headless:=$HEADLESS_ARG &
SIM_PID=$!

# Wait for sim to be ready (clock topic publishing)
echo "Waiting for simulation to start..."
for i in $(seq 1 60); do
    if ros2 topic list 2>/dev/null | grep -q "/clock"; then
        # Check if clock is actually publishing
        if timeout 3 ros2 topic echo /clock --once >/dev/null 2>&1; then
            echo "Simulation ready."
            break
        fi
    fi
    sleep 1
    if [ $i -eq 60 ]; then
        echo "ERROR: Simulation failed to start after 60s"
        exit 1
    fi
done

# 2. Launch lidar TF bridge (remaps Ignition lidar frame to expected frame)
echo "Starting lidar TF bridge..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 lidar_link littleblue/base_footprint/gpu_lidar &
TF_PID=$!

# 3. Launch fake lane publisher (synthetic lanes + world pose)
echo "Starting fake lane publisher..."
python3 /tmp/fake_lanes.py &
LANES_PID=$!

# Wait for lane points to start publishing
echo "Waiting for lane points..."
for i in $(seq 1 30); do
    if timeout 3 ros2 topic echo /lane_points --once >/dev/null 2>&1; then
        echo "Lane points publishing."
        break
    fi
    sleep 1
done

# 4. Launch autonomy
echo "Starting autonomy..."
ros2 launch littleblue_autonomy autonomy.launch.py rviz:=false &
AUTO_PID=$!
sleep 2

# 5. Launch RViz
echo "Starting RViz..."
RVIZ_CONFIG="$(ros2 pkg prefix littleblue_autonomy)/share/littleblue_autonomy/config/autonomy_rviz.rviz"
rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true &
RVIZ_PID=$!

echo ""
echo "=== All systems running ==="
echo "  Sim PID:       $SIM_PID"
echo "  TF PID:        $TF_PID"
echo "  Fake lanes PID: $LANES_PID"
echo "  Autonomy PID:  $AUTO_PID"
echo "  RViz PID:      $RVIZ_PID"
echo ""
echo "Press Ctrl-C to stop all."
echo ""

# Wait for any child to exit
wait
