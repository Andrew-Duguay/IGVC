#!/bin/bash

source /opt/ros/iron/setup.bash
source install/setup.bash

echo "Starting headless simulation..."

ros2 launch skid_steer_robot mohamed_playing.launch.py gui:=false &
SIM_PID=$!

sleep 5


ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" -r 10 &
DRIVE_PID=$!


ros2 run skid_steer_py_robot finish_monitor

echo "Stopping simulation..."
kill $DRIVE_PID 2>/dev/null
kill $SIM_PID 2>/dev/null

echo "Run complete."
