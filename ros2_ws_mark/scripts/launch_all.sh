#!/bin/bash
# launch_all.sh — Launch sim + fake_lanes + autonomy in one shot.
# Usage: bash scripts/launch_all.sh [approach]
#   approach: A (default), baseline, B, C, D

APPROACH="${1:-A}"
WS_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS_DIR"

# Fix GPU permissions (non-persistent, needed after each WSL restart)
sudo chmod 666 /dev/dri/renderD128 2>/dev/null

# Clean DDS
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*

source install/setup.bash

echo "[LAUNCH] Starting sim (course=autonav2)..."
ros2 launch littleblue_sim sim.launch.py course:=autonav2 &
SIM_PID=$!

# Wait for sim
for i in $(seq 1 60); do
    if ros2 topic info /odom 2>/dev/null | grep -q "Publisher count: [1-9]"; then
        echo "[LAUNCH] Sim up."
        break
    fi
    sleep 1
done

echo "[LAUNCH] Starting fake_lanes..."
python3 scripts/fake_lanes_autonav.py &
LANES_PID=$!

# Wait for fake_lanes
for i in $(seq 1 20); do
    if ros2 topic info /lane_points 2>/dev/null | grep -q "Publisher count: [1-9]"; then
        echo "[LAUNCH] Fake lanes up."
        break
    fi
    sleep 1
done

echo "[LAUNCH] Starting candidate lane detectors (method=${DETECTOR:-hsv_threshold})..."
ros2 run littleblue_vision lane_candidate_node --ros-args \
    -r __node:=left_lane_candidate \
    -p method:=${DETECTOR:-hsv_threshold} \
    -p use_sim_time:=true \
    -p image_topic:=/left_camera/image_raw \
    -p output_topic:=/candidate/left_lane_points \
    -p debug_topic:=/candidate/left_debug \
    -p camera_lateral_offset:=0.28 \
    -p camera_forward_offset:=0.38 \
    -p camera_pitch:=0.09 \
    -p camera_yaw:=0.26 \
    -p frame_skip:=2 \
    -p subsample_stride:=8 &
CAND_L_PID=$!

ros2 run littleblue_vision lane_candidate_node --ros-args \
    -r __node:=right_lane_candidate \
    -p method:=${DETECTOR:-hsv_threshold} \
    -p use_sim_time:=true \
    -p image_topic:=/right_camera/image_raw \
    -p output_topic:=/candidate/right_lane_points \
    -p debug_topic:=/candidate/right_debug \
    -p camera_lateral_offset:=-0.28 \
    -p camera_forward_offset:=0.38 \
    -p camera_pitch:=0.09 \
    -p camera_yaw:=-0.26 \
    -p frame_skip:=2 \
    -p subsample_stride:=8 &
CAND_R_PID=$!

echo "[LAUNCH] Starting lane accumulator (world-frame persistence)..."
ros2 run littleblue_vision lane_accumulator_node --ros-args \
    -p use_sim_time:=true \
    -p mode:=accumulate \
    -p max_range:=8.0 \
    -p grid_res:=0.15 \
    -p publish_rate:=15.0 &
ACCUM_PID=$!

echo "[LAUNCH] Starting LiDAR obstacle detector..."
ros2 run littleblue_vision lidar_obstacle_node --ros-args \
    -p use_sim_time:=true \
    -p min_range:=0.5 \
    -p max_range:=8.0 \
    -p cluster_gap:=0.2 \
    -p min_cluster_points:=3 &
LIDAR_PID=$!

echo "[LAUNCH] Starting autonomy (approach=$APPROACH, rviz=true)..."
ros2 launch littleblue_autonomy autonomy.launch.py approach:=$APPROACH rviz:=true &
AUTO_PID=$!

# Wait for autonomy
for i in $(seq 1 30); do
    if ros2 topic info /cmd_vel 2>/dev/null | grep -q "Publisher count: [1-9]"; then
        echo "[LAUNCH] Autonomy up. Robot driving."
        break
    fi
    sleep 1
done

echo ""
echo "[LAUNCH] All running. PIDs: sim=$SIM_PID lanes=$LANES_PID cand_L=$CAND_L_PID cand_R=$CAND_R_PID auto=$AUTO_PID"
echo "[LAUNCH] Diagnostics → /tmp/autonomy_diag/"
echo "[LAUNCH] Press Ctrl+C to stop everything."
echo ""

# Wait for Ctrl+C, then clean up
trap "echo '[LAUNCH] Stopping...'; kill $AUTO_PID $LIDAR_PID $ACCUM_PID $CAND_L_PID $CAND_R_PID $LANES_PID $SIM_PID 2>/dev/null; sleep 2; pkill -9 -f 'gz|ruby' 2>/dev/null; rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*; echo '[LAUNCH] Done.'; exit 0" INT TERM

wait
