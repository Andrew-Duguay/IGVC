#!/bin/bash
# test_rviz_perf.sh — Minimal test: sim + drive forward + obstacle markers + RViz
# Run this in your own terminal to test RViz performance.
cd "$(dirname "$0")/.."
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
source install/setup.bash

echo "[TEST] Launching sim..."
ros2 launch littleblue_sim sim.launch.py course:=autonav &
SIM_PID=$!
for i in $(seq 1 30); do
    ros2 topic info /odom 2>/dev/null | grep -q "Publisher count: [1-9]" && break
    sleep 2
done
echo "[TEST] Sim up."

echo "[TEST] Launching test node (drive forward + 5 markers at 20Hz)..."
python3 -c "
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray

class T(Node):
    def __init__(self):
        super().__init__('test_viz')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mrk = self.create_publisher(MarkerArray, '/autonomy/obstacles', 10)
        self.create_timer(0.05, self.tick)
    def tick(self):
        c = Twist(); c.linear.x = 0.35; self.cmd.publish(c)
        ma = MarkerArray()
        for i in range(5):
            m = Marker()
            m.header.stamp.sec = 0; m.header.stamp.nanosec = 0
            m.header.frame_id = 'base_footprint'
            m.ns = 'obs'; m.id = i; m.type = Marker.CYLINDER; m.action = Marker.ADD
            m.pose.position.x = 2.0 + i; m.pose.position.y = (i%2)*0.5 - 0.25; m.pose.position.z = 0.25
            m.scale.x = 0.3; m.scale.y = 0.3; m.scale.z = 0.5
            m.color.r = 1.0; m.color.a = 0.9
            ma.markers.append(m)
        self.mrk.publish(ma)

rclpy.init(); rclpy.spin(T())
" &
TEST_PID=$!

echo "[TEST] Launching RViz..."
rviz2 -d src/littleblue_autonomy/config/autonomy_rviz.rviz &
RVIZ_PID=$!

echo "[TEST] All running. Watch RViz — are the 5 red cylinders smooth?"
echo "[TEST] Press Ctrl+C to stop."

trap "kill $RVIZ_PID $TEST_PID $SIM_PID 2>/dev/null; pkill -9 -f 'gz|ruby' 2>/dev/null; exit 0" INT TERM
wait
