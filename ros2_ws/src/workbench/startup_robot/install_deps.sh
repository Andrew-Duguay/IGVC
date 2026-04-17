#!/usr/bin/env bash
# Install all apt + rosdep dependencies for running the real-robot
# deploy launch on a fresh Ubuntu 22.04 + ROS 2 Iron system.
#
# Run with sudo. Idempotent — safe to re-run.
set -eo pipefail

if [[ "$(lsb_release -cs)" != "jammy" ]]; then
  echo "Expected Ubuntu 22.04 (jammy). Detected: $(lsb_release -cs)"
  exit 1
fi

echo "==> apt update"
sudo apt update

echo "==> ROS 2 Iron core packages (if not already present)"
sudo apt install -y \
  ros-iron-ros-base \
  ros-iron-robot-state-publisher \
  ros-iron-xacro \
  ros-iron-joy \
  ros-iron-rviz2 \
  python3-colcon-common-extensions

echo "==> Localization + sensor drivers"
sudo apt install -y \
  ros-iron-robot-localization \
  ros-iron-usb-cam \
  ros-iron-nmea-navsat-driver \
  ros-iron-image-transport \
  ros-iron-compressed-image-transport

echo "==> Vision / autonomy runtime deps"
sudo apt install -y \
  python3-opencv \
  python3-numpy \
  ros-iron-cv-bridge \
  ros-iron-vision-msgs \
  ros-iron-sensor-msgs-py \
  ros-iron-tf2-geometry-msgs \
  ros-iron-message-filters

echo "==> Serial for motor controller"
sudo apt install -y python3-serial

echo "==> YOLO TensorRT Python runtime (Jetson)"
# Ultralytics is the easiest runtime for loading .pt / .engine weights.
# On Jetson it uses the system TensorRT + CUDA from JetPack.
# Requires torch built for the Jetson's JetPack. Ultralytics pulls a
# compatible wheel where possible; for older Jetsons you may need
# to install torch from NVIDIA's Jetson wheel index manually first.
sudo -H pip3 install --upgrade ultralytics opencv-python

echo
echo "Still to install manually (pick the driver matching your hardware):"
echo "  * IMU driver     (e.g. ros-iron-microstrain-inertial-driver)"
echo "  * LiDAR driver   (sllidar_ros2 — build from source from the existing"
echo "                    real-stack workspace if no apt package available)"
echo "  * Depth camera   (e.g. ros-iron-realsense2-camera)"
echo
echo "Still required:"
echo "  * YOLO weights file at \$HOME/yolo_weights/<name>.engine"
echo "    (or override via YOLO_ENGINE env var — see DEPLOY.md §YOLO)"
echo
echo "==> Copy udev rules"
if [[ -f /etc/udev/rules.d/99-littleblue.rules ]]; then
  echo "    /etc/udev/rules.d/99-littleblue.rules already exists, skipping."
else
  SRC="$(dirname "$(readlink -f "$0")")/udev/99-littleblue.rules"
  sudo cp "$SRC" /etc/udev/rules.d/
  sudo udevadm control --reload-rules
  sudo udevadm trigger
  echo "    Installed. Update the REPLACE_WITH_* lines in"
  echo "    /etc/udev/rules.d/99-littleblue.rules for this robot's hardware."
fi

echo
echo "Done. Next steps:"
echo "  1. Edit /etc/udev/rules.d/99-littleblue.rules with real VID/PID/serials"
echo "  2. Source /opt/ros/iron/setup.bash"
echo "  3. Build the workspace: colcon build"
echo "  4. Launch: ros2 launch startup_robot deploy.launch.py"
