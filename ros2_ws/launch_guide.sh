###################################################################################
### THIS IS NOT A SCRIPT, IT'S A LIST OF COMMANDS. FOLLOW THESE STEPS TO LAUNCH ###
###################################################################################
exit 1


# 1. Navigate to ros2_ws to run script
cd ~/put-your-path-here/ros2_ws

# 2. Generates build, log, and install directories. This is why we're in ros2_ws
colcon build --packages-select skid_steer_robot yolo_trt # or simply
colcon build

# 3. Source the newly built pkg
source install/setup.bash

# 4. Launch the gazebo simulation. Pick which one you want
ros2 launch skid_steer_robot race_track.launch.py                   # Launches a large circuit course
ros2 launch skid_steer_robot small_course.launch.py                 # Launches a small, textured course with obstacles
ros2 launch skid_steer_robot small_course_no_barrels.launch.py      # Launches a small, textured course with no obstacles

# 5. Open viewer to see sensor data
rviz2 -d src/skid_steer_robot/config/robot_config.rviz

# 6.a. Install teleop if not already done
sudo apt install ros-<your-distro>-teleop-twist-keyboard

# 6.b. Run in separate terminal for manual control
ros2 run teleop_twist_keyboard teleop_twist_keyboard

