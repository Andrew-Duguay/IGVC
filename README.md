# Simulation User Guide
This repo is a simulation framework to aid in prototyping/testing of little blue. The framework has the following core structure:

ros2_ws
├── launch_guide.sh       # Commands to interact with simulation
├── build                 # Colcon generated
├── install               # Colcon generated
├── log                   # Colcon generated
└── src                   # Code lives in here
    ├── skid_steer_robot
    │   ├── launch        
    │   │   ├── world1.launch.py
    │   │   ├── world2.launch.py
    │   │   └── world3.launch.py
    │   └── worlds
    │       ├── world1
    │       ├── world2
    │       └── world3
    └── workbench         # You can touch
        ├── my_ros_pkg
        ├── my_other_ros_pkg
        └── standalone_node.py

## 1. Installing Dependencies
In order to run this simulation framework, you must install the appropriate dependencies.
### 1. ROS2 (distro)
### 2. Gazebo (classic)
### 3. RViz
### 4. TO DO

## 2. Launching The Simulation Manually
Before the simulation can be launched, it must be built. This is done FROM THE ROOT:
    cd ~/your-local-path/ros2_ws
When in the appropriate directory:
    colcon build --packages-select skid_steer_robot
TO DO

## 3. Creating Custom Simulations
TO DO

## 4. Adding Your Code
TO DO

## 5. Launching Your Code
TO DO

## 6. Customizing Robot
TO DO

