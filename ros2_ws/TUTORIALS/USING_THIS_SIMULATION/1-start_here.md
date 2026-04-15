
# <span style="color:LightGreen">🚀 Simulation Framework: Quick Start Guide

Welcome to the autonomous vehicle simulation framework. This repository provides a robust collection of simulation environments and a streamlined workflow to help you prototype and test your autonomous systems.

> **💡 LISTEN :** To get the most out of this framework without running into setup headaches, we highly recommend reading this brief guide. Once your dependencies are configured, the workflow is fast, safe, and intuitive.



## <span style="color:LightSkyBlue">📂 WORKSPACE STRUCTURE

The workspace is organized with a strict separation between the core simulation engine and your custom experimental code.


```yaml
ros2_ws
├── build                   # Colcon generated
├── install                 # Colcon generated
├── log                     # Colcon generated
└── src                   
    ├── littleblue_sim      # Simulation stuff lives here, be careful.
    │   ├── urdf            # description of the robot
    │   ├── launch          # launch scripts
    │   |   ├── large_sim.launch.py 
    │   |   └── unit_sim.launch.py  
    │   └── worlds          # Contains the code for worlds 
    │       ├── world1
    │       ├── world2
    │       └── world3
    └── workbench
        ├── startup_robot   # ROS code's entry point
        ├── my_ros_pkg
        ├── ...
        └── standalone_node.py
```
## <span style="color:LightSkyBlue">ABOUT THE SIMULATIONS
There are 3 types of simulations to run:

### <span style="color:LightGreen">Unit Simulations
Think of these simulations as **unit tests**. They are short, very short, and should place the robot in a specific, novel circumstance and test if it can behave correctly. The unit sims are carefully constructed to report whether the robot passes or fails.

### <span style="color:LightGreen">Large Simulations
These are primarily used for visualizing the robot. An example would be a mock IGVC course for the robot to loop endlessly around. It allows you to visually demonstrate the robot and play around with stuff, but offers no objective success/failure criteria. It's not a test, but a showcase/inspection.

### <span style="color:LightGreen">Batch Simulation
This is a simulation script to be ran from the command line. It runs through each of the unit simulations headless (no gui, no rendering) and reports which simulations pass or fail. It helps pinpoint where your robot is weak so you can narrow down your bugs. If you can pass all the unit sims then you're in pretty good shape.

## <span style="color:LightSkyBlue">ABOUT THE MAIN SCRIPTS
You'll notice in the workspace root three launch scripts. Since there is a hard partition between the **robot** code and the **simulation** code, then they need to be launched separately. These launch scripts are a convenient way to launch both the simulation and the robot simultaneously.

### <span style="color:LightGreen">run_unit_sim.py
This launches a single unit sim and your robot. The default is to run the simulation with graphics so you can visualize it. A use case of this script would be **"Hey my robot failed simulation *n*, lets see what happened"**.

### <span style="color:LightGreen">run_large_sim.py
This launches a large sim and your robot. Simple. Someone wants to see your robot running? Just run this script and voila, it renders and launches everything and your robot goes!

### <span style="color:LightGreen">run_batch.py
This is the "unit testing framework". This is where the magic happens. Running this script will take some time, as it will loop through every unit sim and launch it in gazebo, launch your robot, and run it. By default there is no GUI, but there *IS* a pretty terminal interface to update you what's happening.

## <span style="color:LightSkyBlue">ADDING YOUR CODE</span>


All your ROS nodes for the robot are to be configured to launch with ```ros2 launch startup_robot robot.launch.py```. You code that file however your heart desires, but that's how you need to launch your robot.

1. Each sub-system on your robot will have its own package (e.g. vision_pkg, gps_pkg, etc.)
2. All your packages will be placed in `workbench`. 
3. Each package will have a launch script to launch that subsystem.
4. You will configure the `startup_robot` package to launch each subsystem on your robot.
5. The main scripts (`run_unit_sim.py`, `run_batch.py`, and `run_large_sim.py`) will run the launch script in `startup_robot` to start your robot.
```yaml
    ros2_ws
    ├── ...
    └── src                   
        ├── littleblue_sim  
        └── workbench         
            ├── startup_robot # MUST KEEP
            ├── ...           # Anything you want
            └── standalone_node.py
```

## <span style="color:Gold">THREE GOLDEN RULES
To keep your development process smooth and avoid "dependency hell", please adhere to these three structural rules:

### <span style="color:Khaki">1. Always execute commands from the workspace root (ros2_ws)
* ***Why***: It guarantees that all relative paths, environment variables, and sourced setup files resolve correctly.

* ***The Risk***: If you change the execution flow or try launching scripts from deep within subdirectories, you will likely break underlying dependencies and find yourself crawling through nested config files trying to fix paths.

* ***The Takeaway***: Don't reinvent the wheel. Use the provided tools and launch files exactly as intended from the root directory.

### <span style="color:Khaki">2. Keep all your code in the ```/workbench``` directory
* ***Why***: It creates a strict, safe boundary between the core simulation physics/environments and your autonomous code.

* ***Your Freedom***: Inside the workbench directory, you have total control. You can structure your custom ROS 2 packages, standalone scripts, and nodes however you see fit without worrying about breaking the simulator.

### <span style="color:Khaki">3. Launch your robot using the startup_robot package.
* ***Why***: It abstracts all of your robots control logic into a single startup script, simplifying the simulation scripts

* ***Your Freedom***: The way you launch individual nodes/packages/systems is up to you. You have full control of the contents of startup_robot, the configs etc, just keep the file names.

## <span style="color:LightSkyBlue"> INSTALL DEPENDENCIES:
#### ROS and Ignition Packages:
```bash
sudo apt update && sudo apt install -y \
  ros-iron-desktop \
  ros-iron-ros-gz \
  ros-iron-xacro \
  ros-iron-robot-state-publisher \
  ros-iron-joint-state-publisher \
  ros-iron-teleop-twist-keyboard \
  ros-iron-rviz2 \
  xterm
```
#### Python Environment Setup
```bash
pip uninstall -y numpy && \
pip install "numpy==1.26.4" "opencv-python<4.10.0"
```
#### Verify Install
```bash
# 1. Source the global ROS install
source /opt/ros/iron/setup.bash
# 2. Check versions
python3 -c "import numpy; import cv2; print(f'NumPy: {numpy.__version__} | OpenCV: {cv2.__version__}')"
```