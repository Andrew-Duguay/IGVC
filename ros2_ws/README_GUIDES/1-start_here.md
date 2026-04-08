
# <span style="color:LightGreen">🚀 Simulation Framework: Quick Start Guide

Welcome to the autonomous vehicle simulation framework. This repository provides a robust collection of simulation environments and a streamlined workflow to help you prototype and test your autonomous systems.

> **💡 LISTEN :** To get the most out of this framework without running into setup headaches, we highly recommend reading this brief guide. Once your dependencies are configured, the workflow is fast, safe, and intuitive.



## <span style="color:LightSkyBlue">📂 Core Repository Structure

The workspace is organized into a standard ROS 2 architecture, with a strict separation between the core simulation engine and your custom experimental code.


```yaml
ros2_ws
├── build                 # Colcon generated
├── install               # Colcon generated
├── log                   # Colcon generated
└── src                   
    ├── littleblue_sim  # Simulation stuff lives here, be careful.
    │   ├── urdf   # description of the robot
    │   ├── launch # launch scripts
    │   |   ├── large_sim.launch.py 
    │   |   └── unit_sim.launch.py  
    │   └── worlds # Contains the code for worlds 
    │       ├── world1
    │       ├── world2
    │       └── world3
    └── workbench
        ├── startup_robot # ROS code's entry point
        ├── my_ros_pkg
        ├── ...
        └── standalone_node.py
```

## <span style="color:Gold">The Three Golden Rules
To keep your development process smooth and avoid "dependency hell", please adhere to these two structural rules:

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