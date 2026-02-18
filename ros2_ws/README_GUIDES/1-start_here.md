# START HERE
This repo is a simulation framework to aid in prototyping/testing of autonomous vehicles. It contains a robust collection of simulations and an easy workflow. 
To effectively use the simulation it is recommended you follow along with the guide. It's very easy to understand and won't take you very long once dependencies are all set up. 

The framework has the following core structure:

```yaml
ros2_ws
├── launch_guide.sh
├── build                 # Colcon generated
├── install               # Colcon generated
├── log                   # Colcon generated
└── src                   
    ├── skid_steer_robot  # Simulation stuff lives here, be careful.
    │   ├── launch              # Launch files for starting the simulation
    │   │   ├── world1.launch.py
    │   │   ├── world2.launch.py
    │   │   └── world3.launch.py
    │   ├── urdf                # description of the robot
    │   └── worlds              # Contains the code for worlds  
    │       ├── world1
    │       ├── world2
    │       └── world3
    └── workbench         # Ros code goes here
        ├── my_ros_pkg
        ├── my_other_ros_pkg
        └── standalone_node.py
```

* Anything you will launch should be from the ros2_ws. 
    * It simplifies everything. 
    * If you change up the flow then you might throw yourself into a dependency hell and be stuck crawling through nested launch and config files. 
    * Don't reinvent the wheel, just use the tool as is.

* All your code will be in the workbench directory. 

    * It's isolated, safe, clean and organized. 
    * Use whatever structure you want in there, that's your room to do what you want with. 
    * Right now it holds only the example program.