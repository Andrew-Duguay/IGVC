# Launching The Simulation Manually

There are 2 types of simulations:
* **Large Simulations**: These are ran without any pass/fail testing. They are large courses, typically tracks, that are meant for continuous simulation of the robot. They're meant for you to observe the robot move through a course. 
* **Unit Simulations**: These are snippets of a course. Short simulations meant to test single scenarios the robot might find itself in. They allow you to simulate a wide variety of situations from the terminal and pinpoint where it fails.

```yaml
.
├── README_GUIDES
└── src
    ├── littleblue_sim
    │   ├── ...
    │   └── worlds
    │       ├── large_sims # HERE!
    │       └── unit_sims  # AND HERE!!
    └── workbench
```

The process for launching them is largely similar, but there are key differences. For example, they use different launch files:
```yaml
.
├── README_GUIDES
└── src
    ├── littleblue_sim
    │   ├── ...
    │   └── launch
    │       ├── large_sim.launch.py # HERE AGAIN!
    │       └── unit_sim.launch.py  # AND AGAIN!!
    └── workbench
```

## 1. Building The Simulation
Before the simulation can be launched, you have to build the workspace. This is done with the ```colcon``` command:

* **COMMAND**: ```colcon build```
* **ARGUMENTS**: 
    * ```--symlink-install```: Creates symbolic links to the dynamic scripts in your simulation, that way you don't have to rebuild everytime you tweak a python file.
    * ```--packages-select <pkg1> <optional_pkgs>```: Should be obvious. I allows you to select one or more packages to build, rather than doing them all.

> **💡TIPS:** Sometimes build artifacts from past builds will register and the new packages won't be reflected in executables. You can always run ```rm -rf build install log``` before building for a clean build.

After you build then simply run ```source install/setup.bash``` to source the new workspace and your ready to launch simulations.

## 2. Launch the Simulation:
Remain in ```ros2_ws```.
### Unit Simulations:
* **COMMAND**: ```ros2 launch littleblue_sim unit_sim.launch.py```
* **ARGUMENTS**: All in ROS launch notation. Examples using default values shown.
    * ```world:=template```: Name of the simulation you want to launch.
    * ```speed:=1.0```: Speed factor. For example a factor of 2.0 will run the simulation twice as fast.
    * ```gui:=true```: For running simulation headless or not. Saves a lot of computational load.
    * ```timeout:=30.0```: The number of seconds before the test is considered failed.
    * ```step_size:=0.001```: Like the "FPS" of the simulation. Larger values reduce simulation performance but greatly decrease overhead. A step size of 0.001 requires 1000 calculations per second (in sim time). 
    * ```world_building_mode:=false```: Only used when making permanent edits to the objects in simulation. Setting to true renders only the simulation (no robot) to make it easy to add/move objects and save the world. Covered in section on building simulations.
### Large Simulations:
* **COMMAND**: ```ros2 launch littleblue_sim large_sim.launch.py```
* **ARGUMENTS**: All in ROS launch notation. Examples using default values shown.
    * ```world:=template```: Name of the simulation you want to launch.
    * ```speed:=1.0```: Same as above
    * ```gui:=true```: Same as above
    * ```step_size:=0.001```: Same as above
    * ```world_building_mode:=false```: Same as above

## 3. Launch Your ROS Code
As of now you should have a simulation, a robot rendered in it, a bridge between Gazebo and ROS, maybe a ROS node monitoring for success/failure. 

### Launch the autonomy stack: 

```bash
ros2 launch startup_robot robot.launch.py
```

### Manual control: 
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Now your robot's autonomy (or you via the keyboard) should be moving your robot. Cool.
