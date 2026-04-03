# Launching The Simulation Manually

> **❓ Remember:** To source ROS before you do anything with ros. This is done with ```source /opt/ros/iron/setup.bash```. Really consider adding to ```.bashrc``` or making an alias.

## 1. Building The Simulation
Before the simulation can be launched, it must be built.



> **🚨 IMPORTANT:** Before you build or launch anything, navigate to ros2_ws for everything. Headaches are bound to occur if you don't and you'll clutter the workspace.

Run these commands to build all your packages in ROS:
```bash
cd ~/your-local-path/ros2_ws
colcon build --simlink-install
source install/setup.bash
```

> **💡TIPS:** For ```colcon build``` there are parameter options you might find useful:
>
>a) ```--simlink-install``` (This allows you to make changes to scripts without having to fully rebuild)
>
>b) ```--packages-select <pkg1> <optional_pkgs>``` 

## 2. Launch the Simulation:

Still from the ```ros2_ws``` directory, run:

```bash
ros2 launch skid_steer_robot [sim-name].launch.py
```

And the simulation should pop up!
> **💡TIPS:** Some useful parameters:
>
>a) ```gui:=false``` (To run the simulation without rendering graphics)
>
>b) ```world:=example.world``` (To specify a different world you want to run. You probably won't need this unless your work ON the simulation)

## 3. Launch Your ROS Code
As of now, your robot should be just waiting. Open another terminal and launch your program to run the robot.

If you have no ROS program or aren't too familiar wth how to run your team's code, just use this:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Now your robot's autonomy (or you via the keyboard) should be moving your robot. Cool.

## 4. Launching Them Both Easily
Rather than bothering with multiple terminals and all that jazz, there is a script included for you to run any simulation of your choice AND launch your robot simulataneously.

With your workspace built and sourced and PWD:=ros2_ws, run this:

```bash
python3 src/skid_steer_robot/run_single_sim.py <args-here>
```

The arguments are like this:
| Argument | What It Does | What To Give | Default |
| :--- | :--- | :--- | :--- |
| **--gui** | Determines whether to run the simulation headless (no graphics rendering) | "true" or "false" | "true"
| **--speed** | A multiplier to quicken the simulation time | A double value. (ex: 2.0 double time, 0.5 slow mo) | 1.0
| **--world** | name of the simulation you want to run | If you want to run mySim, you give "mySim". NOT "mySim.launch.py" | The template world (BORING)

This will automatically launch your ROS code by running the launch script in ```/workbench/startup_robot``` then launch the specified simulation.