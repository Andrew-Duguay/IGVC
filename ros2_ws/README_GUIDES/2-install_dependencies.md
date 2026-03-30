# Installing Dependencies
In order to run this simulation framework, you must install the appropriate dependencies. To start off, you'll need:

## 1. ROS2 Iron.
This simulation is distro-speific. Although Iron is EOL Nov2024, it is what was used for this simulation. You can download [iron here](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debs.html).

> **💡 TIP:** Working with ROS you will have to source it in every terminal with ```source /opt/ros/iron/setup.bash```. Maybe make an alias or add to ```.bashrc```? I did.

> **WARNING 1:** ROS2 Humble will work with Gazebo Classic natively and is still recieving updates. You could probably use humble, but I can't guarantee everything will work if you do.

> **WARNING 2:** At some point you will have the option to install ```ros-iron-desktop``` or another package. CHOOSE ```ros-iron-desktop```.

## 2. Gazebo (classic)
> **💡 NOTE:** **This will not work with Gazebo Ignition or Gazebo Harmonic**. Everything is in Gazebo classic, so use Gazebo Classic. Any plugins that use "gz" or "ign" are not supported.

1. Install the wrapper:
    ```bash
    sudo apt install ros-iron-gazebo-ros-pkgs
    ```


2. Install the "extras". 
You probably already have these from the last command, but to be safe:
    ```bash
    sudo apt install ros-iron-xacro ros-iron-robot-state-publisher ros-iron-joint-state-publisher
    ```
    
3. Install the teleop twist controller. It allows manual control of robot from your keyboard:
    ```bash
    sudo apt install ros-iron-teleop-twist-keyboard
    ```
4. Source the build:
    
    You're actually going to source the workspace very frequently, so maybe make an easy bash script or alias.

    ```bash
    source install/setup.bash
    ```

5. Verify Install:
    ```bash
    ros2 launch gazebo_ros gazebo.launch.py 
    ```
    You should see a gray grid and blue sky. In another terminal:

    ```bash
    ros2 topic list         
    ```
    You should see ```/clock```, ```/parameter_events```, and ```/performance_metrics```

## 3. RViz
Rviz is what allows you to "see" what the robot "sees". 
    
Probably installed already if you chose ```ros-iron-desktop``` when you first got ROS2, but to be safe:

    ```bash
    sudo apt install ros-iron-rviz2
    ```

## 4. Xterm
I used xterm to...
These are the 3 main dependencies. That's it!