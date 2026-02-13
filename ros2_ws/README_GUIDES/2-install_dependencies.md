# Installing Dependencies
In order to run this simulation framework, you must install the appropriate dependencies. To start off, you'll need:

1. ROS2 Iron.
This simulation is distro-speific. Although Iron is EOL Nov2024, it is what was used for this simulation and the corresponding project it was designed for. You can download [iron here](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debs.html).

I've heard that ROS2 Humble will work with Gazebo Classic natively and is still recieving updates. You could probably use humble, but I can't guarantee it will all work if you do.

If you need to use a different ROS distro that doesn't support Gazebo Classic, this simulation won't work. You can adapt it to your distro, but it won't be a small undertaking.

2. Gazebo (classic)
This will not work with Gazebo Ignition or Gazebo Harmonic. Everything is in Gazebo classic, so use Gazebo Classic. The plugins that use "gz" or "ign" are not supported.
    1. Install the wrapper:
        sudo apt install ros-iron-gazebo-ros-pkgs
            This gives you the 'gazebo' command.
            This works with .urdf and .xacro files that use <gazebo> tags.

    2. Install the "extras". You probably already have these from the last command, but to be safe:
        sudo apt install ros-iron-xacro ros-iron-robot-state-publisher ros-iron-joint-state-publisher
    
    3. Install the teleop twist controller. It allows manual control of robot from your keyboard:
        sudo apt install ros-iron-teleop-twist-keyboard
    4. Source the build:
        source install/setup.bash
    5. Verify Install:
        ros2 launch gazebo_ros gazebo.launch.py 
            gray grid and blue sky
        ros2 topic list         
            You should see /clock, /parameter_events, and /performance_metrics

3. RViz
Rviz is what allows you to "see" what the robot "sees". 
    1. Probably installed already if you installed got ros-iron-desktop when you first got ROS2, but to be safe:
        sudo apt install ros-iron-rviz2

These are the 3 main dependencies. That's it!