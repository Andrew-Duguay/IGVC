# <span style="color:LightGreen">**Installing Dependencies**
This is a step-by-step guide for installing dependencies. 


## <span style="color:LightSkyBlue">1. ROS2 Iron.
This simulation is distro-specific. Although Iron is EOL Nov2024, it is what was used for this Robot. You can download [iron here](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debs.html).

> **💡 TIP:** Working with ROS you will have to source it in every terminal with ```source /opt/ros/iron/setup.bash```. Maybe make an alias or add to ```.bashrc```? I did.

> **💡 NOTE:** At some point the tutorial will suggest to install ```ros-iron-desktop```. Do it!.

## <span style="color:LightSkyBlue">2. Gazebo (Ignition)

> ### 1. Install the wrapper:
> ```bash
> sudo apt install ros-iron-ros-gz
> ```


> ### 2. Install the "extras". 
> You probably already have these from the last command, but to be safe:
> 
>```bash
> sudo apt install 
> ros-iron-xacro 
> ros-iron-robot-state-publisher 
> ros-iron-joint-state-publisher
> ```
    
> ### 3. Install the teleop twist controller. It allows manual control of robot from your keyboard:
> ```bash
> sudo apt install ros-iron-teleop-twist-keyboard
> ```

> ### 4. Source the build:   
> You're actually going to source the workspace very frequently, so maybe make an easy bash script or alias.
> 
> ```bash
> cd ~/ros2_ws
> source install/setup.bash
> ```

> ### 5. Verify Install:
> ```bash
> ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="empty.sdf"
> ```
> You should see a gray grid and blue sky. In another > terminal:
> 
> ```bash
> ros2 topic list         
> ```
> You should see ```/clock```  and ```/gui/camera/pose```

## <span style="color:LightSkyBlue">3. RViz
Rviz is what allows you to "see" what the robot "sees". 
    
Probably installed already if you chose ```ros-iron-desktop``` when you first got ROS2, but to be safe:

```bash
sudo apt install ros-iron-rviz2
```

## <span style="color:LightSkyBlue">4. Xterm
I used xterm to launch a standard ROS node in the example package. It's just to open a terminal for manual control of the robot. Just run this command:

```bash
sudo apt install xterm
```

## <span style="color:LightSkyBlue">5. NumPy
ROS2 was done using an older version of NumPy, before NumPy 2+. Therefore you need an older version.

Run this in your container to uninstall your current NumPy and install a compatible version : 
```bash
pip uninstall numpy
pip install "numpy==1.26.4"
```

## <span style="color:LightSkyBlue">6. OpenCV
Any version of OpenCV newer than 4.10 will hard-require you to have NumPy 2.0+ so if you just install it straight away then you'll undo the last step. Pip will automatically install the newer version of NumPy.

```bash
pip install "opencv-python<4.10.0"
```
> **❌ WARNING**: Do NOT use the generic command ```pip install opencv-python```

# <span style="color:Gold">That's all the dependencies! That's it!

