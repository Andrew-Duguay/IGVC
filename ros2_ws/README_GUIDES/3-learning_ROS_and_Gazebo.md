# Some of the Basics
If you're familiar with ROS and Gazebo you can go ahead and skip here. This specific guide is primarily for those trying to get up to speed with the how everything works.

## 1. Tutorials
The ROS tutorials can be found [here](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools.html) and Gazebo's are [here](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools.html). 

Before you dive into development on this project it's essential you at least have the basics of the two down. The tutorials are pretty intuitive and easy to follow, you just have to ***ACTUALLY DO THEM***. 

## 2. Do the Tutorials
The ones [here](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools.html) and  [here](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools.html).

Yes this is just step 1 repeated, that's because the tutorials are that important. Take a day and do them. They're kinda fun (to an engineer at least).

## 3. 
Gazebo and ROS are kinda similar in that they both run ```nodes``` and publish ```topics```. They kinda run in different worlds though.

Normally ROS and Gazebo nodes/topics can't see eachother, but Gazebo has a "bridge" that allows the two to communicate. The process looks like this:
1. Gazebo will render sensor data and publish it to a Gazebo topic like /right_camera_raw
2. The bridge will convert Gazebo's /right_camera_raw into ROS2's /right_camera_raw
3. ROS will do some stuff with that image as if it were from it's camera.
4. ROS will publish to some arbitrary topic like /throttle
5. The bridge will translate this into Gazebo topics, then use the data published to control the robot.

This has some implications when working with the simulation. In your actual, full robot's environment you will have ROS2 nodes for all your sensors. Nodes that capture input from the sensor and publish it so other nodes can use the data. 

An example of this might be /camera_raw publishes a 320x320x3 array of integers (3-channel colored image). You'd then likely do some image processing stuff to make use of the data.

Likewise you would have ROS nodes that subscribe to topics and interact with the robot directly.

An example of THIS would be a node subscribes to /desired_velocity and translates that into motor current to move the robot. These nodes directly drive the robot peripherals.

Gazebo abstracts this. Your sensor-reading nodes will be ommitted, because Gazebo ALREADY publishes to /sensor_data_raw. Your peripheral's will be ommitted because gazebo moves the robot based on /cmd_vel.

## 4. How they're Run
Gazebo can be launched entirely separate from ROS. It's a bit boring though because there's nothing to move the robot.

ROS can be run separate from Gazebo. It will do its thing, but there will be not data to do it on and nothing to listen to it.

# 🤖 ROS 2 & Gazebo: The Essentials

If you’re already a ROS veteran, feel free to skip to the project-specific docs. If you’re still figuring ROS, **read this first.**

## 1. The "No-Shortcut" Rule: Tutorials
Before you touch this codebase, you need a baseline. ROS 2 and Gazebo are powerful, but they aren't "guess-and-check" friendly.

* **ROS 2 Beginner CLI Tools:** [Official Docs](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools.html)
* **Gazebo Simulation Basics:** [Official Docs](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Simulators/Gazebo/Gazebo.html)

> **To Be Real:** Step 1 is the only step that matters. These tutorials are intuitive, but you actually have to **run the commands**. Take a day. Build the foundations. Future-you will thank you when you aren't debugging a simple pathing error for six hours.

## 2. The Communication Gap (The Bridge)
Gazebo and ROS 2 are actually two entirely different programs. They don't speak the same language by default. Gazebo handles the **physics** (gravity, collisions), while ROS handles the **brains**. 

To make them work together, we use the **ros_gz_bridge**.

### How the data flows:
1.  **Gazebo** simulates a camera and publishes the image to a *Gazebo topic* (e.g., `/world/camera_data`).
2.  **The Bridge** intercepts that data, translates it into a ROS-compatible format, and republishes it to a *ROS topic* (e.g., `/camera_raw`).
3.  **Your ROS Node** listens to `/camera_raw`, processes the image, and publishes a "Move Forward" command to `/cmd_vel`.
4.  **The Bridge** translates that command back to Gazebo, which then applies physical force to the robot's wheels.



## 3. Hardware vs. Simulation (Abstraction)
One of the coolest parts of ROS is that your "Brain" nodes don't know (or care) if they are in a simulation or on a real sidewalk.

| Component | In the Real World | In the Simulation |
| :--- | :--- | :--- |
| **Input** | Physical camera driver nodes. | **Gazebo** (simulated sensors). |
| **Processing** | Your pathing/autonomy nodes. | **Your pathing/autonomy nodes.** (Same code!) |
| **Output** | Motor controller/Serial drivers. | **Gazebo** (simulated physics/motors). |

In simulation, we **omit** the physical driver nodes because Gazebo handles the "body." We only run the "mind."

## 5. Running the System
You can run them separately, but it's like a body without a brain (or vice versa):
* **Gazebo alone:** The robot will sit there, gravity will work, but it will never move.
* **ROS alone:** The nodes will spin and wait for data, but since there’s no "world," they will never receive a single pixel of information.

**To get a working robot, you must launch both and initialize the bridge.**

Luckily the scripts handle most of that for you!


> **A Quick Note on "Sim Time":** When running Gazebo, your robot uses **Simulation Time**, not the clock on your wall. If the simulation is lagging, ROS slows down its logic to match. This ensures that **1.0 second** of physics always equals **1.0 second** of "thinking," no matter how fast your computer is.