# ROS 2 & Gazebo: The Essentials

If you’re already a ROS veteran, feel free to skip to the project-specific docs. If you’re still figuring ROS, **read this first.**

## 1. No Shortcuts: Tutorials
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