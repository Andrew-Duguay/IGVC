# Create Custom Simulations

> **🚨 IMPORTANT:** I highly recommend that when you create a new simulation, you follow these steps VERY carefully. The directory structure and scripts to launch the simulations are precise and can be tedious. Even if you are proficient with Gazebo, you should still use the template workflow provided here.

Creating a custom simulation for this framework is actually easy—it is done by customizing a copy of the template world. 

Think of these simulations as **unit tests**. They should place the robot in a novel circumstance and test if it can behave correctly. Do your best to keep the simulation small and simple. Browse through the `/worlds` directory and look at the different simulation previews for reference.


## 1. Copy the Simulation Template and Launch File
The template consists of a directory for the simulation and a launch script. Copy them both and rename them, replacing `template` with your simulation's name. Be descriptive about the contents of your world! 

The template locations are shown below:
```bash
ros2_ws/
├── README_GUIDES/
└── src/
    └── skid_steer_robot/
       ├── launch/
       │   └── template.launch.py  <-- COPY THIS
       └── worlds/
           └── template/           <-- COPY THIS DIR
```

## 2. Refactor the simulation name:
There are multiple places in the copied template where you need to change the simulation name from template to your new name. This must exactly match the name of your new simulation folder and launch file.
1. The name of the ```.world``` file
2. The name IN the ```.world``` file (line 3)
3. The name in the launch file (line 12)

## 3. Draw The Course Floor Plane:
The textures folder in your new simulation has an image that will be cast onto the floor. Currently it is just blank asphault texture. The easiest and most professional way to design your course is to go to some drawing software (I like [photopea](https://www.photopea.com/)) and draw nice, smooth lines on the image. Once you have your course drawn up, save it back in the same location with the same name.
```bash
worlds
├── ...
└── your_new_world
    └── custom_models
        ├── ...
        └── floor
            ├── ...
            └── materials
                ├── ...
                └── textures
                    └── floor_picture.png <-- SAVE HERE

```
If you use [photopea](https://www.photopea.com/) you can open up the image then go to:

```View``` --> ```Show``` --> ```Grid```

Then:

```Edit``` --> ```Preferences```

And change/color the grid based on course size. The template image is blank, square asphault, so a NxN grid allows you to create a scale for drawing your course. Most courses ar 10mx10m so I use a 10x10 grid to make sure the robot has enough clearance to actually navigate the course.

The **brush tool** is the best for drawing curved lines by far, just increase the "smoothness" so the lines aren't jagged and ugly. Straight lines can be done easiest with the **shape tool**.


## 4. Change The Size of Your World (optional):
In ```model.sdf```, you can change the dimensions of the ground plane. 
```bash
worlds
├── ...
└── your_new_world
    └── custom_models
        ├── ...
        └── floor
            ├── ...
            └── model.sdf <--- THIS FILE
```
Line 9 and 16 have the two size attributes. Both lines look like this:
```xml
<size>10 10 0.1</size> <!-- COURSE SIZE HERE IN METERS -->
```
Adjust those to fit your desired world.

## 5. Build and Launch Your Simulation:
This is where it gets fun. Now you need to launch your simulation so you can visually position the obstacles, the robot, and the user camera.
```bash
cd .../ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch skid_steer_robot your-Launch-File.launch.py
```
If you've followed along, a basic simulation with your floor design should render. There is a ramp, a cone, and a barrel in the simulation, with the robot dead in the center.

## 6. Position The Robot
1. Using Gazebo's Translation and Rotation modes (top toolbar), drag and drop the robot exactly where you want it to start.

2. Make it permanent: In Gazebo, click on the robot. In the left-hand ribbon (under the World tab), there is a property called ```pose```. These are your exact coordinate values.

3. Open your launch file. On Line 13, there is a corresponding ```robot_position``` dictionary. Replace the default values with your new ```pose``` values and save the file.

## 7. Place Your Obstacles
1. Drag the barrel, cone, and ramp where you want them. They also have a ```pose``` property we will use.

2. Open the ```.world``` file for your simulation. Line 32 is where the 3 default objects are declared.

3. Set it all up! You can add or subtract as many cones, ramps, and barrels as you want in the ```.world``` file using ```<include>``` tags. 

    Use the objects currently in the running simulation to get the exact pose values needed for the code.

4. Save your ```.world``` file, exit the simulation, rebuild the workspace, and re-launch. Your changes will now be permanent. Rinse and repeat until your course is perfect.


## 8. Position The GUI
When the simulation launches, you don't want to shuffle to set up the camera in time to actually see the robot do anything. Luckily, you can hardcode the default view.

1. Launch your completed simulation.

2. Position the Gazebo camera exactly where you want it to watch your simulation run from start to finish.

3. In the left-hand ribbon, select ```GUI```. Inside, there is an attribute called ```pose```.

In your ```.world``` file, Line 25 defines the ```<camera name='user_camera'>``` position. Copy over the pose values you just set up.

## 9. Take a Screenshot

You (and others in the future) might want a sneak peek of your simulation without having to launch the whole thing. Take a screenshot of your course, crop it, and save it here:
```/worlds/your_new_world/world_preview.png```

## 10. Draw Your Finish Line

Now that you have a pretty simulation setup, you need to add it to the master script so all future students can test their robots against it.

1. Line 21 in your launch file has a Python dictionary defining the finish line's ```x1, y1, x2, y2``` coordinate points.

2. This finish line is automatic. Whatever segment you define here acts as an invisible tripwire, triggering a ```[SUCCESS]``` condition as soon as your robot crosses it.

3. Choose the endpoints of this tripwire carefully in Gazebo. 

    Use an object or the robot's pose attribute to get the coordinates for the end points and save them to your launch file.

## 11. Add It To The Master Script
1. Open ```~/ros2_ws/src/skid_steer_robot/run_sims.py```

2. At Line 29, there is a ```simulations = [...]``` list where all current sims are tracked.

3. Scroll to the bottom and add yours in the exact same dictionary format as the rest.

4. The 'name' key must match the name of your directory and files exactly.

5. Set your TIMEOUT_SECONDS. This is how long you are willing to give a robot before failing it. 

    (Rule of thumb: Calculate roughly the time it takes to cross the map at 1 mph).

6. Save the file.

    a. Open /ros2_ws/src/skid_steer_robot/run_sims.py
    b. At line 29 there is a list of dictionaries where all the current simulations are listed.
    c. Scroll to the bottom and add yours in the same format as the rest.
    d. 'name' needs to match the name of your directory/.world file/launch file/etc. exactly.
    e. TIMEOUT is how long you are willing to give a robot before you'd consider the test a failure. A good rule of thumb I've found: Roughly the time to cross the entire map at 1mph.
    f. Save the file.

## 12. Claim Credit (optional)
Your doing the work, you take the credit!
1. Open the ```/floor/model.config``` file.

2. Add your info to the ```<author>``` block.
---

✅ You're done! That is all it takes! Tedious, sure, but easy!