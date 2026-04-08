# Create Custom Simulations
Creating a custom simulation for this framework is actually really easy. The easiest way is to one of the existing simulations to a new file and just make edits. Below are the steps to do so.

## 1. Copy The World Template:
```bash
cd ~/ros2_ws/src/skid_steer_robot/worlds/
cp ./ccustom_world_template/ ./myWorld
```

## 2. Change Names **of** world file:
```bash
mv ./myWorld/change_me.world ./myWorld/myWorld.world
```
## 3. Open /myWorld.world :
* Go to line 3 and:
```xml
<--  Change this: -->
<world name="change_me">

<--  To this: -->
<world name="myWorld">
```

## 4. Design Your Course:
The EASIEST and MOST PROFESSIONAL way to do this is by just drawing the course in an sketch software. 

* The photo currently saved as **myWorld/floor/materials/texture/floor_picture.png** is a large, blank asphault texture. Feel free to draw on it if you want to. Most of my courses were made by just drawing lanes on that very same picture. 

* Whatever you use (photoshop, paint, etc) just make sure the image resolution is large (Like 4096 x 4096 big).

* Just draw up your ground floor however you want save it back to **/worlds/myWorld/floor/materials/texture/floor_picture.png**
        
## 5. Change The Size of Your World:
Open myWorld/floor/model.sdf there are 2 lines that specify the size of the course that look like this:

```xml
<size>20 20 0.1</size> <!-- PUT YOUR COURSE SIZE HERE IN METERS -->
```
Adjust those to fit your desired world.

## 6. Edit the /floor/model.config file.
You did the work, you take the credit!
 
## 7. The Launch File
1.  In /skid_steer_robot/launch there are all the launch files. Go there, create a copy of custom_world_template.launch.py and rename it myWorld.launch.py
2.  There are two lines at the top that look like this:
```python
world_file = os.path.join(pkg, 'worlds', 'change_me','change_me.world')
model_path = os.path.join(pkg, 'worlds', 'change_me')
```
Swap "change_me" with "myWorld" (or whatever world name you chose)

## 8. Add stuff to your simulation
1. launch the simulation:

```bash
cd ~/blah-blah-blah/ros2_ws
colcon build --packages-select skid_steer_robot
source install/setup.bash
ros2 launch skid_steer_robot myWorld.launch.py
```
2. Drag and drop stuff from the 
    
 



8. Launch Simulation

9. Position Robot

10. Position GUI


8. Adding Obstacles.

5. Change Where GUI Camera Spawns at:
        Open /myWorld.world. On line 27 you should see:
        <pose>-4 -16 4 0 0.3 1.2</pose>       <!-- SETS WHERE CAMERA STARTS AT-->

        These values dictate where the User Camera starts at. The easiest way to set these is to: open the simulation, navigate to where you want to start from and copy those values.

