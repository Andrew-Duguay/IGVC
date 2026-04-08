# <span style="color:LightGreen">Adding Your Code For Development</span>

This part is easy. See the directory ```/workbench```? Remember that? That's **yours**. You do what you want in there. The only requirement is that you don't remove ```/startup_robot```. That package is specially placed as the single entry point to your robot. 

Anything you want to test, all your ROS nodes for the robot, are to be configured to launch with ```ros2 launch startup_robot robot.launch.py```. You code that file however your heart desires, but that's how you launch your robot.
```yaml
ros2_ws
├── ...
└── src                   
    ├── littleblue_sim  
    └── workbench         
        ├── startup_robot # MUST KEEP
        ├── ...           # Anything you want
        └── standalone_node.py
```

Whatever ROS nodes and packages you want to develop, organize it however you want. The only requirements are the 3 golden rules I gave at the beginning. I'll remind you:

# <span style="color:Gold">The Three Golden Rules</span>

### <span style="color:Khaki">1. Always execute commands from the workspace root (```ros2_ws```)</span>

### <span style="color:Khaki">2. Keep all your code in the ```/workbench``` directory</span>


### <span style="color:Khaki">3. Launch your robot using the startup_robot package</span>

* You set up `robot.launch.py` to launch ALL of your robot's autonomy stack.