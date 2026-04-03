# Adding Your Code For Development

This part is easy. See the directory ```/workbench```? Remember that? That's **yours**.
```yaml
ros2_ws
├── ...
└── src                   
    ├── skid_steer_robot  
    └── workbench         
        ├── startup_robot # MUST KEEP
        ├── ...           # Anything you want
        └── standalone_node.py
```

Whatever ROS nodes and packages you want to develop, organize it however you want. The only requirements are the 3 golden rules I gave at the beginning. I'll remind you:

## The Three Golden Rules

1. Always execute commands from the workspace root (ros2_ws)

    * Blah blah blah

2. Keep all your code in the ```/workbench``` directory
    * Why: It creates a strict, safe boundary between the core simulation physics/environments and your autonomous code.

    * Your Freedom: Inside the workbench directory, you have total control. You can structure your custom ROS 2 packages, standalone scripts, and nodes however you see fit without worrying about breaking the simulator.

3. Launch your robot using the startup_robot package.
    * Why: It abstracts all of your robots control logic into a single startup script, simplifying the simulation scripts

    * Your Freedom: The way you launch individual nodes/packages/systems is up to you. You have full control of the contents of startup_robot, the configs etc, just keep the file names.