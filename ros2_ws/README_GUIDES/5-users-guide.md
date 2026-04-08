### <span style="color:SkyBlue">Launching Simulation Only</span>
| Command | Action | Arg Notation |
| :--- | :--- | :--- |
| ```ros2 launch littleblue_sim large_sim.launch.py``` | Run Large Simulation | ROS (e.g., `arg:=val`) |
| ```ros2 launch littleblue_sim unit_sim.launch.py```| Run Unit Simulation | ROS (e.g., `arg:=val`) |

| Argument | Default Value | Description |
| :--- | :--- | :--- |
| `world` | `template` | The name of the world file/track to load (e.g., `ramp`, `chicane`). |
| `speed` | `1.0` | Target Real-Time Factor (RTF). Use `2.0` for double max speed. |
| `gui` | `true` | Set to `false` for headless mode to save CPU during batch runs. |
| `step_size` | `0.001` | Physics time step. Increase (e.g., `0.003`) for faster calculations. |
| `world_building_mode` | `false` | Disables physics and referee logic for easier track editing. |
| `timeout` | `30.0` | Alters the threshold for a failed test. (**unit_sim only**)


---
### <span style="color:SkyBlue">Launching Robot Only</span>
| Command | Action | Arg Notation |
| --- | --- | --- |
| ```ros2 launch startup_robot robot.launch.py``` | Launch all ROS code | ROS (e.g., `arg:=val`) |

---
### <span style="color:SkyBlue">Launching Simulation + Robot</span>
| Command | Action | Arg Notation |
| :--- | :--- | :--- |
| `python3 run_large_sim.py` | Run Large Simulation | STD (e.g., `--arg val`) |
| `python3 run_unit_sim.py`| Run Unit Simulation | STD (e.g., `--arg val`) |

| Argument | Default Value | Description |
| :--- | :--- | :--- |
| `world` | `template` | The name of the world file/track to load (e.g., `ramp`, `chicane`). |
| `speed` | `1.0` | Target Real-Time Factor (RTF). Use `2.0` for double max speed. |
| `gui` | `true` | Set to `false` for headless mode to save CPU during batch runs. |
| `step_size` | `0.001` | Physics time step. Increase (e.g., `0.003`) for faster calculations. |
| `timeout` | `30.0` | Alters the threshold for a failed test. (**unit_sim only**)

---
### <span style="color:SkyBlue">ROS Cheat-Sheet</span>

| Command | Action | Arg Notation |
| :--- | :--- | :--- |
| `ros2 launch [pkg] [file]` | Start a ROS 2 Launch File | `arg:=val` |
| `ros2 run [pkg] [exec]` | Run a single ROS Node | `--ros-args -p param:=val` |
| `colcon build` | Compile your workspace | `--packages-select [name]` |
| `ros2 topic list` | See all active ROS streams | `-t` (Show Types) |
| `ros2 topic echo [topic]` | View live data in terminal | No flag needed |
| `ros2 topic hz [topic]` | Check the frequency (speed) | No flag needed |
| `ros2 node list` | See all active Python/C++ nodes | No flag needed |
| `ros2 service list` | See available ROS services | No flag needed |

---
### <span style="color:SkyBlue">Ignition Cheat-Sheet</span>
Use these to debug the "Physical World." Remember: Ignition is strict—it usually requires the `-t` for topics and `-s` for services.

| Command | Action | Arg Notation |
| :--- | :--- | :--- |
| `ign topic -l` | List all physics topics | No flag needed |
| `ign gazebo [world.sdf]` | Launch Gazebo standalone | `-r` (Run), `-s` (Server/Headless) |
| `ign topic -e -t [topic]` | Echo physics data (e.g., Pose) | `-t` (Target Topic) |
| `ign service -l` | List physics services | No flag needed |
| `ign service -s [srv]` | Call a service (like set_physics) | `--reqtype`, `--reptype`, `--req` |
| `ign model -l` | List all models in the world | No flag needed |



---
### <span style="color:SkyBlue">Troubleshooting Quick-Fixes
* **Sourcing Mantra:** If a package isn't found, run:  

  `source /opt/ros/iron/setup.bash && source install/setup.bash`
* **Ghost Processes:** If Gazebo won't open, kill the "zombie" processes:  

  `pkill -9 ruby` or `pkill -9 ign`
* **Topic Search:** Use `grep` to find specific sensors:

  `ros2 topic list | grep lidar`