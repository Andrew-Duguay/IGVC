## <span style="color:SkyBlue">Launching Simulation Only</span>
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


## <span style="color:SkyBlue">Launching Robot Only</span>
| Command | Action | Arg Notation |
| --- | --- | --- |
| ```ros2 launch startup_robot robot.launch.py``` | Launch all ROS code | ROS (e.g., `arg:=val`) |

## <span style="color:SkyBlue">Launching Simulation + Robot</span>
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

## <span style="color:SkyBlue">Gazebo Cheat-Sheet</span>

## <span style="color:SkyBlue">ROS Cheat-Sheet</span>