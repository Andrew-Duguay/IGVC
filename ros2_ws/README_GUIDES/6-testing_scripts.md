# Launch Scripts

## 1. Single Test
This runs a single unit sim. It launches your robot alongside it automatically and allows you to evaluate this one, specific test you want to see. From ```ros2_ws```:
* **COMMAND**: ```python3 run_single_test.py```
* **ARGUMENTS**: All in python argument notation. Examples using the default values:
    * ```--world template```
    * ```--speed 1.0```
    * ```--gui true```
    * ```--step_size 0.001```
    * ```--timeout 30.0```

## 2. Batch Simulation
This is the bread and butter, the whole point of this simulation framework. This is how you launch a headless batch of simulations from the command line and recieve failure/success stats. From ```ros2_ws```:
* **COMMAND**: ```python3 run_all_tests.py```
* **ARGUMENTS**: All in python argument notation. Examples using the default values:
    * ```--speed 1.0```
    * ```--gui true```
    * ```--step_size 0.001```