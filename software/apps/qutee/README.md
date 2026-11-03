# Base robot (`qutee`)

The `Qutee` class on its own, with no controller. Use it to check the robot (motors, screen, WiFi, ROS) and as the starting point for a new application.

| File | What it is |
|---|---|
| `main.cpp` | Creates a `Qutee` and runs it |
| `../../main/Qutee.hpp`, `Qutee.cpp` | The base class that every application uses |

## What it does

At boot it sets up the motors, the screen, the IMU and the battery monitor, then moves slowly to the rest pose. Then it connects to WiFi and the micro-ROS agent, retrying every 2 s until the agent answers, and shows the menu.

**Menu** (short press: next, long press: select): Select Name, Checkup, and Calibrate IMU (only with an IMU fitted).

**ROS**, under the robot's name (its ROS namespace):

| | |
|---|---|
| `/<name>/status` service | Battery voltage, number of policy weights (0 here), motor problems |
| `/<name>/joint_states` topic | `sensor_msgs/JointState`, 20 Hz, best effort: the 12 joint positions in radians from the standing pose |
| `/<name>/qutee_node` parameter `torque_off_at_rest` | Motors limp in the rest pose |

## Build and flash

From `software/`, in WSL or Linux:

```bash
./build.sh qutee        # -> build/qutee/qutee_qutee.bin
```

Flash from `build/qutee` with the esptool command in [USING_THE_ROBOT.md](../../../USING_THE_ROBOT.md#flash) (`--flash-size 4MB`).

## Writing a new application

1. Create `software/apps/<app>/` with a class derived from `Qutee` (or `QuteeNormalised` for joints in [-1, 1]) and a `main.cpp` whose `app_main()` calls `run()`.
2. Override the hooks you need (see `Qutee.hpp`):
   - `start()`: e.g. start a controller task
   - `app_menu_items()` and `on_app_menu_item()`: your menu entries
   - `ros_setup()`, `ros_handles()`, `ros_parameters()`, `ros_parameter_changed()` and `ros_spin()`: services and parameters
3. Take the motors with `acquire_motors()` before moving them, and `release_motors()` after.
4. Add the name to `QUTEE_APPS` in `software/CMakeLists.txt` and to `build.sh`.
5. Give the class its own micro-ROS client key salt (the second argument of the constructor).
