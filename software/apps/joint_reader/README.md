# Joint reader

A small firmware and GUI to **read the robot's 12 joint positions while you move the legs by hand**, for example to find the range each joint can move safely. The robot starts ROS by itself at boot: no menu, no homeokinesis, no rollouts.

| File | What it is |
|---|---|
| `JointReader.hpp`, `JointReader.cpp` | Minimal version of `Qutee`: screen and motors only |
| `joint_reader_main.cpp` | Entry point: motors with the torque off, WiFi, ROS (publisher and parameter) |
| `joint_reader_gui.py` | PC window: angles, min/max, torque on/off, save ranges |
| `../../tools/rosbridge.Dockerfile`, `../../tools/qutee_docker.py` | rosbridge for the GUI, and starting it and the agent (shared with the homeokinesis GUI) |

## On the robot

**At boot** the motors' torque is switched **off** (the legs move freely), then WiFi and ROS start; if the agent isn't reachable, the robot retries every 2 s. The screen shows the 12 angles (one row per leg, one column per joint) and a status line.

**ROS**, under the robot's name (its ROS namespace, as in the full firmware):

| | |
|---|---|
| `/<name>/joint_states` | `sensor_msgs/JointState`, 20 Hz. Names `"11"`...`"43"` (motor IDs), positions in **radians** from the standing pose (0 = 2048 counts) |
| `/<name>/joint_reader` parameter `torque` | `true`: the motors hold where the joints are now. `false` (default): move the legs by hand |

The joint reader uses a different micro-ROS client key from the full firmware, so the agent keeps the two firmwares' sessions apart: switching between them doesn't need an agent restart.

## Build and flash

The joint reader is one of the applications of the firmware project. From `software/`, in WSL or Linux:

```bash
./build.sh joint_reader        # -> build/joint_reader/qutee_joint_reader.bin
```

Flash as usual (`--flash-size 4MB`, see USING_THE_ROBOT.md), from `build\joint_reader`. Unlike the other applications, it doesn't use the `Qutee` class yet: it has its own small robot class.

## On the PC

The GUI needs PySide6 and roslibpy (`pip install PySide6 roslibpy`; on Windows the `qutee_gui` conda environment has them):

```bash
python joint_reader_gui.py            # or: python joint_reader_gui.py <robot name>
```

1. **Start agent** and **Start rosbridge** (the first start builds the rosbridge image: a few minutes), then **Connect**.
2. Move one joint at a time by hand, through its whole range. The table shows the current angle and the minimum and maximum since the last **Reset min/max**.
3. **Save ranges...** writes a CSV with the limits in degrees and in motor counts (2048 = standing pose, 4096 per turn), for the firmware or for the motors' own Min/Max Position Limit in Dynamixel Wizard.

Without the GUI, from any ROS 2 shell: `ros2 topic echo /qutee/joint_states` and `ros2 param set /qutee/joint_reader torque true`.

Only one rosbridge can use port 9090: stop another one first (for example the full GUI's `qutee_rosbridge`).
