# Homeokinesis (`homeokinesis`)

The self-organising homeokinesis controller ("Sox", from The Playful Machine, Der and Martius, 2012) running on the robot. It learns a model of how its commands move the joints, and keeps adapting its controller so that movement emerges by itself.

| File | What it is |
|---|---|
| `QuteeHomeokinesis.hpp`, `QuteeHomeokinesis.cpp` | `QuteeHomeokinesis`, derived from `QuteeNormalised`: the controller task, its menu entry and its ROS parameters |
| `main.cpp` | Creates a `QuteeHomeokinesis` and runs it |
| `homeokinesis_gui.py` | PC window: start/stop/reset, parameters, joints |
| `../../main/QuteeNormalised.hpp`, `.cpp` | Joints normalised to [-1, 1] over a movement range |
| `../../components/intrinsic_motivations/homeokinesis/cpp/portable/` | The controller (submodule [intrinsic_motivations](https://github.com/wearerobotslab/intrinsic_motivations)) |

## How it runs

- **At 50 Hz in its own task, from boot.** Sensors are the 12 joint positions and outputs the 12 joint targets, both normalised to the **walking** range: hips -45° to 45°, joints 2 -45° to 45°, knees 0° to 90° (forward). 0 is the centre of each range (knees at 45°).
- **Clipping:** targets are clipped to the range. Sensors use the same scale and read beyond ±1 when a joint is outside it.
- **Noise:** white noise is added to the sensors and outputs, because the servos are too precise for homeokinesis to amplify their own fluctuations.

**Start**, on the floor: **Start homeokinesis** in the menu, the GUI's **Start**, or `ros2 param set /qutee/qutee_node running true`. The robot stands up, moves slowly to the centre of the range, and the controller starts. **Stop** brings it back to rest. You can start and stop it as often as you like.

**Safety:** one motor's error status is read each step. On an overload, that motor is rebooted and the controller continues (`auto_recover`), unless the same motor overloads 3 times in a minute. Any other fault stops homeokinesis and sends the robot to rest. While it runs, the checkup is refused.

## ROS

Everything from the [base robot](../qutee/README.md) (`status`, `joint_states`, `torque_off_at_rest`), plus these parameters of `/<name>/qutee_node`, changeable while it runs:

| Parameter | Default | What it does |
|---|---|---|
| `running`, `reset` | false | Start/stop; `reset true` clears what the controller has learned |
| `eps_C`, `eps_A` | 0.5, 0.5 | Learning rates of the controller and the model (0: the robot stops moving) |
| `amplitude` | 0.8 | Targets = output x amplitude (normalised) |
| `sensor_noise`, `motor_noise` | 0.05, 0.05 | Uniform white noise in [-n, n] (normalised: 1 = half a joint's range) |
| `motor_smoothing` | 0 | Filters the targets (0 to 0.99) |
| `creativity`, `harmony`, `damping`, `causeaware` | 0.1, 0.1, 1e-5, 0.01 | Further Sox parameters |
| `steps_delay`, `steps_averaging` | 10 (used as 9) | In steps, 1 to 9 |
| `servo_profile_ms`, `servo_p_gain` | 20, 400 | Servo smoothness and stiffness while it runs |
| `servo_pwm_limit` | 700 | Maximum motor output (885 = 100%); kept after stopping, back to 885 at boot |
| `auto_recover` | true | Recover from single overloads |

Defaults come back at every restart: save good sets with the GUI or `ros2 param dump`. More detail in [USING_THE_ROBOT.md](../../../USING_THE_ROBOT.md#homeokinesis).

## Build, flash and the GUI

From `software/`, in WSL or Linux:

```bash
./build.sh homeokinesis        # -> build/homeokinesis/qutee_homeokinesis.bin
```

Flash from `build/homeokinesis` (see [USING_THE_ROBOT.md](../../../USING_THE_ROBOT.md#flash)). The GUI needs PySide6 and roslibpy (`pip install PySide6 roslibpy`; on Windows the `qutee_gui` conda environment has them):

```bash
python homeokinesis_gui.py            # or: python homeokinesis_gui.py <robot name>
```

Click **Start agent** and **Start rosbridge** (the first start builds the rosbridge image: a few minutes), then **Connect**. Double-click a value and press Enter to send it, or tick a box to switch it.
