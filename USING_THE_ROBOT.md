# Using the Qutee V2 robot (`qutier` branch)

This guide covers building the firmware, flashing it, watching the robot's logs and using it over ROS, on Windows and on Linux.

> **Status: work in progress.**
> - **Tested on Windows 11:** building the firmware, flashing and monitoring, boot, screen and menu, battery monitor, the motors and the checkup, the connection to the micro-ROS agent, and the `status` and `rollout` services.
> - **Built but not tested on the robot yet:** the firmware split into applications (neural network, homeokinesis), with ROS started at boot (branch `qutier_split`).
> - **Not tested yet:** the whole Linux section, and the IMU (our robot has none fitted).

## Contents
- [Hardware](#hardware)
- [Get the code](#get-the-code)
- [The applications](#the-applications)
- [Configure the firmware](#configure-the-firmware)
- [Windows](#windows)
- [Linux](#linux)
- [Using the robot](#using-the-robot)
- [Setting up the motors](#setting-up-the-motors)
- [Troubleshooting](#troubleshooting)
- [Building for the Reverse TFT Feather](#building-for-the-reverse-tft-feather)

## Hardware

| Part | Notes |
|---|---|
| Adafruit **ESP32-S3 TFT Feather** (regular, not Reverse) | One button (**Boot**, labelled D0), 240x135 screen. The upstream project uses the Reverse TFT; see [the last section](#building-for-the-reverse-tft-feather) for that board. |
| 12 x Dynamixel XL330-M288-T | IDs 11-13, 21-23, 31-33, 41-43 (leg x 10 + joint), 3 Mbps, protocol 2.0. Data on the Feather's A0/A1 pins. An XL330-M077 also works (same control table and position scale) but has less than half the torque and is about 3.7 times faster, so that joint is weaker; our robot has one at ID 11. |
| ROBOTIS U2D2 PHB power board | Powers the motors and the Feather (VBAT pin). The U2D2 USB adapter in the same set is used to configure the motors. |
| 4.8 V NiMH battery | About 5.5 V when fully charged. The Feather's MAX17048 monitor reads it. |
| BNO055 IMU (optional) | STEMMA QT connector. Without it, the firmware runs and reports orientation and acceleration as zero. |

## Get the code

```bash
git clone --recursive -b qutier https://github.com/wearerobotslab/Qutee_v2.git
cd Qutee_v2
```

The submodules are several GB, so the clone takes a while. If you already have the repo, or after switching branches:

```bash
git checkout qutier
git submodule update --init --recursive
```

All the commands below run from `Qutee_v2/software` unless stated otherwise.

## The applications

The firmware is one robot, `Qutee` (in `software/main`), plus one **application** chosen when building. Each application is its own binary; flash the one you want.

| Application | Folder | What it does |
|---|---|---|
| `nn` | `software/apps/nn` | Neural network policy. Its weights come over ROS: the `rollout` service runs one episode and returns the states and actions. |
| `homeokinesis` | `software/apps/homeokinesis` | The homeokinesis controller runs on the robot; start, stop and tune it over ROS or from the menu. |
| `joint_reader` | `software/apps/joint_reader` | Torque off, publishes the joint positions, for reading the joint ranges by hand (see its README). |

`Qutee` does what every application needs: it sets up the motors, the screen, the IMU and the battery monitor, moves slowly to the rest pose, connects to WiFi and starts ROS **at boot** (node `/<name>/qutee_node`, with the `status` service), and shows the [menu](#the-menu-boot-button). The `nn` and `homeokinesis` applications derive from it and add their controller, menu entries and ROS services or parameters. A new application is a new folder in `software/apps/` with a class derived from `Qutee` and an `app_main()` that calls `run()`.

## Configure the firmware

The firmware is built in a Docker image (`qutee_idf`), on both Windows and Linux.

Settings are changed with menuconfig, `./build.sh nn menuconfig` (any application name works: all applications share the same settings). The ones that matter:

| Menu | Setting | What to set |
|---|---|---|
| **micro-ROS Settings → WiFi Configuration** | WiFi SSID / Password | Your network. The ESP32-S3 only supports 2.4 GHz. |
| **micro-ROS Settings** | micro-ROS Agent IP / Port | IP of the computer running the micro-ROS agent, port `8888`. |
| **QUTEE settings** | Name of the Qutee robot | One name, or several separated by `;` (you pick one on the robot). Used as the ROS namespace, so use only letters, digits and `_`. |
| **QUTEE settings** | Leg angle that puts the torso on the ground | Default 60°. See [Poses](#poses). |
| **QUTEE settings** | Leg angle of the rest pose | Default 35°: the torso rests on the ground. See [Poses](#poses). |
| **QUTEE settings** | Turn the motors' torque off in the rest pose | Default on. Turn it off if the robot hangs on a stand, so the legs stay up. |
| **QUTEE settings** | Duration of a slow move between poses | Default 1000 ms. |
| **QUTEE settings** | UDP port for the logs over WiFi | Default 8889; 0 turns it off. See [Watch the logs over WiFi](#watch-the-logs-over-wifi). |
| **QUTEE settings → Neural network controller** | Hidden layers, neurons per layer, episode duration | Used by the `nn` application. |

Press **S** to save and **Q** to quit, then rebuild. After adding a new setting to `main/Kconfig.projbuild`, run `./build.sh <app> reconfigure` before building so that it appears in `sdkconfig`.

The settings are saved in `software/sdkconfig`, which git ignores, so your WiFi password is not committed. Do not put your own WiFi details in `sdkconfig.defaults`, which is committed.

## Windows

Tested on Windows 11 with WSL2 (Ubuntu) and Docker Desktop. The repo lives inside WSL. Docker on Windows cannot see USB devices, so flashing and the serial monitor run on Windows itself, with Python.

### One-time setup

1. **WSL2 and Docker Desktop.** Install both, and enable Docker's WSL integration for your Ubuntu distribution. Clone the repo inside WSL (for example in `~/Qutee_v2`), not on the Windows drive.
2. **Python with esptool and pyserial**, on Windows. Any Python 3 works; with conda:
   ```powershell
   conda create -n robot python=3.12
   conda activate robot
   python -m pip install esptool pyserial
   ```
   Use `python`, not `py`, inside a conda environment. If `python` opens the Microsoft Store, the environment isn't active.
3. **Networking for the micro-ROS agent** (only if the agent runs in WSL or Docker). By default WSL2 has its own internal network that the robot can't reach. Create `C:\Users\<you>\.wslconfig` with:
   ```ini
   [wsl2]
   networkingMode=mirrored
   ```
   then run `wsl --shutdown` in PowerShell and restart Docker Desktop. WSL now shares Windows' IP address (check with `hostname -I` in WSL and `ipconfig` in Windows). Use that IP as the micro-ROS Agent IP.
4. **Firewall.** In PowerShell **as Administrator**:
   ```powershell
   New-NetFirewallRule -DisplayName "micro-ROS agent UDP 8888" -Direction Inbound -Protocol UDP -LocalPort 8888 -Action Allow
   New-NetFirewallRule -DisplayName "Qutee logs UDP 8889" -Direction Inbound -Protocol UDP -LocalPort 8889 -Action Allow
   ```
   The second rule is for [the logs over WiFi](#watch-the-logs-over-wifi). Also make sure your WiFi network is set to **Private** in Windows.

### Build the Docker image (once, or after changing the Dockerfile)

In a WSL terminal:

```bash
cd ~/Qutee_v2/software
docker build -t qutee_idf docker/
```

Add `--no-cache` after changing the Dockerfile.

### Configure and build the firmware

In a WSL terminal, from `software/`, build the [application](#the-applications) you want with `build.sh`, which runs `idf.py` in the container:

```bash
./build.sh nn menuconfig      # only when you need to change settings
./build.sh nn                 # -> build/nn/qutee_nn.bin
./build.sh homeokinesis       # -> build/homeokinesis/qutee_homeokinesis.bin
```

A successful build ends with `Successfully created esp32s3 image`. Each application has its own build folder, `build/<app>`, so switching between them doesn't rebuild everything. The first build compiles micro-ROS from source and takes a long time; later builds take a minute or two.

`./build.sh <app> <command>` runs any other `idf.py` command for that application (for example `fullclean`). Without the script, the container command is:

```bash
docker run --rm --user espidf -v "$(pwd)":/micro_ros_espidf_component -w /micro_ros_espidf_component qutee_idf bash -lc '. $IDF_PATH/export.sh && idf.py -B build/nn -DQUTEE_APP=nn build'
```

You must be in `software/`, because `$(pwd)` is the folder that gets mounted. Keep the mount path `/micro_ros_espidf_component`, which the build scripts expect.

### Flash

Connect the Feather by USB and find its port (it was `COM3` on our machine):

```powershell
conda activate robot
python -m serial.tools.list_ports -v
```

Then, from the application's build folder (the WSL path works directly from Windows), for example the neural network:

```powershell
cd \\wsl.localhost\Ubuntu\home\<user>\Qutee_v2\software\build\nn
python -m esptool --chip esp32s3 -p COM3 -b 460800 --before default-reset --after hard-reset write-flash --flash-mode dio --flash-freq 40m --flash-size 4MB 0x0 bootloader\bootloader.bin 0x8000 partition_table\partition-table.bin 0x10000 qutee_nn.bin
```

For homeokinesis, use `build\homeokinesis` and `qutee_homeokinesis.bin`.

- If esptool can't connect: hold **Boot**, press and release **Reset**, release **Boot**, and try again (the port number may change in this mode). Press **Reset** after flashing.
- esptool v5 uses hyphenated names (`write-flash`, `default-reset`). Older versions print deprecation warnings for the underscore names but still work.
- Close the serial monitor first; esptool can't open a port that another program is using.

### Watch the logs

```powershell
python ..\..\tools\serial_monitor.py COM3
```

`software/tools/serial_monitor.py` reconnects automatically when the board resets (the Feather's USB port disappears for a moment on every reset) and shows the log colours. Quit with **Ctrl+C**, **Ctrl+X** or **q**.

If the board was reset with its **Reset** button and is now quiet (for example in the menu), quitting takes about 30 s: the monitor prints `Closing COM3 ...` and Windows' USB serial driver takes that long to release the port. The port can't be used (for example by esptool) until the monitor has exited.

The first half second of boot output can be lost after a reset while Windows re-detects the port; everything from the application is shown.

If you use pyserial's `miniterm` instead, add `--exit-char 24` so **Ctrl+X** quits: VS Code's terminal captures miniterm's default Ctrl+], and ESP-IDF's colour codes show up as `␛[0;32m`.

### Watch the logs over WiFi

To run the robot without the USB cable, read its logs over WiFi instead:

```powershell
python ..\..\tools\wifi_monitor.py
```

Run it on the computer whose IP is the micro-ROS Agent IP (it needs the firewall rule for UDP 8889, see [One-time setup](#one-time-setup)). The robot joins WiFi at boot, once it has moved to the rest pose: the monitor then shows `Receiving from <robot IP>`, then the logs since the robot was switched on (it keeps the last 16 KB until WiFi connects), then the live logs. It needs only Python's standard library and quits at once with **Ctrl+C**, **Ctrl+X** or **q**. Lines containing "password" are never sent over the network.

### Run the micro-ROS agent

In a WSL terminal, start it in the background with a name:

```bash
docker run -d --name qutee_agent -p 8888:8888/udp microros/micro-ros-agent:humble udp4 --port 8888 -v6
```

- Watch it with `docker logs -f --tail 50 qutee_agent` (Ctrl+C stops watching, not the agent). When the robot connects you see `session established` and `replier created` lines (the `status` service, the parameter server's services and, for `nn`, `rollout`).
- Stop it with `docker stop qutee_agent`; start it again with `docker start qutee_agent`.
- Only one agent can use port 8888. `port is already allocated` means one is already running: check with `docker ps`.
- The agent can keep running while you reset the robot: the robot reconnects with the same client key and the agent replaces its old session. Each application uses its own client key, so switching between them doesn't mix up their services.
- Use `-p 8888:8888/udp`, not `--net=host`. On Windows, Docker Desktop's host networking makes the robot's packets appear to come from `127.0.0.1`, so the agent's replies never reach the robot ("Agent unreachable" on the screen).

## Linux

> Not tested yet on this robot. The commands follow the upstream instructions and the Windows setup above.

On Linux, Docker can access USB devices, so you can flash from inside the container.

### One-time setup

```bash
sudo usermod -aG dialout $USER   # access to serial ports; log out and back in
cd Qutee_v2/software
docker build -t qutee_idf docker/
```

The Feather appears as `/dev/ttyACM0` (check with `ls /dev/ttyACM*` after plugging it in).

### Configure, build and flash

From `software/`:

```bash
docker run -it --rm --user espidf -v "$(pwd)":/micro_ros_espidf_component -v /dev:/dev --privileged -w /micro_ros_espidf_component qutee_idf bash
```

`-v /dev:/dev --privileged` gives the container access to the serial port, including after the board resets and the port reappears. Inside the container:

```bash
. $IDF_PATH/export.sh
idf.py -B build/nn -DQUTEE_APP=nn menuconfig      # only when you need to change settings
idf.py -B build/nn -DQUTEE_APP=nn build           # or QUTEE_APP=homeokinesis, see The applications
idf.py -B build/nn -p /dev/ttyACM0 flash
```

You can also build with `./build.sh nn` on the host, and flash from the host instead, with `pip install esptool` and the same esptool command as in the Windows section, using `-p /dev/ttyACM0` and `/` in the paths.

### Watch the logs

On the host:

```bash
pip install pyserial
python3 tools/serial_monitor.py /dev/ttyACM0
```

Quit with **Ctrl+C**. Alternatively, `idf.py -p /dev/ttyACM0 monitor` inside the container (quit with **Ctrl+]**).

### Run the micro-ROS agent

```bash
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
```

Use your computer's IP on the robot's network (`hostname -I`) as the micro-ROS Agent IP. If you run a firewall, allow incoming UDP on port 8888 (for example `sudo ufw allow 8888/udp`).

## Using the robot

### Turning it on

Switch on the battery (or plug in USB for testing without the motors). The Feather's small orange **CHG** LED going off when the battery is on is normal: it is the charging light.

The screen shows "Hi! My name is ... Loading!", then a status line along the bottom for each boot stage:

| Status line | Meaning |
|---|---|
| Powered on | Firmware started. |
| IMU not found, continuing without it | No BNO055. Orientation and acceleration read as zero. |
| No battery monitor, continuing | Battery voltage reads as 0. |
| Switch on the motor battery | The motors answer but report less than 4.6 V: they are powered only through USB. Switch on the battery; the robot then continues by itself. |
| Motor N error 0xNN | Motor N still reports a hardware error after the firmware rebooted it (see [Troubleshooting](#troubleshooting)). |
| Setting up motors... | Configuring the 12 motors. Takes about 10 s when they don't answer. |
| Motors found: N/12 | Green if all 12 answer, yellow otherwise. |

The same information, in more detail, is in the serial log. The robot then settles into its [rest pose](#poses): it raises its legs so the torso comes down onto the ground, lowers them to the rest pose, and turns the motors' torque off. Then it connects to WiFi and the agent (see [ROS](#ros)) and shows the [menu](#the-menu-boot-button).

### Poses

The robot never jumps between poses: every change is a smooth move of about 1 s (menuconfig: QUTEE settings). Only joint 2 of each leg (motors 12, 22, 32, 42) changes; on all four legs a negative angle raises the leg.

| Pose | Joint 2 | Used |
|---|---|---|
| Legs up | -60° | Torso on the ground, legs off it, so every joint can realign without load. |
| Rest | -35° | After boot, after a rollout and after the checkup. The torso rests on the ground; with "Turn the motors' torque off in the rest pose" the motors then go limp to save the battery. |
| Standing | 0° | Start of each rollout and of the checkup. Reached from the rest pose through "legs up", because limp legs drift (hips splayed, knees folded) and can't lift the body from there. |

**Tune the rest angle on the floor.** With the torque turned off at rest, the torso must already rest on the ground: if the legs still carry the robot when they go limp, the torso drops and the falling legs can trip the motors (red LEDs). If the legs are lifted clearly into the air at rest, lower the angle a little so they don't fall when released.

### The menu (Boot button)

| Action | Effect |
|---|---|
| Short press | Move the `>` marker to the next option. |
| Long press (about 0.7 s, "Selected" appears) | Choose the option. |

The menu stays on the screen while the robot runs; ROS works at the same time. The application's entries come first, then the robot's own:
- **Start homeokinesis** / **Stop homeokinesis** (`homeokinesis` only): see [Homeokinesis](#homeokinesis).
- **Select Name**: pick the robot's name from the list set in menuconfig (short press: next name, long press: choose). The choice is remembered across reboots and, since the name is the ROS namespace, used after the next restart. Until a name is chosen, the robot uses the first name in the list.
- **Checkup**: stands up, then moves all motors in a slow sine wave and shows each motor's average position error, one row per leg and one column per joint. Values above 0.1 are red. Press **Boot** to stop it; the robot then goes back to rest.
- **Calibrate IMU** (only with an IMU fitted): move the robot until the IMU reports full calibration, then keep it still for 1 s; the calibration is saved and loaded at every boot. Press **Boot** to cancel.

Only one thing moves the motors at a time: a rollout, the checkup or homeokinesis. Starting another meanwhile is refused (the screen shows "Busy: … is running", and `status` reports it in `error_message`).

### ROS

ROS starts at boot. The ROS line, just above the status line at the bottom of the screen, shows:

| ROS line | Meaning |
|---|---|
| Connecting to WiFi `<SSID>` | |
| WiFi connected, IP `x.x.x.x` | |
| WiFi failed: check SSID/password | Fix the settings in menuconfig, rebuild and flash. |
| Connecting to agent `<IP>:<port>` | |
| Agent unreachable, retrying... | The robot retries every 2 s, so start the agent (and check the IP, firewall and networking); no reset needed. Meanwhile the menu works. |
| ROS ready: /`<name>`/qutee_node | The robot's services and parameters are available. |

Every application offers, under the robot's name (for example `/qutee/status`):
- **status** service: battery voltage, number of policy weights (0 without a neural network), and `error_message`: motor problems found during the last rollout or homeokinesis run (empty if none, see [Motor faults during rollouts](#motor-faults-during-rollouts)).
- **torque_off_at_rest** parameter of `/qutee/qutee_node`.
- **joint_states** topic (`/qutee/joint_states`, `sensor_msgs/JointState`, 20 Hz, best effort): the 12 joint positions, names `"11"`...`"43"` (motor IDs), in **radians** from the standing pose. Time stamps come from the agent's clock. While a rollout, the checkup or homeokinesis runs, these are the positions it reads; otherwise the robot reads them itself. During a rollout it pauses while the robot stands up and goes back to rest (about 2 s each, no position reads then), and publishes during the episode itself. Read it with `ros2 topic echo /qutee/joint_states` (add `--qos-reliability best_effort` if nothing shows).

The `nn` application adds:
- **rollout** service: runs one episode (`EPISODE_DURATION` seconds at `CONTROL_FREQUENCY` Hz, both in menuconfig) with the neural network weights you send, and returns the states and actions.

The `homeokinesis` application adds its [parameters](#homeokinesis).

### Calling the services

You need ROS 2 Humble with the `qutee_interface` package (`software/main/src/Qutee_interface`). The simplest way is a `ros:humble` container next to the agent. In a WSL terminal (or Linux), from `software/`:

```bash
docker run -it --rm -v "$(pwd)/main/src/Qutee_interface":/ws/src/qutee_interface:ro ros:humble bash
```

On Linux, add `--net=host` (the agent runs with `--net=host` there). On Windows, leave it out: the container and the agent share Docker's network. Inside the container:

```bash
source /opt/ros/humble/setup.bash
cd /ws && colcon build --packages-select qutee_interface
source install/setup.bash
ros2 service list                                           # shows /qutee/status (and /qutee/rollout with nn)
ros2 service call /qutee/status qutee_interface/srv/Status
```

The status reply looks like `Status_Response(battery=5.15, number_weights=322, error_message='')`. If `ros2 service list` shows nothing just after starting the container, wait a few seconds for ROS 2 discovery and try again.

To run a rollout (`nn` application), send the weights from Python (the `ros:humble` image includes numpy). **The legs move**: put the robot on a stand the first time.

```bash
python3 - <<'EOF'
import numpy as np, rclpy
from qutee_interface.srv import Rollout
from std_msgs.msg import Float32MultiArray

rclpy.init()
node = rclpy.create_node("rollout_example")
client = node.create_client(Rollout, "/qutee/rollout")
client.wait_for_service()
request = Rollout.Request()
request.weights = Float32MultiArray(data=np.random.uniform(-0.1, 0.1, 322).astype(np.float32).tolist())
future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)
reply = future.result()
states = np.array(reply.states.data).reshape(-1, 18)
actions = np.array(reply.actions.data).reshape(-1, 12)
print("states", states.shape, "actions", actions.shape)
EOF
```

The reply comes after about 10 s: the robot stands up from rest (legs up, then standing, about 2 s), waits 1 s, runs the 5 s episode and goes back to rest (about 1 s).

### Rollout data

- **Policy**: a neural network with 18 inputs, `NB_HIDDEN_LAYERS` hidden layers of `NB_NEURONS_PER_LAYER` neurons (menuconfig; 1 x 10 by default) and 12 outputs, all with tanh. Use `number_weights` from `status` (322 by default). The weights are the layers' matrices one after the other, each stored column by column (Eigen's default) with the bias in the last column: by default first the 10 x 19 input layer (index = neuron + 10 x input, input 18 is the bias), then the 12 x 11 output layer.
- **states**: one row per step, 18 values: orientation x, y, z (rad) and linear acceleration x, y, z (m/s²) from the IMU (zeros without one), then the 12 joint positions.
- **actions**: one row per step, 12 values in [-1, 1].
- Joint positions and actions use the same scale: 0 is the neutral pose and 1 is 45°. The joint order is 11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43.

### Motor faults during rollouts

Large weights (for example ±1) make the joints swing between ±45° at full speed, and a motor that can't keep up can trip on **overload** (its LED blinks red and its torque turns off). Before each rollout the firmware reboots any motor in that state and sets it up again, and `status` then reports it in `error_message`, for example `motor 43 overload (rebooted);`. `error_message` also reports `N incomplete motor reads;` when some position reads failed during the episode even after a retry (the state then keeps the last known position for those motors; replies get lost more often while the legs carry the robot), and `motor N torque did not turn on;` when a motor didn't accept the command after 3 attempts. Check `error_message` after each rollout and treat a non-empty one as a sign that the episode's data may be unreliable.

## Homeokinesis

Homeokinesis is a self-organising controller ("Sox", from The Playful Machine, Der and Martius, 2012): it learns a model of how its commands move the joints and continually adapts its controller to stay at the edge of instability, so movement emerges by itself. It runs on the robot, at 50 Hz, with the 12 joint positions as sensors and the 12 joint targets as outputs. The controller is the portable C++ version in the `intrinsic_motivations` submodule (`software/components/intrinsic_motivations/homeokinesis/cpp/portable`); the application is `software/apps/homeokinesis` (flash `qutee_homeokinesis.bin`).

**Normalised joints:** the controller works in [-1, 1] for every joint, mapped to a movement range (`QuteeNormalised`, in `software/main`). The range is "walking": hips -45° to 45°, joints 2 -45° to 45°, knees 0° to 90° (forward). So 0 is the centre of each range (knees at 45°), and a hip given 1 goes to 45°. Targets are clipped to the range; the sensors use the same scale and can read beyond ±1 when a joint is outside it.

**Start**, on the floor: choose **Start homeokinesis** in the menu, or `ros2 param set /qutee/qutee_node running true`. The robot stands up, moves slowly to the centre of the range (knees forward to 45°), and the controller starts. **Stop:** **Stop homeokinesis** in the menu, or `running false`: the robot goes back to rest. You can start and stop it as often as you like, without restarting the robot.

**GUI:** `software/apps/homeokinesis/homeokinesis_gui.py` starts, stops and resets the controller, shows and changes every parameter (read from the robot), saves and loads sets of values (JSON), and shows the joint positions in degrees and normalised. It needs PySide6 and roslibpy (the `qutee_gui` conda environment on Windows):

```powershell
conda activate qutee_gui
cd \\wsl.localhost\Ubuntu\home\<user>\Qutee_v2\software\apps\homeokinesis
python homeokinesis_gui.py          # or: python homeokinesis_gui.py <robot name>
```

Click **Start agent** and **Start rosbridge** (the first start builds the rosbridge image: a few minutes), then **Connect**. Only one rosbridge can use port 9090: stop another one first (`docker stop qutee_rosbridge`).

**Parameters**, from a ROS 2 shell (a WSL terminal on Windows; add `--net=host` on Linux):

```bash
docker run -it --rm --name qutee_ros -v ~/hk_params:/params ros:humble bash
source /opt/ros/humble/setup.bash
ros2 param dump /qutee/qutee_node                   # all values
ros2 param set /qutee/qutee_node eps_C 0.3          # decimals for decimal parameters ("0.0", not "0")
ros2 param dump /qutee/qutee_node > /params/good.yaml        # save a set of values
ros2 param load /qutee/qutee_node /params/good.yaml          # restore it (the robot starts with the defaults)
```

| Parameter | Default | What it does |
|---|---|---|
| `eps_C`, `eps_A` | 0.5, 0.5 | Learning rates of the controller and the model. The learning keeps the robot moving: with them at 0, it stops. |
| `amplitude` | 0.8 | Joint targets = output × amplitude, in normalised units (1 = the whole range). |
| `sensor_noise`, `motor_noise` | 0.05, 0.05 | Uniform white noise in [-n, n], normalised (1 = half a joint's range, 45° in "walking"), added to each joint reading and to each output. Homeokinesis amplifies fluctuations; the servos are too precise to provide them, so without noise the robot may need a push to start. |
| `servo_p_gain` | 400 | Servo stiffness while homeokinesis runs (rollouts and poses use 1600). Softer means fewer overloads. |
| `servo_profile_ms` | 20 | Time each servo takes to reach a new target (0 = as fast as possible): smoother movement and lower current peaks. |
| `servo_pwm_limit` | 700 | Maximum output of each motor (0 to 885 = 100%), to make overloads rarer. Set at rest with the torque off, so it also applies while standing up; changing it while running pauses homeokinesis (rest, change, stand up). It stays set when homeokinesis stops (also for the checkup), to spare the motors' EEPROM; back to 885 at boot. |
| `auto_recover` | true | On an overload, reboot only that motor, let it cool down 2 s and continue. A motor that overloads 3 times within a minute, or any other fault, stops homeokinesis. |
| `motor_smoothing` | 0 | Filters the targets (0 to 0.99). |
| `creativity`, `harmony`, `damping`, `causeaware` | 0.1, 0.1, 1e-5, 0.01 | Further Sox parameters. |
| `steps_delay`, `steps_averaging` | 10 (used as 9) | Delay between a command and its effect, and sensor smoothing, in steps (1 to 9). |
| `torque_off_at_rest` | menuconfig | Turn off when the robot hangs on a stand: there the released legs fall and the knees can trip. |
| `running`, `reset` | | Start/stop (follows the menu too); `reset true` clears what the controller has learned. |

The defaults were tuned on the floor; they reset whenever the robot restarts, so save good values with `ros2 param dump` and load them after a start.

**Safety:** each step one motor's error status is read. If a motor trips (red LED, usually overload), homeokinesis stops, the robot goes to rest, the screen shows "Motor N fault: stopped" and `status` reports it; the next start reboots the motor. Overloads come from a strong loop (high amplitude, stiff servos) with joints pushing against the floor. Several motors tripping at once with `input-voltage` errors means a weak battery: charge it.

**Log:** every second, `HK: step …` shows the controller's step time (about 4.5 ms of the 20 ms period), `|y|`, `|C|`, `|A|`, the prediction error and incomplete reads.

## Setting up the motors

The firmware expects the motors at IDs 11-13, 21-23, 31-33, 41-43 (leg x 10 + joint, matching the numbers on the robot's body) and 3 Mbps. New XL330s come as ID 1 at 57600 baud, so each one must be configured once. Use ROBOTIS **Dynamixel Wizard 2.0** (Windows and Linux) with the U2D2:

1. Connect the U2D2 by USB and to the power board's Dynamixel port, with the battery on.
2. In Dynamixel Wizard, **Scan** with protocol 2.0 and all baud rates to see what is connected.
3. Configure **one motor at a time** (only that motor connected, since motors with the same ID clash): set its **ID** and set **Baud Rate** to 3 Mbps.

When all 12 are set, the boot screen should show "Motors found: 12/12".

## Troubleshooting

| Problem | What to do |
|---|---|
| `CMakeLists.txt not found in project directory` | You started the container outside `software/`. Exit and `cd` there first. |
| micro-ROS build fails in `builtin_interfaces` | The Docker image is old. Rebuild it with `--no-cache` (the Dockerfile installs `lark` and `"empy<4"`). |
| `Choose the application with -DQUTEE_APP=<app>` | Build with `./build.sh <app>` (see [The applications](#the-applications)), not plain `idf.py build`. |
| Build fails after an earlier failure | Clean the build output (from `software/`, in WSL or Linux; this removes every application's build): `sudo rm -rf build main/build main/install main/log main/include main/libquteeinterface.a managed_components components/micro_ros_espidf_component/micro_ros_* components/micro_ros_espidf_component/include components/micro_ros_espidf_component/libmicroros.a` |
| Screen stays black | Firmware built for the wrong board. Check `CONFIG_ARDUINO_VARIANT` in `sdkconfig` (see the next section). |
| Serial monitor shows nothing | The robot is waiting in the menu, which only logs once. Press **Reset** with the monitor running, or use [the logs over WiFi](#watch-the-logs-over-wifi), which start with the boot logs. |
| `task_wdt: Task watchdog got triggered` during "Setting up motors" | The motors aren't answering; the library keeps the CPU busy while it waits. Harmless, but see [Setting up the motors](#setting-up-the-motors). |
| A motor's LED blinks red | The motor has a hardware error and has turned its torque off. The most common cause is powering the robot through USB with the battery off: the Feather's charger then feeds the motors through VBAT at too low a voltage. At boot the firmware waits for the battery and reboots motors with an error, logging each motor's voltage and error bits (`input-voltage`, `overheating`, `encoder`, `electrical-shock`, `overload`). If it comes back, read **Hardware Error Status** (address 70) in Dynamixel Wizard. |
| Motors turn red when the robot goes to rest | The legs were still carrying the robot when the torque turned off. Increase "Leg angle of the rest pose" until the torso rests on the ground first (see [Poses](#poses)). |
| The robot doesn't stand up at the start of a rollout | Check the battery (`status`): below about 4.9 V the motors may not lift the body. The serial or WiFi log shows where joint 2 of each leg ended up after each move (`Pose: joint 2 target ... reached ...`). |
| `Wire.cpp ... requestFrom(): ... Error -1` at boot | Printed while the battery monitor resets. Harmless. |
| `Detected size(4096k) larger than the size in the binary image header(2048k)` | Firmware from before the homeokinesis branch, which used 2 MB of the 4 MB flash. Harmless. The current firmware uses all 4 MB (flash with `--flash-size 4MB`). |
| After flashing, `ros2 param get` times out, or some of the robot's services are missing | The new firmware creates different services, and the agent reused the old ones by number (the robot reconnects with the same client key). Restart the agent once: `docker restart qutee_agent`, then reset the robot. |
| Robot can't reach the agent from Windows | Check mirrored networking (`hostname -I` in WSL shows the Windows IP), the firewall rule, that the WiFi network is Private, the Agent IP in menuconfig, and that the agent runs with `-p 8888:8888/udp` (not `--net=host`). |
| `ros2 service list` doesn't show the robot's services | Check that the robot shows "ROS ready" and that the agent's log (`docker logs qutee_agent`) shows `replier created` lines for its session. |

## Building for the Reverse TFT Feather

The upstream Qutee V2 uses the **ESP32-S3 Reverse TFT Feather**, which has three buttons (D0, D1, D2) and different pins. This branch defaults to the regular ESP32-S3 TFT Feather. To build for the Reverse TFT, set this in `software/sdkconfig` (and in `sdkconfig.defaults` for fresh builds):

```
CONFIG_ARDUINO_VARIANT="adafruit_feather_esp32s3_reversetft"
```

The menu then uses the three buttons: D0 = previous entry, D1 = select, D2 = next entry, also in the name list. Any button stops the checkup.
