# Using the Qutee V2 robot (`qutier` branch)

This guide covers building the firmware, flashing it, watching the robot's logs and starting it in ROS mode, on Windows and on Linux.

> **Status: work in progress.**
> - **Tested:** building the firmware, flashing and monitoring on Windows 11, boot, screen and menu, battery monitor.
> - **Not tested yet:** the motors (they must be configured first, see [Setting up the motors](#setting-up-the-motors)), the connection to the micro-ROS agent, the ROS services, and the whole Linux section.

## Contents
- [Hardware](#hardware)
- [Get the code](#get-the-code)
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
| 12 x Dynamixel XL330-M288-T | IDs 11-13, 21-23, 31-33, 41-43 (leg x 10 + joint), 3 Mbps, protocol 2.0. Data on the Feather's A0/A1 pins. |
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

## Configure the firmware

The firmware is built in a Docker image (`qutee_idf`), on both Windows and Linux.

Settings are changed with `idf.py menuconfig` inside the container (see the Windows or Linux section for how to open it). The ones that matter:

| Menu | Setting | What to set |
|---|---|---|
| **micro-ROS Settings → WiFi Configuration** | WiFi SSID / Password | Your network. The ESP32-S3 only supports 2.4 GHz. |
| **micro-ROS Settings** | micro-ROS Agent IP / Port | IP of the computer running the micro-ROS agent, port `8888`. |
| **QUTEE settings** | Name of the Qutee robot | One name, or several separated by `;` (you pick one on the robot). Used as the ROS namespace, so use only letters, digits and `_`. |

Press **S** to save and **Q** to quit, then rebuild.

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
   ```
   Also make sure your WiFi network is set to **Private** in Windows.

### Build the Docker image (once, or after changing the Dockerfile)

In a WSL terminal:

```bash
cd ~/Qutee_v2/software
docker build -t qutee_idf docker/
```

Add `--no-cache` after changing the Dockerfile.

### Configure and build the firmware

In a WSL terminal, from `software/`, open a shell in the container:

```bash
docker run -it --rm --user espidf -v "$(pwd)":/micro_ros_espidf_component -w /micro_ros_espidf_component qutee_idf bash
```

You must be in `software/` when you run this, because `$(pwd)` is the folder that gets mounted. Keep the mount path `/micro_ros_espidf_component`, which the build scripts expect. Then, inside the container:

```bash
. $IDF_PATH/export.sh
idf.py menuconfig    # only when you need to change settings
idf.py build
```

A successful build ends with `Successfully created esp32s3 image` and produces `build/QuteeV2.bin`. The first build compiles micro-ROS from source and takes a long time; later builds take a minute or two.

To build in one step without entering the container:

```bash
docker run --rm --user espidf -v "$(pwd)":/micro_ros_espidf_component -w /micro_ros_espidf_component qutee_idf bash -lc '. $IDF_PATH/export.sh && idf.py build'
```

### Flash

Connect the Feather by USB and find its port (it was `COM3` on our machine):

```powershell
conda activate robot
python -m serial.tools.list_ports -v
```

Then, from the build folder (the WSL path works directly from Windows):

```powershell
cd \\wsl.localhost\Ubuntu\home\<user>\Qutee_v2\software\build
python -m esptool --chip esp32s3 -p COM3 -b 460800 --before default-reset --after hard-reset write-flash --flash-mode dio --flash-freq 40m --flash-size 2MB 0x0 bootloader\bootloader.bin 0x8000 partition_table\partition-table.bin 0x10000 QuteeV2.bin
```

- If esptool can't connect: hold **Boot**, press and release **Reset**, release **Boot**, and try again (the port number may change in this mode). Press **Reset** after flashing.
- esptool v5 uses hyphenated names (`write-flash`, `default-reset`). Older versions print deprecation warnings for the underscore names but still work.
- Close the serial monitor first; esptool can't open a port that another program is using.

### Watch the logs

```powershell
python ..\tools\serial_monitor.py COM3
```

`software/tools/serial_monitor.py` reconnects automatically when the board resets (the Feather's USB port disappears for a moment on every reset) and shows the log colours. Quit with **Ctrl+C**, **Ctrl+X** or **q**.

If the board was reset with its **Reset** button and is now quiet (for example in the menu), quitting takes about 30 s: the monitor prints `Closing COM3 ...` and Windows' USB serial driver takes that long to release the port. The port can't be used (for example by esptool) until the monitor has exited.

The first half second of boot output can be lost after a reset while Windows re-detects the port; everything from the application is shown.

If you use pyserial's `miniterm` instead, add `--exit-char 24` so **Ctrl+X** quits: VS Code's terminal captures miniterm's default Ctrl+], and ESP-IDF's colour codes show up as `␛[0;32m`.

### Run the micro-ROS agent

In a WSL terminal:

```bash
docker run -it --rm -p 8888:8888/udp microros/micro-ros-agent:humble udp4 --port 8888 -v6
```

If the robot can't reach it, enable **Docker Desktop → Settings → Resources → Network → Enable host networking** and use `--net=host` instead of `-p 8888:8888/udp`.

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
idf.py menuconfig                    # only when you need to change settings
idf.py build
idf.py -p /dev/ttyACM0 flash
```

You can also flash from the host instead, with `pip install esptool` and the same esptool command as in the Windows section, using `-p /dev/ttyACM0` and `/` in the paths.

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

The same information, in more detail, is in the serial log.

### The menu (Boot button)

| Action | Effect |
|---|---|
| Short press | Move the `>` marker to the next option. |
| Long press (about 0.7 s, "Selected" appears) | Choose the option. |

Options:
- **Select Name**: pick the robot's name from the list set in menuconfig (short press: next name, long press: choose). The choice is remembered across reboots. Until a name is chosen, the robot uses the first name in the list.
- **Start Checkup**: moves all motors in a slow sine wave and shows each motor's average position error, one row per leg and one column per joint. Values above 0.1 are red. Hold **Boot** until the menu comes back to stop it.
- **Start ROS**: connects to WiFi and the micro-ROS agent. You can't go back to the menu from ROS mode; press **Reset**.

### ROS mode

The screen shows the robot's name and "ROS mode", and the status line shows:

| Status line | Meaning |
|---|---|
| Connecting to WiFi `<SSID>` | |
| WiFi connected, IP `x.x.x.x` | |
| WiFi failed: check SSID/password | Fix the settings in menuconfig, rebuild and flash. |
| Connecting to agent `<IP>:<port>` | |
| Connected to agent, ready | The robot's services are available. |
| Agent unreachable: start it, then reset | Start the agent (and check the IP, firewall and networking), then press **Reset**. |

When connected, the robot offers two services under its name (for example `/qutee/status` and `/qutee/rollout`):
- **status**: battery voltage, number of policy weights, error message.
- **rollout**: runs one episode (`EPISODE_DURATION` seconds at `CONTROL_FREQUENCY` Hz, both in menuconfig) with the neural network weights you send, and returns the states and actions.

Calling them needs ROS 2 Humble and the `qutee_interface` package (`software/main/src/Qutee_interface`) built in a ROS 2 workspace. This hasn't been tested yet.

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
| Build fails after an earlier failure | Clean the build output (from `software/`, in WSL or Linux): `sudo rm -rf build main/build main/install main/log main/include main/libquteeinterface.a managed_components components/micro_ros_espidf_component/micro_ros_* components/micro_ros_espidf_component/include components/micro_ros_espidf_component/libmicroros.a` |
| Screen stays black | Firmware built for the wrong board. Check `CONFIG_ARDUINO_VARIANT` in `sdkconfig` (see the next section). |
| Serial monitor shows nothing | The robot is waiting in the menu, which only logs once. Press **Reset** with the monitor running. |
| `task_wdt: Task watchdog got triggered` during "Setting up motors" | The motors aren't answering; the library keeps the CPU busy while it waits. Harmless, but see [Setting up the motors](#setting-up-the-motors). |
| A motor's LED blinks red | The motor has a hardware error and has turned its torque off. The most common cause is powering the robot through USB with the battery off: the Feather's charger then feeds the motors through VBAT at too low a voltage. At boot the firmware waits for the battery and reboots motors with an error, logging each motor's voltage and error bits (`input-voltage`, `overheating`, `encoder`, `electrical-shock`, `overload`). If it comes back, read **Hardware Error Status** (address 70) in Dynamixel Wizard. |
| `Wire.cpp ... requestFrom(): ... Error -1` at boot | Printed while the battery monitor resets. Harmless. |
| `Detected size(4096k) larger than the size in the binary image header(2048k)` | Harmless: the firmware uses 2 MB of the board's 4 MB flash. |
| Robot can't reach the agent from Windows | Check mirrored networking (`hostname -I` in WSL shows the Windows IP), the firewall rule, that the WiFi network is Private, and the Agent IP in menuconfig. |

## Building for the Reverse TFT Feather

The upstream Qutee V2 uses the **ESP32-S3 Reverse TFT Feather**, which has three buttons (D0, D1, D2) and different pins. This branch defaults to the regular ESP32-S3 TFT Feather. To build for the Reverse TFT, set this in `software/sdkconfig` (and in `sdkconfig.defaults` for fresh builds):

```
CONFIG_ARDUINO_VARIANT="adafruit_feather_esp32s3_reversetft"
```

The firmware then uses the original three-button menu: D0 = Select Name, D1 = Start Checkup, D2 = Start ROS. In the name list, D0 = up, D1 = select, D2 = down. D0 also exits the checkup.
