"""Joint reader GUI: shows the 12 joint positions published by the robot's joint reader firmware, with the
minimum and maximum reached since the last reset (to find each joint's range by moving the legs by hand),
and turns the motors' torque on or off.

Needs the micro-ROS agent and rosbridge running (buttons below, or see README.md), and
  pip install PySide6 roslibpy           (the qutee_gui conda environment has them)
Run:  python joint_reader_gui.py [robot name]       (default: qutee)
Works on Windows (Docker Desktop through WSL) and Linux (native Docker).
"""
import csv
import math
import subprocess
import sys
import time
from pathlib import Path

import roslibpy
from PySide6.QtCore import QObject, Qt, QTimer, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (QApplication, QFileDialog, QHBoxLayout, QLabel, QLineEdit, QMainWindow, QPushButton,
                               QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

IDS = [11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43]
JOINT_NAMES = {1: "hip (x1)", 2: "joint 2 (up/down)", 3: "knee (x3)"}
WINDOWS = sys.platform == "win32"
WSL_DISTRO = "Ubuntu"
HERE = Path(__file__).resolve().parent


# ----- Docker: the agent and rosbridge -----
def docker(*args, timeout=300):
    cmd = (["wsl", "-d", WSL_DISTRO, "--", "docker"] if WINDOWS else ["docker"]) + list(args)
    flags = subprocess.CREATE_NO_WINDOW if WINDOWS else 0
    try:
        p = subprocess.run(cmd, capture_output=True, timeout=timeout, creationflags=flags)
    except (OSError, subprocess.TimeoutExpired) as e:
        return 1, str(e)
    return p.returncode, (p.stdout + p.stderr).decode("utf-8", errors="replace").replace("\x00", "").strip()


def docker_path(path):
    """On Windows Docker runs in WSL: \\\\wsl.localhost\\<distro>\\x -> /x, C:\\x -> /mnt/c/x."""
    s = str(path)
    if not WINDOWS:
        return s
    if s.lower().startswith(("\\\\wsl.localhost\\", "\\\\wsl$\\")):
        return "/" + "/".join(s.split("\\")[4:])
    if len(s) > 2 and s[1] == ":":
        return f"/mnt/{s[0].lower()}/" + s[3:].replace("\\", "/")
    return s.replace("\\", "/")


def start_container(name, run_args):
    code, state = docker("inspect", "-f", "{{.State.Status}}", name)
    if code == 0 and state == "running":
        return f"{name} already running"
    if code == 0:
        return docker("start", name)[1] or f"{name} started"
    return docker("run", "-d", "--name", name, *run_args)[1] or f"{name} started"


def start_agent():
    net = ["-p", "8888:8888/udp"] if WINDOWS else ["--net=host"]
    return start_container("qutee_agent", [*net, "microros/micro-ros-agent:humble", "udp4", "--port", "8888"])


def start_rosbridge():
    if docker("image", "inspect", "qutee_rosbridge_basic")[0] != 0:
        code, out = docker("build", "-t", "qutee_rosbridge_basic", "-f",
                           docker_path(HERE / "rosbridge.Dockerfile"), docker_path(HERE), timeout=1200)
        if code != 0:
            return "building rosbridge failed: " + out[-300:]
    net = ["-p", "9090:9090"] if WINDOWS else ["--net=host"]
    return start_container("qutee_rosbridge_basic", [*net, "qutee_rosbridge_basic"])


# ----- window -----
class Bridge(QObject):
    joints = Signal(list, list)  # names, positions (rad)


class Window(QMainWindow):
    def __init__(self, robot):
        super().__init__()
        self.setWindowTitle("Qutee joint reader")
        self.ros = None
        self.topic = None
        self.bridge = Bridge()
        self.bridge.joints.connect(self.on_joints)
        self.angles = {i: math.nan for i in IDS}
        self.mins = {i: math.inf for i in IDS}
        self.maxs = {i: -math.inf for i in IDS}
        self.message_times = []

        central = QWidget()
        layout = QVBoxLayout(central)
        row = QHBoxLayout()
        row.addWidget(QLabel("Robot"))
        self.robot = QLineEdit(robot)
        self.robot.setFixedWidth(100)
        row.addWidget(self.robot)
        row.addWidget(QLabel("rosbridge"))
        self.host = QLineEdit("localhost:9090")
        self.host.setFixedWidth(140)
        row.addWidget(self.host)
        for text, slot in [("Start agent", lambda: self.run_docker(start_agent)),
                           ("Start rosbridge", lambda: self.run_docker(start_rosbridge)),
                           ("Connect", self.connect_ros)]:
            b = QPushButton(text)
            b.clicked.connect(slot)
            row.addWidget(b)
        row.addStretch(1)
        layout.addLayout(row)
        self.state = QLabel("Not connected")
        layout.addWidget(self.state)

        self.table = QTableWidget(len(IDS), 6)
        self.table.setHorizontalHeaderLabels(["Motor", "Joint", "Angle (deg)", "Min (deg)", "Max (deg)", "Range (deg)"])
        self.table.verticalHeader().setVisible(False)
        big = QFont()
        big.setPointSize(12)
        for r, mid in enumerate(IDS):
            for c, text in enumerate([str(mid), f"leg {mid // 10}, {JOINT_NAMES[mid % 10]}"]):
                item = QTableWidgetItem(text)
                item.setFlags(Qt.ItemIsEnabled)
                self.table.setItem(r, c, item)
            for c in range(2, 6):
                item = QTableWidgetItem("")
                item.setFlags(Qt.ItemIsEnabled)
                item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                item.setFont(big)
                self.table.setItem(r, c, item)
        self.table.resizeColumnsToContents()
        self.table.setColumnWidth(1, 200)
        for c in range(2, 6):
            self.table.setColumnWidth(c, 110)
        layout.addWidget(self.table)

        row = QHBoxLayout()
        for text, slot in [("Reset min/max", self.reset_ranges), ("Torque off (move by hand)", lambda: self.torque(False)),
                           ("Torque on (hold)", lambda: self.torque(True)), ("Save ranges...", self.save_ranges)]:
            b = QPushButton(text)
            b.clicked.connect(slot)
            row.addWidget(b)
        layout.addLayout(row)
        note = QLabel("Angles from the standing pose (0 deg). Joint 2: negative raises the leg. Knee: positive "
                      "moves it forward. Move one joint at a time with the torque off.")
        note.setWordWrap(True)
        layout.addWidget(note)
        self.setCentralWidget(central)
        self.resize(800, 560)

        self.timer = QTimer(interval=200)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()

    # ----- ROS -----
    def connect_ros(self):
        if self.ros is not None:
            self.ros.close()
        host, _, port = self.host.text().partition(":")
        self.ros = roslibpy.Ros(host=host or "localhost", port=int(port or 9090))
        try:
            self.ros.run(timeout=5)
        except Exception as e:  # noqa: BLE001
            self.state.setText(f"Can't reach rosbridge at {self.host.text()}: {e}")
            self.ros = None
            return
        robot = self.robot.text().strip()
        self.topic = roslibpy.Topic(self.ros, f"/{robot}/joint_states", "sensor_msgs/JointState")
        self.topic.subscribe(lambda m: self.bridge.joints.emit(m["name"], m["position"]))
        self.state.setText(f"Connected to rosbridge, waiting for /{robot}/joint_states...")

    def torque(self, on):
        if self.ros is None or not self.ros.is_connected:
            self.state.setText("Not connected")
            return
        robot = self.robot.text().strip()
        service = roslibpy.Service(self.ros, f"/{robot}/joint_reader/set_parameters", "rcl_interfaces/srv/SetParameters")
        request = {"parameters": [{"name": "torque", "value": {"type": 1, "bool_value": on}}]}
        try:
            r = service.call(roslibpy.ServiceRequest(request), timeout=5)
            ok = r["results"][0]["successful"]
        except Exception as e:  # noqa: BLE001
            ok, e_text = False, str(e)
        else:
            e_text = ""
        self.state.setText(f"Torque {'on' if on else 'off'}" if ok else f"Torque change failed {e_text}")

    def on_joints(self, names, positions):
        self.message_times.append(time.time())
        for name, rad in zip(names, positions):
            try:
                mid = int(name)
            except ValueError:
                continue
            if mid in self.angles:
                deg = math.degrees(rad)
                self.angles[mid] = deg
                self.mins[mid] = min(self.mins[mid], deg)
                self.maxs[mid] = max(self.maxs[mid], deg)

    # ----- display -----
    def refresh(self):
        now = time.time()
        self.message_times = [t for t in self.message_times if now - t < 2]
        if self.ros is not None and self.ros.is_connected and self.message_times:
            self.state.setText(f"Receiving /{self.robot.text().strip()}/joint_states at "
                               f"{len(self.message_times) / 2:.0f} Hz")
        fmt = lambda v: "" if not math.isfinite(v) else f"{v:7.1f}"  # noqa: E731
        for r, mid in enumerate(IDS):
            span = self.maxs[mid] - self.mins[mid]
            for c, v in zip(range(2, 6), [self.angles[mid], self.mins[mid], self.maxs[mid], span]):
                self.table.item(r, c).setText(fmt(v))

    def reset_ranges(self):
        for mid in IDS:
            self.mins[mid], self.maxs[mid] = math.inf, -math.inf

    def save_ranges(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save joint ranges", str(Path.home() / "qutee_joint_ranges.csv"),
                                              "CSV (*.csv)")
        if not path:
            return
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["motor", "min_deg", "max_deg", "min_counts", "max_counts"])
            for mid in IDS:
                lo, hi = self.mins[mid], self.maxs[mid]
                if math.isfinite(lo):
                    # counts as the motor sees them: 2048 = standing pose, 4096 per turn
                    w.writerow([mid, f"{lo:.1f}", f"{hi:.1f}", round(2048 + lo * 4096 / 360), round(2048 + hi * 4096 / 360)])
        self.state.setText(f"Saved {path}")

    def run_docker(self, fn):
        self.state.setText("Working (building rosbridge takes a few minutes the first time)...")
        QApplication.processEvents()
        self.state.setText(fn())

    def closeEvent(self, event):
        if self.ros is not None:
            self.ros.close()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv[:1])
    w = Window(sys.argv[1] if len(sys.argv) > 1 else "qutee")
    w.show()
    app.exec()
