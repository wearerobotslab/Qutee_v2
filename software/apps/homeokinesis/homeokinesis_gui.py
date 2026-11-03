"""Homeokinesis GUI: start, stop and reset the controller on the robot, read and change its parameters, save
and load sets of values, and watch the joint positions.

The parameters are read from the robot (/<robot>/qutee_node), so the table always shows what the firmware has.
Needs the micro-ROS agent and rosbridge running (buttons below), and
  pip install PySide6 roslibpy           (the qutee_gui conda environment has them)
Run:  python homeokinesis_gui.py [robot name]       (default: qutee)
Works on Windows (Docker Desktop through WSL) and Linux (native Docker).
"""
import json
import math
import sys
import time
from pathlib import Path

import roslibpy
from PySide6.QtCore import QObject, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont
from PySide6.QtWidgets import (QApplication, QFileDialog, QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit,
                               QMainWindow, QPushButton, QSplitter, QTableWidget, QTableWidgetItem, QVBoxLayout,
                               QWidget)

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from qutee_docker import start_agent, start_rosbridge  # noqa: E402

# rcl_interfaces/msg/ParameterType
BOOL, INT, DOUBLE = 1, 2, 3
VALUE_FIELD = {BOOL: "bool_value", INT: "integer_value", DOUBLE: "double_value"}

# Shown in this order, with a description and the range the firmware accepts. Parameters not listed here
# (added to the firmware later) are shown after these.
PARAMS = {
    "eps_C": ("Learning rate of the controller (C, h). 0: the robot stops moving.", ">= 0"),
    "eps_A": ("Learning rate of the model (A, b).", ">= 0"),
    "amplitude": ("Joint targets = output x amplitude (normalised: 1 = the whole range).", "0 to 1"),
    "sensor_noise": ("White noise added to each joint reading (normalised: 1 = half a joint's range).", "0 to 1"),
    "motor_noise": ("White noise added to each output.", "0 to 1"),
    "motor_smoothing": ("Filters the targets: each step they move (1 - s) of the way to the output.", "0 to 0.99"),
    "creativity": ("Sox: how much the controller explores.", ">= 0"),
    "harmony": ("Sox: pulls the controller towards the identity.", ">= 0"),
    "damping": ("Pulls A and C back towards their initial values.", ">= 0"),
    "causeaware": ("Sox: weight of the model error in the model update.", ">= 0"),
    "steps_delay": ("Delay between a command and its effect, in steps (50 Hz).", "1 to 9"),
    "steps_averaging": ("Sensor smoothing, in steps.", "1 to 9"),
    "servo_profile_ms": ("Time each servo takes to reach a new target (0 = as fast as possible).", "0 to 1000"),
    "servo_p_gain": ("Servo stiffness while homeokinesis runs (the poses use 1600).", "0 to 16383"),
    "servo_pwm_limit": ("Maximum motor output (885 = 100%). Changing it while running pauses (rest, change, stand).",
                        "0 to 885"),
    "auto_recover": ("On an overload, reboot only that motor and continue (3 in a minute: stop).", "on/off"),
    "torque_off_at_rest": ("Motors limp at rest. Turn off when the robot hangs on a stand.", "on/off"),
}
CONTROL = ("running", "reset")  # shown as buttons, not in the table

IDS = [11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43]
JOINT_NAMES = {1: "hip", 2: "joint 2", 3: "knee"}
# The "walking" range of the firmware (QuteeNormalised::WALKING), in degrees: hip, joint 2, knee
WALKING = {1: (-45.0, 45.0), 2: (-45.0, 45.0), 3: (0.0, 90.0)}


def normalised(motor_id, degrees):
    lo, hi = WALKING[motor_id % 10]
    return (degrees - 0.5 * (lo + hi)) / (0.5 * (hi - lo))


def param_value(value):
    """Python value of a ParameterValue message (None if the type isn't handled)."""
    field = VALUE_FIELD.get(value.get("type"))
    return value.get(field) if field else None


def make_value(kind, v):
    return {"type": kind, VALUE_FIELD[kind]: v}


def format_value(kind, v):
    if kind == DOUBLE:
        return f"{v:.6g}"
    return str(v)


class Bridge(QObject):
    """roslibpy calls back from its own thread: these signals bring the results to the window's thread."""
    listed = Signal(list)                # parameter names
    values = Signal(list, list)          # names, ParameterValue dicts
    set_done = Signal(list, list)        # names, SetParametersResult dicts
    failed = Signal(str)                 # error message
    joints = Signal(list, list)          # names, positions (rad)


class Window(QMainWindow):
    def __init__(self, robot):
        super().__init__()
        self.setWindowTitle("Qutee homeokinesis")
        self.ros = None
        self.topic = None
        self.services = {}
        self.bridge = Bridge()
        self.bridge.listed.connect(self.on_listed)
        self.bridge.values.connect(self.on_values)
        self.bridge.set_done.connect(self.on_set_done)
        self.bridge.failed.connect(lambda text: self.state.setText(text))
        self.bridge.joints.connect(self.on_joints)
        self.kinds = {}        # parameter name -> type
        self.rows = {}         # parameter name -> table row
        self.filling = False   # True while the table is filled from the robot (not an edit)
        self.running = None
        self.angles = {i: math.nan for i in IDS}
        self.joint_times = []

        central = QWidget()
        layout = QVBoxLayout(central)

        # Connection
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
        self.state.setWordWrap(True)
        layout.addWidget(self.state)

        # Controller
        box = QGroupBox("Controller")
        row = QHBoxLayout(box)
        self.status = QLabel("unknown")
        big = QFont()
        big.setPointSize(14)
        big.setBold(True)
        self.status.setFont(big)
        self.status.setMinimumWidth(160)
        row.addWidget(self.status)
        self.start_button = QPushButton("Start")
        self.start_button.clicked.connect(lambda: self.set_params({"running": make_value(BOOL, True)}))
        self.stop_button = QPushButton("Stop")
        self.stop_button.clicked.connect(lambda: self.set_params({"running": make_value(BOOL, False)}))
        reset = QPushButton("Reset (forget what it learned)")
        reset.clicked.connect(lambda: self.set_params({"reset": make_value(BOOL, True)}))
        for b in (self.start_button, self.stop_button, reset):
            row.addWidget(b)
        row.addStretch(1)
        layout.addWidget(box)

        splitter = QSplitter(Qt.Horizontal)
        # Parameters
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["Parameter", "Value", "Allowed", "What it does"])
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)
        self.table.itemChanged.connect(self.on_item_changed)
        left_layout.addWidget(self.table)
        row = QHBoxLayout()
        for text, slot in [("Read from robot", self.refresh_params), ("Save to file...", self.save_params),
                           ("Load from file...", self.load_params)]:
            b = QPushButton(text)
            b.clicked.connect(slot)
            row.addWidget(b)
        row.addStretch(1)
        left_layout.addLayout(row)
        note = QLabel("Double-click a value to change it (Enter sends it to the robot); tick a box to switch it. "
                      "Values go back to the defaults when the robot restarts: save good sets to a file.")
        note.setWordWrap(True)
        left_layout.addWidget(note)
        splitter.addWidget(left)

        # Joints
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        self.joint_state = QLabel("Joints: no data")
        right_layout.addWidget(self.joint_state)
        self.joint_table = QTableWidget(len(IDS), 3)
        self.joint_table.setHorizontalHeaderLabels(["Joint", "deg", "normalised"])
        self.joint_table.verticalHeader().setVisible(False)
        for r, mid in enumerate(IDS):
            for c, text in enumerate([f"{mid} {JOINT_NAMES[mid % 10]}", "", ""]):
                item = QTableWidgetItem(text)
                item.setFlags(Qt.ItemIsEnabled)
                if c:
                    item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self.joint_table.setItem(r, c, item)
        self.joint_table.resizeColumnsToContents()
        self.joint_table.setColumnWidth(0, 90)
        self.joint_table.setColumnWidth(1, 70)
        self.joint_table.setColumnWidth(2, 90)
        right_layout.addWidget(self.joint_table)
        note = QLabel("Normalised to the walking range: hip and joint 2 -45 to 45 deg, knee 0 to 90 deg. "
                      "Beyond +-1 (red) = outside the range.")
        note.setWordWrap(True)
        right_layout.addWidget(note)
        splitter.addWidget(right)
        splitter.setSizes([700, 280])
        layout.addWidget(splitter)

        self.setCentralWidget(central)
        self.resize(1050, 640)

        self.poll = QTimer(interval=1000)  # the "running" parameter (it also follows the robot's menu)
        self.poll.timeout.connect(lambda: self.get_params(["running"]))
        self.display = QTimer(interval=200)
        self.display.timeout.connect(self.refresh_joints)
        self.display.start()

    # ----- ROS -----
    def node(self):
        return f"/{self.robot.text().strip()}/qutee_node"

    def connected(self):
        if self.ros is None or not self.ros.is_connected:
            self.state.setText("Not connected: start the agent and rosbridge, then Connect")
            return False
        return True

    def service(self, name, srv_type):
        key = (self.node(), name)
        if key not in self.services:
            self.services[key] = roslibpy.Service(self.ros, f"{self.node()}/{name}", srv_type)
        return self.services[key]

    def call(self, name, srv_type, request, on_result):
        if not self.connected():
            return
        self.service(name, srv_type).call(roslibpy.ServiceRequest(request), on_result,
                                          lambda e: self.bridge.failed.emit(f"{name} failed: {e}"))

    def connect_ros(self):
        if self.ros is not None:
            self.poll.stop()
            self.ros.close()
        self.services = {}
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
        self.state.setText(f"Connected to rosbridge, reading the parameters of {self.node()}...")
        self.refresh_params()
        self.poll.start()

    def refresh_params(self):
        self.call("list_parameters", "rcl_interfaces/srv/ListParameters", {"prefixes": [], "depth": 0},
                  lambda r: self.bridge.listed.emit(list(r["result"]["names"])))

    def get_params(self, names):
        self.call("get_parameters", "rcl_interfaces/srv/GetParameters", {"names": names},
                  lambda r: self.bridge.values.emit(names, list(r["values"])))

    def set_params(self, values):
        """values: {name: ParameterValue dict}, sent in one call (the robot checks each one)."""
        names = list(values)
        request = {"parameters": [{"name": n, "value": values[n]} for n in names]}
        self.call("set_parameters", "rcl_interfaces/srv/SetParameters", request,
                  lambda r: self.bridge.set_done.emit(names, list(r["results"])))

    # ----- results (in the window's thread) -----
    def on_listed(self, names):
        if "eps_C" not in names:
            self.state.setText(f"{self.node()} has no homeokinesis parameters: is the homeokinesis firmware "
                               "flashed? Showing what it has.")
        ordered = [n for n in PARAMS if n in names] + sorted(n for n in names if n not in PARAMS and n not in CONTROL)
        self.filling = True
        self.table.setRowCount(len(ordered))
        self.rows = {}
        for r, name in enumerate(ordered):
            self.rows[name] = r
            description, allowed = PARAMS.get(name, ("", ""))
            for c, text in enumerate([name, "", allowed, description]):
                item = QTableWidgetItem(text)
                item.setFlags(Qt.ItemIsEnabled)
                self.table.setItem(r, c, item)
        self.table.resizeColumnToContents(0)
        self.table.setColumnWidth(1, 110)
        self.table.resizeColumnToContents(2)
        self.filling = False
        self.get_params(ordered + [n for n in CONTROL if n in names])

    def on_values(self, names, values):
        self.filling = True
        for name, value in zip(names, values):
            kind = value.get("type")
            v = param_value(value)
            if name == "running":
                self.show_running(bool(v))
                continue
            if name not in self.rows or v is None:
                continue
            self.kinds[name] = kind
            item = self.table.item(self.rows[name], 1)
            if kind == BOOL:
                item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsUserCheckable)
                item.setCheckState(Qt.Checked if v else Qt.Unchecked)
                item.setText("on" if v else "off")
            else:
                item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsEditable | Qt.ItemIsSelectable)
                item.setText(format_value(kind, v))
            item.setBackground(QColor(0, 0, 0, 0))
        self.filling = False
        if len(names) > 1:
            self.state.setText(f"Read {len(names)} parameters from {self.node()}")

    def on_set_done(self, names, results):
        failed = [(n, r.get("reason", "")) for n, r in zip(names, results) if not r.get("successful")]
        if failed:
            self.state.setText("Refused by the robot: " + "; ".join(f"{n} {why}".strip() for n, why in failed)
                               + " (see the allowed values)")
        else:
            self.state.setText("Set " + ", ".join(names))
        self.get_params(names)  # show what the robot has now (also undoes a refused edit)

    def on_item_changed(self, item):
        if self.filling or item.column() != 1:
            return
        name = self.table.item(item.row(), 0).text()
        kind = self.kinds.get(name)
        try:
            if kind == BOOL:
                v = item.checkState() == Qt.Checked
            elif kind == INT:
                v = int(item.text())
            elif kind == DOUBLE:
                v = float(item.text())
            else:
                return
        except ValueError:
            self.state.setText(f"{name}: '{item.text()}' is not a number")
            self.get_params([name])
            return
        self.filling = True
        item.setBackground(QColor(255, 230, 150))  # sent, waiting for the robot
        self.filling = False
        self.set_params({name: make_value(kind, v)})

    def show_running(self, running):
        self.running = running
        self.status.setText("Running" if running else "Stopped")
        self.status.setStyleSheet("color: #1a7f37;" if running else "color: #b42318;")
        self.start_button.setEnabled(not running)
        self.stop_button.setEnabled(running)

    # ----- files -----
    def save_params(self):
        values = {}
        for name, r in self.rows.items():
            kind = self.kinds.get(name)
            item = self.table.item(r, 1)
            if kind == BOOL:
                values[name] = item.checkState() == Qt.Checked
            elif kind == INT:
                values[name] = int(item.text())
            elif kind == DOUBLE:
                values[name] = float(item.text())
        if not values:
            self.state.setText("Nothing to save: connect first")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Save parameters", str(Path.home() / "hk_params.json"),
                                              "JSON (*.json)")
        if path:
            Path(path).write_text(json.dumps(values, indent=2))
            self.state.setText(f"Saved {len(values)} parameters to {path}")

    def load_params(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load parameters", str(Path.home()), "JSON (*.json)")
        if not path:
            return
        try:
            saved = json.loads(Path(path).read_text())
        except (OSError, ValueError) as e:
            self.state.setText(f"Can't read {path}: {e}")
            return
        values = {}
        for name, v in saved.items():
            kind = self.kinds.get(name)
            if kind is None or name in CONTROL:
                continue  # not on this robot (or start/stop, which stays as it is)
            values[name] = make_value(kind, bool(v) if kind == BOOL else int(v) if kind == INT else float(v))
        if values:
            self.set_params(values)
        else:
            self.state.setText("No matching parameters in the file: connect first")

    # ----- joints -----
    def on_joints(self, names, positions):
        self.joint_times.append(time.time())
        for name, rad in zip(names, positions):
            try:
                mid = int(name)
            except ValueError:
                continue
            if mid in self.angles:
                self.angles[mid] = math.degrees(rad)

    def refresh_joints(self):
        now = time.time()
        self.joint_times = [t for t in self.joint_times if now - t < 2]
        self.joint_state.setText(f"Joints: {len(self.joint_times) / 2:.0f} Hz" if self.joint_times
                                 else "Joints: no data")
        for r, mid in enumerate(IDS):
            deg = self.angles[mid]
            if not math.isfinite(deg):
                continue
            n = normalised(mid, deg)
            n = 0.0 if abs(n) < 0.005 else n  # no "-0.00"
            self.joint_table.item(r, 1).setText(f"{deg:6.1f}")
            item = self.joint_table.item(r, 2)
            item.setText(f"{n:+.2f}")
            item.setForeground(QColor("#d92d20") if abs(n) > 1 else self.palette().text().color())

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
