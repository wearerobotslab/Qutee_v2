"""Docker helpers for the Qutee GUIs: start the micro-ROS agent and rosbridge.

Works on Windows (Docker Desktop, called through WSL) and Linux (native Docker).
"""
import subprocess
import sys
from pathlib import Path

WINDOWS = sys.platform == "win32"
WSL_DISTRO = "Ubuntu"
TOOLS = Path(__file__).resolve().parent


def docker(*args, timeout=300):
    """Run a docker command; returns (exit code, output)."""
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
    """rosbridge with ROS 2's built-in types (parameters, JointState), on port 9090. Built the first time."""
    if docker("image", "inspect", "qutee_rosbridge_basic")[0] != 0:
        code, out = docker("build", "-t", "qutee_rosbridge_basic", "-f", docker_path(TOOLS / "rosbridge.Dockerfile"),
                           docker_path(TOOLS), timeout=1200)
        if code != 0:
            return "building rosbridge failed: " + out[-300:]
    net = ["-p", "9090:9090"] if WINDOWS else ["--net=host"]
    return start_container("qutee_rosbridge_basic", [*net, "qutee_rosbridge_basic"])
