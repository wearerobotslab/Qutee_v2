"""Show the Qutee's logs over WiFi, like serial_monitor.py does over USB.

In ROS mode, once WiFi is connected, the robot sends its logs (starting with the ones since boot)
by UDP to the micro-ROS agent's IP, on port 8889 by default (menuconfig: QUTEE settings).
Run this on the computer with that IP. The robot never sends lines containing "password".

Usage:  python wifi_monitor.py [PORT]     (default: 8889)
Quit with Ctrl+C, Ctrl+X or q.

On Windows, allow the port through the firewall once (PowerShell as Administrator):
  New-NetFirewallRule -DisplayName "Qutee logs UDP 8889" -Direction Inbound -Protocol UDP -LocalPort 8889 -Action Allow
"""
import os
import socket
import sys

try:
    import msvcrt  # Windows only: lets us read key presses without blocking
except ImportError:
    msvcrt = None

port = int(sys.argv[1]) if len(sys.argv) > 1 else 8889

os.system("")  # enables ANSI colour codes in the Windows console
out = sys.stdout.buffer


def quit_requested():
    if msvcrt is not None:
        while msvcrt.kbhit():
            if msvcrt.getwch() in ("\x03", "\x18", "q", "Q"):  # Ctrl+C, Ctrl+X, q
                return True
    return False


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", port))
sock.settimeout(0.1)
print(f"--- Listening for Qutee logs on UDP port {port}. Quit with Ctrl+C, Ctrl+X or q ---", flush=True)
print("--- The robot starts sending when it connects to WiFi (Start ROS) ---", flush=True)

sender = None
try:
    while not quit_requested():
        try:
            data, (address, _) = sock.recvfrom(4096)
        except socket.timeout:
            continue
        if address != sender:
            sender = address
            print(f"\n--- Receiving from {address} ---", flush=True)
        out.write(data)
        out.flush()
except KeyboardInterrupt:
    pass
sock.close()
print("\n--- Exiting ---")
