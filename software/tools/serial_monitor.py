"""Serial monitor for the Qutee that survives resets.

The ESP32-S3 uses its native USB port, so the COM port disappears every time the
board resets. This monitor waits for the port to come back and reconnects,
and prints the ESP-IDF log colours instead of escape codes.

Usage:  python serial_monitor.py [PORT] [BAUD]     (defaults: COM3 115200)
Quit with Ctrl+C, Ctrl+X or q.
"""
import os
import signal
import sys
import time

import serial

try:
    import msvcrt  # Windows only: lets us read key presses without blocking
except ImportError:
    msvcrt = None

port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

os.system("")  # enables ANSI colour codes in the Windows console
out = sys.stdout.buffer

# Ctrl+C sets a flag instead of raising KeyboardInterrupt, which pyserial on Windows
# can turn into a SerialException that looks like a disconnect.
stop = False


def request_stop(*_):
    global stop
    stop = True


signal.signal(signal.SIGINT, request_stop)


def quit_requested():
    if not stop and msvcrt is not None:
        while msvcrt.kbhit():
            if msvcrt.getwch() in ("\x03", "\x18", "q", "Q"):  # Ctrl+C, Ctrl+X, q
                request_stop()
    return stop


print(f"--- Monitoring {port} at {baud}. Quit with Ctrl+C, Ctrl+X or q ---", flush=True)
while not quit_requested():
    try:
        with serial.Serial(port, baud, timeout=0.1) as ser:
            print(f"\n--- Connected to {port} ---", flush=True)
            while not quit_requested():
                data = ser.read(ser.in_waiting or 1)
                if data:
                    out.write(data)
                    out.flush()
    except (serial.SerialException, OSError):
        if quit_requested():
            break
        print(f"\n--- {port} disconnected, waiting for it to come back ---", flush=True)
        while not quit_requested():
            time.sleep(0.2)
            try:
                serial.Serial(port).close()
                break
            except (serial.SerialException, OSError):
                pass
print("\n--- Exiting ---")
