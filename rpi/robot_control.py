#!/usr/bin/env python3
"""
Real-time keyboard control for the ESP32 / TMC2209 robot.
Run on the Raspberry Pi with the ESP32 connected via USB.

Keys:
    f / b / r / l = forward / backward / right / left
    s             = stop
    q             = quit (sends stop first)

Usage:
    python3 robot_control.py
    python3 robot_control.py /dev/ttyACM0   # override port
"""

import sys
import tty
import termios
import serial

DEFAULT_PORT = "/dev/ttyUSB0"
BAUD = 115200
VALID_KEYS = set("fbrls")


def getch():
    """Read a single keypress from stdin without waiting for Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT

    try:
        ser = serial.Serial(port, BAUD, timeout=0.05)
    except serial.SerialException as e:
        print(f"Could not open {port}: {e}")
        print("Tip: run  ls /dev/ttyUSB* /dev/ttyACM*  to find the right port.")
        sys.exit(1)

    print(f"Connected on {port}.  Keys: f b r l s   (q to quit)")

    try:
        while True:
            key = getch().lower()

            if key == "q":
                ser.write(b"s")
                print("\nQuitting (motors stopped).")
                break

            if key in VALID_KEYS:
                ser.write(key.encode())
                # Drain any response from the ESP32 for nice feedback
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    print(f"[{key}] -> {line}")
                else:
                    print(f"[{key}] sent")
            # ignore anything else silently

    except KeyboardInterrupt:
        ser.write(b"s")
        print("\nCtrl+C — motors stopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
