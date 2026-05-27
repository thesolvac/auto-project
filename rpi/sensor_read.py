#!/usr/bin/env python3
"""
Read HC-SR04 distance sensor data from the ESP32 over serial.
The ESP32 must be running ultrasonic_test.ino.

Output format from ESP32:
    U3: 12.4 cm   U4: 25.1 cm
    U3: ---       U4: 8.3 cm

Usage:
    python3 sensor_read.py
    python3 sensor_read.py /dev/ttyACM0   # override port

Press Ctrl+C to quit.
"""

import sys
import re
import time
import serial

DEFAULT_PORT = "/dev/ttyUSB0"
BAUD = 115200

# Matches "U3: 12.4 cm" or "U3: ---"
PATTERN = re.compile(r"U(\d):\s*(?:([0-9.]+)\s*cm|---)", re.IGNORECASE)


def status_bar(cm: float) -> str:
    """Return a short text indicator for the distance."""
    if cm < 10:
        return "[#####] TOO CLOSE"
    if cm < 30:
        return "[####.] close"
    if cm < 60:
        return "[###..] medium"
    if cm < 100:
        return "[##...] far"
    return "[#....] very far"


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT

    try:
        ser = serial.Serial(port, BAUD, timeout=1)
    except serial.SerialException as e:
        print(f"Could not open {port}: {e}")
        print("Tip: run  ls /dev/ttyUSB* /dev/ttyACM*  to find the right port.")
        sys.exit(1)

    print(f"Reading from {port}.  Ctrl+C to quit.\n")

    try:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue

            matches = PATTERN.findall(line)
            timestamp = time.strftime("%H:%M:%S")

            if not matches:
                # Non-sensor line (e.g. ESP32 boot message)
                print(f"[{timestamp}]  {line}")
                continue

            parts = []
            for sensor_num, value in matches:
                if value:
                    dist = float(value)
                    parts.append(f"U{sensor_num}: {dist:6.1f} cm  {status_bar(dist)}")
                else:
                    parts.append(f"U{sensor_num}:    ---     [.....] out of range")

            print(f"[{timestamp}]  " + "   |   ".join(parts))

    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
