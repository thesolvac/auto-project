#!/usr/bin/env python3
"""
Manual driving + AprilTag vision, running on the Raspberry Pi.

  - Keyboard keys drive the robot (commands sent to the ESP32 over serial).
  - The webcam detects AprilTags and reports distance + orientation.
  - No ultrasonic sensors are used here.

The ESP32 must be running the MOTOR control sketch (two_motor_control.ino),
which understands the single-character commands: f b r l s.

Controls (press while the video window is focused):
    f / b / r / l = forward / backward / right / left
    s             = stop
    q             = quit (sends stop first)

Dependencies:
    pip install opencv-python numpy pyserial
  (on Raspberry Pi OS, alternatively: sudo apt install python3-opencv python3-numpy python3-serial)

Usage:
    python3 robot_vision_control.py
    python3 robot_vision_control.py /dev/ttyACM0   # override ESP32 serial port

------------------------------------------------------------------
Camera calibration note:
Distance/orientation accuracy depends on the camera matrix (K) and the
physical tag size (TAG_SIZE_M). The defaults below are a rough guess.
For real accuracy, calibrate with a chessboard and paste the real K/dist,
and MEASURE your printed tag's black-square side into TAG_SIZE_M.
------------------------------------------------------------------
"""

import sys
import math
import cv2
import numpy as np
import serial

# ---------- ESP32 serial ----------
DEFAULT_PORT = "/dev/ttyUSB0"    # the ESP32 (NOT the webcam, which is /dev/video0)
BAUD = 115200
VALID_KEYS = set("fbrls")

# ---------- Camera / AprilTag ----------
CAMERA_INDEX = 0                 # USB webcam is usually 0
FRAME_W, FRAME_H = 640, 480
TAG_SIZE_M = 0.05                # <-- MEASURE your tag's black border side, in METERS
TAG_DICT = cv2.aruco.DICT_APRILTAG_36h11   # change family if needed


# ============================================================
# AprilTag helpers
# ============================================================
def build_detector():
    """Create an aruco detector that works across OpenCV versions."""
    aruco = cv2.aruco
    if hasattr(aruco, "getPredefinedDictionary"):
        dictionary = aruco.getPredefinedDictionary(TAG_DICT)
    else:
        dictionary = aruco.Dictionary_get(TAG_DICT)

    if hasattr(aruco, "ArucoDetector"):
        params = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(dictionary, params)
        return lambda gray: detector.detectMarkers(gray)
    else:
        params = aruco.DetectorParameters_create()
        return lambda gray: aruco.detectMarkers(gray, dictionary, parameters=params)


def default_camera_matrix(w, h):
    """Rough intrinsics when no calibration is available."""
    fx = fy = float(w)
    cx, cy = w / 2.0, h / 2.0
    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0,  0,  1]], dtype=np.float64)
    dist = np.zeros((5, 1), dtype=np.float64)
    return K, dist


def rotation_to_euler(R):
    """3x3 rotation matrix -> yaw, pitch, roll in degrees."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        roll  = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw   = math.atan2(R[1, 0], R[0, 0])
    else:
        roll  = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw   = 0.0
    return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)


def estimate_pose(corner, tag_size, K, dist):
    """Pose of one square tag. Returns (distance_cm, yaw, pitch, roll, rvec, tvec) or None."""
    half = tag_size / 2.0
    obj_pts = np.array([
        [-half,  half, 0.0],
        [ half,  half, 0.0],
        [ half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float64)
    img_pts = corner.reshape(4, 2).astype(np.float64)

    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    distance_cm = float(np.linalg.norm(tvec)) * 100.0
    R, _ = cv2.Rodrigues(rvec)
    yaw, pitch, roll = rotation_to_euler(R)
    return distance_cm, yaw, pitch, roll, rvec, tvec


# ============================================================
# Main
# ============================================================
def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT

    # --- Connect to ESP32 ---
    try:
        ser = serial.Serial(port, BAUD, timeout=0.05)
    except serial.SerialException as e:
        print(f"Could not open ESP32 serial port {port}: {e}")
        print("Tip: run  ls /dev/ttyUSB* /dev/ttyACM*  to find it.")
        sys.exit(1)

    # --- Open camera ---
    detect = build_detector()
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    if not cap.isOpened():
        print(f"Could not open camera index {CAMERA_INDEX}.")
        print("Tip: run  ls /dev/video*  to see available cameras.")
        ser.close()
        sys.exit(1)

    ret, frame = cap.read()
    if not ret:
        print("Camera opened but no frame received.")
        cap.release(); ser.close()
        sys.exit(1)
    h, w = frame.shape[:2]
    K, dist = default_camera_matrix(w, h)

    print(f"Ready. ESP32 on {port}, camera {w}x{h}, tag {TAG_SIZE_M*100:.1f} cm.")
    print("Click the video window, then use: f b r l s  (q to quit)\n")

    last_cmd = "-"

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            # --- AprilTag detection ---
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detect(gray)

            if ids is not None and len(ids) > 0:
                for corner, tag_id in zip(corners, ids.flatten()):
                    result = estimate_pose(corner, TAG_SIZE_M, K, dist)
                    if result is None:
                        continue
                    distance_cm, yaw, pitch, roll, rvec, tvec = result
                    print(f"ID {tag_id:>3} | dist {distance_cm:6.1f} cm | "
                          f"yaw {yaw:6.1f}  pitch {pitch:6.1f}  roll {roll:6.1f}")

                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    cv2.drawFrameAxes(frame, K, dist, rvec, tvec, TAG_SIZE_M * 0.5)
                    c = corner.reshape(4, 2).mean(axis=0).astype(int)
                    cv2.putText(frame, f"id{tag_id} {distance_cm:.0f}cm",
                                (c[0] - 40, c[1]), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 255, 0), 2)

            # --- Overlay current command ---
            cv2.putText(frame, f"cmd: {last_cmd}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

            cv2.imshow("Robot - AprilTag + manual drive", frame)

            # --- Keyboard -> motor commands ---
            raw = cv2.waitKey(1)
            if raw != -1:
                ch = chr(raw & 0xFF).lower()
                if ch == 'q':
                    ser.write(b's')
                    print("\nQuitting (motors stopped).")
                    break
                if ch in VALID_KEYS:
                    ser.write(ch.encode())
                    last_cmd = ch
                    # optional feedback from ESP32
                    resp = ser.readline().decode(errors="ignore").strip()
                    if resp:
                        print(f"[{ch}] -> {resp}")

    except KeyboardInterrupt:
        ser.write(b's')
        print("\nCtrl+C - motors stopped.")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        ser.close()


if __name__ == "__main__":
    main()
