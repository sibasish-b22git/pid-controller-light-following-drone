from __future__ import print_function
import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil

import cv2
import numpy as np
import math
import sys

# --- Camera Selection ---
USE_PICAMERA2 = True   # True for Raspberry Pi AI Camera, False for USB camera

if USE_PICAMERA2:
    from picamera2 import Picamera2

# --- Configuration ---
# drone_connect = "tcp:127.0.0.1:5762"
drone_connect = "/dev/ttyACM0"
BAUD_RATE = 57600
TAKEOFF_ALTITUDE = 0.4
VELOCITY_FORWARD = 0.1          # max forward speed while tracking (m/s)

# --- PID Configuration ---
# Lateral PID: input = normalised offset (-1..1, fraction of half frame width)
#              output = sideways velocity (m/s)
LAT_KP, LAT_KI, LAT_KD = 0.30, 0.02, 0.05
LAT_MAX_OUT = 0.15              # max sideways speed (m/s)
LAT_DEADBAND_PX = 8             # ignore tiny offsets (pixels)

# Yaw PID: input = line angle error (degrees, 0 = line points straight ahead)
#          output = yaw rate (deg/s)
YAW_KP, YAW_KI, YAW_KD = 0.80, 0.00, 0.10
YAW_MAX_OUT = 30.0              # max yaw rate (deg/s)
YAW_DEADBAND_DEG = 2.0

# Sign conventions - verify on the bench with PROPS OFF / in SITL.
# Body NED: +vy = right, +yaw_rate = clockwise.
# The original code moved LEFT when the line was to the RIGHT of centre
# (camera mounting dependent), so LATERAL_SIGN = -1 preserves that behaviour.
# If the drone moves away from the line instead of toward it, flip the sign.
LATERAL_SIGN = -1
YAW_SIGN = -1

# --- Navigation State Variables ---
LOST_FRAME_COUNT = 0
SEARCHING = False
SEARCH_START_TIME = 0
MAX_SEARCH_TIME = 10  # Seconds to search before hovering in place

print("Connecting to vehicle on:", drone_connect)
vehicle = connect(drone_connect, wait_ready=True, baud=BAUD_RATE)


# --- PID Controller ---
class PID:
    def __init__(self, kp, ki, kd, out_limit, i_limit=None, d_alpha=0.7):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit
        self.i_limit = i_limit if i_limit is not None else out_limit
        self.d_alpha = d_alpha      # low-pass factor for derivative (0..1)
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.prev_time = None
        self.d_filtered = 0.0

    def update(self, error):
        now = time.time()
        if self.prev_time is None:
            dt = None
        else:
            dt = max(now - self.prev_time, 1e-3)

        p = self.kp * error

        if dt is not None:
            # Integral with anti-windup clamp
            self.integral += error * dt
            max_i = self.i_limit / self.ki if self.ki != 0 else 0.0
            self.integral = max(-max_i, min(max_i, self.integral))

            # Filtered derivative
            raw_d = (error - self.prev_error) / dt
            self.d_filtered = self.d_alpha * self.d_filtered + (1 - self.d_alpha) * raw_d
        i = self.ki * self.integral
        d = self.kd * self.d_filtered

        self.prev_error = error
        self.prev_time = now

        out = p + i + d
        return max(-self.out_limit, min(self.out_limit, out))


lateral_pid = PID(LAT_KP, LAT_KI, LAT_KD, LAT_MAX_OUT)
yaw_pid = PID(YAW_KP, YAW_KI, YAW_KD, YAW_MAX_OUT)


def reset_pids():
    lateral_pid.reset()
    yaw_pid.reset()


# --- Drone Movement Functions ---
def send_ned_velocity(vx, vy, vz, yaw_rate=0.0, duration=0.0):
    """Body-frame velocity (m/s) + yaw rate (rad/s, +ve = clockwise)."""
    # Use velocity + yaw_rate; ignore position, acceleration and yaw angle
    type_mask = 0b0000010111000111
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        type_mask,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, yaw_rate
    )
    vehicle.send_mavlink(msg)
    if duration > 0:
        time.sleep(duration)


def condition_stop():
    print("Applying active brakes...")
    reset_pids()
    # Quick reverse pulse to counter forward momentum
    send_ned_velocity(-VELOCITY_FORWARD, 0, 0, duration=0.3)
    # Now hold position
    send_ned_velocity(0, 0, 0)
    time.sleep(5)


# --- Vision Functions ---
def detect_led_line(frame, color_name):
    color_ranges = {
        "green": ([40, 100, 100], [80, 255, 255]),
        "blue": ([110, 50, 50], [130, 255, 255]),
        "red": ([0, 100, 100], [10, 255, 255]),
        "white": ([0, 0, 200], [180, 40, 255])
    }

    lower, upper = [np.array(v, dtype=np.uint8) for v in color_ranges[color_name]]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centers = []

    for cnt in contours:
        if cv2.contourArea(cnt) > 10:
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centers.append((cx, cy))
                cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)
    return centers, mask


def process_frame(frame):
    global LOST_FRAME_COUNT, SEARCHING, SEARCH_START_TIME

    frame_height, frame_width, _ = frame.shape
    frame_center_x = frame_width // 2
    frame_center_y = frame_height // 2

    # Check for Red (Stop) and White (Path)
    red_centers, _ = detect_led_line(frame, "red")
    white_centers, _ = detect_led_line(frame, "white")

    # 1. Stop Condition (Highest Priority)
    if len(red_centers) >= 1:
        condition_stop()
        print("Red Detected, Vehicle stopped")
        return frame

    # 2. Line Tracking Condition (PID)
    if len(white_centers) >= 2:
        if SEARCHING:
            print("[SEARCH] Line recovered! Resuming track.")
            SEARCHING = False
            reset_pids()
        LOST_FRAME_COUNT = 0

        points = np.array(white_centers, dtype=np.int32)
        vx, vy, x0, y0 = [v.item() for v in cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)]

        # Make direction vector point "down" the image so the angle is well defined
        if vy < 0:
            vx, vy = -vx, -vy
        if vy < 1e-3:
            # Line is (almost) horizontal in the image - can't align to it
            send_ned_velocity(0, 0, 0)
            return frame

        # Draw fitted line (x as a function of y, safe for vertical lines)
        x_top = int(round(x0 + (0 - y0) * vx / vy))
        x_bot = int(round(x0 + (frame_height - 1 - y0) * vx / vy))
        if abs(x_top) < 10 ** 6 and abs(x_bot) < 10 ** 6:
            cv2.line(frame, (x_top, 0), (x_bot, frame_height - 1), (0, 255, 0), 2)

        # --- Errors ---
        # Lateral: where the line crosses the image centre row vs the image centre
        line_x_at_center_y = x0 + (frame_center_y - y0) * vx / vy
        dx = line_x_at_center_y - frame_center_x            # pixels, +ve = line is right
        # Heading: angle between line and image vertical (deg), +ve = line leans up-left
        angle = math.degrees(math.atan2(vx, vy))

        # Deadbands
        dx_eff = 0.0 if abs(dx) < LAT_DEADBAND_PX else dx
        angle_eff = 0.0 if abs(angle) < YAW_DEADBAND_DEG else angle

        # Normalise lateral error to -1..1
        lat_err = dx_eff / (frame_width / 2.0)

        # --- PID outputs ---
        lat_out = lateral_pid.update(lat_err)                # m/s
        yaw_out_deg = yaw_pid.update(angle_eff)              # deg/s

        cmd_vy = LATERAL_SIGN * lat_out
        cmd_yaw_rate = math.radians(YAW_SIGN * yaw_out_deg)  # rad/s

        # Slow down the forward speed when badly misaligned
        align_factor = 1.0 - 0.5 * min(abs(lat_err), 1.0) - min(abs(angle), 60.0) / 60.0
        align_factor = max(0.0, min(1.0, align_factor))
        cmd_vx = VELOCITY_FORWARD * align_factor

        print(f"[PID] dx: {dx:6.1f}px  angle: {angle:6.2f}deg | "
              f"vx: {cmd_vx:.2f}  vy: {cmd_vy:+.2f}  yaw_rate: {math.degrees(cmd_yaw_rate):+.1f}deg/s")

        send_ned_velocity(cmd_vx, cmd_vy, 0, yaw_rate=cmd_yaw_rate)
        return frame

    # 3. Lost Line / Search Condition
    else:
        LOST_FRAME_COUNT += 1
        reset_pids()  # don't let stale integral/derivative state carry over

        # Require 5 consecutive lost frames to trigger search to filter out blips
        if LOST_FRAME_COUNT > 5:
            if not SEARCHING:
                print("[SEARCH] Line lost. Starting constant-heading spiral.")
                SEARCHING = True
                SEARCH_START_TIME = time.time()

            t = time.time() - SEARCH_START_TIME

            if t < MAX_SEARCH_TIME:
                # Expanding spiral: speed starts at 0.05 m/s and grows 0.05 m/s per second
                spiral_speed = min(0.05 + (0.05 * t), 1.0)
                omega = 1.5  # rotation rate of the velocity vector

                search_vx = spiral_speed * math.cos(omega * t)
                search_vy = spiral_speed * math.sin(omega * t)

                send_ned_velocity(search_vx, search_vy, 0, duration=0.1)
            else:
                print("[SEARCH] Timeout reached. Hovering in place.")
                send_ned_velocity(0, 0, 0, duration=0.5)

    return frame


def arm_and_takeoff(aTargetAltitude):
    print("Pre-arm checks...")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {alt:.2f}m")
        if alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


# --- MAIN EXECUTION ---
if __name__ == "__main__":
    arm_and_takeoff(TAKEOFF_ALTITUDE)

    if USE_PICAMERA2:
        print("Using Raspberry Pi AI Camera (Picamera2)")
        picam2 = Picamera2()
        picam2.preview_configuration.main.size = (640, 480)
        picam2.preview_configuration.main.format = "RGB888"
        picam2.configure("preview")
        picam2.start()
        time.sleep(1)
    else:
        print("Using USB Camera...")
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Error: Could not open camera.")
            sys.exit()

    print("Starting LED navigation... Press 'q' to land.")

    try:
        while True:
            if USE_PICAMERA2:
                frame = picam2.capture_array()
            else:
                ret, frame = cap.read()
                if not ret:
                    print("Camera stream ended.")
                    break

            output = process_frame(frame.copy())
            cv2.imshow("LED Navigation", output)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Landing...")
                break
    finally:
        vehicle.mode = VehicleMode("LAND")
        if not USE_PICAMERA2:
            cap.release()
        cv2.destroyAllWindows()
