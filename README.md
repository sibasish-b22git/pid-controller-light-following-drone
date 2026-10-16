# LED Line-Following Drone (PID)

A vision-guided indoor drone controller. A Raspberry Pi reads a downward-facing camera, finds a line of white LEDs on the floor, and uses **PID control** to keep an ArduPilot drone centred on and aligned with that line. A red LED acts as a stop marker.

## What it does

- Detects white LED points in each camera frame (OpenCV HSV thresholding) and fits a line through them.
- Computes two errors: **sideways offset** from the line and **heading angle** relative to the line.
- Feeds each error into its own PID controller and sends the result to the flight controller as body-frame velocity and yaw-rate commands over MAVLink (DroneKit).
- Stops and holds position when a red LED is seen.
- If the line is lost, flies an expanding search spiral, then hovers if it times out.

## Why it's useful

- **Smooth tracking:** continuous PID commands replace the old step-and-pause corrections, so the drone doesn't jerk between moves.
- **Tunable:** all gains, limits and deadbands are constants at the top of the script.
- **Safe defaults:** low speeds, output clamping, anti-windup, PID reset on line loss, and automatic LAND when the program exits.
- **Camera flexibility:** works with the Raspberry Pi AI Camera (Picamera2) or a USB camera.

## Project files

| File | Description |
| --- | --- |
| [`light_following_pid.py`](light_following_pid.py) | PID-based line follower (current version) |

## Getting started

### Hardware

- ArduPilot-based flight controller (Copter) connected to the Pi over USB serial
- Raspberry Pi with a downward-facing camera (Pi AI Camera or USB camera)
- White LED line on the floor, plus a red LED as the stop marker

### Requirements

- Python 3.8+
- [DroneKit](https://github.com/dronekit/dronekit-python), [pymavlink](https://github.com/ArduPilot/pymavlink), OpenCV, NumPy
- [Picamera2](https://github.com/raspberrypi/picamera2) (only if `USE_PICAMERA2 = True`)

### Install

```bash
pip install dronekit pymavlink numpy opencv-python
# On Raspberry Pi OS, Picamera2 is normally installed via apt:
sudo apt install python3-picamera2
```

> **Tip:** DroneKit can fail on newer Python versions with
> `module 'collections' has no attribute 'MutableMapping'`. If you hit this,
> use a maintained fork of DroneKit or patch the import in the installed package.

### Configure

Edit the constants at the top of `light_following_pid.py`:

```python
USE_PICAMERA2 = True              # False for a USB camera
drone_connect = "/dev/ttyACM0"    # or "tcp:127.0.0.1:5762" for SITL
BAUD_RATE = 57600
TAKEOFF_ALTITUDE = 0.4            # metres
VELOCITY_FORWARD = 0.1            # max forward speed, m/s
```

### Run

```bash
python3 light_following_pid.py
```

The drone arms, switches to `GUIDED`, takes off, and starts following the line. Press **`q`** in the video window to land.

> **Test in SITL or with propellers removed first.** Always keep your RC transmitter ready to take over from `GUIDED`.

## How the PID control works

| Loop | Input | Output |
| --- | --- | --- |
| Lateral | Line offset from image centre, normalised to -1..1 | Sideways velocity `vy` (m/s) |
| Yaw | Angle between the line and the image vertical (degrees) | Yaw rate (deg/s) |

Forward speed is scaled down automatically when either error is large.

### Tuning

Gains live in the `PID Configuration` block:

```python
LAT_KP, LAT_KI, LAT_KD = 0.30, 0.02, 0.05
YAW_KP, YAW_KI, YAW_KD = 0.80, 0.00, 0.10
```

Suggested approach:

1. Tune **yaw** first, then lateral.
2. Start with Ki and Kd at 0 and raise Kp until the response is quick but not oscillating.
3. Add Kd to damp oscillation; add a small Ki only if the drone settles off-centre.

**Sign conventions:** `LATERAL_SIGN` and `YAW_SIGN` depend on how your camera is mounted. If the drone moves away from the line or turns the wrong way, flip the sign.

## Getting help

- Check the comments in [`light_following_pid.py`](light_following_pid.py) for what each setting does.
- ArduPilot guided-mode and MAVLink behaviour: [ardupilot.org/copter](https://ardupilot.org/copter/)
- DroneKit usage: [dronekit-python docs](https://dronekit-python.readthedocs.io/)


