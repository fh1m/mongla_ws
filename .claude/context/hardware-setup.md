# Hardware Setup & Pool Operations — BRACU Duburi 4.2

Pre-dive runbook for the actual sub. The canonical hardware spec lives
in [`vehicle-spec.md`](./vehicle-spec.md); this file is the *operational*
companion — wiring, IPs, checklists, recovery.

---

## 1. Physical hardware (one-line each)

| Component         | Model                              | Interface              | IP / port                     |
|-------------------|------------------------------------|------------------------|-------------------------------|
| Hull              | Octagonal Marine 5083 aluminum     | —                      | —                             |
| Flight controller | Pixhawk 2.4.8 (ArduSub 4.x)        | USB → BlueOS           | via MAVLink endpoint 14550    |
| Companion         | Raspberry Pi running BlueOS        | Ethernet switch        | `192.168.2.1`                 |
| Main SBC          | Nvidia Jetson Orin Nano            | Ethernet switch        | `192.168.2.69` static         |
| External IMU      | ESP32-C3 + BNO085                  | USB CDC to Jetson      | `/dev/ttyACM0` (typical)      |
| DVL               | Nortek Nucleus1000                 | Ethernet switch        | `192.168.2.201` (driver TODO) |
| Cameras           | 2× Blue Robotics Low-Light HD USB  | USB to Jetson          | `/dev/v4l/by-id/...`          |
| ESCs              | 8× Blue Robotics Basic ESC         | Pixhawk MAIN PWM       | —                             |
| Thrusters         | 8× Blue Robotics T200              | via ESCs               | —                             |
| Depth sensor      | Bar30                              | I2C → Pixhawk          | internal (read via AHRS2)     |
| Tether            | FathomX                            | Ethernet switch        | per-port speed                |
| Power             | Dual LiPo (propulsion + compute)   | isolated rails         | —                             |
| Payload           | Torpedo / grabber / dropper        | Pixhawk AUX (servo)    | AUX1..AUX6 (`set_servo_pwm`)  |
| Kill switch       | Latex-balloon, non-magnetic        | mechanical             | —                             |

---

## 2. Network topology

```
                    [Tether: FathomX over Ethernet]
                                ↕
                  [Topside Laptop / Ground Station]
                  192.168.2.xxx (same subnet via tether)
                                ↕
                  ┌────────────────────────────┐
                  │   AUV Ethernet Switch      │
                  │   192.168.2.x subnet       │
                  └─┬─────┬─────┬──────────────┘
                    │     │     │
              [Jetson]  [Pi]   [DVL Nucleus1000]
              .2.69   .2.1     .2.201   (driver TODO)
                            Gateway: .2.2
```

Accessing the AUV from the ground station:

```bash
# SSH to Jetson (ROS2 host)
ssh fh1m@192.168.2.69

# Remote desktop (Remmina or any VNC client)
# 192.168.2.69:5900

# BlueOS web UI (telemetry, calibration, video)
# http://192.168.2.1

# Tap the same MAVLink stream from a third machine (read-only):
mavproxy.py --master=udpin:0.0.0.0:14551
```

---

## 3. MAVLink endpoint (BlueOS configuration)

In BlueOS web UI → **Vehicle → Pixhawk → Endpoints** create:

```
Name: inspector
Type: UDP Client
IP:   192.168.2.69    (Jetson — where our code runs)
Port: 14550

Flow: Pixhawk → USB → Pi (BlueOS) → UDP client → Jetson:14550
      Jetson: mavutil.mavlink_connection("udpin:0.0.0.0:14550")
```

Meaning:

- BlueOS is the MAVLink router.
- Pi *sends* UDP packets to the Jetson (client mode).
- Jetson *receives* with `udpin` (server mode, binds to port 14550).

Direct USB serial fallback (no BlueOS — `mode:=desk`):

```python
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
```

(Today the codebase doesn't switch on `mode:=desk` — it uses the same
UDP listener and assumes BlueOS is between you and the Pixhawk. If you
need raw serial, edit `connection_config.PROFILES['desk']`.)

---

## 4. External IMU — ESP32-C3 + BNO085

The TDR Appendix A still lists VectorNav VN200; we deviated to BNO085
(see [`vehicle-spec.md`](./vehicle-spec.md) §"Why BNO085 instead of the
TDR's VectorNav VN200" for the full rationale).

### Wiring

- ESP32-C3 dev board → BNO085 breakout via I2C (SDA, SCL, GND, 3.3V).
- ESP32-C3 → Jetson via USB-C (CDC serial). Mounts as `/dev/ttyACM0`
  (or similar). Mounting orientation of the BNO085 doesn't matter —
  we capture a one-shot Pixhawk-mag offset at boot to align it with
  Earth.

### Firmware contract

JSON-line over USB CDC at 115200 baud, 50 Hz target. Wire format and
reference Arduino sketch:
[`src/duburi_sensors/firmware/esp32c3_bno085.md`](../../src/duburi_sensors/firmware/esp32c3_bno085.md)

Smoke-test before relying on it:

```bash
cat /dev/ttyACM0 | head -20
# Expect lines like {"yaw": 123.45, "ts": 12345}
```

### Software path

```bash
ros2 run duburi_manager start --ros-args \
    -p mode:=pool \
    -p yaw_source:=bno085 \
    -p bno085_port:=/dev/ttyACM0 \
    -p bno085_baud:=115200
```

The manager auto-runs the one-shot Pixhawk-mag offset at startup and
prints the locked offset in the banner. See
[`sensors-pipeline.md`](./sensors-pipeline.md) §"Calibration model"
for the full design.

---

## 5. DVL — Nortek Nucleus1000

> **Driver status: STUB.** `dvl_stub.py` exists in `duburi_sensors`
> but raises `NotImplementedError` on instantiation. Tracked in
> [`known-issues.md`](./known-issues.md).

When the driver lands:

- TCP (primary): `192.168.2.201:9000`
- Serial: `/dev/ttyUSB0` at 115200 baud (backup)

Key Nucleus1000 packet types:

| Packet | Contents                                                            |
|--------|---------------------------------------------------------------------|
| `0xD2` | AHRS — heading, roll, pitch, depth (attitude backup)                |
| `0xDC` | INS — absolute position x/y/z, velocity                             |
| `0xB4` | Bottom track — velocity relative to bottom (distance traveled)      |
| `0xBE` | Water track — water-relative velocity                               |
| `0xAA` | Altimeter — distance to seafloor                                    |

Calibration constant from RoboSub 2025: `dvl_depth_match = 0.78`
(offset from DVL depth to true depth). Will move into `duburi_sensors`
config when the driver is integrated.

---

## 6. Cameras

```python
CAMERA_FRONT_PIPELINE = (
    "v4l2src device=/dev/v4l/by-id/"
    "usb-H264_USB_Camera_H264_USB_Camera_2020032801-video-index0 "
    "! video/x-raw, width=640, height=480, framerate=30/1 "
    "! videoconvert ! video/x-raw, format=BGR ! appsink"
)
CAMERA_DOWN_PIPELINE = (
    "v4l2src device=/dev/v4l/by-id/"
    "usb-Sonix_Technology_Co.__Ltd._exploreHD_USB_Camera_SN00009-video-index0 "
    "! video/x-raw, width=640, height=480, framerate=30/1 "
    "! videoconvert ! video/x-raw, format=BGR ! appsink"
)

import cv2
cap = cv2.VideoCapture(CAMERA_FRONT_PIPELINE, cv2.CAP_GSTREAMER)
```

Vision lives in a future `duburi_vision` package — this repo does not
own image processing yet.

---

## 7. Payload actuators (torpedo / grabber / dropper)

Driven from the Pixhawk via `MAV_CMD_DO_SET_SERVO` on AUX outputs
(NOT GPIO). The actuation board (MOSFET-based) sits between the
Pixhawk AUX channels and the actuator hardware.

```python
# Through Pixhawk. Internal +8 AUX offset and PWM clamping handled for you.
pixhawk.set_servo_pwm(aux_n=1, pwm=1900)   # AUX1 = torpedo, etc.
```

The grabber has a current sensor on the actuator line — successful
grasp is detected by the current spike + safety trip. Wiring runs
through the same MOSFET board.

> Older notes referred to Jetson GPIO pins for the torpedo/dropper.
> That is not how Duburi 4.2 ships — payload is autopilot-side via AUX
> servos, with the kill-switch and current sensing handled at the
> actuation board. Don't add `Jetson.GPIO` to this codebase.

---

## 8. Pool testing procedure

### Pre-dive checklist

```
[ ] O-rings greased and seated properly
[ ] All penetrators hand-tight
[ ] Battery charged (> 90%) — both LiPos
[ ] Tether secured to AUV strain relief
[ ] Jetson booted (can SSH in)
[ ] All cameras visible via BlueOS
[ ] MAVLink connected (BlueOS shows "connected", AHRS2 rate > 0)
[ ] ArduSub params correct (ARMING_CHECK, motor mapping, FRAME_TYPE=vectored_6dof)
[ ] Test arm/disarm via QGC
[ ] Test each thruster individually (low PWM, 10 seconds)
[ ] DVL connected and sending packets (when driver lands)
[ ] BNO085 streaming JSON to Jetson (cat /dev/ttyACM0)
[ ] Code deployed to Jetson, colcon build clean
[ ] Run code → /duburi/state shows armed=false, mode=MANUAL, yaw, depth
[ ] Test depth hold at 0.3 m (brief) — ros2 run duburi_manager duburi set_depth -0.3
[ ] Test heading hold (rotate 90° left, 90° right)
```

### Bringing up the stack on the Jetson

```bash
ssh fh1m@192.168.2.69
cd ~/Ros_workspaces/duburi_ws
source install/setup.bash

# Default — ArduSub AHRS as yaw source
ros2 run duburi_manager start --ros-args -p mode:=pool

# With external BNO085 (after firmware is flashed + sensor wired)
ros2 run duburi_manager start --ros-args \
    -p mode:=pool -p yaw_source:=bno085 -p bno085_port:=/dev/ttyACM0
```

### Scripted mission

```bash
# In a second SSH session on the Jetson
ros2 run duburi_planner mission --list
ros2 run duburi_planner mission square_pattern
```

Mission scripts live in
`src/duburi_planner/duburi_planner/missions/` (`square_pattern.py` is the
legacy `test_runner` choreography; `arc_demo.py` and `heading_lock_demo.py`
exercise the newer verbs). The TDR target is YASMIN FSM in
`src/duburi_planner/duburi_planner/state_machines/`; that's deferred until
missions get nontrivial (see [`mission-design.md`](./mission-design.md)).

### Emergency recovery

1. SSH to Jetson: `ssh fh1m@192.168.2.69`.
2. From the manager terminal, Ctrl-C → `Duburi.stop()` triggers
   neutral RC + disarm.
3. From a separate terminal: `ros2 run duburi_planner duburi disarm`.
4. From BlueOS terminal / MAVProxy: `mode surface`.
5. Pull tether to physically guide the AUV if needed.

> There is **no** `/duburi/emergency_surface` service in this codebase
> today. Earlier docs referenced it; we removed the reference. If you
> need one, add it to the action dispatch instead of inventing a new
> service.

---

## 9. Jetson Orin Nano setup notes

```bash
nvidia-smi   # Check GPU is visible (for vision when it lands)
python3 -c "import torch; print(torch.cuda.is_available())"

# Static IP — /etc/netplan/01-network.conf:
# Address: 192.168.2.69/24, Gateway: 192.168.2.2, DNS: 8.8.8.8

# ROS2 settings
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# Serial port permissions for BNO085
sudo usermod -aG dialout "$USER"   # log out / back in
ls -l /dev/ttyACM0                 # crw-rw---- root dialout
```

---

## 10. BlueOS tips

- **Access**: `http://192.168.2.1` from any device on the same subnet.
- **ArduSub params**: Terminal → `param show <PATTERN>` or web param editor.
- **Video streams**: Video tab → manage camera streams.
- **Log download**: Tools → Log download.
- **Calibration**: Sensors tab → compass, accelerometer.
- **Ping sonar**: Extensions → Ping Sonar (if installed).
- **MAVLink health**: MAVLink Inspector shows live message rates. If
  `AHRS2` rate is 0, something is wrong with ArduSub / Pixhawk.

---

## 11. Troubleshooting common issues

| Problem                        | Likely cause                              | Fix                                                          |
|--------------------------------|-------------------------------------------|--------------------------------------------------------------|
| Can't ARM                      | Pre-arm check fails                       | Check `ARMING_CHECK` param, watch `[ARDUB]` STATUSTEXT lines |
| Vehicle sinks immediately      | Buoyancy misconfigured                    | Adjust ballast foam                                          |
| Heading drifts in ALT_HOLD     | Compass interference (mag noise)          | Try `yaw_source:=bno085`; recalibrate compass away from thrusters |
| Depth oscillates               | ArduSub PID gains too hot                 | Reduce `PSC_POSZ_P` / `PSC_VELZ_P` via QGC                   |
| Can't connect MAVLink          | BlueOS endpoint misconfigured             | Verify `inspector` endpoint matches Jetson IP                |
| DVL not reading                | Wrong IP / TCP port / driver not yet live | Ping `192.168.2.201`, check port 9000; driver is a stub      |
| Camera not found               | Device path changed                       | `ls /dev/v4l/by-id/` and update pipeline string              |
| Node crashed                   | Missing Python dep                        | `pip install pymavlink` on Jetson; or rebuild via `./build_duburi.sh` |
| `BNO085 calibration timed out` | Pixhawk yaw or BNO yaw stayed unavailable | Confirm both work in isolation (sensors_node first)          |
| Payload (torpedo/grabber) silent or wrong actuator fires | Calling `set_servo_pwm` with the wrong AUX index | Use `aux_n` matching the AUX1..AUX6 silkscreen — the +8 offset is added internally by `pixhawk.py`. Range-checked since 2026-04. |
