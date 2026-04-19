# ArduSub Reference — Duburi AUV

Specific knowledge about ArduSub firmware, parameters, behaviors, and quirks.
Hardware target: Pixhawk 2.4.8 running ArduSub 4.x.

---

## ArduSub vs ArduCopter

ArduSub is ArduPilot ported for underwater vehicles. Key differences:
- Altitude = depth (negative = below surface)
- No GPS by default; uses depth sensor + DVL for positioning
- Frame types: vectored (6 thrusters), vectored_6dof (8 thrusters/BlueROV2 Heavy)
- MANUAL mode = direct RC passthrough (not "full manual" like ArduCopter)
- ALT_HOLD = depth hold (not altitude hold in air sense)

---

## Frame: vectored_6dof (Our Frame)

> BRACU **Duburi 4.2** uses this same `vectored_6dof` frame type (8x T200 thrusters in the same vectored layout). The Gazebo SITL **proxy** is BlueROV2 Heavy because it ships an off-the-shelf model with the identical frame. Hull shape and exact mass differ from the real sub; control behavior matches. See [`vehicle-spec.md`](./vehicle-spec.md).

BlueROV2 Heavy configuration — 8 thrusters:
```
          ↑ Forward
    T5[↗] T6[↖]
    T7[↙] T8[↘]   ← These 4 are tilted 45° for horizontal thrust
    T1[↑] T2[↑]   ← These 4 are vertical (depth control)
    T3[↑] T4[↑]
```

Degrees of Freedom:
- Forward/Backward (surge)
- Left/Right (sway)
- Up/Down (heave)
- Yaw (rotation around Z)
- Roll (rotation around X) — unique to Heavy config
- Pitch (rotation around Y) — unique to Heavy config

---

## Flight Modes (ArduSub Specific)

| Mode | ID | Description |
|---|---|---|
| STABILIZE | 0 | Attitude stabilized. Pilot controls movement via RC. ArduSub holds level. |
| ACRO | 1 | Rate control. Full manual (not used for missions) |
| ALT_HOLD | 2 | **Depth Hold.** Autopilot maintains depth, pilot controls XY+yaw |
| AUTO | 3 | Execute pre-programmed waypoint mission |
| GUIDED | 4 | Accept position setpoints from GCS/companion |
| CIRCLE | 7 | Circle around a point |
| SURFACE | 9 | Emergency surface: ascend to depth=0 |
| POSHOLD | 16 | Maintain XY position (requires DVL or GPS) |
| MANUAL | 19 | **Full RC passthrough.** No autopilot assist. |

```python
# In pymavlink, use these exact mode name strings:
master.mode_mapping():
# 'STABILIZE': 0, 'ALT_HOLD': 2, 'AUTO': 3, 'GUIDED': 4,
# 'CIRCLE': 7, 'SURFACE': 9, 'POSHOLD': 16, 'MANUAL': 19
```

---

## Key ArduSub Parameters

### EKF3 (Extended Kalman Filter — state estimation)

```
EK3_ENABLE = 1         # Use EKF3 (required)
EK3_SRC1_POSXY = 3     # XY position source: 3=DVL
EK3_SRC1_VELXY = 5     # XY velocity: 5=DVL
EK3_SRC1_POSZ = 1      # Z position: 1=Baro (depth sensor)
EK3_SRC1_VELZ = 0      # Z velocity: 0=None
EK3_SRC1_YAW = 1       # Yaw: 1=compass
```

### Position/Velocity Controllers (PSC prefix)

```
PSC_POSXY_P = 2.5       # XY position P gain
PSC_VELXY_P = 5.0       # XY velocity P gain
PSC_VELXY_I = 0.5       # XY velocity I gain
PSC_VELXY_D = 0.8       # XY velocity D gain
PSC_POSZ_P  = 3.0       # Z position P gain
PSC_ACCZ_P  = 0.5       # Z acceleration P gain
PSC_ACCZ_I  = 1.0       # Z acceleration I gain
PSC_ACCZ_FF = 0.0       # Z acceleration feedforward
```

### Thruster/Motor Parameters

```
MOT_PWM_TYPE = 0        # 0=Normal PWM, 4=DShot300
MOT_PWM_MIN = 1100      # Minimum thruster PWM
MOT_PWM_MAX = 1900      # Maximum thruster PWM
MOT_SPIN_MIN = 0.15     # Min throttle as fraction
MOT_THST_EXPO = 0.0     # Thrust curve expo (0=linear)
```

### Depth/Surface Parameters

```
SURFACE_DEPTH = -0.10   # Depth considered "at surface" (meters, negative)
FS_CRASH_CHECK = 0      # Disable crash detection (not relevant underwater)
```

### Arming Checks

```
ARMING_CHECK = 0        # Disable all pre-arm checks (for testing)
                        # OR fine-grained bit field to disable specific checks
ARMING_CHECK = 8192     # Disable only "parameters" check
```

### RC Failsafe

```
FS_THR_ENABLE = 0       # Disable throttle failsafe (companion sends RC)
FS_THR_VALUE = 975      # Failsafe threshold PWM
RC_FS_THR = 975
```

---

## ArduSub Control Inputs (Priority Order)

ArduSub accepts control from multiple sources, higher priority wins:

1. **RC override** (RC_CHANNELS_OVERRIDE) — highest priority
2. **Guided commands** (SET_POSITION_TARGET, SET_ATTITUDE_TARGET) — only in GUIDED
3. **Mission** (AUTO mode)
4. **Pilot RC** (physical radio) — lowest priority

In our system: we send RC override (source 1) for all control. When we want
ArduSub to hold depth (ALT_HOLD), we set the vertical RC channel to neutral (1500)
and let the firmware's depth PID take over.

---

## AHRS2 Message — Primary Data Source

```
AHRS2 (message ID 178):
  roll     : float  — Roll angle (radians, -π to π)
  pitch    : float  — Pitch angle (radians)
  yaw      : float  — Yaw/heading (radians, -π to π → convert: if<0, +=2π)
  altitude : float  — Depth in meters (NEGATIVE when underwater, e.g. -0.5)
  lat      : int32  — Latitude (degE7, not useful for AUV)
  lng      : int32  — Longitude (degE7, not useful for AUV)
```

**Getting yaw in 0-360 degrees:**
```python
msg = master.recv_match(type='AHRS2', blocking=True)
yaw_deg = math.degrees(msg.yaw)
if yaw_deg < 0:
    yaw_deg += 360   # Convert -180..180 → 0..360
depth = msg.altitude  # e.g. -0.5 for 50cm deep
```

---

## BlueOS Integration

BlueOS (on Raspberry Pi, IP 192.168.2.1) provides:
- MAVLink router: creates endpoints, forwards to Jetson (192.168.2.69:14550)
- Web interface: http://192.168.2.1
- Video streams management
- Cockpit (web-based GCS)
- Parameter configuration
- ArduSub firmware update

**BlueOS MAVLink endpoint setup:**
```
Type: UDP Client
IP: 192.168.2.69 (Jetson)
Port: 14550
```
This means BlueOS SENDS to Jetson (client mode).
Jetson LISTENS with: `mavutil.mavlink_connection("udpin:0.0.0.0:14550")`

---

## ArduSub Quirks & Gotchas

### 1. Depth Sign Convention
Depth is negative when underwater. Target -0.5 = 0.5m below surface.
AHRS2.altitude will show -0.5 when at 50cm depth.

### 2. ALT_HOLD RC Override
In ALT_HOLD, sending ch3 (throttle) = 1500 (neutral) lets ArduSub maintain depth.
Sending ch3 > 1500 ascends, < 1500 descends (relative change, not absolute depth).
To set absolute depth: use `set_position_target_global_int_send` with depth value.

### 3. Servo AUX Offset
AUX outputs 1-8 on Pixhawk correspond to MAVLink servo instances 9-16.
```python
# AUX1 = servo instance 9
set_servo_pwm(servo_n=1, microseconds=1500)
# Uses MAV_CMD_DO_SET_SERVO with servo_n+8 = 9
```

### 4. RC Override Timeout
If RC_CHANNELS_OVERRIDE messages stop, ArduSub reverts to physical RC input
(or failsafe if no RC). Always maintain RC override stream in MANUAL mode.
In ALT_HOLD + GUIDED mode, you can pause RC override and use setpoints instead.

### 5. Mode Change Verification
Always verify mode change was accepted with COMMAND_ACK. Mode changes can fail
if pre-arm checks are triggered (e.g., changing to GUIDED without GPS/DVL).

### 6. Arming from Companion
Companion computers (Jetson) can arm via MAVLink. Pre-arm checks may block this.
For competition: `ARMING_CHECK = 0` disables all checks, or use specific bit to
disable only the GPS check (since we don't have GPS).

### 7. EKF3 Initialization
After boot, EKF3 needs time to initialize before ALT_HOLD/POSHOLD works.
Wait for `EKF3 IMU0 is using barometer` in STATUSTEXT messages.
Typically 10-30 seconds after power-on.

### 8. DVL Depth vs Bar30 Depth
DVL (Nucleus1000) provides its own depth measurement.
Bar30 depth goes through ArduSub EKF3 → AHRS2.altitude.
DVL depth is more accurate but needs offset calibration: `DVL_DEPTH_MATCH = 0.78`.

### 9. HEARTBEAT Required at ≥ 1 Hz
If our companion heartbeat stops, ArduSub triggers failsafe after ~3 seconds.
This disarms the vehicle. Always run heartbeat in a separate thread.

---

## ArduSub SITL Specific Notes

SITL connection: `udpin:0.0.0.0:14550` (same as hardware — transparent!)

SITL does NOT simulate:
- Real thruster dynamics (thrust is idealized)
- Real buoyancy physics (Gazebo sim handles this)
- Actual DVL measurements (use ground_truth_to_mavros bridge)

SITL does simulate:
- Full EKF3 state estimation
- All MAVLink message handling
- Mode logic and PID controllers
- Arming/disarming sequences

This means: if our MAVLink commands work in SITL, they'll work on hardware.
The only differences are physical tuning (PID gains may need adjustment in water).
