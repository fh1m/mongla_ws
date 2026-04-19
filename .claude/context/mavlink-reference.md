# MAVLink Reference — Duburi AUV

> Companion docs:
> [`ardusub-canon.md`](./ardusub-canon.md) (mode + parameter + failsafe
> theory),
> [`heading-lock.md`](./heading-lock.md) (Ch4 yaw-rate streamer),
> [`axis-isolation.md`](./axis-isolation.md) (per-channel ownership).

Sources: mavlink.io, ardusub.com/developers/pymavlink, ArduPilot
ArduSub source, our own pixhawk.py.

---

## KEY MESSAGE TYPES (what we use)

### SENT TO PIXHAWK

| Message | ID | Function | Key Fields |
|---|---|---|---|
| `HEARTBEAT` | 0 | Keep-alive (≥1 Hz mandatory) | type, autopilot, base_mode |
| `SET_MODE` | 11 | Change flight mode | base_mode, custom_mode |
| `COMMAND_LONG` | 76 | General commands (arm, servo, etc) | command, param1–7 |
| `RC_CHANNELS_OVERRIDE` | 70 | Direct thruster PWM | chan1–18_raw (1100–1900) |
| `SET_ATTITUDE_TARGET` | 82 | Attitude setpoint (heading) | time_boot_ms, type_mask, q, rates |
| `SET_POSITION_TARGET_GLOBAL_INT` | 86 | Depth setpoint | time_boot_ms, frame, type_mask, alt |
| `PARAM_REQUEST_READ` | 20 | Read single param | param_id |
| `PARAM_REQUEST_LIST` | 21 | Read all params | — |
| `PARAM_SET` | 23 | Write param | param_id, param_value, param_type |
| `MANUAL_CONTROL` | 69 | Alt to RC override | x,y,z,r (–1000..1000), buttons |

### RECEIVED FROM PIXHAWK

| Message | ID | Function | Key Fields We Use |
|---|---|---|---|
| `HEARTBEAT` | 0 | System alive, current mode | custom_mode, system_status |
| `AHRS2` | 178 | **Primary attitude + depth** | roll, pitch, yaw (rad), altitude (m) |
| `ATTITUDE` | 30 | Raw IMU attitude | rollspeed, pitchspeed, yawspeed |
| `SYS_STATUS` | 1 | System health | voltage_battery, current_battery |
| `BATTERY_STATUS` | 147 | Detailed battery | voltages[], current_battery, current_consumed |
| `COMMAND_ACK` | 77 | Command result | command, result (0=success) |
| `PARAM_VALUE` | 22 | Parameter response | param_id, param_value |
| `STATUSTEXT` | 253 | Log/debug text | severity, text |
| `RC_CHANNELS` | 65 | Current RC input | chan1_raw..chan18_raw |

---

## COMMAND_LONG COMMANDS (MAV_CMD)

```python
# Arm/Disarm
MAV_CMD_COMPONENT_ARM_DISARM = 400
# param1: 1=arm, 0=disarm
# param2: 0=normal, 21196=force (bypass pre-arm checks)

# Servo PWM
MAV_CMD_DO_SET_SERVO = 183
# param1: servo instance (AUX1=9, AUX2=10, AUX3=11)
# param2: PWM in microseconds

# Set message interval
MAV_CMD_SET_MESSAGE_INTERVAL = 511
# param1: message ID
# param2: interval in microseconds (1e6/Hz)

# Reboot autopilot
MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246
# param1: 1=reboot, 0=shutdown

# Mount control (camera gimbal)
MAV_CMD_DO_MOUNT_CONTROL = 205
# param1: tilt (centidegrees), param2: roll, param3: pan
```

---

## RC_CHANNELS_OVERRIDE — Deep Dive

```python
# MAVLink 2.0: 18 channels
# MAVLink 1.0: 8 channels (why MAVLINK20=1 matters)

# PWM Values:
# 1100 µs = full reverse/min
# 1500 µs = neutral/stop
# 1900 µs = full forward/max
# 65535   = "do not override" (release to RC input)

# Our channel mapping:
CHANNEL = {
    'PITCH':    1,    # index 0 in array
    'ROLL':     2,    # index 1
    'THROTTLE': 3,    # index 2 — vertical/depth
    'YAW':      4,    # index 3
    'FORWARD':  5,    # index 4
    'LATERAL':  6,    # index 5
    # Channels 7-18 available for accessories
}

# Sending:
vals = [65535] * 18
vals[4] = 1600       # Forward channel = 75% forward
master.mav.rc_channels_override_send(
    master.target_system, master.target_component, *vals)
```

---

## SET_ATTITUDE_TARGET — Type Mask

```python
# type_mask bitmask — set bit to IGNORE that field:
ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE  = 1   # bit 0
ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE = 2   # bit 1
ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE   = 4   # bit 2
ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE        = 64  # bit 6
ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE        = 128 # bit 7

# Most common use: attitude only, ignore rates and throttle
type_mask = (
    mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE |
    mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE |
    mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE |
    mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
)
# = 64 | 1 | 2 | 4 = 71

# Quaternion from Euler angles:
from pymavlink.quaternion import QuaternionBase
import math
q = QuaternionBase([math.radians(roll_deg), 
                    math.radians(pitch_deg), 
                    math.radians(yaw_deg)])
# q is [w, x, y, z] internally
```

---

## SET_POSITION_TARGET — Type Mask

```python
# IGNORE flags — OR together fields you DON'T want used:
POSITION_TARGET_TYPEMASK_X_IGNORE        = 1
POSITION_TARGET_TYPEMASK_Y_IGNORE        = 2
POSITION_TARGET_TYPEMASK_Z_IGNORE        = 4
POSITION_TARGET_TYPEMASK_VX_IGNORE       = 8
POSITION_TARGET_TYPEMASK_VY_IGNORE       = 16
POSITION_TARGET_TYPEMASK_VZ_IGNORE       = 32
POSITION_TARGET_TYPEMASK_AX_IGNORE       = 64
POSITION_TARGET_TYPEMASK_AY_IGNORE       = 128
POSITION_TARGET_TYPEMASK_AZ_IGNORE       = 256
POSITION_TARGET_TYPEMASK_YAW_IGNORE      = 1024
POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE = 2048

# For depth-only control (ignore everything except Z/alt):
DEPTH_ONLY_MASK = (1 | 2 | 8 | 16 | 32 | 64 | 128 | 256 | 1024 | 2048)
# = 0b0000111111111011 = 3579
# Equivalent binary: 0b0000111111111000 (from 2023 codebase)

# Coordinate frames:
MAV_FRAME_GLOBAL_INT = 5   # WGS84, lat/lon in degE7, alt in meters MSL
MAV_FRAME_LOCAL_NED  = 1   # Local NED origin
MAV_FRAME_BODY_NED   = 8   # Body-relative
```

---

## MAVLink Services (Acknowledgment-Based)

For reliable command delivery, use the Command Protocol:
1. Send `COMMAND_LONG` with `confirmation=0`
2. Wait for `COMMAND_ACK` response
3. Check `ack.result == MAV_RESULT_ACCEPTED (0)`
4. If `MAV_RESULT_IN_PROGRESS (5)`, wait for follow-up ACK
5. If `MAV_RESULT_FAILED (4)`, retry or abort

```python
def send_command_wait_ack(command, *params, timeout=5.0):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        command, 0, *params)
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=timeout)
    if ack and ack.command == command:
        return ack.result == 0   # 0 = MAV_RESULT_ACCEPTED
    return False
```

---

## Parameter Protocol

```python
# Read all parameters
master.mav.param_request_list_send(master.target_system, master.target_component)
params = {}
while True:
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    if not msg:
        break
    params[msg.param_id] = msg.param_value
    if msg.param_index == msg.param_count - 1:
        break

# Read single parameter
master.mav.param_request_read_send(
    master.target_system, master.target_component,
    b'SURFACE_DEPTH', -1)           # param_id bytes, index=-1 means use name
msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)

# Write parameter (temporary, RAM only)
master.mav.param_set_send(
    master.target_system, master.target_component,
    b'PSC_POSXY_P',                 # Param name as bytes, null-padded to 16 chars
    2.5,                            # Value
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
```

---

## Requesting Message Intervals

```python
def request_message_hz(msg_id: int, hz: float):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        msg_id,
        int(1e6 / hz),   # interval in microseconds
        0, 0, 0, 0, 0)

# Useful message IDs:
MAVLINK_MSG_ID_AHRS2    = 178
MAVLINK_MSG_ID_ATTITUDE = 30
MAVLINK_MSG_ID_SYS_STATUS = 1
MAVLINK_MSG_ID_BATTERY_STATUS = 147

# Example: get AHRS2 at 20 Hz:
request_message_hz(178, 20)
```

---

## MANUAL_CONTROL Alternative to RC Override

```python
# MANUAL_CONTROL axes: -1000 to +1000 (normalized)
# x=pitch, y=roll, z=thrust(0-1000 not -1000!), r=yaw
# For submarines: z=500 is neutral, >500 up, <500 down

master.mav.manual_control_send(
    master.target_system,
    x=0,        # pitch
    y=0,        # roll
    z=500,      # throttle (500=neutral in z axis)
    r=0,        # yaw
    buttons=0)  # button bitmask
```

Note: We prefer `RC_CHANNELS_OVERRIDE` over `MANUAL_CONTROL` — it gives direct per-channel PWM control and matches our reference codebase patterns.

---

## Per-call audit (every send_* / set_* on `pixhawk.py`)

Every method on `Pixhawk` that puts a frame on the wire emits a
single `[MAV <fn>[ cmd=<verb>]] <body>` DEBUG line via
`_log_mavlink(...)`. Launch the manager with `debug:=true` to flip
the trace on (it also raises the logger level to DEBUG so the lines
actually print):

```bash
ros2 run duburi_manager auv_manager --ros-args -p debug:=true
```

The banner prints `DEBUG TRACE: ON` so you can see at a glance the
trace is active.

| `pixhawk.py` method            | MAVLink message                                | Used by                            | Notes                                                                                          |
| ------------------------------ | ---------------------------------------------- | ---------------------------------- | ---------------------------------------------------------------------------------------------- |
| `arm()` / `disarm()`           | `COMMAND_LONG (MAV_CMD_COMPONENT_ARM_DISARM)`  | `Duburi.arm` / `Duburi.disarm`     | Wait for `COMMAND_ACK`; ArduSub also needs the heartbeat-armed bit before motors hot, hence we poll `is_armed()` too. |
| `set_mode(name)`               | `SET_MODE`                                     | `Duburi.set_mode` and auto-engage in `set_depth` / yaw / `lock_heading` | Mode mapping comes from `master.mode_mapping()`; never hardcode ints. |
| `send_rc_override(**chans)`    | `RC_CHANNELS_OVERRIDE` (all six driving channels) | constant/eased motion, `arc`, `Heartbeat`, `HeadingLock` | ArduSub trips `FS_PILOT_INPUT` after ~3 s without overrides -- we re-send at >=5 Hz. 1500 = neutral, 65535 = no-override. |
| `send_rc_translation(**chans)` | `RC_CHANNELS_OVERRIDE` (Ch3/Ch5/Ch6 only)      | translation while `lock_heading` active | Sends 1500 on Ch3/Ch5/Ch6 (or commanded value), 65535 on Ch1/Ch2/Ch4 so the lock thread keeps Ch4 authority. |
| `send_neutral()`               | `RC_CHANNELS_OVERRIDE` (six 1500s)             | `Duburi.stop`, brake settles, `Heartbeat` | Active hold. Use `release_rc_override` instead when you want ArduSub's automation to run unhindered. |
| `release_rc_override()`        | `RC_CHANNELS_OVERRIDE` (six 65535s)            | `Duburi.pause`, manager shutdown   | Pilot OFF the loop. ArduSub's mode automation (`ALT_HOLD`, `POSHOLD`) runs without us. |
| `set_target_depth(depth_m)`    | `SET_POSITION_TARGET_GLOBAL_INT`               | `motion_depth.hold_depth` (one-shot per `set_depth`) | Typemask ignores everything but Z. `alt = depth_m`, negative = below surface. ALT_HOLD or GUIDED only. After the target is reached we hand depth back to ALT_HOLD's onboard PID -- no continuous streaming. |
| `set_servo_pwm(aux_n, pwm)`    | `COMMAND_LONG (MAV_CMD_DO_SET_SERVO)`          | payload (torpedo, grabber, dropper) | AUX1..AUX6 maps to servo channels 9..14. We add the +8 offset internally; calling with 1..8 would silently drive a thruster. PWM clamped to 1100..1900 to prevent stall current spikes. |
| `set_message_rate(msg_id, hz)` | `COMMAND_LONG (MAV_CMD_SET_MESSAGE_INTERVAL)`  | `auv_manager_node.MESSAGE_RATES` startup | Without this ArduSub picks defaults (often 4 Hz AHRS2). Pin to 50 Hz so our control loops aren't bottlenecked. |
| `send_heartbeat()`             | `HEARTBEAT`                                    | `auv_manager_node` 0.5 Hz timer    | Mandatory >= 1 Hz or ArduSub considers us disconnected and may drop overrides. |
| `clear_ack()` / `wait_ack()`   | (reads `COMMAND_ACK` cache)                    | every ACK-bearing command          | Stale ACK from previous command would be a false positive -- always clear first. |

### MAVLink-trace via DEBUG logs

Each method above emits exactly one DEBUG line via
`Pixhawk._log_mavlink(body)`. With `debug:=true`, the line carries
TWO things plus a compact body:

* `<fn>` -- the Pixhawk method that produced the frame
  (`send_rc_override`, `set_target_depth`, `arm`, ...), captured
  automatically via `sys._getframe(1)`. Always present, even when no
  Duburi verb is on the stack (e.g. background daemons).
* ` cmd=<verb>` -- the high-level Duburi verb that caused the call,
  set by `tracing.command_scope(verb)` (a contextvar opened by every
  public method on `Duburi` / `VisionVerbs`). Absent for daemon
  threads (`Heartbeat`, `HeadingLock`) because contextvars don't
  cross thread boundaries -- the `<fn>` half still pinpoints them.
* `<body>` -- minimum useful payload only. For RC overrides we skip
  channels at neutral / released, so a "yaw correction only" tick is
  one short token instead of six. Idle states get the explicit
  shorthands `all=neutral` / `all=released`.

So a single `Duburi.set_depth(-1.5)` produces (at DEBUG):

```
[MAV set_mode cmd=set_depth]         ALT_HOLD (id=2)
[MAV set_target_depth cmd=set_depth] depth=-1.50m
```

A heading-lock streaming Ch4 in the background, simultaneously,
prints (no `cmd=` because the streamer is its own thread):

```
[MAV send_rc_override] yaw=1430
```

This is the recommended way to debug "did we *send* the right
thing?" before reaching for tcpdump on the MAVLink port. The
`cmd=<verb>` tag means a single
`rg "cmd=lock_heading" session.log` returns every frame the
high-level verb produced, across every file involved.

---

## Things we found wrong / risky during past audits

### Fixed in earlier refactors

1. **`prime_alt_hold` was sending plain `pixhawk.send_neutral()`**
   -- when a `lock_heading` is active, that 1500 us on Ch4 would
   override the lock thread's Ch4 yaw-rate command. **Fix:**
   `prime_alt_hold` now takes a `neutral_writer` callable and the
   `Duburi.set_depth` facade passes `writers.neutral` (lock-aware).
2. **Old `motion_linear.py` returned `pixhawk.send_rc_override(forward=..., lateral=...)` even when only one axis was active**
   -- that wrote 1500 to the *other* axis explicitly, blocking
   another command from reusing it concurrently. **Fix:** new
   `motion_forward.py` and `motion_lateral.py` write only their own
   axis via the `Writers` bundle.
3. **No way to combine forward thrust with yaw rate** -- meant
   every "turn" was a sharp pivot, no curved trajectories possible.
   **Fix:** `arc(...)` writes Ch5 + Ch4 in the same
   `RC_CHANNELS_OVERRIDE` packet at 20 Hz.
4. **No persistent heading hold** -- ArduSub's internal compass-
   based hold is unreliable under ESC interference. **Fix:**
   `heading_lock.py` daemon thread streams a Ch4 yaw-rate override
   continuously, source-agnostic over the `YawSource` interface.
5. **`DepthLock` daemon was redundant** -- ArduSub's onboard
   ALT_HOLD already holds depth indefinitely once the target is
   reached. **Fix:** `set_depth` now drives once via
   `SET_POSITION_TARGET_GLOBAL_INT` and lets ALT_HOLD do the rest;
   the always-warm `Heartbeat` daemon prevents `FS_PILOT_INPUT`
   from disarming us.

### Known good but worth flagging

* **`MAVLINK20=1` env var is set inside `pixhawk.py` BEFORE pymavlink imports.**
  Forgetting this silently downgrades us to MAVLink v1, which lacks
  several typemask bits AND caps `RC_CHANNELS_OVERRIDE` at 8
  channels. Any new module that imports pymavlink outside
  `pixhawk.py` needs the same env shim.
* **`master.messages` cache vs `recv_match()`.** Only the reader
  thread in `auv_manager_node.reader_loop` calls `recv_match`. Every
  other consumer reads `master.messages.get('AHRS2')` etc. from the
  cache. Mixing the two from worker threads causes pymavlink to
  drop messages -- consumers see stale data and we get apparent
  telemetry freezes mid-mission.
* **`COMMAND_ACK` collisions.** Two ACK-bearing commands fired in
  quick succession can land on the same cached ACK. We
  `clear_ack()` before every wait, but if a future module forgets,
  the first command's stale ACK will satisfy the second command's
  wait.

---

## Hidden gems we deliberately don't (yet) use

| Feature                                | Why not (yet)                                                                                                                |
| -------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| `MAV_CMD_GUIDED_CHANGE_SPEED`          | Speeds in m/s; we run open-loop in % thrust. Would require a velocity feedback source we don't have.                          |
| `SET_POSITION_TARGET_LOCAL_NED`        | Lateral/forward position setpoints in body frame. Useful if/when we have DVL or visual odometry providing position estimate.  |
| `STATUS_TEXT` severity filtering       | We log every STATUSTEXT at INFO. Severity-based routing (ERROR -> rclpy `logger.error`) would surface ArduSub failures louder. |
| `MISSION_ITEM_INT` upload + AUTO mode  | ArduSub's onboard mission system. Worth exploring once we have a stable depth + heading sensor stack -- offloads timing.     |
| `MAV_CMD_DO_REPOSITION`                | Single-shot lat/lon move in GUIDED. Same blocker as POSITION_TARGET_LOCAL_NED -- needs position estimate.                    |
| `EKF_STATUS_REPORT`                    | Useful for surfacing ArduSub-side EKF rejections (compass disagrees with gyro etc.). Currently we infer this from `STATUSTEXT`. |


---

## Message Receive Patterns

```python
# Blocking (wait indefinitely — use only at startup)
msg = master.recv_match(type='HEARTBEAT', blocking=True)

# Blocking with timeout (use in control loops)
msg = master.recv_match(type='AHRS2', blocking=True, timeout=0.1)

# Non-blocking (use for telemetry polling)
msg = master.recv_match(type='AHRS2', blocking=False)

# Multiple types
msg = master.recv_match(type=['AHRS2', 'ATTITUDE'], blocking=True, timeout=0.5)

# Latest cached message (fastest — no receive overhead)
if 'AHRS2' in master.messages:
    msg = master.messages['AHRS2']
    time_since = master.time_since('AHRS2')   # Seconds since last update

# Read and discard buffer (important: call regularly to prevent buildup)
master.recv_match()   # Non-blocking, clears one message from buffer
```

---

## System/Component IDs

```python
# Standard IDs for our setup:
# Pixhawk (ArduSub):     system=1, component=1
# Companion/GCS:         system=255, component=190 (QGC convention)
# Our Jetson node:       system=1, component=100 (onboard controller)

# After wait_heartbeat():
master.target_system    # Set automatically from received heartbeat
master.target_component # Set automatically

# To broadcast to all:
target_system=0, target_component=0
```

---

## STATUSTEXT (Sending Debug Messages to QGC/MAVProxy)

```python
# Severity levels:
MAV_SEVERITY_EMERGENCY = 0
MAV_SEVERITY_ALERT     = 1
MAV_SEVERITY_CRITICAL  = 2
MAV_SEVERITY_ERROR     = 3
MAV_SEVERITY_WARNING   = 4
MAV_SEVERITY_NOTICE    = 5
MAV_SEVERITY_INFO      = 6
MAV_SEVERITY_DEBUG     = 7

def mav_log(text: str, severity=mavutil.mavlink.MAV_SEVERITY_INFO):
    master.mav.statustext_send(severity, text[:50].encode('utf8'))
```
