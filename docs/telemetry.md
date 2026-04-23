# Telemetry & log cheatsheet

Every log line starts with a bracketed tag. Grep for a tag to isolate a specific subsystem.

| Tag       | Meaning |
|-----------|---------|
| `[STATE]` | Periodic snapshot: arm state, mode, yaw, depth, battery voltage |
| `[ACT  ]` | Action server state transition (EXECUTING / DONE / ABORTED) |
| `[CMD  ]` | Command boundary — STOP, pause, set_depth, brake, settle, ... |
| `[RC   ]` | Active RC override PWM values (Thr, Yaw, Fwd, Lat) |
| `[DEPTH]` | Depth tracking: target, current, error (ArduSub onboard PID drives) |
| `[YAW  ]` | Yaw tracking: target, current, error |
| `[FOR  ]` | Forward translation progress |
| `[BAC  ]` | Backward translation progress |
| `[VIS  ]` | Vision loop: target class, bbox error, size proxy, lock mode |
| `[ARDUB]` | Relayed `STATUSTEXT` from ArduSub (EKF switches, arming checks, ...) |
| `[INIT ]` | One-shot init notes (startup banner, message-rate pins) |
| `[MAV  ]` | Per-frame MAVLink trace (`debug:=true` only — off by default) |

---

## `[STATE]` throttle

`[STATE]` only prints when something actually changes:
- Yaw moves > 5°
- Depth moves > 8 cm
- Battery moves > 0.2 V
- Or 30 s has passed without any change (heartbeat)

The same snapshot is published as a typed `DuburiState` message:

```bash
ros2 topic echo /duburi/state
```

Fields: `armed` (bool), `mode` (string), `yaw_deg` (float), `depth_m` (float), `battery_voltage` (float). Missing numerics are `NaN`, missing strings are `''`.

---

## `[MAV ]` debug trace

Enable with `debug:=true` at manager startup:

```bash
ros2 run duburi_manager start --ros-args -p debug:=true
```

Every MAVLink frame the current verb emits gets tagged:

```
[MAV send_rc_override cmd=move_forward] fwd=1700
[MAV set_target_depth cmd=set_depth] depth=-1.50m
[MAV send_rc_override cmd=lock_heading] yaw=1430
```

Format: `[MAV <pixhawk_method> cmd=<verb>] <non-neutral channels only>`

Grep for a single verb's frames:

```bash
# All RC frames from the last vision_align_yaw call
grep "cmd=vision_align_yaw" session.log
```

Full MAVLink message catalogue: [`.claude/context/mavlink-reference.md`](../../.claude/context/mavlink-reference.md).

---

## Useful one-liners

```bash
# Watch live state
ros2 topic echo /duburi/state

# Watch all log output from the manager
ros2 topic echo /rosout_agg | grep duburi_manager

# Depth tracking only
ros2 topic echo /rosout_agg | grep DEPTH

# Vision alignment errors
ros2 topic echo /rosout_agg | grep VIS
```
