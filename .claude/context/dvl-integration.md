# DVL Integration — Nortek Nucleus 1000

> Status: **SHIPPED** (2026-04). Distance-based motion works at pool.
> Yaw stability during DVL moves: **FIXED** (heading lock stays active).

## Hardware

| Property          | Value                               |
|-------------------|-------------------------------------|
| Device            | Nortek Nucleus 1000 AHRS/DVL        |
| IP                | 192.168.2.201 (static on AUV switch)|
| Port              | 9000 TCP                            |
| Password          | "nortek"                            |
| Packet: AHRS      | ID 0xD2 — heading/roll/pitch        |
| Packet: BottomTrack | ID 0xB4 — vx/vy/vz + validity bits |
| Startup commands  | SETCLOCKSTR, GETALL, START          |
| Output rate       | ~20 Hz typical                      |

## Packet Fields (Bottom Track, 0xB4)

| Field              | Offset | Type   | Notes                               |
|--------------------|--------|--------|-------------------------------------|
| `velocity_x`       | 96     | float32| m/s body-forward (+ = forward)      |
| `velocity_y`       | 100    | float32| m/s body-lateral (+ = right)        |
| `velocity_z`       | 104    | float32| m/s body-vertical                   |
| Validity bits      | 36     | uint32 | bits 0-14; check beam1/beam2 FOM + vx/vy |

Validity guard before integrating:
```python
pkt['beam1_fom_valid'] and pkt['beam2_fom_valid']
    and pkt['x_velocity_valid'] and pkt['y_velocity_valid']
```

## Source Files

| File                                  | Role                               |
|---------------------------------------|------------------------------------|
| `sources/nucleus_dvl.py`              | TCP reader + integrator + YawSource|
| `sources/nucleus_parser.py`           | Packet decoder (no TCP dependency) |
| `sources/composite_bno_dvl.py`        | BNO085 heading + DVL position      |
| `factory.py`                          | Registered as `dvl`, `nucleus_dvl`, `bno085_dvl`, `dvl_bno` |

## Yaw Source Options (DVL-related)

```
yaw_source=dvl          → NucleusDVLSource (heading from DVL AHRS + position from DVL)
yaw_source=bno085_dvl   → CompositeBnoDvlSource (heading from BNO085 + position from DVL)
```

`bno085_dvl` is recommended when BNO heading is preferred (more stable gyro fusion)
while still needing DVL position for distance commands.

## Auto-Connect

DVL sources start DISCONNECTED. Auto-connect behaviour:

- `dvl_auto_connect: true` (default) → background thread retries every `dvl_retry_s` (5s) until successful
- `dvl_auto_connect: false` → manual: `ros2 run duburi_planner duburi dvl_connect`
- `dvl_connect` verb: always available as manual override regardless of auto_connect setting

Banner at startup shows: `(192.168.2.201:9000 auto-connecting...)` or `DISCONNECTED`.

## DVL Distance Commands

```bash
# Forward N metres (closed-loop, DVL feedback):
ros2 run duburi_planner duburi move_forward_dist --distance_m 2.0 --gain 60

# Lateral N metres:
ros2 run duburi_planner duburi move_lateral_dist --distance_m 1.0 --gain 40
```

Falls back to open-loop timed estimate if DVL position source unavailable (logs warning).

## Heading Lock + DVL Interaction (IMPORTANT)

**Heading lock STAYS ACTIVE during DVL distance moves.** This is intentional:
- Lock owns Ch4 (yaw rate) → maintains heading during translation
- DVL drives Ch5/Ch6 (forward/lateral) → position closed-loop
- Writers handle the split: `release_yaw=True` when lock is live

Prior to fix (2026-04-24): `_drive_forward_dist` wrapped in `_suspend_heading_lock()`,
which paused the lock during DVL moves and let the AUV weather-cock.

DSL usage with lock:
```python
duburi.lock_heading(target=0.0)     # lock heading
duburi.move_forward_dist(3.0)       # DVL move — lock stays active
duburi.unlock_heading()
```

## Position Integration

`NucleusDVLSource` integrates velocity in body frame:
```python
dt = now - last_bt_time          # seconds since last packet
if 0 < dt < 0.5:                 # guard for stale packet on reconnect
    pos_x += vx * dt
    pos_y += vy * dt
```

`reset_position()` must be called before each DVL distance command (done automatically
by `drive_forward_dist` and `drive_lateral_dist`).

Tolerance: `dvl_tolerance=0.1` m (stop when `|error| ≤ 0.1 m`).

## BNO Drift After DVL Sessions

**Symptom**: After using DVL-sourced heading, switching to BNO shows drift.  
**Cause**: BNO recalibration offset uses Pixhawk AHRS as reference at startup.
If Pixhawk AHRS has drifted (e.g. after a long DVL session), the BNO offset
will be incorrect.  
**Fix**: Always restart the manager when switching yaw_source. The BNO calibration
runs fresh at startup and reads the Pixhawk heading at that moment.  
**Alternative**: Use `yaw_source=bno085_dvl` which keeps BNO warm throughout.

## Yaw PID for Turns (2026-04-24)

Yaw turns now use a full **PID** controller (was P-only):
- **Kp=1.2** (%/deg): proportional
- **Ki=0.03** (%/deg·sample): integral to overcome T200 dead zone near target
- **Kd=0.5** (%/deg change): derivative to dampen overshoot on large turns
- **Anti-windup**: Ki accumulator clamped to ±8.0%
- **I reset**: when error crosses zero (overshoot), I accumulator resets

The `_YawPID` class in `motion_yaw.py` carries state per turn. `heading_lock.py`
still uses the existing P controller (suitable for small heading corrections).

## Pool-Day Startup (One Command)

```bash
# Control + gate vision (DVL auto-connects in background):
ros2 launch duburi_manager bringup.launch.py vision:=true

# Run gate mission (DVL connects automatically, no manual step needed):
ros2 run duburi_planner mission gate_prequal
```

## Smoke Tests

```bash
# 1. DVL connect check
ros2 run duburi_planner duburi dvl_connect
# Expect: [DVL  ] auto-connect succeeded (attempt 1)

# 2. DVL position reset + 2m forward
ros2 run duburi_planner duburi arm
ros2 run duburi_planner duburi set_mode --target_name ALT_HOLD
ros2 run duburi_planner duburi set_depth --target -0.8
ros2 run duburi_planner duburi move_forward_dist --distance_m 2.0 --gain 60
ros2 run duburi_planner duburi disarm

# 3. Heading lock + DVL combo
ros2 run duburi_planner duburi arm
ros2 run duburi_planner duburi set_mode --target_name ALT_HOLD
ros2 run duburi_planner duburi set_depth --target -0.8
ros2 run duburi_planner duburi lock_heading --target 0 --timeout 120 &
ros2 run duburi_planner duburi move_forward_dist --distance_m 3.0 --gain 60
ros2 run duburi_planner duburi disarm
```
