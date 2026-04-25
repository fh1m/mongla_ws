# Command reference -- every verb on `/duburi/move`

> One row per verb. CLI form, Python `Duburi` facade form, mission DSL
> form, defaults, and the MAVLink message it eventually emits.
>
> Each verb's *implementation* lives in exactly one file -- when you
> need to change behaviour, edit that file. We deliberately do NOT
> consolidate the verbs into a single dispatcher so that a wrong
> motion can be traced back to one specific module without having to
> grep through layers of indirection.

## How to read this doc

Every verb is published in three shapes:

| Shape           | Where you call it                                         |
| --------------- | --------------------------------------------------------- |
| **CLI**         | `ros2 run duburi_planner duburi <cmd> [--field=value ...]` -- driver in `src/duburi_planner/duburi_planner/cli.py` |
| **Python facade** | `Duburi.<cmd>(...)` -- code in `src/duburi_control/duburi_control/duburi.py` (open-loop) and `vision_verbs.py` (vision) |
| **Mission DSL** | `duburi.<verb>(...)` inside `def run(duburi, log)` -- thin wrapper over the action client; lives in `src/duburi_planner/duburi_planner/duburi_dsl.py` |

The action surface itself is the `Move.action` defined in
`src/duburi_interfaces/action/Move.action`. The ONE place that lists
every verb's fields and defaults is
`src/duburi_control/duburi_control/commands.py` (the `COMMANDS` dict).
If a verb is in this doc and not in `COMMANDS`, that's a bug.

DEBUG-level MAVLink trace: every verb that ends in a wire-write
emits one or more `[MAV <fn>[ cmd=<verb>]] <body>` lines. Flip them
on with the manager's `debug` ROS-param (default off, production
runs stay quiet):

```bash
ros2 run duburi_manager start --ros-args -p debug:=true
```

That single param sets two things: it raises the manager logger to
DEBUG and enables the `cmd=<verb>` tag so a single
`rg "cmd=<verb>"` over the session log returns every frame the
verb produced. `<fn>` is the Pixhawk method that emitted the frame
(e.g. `send_rc_override`, `set_target_depth`); the body shows only
non-neutral channels, so a typical "yaw correction only" tick looks
like `[MAV send_rc_override cmd=lock_heading] yaw=1430` rather than
six redundant `ch=1500` tokens. The `[MAV ]` examples below assume
`debug:=true`.

Each verb's `Implements` row gives the implementation breadcrumb
(`<file>.<func>`); the same string is in the verb's docstring as
`impl: ...` so it shows up at edit time.

---

## Quick Reference

All 33 verbs at a glance. Required fields have no default listed.

| Verb | Fields (→ default) | What it does |
|---|---|---|
| `arm` | timeout→15 s | Power on thrusters |
| `disarm` | timeout→20 s | Safe shutdown |
| `set_mode` | **target_name** required, timeout→8 s | Switch ArduSub mode |
| `stop` | — | Neutral all channels (active hold) |
| `pause` | duration→2 s | Release RC override |
| `move_forward` | **duration** required, gain→80 %, settle→0 s | Open-loop forward thrust |
| `move_back` | **duration** required, gain→80 %, settle→0 s | Open-loop reverse thrust |
| `move_left` | **duration** required, gain→80 %, settle→0 s | Open-loop lateral strafe left |
| `move_right` | **duration** required, gain→80 %, settle→0 s | Open-loop lateral strafe right |
| `arc` | **duration** required, gain→50 %, yaw_rate_pct→30 %, settle→0 s | Curved motion: forward + yaw |
| `yaw_left` | **target** deg required, timeout→30 s, settle→0 s | PID pivot left |
| `yaw_right` | **target** deg required, timeout→30 s, settle→0 s | PID pivot right |
| `set_depth` | **target** m required, timeout→30 s, settle→0 s | Drive to absolute depth |
| `lock_heading` | target→0.0 °, timeout→300 s | Background heading correction loop |
| `unlock_heading` | — | Stop heading lock |
| `dvl_connect` | — | Connect Nucleus DVL (auto-connect also available) |
| `move_forward_dist` | **distance_m** required, gain→60 %, dvl_tolerance→0.1 m, settle→0 s | DVL closed-loop forward (heading lock stays active) |
| `move_lateral_dist` | **distance_m** required (±), gain→36 %, dvl_tolerance→0.1 m, settle→0 s | DVL closed-loop lateral (heading lock stays active) |
| `vision_acquire` | camera→laptop, target_class→person, target_name→'', timeout→30 s, gain→25 %, yaw_rate_pct→25 %, stale_after→1.5 s, tracking→false | Sweep until target seen |
| `vision_align_yaw` | camera→laptop, target_class→person, duration→15 s, deadband→0.18, kp_yaw→60, on_lost→fail, stale_after→1.5 s, lock_mode→'', tracking→false | Centre target horizontally (heading) |
| `vision_align_lat` | camera→laptop, target_class→person, duration→15 s, deadband→0.18, kp_lat→60, on_lost→fail, stale_after→1.5 s, lock_mode→'', tracking→false | Centre target horizontally (strafe) |
| `vision_align_depth` | camera→laptop, target_class→person, duration→15 s, deadband→0.18, kp_depth→0.05, on_lost→fail, stale_after→1.5 s, depth_anchor_frac→0, lock_mode→'', tracking→false | Centre target vertically |
| `vision_hold_distance` | camera→laptop, target_class→person, duration→20 s, deadband→0.05, kp_forward→200, target_bbox_h_frac→0.30, on_lost→fail, stale_after→1.5 s, lock_mode→'', distance_metric→'', tracking→false | Hold standoff distance |
| `vision_align_3d` | camera→laptop, target_class→person, axes→yaw,forward, duration→30 s, deadband→0.18, kp_yaw→60, kp_lat→60, kp_depth→0.05, kp_forward→200, target_bbox_h_frac→0.30, on_lost→fail, stale_after→1.5 s, depth_anchor_frac→0, lock_mode→'', distance_metric→'', tracking→false | Multi-axis simultaneous |

---

## 1. Power & mode

### `arm`

| Field | Type | Default | Accepted values | Notes |
|---|---|---|---|---|
| `timeout` | float (s) | `15.0` | `1.0 – 60.0` | Max seconds to wait for armed heartbeat |

| Aspect | Value |
|---|---|
| CLI | `duburi arm [--timeout 15.0]` |
| DSL | `duburi.arm(timeout=15.0)` |
| MAVLink | `COMMAND_LONG (MAV_CMD_COMPONENT_ARM_DISARM, p1=1)` |
| Result | `final_value` = current depth (m); `success=False` if timeout or pre-arm failure |
| Failure modes | `RC_FAIL` if pre-arm checks reject; `NOT_ARMED_AFTER_ACK` if ACK arrives but `is_armed()` stays False |
| `[MAV ]` | `[MAV arm cmd=arm] COMPONENT_ARM_DISARM p1=1` |

### `disarm`

| Field | Type | Default | Accepted values | Notes |
|---|---|---|---|---|
| `timeout` | float (s) | `20.0` | `5.0 – 60.0` | Extra time for the mode-switch + settle sequence |

| Aspect | Value |
|---|---|
| CLI | `duburi disarm [--timeout 20.0]` |
| DSL | `duburi.disarm(timeout=20.0)` |
| Sequence | `SET_MODE → MANUAL` → 3 s settle → `RC neutral` → `COMPONENT_ARM_DISARM p1=0` |
| Result | `final_value` = current depth (m) |
| `[MAV ]` | `[MAV set_mode cmd=disarm] MANUAL` → `[MAV disarm cmd=disarm] COMPONENT_ARM_DISARM p1=0` |

### `set_mode`

| Field | Type | Default | Accepted values | Notes |
|---|---|---|---|---|
| `target_name` | string | — **required** | `MANUAL`, `STABILIZE`, `ALT_HOLD`, `DEPTH_HOLD`, `POSHOLD`, `GUIDED`, `AUTO`, `SURFACE` | Case-insensitive; `DEPTH_HOLD` is an alias for `ALT_HOLD` |
| `timeout` | float (s) | `8.0` | `1.0 – 30.0` | Max seconds polling heartbeat for mode change |

| Aspect | Value |
|---|---|
| CLI | `duburi set_mode --target_name ALT_HOLD [--timeout 8.0]` |
| DSL | `duburi.set_mode('ALT_HOLD', timeout=8.0)` |
| MAVLink | `SET_MODE` (legacy, no ACK) — retries every 300 ms, polls heartbeat |
| Auto-engaged by | `set_depth` (→ ALT_HOLD), `yaw_left/right`, `lock_heading` |
| `[MAV ]` | `[MAV set_mode cmd=set_mode] ALT_HOLD (id=2)` |

---

## 2. Stop / pause

### `stop`

No parameters.

| Aspect | Value |
|---|---|
| CLI | `duburi stop` |
| DSL | `duburi.stop()` |
| MAVLink | `RC_CHANNELS_OVERRIDE` six 1500s for ~0.6 s |
| Behaviour | **Active hold** — ArduSub still sees us as pilot; heading/depth hold latches at current state |
| `[MAV ]` | `[MAV send_rc_override cmd=stop] all=neutral` |

### `pause`

| Field | Type | Default | Accepted values | Notes |
|---|---|---|---|---|
| `duration` | float (s) | `2.0` | `0.1 – 300.0` | Seconds to release RC override |

| Aspect | Value |
|---|---|
| CLI | `duburi pause [--duration 2.0]` |
| DSL | `duburi.pause(seconds=2.0)` |
| MAVLink | `RC_CHANNELS_OVERRIDE` six 65535s (no override) for `duration` s |
| Behaviour | **Release** — autopilot takes over. ALT_HOLD holds depth + heading on its own. Suspends Heartbeat. |
| `[MAV ]` | `[MAV release_rc_override cmd=pause] all=released` |

---

## 3. Translations (Ch5 forward, Ch6 lateral)

`gain` is stick percentage (0–100). `duration` is open-loop seconds.
Currents and battery state change the metres-per-second mapping every run — use vision verbs (§8) for closed-loop precision.

**Common fields for all four move_* verbs:**

| Field | Type | Default | Accepted values | Notes |
|---|---|---|---|---|
| `duration` | float (s) | — **required** | `0.1 – 300.0` | Time to thrust |
| `gain` | float (%) | `80.0` | `0.0 – 100.0` | Thrust percentage; 100 = full stick |
| `settle` | float (s) | `0.0` | `0.0 – 10.0` | Extra neutral hold after drive |

| Verb | Channel | CLI | DSL | MAVLink |
|---|---|---|---|---|
| `move_forward` | Ch5 + | `duburi move_forward --duration 5 [--gain 80]` | `duburi.move_forward(5, gain=80)` | `RC_OVERRIDE` @ 20 Hz |
| `move_back` | Ch5 − | `duburi move_back --duration 5 [--gain 80]` | `duburi.move_back(5, gain=80)` | `RC_OVERRIDE` @ 20 Hz |
| `move_left` | Ch6 − | `duburi move_left --duration 5 [--gain 80]` | `duburi.move_left(5, gain=80)` | `RC_OVERRIDE` @ 20 Hz |
| `move_right` | Ch6 + | `duburi move_right --duration 5 [--gain 80]` | `duburi.move_right(5, gain=80)` | `RC_OVERRIDE` @ 20 Hz |

* **Profile:** constant-gain (default) or eased (`smooth_translate:=true` on manager). Verb names don't change.
* **Lock-aware:** if `lock_heading` is active, Ch4 is released to 65535 so the lock keeps authority.
* **Result:** `final_value` = current depth (m); `error_value` = 0.0

---

## 4. Yaw (sharp pivots)

**Common fields for `yaw_left` / `yaw_right`:**

| Field | Type | Default | Accepted values | Notes |
|---|---|---|---|---|
| `target` | float (°) | — **required** | `0.1 – 360.0` | Degrees to turn; cumulative, not absolute |
| `timeout` | float (s) | `30.0` | `1.0 – 120.0` | Abort if not reached within this time |
| `settle` | float (s) | `0.0` | `0.0 – 10.0` | Neutral hold after yaw completes |

| Verb | CLI | DSL | Result |
|---|---|---|---|
| `yaw_left` | `duburi yaw_left --target 90 [--timeout 30]` | `duburi.yaw_left(90, timeout=30)` | `final_value` = final yaw °; `error_value` = heading error ° |
| `yaw_right` | `duburi yaw_right --target 90 [--timeout 30]` | `duburi.yaw_right(90, timeout=30)` | same |

* **Mode:** auto-engages ALT_HOLD (required for absolute yaw setpoints)
* **Profile:** `yaw_snap` (default, bang-bang) or `yaw_glide` (`smooth_yaw:=true`, smootherstep)
* **Lock-aware:** heading lock suspends during yaw, retargets to new heading on exit
* **MAVLink:** `RC_CHANNELS_OVERRIDE` Ch4 only @ 10 Hz; positive pct = right turn

---

## 5. Curved motion (arc)

### `arc`

| Field | Type | Default | Accepted values | Notes |
|---|---|---|---|---|
| `duration` | float (s) | — **required** | `0.1 – 300.0` | Open-loop seconds |
| `gain` | float (%) | `50.0` | `-100.0 – 100.0` | Forward thrust %; negative = arc in reverse |
| `yaw_rate_pct` | float (%) | `30.0` | `-100.0 – 100.0` | Yaw stick %; positive = right curve, negative = left curve |
| `settle` | float (s) | `0.0` | `0.0 – 10.0` | Neutral hold after arc |

| Aspect | Value |
|---|---|
| CLI | `duburi arc --duration 5 [--gain 50] [--yaw_rate_pct 30]` |
| DSL | `duburi.arc(5, gain=50, yaw_rate_pct=30)` |
| MAVLink | `RC_CHANNELS_OVERRIDE` Ch5 + Ch4 in one packet @ 20 Hz |
| Result | `final_value` = final yaw °; `error_value` = heading drift ° |
| `[MAV ]` | `[MAV send_rc_override cmd=arc] yaw=<pwm> fwd=<pwm>` |

---

## 6. Depth

### `set_depth`

| Field | Type | Default | Accepted values | Notes |
|---|---|---|---|---|
| `target` | float (m) | — **required** | `-50.0 – 0.0` | Absolute depth; negative = below surface (e.g. `-1.0` = 1 m down) |
| `timeout` | float (s) | `30.0` | `1.0 – 120.0` | Abort if target not reached |
| `settle` | float (s) | `0.0` | `0.0 – 10.0` | Hold still after reaching target |

| Aspect | Value |
|---|---|
| CLI | `duburi set_depth --target -1.5 [--timeout 30] [--settle 0]` |
| DSL | `duburi.set_depth(-1.5, timeout=30)` |
| MAVLink | Auto-engage `ALT_HOLD` → `SET_POSITION_TARGET_GLOBAL_INT` (alt only) @ 5 Hz while driving |
| Converges when | `\|depth - target\| < 0.05 m` for 0.5 s |
| After converge | ArduSub ALT_HOLD holds indefinitely; we stop streaming |
| Result | `final_value` = final depth (m); `error_value` = `\|target - final\|` (m) |
| `[MAV ]` | `[MAV set_target_depth cmd=set_depth] depth=-1.50m` per tick |

> **There is no `lock_depth` / `unlock_depth` verb.** ALT_HOLD inherits the current depth whenever Ch3 goes neutral. The Heartbeat daemon (5 Hz neutral) keeps the wire warm.

---

## 7. Heading lock (background)

### `lock_heading`

| Field | Type | Default | Accepted values | Notes |
|---|---|---|---|---|
| `target` | float (°) | `0.0` | `0.0 – 360.0` | Target heading; **0.0 = lock at current heading right now** |
| `timeout` | float (s) | `300.0` | `1.0 – 600.0` | Auto-unlocks after this duration |

| Aspect | Value |
|---|---|
| CLI | `duburi lock_heading [--target 0] [--timeout 300]` |
| DSL | `duburi.lock_heading(degrees=0.0, timeout=300.0)` — returns immediately (non-blocking) |
| MAVLink | Background thread streams `RC_CHANNELS_OVERRIDE` Ch4 only @ 20 Hz |
| Behaviour | `target=0` = lock current heading. Reads `yaw_source` (BNO085 / AHRS), proportional Ch4 rate command. Pauses Heartbeat (the lock IS the heartbeat). |
| `[MAV ]` | `[MAV send_rc_override] yaw=<pwm>` per tick |
| See | [`heading-lock.md`](./heading-lock.md) for state diagram and failure modes |

### `unlock_heading`

No parameters.

| Aspect | Value |
|---|---|
| CLI | `duburi unlock_heading` |
| DSL | `duburi.release_heading()` |
| MAVLink | Stops lock thread → `RC_CHANNELS_OVERRIDE` six 1500s → resumes Heartbeat |
| `[MAV ]` | `[MAV send_rc_override cmd=unlock_heading] all=neutral` |

---

## 8. DVL distance (closed-loop, Nucleus 1000)

DVL verbs use position feedback from the Nortek Nucleus 1000 to stop at an exact
distance rather than relying on open-loop timing. The DVL source must be
connected before the first distance move; auto-connect handles this automatically
when `dvl_auto_connect:=true` (the default).

> **Heading lock interaction (IMPORTANT):** heading lock stays **active** during
> DVL distance moves. The lock owns Ch4 (yaw rate) and keeps the AUV pointing on
> target; the DVL verb owns Ch5/Ch6 (forward/lateral). The two channels are
> independent so they coexist safely. Do **not** call `unlock_heading` before a
> DVL move — the AUV will weather-cock.

### `dvl_connect`

Manually trigger a DVL connection attempt. Normally auto-connect handles this,
so you only need `dvl_connect` if `dvl_auto_connect:=false` or to re-trigger
after a cable event.

No parameters.

| Aspect | Value |
|---|---|
| CLI | `duburi dvl_connect` |
| DSL | `duburi.dvl_connect()` |
| Result | `success=True` if TCP handshake to 192.168.2.201:9000 completes; `success=False` with reason otherwise |
| Auto-connect | When manager starts with `dvl_auto_connect:=true` (default), a background daemon retries every `dvl_retry_s` (5 s) until success — `dvl_connect` is not needed |
| Banner | On manager startup: `(192.168.2.201:9000 auto-connecting...)` or `DISCONNECTED` depending on `dvl_auto_connect` setting |

```bash
# Manual connect (if dvl_auto_connect:=false)
ros2 run duburi_planner duburi dvl_connect
# → [DVL  ] auto-connect succeeded (attempt 1)
```

### `move_forward_dist`

Drive forward (positive) or backward (negative) exactly `distance_m` metres using
DVL bottom-track position feedback. Heading lock stays active throughout.

| Field | Type | Default | Accepted values | Notes |
|---|---|---|---|---|
| `distance_m` | float (m) | — **required** | any non-zero float | Positive = forward; negative = backward |
| `gain` | float (%) | `60.0` | `10.0 – 100.0` | Constant thrust percentage during move |
| `dvl_tolerance` | float (m) | `0.1` | `0.01 – 1.0` | Stop when \|error\| ≤ this value |
| `settle` | float (s) | `0.0` | `0.0 – 10.0` | Neutral hold after reaching target |

| Aspect | Value |
|---|---|
| CLI | `duburi move_forward_dist --distance_m 2.0 [--gain 60] [--dvl_tolerance 0.1]` |
| DSL | `duburi.move_forward_dist(2.0, gain=60)` |
| MAVLink | `RC_CHANNELS_OVERRIDE` Ch5 @ 20 Hz constant during move; `send_neutral()` on stop |
| DVL feedback | `NucleusDVLSource.get_position()` → integrated (x_m, y_m); `reset_position()` called at start |
| Fallback | If DVL source has no `get_position()` method (e.g. `yaw_source=mavlink_ahrs`), logs warning and falls back to open-loop time estimate |
| Timeout | Generous auto-timeout: `|distance_m| / 0.05 + 10.0` seconds |
| Result | `final_value` = current depth (m); `error_value` = 0.0 |
| `[MAV ]` | `[MAV send_rc_override cmd=move_forward_dist] fwd=<pwm>` per tick |

```bash
# 2 m forward with DVL feedback
ros2 run duburi_planner duburi move_forward_dist --distance_m 2.0 --gain 60

# With heading lock active
ros2 run duburi_planner duburi lock_heading --target 0 --timeout 120 &
ros2 run duburi_planner duburi move_forward_dist --distance_m 3.0 --gain 60
ros2 run duburi_planner duburi unlock_heading

# DSL usage in a mission
duburi.lock_heading(target=0.0)
duburi.move_forward_dist(3.0, gain=60)
duburi.unlock_heading()
```

### `move_lateral_dist`

Strafe left (negative) or right (positive) exactly `distance_m` metres using DVL
lateral-velocity feedback. Heading lock stays active throughout.

| Field | Type | Default | Accepted values | Notes |
|---|---|---|---|---|
| `distance_m` | float (m) | — **required** | any non-zero float | Positive = right; negative = left |
| `gain` | float (%) | `36.0` | `10.0 – 100.0` | Constant thrust percentage during move |
| `dvl_tolerance` | float (m) | `0.1` | `0.01 – 1.0` | Stop when \|error\| ≤ this value |
| `settle` | float (s) | `0.0` | `0.0 – 10.0` | Neutral hold after reaching target |

| Aspect | Value |
|---|---|
| CLI | `duburi move_lateral_dist --distance_m 1.0 [--gain 36] [--dvl_tolerance 0.1]` |
| DSL | `duburi.move_lateral_dist(1.0, gain=36)` |
| MAVLink | `RC_CHANNELS_OVERRIDE` Ch6 @ 20 Hz constant during move; `send_neutral()` on stop |
| DVL feedback | `NucleusDVLSource.get_position()` → `y_m` component; `reset_position()` called at start |
| Fallback | Falls back to open-loop time estimate if DVL position unavailable |
| Result | `final_value` = current depth (m); `error_value` = 0.0 |
| `[MAV ]` | `[MAV send_rc_override cmd=move_lateral_dist] lat=<pwm>` per tick |

```bash
# 1 m right strafe
ros2 run duburi_planner duburi move_lateral_dist --distance_m 1.0 --gain 36

# Negative distance = left
ros2 run duburi_planner duburi move_lateral_dist --distance_m -1.0 --gain 36
```

### DVL source selection

| `yaw_source` | Heading from | Position from | Use when |
|---|---|---|---|
| `dvl` / `nucleus_dvl` | Nucleus AHRS (0xD2 packet) | Nucleus bottom-track (0xB4) | DVL is the sole IMU |
| `bno085_dvl` / `dvl_bno` | BNO085 (USB CDC) | Nucleus bottom-track (0xB4) | Want BNO's stable gyro fusion + DVL distance |
| `bno085` | BNO085 | — (no position) | No DVL; distance moves fall back to open-loop |
| `mavlink_ahrs` | Pixhawk AHRS2 | — (no position) | Bench / sim; no DVL at all |

`bno085_dvl` is the recommended pool configuration: BNO provides better gyro
fusion than the Nucleus AHRS while the DVL bottom-track handles all position.

> **BNO drift after DVL sessions**: if `yaw_source=bno085`, the BNO calibration
> offset is calculated at startup using the Pixhawk AHRS heading. If the Pixhawk
> AHRS drifts during a long DVL session, the BNO offset will be stale. Fix:
> restart the manager when switching source, or use `yaw_source=bno085_dvl`
> throughout (BNO stays warm, calibration never goes stale).

---

## 9. Vision verbs (closed-loop)

Every vision verb watches the largest detection of `target_class` in
`camera`. Defaults are the rosidl zeros so the manager's live
`vision.*` ROS-param values apply -- override per-call only when you
mean to *pin* the value for that command.

The whole vision sub-namespace is the
[`VisionVerbs`](../../src/duburi_control/duburi_control/vision_verbs.py)
mixin on `Duburi`. The DSL exposes the same verbs under
`duburi.vision.*` with friendlier kwarg names (`distance` instead of
`target_bbox_h_frac`, etc.).

Implementation chain for every vision verb (mirrored in each verb's
docstring as `impl: ...`):

| Verb                   | Impl chain                                                                                                          |
| ---------------------- | ------------------------------------------------------------------------------------------------------------------- |
| `vision_align_yaw`     | `vision_verbs._run_vision_track` -> `motion_vision.vision_track_axes(axes={'yaw'})` -> `pixhawk.send_rc_override` (Ch4) |
| `vision_align_lat`     | `vision_verbs._run_vision_track` -> `motion_vision.vision_track_axes(axes={'lat'})` -> `pixhawk.send_rc_override` (Ch6) |
| `vision_align_depth`   | `vision_verbs._run_vision_track` -> `motion_vision.vision_track_axes(axes={'depth'})` -> `pixhawk.set_target_depth`     |
| `vision_hold_distance` | `vision_verbs._run_vision_track` -> `motion_vision.vision_track_axes(axes={'forward'})` -> `pixhawk.send_rc_override` (Ch5) |
| `vision_align_3d`      | `vision_verbs._run_vision_track` -> `motion_vision.vision_track_axes` -> Ch4/Ch5/Ch6 + `set_target_depth`           |
| `vision_acquire`       | `vision_verbs.vision_acquire` -> `motion_vision.vision_acquire` + `_build_acquire_drive` closure -> `pixhawk.send_rc_override` |

### `vision_acquire`

Scan / sweep until at least one fresh detection of `target_class` arrives.

| Field | Type | Default | Accepted values | Notes |
|---|---|---|---|---|
| `camera` | string | `laptop` | any camera profile name | `laptop`, `sim_front`, `forward`, `downward` |
| `target_class` | string | `person` | any detector class label or `''` | `''` = accept any class |
| `target_name` | string | `''` | `''`, `yaw_left`, `yaw_right`, `move_forward`, `arc` | Drive maneuver to run while searching |
| `timeout` | float (s) | `30.0` | `1.0 – 300.0` | Abort if nothing seen |
| `gain` | float (%) | `25.0` | `0.0 – 100.0` | Thrust during search maneuver |
| `yaw_rate_pct` | float (%) | `25.0` | `-100.0 – 100.0` | Yaw rate for `arc` target_name |
| `stale_after` | float (s) | `1.5` | `0.1 – 10.0` | Max detection age to count as "seen" |
| `tracking` | bool | `false` | `true` / `false` | Subscribe `/tracks` (requires tracker_node) instead of `/detections` |

| Aspect | Value |
|---|---|
| CLI | `duburi vision_acquire --target_class person [--target_name yaw_right] [--timeout 30] [--gain 25] [--tracking false]` |
| DSL | `duburi.vision.scan(target='gate', sweep='right', timeout=25.0)` — or legacy `vision.find(...)` |
| Sweep modes | `''`/`still` (wait in place), `yaw_left`, `yaw_right`, `move_forward`, `arc` |
| Result | `final_value` = time-to-acquire (s); `success=False` + timeout reason on failure |

**Common fields for all vision verbs** (see full tables per verb below):

| Field | Type | Default | Accepted values | Notes |
|---|---|---|---|---|
| `camera` | string | `laptop` | any camera profile | `laptop`, `sim_front`, `forward`, `downward` |
| `target_class` | string | `person` | any detector class or `''` | `''` = largest detection regardless of class |
| `duration` | float (s) | varies | `0.5 – 300.0` | Max seconds for the control loop |
| `deadband` | float | `0.18` | `0.01 – 1.0` | Normalized error ≤ deadband counts as "centred" |
| `on_lost` | string | `fail` | `fail`, `hold` | `fail` = abort after ~2 s lost; `hold` = freeze setpoints and wait |
| `stale_after` | float (s) | `1.5` | `0.1 – 10.0` | Detection older than this is treated as lost |
| `tracking` | bool | `false` | `true`, `false` | `true` = subscribe `/tracks` (ByteTrack IDs + Kalman-smoothed); requires tracker_node running for this camera |

---

### `vision_align_yaw`

Centre target horizontally via Ch4 yaw rate (P loop on `ex`).

| Field | Type | Default | Accepted values | Notes |
|---|---|---|---|---|
| `kp_yaw` | float | `60.0` | `1.0 – 200.0` | P gain: `yaw_pct = clamp(ex × kp_yaw, ±35)` |
| `lock_mode` | string | `''` | `''`, `settle`, `follow` | `''` = use ROS param default (`settle`) |
| `tracking` | bool | `false` | `true` / `false` | Enable ByteTrack stable IDs |

| Aspect | Value |
|---|---|
| CLI | `duburi vision_align_yaw --target_class person [--duration 15] [--kp_yaw 60] [--deadband 0.18] [--lock_mode settle] [--tracking false]` |
| DSL | `duburi.vision.steer(target='gate', duration=8.0)` — or legacy `vision.yaw(...)` |
| Channel | Ch4 only |
| Result | `final_value` = composite normalized error; `error_value` = detection age (s) |

### `vision_align_lat`

Centre horizontally via Ch6 lateral strafe — doesn't change heading (P loop on `ex`).

| Field | Type | Default | Accepted values | Notes |
|---|---|---|---|---|
| `kp_lat` | float | `60.0` | `1.0 – 200.0` | P gain: `lat_pct = clamp(−ex × kp_lat, ±35)` (negated — strafe right to move target right) |
| `lock_mode` | string | `''` | `''`, `settle`, `follow` | |
| `tracking` | bool | `false` | `true` / `false` | |

| Aspect | Value |
|---|---|
| CLI | `duburi vision_align_lat --target_class person [--duration 15] [--kp_lat 60] [--tracking false]` |
| DSL | `duburi.vision.strafe(target='gate', duration=8.0)` — or legacy `vision.lateral(...)` |
| Channel | Ch6 only |

### `vision_align_depth`

Centre target vertically via incremental ALT_HOLD depth setpoint nudges.

| Field | Type | Default | Accepted values | Notes |
|---|---|---|---|---|
| `kp_depth` | float (m/unit) | `0.05` | `0.001 – 0.5` | Small! Metres of nudge per unit `ey_anchor` per 5 Hz tick |
| `depth_anchor_frac` | float | `0.0` | `0.0 – 1.0` | Which point on bbox to centre: 0=top, 0.5=centre, 1=bottom. **0.0 on wire = use ROS param (default 0.5)** |
| `lock_mode` | string | `''` | `''`, `settle`, `follow` | |
| `tracking` | bool | `false` | `true` / `false` | |

| Aspect | Value |
|---|---|
| CLI | `duburi vision_align_depth --target_class person [--duration 15] [--kp_depth 0.05] [--depth_anchor_frac 0.5] [--tracking false]` |
| DSL | `duburi.vision.level(target='flare', duration=8.0)` — or legacy `vision.depth(...)` |
| Channel | Depth setpoint (`SET_POSITION_TARGET_GLOBAL_INT`) @ 5 Hz |
| Math | `ey_anchor = ey + (2×anchor − 1)×h_frac` → `nudge = clamp(ey_anchor × kp_depth, ±0.02 m)` |
| Tip | Use `depth_anchor_frac=0.2` for tall targets (person, pole) — centering on bbox top avoids depth stall |

### `vision_hold_distance`

Drive Ch5 to match target size proxy to `target_bbox_h_frac`.

| Field | Type | Default | Accepted values | Notes |
|---|---|---|---|---|
| `kp_forward` | float | `200.0` | `10.0 – 500.0` | P gain: `fwd_pct = clamp((target_frac − size) × kp_fwd, ±50)` |
| `target_bbox_h_frac` | float | `0.30` | `0.05 – 0.95` | Stop-distance: target fills this fraction of frame (in chosen metric) |
| `distance_metric` | string | `''` | `''`, `height`, `area`, `diagonal` | `''` = ROS param default (`height`) |
| `lock_mode` | string | `''` | `''`, `settle`, `follow`, `pursue` | `pursue` = forward-only, exits when size ≥ target |
| `tracking` | bool | `false` | `true` / `false` | |

| Aspect | Value |
|---|---|
| CLI | `duburi vision_hold_distance --target_class person --target_bbox_h_frac 0.55 [--duration 20] [--kp_forward 200] [--distance_metric area] [--tracking false]` |
| DSL | `duburi.vision.approach(target='gate', distance=0.42, duration=12.0)` — or legacy `vision.forward(...)` |
| Channel | Ch5 only |

### `vision_align_3d`

Hold multiple axes simultaneously. All active axes must be within `deadband` to "settle".

**DSL preferred form — boolean flags (no CSV to mis-type):**

```python
# Steer + approach (classic gate setup)
duburi.vision.align(yaw=True, forward=True, distance=0.42)

# Full 3-axis flare lock
duburi.vision.align(yaw=True, forward=True, depth=True,
                    distance=0.38, duration=20.0,
                    on_lost='hold', distance_metric='height')

# Orbit re-lock (short follow window, no settle exit)
duburi.vision.align(yaw=True, forward=True, depth=True,
                    distance=0.38, duration=3.0,
                    on_lost='hold', lock_mode='follow')
```

`duburi.vision.lock(axes='yaw,forward', ...)` is an alias that accepts the same kwargs as CSV.

| Field | Type | Default | Accepted values | Notes |
|---|---|---|---|---|
| `axes` | string (CSV) | `yaw,forward` | any subset of `yaw`, `lat`, `depth`, `forward` | e.g. `'yaw,forward'`, `'yaw,lat,depth,forward'` |
| `kp_yaw` | float | `60.0` | `1.0 – 200.0` | |
| `kp_lat` | float | `60.0` | `1.0 – 200.0` | |
| `kp_depth` | float | `0.05` | `0.001 – 0.5` | |
| `kp_forward` | float | `200.0` | `10.0 – 500.0` | |
| `target_bbox_h_frac` | float | `0.30` | `0.05 – 0.95` | Target size (stop distance) |
| `depth_anchor_frac` | float | `0.0` | `0.0 – 1.0` | 0.0 on wire = use ROS param (default 0.5) |
| `distance_metric` | string | `''` | `''`, `height`, `area`, `diagonal` | |
| `lock_mode` | string | `''` | `''`, `settle`, `follow`, `pursue` | |
| `visual_pid` | bool | `false` | `true` / `false` | Structural placeholder; body is P-only today |
| `tracking` | bool | `false` | `true` / `false` | Enable ByteTrack + Kalman |

| Aspect | Value |
|---|---|
| CLI | `duburi vision_align_3d --target_class gate --axes yaw,forward,depth --target_bbox_h_frac 0.50 [--duration 20] [--lock_mode settle] [--distance_metric area] [--tracking false]` |
| DSL | `duburi.vision.align(yaw=True, forward=True, depth=True, distance=0.50, duration=15.0)` — or legacy `vision.lock(axes='...', ...)` |
| Loop | Single 20 Hz tick: writes Ch4+Ch5+Ch6 in one RC packet + 5 Hz depth sub-tick |
| Result | `final_value` = composite normalized error; `error_value` = detection age (s) |

### Common vision overrides

| Override             | ROS-param fallback           | Effect                                                    |
| -------------------- | ---------------------------- | --------------------------------------------------------- |
| `kp_yaw`             | `vision.kp_yaw`              | Proportional gain on `ex`, Ch4 percent units              |
| `kp_lat`             | `vision.kp_lat`              | Same on Ch6                                               |
| `kp_depth`           | `vision.kp_depth`            | Metres of nudge per unit `ey_anchor` per 5 Hz tick        |
| `kp_forward`         | `vision.kp_forward`          | Ch5 percent per unit (target_h_frac - size)               |
| `deadband`           | `vision.deadband`            | Per-axis settle band; \|err\| < deadband counts as centred |
| `target_bbox_h_frac` | `vision.target_bbox_h_frac`  | Stop-distance threshold used by `forward` / `lock`        |
| `stale_after`        | `vision.stale_after`         | Seconds after which a detection is treated as lost        |
| `on_lost`            | `vision.on_lost`             | `'fail'` (abort on lost) or `'hold'` (pause, keep waiting) |
| `depth_anchor_frac`  | `vision.depth_anchor_frac`   | Which point on the bbox to vertically centre (0=top, 0.5=centre, 1=bottom). Use **0.2** for tall objects (person standing, pole) where centering on the bbox centre stalls the depth controller. |
| `lock_mode`          | `vision.lock_mode`           | When to exit the loop — see Lock modes below              |
| `distance_metric`    | `vision.distance_metric`     | How "size" is measured from the bbox — see Distance metrics below |
| `visual_pid`         | -                            | Structural placeholder; body is P-only today. |
| `tracking`           | `vision.use_tracks`          | `true` = subscribe `/tracks` (ByteTrack IDs + Kalman-smoothed bbox). Requires `tracker_node` running for that camera. Also settable globally: `ros2 param set /duburi_manager vision.use_tracks true`. |

Loop body (the one place all of this lives):
[`src/duburi_control/duburi_control/motion_vision.py`](../../src/duburi_control/duburi_control/motion_vision.py).

### Lock modes (`lock_mode`)

Controls when a vision verb exits, beyond duration and target-lost:

| Value     | Exits on settle | Forward thrust   | Extra exit condition                                      | Best for |
| --------- | --------------- | ---------------- | --------------------------------------------------------- | -------- |
| `settle`  | ✅ yes          | bidirectional    | All active axes within `deadband` for 2 consecutive ticks | Standard one-shot alignment |
| `follow`  | ❌ no           | bidirectional    | Duration only (or target lost)                            | Tracking a moving target for a fixed time window |
| `pursue`  | ❌ no           | one-way forward only | Target `size >= target_bbox_h_frac` (fills that fraction of frame) OR timeout | Torpedo approach / contact run — vehicle keeps driving until close enough |

`lock_mode=''` resolves to the `vision.lock_mode` ROS-param (default `'settle'`).

DSL convenience: `duburi.vision.follow(...)` is shorthand for `lock(..., lock_mode='follow')`.

### Distance metrics (`distance_metric`)

How `motion_vision._distance_size(sample, metric)` measures how far away the target is:

| Value      | Formula                              | Best for |
| ---------- | ------------------------------------ | -------- |
| `height`   | `h_frac` (bbox height / image height) | Tall uniform targets — poles, buoys, vertical markers |
| `area`     | `sqrt(h_frac × w_frac)`              | Mixed-aspect targets — gates, torpedo holes, wide objects |
| `diagonal` | `sqrt(h_frac² + w_frac²) / sqrt(2)` | Best all-rounder when target shape varies or is unknown |

`distance_metric=''` resolves to the `vision.distance_metric` ROS-param (default `'height'`).

`target_bbox_h_frac` retains the same name regardless of metric — it is the threshold value in the chosen metric's units (e.g. `target_bbox_h_frac=0.30` with `distance_metric='area'` means stop when geometric-mean size equals 0.30).

### Depth anchor (`depth_anchor_frac`)

Fixes a stall problem with tall bounding boxes. The depth error is normally computed from the bbox *center*, but for a person standing upright the center is already near the image center even when the AUV is at the wrong depth — giving a near-zero error that leaves the depth controller with nothing to do.

`depth_anchor_frac` picks a different reference point on the bbox:

```
ey_anchor = ey + (2 × anchor − 1) × h_frac
```

| Anchor value | Reference point | When to use |
| ------------ | --------------- | ----------- |
| `0.5`        | Bbox centre (default, backward-compatible) | Short or compact targets |
| `0.2`        | Near top of bbox | Tall targets: person standing, pole, post |
| `0.0`        | Top edge of bbox | Extremely tall targets filling most of frame height |

`depth_anchor_frac=0.0` on the wire resolves to `vision.depth_anchor_frac` ROS-param (default 0.5).

---

## 10. Tracker node (optional, off by default)

`tracker_node` subscribes the detector's `/detections` topic, runs **ByteTrack** (two-stage association) + a **per-track 4-state Kalman smoother** (`[cx, cy, vx, vy]`), and republishes `/tracks` — same `Detection2DArray` format but with `tracking_id` populated and bbox centers smoothed.

### Start with tracking enabled

```bash
# With cameras_.launch.py
ros2 launch duburi_vision cameras_.launch.py with_tracking:=true

# Or start tracker_node standalone (detector must already be running)
ros2 run duburi_vision tracker_node --ros-args -p camera:=laptop

# Then tell the manager to use /tracks for all vision verbs
ros2 param set /duburi_manager vision.use_tracks true
# Or per-verb in CLI:
ros2 run duburi_planner duburi vision_align_yaw --target_class person --tracking true
```

### ROS params on `tracker_node` (live-tunable via `ros2 param set`)

| Param | Type | Default | Notes |
|---|---|---|---|
| `camera` | string | `laptop` | Must match the detector's camera profile |
| `track_buffer` | int (frames) | `30` | Frames to keep a lost track alive (~1.5 s @ 20 Hz) |
| `min_hits` | int (frames) | `3` | Confirmed frames before a new track is published |
| `iou_threshold` | float | `0.3` | ByteTrack low-confidence association IoU threshold |
| `enable_kalman` | bool | `true` | Apply 4-state CV Kalman smoother to cx/cy |
| `kalman_process_noise` | float | `0.1` | Kalman Q diagonal; higher = trust measurements more |
| `kalman_measurement_noise` | float | `1.0` | Kalman R; higher = trust predictions more |
| `max_predict_frames` | int | `5` | Kalman-only frames before track is dropped (~0.25 s) |

All params are stored in `src/duburi_vision/config/tracker.yaml`.

### Topics published by `tracker_node`

| Topic | Type | Notes |
|---|---|---|
| `/duburi/vision/<cam>/tracks` | `vision_msgs/Detection2DArray` | `tracking_id` = ByteTrack int ID; bbox smoothed |

### Smoke test

```bash
ros2 run duburi_vision tracker_check --camera laptop --duration 5 --require-class person
# Exit 0 = stable track seen; exit 1 = failure with actionable hint
```

Reports: `msg_hz`, unique IDs seen, ID stability %, predicted frame ratio, top classes.

### Predicted frames (occlusion bridging)

When the detector misses a frame but ByteTrack's buffer hasn't expired, `tracker_node` emits an entry with `score=0.0` and the bbox from the Kalman prediction. The manager's `stale_after` gate treats Kalman-only frames as fresh (age_s is time since last real detection) — so short occlusions bridge without triggering `on_lost`. After `max_predict_frames` consecutive Kalman frames the track is dropped.

---

## 11. Model and class selection

### Model selection

Drop `<stem>.pt` in `src/duburi_vision/models/`. **No YAML required** — the detector
reads class names from the embedded `model.names` table. Only add a `<stem>.yaml`
if you need to remap integer IDs to human names.

Current model stems (pool day):

| Stem | Classes | Use |
|---|---|---|
| `gate_flare_medium_100ep` | gate, flare | **Default** — used by `bringup.launch.py vision:=true` |
| `gate_nano_100ep` | gate | Gate-only, faster on Jetson Orin |
| `gate_medium_100ep` | gate | Gate-only, higher accuracy |
| `flare_medium_100ep` | flare | Flare-only |

Pass the stem at launch — no path, no `.pt` extension:

```bash
# Default prequal bringup (gate_flare_medium_100ep, conf=0.45, classes=gate)
ros2 launch duburi_manager bringup.launch.py vision:=true

# Explicit model override
ros2 launch duburi_vision cameras_.launch.py model:=gate_nano_100ep classes:=gate
```

### Live class switching (no restart)

The model is loaded once. `classes` is a post-inference filter:

```bash
ros2 param set /duburi_detector classes gate
ros2 param set /duburi_detector classes "gate,flare"
```

### Offline testing with `video_file`

```bash
ros2 launch duburi_vision cameras_.launch.py \
    video_file:=/tmp/pool_run.mp4 model:=gate_v1 classes:=gate
# loop:=false to stop at EOF; rqt:=false for headless; with_tracking:=true for ByteTrack
```

`video_file` is a fully supported camera source — all downstream detection
and vision verbs work identically.

---

## 12. Cross-references

* Model/class selection: [`vision-architecture.md`](./vision-architecture.md#model-and-class-selection)
* Offline video testing + three-model pattern: [`mission-cookbook.md §3.4`](./mission-cookbook.md)

* `COMMANDS` registry (single source of truth for fields/defaults): [`commands.py`](../../src/duburi_control/duburi_control/commands.py)
* Python facade (open-loop verbs): [`duburi.py`](../../src/duburi_control/duburi_control/duburi.py)
* Python facade (vision verbs): [`vision_verbs.py`](../../src/duburi_control/duburi_control/vision_verbs.py)
* Action server dispatch: [`auv_manager_node.py`](../../src/duburi_manager/duburi_manager/auv_manager_node.py)
* Mission DSL: [`duburi_dsl.py`](../../src/duburi_planner/duburi_planner/duburi_dsl.py)
* CLI driver: [`cli.py`](../../src/duburi_planner/duburi_planner/cli.py)
* Action definition: [`Move.action`](../../src/duburi_interfaces/action/Move.action)
* Mission cookbook (how to compose verbs): [`mission-cookbook.md`](./mission-cookbook.md)
* Testing every verb end-to-end: [`testing-guide.md`](./testing-guide.md)
* MAVLink shapes the verbs end up emitting: [`mavlink-reference.md`](./mavlink-reference.md)
* ArduSub modes / params / failsafes that frame this all: [`ardusub-canon.md`](./ardusub-canon.md)
