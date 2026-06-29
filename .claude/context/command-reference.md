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

All verbs at a glance (canonical list: `COMMANDS` registry in `duburi_control/commands.py`). Required fields have no default listed.

| Verb | Fields (→ default) | What it does |
|---|---|---|
| `arm` | timeout→15 s | Power on thrusters |
| `disarm` | timeout→20 s | Safe shutdown |
| `set_mode` | **target_name** required, timeout→8 s | Switch ArduSub mode |
| `stop` | — | Neutral all channels (active hold) |
| `surface` | — | **Safety**: ascend to 0 m. Bypasses command_active gate (works during a running mission) |
| `pause` | duration→2 s | Release RC override |
| `move_forward` | **duration** required, gain→80 %, settle→0 s | Open-loop forward thrust |
| `move_back` | **duration** required, gain→80 %, settle→0 s | Open-loop reverse thrust |
| `move_left` | **duration** required, gain→80 %, settle→0 s | Open-loop lateral strafe left |
| `move_right` | **duration** required, gain→80 %, settle→0 s | Open-loop lateral strafe right |
| `arc` | **duration** required, gain→50 %, yaw_rate_pct→30 %, settle→0 s | Curved motion: forward + yaw |
| `yaw_left` | **target** deg required, timeout→30 s, settle→0 s | PID pivot left (relative, degrees) |
| `yaw_right` | **target** deg required, timeout→30 s, settle→0 s | PID pivot right (relative, degrees) |
| `turn` | **target** deg required, timeout→30 s, settle→0 s | Absolute heading, direction auto-selected |
| `set_depth` | **target** m required, timeout→30 s, settle→0 s | Drive to absolute depth |
| `lock_heading` | target→0.0 °, timeout→300 s | Background heading correction loop |
| `unlock_heading` | — | Stop heading lock |
| `mission_reset` | — | Stop heading lock + clear abort event + RC neutral. **Call at start of every `run()`.** Safe before arm (`_UNARM_SAFE`). |
| `dvl_connect` | — | Connect Nucleus DVL (auto-connect also available) |
| `move_forward_dist` | **distance_m** required, gain→60 %, dvl_tolerance→0.1 m, settle→0 s | DVL closed-loop forward (heading lock stays active) |
| `move_back_dist` | **distance_m** required, gain→60 %, dvl_tolerance→0.1 m, settle→0 s | DVL closed-loop backward (same as move_forward_dist with reversed direction) |
| `move_lateral_dist` | **distance_m** required (±), gain→36 %, dvl_tolerance→0.1 m, settle→0 s | DVL closed-loop lateral (heading lock stays active) |
| `vision_align` | camera→forward, target_class→'', axes→'' (≥1 of lat,yaw,depth), offset_lat/yaw/depth→0 px, err_px→40, duration→20 s, gain→30 %, gain_lat/yaw/depth→0 (inherit gain), brake_off→false (brake on), brake_gain→0 (default), hold_s→0 s, hold_through_loss→false, fire_channels→'' (none), fire_t→0 s, kp_lat→60, kp_yaw→60, kp_depth→0.05, lost_grace_s→1.0, align_stable_frames→3 | Centre target on lat/yaw/depth at signed pixel offsets; per-axis gain caps; lateral arrival brake; optional mid-hold payload fire (`fire_channels`/`fire_t`) |
| `vision_move` | camera→forward, target_class→'', fwd_fill→95 %, mode→area, maintain_px→0/maintain_on→false, hold_s→0 s, err_px→40, duration→20 s, gain→30 %, hold_through_loss→false, kp_forward→200, kp_lat→60, lost_grace_s→1.0 | Drive forward until target's bbox fills fwd_fill% of frame |
| `fire` | fire_channel→1.0 (1/2=torpedo, 3/4=dropper) | Fire ESP32 payload channel directly |

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
Currents and battery state change the metres-per-second mapping every run — use DVL distance (§8) or vision verbs (§9) for closed-loop precision.

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
| `turn` | `duburi turn --target 90 [--timeout 30]` | `duburi.turn(90, timeout=30)` | same; direction auto-selected |

`yaw_left` / `yaw_right` take **relative** degrees from current heading. `turn` takes an **absolute** heading (0-360); shortest-path direction is computed automatically via `Pixhawk.heading_error`.

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

### `move_back_dist`

Drive backward exactly `distance_m` metres using DVL position feedback. Identical to
`move_forward_dist` but always in reverse — no negative distance needed.

| Aspect | Value |
|---|---|
| CLI | `duburi move_back_dist --distance_m 1.0 [--gain 60] [--dvl_tolerance 0.1]` |
| DSL | `duburi.move_back_dist(1.0, gain=60)` |
| Implementation | Calls `drive_forward_dist(signed_dir=-1, ...)` — same code path as move_forward_dist |

```bash
# Back off 1 m after approaching gate
ros2 run duburi_planner duburi move_back_dist --distance_m 1.0 --gain 60

# DSL — return to start after passing gate
duburi.move_forward_dist(3.5, gain=60)   # pass gate
duburi.move_back_dist(3.5, gain=60)      # return
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

Exactly **two** vision verbs: `vision_align` and `vision_move`. Both are
P-control on **pixel error** read straight off the detector; `gain` is a
hard max-speed cap (% thrust), never a target speed. There are no
frame-age knobs, no `on_lost` policies and no lock modes -- search and
recovery are the mission's job via a Python `fallback` function (below).

The control loop ALWAYS reads `/detections` (the same topic the HUD
shows). There is **no `--tracking` flag** and no `/tracks` control path:
the tracker (§10) only feeds the HUD.

The whole vision sub-namespace is the
[`VisionVerbs`](../../src/duburi_control/duburi_control/vision_verbs.py)
mixin on `Duburi`. The DSL exposes the same two verbs under
`duburi.vision.align(...)` / `duburi.vision.move(...)`
([`vision_dsl.py`](../../src/duburi_planner/duburi_planner/vision_dsl.py)),
which add the client-side `fallback` orchestration and return a
`VisionResult`.

### Never-fail contract

Neither verb ever raises or aborts a mission. The action server ALWAYS
returns `success=True`; the real outcome rides in `Move.Result.final_value`
as an integer code, and `error_value` carries the residual error.

| Code | `final_value` | Meaning |
|---|---|---|
| `ALIGNED`   | `0` | align: every active axis centred · move: bbox reached `fwd_fill` |
| `LOST`      | `1` | target gone past `lost_grace_s` (DSL then runs the `fallback`) |
| `TIMEOUT`   | `2` | `duration` elapsed without success |
| `NO_CAMERA` | `3` | camera pipeline never published `CameraInfo` (not up) |
| `ABORTED`   | `4` | cooperative abort (goal cancelled) |

The DSL wraps that code in a `VisionResult(ok, reason, code, last_err_px, fill,
x_px, y_px, saw_target, elapsed_s)` -- the **finish-state** a hybrid
vision+control mission branches on. Full contract, recovery patterns, and pitfalls:
[`vision-results.md`](vision-results.md).

* `ok` / truthiness -- **True only on `ALIGNED`** (so `if duburi.vision.align(...):` works).
* `status` / `reason` -- `'ALIGNED'`, `'LOST'`, `'TIMEOUT'`, `'NO_CAMERA'`, `'ABORTED'`, or `'FAILED'`.
* `x_px` / `y_px` -- **signed** px of the target from frame **centre** at the last seen
  frame (`+x`=ended right, `+y`=ended below); **`NaN` when never seen**. The raw
  observable -- offset-blind. Recovery sign matches `align`: `x_px>0` → `move_right`.
* `saw_target` -- bool; was the target detected at least once. **Check this before
  reading `x_px`** (`NaN<threshold` is silently False).
* `last_err_px` -- worst residual px from the **goal** (centre+offset) at exit.
* `fill` -- bbox fill fraction at exit (move; `0` for align).
* `elapsed_s` -- verb duration.

`'FAILED'` (with `code=TIMEOUT`) is a DSL-only outcome for a server/setup
error -- bad camera name, `ALT_HOLD` rejected, disarmed, server abort.
Still non-fatal: the mission moves on to its next step.

**Live feedback:** during the verb, `Move.Feedback.err_x_px`/`err_y_px` carry the same
signed px live at ~2.5 Hz (`ros2 topic echo /duburi/move/_action/feedback`; `NaN` when
no vision verb active).

### Search / recovery via `fallback`

`fallback` is a mission-authored search function -- `fn(duburi)` or
`fn(duburi, should_stop)`. It runs on a **real** target loss (after the
server coasts `lost_grace_s`), then the verb **re-enters** the vision
loop -- all inside the original `duration` budget owned by the DSL. The
two-arg form is handed a `should_stop()` that returns `True` the moment
the target reappears, so a longer sweep can bail early. This replaces the
old `vision_acquire` / `look_around` search verbs.

When **no** `fallback` is supplied the DSL sets `hold_through_loss=true`
on the wire, so the server coasts through blackouts (drives neutral and
keeps waiting) and the step rides out turbidity until it aligns or
`duration` elapses -- it never returns `LOST`.

### Sign rules (forward camera, image y grows downward)

From [`motion_vision.py`](../../src/duburi_control/duburi_control/motion_vision.py):

| Quantity | Rule |
|---|---|
| `ex > 0` | target is RIGHT of centre |
| `ey > 0` | target is BELOW centre |
| yaw | Ch4 `> 1500` = yaw RIGHT (current hull), so target-right needs Ch4 `> 1500` → **no negate** (same polarity as lat; pool-verified 2026-06) |
| lat | Ch6 `> 1500` = strafe RIGHT, so target-right needs Ch6 `> 1500` → **no negate** |
| depth | target below → descend → depth setpoint more negative |

A positive `lat`/`yaw` offset keeps the target that many px to the RIGHT
of centre; a positive `depth` offset keeps it that many px BELOW centre.
`0` = dead centre.

### Implementation chain

Each verb's `impl: ...` breadcrumb is mirrored in its docstring:

| Verb | Impl chain |
|---|---|
| `vision_align` | `vision_verbs.vision_align` → `motion_vision.align_loop` → `pixhawk.send_rc_override` (Ch6 lat / Ch4 yaw) + `set_target_depth` (depth axis) |
| `vision_move`  | `vision_verbs.vision_move` → `motion_vision.move_loop` → `pixhawk.send_rc_override` (Ch5 fwd, + Ch6 when `maintain`) |

`target` may be a class string OR a `duburi.models.<alias>.<class>`
`ClassRef` -- the DSL switches the detector model + class filter
(`set_model` + `set_classes`) before the loop runs. `camera` defaults to
`forward` (the sticky `duburi.camera` in the DSL).

---

### `vision_align`

Centre `target_class` on the active axes -- any subset of `lat`, `yaw`,
`depth` -- each held at its signed pixel offset. `lat` + `yaw` are
horizontal (strafe / rotate), `depth` is vertical. At least one axis is
required. Aligned when **every** active axis stays within `err_px` for
`align_stable_frames` consecutive ticks.

| Field | Type | Default | Accepted values | Notes |
|---|---|---|---|---|
| `camera` | string | `forward` | any camera profile | `forward`, `downward`, `sim_front`, ... |
| `target_class` | string | `''` | detector class label | `''` → sticky `duburi.target` (DSL) |
| `axes` | string (CSV) | `''` | subset of `lat,yaw,depth` | which axes to control; **≥1 required** (empty axes = no-op, returns `TIMEOUT`) |
| `offset_lat` | float (px) | `0.0` | signed px | keep target this many px RIGHT of centre (− = left); used only if `lat` in `axes` |
| `offset_yaw` | float (px) | `0.0` | signed px | same, via heading; used only if `yaw` in `axes` |
| `offset_depth` | float (px) | `0.0` | signed px | keep target this many px BELOW centre (− = above); used only if `depth` in `axes` |
| `err_px` | float (px) | `40.0` | `> 0` | per-axis pixel tolerance; "in band" when `\|err\| ≤ err_px` |
| `duration` | float (s) | `20.0` | `> 0` | total time budget (DSL-owned; spans fallback cycles) |
| `gain` | float (%) | `30.0` | `0 – 100` | **hard max-speed cap** on each axis, not a target speed |
| `hold_through_loss` | bool | `false` | `true`/`false` | coast through target loss instead of returning `LOST`. The DSL sets this `true` automatically when no `fallback` is given |
| `kp_lat` | float | `60.0` | `> 0` | P gain on Ch6 lateral (live: `vision.kp_lat`) |
| `kp_yaw` | float | `60.0` | `> 0` | P gain on Ch4 yaw (live: `vision.kp_yaw`) |
| `kp_depth` | float (m/unit) | `0.05` | `> 0` | metres of depth nudge per unit normalized error per 5 Hz tick (live: `vision.kp_depth`) |
| `lost_grace_s` | float (s) | `1.0` | `≥ 0` | seconds the server coasts on loss before reporting `LOST` (live: `vision.lost_grace_s`) |
| `align_stable_frames` | float | `3.0` | `≥ 1` | consecutive in-band ticks (~20 Hz) before `ALIGNED` (~0.15 s) (live: `vision.align_stable_frames`) |

| Aspect | Value |
|---|---|
| CLI | `duburi vision_align --camera forward --target_class gate --axes yaw,lat [--offset_yaw 0] [--offset_lat 0] [--err_px 40] [--duration 20] [--gain 30] [--gain_yaw 10] [--gain_lat 0] [--gain_depth 0] [--brake_off] [--hold_s 4] [--fire_channels 1,2] [--fire_t 1]` |
| DSL | `duburi.vision.align('gate', yaw=0, lat=0, err=40, duration=20, gain=30, lat_gain=None, yaw_gain=None, depth_gain=None, brake=True, brake_gain=None, hold=None, fire=None, fire_t=None, fallback=None, camera=None)` |
| Facade | `Duburi.vision_align(camera, target_class, axes, offset_lat=, offset_yaw=, offset_depth=, err_px=, duration=, gain=, gain_lat=, gain_yaw=, gain_depth=, brake_off=, brake_gain=, hold_s=, hold_through_loss=, fire_channels=, fire_t=, kp_lat=, kp_yaw=, kp_depth=, lost_grace_s=, align_stable_frames=)` |
| Mid-hold fire | `fire=` (int or list, 1/2=torpedo 3/4=dropper) + `fire_t=` (s into the `hold` window; 0=at lock) fire the payload **while the loop is still correcting** — gated on alignment (no off-target shot), non-blocking (background thread), requires `fire_t < hold` (else clamped to 0). Wire fields `fire_channels` (CSV) + `fire_t`. Pair with `brake=False`. Full detail: [`vision-results.md`](vision-results.md) §4. |
| Per-axis gain | `lat_gain`/`yaw_gain`/`depth_gain` cap one axis; **unset/0 = inherit `gain`, NOT disable** (omit `lat`/`yaw`/`depth` to drop an axis). The yaw spin-up floor only engages close-up (large bbox ≥ `VISION_YAW_FLOOR_FILL`); far-field yaw is pure-proportional so it cannot limit-cycle/wobble. |
| Arrival brake | `brake=True` (default) reverse-kicks the **lateral** axis on arrival to bleed water inertia so the hull stops square (yaw/depth never brake). Self-gating on the exit-velocity EMA: a gently-converged lock usually exits ~0 and is not kicked, but a fast snap-in can cross the gate → **pass `brake=False` on the torpedo fire path**. `brake=False` coasts; `brake_gain` scales it. Wire field is `brake_off` (default false = brake on). |
| MAVLink | `RC_CHANNELS_OVERRIDE` Ch6 (lat) + Ch4 (yaw) @ 20 Hz; `SET_POSITION_TARGET_GLOBAL_INT` (alt) @ 5 Hz when `depth` is active |
| Mode | auto-engages `ALT_HOLD` when `depth` is an axis; a downward camera (`downward`/`sim_bottom`) inverts the depth sign automatically |
| Heading lock | when `yaw` is an axis the loop owns Ch4 (lock suspended, retargeted on exit); when `yaw` is NOT an axis and a lock is live, the lock keeps Ch4 and the loop writes lateral only (no fight) |
| Result | `final_value` = outcome code (0–4); `error_value` = worst per-axis residual px. Also: `end_x_px`/`end_y_px` (signed target-from-centre px, NaN if never seen), `elapsed_s` — surfaced on the DSL `VisionResult` as `x_px`/`y_px`/`saw_target`/`elapsed_s` for hybrid recovery ([`vision-results.md`](vision-results.md)). |
| `[MAV ]` | `[MAV send_rc_override cmd=vision_align] yaw=<pwm> lat=<pwm>` |

> **DSL axis idiom:** in `duburi.vision.align(...)` each of `lat=` / `yaw=`
> / `depth=` is `None` = axis OFF, or a **number** = axis ON where the
> number is the signed pixel offset (`0` = centre). So `align('gate',
> yaw=0, lat=0)` turns on yaw+lat at centre; `align('bin', lat=-120,
> depth=0)` holds the bin 120 px left while centring depth. On the wire
> this becomes `axes='yaw,lat'` + the matching `offset_*`.

### `vision_move`

Drive forward until `target_class` fills `fwd_fill` % of the frame
(measured by `mode`). Optionally hold a lateral pixel offset while
driving (`maintain`). **Never re-centres yaw or depth** -- depth is left
to ArduSub's ALT_HOLD, yaw to the heading lock or the autopilot.

| Field | Type | Default | Accepted values | Notes |
|---|---|---|---|---|
| `camera` | string | `forward` | any camera profile | |
| `target_class` | string | `''` | detector class label | `''` → sticky `duburi.target` (DSL) |
| `fwd_fill` | float (%) | `95.0` | `0 – 100` | stop when bbox fills this % of frame in `mode` units (live: `vision.frame_fill_default`) |
| `mode` | string | `area` | `area`, `width`, `height` | fill metric -- see Fill modes below; `height` for tall slalom |
| `maintain_px` | float (px) | `0.0` | signed px | lateral offset to hold while driving (only when `maintain_on`) |
| `maintain_on` | bool | `false` | `true`/`false` | enable lateral hold; `false` = pure forward, never touches lat/yaw/depth |
| `hold_s` | float (s) | `0.0` | `≥ 0` | station-keep this long after reaching fill; `0` = exit on reach |
| `err_px` | float (px) | `40.0` | `> 0` | pixel tolerance for the `maintain` offset |
| `duration` | float (s) | `20.0` | `> 0` | total time budget (DSL-owned) |
| `gain` | float (%) | `30.0` | `0 – 100` | **hard max-speed cap** (forward and lateral), not a target speed |
| `hold_through_loss` | bool | `false` | `true`/`false` | coast through loss; DSL sets `true` when no `fallback` |
| `kp_forward` | float | `200.0` | `> 0` | P gain on Ch5 forward (live: `vision.kp_forward`) |
| `kp_lat` | float | `60.0` | `> 0` | P gain on Ch6 lateral for `maintain` (live: `vision.kp_lat`) |
| `lost_grace_s` | float (s) | `1.0` | `≥ 0` | coast before `LOST` (live: `vision.lost_grace_s`) |

| Aspect | Value |
|---|---|
| CLI | `duburi vision_move --camera forward --target_class gate --fwd_fill 80 --mode area [--duration 20] [--gain 30] [--gain_lat 0] [--brake_off]` |
| DSL | `duburi.vision.move('gate', fwd=80, mode='area', maintain=None, hold=None, err=40, duration=20, gain=30, lat_gain=None, brake=True, brake_gain=None, fallback=None, camera=None)` |
| Facade | `Duburi.vision_move(camera, target_class, fwd_fill=, mode=, maintain_px=, maintain_on=, hold_s=, err_px=, duration=, gain=, gain_lat=, brake_off=, brake_gain=, hold_through_loss=, kp_forward=, kp_lat=, lost_grace_s=)` |
| Per-axis gain | `gain` caps forward speed; `lat_gain` caps the `maintain` strafe (unset/0 = inherit `gain`). |
| Arrival brake | `brake=True` (default) reverse-kicks **forward (+ `maintain`)** on a **fill-stop** arrival so the hull halts in front instead of creeping in. **PASS-THROUGH (`fwd=None`) never brakes** — it must coast through the gate. Timeout/loss exits don't brake. `brake=False` coasts. |
| MAVLink | `RC_CHANNELS_OVERRIDE` Ch5 forward @ 20 Hz (+ Ch6 when `maintain` set); yaw + depth never commanded |
| Mode | auto-engages `ALT_HOLD` so depth holds during the approach even if the mission jumps straight to `vision_move` |
| Result | `final_value` = outcome code (0–4); `error_value` = bbox fill fraction at exit |
| `[MAV ]` | `[MAV send_rc_override cmd=vision_move] fwd=<pwm>` |

> **DSL idiom:** `fwd` (DSL) → `fwd_fill` (wire). `maintain=±px` sets
> `maintain_px` + `maintain_on`; `maintain=None` = pure forward.
> `hold=s` → `hold_s`; `hold=None` = exit the moment fill is reached.
> `mode='height'` suits tall slalom pipes; `mode='width'` wide bars.

### Vision ROS-param overrides

Gains and grace live as `vision.*` ROS params on `/duburi_manager`, so a
deck operator can tune them with `ros2 param set` and the **next** vision
goal picks up the value -- changes never land mid-loop. Per-call wire
fields (in the tables above) still win when explicitly set; otherwise
these apply.

| Field | ROS param | Default | Effect |
|---|---|---|---|
| `kp_lat` | `vision.kp_lat` | `60.0` | P gain on Ch6 lateral (% thrust per unit normalized px error) |
| `kp_yaw` | `vision.kp_yaw` | `60.0` | P gain on Ch4 yaw |
| `kp_depth` | `vision.kp_depth` | `0.05` | metres of depth nudge per unit `ey` per 5 Hz tick |
| `kp_forward` | `vision.kp_forward` | `200.0` | P gain on Ch5 forward (`vision_move`) |
| `lost_grace_s` | `vision.lost_grace_s` | `1.0` | seconds the server coasts on target loss before reporting `LOST` |
| `fwd_fill` | `vision.frame_fill_default` | `95.0` | `vision_move` fill target when the mission leaves `fwd_fill` at 0 |
| `align_stable_frames` | `vision.align_stable_frames` | `3.0` | in-band ticks before `vision_align` reports `ALIGNED` (~0.15 s @ 20 Hz) |

Defaults live in
[`vision_tunables.py`](../../src/duburi_manager/duburi_manager/vision_tunables.py)
(a Python module, **not** a `.yaml`). Example live tune:

```bash
ros2 param set /duburi_manager vision.kp_yaw 80.0
ros2 param set /duburi_manager vision.lost_grace_s 1.5
```

The control engine (both loops) lives in one file:
[`motion_vision.py`](../../src/duburi_control/duburi_control/motion_vision.py).

### Fill modes (`mode`)

How `vision_move` measures bbox "fill" (`motion_vision._fill`):

| Value | Formula | Best for |
|---|---|---|
| `area` (default) | `sqrt(w_frac × h_frac)` | mixed-aspect targets -- gates, bins, torpedo holes |
| `width` | `w_frac` (bbox width / image width) | wide targets -- torpedo bar, horizontal pipe |
| `height` | `h_frac` (bbox height / image height) | tall targets -- slalom pipes, poles, flare |

`fwd_fill` is a percentage of that metric (e.g. `fwd_fill=80, mode=area`
means stop when `sqrt(w × h) = 0.80`).

### Renamed / removed (2026-06 two-verb rewrite)

The old nine-verb vision API collapsed into `vision_align` + `vision_move`:

| Old | New |
|---|---|
| `vision_align_yaw` / `vision_align_lat` / `vision_align_depth` / `vision_align_3d` | `vision_align` (choose axes via `axes` / DSL `lat=`,`yaw=`,`depth=`) |
| `vision_hold_distance` / `vis_approach` | `vision_move` (drive to a bbox fill ratio) |
| `vision_acquire` / `look_around` | removed -- pass a mission-authored `fallback` search fn to `align`/`move` |
| `vision_lock_fire` | removed -- `vision_align` then `duburi.fire(channel)` (see §11) |
| DSL `duburi.vision.find` / `.home` / `.turn` / `.slide` / `.hover` / `.approach` / `.track` / `.scan` / `.hold` | `duburi.vision.align` / `duburi.vision.move` |

Removed ROS params: `vision.deadband`, `vision.lock_mode`,
`vision.depth_anchor_frac`, `vision.distance_metric`,
`vision.target_bbox_h_frac`, `vision.stable_lock_s`,
`vision.h_frac_close`, `vision.proximity_min_scale`, `vision.speed`,
`vision.use_tracks`.

Removed per-verb flag `--tracking` (control always reads `/detections`)
and the per-call kwargs `on_lost=`, `gate_guard=`, `pass_at=`, `dist=`,
`metric=`, `speed=`, `settle=`, `dwell=`, `move=`, plus per-call `kp_*=`
overrides (gains now live only as `vision.*` ROS params). `stale_after=`
survives only on `duburi.detected(target, stale_after=1.0)`.

---

## 10. Tracker node (optional, off by default)

`tracker_node` subscribes the detector's `/detections` topic, runs **ByteTrack** (two-stage association) + a **per-track 4-state Kalman smoother** (`[cx, cy, vx, vy]`), and republishes `/tracks` — same `Detection2DArray` format but with `tracking_id` populated and bbox centers smoothed.

**`/tracks` feeds the mission-control HUD (`vision_display`) only** — the
vision verbs always steer on `/detections`, so the tracker is never in
the control path. There is no `--tracking` flag and no `vision.use_tracks`
param.

### Start the tracker (HUD overlay)

```bash
# With vision.launch.py (tracking on by default)
ros2 launch duburi_vision vision.launch.py camera:=forward tracking:=true

# Or start tracker_node standalone (detector must already be running)
ros2 run duburi_vision tracker_node --ros-args -p camera:=laptop
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

When the detector misses a frame but ByteTrack's buffer hasn't expired, `tracker_node` emits an entry with `score=0.0` and the bbox from the Kalman prediction, so the HUD's `/tracks` overlay glides through short occlusions instead of flickering. After `max_predict_frames` consecutive Kalman frames the track is dropped. (The vision control loops read `/detections`, not `/tracks`; their own loss handling is `lost_grace_s` — see §9.)

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
ros2 launch duburi_vision vision.launch.py camera:=forward model:=gate_nano_100ep classes:=gate
```

### Live class switching (no restart)

The model is loaded once. `classes` is a post-inference filter (node =
`/duburi_detector_<camera>`):

```bash
ros2 param set /duburi_detector_forward classes gate
ros2 param set /duburi_detector_forward classes "gate,flare"
```

### Offline testing with `video_file`

```bash
ros2 launch duburi_vision vision.launch.py camera:=forward \
    video_file:=/tmp/pool_run.mp4 model:=gate_v1 classes:=gate
# loop:=false to stop at EOF; viewer:=false for headless; tracking:=false to skip ByteTrack
```

`video_file` is a fully supported camera source — all downstream detection
and vision verbs work identically.

---

---

## 11. Payload verbs

> **There is no `vision_lock_fire` verb.** Align first with `vision_align`
> (e.g. `duburi.vision.align('torpedo_hole', yaw=0, lat=0, depth=0, err=12)`),
> check the returned `VisionResult`, then fire with `duburi.fire(channel)`
> (the `fire` verb below). 1/2 = torpedo, 3/4 = dropper. The old align +
> stable-hold + retry-fire loop is now mission-authored: align, then fire
> once the result is truthy.

### `fire`

Fire an ESP32 payload channel directly (without vision alignment). Requires `PayloadDriver` auto-detected at startup.

```python
duburi.fire(fire_channel=1.0)   # torpedo_1
duburi.fire(fire_channel=3.0)   # dropper_1
```

| Channel | Payload |
|---|---|
| 1 | torpedo_1 |
| 2 | torpedo_2 |
| 3 | dropper_1 |
| 4 | dropper_2 |

| Aspect | Value |
|---|---|
| CLI | `duburi fire --fire_channel 1` |
| DSL | `duburi.fire(fire_channel=1.0)` |
| Check | `duburi.payload_ready` → `bool` |
| Source | `duburi_control/payload.py` · `PayloadDriver` — auto-detects Espressif/CH340 USB-serial at startup, excludes BNO085 port |
| Fallback | If `PayloadDriver` not connected, logs warning; returns `success=False` |

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
