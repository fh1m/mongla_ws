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
ros2 run duburi_manager auv_manager --ros-args -p debug:=true
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

## 1. Power & mode

### `arm`

| Aspect      | Value                                                   |
| ----------- | ------------------------------------------------------- |
| CLI         | `duburi arm [--timeout 15.0]`                           |
| Python      | `duburi.arm(timeout=15.0)`                              |
| MAVLink     | `COMMAND_LONG (MAV_CMD_COMPONENT_ARM_DISARM, p1=1)`     |
| Implements  | `Pixhawk.arm` in `pixhawk.py` (waits for COMMAND_ACK + heartbeat-armed bit) |
| Failure modes | `RC_FAIL` if pre-arm checks reject; `NOT_ARMED_AFTER_ACK` if ACK arrives but `is_armed()` stays False |
| `[MAV ]`    | `[MAV arm cmd=arm] COMPONENT_ARM_DISARM p1=1`                                |

### `disarm`

| Aspect      | Value                                                   |
| ----------- | ------------------------------------------------------- |
| CLI         | `duburi disarm [--timeout 20.0]`                        |
| Python      | `duburi.disarm(timeout=20.0)`                           |
| MAVLink     | `SET_MODE -> MANUAL` then 3 s settle, then `RC neutral`, then `COMMAND_LONG (MAV_CMD_COMPONENT_ARM_DISARM, p1=0)` |
| Implements  | `Pixhawk.disarm` in `pixhawk.py` -- mirrors what QGC does, dodges ArduSub's "still moving" disarm rejection |
| `[MAV ]`    | `[MAV set_mode cmd=disarm] MANUAL (id=19)` ... `[MAV send_rc_override cmd=disarm] all=neutral` ... `[MAV disarm cmd=disarm] COMPONENT_ARM_DISARM p1=0` |

### `set_mode`

| Aspect      | Value                                                   |
| ----------- | ------------------------------------------------------- |
| CLI         | `duburi set_mode --target_name ALT_HOLD [--timeout 8.0]` |
| Python      | `duburi.set_mode('ALT_HOLD', timeout=8.0)`              |
| Mode names  | `MANUAL`, `STABILIZE`, `ACRO`, `DEPTH_HOLD` (alias for `ALT_HOLD`), `ALT_HOLD`, `POSHOLD`, `GUIDED`, `AUTO`, `CIRCLE`, `SURFACE` |
| MAVLink     | `SET_MODE` (legacy message, no COMMAND_ACK) -- we retry every 300 ms and poll the heartbeat for the actual change |
| `[MAV ]`    | `[MAV set_mode cmd=set_mode] <NAME> (id=<N>)` once before the retry loop      |
| Notes       | Auto-engaged by `set_depth` (-> ALT_HOLD), `yaw_left/right`, `lock_heading`. ALT_HOLD inherently holds depth + heading once Ch3/Ch4 are neutral. |

---

## 2. Stop / pause

### `stop`

| Aspect      | Value                                                   |
| ----------- | ------------------------------------------------------- |
| CLI         | `duburi stop`                                           |
| Python      | `duburi.stop()`                                         |
| MAVLink     | `RC_CHANNELS_OVERRIDE` six 1500s for ~0.6 s            |
| Behaviour   | **Active hold** -- ArduSub still sees us as the pilot, so its onboard heading/depth holds latch at the current state. Use between commands. |
| `[MAV ]`    | `[MAV send_rc_override cmd=stop] all=neutral` (one per tick, ~12 frames over 0.6 s) |

### `pause`

| Aspect      | Value                                                   |
| ----------- | ------------------------------------------------------- |
| CLI         | `duburi pause [--duration 2.0]`                         |
| Python      | `duburi.pause(seconds=2.0)`                             |
| MAVLink     | `RC_CHANNELS_OVERRIDE` six 65535s for `duration` s       |
| Behaviour   | **Release** -- pilot OFF the loop. ArduSub's mode automation (ALT_HOLD, POSHOLD) runs unhindered. Use between mode changes or to A/B-test what ArduSub does on its own. Suspends `Heartbeat` for the duration. |
| `[MAV ]`    | `[MAV release_rc_override cmd=pause] all=released`                              |

---

## 3. Translations (Ch5 forward, Ch6 lateral)

`gain` is the stick percentage (0..100) and `seconds` is open-loop
time-of-flight. Currents and battery state change the metres-per-
second mapping every run -- use vision verbs (§6) for precision and
open-loop for *getting close*.

| Verb           | Channel | CLI                                              | Python                          | MAVLink            |
| -------------- | ------- | ------------------------------------------------ | ------------------------------- | ------------------ |
| `move_forward` | Ch5 +   | `duburi move_forward --duration 5 [--gain 80]`   | `duburi.move_forward(5, gain=80)` | RC_OVERRIDE @ 20 Hz |
| `move_back`    | Ch5 -   | `duburi move_back --duration 5 [--gain 80]`      | `duburi.move_back(5, gain=80)`    | RC_OVERRIDE @ 20 Hz |
| `move_left`    | Ch6 -   | `duburi move_left --duration 5 [--gain 80]`      | `duburi.move_left(5, gain=80)`    | RC_OVERRIDE @ 20 Hz |
| `move_right`   | Ch6 +   | `duburi move_right --duration 5 [--gain 80]`     | `duburi.move_right(5, gain=80)`   | RC_OVERRIDE @ 20 Hz |

* **Profile:** constant-gain (default) or eased (smootherstep envelope
  via `motion_easing.trapezoid_ramp`). Switch to eased by launching
  the manager with `-p smooth_translate:=true` -- verb names don't
  change.
* **Lock-aware:** if `lock_heading` is active, the writer is
  `send_rc_translation` (Ch4 released to 65535 so the lock thread
  keeps Ch4 authority). Otherwise `send_rc_override` (Ch4 = 1500).
* **Implementation:** `motion_forward.py` for Ch5, `motion_lateral.py`
  for Ch6.
* **Common kwargs:** `settle` (extra hold-still seconds after the
  drive completes; default 0).
* **Brake:** `_constant` profile fires a reverse-kick brake before the
  settle. `_eased` profile uses the ease-out as the brake (no kick).

---

## 4. Yaw (sharp pivots)

| Verb         | CLI                                              | Python                             |
| ------------ | ------------------------------------------------ | ---------------------------------- |
| `yaw_left`   | `duburi yaw_left  --target 90 [--timeout 30]`    | `duburi.yaw_left(90, timeout=30)`  |
| `yaw_right`  | `duburi yaw_right --target 90 [--timeout 30]`    | `duburi.yaw_right(90, timeout=30)` |

* **Mode:** auto-engages ALT_HOLD if not already in a yaw-capable
  mode (so ArduSub's attitude stabiliser can convert Ch4 rate
  commands into actual yaw via the four-thruster mixer).
* **Profile:** `yaw_snap` (default) is bang-bang Ch4 rate to a target
  heading. `yaw_glide` is smootherstep envelope; switch with the
  manager's `-p smooth_yaw:=true` flag.
* **Implementation:** `motion_yaw.py`.
* **MAVLink:** RC_CHANNELS_OVERRIDE, Ch4 only, at 10 Hz
  (`YAW_RATE_HZ`). Sign convention: +stick = right turn (so
  `yaw_left(90)` writes negative Ch4 percent).
* **Lock-aware:** if `lock_heading` is active, the lock is
  *suspended* while yaw runs, then *retargeted* to the new heading on
  exit (so after `yaw_left(90)` the lock now holds 90 deg less than
  before, with no extra calls).

---

## 5. Curved motion (arc)

### `arc`

| Aspect      | Value                                                                    |
| ----------- | ------------------------------------------------------------------------ |
| CLI         | `duburi arc --duration 5 [--gain 50] [--yaw_rate_pct 30]`                |
| Python      | `duburi.arc(5, gain=50, yaw_rate_pct=30)`                                |
| MAVLink     | `RC_CHANNELS_OVERRIDE` writing **Ch5 + Ch4 in the same packet** at 20 Hz |
| Implements  | `motion_forward.arc`                                                     |
| Behaviour   | Car-style curved motion. `gain` is forward thrust pct (signed: negative = arc in reverse). `yaw_rate_pct` is signed yaw stick (+ = right turn, - = left). Duration is open-loop seconds. Suspends `lock_heading`, retargets to the exit heading. |
| `[MAV ]`    | `[MAV send_rc_override cmd=arc] yaw=<pwm> fwd=<pwm>` (one per tick; `pitch/roll/thr/lat` skipped because they're at neutral) |

---

## 6. Depth

### `set_depth`

| Aspect      | Value                                                                                 |
| ----------- | ------------------------------------------------------------------------------------- |
| CLI         | `duburi set_depth --target -1.5 [--timeout 30] [--settle 0]`                          |
| Python      | `duburi.set_depth(-1.5, timeout=30)`                                                  |
| MAVLink     | Auto-engage `SET_MODE -> ALT_HOLD` if needed, then `SET_POSITION_TARGET_GLOBAL_INT` (alt only, all other axes masked) at `DEPTH_SETPOINT_HZ` (5 Hz) **only while driving to target** |
| Implements  | `motion_depth.hold_depth` -> `Pixhawk.set_target_depth`                               |
| Behaviour   | Drives the AUV to absolute depth `target` metres (negative below surface). Returns once `\|altitude - target\| < 0.05 m` for 0.5 s. After the target is reached, ArduSub's onboard ALT_HOLD (400 Hz internal PID) keeps holding -- we stop streaming. |
| `[MAV ]`    | `[MAV set_target_depth cmd=set_depth] depth=-1.50m` per tick during the drive    |

> **There is no `lock_depth` or `unlock_depth` verb.** ALT_HOLD
> inherently holds whatever altitude is current the moment Ch3 goes
> back to neutral. The `Heartbeat` daemon (5 Hz neutral RC override)
> keeps the wire warm so ArduSub does not trip `FS_PILOT_INPUT` and
> disarm. See [`ardusub-canon.md`](./ardusub-canon.md) §depth for
> theory.

---

## 7. Heading lock (background)

### `lock_heading`

| Aspect      | Value                                                                            |
| ----------- | -------------------------------------------------------------------------------- |
| CLI         | `duburi lock_heading [--target 0] [--timeout 300]`                              |
| Python      | `duburi.lock_heading(degrees=0.0, timeout=300.0)` (returns immediately)         |
| MAVLink     | Background daemon thread streams `RC_CHANNELS_OVERRIDE` Ch4-only (rate command) at `LOCK_STREAM_HZ` (20 Hz) |
| Implements  | `heading_lock.HeadingLock`; owned and started by `Duburi.lock_heading`           |
| Behaviour   | `target=0` means lock at the current heading right now. Loop reads heading from configured `yaw_source` (BNO085 / AHRS / SITL), computes proportional Ch4 yaw-rate command, clamps to `+/-LOCK_PCT_MAX`. Pauses `Heartbeat` while active (the lock IS the heartbeat). |
| `[MAV ]`    | `[MAV send_rc_override] yaw=<pwm>` per tick (no `cmd=` -- the lock thread is its own thread; `lock_heading`'s own setup frames carry `cmd=lock_heading`) |
| See         | [`heading-lock.md`](./heading-lock.md) for state diagram and failure modes.       |

### `unlock_heading`

| Aspect      | Value                                                                            |
| ----------- | -------------------------------------------------------------------------------- |
| CLI         | `duburi unlock_heading`                                                          |
| Python      | `duburi.unlock_heading()` (Duburi facade) / `duburi.release_heading()` (DSL)     |
| MAVLink     | Stops the lock thread, sends `RC_CHANNELS_OVERRIDE` six 1500s, resumes `Heartbeat` |
| `[MAV ]`    | `[MAV send_rc_override cmd=unlock_heading] all=neutral` (single neutral frame)  |

---

## 8. Vision verbs (closed-loop)

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

| Aspect      | Value                                                                            |
| ----------- | -------------------------------------------------------------------------------- |
| CLI         | `duburi vision_acquire --target_class person [--target_name yaw_right] [--timeout 30] [--gain 25] [--yaw_rate_pct 25]` |
| Python      | `duburi.vision_acquire(camera='laptop', target_class='person', target_name='yaw_right', ...)` |
| DSL         | `duburi.vision.find(target='person', sweep='right', timeout=25.0, ...)`         |
| Sweep modes | `''` / `'still'` (wait), `'yaw_left'`, `'yaw_right'`, `'move_forward'`, `'arc'` |
| Returns     | `final_value = time-to-acquire seconds` on success, `error_value = 0` on success, timeout reason on failure |

### `vision_align_yaw`

Centre target horizontally via Ch4 yaw rate (P loop on `ex`).

| Aspect      | Value                                                                            |
| ----------- | -------------------------------------------------------------------------------- |
| CLI         | `duburi vision_align_yaw --target_class person [--duration 15] [--kp_yaw 60] [--deadband 0.18] [--lock_mode settle]` |
| Python      | `duburi.vision_align_yaw(camera='laptop', target_class='person', ...)`           |
| DSL         | `duburi.vision.yaw(target='person', duration=8.0, kp_yaw=60.0, ...)`            |
| Channel     | Ch4 only                                                                         |
| Math        | `yaw_pct = clamp(ex * kp_yaw, ±35)` -> `RC_CHANNELS_OVERRIDE`                  |
| New params  | `lock_mode` — controls when the verb exits (see Lock modes below)                |

### `vision_align_lat`

Centre horizontally via Ch6 lateral strafe (P loop on `ex`).

| Aspect      | Value                                                                            |
| ----------- | -------------------------------------------------------------------------------- |
| CLI         | `duburi vision_align_lat --target_class person [--duration 15] [--kp_lat 60] [--lock_mode settle]` |
| Python      | `duburi.vision_align_lat(camera='laptop', target_class='person', ...)`           |
| DSL         | `duburi.vision.lateral(target='person', duration=8.0, kp_lat=60.0, ...)`        |
| Channel     | Ch6 only                                                                         |
| Math        | `lat_pct = clamp(ex * kp_lat, ±35)` -> `RC_CHANNELS_OVERRIDE`                  |
| New params  | `lock_mode` — controls when the verb exits (see Lock modes below)                |

### `vision_align_depth`

Centre vertically via incremental ALT_HOLD setpoint nudges (P loop on `ey_anchor`).

| Aspect      | Value                                                                            |
| ----------- | -------------------------------------------------------------------------------- |
| CLI         | `duburi vision_align_depth --target_class person [--duration 15] [--kp_depth 0.05] [--depth_anchor_frac 0.5]` |
| Python      | `duburi.vision_align_depth(camera='laptop', target_class='person', ...)`         |
| DSL         | `duburi.vision.depth(target='person', duration=8.0, kp_depth=0.05, ...)`        |
| Channel     | Depth setpoint (`SET_POSITION_TARGET_GLOBAL_INT`) at 5 Hz                        |
| Math        | `ey_anchor = ey + (2×anchor - 1)×h_frac` then `depth_nudge = clamp(ey_anchor * kp_depth, ±0.02)` |
| New params  | `depth_anchor_frac` (0=top, 0.5=centre, 1=bottom of bbox); `lock_mode`          |

### `vision_hold_distance`

Drive Ch5 so the target's size proxy matches `target_bbox_h_frac`
(P loop on `target_h_frac - size`).

| Aspect      | Value                                                                            |
| ----------- | -------------------------------------------------------------------------------- |
| CLI         | `duburi vision_hold_distance --target_class person --target_bbox_h_frac 0.55 [--duration 20] [--kp_forward 200] [--distance_metric height] [--lock_mode settle]` |
| Python      | `duburi.vision_hold_distance(camera='laptop', target_class='person', ...)`       |
| DSL         | `duburi.vision.forward(target='person', distance=0.55, duration=12.0, ...)`     |
| Channel     | Ch5 only                                                                         |
| Math        | `size = _distance_size(sample, metric)` then `fwd_pct = clamp((target_h_frac - size) * kp_forward, ±50)` |
| New params  | `distance_metric` (height/area/diagonal); `lock_mode`                           |

### `vision_align_3d`

Hold N axes at once. All axes settle together (or run until duration in `follow`/`pursue` modes).

| Aspect      | Value                                                                            |
| ----------- | -------------------------------------------------------------------------------- |
| CLI         | `duburi vision_align_3d --target_class gate --axes yaw,forward,depth --target_bbox_h_frac 0.50 [--duration 20] [--lock_mode settle] [--distance_metric height] [--depth_anchor_frac 0.5]` |
| Python      | `duburi.vision_align_3d(camera='laptop', target_class='gate', axes='yaw,forward,depth', ...)` |
| DSL         | `duburi.vision.lock(target='gate', axes='yaw,forward,depth', distance=0.50, duration=15.0, ...)` |
| `axes`      | CSV, any subset of `'yaw,lat,depth,forward'`                                    |
| Loop body   | Single 20 Hz tick writes Ch4+Ch5+Ch6 in one RC packet, plus a 5 Hz depth-setpoint sub-tick |
| New params  | `lock_mode`, `distance_metric`, `depth_anchor_frac`                             |

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
| `visual_pid`         | -                            | Structural placeholder for v2; body is P-only today.      |

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

## 9. Cross-references

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
