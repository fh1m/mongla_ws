# Duburi Mission Cookbook

> Read this once, then design any mission you can imagine in 5 minutes.
>
> The CLI cookbook lives in the README §9 — one-liners for the deck.
> This file is the **mission-author's** companion: principles first,
> verbs second, full samples last.

The mission DSL lives in
[`src/duburi_planner/duburi_planner/duburi_dsl.py`](../../src/duburi_planner/duburi_planner/duburi_dsl.py).
Every mission file in
[`src/duburi_planner/duburi_planner/missions/`](../../src/duburi_planner/duburi_planner/missions/)
is a plain Python module that exposes one `run(duburi, log)` function.

---

## 0. Designing a mission in 30 seconds

1. **Drop a file** in
   [`src/duburi_planner/duburi_planner/missions/`](../../src/duburi_planner/duburi_planner/missions/),
   for example `follow_gate.py`.
2. **Expose `def run(duburi, log)`** — that is the entire contract. No
   registry table, no `__init__.py` edit. Files starting with `_` are
   skipped (use them for shared helpers).
3. **Inside `run`, call `duburi.<verb>(...)` lines top to bottom.** The
   DSL prints one outcome line per call — no logging boilerplate.
4. **Build + run:**

   ```bash
   colcon build --packages-select duburi_planner && source install/setup.bash
   ros2 run duburi_planner mission --list          # your file appears
   ros2 run duburi_planner mission follow_gate
   ```

A complete file is just this:

```python
# src/duburi_planner/duburi_planner/missions/follow_gate.py
def run(duburi, log):
    duburi.target = 'gate'
    duburi.arm()
    duburi.set_depth(-1.0)
    duburi.vision.find(sweep='right', timeout=20.0)
    duburi.vision.lock(axes='yaw,forward,depth', distance=0.55, duration=15.0)
    duburi.set_depth(0.0)
    duburi.disarm()
```

That's the whole pattern. The rest of this cookbook fills in **which
verbs exist** and **how to tune them**.

---

## 1. Mental model in 60 seconds

A mission is a script. Each line is **one MAVLink command** that runs
to completion (or raises) before the next line starts. There is no
parallelism inside a mission — it's a straight line, top to bottom.

Two namespaces, one DSL:

| Namespace | What it does                       | Closed loop?          |
| --------- | ---------------------------------- | --------------------- |
| `duburi.*`        | Open-loop motion (RC overrides + ALT_HOLD setpoints) | No |
| `duburi.vision.*` | Closed-loop, bbox-driven motion                       | Yes |

**Both live on the same `duburi` object.** A mission usually pings
between them: open-loop to *go somewhere*, vision to *land precisely*.

```python
def run(duburi, log):
    duburi.arm()
    duburi.set_depth(-0.5)
    duburi.move_forward(3.0, gain=60)         # open-loop -- go to area
    duburi.vision.find()                      # search for the target
    duburi.vision.lock(axes='yaw,forward',    # closed-loop -- land on it
                       distance=0.55,
                       duration=12)
    duburi.move_back(2.0, gain=60)
    duburi.disarm()
```

That's the entire pattern. The rest of this cookbook fills in the
verbs and tuning.

---

## 2. Hard rules (read them, internalise them)

1. **One axis per command.** A `move_forward` only writes Ch5. A
   `yaw_right` only writes Ch4. Mixing axes is the controller's job
   (`arc` for forward+yaw, `vision.lock` for any subset). This is
   AXIS ISOLATION and it is what keeps the sub predictable.

2. **Vision informs control, never fights it.** Vision verbs talk
   *to* the same control stack the open-loop verbs use. They don't
   open a parallel channel. So `duburi.vision.yaw(...)` is exactly
   `yaw_right` driven by a bbox-error PI loop instead of a clock.

3. **Sticky context.** `duburi.camera` and `duburi.target` default to
   `'laptop'` and `'person'`. Set them once at the top of the mission
   and every `duburi.vision.*` call reads them. Override per call only
   when you mean it.

4. **Tunables are ROS params.** Gains, deadbands, and distance
   targets all have live `vision.*` ROS-param fallbacks declared on
   `auv_manager_node`. **Leave overrides at the rosidl zero default
   in your mission** (don't pass them) and tune from the deck:

   ```bash
   ros2 param set /duburi_manager vision.kp_yaw     80.0
   ros2 param set /duburi_manager vision.deadband   0.08
   ros2 param set /duburi_manager vision.target_bbox_h_frac 0.55
   ```

5. **Every verb blocks until the action server returns.** No
   threading. If you want a background hold (yaw lock during motion),
   use `duburi.lock_heading(...)` — that's the one verb that returns
   immediately and runs a daemon thread.

6. **Failure raises.** `MoveRejected` / `MoveFailed` propagate. Wrap
   the whole mission in a try/finally that calls `duburi.disarm()` if
   you want a guaranteed safe exit.

---

## 3. The verbs

### 3.1  Open-loop motion (`duburi.*`)

Every verb is a method on the `DuburiMission` instance you receive
in `run(duburi, log)`. Defaults match the action server's
`COMMANDS` registry; pass kwargs to override.

#### Power & mode

```python
duburi.arm()                                # waits for ACK (timeout=15s)
duburi.disarm()
duburi.set_mode('ALT_HOLD')                 # 'STABILIZE', 'POSHOLD', 'GUIDED', ...
```

#### Translations (Ch5 forward, Ch6 lateral)

```python
duburi.move_forward(seconds, gain=80, settle=0.0)
duburi.move_back   (seconds, gain=80, settle=0.0)
duburi.move_left   (seconds, gain=80, settle=0.0)
duburi.move_right  (seconds, gain=80, settle=0.0)
```

| Param   | Type  | Meaning                                                     |
| ------- | ----- | ----------------------------------------------------------- |
| seconds | float | Drive duration. Open-loop: time-of-flight, NOT distance.    |
| gain    | float | Stick percentage, 0..100. >50% in pool, ~30% on the bench. |
| settle  | float | Extra hold-still seconds *after* the drive completes.       |

> Switch to **eased** translation profiles (trapezoidal ramp, no
> reverse-kick brake) by launching the manager with
> `--ros-args -p smooth_translate:=true`. The verb names don't change.

#### Depth

```python
duburi.set_depth(metres, timeout=30.0, settle=0.0)   # drive-to-depth (blocks)
```

`set_depth` auto-engages ALT_HOLD if not already in it, then drives
the AUV to the target via `SET_POSITION_TARGET_GLOBAL_INT` and
returns once the controller is within 0.05 m of the target for 0.5 s.
`metres` is **negative below surface**: `set_depth(-1.5)` = 1.5 m
deep.

There is **no `lock_depth` verb** — and you don't need one. ArduSub's
onboard ALT_HOLD inherently latches whatever altitude is current the
moment Ch3 (throttle) returns to neutral 1500. So once `set_depth`
returns, the autopilot's 400 Hz internal depth PID continues to hold
that depth for free, with zero further MAVLink traffic from us. The
[`Heartbeat`](../../src/duburi_control/duburi_control/heartbeat.py)
daemon keeps streaming neutral RC overrides at 5 Hz in the
background so ArduSub never trips the `FS_PILOT_INPUT` failsafe and
disarms — that is the *only* depth-related continuous traffic.

#### Yaw (sharp pivots)

```python
duburi.yaw_left (degrees, timeout=30.0, settle=0.0)
duburi.yaw_right(degrees, timeout=30.0, settle=0.0)
```

Engages ALT_HOLD if needed. Auto-suspends `lock_heading` for the
duration and re-targets it on exit. The verb sign is implicit
(`yaw_left(90)` rotates +90° to port; the underlying signed degrees
get flipped for you).

#### Curved trajectory (`arc`)

```python
duburi.arc(seconds, gain=50, yaw_rate_pct=30, settle=0.0)
```

Forward thrust *and* yaw rate in **one** RC packet. Suspends
`lock_heading`, re-targets to the exit heading. Negative `gain`
runs the arc in reverse.

#### Heading lock (background)

```python
duburi.lock_heading(degrees=0.0, timeout=300.0)   # returns immediately
... mission body ...
duburi.release_heading()                          # joins the daemon
```

Spawns a 20 Hz proportional Ch4 yaw-rate streamer
([`heading_lock.py`](../../src/duburi_control/duburi_control/heading_lock.py))
in a background thread. The loop reads heading from the configured
`yaw_source` (BNO085 / AHRS / SITL), computes a yaw error, and
writes a clamped Ch4 RC override every 50 ms — this is what
ArduSub interprets as "the pilot is requesting this yaw rate", so
the closed-loop is Python -> Ch4 -> ArduSub's 400 Hz attitude
stabiliser. `degrees=0` means *lock at the current heading right
now*. The thread auto-suspends during yaw / arc / pause and
re-targets on exit.

#### Stop / pause

```python
duburi.stop()                # 1500 PWM on every channel for 0.6s
duburi.pause(2.0)            # NO_OVERRIDE for 2s -- autopilot takes over
```

`stop` is **active hold**; `pause` is **release**. Use `stop`
between commands, `pause` for stabilisation between mode changes.

---

### 3.2  Vision-driven motion (`duburi.vision.*`)

Same axis names as the open-loop verbs above, but each one runs a
closed P (or PI in v2) loop on the **largest detection** of
`target` in `camera`. Sticky context comes from the parent
`duburi`; pass `target=...` / `camera=...` to override per call.

#### Find a target

```python
duburi.vision.find(target=None, sweep='right', timeout=25.0,
                   gain=25.0, yaw_rate_pct=22.0)
```

Drives the chosen sweep and returns the moment one fresh detection
arrives. Sweeps:

| sweep      | Drive while waiting               |
| ---------- | --------------------------------- |
| `'right'`  | yaw right at `yaw_rate_pct`       |
| `'left'`   | yaw left  at `yaw_rate_pct`       |
| `'forward'`| Ch5 forward at `gain`             |
| `'arc'`    | Ch5 + Ch4 (uses both knobs)       |
| `'still'`  | wait in place                     |

#### Single-axis alignment

| Verb                          | Channel  | Bbox dim it watches                    |
| ----------------------------- | -------- | -------------------------------------- |
| `duburi.vision.yaw(...)`      | Ch4 yaw  | horizontal centre error `ex`           |
| `duburi.vision.lateral(...)`  | Ch6 lat  | horizontal centre error `ex`           |
| `duburi.vision.depth(...)`    | depth SP | vertical centre error `ey`             |
| `duburi.vision.forward(...)`  | Ch5 fwd  | bbox height fraction (distance proxy)  |

```python
duburi.vision.yaw     (target=None, duration=8.0,  **overrides)
duburi.vision.lateral (target=None, duration=8.0,  **overrides)
duburi.vision.depth   (target=None, duration=8.0,  **overrides)
duburi.vision.forward (target=None, distance=0.55, duration=12.0, **overrides)
```

`distance` IS the bbox height as a fraction of image height
(0..1). 0.30 is far, 0.70 is "right in your face". Tune this in
the pool against your actual target size.

#### Multi-axis lock

```python
duburi.vision.lock(target=None, axes='yaw,forward',
                   distance=0.55, duration=15.0, **overrides)
```

`axes` is a CSV: any subset of `'yaw,lat,depth,forward'`. **All
axes settle together.** The loop runs at 20 Hz, depth setpoints
stream at 5 Hz (so ALT_HOLD doesn't fight you).

#### Overrides (only when you really mean it)

Every vision verb accepts these kwargs, but **prefer leaving them
out** so the live ROS-param value applies:

| Override             | Default param       | Effect                                                    |
| -------------------- | ------------------- | --------------------------------------------------------- |
| `kp_yaw`             | `vision.kp_yaw`     | Proportional gain on `ex`, Ch4 percent units              |
| `kp_lat`             | `vision.kp_lat`     | Same on Ch6                                               |
| `kp_depth`           | `vision.kp_depth`   | Metres of nudge per unit `ey` per 5 Hz tick               |
| `kp_forward`         | `vision.kp_forward` | Ch5 percent per unit (target_h_frac − h_frac)             |
| `deadband`           | `vision.deadband`   | Per-axis settle band; `\|err\| < deadband` counts as in  |
| `target_bbox_h_frac` | `vision.target_bbox_h_frac` | Distance proxy used by `forward` / `lock`         |
| `stale_after`        | `vision.stale_after`| Seconds after which a Sample is "lost"                    |
| `on_lost`            | `vision.on_lost`    | `'fail'` (default) or `'hold'` (park, never raise)        |

A vision verb succeeds when `axes_in_deadband` is True for
`SETTLED_TICK_BUDGET` consecutive ticks (default 4 ticks @ 20 Hz =
0.2 s). It fails on `LOST_TICK_BUDGET` consecutive lost ticks
(default 12 ticks = 0.6 s) when `on_lost='fail'`.

---

## 3.3  Tracking while moving — ByteTrack + Kalman

### What tracking adds

The bare vision pipeline (`/detections`) gives you the largest box this
frame. Tracking (`/tracks`) gives you a **stable ID** that persists across
frames, through brief occlusions, and across any move command you issue
while the target is in view. The Kalman smoother inside `tracker_node`
also removes per-frame bbox jitter so the P-loop setpoint is smoother.

### Turning tracking on

Two layers control whether tracking is active:

**1. Launch layer — start `tracker_node`:**

```bash
ros2 launch duburi_vision cameras_.launch.py with_tracking:=true
```

**2. Per-goal DSL flag — route this goal through `/tracks`:**

```python
duburi.vision.lock(target='gate', axes='yaw,forward',
                   tracking=True, ...)
```

`tracking=True` tells `auv_manager_node` to subscribe `/tracks` for this
goal instead of `/detections`. If `tracker_node` is not running the goal
will stall waiting for its first sample — always pair with `with_tracking:=true`.

### `lock_mode` — the key parameter for tracking-while-moving

`lock_mode` controls when `vision.lock` exits:

| `lock_mode` | Exits when ...                                       | Use case                            |
|-------------|------------------------------------------------------|-------------------------------------|
| `'settle'`  | target is inside deadband for `SETTLED_TICK_BUDGET`  | Approach and stop at target         |
| `'follow'`  | `duration` expires (never exits on deadband alone)   | Continuous tracking, orbit steps    |
| `'pursue'`  | reserved (NYI)                                       | —                                   |

**Rule of thumb:** use `settle` to *arrive*; use `follow` to *keep moving*.

### Predicted frames and `on_lost`

When the detector misses a frame, the Kalman filter forward-predicts the
track position and emits a Detection with `score=0.0`. `VisionState` treats
these as live samples — they do NOT increment the lost-tick counter. This
means `on_lost` is only triggered when the tracker itself drops the track
entirely (after `track_buffer` frames with no detector hit), not on single-
frame occlusions.

Implication: with `tracking=True` + `on_lost='hold'` you can drive through
complete momentary occlusions (e.g. a fish crossing the gate) without the
goal aborting.

### Orbit pattern — canonical example

The orbit is the flagship use case for `follow` mode. Each step:
1. `yaw_left(30°)` — pivot in place; target drifts off-centre
2. `vision.lock(..., lock_mode='follow', duration=3.0)` — re-centre and hold
   for 3 seconds, then exit and go to the next step

```python
ORBIT_STEPS     = 12      # 12 × 30° = 360°
ORBIT_STEP_DEG  = 30.0
ORBIT_HOLD_S    = 3.0

for step in range(ORBIT_STEPS):
    duburi.yaw_left(ORBIT_STEP_DEG, timeout=10.0, settle=0.3)
    # lock_mode='follow': keep tracking for the full 3 s window,
    # never exit early just because we hit deadband.
    duburi.vision.lock(
        target='flare',
        axes='yaw,forward,depth',
        distance=0.40,
        duration=ORBIT_HOLD_S,
        on_lost='hold',       # survive brief occlusions
        lock_mode='follow',
        tracking=True,
    )
```

The 12-step polygon is a good approximation of a circle for pre-qual.
For a smoother arc replace the loop with `arc()` calls.

### Tracking during linear moves

For approach-and-follow (e.g. swimming alongside a moving object):

```python
# Move forward while re-locking every 2 s — functional tracking loop
for _ in range(5):
    duburi.move_forward(2.0, gain=40.0)     # open-loop
    duburi.vision.lock(
        target='buoy', axes='yaw',
        duration=2.0,
        lock_mode='follow',
        tracking=True,
    )
```

For pure heading-correction while driving forward, `axes='yaw'` with
`lock_mode='follow'` and a short `duration` is enough:

```python
duburi.vision.lock(target='gate', axes='yaw',
                   duration=20.0, lock_mode='follow', tracking=True)
# (runs for up to 20 s, correcting yaw to stay centred on gate)
```

### Enabling tracking globally via ROS params

```bash
# From a second terminal while mission is running:
ros2 param set /duburi_manager vision.use_tracks true   # switch to /tracks
ros2 param set /duburi_manager vision.use_tracks false  # back to /detections
```

Or set it at node start:
```bash
ros2 run duburi_manager start --ros-args -p vision.use_tracks:=true
```

### Smoke-testing the tracker pipeline

```bash
# 1. Launch vision stack with tracking enabled
ros2 launch duburi_vision cameras_.launch.py with_tracking:=true

# 2. Topic health check
ros2 run duburi_vision vision_check --camera laptop

# 3. Confirm /tracks is publishing
ros2 topic hz /duburi/vision/laptop/tracks

# 4. Inspect a track (should have tracking_id set, score > 0 for real detections)
ros2 topic echo /duburi/vision/laptop/tracks --once

# 5. Full integration test via tracker_check CLI
ros2 run duburi_vision tracker_check --camera laptop --class person
```

### Performance notes

- `tracker_node` runs on the same Jetson Orin Nano as the detector.
  ByteTrack state is cheap (~10 µs per update); Kalman adds ~5 µs per track.
- At 30 fps with 5 tracked objects the combined overhead is < 1 ms, well
  within the 33 ms frame budget.
- `track_buffer=30` (default) means a track survives 1 s of occlusion at
  30 fps before being dropped. Tune lower for fast-moving objects in busy
  scenes, higher for slow targets in clean scenes.

---

## 3.4  Model and class selection

### Drop-and-use — no YAML required

Drop any `.pt` file into `src/duburi_vision/models/`. The detector reads
class names from the model's embedded names table (`model.names`), which
Ultralytics always populates at training time. **No sidecar YAML needed.**

```
models/
├── gate_nano_100ep.pt            # gate only, nano, fast
├── gate_medium_100ep.pt          # gate only, medium accuracy
├── gate_medium_200ep.pt          # gate only, fine-tuned 200 ep
├── flare_medium_100ep.pt         # flare only
└── gate_flare_medium_100ep.pt    # gate + flare combined  (prequal default)
```

Optional YAML override (only if you need to remap class IDs):
```yaml
# gate_flare_medium_100ep.yaml  -- optional; omit this and model.names is used
names:
  0: gate
  1: flare
```

### Selecting model and classes at launch

```bash
# Gate-only model
ros2 launch duburi_vision cameras_.launch.py model:=gate_medium_100ep classes:=gate

# Flare-only model
ros2 launch duburi_vision cameras_.launch.py model:=flare_medium_100ep classes:=flare

# Combined model — gate approach phase
ros2 launch duburi_vision cameras_.launch.py \
    model:=gate_flare_medium_100ep classes:=gate

# Combined model — show both classes (debug)
ros2 launch duburi_vision cameras_.launch.py \
    model:=gate_flare_medium_100ep classes:=gate,flare
```

### Switching class filter live (no node restart)

The model is loaded once at launch. `classes` is a post-inference allowlist —
same forward pass every frame, different box list published. Change it:

```bash
# Shell
ros2 param set /duburi_detector classes gate
ros2 param set /duburi_detector classes flare
ros2 param set /duburi_detector classes "gate,flare"
ros2 param set /duburi_detector classes ""    # publish ALL model classes
```

```python
# From inside a mission (takes effect next inference frame)
duburi.set_classes('gate')
duburi.set_classes('flare')
duburi.set_classes('gate,flare')
duburi.set_classes('')          # all classes
duburi.set_classes(['gate', 'flare'])  # list form also accepted
```

### Full gate+flare prequal mission pattern

```python
def run(duburi, log):
    duburi.camera = 'forward'

    # Tether removal window -- operator disconnects tether during countdown
    duburi.countdown(10)

    duburi.arm()
    duburi.set_mode('ALT_HOLD')
    duburi.set_depth(-1.0, settle=2.0)
    duburi.dvl_connect()

    # Gate phase -- filter to gate class only
    duburi.set_classes('gate')
    duburi.target = 'gate'
    duburi.vision.find(sweep='forward', timeout=45.0, gain=40.0)
    duburi.vision.lock(axes='yaw,forward', distance=0.42,
                       duration=20.0, on_lost='hold',
                       distance_metric='area')
    duburi.move_forward_dist(3.5, gain=60.0)

    # Flare phase -- switch filter to flare
    duburi.set_classes('flare')
    duburi.target = 'flare'
    duburi.vision.find(sweep='right', timeout=40.0, gain=0.0)
    duburi.vision.lock(axes='yaw,forward,depth', distance=0.38,
                       duration=20.0, on_lost='hold', lock_mode='settle')

    # Orbit flare 360 degrees
    for _ in range(12):
        duburi.yaw_left(30.0, timeout=10.0, settle=0.3)
        duburi.vision.lock(axes='yaw,forward,depth', distance=0.38,
                           duration=3.0, on_lost='hold', lock_mode='follow')

    # Return through gate
    duburi.yaw_right(180.0, timeout=25.0, settle=0.5)
    duburi.set_classes('gate')
    duburi.target = 'gate'
    duburi.vision.find(sweep='right', timeout=30.0, gain=0.0)
    duburi.vision.lock(axes='yaw,forward', distance=0.42,
                       duration=20.0, on_lost='hold', distance_metric='area')
    duburi.move_forward_dist(3.5, gain=60.0)

    duburi.stop()
    duburi.set_depth(0.0)
    duburi.disarm()
```

See `missions/gate_flare_prequal.py` for the production-tuned version of this.

### Edge cases and robustness notes (from preview footage)

| Situation | Behaviour |
|-----------|-----------|
| Gate partially off-frame (one post visible) | yaw axis steers toward visible bbox center — no special code needed |
| Gate fills 90%+ of frame (too close) | forward axis backs off until `distance` fraction is met (`distance_metric='area'`) |
| Flare at distance (narrow, ~10% frame height) | `distance=0.38` with `height` metric still has signal; yaw locks on horizontal center |
| Detection flicker in turbid water | `on_lost='hold'` freezes setpoints for up to `stale_after` seconds (default 1.5 s) |
| False positives (non-target debris) | `conf=0.45` in detector.yaml already filters these; real detections are 0.90-0.97 |

**`distance_metric`** guidance:
- `'area'` (width × height) — use for gates (wide, squat objects)
- `'height'` (default) — use for flares and vertical pipes

### Tether removal countdown

```python
duburi.countdown(10)          # 10-second window, default message
duburi.countdown(15, message='Stand clear. Starting autonomous run.')
```

Prints an ASCII box countdown to stdout. The mission continues immediately
after. All onboard compute (Jetson, Pi, DVL, Pixhawk) is self-sufficient —
removing the tether during this window leaves the AUV fully autonomous.

### Offline pre-pool testing with a video file

```bash
# Run gate model on recorded pool footage
ros2 launch duburi_vision cameras_.launch.py \
    video_file:=/tmp/pool_run.mp4 model:=gate_flare_medium_100ep classes:=gate

# Loop OFF (stop at EOF), with ByteTrack
ros2 launch duburi_vision cameras_.launch.py \
    video_file:=/tmp/gate_run.mp4 model:=gate_medium_100ep classes:=gate \
    loop:=false with_tracking:=true
```

All downstream nodes (`detector_node`, `tracker_node`, vision verbs)
behave identically with `video_file` as with a live camera.

---

## 4. The math, in 30 lines

```
For each tick @ 20 Hz:
  Sample = vision_state.bbox_error(target_class)   # may be None
  if Sample is None or Sample.age_s > stale_after:
    -> write neutral RC, freeze depth setpoint, count lost
  else:
    ex     = (cx - W/2) / (W/2)         # [-1,+1] horizontal centre error
    ey     = (cy - H/2) / (H/2)         # [-1,+1] vertical   centre error
    h_frac =  bbox_h / H                # [0,1]   distance proxy (bigger = closer)

    # Per active axis, P-step + safety clamp, then RC override:
    yaw_pct      = clamp(ex                       * kp_yaw    , +-35)   -> Ch4
    lat_pct      = clamp(ex                       * kp_lat    , +-35)   -> Ch6
    fwd_pct      = clamp((target_h_frac - h_frac) * kp_forward, +-50)   -> Ch5
    depth_nudge  = clamp(ey                       * kp_depth  , +-0.02)
    depth_setpt -= depth_nudge * depth_sign       # +1 fwd-cam, -1 down-cam

  send ONE RC packet (Ch4+Ch5+Ch6) every tick
  send depth setpoint @ 5 Hz on its own sub-tick
```

- Depth integrates **incrementally** so ALT_HOLD never sees a step
  jump.
- All four axes share the same controller body; opting in/out is
  set membership in `axes`, not a separate code path.
- The `visual_pid=True` flag is a structural placeholder for v2;
  the body is P-only today.

Reference implementation:
[`src/duburi_control/duburi_control/motion_vision.py`](../../src/duburi_control/duburi_control/motion_vision.py).

---

## 5. Tuning knobs you'll actually touch

> Every knob below has a `vision.*` ROS-param fallback. Set it on
> the deck; don't bake numbers into mission files unless you want
> them pinned for that specific phase.

### Gains

| Param                | Sane range  | What it costs you when wrong                                        |
| -------------------- | ----------- | ------------------------------------------------------------------- |
| `vision.kp_yaw`      | 30 .. 90    | Too low: slow centre. Too high: oscillation around dead-on.         |
| `vision.kp_lat`      | 30 .. 90    | Same as yaw, but with strafe oscillation.                           |
| `vision.kp_depth`    | 0.02 .. 0.1 | Too low: target drifts vertically. Too high: ALT_HOLD fights you.   |
| `vision.kp_forward`  | 100 .. 300  | Too low: never reaches target distance. Too high: surge / overshoot. |

### Deadband / settle

| Param                       | Sane range | Notes                                          |
| --------------------------- | ---------- | ---------------------------------------------- |
| `vision.deadband`           | 0.05 .. 0.15 | Per-axis. Tighter = more precision, more time. |
| `vision.target_bbox_h_frac` | 0.30 .. 0.70 | Pool-tune against your actual target size.   |
| `vision.stale_after`        | 0.5 .. 1.5   | Seconds. Longer = tolerates dropped frames.    |
| `vision.on_lost`            | 'fail' / 'hold' | 'hold' is for "I know it'll come back". |
| `vision.acquire_yaw_rate_pct` | 15 .. 30 | Sweep speed during `vision.find`.             |
| `vision.acquire_gain`       | 15 .. 35   | Sweep forward thrust during `vision.find('forward')`. |

Defaults live in
[`src/duburi_manager/config/vision_tunables.yaml`](../../src/duburi_manager/config/vision_tunables.yaml).

---

## 6. Ready-to-steal mission samples

Every sample below is a complete `run(duburi, log)`. Drop it in
`src/duburi_planner/duburi_planner/missions/<name>.py`, rebuild, and
run with `ros2 run duburi_planner mission <name>`. The runner
auto-discovers any `*.py` not starting with `_`; no registration
required.

### 6.1  Hello world — arm, dive, surface, disarm

```python
def run(duburi, log):
    log.info('hello_world: starting')
    duburi.arm()
    duburi.set_depth(-0.5)
    duburi.pause(2.0)
    duburi.set_depth(0.0)
    duburi.disarm()
```

### 6.2  Square pattern with heading lock

```python
def run(duburi, log):
    duburi.arm()
    duburi.set_depth(-1.0)
    duburi.lock_heading()                         # latch current heading
    for _ in range(4):
        duburi.move_forward(3.0, gain=60)
        duburi.yaw_right(90.0)                    # lock auto-retargets
    duburi.release_heading()
    duburi.set_depth(0.0)
    duburi.disarm()
```

### 6.3  Find person, centre, hold distance

```python
def run(duburi, log):
    duburi.target = 'person'
    duburi.arm()
    duburi.set_depth(-0.5)
    duburi.move_forward(3.0, gain=60)
    duburi.vision.find(sweep='right', timeout=25.0)
    duburi.vision.lock(axes='yaw,forward',
                       distance=0.55,
                       duration=12.0)
    duburi.move_back(2.0, gain=60)
    duburi.disarm()
```

### 6.4  Reacquire after losing the target

```python
def run(duburi, log):
    duburi.target = 'person'
    duburi.arm()
    duburi.set_depth(-0.5)

    for attempt in range(3):
        log.info(f'attempt {attempt+1}: searching')
        try:
            duburi.vision.find(sweep='arc',  timeout=20.0)
            duburi.vision.lock(axes='yaw,forward',
                               distance=0.55,
                               duration=10.0)
            log.info('locked & held')
            break
        except Exception as exc:
            log.warning(f'lost target ({exc!r}) -- backing off & retrying')
            duburi.move_back(2.0, gain=50)
            duburi.yaw_right(45.0)

    duburi.disarm()
```

### 6.5  Full 3D align (yaw + lat + depth + forward at once)

```python
def run(duburi, log):
    duburi.target = 'gate'
    duburi.arm()
    duburi.set_depth(-1.5)
    duburi.lock_heading()
    duburi.vision.find(sweep='still', timeout=30.0)
    duburi.vision.lock(axes='yaw,lat,depth,forward',
                       distance=0.50,
                       duration=20.0)
    duburi.release_heading()
    duburi.disarm()
```

### 6.6  Patrol pattern: drive, scan, drive, scan

```python
def run(duburi, log):
    duburi.target = 'person'
    duburi.arm()
    duburi.set_depth(-0.8)

    for leg in range(3):
        log.info(f'leg {leg}: drive')
        duburi.move_forward(5.0, gain=55)
        log.info(f'leg {leg}: scan')
        try:
            duburi.vision.find(sweep='right', timeout=8.0)
            log.info('hit -- aligning')
            duburi.vision.yaw(duration=6.0)
            duburi.vision.forward(distance=0.55, duration=8.0)
            break
        except Exception:
            log.info('no hit, continue patrol')
            duburi.yaw_right(60.0)

    duburi.disarm()
```

### 6.7  Pinned overrides (when you do NOT want the live param)

```python
def run(duburi, log):
    duburi.arm()
    duburi.set_depth(-0.5)
    # An aggressive yaw-only loop with a tighter deadband than the deck default.
    duburi.vision.yaw(duration=10.0, kp_yaw=85.0, deadband=0.06,
                      stale_after=1.2, on_lost='hold')
    duburi.disarm()
```

---

## 7. DVL distance moves

DVL-based closed-loop distance commands are available when `yaw_source` has a
DVL component (`dvl`, `nucleus_dvl`, `bno085_dvl`). Auto-connect means you
don't need `dvl_connect` in the mission — the manager's background thread has
already connected before any mission code runs.

### 7.1  Basic DVL forward move

```python
def run(duburi, log):
    duburi.arm()
    duburi.set_depth(-0.8, settle=1.0)
    # DVL closed-loop: stops exactly 2.0 m from start position
    duburi.move_forward_dist(2.0, gain=60)
    duburi.disarm()
```

### 7.2  Heading-stable DVL translation (recommended pattern)

Lock heading BEFORE starting the DVL move. The lock owns Ch4 (yaw rate) and
holds the AUV pointed at the target while DVL drives Ch5 (forward) or Ch6
(lateral). Do NOT call `unlock_heading` between them.

```python
def run(duburi, log):
    duburi.arm()
    duburi.set_depth(-0.8, settle=1.0)

    # Lock heading at 0° and keep it active throughout
    duburi.lock_heading(target=0.0, timeout=120)

    # DVL moves — heading lock stays alive, no yaw drift
    duburi.move_forward_dist(3.0, gain=60)
    duburi.move_lateral_dist(1.0, gain=36)      # strafe 1 m right
    duburi.move_forward_dist(2.0, gain=60)

    duburi.unlock_heading()
    duburi.disarm()
```

### 7.3  Vision approach + DVL final run

Use vision to centre on the target, then DVL for a precise close-in:

```python
def run(duburi, log):
    duburi.target = 'gate'
    duburi.camera = 'forward'
    duburi.arm()
    duburi.set_depth(-1.0)
    duburi.lock_heading(target=0.0, timeout=120)

    # Phase 1: find and centre on gate
    duburi.vision.find(sweep='right', timeout=30.0)
    duburi.vision.lock(axes='yaw', duration=5.0, lock_mode='settle')

    # Phase 2: update heading lock to current (post-alignment) heading
    duburi.unlock_heading()
    duburi.lock_heading(target=0.0, timeout=120)   # target=0 = lock current

    # Phase 3: drive through gate with DVL precision
    duburi.move_forward_dist(4.0, gain=60)

    duburi.unlock_heading()
    duburi.disarm()
```

### 7.4  Composite BNO+DVL (recommended pool config)

`yaw_source=bno085_dvl` uses BNO085 heading (stable gyro fusion) + DVL position.
Configure in `config/sensors.yaml`:

```yaml
duburi_manager:
  ros__parameters:
    yaw_source: bno085_dvl
```

Mission code is identical — the yaw source selection is transparent to the DSL.

### 7.5  DVL gotchas

- `move_forward_dist` / `move_lateral_dist` call `reset_position()` internally.
  No need to call it explicitly unless building your own control loop.
- If `yaw_source` has no DVL component (e.g. `mavlink_ahrs` or `bno085`), both
  commands fall back to open-loop time estimate and log a warning. They never
  raise; the mission continues.
- Heading lock MUST stay active during DVL moves (see §7.2). Suspending it
  causes the AUV to weather-cock.
- After a long DVL session, if you switch to `yaw_source=bno085` (not
  `bno085_dvl`), restart the manager. The BNO calibration runs at startup using
  the Pixhawk AHRS heading; if the Pixhawk AHRS drifted during the DVL session,
  the BNO offset will be stale.

---

## 8. Designing your own mission, step by step

1. **Pick a stable starting condition.** `arm()` then `set_depth(-0.5)`
   gives you altitude headroom and engages ALT_HOLD.
2. **Decide what each phase isolates.** A phase that drives forward
   blindly is `move_forward`. A phase that lands on a target is
   `vision.lock`. Don't mix them inside one verb; chain them.
3. **Use `lock_heading()` whenever you do open-loop translations.** It
   keeps yaw drift bounded for free. Depth needs no equivalent --
   `set_depth(...)` already hands depth back to ArduSub's onboard
   ALT_HOLD, which holds it for the rest of the mission with zero
   further traffic from us.
4. **Vision verbs first, distance second.** Settle yaw/lat/depth on
   the target *before* committing to a `forward` close-in — otherwise
   the bbox error grows as you approach.
5. **Always provide an exit.** `disarm()` at the end (or in a
   `finally:` block).
6. **Bake gains and deadbands into ROS params, not the mission.** A
   mission that runs the same in pool and bench is a mission that
   trusts the deck.
7. **Small phases > big phases.** Aim for 5–10 verbs per mission,
   each with a clear single intent.

---

## 9. Gotchas

- **A `vision.*` verb with `target=''`** raises — sticky context is
  required, set `duburi.target` once before the first vision verb.
- **`vision.depth` on a downward camera** wants `depth_sign=-1`. The
  verb infers this from the camera name (`'downward'` flips the sign);
  if you use a custom camera ID, override at the manager level.
- **`vision.find(sweep='still')`** does not move. If your target
  isn't in the camera frame at start, this returns a timeout failure
  no matter how long you wait.
- **Open-loop seconds are NOT distances.** Currents and battery
  state change the metres-per-second mapping every run. Use vision
  verbs for precision; use open-loop for *getting close*.
- **`duburi.stop()` between vision verbs is usually unnecessary** —
  every vision verb exits with a neutral RC write. Add `stop` only
  if you need an *extra* settle pause.

---

## 10. Cross-references

- Open-loop verb implementations: `src/duburi_control/duburi_control/{motion_forward,motion_lateral,motion_yaw,motion_depth,heading_lock}.py`
- DVL distance verbs:             `src/duburi_control/duburi_control/{motion_forward,motion_lateral}.py` (`drive_*_dist`)
- DVL sources:                    `src/duburi_sensors/duburi_sensors/sources/{nucleus_dvl,composite_bno_dvl}.py`
- Vision loop body:               `src/duburi_control/duburi_control/motion_vision.py`
- Vision state cache:             `src/duburi_manager/duburi_manager/vision_state.py`
- DSL surface:                    `src/duburi_planner/duburi_planner/duburi_dsl.py`
- ROS param defaults:             `src/duburi_manager/config/vision_tunables.yaml`
- DVL integration reference:      `.claude/context/dvl-integration.md`
- Sensors pipeline design:        `.claude/context/sensors-pipeline.md`
- CLI cookbook (deck one-liners): `README.md` §9
- Architecture (visual flow):     `.claude/context/vision-architecture.md`
- Roadmap (what's next):          `.claude/context/vision-roadmap.md`
