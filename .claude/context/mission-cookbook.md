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
    duburi.vision.find(move='yaw_right', timeout=20.0)
    duburi.vision.home(yaw=True, forward=True, depth=True, dist=0.55, duration=15.0)
    duburi.set_depth(0.0)
    duburi.disarm()
```

That's the whole pattern. The rest of this cookbook fills in **which
verbs exist** and **how to tune them**.

---

## 0.5 Desk / Bench Testing with Pretrained Model (yolov11n + person)

No custom weights yet? Run this on a laptop with a webcam — no pool, no vehicle
needed (use `mode:=sim` or just call the mission runner without arming).

```python
def run(duburi, log):
    # Auto-downloads yolov11n.pt (~5 MB, COCO 80-class) on first run.
    duburi.models(person='yolov11n')
    duburi.camera = 'laptop'

    duburi.arm()
    duburi.set_depth(-0.5)
    duburi.move_forward(3.0, gain=60)

    target = duburi.models.person.person   # ClassRef — sets model + class automatically

    duburi.vision.find(target=target, move='forward', gain=30, timeout=45)
    duburi.vision.home(target=target, yaw=True, lat=True, depth=True, duration=20)
    duburi.disarm()
```

**Pretrained weight aliases** (Ultralytics COCO, auto-downloaded):

| Alias      | Size  | Speed  | Notes                          |
|------------|-------|--------|--------------------------------|
| `yolov11n` | ~5 MB | fast   | Recommended for desk testing   |
| `yolov11s` | ~10 MB| medium | Better accuracy, still laptop  |
| `yolov11m` | ~40 MB| slower | Pool-quality accuracy          |
| `yolov11l` | ~80 MB| slow   | High accuracy, needs GPU       |
| `yolov11x` | ~150 MB| very slow | Maximum accuracy            |

Custom model syntax is **identical** — only the name changes:

```python
duburi.models(gate='gate_flare_medium_100ep')    # custom weights in models/
duburi.models(person='yolov11n')                 # COCO pretrained, auto-download
```

**Run the canonical demo mission:**

```bash
ros2 launch duburi_vision cameras_.launch.py camera:=laptop
ros2 run duburi_planner mission find_person_demo
```

> See `src/duburi_planner/duburi_planner/missions/find_person_demo.py` for the full
> reference mission — it exercises every `duburi.vision.*` verb in sequence.

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
    duburi.vision.find(move='yaw_right')      # search for the target
    duburi.vision.home(yaw=True, forward=True,# closed-loop -- land on it
                       dist=0.55, duration=12)
    duburi.move_back(2.0, gain=60)
    duburi.disarm()
```

That's the entire pattern. The rest of this cookbook fills in the
verbs and tuning.

---

## 2. Hard rules (read them, internalise them)

1. **One axis per command.** A `move_forward` only writes Ch5. A
   `yaw_right` only writes Ch4. Mixing axes is the controller's job
   (`arc` for forward+yaw, `vision.home` for any subset). This is
   AXIS ISOLATION and it is what keeps the sub predictable.

2. **Vision informs control, never fights it.** Vision verbs talk
   *to* the same control stack the open-loop verbs use. They don't
   open a parallel channel. So `duburi.vision.turn(...)` is exactly
   `yaw_right` driven by a bbox-error P loop instead of a clock.

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

Each verb describes what the AUV physically does and maps 1:1 to a
future YASMIN state. All run a closed P loop on the **largest
detection** of `target` in `camera`. Sticky context from the parent
`duburi`; pass `target=` / `camera=` to override per call.

```python
# ── Search ──────────────────────────────────────────────────────────────── #
duburi.vision.find(target=None,
                   move='still',       # 'still'|'forward'|'yaw_right'|'yaw_left'|'arc'
                   gain=25.0,          # forward thrust % (forward / arc)
                   yaw_rate_pct=22.0,  # yaw % (yaw_right / yaw_left / arc)
                   timeout=25.0)       # abort if not seen after this many s

# ── Single-axis verbs ────────────────────────────────────────────────────── #
duburi.vision.turn  (target=None, duration=8.0,  **overrides)  # Ch4: yaw to centre
duburi.vision.slide (target=None, duration=8.0,  **overrides)  # Ch6: slide to centre
duburi.vision.hover (target=None, duration=8.0,  **overrides)  # depth to v-centre
duburi.vision.approach(target=None,
                        dist=0.55,      # bbox fraction to stop at
                        metric='height',# 'height'|'width'|'area'|'diagonal'
                        duration=12.0,
                        pass_at=0.0,    # commit to drive-through once size >= this
                        pass_at_gain=50.0,
                        **overrides)

# ── Multi-axis (home in on target) ──────────────────────────────────────── #
duburi.vision.home(target=None,
                   yaw=True, lat=False, depth=False, forward=False,
                   dist=0.55, metric='height',
                   gate_guard=False,            # suppress forward when gate angled
                   gate_guard_min_w_frac=0.35,
                   pass_at=0.0, pass_at_gain=50.0,
                   duration=15.0, **overrides)

# ── Continuous tracking (never exits on settle) ──────────────────────────── #
duburi.vision.track(target=None,
                    yaw=True, forward=True, lat=False, depth=False,
                    dist=0.55, duration=60.0, **overrides)

# ── Orbit scan (POSHOLD + incremental yaw, exits on first detection) ─────── #
duburi.vision.scan(target=None,
                   step=20.0,       # degrees CW per stop (+ve=right, -ve=left)
                   speed=40.0,      # turn speed %
                   dwell=1.5,       # observe seconds per stop
                   duration=60.0,   # total time budget
                   start_yaw=0.0,   # snap to this heading first (0 = current)
                   **overrides)
```

**Typical competition patterns:**

```python
# Register models at the top of run(); access via duburi.models.alias.class_name
duburi.models(gate='gate_flare_medium_100ep')

# Gate: yaw + lateral with angle guard + committed pass-through
duburi.vision.home(target=duburi.models.gate.gate,
                   yaw=True, lat=True, forward=True,
                   dist=0.42, metric='area',
                   gate_guard=True, pass_at=0.38, pass_at_gain=55,
                   duration=20, on_lost='hold')

# Flare: 3-axis settle (tall narrow pipe, height metric)
duburi.vision.home(target=duburi.models.gate.flare,
                   yaw=True, forward=True, depth=True,
                   dist=0.38, metric='height',
                   duration=20, on_lost='hold')

# Flare orbit re-lock after each yaw step (3 s follow window)
duburi.vision.track(target=duburi.models.gate.flare,
                    yaw=True, forward=True, depth=True,
                    dist=0.38, duration=3, on_lost='hold')

# Torpedo pursue: drive forward-only until target fills 80% of frame
duburi.vision.home(yaw=True, forward=True,
                   dist=0.80, duration=20, lock_mode='pursue')
```

#### `move=` values for `find`

| `move=`       | AUV behaviour while searching      |
| ------------- | ---------------------------------- |
| `'still'`     | wait in place                      |
| `'forward'`   | drive forward at `gain`%           |
| `'yaw_right'` | rotate right at `yaw_rate_pct`%    |
| `'yaw_left'`  | rotate left at `yaw_rate_pct`%     |
| `'arc'`       | forward + yaw simultaneously       |

#### Distance metrics for `approach` / `home`

| `metric=`    | Best for                                  |
| ------------ | ----------------------------------------- |
| `'height'`   | Tall objects: buoy, flare, pole (default) |
| `'width'`    | Wide horizontal objects: bar, banner      |
| `'area'`     | Square-ish or variable: gate, bin         |
| `'diagonal'` | Best all-rounder when shape is uncertain  |

#### Overrides (only when you mean to pin a value)

Prefer leaving these out — the live `vision.*` ROS param applies and
lets you tune from the deck without a rebuild:

```bash
ros2 param set /duburi_manager vision.kp_yaw     80.0
ros2 param set /duburi_manager vision.deadband    0.08
ros2 param set /duburi_manager vision.target_bbox_h_frac 0.55
```

| Override             | Default param       | Effect                                                    |
| -------------------- | ------------------- | --------------------------------------------------------- |
| `kp_yaw`             | `vision.kp_yaw`     | Proportional gain on `ex`, Ch4 percent units              |
| `kp_lat`             | `vision.kp_lat`     | Same on Ch6                                               |
| `kp_depth`           | `vision.kp_depth`   | Metres of nudge per unit `ey` per 5 Hz tick               |
| `kp_forward`         | `vision.kp_forward` | Ch5 percent per unit (target_h_frac − h_frac)             |
| `deadband`           | `vision.deadband`   | Per-axis settle band; `\|err\| < deadband` counts as in  |
| `distance_metric`    | `vision.distance_metric` | `'height'` (poles, buoys), `'width'` (wide bars), `'area'` (gates), `'diagonal'` |
| `on_lost`            | `vision.on_lost`    | `'fail'` (abort after ~0.6 s lost) or `'hold'` (freeze setpoints) |
| `lock_mode`          | `vision.lock_mode`  | `'settle'` (exit on centred), `'follow'` (run full duration), `'pursue'` (forward-only until close) |
| `gate_guard`         | —                   | `True` = suppress forward when gate bbox appears angled (w/h < threshold) |
| `gate_guard_min_w_frac` | —               | Aspect ratio threshold for gate_guard (default 0.35; calibrate at pool) |
| `pass_at`            | —                   | Commit to straight drive-through once size metric >= this value (0 = off) |
| `pass_at_gain`       | —                   | Forward thrust % during committed pass phase (default 50%) |
| `stale_after`        | `vision.stale_after`| Seconds after which a detection is treated as lost        |
| `tracking`           | `vision.use_tracks` | `True` = use ByteTrack IDs + Kalman-smoothed bboxes       |

A vision verb settles when all active axes are within `deadband` for 4
consecutive ticks (0.2 s @ 20 Hz). `on_lost='fail'` aborts after 12
consecutive lost ticks (0.6 s).

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
duburi.vision.home(target='gate', yaw=True, forward=True,
                   tracking=True, ...)
```

`tracking=True` tells `auv_manager_node` to subscribe `/tracks` for this
goal instead of `/detections`. If `tracker_node` is not running the goal
will stall waiting for its first sample — always pair with `with_tracking:=true`.

### `lock_mode` — how `home` / `track` exit

`lock_mode` controls when `vision.home` exits (pass via `**overrides`).
`vision.track` always uses `'follow'` internally.

| `lock_mode` | Exits when ...                                       | Use case                               |
|-------------|------------------------------------------------------|----------------------------------------|
| `'settle'`  | all axes inside deadband for `SETTLED_TICK_BUDGET`   | Approach and stop at standoff          |
| `'follow'`  | `duration` expires (never exits on deadband alone)   | Continuous tracking, orbit hold steps  |
| `'pursue'`  | forward axis reaches `dist` fraction (never backs off) | Torpedo: drive to target fill        |

**Rule of thumb:** use `settle` to *arrive*; `follow` to *keep moving*; `pursue` to *close in*.

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
2. `vision.track(..., duration=3.0)` — re-centre and hold for 3 seconds,
   then exit and go to the next step

```python
ORBIT_STEPS    = 12      # 12 × 30° = 360°
ORBIT_STEP_DEG = 30.0
ORBIT_HOLD_S   = 3.0

for step in range(ORBIT_STEPS):
    duburi.yaw_left(ORBIT_STEP_DEG, timeout=10.0, settle=0.3)
    # track: keep tracking for the full 3 s window,
    # never exit early on deadband (lock_mode='follow' internally).
    duburi.vision.track(
        target=m.gate.flare,
        yaw=True, forward=True, depth=True,
        dist=0.40,
        duration=ORBIT_HOLD_S,
        on_lost='hold',       # survive brief occlusions
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
    duburi.vision.track(
        target='buoy', yaw=True,
        duration=2.0,
        tracking=True,
    )
```

For pure heading-correction while driving forward, `track(yaw=True)` with
a short `duration` is enough:

```python
duburi.vision.track(target='gate', yaw=True,
                    duration=20.0, tracking=True)
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
# ── Sim / webcam / bench (ROBOSUB-tested pretrained ★) ──────────────────────
# yolov11n detects COCO 80 classes — use 'person' to test move_and_see
ros2 launch duburi_vision cameras_.launch.py model:=yolov11n classes:=person

# Or single-command launch+display:
ros2 run duburi_vision vision_display --ros-args \
    -p launch_pipeline:=true -p model:=yolov11n -p classes:=person

# ── Pool / competition custom models ─────────────────────────────────────────
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
# Manual control (only needed when NOT using duburi.models ClassRef targets):
duburi.set_classes('gate')
duburi.set_classes('flare')
duburi.set_classes('gate,flare')
duburi.set_classes('')          # all classes
duburi.set_classes(['gate', 'flare'])  # list form also accepted
# When using duburi.models.gate.gate as target=, set_classes is called automatically.
```

### Full gate+flare prequal mission pattern

```python
def run(duburi, log):
    duburi.camera = 'forward'
    duburi.models(gate='gate_flare_medium_100ep')   # register once; use anywhere

    # Tether removal window -- operator disconnects tether during countdown
    duburi.countdown(10)

    duburi.arm()
    duburi.set_mode('ALT_HOLD')
    duburi.set_depth(-1.0, settle=2.0)
    duburi.dvl_connect()

    # Gate phase — ClassRef auto-switches model+class in each verb call
    duburi.vision.find(target=duburi.models.gate.gate, move='forward', timeout=45.0, gain=40.0)
    duburi.vision.home(target=duburi.models.gate.gate, yaw=True, forward=True,
                       dist=0.42, metric='area',
                       duration=20.0, on_lost='hold')
    duburi.move_forward_dist(3.5, gain=60.0)

    # Flare phase
    duburi.vision.find(target=duburi.models.gate.flare, move='yaw_right', timeout=40.0, gain=0.0)
    duburi.vision.home(target=duburi.models.gate.flare, yaw=True, forward=True, depth=True,
                       dist=0.38, metric='height',
                       duration=20.0, on_lost='hold')

    # Orbit flare 360 degrees
    for _ in range(12):
        duburi.yaw_left(30.0, timeout=10.0, settle=0.3)
        duburi.vision.track(target=duburi.models.gate.flare,
                            yaw=True, forward=True, depth=True,
                            dist=0.38, duration=3.0, on_lost='hold')

    # Return through gate
    duburi.yaw_right(180.0, timeout=25.0, settle=0.5)
    duburi.vision.find(target=duburi.models.gate.gate, move='yaw_right', timeout=30.0, gain=0.0)
    duburi.vision.home(target=duburi.models.gate.gate, yaw=True, forward=True,
                       dist=0.42, metric='area',
                       duration=20.0, on_lost='hold')
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
    duburi.vision.find(move='yaw_right', timeout=25.0)
    duburi.vision.home(yaw=True, forward=True, dist=0.55, duration=12.0)
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
            duburi.vision.find(move='arc', timeout=20.0)
            duburi.vision.home(yaw=True, forward=True, dist=0.55, duration=10.0)
            log.info('homed & held')
            break
        except Exception as exc:
            log.warning(f'lost target ({exc!r}) -- backing off & retrying')
            duburi.move_back(2.0, gain=50)
            duburi.yaw_right(45.0)

    duburi.disarm()
```

### 6.5  Full 4-axis lock (yaw + lat + depth + forward at once)

```python
def run(duburi, log):
    duburi.target = 'gate'
    duburi.arm()
    duburi.set_depth(-1.5)
    duburi.lock_heading()
    duburi.vision.find(move='still', timeout=30.0)
    duburi.vision.home(yaw=True, lat=True, depth=True, forward=True,
                       dist=0.50, duration=20.0)
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
            duburi.vision.find(move='yaw_right', timeout=8.0)
            log.info('found -- homing in')
            duburi.vision.turn(duration=6.0)
            duburi.vision.approach(dist=0.55, duration=8.0)
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
    # Aggressive yaw-only with tighter deadband than the deck default.
    duburi.vision.turn(duration=10.0, kp_yaw=85.0, deadband=0.06,
                       stale_after=1.2, on_lost='hold')
    duburi.disarm()
```

### 6.8  Camera switching mid-mission (forward → downward)

`duburi.camera` is a sticky string. Assign it once; all subsequent `duburi.vision.*` calls use that
camera. The camera name must match a running `camera_node` profile — verify with
`ros2 topic list | grep image_raw`.

```python
def run(duburi, log):
    duburi.arm()
    duburi.set_depth(-0.8, settle=1.0)

    # Phase 1: gate approach with forward camera
    duburi.camera = 'forward'
    duburi.target = 'gate'
    duburi.vision.home(yaw=True, lat=True, dist=0.6, duration=15)
    duburi.move_forward(duration=4, gain=60)

    # Phase 2: descend and switch to downward camera for bin
    duburi.set_depth(-1.5, settle=1.5)
    duburi.camera = 'downward'
    duburi.target = 'bin'
    # axes must be set explicitly — see §6.9
    duburi.vision.home(yaw=False, lat=True, forward=True, dist=0.85, duration=20)
    duburi.disarm()
```

### 6.9  Downward-camera alignment (bin centering)

When the camera faces downward, `ex`/`ey` map to lateral/forward motion — not yaw/depth as they
do for a forward-facing camera. **This axis remap is NOT automatic.** You must pass the correct
boolean flags to `vision.home()` so the loop drives the right channels:

```python
duburi.camera = 'downward'
duburi.target = 'bin'

# CORRECT: lat+forward drives vehicle over the bin; depth locked; yaw skipped
duburi.vision.home(
    yaw=False,      # top-down view: yaw from bbox is unreliable
    lat=True,       # ex → Ch6 lateral (left/right over bin)
    forward=True,   # ey → Ch5 forward (fore/aft over bin)
    depth=False,    # depth already set; don't let bbox height pull depth
    dist=0.85,      # target bbox fill fraction (0=small/far, 1=large/close)
    duration=20,
)

# WRONG (default home axes include yaw+depth, skip lat+forward):
# duburi.vision.home(dist=0.85, duration=20)  # yaws and changes depth instead of centering
```

The only automatic downward adaptation is `depth_sign=-1` (negates the depth correction). Everything
else is caller's responsibility. Phase 3 will automate the axis remap when `camera='downward'`.

### 6.10  Offset alignment (merged — available on all PID verbs)

`offset_x` and `offset_y` keep the target a fixed number of pixels away from frame centre.
Positive `offset_x` = target stays to the RIGHT; positive `offset_y` = target stays BELOW.
Normalization is done internally (`norm = offset / (image_dim * 0.5)`), clamped to ±1.5.

**Slalom side-of-pipe pass** — keep the pipe 80 px right while driving forward:
```python
duburi.vision.turn(target='slalom_red', offset_x=80, duration=4.0)
duburi.move_forward(duration=3.0, gain=40)
```

**Torpedo board bullseye** — aim slightly right and up from bbox centre:
```python
duburi.vision.home(target='torpedo_board',
                   yaw=True, lat=True, depth=True,
                   offset_x=60,    # aim 60 px right of board bbox centre
                   offset_y=-40,   # aim 40 px above board bbox centre
                   duration=15)
```

All PID verbs support `offset_x`/`offset_y`: `vision_align_yaw`, `vision_align_lat`,
`vision_align_depth`, `vision_hold_distance`, `vis_approach`, `vision_align_3d`.
`vision_acquire` and `look_around` have no PID loop — offsets are ignored.

### 6.11  Dubomini — vision-held position (no DVL)

Dubomini 2.0 has no DVL. Use `vision_align_3d` with lat+yaw+depth axes to hold station on a
detected target. Launch with `yaw_source:=bno085` and `dvl_auto_connect:=false`.

```python
# Dubomini bringup: ros2 run duburi_manager start
#   --ros-args -p yaw_source:=bno085 -p dvl_auto_connect:=false

def run(duburi, log):
    duburi.arm()
    duburi.set_depth(-0.6, settle=1.0)
    duburi.target = 'gate'

    # Hold station on gate using vision (replaces POSHOLD — Dubomini has no DVL)
    duburi.vision.home(yaw=True, lat=True, depth=True, forward=False,
                       dist=0.5, duration=30, on_lost='hold')

    # Approach: forward open-loop (no DVL dist metric available)
    duburi.move_forward(duration=5, gain=55)
    duburi.disarm()
```

VehicleProfile auto-selects Dubomini vs Duburi 4.5 at runtime via `VehicleProfile.auto()` —
`has_dvl=False` disables DVL-distance verbs so any call to `move_forward_dist` will raise early
rather than silently produce open-loop motion.

### 6.12  Torpedo firing (vision_lock_fire)

`vision_lock_fire` aligns on multiple axes, verifies stable hold for `stable_lock_s`, then fires
via the ESP32 `PayloadDriver`. Retries up to `max_attempts`; fires at last pose on total failure.

```python
def run(duburi, log):
    duburi.arm()
    duburi.set_depth(-1.2)

    # Coarse approach
    duburi.vision.find(target='torpedo_hole', move='forward', gain=35, timeout=45)
    duburi.vision.home(target='torpedo_hole', yaw=True, lat=True, depth=True,
                       duration=20, on_lost='hold')

    # Lock + fire
    result = duburi.vision.vision_lock_fire(
        target='torpedo_hole',
        yaw=True, lat=True, depth=True,
        stable_lock_s=4.0,       # hold in deadband 4 s before fire
        max_attempts=3,
        fire_channel=1,          # torpedo_1 (ESP32 serial)
        attempt_timeout=20.0,
        duration=60.0)

    if result.success:
        log.info('torpedo fired on stable lock')
    else:
        log.warning('fallback fire at last aim pose')

    duburi.move_forward(duration=2, gain=40)   # clear the board
    duburi.disarm()
```

**Key fields:**

| Param | Notes |
|---|---|
| `fire_channel` | 1/2=torpedo, 3/4=dropper. **Preferred over `fire_aux_channel`**. |
| `stable_lock_s` | Seconds all axes must be in deadband before fire (default 3.0 s) |
| `max_attempts` | Retry count; fallback fires at last pose if all attempts fail |
| `offset_x/y` | Aim offset from bbox centre (e.g. for off-centre bullseye) |

---

### 6.13  Bin drop (downward camera + dropper lock-fire)

Axis remap: downward camera maps `ex→lat`, `ey→forward`. Pass `lat=True, forward=True, yaw=False`.

```python
def run(duburi, log):
    duburi.arm()
    duburi.set_depth(-1.5)

    # Fly over and find the bin
    duburi.camera = 'downward'
    duburi.vision.find(target='fire_bin', move='still', timeout=30)

    # Drop — lat+forward centering, then fire dropper when stable
    result = duburi.vision.vision_lock_fire(
        target='fire_bin',
        yaw=False, lat=True, forward=True, depth=False,
        stable_lock_s=3.0,
        max_attempts=2,
        fire_channel=3,          # dropper_1
        duration=45.0)

    duburi.disarm()
```

> The downward camera's `ey→forward` remap is applied automatically by `_run_vision_track`
> when `camera='downward'` — you don't pass `forward_uses_ey=True` explicitly.

---

### 6.14  Hold pattern (maintain lock without exiting)

`vision.hold()` runs `lock_mode='follow'` (never exits on settle). Use for timed holds,
waiting for an external trigger, or as a positioning phase before a fire sequence.

```python
# Hold position on target for 5 s, then do something else
duburi.vision.hold(target='gate', yaw=True, lat=True, duration=5.0)
duburi.move_forward(duration=2.0, gain=60)

# Hold while the dropper loads (timed)
duburi.vision.hold(target='bin', yaw=False, lat=True, forward=True, duration=3.0)
duburi.fire(fire_channel=3)   # manual fire after hold
```

**Trilogy:**
- `vision.home()` — align, exit once settled in deadband
- `vision.hold()` — align, keep running for `duration` (never exits on settle)
- `vision.vision_lock_fire()` — align, maintain, fire when stable

---

## 7. DVL distance moves

DVL-based closed-loop distance commands are available when `yaw_source` has a
DVL component (`dvl`, `nucleus_dvl`, `bno085_dvl`). Auto-connect means you
don't need `dvl_connect` in the mission — the manager's background thread has
already connected before any mission code runs.

### 7.1  Basic DVL forward / back move

```python
def run(duburi, log):
    duburi.arm()
    duburi.set_depth(-0.8, settle=1.0)
    # DVL closed-loop: stops exactly 2.0 m from start position
    duburi.move_forward_dist(2.0, gain=60)
    # Return to start (same tolerance applies — stops when back within 0.1 m)
    duburi.move_back_dist(2.0, gain=60)
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
    duburi.vision.find(move='yaw_right', timeout=30.0)
    duburi.vision.turn(duration=5.0)

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

### 7.5  Orbit scan — `vision.scan()` / `look_around`

`scan()` switches to POSHOLD, locks position, then pivots in incremental yaw
steps. At each stop it watches the detection topic for `dwell` seconds.
The moment the target class appears, it exits with `result.success=True`.
Falls back to ALT_HOLD + heading lock if POSHOLD is unavailable.

```python
def run(duburi, log):
    duburi.models(gate='gate_flare_medium_100ep')
    duburi.camera = 'forward'
    duburi.arm()
    duburi.set_depth(-1.0, settle=1.0)

    # Orbit right in 20° steps until gate is found (or 90 s budget spent)
    result = duburi.vision.scan(
        target=duburi.models.gate.gate,
        step=20,       # degrees CW per stop
        dwell=1.5,     # seconds to observe at each stop
        speed=40,      # turn speed %
        duration=90)

    if result.success:
        log(f'gate found after {result.final_value:.0f}° sweep — aligning')
        duburi.vision.home(target=duburi.models.gate.gate,
                           yaw=True, lat=True, gate_guard=True, duration=10)
    else:
        log('gate not found — advancing and retrying')
        duburi.move_forward(3.0, gain=35)
```

CLI equivalent:
```bash
ros2 run duburi_planner duburi look_around \
    --camera forward --target_class gate \
    --yaw_rate_pct 20 --settle 1.5 --gain 40 --duration 90
```

### 7.6  Detection guards — `duburi.detected()` paradigm

`duburi.detected(class, *, camera=None, stale_after=1.0) -> bool` is a
**non-blocking, cache-backed observation query**. It reads a local Python
dict that is refreshed automatically during every blocking DSL verb (the
verb's `rclpy.spin_until_future_complete` call fires pending ROS callbacks,
including the `/detections` subscription).

This enables a new class of mission design: **the AUV executes open-loop
maneuvers *until* a target comes into view, then hands off to vision-closed
control**. This is the architecture step toward YASMIN FSMs — each
`while detected()` loop IS a proto-state.

**Full deep-dive reference:** [`.claude/context/detected-paradigm.md`](./detected-paradigm.md)

#### Core paradigm

```python
def run(duburi, log):
    duburi.camera = 'forward'
    duburi.models(gate='gate_flare_medium_100ep')
    duburi.arm()
    duburi.set_depth(-0.8)
    duburi.lock_heading(0.0, timeout=180)

    # ── Search: creep forward until gate visible ──────────────────────── #
    MAX_STEPS = 60   # safety budget: 60 × 0.5s = 30s of search
    for _ in range(MAX_STEPS):
        if duburi.detected(duburi.models.gate.gate, stale_after=0.5):
            break
        duburi.move_forward(0.5, gain=30)  # 0.5s steps → max 0.15m overshoot
    else:
        log.warn('gate not found — aborting')
        duburi.set_depth(0.0); duburi.disarm(); return

    # ── Align and pass ────────────────────────────────────────────────── #
    duburi.vision.home(
        target=duburi.models.gate.gate,
        yaw=True, lat=True,
        gate_guard=True, pass_at=0.38, pass_at_gain=55,
        dist=0.40, metric='area', duration=20,
    )
    duburi.move_forward_dist(3.0, gain=60)

    # ── Search: yaw-sweep for flare ────────────────────────────────────── #
    duburi.set_classes('gate,flare')          # must set BEFORE detecting flare
    for _ in range(36):                       # 36 × 10° = full 360°
        if duburi.detected('flare', stale_after=0.5):
            break
        duburi.yaw_right(10); duburi.pause(0.5)

    if duburi.detected('flare', stale_after=0.5):
        duburi.vision.home(
            target=duburi.models.gate.flare,
            yaw=True, forward=True, depth=True,
            dist=0.38, metric='height', duration=20,
        )

        # ── Orbit flare: exit when gate re-appears ─────────────────────── #
        # IMPORTANT: vision.home above called set_classes('flare').
        # Restore both BEFORE the orbit loop.
        duburi.set_classes('gate,flare')
        for _ in range(18):                   # 18 × 20° = 360°
            if duburi.detected('gate', stale_after=0.3):
                break
            duburi.yaw_right(20); duburi.pause(1.0)

        if duburi.detected('gate', stale_after=0.5):
            duburi.vision.home(target=duburi.models.gate.gate,
                               yaw=True, lat=True, gate_guard=True, duration=15)
            duburi.move_forward_dist(1.5, gain=60)

    duburi.unlock_heading()
    duburi.set_depth(0.0)
    duburi.disarm()
```

#### The four rules you must not break

**Rule 1 — Short steps (0.5 s or less).** Detection check fires only after
the verb returns. At `gain=30` (~0.3 m/s), a `move_forward(2.0)` step means
~0.6 m of overshoot past the detection point. Use 0.3–0.5 s steps.

**Rule 2 — Always have a safety budget.** If the detector is offline or the
target never appears, an unbounded `while` loop runs forever. Use a `for`
loop with `MAX_STEPS`.

**Rule 3 — Class filter coupling.** Any `vision.*` verb that takes a `ClassRef`
calls `set_classes()` automatically. After `vision.home(target=flare_ref)`, the
detector publishes flare detections only. `detected('gate')` will always return
`False` until you call `duburi.set_classes('gate,flare')`.

**Rule 4 — Set the camera first.** `detected()` falls back to `duburi.camera`
(default `'laptop'`). Set `duburi.camera = 'forward'` at the top of `run()`.

#### What blocks vs what doesn't

**Blocking (keeps cache warm via spin_until_future_complete):**
`move_forward`, `move_back`, `move_left`, `move_right`, `yaw_left`, `yaw_right`,
`arc`, `set_depth`, `arm`, `disarm`, `pause`, `stop`, `lock_heading`,
`dvl_connect`, `move_forward_dist`, `move_lateral_dist`, ALL `vision.*` verbs.

**Non-blocking (cache NOT updated):**
`duburi.camera =`, `duburi.target =`, `duburi.models(...)`.

`detected()` itself calls `spin_once(timeout=0.05)` internally — so even
without a preceding blocking verb, it always fires at least one callback cycle
before checking the cache. But use it after a blocking verb for freshest results.

#### Forbidden patterns

```python
# ✗ Step too long — 0.6m overshoot at gain=30
while not duburi.detected('gate'):
    duburi.move_forward(2.0, gain=30)

# ✗ No safety budget — runs forever if detector offline
while not duburi.detected('gate'):
    duburi.move_forward(0.5)

# ✗ Class filter trap — vision.home set classes='flare', gate never detected
duburi.vision.home(target=duburi.models.gate.flare, ...)
for _ in range(18):
    if duburi.detected('gate'):   # ALWAYS FALSE
        break

# ✗ Wrong camera — subscribes laptop/detections instead of forward/detections
while not duburi.detected('gate'):   # duburi.camera still 'laptop'
    ...
```

#### `detected()` + `result.success` together (full decision tree)

```python
# Try vision find; branch on whether it succeeded
result = duburi.vision.find(target='gate', move='forward', timeout=30)
if result.success:
    duburi.vision.home(target='gate', yaw=True, lat=True)
    duburi.move_forward_dist(3.0, gain=60)
else:
    log.warn('gate not found via vision.find — advancing blindly')
    duburi.move_forward(4.0, gain=35)

# Confirm target still visible after maneuver
duburi.yaw_right(45)
if duburi.detected('gate', stale_after=0.5):
    duburi.vision.home(target='gate', yaw=True, lat=True)
elif duburi.detected('flare', stale_after=0.5):
    duburi.vision.home(target='flare', yaw=True, depth=True)
else:
    log.warn('no targets visible after yaw step')
```

#### API quick reference

```python
duburi.detected(
    target_class,             # str | ClassRef — e.g. 'gate', duburi.models.gate.gate
    *,
    camera: str | None = None,      # defaults to duburi.camera
    stale_after: float = 1.0,       # seconds; detections older than this → False
) -> bool
```

| `stale_after` | Use case |
|---------------|----------|
| 0.3 | Orbit gate-break: want fresh confirmation |
| 0.5 | Standard search loop step |
| 1.0 | Default; fine for most uses |
| 2.0 | Target flickers (turbid water); ride out drop-outs |

#### Testing detected() live

```bash
# 1. Confirm detection topic streaming
ros2 topic hz /duburi/vision/forward/detections   # should be 15-25 Hz

# 2. Confirm class names match your code strings (case-sensitive)
ros2 topic echo /duburi/vision/forward/detections --once   # look for class_id

# 3. Confirm class filter correct
ros2 param get /duburi_detector classes   # should be 'gate' or 'gate,flare'

# 4. Test one detected() call from CLI (Python one-liner approach):
ros2 run duburi_planner mission detected_test   # see detected-paradigm.md §8.2
```

Full testing guide: [`.claude/context/detected-paradigm.md §8`](./detected-paradigm.md).

### 7.7  DVL gotchas

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

## 7.5  Competition mission chunks (RoboSub 2026)

The full 2026 competition run is built as five standalone chunk files directly
in `missions/` plus a combinator in `missions/full_mission_2026.py`.
Each chunk can be run independently with `ros2 run duburi_planner mission <name>`.
The flat layout is required: `discover()` only globs `missions/*.py` (non-recursive).

### Key patterns introduced by the competition missions

#### Lazy detector activation (`pause_detector` / `resume_detector`)

Both cameras are always streaming. Detectors start `paused=True` in the
competition launch — inference only runs for the task that needs it.

```python
# resume forward detector before gate search
duburi.resume_detector('forward')   # → ros2 param set /duburi_detector_fwd paused false

# ... gate task body ...

# pause again when done (frees GPU for the next chunk's detector)
duburi.pause_detector('forward')    # → ros2 param set /duburi_detector_fwd paused true
```

#### Dual-camera node naming — always pass `node=`

The competition launch creates `/duburi_detector_fwd` and `/duburi_detector_dwn`.
The DSL defaults to `/duburi_detector` (single-camera launch). Always pass `node=`
explicitly in dual-cam missions:

```python
duburi.set_model('gate_rescue_repair', node='/duburi_detector_fwd')
duburi.set_classes('gate,rescue,repair', node='/duburi_detector_fwd')
duburi.resume_detector('forward')                     # camera arg drives the node name
```

#### Bounded search — never infinite forward

Every search loop has an explicit budget and a `look_around` fallback:

```python
MAX_STEPS = 20
for _ in range(MAX_STEPS):
    if duburi.detected('gate', stale_after=1.0):
        break
    duburi.move_forward(1.5, gain=40)    # 1.5 s steps — short enough to re-check
else:
    duburi.vision.scan(target_class='gate', duration=60)  # look_around orbit
```

#### Gate pass with bbox-fill exit (`lock_mode='pursue'`)

The gate chunk exits forward drive when the gate bbox fills 80% of the frame:

```python
duburi.vision.approach(target='gate', dist=0.80,
                       metric='height', duration=25, lock_mode='pursue')
# exits automatically when gate fills 80% of frame height = "through"
```

#### Downward camera centering (bin task) — kp_forward MUST be negative

With the downward camera, `ey > 0` means the target is *aft* of the AUV
(below-frame = must move backward). The `kp_forward` gain must be negative:

```python
duburi.vision.home(target='fire', lat=True, forward=True, yaw=False, depth=False,
                   downward_cam=True,
                   kp_forward=-60.0,   # ← NEGATIVE: ey>0 = target is behind us
                   kp_lat=60.0,
                   deadband=0.06, duration=20)
```

#### Fire channels — always explicit

`fire_channel` has a default but never rely on it. Name the channel in every call:

```python
duburi.fire(3)                        # dropper_1 — always explicit
duburi.vision.vision_lock_fire(
    target='hole', fire_channel=1,    # torpedo_1 — always explicit
    ...)
```

### Pool-day constants (`competition_config.py`)

All depths, headings, bbox fractions, and search budgets live in one file:

```python
from duburi_planner.missions.competition_config import (
    GATE_SEARCH_DEPTH_M,    # -0.4  — initial mission depth
    GATE_PASS_DEPTH_M,      # -0.6  — depth for gate opening
    GATE_PASS_BBOX_FRAC,    # 0.80  — gate height fill = "through"
    BIN_DEPTH_M,            # -1.0  — downward cam clear of obstruction
    TORPEDO_DEPTH_M,        # None  — fill at pool (align with hole)
    SEARCH_MAX_STEPS,       # 20    — safety budget for all search loops
)
```

Set `SLALOM_HEADING_DEG`, `BIN_HEADING_DEG`, `TORPEDO_HEADING_DEG`,
`RETURN_HEADING_DEG` to `None` initially — fill them at pool from compass
readings after navigation.

### Full mission combinator

```python
# missions/full_mission_2026.py
# Chunks are loaded via importlib for hot-reload support (same mechanism as discover()).
# Relative imports don't work under spec_from_file_location without package context.
import importlib.util
from pathlib import Path
from duburi_planner.missions.competition_config import GATE_SEARCH_DEPTH_M

def _chunk(name):
    py = Path(__file__).parent / f'{name}.py'
    spec = importlib.util.spec_from_file_location(f'duburi_planner.missions.{name}', py)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load chunk: {py}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

_gate, _slalom, _bin, _torpedo, _return = (
    _chunk(n) for n in ('gate_task','slalom_task','bin_task','torpedo_task','return_task'))

def run(duburi, log=None):
    try:
        duburi.arm()
        duburi.set_depth(GATE_SEARCH_DEPTH_M, timeout=30)
        duburi.lock_heading(target=0.0, timeout=600)  # BNO lock for full run
        _gate.run(duburi); _slalom.run(duburi); _bin.run(duburi)
        _torpedo.run(duburi); _return.run(duburi)
    except Exception as exc:
        if log: log(f'[MISSION] ABORT: {exc}')
        raise
    finally:
        duburi.unlock_heading(); duburi.stop(); duburi.disarm()
```

### Individual chunk test commands

```bash
# ✅ runnable today (gate_rescue_repair.pt exists)
ros2 run duburi_planner mission gate_task
ros2 run duburi_planner mission return_task

# ⏳ logic test (model missing — verify search + timeout + abort flow)
ros2 run duburi_planner mission slalom_task
ros2 run duburi_planner mission bin_task
ros2 run duburi_planner mission torpedo_task

# full 5-task combinator (all chunks in sequence)
ros2 run duburi_planner mission full_mission_2026
```

See `testing-guide.md §3` for per-chunk expected outputs and `models/README.md §Competition models` for model status.

---

## 8. Designing your own mission, step by step

1. **Pick a stable starting condition.** `arm()` then `set_depth(-0.5)`
   gives you altitude headroom and engages ALT_HOLD.
2. **Decide what each phase isolates.** A phase that drives forward
   blindly is `move_forward`. A phase that lands on a target is
   `vision.home`. Don't mix them inside one verb; chain them.
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
- **Downward camera: only `depth_sign` is automatic.** When
  `camera='downward'`, the loop flips `depth_sign=-1` so depth
  correction stays sensible. **Full axis remapping (`ex`→lateral,
  `ey`→forward) is NOT automatic** — you must pass `yaw=False,
  lat=True, forward=True, depth=False` explicitly (see §6.9). Omitting
  these flags will yaw/adjust-depth instead of centering over the bin.
  If you use a custom camera ID other than `'downward'`, the
  `depth_sign` flip also does not apply — override at the manager level.
- **`vision.find(move='still')`** does not move. If your target
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
