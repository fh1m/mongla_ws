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


> ## ⛔ Read this before copying a recipe
>
> This cookbook predates the move to the SROT board, and some recipes still show the older
> backend's idioms. The **DSL verbs are unchanged** — what changed is where they run:
>
> - **`lock_heading` is refused on srot.** The board holds heading itself at 500 Hz. Guard it:
>   `if duburi.backend != 'srot': duburi.lock_heading(0.0)`
> - **`ALT_HOLD` is not a mode on this board.** `set_depth` works; the board owns depth. A
>   recipe that engages ALT_HOLD explicitly is describing the legacy path.
> - **`move_*_dist`, `arc` and `style_yaw` are refused on srot.** Use the timed moves, or the
>   downward camera's own distance bracket (`calc_distance`).
> - **On the downward camera, the depth-setpoint axes are refused**: sideways and fore/aft
>   run normally.
>
> The current contract is [`command-reference.md`](command-reference.md); the legacy path is
> [`legacy-pixhawk-and-sitl.md`](legacy-pixhawk-and-sitl.md).

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
    duburi.camera = 'forward'
    duburi.arm()
    duburi.set_depth(-1.0)
    # align: centre the gate (yaw + lat); sweep to find it if it's not in frame.
    duburi.vision.align('gate', yaw=0, lat=0, gain=30, duration=20,
                        fallback=sweep_for_gate)
    # move: drive forward until the gate fills 80% of the frame, then we're through.
    duburi.vision.move('gate', fwd=80, mode='area', gain=35, duration=20)
    duburi.set_depth(0.0)
    duburi.disarm()


# Search pattern, defined at the BOTTOM of the file (pure open-loop control).
# Runs automatically on target loss, then align() re-enters — all inside duration.
def sweep_for_gate(duburi, should_stop):
    for _ in range(6):
        if should_stop():       # True the moment the gate reappears
            return
        duburi.yaw_right(15)
        duburi.pause(0.4)
```

That's the whole pattern: **two vision verbs** (`align` then `move`) and a
mission-authored `fallback` search. The rest of this cookbook fills in **how
the two verbs behave** and **how to tune them**.

---

## 0.5 Desk / Bench Testing with Pretrained Model (yolov11n + person)

No custom weights yet? Run this on a laptop with a webcam — no pool, no vehicle
needed (use `mode:=sim` or just call the mission runner without arming).

```python
def run(duburi, log):
    duburi.mission_reset()
    # Auto-downloads yolov11n.pt (~5 MB, COCO 80-class) on first run.
    duburi.models(person='yolov11n')
    duburi.camera = 'laptop'

    duburi.arm()
    duburi.set_depth(-0.5)

    target = duburi.models.person.person   # ClassRef — sets model + class automatically

    # align: yaw-centre the person; sweep to find them if they're not in frame.
    duburi.vision.align(target, yaw=0, err=50, gain=30, duration=12,
                        fallback=sweep_yaw)
    # move: drive in until the person fills 55% of the frame height.
    duburi.vision.move(target, fwd=55, mode='height', gain=35, duration=15,
                       fallback=creep_forward)
    duburi.disarm()


# Search patterns — pure open-loop control, defined at the bottom of the file.
def creep_forward(duburi):
    duburi.move_forward(0.6, gain=30)

def sweep_yaw(duburi, should_stop):
    for _ in range(6):
        duburi.yaw_right(30)
        if should_stop():       # bail the moment the person reappears
            return
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
ros2 run duburi_planner mission demo_find_person
```

> See `src/duburi_planner/duburi_planner/missions/demo_find_person.py` for the full
> reference mission — it exercises both vision verbs (`align` + `move`) plus the
> mission-authored fallback search.

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
    duburi.camera = 'forward'
    duburi.arm()
    duburi.set_depth(-0.5)
    duburi.move_forward(3.0, gain=60)               # open-loop -- get to the area
    duburi.vision.align('gate', yaw=0, lat=0,       # closed-loop -- centre on it
                        gain=30, duration=20)
    duburi.vision.move('gate', fwd=80, mode='area', # closed-loop -- drive in
                       gain=35, duration=20)
    duburi.move_back(2.0, gain=60)                  # open-loop -- withdraw
    duburi.disarm()
```

That's the entire pattern: open-loop to *go somewhere*, then the two vision
verbs (`align` to *centre*, `move` to *close in*) to *land precisely*. The
rest of this cookbook fills in the two verbs and their tuning.

---

## 2. Hard rules (read them, internalise them)

1. **One axis per command.** A `move_forward` only writes Ch5. A
   `yaw_right` only writes Ch4. Mixing axes is the controller's job
   (`arc` for forward+yaw, `vision.align` for any subset of
   lat/yaw/depth). This is AXIS ISOLATION and it is what keeps the
   sub predictable.

2. **Vision informs control, never fights it.** The two vision verbs
   talk *to* the same control stack the open-loop verbs use. They
   don't open a parallel channel. So `duburi.vision.align(t, yaw=0)`
   is exactly `yaw_right` driven by a bbox-error P loop instead of a
   clock, and `duburi.vision.move(t)` is `move_forward` shaped by bbox
   fill instead of a stopwatch.

3. **Sticky context.** `duburi.camera` and `duburi.target` default to
   `'forward'` and `'person'`. Set `duburi.camera` once at the top of
   the mission (or per call with `camera=`), and pass the target as
   the first positional arg to each verb. Switch cameras mid-mission
   with `duburi.use_camera('downward')`.

4. **`gain` caps speed, `err` sets precision, `duration` bounds time.**
   `gain` is a **hard max-speed cap** (% thrust), not a target speed —
   the AUV never exceeds it. `err` is the pixel tolerance for "centred".
   `duration` is the total time budget (search fallbacks count against
   it). The P-gains themselves have live `vision.*` ROS-param fallbacks
   on `auv_manager_node` — leave them out of the mission and tune from
   the deck:

   ```bash
   ros2 param set /duburi_manager vision.kp_yaw       80.0
   ros2 param set /duburi_manager vision.kp_lat       60.0
   ros2 param set /duburi_manager vision.lost_grace_s  1.0
   ```

5. **Every verb blocks until the action server returns.** No
   threading. If you want a background hold (yaw lock during motion),
   use `duburi.lock_heading(...)` — that's the one verb that returns
   immediately and runs a daemon thread.

6. **Vision verbs never raise; open-loop verbs do.** `align` / `move`
   honour a **never-fail contract**: on timeout or target loss they
   log the miss, return a falsy `VisionResult`, and the mission
   continues to the next line. Open-loop verbs (`arm`, `move_forward`,
   `yaw_right`, ...) still raise `MoveRejected` / `MoveFailed` on
   rejection. Wrap the whole mission in a try/finally that calls
   `duburi.disarm()` for a guaranteed safe exit.

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
duburi.set_mode('STABILIZE')                # srot: STABILIZE | DEPTH_HOLD | SURFACE | MANUAL | ACRO
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
if duburi.backend != 'srot':                      # srot: the board holds heading
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

### 3.2  Vision-driven motion (`duburi.vision.*`) — exactly two verbs

The 2026-06 rewrite collapsed the old nine-verb vision API into **two
pixel-native verbs**. Both run a closed P loop on the **largest detection**
of `target` in `camera`; both treat `gain` as a hard max-speed cap; and
**neither ever raises** — on a miss they log it and return a falsy
`VisionResult`, so the mission simply continues to the next line.

```python
duburi.vision.align(target, *, lat=None, yaw=None, depth=None,
                    err=40, duration=20, gain=30,
                    fallback=None, camera=None) -> VisionResult

duburi.vision.move(target, *, fwd=95, mode='area', maintain=None,
                   hold=None, err=40, duration=20, gain=30,
                   fallback=None, camera=None) -> VisionResult
```

`gain` = hard max-speed cap (% thrust, never exceeded). `err` = pixel
tolerance for "centred". `duration` = total time budget (fallback cycles
count against it). `camera` defaults to `duburi.camera` (`'forward'`).
`target` is a class string (`'gate'`) or a `duburi.models.<alias>.<class>`
ClassRef (which auto-switches model + class filter first).

> **Inertial brake (`brake=True`, on by default).** Like the control verbs,
> the vision verbs reverse-kick on arrival to bleed water inertia so the hull
> stops where you planned and the next step starts from the right place —
> `align` brakes lateral, `move` brakes forward (+`maintain`) on a fill-stop.
> It is self-gating on the exit-velocity EMA, so a gently-converged lock that
> ramps down usually isn't kicked — but a fast snap-in can still cross the gate,
> so on the **torpedo fire path pass `brake=False`** (no benefit when firing).
> Pass-through (`move(fwd=None)`) and abort/loss never brake. `brake=False`
> coasts any step.

#### `align` — centre the target on selected axes

Each of `lat` / `yaw` / `depth` is **`None` = axis OFF**, or a **number =
axis ON**, where the number is the **signed pixel offset from centre**
(`0` = dead centre, `+` = right/below, `-` = left/above). `lat` + `yaw` are
horizontal (Ch6 strafe / Ch4 rotate); `depth` is vertical. **At least one
axis is required.** `align` reports `ALIGNED` once *every* active axis sits
within `err` px for `vision.align_stable_frames` ticks (3 @ 20 Hz ≈ 0.15 s).

```python
duburi.vision.align('gate', yaw=0, lat=0)                      # centre horizontally
duburi.vision.align('hole', yaw=0, lat=0, depth=0, err=12)     # tight 3-axis lock
duburi.vision.align('red_pipe', yaw=0, lat=80)                 # hold pipe 80 px right
duburi.vision.align('fire', lat=0, depth=0, camera='downward') # downward: lat + fore/aft
```

#### `move` — drive forward to a bbox fill ratio

`move` drives forward until the target's bbox fills `fwd` % of the frame,
measured by `mode`:

| `mode=`    | Fill metric        | Best for                                  |
| ---------- | ------------------ | ----------------------------------------- |
| `'area'`   | √(w·h) (default)   | gates, bins, square-ish or variable shape |
| `'width'`  | width fraction     | wide horizontal bars / banners            |
| `'height'` | height fraction    | tall narrow targets: slalom pipe, flare   |

`maintain=±px` holds a lateral pixel offset while driving (`None` = pure
forward; never touches lat/yaw/depth). `hold=s` station-keeps at the fill
target for `s` seconds before exiting (`None` = exit on reach). **`move`
never re-centres yaw or depth** — ArduSub's ALT_HOLD owns depth, and the
heading lock (or the autopilot) owns yaw.

```python
duburi.vision.move('gate', fwd=80, mode='area')                    # drive through gate
duburi.vision.move('red_pipe', fwd=60, mode='height', maintain=80) # pass beside the pipe
duburi.vision.move('blood', fwd=30, mode='height', hold=2.0)       # close in, hold 2 s
```

#### The `VisionResult` it returns

Both verbs return `VisionResult(ok, reason, code, last_err_px, fill, x_px, y_px,
saw_target, elapsed_s)`, **truthy only when the goal was achieved** — but also
carrying **where/how it ended** (`x_px`/`y_px` signed target-from-centre px, `NaN`
if never seen; `saw_target`) so a missed align can run tested open-loop recovery
(the hybrid vision+control paradigm — full guide: [`vision-results.md`](vision-results.md)).
You branch on it directly:

```python
# yaw_gain low -> slow, stable yaw so the 20 kg hull holds the hole steady to fire.
# brake=False on the fire path: the arrival brake can emit a 0.2s kick on a fast
# snap-in, nudging the hull off-aim between lock-confirm and fire(). No benefit
# when firing from a lock (you are not moving away), so disable it.
if duburi.vision.align('hole', yaw=0, lat=0, depth=0, err=12, gain=25, yaw_gain=10,
                       brake=False):
    duburi.fire(1)                       # fire only on a confirmed lock
else:
    log.info('hole never locked — holding fire')
```

| `code` | `reason`    | Meaning                                                |
| ------ | ----------- | ------------------------------------------------------ |
| 0      | `ALIGNED`   | aligned (align) / bbox reached `fwd` fill (move)       |
| 1      | `LOST`      | target gone past `vision.lost_grace_s` (runs fallback) |
| 2      | `TIMEOUT`   | `duration` elapsed without success                     |
| 3      | `NO_CAMERA` | camera pipeline not up (no `camera_info`)              |
| 4      | `ABORTED`   | goal cancelled (cooperative abort)                     |
| —      | `FAILED`    | DSL-side server/setup error (bad camera, disarmed, …)  |

A miss never stops the mission — control falls through to the next step.

#### `fallback` — mission-authored search

`fallback` is the search behaviour, written by **you** as a plain Python
function at the bottom of the mission file. It runs **on real target loss**
(`LOST`), after which the verb **re-enters** the vision loop — all inside
the original `duration` budget. Two shapes are accepted:

```python
def creep_forward(duburi):                 # one short manoeuvre, then return
    duburi.move_forward(0.6, gain=40)

def sweep_for_gate(duburi, should_stop):   # longer self-polling sweep
    for _ in range(6):
        if should_stop():                  # True the moment the target reappears
            return
        duburi.yaw_right(15)
        duburi.pause(0.4)
```

- `fn(duburi)` — runs one manoeuvre, returns; the verb re-checks the frame.
- `fn(duburi, should_stop)` — may sweep longer; `should_stop()` returns
  `True` as soon as the target is detected again, so a good sweep bails early.

A `fallback` is **pure open-loop control** (`move_forward` / `yaw_right` /
`turn` / strafe) and must not call vision verbs. With **no** `fallback` the
verb instead holds station through brief losses and rides out the `duration`.

**Typical competition flow** (mirrors `missions/task_gate.py` and
`missions/pool_day_practice.py`): `align` to centre → `move` to close in →
optionally `fire`, each with a mission-authored `fallback`:

```python
# Register models once at the top of run(); access via duburi.models.alias.class
duburi.models(gate='gate_rescue_repair')

duburi.vision.align(duburi.models.gate.gate, yaw=0, lat=0,
                    gain=30, duration=20, fallback=sweep_for_gate)
duburi.vision.move(duburi.models.gate.gate, fwd=80, mode='area',
                   gain=35, duration=20, fallback=creep_forward)
```

---

## 3.3  Tracking, holding, and re-centring while moving

### Where tracking lives now (HUD-only)

There is **no per-goal tracking flag** and **no `vision.use_tracks` control
param**. The two vision verbs **always read `/detections`** — the raw
detector topic — so what the AUV acts on is exactly what
`duburi.detected()` and the HUD report.

`tracker_node` (ByteTrack + a Kalman smoother) still exists, but it is
**display-only**: it republishes `/detections` as `/tracks` with stable IDs
and jitter-free boxes for the mission-control HUD (and feeds the depth
node). It never sits in the control loop. Launch it for a nicer HUD:

```bash
ros2 launch duburi_vision cameras_.launch.py with_tracking:=true
```

### Holding on a target

`align` exits as soon as the target is centred (within `err` for
`vision.align_stable_frames` ticks). To **keep holding** instead of exiting,
use one of:

- `duburi.vision.move(target, fwd=<reached fill>, hold=S)` — drive to a fill
  ratio, then station-keep for `S` seconds before exiting.
- a re-centre loop: open-loop manoeuvre, then `align` again, repeated.

### Surviving brief target loss

Two mechanisms ride out dropped frames — no tracker required:

- `vision.lost_grace_s` (default 1.0 s) — the loop coasts on a momentary
  loss before reporting `LOST`. Tune it from the deck.
- `fallback` — on a real loss the verb runs your search function once, then
  re-enters. With **no** `fallback`, the verb holds station through the loss
  and rides out its `duration` instead of searching.

### Orbit pattern — re-centre after each yaw step

An orbit is just an open-loop yaw step followed by a vision re-centre,
looped. Each step pivots in place (the target drifts off-centre), then
`align` pulls it back:

```python
ORBIT_STEPS   = 12      # 12 × 30° = 360°
ORBIT_STEP    = 30.0
HOLD_PER_STEP = 3.0

duburi.models(gate='gate_flare_medium_100ep')
for _ in range(ORBIT_STEPS):
    duburi.yaw_left(ORBIT_STEP, timeout=10.0, settle=0.3)   # pivot in place
    duburi.vision.align(duburi.models.gate.flare,
                        yaw=0, depth=0, gain=30,
                        duration=HOLD_PER_STEP)             # re-centre on the flare
```

The 12-step polygon approximates a circle for pre-qual. For a smoother arc,
replace the loop body with `duburi.arc(...)` calls.

### Re-centring while driving forward

For heading-correction on a long approach, alternate a short open-loop leg
with a quick yaw re-centre:

```python
for _ in range(5):
    duburi.move_forward(2.0, gain=40)               # open-loop leg
    duburi.vision.align('gate', yaw=0, duration=2)  # snap the heading back
```

Or, when you want the controller to own the whole approach, just use `move`
— it drives forward on bbox fill and (with `maintain=`) holds a lateral
offset the entire way, no re-centre loop needed.

### Smoke-testing the tracker (HUD) pipeline

```bash
# 1. Launch vision stack with the HUD tracker enabled
ros2 launch duburi_vision cameras_.launch.py with_tracking:=true

# 2. Topic health check
ros2 run duburi_vision vision_check --camera laptop

# 3. Confirm /tracks is publishing (HUD / depth feed only)
ros2 topic hz /duburi/vision/laptop/tracks

# 4. Inspect a track (stable tracking_id; score > 0 for real detections)
ros2 topic echo /duburi/vision/laptop/tracks --once

# 5. Full integration test via the tracker CLI
ros2 run duburi_vision tracker_check --camera laptop --class person
```

### Performance notes

- `tracker_node` runs on the same Raspberry Pi 5 as the detector.
  ByteTrack state is cheap (~10 µs per update); Kalman adds ~5 µs per track.
- At 30 fps with 5 tracked objects the combined overhead is < 1 ms, well
  within the 33 ms frame budget.
- `track_buffer=30` (default) means a HUD track survives 1 s of occlusion at
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
# yolov11n detects COCO 80 classes — use 'person' to test demo_move_see
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
    duburi.mission_reset()
    duburi.camera = 'forward'
    duburi.models(gate='gate_flare_medium_100ep')   # register once; use anywhere

    # Tether removal window -- operator disconnects tether during countdown
    duburi.countdown(10)

    duburi.arm()
    duburi.set_mode('ALT_HOLD')
    duburi.set_depth(-1.0, settle=2.0)
    duburi.dvl_connect()

    # Gate phase — ClassRef auto-switches model+class in each verb call
    duburi.vision.align(duburi.models.gate.gate, yaw=0, lat=0,
                        gain=30, duration=20, fallback=creep_forward)
    duburi.vision.move(duburi.models.gate.gate, fwd=80, mode='area',
                       gain=35, duration=20, fallback=creep_forward)
    duburi.move_forward_dist(3.5, gain=60.0)

    # Flare phase — yaw+depth centre, then close in on bbox height
    duburi.vision.align(duburi.models.gate.flare, yaw=0, depth=0,
                        gain=30, duration=20, fallback=sweep_yaw)
    duburi.vision.move(duburi.models.gate.flare, fwd=45, mode='height',
                       gain=35, duration=20, fallback=creep_forward)

    # Orbit flare 360° — yaw step, then re-centre at each stop
    for _ in range(12):
        duburi.yaw_left(30.0, timeout=10.0, settle=0.3)
        duburi.vision.align(duburi.models.gate.flare, yaw=0, depth=0,
                            gain=30, duration=3.0)

    # Return through the gate
    duburi.yaw_right(180.0, timeout=25.0, settle=0.5)
    duburi.vision.align(duburi.models.gate.gate, yaw=0, lat=0,
                        gain=30, duration=20, fallback=sweep_yaw)
    duburi.vision.move(duburi.models.gate.gate, fwd=80, mode='area',
                       gain=35, duration=20, fallback=creep_forward)
    duburi.move_forward_dist(3.5, gain=60.0)

    duburi.stop()
    duburi.set_depth(0.0)
    duburi.disarm()


# ── Mission-authored fallback search patterns (pure control) ────────────────────
def creep_forward(duburi):
    duburi.move_forward(0.6, gain=40)

def sweep_yaw(duburi, should_stop):
    for _ in range(6):
        duburi.yaw_right(20)
        if should_stop():           # bail the moment the target reappears
            return
```

See `missions/gate_flare_prequal.py` for the production-tuned version of this.

### Edge cases and robustness notes (from preview footage)

| Situation | Behaviour |
|-----------|-----------|
| Gate partially off-frame (one post visible) | `align`'s yaw axis steers toward the visible bbox centre — no special code needed |
| Gate fills 90%+ of frame (too close) | `move` exits the moment fill ≥ `fwd`; pick a smaller `fwd` (e.g. 80) with `mode='area'` |
| Flare at distance (narrow, ~10% frame height) | `align(yaw=0, depth=0)` still has signal; `move(fwd=45, mode='height')` closes in |
| Detection flicker in turbid water | the loop coasts for `vision.lost_grace_s` (default 1.0 s) before LOST; raise it to ride out longer drop-outs |
| False positives (non-target debris) | `conf=0.45` in detector.yaml already filters these; real detections are 0.90-0.97 |

**`mode`** guidance (the `move` fill metric):
- `'area'` (√(w·h)) — use for gates and bins (wide, squat objects)
- `'height'` — use for flares and vertical slalom pipes
- `'width'` — use for wide horizontal bars / banners

### Tether removal countdown

```python
duburi.countdown(10)          # 10-second window, default message
duburi.countdown(15, message='Stand clear. Starting autonomous run.')
```

Prints an ASCII box countdown to stdout. The mission continues immediately
after. All onboard compute (Pi, Pi, DVL, Pixhawk) is self-sufficient —
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

The control engine is two P-loops on **pixel error** (`motion_vision.py`).
`gain` clamps every output; `err` (px) is the settle tolerance.

**`align_loop`** — one tick @ 20 Hz:
```
sample = vision_state.bbox_error(target_class)        # None if class absent
if sample is None or sample.age_s > lost_grace_s:
    -> write neutral RC, hold depth setpoint, count toward LOST
else:
    half_w, half_h = W/2, H/2
    # per ACTIVE axis: signed pixel control error (offset shifts the aim point)
    'yaw':   ctrl = sample.ex - off_yaw/half_w;  epx = |ctrl|*half_w
             yaw_pct = clamp( ctrl * kp_yaw, -gain, +gain)      -> Ch4 (no negate)
    'lat':   ctrl = sample.ex - off_lat/half_w;  epx = |ctrl|*half_w
             lat_pct = clamp( ctrl * kp_lat, -gain, +gain)      -> Ch6
    'depth': ctrl = sample.ey - off_dep/half_h;  epx = |ctrl|*half_h
             depth_setpt -= clamp(ctrl*kp_depth, -nudge, +nudge) * depth_sign
    ALIGNED when every active axis has epx <= err_px for align_stable_frames ticks
```

**`move_loop`** — one tick @ 20 Hz:
```
fill = {'area': sqrt(w_frac*h_frac), 'width': w_frac, 'height': h_frac}[mode]
if fill >= fwd_fill:                # reached -> station-keep, exit after hold_s
    fwd_pct = 0
else:
    fwd_pct = clamp((fwd_fill - fill) * kp_forward, 0, gain)        -> Ch5
if maintain_on:                     # optional lateral hold while driving
    ctrl    = sample.ex - maintain_px/half_w
    lat_pct = clamp(ctrl * kp_lat, -gain, +gain)                    -> Ch6
# yaw and depth are NEVER commanded by move (ArduSub holds depth; lock holds yaw)
```

- Depth integrates **incrementally** so ALT_HOLD never sees a step jump
  (`nudge = 0.02 m × gain/100` per tick).
- Sign rules (forward cam): `ex>0` = target right, `ey>0` = target below.
  Ch4>1500 = yaw RIGHT and Ch6>1500 = strafe RIGHT (current hull), so neither
  yaw nor lat is negated — both share `ex` polarity (pool-verified 2026-06).
- `gain` is a hard clamp on every output — the AUV never exceeds it.

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

### Settle / loss / fill

| Param                        | Sane range | Notes                                          |
| ---------------------------- | ---------- | ---------------------------------------------- |
| `vision.lost_grace_s`        | 0.5 .. 1.5 | Seconds the loop coasts on a loss before reporting LOST (then the `fallback` runs). Longer tolerates more dropped frames. |
| `vision.align_stable_frames` | 2 .. 5     | Ticks @ 20 Hz every active axis must stay within `err` before `align` reports ALIGNED. 3 ≈ 0.15 s. |
| `vision.frame_fill_default`  | 60 .. 95   | `move`'s `fwd` fill % used when the mission leaves `fwd` unset. |

The other knobs — `err` (pixel tolerance), `duration` (time budget), `gain`
(max-speed cap), `fwd` / `mode` / `maintain` / `hold` — are **per-call args**,
not ROS params. Set them in the verb call (or pull them from
`competition_config.py`).

Defaults live in
[`src/duburi_manager/duburi_manager/vision_tunables.py`](../../src/duburi_manager/duburi_manager/vision_tunables.py).

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
    if duburi.backend != 'srot':                  # srot holds heading on the board
        duburi.lock_heading()                     # latch current heading
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
    duburi.camera = 'laptop'
    duburi.arm()
    duburi.set_depth(-0.5)
    duburi.move_forward(3.0, gain=60)
    # align: yaw-centre the person (sweep to find them); move: close to 55% fill.
    duburi.vision.align('person', yaw=0, err=50, gain=30, duration=20,
                        fallback=sweep_yaw)
    duburi.vision.move('person', fwd=55, mode='height', gain=35, duration=15)
    duburi.move_back(2.0, gain=60)
    duburi.disarm()


def sweep_yaw(duburi, should_stop):
    for _ in range(6):
        duburi.yaw_right(30)
        if should_stop():
            return
```

### 6.4  Reacquire after losing the target

```python
def run(duburi, log):
    duburi.camera = 'laptop'
    duburi.arm()
    duburi.set_depth(-0.5)

    # align() runs `reacquire` automatically on loss, then re-enters — all
    # inside duration. It never raises, so branch on the result, not try/except.
    for attempt in range(3):
        log.info(f'attempt {attempt+1}: searching')
        if duburi.vision.align('person', yaw=0, gain=30, duration=20,
                               fallback=reacquire):
            log.info('centred')
            break
        log.warning('still not centred -- backing off & retrying')
        duburi.move_back(2.0, gain=50)

    duburi.disarm()


# Search fallback: back off and fan yaw to bring the target back into frame.
def reacquire(duburi, should_stop):
    duburi.move_back(1.5, gain=50)
    for _ in range(4):
        duburi.yaw_right(45)
        if should_stop():
            return
```

### 6.5  Full 3-axis align, then forward close-in

```python
def run(duburi, log):
    duburi.camera = 'forward'
    duburi.arm()
    duburi.set_depth(-1.5)
    # align owns the three centring axes at once (yaw + lat + depth);
    # then move owns the forward close-in. Two verbs, full lock.
    duburi.vision.align('gate', yaw=0, lat=0, depth=0,
                        gain=30, duration=20, fallback=creep_forward)
    duburi.vision.move('gate', fwd=80, mode='area', gain=35, duration=20)
    duburi.disarm()


def creep_forward(duburi):
    duburi.move_forward(0.6, gain=40)
```

### 6.6  Patrol pattern: drive, scan, drive, scan

```python
def run(duburi, log):
    duburi.camera = 'forward'
    duburi.arm()
    duburi.set_depth(-0.8)

    for leg in range(3):
        log.info(f'leg {leg}: drive')
        duburi.move_forward(5.0, gain=55)
        log.info(f'leg {leg}: scan')
        if duburi.detected('gate', stale_after=0.5):
            log.info('found -- homing in')
            duburi.vision.align('gate', yaw=0, lat=0, gain=30, duration=15)
            duburi.vision.move('gate', fwd=80, mode='area', gain=35, duration=15)
            break
        log.info('no hit, continue patrol')
        duburi.yaw_right(60.0)

    duburi.disarm()
```

### 6.7  Per-call tuning (pin precision for one phase)

```python
def run(duburi, log):
    duburi.camera = 'forward'
    duburi.arm()
    duburi.set_depth(-0.5)
    # Tight, slow yaw-only lock: small err (precise) + low gain (gentle).
    # The P-gains (vision.kp_yaw, …) are deck-side ROS params only — set them
    # with `ros2 param set /duburi_manager vision.kp_yaw 85.0`.
    duburi.vision.align('gate', yaw=0, err=12, gain=20, duration=10)
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

    # Phase 1: gate approach with the forward camera
    duburi.use_camera('forward')
    duburi.vision.align('gate', yaw=0, lat=0, gain=30, duration=15)
    duburi.vision.move('gate', fwd=80, mode='area', gain=35, duration=15)

    # Phase 2: descend and switch to the downward camera for the bin
    duburi.set_depth(-1.5, settle=1.5)
    duburi.use_camera('downward')
    # downward cam: lat = left/right, depth axis = fore/aft (see §6.9)
    duburi.vision.align('bin', lat=0, depth=0, err=30, gain=30, duration=20)
    duburi.disarm()
```

### 6.9  Downward-camera alignment (bin centring)

The downward camera looks straight down, so `align`'s axes do different
physical work than on the forward camera. Centre over the target with
`lat` + `depth` (skip `yaw`; `align` has no `forward` axis):

- `lat=0`   → Ch6 strafe: centres the target **left/right** in the frame.
- `depth=0` → nudges the AUV to centre the target along the **fore/aft**
  (image-Y) sense; the loop auto-flips `depth_sign` for the downward cam.
  This `depth_sign` flip is the **only** automatic downward adaptation.
- `yaw` is skipped — top-down yaw from a bbox is unreliable.
- forward *fill* is a separate concern: it lives on `move`, never on `align`.

```python
duburi.use_camera('downward')

# CORRECT: lat + depth centre the AUV over the bin; yaw off.
duburi.vision.align('fire', lat=0, depth=0, err=30, gain=30, duration=20)

# WRONG: yaw-only does nothing useful from a top-down view.
# duburi.vision.align('fire', yaw=0, duration=20)
```

Mirrors `missions/task_bin.py`. If you use a custom camera id other than
`'downward'`, the `depth_sign` flip does not apply — handle polarity yourself.

### 6.10  Offset alignment (the axis value IS the offset)

There is no separate `offset_x` / `offset_y` kwarg any more. The **number you
pass to `lat` / `yaw` / `depth` IS the signed pixel offset** from centre
(`0` = centre, `+` = right/below, `-` = left/above). For `move`, `maintain=±px`
holds a lateral offset while driving forward.

**Slalom side-of-pipe pass** — sit 80 px right of the pipe, then drive in
holding that offset:
```python
duburi.vision.align('red_pipe', yaw=0, lat=80, gain=30, duration=20)
duburi.vision.move('red_pipe', fwd=60, mode='height', maintain=80,
                   gain=35, duration=15)
```

**Torpedo board bullseye** — aim 60 px right and 40 px above the board centre
(one horizontal axis — `yaw` here — plus `depth` for the vertical):
```python
duburi.vision.align('torpedo', yaw=60, depth=-40,
                    err=14, gain=20, duration=15)
```

> **Renamed/removed:** the old `offset_x` / `offset_y` kwargs and the per-axis
> verbs they rode on (`vision_align_yaw`, `vision_align_lat`,
> `vision_align_depth`, `vision_align_3d`, `vision_hold_distance`,
> `vision_lock_fire`, `vision_acquire`, `look_around`) were all deleted in the
> two-verb rewrite. Use the axis values on `align` / `maintain` on `move`.

### 6.11  Dubomini — vision-held position (no DVL)

Dubomini 2.0 has no DVL. Use `align` on lat+yaw+depth to hold station on a
detected target. Launch with `yaw_source:=bno085` and `dvl_auto_connect:=false`.

```python
# Dubomini bringup: ros2 run duburi_manager start
#   --ros-args -p yaw_source:=bno085 -p dvl_auto_connect:=false

def run(duburi, log):
    duburi.camera = 'forward'
    duburi.arm()
    duburi.set_depth(-0.6, settle=1.0)

    # Hold station on the gate using vision (replaces POSHOLD — Dubomini has no DVL).
    # No fallback: align holds through brief losses and rides out the duration.
    duburi.vision.align('gate', yaw=0, lat=0, depth=0, gain=30, duration=30)

    # Approach: forward open-loop (no DVL distance available).
    duburi.move_forward(5.0, gain=55)
    duburi.disarm()
```

VehicleProfile auto-selects Dubomini vs Duburi 4.5 at runtime via `VehicleProfile.auto()` —
`has_dvl=False` means `move_forward_dist` falls back to an open-loop time estimate (and logs a
warning) rather than implying DVL precision it doesn't have.

### 6.12  Torpedo firing (align → move → fine-lock → fire)

There's no lock-fire verb. Firing is just **`if align(...): fire(n)`** — the
fine `align` is truthy only when the hole is centred within `err`, so it is its
own fire gate. Coarse-align the board, close in on `blood`, then take a tight
`hole` lock.

**Pattern: always call `mission_reset()` first.** This stops any leftover heading lock and clears
the abort event from the previous run — critical for back-to-back pool runs.

```python
def run(duburi, log):
    duburi.mission_reset()   # ★ ALWAYS first — clears heading lock + abort from previous run
    duburi.camera = 'forward'
    duburi.models(torpedo='torpedo_blood_hole')   # classes: torpedo, blood, hole
    duburi.arm()
    duburi.set_depth(-1.2)

    # 1. Coarse board align (yaw + lat + depth); creep forward to find it.
    duburi.vision.align(duburi.models.torpedo.torpedo, yaw=0, lat=0, depth=0,
                        err=40, gain=30, duration=15, fallback=creep_forward)

    # 2. Approach: drive forward until 'blood' fills 30% of frame height.
    duburi.vision.move(duburi.models.torpedo.blood, fwd=30, mode='height',
                       gain=35, duration=30, fallback=creep_forward)

    # 3. Fine hole lock (tight err, slow gain). Fire ONLY on a confirmed lock.
    if duburi.vision.align(duburi.models.torpedo.hole, yaw=0, lat=0, depth=0,
                           err=14, gain=12, duration=25, fallback=creep_forward):
        duburi.fire(1)       # torpedo_1 — 1/2 = torpedo, 3/4 = dropper
        log.info('torpedo fired on stable lock')
    else:
        log.warning('hole never locked — holding fire')

    duburi.move_forward(2.0, gain=40)   # clear the board
    duburi.disarm()


def creep_forward(duburi):
    duburi.move_forward(0.6, gain=40)
```

**Key fields:**

| Field | Notes |
|---|---|
| `duburi.fire(channel)` | 1/2 = torpedo, 3/4 = dropper. **Always pass the channel explicitly.** |
| `err` (fine lock) | small px tolerance (e.g. 14) so the fire only triggers dead-on. |
| `gain` (fine lock) | low (e.g. 12) — gentle, precise corrections at close range. |
| `fwd` / `mode` | approach stop: drive until `blood` fills `fwd`% by `mode='height'`. |
| `align(...)` truthiness | truthy only when locked within `err` → use it as the fire gate. |

> **Full two-verb torpedo missions:** `missions/task_torpedo.py` (chunk) and
> `missions/pool_day_practice.py` (Phase 3) — run with
> `ros2 run duburi_planner mission <name>`.

---

### 6.13  Bin drop (downward camera + dropper)

Centre over the bin on the downward camera with `align` (lat + depth), then
drop. `align` is truthy only when centred within `err`, so it gates the drop.

```python
def run(duburi, log):
    duburi.mission_reset()
    duburi.models(bin='bin_fire_blood')      # classes: blood, fire
    duburi.arm()
    duburi.set_depth(-1.5)

    # Fly over and centre on the bin with the downward camera.
    duburi.use_camera('downward')
    if duburi.vision.align(duburi.models.bin.fire, lat=0, depth=0,
                           err=30, gain=30, duration=45, fallback=creep_forward):
        duburi.pause(3.0)        # stability confirmation before the drop
        duburi.fire(3)           # dropper_1 — channel always explicit
    else:
        log.warning('bin never centred — skipping drop')

    duburi.use_camera('forward')
    duburi.disarm()


def creep_forward(duburi):
    duburi.move_forward(0.6, gain=40)
```

Mirrors `missions/task_bin.py`. The downward-cam `depth_sign` flip is automatic
(see §6.9); everything else is the same two-verb vocabulary as the forward cam.

---

### 6.14  Hold pattern (station-keep without exiting)

There's no dedicated hold verb. To **station-keep** instead of exiting on
settle, use `move(..., hold=S)` (drive to a fill, then hold `S` s), or `align`
followed by an explicit `pause`.

```python
# Drive in until the gate fills 70% of area, then station-keep for 5 s.
duburi.vision.move('gate', fwd=70, mode='area', hold=5.0, gain=35, duration=25)
duburi.move_forward(2.0, gain=60)

# Or: centre over a bin, hold briefly, then drop.
duburi.use_camera('downward')
duburi.vision.align('fire', lat=0, depth=0, err=30, gain=30, duration=20)
duburi.pause(3.0)      # let it settle
duburi.fire(3)         # dropper_1
```

**Arrive vs. stay:**
- `vision.align(...)` / `vision.move(...)` — exit as soon as centred / fill reached.
- `vision.move(..., hold=S)` — reach the fill, then station-keep for `S` seconds.
- `if vision.align(...): duburi.fire(n)` — align, confirm the lock, fire.

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
    duburi.lock_heading(0.0, timeout=120)   # ← positional arg, not target=

    # DVL moves — heading lock stays alive, no yaw drift
    duburi.move_forward_dist(3.0, gain=60)
    duburi.move_lateral_dist(1.0, gain=36)      # strafe 1 m right
    duburi.move_forward_dist(2.0, gain=60)

    duburi.release_heading()   # ← release_heading(), not unlock_heading()
    duburi.disarm()
```

### 7.3  Vision approach + DVL final run

Use vision to centre on the target, then DVL for a precise close-in. `align`
owns yaw while it runs, so lock heading only AFTER the alignment phase:

```python
def run(duburi, log):
    duburi.camera = 'forward'
    duburi.arm()
    duburi.set_depth(-1.0)

    # Phase 1: yaw-centre on the gate (sweep to find it if it's not in frame).
    duburi.vision.align('gate', yaw=0, gain=30, duration=30, fallback=sweep_for_gate)

    # Phase 2: lock the (post-alignment) heading so DVL drives straight in.
    duburi.lock_heading(0.0, timeout=120)   # 0 = lock current heading

    # Phase 3: drive through gate with DVL precision.
    duburi.move_forward_dist(4.0, gain=60)

    duburi.release_heading()
    duburi.disarm()


def sweep_for_gate(duburi, should_stop):
    for _ in range(6):
        if should_stop():
            return
        duburi.yaw_right(15); duburi.pause(0.4)
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

### 7.5  Orbit scan — `detected()` search loop

There's no scan/orbit verb anymore. Searching is a mission-authored loop: yaw
in steps, polling `duburi.detected()` (non-blocking, **case-insensitive**) at
each stop, then hand off to `align`. The same pattern, written as a function,
is exactly what you pass to a vision verb as a `fallback`.

```python
def run(duburi, log):
    duburi.models(gate='gate_flare_medium_100ep')
    duburi.camera = 'forward'
    duburi.arm()
    duburi.set_depth(-1.0, settle=1.0)

    # Orbit right in 20° steps until the gate is found (18 × 20° = 360°).
    found = False
    for _ in range(18):
        if duburi.detected(duburi.models.gate.gate, stale_after=0.5):
            found = True
            break
        duburi.yaw_right(20); duburi.pause(1.5)

    if found:
        log('gate found — aligning')
        duburi.vision.align(duburi.models.gate.gate, yaw=0, lat=0,
                            gain=30, duration=10)
    else:
        log('gate not found — advancing and retrying')
        duburi.move_forward(3.0, gain=35)
```

> Prefer to fold the search into the verb itself: pass the loop as a `fallback`
> and `align` will run it on target loss, then re-enter — all inside `duration`.

### 7.6  Vision queries — `detected()` / `wait_for()` / `where()`

Three client-side reads of the `/detections` stream let open-loop motion react
to what the camera sees, then hand off to vision-closed control. Each **pumps
the node** so the answer is the current frame (the default camera is subscribed
eagerly, so the first call never false-negates):

- `duburi.detected(class, *, camera=None, stale_after=1.0) -> bool` — visible
  right now? (point-in-time, case-insensitive)
- `duburi.wait_for(class, *, timeout=10.0, ...) -> bool` — block until seen or
  timeout (loop-free acquire while stationary)
- `duburi.where(class, *, band=0.15, ...) -> 'left'|'center'|'right'|'unknown'`
  — bearing of the largest match (`where_offset` → signed `[-1,+1]`)

This is the architecture step toward YASMIN FSMs — each `while detected()` loop
IS a proto-state.

**Full deep-dive reference:** [`.claude/context/detected-paradigm.md`](./detected-paradigm.md)

#### The three canonical patterns

```python
# 1. CIRCLE SEARCH — keep turning until seen (a moving search NEEDS a while;
#    an `if` runs once and falls through, it is not a loop)
while not duburi.detected('red_pipe'):
    duburi.yaw_left(30)
duburi.move_forward(3)            # runs once the pipe is in frame

# 2. ACQUIRE-THEN-ACT — wait in place, no busy-loop, with a give-up branch
if duburi.wait_for('gate', timeout=8):
    duburi.vision.align('gate', yaw=0, lat=0)
else:
    duburi.move_forward(1)        # never showed — recover

# 3. BEARING STEER — turn toward whichever side the target is on
{'left':  lambda: duburi.yaw_left(20),
 'right': lambda: duburi.yaw_right(20),
}.get(duburi.where('gate'), lambda: duburi.move_forward(1))()
```

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
    duburi.vision.align(duburi.models.gate.gate, yaw=0, lat=0,
                        gain=30, duration=20)
    duburi.vision.move(duburi.models.gate.gate, fwd=80, mode='area',
                       gain=55, duration=20)
    duburi.move_forward_dist(3.0, gain=60)

    # ── Search: yaw-sweep for flare ────────────────────────────────────── #
    duburi.set_classes('gate,flare')          # must set BEFORE detecting flare
    for _ in range(36):                       # 36 × 10° = full 360°
        if duburi.detected('flare', stale_after=0.5):
            break
        duburi.yaw_right(10); duburi.pause(0.5)

    if duburi.detected('flare', stale_after=0.5):
        duburi.vision.align(duburi.models.gate.flare, yaw=0,
                            gain=30, duration=20)
        duburi.vision.move(duburi.models.gate.flare, fwd=60, mode='height',
                           gain=35, duration=20)

        # ── Orbit flare: exit when gate re-appears ─────────────────────── #
        # IMPORTANT: passing the flare ClassRef above called set_classes('flare').
        # Restore both BEFORE the orbit loop.
        duburi.set_classes('gate,flare')
        for _ in range(18):                   # 18 × 20° = 360°
            if duburi.detected('gate', stale_after=0.3):
                break
            duburi.yaw_right(20); duburi.pause(1.0)

        if duburi.detected('gate', stale_after=0.5):
            duburi.vision.align(duburi.models.gate.gate, yaw=0, lat=0,
                                gain=30, duration=15)
            duburi.move_forward_dist(1.5, gain=60)

    duburi.release_heading()
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
calls `set_classes()` automatically. After `vision.align(flare_ref, ...)`, the
detector publishes flare detections only. `detected('gate')` will always return
`False` until you call `duburi.set_classes('gate,flare')`.

**Rule 4 — Set the camera first.** `detected()` falls back to `duburi.camera`
(default `'forward'`). Set it explicitly to the camera you mean — e.g.
`duburi.use_camera('downward')` before a bin search.

#### What blocks vs what doesn't

**Blocking (action round-trip — where mission time is spent):**
`move_forward`, `move_back`, `move_left`, `move_right`, `yaw_left`, `yaw_right`,
`arc`, `set_depth`, `arm`, `disarm`, `pause`, `stop`, `lock_heading`,
`dvl_connect`, `move_forward_dist`, `move_lateral_dist`, ALL `vision.*` verbs.

**Instant (no action goal):**
`duburi.camera =`, `duburi.target =`, `duburi.models(...)`.

`detected()`/`where()` **pump the node themselves** (bounded: ≤0.25 s warm,
≤0.60 s cold-on-first-frame), so they read the current frame regardless of
whether a verb ran just before — no warm-up move needed. `wait_for` pumps until
seen or its `timeout`.

#### Forbidden patterns

```python
# ✗ Step too long — 0.6m overshoot at gain=30
while not duburi.detected('gate'):
    duburi.move_forward(2.0, gain=30)

# ✗ No safety budget — runs forever if detector offline
while not duburi.detected('gate'):
    duburi.move_forward(0.5)

# ✗ Class filter trap — align(flare_ref) set classes='flare', gate never detected
duburi.vision.align(duburi.models.gate.flare, yaw=0)
for _ in range(18):
    if duburi.detected('gate'):   # ALWAYS FALSE until set_classes('gate,flare')
        break

# ✗ Wrong camera — polling 'forward' while the bin is under the downward cam
duburi.use_camera('forward')
while not duburi.detected('bin'):   # never True — bin is on 'downward'
    ...
```

#### `detected()` + `VisionResult` together (full decision tree)

A vision verb never raises — it returns a `VisionResult` that is truthy only on
success. Branch on it directly; pair it with `detected()` to pick the next target.

```python
# Try to centre on the gate; branch on whether it actually aligned.
if duburi.vision.align('gate', yaw=0, lat=0, gain=30, duration=20,
                       fallback=sweep_for_gate):
    duburi.vision.move('gate', fwd=80, mode='area', gain=35, duration=20)
    duburi.move_forward_dist(3.0, gain=60)
else:
    log.warn('gate never centred — advancing blindly')
    duburi.move_forward(4.0, gain=35)

# Confirm a target is still visible after a maneuver, then align on it.
duburi.yaw_right(45)
if duburi.detected('gate', stale_after=0.5):
    duburi.vision.align('gate', yaw=0, lat=0, gain=30, duration=15)
elif duburi.detected('flare', stale_after=0.5):
    duburi.vision.align('flare', yaw=0, depth=0, gain=30, duration=15)
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
in `missions/` plus a combinator in `missions/task_full_2026.py`.
Each chunk can be run independently with `ros2 run duburi_planner mission <name>`.
The flat layout is required: `discover()` only globs `missions/*.py` (non-recursive).

**Mission naming convention (commit cec7f37):**

| Category | Prefix | Examples |
|---|---|---|
| Competition chunks (detected-paradigm) | `task_` | `task_gate`, `task_slalom`, `task_bin`, `task_torpedo`, `task_return`, `task_full_2026` |
| Competition FSM launchers | `fsm_` | `fsm_slalom`, `fsm_bin`, `fsm_torpedo`, `fsm_return`, `fsm_full_2026` |
| Demos / development | `demo_` | `demo_arc`, `demo_square`, `demo_heading_lock`, `demo_move_see`, `demo_pursue`, `demo_find_person` |
| Prior FSM missions (kept) | varies | `gate_flare_fsm`, `prequal_fsm`, `gate_then_bin_fsm` |

### Key patterns introduced by the competition missions

#### Lazy detector activation (`pause_detector` / `resume_detector`)

Both cameras are always streaming. Detectors start `paused=True` in the
competition launch — inference only runs for the task that needs it.

```python
# resume forward detector before gate search
duburi.resume_detector('forward')   # → ros2 param set /duburi_detector_forward paused false

# ... gate task body ...

# pause again when done (frees GPU for the next chunk's detector)
duburi.pause_detector('forward')    # → ros2 param set /duburi_detector_forward paused true
```

#### Dual-camera node naming — pass `camera=`

There is one naming rule for the whole stack: the detector node is
`/duburi_detector_<camera>`, so `vision_dual.launch.py` creates
`/duburi_detector_forward` and `/duburi_detector_downward`. Every DSL helper
derives the node from its `camera` argument (default = the mission's sticky
camera), so just pass `camera=`:

```python
duburi.set_model('gate_rescue_repair', camera='forward')   # → /duburi_detector_forward
duburi.set_classes('gate,rescue,repair', camera='forward')
duburi.resume_detector('forward')                          # camera arg drives the node name
duburi.set_model('bin_fire_blood', camera='downward')      # → /duburi_detector_downward
```

#### Bounded search — never infinite forward

Every search loop has an explicit budget and a yaw-sweep fallback:

```python
MAX_STEPS = 20
for _ in range(MAX_STEPS):
    if duburi.detected('gate', stale_after=1.0):
        break
    duburi.move_forward(1.5, gain=40)    # 1.5 s steps — short enough to re-check
else:
    for _ in range(18):                  # 18 × 20° = 360° orbit fallback
        if duburi.detected('gate', stale_after=0.5):
            break
        duburi.yaw_right(20); duburi.pause(1.0)
```

#### Gate pass with bbox-fill exit (`move`)

The gate chunk exits forward drive when the gate bbox fills 80% of the frame —
that's exactly what `move` does:

```python
duburi.vision.move('gate', fwd=80, mode='height', gain=35, duration=25)
# exits automatically when the gate fills 80% of frame height = "through"
```

#### Downward camera centering (bin task)

With the downward camera, `align` re-maps the axes for you: `lat` is left/right
and `depth` is fore/aft (the `depth_sign` flip is automatic). Pass `lat`/`depth`
offsets — never `yaw` — to centre over the bin:

```python
# Centre over the bin: lat (left/right) + depth (fore/aft), no yaw.
duburi.use_camera('downward')
duburi.vision.align('fire', lat=0, depth=0, err=30, gain=30, duration=20)
```

Tuning that used to be per-call kwargs (`kp_lat`, deadband) is now ROS params —
`vision.kp_lat`, `vision.align_stable_frames` — set on the deck, not in the mission.

#### Fire channels — always explicit

There's no lock-fire verb. Fire after a truthy fine `align` (its own gate), and
always name the channel:

```python
duburi.fire(3)                        # dropper_1 — always explicit
if duburi.vision.align('hole', lat=0, depth=0, err=14, gain=12, duration=15):
    duburi.fire(1)                    # torpedo_1 — only when locked dead-on
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

### Full mission combinator (detected-paradigm)

`task_full_2026` chains all 5 chunks. Chunks are loaded via importlib for hot-reload support
(no colcon build needed for pool-day edits):

```python
# missions/task_full_2026.py  — detected()-paradigm sequential combinator
# Fixed API usage (post cec7f37): positional move_forward, lock_heading, release_heading
def run(duburi, log=None):
    gate    = _chunk('task_gate')
    slalom  = _chunk('task_slalom')
    _bin    = _chunk('task_bin')
    torpedo = _chunk('task_torpedo')
    _return = _chunk('task_return')
    try:
        duburi.pause(10.0)
        duburi.arm()
        duburi.set_depth(GATE_SEARCH_DEPTH_M, timeout=30)
        duburi.lock_heading(0.0, timeout=600)   # ← positional, not target=
        for name, chunk in [('gate',gate),('slalom',slalom),('bin',_bin),
                             ('torpedo',torpedo),('return',_return)]:
            try:
                chunk.run(duburi)
            except Exception as exc:
                if log: log(f'[MISSION] {name} FAILED: {exc} — continuing')
    finally:
        duburi.release_heading()   # ← release_heading, not unlock_heading
        duburi.stop(); duburi.disarm()
```

### FSM alternative (recommended for competition)

`fsm_full_2026` wraps the same tasks as a YASMIN state machine — structured retry,
outcome logging, per-task skip on failure:

```bash
ros2 run duburi_planner mission fsm_full_2026
```

Fill `competition_config.py` headings before running. `torpedo_depth_m=None` → torpedo task skipped.

### Individual chunk test commands

```bash
# ✅ runnable today (gate_rescue_repair.pt exists)
ros2 run duburi_planner mission task_gate
ros2 run duburi_planner mission task_return

# ⏳ logic test (model missing — verify search + timeout + abort flow)
ros2 run duburi_planner mission task_slalom
ros2 run duburi_planner mission task_bin
ros2 run duburi_planner mission task_torpedo

# full 5-task runs
ros2 run duburi_planner mission task_full_2026   # detected-paradigm
ros2 run duburi_planner mission fsm_full_2026    # YASMIN FSM (recommended)
```

See `packages/README.md §3` for per-chunk expected outputs and `models/README.md §Competition models` for model status.

---

## 8. Designing your own mission, step by step

1. **Pick a stable starting condition.** `arm()` then `set_depth(-0.5)`
   gives you altitude headroom and engages ALT_HOLD.
2. **Decide what each phase isolates.** A phase that drives forward
   blindly is `move_forward`. A phase that centres on a target is
   `vision.align`; a phase that closes in on it is `vision.move`.
   Don't mix them inside one verb; chain them.
3. **Use `lock_heading()` whenever you do open-loop translations.** It
   keeps yaw drift bounded for free. Depth needs no equivalent --
   `set_depth(...)` already hands depth back to ArduSub's onboard
   ALT_HOLD, which holds it for the rest of the mission with zero
   further traffic from us.
4. **`align` first, `move` second.** Settle yaw/lat/depth on the
   target with `vision.align` *before* committing to a `vision.move`
   close-in — otherwise the bbox error grows as you approach.
5. **Always provide an exit.** `disarm()` at the end (or in a
   `finally:` block).
6. **Bake gains into `vision.*` ROS params, not the mission.** The
   `kp_*` gains, `lost_grace_s`, and `align_stable_frames` live on the
   deck; keep `err`/`gain`/`duration` per-call. A mission that runs the
   same in pool and bench is a mission that trusts the deck.
7. **Small phases > big phases.** Aim for 5–10 verbs per mission,
   each with a clear single intent.

---

## 9. Gotchas

- **`align` with no axis raises `ValueError`.** You must pass a number to
  at least one of `lat=`, `yaw=`, `depth=` (`0` = centre). Target itself
  may be a class string, a `duburi.models.<alias>.<class>` ClassRef, or
  sticky `duburi.target` — but the axis is never optional.
- **No `fallback` means no search.** A vision verb that can't see the
  target just runs out its `duration` and returns a falsy `VisionResult`
  (`LOST`/`TIMEOUT`); it never spins in place looking. If the target may
  start off-frame, pass a `fallback` search fn (see §3.2 / §6.4).
- **Downward camera: the `depth_sign` flip is automatic.** When
  `camera='downward'` (or `'sim_bottom'`) the loop flips `depth_sign=-1`
  so the `depth` axis nudges fore/aft sensibly. The axis *choice* is still
  yours: centre over a bin with `lat=0, depth=0` (left/right + fore/aft),
  never `yaw` (see §6.9, mirrors `missions/task_bin.py`). A custom camera
  ID other than `'downward'`/`'sim_bottom'` won't get the flip — rename it
  or override at the manager.
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
- DVL integration reference:      `.claude/context/legacy-pixhawk-and-sitl.md`
- Sensors pipeline design:        `.claude/context/sensors-pipeline.md`
- CLI cookbook (deck one-liners): `README.md` §9
- Architecture (visual flow):     `.claude/context/vision-architecture.md`
- Roadmap (what's next):          `.claude/context/ROADMAP.md`
