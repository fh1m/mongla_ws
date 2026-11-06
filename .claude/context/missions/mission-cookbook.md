# Mongla Mission Cookbook

> Read this once, then design any mission you can imagine in 5 minutes.
>
> The CLI cookbook lives in the README §9 — one-liners for the deck.
> This file is the **mission-author's** companion: principles first,
> verbs second, full samples last.

The mission DSL lives in
[`src/mongla_planner/mongla_planner/mongla_dsl.py`](../../../src/mongla_planner/mongla_planner/mongla_dsl.py).
Every mission file in
[`src/mongla_planner/mongla_planner/missions/`](../../../src/mongla_planner/mongla_planner/missions)
is a plain Python module that exposes one `run(mongla, log)` function.


> ## ⛔ Read this before copying a recipe
>
> This cookbook predates the move to the SROT board, and some recipes still show the older
> backend's idioms. The **DSL verbs are unchanged** — what changed is where they run:
>
> - **`lock_heading` is refused on srot.** The board holds heading itself at 500 Hz. Ask
>   first: `if mongla.can('lock_heading'): mongla.lock_heading(0.0)`. A mission that cannot
>   do without a verb says so up front with `mongla.require('<verb>', why=...)`.
> - **`ALT_HOLD` is not a mode on this board.** `set_depth` works; the board owns depth. A
>   recipe that engages ALT_HOLD explicitly is describing the legacy path.
> - **`move_*_dist`, `arc` and `style_yaw` are refused on srot.** Use the timed moves, or the
>   downward camera's own distance bracket (`calc_distance`). The DVL recipes that used
>   to live here were retired (no DVL is fitted); see
>   [`legacy-pixhawk-and-sitl.md`](../platform/legacy-pixhawk-and-sitl.md).
> - **On the downward camera, the depth-setpoint axes are refused**: sideways and fore/aft
>   run normally.
>
> The current contract is [`command-reference.md`](command-reference.md); the legacy path is
> [`legacy-pixhawk-and-sitl.md`](../platform/legacy-pixhawk-and-sitl.md).

---

## 0. Designing a mission in 30 seconds

1. **Drop a file** in
   [`src/mongla_planner/mongla_planner/missions/`](../../../src/mongla_planner/mongla_planner/missions),
   for example `follow_gate.py`.
2. **Expose `def run(mongla, log)`** — that is the entire contract. No
   registry table, no `__init__.py` edit. Files starting with `_` are
   skipped (use them for shared helpers).
3. **Inside `run`, call `mongla.<verb>(...)` lines top to bottom.** The
   DSL prints one outcome line per call — no logging boilerplate.
4. **Build + run:**

   ```bash
   colcon build --packages-select mongla_planner && source install/setup.bash
   ros2 run mongla_planner mission --list          # your file appears
   ros2 run mongla_planner mission follow_gate
   ```

A complete file is just this:

```python
# src/mongla_planner/mongla_planner/missions/follow_gate.py
def run(mongla, log):
    mongla.camera = 'forward'
    mongla.arm()
    mongla.set_depth(-1.0)
    # align: centre the gate (yaw + lat); sweep to find it if it's not in frame.
    mongla.vision.align('gate', yaw=0, lat=0, gain=30, duration=20,
                        fallback=sweep_for_gate)
    # move: drive forward until the gate fills 80% of the frame, then we're through.
    mongla.vision.move('gate', fwd=80, mode='area', gain=35, duration=20)
    mongla.set_depth(0.0)
    mongla.disarm()


# Search pattern, defined at the BOTTOM of the file (pure open-loop control).
# Runs automatically on target loss, then align() re-enters — all inside duration.
def sweep_for_gate(mongla, should_stop):
    for _ in range(6):
        if should_stop():       # True the moment the gate reappears
            return
        mongla.yaw_right(15)
        mongla.pause(0.4)
```

That's the whole pattern: **two vision verbs** (`align` then `move`) and a
mission-authored `fallback` search. The rest of this cookbook fills in **how
the two verbs behave** and **how to tune them**.

---

## 0.5 Desk / Bench Testing with Pretrained Model (yolov11n + person)

No custom weights yet? Run this on a laptop with a webcam — no pool, no vehicle
needed (use `mode:=sim` or just call the mission runner without arming).

```python
def run(mongla, log):
    mongla.mission_reset()
    # Auto-downloads yolov11n.pt (~5 MB, COCO 80-class) on first run.
    mongla.models(person='yolov11n')
    mongla.camera = 'laptop'

    mongla.arm()
    mongla.set_depth(-0.5)

    target = mongla.models.person.person   # ClassRef — sets model + class automatically

    # align: yaw-centre the person; sweep to find them if they're not in frame.
    mongla.vision.align(target, yaw=0, err=50, gain=30, duration=12,
                        fallback=sweep_yaw)
    # move: drive in until the person fills 55% of the frame height.
    mongla.vision.move(target, fwd=55, mode='height', gain=35, duration=15,
                       fallback=creep_forward)
    mongla.disarm()


# Search patterns — pure open-loop control, defined at the bottom of the file.
def creep_forward(mongla):
    mongla.move_forward(0.6, gain=30)

def sweep_yaw(mongla, should_stop):
    for _ in range(6):
        mongla.yaw_right(30)
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
mongla.models(gate='gate_sharks')                # a stem in models/ (sidecar beside it)
mongla.models(person='yolov11n')                 # COCO pretrained, auto-download
```

**Run the canonical demo mission:**

```bash
ros2 launch mongla_vision vision.launch.py camera:=laptop
ros2 run mongla_planner mission demo_find_person
```

> See `src/mongla_planner/mongla_planner/missions/demo_find_person.py` for the full
> reference mission — it exercises both vision verbs (`align` + `move`) plus the
> mission-authored fallback search.

---

## 1. Mental model in 60 seconds

A mission is a script. Each line sends **one verb** to the manager and waits
for it to finish before the next line starts. There is no parallelism inside a
mission: it runs top to bottom.

Two namespaces, one object:

| Namespace | What it does | Who closes the loop |
|---|---|---|
| `mongla.*` | timed moves, turns, depth, arm/disarm | **the SROT board**, at 500 Hz |
| `mongla.vision.*` | steer on a detection (`align`, `move`) | the Pi, on pixel error, 20 Hz |

A mission usually alternates: a timed move to *get near*, then the two vision
verbs to *land precisely*.

```python
def run(mongla, log):
    mongla.camera = 'forward'
    mongla.arm()
    mongla.set_depth(-0.5)
    mongla.move_forward(3.0, gain=60)               # timed -- get to the area
    mongla.vision.align('gate', yaw=0, lat=0,       # vision -- centre on it
                        gain=30, duration=20)
    mongla.vision.move('gate', fwd=80, mode='area', # vision -- drive in
                       gain=35, duration=20)
    mongla.move_back(2.0, gain=60)                  # timed -- withdraw
    mongla.disarm()
```

---

## 2. Hard rules (read them, internalise them)

1. **One verb, one job.** `move_forward` drives forward; `yaw_right` turns.
   Mixing axes is what `vision.align` is for (any subset of lat / yaw / depth).
   Predictable verbs are what make a mission debuggable from its log.

2. **Vision informs control, never fights it.** `vision.align(t, yaw=0)` is a
   turn driven by a pixel error instead of a clock; `vision.move(t)` is a
   forward drive that stops on how much of the frame the target fills.

3. **Sticky context.** `mongla.camera` defaults to `'forward'`. Set it once at
   the top (or per call with `camera=`); switch mid-mission with
   `mongla.use_camera('downward')`.

4. **`gain` caps speed, `err` sets precision, `duration` bounds time.** `gain`
   is a hard ceiling in % thrust, never a target. `err` is the pixel tolerance
   for "centred". `duration` is the whole time budget, search fallbacks
   included. The P-gains are live ROS parameters on the manager -- tune them
   from the deck, not in the mission:

   ```bash
   ros2 param set /mongla_manager vision.kp_yaw       80.0
   ros2 param set /mongla_manager vision.kp_lat       60.0
   ros2 param set /mongla_manager vision.lost_grace_s  1.0
   ```

5. **Every verb blocks until it is done.** No threads in a mission.

6. **Vision verbs never raise; motion verbs report.** `align` / `move` return a
   falsy `VisionResult` on timeout or loss and the mission carries on. A
   motion verb returns a result whose `.success` is False when the board
   refused or the move stalled -- check it when the next line depends on it.
   The mission runner stops, brakes and disarms on any exit, including Ctrl-C.

7. **Ask before using a verb the board may refuse.** `arc`, `lock_heading`,
   `style_yaw` and the `*_dist` verbs are refused on srot:
   `if mongla.can('lock_heading'): ...`, or `mongla.require('arc', why=...)`
   to stop the mission on the deck rather than discover it at depth.

---

## 3. The verbs

### 3.1  Motion (`mongla.*`)

Every verb is a method on the `MonglaMission` you receive in
`run(mongla, log)`. The full generated table, with what runs on the board and
what is refused: [`reference/commands.md`](../reference/commands.md).

⛔ **Every motion verb below enters the board's AUTO mode, and AUTO closes the
depth loop -- which has never run closed in water.** Two armed bench checks
gate them all (CLAUDE.md §2). The vision verbs run in STABILIZE and are not
gated.

#### Power & mode

```python
mongla.arm()                     # waits for the board to confirm (timeout=15 s)
mongla.disarm()                  # confirmed on the heartbeat
mongla.set_mode('STABILIZE')     # STABILIZE | DEPTH_HOLD | SURFACE | MANUAL | ACRO
```

`set_mode` is best-effort on this wire: a refusal looks exactly like success
unless you read the mode back. The vision verbs do that for you.

#### Timed moves

```python
mongla.move_forward(seconds, gain=80, settle=0.0)
mongla.move_back   (seconds, gain=80, settle=0.0)
mongla.move_left   (seconds, gain=80, settle=0.0)
mongla.move_right  (seconds, gain=80, settle=0.0)
```

| Param | Meaning |
|---|---|
| `seconds` | how long to push. Time, **not** distance: there is no DVL. |
| `gain` | % thrust ceiling, 0..100. ~30 on the bench, 50-80 in the pool. |
| `settle` | extra still time after the move. |

Each is one `SROT_MOVE`: the board runs it **and brakes** at the end itself.

#### Depth

```python
mongla.set_depth(metres, timeout=30.0, settle=0.0)
```

`metres` is **negative below the surface**: `set_depth(-1.5)` is 1.5 m down.
The board holds the depth afterwards; there is no `lock_depth`.

⛔ An unhealthy barometer makes the board refuse DEPTH_HOLD and AUTO, so every
motion verb is denied: the vehicle arms and does not move.
`ros2 run mongla_manager bringup_check --srot` reads this off `SYS_STATUS`.

#### Turns

```python
mongla.yaw_left (degrees, timeout=30.0, settle=0.0)   # relative, to port
mongla.yaw_right(degrees, timeout=30.0, settle=0.0)   # relative, clockwise from above
mongla.turn(degrees)                                  # absolute -- needs MAG_YAW_REF=1
```

Firmware behaviour rev 10 inverted yaw; the manager refuses a board below the
required revision at connect and again in `arm()`.

#### Stop / pause

```python
mongla.stop()          # the board's MOVE_STOP brake
mongla.pause(2.0)      # station-keep for 2 s
```

⛔ **AUTO never exits by itself.** After a move, manual frames are discarded
until the mode changes. The vision verbs set STABILIZE and verify it took.

#### Refused on srot (pixhawk / simulator only)

`arc`, `lock_heading`, `style_yaw`, `move_forward_dist`,
`move_back_dist`, `move_lateral_dist`. They run in the simulator, which flies
ArduSub SITL by design. On the vehicle they return `success=False` before
anything is sent.

---

### 3.2  Vision-driven motion (`mongla.vision.*`) — exactly two verbs

The 2026-06 rewrite collapsed the old nine-verb vision API into **two
pixel-native verbs**. Both run a closed P loop on the **largest detection**
of `target` in `camera`; both treat `gain` as a hard max-speed cap; and
**neither ever raises** — on a miss they log it and return a falsy
`VisionResult`, so the mission simply continues to the next line.

```python
mongla.vision.align(target, *, lat=None, yaw=None, depth=None,
                    err=40, duration=20, gain=30,
                    fallback=None, camera=None) -> VisionResult

mongla.vision.move(target, *, fwd=95, mode='area', maintain=None,
                   hold=None, err=40, duration=20, gain=30,
                   fallback=None, camera=None) -> VisionResult
```

`gain` = hard max-speed cap (% thrust, never exceeded). `err` = pixel
tolerance for "centred". `duration` = total time budget (fallback cycles
count against it). `camera` defaults to `mongla.camera` (`'forward'`).
`target` is a class string (`'gate'`) or a `mongla.models.<alias>.<class>`
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
mongla.vision.align('gate', yaw=0, lat=0)                      # centre horizontally
mongla.vision.align('hole', yaw=0, lat=0, depth=0, err=12)     # tight 3-axis lock
mongla.vision.align('red_pipe', yaw=0, lat=80)                 # hold pipe 80 px right
mongla.vision.align('fire', lat=0, depth=0, camera='downward') # downward: lat + fore/aft
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
never re-centres yaw or depth** — the board holds depth, and holds heading in
STABILIZE (on the simulator's ArduSub, ALT_HOLD and the heading lock do).

```python
mongla.vision.move('gate', fwd=80, mode='area')                    # drive through gate
mongla.vision.move('red_pipe', fwd=60, mode='height', maintain=80) # pass beside the pipe
mongla.vision.move('blood', fwd=30, mode='height', hold=2.0)       # close in, hold 2 s
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
if mongla.vision.align('hole', yaw=0, lat=0, depth=0, err=12, gain=25, yaw_gain=10,
                       brake=False):
    mongla.fire(1)                       # fire only on a confirmed lock
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
def creep_forward(mongla):                 # one short manoeuvre, then return
    mongla.move_forward(0.6, gain=40)

def sweep_for_gate(mongla, should_stop):   # longer self-polling sweep
    for _ in range(6):
        if should_stop():                  # True the moment the target reappears
            return
        mongla.yaw_right(15)
        mongla.pause(0.4)
```

- `fn(mongla)` — runs one manoeuvre, returns; the verb re-checks the frame.
- `fn(mongla, should_stop)` — may sweep longer; `should_stop()` returns
  `True` as soon as the target is detected again, so a good sweep bails early.

A `fallback` is **pure open-loop control** (`move_forward` / `yaw_right` /
`turn` / strafe) and must not call vision verbs. With **no** `fallback` the
verb instead holds station through brief losses and rides out the `duration`.

**Typical competition flow** (mirrors `missions/task_gate.py` and
`missions/pool_day_practice.py`): `align` to centre → `move` to close in →
optionally `fire`, each with a mission-authored `fallback`:

```python
# Register models once at the top of run(); access via mongla.models.alias.class
mongla.models(gate='gate_sharks')

mongla.vision.align(mongla.models.gate.gate, yaw=0, lat=0,
                    gain=30, duration=20, fallback=sweep_for_gate)
mongla.vision.move(mongla.models.gate.gate, fwd=80, mode='area',
                   gain=35, duration=20, fallback=creep_forward)
```

---

## 3.3  Tracking, holding, and re-centring while moving

### Where tracking lives now (HUD-only)

There is **no per-goal tracking flag** and **no `vision.use_tracks` control
param**. The two vision verbs **always read `/detections`** — the raw
detector topic — so what the AUV acts on is exactly what
`mongla.detected()` and the HUD report.

`tracker_node` (ByteTrack + a Kalman smoother) still exists, but it is
**display-only**: it republishes `/detections` as `/tracks` with stable IDs
and jitter-free boxes for the mission-control HUD (and feeds the depth
node). It never sits in the control loop. Launch it for a nicer HUD:

```bash
ros2 launch mongla_vision vision.launch.py tracking:=true
```

### Holding on a target

`align` exits as soon as the target is centred (within `err` for
`vision.align_stable_frames` ticks). To **keep holding** instead of exiting,
use one of:

- `mongla.vision.move(target, fwd=<reached fill>, hold=S)` — drive to a fill
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

for _ in range(ORBIT_STEPS):
    mongla.yaw_left(ORBIT_STEP, timeout=10.0, settle=0.3)   # pivot in place
    mongla.vision.align('orange_flare',
                        yaw=0, depth=0, gain=30,
                        duration=HOLD_PER_STEP)             # re-centre on the flare
```

The 12-step polygon approximates a circle. (`arc` would be smoother, but it is
refused on srot.)

### Re-centring while driving forward

For heading-correction on a long approach, alternate a short open-loop leg
with a quick yaw re-centre:

```python
for _ in range(5):
    mongla.move_forward(2.0, gain=40)               # open-loop leg
    mongla.vision.align('gate', yaw=0, duration=2)  # snap the heading back
```

Or, when you want the controller to own the whole approach, just use `move`
— it drives forward on bbox fill and (with `maintain=`) holds a lateral
offset the entire way, no re-centre loop needed.

### Smoke-testing the tracker (HUD) pipeline

```bash
# 1. Launch vision stack with the HUD tracker enabled
ros2 launch mongla_vision vision.launch.py tracking:=true

# 2. Topic health check
ros2 run mongla_vision vision_check --camera laptop

# 3. Confirm /tracks is publishing (HUD / depth feed only)
ros2 topic hz /mongla/vision/laptop/tracks

# 4. Inspect a track (stable tracking_id; score > 0 for real detections)
ros2 topic echo /mongla/vision/laptop/tracks --once

# 5. Full integration test via the tracker CLI
ros2 run mongla_vision tracker_check --camera laptop --class person
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

### A model is three files with one stem

A model is chosen by its **stem**, the file name without extension. The
detector looks in `src/mongla_vision/models/` (mirrored from `~/models` by
`./build_mongla.sh`):

```
models/
├── gate_sharks.pt     # weights, for the dev box (CUDA)
├── gate_sharks.hef    # compiled for the Hailo-8 -- what the VEHICLE runs
└── gate_sharks.yaml   # the sidecar: class names + the measured conf bar
```

⛔ **On the vehicle the sidecar is mandatory.** A `.hef` carries no class
names, so a missing `<stem>.yaml` means an empty allowlist and a silent `[]`
every frame while the pipeline looks healthy. (A `.pt` on a dev box can fall
back to the names embedded in the weights; do not rely on that.)

The models that ship are development harnesses, swapped on water days:

| stem | classes | used by |
|---|---|---|
| `gate_sharks` | `gate`, `shark`, `shaw_fish` | forward camera default |
| `bin_fire_blood` | `blood`, `fire` | downward camera default |
| `sauvc_sim` | `qual_gate`, `final_gate`, flares, drums, `target_mat`, ... | the SAUVC missions |

How to train and compile a new one: the `train-model` skill
(`.claude/skills/train-model/SKILL.md`).

### Choosing the model

```bash
# The vehicle -- both cameras, Hailo:
ros2 launch mongla_manager bringup.launch.py vision:=true \
    fwd_model:=gate_sharks dwn_model:=bin_fire_blood

# A dev box, one camera, .pt on CUDA. yolov11n is COCO -- 'person' for desk tests:
ros2 launch mongla_vision vision.launch.py model:=yolov11n classes:=person
```

A mission names its model in ONE place, so a swap is a one-line edit:

```python
mongla.set_model('sauvc_sim', node='/mongla_detector_forward')
mongla.set_classes('qual_gate', node='/mongla_detector_forward')
```

`test_missions_only_name_classes_a_model_emits.py` fails if a mission names a
class its model does not emit.

### Switching the class filter live (no restart)

`classes` is a post-inference allowlist: same forward pass every frame,
different box list published.

```bash
ros2 param set /mongla_detector_forward classes gate
ros2 param set /mongla_detector_forward classes ""     # every class the model has
```

### A worked mission: `sauvc_qualification`

The clearest complete example in the repo, and srot-safe end to end:
`src/mongla_planner/mongla_planner/missions/sauvc_qualification.py`. Its shape:

```python
def _pass(mongla, blind_s, leg, log):
    """One gate pass: centre, drive through, else dead-reckon."""
    locked = mongla.vision.align(_GATE, camera='forward', yaw=0, lat=0,
                                 err=SAUVC_ALIGN_ERR_PX, gain=SAUVC_ALIGN_GAIN,
                                 duration=SAUVC_QUAL_GATE_ALIGN_S,
                                 fallback=creep_forward)
    passed = None
    if locked:
        # fwd=None: drive until the gate leaves the frame, then commit to clear it.
        passed = mongla.vision.move(_GATE, camera='forward', fwd=None,
                                    hold=SAUVC_GATE_COMMIT_S, gain=SAUVC_APPROACH_GAIN,
                                    duration=SAUVC_QUAL_GATE_MOVE_S,
                                    fallback=creep_forward)
    if SAUVC_QUAL_BLIND_ENABLED and not passed:
        mongla.move_forward(blind_s, gain=SAUVC_BLIND_GAIN)   # flown, never COUNTED
    return bool(passed)
```

Three habits it shows that every mission should copy:

1. **Every constant is named** and lives in `competition_config.py`, with the
   reason beside it.
2. **A verb the board refuses is asked about, not assumed**:
   `if mongla.can('lock_heading'): mongla.lock_heading(...)`.
3. **A guess is never counted as a success.** The blind leg is flown, but the
   return value only counts what vision confirmed.

### Edge cases and robustness notes (from preview footage)

| Situation | Behaviour |
|-----------|-----------|
| Gate partially off-frame (one post visible) | `align`'s yaw axis steers toward the visible bbox centre — no special code needed |
| Gate fills 90%+ of frame (too close) | `move` exits the moment fill ≥ `fwd`; pick a smaller `fwd` (e.g. 80) with `mode='area'` |
| Flare at distance (narrow, ~10% frame height) | `align(yaw=0, depth=0)` still has signal; `move(fwd=45, mode='height')` closes in |
| Detection flicker in turbid water | the loop coasts for `vision.lost_grace_s` (default 1.0 s) before LOST; raise it to ride out longer drop-outs |
| False positives (non-target debris) | the shipped `conf:=0.45` already filters these; real detections are 0.90-0.97 |

**`mode`** guidance (the `move` fill metric):
- `'area'` (√(w·h)) — use for gates and bins (wide, squat objects)
- `'height'` — use for flares and vertical slalom pipes
- `'width'` — use for wide horizontal bars / banners

### Tether removal countdown

```python
mongla.countdown(10)          # 10-second window, default message
mongla.countdown(15, message='Stand clear. Starting autonomous run.')
```

Prints an ASCII box countdown to stdout. The mission continues immediately
after. All onboard compute (Pi, Pi, DVL, Pixhawk) is self-sufficient —
removing the tether during this window leaves the AUV fully autonomous.

### Offline pre-pool testing with a video file

```bash
# Run the gate model on recorded pool footage
ros2 launch mongla_vision vision.launch.py \
    video_file:=/path/to/pool_run.mp4 model:=gate_sharks classes:=gate

# Loop OFF (stop at EOF), with the tracker
ros2 launch mongla_vision vision.launch.py \
    video_file:=/path/to/gate_run.mp4 model:=gate_sharks classes:=gate \
    loop:=false tracking:=true
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

- Depth integrates **incrementally** (`nudge = 0.02 m × gain/100` per tick)
  -- on the simulator. ⛔ On srot the depth-setpoint axes are refused (the
  board owns depth), so `depth=` on the forward camera does nothing there.
- Sign rules (forward cam): `ex>0` = target right, `ey>0` = target below.
  Ch4>1500 = yaw RIGHT and Ch6>1500 = strafe RIGHT (current hull), so neither
  yaw nor lat is negated — both share `ex` polarity (pool-verified 2026-06).
- `gain` is a hard clamp on every output — the AUV never exceeds it.

Reference implementation:
[`src/mongla_control/mongla_control/motion_vision.py`](../../../src/mongla_control/mongla_control/motion_vision.py).

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
| `vision.kp_depth`    | 0.02 .. 0.1 | simulator only (refused on srot). Too low: drifts vertically; too high: fights the depth hold. |
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
[`src/mongla_manager/mongla_manager/vision_tunables.py`](../../../src/mongla_manager/mongla_manager/vision_tunables.py).

---

## 6. Ready-to-steal mission samples

Every sample below is a complete `run(mongla, log)`. Drop it in
`src/mongla_planner/mongla_planner/missions/<name>.py`, rebuild, and
run with `ros2 run mongla_planner mission <name>`. The runner
auto-discovers any `*.py` not starting with `_`; no registration
required.

### 6.1  Hello world — arm, dive, surface, disarm

```python
def run(mongla, log):
    log.info('hello_world: starting')
    mongla.arm()
    mongla.set_depth(-0.5)
    mongla.pause(2.0)
    mongla.set_depth(0.0)
    mongla.disarm()
```

### 6.3  Find person, centre, hold distance

```python
def run(mongla, log):
    mongla.camera = 'laptop'
    mongla.arm()
    mongla.set_depth(-0.5)
    mongla.move_forward(3.0, gain=60)
    # align: yaw-centre the person (sweep to find them); move: close to 55% fill.
    mongla.vision.align('person', yaw=0, err=50, gain=30, duration=20,
                        fallback=sweep_yaw)
    mongla.vision.move('person', fwd=55, mode='height', gain=35, duration=15)
    mongla.move_back(2.0, gain=60)
    mongla.disarm()


def sweep_yaw(mongla, should_stop):
    for _ in range(6):
        mongla.yaw_right(30)
        if should_stop():
            return
```

### 6.4  Reacquire after losing the target

```python
def run(mongla, log):
    mongla.camera = 'laptop'
    mongla.arm()
    mongla.set_depth(-0.5)

    # align() runs `reacquire` automatically on loss, then re-enters — all
    # inside duration. It never raises, so branch on the result, not try/except.
    for attempt in range(3):
        log.info(f'attempt {attempt+1}: searching')
        if mongla.vision.align('person', yaw=0, gain=30, duration=20,
                               fallback=reacquire):
            log.info('centred')
            break
        log.warning('still not centred -- backing off & retrying')
        mongla.move_back(2.0, gain=50)

    mongla.disarm()


# Search fallback: back off and fan yaw to bring the target back into frame.
def reacquire(mongla, should_stop):
    mongla.move_back(1.5, gain=50)
    for _ in range(4):
        mongla.yaw_right(45)
        if should_stop():
            return
```

### 6.5  Full 3-axis align, then forward close-in

```python
def run(mongla, log):
    mongla.camera = 'forward'
    mongla.arm()
    mongla.set_depth(-1.5)
    # align owns the three centring axes at once (yaw + lat + depth);
    # then move owns the forward close-in. Two verbs, full lock.
    mongla.vision.align('gate', yaw=0, lat=0, depth=0,
                        gain=30, duration=20, fallback=creep_forward)
    mongla.vision.move('gate', fwd=80, mode='area', gain=35, duration=20)
    mongla.disarm()


def creep_forward(mongla):
    mongla.move_forward(0.6, gain=40)
```

### 6.6  Patrol pattern: drive, scan, drive, scan

```python
def run(mongla, log):
    mongla.camera = 'forward'
    mongla.arm()
    mongla.set_depth(-0.8)

    for leg in range(3):
        log.info(f'leg {leg}: drive')
        mongla.move_forward(5.0, gain=55)
        log.info(f'leg {leg}: scan')
        if mongla.detected('gate', stale_after=0.5):
            log.info('found -- homing in')
            mongla.vision.align('gate', yaw=0, lat=0, gain=30, duration=15)
            mongla.vision.move('gate', fwd=80, mode='area', gain=35, duration=15)
            break
        log.info('no hit, continue patrol')
        mongla.yaw_right(60.0)

    mongla.disarm()
```

### 6.7  Per-call tuning (pin precision for one phase)

```python
def run(mongla, log):
    mongla.camera = 'forward'
    mongla.arm()
    mongla.set_depth(-0.5)
    # Tight, slow yaw-only lock: small err (precise) + low gain (gentle).
    # The P-gains (vision.kp_yaw, …) are deck-side ROS params only — set them
    # with `ros2 param set /mongla_manager vision.kp_yaw 85.0`.
    mongla.vision.align('gate', yaw=0, err=12, gain=20, duration=10)
    mongla.disarm()
```

### 6.8  Camera switching mid-mission (forward → downward)

`mongla.camera` is a sticky string. Assign it once; all subsequent `mongla.vision.*` calls use that
camera. The camera name must match a running `camera_node` profile — verify with
`ros2 topic list | grep image_raw`.

```python
def run(mongla, log):
    mongla.arm()
    mongla.set_depth(-0.8, settle=1.0)

    # Phase 1: gate approach with the forward camera
    mongla.use_camera('forward')
    mongla.vision.align('gate', yaw=0, lat=0, gain=30, duration=15)
    mongla.vision.move('gate', fwd=80, mode='area', gain=35, duration=15)

    # Phase 2: descend and switch to the downward camera for the bin
    mongla.set_depth(-1.5, settle=1.5)
    mongla.use_camera('downward')
    # downward cam: lat = left/right, depth axis = fore/aft (see §6.9)
    mongla.vision.align('bin', lat=0, depth=0, err=30, gain=30, duration=20)
    mongla.disarm()
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
mongla.use_camera('downward')

# CORRECT: lat + depth centre the AUV over the bin; yaw off.
mongla.vision.align('fire', lat=0, depth=0, err=30, gain=30, duration=20)

# WRONG: yaw-only does nothing useful from a top-down view.
# mongla.vision.align('fire', yaw=0, duration=20)
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
mongla.vision.align('red_pipe', yaw=0, lat=80, gain=30, duration=20)
mongla.vision.move('red_pipe', fwd=60, mode='height', maintain=80,
                   gain=35, duration=15)
```

**Torpedo board bullseye** — aim 60 px right and 40 px above the board centre
(one horizontal axis — `yaw` here — plus `depth` for the vertical):
```python
mongla.vision.align('torpedo', yaw=60, depth=-40,
                    err=14, gain=20, duration=15)
```

> **Renamed/removed:** the old `offset_x` / `offset_y` kwargs and the per-axis
> verbs they rode on (`vision_align_yaw`, `vision_align_lat`,
> `vision_align_depth`, `vision_align_3d`, `vision_hold_distance`,
> `vision_lock_fire`, `vision_acquire`, `look_around`) were all deleted in the
> two-verb rewrite. Use the axis values on `align` / `maintain` on `move`.

### 6.12  Torpedo firing (align → move → fine-lock → fire)

There's no lock-fire verb. Firing is just **`if align(...): fire(n)`** — the
fine `align` is truthy only when the hole is centred within `err`, so it is its
own fire gate. Coarse-align the board, close in on `blood`, then take a tight
`hole` lock.

**Pattern: always call `mission_reset()` first.** This stops any leftover heading lock and clears
the abort event from the previous run — critical for back-to-back pool runs.

```python
def run(mongla, log):
    mongla.mission_reset()   # ★ ALWAYS first — clears heading lock + abort from previous run
    mongla.camera = 'forward'
    mongla.models(torpedo='torpedo_blood_hole')   # classes: torpedo, blood, hole
    mongla.arm()
    mongla.set_depth(-1.2)

    # 1. Coarse board align (yaw + lat + depth); creep forward to find it.
    mongla.vision.align(mongla.models.torpedo.torpedo, yaw=0, lat=0, depth=0,
                        err=40, gain=30, duration=15, fallback=creep_forward)

    # 2. Approach: drive forward until 'blood' fills 30% of frame height.
    mongla.vision.move(mongla.models.torpedo.blood, fwd=30, mode='height',
                       gain=35, duration=30, fallback=creep_forward)

    # 3. Fine hole lock (tight err, slow gain). Fire ONLY on a confirmed lock.
    if mongla.vision.align(mongla.models.torpedo.hole, yaw=0, lat=0, depth=0,
                           err=14, gain=12, duration=25, fallback=creep_forward):
        mongla.fire(1)       # torpedo_1 — 1/2 = torpedo, 3/4 = dropper
        log.info('torpedo fired on stable lock')
    else:
        log.warning('hole never locked — holding fire')

    mongla.move_forward(2.0, gain=40)   # clear the board
    mongla.disarm()


def creep_forward(mongla):
    mongla.move_forward(0.6, gain=40)
```

**Key fields:**

| Field | Notes |
|---|---|
| `mongla.fire(channel)` | 1/2 = torpedo, 3/4 = dropper. **Always pass the channel explicitly.** |
| `err` (fine lock) | small px tolerance (e.g. 14) so the fire only triggers dead-on. |
| `gain` (fine lock) | low (e.g. 12) — gentle, precise corrections at close range. |
| `fwd` / `mode` | approach stop: drive until `blood` fills `fwd`% by `mode='height'`. |
| `align(...)` truthiness | truthy only when locked within `err` → use it as the fire gate. |

> **Full two-verb torpedo missions:** `missions/task_torpedo.py` (chunk) and
> `missions/pool_day_practice.py` (Phase 3) — run with
> `ros2 run mongla_planner mission <name>`.

---

### 6.13  Bin drop (downward camera + dropper)

Centre over the bin on the downward camera with `align` (lat + depth), then
drop. `align` is truthy only when centred within `err`, so it gates the drop.

```python
def run(mongla, log):
    mongla.mission_reset()
    mongla.models(bin='bin_fire_blood')      # classes: blood, fire
    mongla.arm()
    mongla.set_depth(-1.5)

    # Fly over and centre on the bin with the downward camera.
    mongla.use_camera('downward')
    if mongla.vision.align(mongla.models.bin.fire, lat=0, depth=0,
                           err=30, gain=30, duration=45, fallback=creep_forward):
        mongla.pause(3.0)        # stability confirmation before the drop
        mongla.fire(3)           # dropper_1 — channel always explicit
    else:
        log.warning('bin never centred — skipping drop')

    mongla.use_camera('forward')
    mongla.disarm()


def creep_forward(mongla):
    mongla.move_forward(0.6, gain=40)
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
mongla.vision.move('gate', fwd=70, mode='area', hold=5.0, gain=35, duration=25)
mongla.move_forward(2.0, gain=60)

# Or: centre over a bin, hold briefly, then drop.
mongla.use_camera('downward')
mongla.vision.align('fire', lat=0, depth=0, err=30, gain=30, duration=20)
mongla.pause(3.0)      # let it settle
mongla.fire(3)         # dropper_1
```

**Arrive vs. stay:**
- `vision.align(...)` / `vision.move(...)` — exit as soon as centred / fill reached.
- `vision.move(..., hold=S)` — reach the fill, then station-keep for `S` seconds.
- `if vision.align(...): mongla.fire(n)` — align, confirm the lock, fire.

---

## 7. Vision queries — `detected()` / `wait_for()` / `where()`

Three client-side reads of the `/detections` stream let open-loop motion react
to what the camera sees, then hand off to vision-closed control. Each **pumps
the node** so the answer is the current frame (the default camera is subscribed
eagerly, so the first call never false-negates):

- `mongla.detected(class, *, camera=None, stale_after=1.0) -> bool` — visible
  right now? (point-in-time, case-insensitive)
- `mongla.wait_for(class, *, timeout=10.0, ...) -> bool` — block until seen or
  timeout (loop-free acquire while stationary)
- `mongla.where(class, *, band=0.15, ...) -> 'left'|'center'|'right'|'unknown'`
  — bearing of the largest match (`where_offset` → signed `[-1,+1]`)

This is the architecture step toward a declared plan — each `while detected()` loop
IS a proto-state.

**Full deep-dive reference:** [`.claude/context/missions/detected-paradigm.md`](detected-paradigm.md)

#### The three canonical patterns

```python
# 1. CIRCLE SEARCH — keep turning until seen (a moving search NEEDS a while;
#    an `if` runs once and falls through, it is not a loop)
while not mongla.detected('red_pipe'):
    mongla.yaw_left(30)
mongla.move_forward(3)            # runs once the pipe is in frame

# 2. ACQUIRE-THEN-ACT — wait in place, no busy-loop, with a give-up branch
if mongla.wait_for('gate', timeout=8):
    mongla.vision.align('gate', yaw=0, lat=0)
else:
    mongla.move_forward(1)        # never showed — recover

# 3. BEARING STEER — turn toward whichever side the target is on
{'left':  lambda: mongla.yaw_left(20),
 'right': lambda: mongla.yaw_right(20),
}.get(mongla.where('gate'), lambda: mongla.move_forward(1))()
```

#### Core paradigm

```python
_FWD = '/mongla_detector_forward'

def run(mongla, log):
    mongla.camera = 'forward'
    mongla.set_model('sauvc_sim', node=_FWD)          # the ONE line naming the model
    mongla.set_classes('qual_gate,orange_flare', node=_FWD)
    mongla.arm()
    mongla.set_depth(-0.8)

    # ── Search: creep forward until the gate is visible ───────────────── #
    MAX_STEPS = 60   # safety budget: 60 × 0.5 s = 30 s of search
    for _ in range(MAX_STEPS):
        if mongla.detected('qual_gate', stale_after=0.5):
            break
        mongla.move_forward(0.5, gain=30)  # 0.5 s steps → ≤ 0.15 m overshoot
    else:
        log.warn('gate not found — surfacing')
        mongla.surface(); mongla.disarm(); return

    # ── Align and pass ────────────────────────────────────────────────── #
    mongla.vision.align('qual_gate', yaw=0, lat=0, gain=30, duration=20)
    mongla.vision.move('qual_gate', fwd=None, hold=2.0, gain=55, duration=20)

    # ── Search: yaw-sweep for the flare ───────────────────────────────── #
    for _ in range(36):                       # 36 × 10° = full turn
        if mongla.detected('orange_flare', stale_after=0.5):
            break
        mongla.yaw_right(10); mongla.pause(0.5)

    if mongla.detected('orange_flare', stale_after=0.5):
        mongla.vision.align('orange_flare', yaw=0, gain=30, duration=20)
        mongla.vision.move('orange_flare', fwd=60, mode='height',
                           gain=35, duration=20)

    mongla.surface()
    mongla.disarm()
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
`False` until you call `mongla.set_classes('gate,flare')`.

**Rule 4 — Set the camera first.** `detected()` falls back to `mongla.camera`
(default `'forward'`). Set it explicitly to the camera you mean — e.g.
`mongla.use_camera('downward')` before a bin search.

#### What blocks vs what doesn't

**Blocking (action round-trip — where mission time is spent):**
`move_forward`, `move_back`, `move_left`, `move_right`, `yaw_left`, `yaw_right`,
`turn`, `set_depth`, `arm`, `disarm`, `pause`, `stop`, `surface`, ALL `vision.*`
verbs.

**Instant (no action goal):**
`mongla.camera =`, `mongla.can(...)`, `detected()` / `where()` (bounded pumps).

`detected()`/`where()` **pump the node themselves** (bounded: ≤0.25 s warm,
≤0.60 s cold-on-first-frame), so they read the current frame regardless of
whether a verb ran just before — no warm-up move needed. `wait_for` pumps until
seen or its `timeout`.

#### Forbidden patterns

```python
# ✗ Step too long — 0.6m overshoot at gain=30
while not mongla.detected('gate'):
    mongla.move_forward(2.0, gain=30)

# ✗ No safety budget — runs forever if detector offline
while not mongla.detected('gate'):
    mongla.move_forward(0.5)

# ✗ Class filter trap — the allowlist only names the flare, so the gate never arrives
mongla.set_classes('orange_flare', node=_FWD)
for _ in range(18):
    if mongla.detected('qual_gate'):   # ALWAYS FALSE until set_classes includes it
        break

# ✗ Wrong camera — polling 'forward' while the bin is under the downward cam
mongla.use_camera('forward')
while not mongla.detected('bin'):   # never True — bin is on 'downward'
    ...
```

#### `detected()` + `VisionResult` together (full decision tree)

A vision verb never raises — it returns a `VisionResult` that is truthy only on
success. Branch on it directly; pair it with `detected()` to pick the next target.

```python
# Try to centre on the gate; branch on whether it actually aligned.
if mongla.vision.align('gate', yaw=0, lat=0, gain=30, duration=20,
                       fallback=sweep_for_gate):
    mongla.vision.move('gate', fwd=80, mode='area', gain=35, duration=20)
    mongla.move_forward(4.0, gain=60)       # timed: there is no DVL
else:
    log.warn('gate never centred — advancing blindly')
    mongla.move_forward(4.0, gain=35)

# Confirm a target is still visible after a maneuver, then align on it.
mongla.yaw_right(45)
if mongla.detected('gate', stale_after=0.5):
    mongla.vision.align('gate', yaw=0, lat=0, gain=30, duration=15)
elif mongla.detected('flare', stale_after=0.5):
    mongla.vision.align('flare', yaw=0, depth=0, gain=30, duration=15)
else:
    log.warn('no targets visible after yaw step')
```

#### API quick reference

```python
mongla.detected(
    target_class,             # str | ClassRef — e.g. 'gate', mongla.models.gate.gate
    *,
    camera: str | None = None,      # defaults to mongla.camera
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
ros2 topic hz /mongla/vision/forward/detections   # should be 15-25 Hz

# 2. Confirm class names match your code strings (case-sensitive)
ros2 topic echo /mongla/vision/forward/detections --once   # look for class_id

# 3. Confirm class filter correct
ros2 param get /mongla_detector_forward classes   # what the allowlist is now

# 4. Test one detected() call from CLI (Python one-liner approach):
# copy the detected_test.py example from detected-paradigm.md §8.2 into missions/ first
```

Full testing guide: [`.claude/context/missions/detected-paradigm.md §8`](detected-paradigm.md).

## 7.5  Competition mission chunks

The full 2026 competition run is built as five standalone chunk files directly
in `missions/` plus a combinator in `missions/task_full_2026.py`.
Each chunk can be run independently with `ros2 run mongla_planner mission <name>`.
The flat layout is required: `discover()` only globs `missions/*.py` (non-recursive).

**Mission naming convention (commit cec7f37):**

| Category | Prefix | Examples |
|---|---|---|
| Competition chunks (detected-paradigm) | `task_` | `task_gate`, `task_slalom`, `task_bin`, `task_torpedo`, `task_return`, `task_full_2026` |
| Demos / development | `demo_` | `demo_arc`, `demo_square`, `demo_heading_lock`, `demo_move_see`, `demo_pursue`, `demo_find_person` |

### Key patterns introduced by the competition missions

#### Lazy detector activation (`pause_detector` / `resume_detector`)

Both cameras are always streaming. Detectors start `paused=True` in the
competition launch — inference only runs for the task that needs it.

```python
# resume forward detector before gate search
mongla.resume_detector('forward')   # → ros2 param set /mongla_detector_forward paused false

# ... gate task body ...

# pause again when done (frees GPU for the next chunk's detector)
mongla.pause_detector('forward')    # → ros2 param set /mongla_detector_forward paused true
```

#### Dual-camera node naming — pass `camera=`

There is one naming rule for the whole stack: the detector node is
`/mongla_detector_<camera>`, so `vision_dual.launch.py` creates
`/mongla_detector_forward` and `/mongla_detector_downward`. Every DSL helper
derives the node from its `camera` argument (default = the mission's sticky
camera), so just pass `camera=`:

```python
mongla.set_model('gate_sharks', camera='forward')   # → /mongla_detector_forward
mongla.set_classes('gate,shark,shaw_fish', camera='forward')
mongla.resume_detector('forward')                          # camera arg drives the node name
mongla.set_model('bin_fire_blood', camera='downward')      # → /mongla_detector_downward
```

#### Bounded search — never infinite forward

Every search loop has an explicit budget and a yaw-sweep fallback:

```python
MAX_STEPS = 20
for _ in range(MAX_STEPS):
    if mongla.detected('gate', stale_after=1.0):
        break
    mongla.move_forward(1.5, gain=40)    # 1.5 s steps — short enough to re-check
else:
    for _ in range(18):                  # 18 × 20° = 360° orbit fallback
        if mongla.detected('gate', stale_after=0.5):
            break
        mongla.yaw_right(20); mongla.pause(1.0)
```

#### Gate pass with bbox-fill exit (`move`)

The gate chunk exits forward drive when the gate bbox fills 80% of the frame —
that's exactly what `move` does:

```python
mongla.vision.move('gate', fwd=80, mode='height', gain=35, duration=25)
# exits automatically when the gate fills 80% of frame height = "through"
```

#### Downward camera centering (bin task)

With the downward camera, `align` re-maps the axes for you: `lat` is left/right
and `depth` is fore/aft (the `depth_sign` flip is automatic). Pass `lat`/`depth`
offsets — never `yaw` — to centre over the bin:

```python
# Centre over the bin: lat (left/right) + depth (fore/aft), no yaw.
mongla.use_camera('downward')
mongla.vision.align('fire', lat=0, depth=0, err=30, gain=30, duration=20)
```

Tuning that used to be per-call kwargs (`kp_lat`, deadband) is now ROS params —
`vision.kp_lat`, `vision.align_stable_frames` — set on the deck, not in the mission.

#### Fire channels — always explicit

There's no lock-fire verb. Fire after a truthy fine `align` (its own gate), and
always name the channel:

```python
mongla.fire(3)                        # dropper_1 — always explicit
if mongla.vision.align('hole', lat=0, depth=0, err=14, gain=12, duration=15):
    mongla.fire(1)                    # torpedo_1 — only when locked dead-on
```

### Pool-day constants (`competition_config.py`)

All depths, headings, bbox fractions, and search budgets live in one file:

```python
from mongla_planner.missions.competition_config import (
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
def run(mongla, log=None):
    gate    = _chunk('task_gate')
    slalom  = _chunk('task_slalom')
    _bin    = _chunk('task_bin')
    torpedo = _chunk('task_torpedo')
    _return = _chunk('task_return')
    try:
        mongla.pause(10.0)
        mongla.arm()
        mongla.set_depth(GATE_SEARCH_DEPTH_M, timeout=30)
        if mongla.can('lock_heading'):   # refused on srot: the board holds heading
            mongla.lock_heading(0.0, timeout=600)
        for name, chunk in [('gate',gate),('slalom',slalom),('bin',_bin),
                             ('torpedo',torpedo),('return',_return)]:
            try:
                chunk.run(mongla)
            except Exception as exc:
                if log: log(f'[MISSION] {name} FAILED: {exc} — continuing')
    finally:
        mongla.release_heading()   # ← release_heading, not unlock_heading
        mongla.stop(); mongla.disarm()
```

### A declared plan (`run_plan`)

The YASMIN FSM layer was **retired on 2026-09-22** — 3,550 lines that had never
executed, carrying a defect that ended every run one state after DIVE. What it was
supposed to give (per-task skip, structured outcomes, a deadline per task) is now in
the DSL itself, and a mission declares rather than sequences:

```python
mongla.use_budget(900, reserve_s=60)
mongla.run_plan([
    mongla.step('gate',   points=100, worst_case_s=120, run=gate,
                fallback=blind_transit, fallback_s=25, fallback_points=40),
    mongla.step('bins',   points=200, worst_case_s=180, run=bins),
    mongla.step('flares', points=50,  worst_case_s=90,  run=flares, needs='fire'),
])
```

`run_plan` asks the budget for a verdict **between** steps, from the live clock —
never once, up front. It skips a step whose `needs=` verb the backend refuses, runs
each body inside its own deadline, catches `TaskAbandoned` and keeps swimming, and
puts every outcome on the scorecard.

### Individual chunk test commands

```bash
# ✅ runnable today (gate_sharks.pt exists)
ros2 run mongla_planner mission task_gate
ros2 run mongla_planner mission task_return

# ⏳ logic test (model missing — verify search + timeout + abort flow)
ros2 run mongla_planner mission task_slalom
ros2 run mongla_planner mission task_bin
ros2 run mongla_planner mission task_torpedo

# full 5-task runs
ros2 run mongla_planner mission task_full_2026   # detected-paradigm
ros2 run mongla_planner mission sauvc_full       # a declared plan (run_plan)
```

See `packages/README.md §3` for per-chunk expected outputs and `models/README.md §Competition models` for model status.

---

## 8. Designing your own mission, step by step

1. **Pick a stable starting condition.** `arm()` then `set_depth(-0.5)`
   puts the hull clear of the surface, where the board holds depth.
2. **Decide what each phase isolates.** A phase that drives forward
   blindly is `move_forward`. A phase that centres on a target is
   `vision.align`; a phase that closes in on it is `vision.move`.
   Don't mix them inside one verb; chain them.
3. **Let the board hold heading and depth.** On srot every move is a
   `SROT_MOVE` the board runs with its own 500 Hz attitude and depth loops;
   there is nothing to lock from the host. (On the simulator's ArduSub,
   guard a host lock with `if mongla.can('lock_heading'):`.)
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
  may be a class string, a `mongla.models.<alias>.<class>` ClassRef, or
  sticky `mongla.target` — but the axis is never optional.
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
- **`mongla.stop()` between vision verbs is usually unnecessary** —
  every vision verb exits with a neutral RC write. Add `stop` only
  if you need an *extra* settle pause.

---

## 10. Cross-references

- Open-loop verb implementations: `src/mongla_control/mongla_control/{motion_forward,motion_lateral,motion_yaw,motion_depth,heading_lock}.py`
- DVL distance verbs:             `src/mongla_control/mongla_control/{motion_forward,motion_lateral}.py` (`drive_*_dist`)
- DVL sources:                    `src/mongla_sensors/mongla_sensors/sources/{nucleus_dvl,composite_bno_dvl}.py`
- Vision loop body:               `src/mongla_control/mongla_control/motion_vision.py`
- Vision state cache:             `src/mongla_manager/mongla_manager/vision_state.py`
- DSL surface:                    `src/mongla_planner/mongla_planner/mongla_dsl.py`
- ROS param defaults:             `src/mongla_manager/config/vision_tunables.yaml`
- DVL integration reference:      `.claude/context/platform/legacy-pixhawk-and-sitl.md`
- Sensors pipeline design:        `.claude/context/perception/sensors-pipeline.md`
- CLI cookbook (deck one-liners): `README.md` §9
- Architecture (visual flow):     `.claude/context/perception/vision-architecture.md`
- Roadmap (what's next):          `.claude/context/ROADMAP.md`
