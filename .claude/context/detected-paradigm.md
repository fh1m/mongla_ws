# `duburi.detected()` paradigm — complete reference

> **Purpose of this file:** Every fact, rule, gotcha, and test procedure
> for the `duburi.detected()` conditional-loop paradigm. Read this before
> writing any mission that uses `while not duburi.detected(...)` or branches
> on detection state.
>
> Implementation: [`src/duburi_planner/duburi_planner/duburi_dsl.py`](../../src/duburi_planner/duburi_planner/duburi_dsl.py)  
> Related: [`mission-cookbook.md §7.6`](./mission-cookbook.md) (samples), [`client-and-dsl-api.md`](./client-and-dsl-api.md) (API table)

---

## 1. What it is

`duburi.detected(class, *, camera=None, stale_after=1.0) -> bool`

A **recency observation query**: "was `class` seen within the last
`stale_after` s on `camera`?" It does not send a MAVLink command. It reads the
same `/duburi/vision/<cam>/detections` stream the control loop acts on, and
**actively pumps the ROS node** before answering so the cache is current — not
one left over from the last move.

**Per-class last-seen window (not the single latest frame).** `detected()` and
`wait_for()` track the last-seen monotonic stamp *per class* and return True
while `now - last_seen <= stale_after` (default 1.0 s). This is **flicker-
tolerant**: at low/uneven detection FPS (3-4 Hz on the Orin Nano, worse under
motion blur) a class routinely drops out of *individual* raw `/detections`
frames; keying off only the latest frame made `detected()` false-negate on
those gaps and a `while not detected(): <search>` loop could never break even
with the target plainly on the HUD. The window is the reacquire-side analogue
of the control loop's `lost_grace_s`. (The HUD looks continuous because it
overlays Kalman-smoothed `/tracks`, which predict through dropouts; these
queries read raw `/detections`, so "on screen" ≠ "in the latest raw frame" —
the window reconciles them.) `where()`/`where_offset()` deliberately stay on
the **current frame** (bearing must be the live position, never a remembered
spot), so `where()` can return `'unknown'` while `detected()` is still True
inside the window.

Two companions share the same machinery (added 2026-06):

- `duburi.wait_for(class, *, timeout=10.0, camera=None, stale_after=1.0) -> bool`
  — block until the class appears or `timeout` elapses. The loop-free way to
  acquire a target while stationary.
- `duburi.where(class, *, camera=None, stale_after=1.0, band=0.15) -> str`
  — bearing of the largest matching detection: `'left'` | `'center'` |
  `'right'` | `'unknown'`. `where_offset(...)` returns the raw signed
  normalized offset `[-1,+1]` for fine steering.

### `if` runs ONCE — a moving search needs a `while`

The single most common mistake (and the original "2nd command never runs"
bug report) is treating an `if` as a loop:

```python
# ✗ WRONG — an if executes once; this is NOT a circle search
if duburi.detected('gate'):
    duburi.move_forward(2)
else:
    duburi.yaw_left(90)        # runs at most once, then the script falls through

# ✓ CORRECT — a moving search loops until seen
while not duburi.detected('gate'):
    duburi.yaw_left(30)        # keep turning until the gate comes into frame
duburi.move_forward(2)         # runs once the loop exits
```

### The mental model

```
mission code                     |  what's happening
---------------------------------|-------------------------------------------
while not duburi.detected('gate')|  pump node → read current frame → False
    duburi.yaw_left(30)          |  blocking action round-trip (hull turns)
# → loop exits when gate visible |  next detected() pumps → sees gate → True
duburi.vision.align('gate',      |  vision P-loop centres on gate bbox
                    yaw=0, lat=0) |  (signed pixel offsets; 0 = centre)
```

---

## 2. Internal mechanics (why it is reliable)

> **History:** before 2026-06, `detected()` lazily subscribed on first call
> and did a single `spin_once(0.05)`. With no background executor, the first
> poll raced DDS discovery (50–500 ms) and a lone poll serviced one callback —
> so a `while not detected()` loop would spin forever even with the target in
> frame. The fix below (eager subscribe + active pump) removes both failure
> modes.

### 2.1 Subscription lifecycle — eager, not lazy

The **default camera is subscribed in `__init__`**, and `use_camera(name)`
subscribes the new camera immediately. So DDS discovery completes long before
the first `detected()`/`where()`, and the first poll cannot false-negate.
`_subscribe_detections(cam)` is idempotent and creates two subs per camera:
`/duburi/vision/<cam>/detections` (RELIABLE depth-10, matching `VisionState`)
and `/duburi/vision/<cam>/camera_info` (for the image width `where()` needs).

### 2.2 Callback: eager record extraction

`_on_detections` parses each frame into plain-Python `DetRecord` tuples
**immediately** — `(class_lower, cx_px, cy_px, w_px, h_px, conf)` — via
`_parse_detections`. The ROS message is never stored: rclpy reuses the C++
buffer across callbacks, so holding it would read the next frame's data. The
class/bbox/score fields are read through `_det_class_id` / `_det_center` /
`_det_score`, which handle both the Humble-flat and Iron+ nested message
layouts (mirrors `VisionState`'s helpers so queries agree with the control
path). The callback also marks the camera "warm".

### 2.3 Cache entry format

```python
self._det_cache[camera]  # dict[str, tuple[float, list[DetRecord]]]
                         #   value: (monotonic_stamp, [(cls, cx, cy, w, h, conf), ...])
                         #   the LATEST frame -- where()/where_offset() read this
self._det_seen[camera]   # dict[str, float]: class -> last-seen monotonic stamp
                         #   detected()/wait_for() read this (recency window)
self._img_size[camera]   # (width, height) from camera_info (for where())
```

### 2.4 The active pump inside every query

```python
def _pump_detections(self, camera):
    start = monotonic()
    budget = WARM(0.40s) if camera warm else COLD(0.60s)   # cold covers discovery
    while monotonic() < start + budget:
        rclpy.spin_once(node, 0.02)
        if cache[camera] stamped >= start:   # a frame newer than this call
            return
```

The detector publishes a `Detection2DArray` **every frame** (even when
empty), so a live pipeline lands a fresh frame within ~1 frame period and the
pump returns early. A stalled or just-subscribed pipeline burns the budget and
the query then leans on the per-class recency window (§1) rather than the
single latest frame. WARM is 0.40 s (~1.5 frames at 3-4 Hz); raise it if real
FPS is lower. The pump keeps the cache current; the window absorbs per-frame
flicker so a reacquire search breaks the instant the target is back.

### 2.5 Why this is safe (no executor race)

Queries run **between** goals only — the mission is single-threaded, so a
query never executes while `client.send()` is spinning on an action future.
The pump therefore never races the action client; it owns the node for its
bounded window and returns.

---

## 3. API reference

```python
duburi.detected(
    target_class,           # str | ClassRef — class name to look for
    *,
    camera: str | None = None,    # camera to query; defaults to duburi.camera
    stale_after: float = 1.0,     # detections older than this → absent
) -> bool
```

### Parameters

| Parameter | Type | Default | Meaning |
|-----------|------|---------|---------|
| `target_class` | `str` or `ClassRef` | required | Class name to search for: `'gate'`, `'flare'`, `duburi.models.gate.gate` |
| `camera` | `str` or `None` | `duburi.camera` | Which camera's detection topic to subscribe |
| `stale_after` | `float` | `1.0` | Detections older than this many seconds are treated as absent |

### Return value

`True` if `target_class` was present in the most recent detection frame AND
that frame arrived within `stale_after` seconds. `False` otherwise (no
subscription yet, cache empty, class absent, or frame too old).

Matching is **case-insensitive** — `detected('gate')`, `detected('Gate')`,
and `detected('GATE')` are equivalent, consistent with the control path
(`VisionState._hypothesis_matches` lowercases both sides). You still have to
use the right class *name*; only the case is forgiven.

### Using ClassRef

```python
duburi.models(gate='gate_flare_medium_100ep')

# ✓ ClassRef — passes through cleanly, reads .class_name
while not duburi.detected(duburi.models.gate.gate):
    duburi.move_forward(0.5, gain=30)

# ✓ string form — identical result
while not duburi.detected('gate'):
    duburi.move_forward(0.5, gain=30)
```

A `ClassRef` in `detected()` is **read-only** — it extracts `.class_name`
and does not call `set_model()` or `set_classes()`. That's intentional.
`detected()` is an observation query, not a detector control operation.
The same holds for `wait_for()` and `where()`.

### 3.1 `wait_for` — block until seen (loop-free acquire)

```python
duburi.wait_for(target_class, *, timeout=10.0,
                camera=None, stale_after=1.0) -> bool
```

Polls (and pumps) until `target_class` appears, returning `True` the moment
it does, or `False` if `timeout` elapses first. Use it to acquire/re-acquire
a target while holding station — no busy-loop, no `move` between polls:

```python
if duburi.wait_for('gate', timeout=8):
    duburi.vision.align('gate', yaw=0, lat=0)
else:
    # never appeared in 8 s -> mission-authored recovery (no `duburi.recover()` verb):
    while not duburi.detected('gate'):     # e.g. search while moving
        duburi.move_forward(0.6, gain=35)
```

`wait_for` is for waiting **in place**; to search while *moving*, use a
`while not detected(): <small move>` loop (§4 Rule 1).

### 3.2 `where` — bearing of the target

```python
duburi.where(target_class, *, camera=None,
             stale_after=1.0, band=0.15) -> str       # 'left'|'center'|'right'|'unknown'
duburi.where_offset(target_class, ...) -> float|None  # signed [-1,+1], None if unknown
```

Picks the **largest-area** matching detection and reports which side of frame
centre it sits on (`band` = centre dead-zone half-width, normalized).
`'unknown'` = not visible, or `camera_info` not seen yet. Image-frame
semantics: `'left'` = target on the left → yaw left to face it (same polarity
as the vision-yaw axis). Coarse steering:

```python
{'left':  lambda: duburi.yaw_left(20),
 'right': lambda: duburi.yaw_right(20),
}.get(duburi.where('gate'), lambda: duburi.move_forward(1))()
```

`where_offset` gives the continuous offset for proportional steering.

### 3.3 Use inside a vision `fallback`

These queries run between goals, which is exactly the context a vision
`fallback` search executes in. So a fallback can use them freely — the search
re-enters the verb the moment `detected()`/`wait_for()` sees the target:

```python
def sweep(duburi, should_stop):       # should_stop() is duburi.detected(target)
    for ang in (20, -40, 40):
        duburi.turn(duburi.head() + ang)
        if should_stop():
            return
duburi.vision.align('gate', yaw=0, lat=0, fallback=sweep)
```

---

## 4. Rules for correct use

### Rule 1 — Short steps in search loops (0.5 s or less)

```python
# ✓ CORRECT — 0.5s steps, max overshoot ~0.15m at gain=30
while not duburi.detected('gate'):
    duburi.move_forward(0.5, gain=30)

# ✗ WRONG — 2.0s steps, max overshoot ~0.6m at gain=30
while not duburi.detected('gate'):
    duburi.move_forward(2.0, gain=30)   # gate was seen 0.4s in, AUV drives 1.6s past it
```

Why: detection is checked **after** the blocking verb returns. If the target
becomes visible 0.1 s into a 2.0 s `move_forward`, the AUV continues driving
for the remaining 1.9 s before the check fires. At gain=30 (~0.3 m/s) that
is 0.57 m of overshoot. Use 0.5 s or less to keep overshoot below 0.15 m.

### Rule 2 — Always have a safety budget

```python
# ✗ No safety net — runs forever if detector is offline or target never appears
while not duburi.detected('gate'):
    duburi.move_forward(0.5, gain=30)

# ✓ Bounded — give up after N steps, handle failure explicitly
MAX_STEPS = 60   # 60 × 0.5s = 30s total search budget
for _ in range(MAX_STEPS):
    if duburi.detected('gate'):
        break
    duburi.move_forward(0.5, gain=30)
else:
    log('gate not found in search budget — surfacing')
    duburi.set_depth(0.0)
    duburi.disarm()
    return
```

### Rule 3 — Class filter coupling (the orbit trap)

`vision.*` verbs that receive a `ClassRef` call `duburi.set_classes()` before
firing. This changes what the detector publishes. After that call, **only the
specified class appears in the detection topic** — `detected()` for any other
class will always return `False`.

```python
# ✗ BUG: vision.align with flare ClassRef sets classes='flare'
#         Then detected('gate') can never be True — detector only publishes flare
duburi.vision.align(duburi.models.gate.flare, yaw=0)  # ← sets classes='flare'
for _ in range(18):
    if duburi.detected('gate'):   # ← ALWAYS FALSE — detector filtered to flare only
        break
    duburi.yaw_right(20)
```

```python
# ✓ CORRECT: restore both classes before the loop
duburi.vision.align(duburi.models.gate.flare, yaw=0)  # sets classes='flare'
duburi.set_classes('gate,flare')   # ← restore detection of both
for _ in range(18):
    if duburi.detected('gate', stale_after=0.3):   # ← NOW works
        break
    duburi.yaw_right(20)
    duburi.pause(1.0)
```

**The general rule:** whenever you need `detected('X')` after a vision verb
that used a `ClassRef` for a different class, call `duburi.set_classes(...)` 
to restore the filter before the loop.

### Rule 4 — `stale_after` tuning by use case

| Scenario | Recommended `stale_after` | Reason |
|----------|--------------------------|--------|
| Gate detection while creeping forward | 0.5 – 1.0 s | Short steps; cache updates ~every 0.5 s |
| Orbit gate-break check | 0.3 s | You're at each yaw stop for ~1 s; want fresh confirmation |
| Target may flicker (turbid water) | 1.5 – 2.0 s | Allow drop-out during flicker |
| Confirm target is still there after maneuver | 0.5 s | Verify fresh frame, not stale cache |
| Search loop with 1 s pause steps | 1.0 s | Steps update cache every 1 s |

### Rule 5 — A poll-loop now works, but do something useful in it

Since `detected()` pumps the node itself, a bare `while not detected(): pass`
will **exit correctly** when the target appears (the old lazy version could
hang). But it busy-spins the CPU doing nothing. Prefer `wait_for` (to wait in
place) or a small move (to search):

```python
# ✓ BEST for waiting in place — one call, no busy-loop
if duburi.wait_for('gate', timeout=10):
    ...

# ✓ search WHILE moving — the loop both polls and makes progress
while not duburi.detected('gate'):
    duburi.move_forward(0.5, gain=30)

# ⚠ works but wasteful — busy-polls (~8 Hz) doing nothing; use wait_for instead
while not duburi.detected('gate'):
    pass
```

### Rule 6 — Camera defaults to `'forward'`; switch it for the downward cam

`duburi.camera` now defaults to `'forward'` (the `DuburiMission` constructor
default), which matches the forward detector topic. Forward-camera missions
work without setting anything. For a downward-camera task you must switch
first, or `detected()` subscribes the wrong topic.

```python
# ✓ forward cam — default is already 'forward'
while not duburi.detected('gate'):       # /duburi/vision/forward/detections
    duburi.move_forward(0.5, gain=30)

# ✓ downward cam — switch before the loop (use_camera logs the change)
duburi.use_camera('downward')
while not duburi.detected('bin'):         # /duburi/vision/downward/detections
    duburi.move_forward(0.5, gain=30)

# ✓ or override per call without changing the sticky context
while not duburi.detected('bin', camera='downward'):
    duburi.move_forward(0.5, gain=30)
```

---

## 5. What blocks, what doesn't

Understanding this is essential for designing detected()-paradigm missions.

### Blocking (each line waits for action server round-trip)

These run the action round-trip (and incidentally service callbacks).
`detected()` no longer depends on that — it pumps the node itself — but these
are still where the *time* in a mission is spent:

```python
duburi.move_forward(s)      # Ch5 RC override for s seconds
duburi.move_back(s)
duburi.move_left(s)
duburi.move_right(s)
duburi.yaw_left(deg)        # SET_ATTITUDE_TARGET, waits for settle
duburi.yaw_right(deg)
duburi.arc(s)
duburi.set_depth(m)
duburi.arm()
duburi.disarm()
duburi.pause(s)             # NO_OVERRIDE for s seconds
duburi.stop()
duburi.lock_heading(deg)
duburi.dvl_connect()
duburi.move_forward_dist(m)
duburi.move_lateral_dist(m)
duburi.vision.align(...)   # centre on lat/yaw/depth (signed px offsets)
duburi.vision.move(...)    # drive forward to a bbox fill ratio
```

### Non-blocking (return almost immediately, do NOT update cache meaningfully)

```python
duburi.camera = 'forward'           # attribute assignment
duburi.target = 'gate'              # attribute assignment
duburi.models(gate='model_name')    # model registration (Python object)
```

### Near-instant with subprocess overhead (~100ms for ros2 param set)

```python
duburi.set_classes('gate,flare')    # ros2 param set subprocess
duburi.set_model('gate_model')      # ros2 param set subprocess
duburi.use('model', 'gate')         # two subprocess calls
```

These are non-blocking in the mission sense but do a subprocess call. Call
them once before loops, not inside tight detection loops.

---

## 6. Allowed and forbidden patterns

### 6.1 Allowed — safe and idiomatic

**Forward search until gate:**
```python
while not duburi.detected('gate'):
    duburi.move_forward(0.5, gain=30)
```

**Yaw sweep search:**
```python
for _ in range(36):   # 36 × 10° = full 360°
    if duburi.detected('gate'):
        break
    duburi.yaw_right(10)
    duburi.pause(0.5)
```

**Conditional branch:**
```python
if duburi.detected('flare', stale_after=0.5):
    duburi.vision.align('flare', yaw=0, depth=0)
else:
    duburi.move_forward(2.0, gain=35)
```

**While gate visible — crude approach:**
```python
# Move toward gate as long as it's in view (crude, use vision.move for precision)
while duburi.detected('gate', stale_after=0.5):
    duburi.move_forward(0.3, gain=25)
```

**Combined pause-and-check (wait for gate to drift into view):**
```python
while not duburi.detected('gate'):
    duburi.pause(0.5)   # wait in place; cache updated each pause
```

**Post-maneuver confirmation:**
```python
duburi.yaw_right(45)
# Confirm gate is now visible before committing to alignment
if duburi.detected('gate', stale_after=0.5):
    duburi.vision.align('gate', yaw=0, lat=0)
```

**Orbit with gate-break (correct class filter):**
```python
# After flare alignment, restore both classes then orbit
duburi.set_classes('gate,flare')
for _ in range(18):
    if duburi.detected('gate', stale_after=0.3):
        break
    duburi.yaw_right(20)
    duburi.pause(1.0)
```

### 6.2 Forbidden / will silently fail

**Busy poll-loop (works, but wasteful — use `wait_for`):**
```python
# ⚠ detected() pumps, so this DOES exit when the gate appears, but it
#   busy-spins (~8 Hz) doing nothing. Prefer wait_for('gate', timeout=...).
while not duburi.detected('gate'):
    pass
```

**Too-long steps:**
```python
# ✗ 5.0s step → up to 1.5m overshoot at gain=30
while not duburi.detected('gate'):
    duburi.move_forward(5.0, gain=30)
```

**Missing class filter restore:**
```python
# ✗ vision.align(flare_ref) calls set_classes('flare')
#    The loop below can never see gate
duburi.vision.align(duburi.models.gate.flare, yaw=0)
for _ in range(18):
    if duburi.detected('gate'):    # never True
        break
    duburi.yaw_right(20)
```

**Querying wrong camera:**
```python
# ✗ camera left at 'forward' but the bin detector publishes on 'downward'
while not duburi.detected('bin'):     # subscribes forward/detections (nothing there)
    duburi.move_forward(0.5)          # runs until budget exhausted
```

**Unbounded loops:**
```python
# ✗ If detector is offline, this runs forever
while not duburi.detected('gate'):
    duburi.move_forward(0.5, gain=30)
```

**Checking stale cache after a long pause:**
```python
# ✗ If stale_after=1.0 but the detector is running at 25 Hz, a 2.0s
#    pause where no callbacks fire could leave the cache stale.
#    (This can't actually happen because pause() calls send() which
#    spins until_future_complete — but be aware if you ever use bare time.sleep)
import time
time.sleep(2.0)                    # NEVER use in mission code
duburi.detected('gate')            # cache is 2.0s stale
```

---

## 7. Harmony with control commands — nothing blocks nothing

The detected paradigm is **purely sequential** on the mission/planner side.
There is no parallel thread contention because:

1. `detected()` lives in `DuburiMission` (planner side, `duburi_planner`).
2. All blocking verbs also live in `DuburiMission` (planner side).
3. The manager (controller side, `auv_manager_node`) is in a separate process.
4. Communication is over ROS2 actions — one goal at a time, fully serialised.

```
mission script (planner process)     manager process (control process)
─────────────────────────────────    ──────────────────────────────────
detected('gate') → pump → False      /duburi/vision/.../detections ← (detections topic)
move_forward(0.5) ──────────────────→  executes motion command (Ch5 RC)
detected('gate') → pump → True       reads the current frame itself
vision.align(...) ──────────────────→  executes vision P-loop
```

Key property: **at any given instant, at most one action goal is in flight**.
The manager serialises commands with a `threading.Lock`. The planner sends one
goal, blocks, gets the result, then sends the next. A query runs only between
goals, so its pump never races an in-flight action future.

### What a query (detected/wait_for/where) does NOT do

- Does NOT send any MAVLink messages.
- Does NOT send any ROS2 action goals.
- Does NOT change the detector's class filter (ClassRef is read-only here).
- Does NOT change any motion state.
- Does NOT block beyond its bounded pump window (`detected`/`where`: one
  pump, ≤0.25 s warm / ≤0.60 s cold; `wait_for`: until seen or its `timeout`).
  The warm budget (`_PUMP_WARM_S`) must be ≥ ~2 real frame-periods; if the
  detector runs slower than ~8 Hz, raise it (or the query falls back to the
  last cached frame, up to `stale_after` old).
- Does NOT interact with the heading lock or heartbeat.

It is a bounded pump + read of a local in-memory cache. The only persistent
side effect is subscribing to a camera's topics on first use (idempotent;
the default camera is subscribed eagerly at construction).

---

## 8. Testing the paradigm end-to-end

### 8.1 Verify the detection topic before running any mission

```bash
# Check detection topic is alive and streaming
ros2 topic hz /duburi/vision/forward/detections
# Expected: 15-25 Hz; anything below 5 Hz indicates detector issues

# Inspect one message — confirms class names match what you'll use in code
ros2 topic echo /duburi/vision/forward/detections --once
# Look for: detections[0].results[0].hypothesis.class_id = 'gate' (or 'flare')

# Check the class filter is correct
ros2 param get /duburi_detector classes
# Expected: 'gate' or 'gate,flare' for competition missions
```

### 8.2 Minimal detected()-paradigm test mission

Drop this in `missions/detected_test.py` for end-to-end verification:

```python
"""detected_test.py — unit-tests the detected() cache live.

Run with:
    ros2 run duburi_planner mission detected_test
Requires: detector_node publishing /duburi/vision/forward/detections.
Does NOT arm — safe on the bench.
"""
import time

def run(duburi, log):
    duburi.camera = 'forward'

    # 1. First check — eager subscribe + pump means this is already reliable
    #    (True if the gate is in view right now, no warm-up move needed).
    log.info('=== Test 1: first detected() (pumps the current frame) ===')
    result = duburi.detected('gate')
    log.info(f'detected("gate") = {result}')   # True iff gate visible now

    # 2. wait_for — block (in place) until the gate appears or timeout
    log.info('=== Test 2: wait_for (loop-free acquire) ===')
    seen = duburi.wait_for('gate', timeout=3.0)
    log.info(f'wait_for("gate", 3s) = {seen}')

    # 2b. where — bearing of the gate ('left'|'center'|'right'|'unknown')
    log.info(f'where("gate") = {duburi.where("gate")}')

    # 3. stale_after test
    log.info('=== Test 3: stale_after=0.01 (almost always False) ===')
    result = duburi.detected('gate', stale_after=0.01)
    log.info(f'detected("gate", stale_after=0.01) = {result}')  # almost always False

    # 4. Camera fallback — should warn if wrong camera
    log.info('=== Test 4: explicit camera ===')
    result = duburi.detected('gate', camera='forward')
    log.info(f'detected("gate", camera="forward") = {result}')

    # 5. ClassRef form
    duburi.models(gate='gate_flare_medium_100ep')
    log.info('=== Test 5: ClassRef form ===')
    result = duburi.detected(duburi.models.gate.gate)
    log.info(f'detected(models.gate.gate) = {result}')

    log.info('=== All tests complete — check logs above for expected values ===')
```

### 8.3 Observed behavior tests (with detection active)

**Test: step-and-detect loop**
1. Point camera at gate prop.
2. Run: `ros2 run duburi_planner mission detected_test` (or a loop mission).
3. Observer: `ros2 topic echo /duburi/vision/forward/detections --once`.
4. Expected: `detected()` returns `True` within one 0.5s step of gate entering frame.

**Test: class filter coupling**
1. Set `duburi.set_classes('flare')` manually via ros2 param.
2. Run `detected('gate')`.
3. Expected: always `False` (detector only publishes flare detections).
4. Set `duburi.set_classes('gate,flare')`.
5. Run `detected('gate')`.
6. Expected: `True` when gate in view.

**Test: stale detection**
1. Move gate object out of frame.
2. Wait 2 seconds.
3. Run `detected('gate', stale_after=1.0)`.
4. Expected: `False` (cache is >1s old).
5. Run `detected('gate', stale_after=3.0)`.
6. Expected: `True` (cache is 2s old, within 3s window).

**Test: detector offline**
1. Kill `detector_node` while a detected() loop is running.
2. Expected: `detected()` returns `False` (cache stops updating, hits stale_after).
3. AUV should hit its budget (`MAX_STEPS`) and fall back gracefully.

### 8.4 Integration test — full detected()-paradigm mission in sim

```bash
# Terminal 1: SITL
sim_vehicle.py -L RATBeach -v ArduSub -f vectored_6dof --model=JSON --out=udp:0.0.0.0:14550

# Terminal 2: manager
ros2 run duburi_manager start

# Terminal 3a: vision — webcam / sim, detect person with yolov11n (ROBOSUB tested ★)
ros2 launch duburi_vision cameras_.launch.py model:=yolov11n classes:=person

# Terminal 3b: vision — pool/competition gate+flare model (swap in for pool day)
ros2 launch duburi_vision cameras_.launch.py model:=gate_flare_medium_100ep classes:=gate,flare

# Terminal 4: run the autonomous mission (uses whatever classes the detector publishes)
ros2 run duburi_planner mission gate_flare_autonomous

# What to watch for:
# - AUV creeps forward in short steps until 'gate' appears in detections
# - Once gate detected, vision.align() takes over (smooth P-loop centring)
# - After align, vision.move() drives forward to the gate fill ratio
# - After gate pass, search loop for flare (yaw sweep + detected('flare'))
# - After flare lock: orbit with detected('gate') break in each step
# - Scoreboard JSON written on exit
```

### 8.5 Things to check and tweak

| What | How to check | Tuning lever |
|------|-------------|--------------|
| Detection topic flowing | `ros2 topic hz /duburi/vision/<cam>/detections` | Camera/detector pipeline must be up |
| Class names match code | `ros2 topic echo ... --once` and grep for `class_id` | Name must match your `detected('string')`; matching is case-insensitive (`'Gate'` == `'gate'`) |
| Cache freshness | `detected('gate', stale_after=0.1)` → should return True when gate visible | Tune `stale_after` for your use case |
| Correct camera | `ros2 topic echo /duburi/vision/<cam>/detections` exists | Set `duburi.camera = 'forward'` before first loop |
| Class filter not broken | `ros2 param get /duburi_detector classes` | Call `set_classes('gate,flare')` before orbit loop |
| Step size vs overshoot | Use `gain=30` + `step=0.5s` → max 0.15m overshoot | Reduce step size, reduce gain |
| Budget covers search area | 30 steps × 0.5s × 0.3m/s = 4.5m of search | Increase MAX_STEPS if pool lane is longer |

---

## 9. Common errors and fixes

### Error: `detected('gate')` always returns False even when gate is visible

**Likely causes (check in order):**

1. **Wrong camera** — `duburi.camera` defaults to `'forward'`. If your target
   is on the downward detector (bin / path marker), the subscription is on the
   wrong topic.
   ```python
   duburi.use_camera('downward')   # fix: switch before downward-cam loops
   ```

2. **Class filter set to wrong class** — a previous `vision.align(flare_ref)`
   set `classes='flare'`. The detector no longer publishes gate detections.
   ```python
   duburi.set_classes('gate,flare')   # fix: restore both classes
   ```

3. **Detector not running** — no `/duburi/vision/<cam>/detections` topic.
   ```bash
   ros2 topic list | grep detections   # should show your camera
   ```

4. **Class name mismatch** — matching is case-insensitive (`'Gate'` resolves
   to `'gate'`), so capitalisation is never the problem; a genuinely
   different name (e.g. `'gate_left'` vs `'gate'`) still misses. Confirm the
   published class_id:
   ```bash
   ros2 topic echo /duburi/vision/forward/detections --once | grep class_id
   ```

5. **Cache is stale** — detection happened but `stale_after` is too short.
   ```python
   detected('gate', stale_after=2.0)   # test with a generous window
   ```

6. **Model confidence too high** — detections below `conf` threshold are
   dropped. Try lowering confidence at launch:
   ```bash
   ros2 launch ... conf:=0.35
   ```

### Error: AUV overshoots the gate / doesn't stop when gate appears

**Cause:** step duration too long. Each detection check fires only after the
current verb returns.

**Fix:** reduce `move_forward` duration to 0.3–0.5 s:
```python
while not duburi.detected('gate'):
    duburi.move_forward(0.3, gain=30)   # 0.09m max overshoot at gain=30
```

### Error: Loop runs for budget steps even when gate visible (stuck False)

**Diagnosis:**
```bash
# Watch detections in real time while loop is running
ros2 topic echo /duburi/vision/forward/detections
```

If detections arrive but `detected()` returns False, class filter is the culprit.
Check: `ros2 param get /duburi_detector classes`.

### Error: `detected('gate')` in orbit loop never breaks, AUV spins forever

**Cause:** same as the class filter trap above. The flare alignment verb
set `classes='flare'`.

**Fix:** restore the filter before the orbit:
```python
duburi.vision.align(duburi.models.gate.flare, yaw=0, depth=0)   # ← sets classes='flare'
duburi.set_classes('gate,flare')   # ← REQUIRED before orbit
for _ in range(18):
    if duburi.detected('gate', stale_after=0.3):
        break
    duburi.yaw_right(20)
    duburi.pause(1.0)
```

### Error: `detected()` returns True briefly then False (flickering)

**Cause:** detection confidence is near the threshold; the target object is
at the edge of the frame or partially occluded.

**Fix options:**
1. Increase `stale_after`: `detected('gate', stale_after=1.5)` — sustains
   True through detection dropouts.
2. Lower detector confidence: `ros2 launch ... conf:=0.35`.
3. Hand off to a vision verb with a `fallback` search instead of relying on
   raw `detected()`. `duburi.vision.align('gate', yaw=0, lat=0,
   fallback=creep_forward)` rides brief dropouts (it coasts for
   `vision.lost_grace_s` before running the fallback) and re-acquires
   automatically.

---

## 10. Design patterns — canonical mission templates

### Pattern A: Search-then-align (the core paradigm)

```python
def creep_forward(duburi):
    """Fallback for vision.align/move: one short forward creep, then return."""
    duburi.move_forward(0.5, gain=30)

def run(duburi, log):
    duburi.camera = 'forward'
    duburi.models(gate='gate_flare_medium_100ep')
    duburi.arm()
    duburi.set_depth(-0.8)
    duburi.lock_heading(0.0, timeout=180)

    # Search for gate — creep forward until visible
    MAX_SEARCH_STEPS = 60
    for _ in range(MAX_SEARCH_STEPS):
        if duburi.detected(duburi.models.gate.gate, stale_after=0.5):
            break
        duburi.move_forward(0.5, gain=30)
    else:
        log.warn('gate not found — aborting')
        duburi.set_depth(0.0)
        duburi.disarm()
        return

    # Gate visible — centre it (yaw + lat), then drive through to the fill ratio
    duburi.vision.align(duburi.models.gate.gate, yaw=0, lat=0,
                        err=40, gain=30, duration=20, fallback=creep_forward)
    duburi.vision.move(duburi.models.gate.gate, fwd=80, mode='height',
                       gain=45, duration=20, fallback=creep_forward)
    duburi.move_forward_dist(3.0, gain=60)   # DVL commit through the gate

    duburi.release_heading()
    duburi.set_depth(0.0)
    duburi.disarm()
```

### Pattern B: Reactive sweep (search with yaw)

```python
# Yaw sweep: 10° increments, detect at each stop
for _ in range(36):                        # full 360°
    if duburi.detected('gate', stale_after=0.5):
        break
    duburi.yaw_right(10)
    duburi.pause(0.8)                      # dwell at each position
else:
    log.warn('gate not found in sweep')
    return
```

### Pattern C: Orbit with detection exit

```python
# After aligning to flare (centre yaw + depth, then close in):
duburi.vision.align(duburi.models.gate.flare, yaw=0, depth=0, fallback=creep_forward)
duburi.vision.move(duburi.models.gate.flare, fwd=38, mode='height', fallback=creep_forward)

# Restore both classes before orbit
duburi.set_classes('gate,flare')

# Orbit in 20° steps, break when gate re-appears
for _ in range(18):                        # 18 × 20° = 360°
    if duburi.detected('gate', stale_after=0.3):
        break
    duburi.yaw_right(20)
    duburi.pause(1.0)

# Re-align on gate if found
if duburi.detected('gate', stale_after=0.5):
    duburi.vision.align(duburi.models.gate.gate, yaw=0, lat=0, fallback=creep_forward)
    duburi.move_forward_dist(1.5, gain=60)
```

### Pattern D: Multi-target branch

```python
duburi.set_classes('gate,flare')
duburi.pause(1.0)   # warm the cache

if duburi.detected('gate', stale_after=0.5):
    log('gate visible — aligning')
    duburi.vision.align('gate', yaw=0, lat=0)
elif duburi.detected('flare', stale_after=0.5):
    log('flare visible but no gate — centre then approach')
    duburi.vision.align('flare', yaw=0, depth=0)
    duburi.vision.move('flare', fwd=38, mode='height')
else:
    log('nothing visible — advancing')
    duburi.move_forward(2.0, gain=35)
```

### Pattern E: Conditional DVL pass

```python
result = duburi.vision.align(
    duburi.models.gate.gate,
    yaw=0, lat=0,
    err=40, gain=30, duration=20,
)

if result.ok:
    # Vision aligned — use DVL for precise gate passage
    duburi.move_forward_dist(3.0, gain=60)
else:
    # Vision did not centre (gate moved or lost) — open-loop fallback
    log.warn('gate alignment failed — open-loop passage attempt')
    duburi.move_forward(4.0, gain=40)   # conservative open-loop
```

> `VisionResult` is truthy only on `ALIGNED`, so `if result.ok:` and
> `if result:` are equivalent. A miss never raises — the verb logs "not
> aligned" and the mission keeps going.

---

## 11. The paradigm in context of the mission architecture

```
YASMIN FSM (built)             What the imperative paradigm does
─────────────────────          ─────────────────────────────────────────
VisionSearchState          →   while not duburi.detected('gate'):
  SEARCH_GATE → ALIGN_GATE          duburi.move_forward(0.5, gain=30)

VisionAlignState           →   duburi.vision.align('gate', yaw=0, lat=0, ...)
  ALIGN_GATE → MOVE_GATE

VisionMoveState            →   duburi.vision.move('gate', fwd=80, mode='height')
  MOVE_GATE → PASS_GATE

MoveForwardState           →   duburi.move_forward_dist(3.0, gain=60)
  PASS_GATE → SEARCH_FLARE
```

Each `detected()`-based loop IS a proto-state. The YASMIN FSM layer
(`duburi_planner/state_machines/`) wraps the *same* DSL verbs as explicit
state nodes: `VisionSearchState` (search-until-detected), `VisionAlignState`
(wraps `vision.align`), and `VisionMoveState` (wraps `vision.move`). The
detected paradigm is the design that makes that mapping clean — every logical
state is already isolated in the mission script.

When missions outgrow linear scripts (more than ~3 tasks with retry logic),
the YASMIN FSM is the right structure — and each `while detected()` loop maps
1:1 to a `VisionSearchState`, each `vision.align`/`vision.move` to a
`VisionAlignState`/`VisionMoveState`.

---

## 12. Cross-references

- Implementation: `src/duburi_planner/duburi_planner/duburi_dsl.py` (`detected()` method, `_on_detections()` callback)
- Two-verb vision DSL: `src/duburi_planner/duburi_planner/vision_dsl.py` (`vision.align` / `vision.move`)
- Detection topic source: `src/duburi_vision/duburi_vision/detector_node.py` (publishes `Detection2DArray`)
- Vision state (manager side): `src/duburi_manager/duburi_manager/vision_state.py` (`bbox_error()`, used by vision verbs, NOT by `detected()`)
- Mission samples: `src/duburi_planner/duburi_planner/missions/gate_flare_autonomous.py` (canonical use), `pool_day_practice.py` (full two-verb run + fallbacks)
- Mission cookbook: `.claude/context/mission-cookbook.md` §7.6
- Client/DSL API: `.claude/context/client-and-dsl-api.md` §2.5
- YASMIN FSM states: `src/duburi_planner/duburi_planner/state_machines/states/vision.py` (`VisionSearchState` / `VisionAlignState` / `VisionMoveState`)
