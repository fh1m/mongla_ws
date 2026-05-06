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

A **non-blocking, cache-backed observation query**. It does not send a
MAVLink command. It does not block waiting for an action to complete. It
simply asks: "is `class` currently visible on `camera`?"

The answer is drawn from a per-camera timestamp-stamped cache that is
refreshed automatically during every blocking DSL verb (because each verb
calls `rclpy.spin_until_future_complete`, which processes all pending ROS
callbacks including the subscription on `/duburi/vision/<cam>/detections`).

### The three-line mental model

```
mission code                     |  what's happening
---------------------------------|-------------------------------------------
while not duburi.detected('gate')|  read cache → False (gate not visible yet)
    duburi.move_forward(0.5)     |  blocking action round-trip → cache updated
                                 |  spin_until_future_complete fires callbacks
# → loop exits when gate visible |  next detected() call → True
duburi.vision.home(target='gate')|  vision P-loop closes in on gate bbox
```

---

## 2. Internal mechanics (why it is safe)

### 2.1 Subscription lifecycle

On the **first call** for a given camera, `_subscribe_detections(cam)` is
called once. It creates a `Detection2DArray` subscriber on
`/duburi/vision/<cam>/detections` with depth 10 (RELIABLE QoS default). The
`sub` object is stored in `self._det_subs[cam]` to keep it alive for the
process lifetime.

Subsequent calls for the same camera skip this — no repeated allocation.

### 2.2 Callback: eager class extraction

```python
def _on_detections(self, camera: str, msg: Detection2DArray) -> None:
    names: set[str] = set()
    for d in msg.detections:
        if not d.results:
            continue
        hyp = d.results[0]
        if hasattr(hyp, 'hypothesis'):
            names.add(str(hyp.hypothesis.class_id))
        else:
            names.add(str(getattr(hyp, 'id', '')))
    self._det_cache[camera] = (time.monotonic(), names)
```

Class names are extracted to plain Python strings **immediately** in the
callback. The ROS message object is never stored in the cache. This is
critical: rclpy may reuse the underlying C++ `Detection2DArray` memory
across successive callbacks. Storing the message and reading it later would
read garbage or the next frame's data.

### 2.3 Cache entry format

```python
self._det_cache[camera]  # dict[str, tuple[float, set[str]]]
                         #   key:   camera name
                         #   value: (monotonic_stamp, {class_name, ...})
```

### 2.4 The `spin_once` call inside `detected()`

```python
rclpy.spin_once(self.client.node, timeout_sec=0.05)
```

Every `detected()` call spins the ROS event loop for up to 50 ms. This
guarantees:
- At least one callback cycle fires so brand-new detections reach the cache.
- The call is bounded and never hangs — `timeout_sec=0.05` is the ceiling.

50 ms is sufficient because the detector runs at 15–25 Hz (frame interval
40–66 ms). One `spin_once(0.05)` is enough to catch the most recent frame.

### 2.5 How blocking verbs keep the cache warm

Every blocking verb calls `self.client.send(cmd, **fields)` internally.
`send()` uses `rclpy.spin_until_future_complete(node, goal_handle_future, ...)`.
This spin processes ALL pending callbacks on the node's executor — including
the `/detections` subscription. So immediately after `move_forward(0.5)`
returns, `_det_cache[camera]` contains the detection state as of the last
frame received during the move. The cache is never stale right after a verb.

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
# ✗ BUG: vision.home with flare ClassRef sets classes='flare'
#         Then detected('gate') can never be True — detector only publishes flare
duburi.vision.home(target=duburi.models.gate.flare, yaw=True, ...)  # ← sets classes='flare'
for _ in range(18):
    if duburi.detected('gate'):   # ← ALWAYS FALSE — detector filtered to flare only
        break
    duburi.yaw_right(20)
```

```python
# ✓ CORRECT: restore both classes before the loop
duburi.vision.home(target=duburi.models.gate.flare, yaw=True, ...)  # sets classes='flare'
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

### Rule 5 — Never put detected() in a bare while True without any blocking call

```python
# ✗ WRONG — detected() calls spin_once(0.05) each iteration, so this
#            burns ~50ms per loop and hogs the node thread
while not duburi.detected('gate'):
    pass   # no blocking verb

# ✓ CORRECT — pause provides a real sleep + callback cycle
while not duburi.detected('gate'):
    duburi.pause(0.5)   # 0.5s sleep, cache updated

# ✓ CORRECT — move provides the blocking action cycle
while not duburi.detected('gate'):
    duburi.move_forward(0.5, gain=30)
```

### Rule 6 — Camera must be set before first use

```python
# ✗ detected() falls back to duburi.camera='laptop' (the constructor default)
#    if you forgot to set it. This subscribes the wrong topic.
while not duburi.detected('gate'):   # subscribes /duburi/vision/laptop/detections
    ...

# ✓ Set camera context at the top of run()
def run(duburi, log):
    duburi.camera = 'forward'
    ...
    while not duburi.detected('gate'):   # subscribes /duburi/vision/forward/detections
        ...
```

---

## 5. What blocks, what doesn't

Understanding this is essential for designing detected()-paradigm missions.

### Blocking (each line waits for action server round-trip)

Every one of these keeps the detection cache warm during execution:

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
duburi.vision.find(...)
duburi.vision.home(...)
duburi.vision.turn(...)
duburi.vision.slide(...)
duburi.vision.hover(...)
duburi.vision.approach(...)
duburi.vision.track(...)
duburi.vision.scan(...)
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
    duburi.vision.home(target='flare', yaw=True, depth=True)
else:
    duburi.move_forward(2.0, gain=35)
```

**While gate visible — crude approach:**
```python
# Move toward gate as long as it's in view (crude, use vision.home for precision)
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
    duburi.vision.home(target='gate', yaw=True, lat=True)
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

**Loop without motion verb:**
```python
# ✗ detected() calls spin_once(0.05) — so this runs ~20x/s with no sleep
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
# ✗ vision.home(target=flare_ref) calls set_classes('flare')
#    The loop below can never see gate
duburi.vision.home(target=duburi.models.gate.flare, yaw=True, ...)
for _ in range(18):
    if duburi.detected('gate'):    # never True
        break
    duburi.yaw_right(20)
```

**Querying wrong camera:**
```python
# ✗ duburi.camera still 'laptop' but detector is on 'forward'
while not duburi.detected('gate'):    # subscribes laptop/detections (nothing there)
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
detected('gate') → False             /duburi/vision/.../detections ← (detections topic)
move_forward(0.5) ──────────────────→  executes motion command (Ch5 RC)
  spin_until_future_complete         /duburi/vision/.../detections ← updated during spin
detected('gate') → True             (no action sent yet)
vision.home(...) ───────────────────→  executes vision P-loop
```

Key property: **at any given instant, at most one action goal is in flight**.
The manager serialises commands with a `threading.Lock`. The planner sends one
goal, blocks, gets the result, then sends the next. There is never a race
between detected() and a motion command.

### What detected() does NOT do

- Does NOT send any MAVLink messages.
- Does NOT send any ROS2 action goals.
- Does NOT change the detector's class filter.
- Does NOT change any motion state.
- Does NOT block (beyond the 50ms spin_once timeout).
- Does NOT interact with the heading lock or heartbeat.

It is a pure read of a local in-memory dict. The only side effect is
subscribing to a topic on first call (once per camera per process lifetime).

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

    # 1. Cold-cache check (no blocking verb yet)
    log.info('=== Test 1: cold cache (expect False) ===')
    result = duburi.detected('gate')
    log.info(f'detected("gate") = {result}')   # may be False (cache cold)

    # 2. After a pause, cache should be warm
    duburi.pause(1.0)
    log.info('=== Test 2: after pause (cache updated during spin) ===')
    result = duburi.detected('gate')
    log.info(f'detected("gate") = {result}')   # True if gate in view, False if not

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
# - Once gate detected, vision.home() takes over (smooth P-loop alignment)
# - After gate pass, search loop for flare (yaw sweep + detected('flare'))
# - After flare lock: orbit with detected('gate') break in each step
# - Scoreboard JSON written on exit
```

### 8.5 Things to check and tweak

| What | How to check | Tuning lever |
|------|-------------|--------------|
| Detection topic flowing | `ros2 topic hz /duburi/vision/<cam>/detections` | Camera/detector pipeline must be up |
| Class names match code | `ros2 topic echo ... --once` and grep for `class_id` | Must match your `detected('string')` exactly — case-sensitive |
| Cache freshness | `detected('gate', stale_after=0.1)` → should return True when gate visible | Tune `stale_after` for your use case |
| Correct camera | `ros2 topic echo /duburi/vision/<cam>/detections` exists | Set `duburi.camera = 'forward'` before first loop |
| Class filter not broken | `ros2 param get /duburi_detector classes` | Call `set_classes('gate,flare')` before orbit loop |
| Step size vs overshoot | Use `gain=30` + `step=0.5s` → max 0.15m overshoot | Reduce step size, reduce gain |
| Budget covers search area | 30 steps × 0.5s × 0.3m/s = 4.5m of search | Increase MAX_STEPS if pool lane is longer |

---

## 9. Common errors and fixes

### Error: `detected('gate')` always returns False even when gate is visible

**Likely causes (check in order):**

1. **Wrong camera** — `duburi.camera` defaults to `'laptop'`. If your detector
   is on `'forward'`, the subscription is on the wrong topic.
   ```python
   duburi.camera = 'forward'   # fix: set at top of run()
   ```

2. **Class filter set to wrong class** — a previous `vision.home(target=flare_ref)`
   set `classes='flare'`. The detector no longer publishes gate detections.
   ```python
   duburi.set_classes('gate,flare')   # fix: restore both classes
   ```

3. **Detector not running** — no `/duburi/vision/<cam>/detections` topic.
   ```bash
   ros2 topic list | grep detections   # should show your camera
   ```

4. **Class name mismatch** — the model uses `'Gate'` (capitalised) but your
   code checks `'gate'`. class_id is case-sensitive.
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
duburi.vision.home(target=duburi.models.gate.flare, ...)   # ← sets classes='flare'
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
3. Use `vision.find()` instead of `detected()` for more robust target
   acquisition with built-in lost-track recovery.

---

## 10. Design patterns — canonical mission templates

### Pattern A: Search-then-align (the core paradigm)

```python
def run(duburi, log):
    duburi.camera = 'forward'
    duburi.models(gate='gate_flare_medium_100ep')
    duburi.arm()
    duburi.set_depth(-0.8)
    duburi.lock_heading(target=0.0, timeout=180)

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

    # Gate visible — align and pass
    duburi.vision.home(
        target=duburi.models.gate.gate,
        yaw=True, lat=True,
        gate_guard=True, pass_at=0.38, pass_at_gain=55,
        dist=0.40, metric='area',
        duration=20,
    )
    duburi.move_forward_dist(3.0, gain=60)

    duburi.unlock_heading()
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
# After aligning to flare:
duburi.vision.home(target=duburi.models.gate.flare, yaw=True, forward=True, depth=True, ...)

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
    duburi.vision.home(target=duburi.models.gate.gate, yaw=True, lat=True, ...)
    duburi.move_forward_dist(1.5, gain=60)
```

### Pattern D: Multi-target branch

```python
duburi.set_classes('gate,flare')
duburi.pause(1.0)   # warm the cache

if duburi.detected('gate', stale_after=0.5):
    log('gate visible — aligning')
    duburi.vision.home(target='gate', yaw=True, lat=True, gate_guard=True)
elif duburi.detected('flare', stale_after=0.5):
    log('flare visible but no gate — orbit first')
    duburi.vision.home(target='flare', yaw=True, forward=True, depth=True)
else:
    log('nothing visible — advancing')
    duburi.move_forward(2.0, gain=35)
```

### Pattern E: Conditional DVL pass

```python
result = duburi.vision.home(
    target=duburi.models.gate.gate,
    yaw=True, lat=True,
    gate_guard=True, pass_at=0.38,
    duration=20,
)

if result.success:
    # Vision aligned — use DVL for precise gate passage
    duburi.move_forward_dist(3.0, gain=60)
else:
    # Vision failed (gate moved or lost) — open-loop fallback
    log.warn('gate alignment failed — open-loop passage attempt')
    duburi.move_forward(4.0, gain=40)   # conservative open-loop
```

---

## 11. The paradigm in context of the mission architecture

```
YASMIN FSM (future)            What we have now
─────────────────────          ─────────────────────────────────────────
State: SEARCH_GATE         →   while not duburi.detected('gate'):
  SEARCH_GATE → FOUND              duburi.move_forward(0.5, gain=30)
                           
State: ALIGN_GATE          →   duburi.vision.home(target='gate', yaw=True, lat=True, ...)
  ALIGN_GATE → PASS        
                           
State: PASS_GATE           →   duburi.move_forward_dist(3.0, gain=60)
  PASS_GATE → SEARCH_FLARE
```

Each `detected()`-based loop IS a proto-state. When YASMIN replaces linear
scripts, each loop becomes an explicit state node with named transitions. The
detected paradigm is the design that makes that refactor clean: every logical
state is already isolated in the mission script.

The detected paradigm is specifically documented as the design step toward
YASMIN FSM in `duburi_planner/state_machines/` (currently empty). When
missions outgrow linear scripts (more than ~3 tasks with retry logic), YASMIN
is the right next step — and each `while detected()` loop maps 1:1 to a state.

---

## 12. Cross-references

- Implementation: `src/duburi_planner/duburi_planner/duburi_dsl.py` (`detected()` method, `_on_detections()` callback)
- Detection topic source: `src/duburi_vision/duburi_vision/detector_node.py` (publishes `Detection2DArray`)
- Vision state (manager side): `src/duburi_manager/duburi_manager/vision_state.py` (`bbox_error()`, used by vision verbs, NOT by `detected()`)
- Mission samples: `src/duburi_planner/duburi_planner/missions/gate_flare_autonomous.py` (canonical use)
- Mission cookbook: `.claude/context/mission-cookbook.md` §7.6
- Client/DSL API: `.claude/context/client-and-dsl-api.md` §2.5
- YASMIN FSM target: `src/duburi_planner/duburi_planner/state_machines/` (currently empty)
