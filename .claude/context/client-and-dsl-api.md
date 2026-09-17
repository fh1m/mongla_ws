# Mission DSL and client API

Three layers, one action:

1. **`DuburiMission`** — the mission language. What you call inside `def run(duburi)`.
   [`duburi_dsl.py`](../../src/duburi_planner/duburi_planner/duburi_dsl.py)
2. **`DuburiClient`** — the blocking action client underneath it: deadlines, cancellation,
   typed failures. [`client.py`](../../src/duburi_planner/duburi_planner/client.py)
3. **`Duburi`** — the manager-side facade that *implements* the verbs. Mission code never
   instantiates it. [`duburi.py`](../../src/duburi_control/duburi_control/duburi.py)

Every verb in [`command-reference.md`](command-reference.md) is reachable from all three.

---

## A mission

```python
from duburi_planner.client import TaskAbandoned

def run(duburi, log=None):
    duburi.mission_reset()              # clear old state, re-zero depth at the surface
    duburi.use_budget(900, reserve_s=45)

    try:
        duburi.arm()
        duburi.set_depth(-0.8)

        with duburi.task('gate', deadline_s=120):
            if duburi.vision.align('gate', yaw=0, lat=0, gain=30, duration=60):
                duburi.vision.move('gate', fwd=None)     # drive through
    except TaskAbandoned:
        duburi.note('gate', 'ran out of time', success=False)
    finally:
        duburi.surface()
        duburi.disarm()
```

Run it with `ros2 run duburi_planner mission <name>`. The `finally` block is not optional
style: a mission that raises must still surface and disarm.

---

## Moving

```python
duburi.arm(timeout=15.0)
duburi.disarm(timeout=20.0)
duburi.set_mode('STABILIZE')

duburi.set_depth(-0.8, timeout=30)          # negative metres = below the surface
duburi.move_forward(3.0, gain=40)           # seconds, and a thrust CAP
duburi.move_left(2.0, gain=35)
duburi.turn(90.0)                           # absolute heading; direction chosen for you
duburi.yaw_right(45.0)                      # relative

duburi.stop()                               # active neutral hold
duburi.pause(2.0)                           # release control briefly
duburi.surface()
duburi.fire(3)                              # board channel 1..16
```

**`gain` is a ceiling, not a target.** **Depth is negative.** Both are worth saying twice.

## Seeing

```python
duburi.vision.align('gate', yaw=0, lat=0, gain=30, duration=30, fallback=creep)
duburi.vision.move('gate', fwd=80, mode='area', gain=35)

duburi.detected('gate')                 # seen within the last second?
duburi.wait_for('gate', timeout=8)      # block until seen
duburi.where('gate')                    # 'left' | 'center' | 'right' | 'unknown'
duburi.can_see('gate')                  # ...and if not, why not
duburi.side_on('fire', 'bin')           # which side of the structure the symbol is on
duburi.outline('gate')                  # the largest contour, when segmentation is on
```

`detected()` asks about a **window**, not a single frame — a class that flickers out of
individual frames still counts as present for a second. An `if detected()` runs once; a
moving search needs a loop.

### Reading a vision result

Both verbs return a result that is truthy only when the target was actually reached, and that
carries what the camera last saw:

```python
res = duburi.vision.align('hole', lat=0, depth=0, duration=25)
res.status        # 'ALIGNED' | 'LOST' | 'TIMEOUT' | 'NO_CAMERA' | 'ABORTED' | 'FAILED'
res.saw_target    # was it ever seen?
res.x_px, res.y_px  # signed pixels from centre at the last sighting; NaN if never seen
res.last_err_px   # residual from the goal
res.fill          # how much of the frame it filled (move)
res.fired         # the board's answer, when a shot was requested
res.elapsed_s
```

Check `saw_target` before reading `x_px` — `NaN < threshold` is silently False, so a
never-seen target slips straight through a naive guard.

## Measuring

```python
duburi.range_to('gate')            # metres, with a sigma
duburi.bearing_to('gate')          # degrees off the nose
duburi.floor_range()               # to the point under the camera
duburi.floor_height()              # height above the floor
duburi.pose()                      # the filter's current estimate
duburi.motion()                    # is the hull actually moving?
duburi.fix_from_prop('gate')       # a position fix from something we recognise
duburi.standoff_for_prop('board')  # how far back to sit for this prop
duburi.hfov_water_deg()            # this camera's real field of view, in water
```

Every one of these goes through the measured calibration and the flat-port refraction
correction. They return `None` rather than a guess when the inputs are not there.

## Navigating to something known

```python
duburi.use_course('sauvc26')       # load the venue's priors
duburi.acquire('gate')             # search until it is in view
duburi.goto_prop('drum')           # approach something at a known place
duburi.anchor_on('gate')           # take an absolute heading from a known bearing
duburi.absolute_heading()          # ...and read it back
```

Course files live in `duburi_localization/courses/`, and your own copy in `~/.duburi/courses`
wins over the packaged one — so a survey on competition day needs no rebuild.

## Managing the run

```python
duburi.use_budget(900, reserve_s=45)         # the clock starts on a successful arm

v = duburi.worth_attempting('torpedo', points=300, worst_case_s=120,
                            fallback_s=25, fallback_points=100)
if v.mode == 'full':      ...
elif v.mode == 'fallback': ...                # e.g. fire blind, keep the points
                                              # v.mode == 'skip' → do not start

with duburi.task('torpedo', deadline_s=90):   # cancels the goal in flight at the deadline
    ...

duburi.note('navigation', 'unconfirmed: blind transit', success=False)
duburi.countdown(10)                          # tether-removal banner
duburi.log_scoreboard()                       # what happened, verb by verb
```

Without `use_budget()` nothing is rationed — every verdict is "attempt, full". That is the
opt-out: not calling it.

`task()` is what turns a deadline into a *clean* abandonment: the in-flight goal is cancelled
(the vehicle brakes and neutralises as with any cancel) and `TaskAbandoned` is raised so the
mission takes its fallback. Nested tasks keep the earlier deadline, and the safety verbs are
never blocked by one.

## Configuring vision live

```python
duburi.use_camera('downward')       # switch the sticky camera and the live detector
duburi.set_model('sauvc_sim')
duburi.set_classes('final_gate')
duburi.set_conf(0.15)
duburi.set_vision_param('max_depth_m', -1.6)   # a manager tunable, for this mission
duburi.pause_detector('forward')
duburi.resume_detector('downward')
duburi.save_evidence('forward', 'after_align')  # write the annotated frame beside the run
```

Setting a model or class aborts loudly if the detector node is not on the graph. That is
deliberate: the alternative is a mission that idles forever against a target nothing is
looking for.

## Knowing the vehicle

```python
duburi.backend        # 'srot' or 'pixhawk'
```

Read once from the manager, and it **raises rather than guessing** if the manager cannot be
reached. Missions use it to skip a verb one backend refuses:

```python
if duburi.backend != 'srot':
    duburi.lock_heading(0.0)       # on srot the board holds heading itself
```

## The escape hatch

Any name the DSL does not define is forwarded to the raw client, so a new verb works from a
mission before anyone writes a wrapper:

```python
duburi.send('mission_reset')
duburi.calibrate_depth()           # reaches the action server through the same path
```

---

## Underneath: `DuburiClient`

```python
from duburi_planner.client import DuburiClient, MoveRejected, MoveFailed, TaskAbandoned

client = DuburiClient(node)
client.wait_for_connection(timeout=15.0)
result = client.send('move_forward', duration=5.0, gain=60.0)
client.cancel_active()                 # from another thread, or a deadline
client.last_goal_id                    # the same 8 hex characters the manager logs
```

| Exception | When |
|---|---|
| `MoveRejected` | the server refused the goal |
| `MoveFailed` | it ran and reported failure — including a verb this backend refuses |
| `MoveTimeout` | no result inside the budget |
| `TaskAbandoned` | a task deadline passed; the goal was cancelled |

The client blocks on purpose. A mission is a script, and a script that reads top to bottom is
one a tired person can debug at the poolside at midnight.

---

Related: [`command-reference.md`](command-reference.md) ·
[`vision-results.md`](vision-results.md) · [`mission-cookbook.md`](mission-cookbook.md) ·
[`packages/duburi_planner`](packages/duburi_planner/README.md)
