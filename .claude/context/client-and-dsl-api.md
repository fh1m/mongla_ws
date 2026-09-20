# Mission DSL and client API

Three layers, one action:

1. **`MonglaMission`** — the mission language. What you call inside `def run(mongla)`.
   [`mongla_dsl.py`](../../src/mongla_planner/mongla_planner/mongla_dsl.py)
2. **`MonglaClient`** — the blocking action client underneath it: deadlines, cancellation,
   typed failures. [`client.py`](../../src/mongla_planner/mongla_planner/client.py)
3. **`Mongla`** — the manager-side facade that *implements* the verbs. Mission code never
   instantiates it. [`mongla.py`](../../src/mongla_control/mongla_control/mongla.py)

Every verb in [`command-reference.md`](command-reference.md) is reachable from all three.

---

## A mission

```python
from mongla_planner.client import TaskAbandoned

def run(mongla, log=None):
    mongla.mission_reset()              # clear old state, re-zero depth at the surface
    mongla.use_budget(900, reserve_s=45)

    try:
        mongla.arm()
        mongla.set_depth(-0.8)

        with mongla.task('gate', deadline_s=120):
            if mongla.vision.align('gate', yaw=0, lat=0, gain=30, duration=60):
                mongla.vision.move('gate', fwd=None)     # drive through
    except TaskAbandoned:
        mongla.note('gate', 'ran out of time', success=False)
    finally:
        mongla.surface()
        mongla.disarm()
```

Run it with `ros2 run mongla_planner mission <name>`. The `finally` block is not optional
style: a mission that raises must still surface and disarm.

---

## Moving

```python
mongla.arm(timeout=15.0)
mongla.disarm(timeout=20.0)
mongla.set_mode('STABILIZE')

mongla.set_depth(-0.8, timeout=30)          # negative metres = below the surface
mongla.move_forward(3.0, gain=40)           # seconds, and a thrust CAP
mongla.move_left(2.0, gain=35)
mongla.turn(90.0)                           # absolute heading; direction chosen for you
mongla.yaw_right(45.0)                      # relative

mongla.stop()                               # active neutral hold
mongla.pause(2.0)                           # release control briefly
mongla.surface()
mongla.fire(3)                              # board channel 1..16
```

**`gain` is a ceiling, not a target.** **Depth is negative.** Both are worth saying twice.

## Seeing

```python
mongla.vision.align('gate', yaw=0, lat=0, gain=30, duration=30, fallback=creep)
mongla.vision.move('gate', fwd=80, mode='area', gain=35)

mongla.detected('gate')                 # seen within the last second?
mongla.wait_for('gate', timeout=8)      # block until seen
mongla.where('gate')                    # 'left' | 'center' | 'right' | 'unknown'
mongla.can_see('gate')                  # ...and if not, why not
mongla.side_on('fire', 'bin')           # which side of the structure the symbol is on
mongla.outline('gate')                  # the largest contour, when segmentation is on
```

`detected()` asks about a **window**, not a single frame — a class that flickers out of
individual frames still counts as present for a second. An `if detected()` runs once; a
moving search needs a loop.

### Reading a vision result

Both verbs return a result that is truthy only when the target was actually reached, and that
carries what the camera last saw:

```python
res = mongla.vision.align('hole', lat=0, depth=0, duration=25)
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
mongla.range_to('gate')            # metres, with a sigma
mongla.bearing_to('gate')          # degrees off the nose
mongla.floor_range()               # to the point under the camera
mongla.floor_height()              # height above the floor
mongla.pose()                      # the filter's current estimate
mongla.motion()                    # is the hull actually moving?
mongla.fix_from_prop('gate')       # a position fix from something we recognise
mongla.standoff_for_prop('board')  # how far back to sit for this prop
mongla.hfov_water_deg()            # this camera's real field of view, in water
```

Every one of these goes through the measured calibration and the flat-port refraction
correction. They return `None` rather than a guess when the inputs are not there.

## Navigating to something known

```python
mongla.use_course('sauvc26')       # load the venue's priors
mongla.acquire('gate')             # search until it is in view
mongla.goto_prop('drum')           # approach something at a known place
mongla.anchor_on('gate')           # take an absolute heading from a known bearing
mongla.absolute_heading()          # ...and read it back
```

Course files live in `mongla_localization/courses/`, and your own copy in `~/.mongla/courses`
wins over the packaged one — so a survey on competition day needs no rebuild.

## Managing the run

```python
mongla.use_budget(900, reserve_s=45)         # the clock starts on a successful arm

v = mongla.worth_attempting('torpedo', points=300, worst_case_s=120,
                            fallback_s=25, fallback_points=100)
if v.mode == 'full':      ...
elif v.mode == 'fallback': ...                # e.g. fire blind, keep the points
                                              # v.mode == 'skip' → do not start

with mongla.task('torpedo', deadline_s=90):   # cancels the goal in flight at the deadline
    ...

mongla.note('navigation', 'unconfirmed: blind transit', success=False)
mongla.countdown(10)                          # tether-removal banner
mongla.log_scoreboard()                       # what happened, verb by verb
```

Without `use_budget()` nothing is rationed — every verdict is "attempt, full". That is the
opt-out: not calling it.

`task()` is what turns a deadline into a *clean* abandonment: the in-flight goal is cancelled
(the vehicle brakes and neutralises as with any cancel) and `TaskAbandoned` is raised so the
mission takes its fallback. Nested tasks keep the earlier deadline, and the safety verbs are
never blocked by one.

## Configuring vision live

```python
mongla.use_camera('downward')       # switch the sticky camera and the live detector
mongla.set_model('sauvc_sim')
mongla.set_classes('final_gate')
mongla.set_conf(0.15)
mongla.set_vision_param('max_depth_m', -1.6)   # a manager tunable, for this mission
mongla.pause_detector('forward')
mongla.resume_detector('downward')
mongla.save_evidence('forward', 'after_align')  # write the annotated frame beside the run
```

Setting a model or class aborts loudly if the detector node is not on the graph. That is
deliberate: the alternative is a mission that idles forever against a target nothing is
looking for.

## Knowing the vehicle

```python
mongla.backend        # 'srot' or 'pixhawk'
```

Read once from the manager, and it **raises rather than guessing** if the manager cannot be
reached. Missions use it to skip a verb one backend refuses:

```python
if mongla.backend != 'srot':
    mongla.lock_heading(0.0)       # on srot the board holds heading itself
```

## The escape hatch

Any name the DSL does not define is forwarded to the raw client, so a new verb works from a
mission before anyone writes a wrapper:

```python
mongla.send('mission_reset')
mongla.calibrate_depth()           # reaches the action server through the same path
```

---

## Underneath: `MonglaClient`

```python
from mongla_planner.client import MonglaClient, MoveRejected, MoveFailed, TaskAbandoned

client = MonglaClient(node)
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
[`packages/mongla_planner`](packages/mongla_planner/README.md)
