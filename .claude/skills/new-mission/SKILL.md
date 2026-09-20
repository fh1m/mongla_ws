---
name: new-mission
description: Scaffold a mongla_planner mission file from the two-verb vision DSL template (vision.align / vision.move + a mission-authored fallback). Use when starting a new RoboSub task mission or demo.
disable-model-invocation: true
---

# New mission

**Usage:** `/new-mission <name>`

Create `src/mongla_planner/mongla_planner/missions/<name>.py`. The `mission` runner
auto-discovers `missions/<name>.run` — no registration needed.

## Template

```python
"""<name> mission — <one-line goal>."""

POOL_DEPTH_M = -0.8


def run(mongla, log):
    mongla.mission_reset()                          # clear lock/abort carry-over
    mongla.models(gate='gate_flare_medium_100ep')   # register models used
    mongla.camera = 'forward'

    try:
        mongla.arm()
        mongla.set_depth(POOL_DEPTH_M, timeout=30)

        # Two verbs only. align() centres the target — each of lat/yaw/depth
        # is a signed pixel offset from centre (0 = centre). move() drives
        # forward until the bbox fills fwd% of the frame (mode area/width/
        # height). A miss never aborts: the mission-authored `fallback` runs
        # on target loss, then the verb re-enters within `duration`.
        mongla.vision.align(mongla.models.gate.gate, yaw=0, lat=0,
                            err=40, gain=30, duration=20, fallback=search)
        mongla.vision.move(mongla.models.gate.gate, fwd=80, mode='height',
                           gain=35, duration=20, fallback=search)
        log('<name>: primary objective done')

    finally:
        mongla.stop()
        mongla.disarm()          # always disarm, even on exception


# Mission-authored fallback search — pure control, runs on target loss.
# fn(mongla) does one short manoeuvre; fn(mongla, should_stop) may sweep
# and must bail the moment the target reappears.
def search(mongla, should_stop):
    for _ in range(6):
        if should_stop():
            return
        mongla.yaw_right(15)
        mongla.pause(0.4)
```

## Rules baked into the template

- **Two verbs only:** `mongla.vision.align(...)` (centre on signed pixel offsets)
  and `mongla.vision.move(...)` (drive forward to a bbox fill %). No other vision verbs.
- Every vision step passes a mission-authored `fallback=` search (pure control; runs
  on target loss, then the verb re-enters within its `duration`).
- Vision verbs never abort the mission — gate any follow-up (e.g. `mongla.fire(...)`)
  on the truthy `VisionResult`: `if mongla.vision.align(...): ...`.
- `mongla.mission_reset()` at the top of `run()` (clears heading-lock / abort carry-over).
- `disarm()` in `finally` so an exception never leaves thrusters live.
- For autonomous (tether-free) runs, add a timer-delayed start at the top of `run`.

## After

```bash
./build_mongla.sh
ros2 run mongla_planner mission --list      # confirm <name> appears
ros2 run mongla_planner mission <name>      # sim first
```

Ask the `mission-reviewer` agent to check the script before a pool run.
