---
name: new-mission
description: Scaffold a duburi_planner mission file from the two-verb vision DSL template (vision.align / vision.move + a mission-authored fallback). Use when starting a new RoboSub task mission or demo.
disable-model-invocation: true
---

# New mission

**Usage:** `/new-mission <name>`

Create `src/duburi_planner/duburi_planner/missions/<name>.py`. The `mission` runner
auto-discovers `missions/<name>.run` — no registration needed.

## Template

```python
"""<name> mission — <one-line goal>."""

POOL_DEPTH_M = -0.8


def run(duburi, log):
    duburi.mission_reset()                          # clear lock/abort carry-over
    duburi.models(gate='gate_flare_medium_100ep')   # register models used
    duburi.camera = 'forward'

    try:
        duburi.arm()
        duburi.set_depth(POOL_DEPTH_M, timeout=30)

        # Two verbs only. align() centres the target — each of lat/yaw/depth
        # is a signed pixel offset from centre (0 = centre). move() drives
        # forward until the bbox fills fwd% of the frame (mode area/width/
        # height). A miss never aborts: the mission-authored `fallback` runs
        # on target loss, then the verb re-enters within `duration`.
        duburi.vision.align(duburi.models.gate.gate, yaw=0, lat=0,
                            err=40, gain=30, duration=20, fallback=search)
        duburi.vision.move(duburi.models.gate.gate, fwd=80, mode='height',
                           gain=35, duration=20, fallback=search)
        log('<name>: primary objective done')

    finally:
        duburi.stop()
        duburi.disarm()          # always disarm, even on exception


# Mission-authored fallback search — pure control, runs on target loss.
# fn(duburi) does one short manoeuvre; fn(duburi, should_stop) may sweep
# and must bail the moment the target reappears.
def search(duburi, should_stop):
    for _ in range(6):
        if should_stop():
            return
        duburi.yaw_right(15)
        duburi.pause(0.4)
```

## Rules baked into the template

- **Two verbs only:** `duburi.vision.align(...)` (centre on signed pixel offsets)
  and `duburi.vision.move(...)` (drive forward to a bbox fill %). No other vision verbs.
- Every vision step passes a mission-authored `fallback=` search (pure control; runs
  on target loss, then the verb re-enters within its `duration`).
- Vision verbs never abort the mission — gate any follow-up (e.g. `duburi.fire(...)`)
  on the truthy `VisionResult`: `if duburi.vision.align(...): ...`.
- `duburi.mission_reset()` at the top of `run()` (clears heading-lock / abort carry-over).
- `disarm()` in `finally` so an exception never leaves thrusters live.
- For autonomous (tether-free) runs, add a timer-delayed start at the top of `run`.

## After

```bash
./build_dubomini.sh
ros2 run duburi_planner mission --list      # confirm <name> appears
ros2 run duburi_planner mission <name>      # sim first
```

Ask the `mission-reviewer` agent to check the script before a pool run.
