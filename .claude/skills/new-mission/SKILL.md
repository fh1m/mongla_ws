---
name: new-mission
description: Scaffold a duburi_planner mission file from the detected()-paradigm DSL template. Use when starting a new RoboSub task mission or demo.
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
    duburi.models(gate='gate_flare_medium_100ep')   # register models used
    duburi.camera = 'forward'

    try:
        duburi.arm()
        duburi.set_depth(POOL_DEPTH_M, settle=2.0)

        # Detect-then-align (never align blind):
        while not duburi.detected('gate'):
            duburi.move_forward(0.5, gain=30)
        duburi.vision.home(target=duburi.models.gate.gate,
                           yaw=True, lat=True,
                           gate_guard=True, pass_at=0.38, pass_at_gain=55,
                           dist=0.40, metric='area', duration=20)
        duburi.move_forward_dist(distance_m=3.0, gain=60)
        log('<name>: primary objective done')

    finally:
        duburi.set_depth(0.0)
        duburi.disarm()          # always disarm, even on exception
```

## Rules baked into the template

- `detected()` guard before every `vision.*` align.
- `gate_guard=True` + real `pass_at` on gate approaches.
- `disarm()` in `finally` so an exception never leaves thrusters live.
- For autonomous (tether-free) runs, add a timer-delayed start at the top of `run`.

## After

```bash
./build_duburi.sh
ros2 run duburi_planner mission --list      # confirm <name> appears
ros2 run duburi_planner mission <name>      # sim first
```

Ask the `mission-reviewer` agent to check the script before a pool run.
