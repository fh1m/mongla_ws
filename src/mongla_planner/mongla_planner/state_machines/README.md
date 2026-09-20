# State-machine planning (YASMIN — BUILT)

The YASMIN FSM layer is **built and runnable**. It wraps the same DSL/`MonglaClient`
surface the script missions use, so the MAVLink + control path stays uniform — each
state is a thin adapter over a `mongla.*` verb with explicit outcomes, timeouts, and
retries. Run a plan via the mission runner (the `fsm_*` launchers in `../missions/`):

```bash
ros2 run mongla_planner mission fsm_full_2026     # full 5-task YASMIN FSM
ros2 run mongla_planner mission fsm_torpedo        # standalone torpedo FSM
ros2 run mongla_planner mission gate_flare_fsm     # gate + flare (auto-detects vehicle)
```

Full user guide (fundamentals, `VehicleProfile`, the state library, pool-day workflow,
adding a task): [`.claude/context/fsm-guide.md`](../../../../.claude/context/fsm-guide.md).
Vision-guided FSM design: [`.claude/context/fsm-vision-missions.md`](../../../../.claude/context/fsm-vision-missions.md).

## Layout (as built)

```
state_machines/
  core/
    outcomes.py            # shared outcome constants (SUCCEED/FAIL/ABORT/...)
    blackboard.py          # typed shared state passed between states
    vehicle_profile.py     # VehicleProfile.auto() — DVL => Mongla 4.5, else Mongla_agile (timed)
    base_state.py          # BaseState: MonglaClient adapter + abort/timeout plumbing
  states/
    navigation.py          # Arm / Disarm / SetDepth / LockHeading / Move{Forward,Back,Lateral} / Turn / Surface
    vision.py              # VisionSearch / VisionAlign / VisionMove
    utility.py             # Countdown / Pause / LogScore / SetDetector / Fire / StyleRoll
  plans/
    gate_flare.py  prequal.py  gate_then_bin.py  slalom.py
    bin_drop.py  torpedo_fire.py  return_gate.py  full_competition.py
```

Each `plans/<name>.py` exposes a `build_*_fsm(mongla, profile)` that assembles the
states into a `StateMachine`; the matching `missions/fsm_*.py` launcher builds the
profile (`VehicleProfile.auto()`) and runs it.

## When to reach for the FSM vs a script mission

- **Script `missions/*.py` (detected()-paradigm):** prototyping, per-subsystem unit
  tests, linear sequences. The simplest thing that works; edit + run, no rebuild.
- **YASMIN FSM:** competition runs needing declarative branching on perception
  ("if torpedo acquired: fire, else: search"), explicit per-state timeout/retry/abort,
  and dual-vehicle reuse (one plan, `VehicleProfile` picks DVL-distance vs timed moves).

## References

  * YASMIN docs:  https://uleroboticsgroup.github.io/yasmin/4.2.3/
  * smach legacy: https://wiki.ros.org/smach (concepts carry over)
