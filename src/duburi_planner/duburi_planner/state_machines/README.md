# State-machine planning (reserved)

Empty stub. The eventual home for YASMIN-based mission state machines.

## Why empty today

The Python script-based missions in `../missions/` are the simplest
thing that works for the AUV's current mission set (linear sequences
with optional retries on a per-step basis). State machines pay off
when:

  * A mission has branching logic that depends on perception
    (e.g. "if torpedo is acquired: shoot, else: search again").
  * Multiple operators want to share, version, and visualize plans.
  * A run is long enough that automatic recovery (retry, abort,
    safe-mode) has to be declarative rather than threaded through
    every script.

When that day arrives, this folder will hold YASMIN states that wrap
`DuburiClient` calls -- the same client the script missions use, so
the MAVLink + control surface stays uniform.

## Planned layout

```
state_machines/
  __init__.py                # exports build_<plan>_fsm()
  states/
    __init__.py
    move_forward_state.py    # YasminState wrapping client.move_forward
    yaw_state.py             # similar for yaw_left / yaw_right / arc
    set_depth_state.py
    lock_heading_state.py    # async: starts lock, returns immediately
  plans/
    __init__.py
    qualifier_run.py         # build_qualifier_fsm() -> StateMachine
```

## References

  * YASMIN docs:  https://uleroboticsgroup.github.io/yasmin/4.2.3/
  * smach legacy: https://wiki.ros.org/smach (concepts carry over)
