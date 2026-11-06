# Downward camera — the axis flip, explained once and for all

> The most confusing thing in the vision stack: **the same `vision.align` kwarg means a
> different physical axis on the forward and the downward camera.** This page is the one
> reference. Bin task: [`task_bin.py`](../../../src/mongla_planner/mongla_planner/missions/task_bin.py);
> dual-cam hardware: [`dual-camera-setup.md`](dual-camera-setup.md); the loop itself:
> `motion_vision.align_loop`.

## Why the axes flip at all

The forward camera looks **ahead**, along the hull's surge axis. The downward camera looks
**straight down**, along heave. So the same pixel motion means a different hull motion:

| Image motion | Forward cam → hull axis | Downward cam → hull axis |
|---|---|---|
| target moves **left/right** (image-X) | lateral | lateral — **same** |
| target moves **up/down** (image-Y) | would be depth — **not a vision axis** | **fore/aft surge** |
| target **grows/shrinks** (bbox fill) | nearer/farther in surge — the `fwd` standoff | would be depth — **not a vision axis** |

`lat` is the only axis that means the same thing on both cameras.

## Depth is never a vision axis

The board holds depth and heading at 500 Hz. Over this wire it takes no depth setpoint from
a host loop, so **no vision verb commands depth on either camera**: `set_depth` first, then
align. `align_loop` refuses both shapes that would pretend otherwise, with the reason:

- a `'depth'` axis on the forward camera (image-Y → depth) raises `ValueError`;
- a fill target on the downward camera (a fill-driven descent) raises `ValueError`.

The demand leaves as `MANUAL_CONTROL` in STABILIZE: `x` = surge, `y` = lateral, `r` = yaw,
at 50 Hz (`motion_rates.VISION_LOOP_HZ`).

## The kwarg mapping (what you type)

```
                 lat              fwd
FORWARD cam:   lateral (px)     standoff: drive forward until the bbox fills fwd % (one-sided)
DOWNWARD cam:  lateral (px)     surge: image-Y signed px offset, 0 = centre (two-sided, braked)
```

- **`lat`** — signed pixel offset, image-X → lateral. `0` = centre. Same on both cameras.
- **`fwd`** —
  - forward camera: a **fill %** standoff, measured by `fwd_mode` (area / width / height).
    It drives forward while too far, is neutral at or past it, and never reverses; a mid-hold
    `fire` waits for it.
  - downward camera: a **signed pixel offset**, image-Y → surge. `0` = centre, and `0` is
    *active* surge-to-centre, not unset — which is why the remap lives in the DSL (Python)
    and not the ROS goal, where rosidl collapses `0` into unset. In the goal it travels as the
    `depth` axis with `offset_depth`; that name is the wire's, and on this camera it means surge.
- **`yaw`** — signed pixel offset → yaw rate.

A downward call with no centring axis raises, exactly like a forward `fwd`-only align.

## Canonical calls

Forward — the torpedo standoff shot (`fire` takes a named channel, never a number):

```python
mongla.set_depth(TORPEDO_DEPTH_M)          # None in competition_config until set at the pool
align('hole', camera='forward', lat=0, fwd=25, fwd_mode='height',
      lock_on=True, hold=4, fire=TORPEDO_1_CHANNEL)
```

Downward — the bin drop:

```python
mongla.set_depth(BIN_DEPTH_M)
align('bin', camera='downward', lat=0, fwd=0, fire=DROPPER_1_CHANNEL)
```

The payload constants live in `competition_config.py` and are **0 (unassigned) until the
channels are wired** — `fire` refuses 0 rather than guessing a channel (BUGS.md B53).

## The surge sign — check it disarmed before any armed run

`vision.surge_sign` (deck default **−1**, `vision_tunables.py`) sets the fore/aft polarity for
the physical bottom-camera mount. A wrong sign is **positive feedback**: the hull drives *away*
from the bin. Missions omit it; a mission that passes a non-zero value overrides the deck.
The forward camera never reads it.

```bash
ros2 run mongla_vision vision_thrust_check --camera downward
```

A bin **ahead** in the image must command **forward** surge. If it is reversed:
`ros2 param set /mongla_manager vision.surge_sign 1.0`.
