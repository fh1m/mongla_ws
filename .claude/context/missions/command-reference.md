# Command reference — every verb on `/mongla/move`

One action, twenty-three verbs. This page says what each one does, what it takes, and — the part
that matters on this vehicle — **where it actually runs**.

The canonical list is the `COMMANDS` registry in
[`mongla_control/commands.py`](../../../src/mongla_control/mongla_control/commands.py). If a verb
is in this document and not in that registry, the document is wrong.

---

## Three shapes, one goal

| Shape | How you call it |
|---|---|
| **CLI** | `ros2 run mongla_planner mongla <verb> --field value` |
| **Mission DSL** | `mongla.<verb>(...)` inside `def run(mongla)` |
| **Action** | a `Move` goal with `cmd='<verb>'`, from any ROS client |

All three end up as the same goal on `/mongla/move`, served by `auv_manager_node`. The CLI is
generated from the registry, so a new verb appears in it with no extra code.

## Where a verb runs

```
   your verb ───────┬─ on the board  11 verbs — one primitive, run and braked at 500 Hz
                    └─ host          12 verbs — a loop here, or a single message
```

| Group | Verbs |
|---|---|
| **On the board** | `move_forward` `move_back` `move_left` `move_right` `yaw_left` `yaw_right` `turn` `set_depth` `stop` `pause` `style_roll` |
| **Host loop or single message** | `arm` `disarm` `set_mode` `head` `surface` `mission_reset` `calibrate_depth` `calc_distance` `fire` `motor_test` `vision_align` `vision_move` |

There is no third group. A verb the board cannot run does not exist: the manager answers
`Unknown command` and nothing reaches the wire. A missing low-level capability is a firmware
pull request ([`upstream/`](../upstream/README.md)), not a host emulation.

> ⛔ **A board-side move can still be denied.** Every on-board primitive runs in the board's
> automatic mode, which closes the depth loop — and that loop has never run closed in water.
> Until the two bench checks pass, the board may refuse these moves outright. That includes
> the search creep a vision verb falls back on. See
> [`srot-integration.md`](../platform/srot-integration.md).

---

## Quick reference

Required fields have no default.

| Verb | Fields → default | What it does |
|---|---|---|
| `arm` | `timeout`→15 s | thrusters live |
| `disarm` | `timeout`→20 s | safe shutdown |
| `set_mode` | `target_name`, `timeout`→8 s | STABILIZE · DEPTH_HOLD · SURFACE · MANUAL · ACRO |
| `head` | — | read the current heading into the result |
| `stop` | — | halt now: the board's `MOVE_STOP` brake |
| `pause` | `duration`→2 s | hold station for N seconds: the board's `MOVE_HOLD` |
| `mission_reset` | — | clear the abort flag, zero the demand; re-zero depth if disarmed at the surface |
| `calibrate_depth` | — | re-zero the barometer at the surface (disarmed only) |
| `calc_distance` | `phase`→`start` | bracket a move and measure how far it went, from the downward camera |
| `surface` | `timeout`→60 s | brake, then the board's SURFACE mode; works during a running mission |
| `move_forward` / `move_back` | `duration`, `gain`→80, `settle`→0 | timed drive |
| `move_left` / `move_right` | `duration`, `gain`→80, `settle`→0 | timed strafe |
| `yaw_left` / `yaw_right` | `target`, `timeout`→30 s, `settle` | relative turn, in degrees |
| `turn` | `target`, `timeout`→30 s, `settle` | absolute heading; direction chosen for you |
| `set_depth` | `target`, `timeout`→30 s, `settle` | hold a depth (negative metres) |
| `style_roll` | `flips`→1, `timeout`→20 s | N full rolls, run by the board |
| `fire` | `fire_channel` (board channel 1–16) | actuate a payload channel |
| `motor_test` | `target`→1 (thruster 1–8), `gain`→20, `duration`→2 s | spin one thruster; requires ARMED |
| `vision_align` | see below | hold a target at a chosen pixel offset |
| `vision_move` | see below | drive toward or through a target |

### Two conventions that catch everyone

**`gain` is a speed *cap*, not a speed.** `gain=40` means "never exceed 40 % thrust". The loop
may use less.

**Depth is negative below the surface.** `set_depth --target -0.8` is 80 cm down. This holds on
the wire too: the board reports altitude, so a depth of 0.8 m arrives as −0.8.

---

## Power, mode and safety

```bash
ros2 run mongla_planner mongla arm
ros2 run mongla_planner mongla set_mode --target_name STABILIZE
ros2 run mongla_planner mongla surface
ros2 run mongla_planner mongla disarm
```

`arm` refuses on a board whose firmware is older than the host's floor — behaviour revision 10
inverted the yaw direction, so an older board would take every turn command backwards. It also
refuses while the board's depth controller is saturated, which is what a broken depth sensor
looks like from outside.

`stop`, `surface` and `disarm` bypass the "a command is already running" gate, so they work
during a mission. `stop` brakes and `pause` holds station; both are board primitives, so both
enter AUTO.

⛔ **`surface()` is the last motion.** Every board verb — `stop` included — is a `SROT_MOVE`,
which enters AUTO and takes the hull back out of SURFACE. A cleanup that ends
`surface(); stop()` holds the vehicle at depth.

`mission_reset` belongs at the top of every mission: it clears the abort flag carried over
from a previous run and re-zeroes the barometer while the vehicle is still on the surface. Which is
also why a mission must never call another mission's `run()` mid-dive — see
[`sauvc_full`](../../../src/mongla_planner/mongla_planner/missions/sauvc_full.py).

## Moving

```bash
ros2 run mongla_planner mongla move_forward --duration 5 --gain 40
ros2 run mongla_planner mongla turn --target 90        # absolute heading
ros2 run mongla_planner mongla set_depth --target -0.8
```

On the srot board each of these becomes **one** command. The board runs the motion, brakes it
at the end, and reports back — the host is not in the loop, so a busy Pi cannot stretch a
3-second move into a 4-second one.

`--target head` is a magic value on any verb taking a heading: it resolves to the live heading
at the moment the command runs.

## Vision — the two closed-loop verbs

Both are proportional control on **pixel error**, read straight from the detector. No metric
model of the world is required.

```bash
# centre the gate on yaw and sideways
ros2 run mongla_planner mongla vision_align --camera forward --target_class gate \
    --axes yaw,lat --err_px 40 --gain 30 --duration 20

# drive until it fills 80 % of the frame
ros2 run mongla_planner mongla vision_move --camera forward --target_class gate \
    --fwd_fill 80 --mode area --gain 35 --duration 20
```

### The never-fail contract

Neither verb raises, and neither aborts a mission. The server always returns `success=True`;
the real outcome is a code:

| Code | Meaning |
|---|---|
| `ALIGNED` (0) | every requested axis is centred, or the fill target was reached |
| `LOST` (1) | the target went missing for longer than the grace period |
| `TIMEOUT` (2) | the budget elapsed |
| `NO_CAMERA` (3) | the camera pipeline never came up |
| `ABORTED` (4) | the goal was cancelled |
| `DRIFTED` (5) | `align` centred while moving, then re-measured out of band once stopped |

"I could not see it" is a normal outcome for a mission to branch on, not an exception to crash
into. The DSL wraps the code in a result that also carries **where the target was** when the
verb ended — signed pixels from centre, `NaN` if it was never seen — so recovery is a decision
rather than a guess.

```python
res = mongla.vision.align('gate', yaw=0, lat=0, gain=30, duration=20)
if not res and res.saw_target:
    mongla.move_right(1.0) if res.x_px > 0 else mongla.move_left(1.0)
```

Always check `saw_target` before reading a pixel position: `NaN < threshold` is quietly False.

### Axes, and what they mean on each camera

| Axis | Forward camera | Downward camera |
|---|---|---|
| `lat` | strafe sideways | strafe sideways |
| `yaw` | turn toward it | turn toward it (rarely useful top-down) |
| `fwd` | forward standoff, as a fill fraction — one-sided, never reverses | **drive fore/aft** on the image's vertical axis — two-sided, braked, signed by `surge_sign` |

On the wire the downward fore/aft axis is still called `depth` (it is image-Y); the DSL calls
it `fwd` so a mission names the *physical* axis it means.

⛔ **Depth is never a vision axis.** The board owns it: `set_depth` first, then align. The
forward camera's image-Y axis and a downward fill-driven descent both raise `ValueError`
before anything is sent.

### Search: `fallback`

When the target is lost, the DSL runs a mission-supplied search function and then re-enters the
loop — all inside the original budget.

```python
def creep(mongla):
    mongla.move_forward(1.0, gain=30)

mongla.vision.align('gate', yaw=0, lat=0, fallback=creep, duration=60)
```

With **no** fallback the verb rides out the blackout instead: it drives neutral and keeps
waiting, which is usually right in turbid water.

### Firing at the end of a hold

`vision_align` can hold station and fire the payload mid-hold, so the shot happens while the
loop is still correcting rather than after it has drifted:

```python
mongla.vision.align('hole', camera='forward', lat=0,
                    hold=6.0, fire=TORPEDO_1_CHANNEL, fire_t=3.0, brake=False)
```

The shot is gated on a **fresh** detection — a new live box, not a predicted or frozen one —
and on the vehicle being aligned. The board's answer (fired, refused, denied, no-ack) comes
back in the result.

## Payload

`fire` takes a **board channel number**, 1–16, exactly as the board's own configuration names
it (`SERVOn_ROLE`). There is no host-side map to get out of step. A channel configured as a
switch fires; a servo or unroled channel is refused, and so is `0` — what every payload
constant in `competition_config.py` is until the payload is wired.

```bash
ros2 run mongla_manager connect      # lists which channels are fireable
ros2 run mongla_planner mongla fire --fire_channel 9
```

## Measuring a move without a DVL

```python
mongla.calc_distance(phase='start')
mongla.move_forward(3.0, gain=40)
travelled = mongla.calc_distance(phase='stop').final_value
```

The downward camera accumulates distance over the floor while the move runs. Bench-measured
worst error **1.09 cm over 30 cm**. It works with the detectors paused, because it needs the
floor's texture rather than any detection.

---

## Where to look next

- [`client-and-dsl-api.md`](client-and-dsl-api.md) — the mission language around these verbs
- [`vision-results.md`](vision-results.md) — reading a vision result, and recovery patterns
- [`precision-alignment.md`](precision-alignment.md) — holding a 20 kg hull still enough to fire
- [`packages/mongla_control`](../packages/mongla_control/README.md) — where each verb is implemented
- [Capability Map](../capability-map.md) — what is verified, and what is not
