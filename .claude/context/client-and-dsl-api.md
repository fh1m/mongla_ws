# Python client + mission DSL API

> Three layers, three audiences, one underlying action.
>
> 1. **`DuburiClient`** -- raw blocking action client. ROS2-aware,
>    feedback-aware, raises typed exceptions. Lives in
>    [`src/duburi_planner/duburi_planner/client.py`](../../src/duburi_planner/duburi_planner/client.py).
> 2. **`DuburiMission`** (the "DSL") -- thin friendly wrapper over the
>    client; what you call from inside `def run(duburi, log)`. Lives
>    in [`src/duburi_planner/duburi_planner/duburi_dsl.py`](../../src/duburi_planner/duburi_planner/duburi_dsl.py).
> 3. **`Duburi`** facade -- the manager-side serialised dispatcher
>    that *implements* the verbs (owns the lock, the heartbeat, the
>    pixhawk, etc.). Lives in
>    [`src/duburi_control/duburi_control/duburi.py`](../../src/duburi_control/duburi_control/duburi.py).
>    Mission code does NOT instantiate this directly.
>
> Layer 3 is what receives `/duburi/move` goals and runs the actual
> motion. Layers 1 and 2 are what mission code uses to talk to it.
> Every verb in [`command-reference.md`](./command-reference.md) is
> reachable from all three.

---

## 1. `DuburiClient` -- the raw action surface

```python
from duburi_planner.client import DuburiClient, MoveRejected, MoveFailed
import rclpy

rclpy.init()
node   = rclpy.create_node('mission_runner')
client = DuburiClient(node)
client.wait_for_connection(timeout=15.0)            # blocks until server up

result = client.send('move_forward', duration=5.0, gain=60.0)
print(result.success, result.message,
      result.final_value, result.error_value)
```

### Method surface

| Method                       | Behaviour                                                                              |
| ---------------------------- | -------------------------------------------------------------------------------------- |
| `wait_for_connection(timeout=15.0)` | Blocks until the `/duburi/move` server is reachable. Raises `TimeoutError`. |
| `send(cmd, **fields)`        | Sends one goal, blocks until completion. Returns `Move.Result`. Raises `MoveRejected` (server refused goal) or `MoveFailed` (server accepted but `result.success` was False). |
| `client.<cmd>(**fields)`     | Sugar -- equivalent to `send(cmd, **fields)`. Validated against the `COMMANDS` registry: `client.mvoe_forward(...)` raises `AttributeError`, NOT a silent send of an unknown verb. |

### `Move.Goal` fields the client accepts

The `**fields` kwargs map directly onto `duburi_interfaces.action.Move.Goal`.
Unset fields stay at their rosidl defaults (0.0 / '' / False) -- the
manager-side dispatcher then substitutes per-command defaults from
the `COMMANDS` registry, or live `vision.*` ROS-param values for
vision verbs.

There are exactly **two** vision verbs — `vision_align` and `vision_move`.
Their fields below; everything else is the open-loop motion surface.

| Field                | Type     | Used by                               |
| -------------------- | -------- | ------------------------------------- |
| `duration`           | float32  | move_*, arc, pause, vision_align, vision_move |
| `gain`               | float32  | move_*, arc, vision_align, vision_move — for vision it is a **hard max-speed cap** (% thrust) |
| `gain_lat`/`gain_yaw` | float32 | vision_align per-axis speed cap (vision_move uses `gain_lat` for the maintain strafe); **0 = inherit `gain`, not disable**. Depth has no % cap — its rate is `depth_step` (m/update) |
| `depth_step` | float32 | vision_align per-update depth-setpoint resolution (m); 0.02 slow .. 0.10 coarse; freezes inside deadband so ArduSub settles (no z-wobble). DSL `depth_step=`; 0 = default 0.02 |
| `fire_pass_enabled` | bool | vision_align: fire at command end even if never fully aligned, if seen live+recently (partial-points shot). DSL `fire_pass=` |
| `hold_heading` | bool | vision_align: widen the heading-lock deadband during the hold (yaw-released path) so the launcher heading holds steady (no terminal yaw jitter). DSL `hold_heading=` |
| `brake_off` / `brake_gain` | bool / float32 | vision arrival brake (on by default). `brake_off=true` coasts; `brake_gain` scales the reverse kick (0 = default). DSL exposes `brake=True`; sends `brake_off = not brake`. Brakes lateral (align) + forward-on-fill-stop (move); never yaw/depth/pass-through |
| `target`             | float32  | set_depth (m) / yaw_* (deg) / lock_heading (deg) |
| `target_name`        | string   | set_mode                              |
| `timeout`            | float32  | every command; defaults vary          |
| `settle`             | float32  | post-command neutral hold (move_*/yaw_*); default 0 |
| `yaw_rate_pct`       | float32  | arc                                   |
| `camera`             | string   | vision_align, vision_move — default `'forward'` |
| `target_class`       | string   | vision_align, vision_move             |
| `axes`               | string   | vision_align — CSV subset of `lat,yaw,depth` |
| `offset_lat`         | float32  | vision_align — signed px offset for the lat axis (0=centre) |
| `offset_yaw`         | float32  | vision_align — signed px offset for the yaw axis (0=centre) |
| `offset_depth`       | float32  | vision_align — signed px offset for the depth axis (0=centre) |
| `err_px`             | float32  | vision_align, vision_move — per-axis in-band tolerance (px) |
| `fwd_fill`           | float32  | vision_move — target bbox fill, % of frame |
| `mode`               | string   | vision_move — fill metric: `area` / `width` / `height` |
| `maintain_px`        | float32  | vision_move — lateral px offset held while driving |
| `maintain_on`        | bool     | vision_move — enable `maintain_px` (else pure forward) |
| `hold_s`             | float32  | vision_align (active station-keep), vision_move (after reaching fill) |
| `fire_channels`      | string   | vision_align — CSV payload channels fired ONCE mid-hold, e.g. `"1,2"` |
| `fire_t`             | float32  | vision_align — seconds into the hold to fire (must be `< hold_s`) |
| `lock_target`        | bool     | vision_align — **precision (`lock_on`):** steer to the box nearest the last centre, not the largest |
| `ctrl_conf`          | float32  | vision_align — **precision:** control-side min detection score (live: `vision.ctrl_conf`) |
| `range_gain_floor`   | float32  | vision_align, vision_move — **precision:** lat/depth kp multiplier as the bbox fills (live: `vision.range_gain_floor`) |
| `ki_lat`             | float32  | vision_align — **precision:** lateral integral gain during the hold (live: `vision.ki_lat`) |
| `coast_s`            | float32  | vision_align, vision_move — **gap-bridging coast (opt-in):** steer on the tracker's predicted box of the locked id for `coast_s` s after a detection drops, decaying authority; default `0.8` s, `0`=OFF, live detection always wins (live: `vision.coast_s`) |
| `pass_through`       | bool     | move_*/arc — reinterpret `gain`(+`yaw_rate_pct`) as a RAW PWM delta (1500±value) |
| `hold_through_loss`  | bool     | vision_align, vision_move — coast on target loss (the DSL sets this when no `fallback` is supplied) |
| `kp_lat`             | float32  | vision_align, vision_move             |
| `kp_yaw`             | float32  | vision_align                          |
| `kp_depth`           | float32  | vision_align                          |
| `kp_forward`         | float32  | vision_move                           |
| `lost_grace_s`       | float32  | vision_align, vision_move — coast seconds before reporting LOST |
| `align_stable_frames`| float32  | vision_align — distinct in-band DETECTIONS (not loop ticks) before ALIGNED; one lucky frame at low FPS can't declare aligned or arm the fire |
| `fire_channel`       | float32  | `fire` — ESP32 payload channel: 1/2=torpedo, 3/4=dropper, 0=stub |

The vision `kp_*`, `lost_grace_s`, `align_stable_frames`, and `fwd_fill`
fields are normally left at the rosidl zero by mission code; the manager
then fills them from live `vision.*` ROS params (see §2 "Gains & grace").
The two-verb DSL (`duburi.vision.align` / `.move`) does not expose
per-call `kp_*` at all — set them via ROS param or the raw CLI fields.

### `Move.Result` fields you get back

| Field           | Meaning                                                                          |
| --------------- | -------------------------------------------------------------------------------- |
| `success`       | True iff the verb completed without error                                        |
| `message`       | Human-readable outcome / failure reason                                          |
| `final_value`   | Axis-correct: yaw deg for yaw verbs, depth m for depth verbs, etc. (see `Move.action` for the full mapping) |
| `error_value`   | Axis-correct error: heading error deg / depth error m / age-of-last-detection s  |
| `end_x_px` / `end_y_px` | **Vision verbs:** signed target-from-centre px at the last seen frame (`NaN` if never seen). Surfaced on the DSL `VisionResult` as `x_px`/`y_px`/`saw_target`. Non-vision verbs leave them `NaN`. |
| `fill_frac`     | **Vision_move:** bbox fill fraction at exit [0..1] (`0` otherwise) → `VisionResult.fill` |
| `elapsed_s`     | **Vision verbs:** verb duration → `VisionResult.elapsed_s`                        |

> The two-verb DSL returns a `VisionResult` you read directly — you rarely touch the
> raw `Move.Result`. See [`vision-results.md`](vision-results.md) for the mission-facing
> contract. **Feedback** (`Move.Feedback`) also gained `err_x_px`/`err_y_px` — the live
> signed px at ~2.5 Hz during a vision verb.

### Exceptions

| Exception        | When                                                            |
| ---------------- | --------------------------------------------------------------- |
| `MoveRejected`   | Server refused the goal (e.g. another command already active, or the action server is down). |
| `MoveFailed`     | Server accepted the goal but `result.success` was False (timeout, mode rejected, motion exception). |
| `TimeoutError`   | `wait_for_connection` couldn't reach the server.                |
| `ValueError`     | `send(...)` called with a `cmd` not in the `COMMANDS` registry. |

### Why blocking?

Missions are scripts. Each line is one MAVLink command that runs to
completion before the next line starts. There is no parallelism
*inside* a mission -- it's a straight line, top to bottom. Daemon
threads (`HeadingLock`, `Heartbeat`) handle the few things that
*do* run in parallel; the mission-author surface stays sequential.

---

## 2. `DuburiMission` -- the mission DSL

This is what every `missions/<name>.py` file actually receives:

```python
def run(duburi, log):       # `duburi` is a DuburiMission instance
    duburi.camera = 'forward'
    duburi.models(gate='gate_flare_medium_100ep')

    duburi.arm()
    duburi.set_depth(-1.0)
    duburi.move_forward(3.0, gain=40)

    # Centre the gate on yaw + lateral, then drive in until it fills 80% of frame
    duburi.vision.align(duburi.models.gate.gate, yaw=0, lat=0, gain=30, duration=20)
    duburi.vision.move(duburi.models.gate.gate, fwd=80, mode='area', gain=35, duration=20)
    duburi.move_forward(3.0, gain=55)
    duburi.disarm()
```

Internally `DuburiMission` does three things:

1. **Wraps a `DuburiClient`** -- every verb funnels through
   `self._send(cmd, **fields)` which calls `client.send(cmd, **fields)`.
2. **Logs every outcome** -- `_format_outcome(cmd, result)` produces
   one human-readable line per verb (`[OK ] move_forward 5.0 s
   gain=60 final=-0.50 m`) so missions don't need any logging
   boilerplate.
3. **Holds sticky context** -- `self.camera = 'forward'` and
   `self.target = 'person'` are read by every `duburi.vision.*` call
   that doesn't specify them explicitly.

### Constructor

```python
DuburiMission(client, log, *, camera='forward', target='person')
```

You don't usually call this -- the runner
([`src/duburi_planner/duburi_planner/mission.py`](../../src/duburi_planner/duburi_planner/mission.py))
constructs it for you and hands it to your `run(duburi, log)`.

### Model context registry (`duburi.models`)

`duburi.models` is a `ModelRegistry` singleton always present on every `DuburiMission`.
Call it to register named model aliases, then pass typed `ClassRef` objects to vision verbs.

```python
# Register one or more model aliases before first use
duburi.models(gate='gate_flare_medium_100ep')

# Multiple aliases in one call (all loaded at detector startup)
duburi.models(
    gate='gate_flare_medium_100ep',
    slalom='slalom_combined_100ep',
)

# Access any class on a registered model
duburi.models.gate.gate     # ClassRef → auto-switches model+class before each vision goal
duburi.models.gate.flare    # ClassRef for flare class on the gate model
duburi.models.slalom.slalom_red

# Positional access (requires explicit class list at registration)
duburi.models(gate=('gate_flare_medium_100ep', ['gate', 'flare']))
duburi.models.gate[0]       # → ClassRef('gate', 'gate')
duburi.models.gate[1]       # → ClassRef('gate', 'flare')
```

When a `ClassRef` is passed to any vision verb, `_resolve_target()` automatically
calls `set_model(alias)` + `set_classes(class_name)` on the detector before the
goal fires — no manual `duburi.use()` / `duburi.set_classes()` calls needed.

### Top-level verbs (`duburi.*`)

Every verb that exists in [`command-reference.md`](./command-reference.md)
is exposed as a method on `DuburiMission`. The signature uses
human-friendly kwarg names where they differ from the action field
names:

```python
duburi.arm(timeout=15.0)
duburi.disarm(timeout=20.0)
duburi.set_mode('ALT_HOLD', timeout=8.0)

duburi.stop()
duburi.pause(seconds=2.0)

duburi.set_depth(meters=-1.5, timeout=30.0)

duburi.move_forward(seconds=5.0, gain=80.0, settle=0.0)
duburi.move_back   (seconds=5.0, gain=80.0)
duburi.move_left   (seconds=5.0, gain=80.0)
duburi.move_right  (seconds=5.0, gain=80.0)

duburi.yaw_left (degrees=90.0, timeout=30.0)
duburi.yaw_right(degrees=90.0, timeout=30.0)

duburi.arc(seconds=5.0, gain=50.0, yaw_rate_pct=30.0)

duburi.lock_heading(degrees=0.0, timeout=300.0)   # returns immediately
duburi.release_heading()                          # joins the daemon

# DVL closed-loop distance (heading lock stays active during these)
duburi.dvl_connect()                              # manual connect; auto-connect is default
duburi.move_forward_dist(meters=2.0, gain=60.0, dvl_tolerance=0.1)
duburi.move_lateral_dist(meters=1.0, gain=36.0, dvl_tolerance=0.1)
# Negative meters = reverse / left:
duburi.move_forward_dist(-1.5, gain=60.0)         # 1.5 m backward
duburi.move_lateral_dist(-0.5, gain=36.0)         # 0.5 m left

# Payload (ESP32 serial — PayloadDriver auto-detected at startup)
duburi.fire(fire_channel=1)    # 1/2=torpedo_1/2, 3/4=dropper_1/2
duburi.payload_ready           # bool — True if PayloadDriver connected

duburi.countdown(seconds=10)                      # tether-removal countdown with banner

# Scoreboard — called automatically by mission.py; also callable mid-mission
duburi.log_scoreboard()                           # print table to stdout
duburi.log_scoreboard(json_path='auto')           # + write mission_scoreboard_YYYYMMDD_HHMMSS.json
duburi.log_scoreboard(json_path='/tmp/run.json')  # + write to explicit path
```

**DVL distance gotchas:**
- Heading lock is NOT suspended during `move_forward_dist` / `move_lateral_dist`.
  The lock owns Ch4 (yaw rate) and keeps the AUV on heading while DVL owns Ch5/Ch6.
  Call `lock_heading()` BEFORE the distance moves.
- Falls back to open-loop time estimate (with a warning) if the yaw source has no
  DVL component (i.e. `yaw_source=mavlink_ahrs` or `yaw_source=bno085`).
- `dvl_connect()` is only needed if `dvl_auto_connect:=false`. The default is
  `dvl_auto_connect:=true` (auto-connect at manager startup).

### Vision verbs (`duburi.vision.*`)

`duburi.vision` is a `_VisionDSL` sub-namespace with **exactly two**
pixel-native verbs. Both block until they reach their goal, time out, or
exhaust their `duration` budget, and **neither ever raises** — each
returns a [`VisionResult`](#visionresult) and the mission keeps running.

| Verb | What it does |
|------|--------------|
| `vision.align(target, *, lat=, yaw=, depth=, fwd=, fwd_mode=, hold=, ...)` | Centre `target` on the named axes, each at a signed **pixel offset** from frame centre. `hold=`s = **active station-keep**: keep correcting on-target for `hold` s before exiting (holds the hull steady for a torpedo/dropper shot); counts against `duration`. **`fwd=`** (% fill, `fwd_mode=`area/width/height) adds a **forward range-hold axis** — align ALSO drives forward to that standoff and holds it, so one verb does forward-standoff + lat/depth + hold + mid-hold fire (the torpedo standoff shot; one-sided forward, never reverses; fire gated on the standoff too). `fwd` unset = lat/yaw/depth only. |
| `vision.move(target, *, fwd=, mode=, ...)` | Drive forward until `target`'s bbox fills `fwd` % of the frame. Never re-centres yaw/depth. |

#### `vision.align` — centre on lat / yaw / depth

```python
duburi.vision.align(
    target,                 # class str OR duburi.models.<alias>.<class> (ClassRef)
    *,
    lat=None,               # None = axis OFF; number = ON (signed px offset from centre)
    yaw=None,               # None = axis OFF; number = ON (signed px offset from centre)
    depth=None,             # None = axis OFF; number = ON (signed px offset from centre)
    err=40,                 # per-axis in-band tolerance (px)
    duration=20,            # total budget (s), including any fallback cycles
    gain=30,                # HARD max-speed cap (% thrust) — never exceeded (lat/yaw)
    lat_gain=None, yaw_gain=None,   # per-axis cap (None = inherit gain). Depth: no % cap
    depth_step=None,        # depth-setpoint resolution (m/update): 0.02 slow .. 0.10 coarse;
                            #   freezes in-deadband so ArduSub settles (no z-wobble). None = 0.02
    brake=True, brake_gain=None,   # lateral arrival brake (brake=False to coast / fire)
    hold=None,              # active station-keep (s) after centring — fights inertia
    fire=None,              # mid-hold payload fire: channel int or list (1/2 torpedo, 3/4 dropper)
    fire_t=None,            # s into the hold to fire (0 = at lock); must be < hold
    lock_on=False,          # precision continuity lock: steer to the box nearest the last centre
    settle=None,            # settle gate (px): end SETTLED not mid-pass — coarse aligns only,
                            #   NEVER the fire-lock (it gates the mid-hold fire)
    fire_pass=False,        # fire at command end even if never fully aligned (seen live+recently)
    hold_heading=False,     # widen heading-lock deadband for the hold (steady launcher heading)
    fallback=None,          # search fn run on target loss (see Fallback)
    camera=None,            # defaults to duburi.camera ('forward')
) -> VisionResult

# The mid-hold fire leaves ONLY on the tick a genuinely NEW, non-coasted detection
# lands (is_new_frame) — FPS-robust yet never on a frozen/coasted box. fwd=<%fill>
# (+ fwd_mode) adds the forward range-hold standoff axis (the unified torpedo shot).
```

> `hold` / `fire` / `fire_t` fire a payload **mid-hold while still correcting** —
> see [`vision-results.md`](vision-results.md) §4 (gated on alignment, non-blocking).
>
> `lock_on=True` + the deck params `vision.ctrl_conf` / `vision.range_gain_floor` /
> `vision.ki_lat` are the close-in precision layer (kill last-moment misclass +
> hold a 20 kg hull on a small target). All off by default —
> [`precision-alignment.md`](precision-alignment.md). Note **`err=0` means "use the
> default", not zero-tolerance** (rosidl 0==unset); pass a small positive `err` for
> a tight deadband (floored at ~5 px).

- Each of `lat` / `yaw` / `depth` is `None` (axis off) or a **number**
  (axis on; the number is the signed pixel offset from centre — `0` =
  centre, `+` = right/below, `-` = left/above). `lat` and `yaw` are
  horizontal (strafe / rotate); `depth` is vertical. **At least one axis
  is required** (passing none raises `ValueError` before any motion).
- Aligned when every active axis stays within `err` px for
  `vision.align_stable_frames` consecutive ticks (default 3).
- A `depth` axis engages `ALT_HOLD` and nudges the depth setpoint. With a
  live heading lock and no `yaw` axis, the lock keeps Ch4 and align only
  strafes. The `downward` / `sim_bottom` cameras auto-negate the depth
  correction (target large = already close).

```python
# Centre the gate dead-centre on yaw + lateral
duburi.vision.align(duburi.models.gate.gate, yaw=0, lat=0, gain=30, duration=20)

# Keep a slalom pipe 80 px to the RIGHT of centre (pass on its left)
duburi.vision.align('slalom_red', lat=80, err=30, duration=6)

# Bin: centre laterally + vertically on the downward camera
duburi.vision.align('bin', lat=0, depth=0, camera='downward', duration=20)
```

#### `vision.move` — drive forward to a bbox fill ratio

```python
duburi.vision.move(
    target,                 # class str OR ClassRef
    *,
    fwd=95,                 # stop when the bbox fills this % of the frame
    mode='area',            # fill metric: 'area' | 'width' | 'height'
    maintain=None,          # ±px lateral offset to hold while driving (None = pure forward)
    hold=None,              # seconds to station-keep at the fill target (None = exit on reach)
    err=40,                 # lateral in-band tolerance (px) when maintain is set
    duration=20,            # total budget (s)
    gain=30,                # HARD max-speed cap (% thrust)
    fallback=None,          # search fn run on target loss
    camera=None,            # defaults to duburi.camera ('forward')
) -> VisionResult
```

- Drives forward until the bbox fill reaches `fwd` % under the chosen
  `mode` (`area` = sqrt(w*h), `width` for wide bars like the gate,
  `height` for tall pipes). `maintain` holds a lateral pixel offset while
  driving; `maintain=None` is pure forward and never touches lat / yaw /
  depth.
- `hold` station-keeps at the fill target for that many seconds before
  exiting (`None` exits on first reach). Depth is left to ArduSub's
  depth-hold; yaw is left to the heading lock / autopilot.

```python
# Drive in until the gate fills 80% of the frame
duburi.vision.move(duburi.models.gate.gate, fwd=80, mode='area', gain=35, duration=20)

# Approach a tall slalom pipe by height, holding it 60 px to the right
duburi.vision.move('slalom_red', fwd=55, mode='height', maintain=60, duration=15)
```

#### `gain` is a speed cap, not a target speed

For both verbs `gain` is the **hard maximum** thrust (%) the P-controller
output is clamped to — the AUV slows as the error shrinks and never
exceeds `gain` on any axis. Lower it for tight quarters; raise it to close
distance faster.

#### `target` resolution

`target` is either a class string (`'gate'`) or a `ClassRef`
(`duburi.models.gate.gate`). A `ClassRef` auto-switches the detector
model + class filter before the goal fires (`set_model` + `set_classes`);
a bare string is used as-is. Omit `target` to fall back to the sticky
`duburi.target`. Matching is **case-insensitive**.

#### `VisionResult`

Both verbs return a `VisionResult` dataclass — **truthy only on success**
(`__bool__` returns `ok`), so `if duburi.vision.align(...):` reads
naturally. The non-`ok` fields are the **finish-state** a hybrid
vision+control mission branches on — read **where/how** the verb ended and run
tested open-loop recovery. Full contract, recovery patterns & pitfalls:
[`vision-results.md`](vision-results.md).

| Field | Meaning |
|-------|---------|
| `ok` | `True` iff aligned (align) / reached fill (move). `bool(res)` == this. |
| `status` / `reason` | Outcome string: `'ALIGNED'`, `'LOST'`, `'TIMEOUT'`, `'NO_CAMERA'`, `'ABORTED'`, or `'FAILED'` |
| `code` | Raw integer outcome code from the server |
| `x_px` / `y_px` | **Signed** px of the target from frame **centre** at the last seen frame (`+x`=ended right, `+y`=ended below); **`NaN` if never seen**. Recovery sign matches `align`: `x_px>0` → `move_right`. |
| `saw_target` | Bool — target detected at least once. **Check before reading `x_px`** (`NaN<threshold` is silently False). |
| `last_err_px` | Worst residual px from the **goal** (centre+offset) at exit |
| `fill` | Bbox fill fraction at exit (move; `0` for align) |
| `elapsed_s` | Verb duration (s) |

Hybrid-recovery shape (full version in [`vision-results.md`](vision-results.md)):
```python
res = duburi.vision.align('gate', yaw=0, lat=0)
if res:               duburi.move_forward(3)                 # aligned
elif res.saw_target:                                         # saw it, off-centre
    duburi.move_right(1) if res.x_px > 30 else duburi.move_left(1)
else:                                                        # never saw it -> search
    while not duburi.detected('gate'): duburi.move_forward(0.6, gain=35)
```

Outcome codes (defined in `motion_vision`, copied into
`Move.Result.final_value`):

| Code | Name | Meaning |
|------|------|---------|
| `0` | `ALIGNED` | Centred (align) / reached fill (move) |
| `1` | `LOST` | Target gone past `lost_grace_s` — DSL runs `fallback`, then re-enters |
| `2` | `TIMEOUT` | `duration` elapsed without success |
| `3` | `NO_CAMERA` | No `camera_info` seen — pipeline not up |
| `4` | `ABORTED` | Cooperative abort (goal cancelled) |

A server / setup error (bad camera name, `ALT_HOLD` rejected, disarmed)
surfaces in the DSL as a non-fatal `VisionResult(False, 'FAILED')` — the
mission moves to its next step instead of unwinding.

```python
res = duburi.vision.align(duburi.models.gate.gate, yaw=0, lat=0, duration=20)
if res:
    duburi.vision.move(duburi.models.gate.gate, fwd=80, gain=35)
else:
    log.warn(f'gate not centred: {res.reason} (err {res.last_err_px:.0f}px)')
```

#### Fallback search (recover, don't fail)

`fallback` is a mission-authored search function run **once per target
loss**, after which the verb re-enters its loop with the remaining
`duration` budget. It comes in two shapes:

```python
def nudge(duburi):                     # one short manoeuvre, then return
    duburi.move_forward(0.6, gain=35)

def sweep(duburi, should_stop):        # longer self-polled sweep
    for _ in range(8):
        if should_stop():              # True the moment the target reappears
            return
        duburi.yaw_right(10)

duburi.vision.align(duburi.models.gate.gate, yaw=0, lat=0,
                    duration=40, fallback=sweep)
```

`should_stop()` is `duburi.detected(target, camera=...)`. With **no**
`fallback`, the verb instead coasts through brief losses (it sets
`hold_through_loss=True` for you) until `duration` runs out. A `fallback`
that raises is caught and logged — it can never kill the mission.

#### Firing — mid-hold (accurate) or align-then-fire (simple)

**Preferred (accurate):** fire **mid-hold** with `fire`/`fire_t` so the shot leaves
while `align` is *still correcting* against the target — no align-then-fire drift gap:

```python
duburi.vision.align('torpedo_hole', yaw=0, lat=0, depth=0, err=12,
                    gain=25, yaw_gain=10, hold=4, fire=1, fire_t=1, brake=False)
```
Gated on alignment (never fires off-target), non-blocking, `fire_t < hold`. See
[`vision-results.md`](vision-results.md) §4.

**Simple (has a drift gap — only when the hull is already steady):** compose
`vision.align` (or `.move`) with the standalone `duburi.fire(channel)` verb:

```python
if duburi.vision.align('torpedo_hole', yaw=0, lat=0, depth=0, err=12).ok:
    duburi.fire(1)        # 1/2 = torpedo, 3/4 = dropper
```

#### Gains & grace are live-tunable

The DSL verbs deliberately take **no per-call `kp_*`**. Gains, grace, and
the stable-frame count come from live `vision.*` ROS params on
`/duburi_manager` (applied on the NEXT goal), so deck-side tuning needs no
mission edit:

| ROS param | Default | Feeds |
|-----------|---------|-------|
| `vision.kp_lat` | `60.0` | align lat, move maintain |
| `vision.kp_yaw` | `60.0` | align yaw |
| `vision.kp_depth` | `0.05` | align depth (m/tick) |
| `vision.kp_forward` | `200.0` | move forward |
| `vision.lost_grace_s` | `1.0` | both — coast seconds before LOST |
| `vision.frame_fill_default` | `95.0` | move `fwd_fill` when left unset |
| `vision.align_stable_frames` | `3` | align in-band ticks before ALIGNED |
| `vision.range_gain_floor` | `1.0` | **precision:** lat/depth kp multiplier as the bbox fills close-in (`1.0`=off, `~0.3`=gentle) |
| `vision.ki_lat` | `0.0` | **precision:** lateral integral gain; nulls a steady-current offset during the hold (`0`=off) |
| `vision.ctrl_conf` | `0.0` | **precision:** control-side min detection score to accept a box (`0`=off) |
| `vision.coast_s` | `0.8` | **gap-bridging coast (ON):** steer on the tracker's predicted box of the locked id for this many s after a detection drops (`0`=OFF). `< vision.lost_grace_s`. See [`BUGS.md`](BUGS.md) D10 |

```bash
ros2 param set /duburi_manager vision.kp_yaw 80.0
```

Defaults live in
[`vision_tunables.py`](../../src/duburi_manager/duburi_manager/vision_tunables.py);
the engine floor is in
[`motion_vision.py`](../../src/duburi_control/duburi_control/motion_vision.py).
The two raw CLI verbs (`vision_align` / `vision_move`) additionally accept
`kp_*`, `lost_grace_s`, and `align_stable_frames` as one-off goal fields —
see [`command-reference.md`](./command-reference.md).

#### FSM state wrappers

The YASMIN FSM layer wraps these same two verbs as states — `VisionAlign`,
`VisionMove`, and `VisionSearch` in
[`state_machines/states/vision.py`](../../src/duburi_planner/duburi_planner/state_machines/states/vision.py).
See [`fsm-guide.md`](./fsm-guide.md) §4.

#### Migration from the old 9-verb API (removed)

The previous axis-named / body-named verbs are gone. Map old -> new:

| Removed | Replacement |
|---------|-------------|
| `vision.turn` / `vision.slide` / `vision.hover` (and `vision_align_yaw` / `_lat` / `_depth`) | `vision.align(target, yaw=.../lat=.../depth=...)` |
| `vision.home` / `vision.track` / `vision.hold` / `vision_align_3d` | `vision.align(...)` then `vision.move(...)` |
| `vision.approach` / `vision_hold_distance` | `vision.move(target, fwd=, mode=)` |
| `vision.find` / `vision.scan` / `vision_acquire` / `look_around` | `duburi.detected()` search loop + `fallback=` |
| `vision_lock_fire` | `vision.align(...).ok` then `duburi.fire(ch)` |

Dropped fields/kwargs (`on_lost`, `lock_mode`, `dist`, `metric`,
`deadband`, `gate_guard`, `pass_at`, `offset_x/y`, `tracking`,
`downward_cam`, per-call `kp_*`, the `--tracking` flag, and the
`vision.deadband` / `vision.use_tracks` / `vision.distance_metric` params)
have no direct replacement — the two verbs cover their roles with
`lat/yaw/depth` offsets, `fwd/mode`, `fallback`, ROS-param gains, and
`camera=`.

### Vision queries — `detected()` / `wait_for()` / `where()`

Client-side reads of the `/duburi/vision/<camera>/detections` stream the
control loop acts on. Each **pumps the node** so the answer reflects the
current frame; the default camera is subscribed eagerly (and `use_camera`
subscribes a new one) so the first call never false-negates on DDS discovery.
They run between goals only — safe in search loops and vision `fallback`s.

```python
duburi.detected(
    target_class,                 # str | ClassRef — e.g. 'gate', duburi.models.gate.gate
    *,
    camera: str | None = None,    # defaults to duburi.camera
    stale_after: float = 1.0,     # seconds; detections older → False
) -> bool                          # visible RIGHT NOW?

duburi.wait_for(
    target_class, *, timeout=10.0, camera=None, stale_after=1.0,
) -> bool                          # block until seen / timeout (loop-free acquire)

duburi.where(
    target_class, *, camera=None, stale_after=1.0, band=0.15,
) -> str                           # 'left' | 'center' | 'right' | 'unknown'
duburi.where_offset(target_class, ...) -> float | None   # signed [-1,+1]
```

> **`if` runs once — a moving search needs a `while`.** `if detected(): a else: b`
> executes one branch and falls through; it is **not** a loop. To search while
> moving, `while not detected(): <small move>`. To wait in place, use `wait_for`.

**The core paradigm (reactive missions):**

```python
# Move in short steps until gate visible, then align
MAX_STEPS = 60
for _ in range(MAX_STEPS):
    if duburi.detected(duburi.models.gate.gate, stale_after=0.5):
        break
    duburi.move_forward(0.5, gain=30)   # 0.5s max — avoids overshoot
else:
    return   # not found

duburi.vision.align(duburi.models.gate.gate, yaw=0, lat=0, duration=20)
```

**Critical rules:**

| Rule | Consequence if broken |
|------|----------------------|
| Step size ≤ 0.5 s in search loops | 2s step = 0.6m overshoot past detection point |
| Always have `MAX_STEPS` budget | Detector offline → infinite loop |
| Call `set_classes('gate,flare')` before orbit `detected('gate')` | `vision.align(flare_ref, ...)` silently filters detector to flare; gate detection always False |
| Set `duburi.camera` at top of `run()` | Default is `'forward'`; a wrong camera subscribes the wrong `/detections` topic |

**`detected()` accepts `ClassRef` without model-switching side-effects:**
```python
# ✓ ClassRef — extracts class_name only, does NOT call set_model/set_classes
while not duburi.detected(duburi.models.gate.gate):
    duburi.move_forward(0.5, gain=30)
```

**`detected()`/`where()` block only for their bounded pump** (≤0.25 s warm,
≤0.60 s cold-on-first-frame); `wait_for` blocks up to its `timeout`. None
depend on a recent verb to be fresh — the pump reads the current frame.

**`where()` bearing steer:**
```python
{'left':  lambda: duburi.yaw_left(20),
 'right': lambda: duburi.yaw_right(20),
}.get(duburi.where('gate'), lambda: duburi.move_forward(1))()
```

**Internals:** each frame is parsed to plain-Python `DetRecord` tuples
(`class, cx, cy, w, h, conf`) eagerly in `_on_detections()`. The ROS message
is never stored — rclpy reuses the underlying C++ memory across callbacks.
`count()`/`visible()`/`proximity()` are easy future additions on this cache
(not built yet).

Full reference: [`detected-paradigm.md`](./detected-paradigm.md) — mechanics, all rules,
error patterns, testing procedures, canonical templates, orbit anti-patterns.

---

### Escape hatch -- raw `send`

```python
duburi.send('whatever_new_verb', some_field=1.2, other_field='x')
```

Anything not defined as an explicit verb falls through `__getattr__`
to the underlying `DuburiClient.send` and STILL gets the one-line
outcome log. That means any future row in the `COMMANDS` registry
is reachable as `duburi.<cmd>(...)` even before someone wires it
explicitly above.

### Outcome logging format

Every successful verb prints one line via the mission `log`:

```
[OK ] move_forward 5.0 s gain=60.0  final=-0.50 m  err=0.000
[OK ] yaw_right    90 deg            final=125.4 deg  err=-0.4
[OK ] vision_align yaw+lat           final=0 (ALIGNED) err=32 px
[OK ] vision_move  area              final=0 (ALIGNED) err=0.81 fill
```

(For the two vision verbs `final_value` carries the outcome **code** —
`0`=ALIGNED — and `error_value` carries the worst pixel error for
`vision_align` or the bbox fill fraction for `vision_move`.)

Failures raise `MoveFailed` (the runner prints the traceback at the
top level, so missions don't need a try/except unless they want to
recover and continue).

---

## 3. `Duburi` -- the manager-side facade (do not instantiate)

This is the **implementation** of the verbs. The `auv_manager_node`
ActionServer constructs one instance per process, owns it for the
lifetime of the manager, and dispatches incoming `/duburi/move`
goals through it.

### What it owns

| Resource          | Why                                                                  |
| ----------------- | -------------------------------------------------------------------- |
| `Pixhawk`         | The MAVLink connection. Single instance per process.                 |
| `HeadingLock`     | Daemon thread for the Ch4 yaw-rate streamer.                         |
| `Heartbeat`       | Daemon thread for the 5 Hz neutral RC override (FS_PILOT_INPUT guard). |
| `vision_state_provider` callable | Returns the `VisionState` cache instance for vision verbs. |
| `threading.Lock`  | Serialises every command -- AT MOST ONE motion command runs at a time. |

### What you'd touch only when extending

Adding a new verb is a 4-line change:

1. Add a row to `COMMANDS` in `commands.py` (defines fields + defaults).
2. Add a method on `Duburi` (open-loop) or `VisionVerbs` (vision)
   whose name matches the row's key.
3. (optional) Add a friendlier wrapper to `DuburiMission` if the
   default kwarg naming is awkward.
4. (optional) Add a one-line CLI entry in
   `src/duburi_planner/duburi_planner/cli.py` if you want a
   command-line affordance distinct from `duburi <cmd>`.

The dispatch already auto-discovers any new `COMMANDS` row -- the
client and CLI just work without further edits.

### Command-internal context manager

Inside `Duburi`, every command method runs under
`with self._command_scope(verb):`, which:

1. Acquires `self.lock` (serialises commands).
2. Opens a `tracing.command_scope(verb)` so every MAVLink frame the
   verb emits carries `cmd=<verb>` in its `[MAV ]` DEBUG line when
   the manager is started with `debug:=true`.
3. Pauses `self._heartbeat` (so its 5 Hz neutral writes don't race
   the command's own RC writes).
4. On exit: resumes the heartbeat (unless a different command path
   already did, e.g. `lock_heading` keeps it paused for the lifetime
   of the lock).

Vision verbs in `VisionVerbs` use the same `_command_scope()`. There
is no branch in user code that needs to know about either the lock,
the tracing tag, or the heartbeat -- this is the contract that makes
the verbs composable.

---

## 4. Cross-references

* CLI driver (the `duburi` shell command): [`cli.py`](../../src/duburi_planner/duburi_planner/cli.py)
* Mission runner (auto-discovers `missions/*.py`): [`mission.py`](../../src/duburi_planner/duburi_planner/mission.py)
* Action server dispatch (verifies field shape, hands to `Duburi`): [`auv_manager_node.py`](../../src/duburi_manager/duburi_manager/auv_manager_node.py)
* All verbs (CLI + Python + DSL forms in one table): [`command-reference.md`](./command-reference.md)
* Mission cookbook (composing verbs): [`mission-cookbook.md`](./mission-cookbook.md)
* Testing every layer end-to-end: [`testing-guide.md`](./testing-guide.md)
