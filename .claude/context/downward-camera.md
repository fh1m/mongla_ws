# Downward camera — the axis flip, explained once and for all

> The single most confusing thing in the vision stack: **the same `vision.align`
> kwargs mean different physical axes on the forward vs the downward camera.** This
> doc is the canonical reference. Bin task: [`task_bin.py`]; dual-cam hardware:
> [`dual-camera-setup.md`](dual-camera-setup.md); align internals: `motion_vision.align_loop`.

## Why the axes flip at all

The forward camera looks **ahead** (along the hull's surge axis). The downward camera
looks **straight down** (along the hull's heave/depth axis). So the *same pixel motion*
in the two images corresponds to *different hull motions*:

| Image motion | Forward cam → hull axis | Downward cam → hull axis |
|---|---|---|
| target moves **left/right** (image-X) | strafe (Ch6 lateral) | strafe (Ch6 lateral) — **same** |
| target moves **up/down** (image-Y) | rise/sink (depth, Ch3) | **fore/aft (Ch5 surge)** — rotated |
| target **grows/shrinks** (bbox fill) | nearer/farther in surge | nearer/farther in **depth** (altitude) |

`lat` (image-X → strafe) is the **only** axis that means the same thing on both cameras.
The other two rotate 90° because the camera's optical axis rotated 90°.

## The kwarg mapping (what you type)

To keep the operator's mental model simple — **`fwd` is always the fore/aft joystick,
`depth` always drives the real depth setpoint** — the DSL **swaps which kwarg feeds
which** on the downward camera. You never do the rotation in your head; you just use
the kwargs by their plain meaning.

```
                 lat            fwd                     depth
FORWARD cam:   Ch6 strafe    fwd standoff (fill %)    Ch3 up/down (signed px)
DOWNWARD cam:  Ch6 strafe    Ch5 surge (signed px)    DESCENT to fill % (fwd_mode)
                             └─ image-Y offset         └─ "get closer" = go deeper
```

- **`lat`** — signed pixel offset, image-X → **Ch6 strafe**. Same on both. `0` = centre.
- **`fwd`** —
  - forward cam: a **fill %** standoff (drive forward until the bbox fills `fwd`%).
  - downward cam: a **signed pixel offset**, image-Y → **Ch5 surge** fore/aft. `0` = centre.
- **`depth`** —
  - forward cam: a **signed pixel offset**, image-Y → **Ch3 depth** setpoint.
  - downward cam: a **fill %** DESCENT target (descend until the bbox fills `depth`%,
    measured by `fwd_mode` = area/width/height). Unset = no descent (hold `set_depth`).

> **Depth-hold on downward is now enforced (fix 2026-07-15, known-issues P4).** A downward
> **surge-only** align (`lat` + `fwd`, no `depth` descent — the bin task) used to RELEASE Ch3
> while streaming no setpoint, so the hull **sank to the floor ignoring `set_depth`**. The engine
> now streams the **constant** `set_depth` target on every downward align (same mechanism as the
> forward depth axis), so **`set_depth` holds by itself** — you do **not** need `max_depth_m` /
> `depth_ceiling` to keep depth. Those two only bound the optional **descent**, are **NEGATIVE
> metres** (a positive value is a sign error → warns + reads as OFF), and as **per-call kwargs
> override any `ros2 param set vision.*`**. Minimal bin path = `set_depth` + `lat`/`fwd` centering
> + drop; add `depth=<fill%>` + a **negative** `max_depth_m` floor only if you want to descend.

So the *types* of `fwd` and `depth` swap between cameras (pixel ↔ fill%). That is the
price of "fwd = fore/aft, depth = depth" reading naturally on both. `fwd_mode` measures
the fill for whichever axis is the fill axis (forward `fwd`; downward `depth`).

## Canonical calls

Forward — torpedo standoff shot (fill on `fwd`, pixel on `depth`):
```python
align('hole', camera='forward', lat=0, depth=0, fwd=25, fwd_mode='height',
      lock_on=True, hold=4, fire=1)          # depth=0 -> centre vertically; fwd=25% standoff
```

Downward — bin drop (pixel surge on `fwd`, fill descent on `depth`). `surge_sign`,
`max_depth_m`, `depth_ceiling` are **no longer per-`align` kwargs** — they're
`vision.*` tunables (see "Bounds & signs" below), so the call is now just:
```python
# once at the top of the mission (per-mission, not per-command):
duburi.set_vision_param('max_depth_m', BIN_MAX_DEPTH_M)     # <0: floor + enables descent
duburi.set_vision_param('depth_ceiling', BIN_DEPTH_CEILING_M)  # surface guard
# ... then every downward align omits all three (surge_sign is the -1 deck default):
align('fire', camera='downward', lat=0, fwd=0, depth=30, fwd_mode='height')
```

- Centring axes on downward = **`lat` + `fwd`** (both horizontal thrusters). `depth` is the
  optional approach. A `depth`-only call (no `lat`/`fwd`) **raises** — there's no centring
  axis, exactly like a forward `fwd`-only align.
- `fwd=0` on downward is **active surge-to-centre** (0 ≠ unset). The `None`-vs-`0`
  distinction is why the swap lives in the DSL (Python), not the ROS goal (where rosidl
  collapses `0`→unset).

## Bounds & signs (downward descent safety) — now `vision.*` tunables

These three moved off per-`align` kwargs onto the manager's `vision.*` params
(`vision_tunables.py`), so a downward run sets them **once per mission** (or the deck
operator via `ros2 param set /duburi_manager vision.<name> <v>`) instead of on every call.
A per-call kwarg still overrides (layered default), but missions no longer need to pass them.

- **`vision.surge_sign`** (**PERMANENT default −1**) flips the **Ch5 fore/aft polarity** for
  the physical bottom-cam mount. A wrong sign is **positive feedback** — the hull drives
  *away* from the bin. This hull needs −1 (the deck default), so missions **omit `surge_sign`
  entirely**. Downward-only (the forward path never reads it). **Run the DISARMED check
  before an armed run** if the mount changes: `ros2 run duburi_vision vision_thrust_check
  --camera downward` — a bin AHEAD in the image must drive the hull FORWARD (Ch5>1500);
  flip with `ros2 param set /duburi_manager vision.surge_sign +1` if reversed.
- **`vision.max_depth_m`** (default **0.0 = off**; set `<0` per mission) — the deepest
  allowed setpoint (floor). The fill→depth descent **requires a value < 0** or the engine
  drops the descent and just holds ArduSub depth (fail-safe against an unreachable fill
  target driving the hull to the bottom). `0.0` default = the FORWARD torpedo align is
  unchanged; a bin run sets it (`duburi.set_vision_param('max_depth_m', −1.6)`).
- **`vision.depth_ceiling`** (default **0.0** → engine `_MIN_DEPTH_M` surface guard; set a
  tighter negative like **−0.4** per mission) — the shallowest allowed setpoint. Alignment
  can **never surface the hull**.
- The descent is **one-sided** (deeper only) and uses the same `depth_step` stepped/
  deadband-frozen logic as the forward depth axis (no z-wobble).

## What the engine actually does (unchanged by the kwarg swap)

The swap is **purely a DSL kwarg remap**. `motion_vision.align_loop`, the `Move.action`
wire fields, and every thruster sign are **identical** to before — on the wire, the
downward `'depth'` axis has *always* driven Ch5 surge and `fwd_fill` has *always* driven
the descent. The DSL now just routes your `fwd` kwarg into that surge axis and your
`depth` kwarg into that descent, so the names read naturally. CLI users working at the
raw `vision_align` wire level (`--axes depth --offset_depth <px> --fwd_fill <%>`) use the
wire semantics directly (offset_depth = surge, fwd_fill = descent).

## Migration note (if you have old missions)

Old downward calls used `lat=0, depth=0` (surge on `depth`) and `fwd=<fill>` (descent).
New form: **`depth=0` → `fwd=0`** (surge), and **`fwd=<fill>` → `depth=<fill>`** (descent).
All in-repo missions (`task_bin`, `pool_day_practice`, FSM `bin_drop`/`gate_then_bin`) are
already migrated. `VisionAlignState` gained a `fwd=` param for the FSM path.
