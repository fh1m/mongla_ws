# The Jetson↔SROT vision/control split

> **Status: designed, not built.** The firmware spec is
> [`Mongla_others/srot-control-board/VISION_API.md`](../../../Mongla_others/srot-control-board/VISION_API.md);
> the board does not implement it yet. `vision_align` / `vision_move` remain **refused** on the
> srot backend (`srot_fc.UNSUPPORTED_VERBS`) until it does. Nothing in this document is live.

## The problem

Our vision control loop runs **20 Hz in Python on the Jetson**, computing thrust from pixel
error and pushing it down a USB link, while closing over a **10 Hz `ATTITUDE`** stream. The
SROT board runs a **500 Hz** loop, owns all 8 thrusters, and is told nothing about the target.

Control authority and perception are on opposite sides of a cable, and the loop is closed on
the wrong side of it. That is why alignment fights inertia on a 20 kg hull: a correction
derived from a 20 Hz observation, transported over serial, arrives after the hull has moved.

Raising gains does not fix this — it is a loop-rate and latency problem, not a tuning problem.

## The split

**Jetson = perception. Board = control.** Per frame the Jetson emits **one**
`LANDING_TARGET` (msgid 149) describing the currently-selected target as a **bearing**, and
the board closes every axis at 500 Hz with the IMU in hand.

### Why bearings, not pixels

Today's internal quantity is `Sample.ex = (cx − W/2)/(W/2)` (`vision_state.py:279`) — a
normalized pixel error. It is dimensionless but **not physical**: it depends on resolution,
lens and crop, so `kp_lat = 60.0` is silently a property of the camera, and swapping a lens
invalidates the tune with no warning.

Converting once on our side, with the camera's known FOV:

```
angle_x = ex     * (HFOV / 2)     # rad, + = target right
angle_y = ey     * (VFOV / 2)     # rad, + = target below
size_x  = w_frac * HFOV           # rad, angular width  → the standoff measure
size_y  = h_frac * VFOV           # rad, angular height
```

gives the board gains with **units** (thrust per radian) that are a property of the *vehicle*.
It is also what `LANDING_TARGET` already means, so we are using a standard message correctly
rather than as a container. And unlike `ESC_STATUS` (291), msgid 149 **is** in every pymavlink
dialect — verified — so it decodes on both sides.

**FOV is our responsibility.** It will live in `duburi_vision/config/cameras.yaml` per camera;
the board never needs image dimensions.

### Why selection stays here

`VisionState.bbox_error()` takes `near` / `locked_id` (`vision_state.py:186-191`), which looks
like control state feeding back into perception. It is not — `locked_ex/ey` is just *the last
sample perception itself emitted*, so it can stay entirely on this side.

That matters, because selection is not "biggest box": it is a class filter, a control-side
confidence floor (`vision.ctrl_conf`) distinct from the detector's, largest-area acquisition,
a **continuity lock** that switches to nearest-to-last-centre once locked (so a second torpedo
hole cannot steal the aim), and a Kalman **coast** on the locked track id through a dropout.
~150 lines of policy, already tuned against real pool failures, needing the tracker's state.

None of it needs 500 Hz. **Selection is perception; stabilisation is control.**

### What each side owns

| Stays on the Jetson | Moves to the board |
|---|---|
| detection, tracking, coast, continuity lock | the control law for every axis |
| the control-side conf gate (`ctrl_conf`) | thrust allocation, heading and depth |
| model / class switching | staleness handling and degraded behaviour |
| `detected()` / `where()` reactive layer | arbitration with the pilot and failsafes |
| mission DSL, fallback search patterns | |
| **payload fire gating** (needs frame identity) | |

## The timeout ladder — must stay consistent on both sides

Ours today, and the shape the board must mirror:

| Threshold | Meaning |
|---|---|
| **0.10 s** | full command authority |
| **0.40 s** | authority decayed to zero — translation neutral, still "not lost" |
| **0.80 s** | coast expires (`vision.coast_s`) |
| **1.00 s** | LOST declared (`vision.lost_grace_s`) → the DSL runs its fallback search |

The board's `VISION_STALE_MS` (300–500 ms proposed) sits between the first two. The ordering
is the point: **authority reaches zero before loss is declared**, so a dropout produces a
graceful glide rather than a lurch, and the mission-level fallback only fires once the target
is genuinely gone.

If either side changes a rung, both must move. This ladder is part of the contract.

## Migration path

The host loop is not deleted; it becomes the fallback and the reference implementation.

1. **Spec agreed** (now) — `VISION_API.md` on the board side, this file on ours.
2. **Uplink built here** — FOV config, the `ex,ey → angle` conversion, and a default-off
   `LANDING_TARGET` publisher. One-directional, so it is safe to stream at a board that
   ignores unknown messages. This is what validates the bandwidth (~1.9 kB/s at 25 Hz, 16 % of
   the link) and latency (~10 ms typical board-side ingest) numbers before the firmware
   commits to them.
3. **Board implements**, staged (`VISION_API.md` §8): decode + telemetry echo → staleness →
   yaw dry → yaw+lateral wet → standoff → depth → hold.
4. **Verbs re-point.** `vision_align` and `vision_move` map onto the board primitive:

   | our kwarg | becomes |
   |---|---|
   | `lat=` / `yaw=` / `depth=` axes | the axis bitmask (p1) |
   | `fwd=` fill % | target **angular size** (p2) |
   | `gain=` | speed cap (p3) |
   | `hold=` | hold seconds (p4) |
   | `duration=` | timeout (p5) |
   | `err=` px | deadband, converted px → rad |
   | `fire=` / `fire_t=` | **stays here** — fires on our stable-frame counter |
   | `fallback=` | **stays here** — DSL search orchestration |
   | `lock_on=` / `coast_s` / `ctrl_conf` | **stay here** — selection policy |

   They are then removed from `UNSUPPORTED_VERBS`.
5. **The host loop stays** for the Pixhawk backend, which is unchanged throughout, and as the
   thing we diff against when the board's behaviour surprises us.

## Open on our side

- **FOV numbers.** Need the real horizontal/vertical FOV for the Blue Robotics Low-Light cams
  at the resolution we actually run. A measured value beats a datasheet one — a calibration
  target at a known distance is enough.
- **`target_num`.** We currently track one target; classes are strings on our side and need a
  small interned int on the wire. The mapping must be stable across a model switch.
- **Downward camera.** Our downward path remaps axes (`lat`→strafe, `fwd`→surge,
  `depth`→descent) because the camera is rotated. Cleanest is to send bearings already in the
  **vehicle** frame and let the board stay camera-agnostic — decide before building the uplink.
  See [`downward-camera.md`](downward-camera.md).
- **Range.** We can send a monocular `distance`, but `vis_range` has never been read by the
  control path. Send `0` (unknown) until there is a reason not to.

## Cross-repo rules

The wire contract is co-owned. See [`cross-repo-contract.md`](cross-repo-contract.md) and the
matching `AGENTS.md` in each sibling repo. `test_srot_protocol_drift.py` reads the firmware
headers directly and fails on divergence — when the board adds `movement::Type::VISION`, that
test is what tells us.
