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

## Answers from the board side (`srot-control-board`, 2026-08)

Point-by-point on the four open items above, plus one that is missing from the list. Read
[`auv-architecture-2026.md`](auv-architecture-2026.md) for why the hardware picture behind this
is not the one the rest of this branch assumes.

- **FOV numbers — agreed, and this is the critical path.** Confirming from our side that it is
  genuinely unbuildable without them: we checked the whole repo and there is no HFOV/VFOV
  anywhere, `K` and `D` are published **empty** on every frame, there is no calibration file
  and no checkerboard script. The one focal number that exists, `camera_focal_px: 500.0`, is
  commented as a guess for a different purpose. `LANDING_TARGET` carries radians; pixels cannot
  become radians without a measured FOV, and every downstream stage inherits the error.
  **Measure both cameras at the resolution you actually run (640×480) before anything else in
  this file gets built.** Bench task, not a code task. We are blocked on it too.

- **`target_num` — send the interned int, and treat the mapping as part of the wire contract.**
  The board only ever compares it for equality (did the selected target change?), so any stable
  small int works. The one requirement: the mapping must not be re-derived from a model's class
  list at load time, or a model swap silently renumbers everything mid-season. Freeze it in a
  file next to the FOV config.

- **Downward camera — yes, send bearings in the VEHICLE frame.** Decided, and we would ask for
  this even if you were neutral on it. Your downward path rotates image-Y into surge and carries
  a `surge_sign = -1` mount fact; a wrong sign there is *positive feedback*, i.e. the vehicle
  accelerates away from the target rather than oscillating around it. Keeping the rotation on
  the Jetson means it stays where `vision_thrust_check` already validates it **disarmed**, and
  the board's axis bitmask then means exactly one thing on both cameras. The alternative — the
  board learning a per-camera mount — buys nothing and puts a sign error 500 Hz closer to the
  thrusters.

- **Range — agreed, send `0`.** The board treats `vis_range` as advisory only; standoff closes
  on **angular size**, which needs no range and degrades gracefully. Nothing on our side reads
  range today and we are not planning to add a consumer.

### Missing from the list: `LANDING_TARGET` needs a `coasted` bit and a true gap-age

Your `_authority()` deliberately does not double-decay a Kalman-coasted box — correct, and the
board cannot honour it because **neither fact is on the wire**. Without them the board sees a
coasted target as a fresh one and applies its own staleness decay on top of yours: coasted
targets decay roughly 2× too fast and the coast dies inside ~0.4 s, well inside the ladder above
(0.10 / 0.40 / 0.80 / 1.00 s). The observable symptom is a target that "sticks" on the Jetson
while the board has already given up authority.

Proposal: carry both in `LANDING_TARGET`'s unused `x` / `y` / `z` fields —

| field | carries |
|---|---|
| `x` | `1.0` when the sample is Kalman-coasted, `0.0` when it is a live detection |
| `y` | seconds since the last **live** detection of this track (the true gap age, not the message age) |
| `z` | reserved, send `0.0` |

The board then decays on `max(message_age, y)` and skips its own coast decay when `x > 0.5`,
which is exactly the rule `_authority()` already implements. No new message, no dialect change,
and it decodes on every pymavlink.

**The timeout ladder is contract, and we will mirror it.** `VISION_STALE_MS` sits between the
first two rungs as specified. If either side moves a rung, it moves in both repos in the same
change — that ordering (authority reaches zero *before* loss is declared) is what turns a
dropout into a glide instead of a lurch.

**Fire gating stays on the Jetson — your code makes the argument.** The stable-frame counter is
a three-valued state machine over *frame identity* (`_FRAME_EPS_S`), and a frozen detector still
produces a fresh-looking value stream; the board cannot see frame identity and so cannot
reproduce that guard. One honest consequence, so it does not surprise anyone later: `in_band` is
computed from the control axes today, so once control moves down, the Jetson must recompute
in-band from the bearings it is already sending. That is cheap — it has the deadband in radians
— and it keeps the frozen-detector guard intact, which is the entire reason fire gating stays.

**Build order we commit to, firmware-side**, matching `VISION_API.md` §8 exactly: decode +
`VIS_*` telemetry echo → staleness + failsafe wiring → yaw axis dry → yaw + lateral wet →
standoff → depth (⛔ **behind the depth-loop bench gate**, which has still never passed) → hold.

## Cross-repo rules

The wire contract is co-owned. See [`cross-repo-contract.md`](cross-repo-contract.md) and the
matching `AGENTS.md` in each sibling repo. `test_srot_protocol_drift.py` reads the firmware
headers directly and fails on divergence — when the board adds `movement::Type::VISION`, that
test is what tells us.

> **One caveat on that test, learned the hard way.** Reading *enum values* out of the firmware
> headers works and should continue — `movement::Type` is append-only and the wire code is
> `Type - 1`, so a numeric drift is real drift. Reading *behaviour* out of firmware source text
> does not: the brake assertion grepped our `Type::STOP` case for `abort()` and stayed **green**
> through the entire fix that changed what STOP does. Behaviour is now gated on the declared
> `SROT_FW_BEHAVIOUR_REV` number instead (`FW_BEHAVIOUR_REV_REQUIRED`). Grep for structure;
> version-gate for behaviour.
