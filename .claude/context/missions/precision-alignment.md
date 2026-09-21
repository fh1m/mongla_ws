# Precision terminal alignment — hold steady & don't miss the hole

> **Backend note.** The vehicle is the SROT board (firmware Hengla) + a Raspberry Pi 5 with a
> Hailo-8. `lock_heading`, `move_*_dist`, `arc` and `style_yaw` are **refused** there, `ALT_HOLD`
> is not one of its modes, and the depth-setpoint vision axes are refused. Where this page shows
> an older idiom, the current contract is [`command-reference.md`](command-reference.md) and the
> legacy path is [`legacy-pixhawk-and-sitl.md`](../platform/legacy-pixhawk-and-sitl.md).

> **Status:** BUILT on `main`. Every knob is **opt-in** — an un-tuned run behaves
> exactly like before (`range_gain_floor=1.0`, `ki_lat=0`, `ctrl_conf=0`,
> `lock_on=False`). Turn them on per the pool runbook in §6.

This is the operator guide for the close-in robustness layer added after the
2026-06 torpedo testing. It fixes the two failure modes that cost the shot:

1. **Last-moment misclassification** — just before firing, the detector makes a
   *different* box the control target (a second hole on the board, or a spurious
   box), the hull yaws/strafes to it, and the torpedo misses.
2. **Can't hold a 20 kg hull still on the hole** — the AUV oscillates / overshoots
   the small target at the 0.72 m standoff, and a steady pool current pushes it
   off-centre.

They generalise to every close-in task (torpedo, bins, gate-through). All of this
is YOLO-bbox control on `main` — it is **not** the anchor/XFeat lock (that's a
separate `lock`-branch tool; see [`detection-continuity.md`](../perception/detection-continuity.md)).

---

## 1. Why it happens (one paragraph each)

**Misclassification.** The control loop picks the **largest-area box matching the
class** every tick from raw `/detections` (`VisionState.bbox_error`). It has no
memory and no score gate, so a torpedo board with **two holes** (both class
`hole`) lets "largest wins" flip between openings, and a one-frame spurious box is
steered toward immediately — during the `hold` window that's a nudge right before
the shot.

**Can't hold still.** The law is P-control on *normalized* pixel error, so the
image shift per unit hull motion grows ~`1/range` (~ bbox fill). A `kp` that's
critically damped far away is **over-gained up close** → the heavy hull
oscillates. And the lateral axis (Ch6) is open-loop thrust with no downstream
position hold, so a steady current leaves a standing offset pure-P can't null.

---

## 2. The knobs (what each one does)

| Knob | Where you set it | Default (off) | Fixes | What it does |
|---|---|---|---|---|
| `lock_on=True` | **per-call** on `vision.align(...)` | `False` | misclass | Once the target is acquired, steer to the box **nearest the last-accepted centre** (within a gate), not the largest — a 2nd hole / spurious box can't steal the aim. Resets to largest-area acquisition after a real loss. |
| `vision.ctrl_conf` | **ROS param** (deck) | `0.0` | misclass | Control-side **minimum detection score** to accept a box as the target. Distinct from the detector's global `conf`: gates only what the *control loop* steers on. |
| `vision.range_gain_floor` | **ROS param** (deck) | `1.0` | overshoot | Scales **lat/depth** kp **down** as the bbox fills the frame (close). `1.0` = off; `~0.3` = gentle close-in. Applies to `align` and `move`'s `maintain`. |
| `vision.ki_lat` | **ROS param** (deck) | `0.0` | current drift | **Lateral** integral gain; cancels the steady-current offset, accumulated **only during the hold**, clamped, frozen on saturation, reset on loss. **Lateral only** by design. |
| `settle=<px>` | **per-call** on `vision.align(...)` | `None` (off) | **align ends off-target** (the "move centres better than align" report) | **Settle gate.** `align` exits once *position* is in-band for `align_stable_frames` **distinct detections** — so a hull strafing **through** centre at speed can declare ALIGNED mid-pass and coast out on inertia (and the returned px is measured *before* the arrival brake, so it reads clean while the hull ends dirty). `move`'s `maintain` looks near-perfect because it never exits on lateral — it corrects continuously and is genuinely settled when it stops. `settle>0` ports that: a tick only counts toward the exit if the worst error is in-band **AND** barely moving (`|Δerr| ≤ settle`), so `align` ends **settled** on target. Keyed on **error velocity**, not command magnitude, so a steady current (which holds a non-zero command at a perfect lock) does **not** block it — that steady-state offset is `ki_lat`'s job. **Per-call, not a deck param, and NOT for the terminal fire-lock:** the mid-hold `fire` rides the same stable-frame counter, so a `settle` below the bbox jitter (~5px) can **suppress the shot**. Use it on a **coarse** exit-and-move-on align (e.g. the board centre, so `lock_heading` captures a clean heading); the fire-lock wants `lock_on` + `hold` + `ki_lat` instead. |
| `err=<px>` | **per-call** on `vision.align(...)` | `40` | tightness | Pixel deadband. A **small positive** value is the tight knob (`err=8`). **`err=0` ≠ zero tolerance** — it means "use the default / `vision.err_px` param" (rosidl `0==unset` is load-bearing for live-tuning; an explicit 0 and an omitted field are indistinguishable on the wire). The effective deadband is floored at `MIN_ALIGN_ERR_PX` (≈5px bbox jitter) so an over-tight `err` can't perpetually TIMEOUT, and is **printed at align start** + stated in the success line (`aligned (N/Mpx)`) so it's never a surprise. |
| `vision.coast_s` | **ROS param** (deck) | `0.8` | detection flicker | **Gap-bridging coast.** When the `hole` detection drops for a fraction of a second, the lock loses the axis it was steering on and the hull drifts off-aim — the torpedo misses. With `coast_s>0`, the loop keeps steering on the tracker's **coasted (Kalman-predicted) box of the locked id** for up to `coast_s` after the real detection drops, at **decaying authority**, so the bbox never "disappears" for a brief flicker. **ON by default at 0.8 s** — the operator states it was run in water; no measurement of it is recorded in this repo, so treat the value as operator-confirmed, not as a logged result. Set `0` for the byte-identical raw-`/detections` control arm — a live detection always overrides, and the coast is conf-exempt **only for the locked id** (see [`BUGS.md`](../BUGS.md) D10). Pair with `lock_on=True` so the id you coast is the right hole. **Ladder:** `coast_s` (~0.8) **must** be `< vision.lost_grace_s` (1.0) and `<` the tracker `max_predict`/buffer in wall-time. |
| `depth_step=<m>` | **per-call** on `vision.align(...)` | `None` (→0.02) | **depth z-wobble** | **Per-update depth-setpoint resolution.** The depth axis nudges an ArduSub **ALT_HOLD setpoint** (not raw RC). It moves that setpoint by **≤ `depth_step` m each 5 Hz update** and **freezes it inside the deadband**, so ArduSub's depth PID settles between steps instead of chasing a setpoint that jitters with the bbox-y (the up/down bob when the `depth` axis is on). `0.02` = slow/fine, `0.10` = coarse/faster; max slew = `depth_step × 5 Hz`. The **sole** depth-rate knob — depth has no `%` gain like lat/yaw. To drop depth entirely, omit the axis. See [`BUGS.md`](../BUGS.md) D12. |
| `fire_pass=True` | **per-call** on `vision.align(...)` | `False` | **no points if we can't fully align** | **Guaranteed end-of-command shot.** If the strict in-band mid-hold fire never landed (couldn't hold a fresh lock in time), fire the payload anyway on a **natural exit** (TIMEOUT / hold-complete) — provided the target was seen **live within `lost_grace_s`** (never on a never-seen or coasted-only target). "Fire *something* if we saw the hole." Off = strict lock only. |
| `hold_heading=True` | **per-call** on `vision.align(...)` | `False` | **terminal yaw jitter** | **Fire-window quiet mode.** At the hole you **drop the yaw axis** so Ch4 is owned by the background `heading_lock`; on the 20 kg hull the lock limit-cycles against the lateral-strafe yaw moment, so the launcher wobbles a few degrees left/right. `hold_heading=True` widens the lock deadband (`1°→3°`, `LOCK_HOLD_DEADBAND_DEG`) **for the duration of the call** so the lock holds steady and only corrects real drift; restored on exit. At a ~0.4 m standoff a few degrees of hull yaw is a small linear error and `lat` still centres the shot. Use it on the yaw-dropped fire-lock. |

**Reading the logs (don't confuse these two lines):**
- The detector's always-on `[ offset lat=… depth=…px ] '<class>' bearing (live)` is **raw bbox offset telemetry** — it fires whenever the class is seen, verb running or not. It is *not* an alignment verdict.
- The verb's `vision_align: aligned (N/Mpx)` is the **outcome**: residual `N` within the effective deadband `M`.

**Why those axis choices** (don't "fix" them):
- The integral is **lateral-only**. Yaw is a Ch4 *rate* (ArduSub's rate loop
  already integrates → an I-term double-integrates/winds up); depth is ALT_HOLD
  (its own integral). Only open-loop Ch6 lateral has a true steady-state offset.
- The range-gain softening **excludes yaw** — near-field yaw has a separate
  stiction *floor* that deliberately *adds* authority; and at the hole we hand
  yaw to `heading_lock` entirely (see §4).

Code-tunable constants (rarely touched; edit `motion_vision.py` if a target
legitimately moves faster than the gate or the ramp window is wrong):
`VISION_RANGE_GAIN_FILL_LO=0.25`, `VISION_RANGE_GAIN_FILL_HI=0.60`,
`VISION_I_LAT_MAX=15.0`, `VISION_LOCK_GATE_NORM=0.30`.

---

## 3. How to enable it in a mission

`lock_on` is a **per-call** argument (a structural choice — which phase needs the
lock); the gain/conf/integral knobs are **deck ROS params** (tuning values you
dial in water). Keep them separate: the mission says *what to do*, the params say
*how hard*.

```python
# UNIFIED STANDOFF SHOT: forward-standoff + lat/depth + hold + mid-hold fire in
# ONE verb. fwd= adds the forward range-hold axis: align drives the hull to the
# standoff fill and HOLDS it while centering lat/depth -- no separate move()/align
# seam where the hull drifts. lat+depth only (yaw -> heading_lock, see §4),
# continuity lock so a 2nd hole can't steal the aim, fire MID-HOLD.
mongla.set_classes('hole', node='/mongla_detector_forward')
res = mongla.vision.align(
    'hole', camera='forward',
    lat=0, depth=0,                       # NO yaw -> heading_lock holds Ch4
    fwd=35, fwd_mode='height',            # drive to + HOLD the firing standoff
    err=15, gain=12, duration=25,
    lock_on=True,                         # continuity lock (misclass fix)
    hold=4, fire=1, fire_t=1.5,           # station-keep + mid-hold torpedo
    brake=False)                          # never brake on a fire-from-lock
```

The forward term is **one-sided** (drives forward while the bbox is smaller than
the standoff, neutral at/past it — **never reverses**, so no reverse-kick and no
ramming the board), and the **mid-hold fire is gated on reaching the standoff too**
(the shot won't leave while still far). **Fire from a standoff, not point-blank:**
RoboSub awards bonus points for firing further from the board (far 0.3 m / farther
0.46 m), and a large+stable bbox at standoff holds far steadier than point-blank —
tune `fwd` smaller to park further back. This is what fixed the "drives forward
then drifts back, can't close on the hole" pool failure: the old terminal
`align('hole')` had **no forward axis**, so the noisy fill-based `move()` was the
only thing closing distance and it reverse-kicked out on the first fill-touch.

> **The fire only leaves on a LIVE, FRESH sighting of the hole.** Three independent
> gates protect the shot, and all must hold on the same tick: (1) **alignment** —
> every active axis in-band; (2) **distinct frames** — `align_stable_frames` counts
> real detections, not 20 Hz loop ticks, so a single frame re-read at low FPS can't
> arm it; (3) **a fresh new frame** — the fire tick must be one where a genuinely new,
> non-coasted detection just landed (`is_new_frame and not coasted`). This replaced the
> old `age_s ≤ VISION_FRESH_FULL_S` (0.10 s) window (2026-07-01, D12): at 3-4 Hz that
> window was narrower than one frame period, so an aligned hull kept *missing* the fire.
> Gating on a **new** frame is fresh by construction at any FPS (fixes the miss) and is
> strictly safer — a frozen detector produces no new frame, so it can't fire on a stale
> box even while `stable` stands held at threshold through a mid-hold freeze. So a
> torpedo **never** fires on a tracker-coasted (Kalman-predicted) box during a `coast_s`
> gap, nor on a frozen detector's stale cached frame. Coast exists to *hold the lock*,
> not to *take the shot*. If the hole detection is dropping at the standoff, the shot
> waits for a fresh box (within the hold window) rather than firing blind — or set
> `fire_pass=True` for a guaranteed partial-points shot at command end if the strict
> lock never lands. Raising real detector FPS (`[YOLO] backend=TensorRT`) is still the
> single biggest lever on fire reliability.

Tuning is live from the deck and applies on the **next** goal — no mission edit,
no restart:

```bash
ros2 param set /mongla_manager vision.range_gain_floor 0.35   # soften close-in gain
ros2 param set /mongla_manager vision.ctrl_conf        0.55   # reject low-score boxes
ros2 param set /mongla_manager vision.ki_lat           0.4    # null steady current
```

`settle` is the exception — it's a **per-call** `vision.align(settle=<px>)` kwarg, not
a deck param (it also gates the mid-hold fire, so a global value could silently
suppress the torpedo shot). Put it on a coarse exit-and-move-on align in mission code:

```python
mongla.vision.align('torpedo', yaw=0, lat=0, depth=0, settle=8, ...)  # exit settled
```

You can also branch on the rich result (see [`vision-results.md`](vision-results.md)):

```python
if not res and res.saw_target and res.x_px > 30:
    mongla.move_right(1)        # ended off to the right -> nudge, then retry/fire
```

---

## 4. The phased pattern (state-machine framing)

Compose the verbs into three phases — this is how the fixes work together:

> ⛔ **On srot this exact pattern does not run.** `lock_heading()` /
> `unlock_heading()` are refused before dispatch, and the `depth` axis of
> `align` is refused too (it needs a streamed depth setpoint the board does not
> take). The srot shape of the same idea: drop the lock calls entirely — the
> board holds heading itself at 500 Hz in STABILIZE, which is what the lock was
> emulating — and drive `lat` / `yaw` / `fwd` only, adding depth with a separate
> `set_depth` once the depth loop is water-verified. The pattern below is
> pixhawk/sim, and is kept because the reasoning about phases, standoff and the
> settle gate carries over unchanged.

```
COARSE   align('torpedo', yaw=0, lat=0, depth=0)        # all axes: square up + null heading
   |     lock_heading()                                 # hand heading to the background lock
APPROACH move('blood', fwd=.., mode='height', brake=False)  # COAST into range (no reverse-kick exit)
   |
TERMINAL align('hole', lat=0, depth=0, lock_on=True,    # lat+depth only -> Ch4 owned by lock
                fwd=35, fwd_mode='height',              # forward-standoff in the SAME verb
                hold_heading=True,                      # widen lock deadband -> steady launcher heading
                depth_step=0.05,                        # slow, stepped depth -> no z-wobble (optional)
                hold=4, fire=1, fire_t=1.5, brake=False)  # steady, no yaw wobble, fire mid-hold
   |     unlock_heading()
```

The APPROACH `move('blood')` is now a **coarse** "get into hole-detection range"
step (`brake=False` so it coasts in without the reverse-kick exit); the TERMINAL
`align('hole', fwd=..)` then **closes the last bit to the standoff and holds it in
one verb** — there is no longer a verb seam where the hull drifts forward/back
between "stopped approaching" and "started the hole lock."

**Why omit `yaw` at the hole.** An `align` call that does **not** include the
`yaw` axis **never writes Ch4** — the verb takes the `release_yaw` path and
writes lateral via `send_rc_translation`, gated purely on the *yaw axis being
absent* (not on lock-state, since 2026-06-29). So if a `heading_lock` is active
it holds heading on its steady BNO P-loop; if not, Ch4 falls to the heartbeat /
ArduSub — either way the verb stays off the yaw channel instead of fighting it.
**Null the heading during COARSE, then `lock_heading()` before the terminal
phase** so the lock captures the right heading and the hull holds steady on BNO
while lat/depth correct.

> **"The yaw jitters while I align — should I release the heading lock?" No.**
> Releasing it hands yaw to ArduSub's aluminum-hull compass — the *worst* jitter
> source. The wobble was the lock's old hard min-PWM floor limit-cycling against
> the lateral-strafe yaw moment (a relay on a rate channel); fixed 2026-06-29 by
> **tapering** the floor (`heading_lock`, mirrors the `motion_yaw` `ab2014f`
> fix — see [`BUGS.md`](../BUGS.md) D7). Keep the BNO lock; it is
> the right heading authority up close.

**The lock must be ACTIVE for a steady hold:** `lock_heading()` activates
immediately only when the vehicle is **armed** (mid-mission, after
`set_depth`/`align`/`move`, it is). If you script it from a disarmed state,
activation is *deferred* to the first armed command. The reference mission is
[`missions/task_torpedo.py`](../../../src/mongla_planner/mongla_planner/missions/task_torpedo.py).

---

## 5. What NOT to do

- **Don't enable `ki_lat` before damping the loop.** An integral on an
  under-damped (oscillating) loop makes it *worse*. Order: tune
  `range_gain_floor` until the close-in oscillation stops, *then* raise `ki_lat`.
- **Don't keep `yaw` in the terminal `align`** if you want heading held by the
  lock — including the yaw axis *suspends* the heading lock and re-introduces the
  Ch4 vision-yaw wobble. Pick one owner of Ch4.
- **Don't set `range_gain_floor=0`.** `0` is the "unset" sentinel (→ falls back to
  the param/`1.0`); a true 0 would kill lat/depth authority up close anyway.
  Use `~0.3` for gentle, `1.0` for off.
- **Don't use `lock_on` for far-field acquisition / search.** It's for *after*
  you've acquired the target; during search you want largest-area + the
  mission's `fallback`. Turn it on only for the terminal lock.
- **Don't rely on `ctrl_conf` to fix two *real* holes** — both are
  high-confidence, so a conf floor won't separate them. That's exactly what
  `lock_on` (nearest-to-last) is for. Conf floors fix *low-score flickers*.
- **Don't fire on `align`-then-`fire`** for a tight target — use the mid-hold
  `fire=`/`fire_t=` (the gap between a finished align and a separate fire drifts
  the hull off the hole). See [`vision-results.md`](vision-results.md) §4.

---

## 6. Pool runbook (cheapest rung first)

1. **Misclassification — try the free fix first.** Raise the **detector** global
   conf and see if the wrong box stops appearing:
   ```bash
   ros2 param set /mongla_detector_forward conf 0.6     # or mongla.set_conf(0.6) in a mission
   ```
   If that fixes it, you're done. If it *persists*, the bad box is
   high-confidence (a genuine second hole) → enable the **control-side** floor and
   the continuity lock:
   ```bash
   ros2 param set /mongla_manager vision.ctrl_conf 0.55
   ```
   and run the terminal align with `lock_on=True` (mission already does).
2. **Overshoot / can't hold still — damp first.** Lower the close-in gain until
   the hull stops oscillating on the hole:
   ```bash
   ros2 param set /mongla_manager vision.range_gain_floor 0.35   # try 0.5 -> 0.3
   ```
3. **Steady drift under current — integral last.** Only after step 2 is stable:
   ```bash
   ros2 param set /mongla_manager vision.ki_lat 0.4             # raise slowly
   ```
4. **Terminal phase check.** With `lock_heading` engaged and a yaw-less terminal
   `align`, confirm Ch4 is steady (no vision-yaw wobble) and the hull holds within
   `err` for the hold while the torpedo leaves mid-hold.

All four params are read fresh on every new goal, so tune between attempts without
restarting anything.

---

## 7. Other tasks

- **Bins (downward cam):** same pattern — `align('fire', lat=0, depth=0,
  lock_on=True, hold=…, fire=3, brake=False)` to lock a dropper on the bin
  marker; raise `range_gain_floor` for the close hold. (`lock_on` works on any
  camera.) **Always `brake=False` on a fire-from-hold** (see §5).
- **Gate-through / slalom:** `range_gain_floor` also softens `move`'s `maintain`
  strafe, so the lateral offset hold is gentler as the target fills the frame.

See also: [`vision-results.md`](vision-results.md) (branch on where/how a verb
ended), [`fsm-vision-missions.md`](fsm-vision-missions.md) (search patterns +
mission design), [`command-reference.md`](command-reference.md) (full verb table).
