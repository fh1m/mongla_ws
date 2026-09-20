# Vision verb results, mid-hold fire & live feedback

> **Read this before writing a vision mission.** It documents what
> `mongla.vision.align(...)` and `mongla.vision.move(...)` *give back* (so you can
> build robust hybrid vision+control missions), how to fire a payload **mid-hold**
> while still correcting, and how to watch a verb converge live during practice.
>
> API source of truth: [`vision_dsl.py`](../../../src/mongla_planner/mongla_planner/vision_dsl.py)
> (`VisionResult`), [`vision_verbs.py`](../../../src/mongla_control/mongla_control/vision_verbs.py),
> [`motion_vision.py`](../../../src/mongla_control/mongla_control/motion_vision.py)
> (`align_loop` / `move_loop`), [`Move.action`](../../../src/mongla_interfaces/action/Move.action).
> Companion: [`client-and-dsl-api.md`](client-and-dsl-api.md) · [`command-reference.md`](command-reference.md)
> · [`mission-cookbook.md`](mission-cookbook.md) · [`detected-paradigm.md`](detected-paradigm.md).

---

## 1. Why this exists — the hybrid paradigm

Fully-autonomous vision runs are not trustworthy: water inertia makes the hull
wobble, so `vision.align` often **TIMES OUT just-off-centre** instead of converging.
A mission that only checks "did it align?" has no way to recover — it either blindly
proceeds (and misses the gate/hole) or gives up.

The fix is **vision + tested open-loop control**: read **where and how** the verb
finished, then run a known-good `move_*` / `set_depth` to finish the job.

```python
res = mongla.vision.align('gate', yaw=0, lat=0, duration=15)
if res:                          # ALIGNED — the happy path
    mongla.move_forward(3, gain=40)
elif res.saw_target:             # saw it, couldn't fully centre -> nudge + commit
    if res.x_px > 30:   mongla.move_right(1, gain=30)   # target ended RIGHT  -> chase right
    elif res.x_px < -30: mongla.move_left(1, gain=30)   # target ended LEFT   -> chase left
    mongla.move_forward(3, gain=40)                     # drive through anyway
else:                            # NEVER saw the gate -> search, don't drive blind
    while not mongla.detected('gate'):                  # the real search idiom
        mongla.move_forward(0.6, gain=35)               #   (see detected-paradigm.md)
```

> **"Search" is mission-authored — there is no `mongla.search_pattern()` verb.** Use a
> `while not mongla.detected(...)` loop or a `fallback=` search fn (see
> [`detected-paradigm.md`](detected-paradigm.md) and §3 below). The examples here show the
> real idiom.

`res` is a [`VisionResult`](#2-visionresult--the-finish-state). Three questions it
answers, every time, on success **and** failure:

| Question | Field |
|---|---|
| Did it succeed? | `bool(res)` / `res.ok` |
| Did it ever even see the target? | `res.saw_target` |
| Where did the target end up? | `res.x_px`, `res.y_px` |
| How did it end / how close? | `res.status`, `res.last_err_px`, `res.fill`, `res.elapsed_s` |

---

## 2. `VisionResult` — the finish-state

```python
res = mongla.vision.align('hole', yaw=0, lat=0, depth=0)
# res is a VisionResult:
res.ok           # bool  — True only when ALIGNED (move: reached fill / passed through)
res.status       # str   — 'ALIGNED'|'LOST'|'TIMEOUT'|'NO_CAMERA'|'ABORTED'|'FAILED'
res.reason       # str   — same string as status (alias)
res.code         # int   — raw outcome code (ALIGNED=0,LOST=1,TIMEOUT=2,NO_CAMERA=3,ABORTED=4)
res.x_px         # float — SIGNED px of target from frame CENTRE at the LAST SEEN frame
res.y_px         # float —   (+x = target ended RIGHT, +y = target ended BELOW); NaN if never seen
res.saw_target   # bool  — was the target detected at least once during the verb
res.last_err_px  # float — worst residual px from the GOAL (centre+offset) at exit
res.fill         # float — bbox fill fraction at exit [0..1] (vision.move; 0 for align)
res.elapsed_s    # float — how long the verb ran
bool(res)        # == res.ok  ->  `if mongla.vision.align(...):` still works unchanged
```

### 2.1 `x_px` / `y_px` — WHERE the target finished (sign-critical)

These are the **target's position relative to frame centre** on the **last frame the
verb saw it**, in pixels. They are the **raw observable** — independent of any `lat=`/
`yaw=`/`depth=` offset you asked for.

| Value | Meaning | To centre it, the hull must… |
|---|---|---|
| `x_px > 0` | target ended **RIGHT** of centre | strafe **right** → `mongla.move_right(...)` |
| `x_px < 0` | target ended **LEFT** of centre  | strafe **left**  → `mongla.move_left(...)` |
| `y_px > 0` | target ended **BELOW** centre    | go **deeper** → `set_depth(more negative)` |
| `y_px < 0` | target ended **ABOVE** centre    | go **shallower** → `set_depth(less negative)` |
| `NaN`      | target was **never seen**         | `saw_target` is `False` — search, don't move |

> **The sign matches `align`'s own correction direction.** `align` drives the hull
> *toward* the target (target-right → strafe right). So your recovery `move_*` is
> simply "keep going the way align was going" — same sign, no inversion. Getting this
> backwards drives the hull **away** from the target; see [§6 pitfalls](#6-what-not-to-do).

### 2.2 `saw_target` — the "never detected" guard (the important one)

`saw_target` is `True` iff the target produced at least one detection during the verb.
**Always branch on it before reading `x_px`/`y_px`** — they are `NaN` when the target
was never seen, and a `NaN` comparison is silently `False`:

```python
# CORRECT — distinguish "ended off-centre" from "never saw it"
if res:                                          # aligned
    mongla.move_forward(3)
elif res.saw_target:                             # saw it, off-centre -> chase the residual
    mongla.move_right(1) if res.x_px > 0 else mongla.move_left(1)
else:                                            # never saw it -> search (mission-authored)
    while not mongla.detected('gate'):
        mongla.move_forward(0.6, gain=35)

# WRONG — a never-seen target has x_px = NaN; `NaN < -30` is False, so this
# silently falls through to "proceed" on a target the AUV never even detected.
if res.x_px < -30: mongla.move_left(1)
```

`saw_target == False` almost always means **wrong model/classes loaded** or the target
isn't in view. The verb's log line and `res.status` say which. (See
[`detected-paradigm.md`](detected-paradigm.md) for acquiring before aligning.)

### 2.3 `last_err_px` vs `x_px` — two different distances

- **`x_px`/`y_px`** = where the target is **relative to frame centre** (raw, offset-blind).
- **`last_err_px`** = worst residual from the **goal you asked for** (centre **+** your
  `lat=`/`yaw=`/`depth=` offset). This is "how far from *aligned*".

If you align dead-centre (`lat=0, yaw=0`) they track each other. If you align to an
offset (`lat=80` — keep the target 80 px right), `x_px≈80` at success while
`last_err_px≈0`. Use `x_px` to reason about the **scene**; use `last_err_px` to reason
about **alignment quality**.

### 2.4 `fill` (move only)

`vision.move` returns the bbox fill fraction at exit in `res.fill` ([0..1]). A
`move(fwd=80)` that reaches the wall returns `res.ok == True, res.fill ≈ 0.80`. A move
that TIMES OUT returns `res.ok == False` with `res.fill` = how full it actually got —
so you can tell "almost there (0.72)" from "barely moved (0.15)".

### 2.5 Result is populated on SUCCESS too

The end-state is filled in on **every** terminal outcome, not just failure. After a
successful gate align you still know exactly where the gate sat (`x_px`) — useful to
seed the **next** task ("gate was 40 px right, so the course bends right").

---

## 3. Worked recovery patterns

### 3.1 Gate — align, else nudge toward where it ended, then commit
```python
def creep_forward(mongla):            # mission-authored search/fallback fn
    mongla.move_forward(0.6, gain=35) #   one short creep, then return so align retries

def pass_gate(mongla):
    res = mongla.vision.align('gate', yaw=0, lat=0, err=40, duration=15,
                              fallback=creep_forward)
    if not res and res.saw_target:
        # Off-centre at timeout: chase the residual the SAME direction align tried.
        if res.x_px > 40:   mongla.move_right(1.0, gain=30)
        elif res.x_px < -40: mongla.move_left(1.0, gain=30)
    if res or res.saw_target:         # aligned, or at least saw it -> commit through
        mongla.set_depth(-1.4)         # drop below the gate bar first
        mongla.move_forward(4, gain=45)
    else:                             # never saw the gate -> keep searching forward
        while not mongla.detected('gate'):
            mongla.move_forward(0.6, gain=35)
```

### 3.2 Torpedo — note the miss for the NEXT shot
```python
res = mongla.vision.align('hole', yaw=0, lat=0, depth=0, err=12,
                          gain=25, yaw_gain=10, hold=4, fire=1, fire_t=1,
                          brake=False)            # fire mid-hold (see §4)
if not res and res.saw_target:
    log(f'shot likely off: hole ended ({res.x_px:+.0f},{res.y_px:+.0f})px '
        f'(err={res.last_err_px:.0f}px) — adjust standoff/gain next run')
```

### 3.3 Bin — drop only when actually centred, else reposition by sign
```python
res = mongla.vision.align('bin', camera='downward', lat=0, depth=0,
                          err=25, duration=12)
if res:
    mongla.fire(3)                              # dropper
elif res.saw_target and abs(res.x_px) < 60:
    mongla.move_right(0.5) if res.x_px > 0 else mongla.move_left(0.5)
    if mongla.vision.align('bin', camera='downward', lat=0, depth=0, err=25):
        mongla.fire(3)
# else: never saw the bin -> hold drop, continue search
```

> **Downward camera note:** `x_px` is still "left/right in the image". On the downward
> cam that maps to the hull's lateral axis the same way; depth (`y_px`) maps to
> forward/back of the image. Confirm the physical mapping at the pool before trusting
> a downward recovery sign (the forward-cam table above is pool-verified; the downward
> cam is not).

---

## 4. Mid-hold fire — `fire` + `fire_t`

The old "align, **then** `fire()`" pattern misses: the gap between the verb finishing
and the torpedo leaving lets the hull drift off the hole. `fire`/`fire_t` fire the
payload **while `align` is still actively correcting**, so the shot leaves *glued*.

```python
mongla.vision.align('hole', yaw=0, lat=0, depth=0,
                    hold=4,            # active station-keep for 4 s after centring
                    fire=1,            # fire payload channel 1 (1/2=torpedo, 3/4=dropper)
                    fire_t=1,          # 1 s into the hold (0 = at the moment it locks)
                    brake=False)       # no pre-shot lateral nudge
```

- **`fire`** — a channel `int` or a list (`fire=[1, 2]` fires both, one-by-one). 1/2 =
  torpedo, 3/4 = dropper.
- **`fire_t`** — seconds **into the hold window** to fire. `0` = the instant the lock is
  confirmed. Must be `< hold` (else it's clamped to `0` with a loud warning).
- **Gated on alignment, distinct frames, AND a fresh new frame — three conditions, all
  on the same tick:** the shot leaves on the first **stably-aligned** tick at/after
  `fire_t`, where "stable" means `align_stable_frames` **distinct in-band detections**
  (not 20 Hz loop ticks — a re-read frame counts once, so one lucky frame at low FPS
  can't arm it), **and** that tick must be one where a **genuinely new, non-coasted
  detection just landed** (`is_new_frame and not coasted`). This `is_new_frame` gate
  (2026-07-01, D12) replaced the old `age_s ≤ 0.10 s` window: at 3-4 Hz that window was
  *narrower than one frame period*, so a perfectly-aligned hull kept **missing** the
  fire — while a frozen detector re-serving one stale frame could still satisfy an age
  gate. Gating on a **new** frame is fresh by construction at ANY FPS (fixes the miss)
  **and** strictly safer: a frozen detector produces no new frame, so it can't fire on
  a stale box even while `stable` stands held at threshold through a mid-hold freeze. So
  a torpedo **never** fires (a) off-target, (b) on a frozen detector's re-read frame, or
  (c) on a tracker-coasted (Kalman-predicted) box during a `coast_s` gap. Raise real
  detector FPS (`[YOLO] backend=TensorRT`) so new frames — hence fire opportunities —
  are plentiful (still the single biggest lever on fire reliability).
- **`fire_pass=True`** (opt-in) — if the strict in-band fire above never landed, fire
  the payload anyway on a **natural exit** (TIMEOUT or hold-complete), provided the
  target was seen **live within `lost_grace_s`** (never on a never-seen or coasted-only
  target). A guaranteed partial-points shot when full alignment wasn't reached — "fire
  *something* if we saw it." Off by default (strict lock only).
- **`hold_heading=True`** (opt-in) — when yaw is released to the background heading lock
  (the terminal hole-lock drops the yaw axis), widen the lock deadband 1°→3° for the
  hold so the launcher heading holds **steady** instead of micro-correcting sub-degree
  noise (the terminal yaw jitter). Use it on the yaw-dropped fire-lock.
- **Non-blocking:** the fire runs on a background thread so the 20 Hz correction loop
  never stalls (the payload board can sleep ~2 s on a USB reconnect). The hull keeps
  correcting through the shot.
- **Always pair with `brake=False`** on a fire-from-lock — the arrival brake's pre-shot
  nudge would disturb the aim, and there's no benefit braking when you're firing.

**Caveat (pool):** on a payload-board USB reconnect the shot can leave up to ~2 s late.
Use a **small `fire_t` and a generous `hold`** (e.g. `hold=4, fire_t=1`) so a delayed
shot still lands inside the hold window.

> The geometric-lock verb `anchor_align` (XFeat superglue) also fires mid-hold, but it
> lives on the **`lock` branch only** and is not part of this (main) stack. Don't reach
> for it here.

---

## 5. Live feedback — watch convergence during a run

While a vision verb runs, the action streams **live signed pixel error** at ~2.5 Hz on
the goal feedback, so you can watch the hull converge in real time (great for tuning
gains during practice):

```bash
# In another terminal during a mission / a standalone vision_align goal:
ros2 topic echo /mongla/move/_action/feedback
# ...
#   err_x_px: -120.0   err_y_px: 30.0    status_line: 'YAW:1.2  DEPTH:-1.40m  VIS:(-120,+30)px'
#   err_x_px: -60.0    err_y_px: 12.0    status_line: '...  VIS:(-60,+12)px'
#   err_x_px: -42.0    err_y_px: 8.0     status_line: '...  VIS:(-42,+8)px'
```

- `err_x_px` / `err_y_px` carry the same signed target-from-centre px as `res.x_px`/
  `res.y_px`, but **live, every tick** (vs the single end value the result returns).
- They are `NaN` when no vision verb is active (or no fresh detection), and the
  `status_line` shows `VIS:(x,y)px` only while a target is being tracked.
- The detector node *also* prints the always-on operator **bearing** line
  (`[ offset lat=… depth=…px ] '<class>' bearing (live)`) to the console
  regardless of any verb — raw bbox offset for a quick eyeball without echoing
  the topic. It is telemetry, **not** an alignment verdict (the verb's
  `aligned (N/Mpx)` is the verdict).

---

## 6. What NOT to do

- **Don't read `x_px`/`y_px` without checking `saw_target` first.** `NaN` comparisons
  are silently `False`, so a never-seen target slips through an `if res.x_px < -30`
  guard as if it were centred. Branch `saw_target` (or `if not res:` then `res.saw_target`).
- **Don't invert the recovery sign.** `x_px > 0` (target right) → `move_right`. It is the
  *same* direction `align` was driving. Reversing it drives the hull away from the target.
- **Don't treat `x_px` as a distance in metres.** It's pixels at the current standoff —
  a **direction + rough magnitude** to tune at the pool, not a metric offset. Pixel→move
  scaling depends on how close you are.
- **Don't `align(hold=H)` then a separate `fire()` for an accurate shot.** Use
  `fire=`/`fire_t=` so the shot leaves mid-hold; the align-then-fire gap drifts the aim.
- **Don't set `fire_t >= hold`.** It clamps to `0` (fires at lock-start) with a warning —
  usually not what you intended. Budget `hold > fire_t` so the shot is a *deliberate*
  delay into a settled lock.
- **Don't assume a non-`ok` result means "do nothing".** A `TIMEOUT`/`LOST` with
  `saw_target == True` is the *most useful* case — it's exactly when hybrid recovery
  earns its keep.
- **Don't rely on the result to fix a missing detector.** `saw_target == False` every
  run usually means the vision stack isn't running or the wrong model is loaded — the DSL
  aborts loudly if the detector node is absent; check that first.

---

## 7. Field reference (quick)

`VisionResult` (mission-facing): `ok`, `status`/`reason`, `code`, `x_px`, `y_px`,
`saw_target`, `last_err_px`, `fill`, `elapsed_s`; `bool(res) == res.ok`; `repr(res)`
prints a one-line summary.

`Move.Result` (the wire — you normally read the `VisionResult`, not this): `success`,
`message`, `final_value` (=code), `error_value` (=residual px), `end_x_px`, `end_y_px`,
`fill_frac`, `elapsed_s`.

`Move.Feedback` (live, ~2.5 Hz): `phase`, `current_value` (depth), `err_x_px`,
`err_y_px`, `status_line`.

> **NaN survives the wire.** `saw_target` is derived from `x_px` being `NaN`, and that
> `NaN` is shipped across the DDS boundary as a `float32`; the round-trip is pinned by
> `test_move_result_roundtrip.py` so a future interface change can't silently turn
> "never seen" into "ended dead-centre".
