# Round 4a — the identification pool day, as a script

> A pool session is the most expensive experiment we run. This page exists so the
> day is a **script**, not an afternoon of improvisation. Everything here has been
> rehearsed on the bench against a known answer.

**Prerequisite reading:** [`pool-day.md`](pool-day.md) for the general operating
rules. This page is only the identification runs.

---

## 0. ⭐ What this day is worth, stated up front

Four things are currently **guesses** that every control claim rests on:

| unknown | today | what it blocks |
|---|---|---|
| `q/I` — drag over inertia | a `Cd` guess, swept 0.15–0.30 | the plant's only unmeasured term |
| net buoyancy | assumed neutral | every force claim; the depth loop's standing effort |
| inertia `I` | a **uniform-body guess** | ⛔ **INDI** — [§12 of `measured-bars`](../measured-bars.md) found INDI destabilises when `I` is 1.7× off |
| `BG` | assumed 10 mm | the passive roll stability the **unactuated roll axis** depends on |

Three of the four are settled without ever arming a thruster (§2).

---

## 1. ⛔ Before anything: the two armed bench checks

**The depth loop has never run closed.** `SROT_MOVE` enters the board's automatic
mode and **every** primitive there closes the depth loop — including a plain
`move_forward`. An in-air move is *not* partial validation: at ~0 m, target and
measurement agree.

```bash
ros2 run mongla_manager bringup_check --srot     # grades and gates; exits non-zero
```

⚠ **An unhealthy barometer refuses `DEPTH_HOLD`/`AUTO`/`PATTERN`** — and since
`SROT_MOVE` enters `AUTO`, that means **every move verb is denied**: the vehicle
arms and simply will not move. `VFR_HUD` is *not* gated on baro health, so a depth
value being present proves nothing.

Also read, and write down, before the water:

```bash
ros2 run mongla_manager connect --watch          # everything the board sends
```

`PILOT_YAW_RATE` · `ATC_ANG_YAW_P` · `ATC_RAT_YAW_*` · `MOT_SPIN_MIN` ·
`MOT_THST_EXPO` · `FRAME_REVERSE` · live `GAIN`. ⛔ **A params reset restores
`config.h` defaults silently, and `DEF_PILOT_YAW_RATE` is 45 against the board's
160** — a 3.556× error on every yaw number. Capture them to
`workbench/data/board_params_<date>.json` the way 2026-09-23 did.

---

## 2. The dry runs — three unknowns, no thrusters turning

### 2.1 ⭐ The float test — net buoyancy and `BG`

**This is the highest value per minute of the whole day.** Sealed hull, no power
needed.

1. Put the hull in the water. Add or remove ballast until it **hovers** —
   neither rising nor sinking.
2. **Record the ballast mass added or removed.** That IS the net buoyancy, in the
   only units that matter.
3. Weigh the hull as floated, on a luggage scale. That is **mass**, and with it
   the displaced volume, and the 8.0–13.6 kg band in
   [`hydrodynamics.py`](../../../src/mongla_control/mongla_control/hydrodynamics.py)
   collapses to a number.
4. **Tilt it ~20° in roll and let go.** Time ten oscillations with a phone.
   `T = 2π√(I/(BG·W))` → **BG**, given `I`.

⚠ **Net buoyancy is a difference between two large numbers**, so no CAD volume
can reach it. This measurement cannot be replaced by arithmetic.

### 2.2 The roll decay — the axis we cannot actuate

Same tilt-and-release, but **log the gyro** rather than using a phone:

```bash
ros2 bag record /mongla/imu /mongla/state -o roll_decay
```

Release from ~20°, let it settle completely. ⭐ This is the only direct evidence
that **passive roll stability is sufficient**, and roll is unactuated by design.

---

## 3. The powered runs — `q/I` per axis

⛔ **Props clear, a human on the kill switch, and a disarm path at all times.**

### 3.1 The free-decay rotation

Per axis (**yaw first** — it is the axis every vision verb uses):

1. `arm`, `set_mode ACRO`. ⚠ **ACRO, not STABILIZE** — STABILIZE holds heading and
   will fight the decay. ACRO has no stick gate and no attitude hold.
2. Spin up to **≥ 2 rad/s** (≈115°/s). Faster is better: quadratic drag is only
   visible when the rate is large.
3. **Cut the command to exactly zero** and let it decay to about a **tenth** of
   the starting rate. Expect tens of seconds.
4. Repeat **three times per axis.**

```bash
ros2 bag record /mongla/imu /mongla/state /mongla/demand -o decay_yaw_1
```

**The analysis is written down and already validated:**

```bash
python3 tools/fit_free_decay.py decay_yaw_1.csv       # time_s, rate_rad_s
```

It reports `q/I`, the starting rate, a rate-dependent half-life and its own R².

⚠ **It refuses a hull still under power** — if the command was not really cut, or
`AUTO` re-engaged, the rate does not decay and a naive fit would return a small
positive number that reads as *very low drag*. It also **refuses a record cut
short**, and it **warns when the damping looks LINEAR rather than quadratic**, in
which case `q/I` is the wrong model and must not be quoted. With four open
tunnels at low rates, linear damping is a live possibility, not an academic one.

⭐ **What this identifies is the RATIO `q/I`, not drag.** That ratio is what the
closed-loop time constant depends on, so it is what the controller needs. Turning
it into a drag coefficient needs `I`, and §2.1 is what supplies it. **Say "q/I"
when quoting it.**

### 3.2 ⭐ The prediction to falsify

From the bench plant at the `Cd` band midpoint, yaw: **`q/I ≈ 0.70 1/rad`**
(`q = 0.198 N·m·s²`, `I = 0.284 kg·m²`).

**If the measurement lands in 0.4–1.2, the plant is good** and every bench result
in [`measured-bars` §11–12](../measured-bars.md) keeps its footing. **If it lands
at 3, the plant is wrong** and Rounds 3a, 5 and 6 must be re-run against the
measured value — which is exactly why they are recorded as bench results and not
as vehicle results.

### 3.3 The coast-down — translational drag

Only if §3.1 went cleanly. `move_forward` at 60 %, cut to zero, and let the
**downward camera's optical flow** witness the deceleration (verified to 1.09 cm
over 30 cm). Same fit, translational axis.

---

## 4. If there is time — the two that would change a decision

| run | what it settles |
|---|---|
| **Loop-rate A/B** | [§11](../measured-bars.md) says disturbance rejection degrades 8 % at 100 Hz and 21 % at 50 Hz **on a model with no sensor noise**. Real noise punishes slow loops harder. Worth one confirmation |
| **Thruster step response** | [ask O](../upstream/pr-o-thruster-step-response.md) properly wants a load cell, but a **rate step in ACRO** gives the lag through the hull's own inertia — a cheap proxy for τ, which the bench says dominates everything by 131× |

---

## 5. What comes back, and where it goes

| measured | lands in |
|---|---|
| net buoyancy, mass, `BG` | `hull_geometry.yaml` `unknown:` — retiring the last three nulls |
| `q/I` per axis | `measured-bars.md`, and the bench plant's `Damping.provenance` stops saying "UNMEASURED" |
| roll decay | the passive-stability claim, or its retraction |

⛔ **Then re-run every bench result.** [`measured-bars` §11–12](../measured-bars.md)
are explicitly labelled statements about a *model*. Any conclusion that moves was
resting on a guessed `Cd`, and must be re-recorded — including, if it comes to
it, the INDI verdict.
