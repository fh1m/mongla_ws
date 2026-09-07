# What we owe the water — the measurement backlog

> **This is not `pool-day.md`.** That file is the *runbook*: how to bring the
> vehicle up, what to check, how to recover. **This file is the debt**: every
> quantity that is shipped, wired, or claimed and that **only water can
> settle**, with the procedure, the pass bar, and what each one unblocks.
>
> An item leaves this file by being **measured**, not by being believed.
> Written 2026-09-07, srot `b0f5404`. Keep it current — an item that quietly
> disappears is how a dry number becomes a competition assumption.

## Why a separate file

Rounds 33–38 shipped a great deal of behaviour and **every headline number is
dry**. That is not a complaint about the work; it is the honest boundary of a
bench. Three things are true at once and they must not be blurred:

- **Measured in air, on this hardware** — real, reproducible, and possibly
  irrelevant underwater (refraction, turbidity, texture, lighting all change).
- **Measured in a simulator** — settles logic, never optics or transport.
- **Shipped and never measured anywhere** — the dangerous class, because a
  default that nobody chose still multiplies every output.

The bars below are written so a pool session produces **numbers**, not
impressions. Where a bar cannot be met the item says what to do instead,
because "it looked fine" is how `recommend()`, CLAHE and the 25 s gap all got
into the record and had to be retracted.

---

## ⛔ PRE-WATER GATES — do these before the hull is wet

These are not measurements. They are conditions under which a run is unsafe or
uninformative, and every one is verified-false today.

| # | gate | state | why it blocks |
|---|---|---|---|
| **G1** | **`LEAK_EN = 0` on the board** | **verified 0** | Both the leak failsafe and the pre-arm refusal are gated on it. As configured a leak **neither blocks arming nor surfaces the hull**. The sensor reads DRY, so enabling it looks safe — that is exactly why it was never noticed. **Set `LEAK_EN = 1` and confirm the pre-arm refusal actually fires.** |
| **G2** | **`FRAME_REVERSE = 1` + `MOT_n_DIRECTION = -1` on M1/M8** | set, **runtime-only, unsaved** | A double inversion nobody has resolved. `CAL_MDIRn` **multiplies** with `MOT_n_DIRECTION`, so a successful MOTOR_DETECT makes `FRAME_REVERSE` wrong. **Read all three back and write them down before arming.** |
| **G3** | **GATE 0 — axis configuration** | **never verified** | Needs physical thrust, one motor at a time, hull restrained. `DO_MOTOR_TEST` (209) is fully implemented in firmware and runnable from `duburi_ws` today. **Until this is done every axis sign is an assumption**, including the ones the flow verification below depends on. |
| **G4** | **GATE 2 — depth loop closed** | Bar30 healthy, loop *running*, never closed in water | It gates every `SROT_MOVE`. The host half is done. |
| **G5** | **`MOT_BAT_V_MAX`** | unset | Without it a timed leg is **pack-state dependent** — the same command travels different distances at 16 V and 13 V (measured: 14.2 % less). |

---

## 1. ⭐ Flow-as-DVL in water — the biggest single unknown

**Status:** verified in **air** to 3.6 % worst-case error over 30 cm on three
axes (`measured-bars.md` §13). Every constant in that path was measured in air.

### 1a. `f_water = 741` — validate, do not re-calibrate

`f_air = 513.94`, `f_water = 741.0`, ratio **1.44**, from the round-25
held-out-validated FOV work (63.8° air / 46.7° water ±0.7°). The literature's
flat-port figure is 25–33 %; ours is 44 %. **The measurement wins** — the
literature only explains why they differ (port thickness and geometry).

- **Do:** a known-length slide **in water** at a known height, `medium:=water`.
- **Bar:** measured distance within **±5 %** of tape. A systematic error near
  **−31 %** means `f_air` is being used (`741/514 = 1.44`); near **+8 %** means
  something defaulted to a 1.33 refractive guess.
- **Do NOT** re-fit `f_water` from this run. It is a **validation**. If it
  disagrees by more than the bar, that is a finding to investigate, not a
  number to overwrite — a single in-water slide is far weaker evidence than
  25 held-out calibration views.

### 1b. `pool_depth_m` — the parameter that multiplies everything

It defaults to **NaN** and the node **refuses to publish velocity without it**
(`flow_node.py:337`), which is the fix for the class of bug that produced an
83 % scale error from an unset height. Height is a clean multiplier on every
velocity.

- **Do:** measure the actual pool depth with a tape. Set it explicitly. Read
  the startup banner back and confirm the printed value.
- **Bar:** the banner prints what you measured, and `HeightFromDivergence`
  agrees with the `pool_depth_m` path to within its **20 %** warn threshold.
  A persistent disagreement means one of the two is wrong — and in air the
  **optics measured height better than the tape did** (0.7025 m vs 0.720 m),
  so do not assume the tape wins.

### 1c. Drift over a real leg — the number the bench cannot produce

Every distance number we have is a **30 cm hand slide with ~±1 cm of
operator precision**, so the operator sits inside our error bar. The quantity
that decides whether `move_forward_dist` closes over a mission leg is **drift
accumulation over metres**, and it is unmeasured.

- **Do:** known-length legs of **2–3 m** at **2–3 known altitudes**, out and
  back, `move_forward_dist` against a tape.
- **Bar:** report as **% of distance travelled**, the same unit as the
  benchmarks — Nortek DVL bottom-track **0.5–1 %**, DVL-aided INS **0.08 %**,
  Ferrera et al. monocular VO in turbid water **0.89–1.88 % ATE**. Under
  **2 %** is competitive; over **5 %** means the flow path is not yet a DVL.
- **Also record:** the symmetric-leg residual (out then back should return to
  origin). Cross-track heading drift dominated in sim; expect the same.

### 1d. Does the pool floor give features at all?

Tile grout lines are a strong, repeatable feature set — and a tile lattice is
also the documented **perceptual-ambiguity** case, where LK can lock one tile
off with `status=1`. Our forward-backward rejection at 2 px and the planar
rigid fit both exist for this.

- **Do:** log `flow_quality`, surviving point count, and the fit residual over
  a full leg. **Also** point it at a plain (untextured) section of floor.
- **Bar:** on tile, ≥ **40** surviving points and residual < **0.5 px**. On
  plain floor the node must **refuse** (quality 0) rather than emit a
  confident wrong velocity. **A refusal on featureless floor is a PASS.**

### 1e. `ROT_FRACTION_MAX` — re-derive, do not just move it

Currently **0.80**. The dry evidence is two-sided and must not be read
one-sided (`measured-bars.md` §12): at 0.638 rad/s the gate **refused 533 of
670 intervals** while de-rotation was working (85.5 % of truth recovered), and
at 1.128 rad/s de-rotation made the answer **worse**. So 0.80 is too strict at
the low end and a real ceiling exists at the high end.

- **Do:** collect the residual and the rotation fraction together over a leg
  with deliberate yaw.
- **Bar:** replace the fraction test with a **residual-based** criterion. Do
  **not** simply raise 0.80 — the first row alone would justify that and the
  second row says it would be wrong.

---

## 2. ⭐ Camera↔IMU `td` — the one term with no measurement anywhere

Board-clock stamping is now measured on hardware (**sd 6.402 → 0.528 ms, 12.1×**,
`measured-bars.md` §14). The other three corrections are wired, unit-tested,
and **unmeasured on hardware**:

| term | size | state |
|---|---|---|
| interval midpoint | up to **375 ms** | pure arithmetic, unit-tested |
| half-exposure | **7.85 ms**, moves with light | read live, unmeasured |
| **`td`** | **unknown** | every number is injected-offset recovery |

`td` is **bounded at 150 ms** (`time_offset_max_s`) precisely because we have
no in-water value to sanity-check a large estimate against.

- **Do (can be done DRY, needs only a hand):** oscillate the rig gently about
  the optical axis. It must be a **changing** rate — Li & Mourikis show
  constant velocity is degenerate and the estimator will refuse it.
- **Bar:** `quality ≥ 0.5`, and **successive windows agree** to within a few
  ms. Disagreement means the excitation gate is doing its job and the answer
  is not ready — **that is not a failure to work around.**
- **In water additionally:** exposure rises in darker water, so half-exposure
  grows. Log `exposure_us` through the run; if it moves, the offset moves with
  it and that is the term to watch.
- **Fallback:** if no stable `td` is obtained, ship `estimate_time_offset:=false`
  rather than a value nobody trusts.

---

## 3. Vision — five things shipped on dry evidence

| # | item | shipped state | what water decides | bar |
|---|---|---|---|---|
| **3a** | **Adaptive Kalman R** (confidence-driven) | **ON by default** | measured dry on a person in air (selectivity 0.78× → 5.37×, the shipped filter had been *inverted*); the archive agrees, neither is water | selectivity > 1 on real props, and alignment converges **faster**, not merely differently |
| **3b** | **conf floor 0.10** | shipped | §8 precision says 0.10 is defensible and the original **+8.5-presence justification was not** — presence is blind to false positives by construction | recall **and precision** on real props; a false gate detection steers the hull at a wall |
| **3c** | **`ConfidenceTrend`** | shipped, **wired to nothing** | on `bin`, confidence fell 0.53 **before** the box wandered — a genuine early warning if it holds | does confidence lead the error signal in water? If yes, wire it |
| **3d** | **`vision.ctrl_conf`** | `ConfidenceModel.authority()` implemented, **called by nothing** | it moves thrusters, which is why it is not enabled dry | enable only after 3a and 3b hold |
| **3e** | **`range_gain_floor`** | measured, **still off (1.0)** | the close-in instability lever: loop gain rises ~1/range, so a `kp` stable far-field over-drives close in | does the measured jitter-vs-fill curve cancel? This is the terminal-alignment fix |

---

## 4. Control — closed-loop gains

`kp_lat`, `kp_yaw`, `ki_lat` **need water and the replay rig deliberately does
not pretend otherwise.** Archived footage can prove sign, deadband, freshness
decay, per-axis cap *ratios* and the fire gate — it **cannot** prove
convergence, overshoot, settle time or `kp` magnitude, because the image does
not respond to the command.

Also unmeasured in water: **`PILOT_EXPO = 0.30` is inside our loop.** The board
applies `((1−EXPO)·sp + EXPO·sp³)·PILOT_SPEED` to translation *and* yaw, so
small-signal authority is `GAIN·0.70`, never 1.0, plus a cubic term that makes
one `kp` wrong at both ends — exactly the terminal-alignment regime.

- **Bar:** tune with `GAIN` **read from the board**, not assumed. And remember
  `set_default_gain()` is a **no-op for its own session** (latch at boot); the
  proven in-session path is pulsing `JS_GAIN_INC = 42`.

---

## 5. The return leg — one command, highest consequence

Every 2025 gate model scores **0.0 %** on the 302 labelled back-side images.
Production `gate_rescue_repair` declares `gate / rescue / repair` — **no
back-side class**. `tools/return_check.py` is written and **unrun**; the
weights are on the Pi.

**This does not need water — it needs one command on the Pi**, and it decides
whether a phase-1 scoring task is silently dead. Do it before the session so
the pool time is not spent discovering it.

---

## 6. Session logistics — so the data survives

- **`.tlog` recorder on for every run** (round 27). It captures both
  directions; a reader-only log has no *decisions* in it.
- **`source scripts/pool_session.sh <name>`** in every terminal — pins
  `DUBURI_RUN_DIR` and `ROS_LOG_DIR` to one folder per run.
- **Record `image_raw`** for at least one run per venue. `pool_record.sh`
  **excludes it by default**, and without frames no offline A/B is possible —
  bag playback is wall-clock paced and drops a different frame subset each
  time, so it cannot be an A/B rig.
- **Write down the water**: turbidity, lighting, time of day, and a Laplacian
  sharpness number. Our archive shows the gate clip is **3.7× blurrier** than
  the bin clip at identical brightness, and that was the variable that
  mattered — not the model.
- **One variable per arm.** The CLAHE retraction happened because a tracker
  fix, a clamp fix and a conf drop were stacked into one arm.

---

## Ledger — everything blocked on water, one line each

| # | item | owner section |
|---|---|---|
| 1 | `LEAK_EN = 1` and the pre-arm refusal fires | G1 |
| 2 | `FRAME_REVERSE` / `MOT_n_DIRECTION` read back and recorded | G2 |
| 3 | GATE 0 — per-axis thrust direction via `DO_MOTOR_TEST` | G3 |
| 4 | GATE 2 — depth loop closed in water | G4 |
| 5 | `MOT_BAT_V_MAX` set | G5 |
| 6 | `f_water = 741` validated against tape | 1a |
| 7 | `pool_depth_m` set and cross-checked vs divergence height | 1b |
| 8 | drift over 2–3 m legs, as % of distance | 1c |
| 9 | pool-floor feature yield + **refusal** on plain floor | 1d |
| 10 | `ROT_FRACTION_MAX` re-derived as a residual criterion | 1e |
| 11 | real `td` measured, or `estimate_time_offset:=false` | 2 |
| 12 | half-exposure tracked as light changes | 2 |
| 13 | adaptive Kalman R in water | 3a |
| 14 | conf floor: recall **and** precision on real props | 3b |
| 15 | `ConfidenceTrend` — does confidence lead error? | 3c |
| 16 | `vision.ctrl_conf` enabled only after 3a/3b | 3d |
| 17 | `range_gain_floor` — the close-in instability fix | 3e |
| 18 | `kp_lat` / `kp_yaw` / `ki_lat` with GAIN read from the board | 4 |
| 19 | `PILOT_EXPO` accounted for in terminal alignment | 4 |
| 20 | return leg — **run `return_check.py` BEFORE the session** | 5 |
