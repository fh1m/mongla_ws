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

> **RUN 2026-09-07 — and the finding above is wrong in three places.** See
> `measured-bars.md` §15. Short version: the production model is **not blind**
> (41.7 % at conf 0.10, held out) but is **~26 % at the shipped operating
> point** — a coin flip, which fails on the day rather than loudly. Only the
> older SAUVC-family models score a true 0.0 %. It is **302 images and 72
> labels**, not 302 labelled. And the archived back-side specialist's 88.9 %
> is **memorisation** — its `data.yaml` says `val: train`, so it was scored on
> its own training set and that number must not be deployed on.

**What the pool owes this item:** point the forward camera at the gate **from
the far side** and record a full pass. We have 72 labelled back-side frames
from one session; that is not enough to decide anything, and it is the
scarcest asset in the archive for the one scoring task nobody has measured.

- **Bar:** enough frames from a **second** session to build a held-out split.
  Never a random split — these are consecutive video frames and neighbours are
  near-duplicates, which is exactly how the specialist came to score 98.6 % on
  nothing.

---

## 5b. The commands, verified on the vehicle 2026-09-07

Not written from the launch file — **run on the Pi and the output pasted
below**, because the whole of §17 in `measured-bars.md` is about a
configuration that was correct in a tool and wrong in the launch.

```bash
# The DVL. flow:=true is OFF by default and pool_depth_m has NO default.
ros2 launch duburi_vision vision_pi.launch.py flow:=true pool_depth_m:=1.6
```

Expected banner, and check every field of it:

```
[FLOW ] camera='downward' medium='water' f=685.1px (air 513.9 / water 741.0)
        port=RECTIFIED  pool_depth=1.60m  gyro_gain=(-1.000,-1.000)
```

- **`port=RECTIFIED`** — the flat-port correction is live. `single-f` means the
  calibration did not load, and you are carrying a 1–2.8 % anisotropic scale
  error (§14).
- **`f=685.1px`** — the rectified reference focal length. Seeing **741** means
  `port=single-f`.
- **`pool_depth=1.60m`** — what you measured with a tape. `nan` prints
  `VELOCITY PATH DISABLED` and nothing is published.

Both refusal paths were exercised live and both are worth recognising:

```
[FLOW ] VELOCITY PATH DISABLED -- pool_depth_m was never set.
[FLOW ] REFUSING: no trackable texture (0/19 points survived)
        -- dark or featureless floor, not a tracking fault
[FLOW ] REFUSING: LK lost the anchor (2/12 survived)
```

The middle one is the murky-water message and it is **not a bug report**: it
means the floor gave nothing to track. The last one means tracking started and
degraded. Telling them apart at 2 a.m. is the reason they are separate lines.

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
| 12 | ✅ **CLOSED 2026-09-07** (§23) — auto exposure chose a **200 ms shutter** (66 px of blur at 0.64 rad/s); `webcam.py` now pins a manual shutter with a `f·ω·t` blur cap. Gain measured INERT. Next: drive the cap from the LIVE gyro rather than a static max rate | §23 |
| 13 | adaptive Kalman R in water | 3a |
| 14 | conf floor: recall **and** precision on real props | 3b |
| 15 | `ConfidenceTrend` — does confidence lead error? | 3c |
| 16 | `vision.ctrl_conf` enabled only after 3a/3b | 3d |
| 17 | `range_gain_floor` — the close-in instability fix | 3e |
| 18 | `kp_lat` / `kp_yaw` / `ki_lat` with GAIN read from the board | 4 |
| 19 | `PILOT_EXPO` accounted for in terminal alignment | 4 |
| 20 | return leg — **run `return_check.py` BEFORE the session** | 5 |
| 27 | **ChArUco board** — OpenCV recommends it over a chessboard and 4.6 on the Pi has the full API. Partial views become legal (the board may leave frame) and the 180° pose ambiguity goes. Directly targets the 'cannot hit the pose' failure. See `camera-and-calibration.md` §8 | §8 |
| 28 | **does the Fantech actually autofocus?** Marketing says yes; v4l2 exposes NO focus control, so we could not lock it. If it refocuses, its intrinsics are not constant and every bearing drifts — and it is a candidate cause of the historical fx spread | §6 |
| 21 | ✅ **DONE 2026-09-07 — fx 851.23, HFOV 73.88 air / 53.59 water, RO 5/5 folds, sigma(fx) 0.27 %, WELL CONDITIONED.** Installed as `pi_forward_1280x720.json`, which the launch already names. ⚠ Max ERE 14.47 px is still above AprilCal's 1 px bar, so more views (below centre) would tighten it. ~~calibrate the FANTECH forward camera~~ — never done; it is the camera the vision uplink AIMS with, and it was publishing the downward camera's intrinsics until 2026-09-07 (`measured-bars.md` §17 — LATENT, since the uplink is default-off, so this is 'wrong the moment it is switched on', not 'wrong every mission'). Needs a printed board and 25 views; **does not need water** | §17 |
| 22 | ✅ **DONE 2026-09-07 — 31.94 cm on a 30 cm truth, 106.5 %, through the launch** (`measured-bars.md` §18). The four wrong causes on the way were all instrument faults, not sensor faults. ~~re-verify the 30 cm result **through the launch**~~, not through `flow_console --calibration <path>`. Every §13 number came from a tool that passed the path by hand, and the launch wired the calibration to the wrong camera | §17 |
| 23 | ✅ **DONE 2026-09-07 by the same slide** — the launch path in `medium:=air` produces a correct measurement. ⚠ the RECTIFIED water path still has no measured velocity; that half stays water-only. ~~one dry slide with `medium:=air` through the launch~~ — the regression check on the rectifier refactor. `port=RECTIFIED` is verified to come up and the maths is verified against a pinhole, but the rectified path has produced **no measured velocity at all**; every §13 number predates the refactor. Needs light and one hand slide, **no water, no rig** | §13 |
| 24 | **HIGH-ROTATION de-rotation test at the vehicle's own pivot** — the gains were re-derived (`+1.058/+0.830` about the lens) and then REFUTED by the validation A/B (§21): they over-correct on a slide because the pivot differs. Default is now **0.0 = off**, which is best on translation. §12 measured de-rotation halving the error at 0.638 rad/s, so it must be re-enabled once tested in the regime that needs it. ~~re-derive the de-rotation gains on THIS mount~~ — measured three ways on the vehicle (`measured-bars.md` §19): shipped `(-1,-1)` 109.4 %, **off `(0,0)` 105.0 % and 3x tighter**, flipped `(+1,+1)` 94.6 %. The true gain is inside (−1,+1) and near zero. §12 predicted this and it was never done. `tools/flow_derot_calibrate.py`, tilt-only, one axis per run. **Needs no water** | §19 |
| 25 | ✅ **CLOSED 2026-09-07 — measured and WITHDRAWN** (§22). Holding the anchor across a refusal is worse at every refusal rate and *raises* the refusal count, because the widened window still contains the disturbance and it cascades. Parameter deleted, behaviour reverted, numbers pinned in a test | §22 |
| 26 | the RECTIFIED water path has still produced **no measured velocity** — verified to select and its maths verified against a pinhole, but never measured wet. **Water only** | §13 |

## 5c. THE DRY DVL CHECK NEEDS THREE PROCESSES, NOT ONE — measured 2026-09-07

Found by running it. `vision_pi.launch.py flow:=true` alone gets you a node
that comes up, prints a correct banner, and **refuses every interval**. The
first two reasons it gives are misleading and the third is the real one:

| symptom | actual cause |
|---|---|
| `no trackable texture (0/8 points survived)` | transient — the live scene measured **190 corners** on every frame seconds later. Do not chase texture on a single sample. |
| `no depth yet, so no height above the floor` | no manager, and **no barometer is fitted on this bench anyway**. `flow_launch_check.py` publishes `/duburi/state` with depth 0 so height == the tape measure. The manager's own NaN depth is ignored by the node, so the two coexist. |
| **`no gyro sample for this interval`** | **the real blocker.** The node will not measure without `/duburi/imu_rates`, which only `auv_manager_node` publishes (measured **exactly 50.0 Hz**, sd 1.9 ms). No manager, no DVL — on a dry bench and in a pool alike. |

So the dry check is:

    1  ros2 run duburi_manager start                    # gyro @ 50 Hz
    2  ros2 launch duburi_vision vision_pi.launch.py \
           flow:=true flow_medium:=air pool_depth_m:=<tape m>
    3  python3 tools/flow_launch_check.py --height <tape m> --truth-cm 30

**Do NOT pass `vision:=off`** — that value is coerced to boolean `False` and
kills the composed process that owns BOTH cameras, so flow silently receives
no frames while every node looks healthy. Round 33's launch-type coercion,
met again.

**Verified working, stationary rig, 2026-09-07:** 22 intervals, **0 refused**,
reading **−0.68 cm over 10 s** — correctly near zero, which is the negative
control for the whole chain. A moving measurement is the operator's slide.


