# Bench — everything provable without water

> Pool time is the scarce resource. **Anything provable dry must never consume it.**
> States: `[ ]` open · `[~]` blocked (blocker named) · `[x]` done (number in the row).
> Method and rules: [`README.md`](README.md).

---

## B-1 · Depth-sign check  `[ ]`

**Why it is first.** The board's depth loop **has never run closed** and its sign **was inverted
once**. The SURFACE failsafe routes through the same loop — a leak, a flat thruster pack or a lost
GCS all call `depth::update()`. If the sign is still wrong, the first emergency **drives the
vehicle down**.

**Needs:** the board, powered, **disarmed**. Nothing else. No thrusters, no water.

**Procedure.** `python3 tools/depth_sign_check.py` — the tool already exists (170 lines) and
prints a verdict. It reads `DEPTH_CMD`, which `depth::preview()` computes through a separate
**proportional-only** instance, live while disarmed. Pressurise the Bar30 port with a thumb.

⚠ You cannot do this by lifting the vehicle: a whole metre of air is ~0.12 mbar, about **1.2 mm**
of equivalent depth.

| thumb says | `DEPTH_CMD` | verdict |
|---|---|---|
| deeper than target | **positive** → ascend | correct |
| shallower than target | **negative** → descend | correct |
| demand moves *away* from target | — | ⛔ **still inverted — do not dive** |

**Expected before running:** positive under pressure. **Falsifier:** if `DEPTH_CMD` does not move
at all, the preview path is not wired as the firmware document says and everything above is
unproven.

**Result:** _(not yet run)_ · **Lands in:** `bringup_check --srot`, `measured-bars.md`

---

## B-2 · ESC deadband ladder  `[~]` command half DONE 2026-09-22, physical half blocked: one thruster

**Why.** Blue Robotics publish a **±25 µs deadband** around 1500. On our ±400 µs full scale that
is **±6.25 %** of range producing *exactly zero thrust* — and `VISION_YAW_MIN_PCT = 5.0`, which the
code itself labels *"a hardware spin-up assumption, NOT a measured value"*. **Our stiction floor
may command nothing at all.**

**Procedure.** One thruster, a PWM ladder in 1 µs steps either side of 1500, RPM (or a current
clamp) as the witness. Find where motion actually starts, both directions.

**Expected:** motion starts somewhere near ±25 µs. **Falsifier:** if it starts below 5 % of range,
the floor is fine — and we still replace an assumption with a measurement.

**Result, command domain, 2026-09-22.** ⛔ **The premise was wrong. There is no deadband — there is
a FLOOR, and it is three times higher than our constant.** `MOT_SPIN_MIN = 0.15` lifts every
non-zero demand into `[spin_min, 1]` deliberately (`mixer.cpp:127`), so the smallest output this
vehicle can command is **15 % of full scale**. Measured on the board, nothing attached, armed, a
0.02 demand — the smallest tried — already produced **18.1 %**. `VISION_YAW_MIN_PCT = 5.0` is not
merely unmeasured; it is the wrong model, and no host-side gain crosses a floor.

The whole chain is now modelled and verified to **0.30 %** worst case
(`actuation_model.py`, `tools/mixer_map.py --ladder`), and it confirms the
source-derived table in [`upstream/pr-i`](../upstream/pr-i-spin-min-relay.md) §1.

**Still open:** the *physical* stiction floor — where a real motor starts turning — which needs a
motor. The command-domain number above says nothing about force.

**Lands in:** `actuation_model.py` (done), `motion_vision.py` floor constants (not yet wired —
that is a flight-behaviour change), `measured-bars.md`

---

## B-3 · Thrust vs RPM², on the load cell  `[~]` blocked: thrusters + load cell

**Why.** `k_n_per_rpm2` has **no default on purpose** — neither host nor firmware carries an
absolute N per RPM². `thrust_model.py` has zero callers, asserted by a test. Every force-domain
statement the stack could make is blocked behind this one number.

**Procedure.** Load cell, flight voltage, sweep RPM, fit thrust against RPM² **through the
origin**. Record voltage alongside: the vendor's own reverse ratio drifts 0.787 → 0.754 across
12–20 V.

**Expected:** a straight line through the origin (bollard condition). **Falsifier:** if the fit
does not pass through the origin within its residual, `T = k·n²` is the wrong form for this
thruster and the model is *replaced*, not tuned.

⚠ **Write it down as a bollard number.** K_T falls with advance ratio, so this `k` **flatters us at
cruise** — the four-quadrant literature is explicit that a manufacturer's bollard figure can only
scale `c_T`, never supply its J-dependence.

**Result:** _(not yet run)_ · **Lands in:** `estimator/thrust_model.py`, `measured-bars.md`

---

## B-4 · Dead-thruster signature  `[~]` blocked on **one line of firmware**, not on hardware

**Why.** Published AUV thruster fault detection uses **voltage, current and speed**. Our RPM
channel read **0 in 958/958 frames with nothing attached** — an RPM-based detector reports a
*healthy zero* for a missing thruster.

⚠ **2026-09-22: the premise moved.** Presence is already measured — it just never reaches us.
The Pico decodes a `TL_ST_PRESENT` bit and four per-motor counters (`e` eRPM frames, `d` extended
telemetry, `c` CRC failures, `n` nothing-came-back), prints them on its own USB console
(`src/pico/main.cpp:405`), and publishes `status[]` to the ESP32 — where `sendEscStatus`
(`mav_stream.cpp:190`) packs **literal zeros** into `cnt[4]` and drops the status entirely.

Read on the bench with nothing attached: `pres=0`, `n≈998` per 500 ms window. The firmware's own
comment says the healthy-with-no-motor result is `pres=1 rpm=0`, so the two states are already
distinguishable **at the source**.

**This ranks H-4 (our own per-thruster current-sense board, weeks 3–6) below a one-line PR.**
That ask already exists and is already ranked top-capability:
[`upstream/pr-f-esc-presence-on-the-wire`](../upstream/pr-f-esc-presence-on-the-wire.md).

**Still needs a motor:** separating *stalled* from *spinning*. Presence separates *missing* from
*present*, which is the class that ends runs.

**Result:** presence measured on the Pico console 2026-09-22; unreachable over MAVLink until PR F ·
**Lands in:** a thruster-health detector, `measured-bars.md`

---

## B-5 · Hull mass properties → BG  `[~]` blocked: a scale, or materials assigned in CAD

**Why.** Roll is **unactuated**, so the entire "roll is passively stable" claim rests on
hydrostatics — and **BG cannot be computed**: Onshape reports *no material assigned to any part*.
The literature's caution is that small AUVs have a small stabilising moment (a published box-hull
design quotes 7.39 cm metacentric height as what makes it work).

**Procedure.** Weigh each part (or assign materials), compute CoG, CoB, BG. Then an incline test on
the assembled hull to check the computed number.

**Expected:** a positive BG with margin. **Falsifier:** if the restoring moment is small enough
that thruster wash rolls the hull past usable camera tilt, "leave roll passive" stops being a
design choice and becomes a ballast problem.

**Result:** _(not yet run)_ · **Lands in:** `vehicle-spec.md`, `measured-bars.md`

---

## B-6 · Allan variance, 12 h static IMU  `[ ]`

**Why it matters more than it looks.** The filter's `Q` is an **unmeasured nominal diagonal** and
the position block is **exactly zero**. Worse, the NEES/NIS literature states that **NIS is
chi-squared distributed only for an already-tuned filter** — so our χ²-99 % gate, the
five-rejection lockout break and the ×4 inflation are all built on a statistic that is not valid
until this run happens.

**Procedure.** `python3 tools/allan_variance.py --log --port /dev/ttyACM0 --hours 12`, board flat
and undisturbed, then `--analyse` on the `.npz` it writes. The tool is built and its maths is
**truth-tested** against signals with closed-form answers (`test_allan_variance.py`, 6 tests): the
τ = 1 s intercept — the number that becomes `sigma_gyro` — recovers a known white-noise level to
**0.11 %**.

⚠ It **refuses** to report a bias instability when the curve has not turned (the minimum sitting at
the longest τ means the log was too short, not that a floor was found) — which is the standard way
this measurement is published wrongly. Verified: pure white noise comes back UNRESOLVED.

⚠ The vehicle must be **still** for the whole run. A door slam is a rate-random-walk artefact no
analysis can remove.

**Expected:** numbers in the consumer-MEMS band. **Falsifier:** if measured noise is far from the
nominal `Q`, every gate threshold in the estimator is re-derived from it.

**Result:** _(not yet run)_ · **Lands in:** `inekf.py` `Q`/`P₀`, `measured-bars.md`

---

## B-7 · Retrodiction cost on the Pi  `[ ]`

**Why.** `retro.py` is verified correct to **1e-9** against an on-time filter and costs 0.8 ms per
sample 60 ms late — and it **ships off** purely because its Pi cost was never measured.

**Procedure.** Full graph running on the Pi, `retrodict:=true`, measure per-sample cost at our real
lateness distribution.

**Expected:** well under one IMU period. **Falsifier:** if it exceeds that, it stays off and the
row closes with a reason rather than a hope.

**Result:** _(not yet run)_ · **Lands in:** the `retrodict` default, `measured-bars.md`

---

## B-8 · Caustics replay through the KLT-Hessian gate  `[ ]`

**Why.** Every published health mechanism except an independent estimator detects the **absence**
of signal. Our measured caustics failure is the opposite — a **confident wrong** signal: tracking
is healthy, it is tracking the wrong thing. Super Odometry 2.0 gates visual health on the
**Hessian of KLT tracking**, which is the matrix Shi-Tomasi already computes in our flow node.

**Procedure.** Replay the recorded caustics clip; log the smaller eigenvalue alongside the existing
point-survival fraction; compare their separation on the known-bad interval.

**Expected:** the eigenvalue separates caustic-tracking from good tracking. **Falsifier:** if it
does not, only an independent estimator can catch this class — which is itself the answer, and it
promotes the second-opinion velocity model.

**Result:** _(not yet run)_ · **Lands in:** `flow_node.py` health channel, `measured-bars.md`

---

## B-9 · Model pipeline, timed end to end  `[ ]`

**Why.** How long it takes to produce a model for a new class is **recorded nowhere** — not in our
repo, and **not in the 2026 literature**, where "human-minutes saved per class" is an empty cell
across every auto-labelling paper. If we measure it, the number does not currently exist anywhere.

**Procedure.** One genuinely new prop class, from raw footage to a running `.hef`. Stopwatch every
stage; separate **human attention** from wall-clock.

**Expected:** hours of human attention today. **Falsifier:** if it is already under an hour, the
pipeline is not the bottleneck and the work shrinks to versioning the datasets.

**Result:** _(not yet run)_ · **Lands in:** the pipeline design, `measured-bars.md`

---

## B-10 · Mission pre-flight checker  `[ ]`

**Why.** We **cannot test in water before we fly**. COLA2 compiles its mission language to a Petri
net with reachability-based safety proofs; BehaVerify model-checks 100× faster than its
predecessor. We prove nothing today.

**Procedure.** Static pass over a mission: does every path reach `disarm`? Does it call a verb this
backend refuses (`can()` knows)? Does every task carry a deadline? Does the declared worst case fit
the budget? Run it against a good mission and a deliberately broken one.

**Expected:** flags the broken one, passes the good one. **Falsifier:** if it cannot separate them,
it is a linter with opinions, not a proof of anything.

**Result:** _(not yet run)_ · **Lands in:** `tools/`, and the pre-dive checklist

---

## B-11 · Refusal sweep across every mission  `[x]` done 2026-09-22

**Why.** J04: a verb the backend refuses, discovered underwater. The FSM layer carried a capability
flag with **zero callers** and every plan ended one state after DIVE.

**Result.** The FSM layer was **retired** (3,550 lines). The oracle now lives in the DSL —
`mongla.can(verb)` reads `srot_fc.UNSUPPORTED_VERBS` **at call time**, so there is no second copy
to drift. `test_run_plan.py` executes the path; **injecting the J04 shape fails 4 of 12 tests**,
including the exact "attempted a refused verb" case. Guard verified to bite, then restored.

**Lands in:** `BUGS.md` J04 (closed), `sota/SOTA-GAPS.md` G-01 (closed)

---

## B-12 · Link budget at 1 Mbaud  `[~]` blocked: firmware #17

**Why.** Measured **21.0 %** of the wire used at 115 200 baud (11 520 B/s). Every uplink we want —
velocity, position, vision targets — is gated on headroom.

**Procedure.** After the baud change, re-run `tools/board_snapshot.py` and compare bytes/s,
messages/s and rejected frames against the 2026-09-22 reading.

**Expected:** the same traffic at ~2.4 % of the wire. **Falsifier:** if rejected frames rise, the
link is not clean at that rate and the uplink plan changes.

**Result:** _(not yet run)_ · **Lands in:** `measured-bars.md`, the uplink defaults

---

## B-13 · Does the loop rate matter?  `[ ]`

**Why.** **No AUV-specific study was found** tying loop rate to performance — every AUV control
paper the sweep reached ran at **10–100 Hz**, and ArduSub's 400 Hz is an *in-air* inheritance. Our
500 Hz board may be buying headroom rather than stability, and that should be said accurately.

**Procedure.** Log `DEPTH_ERR` and attitude error at full rate, then decimate the **controller** to
250 / 100 / 50 Hz and compare tracking error and effort.

**Expected:** unknown — that is the point. **Falsifier:** if nothing degrades until 50 Hz, the
claim "500 Hz is why it works" is retired and replaced with what 500 Hz actually buys.

⚠ Needs water, or at least a hull in motion, to mean anything; listed here because the logging half
is bench work.

**Result:** _(not yet run)_ · **Lands in:** `measured-bars.md`, and the honest version of our own
headline

---

## B-14 · The mixer matrix, from the board  `[x]` done 2026-09-22

**Why it was thought impossible.** `srot_fc.py:2114` states this backend has no output readback,
and the ledger rates the allocation gap BLOCKED on exactly that: the achieved wrench is computed
on the board every tick and, as far as MAVLink is concerned, goes nowhere.

It goes to the **other USB device**. The board presents two and we had only ever connected one:
`/dev/ttyUSB0` is the ESP32 (CH340, MAVLink); `/dev/ttyACM0` is the RP2350's ESC console, which
prints every motor's commanded and output value at 2 Hz. That is enough to measure the mixer with
**nothing attached to any wire**, because a mixer is a linear map from axis demands to motor
commands and no part of it involves force.

**Procedure.** `python3 tools/mixer_map.py --arm`. Drives one axis at a time, reads all eight
motors, and differences the +demand and −demand runs — which cancels whatever the attitude loop
was holding (M6 sat at +190 and M7 at −811 in every horizontal row).

**Result.**

```
        M1     M2     M3     M4  |  M5   M6   M7   M8
fwd  +1.00  +1.00  -1.00  -1.00  |   0    0    0    0
lat  -1.00  +1.00  -1.00  +1.00  |   0    0    0    0
 up      0      0      0      0  | +1   +1   +1   +1
yaw  -1.00  +1.00  +1.00  -1.00  |   0    0    0    0
```

Exactly ±1, reproducing the firmware's `M[8][6]` (`mixer.cpp:26`) negated by `FRAME_REVERSE = 1`.
**The ledger's 41 % surge/sway overstatement (1/cos 45° = 1.414) was read off the firmware; it is
now measured on the board.**

⚠ **Two traps, each of which returned a confident wrong answer first.**
- **Bidirectional DShot is two half-bands, not a signed range.** 48–1047 and 1048–2047 each count
  upward from their own floor, so there are two zeros and no midpoint. Decoded as a deviation from
  1048, a 0.02 demand reads −819 on M1 when the truth is −181, and the heave row grows a 3 %
  imbalance that does not exist. `test_dshot_decode.py` pins this, including a guard that fails if
  the decoder regresses to the midpoint reading.
- **The mixer does not run disarmed.** The disarmed sweep reads all zeros, and the tool **refuses**
  rather than reporting a matrix of them — a mixer that does nothing and a board that is not
  listening look identical on the wire.

**Falsifier that passed:** predicting all eleven ladder points from the firmware source agreed with
the board to **0.30 %** worst case. If the chain had been mis-transcribed, that error would be
structural, not sub-count.

**Lands in:** `actuation_model.py`, [`upstream/pr-i`](../upstream/pr-i-spin-min-relay.md) addendum,
`measured-bars.md`

---

## B-15 · Hailo throughput, the three models we fly  `[x]` done 2026-09-22

**Procedure.** `hailortcli benchmark` on each `.hef`, Pi 5 + Hailo-8, nothing else running.

| model | classes | NMS output | FPS (hw_only) | latency (hw) |
|---|---|---|---|---|
| `gate_rescue_repair` | 3 | 6 012 | 97.92 | 8.340 ms |
| `sauvc_sim` | 11 | 22 044 | 98.36 | 8.315 ms |
| `bin_fire_blood` | 2 | 4 008 | 98.45 | 8.308 ms |
| `yolov11n` (stock) | 80 | 160 320 | 92.53 | 7.794 ms |
| `yolov11s` (stock) | 80 | 160 320 | 42.65 | 19.727 ms |

All six inputs are 640×640×3, so these are comparable. **Class count buys throughput** (80 → 3
classes is +5.8 %, via output bandwidth); **backbone size dominates** (n → s halves it). Our three
models sitting within 0.5 % of each other is not a pipeline ceiling — `yolov11s` proves the
silicon scales with model cost.

⚠ **What this does NOT establish.** `hw_only` deliberately excludes host pre- and
post-processing, and the 80.9 Hz standalone / 53.9 Hz through-the-graph figures in `CLAUDE.md`
were measured separately, under conditions and on a model this run did not record. Lining the
three up as "98 → 80.9 → 53.9, so the graph costs 45 %" would be comparing three measurements of
different things — the same inconsistency that got the `yolov8s` number discarded two paragraphs
up. **The 45 % is not claimed.** Attributing the gap needs one run that measures all three stages
on one model in one session; until then, 98 Hz is the silicon figure and nothing more.

⛔ **Discarded, not explained away:** `yolov8s.hef` reported **466 FPS** at the same 640×640 input
and the same 80-class NMS output as `yolov11s` at 42.65. That is not physically consistent; the
artifact is suspect and the number is not used anywhere.

**Lands in:** `measured-bars.md`, `hailo-vision.md` · raw: `data/hailo_bench_20260922.txt`

---

## B-16 · Mode interlocks and group saturation, at the actuator  `[x]` done 2026-09-22

Two claims the host depends on, neither ever verified *at the motors*. Both need **both USB
cables** — MAVLink to command, the Pico console to see what the mixer did — which is a rare state.

### The barometer interlock HOLDS

With the Bar30 **not connected**, requesting each mode and reading the mode back from `HEARTBEAT`
(`set_mode` is best-effort on this wire — a silent refusal looks exactly like success):

| requested | observed | verdict |
|---|---|---|
| STABILIZE (0) | 0 | accepted |
| **DEPTH_HOLD (2)** | **0** | **refused** |
| **AUTO (23)** | **0** | **refused** |

This matters more than it reads. `SROT_MOVE` enters AUTO, so the refusal means **every move verb
is denied** — the vehicle arms and will not move. And `depth::update()` is what the SURFACE
failsafe routes through, so an AUTO accepted without a barometer would close an emergency loop on
a measurement that does not exist. **The interlock is now verified rather than believed.**

### The two thruster groups are independent — their fix works

`mixer.cpp:44-66` scales down uniformly **within each group**, and the comment records why: it
"used to compute ONE maxabs across all eight thrusters", so a saturating forward command scaled
down roll and pitch as well — "a hard forward burst silently cost a third of the vehicle's
roll/pitch authority, in the manoeuvre where you want it most."

```
                              M1    M2    M3    M4    M5    M6    M7    M8
fwd 1.0 alone               1047  2046  1047  1046  1048  1238   237  1048
fwd 1.0 + lat 0.5            705  2046  1047   705  1048  1238   237  1048
up 0.5 alone                1048  1048  1048  1048  1750  1761  1735   746
fwd 1.0 + lat 0.5 + up 0.5   705  2046  1047   705  1750  1761  1735   746
```

Adding heave to a **saturating** horizontal command left the horizontals **bit-identical**, and the
verticals identical to heave alone. **Decoupled, measured.**

### The saturation arithmetic, predicted before the run

Written into the tool's docstring before it was executed: `fwd 1.0` and `lat 0.5` shape to 1.000
and 0.3875, giving M2 = −1.3875 as the largest, so the group scales by 1/1.3875 = 0.7207 and the
four normalised outputs are 0.4414, 1.0, 1.0, 0.4414 — about **657, 999, 999, 657** counts.

Measured: **657, 998, 999, 657.**

`up 0.5 alone` also re-confirms the heave path at a second level: ~700 counts measured against
700.3 predicted **without** `PILOT_EXPO`.

**Falsifier that passed:** had the groups been coupled, or had the scale-down been anything but
uniform-within-group, these four numbers would have been structurally wrong, not slightly off.

**Lands in:** `measured-bars.md`, `actuation_model.py` (axis paths) · raw: `data/verify.json`
