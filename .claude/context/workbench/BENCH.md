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

## B-2 · ESC deadband ladder  `[~]` blocked: one thruster

**Why.** Blue Robotics publish a **±25 µs deadband** around 1500. On our ±400 µs full scale that
is **±6.25 %** of range producing *exactly zero thrust* — and `VISION_YAW_MIN_PCT = 5.0`, which the
code itself labels *"a hardware spin-up assumption, NOT a measured value"*. **Our stiction floor
may command nothing at all.**

**Procedure.** One thruster, a PWM ladder in 1 µs steps either side of 1500, RPM (or a current
clamp) as the witness. Find where motion actually starts, both directions.

**Expected:** motion starts somewhere near ±25 µs. **Falsifier:** if it starts below 5 % of range,
the floor is fine — and we still replace an assumption with a measurement.

**Result:** _(not yet run)_ · **Lands in:** `motion_vision.py` floor constants, `measured-bars.md`

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

## B-4 · Dead-thruster signature on current  `[~]` blocked: current sense (our board, or firmware #4)

**Why.** Published AUV thruster fault detection uses **voltage, current and speed**. Our RPM
channel read **0 in 958/958 frames with nothing attached** — an RPM-based detector reports a
*healthy zero* for a missing thruster.

**Procedure.** Spin one thruster, record current. Unplug it, record again. Stall it, record again.
Three signatures, one plot.

**Expected:** current separates spinning / missing / stalled cleanly. **Falsifier:** if it does
not, the channel is not discriminating and the detector needs another input.

**Result:** _(not yet run)_ · **Lands in:** a thruster-health detector, `measured-bars.md`

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
