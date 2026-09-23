# Control, against the world

> Dive 1 of four. Method and rubric: [`README.md`](README.md). Our side is read out of the tree
> at `file:line`, never out of a document. Their side carries a URL and a number.

---

## 1. What we do today

### 1.1 The one-paragraph shape

Two actuation backends behind one HAL (`src/mongla_control/mongla_control/fc/base.py`). On
**ArduSub/Pixhawk** the host runs *every* outer loop in Python over `RC_CHANNELS_OVERRIDE`. On
**srot** the host sends *intent* (`SROT_MOVE`, cmd 31000) and the board runs the loops at 500 Hz;
the only streamed host loop left is the vision loop over `MANUAL_CONTROL`.

**Every controller in the package is P, P-with-floor, or PID on a scalar error.** There is no
state-space, no LQR, no MPC, no QP, no pseudo-inverse, no feedforward plant model, no added mass,
no drag model, and no acceleration or jerk limit anywhere in the stack.

### 1.2 Every loop that exists

| loop | file:line | rate | law |
|---|---|---|---|
| vision **lateral** | `motion_vision.py:1230-1245` | detection rate (floor 50 Hz srot / 20 Hz ArduSub, `motion_rates.py:15-32`) | **PI** on normalised pixel error, `p_lat = ctrl·kp_lat·rgain`, `lat_i` clamped ±15 % |
| vision **yaw** | `motion_vision.py:1247-1274` | same | **P + tapered stiction floor**, deadband, no I, no D |
| vision **depth** | `motion_vision.py:1288-1296, 1392-1426` | setpoint stepped at 5 Hz | **rate-limited P integrator into ArduSub's own PID**; refused on srot |
| vision **surge** (downward) | `motion_vision.py:1276-1287` | same | two-sided P reusing `kp_lat` |
| **standoff** (forward fill) | `motion_vision.py:1307-1361` | same | **one-sided** P on fill deficit, never reverses |
| `_YawPID` | `motion_yaw.py:134-189` | 10 Hz | **full PID**, dt-normalised. Kp 1.2 %/deg, Ki 0.03, Kd 0.5; integral reset on sign flip and inside tolerance |
| `HeadingLock` | `heading_lock.py:311-375` | 50 Hz | **pure P**, deadband 1.0°, tapered floor 5 %→0 over 1–6°. Ch4 only |
| `hold_depth` | `motion_depth.py:79-253` | 5 Hz | **no host PID** — a setpoint streamer with a 2.5 s ramp, hull-tracking clamp, 0.30 m brake zone |
| `thrust_loop` | `motion_writers.py:161-204` | 20 Hz | **open loop**; heading drift logged, not corrected |
| `drive_*_dist` | `motion_forward.py:180-285` | 20 Hz | **bang-bang** + reverse-kick brake; refuses without a position source |
| `style_roll` depth hold | `mongla.py:625-800` | 20 Hz | cos-modulated P inside ACRO, hard surface abort at −0.15 m |

Output conversion is one line: `percent_to_pwm` (`pixhawk.py:763-780`), `1500 + pct/100·400`
clamped to 1100–1900 — **the ±100 % limit is enforced at the wire, not in any controller.**

### 1.3 Allocation — the only place the mixer is reasoned about

`src/mongla_control/mongla_control/allocation.py`, 227 lines.

- `MIXER` (`:46-55`) is an 8×6 matrix of **exactly 0 or ±1**, a hand-mirror of the firmware's
  `mixer.cpp` for `SUB_FRAME_VECTORED_6DOF`. Block-diagonal: motors 1–4 carry yaw/forward/lateral,
  motors 5–8 carry roll/pitch/throttle.
- `allocate()` (`:90-117`) reproduces the board's own law: per-group **uniform scale-down**,
  `s = 1/max|u|` when `max|u| > 1`, computed separately for the horizontal and vertical groups.
- `headroom()` (`:120-140`), `largest_axis_within_budget()` (`:143-173`) — a **1-D feasibility
  bound**, solved per motor row, not an optimisation.
- `prioritise()` (`:193-227`) — a **greedy lexicographic fit**. Horizontal order
  (yaw, lateral, forward), vertical order (roll, pitch, throttle). Each axis takes
  `min(|want|, room)` against what is already granted.

Consumed from exactly two places, both in the vision path: `motion_vision.py:629-645`
(`_mixer_saturated` → lateral anti-windup) and `:648-692` (`_srot_drive` → priority fit before the
frame leaves). **Nothing on the ArduSub RC-override path consults it at all.**

⛔ Two facts the file states about itself: the board's scale-down **is never reported on any
message** (`:3-15`), so the host recomputes it forward; and the ±1 entries are a **demand mix, not
geometry** — used as a force sum they overestimate surge/sway by √2 = 41 %
(`estimator/thrust_model.py:45-51`).

### 1.4 The vehicle model we have, and the one we do not

| term | state |
|---|---|
| mixer | present, ±1, mirrored from firmware and asserted by `test_allocation.py:46` against `mixer.cpp` |
| thrust vs RPM | `thrust_model.py:57-107`, `T = k·n²` with `REVERSE_EFFICIENCY = 0.77` — and **`k` has no default on purpose**; nothing in the host or the firmware carries an absolute N-per-RPM². **Zero callers, asserted by a test.** |
| drag | **none.** The only drag reasoning is a comment arguing `T ∝ n²` against quadratic drag ⇒ steady speed ≈ linear in demand (`command_velocity.py:26-29`) |
| added mass | **none, anywhere.** Water inertia is handled empirically as reverse kicks (`REVERSE_KICK_PCT = 25`, `REVERSE_KICK_SEC = 0.20`) |
| battery compensation | **none on the host** — the firmware mixer scales by PM2 voltage; host compensation would double-count (`ROADMAP` L2) |
| the board's pilot shaping | **known and unmodelled**: `demand = ((1−EXPO)·u + EXPO·u³)·SPEED`, `EXPO = 0.30` (`srot_protocol.py:528-541`) — a **30 % small-signal droop exactly in the terminal-alignment regime**, plus a cubic that makes one `kp` wrong at both ends |
| learned velocity model | `command_velocity.py` — `v_ss = g·u + b` per axis by **RLS** (forgetting 0.995) + 1 s lag, gated on 100 excited samples and RMS ≤ 0.10 m/s. It feeds **localization**, and nothing in `mongla_control` reads it |

### 1.5 What the board owns vs what we own (srot)

| axis | host | board |
|---|---|---|
| roll / pitch | nothing — `MANUAL_CONTROL` has no such field | 500 Hz stabiliser |
| heading | nothing during vision (`release_yaw` → yaw 0); discrete turns are `MOVE_TURN` intent | 500 Hz hold + turn state machine |
| depth | **nothing** — `up = 0.0` always; no `set_target_depth`; depth-setpoint aligns refused | `DEPTH_HOLD` latch or `MOVE_DIVE` — ⛔ **never run closed** |
| surge / sway | the vision P loop, host-prioritised, streamed as `MANUAL_CONTROL` | mixer + the unreported group scale-down |
| timed legs, braking | send intent, relay ACKs | ramp, cruise, brake (the host brake was removed at fw rev 2) |

Refused before dispatch (`srot_fc.py:2823-2827`): `lock_heading`, `move_forward_dist`,
`move_back_dist`, `move_lateral_dist`, `arc`, `style_yaw`.

### 1.6 The setpoint shaping that does exist

No Ruckig (measured and rejected), no jerk limit, no online trajectory generation. What exists:
`smoothstep` / `smootherstep` (`motion_easing.py:11-29` — quintic chosen because ArduSub
differentiates the setpoint, so a C¹ corner appears as a rate spike), `trapezoid_ramp` (`:32-51`,
a thrust-scale envelope, not a velocity profile), the yaw sweep (`motion_yaw.py:336`), the depth
ramp (`motion_depth.py:198-212`), and the 5 Hz vision depth step (max slew 0.1 m/s).

### 1.7 Honest labels already in the code

`VISION_YAW_MIN_PCT = 5.0` — *"a hardware spin-up assumption, NOT a measured value"*
(`motion_vision.py:104-109`). `VISION_YAW_FLOOR_FILL`, `VISION_YAW_APPROACH_BAND_PX`, the four
brake constants, `MIN_ALIGN_ERR_PX`, `FWD_BAND`, `LOCK_HOLD_DEADBAND_DEG` — all labelled
**pool-tunable**, none measured. `heading_lock.py:96-99` states the pure-P **steady-state droop**
under a sustained disturbance as a known, accepted error. `STYLE_ROLL_DEPTH_KP = 150.0` and its
three siblings cite no measurement at all.

Shipped **off** by default: `ki_lat = 0.0` (lateral integral), `range_gain_floor = 1.0`
(range-adaptive gain scheduling), `settle_px = 0.0`.

---

## 2. What the best work does

> **Coverage note, stated first because it matters.** This section is complete for **allocation,
> thruster modelling and actuator fault tolerance**. The sweeps for *control laws* (INDI, MPC,
> sliding mode, learning-based) and for *system identification without a basin* were killed by a
> session limit before reporting, and nothing was written to disk. §6 lists exactly what is owed.
> Nothing below is filled in from memory.

### 2.1 The theorem that indicts our mixer

[Johansen & Fossen, *Control allocation — A survey*, Automatica 49(5):1087–1103,
2013](https://torarnj.folk.ntnu.no/ca_survey_final.pdf), citing Durham 1993:

> *"no single generalized inverse (i.e. weight matrix W) can yield exact allocation whenever
> possible using simple saturation."*

That is a **proof** that clip-or-scale after a fixed mix cannot be made exact by any choice of
mixer weights. Our uniform per-group scale-down is strictly **weaker** than the thing that theorem
rules out: it is not a generalised inverse plus saturation, it is a generalised inverse plus a
direction-preserving-per-group projection, applied in **demand space rather than force space**.

The method family, with what each costs:

| method | what it needs | cost per solve | what it buys |
|---|---|---|---|
| weighted pseudo-inverse, `C = W⁻¹Bᵀ(BW⁻¹Bᵀ)⁻¹` | geometric `B`, weights | **one p×m mat-vec — C is constant, computed offline** | least-squares optimal before limits |
| damped / SVD-truncated inverse, `C_ε = W⁻¹Bᵀ(BW⁻¹Bᵀ + εI)⁻¹` | + ε or δ | same | survives rank loss — i.e. **a dead thruster** |
| redistributed pseudo-inverse | B, u limits | 1–3 reduced solves; **≤2⁵ = 32 active-set patterns can be precomputed at p = 5** | exact in many saturating cases; no optimality guarantee |
| daisy chaining | B, priority order | ≤ #groups solves | passes the **residual** down the chain |
| direct allocation (Durham) | B, U, AMS facets | facet search / LP | **preserves the demand's direction** and provably reaches the attainable-moment-set boundary |
| WLS active set (Härkegård) | B, U, weights, `u_p` | iterations ≈ linear in p; exact in finite steps | optimal, absorbs `u_min = u_max = 0` for a dead thruster |
| explicit mp-QP | offline QP over fixed B, U | tree lookup | QP quality at lookup cost — **but B and U must be time-invariant, so it cannot host fault reconfiguration** |

Two numbers that decide the embedded question:
[Bodson & Frost](https://my.ece.utah.edu/~bodson/pdf/Constrained%20Quadratic%20Programming%20Techniques%20for%20Control%20Allocation.pdf)
measured that active-set iteration count grows **linearly in the number of effectors** and that
interior-point only wins **above ≈15 controls** — at our p = 5, active set is the right family and
IP is not worth it. And
[Bodson 2002](https://my.eng.utah.edu/~bodson/code/Evaluation%20of%20optimization%20methods.pdf):
*"constrained optimization may be performed with computational requirements that fall within an
order of magnitude of those of simpler methods. The performance gains … are found to be small on
the average, but sometimes significant."*

⚠ Two warnings that apply directly to this hull: iteration count **cannot be guaranteed**, so an
embedded implementation must accept sub-optimality at a cap; and *"anti-cycling procedures are
indeed needed since symmetric effectors may easily lead to degeneracies"* — a symmetric
4-tunnel + 1-axial layout is exactly the degenerate case.

### 2.2 Marine practice — and the one place we already agree with it

From the same survey, §4.2:

- **Yaw-first priority is published DP convention**, not an invention: *"Surge, sway and yaw
  control, usually with a priority on the yaw axis since loss of heading will usually imply loss
  of position under heavy wind conditions."* Our `HORIZONTAL_PRIORITY = (yaw, lateral, forward)`
  matches the field. **But DP expresses it as a weight inside a QP; we express it as a sequential
  greedy fit that never passes the residual on.**
- A **fixed tunnel thruster is the easy case**: 1-DOF, a constant linear column of `B`. There is no
  modelling excuse for not doing least squares on this geometry.
- **Fault tolerance is an allocation design driver**: *"Thrusters may be disabled and enabled
  dynamically in order to guarantee fault tolerance."* The worst-case single-point failure in the
  industrial requirement is loss of **half** of thrust capacity.
- For underwater vehicles specifically: *"Commonly used methods include pseudo-inverses,
  redistributed pseudo-inverses or simple optimization formulations."*

### 2.3 The anti-windup architecture we cannot implement yet

[Johansen et al., *Anti-wind-up designs for dynamic positioning of marine vehicles with control
allocation*](https://www.sciencedirect.com/science/article/pii/S1474667016319012) maps the
actuator constraint set **up** to the motion-controller level so the allocator solves an
unconstrained problem and the controller handles the constraint. The generic cascade rule is the
same everywhere: feed back the **achieved** `τ = B·Proj_U(u)`, not the demanded `τ_c`.

The survey states the quantity plainly: *"the allocated generalized force τ = BProj_U(u) may be
different from the required/commanded force τ_c"* — and the allocator knows `Δτ` for free.

⛔ **Our firmware computes that Δτ every tick and throws it away.** The per-group divide is never
reported on any message (`allocation.py:3-15`, firmware PR #20). **No anti-windup design in this
family is implementable on our vehicle today, whichever one we pick.**

### 2.4 Thruster modelling — one of our assumptions survives, one does not

[Blue Robotics T200 published performance](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/):

| voltage | forward | reverse | rev/fwd | RPM at full throttle |
|---|---|---|---|---|
| 12 V | 3.71 kgf | 2.92 kgf | **0.787** | 3075 |
| 16 V | 5.25 kgf | 4.10 kgf | **0.781** | 3600 |
| 20 V | 6.70 kgf | 5.05 kgf | **0.754** | — |

✅ Our `REVERSE_EFFICIENCY = 0.77` sits **within 1–3 % of the vendor's own ratio across 12–20 V**.
It is one of the few constants in the stack that survives contact with a datasheet — though it is
voltage-dependent and degrades as voltage rises.

⛔ **The vendor data is bollard-only**, by the definition of bollard (*"zero advance speed… the
only flow over the propeller is that induced by its own rotation"*). A `k` fitted to that
spreadsheet is **a `k` at J = 0 by construction**, and over-predicts thrust at any transit speed —
which is exactly the warning `thrust_model.py:16-32` already carries about itself.

The right model for a purely thruster-propelled vehicle is four-quadrant
([Häusler, Saccon, Hauser, Pascoal & Aguiar, IFAC/SMP 2015](https://pages.up.pt/~up519521/publications/SMP15.pdf)):
`T = ½ρ c_T(β)(v_a² + v_p²)πR²` with `v_p = 0.7Rω` and `β = π − atan2(v_a, v_p)` — because for a
thruster-propelled hull the sign changes in `n` make the advance ratio `J = v_a/(nd)` pass through
infinity, and the first-quadrant model *"is adequate for … vehicles designed to keep a minimum
speed and manoeuvre using control surfaces. This is in striking contrast to … vehicles that are
purely thruster propelled."* Their worked case also shows the manufacturer's bollard spec can only
serve as a **multiplicative correction** on `c_T` — it cannot supply the J-dependence.

**The ESC deadband is bigger than our trim corrections.**
[Blue Robotics](https://bluerobotics.com/learn/controlling-basic-esc-with-the-arduino-serial-monitor/):
*"a deadband of +/- 25 microseconds centered around 1500"*. On our ±400 µs full scale that is
**±6.25 % of range producing exactly zero thrust** — and our vision yaw floor is 5.0 %, i.e. inside
the deadband. That 5.0 % is already labelled *"a hardware spin-up assumption, NOT a measured
value"*; this is the first external number that says what it should be.

### 2.5 Tunnel thrusters lose almost all authority in transit

Two US patents, full text, quoting in-house AUV experiments:
[US 6,286,447 B1](https://patents.google.com/patent/US6286447B1/en) — *"as the forward velocity of
the vehicle was increased to a speed on the order of 3 knots, the effective side force… decreased
to as low as 10 percent of the side force measured at zero forward vehicle velocity."*
[US 6,164,230 A](https://patents.google.com/patent/US6164230A/en) gives the mechanism: the thruster
jet obstructs the hull boundary layer, and the resulting suction **counteracts the force on the
blades**.

⚠ Provenance is weak — neither patent cites academic literature for the figure, and the primary
academic sources (Palmer, Hearn & Stevenson, IEEE OCEANS; *Ocean Systems Engineering* on drift
angle) were behind 403s. But the mechanism is the **same advance-ratio effect as §2.4, showing up
on the lateral axis**.

**Consequence for us, as a conditional:** if 3 kn / 10 % is even order-of-magnitude right for a
⌀84 mm tunnel in a 702 mm hull, then a lateral tunnel's column in `B` **is not a constant — it
collapses with surge speed.** A ±1 mixer encodes the bollard case and silently over-promises
lateral authority during every transit. **That is a stronger argument against the fixed mixer than
saturation is.**

### 2.6 Fault tolerance — and why RPM is the wrong channel to detect it on

- **Allocation side** (survey §2.2.6): a fault is *"changes in the B-matrix or the constraints…
  an actuator that is locked in a faulty position could be systematically treated by setting the
  lower and upper constraint limits to the locked value."* A dead thruster is `u_min = u_max = 0`,
  which any constrained allocator absorbs with **no structural change**. A fixed ±1 mixer has no
  way to express it at all.
- **The closest published analogue to our hull**:
  [Cristofaro & Johansen, *Fault tolerant control allocation using unknown input observers*,
  Automatica 50:1891–1897, 2014](https://torarnj.folk.ntnu.no/cristofaro_automatica.pdf) — a
  **5-thruster vessel, 3 azimuth + 2 transverse tunnels**. Their idea is the one worth stealing:
  control allocation is used *actively*, constraining the redundant degrees of freedom **to make
  the faults observable**. Redundancy is not only for recovery; it is the excitation that makes
  detection possible.
- **Measured detection speed**:
  [arXiv:2504.16037](https://arxiv.org/html/2504.16037v1) on a BlueROV2 Heavy — an EKF bank with a
  Bayesian posterior over failure models identifies a two-thruster failure **within 0.1 s**, and
  soft switching reaches p = 0.9 within 5 s after a mid-run model change.
- ⛔ **The channel matters.** Published AUV thruster FDI datasets are built from **voltage, current
  and speed** per thruster. We have `ESC_STATUS(291)` undecodable here and **958/958 frames reading
  exactly 0 RPM with nothing attached** — an RPM-based detector on this vehicle would report a
  healthy zero for a missing thruster. **Current is the discriminating channel**, and it is exactly
  what firmware PR #4 (Pico ESC voltage/current/temp decoded then discarded) would put on the wire.
- **Cornell CUAUV** already ship the cheap end of this:
  [2022 report](https://robonation.org/app/uploads/sites/4/2022/07/CUAUV-Technical-Report-2022.pdf)
  — *"a new tool for automatically detecting and correcting the accidental reversal of our subs'
  thrusters… By noticing and acting on discrepancies between attempted and actual movement."*

### 2.7 Unactuated roll — what the field actually does

- The formal consequence of under-actuation is Brockett: *"the system's equilibrium cannot be
  stabilized using continuous pure state feedback."* **Scoping note that matters for us:** that
  applies to setpoint stabilisation of the *full* configuration. If roll is left to hydrostatics
  and never controlled, the 5-DOF subsystem does not hit Brockett — **the cost is paid in the
  estimator, not the controller.**
- The standard answer is **passive: centre of buoyancy above centre of gravity**.
  [A box-shaped AUV design study](https://www.sciencedirect.com/science/article/pii/S1877050915038387)
  reports a **7.39 cm metacentric height** with CoB directly above CoG, and the literature's
  comparative point: **smaller AUVs have a relatively small stabilising moment because the vertical
  CoG–CoB distance is small.**
- When passive is not enough, the published fix is **an internal moving mass**
  ([internal rolling mass](https://link.springer.com/chapter/10.1007/978-3-319-07488-7_16)), not a
  roll thruster. Worth knowing, because "add a thruster" is not the field's default.
- ⛔ **We cannot evaluate any of this on our hull**: Onshape reports **no material assigned to any
  part**, so mass, CoG, CoB and therefore BG cannot be computed. Without BG there is no published
  way to state our roll restoring moment — so "roll is passively stable" is currently an
  assumption, not a finding.

### 2.8 What the strongest competition teams run

| team | what their TDR says |
|---|---|
| **NUS Bumblebee** (RoboSub champions) | [2023](https://robonation.org/app/uploads/sites/4/2023/06/TDR_NUS-Bumblebee_RS2023-compressed.pdf): *"Our thrust allocator uses **quadratic programming** … and **maintains control along each axis of motion even during thruster saturation**."* [2025](https://bumblebee.sg/pdf/Bumblebee_Robosub_Paper_2025.pdf): QP again, *"to prolong in-water testing time and mitigate wear"*. Their trajectory planner also imposes velocity/accel/jerk limits **to keep the allocator out of saturation upstream** — belt and braces. |
| **Caltech** | [RS20](https://robonation.org/app/uploads/sites/4/2020/08/RS20_TDR_Caltech.pdf): 18-state LQR, then an explicit **4-tier priority saturation ladder** — *"(1) forces required to keep the sub static, (2) other vertical forces, (3) all other torques, (4) all other forces"*. Daisy chaining by another name — and note their **first** priority is hold-station/stay-level, not yaw. They also compensate thrust for battery voltage **on the host**. |
| **Stanford** | [RS24](https://robonation.org/app/uploads/sites/4/2024/07/RS24_TDR_Stanford.pdf): 6 PID loops → a wrench → *"allocated as forces to the eight thrusters using an allocation matrix, **informed by the physical locations of the thrusters**"*, plus a stepwise inverse-quadratic PWM fit to the T200 curve. |

**The field's verdict, in one line:** the two strongest teams in the archive both moved past fixed
mixers — Bumblebee to QP in two consecutive reports, Caltech to a documented priority ladder.
**Nobody in the sampled TDRs describes uniform per-group scale-down.**

### 2.9 Control laws — and an uncomfortable question about our own headline

> Source dossier: [`sources/sota-control-laws.md`](sources/sota-control-laws.md). ⚠ Ran with
> **zero web-search budget**; worked through the arXiv API and direct fetches. Most of the
> classical marine system-ID literature (IEEE/Elsevier/MDPI) was unreachable — §6.

⛔ **RETRACTED 2026-09-23 — "nobody has taken it underwater" was WRONG.** The claim below was made
by a sweep running on **zero search budget**, and the very first search after the quota was
restored refuted it. INDI is not an unexplored idea underwater; it is a **published method with an
open-source reference implementation on a vehicle whose thruster topology matches our firmware's**:

| | |
|---|---|
| [Cuttlefish, IROS 2024 (DFKI)](https://ieeexplore.ieee.org/document/10802674/) | attitude control of a hydrobatic intervention AUV by INDI |
| [`dfki-ric-underactuated-lab/auv_control_indi`](https://github.com/dfki-ric-underactuated-lab/auv_control_indi) | **BSD-3, Drake-based.** Vehicle: **8 thrusters, 4 vertical + 4 horizontal** — our mixer's topology |
| [Aerial-underwater vehicle, IEEE 2020](https://ieeexplore.ieee.org/document/9164924/) | attitude *and altitude* control by INDI |

That is **better** news than the original claim, and it changes the work: there is a reference
implementation to build against rather than a method to invent. Three things from its README bear
directly on us:

- INDI needs *"only a 6×6 mass-inertia matrix and an actuation model"*. **We have the actuation
  model, verified to 0.30 %.** The mass-inertia matrix is blocked on one scale.
- **INDI-QP** performs prioritised allocation *and* fault tolerance in a single method — it
  "safely executes the inspection even with fewer than six functional thrusters". That is the
  allocation gap (C-1, C-3) and the control law answered together.
- ⭐ *"**No RPM measurements needed** for fault tolerance in INDI-QP."* Decisive here: our per-ESC
  RPM channel reads **0 in 958/958 frames with nothing attached**, so every RPM-based fault
  detector we considered was built on a channel that cannot discriminate.

**The lesson, recorded rather than buried:** a "nobody has done this" claim produced without a
search budget is a statement about the search, not about the field. The paragraph below is kept
verbatim so the retraction can be checked against what it corrects.

**INDI is the strongest candidate the sweep found, and [RETRACTED: "nobody has taken it
underwater"].**
[Smeur, Chu & de Croon (arXiv 1701.07254)](https://arxiv.org/abs/1701.07254): **512 Hz** on a real
quadrotor, **7× lower gust deviation than PID — 0.21 m vs 1.51 m**. It needs only a
finite-difference angular-acceleration estimate and a control-effectiveness matrix,
**identifiable from a single test flight** — not a hydrodynamic model. ⛔ **No underwater
application was found anywhere**, actively searched, zero hits.

That combination — sensor-based, model-light, high-rate, and unexplored in water — is the single
most interesting control result in this dive **for a vehicle that already has a 500 Hz board.**

**MPC on AUVs is not ready for us, and the literature will not admit it.** The two AUV MPC papers
([arXiv 2509.17237](https://arxiv.org/html/2509.17237),
[arXiv 2503.09628](https://arxiv.org/html/2503.09628)) are **sim-only** and report **zero
solve-time or hardware numbers** despite explicit searching — a real gap in that literature, not a
gap in the search. Both ran their control loops at **10 Hz**.

**Real-vehicle evidence that adaptive identification is tractable**:
[arXiv 2603.06548](https://arxiv.org/abs/2603.06548) runs online **27-parameter** dynamics
identification on an actual BlueROV2 Heavy at a median **0.023 s per update (≥ 33 Hz)** — on an
Intel Core Ultra 9 desktop, not embedded.

#### What the 2026-09-22 bench measurement did to this row

Two of INDI's three prerequisites moved, and the third moved **against** us.

**Prerequisite 1 — knowing what a command does. CLOSED.** INDI is incremental: it computes a
small Δu each tick and must convert it into an actuator command. On this board that conversion is
**not linear** — `PILOT_EXPO`, then `thstExpo`, then a floor, then a split DShot band. That chain
is now transcribed from the firmware and verified against the vehicle to **0.30 %** worst case
([`measured-bars` §10](../measured-bars.md), [`BENCH` B-14](../workbench/BENCH.md)), and inverted
in `actuation_model.py`. An INDI increment can now be turned into a command exactly, which
previously could not have been done at all.

**Prerequisite 2 — the control-effectiveness matrix. HALF CLOSED.** The *command-side* map is
measured exactly (±1, all four horizontal axes). The map INDI actually needs is command → **angular
acceleration**, and that still requires a vehicle free to rotate. The bench gives the first factor
and nothing of the second.

⛔ **Prerequisite 3 — and this is the one that moved against us. `MOT_SPIN_MIN` and INDI are in
direct conflict.** The measured floor means the smallest output the vehicle can command is **15 %
of full scale**; there is no such thing as a small actuator increment. An incremental controller
whose increments are all quantised to 15 % is not an incremental controller. INDI would be
computing Δu in a domain the actuator cannot express.

So **[`upstream/pr-i`](../upstream/pr-i-spin-min-relay.md) is not an adjacent improvement — it is
INDI's precondition.** That was not visible when this dossier was written, and it re-ranks PR I
upward: it is the gate on the single most interesting control result in this dive.

**And INDI belongs on the board, not on the Pi.** It wants angular acceleration at loop rate; we
receive `SCALED_IMU2` at a measured **50.05 Hz**, clamped at the link's 20 ms floor, while the
board runs its own IMU at 500 Hz. Running a sensor-based incremental law at 50 Hz over a
115 200-baud link, around an actuator we cannot observe, is not a smaller version of INDI — it is a
different and worse controller. If INDI happens here it is a **firmware collaboration** in which we
supply the analysis and the identification data, not a host feature we can ship alone.

#### ⚠ Is 500 Hz justified?

**No AUV-specific study was found that ties loop rate to performance.** Every AUV control paper the
sweep reached ran at **10–100 Hz**. ArduSub/Pixhawk's 400 Hz is an *in-air* inheritance. The
assembled evidence is consistent with 500 Hz being a number carried over from multirotors rather
than derived from underwater dynamics — **and that is inference from an absence, not a citable
finding**, which is exactly how it is recorded here.

What would settle it, and it is ours to run: log `DEPTH_ERR` and attitude error at the board's
full rate, then decimate the *controller* to 250 / 100 / 50 Hz and compare. If nothing measurably
degrades until 50 Hz, the 500 Hz board is buying headroom for something else — still worth having,
but it should be said accurately.

⚠ Also worth knowing before assuming a parameter set exists: **Fossen's own MSS toolbox ships
REMUS100, NPS-AUV and DSRV — and no BlueROV2 or Girona500.**

---

## 3. The gap

| # | what the best work does | what we do | the gap, in numbers |
|---|---|---|---|
| C-1 | allocate in **force space** through a geometric `B` | allocate in **demand space** through ±1 mix coefficients | using ±1 as a force sum overstates surge/sway by **1/cos 45° = 41 %**; the fix is ~25 multiply-accumulates |
| C-2 | feed the **achieved** wrench back for anti-windup | the achieved wrench is computed on the board and **never transmitted** | every published anti-windup in this family is **unimplementable** for us today |
| C-3 | treat a dead thruster as `u_min = u_max = 0` in a constrained allocator | a fixed ±1 mixer **cannot express a dead thruster at all** | published identification of a 2-thruster failure: **0.1 s**. Ours: none |
| C-4 | detect thruster faults on **current** | we have RPM, undecodable, and **958/958 frames read 0 with nothing attached** | an RPM detector here reports a healthy zero for a missing thruster |
| C-5 | model `k_T` falling with advance ratio (four-quadrant) | `T = k·n²` bollard-only, and `k` **unmeasured** | thrust over-predicted at cruise by an amount we cannot state — the J-slope for a T200 is behind a paywall (§6) |
| C-6 | size the control floor above the **measured** ESC deadband | `VISION_YAW_MIN_PCT = 5.0`, self-labelled an assumption | the deadband is **±25 µs = ±6.25 %** of our range. **Our floor sits inside the dead zone** |
| C-7 | know `B` changes with speed for tunnel thrusters | `B` is constant | lateral authority may fall to **~10 % at 3 kn** (weak provenance, §2.5) |
| C-8 | state roll stability from a measured BG | roll assumed passively stable | **BG is not computable** — no materials in CAD |

---

## 4. Candidate moves

Ranked in [`SOTA-GAPS.md`](SOTA-GAPS.md). The shape of dive 1's answer:

- **C-1 + C-2 together are the cheapest real move in the stack**: a measured geometric `B`, one
  offline weighted pseudo-inverse, and **the applied scale factor on the wire**. That removes the
  41 % error, and it is the precondition for every anti-windup design published in this family. It
  is ~25 MACs — it fits at 500 Hz with room to spare.
- **A QP does not belong in the 500 Hz loop.** The measured solve times for a general solver are
  **10–13.2 ms** (SeDuMi class, on a laptop), against a 2 ms budget. If we want QP-quality
  allocation it runs **on the Pi at 10–50 Hz**, or as a precomputed active-set table — at p = 5
  there are only **32 saturation patterns**, and they can all be inverted offline.
- **Fault tolerance is an allocation property, not a feature.** The move is to make `B` a runtime
  object with per-thruster limits, so a dead thruster is a limit change rather than a rewrite.
- **The deadband finding (C-6) is testable this week on a bench with one thruster** and may
  invalidate a shipped constant.

---

## 5. Rejected, with the reason

| rejected | why |
|---|---|
| **Lipschitz-continuous analytic allocation** ([arXiv:2510.08119](https://arxiv.org/html/2510.08119)) | Its discontinuity is in **actuator orientation** (azimuth/tilt angle). Our geometry is fixed, so the result does not transfer. It becomes relevant only if a discrete mode switch — "thruster 3 declared dead" — makes the allocator's output jump, and then the same nullspace smoothing applies. Parked with that trigger written down. |
| **Explicit / mp-QP allocation** | Fast (tree lookup) but the survey states it needs `B` and `U` **time-invariant**: *"the online computer memory requirements may limit the applicability … where the requirements for fault tolerance and reconfigurability are simple."* It trades away exactly the property C-3 wants. |
| **Interior-point QP** | Bodson & Frost measured the crossover at **≈15 controls**; at p = 5 active set wins. |
| **LP-based direct allocation as the online method** | Simplex iteration count is unbounded in the worst case and the survey warns that **symmetric effectors cause degeneracies needing anti-cycling** — our layout is symmetric. Viable only as an offline AMS enumeration plus a lookup. |

---

## 6. Research still owed on this dive

Written down so the dossier cannot be mistaken for complete.

| owed | why it is missing |
|---|---|
| **Control laws** — INDI / adaptive INDI, Lyapunov-constrained MPC, Koopman MPC, sliding mode / super-twisting, learning-based control that actually got wet, and *what loop rate an AUV demonstrably needs* | the sweep was killed by a session limit before it reported; nothing durable was written |
| **System ID without a basin** — what is identifiable from free-running tests, added mass from geometry, coast-down drag, thruster ID without a load cell, physics-informed / GP / Koopman ID, excitation design, and what a model measurably buys | same |
| **The T200's `k` and its J-dependence** | [Lam et al., OCEANS 2023](https://ieeexplore.ieee.org/document/10244513/) is the right paper and is paywalled. We have only the qualitative statement that `K_T` falls linearly with J and reaches zero at the geometric pitch. **The error from ignoring J at 0.5–1.5 m/s therefore cannot be stated** — and writing a number for it would be exactly the plausible-number-for-a-measurement failure this project names. |
| **The tunnel-thruster speed penalty, from an academic source** | the 3 kn / 10 % figure is confirmed in two patents' full text but neither cites literature; the academic primaries were 403. |
| **Fossen & Johansen's marine-only survey (MED 2006)** and **Sarkar/Podder/Antonelli 2002** | records verified, full texts not obtainable. |

**And the honest bottom line for this dive:** none of the numbers above were measured on *this*
vehicle. Every one is somebody else's hull.
