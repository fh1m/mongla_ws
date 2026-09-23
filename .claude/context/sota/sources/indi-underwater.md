# SOTA sources: INDI (Incremental Nonlinear Dynamic Inversion) underwater, and INDI-QP allocation

Research sweep for the mongla AUV control programme. Focus: the actual update equations,
angular-acceleration estimation and filter sync, INDI-QP formulation, sensitivity to
control-effectiveness-matrix error, actuator quantisation/deadband treatment, loop rate,
and numbers vs baseline — underwater specifically. Written incrementally; see rules at
top of the source task for why.

Vehicle context this is being judged against: SROT board, 500 Hz loop, 8-thruster +-1
demand mixer (not geometric), MOT_SPIN_MIN floor = 15.8% of full scale (no small actuator
increments exist today), no usable per-thruster RPM telemetry, unknown vehicle mass.

---

## Source 1 — dfki-ric-underactuated-lab/auv_control_indi (GitHub, BSD-3-Clause)

URL: https://github.com/dfki-ric-underactuated-lab/auv_control_indi
Fetched: raw README, 2026-09-23.

This is the code + doc hub for **both** DFKI Cuttlefish papers below. Vehicle: **Cuttlefish**,
a dual-arm hydrobatic intervention AUV, 6-DOF, **8 thrusters** (4 vertical: front-left,
front-right, tail-right, tail-left; 4 horizontal: front-left, front-right, tail-right,
tail-left — i.e. a 4v+4h layout, structurally similar in thruster *count* to our M1-M4
horizontal/M5-M8 vertical mixer, though Cuttlefish's is a real intervention AUV with arms,
ours is a competition vehicle).

Key facts confirmed from the README (beyond what was already known):
- **INDI "requires only a 6×6 mass-inertia matrix and an actuation model."** This is the
  core "model-light" claim — no drag model, no added-mass model needed for the controller
  itself (though they *did* separately identify linear and linear-quadratic drag models,
  stored in `models/cuttlefish/*.yml`, apparently for simulation/plant fidelity or for the
  NDI baseline comparison, not for the INDI controller's own feedback law).
- **Two papers, two roles**: Work 1 (IROS 2024) is pure attitude INDI, quaternion-based,
  demonstrated on a 90° pitch-up maneuver. Work 2 (RA-L 2026, "INDI-QP") builds directly on
  Work 1's INDI and adds QP-based control allocation for actuator-failure prioritization,
  demonstrated on a 360° inspection orbit around a fixed object.
- **INDI-QP fault representation — confirmed mechanism**: "Passive fault tolerance is
  achieved **without RPM measurements** by running a **parallel thruster model** inside the
  control architecture — no explicit fault detection and isolation required." This directly
  answers Q2 (fault representation): faults are NOT injected as a modified effectiveness
  matrix column set to zero from an external FDI block; instead a thruster model runs
  alongside the real thrusters and the QP naturally de-weights an actuator whose modeled
  output diverges from what the achieved acceleration implies. This is architecturally
  relevant to us because **our own per-thruster RPM telemetry is useless (958/958 frames
  read exactly 0)** — DFKI's method is a plausible template for fault tolerance without RPM
  feedback, though we have not yet found the mechanism's exact math (need the RA-L paper).
- **Failure demo is interactive**: the GUI lets an operator toggle thrusters live
  ("green = enabled, red = failed") and watch reallocation happen in real time — this is a
  qualitative/visual demo, not a numeric benchmark table, in the README itself.
- **Quantitative claim, still qualitative in the README**: "significantly smaller
  line-of-sight error than a non-prioritized INDI baseline" — **no actual number given in
  the README.** Must be checked in the RA-L 2026 paper directly (paywalled, see below).
- Simulation stack: **Drake** (MIT), Python. Not real-hardware benchmarked in this repo —
  the repo is simulation-only; hardware validation claims (steadier station-keeping "in the
  maritime test basin at DFKI RIC") are reported qualitatively for Work 1, not quantified
  here.
- No loop rate stated in the README. No actuator quantization/deadband treatment mentioned
  in the README (need the papers).
- License: BSD-3-Clause, confirmed. Real, runnable code — worth cloning to read the actual
  INDI update law in source rather than relying on paper text.

**Citations resolved (previously only had bare IEEE URLs):**
- **INDI (IROS 2024)**: T. Slawik, S. Vyas, L. Christensen, F. Kirchner, "Attitude Control
  of the Hydrobatic Intervention AUV Cuttlefish using Incremental Nonlinear Dynamic
  Inversion," IEEE/RSJ IROS 2024, Abu Dhabi, pp. 781–786.
  DOI: 10.1109/IROS58592.2024.10802674. Paywalled at IEEE Xplore
  (https://ieeexplore.ieee.org/document/10802674/); DFKI hosts what should be an open PDF at
  https://www.dfki.de/fileadmin/user_upload/import/15045_20240704_root.pdf (the DFKI
  publication landing page itself, https://www.dfki.de/en/web/research/projects-and-publications/publication/15045,
  returned HTTP 403 to WebFetch — **could not verify the PDF is actually reachable**, flagged
  as unverified, retry with a real browser/curl if this matters).
- **INDI-QP (RA-L 2026)**: T. V. Slawik, S. Vyas, B. Wehbe, L. Christensen, F. Kirchner,
  "Prioritized Motion Control Robust to Actuator Failure for Hovering-Type AUVs Using
  INDI-QP," IEEE Robotics and Automation Letters, vol. 11, no. 9, pp. 10481–10488, Sep 2026.
  DOI: 10.1109/LRA.2026.3711829. **This is a very recent (this month) RA-L publication — no
  arXiv preprint found yet in searches.** Paywalled, not yet fetched in full text.

**Transferability flag**: Cuttlefish is a much larger, actuated (dual-arm), hydrobatic
intervention AUV — not directly comparable in mass/inertia scale to our competition vehicle,
but the *architecture* (8 thrusters, need for fault tolerance without RPM feedback, 6×6
inertia-only model requirement) maps closely enough to be the primary template.

---

## Source 2 — IEEE 9164924: Attitude and Altitude Control of an Unmanned Aerial-Underwater Vehicle via INDI (Chen, Liu, Hu, Feng, Ma; IEEE Access 2020)

URL (IEEE): https://ieeexplore.ieee.org/document/9164924/
URL (open, IEEE Access — this journal is fully open-access, no paywall in principle):
https://ieeexplore.ieee.org/document/9164924 — full citation: G. Chen, A. Liu, J. Hu, J. Feng,
Z. Ma, "Attitude and Altitude Control of Unmanned Aerial-Underwater Vehicle Based on
Incremental Nonlinear Dynamic Inversion," **IEEE Access, vol. 8, pp. 156129–156138, 2020.**
ResearchGate fulltext PDF located:
https://www.researchgate.net/publication/343589942_Attitude_and_Altitude_Control_of_Unmanned_Aerial-Underwater_Vehicle_Based_on_Incremental_Nonlinear_Dynamic_Inversion

**Not yet fully fetched — abstract-level only so far, from search snippets:**
- Vehicle: a **quadrotor-configuration** unmanned aerial-underwater vehicle (UAUV) —
  transitions air/water, not a dedicated AUV. Lower transferability than the DFKI work, but
  directly answers Q1 (the actual update equations) per the abstract: they derive "the
  relationship between thrust increment and vertical/angular acceleration increment... through
  Taylor series expansion and angular acceleration feedback," and explicitly use
  **second-order low-pass filters** — this is the first concrete filter-order detail found
  so far and needs full-text confirmation (cutoff frequency, whether it's on the gyro signal,
  the actuator feedback, or both).
- Compares NDI (full model-based) vs INDI, analyzes robustness of both.
- **Action item for next pass**: fetch the ResearchGate PDF in full for the actual filter
  cutoff numbers and any quantitative water-vs-air control performance comparison — this
  IEEE Access paper is open-access so a fuller fetch should be possible without a paywall
  workaround.

---

## Source 3 — arXiv 1701.07254: Smeur, Chu, de Croon — quadrotor INDI (already in "already found" list; recorded here for completeness / reference formulation)

URL: https://arxiv.org/abs/1701.07254 — "Adaptive Incremental Nonlinear Dynamic Inversion
for Attitude Control of Micro Air Vehicles" (published version: E. J. J. Smeur, Q. Chu,
G. C. H. E. de Croon, *Journal of Guidance, Control, and Dynamics*, 2016, DOI:
10.2514/1.G001490 — note: this is the **adaptive** INDI paper, JGCD, not the 512 Hz
gust-rejection number already cited in the task prompt from a *different* Smeur paper; the
"already found" 512 Hz / 0.21 m vs 1.51 m gust figures are from Smeur, Bronz, de Croon,
"Incremental Control and Guidance of Hybrid Aircraft Applied to a Tailsitter UAV" — confirmed
distinct paper, arXiv PDF at https://arxiv.org/pdf/1802.00714. **Correction to task framing**:
these are two different Smeur papers; flagging so later citation doesn't conflate them.

This is the **canonical INDI reference formulation** (quadrotor domain) that essentially
every underwater INDI paper cites for the base method. Not yet fetched in full text this
pass — queued for next batch, needed to write out Q1's actual update equations precisely
(the standard INDI increment law: `u = u0 + G1^-1 (v_dot_cmd - v_dot_est)` with angular
acceleration `v_dot_est` obtained by filtering+differentiating gyro rate, and `u0`/rate
`v_dot_est` sampled at the **same instant**, which is the crux of the sync requirement in
Q1).

---

## Source 3 (completed) — arXiv 1802.00714: Smeur, Bronz, de Croon — "Incremental Control and Guidance of Hybrid Aircraft Applied to a Tailsitter UAV" (Cyclone tailsitter)

URL: https://arxiv.org/pdf/1802.00714 (arXiv:1802.00714v2, cs.RO, 25 Sep 2019). Full text
extracted via PyMuPDF (WebFetch could not parse the PDF as it returned raw JPEG-embedded
binary; note for future passes — route arXiv PDFs through `python3 -c "import fitz..."`
rather than WebFetch when WebFetch reports "binary/encoded" content).

**This is the single most load-bearing source found this pass for Q1 (exact update
equations) and Q2 (QP allocation).** Not underwater — it is a tailsitter hybrid MAV — but it
is the canonical, fully-worked INDI derivation that the DFKI AUV papers build on, and it is
open (no paywall).

### Q1 — the actual INDI update equations, verbatim (Eq. 1–6 of the paper)

Actuator dynamics are modeled as a first-order discrete filter, **not assumed instantaneous**:
```
A(z) = a / (z - (1-a))         (Eq. 1)
```
with `a = 0.1` at a **500 Hz** sample rate for the flap servos (rate limit 272 deg/s), and
`a = 0.045` for the motors (no rate limit modeled). **This actuator state model is used to
estimate the actual current actuator position `u`, separately from the commanded `uc`** —
i.e., INDI needs to know where the actuator *actually is*, not just what was last commanded,
because command != achieved position under first-order actuator lag.

Core increment law (Eq. 2–3):
```
[Ω̇; T] = [Ω̇₀; T₀] + G(u - u₀)                      (Eq. 2)
uc = uf + G⁺ (ν - [Ω̇f; Tf])                          (Eq. 3)
```
where `Ω̇` is angular acceleration (rad/s²), `T` is specific thrust, `G` is the control
effectiveness matrix (`G_jk` = effect of actuator `k` on axis `j`), `G⁺` is the
Moore-Penrose pseudoinverse, and subscript `f` denotes **filtered** signals — this is the
key mechanism for Q1's sync question, below. `ν` is the virtual/pseudo control (desired
angular accel + thrust), formed by an outer proportional loop on rate error (Eq. 4) and a
quaternion-error proportional loop on attitude (Eq. 5).

### Q1 — how angular acceleration is obtained, and the sync mechanism (THE key finding)

Angular acceleration is obtained by **filtering the gyroscope measurement directly** (not
literally a raw finite difference — the paper says "measured by deriving it from the
gyroscope measurement," filtered with a **second-order Butterworth filter**). The
paper is explicit about the classic INDI failure mode asked about in Q1:

> "To deal with noise, sensor values will be filtered with a second order Butterworth
> filter, **which will introduce some delay. To keep all signals synchronized, all signals
> in Eq. 2 with subscript 0 will be filtered with the same filter and receive subscript f
> instead.**"

This is the general solution pattern used throughout the paper (repeated again for the
velocity-loop specific-force filtering, Eq. 30, and again for the flap-effect compensation,
Eq. 37): **every signal that enters the increment equation together — the acceleration
feedback AND the actuator-position feedback AND the previous-instant reference — must pass
through the *same* filter, so that the group delay cancels in the subtraction.** Filtering
only the acceleration channel (or filtering channels with different cutoffs/orders) is
exactly the desynchronization failure mode; the fix is not "filter less" but "filter
identically and consistently across every term used in the same increment."

Concrete filter cutoffs found (answers Q1's "what cutoff" sub-question numerically):
- Specific force `f_y` (sideslip estimation path): 2nd-order Butterworth, **cutoff 5 Hz**.
- High-pass filter for flap-effect compensation on body-X acceleration: **4th-order
  Butterworth, cutoff 0.5 Hz**.
- No explicit cutoff number given for the primary gyro→angular-acceleration filter in the
  extracted text (likely in a table/figure not captured by text extraction) — **flagged as
  not fully verified**, worth a follow-up read of the actual figures/appendix if this exact
  number matters.
- Sample/control rate: **500 Hz** (stated for the actuator model, implying the control loop
  itself runs at 500 Hz) — this matches our own SROT 500 Hz loop rate closely, a good
  transferability signal for Q5.

### Q1/Q4 — actuator dynamics are explicitly part of the model, not neglected

Contrary to the idea that INDI needs "no actuator model": this paper's Section 2.2 opens
with "**Because INDI neglects the plant dynamics, but relies heavily on the relation between
input and output, it is important to know the position of each actuator at every time**" —
so a first-order actuator-position model (Eq. 1) is required input to the increment law, it
is just a much simpler model (single pole + rate limit) than a full flight/hydrodynamic
model. This directly matters for Q4: **actuator dynamics (lag, rate limit) are explicitly
tracked and compensated for; they are not treated as negligible.**

### Q1/Q4 — minimum-thrust / saturation handling that is directly analogous to our 15.8% floor

Section 2.4 describes a **minimum thrust floor enforced for a different physical reason**
(to avoid propeller flow reversal over the control surfaces): "the minimum thrust level is
defined to be **42% of maximum thrust** when airspeed is low (<8 m/s), and **16% otherwise**."
This is a directly analogous number to our MOT_SPIN_MIN 15.8% floor — a nonzero minimum
commandable actuator level coexisting with an incremental control law — and the paper does
NOT report this floor as breaking INDI; it is simply built into the control-effectiveness
piecewise function (Eq. 11, `G23`) and the saturation-aware QP allocator (below) manages the
consequence (reduced authority near the floor), rather than the increment law itself needing
special handling for the floor. This is the strongest available evidence bearing on Q4, even
though it is a different actuator type (flap/prop, not DShot thruster) — the pattern
("built-in nonzero floor is compatible with INDI, provided saturation-aware allocation
manages authority near the floor") should transfer in principle. **Caveat: this paper's
floor is continuous/analog (thrust %), unlike our board's floor which is a genuine step
discontinuity — the region between 0 and 15.8% is entirely uncommandable on our vehicle,
which this paper's flap/motor system does not have.** This is flagged in the "cannot verify"
section below as the one open transferability risk.

### Q2 — the QP / control-allocation formulation (WLS, active-set)

Section 2.7, **directly answers Q2** even though it's not called "INDI-QP" here (the DFKI
"INDI-QP" work is presumably a direct descendant of this exact method, applied to AUV
thruster failure instead of flap saturation):

> "We have discussed the Weighted Least Squares (WLS) control allocation algorithm [Härkegård
> 2004] in previous work for quadrotor control, and apply the same method here. With relative
> weights for each controlled axis, **a quadratic programming problem is constructed, which
> is solved with the active set algorithm.** The relative priority factors used for the
> Cyclone are **[100, 1000, 0.1, 10]** for rotation around body X, Y, Z axes and thrust...
> The algorithm minimizes a cost function, taking into account the minimum and maximum input
> increments. The error in the output increment is multiplied by the priority factors,
> squared and summed to produce the cost function."

So: **prioritization is expressed as per-axis weights in a quadratic cost function**
(squared, weighted output-error terms), constrained by actuator min/max increment bounds,
solved via an **active-set QP solver** — this is the direct mechanism DFKI's INDI-QP almost
certainly generalizes to per-thruster failure weighting (need the RA-L 2026 paper to confirm
this maps 1:1, but architecturally it is the same allocation problem: axis priority instead
of "thruster priority," but same QP shape). **No solve-time number given in this paper**
(runs onboard the Cyclone's autopilot in real time per the flight tests, implying it's fast
enough for whatever the Paparazzi autopilot's control loop rate is, but no explicit number
— flagged for Q2's "solve times on hardware" ask, not yet answered numerically by any source
found so far).

Reported behavioral result (qualitative, not a clean number): under flap saturation while
pitching up, "the control of the yaw angle deteriorates... it is not unstable" — i.e., the
lower-priority axis visibly degrades gracefully rather than the system failing, which is
the whole point of the prioritized QP. Figure 5 shows psi (yaw) error peaks of order
tens of degrees during saturation events, recovering each time — a real degradation-not-
failure numeric trace, though not reducible to one clean scalar.

### Section 7 "Guidelines for implementing INDI on hybrids" — a validated checklist (useful directly for us)

The paper closes with an explicit 8-step implementation checklist, which for a control team
doing a first INDI implementation is close to a ready-made task list. Reproduced because it
answers "how would we even start":
1. Identify actuator dynamics by step-response testing (RPM vs time for motors, potentiometer
   position vs time for servos).
2. Choose a filter cutoff for gyro/accelerometer noise — explicit trade-off stated: **"More
   filtering means less noise propagated to actuators, but the system reacts slower to
   disturbances"** — i.e., the filter cutoff is the direct dial on the sync-delay vs
   noise-rejection tradeoff central to Q1.
3. Identify control effectiveness via flight-test data (linear least-squares fit of Δaccel
   vs Δinput) — **not a CAD/geometric derivation**, an empirically fitted one.
4. Tune the rate-loop and attitude-loop gains.
5–7. Add secondary effectiveness terms (thrust-on-pitch, flap-on-lift) as needed for the
   specific airframe.
8. Test acceleration control, then full autonomy.

Also directly relevant self-critique from the paper (useful for judging the "model-light"
marketing claim, Q3): the authors themselves note INDI is *not* effort-free — "there are a
few parameter estimation steps" — but argue it is still simpler than full dynamic modeling
because **only control derivatives (G matrix) and actuator dynamics need identifying**, not
full lift/drag/moment functions of state. This is the most precise, source-grounded version
of the "model-light" claim found so far: **INDI needs G (effectiveness) + actuator dynamics
only; NDI/model-based needs the full force/moment model.** It does NOT claim G can be wrong
by an arbitrary amount — G itself is still empirically identified from real test data via
least-squares fit, not assumed or guessed. This bears directly on Q3: the paper never
tests or quantifies "how wrong can G be," it only documents that G must be *identified*
(flight-tested), not modeled from first principles. No sensitivity analysis / error-bound
on G found in this paper — **flagged as an open question, not answered by this source.**

---

## Source 4 — arXiv 2201.09805: Steffensen, Steinert, Smeur — "Nonlinear Dynamic Inversion with Actuator Dynamics: an Incremental Control Perspective" (AIAA, TU Munich / TU Delft)

URL: https://arxiv.org/pdf/2201.09805 (arXiv:2201.09805v2, eess.SY, 25 Nov 2022). Full text
extracted via PyMuPDF (same WebFetch-fails-on-arXiv-PDF issue as Source 3).

This is a **theoretical/analytical** paper (with a fixed-wing roll-motion simulation, not
underwater) that directly and formally answers the "how good does INDI's actuator-instant
assumption need to be" question buried in Q1/Q4/Q5. Central result:

> "It is shown that for first-order actuator dynamics, **INDI approximates the corresponding
> NDI control law arbitrarily well under the condition of sufficiently fast actuators.** If
> the actuator bandwidth is low compared to changes in the states, the derived [actuator-
> aware] NDI control law has advantages compared to INDI: 1) compensation of state
> derivative terms 2) well-defined error dynamics 3) exact tracking of a reference model,
> independent of error controller gains in nominal conditions."

Formally: the paper derives an alternative "ANDI" (Actuator-NDI) control law (Eq. 6) that
explicitly includes actuator bandwidth Ω in the inversion, and proves in the Appendix
(Eq. 47–48) that **INDI is the limit of ANDI as actuator bandwidth ω → ∞, and that this
limit does NOT recover the exact/desired error dynamics** — i.e., **INDI has an inherent,
provable error-dynamics deficiency whenever actuator bandwidth is finite**, is worse the
slower the actuators are, and is exactly the mathematical form of the "classic INDI failure
mode" the task asked about (Q1). Quote: "When taking the limit of ω tending to infinity, Eq.
(48) does not tend to the desired error dynamics. **Hence, even in the limit, the INDI does
not produce the correct error dynamics. INDI only approximates the true NDI control law
signal arbitrarily well.**"

Direct relevance to our vehicle (Q4/Q5): **our actuator chain is not first-order-fast** —
DShot ESC response + prop spin-up + bidirectional reversal + the MOT_SPIN_MIN floor is a
much slower, much more nonlinear actuator path than a servo or brushless motor commanded
directly. This paper's central warning — that INDI's accuracy degrades as actuator bandwidth
drops relative to the vehicle's own state bandwidth — is the most directly load-bearing
theoretical result found so far for judging whether INDI is appropriate on OUR actuator
stack, and argues we should characterize actual thruster response bandwidth (step-response
time from command to force realized) before assuming INDI will behave the same as in these
papers' aerial-actuator-dominated demonstrations.

Also of note (background, referenced but not re-derived here): the paper cites prior work
(Cordeiro et al., and Pfeifle & Fichter) on **scaling the control-effectiveness matrix
specifically to improve robustness to time delay** — i.e., there is a known literature
thread on deliberately detuning/scaling G to trade bandwidth for delay-robustness. Not yet
fetched in full — **flagged as a follow-up** if we need the actual scaling law.

No underwater content, no thruster-specific quantization treatment found in this paper —
its rolling-motion example uses a fixed-wing aileron, not a marine thruster. **Flagged: still
no source found so far that treats actuator quantization/deadband/minimum-step explicitly
inside the INDI increment law** (Q4's core ask remains only indirectly answered, via the
Cyclone paper's continuous-floor analogy above, not a genuine step-discontinuity case).

---

## Source 5 — arXiv 2605.12071: Yilmaz, Turan, Pries, Ryll — "Control of Fully Actuated Aerial Vehicles: A Comparison of Model-based and Sensor-based Dynamic Inversion" (INTL Conf. on Unmanned Aircraft Systems 2026, preprint, accepted May 2026)

URL: https://arxiv.org/pdf/2605.12071 (arXiv:2605.12071v1). Full text extracted via PyMuPDF.
**Not underwater** — a fixed-tilt fully-actuated hexarotor, 2.95 kg, 13×4.5" props, Kakute H7
FC + x86 companion computer — but this is by far the **strongest quantitative source found
this pass for Q3 (effectiveness-matrix error tolerance) and Q5 (loop-rate degradation)**,
with real hardware experiments (Vicon mocap ground truth, not pure simulation), and directly
comparable INDI-vs-model-based numbers.

### Q1 — update law and sync mechanism (matches Sources 3/4, independent confirmation)

Confirms the identical sync pattern found in Source 3, stated even more explicitly:

> "The translational acceleration measurements are directly available through the onboard
> accelerometer; however, common UAV platforms lack a rotational acceleration sensor.
> Instead, the rotational velocity is measured by the onboard gyroscope, then passed through
> a **second-order low-pass filter and differentiated**. **The same second-order filter is
> applied to the actuator and translational acceleration feedback to achieve
> synchronization** [cites Smeur et al.]."

So: angular acceleration = gyro rate -> 2nd-order low-pass filter -> numerical
differentiation (not a raw finite difference of an already-noisy signal — filter first,
then differentiate). Filter cutoffs used in this paper's INDI implementation: **first-order
low-pass filters at 30–60 Hz cutoff** (stated in the paper's filtering-effects discussion),
with the NDI+disturbance-observer baseline's observer gains deliberately matched near the
same 30/60 Hz band for a fair comparison. IMU runs at 500 Hz; Vicon ground truth at 250 Hz;
control loop executed at up to 500 Hz ("the communication bandwidth limit of the
experimental platform... higher than typically reported in industrial/research multirotor
flight control systems").

INDI update law given explicitly (their Eq. 18):
```
u = F⁻¹ [[mI₃,0],[0,J]] (-ẏ₀ + ν_d) + u₀
```
where `ẏ₀` is measured translational+rotational acceleration (filtered as above), `u₀` is
**measured** squared propeller rotation speed (a genuine actuator-feedback signal, not the
last command) — same pattern as Source 3's "must know actual actuator position, not
commanded."

### Q3 — control-effectiveness-matrix error tolerance, WITH A REAL NUMBER (Experiment 1)

This is the single best number found so far for Q3. The rotor force coefficient `c_f` (which
directly scales the control-effectiveness matrix used by **both** the model-based NDI
inversion and INDI's own effectiveness matrix) was deliberately **reduced to 50% of its true
value** in the controllers, while the real hardware still had the true, un-derated rotors.
Results (Table I/II, real hardware, Vicon-tracked):

| Metric | GEO (nominal) | INDI (nominal) | GEO (0.5× c_f) | INDI (0.5× c_f) |
|---|---|---|---|---|
| mean |Δz| position error [m] | 0.21 | 0.01 | **0.39** | **0.01** |
| peak |Δz| [m] | 0.24 | 0.03 | **0.41** | **0.03** |
| mean roll error [deg] | 0.49 | 0.36 | 0.45 | **0.30** |
| longitudinal attitude error (aggregate) [deg] | 1.17 ± 1.73 | 0.58 ± 1.80 | 0.75 ± 1.63 | **0.54 ± 1.76** |
| position error (aggregate) [m] | 0.2520 ± 0.0290 | 0.0074 ± 0.0050 | 0.4033 ± 0.0187 | **0.0111 ± 0.0053** |

**A 50% error in the control-effectiveness scaling roughly doubled the model-based (GEO/NDI)
controller's z-position error, while INDI's tracking was essentially unaffected (its
attitude error even improved slightly, within noise) — INDI's position-error degradation
under 50% effectiveness-matrix error was only ~0.004 m aggregate, vs GEO's ~0.15 m.** This is
a direct, numeric answer to Q3: **INDI on this hardware tolerated a 50% control-effectiveness
error with near-zero measurable tracking degradation**, dramatically outperforming the
model-based alternative under the same error. Caveat for transferability: this is a *uniform
scalar* misspecification of one physical coefficient shared identically across all rotors —
not a case where the effectiveness matrix has the wrong *shape/sign pattern* (e.g., our +-1
demand mixer standing in for real thruster geometry, which could introduce structured,
non-uniform errors across axes rather than one clean scale factor). **This is a meaningfully
different error mode than "our mixer might not represent true thruster geometry" and should
not be over-read as proof our exact mixer error will be tolerated equally well** — flagged
explicitly in the "could not verify" section.

### Q5 — loop-rate degradation, WITH REAL NUMBERS (Experiment 4, Table V)

Real hardware, hover task, controller execution swept 500 -> 250 -> 125 -> 62.5 -> 50 Hz:

| Freq | INDI longitudinal attitude err [deg] | INDI position err [m] | GEO longitudinal attitude err [deg] | GEO position err [m] |
|---|---|---|---|---|
| 500 Hz | 0.19 ± 0.11 | 0.0264 ± 0.0050 | 0.76 ± 0.21 | 0.2158 ± 0.0111 |
| 250 Hz | 0.11 ± 0.09 | 0.0075 ± 0.0024 | 0.81 ± 0.23 | 0.2338 ± 0.0090 |
| 125 Hz | 0.15 ± 0.10 | 0.0069 ± 0.0019 | 0.49 ± 0.22 | 0.2423 ± 0.0095 |
| 62.5 Hz | 0.24 ± 0.12 | 0.0119 ± 0.0042 | 0.43 ± 0.15 | 0.2693 ± 0.0144 |
| **50 Hz** | **1.34 ± 0.68** | **0.0095 ± 0.0041** | 0.26 ± 0.15 | 0.2144 ± 0.0092 |

Paper's own framing: **"With a decreasing control frequency, the attitude tracking
performance of INDI degrades significantly"** — the longitudinal attitude error is roughly
flat/best in the 125–250 Hz band, and only blows up sharply at 50 Hz (0.19 deg at 500 Hz to
1.34 deg at 50 Hz — a ~7x degradation). Notably **the degradation is non-monotonic**
(500 Hz is not actually the best — 125/250 Hz have the lowest attitude error in this
dataset, likely a filter/gain interaction at very high rate) — worth flagging so we don't
over-read "higher rate is strictly always better" from this table. **Position tracking stays
good for INDI even at 50 Hz** (0.0095 m, still ~20x better than GEO's 0.21 m at the same
rate) — so the specific failure mode of low-rate INDI in this experiment is concentrated in
the *attitude/angular-acceleration* channel, consistent with Q1's sync/filter-delay concern:
angular acceleration estimation is the piece most sensitive to sample rate, because
differentiating a filtered gyro signal amplifies the effect of a coarser sample interval.

**Direct relevance to our vehicid**: our SROT loop runs at 500 Hz, squarely in the "good"
band from this dataset (though note 500 Hz was not literally the minimum-error point in this
one experiment — 125/250 Hz was slightly better here, plausibly filter-tuning-specific, not
a general rule). This is reassuring for Q5 but is a single hexarotor result, not underwater,
and the paper itself frames it as "how strongly both inversion strategies depend on update
rate" without giving a general closed-form degradation law — **no general rate threshold
number beyond this one dataset was found in any source this pass.**

**Also confirms Q3's "how wrong can it be" boundary is not unlimited**: the paper frames
INDI's whole value proposition as needing *only* control-effectiveness + actuator feedback,
but Experiment 1's own baseline nominal-vs-nominal difference (control action RMS ~1e-14 in
distinct nominal case per an earlier passage) shows INDI and NDI are structurally identical
absent model error — the entire performance gap opens up only once effectiveness/mass/inertia
are wrong, which is INDI's raison d'être, not a weakness.

Citation: N. Yilmaz, O. Turan, D. Pries, M. Ryll, "Control of Fully Actuated Aerial Vehicles:
A Comparison of Model-based and Sensor-based Dynamic Inversion," *International Conference
on Unmanned Aircraft Systems (ICUAS) 2026*, preprint accepted May 2026. arXiv:2605.12071.
(Note: arXiv IDs of form 2605.xxxxx / 2604.xxxxx / 2608.xxxxx appearing in this sweep are
from 2026 — arXiv's YYMM numbering, consistent with "today" being 2026-09-23; these are very
recent preprints, not typos.)

---

## Source 6 — DFKI project page (auv_control_indi), second pass: confirms Fossen model, OSQP solver, Wx priority matrix, "2 functional thrusters" survival number

URL: https://dfki-ric-underactuated-lab.github.io/auv_control_indi/

New details beyond the README (Source 1), fetched this pass:
- **"The approach is derived on a Fossen model."** Confirms the INDI derivation for Cuttlefish
  starts from the standard Fossen 6-DOF marine-vehicle equations of motion (the same
  reference-frame convention essentially all AUV control literature, including our own
  vehicle's likely modeling assumptions, is built on) — reassuring for transferability, this
  is not some bespoke non-standard formalism.
- **QP solver named: OSQP**, and it explicitly **"respects actuator-rate limits"** — this is
  new and directly relevant to Q2 and Q4: the QP constraint set includes actuator *rate*
  limits (not just magnitude/min-max bounds as in the Cyclone paper's WLS formulation,
  Source 3) — i.e. DFKI's INDI-QP treats how fast a thruster's command is allowed to change
  per solve as a hard constraint, not just its absolute range. Still **no explicit
  deadband/quantization/minimum-step term found** — a rate limit is not a deadband, and this
  remains the closest analog found to Q4's actual ask.
- **Prioritization mechanism, precise**: "the diagonal cost matrix `Wx` establishes a task
  hierarchy: roll and pitch (line-of-sight alignment) are weighted far above yaw and
  translation, so the rotation about the sensor axis is sacrificed first" — directly answers
  Q2's "how is prioritization expressed": a diagonal weighting matrix in the QP's quadratic
  cost, structurally identical to the Cyclone paper's `[100, 1000, 0.1, 10]` priority vector
  (Source 3) — the same WLS/QP pattern generalizes cleanly from aerial actuator-saturation
  prioritization to underwater thruster-failure prioritization.
- **New quantitative claim (still not sourced to an exact number/table, but more specific
  than the README's vague "fewer than six")**: "completes the 360° inspection with **as few
  as two functional thrusters**" — i.e., out of Cuttlefish's 8 thrusters, INDI-QP was
  demonstrated tolerating the loss of **6 of 8** thrusters and still completing the mission
  (with degraded, but bounded, line-of-sight error — the actual LOS-error number is still
  only in the paywalled RA-L 2026 paper, not found in any openly fetchable source this pass).
- Fault representation confirmed again: no RPM measurement, parallel thruster model runs
  alongside real thrusters (same as Source 1).
- **Could not fetch the actual RA-L 2026 or IROS 2024 PDFs this pass** — DFKI's own PDF host
  is behind a Cloudflare bot-challenge page (confirmed via curl: returns a "Just a moment..."
  challenge page, not the PDF, regardless of user-agent), and IEEE Xplore requires
  institutional access. **No arXiv preprint of either paper could be located.** This means
  Q2's numeric solve-time and Q6's actual underwater-vs-baseline numeric comparison remain
  **unanswered by any source reachable this pass** — flagged prominently below.

---

## Source 7 (partial / could not fully verify) — IEEE Access 2020: Chen, Liu, Hu, Feng, Ma — UAUV INDI (quadrotor aerial-underwater vehicle)

URL: https://ieeexplore.ieee.org/document/9164924/ (IEEE Access — nominally open-access, but
WebFetch could not retrieve full text; the ResearchGate mirror at
https://www.researchgate.net/publication/343589942 also returned HTTP 403 to both WebFetch
and curl with a browser user-agent). **Full text not obtained this pass — recorded here only
at the abstract/search-snippet level, already captured in the earlier append:** second-order
low-pass filters, Taylor-series derivation of thrust-increment vs acceleration-increment
relationship, NDI-vs-INDI robustness comparison. **No numeric results extracted — flagged as
unverified**, would need a different retrieval route (e.g. Sci-Hub is out of scope for this
sweep; institutional IEEE access; or a direct request to the authors) to get the actual
underwater-navigation numbers this paper reportedly contains.

---

## Source 8 (found, not fetchable) — MDPI Aerospace 2023: "Incremental Nonlinear Dynamic Inversion Attitude Control for Helicopter with Actuator Delay and Saturation"

URL: https://www.mdpi.com/2226-4310/10/6/521 — title alone is the single most directly
relevant hit found in this entire sweep for **Q4 (actuator saturation/delay treatment inside
INDI)**, but **both WebFetch and a direct curl with a full browser user-agent were blocked
(HTTP 403 / 400)** — MDPI's site is evidently blocking this fetch path entirely, not just
this one paper. **Not verified in any way beyond the title** — flagged as the top follow-up
target for a future pass if actuator saturation/delay treatment inside INDI is needed in
more depth than the Cyclone paper's continuous-floor analogy (Source 3) provides.

---

## Source 9 (ruled out) — arXiv 2312.07290: Huang et al., "Underwater Motions Analysis and Control of a Coupling-Tiltable Unmanned Aerial-Aquatic Vehicle" (Mirs-Alioth, CUHK)

URL: https://arxiv.org/pdf/2312.07290. Fetched and searched full text via PyMuPDF: **zero
occurrences of "INDI," "incremental," or "acceleration feedback."** This paper controls its
UAAV's underwater motion with a Singular-Thrust-Tilt-Angle analysis, a Nussbaum-function
auxiliary controller, and a logic-switching scheme — a completely different (non-INDI, non-
acceleration-feedback) control paradigm. **Ruled out as an INDI source**, recorded here only
so a future pass doesn't re-fetch it expecting INDI content.

---

## Claims I could not verify

- **Q2 solve-time on hardware**: no source found in this sweep states an actual QP solve
  time in milliseconds/microseconds for INDI-QP on real or even simulated AUV hardware. The
  DFKI project page confirms the solver (OSQP) and that it respects rate limits, but gives no
  timing number, and the RA-L 2026 paper (which almost certainly has this number, since OSQP
  papers/benchmarks routinely report solve times) was unreachable (Cloudflare-blocked PDF,
  paywalled IEEE Xplore, no arXiv preprint found).
- **Q6 underwater-specific numeric comparison vs baseline**: the DFKI IROS 2024 paper's
  actual station-keeping/pitch-up numbers (vs the "classical model-based scheme" baseline)
  are only described qualitatively ("much steadier") in every source reachable this pass —
  the source PDF itself was not obtainable. The RA-L 2026 LOS-error number for INDI-QP vs a
  non-prioritized INDI baseline is likewise only qualitative ("significantly smaller") in
  every reachable source.
- **IEEE Access 2020 UAUV paper's actual numeric NDI-vs-INDI comparison**: not obtained
  (403 on both IEEE and the ResearchGate mirror).
- **Q4, actuator quantization/deadband/minimum-step treated explicitly inside an INDI
  increment law**: no source found in this sweep treats a genuine step-discontinuity
  actuator floor (as opposed to a continuous minimum-thrust floor, Source 3, or a rate limit,
  Source 6) inside the INDI formulation itself. This is a real, not just an unfetched, gap —
  worth treating as a genuinely open research question for our vehicle rather than assuming
  the literature has already answered it. The MDPI helicopter paper (Source 8) is the most
  promising unexplored lead; it was not reachable this pass.
- **The exact gust-rejection numbers (0.21 m vs 1.51 m, 7x, quoted in the task's "already
  found" list) were not re-verified against primary text this pass** — they were not present
  in the Source 3 tailsitter paper's extracted text (searched for "512" and found nothing);
  they most likely come from a different, closely related Smeur/Bronz/de Croon quadrotor
  paper not fetched in this sweep. Flagged so this number isn't propagated without a direct
  citation check in a future pass.

---

## Answers to the seven questions, synthesized (see per-source detail above for citations/numbers)

**Q1 — formulation & sync.** Standard INDI increment law across every source: `u_c = u_f +
G⁺(ν − ẏ_f)`, where `ẏ_f` (accel) and `u_f` (actuator position) are BOTH measured, BOTH
passed through the **same filter** (2nd-order Butterworth or 2nd-order low-pass in every
source that specified filter order; some sources add a 4th-order high-pass for specific
secondary terms), and only then differenced against the previous-instant values, which
receive the identical filter. The delay/sync failure mode is solved not by minimizing delay
but by **matching delay exactly across every signal entering the same increment equation**.
Cutoffs found: 5 Hz (2nd-order, specific-force), 0.5 Hz (4th-order high-pass, secondary
term), 30–60 Hz (1st-order, hexarotor). No single "the" cutoff — it's tuned per signal and
per platform's noise/bandwidth tradeoff (explicitly named as a tunable dial in Source 3's
implementation guidelines, item 2).

**Q2 — INDI-QP.** Confirmed as a QP over per-axis (or per-thruster-implied) increments,
solved via active-set (Cyclone/aerial precedent) or OSQP (DFKI's actual AUV implementation),
with a diagonal weighting matrix expressing priority (roll/pitch over yaw/translation for
DFKI; [X,Y,Z,thrust] weights for the Cyclone). Failed thrusters are represented **not** as a
zeroed effectiveness-matrix column from an explicit FDI step, but via a parallel thruster
model whose divergence from measured acceleration implicitly starves the QP's allocation to
that actuator — passive fault tolerance, no RPM needed. **No solve-time number found
anywhere in this sweep** — the biggest unanswered item for Q2.

**Q3 — sensitivity to effectiveness-matrix error.** The single hardest number found: a 50%
scalar error in control-effectiveness (rotor force coefficient) produced **near-zero**
INDI tracking degradation (aggregate position error 0.0074 m → 0.0111 m) vs the model-based
alternative's error roughly doubling (0.2520 m → 0.4033 m) on the SAME hardware, SAME error
(Source 5, real quadrotor hardware). This is strong support for "our +-1 demand mixer is
probably good enough as a starting G, as long as it's the right *shape*" — but this is a
uniform scalar error, not a structurally wrong (wrong relative axis coupling/sign) matrix,
which is the more realistic risk with a demand-mix standing in for real thruster geometry.
**No source quantified tolerance to a structurally-wrong (not just scalar-wrong)
effectiveness matrix** — this is the actual open question for our specific situation, not
fully answered by anything found.

**Q4 — actuator resolution/quantization/deadband.** The weakest-answered question in this
sweep. No source treats a genuine step-discontinuity minimum-commandable-output inside the
INDI law. The closest analogs: (a) a *continuous* minimum-thrust floor (16–42%) coexisting
fine with INDI in the Cyclone paper, managed by the allocator rather than the increment law
(Source 3); (b) DFKI's QP explicitly enforcing actuator *rate* limits (Source 6), a related
but distinct constraint from a *magnitude* deadband. Our 15.8% floor is a genuine
discontinuity (0% and 15.8–100% are the only reachable regions, nothing between) — this is
qualitatively different from every actuator model found in the literature this pass, and
should be treated as an open engineering risk, not something the literature has already
resolved for us.

**Q5 — loop rate.** Best data: real hexarotor hardware, INDI attitude error 0.19±0.11 deg at
500 Hz degrading to 1.34±0.68 deg at 50 Hz (~7x), with a non-monotonic dip at 125–250 Hz
(Source 5). Position tracking stayed good even at 50 Hz. Our SROT 500 Hz loop sits at the
high/safe end of every rate tested in the literature found this pass — the strongest
"transferability is fine" evidence in the whole sweep. Caveat: 500 Hz control loop is not the
same as 500 Hz *acceleration estimation* — need to confirm what rate the actual angular-
acceleration filter+differentiation runs at on our board vs. what rate new demand values
reach the actuators (mixer, DShot) — a mismatch there would recreate exactly the sync issue
in Q1 even with a fast outer loop.

**Q6 — numbers vs baseline, underwater specifically.** Genuinely thin: every underwater-
specific numeric comparison (DFKI IROS 2024, DFKI RA-L 2026, IEEE Access 2020 UAUV) was
either paywalled, Cloudflare-blocked, or 403'd in this sweep, and only qualitative claims
("much steadier," "significantly smaller LOS error") were recoverable. The best real numbers
in this whole sweep are all **aerial** (Source 5's hexarotor table). This is the single
biggest gap to flag to whoever reads this file next — **the underwater numeric evidence
specifically has not actually been read**, only its existence and qualitative conclusions
confirmed.

**Q7 — other underwater INDI work.** Beyond the DFKI Cuttlefish work and the Chen et al. 2020
UAUV (quadrotor, not a true AUV), no other underwater-INDI-specific paper was found despite
multiple search-term variations ("sensor-based control AUV," "incremental backstepping
underwater," "INDI marine," "acceleration feedback AUV control"). One aerial-aquatic vehicle
paper (Mirs-Alioth, Source 9) was found and ruled out — it does not use INDI. **The DFKI lab
appears to be functionally the only group publishing INDI specifically for underwater
vehicles as of this search date (2026-09-23)** — worth stating plainly rather than implying
a broader literature exists.

## Remaining open follow-ups (not done this pass — for a future session)

- [ ] Retrieve the DFKI IROS 2024 and RA-L 2026 full texts through an institutional-access
      route (Cloudflare blocks the DFKI PDF host; IEEE Xplore is paywalled; no arXiv
      preprint exists as of 2026-09-23) — this is the single highest-value fetch left, since
      it would answer Q2's solve-time and Q6's underwater numeric comparison, the two
      weakest-answered questions in this sweep.
- [ ] Retrieve the MDPI Aerospace 2023 helicopter actuator-delay/saturation INDI paper
      (Source 8) through a route other than direct WebFetch/curl (both blocked) — most
      promising unexplored lead for Q4's actual ask (a genuine deadband/quantization
      treatment).
- [ ] Retrieve the IEEE Access 2020 UAUV paper (Source 7) full text — ResearchGate and IEEE
      both 403'd this pass.
- [ ] Verify the "512 Hz / 0.21 m vs 1.51 m / 7x" gust-rejection figures from the task's
      "already found" list against a primary source — not found in the Source 3 tailsitter
      paper text; likely a different Smeur/Bronz/de Croon paper not yet identified precisely.
- [ ] If actuator quantization remains unaddressed by the literature after the above, treat
      it as a genuinely open problem for this vehicle and consider it the primary novel
      contribution needed before adopting INDI on the SROT board, rather than continuing to
      search for a paper that may not exist.
