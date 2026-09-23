# SOTA: attitude / depth / station-keeping control — what makes a small AUV hold still

Research pass, 2026-09-23. Topic: cascaded attitude tuning, near-surface depth control,
station-keeping precision, disturbance observers (ADRC/ESO), unactuated-roll design practice,
integral/trim schemes, and what strong RoboSub/SAUVC teams report for control specifically.

Our vehicle for transferability judgment: ~702×176×172 mm, 4× 84 mm tunnel thrusters + 1 axial
nose thruster, **roll unactuated**, 500 Hz cascaded angle-P → rate-PID on ESP32, Bar30 depth
(loop never closed), hydrodynamic feedforward gains all zero, 50 Hz vision servo for precision
alignment on the companion computer. Written incrementally — do not hold findings in memory.

Every finding is tagged **[REAL VEHICLE]** or **[SIM]**. A vehicle class note follows every
number so a torpedo/survey-AUV result isn't silently read as applying to us.

---

## Q1 — Cascaded attitude-loop tuning practice

No source found in this pass gives a numeric, reusable "angle-P here, rate-PID there, N:1
bandwidth separation" recipe for an underwater vehicle the way multirotor autopilots publish one.
What is recoverable instead:

**[General pattern, multiple sources, mostly SIM]** — Search summary across several AUV control
papers (self-adaptive fuzzy PID for heading/depth, inverse-kinematics + self-tuning fuzzy PID for
full 6-DOF stabilization) confirms the **architecture** matches ours — attitude control cascaded
with angular-velocity control, outer and inner loop gains tuned sequentially/separately — but
none of the surfaced sources publish an outer:inner bandwidth ratio or a gain-selection procedure
tied to added mass / quadratic drag specifically. Tuning methods mentioned only in the abstract
layer: pole placement, Ziegler–Nichols, and AI-based (fuzzy/PSO) autotuning.
[Modeling/control of AUV heading and depth via self-adaptive fuzzy PID (ResearchGate)](https://www.researchgate.net/publication/276886834_Modeling_and_control_of_autonomous_underwater_vehicle_AUV_in_heading_and_depth_attitude_via_self-adaptive_fuzzy_PID_controller) —
not fetched full-text, abstract-level only.
[Trajectory following & stabilization via inverse kinematics + self-tuning fuzzy PID — PLOS ONE / PMC](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5500310/) —
open access, fully-actuated AUV, **not yet read full-text this pass** — flagged as a lead to
re-fetch for numeric gains before citing further.
[Saturation-based nonlinear PID for underwater vehicles: design, stability analysis and experiments](https://www.academia.edu/81273544/Saturation_based_nonlinear_PID_control_for_underwater_vehicles_Design_stability_analysis_and_experiments) —
title claims **real-vehicle experiments**; not yet fetched full-text this pass.

**Working conclusion so far**: the published literature treats cascaded angle→rate control as the
standard shape for underactuated/AUV attitude control (consistent with our own architecture) but
publishes gain *values* almost never in the abstract layer — they are buried in each paper's own
identified hydrodynamic model, which doesn't transfer numerically across hull shapes anyway. The
actionable transferable claim is qualitative: **tune the inner (rate) loop against the vehicle's
identified linear+quadratic drag and added-mass coefficients, not against a generic bandwidth
ratio rule** — no source contradicts this, several imply it structurally.

---

## Q5 — Unactuated roll: passive hydrostatic stability, is it the standard answer?

**[REAL DESIGN NUMBERS, box-shaped AUV, not flight-tested in this excerpt — verify]**
[Passive Hydrostatic Stability Design of a Box-shaped Autonomous Underwater Vehicle — ScienceDirect](https://www.sciencedirect.com/science/article/pii/S1877050915038387) /
[same, Xi'an Jiaotong-Liverpool repository page](https://scholar.xjtlu.edu.cn/en/publications/passive-hydrostatic-stability-design-of-a-box-shaped-autonomous-u/)
- Explicit design rule: **heavy components low, light components high** to maximize the
  restoring moment in roll and pitch — i.e. maximize BG (vertical separation of centre of
  buoyancy above centre of gravity), which is exactly the lever passive stability depends on.
  This is stated as necessary specifically because **box-shaped / non-torpedo hulls have limited
  passive restoring moment compared to a torpedo shape** — directly relevant to our tunnel-thruster
  hull, which is not torpedo-shaped either.
- **Numeric result for their vehicle**: 68.8 kg in air, 0.6 kg positive buoyancy, **GM (metacentric
  height) = 7.39 cm**, with the centre of buoyancy sitting directly above the centre of gravity
  (their stated design target). **Vehicle mass is ~2-3x ours** (our hull is smaller, likely
  10-30 kg per the brief) — treat 7.39 cm as an order-of-magnitude reference for a "how much GM is
  achievable/typical," not a target to copy directly; GM does not scale linearly with mass or
  length, and this number has not been independently re-derived for our hull.

**[Directly on point — necessity paper]**
[Necessity of Hydrostatic Stability in Autonomous Underwater Vehicles — DFKI, OCEANS 2022 SOS](https://www.dfki.de/fileadmin/user_upload/import/12795_oceans2022_sos_paper_final.pdf)
— title is exactly our Q5 question; **not yet fetched full-text this pass**, flagged as the single
highest-priority re-fetch for the next session on this topic.

**[General/qualitative, search-summary level]** — Confirmed across multiple sources: reduced GM as
AUVs get smaller is a recognized problem ("roll motion can be ambiguous without active roll
stabilization, especially for smaller AUVs which have a relatively small stabilization moment due
to limited vertical distance from CG to CB") — this is evidence *against* assuming passive
stability scales down for free; a small hull like ours may have materially less restoring moment
per unit disturbance than a large survey AUV, which is the class most "passive roll is fine" case
studies come from.
[Roll Control of an AUV Using an Internal Rolling Mass — Springer](https://link.springer.com/chapter/10.1007/978-3-319-07488-7_16) /
[ResearchGate copy](https://www.researchgate.net/publication/275025601_Roll_control_of_an_autonomous_underwater_vehicle_using_an_internal_rolling_mass) —
exists specifically because passive-only roll control was judged insufficient for their vehicle;
their answer was an **active internal moving mass**, not a purely passive design — evidence that
"passive hydrostatic stability is the standard answer" is not universal; some real AUV programs
added active roll actuation instead of accepting passive-only. Not yet fetched full-text — the
vehicle class (torpedo vs box) that motivated this choice is unconfirmed.

**No source found yet** (this pass) documenting a specific real case of thruster wash or
maneuvering rolling a passively-stable small AUV beyond usable camera tilt — this remains an open
item, not found, not refuted.

**Working conclusion**: passive hydrostatic stability (maximize BG) is confirmed as standard
design practice and the literature gives a concrete "heavy low, light high" rule plus one real
GM number (7.39 cm, 68.8 kg box AUV) to sanity-check against. But it is **not universally treated
as sufficient** — at least one program added active roll mass control — and the "GM shrinks as
vehicles get smaller" caution applies directly to a ~700 mm hull like ours, so passive stability
should be measured on our own hull (Onshape has no material assigned per CLAUDE.md — mass/CoM/CoB
are not yet computable), not assumed from a larger vehicle's numbers.

`https://www.dfki.de/fileadmin/user_upload/import/12795_oceans2022_sos_paper_final.pdf` (the
paper whose title is exactly our Q5) returned **403 Forbidden to WebFetch** on both attempts this
pass — not reached. Highest-priority re-fetch for a follow-on session (try a different fetch path
or the OCEANS 2022 IEEE Xplore listing instead of the DFKI-hosted PDF directly).

---

## Q3 — Station-keeping / hover precision, real numbers

**[REAL VEHICLE — closest vehicle-class match found in this whole pass]**
[Model-Free High-Order Sliding Mode Controller for Station-Keeping of an AUV in Manipulation Task — PMC9231013](https://pmc.ncbi.nlm.nih.gov/articles/PMC9231013/)
- Platform: **BlueROV2**, 6 thrusters (vectored, not tunnel — but same T200-class thruster family
  and comparable overall size/mass to our hull; this is the single most transferable real-vehicle
  result found in this pass). Depth from **Bar-30** (identical sensor to ours). Velocity from an
  **exact differentiator**, i.e. no DVL — numerically differentiated position, same sensor-poverty
  situation we're in without the downward-camera velocity estimate.
- Tested in a **semi-Olympic swimming pool** — real water, not a tank or sim.
- **Position-hold RMSE, no added disturbance: 1.1 cm.** With ~10 N applied disturbance: 1.26 cm
  RMSE. With ~20 N: 2.4 cm RMSE. Under thruster saturation (max tested disturbance): **4.3 cm
  RMSE** — this is their worst case, not a failure.
- Controller: **model-free high-order sliding-mode control**, time-parametrized gain, explicitly
  claims **no vehicle hydrodynamic model and no disturbance observer needed** — i.e. it is in the
  same "model-light" family INDI and ADRC belong to, but is SMC-flavored rather than ESO-flavored.
- Cites a comparator: another study reported **up to 20 cm position drift under 1-knot current**
  with a different control approach (uncited in the extraction — treat as secondhand until the
  primary source is checked) — their own 1–4 cm number is presented as beating that by roughly an
  order of magnitude.
- **This gives us a concrete target number for pixel-accurate alignment**: sub-2 cm hold under
  moderate disturbance is demonstrated on hardware very close to our class. Our 50 Hz vision servo
  doing precision alignment should be judged against this bar, not against survey-AUV numbers.

**[REAL VEHICLE, DVL-equipped, tank test]** — search-summary only, not independently fetched —
a DVL-enabled ROV station-keeping test in a water tank over 30 minutes held **start/end position
variation under 1 cm**. [Water Linked — ROV navigation and positioning](https://waterlinked.com/applications/rov-navigation-and-positioning) —
**this is a DVL-based number, not comparable to our downward-camera-only velocity source without
a DVL**; flag explicitly that DVL-based hold is the better-instrumented case and our vision-only
velocity estimate should not be expected to match it without validation.

---

## Q4 — Disturbance observers / ADRC on real underwater vehicles

**No fully-real-vehicle ADRC/ESO result was found and independently confirmed in this pass.**
Specific findings:

**[SIM ONLY, confirmed by direct fetch]**
[Attitude Stabilization Control of AUV Based on Decoupling Algorithm and PSO-ADRC — PMC8918931](https://pmc.ncbi.nlm.nih.gov/articles/PMC8918931/)
- Explicitly **simulation only** — paper states "simulation experiments are performed," no real
  vehicle. 6-DOF AUV model, 5.8 kg mass (small-vehicle class, order of magnitude below ours, but
  simulated only so hull-transferability doesn't really apply).
- Numeric comparison **under external interference** (their table): PSO-PID — 2.242 s response
  time, 0.2% overshoot, 0.7% steady-state error. Plain ADRC — 2.601 s, 2.7%, 0.1%. **PSO-ADRC —
  2.628 s, 0.4% overshoot, 0.07% steady-state error** — ADRC variants win on steady-state accuracy
  and disturbance robustness, PID wins on raw response speed. Under **internal parameter changes**
  (a mass/inertia mismatch test), the paper states PSO-PID "basically fails to satisfy the control
  requirements" while PSO-ADRC is "least affected" — this is the clearest sim-only evidence in
  this pass that ADRC's model-light robustness claim holds up against a tuned PID, at least in
  simulation.

**[REAL VEHICLE, field experiment — not yet independently confirmed, WebFetch blocked]**
[ADRC-SMC-based disturbance rejection depth-tracking control of underactuated AUV — Journal of Field Robotics 2024](https://onlinelibrary.wiley.com/doi/full/10.1002/rob.22312) —
**WebFetch returned 403 Forbidden**, not reached this pass. Search-summary claims **field
experiments in Mulan Lake** (a real lake, not a pool/tank) comparing ADRC-SMC pitch autopilot
against unspecified baselines with "remarkable performance in disturbance rejections" — this is
exactly the real-vehicle ADRC result Q4 asks for, but **the actual numbers were not retrieved
and this must be treated as an unverified lead**, not a citable result, until re-fetched (try
Sci-Hub-free alternate host, or check if the authors posted an open-access preprint).
[A hierarchical disturbance rejection depth tracking control of underactuated AUV with
experimental verification — ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0029801822017413) —
title also claims experimental verification; abstract-level only, **not fetched**, second
priority lead for a follow-on pass. Uses LOS/ALOS guidance to turn depth tracking into pitch
tracking, structurally similar to a cascaded approach.

**Working conclusion on Q4**: ADRC "got wet" claims exist in the grey literature (Mulan Lake field
test, hierarchical experimental-verification paper) but neither was independently confirmed this
pass — both are paywalled/blocked. The only number-backed comparison available (PSO-ADRC vs PID)
is simulation-only. Treat ADRC-on-real-AUV as **plausible but not yet verified** — the strongest
real disturbance-rejection number in this whole brief remains the SMC station-keeping result in
Q3 (1–4 cm RMSE, BlueROV2, real pool), which is architecturally adjacent to ADRC (model-light,
observer-free) even though it isn't ADRC by name.

---

## Q6 — Integral action / buoyancy trim as published practice

**[Named concept found, but from a different vehicle class — full-ocean-depth survey AUV, NOT
transferable in scale]** — Search-summary only, title/authors not yet confirmed by direct fetch:
an "integrated vertical control method" for **full-ocean-depth (FOD) AUVs** performs **online
identification and compensation of residual buoyancy**, triggered by a velocity/position-mode
switch, where "the equivalent residual buoyancy is estimated online via thruster force
equilibrium and applied as feedforward compensation to achieve precise and stable hovering." This
is functionally the same idea as our CoB auto-trim (bleed a steady-state signal into a persistent
feedforward term) — but the motivating physics is different (hull compression and seawater
density change under extreme pressure, not CoB/CoM offset), and the vehicle class (large,
full-ocean-depth survey AUV) is explicitly **not transferable** to a 10–30 kg tunnel-thruster
vehicle. Not independently fetched — flagged as a lead on the *name* of the technique
("residual buoyancy estimation via thruster force equilibrium," "feedforward buoyancy
compensation") to search further, not as a verified transferable result.

**[Directly relevant mechanism, sliding-mode not PID]** — Search-summary only: "the inner pitch
controller is designed using sliding mode control (SMC) with integrator effect to overcome a
constant offset term due to positive buoyancy of the AUV" — this is the generic control-theory
justification for why an integrator (or an integrator-fed feedforward trim) is the correct tool
against a constant force bias like buoyancy or CoB offset — consistent with, but not proof of,
our own auto-trim design. No specific paper title/authors captured for this snippet — re-derive
by re-running the search if this needs a citable source.

**Working conclusion on Q6**: the *general* pattern — use an integrator (or a value bled from one)
to absorb a constant-force disturbance like residual buoyancy — is confirmed as standard control
practice and appears under the name **"residual buoyancy compensation"** / **"feedforward
buoyancy/trim compensation"** in the literature, most concretely in variable-buoyancy-system (VBS)
and trim-tank AUV papers (survey-class vehicles). No source found in this pass that names the
*specific* technique of bleeding a rate-integrator into a persistent CoB trim (our design) as a
named, published pattern under that exact framing — it may be a genuine variant rather than a
textbook technique. Not confirmed either way; treat as **novel until shown otherwise**, and note
that the closest published analogue is buoyancy trim, not attitude/CoB trim specifically.

---

## Q7 — What strong RoboSub teams report for attitude/depth control specifically

**[REAL VEHICLE — Caltech Robotics Team, RoboSub 2020 TDR, directly fetched and read]**
[Caltech RoboSub 2020 Technical Design Report](https://robonation.org/app/uploads/sites/4/2020/08/RS20_TDR_Caltech.pdf)
- Confirms the brief's claim: **18-dimensional LQR** — six translational/angular position errors,
  six integrals of those errors (i.e. the LQR state itself includes integral terms — this is an
  LQR analog of PID's I-term, built into the state vector rather than bolted on), six rates of
  change of those errors. System is **locally linearized about the target state and discretized**
  before solving for gain K via Drake's LQR solver.
- **Saturation handling matches the brief's "4-tier priority ladder" description exactly**: LQR
  output decomposed into (1) forces to keep the sub static, (2) other vertical forces, (3) all
  other torques, (4) all other forces — summed **in that priority order**, adding as much of each
  component as fits under the thrust caps before moving to the next tier.
- **Their own stated comparison, real vehicle**: "Experimentally, this controller was far superior
  to even our best-tuned PID control systems." Tuning cost: LQR's Q/R cost-matrix tuning took
  **"only twenty minutes,"** versus **"many months"** for PID tuning on previous vehicles — the
  single most concrete tuning-effort number found in this entire pass, real vehicle, real
  competition team, direct quote.
- Vehicle class: **VideoRay M5 thrusters** (nominal max thrust 10 kg each), small hobbyist-class
  vehicle — closer to our own scale than most other sources in this pass, though still a
  6-thruster vectored (not tunnel) configuration and roll is presumably actuated (not confirmed).
- Sensors: VectorNav VN-100 IMU/compass at 800 Hz data rate, Teledyne Pathfinder DVL at 12 Hz.
  **Note the IMU/DVL rate mismatch (800 Hz vs 12 Hz)** — relevant context for our own "is 500 Hz
  justified" question from the companion sota/control.md dossier: even a competitive team running
  an 800 Hz-capable IMU only gets 12 Hz velocity truth from their DVL, i.e. the fusion/control
  loop is ultimately gated by the slowest real sensor, not the fastest.

**[REAL VEHICLE — NUS Bumblebee, RoboSub 2023 TDR, directly fetched and read]**
[NUS Bumblebee RoboSub 2023 Technical Design Report](https://robonation.org/app/uploads/sites/4/2023/06/TDR_NUS-Bumblebee_RS2023-compressed.pdf)
- Confirms the brief's claim: **6 PID loops** (surge, sway, heave, roll, pitch, yaw) plus a
  **QP thrust allocator** that "maintains control along each axis of motion even during thruster
  saturation" — i.e. graceful degradation under saturation is solved at the allocation layer here,
  vs. Caltech's priority-ladder-at-the-controller-output approach. Two different real, competitive
  answers to the same saturation problem.
- Uses a **"control law partitioning"** scheme: full-state-feedback controller for position/
  velocity tracking + a **separate feedforward controller to compensate nonlinear terms in the
  vehicle's motion dynamics** — structurally the same feedforward-alongside-feedback shape as our
  own (currently-zeroed) hydrodynamic feedforward layer.
- **Explicitly reported limiting factor, real vehicle, direct quote**: PID oscillation was reduced
  not by re-tuning gains but by **cutting communication latency** — moving firmware from polling
  loops to "interrupt-driven, RTOS-like code structure" reduced the controller-to-ESC latency, and
  the team states "less oscillatory behaviour was observed in our PID controller due to the faster
  feedback loop between controller and ESCs." **This is a real-team data point that loop latency,
  not loop rate per se, was their actual limiting factor** — directly relevant to the still-open
  Q1 in `control-laws-and-rate.md` about whether 500 Hz is justified: this team's fix was reducing
  jitter/latency in the existing loop, not raising the nominal rate.
- Vehicle class: large, well-funded (DVL $16,000, multibeam sonar $21,300) — **not size/cost
  transferable** to our 10–30 kg vehicle, but the qualitative control-architecture and
  limiting-factor findings above are still useful.
- No specific PID gain values, loop rate number, or depth-control numeric accuracy was found in
  the readable text extracted from this PDF (figures/diagrams likely carry numbers the text
  extraction did not capture — flagged as incomplete, not absent).

**Not reached this pass**: Stanford's 6-PID-loops-into-geometry-informed-allocation-matrix TDR
(cited in the brief as already known) — not independently re-verified or expanded on. USC and
CMU RoboSub 2025 TDRs surfaced in search but not opened — leads for a follow-on pass if more
team-specific attitude/depth numbers are wanted.

---

## Near-surface depth control (folded into Q2/Q3)

**[SIM ONLY, confirmed]** [Station-keeping of a ROV under wave disturbance — de Oliveira, Donha,
Fleury, de Barros, 2023](https://journals.sagepub.com/doi/10.1177/14750902221116673) — tested by
**numerical simulation only**, on a model called "Mandi II-ROV" — combines an Augmented Wave
Filter (EKF-based) with Adaptive-MPC, explicitly designed to filter the oscillatory wave signal
*out* of the feedback loop before it reaches the actuators, rather than trying to track through
it — offset-free control claimed for wave-induced depth variation. **No real-vehicle numbers.**
This is the clearest **architectural** answer found to "what does the literature say about the
top-metre problem": estimate/filter the wave motion explicitly (via an observer), don't feed it
straight into the depth PID.

**General/qualitative, not independently verified**: multiple sources (search-summary level)
describe near-surface AUV dynamics as dominated by **suction effects, partial emergence, and
increasing drag as wave height increases / depth decreases**, consistent with what our own
vehicle-spec doc already flags qualitatively. No new quantitative threshold (e.g. "stay below
wave-orbital-velocity X at depth Y") was found and confirmed this pass.
[Autopilot System for Depth and Pitch Control: Navigating Near-Surface Waves — arXiv 2402.03510](https://arxiv.org/html/2402.03510v1) —
combines LQR + L1 adaptive autopilot augmentation with filtering to keep wave disturbance out of
the actuator loop without hurting robustness — **not yet confirmed real-vehicle vs sim**, flagged
as a lead; the LQR+L1 combination is notable as a second real-world-flavored architecture
(L1 adaptive control has flight-tested lineage on aircraft) worth a closer read.

---

## Claims I could not verify this pass

1. **T200-class thruster dead-time (~0.59 s) / settling time (~2.95 s)** — carried over from
   `control-laws-and-rate.md`, still flagged there as needing a second read of the primary PDF
   table; repeating the flag here because it underpins the Q1 cross-reference above.
2. **DFKI "Necessity of Hydrostatic Stability in AUVs" paper (OCEANS 2022 SOS)** — 403 Forbidden,
   not read at all this pass despite being the single most on-point title for Q5.
3. **ADRC-SMC Mulan Lake field experiment (Journal of Field Robotics 2024)** — 403 Forbidden,
   real-vehicle claim not independently confirmed; treat the "remarkable performance" claim as
   unverified marketing language from the abstract, not a number.
4. **"Hierarchical disturbance rejection depth tracking...with experimental verification"
   (ScienceDirect)** — not fetched at all, abstract-level claim of experimental verification
   unconfirmed.
5. **"Up to 20 cm position drift under 1 knot current" comparator** cited secondhand inside the
   PMC9231013 extraction — the actual source of that 20 cm number was not identified or checked.
6. **Full-ocean-depth "residual buoyancy via thruster force equilibrium" paper** — title, authors,
   and venue not captured; found only as a search-engine paraphrase. Do not cite as a real source
   until the paper itself is located and read.
7. **Water Linked's "<1 cm over 30 min" DVL station-keeping tank test** — vendor marketing page,
   not a peer-reviewed source; treat as directionally indicative only, not a rigorous number.
8. **Stanford's 6-PID-loop geometry-informed allocation matrix** (from the task brief, said to be
   "already known") — not independently re-verified this pass; no Stanford TDR was opened.

