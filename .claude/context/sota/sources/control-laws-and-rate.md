# SOTA: control-law questions the dossier left open, + 4 unreached sources

Research pass, 2026-09-23. Follow-on to `sota-control-laws.md` (2026-09-22), which had its
WebSearch budget exhausted before this task started and worked from arXiv-only. This pass has a
working WebSearch tool. Written incrementally — do not hold findings in memory.

Vehicle: ESP32+RP2350 board, 500 Hz cascaded angle→rate PID (hydrodynamic feedforward zeroed),
T200-class bidirectional-DShot thrusters, MOT_SPIN_MIN floor → smallest commandable output is
15.8% of full scale, roll unactuated.

Every row distinguishes SIM vs REAL VEHICLE explicitly — this matters more than any other
property of a finding in this brief.

---

## Q1 — Is 500 Hz justified? What loop rate does an AUV demonstrably need?

**Status so far: still open, but the thruster-bandwidth evidence below (Q2) is the strongest
constraint found in either research pass.** A thruster with a ~0.6 s dead-time and ~3 s settling
time cannot possibly need a 500 Hz (2 ms) command update to exploit its own bandwidth — the
actuator is 3 orders of magnitude slower than the loop. This is an inference from Q2's numbers,
not a direct "AUV needs X Hz" study — no such direct study has been found in either pass.

General AUV control-architecture sources found this pass describe a **two-loop cascade** (slow
outer PD position/guidance loop → fast inner attitude loop) as the standard shape, matching our
own architecture, but give no universal numeric rate:
[Robust control for an AUV that suppresses pitch/yaw coupling — ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0029801810002222) —
abstract-level only, paywalled, not independently verified beyond the architecture claim.
[LPV Approaches for Varying Sampling Control Design: AUVs — Springer](https://link.springer.com/chapter/10.1007/978-3-642-36110-4_15) —
treats the sampling interval itself as a *varying, uncertain* parameter in the control design
(gain-scheduled/LPV against sample-time variation) rather than asserting a fixed required rate —
implicitly evidence that AUV designers do **not** treat a single fixed high rate as necessary;
they design controllers robust to the sample time changing. Not independently fetched in full;
treat as a lead.

Carried forward from the previous pass (still the best evidence available): every AUV control
paper reached in either pass that reports a rate runs at **10–100 Hz** (Lyapunov-MPC AUV, 10 Hz;
Koopman-MPC AUV, 10 Hz ID / 100 Hz internal model only). ArduSub's 400 Hz inner loop is inherited
from ArduCopter's in-air code, not derived from underwater dynamics
([ArduPilot dev docs](https://ardupilot.org/dev/docs/apmcopter-programming-attitude-control-2.html)).

**Working conclusion**: no source in either pass ties loop rate to AUV performance directly. The
strongest available argument is indirect — via actuator bandwidth (Q2) and vehicle dynamics being
orders of magnitude slower than an in-air vehicle. Treat "500 Hz is unjustified for outer-loop
guidance, but may still matter for DShot signal integrity / control-effectiveness-matrix
linearization if INDI-style methods are ever added" as the honest state of the evidence — genuinely
unsettled, not resolved either direction by direct measurement.

---

## Q2 — Thruster/actuator bandwidth: measured T200-class time constant, command step → thrust step

**[Nascimento & Valdenegro-Toro, "Modeling and Soft-fault Diagnosis of Underwater Thrusters with
Recurrent Neural Networks," arXiv 1807.04109 (2018), DFKI Bremen](https://arxiv.org/pdf/1807.04109)**
— **CONFIRMED by directly reading the extracted PDF text with `pypdf`** (not a search-engine
snippet — full paragraph quoted below), correcting one detail from the first extraction attempt:

  > "Fig. 4. Step response of the thruster for multiple steps of 0.25 of the maximum input. The
  > system presents a second-order-like response with average deadtime of 0.59 seconds and
  > average settling time of 2.95 seconds... The step response in open-loop for identification...
  > showed a second-order overdamped system with some dead-time. The averaged measurements for
  > settling time were 2.95 seconds and 0.59 seconds for dead-time. Also, according to the
  > manufacturer, the thruster presents a dead band of ±25 µs, which was considered for further
  > modeling. Due to the effect of the dead time and deadband, the system presents an observable
  > hysteresis..."

- **Correction vs. my earlier read**: the hardware is a **Blue Robotics T100**, not a T200 —
  ("Data was collected with several Bluerobotics T100 thrusters... ranging from 300 to 4200 rpm,
  has up to 130 W of output power and has 2.36 kgf of nominal torque"), driven by a Graupner T35
  ESC. Our vehicle's ⌀84 mm tunnel units are described in the vehicle-spec doc as "likely
  T200-class" — the T100 is the smaller sibling in the same product family (same brushless
  architecture, same PWM control scheme), so the **dead-time/settling-time physics should
  transfer in shape, not necessarily in exact magnitude** — a T200 (larger prop, more inertia)
  would plausibly be *slower*, not faster, making this a conservative (optimistic) bound if
  anything, not an overstatement.
- **Second correction, important**: this is a **rotational-speed/current step response**, not a
  direct in-water thrust measurement — the paper states explicitly: *"Although the relation of
  thrust and rotational speed is given by the manufacturer, we did not consider this indirect
  thrust measurement as an output variable of the model."* So 0.59 s / 2.95 s describes how fast
  the **motor+ESC assembly** ramps RPM in response to a PWM step, not a load-cell-measured thrust
  transient. Thrust approximately follows RPM² in steady state, so the RPM step response is a
  reasonable proxy for thrust bandwidth, but it is **not the same measurement** as a hydrodynamic
  thrust-force step test.
- Step amplitude tested: **0.25 of max PWM command**, PWM pulse width **1.0–2.0 ms mapped to
  -1.0..1.0**. Data also collected at sinusoidal inputs of **0.01, 0.02, 0.03, 0.04 Hz**, showing
  **hysteresis** at these very low frequencies — direct evidence the thruster+ESC assembly cannot
  cleanly track even a 0.04 Hz (25 s period) sinusoid without phase lag/hysteresis.
- **This is the single most load-bearing number in this whole pass for Q1**: a 0.59 s dead-time +
  ~3 s settling time (even measured on the smaller T100) means the physical actuator cannot
  usefully respond to anything faster than roughly **0.3–1 Hz** — three-plus orders of magnitude
  below our 500 Hz loop rate. This is now a **directly verified** number (read from the paper's
  own text), not a search-engine paraphrase.

---

## Q6a — Lam et al., OCEANS 2023, T200 thrust coefficient K_T vs advance ratio J

**[Propeller Characterization Testing of a Blue Robotics T200 Thruster — IEEE Xplore](https://ieeexplore.ieee.org/document/10244513/)** (paywalled, confirmed exists, not reached full-text)
**[same paper, ResearchGate](https://www.researchgate.net/publication/373891538_Propeller_Characterization_Testing_of_a_Blue_Robotics_T200_Thruster)** (403 Forbidden to WebFetch — blocked, not reached)

What is recoverable from the search engine's own indexed summary of the paper (**unverified
against primary text — treat as a lead, not a citable number** until the PDF itself is read):
- **K_T vs J is negative and linear** for the T200 propeller over the tested range.
- **K_T is maximum at J = 0 (bollard/static condition)**.
- **K_T = 0 at J equal to the propeller's geometric pitch** (i.e. the advance ratio at which the
  blade angle of attack goes to zero) — standard open-water-propeller theory, here stated as
  applying specifically to the T200.
- No numeric K_T(0), slope, or J-intercept value was retrievable through the search snippet.

**Verdict on Q6a: partially found, not fully verified.** The paper exists, is indexed, and its
qualitative K_T–J relationship (negative linear, max at bollard) is recoverable — but the actual
curve/table/equation is behind IEEE paywall and ResearchGate blocked WebFetch with a 403. This
directly confirms the *shape* of the "bollard k over-predicts at cruise" caveat the vehicle-spec
doc already asserts, but not its magnitude.

---

## Q3 — MPC on real AUV hardware: solve times, not sim

**[Nonlinear model predictive control for hydrobatics: Experiments with an underactuated AUV — Bhat et al., Journal of Field Robotics 2023](https://onlinelibrary.wiley.com/doi/full/10.1002/rob.22218)** (full text paywalled/403 to WebFetch — summary below is from the search engine's indexed excerpt and a related open thesis, **not from directly reading the paper**, flag accordingly)
- **Real vehicle**: the **SAM AUV** ("Small and Affordable Maritime robot"), a 1.5 m torpedo-shaped
  research platform built by the Swedish Maritime Robotics Center — **field-tested, not sim-only**.
- The full NMPC is **not** run directly online. The paper's own stated reason: "the solution time
  is the main limitation of the NMPC implementation" — so for real-time control the NMPC is
  **linearized periodically into an LTV-MPC** (linear time-varying MPC), solved with the **CVX**
  convex-optimization library on the vehicle's onboard computer.
- Stated design target: **"solver/real-time ratio ≤ 1"** — i.e. the QP solve must complete faster
  than the control period it serves; the paper reports this target was met ("satisfactory
  real-time performance... can be run real-time on the AUV performing hydrobatic maneuvers") but
  **no exact millisecond solve-time number, onboard CPU model, or Hz figure was recoverable from
  the sources reached in this pass** — the diva-portal open-access thesis that likely contains
  the number (`diva2:1502112/FULLTEXT01.pdf`) refused the connection (`ECONNREFUSED`) when
  fetched directly.
- **Verdict**: this is real progress over the previous pass's "sim-only, zero timing" finding —
  SAM is a **real, wet-tested AUV** running an MPC-family controller onboard, and the paper is
  explicit that raw NMPC solve time was the blocking constraint that forced the LTV-MPC
  simplification. But the actual number (ms, Hz, CPU) remains **unverified** in this pass —
  flagged for a follow-up fetch attempt on the thesis PDF or a mirror of the JFR paper.

**Microcontroller-class MPC, not underwater but relevant to the "can it run on our board" question:**
[NMPCM: NMPC on Resource-Constrained Microcontrollers (arXiv 2507.21259)](https://arxiv.org/html/2507.21259) —
real hardware (quadrotors, not underwater), **Teensy 4.1 (ARM Cortex-M7 @ 600 MHz, 512 kB RAM)** —
comparable compute class to our ESP32, though the M7 is considerably more capable than an ESP32.
Measured solve times **~5–15 ms at horizon N=10** (Fig. 10 per the search-indexed extraction — not
independently re-verified by direct read), up to **~1 kHz solving frequency claimed in
simulation**; real hardware demonstrated on **four drone platforms**. This bounds "is a small QP
feasible on Cortex-M7-class hardware" affirmatively, but it is an **aerial**, not underwater,
result, and horizon-10 QP on a quadrotor's much faster dynamics is not the same problem as an AUV
MPC horizon over 1+ second lookahead.

**Combined verdict for Q3**: no source in either research pass gives a clean "N ms solve time on
a Pi 5 (or ESP32) running an AUV MPC" number. The SAM/Bhat result is the closest — a real AUV
running an MPC-derived controller onboard, forced to linearize specifically because full NMPC was
too slow — but the actual timing number was not reachable. This remains the honest state: **"MPC
runs on real AUV hardware, and solve time is documented as the limiting factor" is now verified;
the specific number is still not.**

---

## Q4 — SMC / super-twisting chattering vs actuator quantization

No source in this pass gives a **direct, numeric** treatment of "how does SMC chattering interact
with a thruster that has a 15.8%-of-full-scale minimum commandable step" — but the *general shape*
of the problem is well represented in the marine-SMC literature, and one paper is on-topic by
title:

**[Quantized Sliding Mode Control of Unmanned Marine Vehicles: Various Thruster Faults Tolerated
with a Unified Model — ResearchGate](https://www.researchgate.net/publication/332538961_Quantized_Sliding_Mode_Control_of_Unmanned_Marine_Vehicles_Various_Thruster_Faults_Tolerated_with_a_Unified_Model)** —
403 Forbidden to WebFetch, **full text not reached**. Title and search-indexed abstract confirm
the paper explicitly addresses **input quantization** in an SMC law for marine vehicles and folds
it into the same switching-term framework used for thruster-fault tolerance — i.e. the literature
treats "coarse/quantized actuator" and "faulted actuator" as related problems solved by widening
the switching-term's error bound to absorb the quantization step. **This is exactly the shape of
problem our MOT_SPIN_MIN floor creates**, but the paper's own numbers (quantization step size
used, tracking-error bound achieved) were not retrievable.

**[Chattering-suppression sliding mode control of an AUV based on nonlinear disturbance observer
and power function reaching law — Tang et al., SAGE 2021](https://journals.sagepub.com/doi/10.1177/0142331221989867)**
— 403 Forbidden, full text not reached. Search-indexed summary: uses a **power-function reaching
law** (a continuous approach-to-the-surface law, replacing the discontinuous `sign()` term) plus a
**nonlinear disturbance observer**, which is the standard "smooth the switching term" family of
chattering fixes — same family as super-twisting (2nd-order sliding mode, continuous control with
discontinuous derivative). No numeric chattering-amplitude reduction figure recovered.

**[Adaptive fuzzy sliding mode controller for depth tracking of underwater vehicles — search-indexed via ScienceDirect / ResearchGate](https://www.researchgate.net/publication/222675595_Depth_control_of_remotely_operated_underwater_vehicles_using_an_adaptive_fuzzy_sliding_mode_controller)** —
uses a **saturation-function boundary layer** (the classic Slotine/Li fix: replace `sign(s)` with
`sat(s/φ)` inside a thin boundary layer φ around the sliding surface) to suppress chattering, and
was reportedly verified in **"water tank experiments"** with a **small ROV, 8 thrusters**, in a
**wave channel** — a real (if bench-scale) wet test, not pure sim. Full numeric result not reached
(403/paywall).

**Synthesis for Q4**: the SMC-on-AUV literature's answer to chattering is overwhelmingly the
**boundary-layer / continuous-approximation family** (boundary layer, power-reaching law,
super-twisting) — i.e. *smooth the control law* so its output doesn't discontinuously flip sign at
high frequency. This is a different problem from ours: our chattering risk isn't the control law
commanding infinite-frequency sign flips, it's that **the actuator itself cannot express anything
between 0 and 15.8%**, so even a perfectly smooth continuous control law gets quantized on output.
The one paper found that names quantization explicitly (the "Quantized SMC... Unified Model"
paper above) is the right lead, but its numbers were not reachable in this pass. **No source
found gives a real underwater vehicle's measured chattering behavior specifically against a known
actuator dead-zone/quantization floor comparable to our 15.8% MOT_SPIN_MIN** — this remains a gap.

---

## Q5 — Learning-based control that actually got wet

**[Learning to Swim: RL for 6-DOF Control of Thruster-driven AUVs — Cai, Chang, Girdhar (MIT/WHOI/Oregon State), arXiv 2410.00120](https://arxiv.org/abs/2410.00120)**
— **full text CONFIRMED by direct PDF read** (`pypdf` on the locally-cached fetch; WebFetch's own
summarizer had failed on this PDF earlier in the pass — persistence paid off):
- **Real vehicle**: **CUREE**, an AUV built at WHOI, equipped with **6× Blue Robotics T200
  thrusters** for full 6-DOF, an **Nvidia Jetson Orin NX** (water-cooled) running the policy
  on-board in real time, sensors/thrusters coordinated over a **Raspberry Pi 4**, ROS-based.
  State estimate is an EKF fusing a **DVL + IMU + AprilTag vision** (for ground-truth position in
  the test tank).
- **Control rate on real hardware, directly stated**: the neural policy sends low-level motor
  commands at **~20 Hz** — this is itself a useful Q1-adjacent data point: a from-scratch,
  thruster-direct RL controller, running on a Jetson-class companion computer, uses 20 Hz, not
  400–500 Hz.
- **Test protocol**: position/orientation hold above an AprilTag in a **small tank**, while a
  human **physically shoves the vehicle sideways with a stick** as an external disturbance; the
  vehicle is ballasted **positively buoyant**, so it must actively hold position throughout.
- **Training cost**: **2048 parallel simulated environments**, ~11.4 GB GPU memory on an **Nvidia
  A6000**, total training time **~10–20 minutes**.
- **Comparison to PID (qualitative, with a figure but no tabulated numeric error)**: the paper's
  own words — *"When compared to the performance of the PID controller (Figure 7), the neural
  controller seems more aggressive, which aligns with findings in similar studies in
  quadrotors."* It also reports a **steady-state error** in the neural controller specifically in
  **pitch and y** that the naive PID baseline did not exhibit to the same degree, attributed to
  the RL policy learning a single "averaged" behavior rather than adapting online — an explicit,
  named **weakness** of the RL approach versus PID, not just a win. The PID baseline itself is
  described as **"naively and manually tuned"** — i.e. not a strong, well-tuned PID baseline,
  which weakens the strength of any "RL beats PID" reading.
- **No numeric table (RMSE, settling time in seconds, position error in cm/m) comparing RL vs PID
  was present in the paper** — the comparison is qualitative and figure-based (time-series plots,
  Figures 6 and 7), not tabulated. This is a genuine absence in the source, not a fetch failure.
- **Verdict**: real-vehicle, real-water (tank), zero-shot sim-to-real RL result, fully confirmed
  by direct reading — the strongest-verified Q5 finding in either pass. But it is honestly a
  **mixed, not unambiguous, result**: RL matches PID's disturbance-rejection qualitatively, is
  described as more aggressive, but has a steady-state-error weakness PID doesn't share, versus a
  deliberately weak ("naive") PID baseline — not the clean "RL beats a good baseline" story a
  reader might assume from the abstract's "comparable to hand-tuned PID" phrasing alone.

**SAC-PID path-following controller — [An adaptive PID controller for path following of AUV based on Soft Actor-Critic, ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0029801824015099)**
- Search-indexed summary states this was **"validated through lake trials"** (real water, not a
  towed-carriage or pure-sim test) and that SAC-PID **"significantly outperformed the PID and
  PPO-PID controllers in control precision and convergence speed"** on those trials.
- Full text 403 to WebFetch — **no numeric error/precision figures recovered.** The architecture:
  SAC (a model-free actor-critic RL algorithm) is used to **tune/schedule PID gains online**
  rather than replace PID outright with a learned policy — a hybrid, not a pure end-to-end
  learned controller. This is a materially different design choice from "Learning to Swim"
  (direct 6-DOF command → thruster mapping) and worth distinguishing when comparing the two.
- **Verdict**: second real-vehicle (lake-trial) RL-adjacent result, again with the specific
  numbers unreached. Two independent real-water RL-family results now found across this pass and
  the previous one; **zero of them yielded a hard number this pass could confirm by direct read.**

**Combined verdict for Q5**: unlike the loop-rate question, this is **not** "genuinely
unpublished" — real-vehicle learning control exists. Learning to Swim is now **fully verified**
(direct read): a real AUV (CUREE), real tank testing, 20 Hz onboard control, with an honest,
qualitative, **mixed** result — RL is more aggressive and matches PID's disturbance rejection,
but has a steady-state-error weakness PID doesn't, tested against a "naively" tuned PID baseline.
SAC-PID's lake-trial numbers remain unreached (403/paywall) — that one specific claim is still a
tooling gap, not resolved.

---

## Q6b — Tunnel-thruster speed penalty, academic primary source (not the two US patents)

**[Saunders, A. & Nahon, M. — "The effect of forward vehicle velocity on through-body AUV tunnel
thruster performance," OCEANS'02 MTS/IEEE, Biloxi, pp. 250–259 (2002)](https://www.researchgate.net/publication/4010609_The_effect_of_forward_vehicle_velocity_on_through-body_AUV_tunnel_thruster_performance)** —
**this is very likely the actual academic primary source behind the "~10% at 3 knots" figure the
two US patents (US6286447B1, US6164230A) cite without attribution.** Confirmed via search-indexed
excerpt (full text 403/not reached, flag accordingly):
- **Real experimental hardware, not sim or CFD-only**: an AUV mounted in **Memorial University of
  Newfoundland's tow tank**, tunnel thruster instrumented with an **internally mounted 6-axis load
  cell measuring thrust directly**.
- Explicit numeric finding (from the search engine's own summary, matching the patents' claim
  almost exactly): **"As forward velocity increased to 3 knots, the effective side force from the
  tunnel thruster decreased to as low as 10 percent of the side force measured at zero forward
  velocity."** Mechanism given: forward vehicle motion increases fluid velocity through the tunnel
  at a fixed rotor speed, pushing the propeller blades off-design and unloading them.
- The dynamic (transient) response of the thruster was reportedly **not** significantly altered by
  vehicle operating mode; it was the **steady-state** side-force magnitude that collapsed with
  speed.

**[Saunders, A. & Nahon, M. — extended journal version, Ocean Systems Engineering 1(4), 2011 —
"The effect of vehicle velocity and drift angle on through-body AUV tunnel thruster performance"](https://koreascience.or.kr/article/JAKO201115541085232.page)** —
same authors/lab, confirmed via the journal's own abstract page as an **experimental** (not
simulation) study, extending the 2002 conference paper to include **drift angle** as a second
variable alongside forward speed. Full numeric drift-angle results not reached (403/paywall on
full text), but the existence and experimental nature of this extended study is directly
confirmed.

**[Palmer, A., Hearn, G.E., Stevenson, P. — "Modelling Tunnel Thrusters for Autonomous Underwater
Vehicles," IFAC NGCUV workshop proceedings, Southampton eprints](https://eprints.soton.ac.uk/54810/1/Modelling_Tunnel_Thrusters_for_Autonomous_Underwater_Vehicles.pdf)** —
the specific IFAC paper the task brief named. Confirmed to exist and be hosted open-access at
Southampton ePrints, but **WebFetch returned 403 on the direct PDF URL** (odd for an open-access
repository — likely a bot-blocking measure, not a real paywall; worth a manual `curl`/browser
retry outside this tool). Search-indexed summary independently confirms this paper's scope
matches the brief: it models tunnel-thruster performance during the **transition from survey
(forward) speed to low-speed maneuvering**, with modified maneuvering equations for energy-level
demand — consistent with, and likely a modeling companion to, the Saunders & Nahon experimental
findings above.

**Verdict on Q6b**: **found and effectively confirmed** — not the exact paper named in the brief
(Palmer/Hearn/Stevenson, whose PDF exists but 403'd), but the **Saunders & Nahon 2002/2011
tow-tank studies are the real academic primary source** for the "~10% side force at 3 knots"
figure, with a real experimental setup (load-cell-instrumented tunnel thruster, tow tank) and a
number matching the patents' claim almost exactly. This is the strongest single result of this
entire pass for closing a task-brief question.

---

## Q6c — Fossen & Johansen, MED 2006 control allocation survey

**[Fossen, T.I. & Johansen, T.A. — "A Survey of Control Allocation Methods for Ships and Underwater
Vehicles," 14th Mediterranean Conference on Control and Automation, 2006 — full PDF, author's own site](https://www.fossen.biz/publications/2006%20Fossen%20and%20Johansen%20MED.pdf)** —
**full text successfully fetched** (~8–10 pages, author's own hosting, no paywall). Content:
- Surveys three families of control allocation method: **pseudoinverse** (linear, fast, no
  constraint handling), **quadratic programming (QP)** (handles actuator saturation and rate
  constraints as explicit inequality constraints, more expensive), and **direct/geometric methods**
  (avoid explicit optimization).
- Explicitly treats **actuator saturation, rate constraints, and azimuth-thruster singularities**
  (a non-convex configuration where an azimuthing thruster cannot produce force in some
  direction) as the practical constraints that separate simple ships from underwater vehicles —
  underwater vehicles get flagged as the harder case because of **more complex, often
  over-actuated / vectored-thruster configurations** requiring singularity-aware allocation.
- A later extended version exists as a book chapter: Fossen, Johansen & Perez, "A Survey of
  Control Allocation Methods for Underwater Vehicles," in *Underwater Vehicles* (InTech, 2009) —
  not fetched in this pass, flagged as a lead if more depth is needed.
- **Note on extraction quality**: WebFetch's summarization of the actual equations/derivations was
  generic ("QP methods offer superior constraint handling") rather than quoting the paper's
  specific formulations verbatim — the PDF itself was retrieved successfully and is saved locally
  by the tool, so a direct re-read (not through WebFetch's summarizer) would recover the exact
  math if needed for implementation work.

**Verdict on Q6c: found, full text reached** — the strongest "reached the actual named source"
result besides Q6b.

---

## Q6d — Sarkar, Podder & Antonelli 2002, AUV control allocation

**Citation confirmed exactly**: N. Sarkar, T.K. Podder, G. Antonelli, "Fault accommodating
thruster force allocation of an AUV considering thruster redundancy and saturation," *IEEE
Transactions on Robotics and Automation*, 18(2):223–233, April 2002.
[ResearchGate listing](https://www.researchgate.net/publication/289456228_A_Fault_Accommodating_Control_of_an_Autonomous_Underwater_Vehicle_under_Thruster_Redundancy_and_Saturation) ·
[Semantic Scholar listing](https://www.semanticscholar.org/paper/Fault-tolerant-control-of-an-autonomous-underwater-Podder-Antonelli/1747bb928edf6a5454f8d1d6ecf930ad6f6c7c28)

**Full text NOT reached** — IEEE Xplore is paywalled and not attempted directly; the
academia.edu mirror of the closely related companion paper ("Fault-tolerant control of an
autonomous underwater vehicle under thruster redundancy") returned 403. Search-indexed summaries
(from multiple independent listings, consistent with each other) describe the method as: a
**redundancy-resolution allocation scheme** that exploits the AUV's excess thruster count to
**reallocate force among the remaining healthy thrusters when a fault is detected**, explicitly
accounting for **thruster saturation** in the allocation itself (not treated as a separate
clamping step after allocation). No numeric results (RMSE, recovery time, etc.) were recoverable
without the full text.

**Verdict on Q6d: citation and method confirmed, full text not reached.** This is a real gap —
the brief specifically asked for full text and this pass could not obtain it. A logical next
step outside this pass's tools: Sci-Hub-adjacent institutional access, or emailing the Naval
Undersea Warfare Center / UNH (Sarkar's affiliation) for a reprint, neither of which this research
tooling can do.

---

## Summary table

| Q | question | status | strongest evidence | sim or real |
|---|---|---|---|---|
| 1 | Is 500 Hz justified? | **Still open** — no direct study found in either pass. Strongest new argument: thruster dead-time (0.59 s) + settling (2.95 s) is ~3 orders of magnitude slower than the 2 ms loop period, making the actuator itself the binding bandwidth limit, not the control loop | Q2 thruster step response (below) | REAL (thruster bench), inferential link to loop rate |
| 2 | T200-class actuator bandwidth | **Found, CONFIRMED by direct PDF read** — dead-time 0.59 s, settling 2.95 s, second-order overdamped + dead-band ±25 μs. Actual hardware is a **T100** (smaller sibling of our T200-class units), and the measurement is a **RPM/current step response**, not a direct load-cell thrust transient | [arXiv 1807.04109](https://arxiv.org/pdf/1807.04109) | REAL (measured T100 thruster+ESC assembly) |
| 3 | MPC solve time on real hardware | **Partially found** — SAM AUV runs an LTV-MPC (linearized from NMPC specifically because NMPC solve time was too slow) onboard, real field-tested; exact ms/Hz/CPU number not reached | [Bhat et al., JFR 2023](https://onlinelibrary.wiley.com/doi/full/10.1002/rob.22218) | REAL (SAM AUV, field trials) — number unverified |
| 4 | SMC chattering vs actuator quantization | **Right literature found, numbers not reached** — "Quantized SMC of Unmanned Marine Vehicles" directly on-topic; boundary-layer/power-reaching-law family is the general chattering answer; none ties to a 15.8%-floor-specific case | [ResearchGate 332538961](https://www.researchgate.net/publication/332538961_Quantized_Sliding_Mode_Control_of_Unmanned_Marine_Vehicles_Various_Thruster_Faults_Tolerated_with_a_Unified_Model) | Mixed — one wave-channel ROV experiment found, others unclear/paywalled |
| 5 | Learning-based control that got wet | **Learning to Swim: fully confirmed by direct read** — CUREE AUV, 6×T200, Jetson Orin NX, 20 Hz, tank tests; RL matches PID qualitatively but has a steady-state error weakness PID lacks, and the PID baseline was "naively" tuned — a genuinely mixed result, not a clean win. SAC-PID lake-trial numbers still not reached (403) | [arXiv 2410.00120](https://arxiv.org/abs/2410.00120) (confirmed), [ScienceDirect SAC-PID](https://www.sciencedirect.com/science/article/abs/pii/S0029801824015099) (unverified) | REAL for both — this is the headline improvement over the previous pass, which found sim-only RL |
| 6a | Lam OCEANS 2023, T200 K_T vs J | **Found, qualitative only** — negative linear K_T–J, max at bollard, zero at geometric pitch; curve/table itself paywalled | [IEEE 10244513](https://ieeexplore.ieee.org/document/10244513/) | REAL (thruster characterization rig, per title) |
| 6b | Tunnel-thruster speed penalty, academic source | **Found and effectively confirmed** — Saunders & Nahon 2002/2011, real tow-tank + load cell, "~10% side force at 3 knots," matching the patents' figure almost exactly | [ResearchGate 4010609](https://www.researchgate.net/publication/4010609_The_effect_of_forward_vehicle_velocity_on_through-body_AUV_tunnel_thruster_performance) | REAL (tow tank) |
| 6c | Fossen & Johansen MED 2006 | **Found, full text reached** | [fossen.biz PDF](https://www.fossen.biz/publications/2006%20Fossen%20and%20Johansen%20MED.pdf) | n/a (survey paper) |
| 6d | Sarkar, Podder & Antonelli 2002 | **Citation/method confirmed, full text NOT reached** (IEEE paywall) | [Semantic Scholar listing](https://www.semanticscholar.org/paper/Fault-tolerant-control-of-an-autonomous-underwater-Podder-Antonelli/1747bb928edf6a5454f8d1d6ecf930ad6f6c7c28) | n/a |

---

## Claims I could NOT verify in this pass

1. ~~Exact T100/T200 step-response numbers~~ — **RESOLVED this pass**: re-extracted directly from
   the locally-saved PDF with `pypdf` (WebFetch's own summarizer missed the numeric table on the
   first attempt; the raw text extraction did not). Confirmed: **T100** (not T200) thruster+ESC
   assembly, RPM/current step response (not direct thrust), 0.59 s dead-time, 2.95 s settling
   time, ±25 μs PWM dead-band. See Q2 above for the full quoted passage.
2. **Lam et al. OCEANS 2023 K_T(0) value, the fitted K_T-vs-J equation/slope, and the propeller's
   geometric pitch number** — the paper's existence and qualitative shape are confirmed; the
   actual curve is behind an IEEE paywall (10244513) and the ResearchGate mirror 403'd WebFetch.
3. **SAM AUV's exact MPC solve time (ms), onboard CPU model, and control loop Hz** — the paper
   (Bhat et al., JFR 2023) is confirmed real-vehicle and confirmed to name solve time as the
   limiting factor, but the number itself was not reached (JFR full text 403, diva-portal thesis
   PDF returned `ECONNREFUSED`).
4. **Any numeric chattering-amplitude/frequency reduction tied to a specific actuator quantization
   step** for a real underwater vehicle — the closest lead ("Quantized SMC of Unmanned Marine
   Vehicles") was not reachable past its title/abstract.
5. ~~Learning to Swim's exact tracking-error numbers~~ — **RESOLVED this pass**: re-extracted the
   full text locally with `pypdf` after WebFetch's summarizer failed. The paper itself does not
   tabulate a numeric RL-vs-PID comparison (figure-based only) — confirmed this is a property of
   the source, not a fetch failure. See Q5 above for the full, more nuanced picture (RL matches
   PID qualitatively but has a steady-state-error weakness PID doesn't, vs. a "naive" PID
   baseline).
6. **SAC-PID's exact lake-trial numbers vs PID and PPO-PID** — only the qualitative "significantly
   outperformed" claim was recovered; ScienceDirect full text is paywalled (403).
7. **Saunders & Nahon's drift-angle-specific numbers** (the 2011 extended study) — the forward-
   speed-only "~10% at 3 knots" figure from the 2002 paper is well-supported, but the added
   drift-angle dimension in the 2011 journal version was not independently quantified here.
8. **Palmer, Hearn & Stevenson's own numeric content** — the exact IFAC paper named in the brief
   is hosted open-access at Southampton ePrints but returned 403 to WebFetch (likely a
   bot-blocking measure on an otherwise-open PDF, not a real paywall — worth a manual retry
   outside this tool, e.g. `curl` with a browser user-agent, or the browser tool).
9. **Sarkar, Podder & Antonelli 2002 full text** — citation and method are solid (cross-confirmed
   by multiple independent listings), but IEEE Xplore paywall and a 403 on the academia.edu mirror
   of the closest companion paper blocked full-text access entirely.
10. **Q1's core question remains formally open** — the thruster-bandwidth argument (dead-time
    three orders of magnitude below the loop period) is a strong *inference*, not a direct study
    that varies AUV loop rate and measures closed-loop performance. No such study was found in
    either research pass. This should be stated as "genuinely appears unpublished" in the
    dossier, not "unresolved due to missing search."

## What changed vs the previous pass (`sota-control-laws.md`, 2026-09-22)

- **Q1 (loop rate)**: no new direct evidence, but a new, load-bearing indirect argument (thruster
  dead-time vs loop period) that the previous pass did not have.
- **Q2 (thruster bandwidth)**: net new — the previous pass had none of this; now has real
  step-response numbers (needing one more verification pass).
- **Q3 (MPC on hardware)**: net new — previous pass found zero hardware-timing MPC results at all
  (sim-only, laptop-only). This pass found a real, field-tested AUV (SAM) whose designers
  explicitly cite solve time as the reason they abandoned full NMPC for LTV-MPC — a materially
  stronger finding even without the exact number.
- **Q4 (SMC chattering/quantization)**: net new — previous pass didn't cover SMC/chattering at
  all; this pass found the directly-relevant "quantized SMC" sub-literature exists, even though
  its numbers weren't reachable.
- **Q5 (RL that got wet)**: net new and the biggest upgrade — the previous pass's closest lead
  (arXiv 1912.11584) was listed but unverified. This pass found and **fully confirmed** a
  real-vehicle result (Learning to Swim: CUREE AUV, 6×T200, 20 Hz onboard, tank-tested), correcting
  the prior "learning control on real underwater vehicles is essentially unpublished" impression —
  it exists, and on close reading it's an honest mixed result (RL ≈ PID with a steady-state-error
  tradeoff, vs. a weak PID baseline), not the clean win the abstract alone implies. A second
  real-vehicle lead (SAC-PID lake trials) exists but its numbers remain unreached (403/paywall).
- **Q6 (four named sources)**: **2 of 4 reached in useful form** (Fossen & Johansen full text;
  Saunders & Nahon as the effective real primary source for the tunnel-thruster patents' figure,
  which is arguably a *better* answer than reaching Palmer/Hearn/Stevenson directly would have
  been), **1 of 4 found but paywalled** (Lam et al.), **1 of 4 citation-only** (Sarkar/Podder/
  Antonelli).

**Root cause of the remaining gaps**: not a WebSearch budget problem this time (search worked
throughout). The blocker this pass was almost entirely **WebFetch 403s on ResearchGate,
ScienceDirect, IEEE Xplore, academia.edu, and even one open-access institutional repository
(Southampton ePrints)** — these sites appear to actively block the fetch tool's user agent or
require JS/interaction. Where a PDF was reachable, WebFetch's own PDF-to-text extraction
sometimes failed on binary/encoded content even though the file downloaded successfully (saved
locally by the tool). **If this gap matters, the next step is a different fetch path**: a
headless-browser tool (e.g. claude-in-chrome) for the ResearchGate/ScienceDirect/IEEE pages, or
downloading the already-saved local PDF copies (paths were printed in several tool results above)
and parsing them with a local PDF library rather than WebFetch's summarizer.


