# SOTA sweep — per-thruster RPM telemetry: closed-loop control, thrust normalisation, fault detection

Scope: what the firmware's `thrust_trim` (slow RPM-based thrust normalisation) and `rpm_mode`
(PI on shaft speed) should look like in light of the published marine-thruster-control
literature, ahead of turning on real bidirectional-DShot eRPM telemetry on Bluejay ESCs.

Status legend: **REAL-VEHICLE/REAL-HARDWARE** vs **SIM/MODEL-ONLY**. Every claim carries a URL.

---

## 1. Is shaft-speed (RPM) control good for a thruster in dynamic conditions?

### Source: Sørensen, Smogeli & Ruth (2009), "Propulsion Control Strategies for Fixed Pitch
Propellers at Low Advance Speed", 1st Int. Symposium on Marine Propulsors (smp'09), Trondheim.
PDF: https://www.marinepropulsors.com/proceedings/2009/MB2-2-Sorensen%20-%20Propulsion%20Control%20Strategies%20for%20Fixed%20Pitch%20Propell.pdf
**REAL-HARDWARE** (physical ducted propeller rig, MCLab basin at NTNU, 40 m × 6.45 m × 1.5 m,
towing carriage + wave-maker; D=0.25 m propeller, KT0=0.513/0.570, KQ0=0.0444/0.075 depending on
duct).

This is a comparison of shaft-speed control, torque control, power control and a combined
torque/power controller — the exact three/four control modes the firmware's header is
implicitly choosing between.

**Quasi-static test (Tr = 100 N constant thrust reference, advance velocity Va swept
0 → 1.5 m/s, 54 runs, repeatable):**
- Shaft-speed control: thrust *collapses* from 100 N → **18 N** at Va = 1.5 m/s (an 82% loss).
- Torque control: thrust holds 100 N → **65 N** at the same advance speed (35% loss).
- Power control: thrust holds 100 N → **50 N** (50% loss).
- Quote: "the effective angle of attack of the propeller blades is decreased, and the propeller
  loading decreases for a constant shaft speed" — a fixed-RPM setpoint does **not** track thrust
  when the inflow (advance velocity) changes, because thrust depends on advance ratio, not just
  RPM².

**Dynamic test in regular waves (34 tests, wave height 8 cm, period 1 s, thrust ref 90 N,
carriage stationary):**
- "The shaft speed controller keeps the shaft speed constant, and has to vary the motor torque
  and power in order to achieve this. **The resulting propeller thrust and torque have the
  largest variance.**"
- "The torque controller keeps the motor torque constant... **the resulting propeller thrust
  and torque have the smallest variance.**"
- "The power controller... lie between the shaft speed and torque controller values."

**Conclusions section, verbatim finding:** "The conventional shaft speed controller gave the
thrust, torque, and power with the largest variance, and it was the least robust to
disturbances in the in-line flow velocity. The torque controller produced the thrust and torque
with the smallest variance... The combined torque and power controller gave the overall best
improvement in the performance from low to high loadings."

**Verdict on the firmware's claim** ("shaft-speed control...documented to induce thrust
oscillation in dynamic conditions"): **VERIFIED**, with a caveat — this literature is
surface-vessel/DP-thruster-in-waves, driven by *time-varying inflow* (waves, advance velocity),
not by the AUV's own attitude disturbance the firmware comment describes (a 1° pointing error).
The mechanism differs (inflow disturbance vs. commanded-torque disturbance from an outer
attitude loop) but the qualitative result is the same family of problem: **a pure shaft-speed
setpoint is provably the worst of the three basic strategies for holding thrust constant under
disturbance, and it is the industry-documented reason DP vessels moved away from pure RPM
control.** Text: "For surface vessels with FPP, shaft speed control is the industry standard,
whereas torque and power control was introduced by Sørensen et al. (1997)... For underwater
vehicles, both torque and various shaft speed control schemes have been proposed, see Yoerger
et al. (1991), Whitcomb and Yoerger (1999b), Fossen and Blanke (2000)."

Also relevant to ventilation/fault behavior (§4 below): "For a heavily loaded propeller,
ventilation may lead to an abrupt loss of thrust and torque as high as **70–80%**... a reduction
of shaft speed in such a case may increase the thrust, and an increase in shaft speed will not
increase the thrust" — i.e. naive shaft-speed control can actively make thrust *worse* during a
fault by spinning up into ventilation ("propeller racing"), which is why they build a dedicated
anti-spin controller.

---

## 2. Whitcomb & Yoerger's thruster-control work specifically

Two companion 1999 papers, IEEE Journal of Oceanic Engineering, Vol. 24, No. 4 (October 1999):

1. Whitcomb, L.L. & Yoerger, D.R., **"Development, comparison, and preliminary experimental
   validation of nonlinear dynamic thruster models,"** pp. 481–494.
   DOI: 10.1109/48.809270. IEEE Xplore: https://ieeexplore.ieee.org/document/809270/
2. Whitcomb, L.L. & Yoerger, D.R., **"Preliminary experiments in model-based thruster control
   for underwater vehicle positioning,"** pp. 495–506.
   Citation record: https://www.researchgate.net/publication/224635339_Preliminary_thruster_control_experiments_for_underwater_vehicle_positioning

**Status: PARTIALLY VERIFIED — abstract-level only.** Both papers are paywalled (IEEE Xplore
403, academia.edu 403 to this agent); only abstracts/citations were retrievable. What is
confirmed from secondary citation (Sørensen et al. 2009 above, which cites both by name as the
underwater-vehicle precedent for "both torque and various shaft speed control schemes"):
- Paper 2's abstract (via search snippet, ResearchGate citation record) describes **"comparative
  experiments with two novel and one conventional thrust control algorithms for the unsteady
  (transient) control of thrust"** on a real bladed-propeller marine thruster — i.e. this is a
  REAL-HARDWARE bench comparison, not simulation, and it explicitly frames pure open-loop/
  conventional control as the baseline being beaten by two "novel" (model-based) approaches.
- Both papers are cited by name, together with Yoerger, Cooke & Slotine (1991) and Fossen &
  Blanke (2000), as the foundational underwater-vehicle-specific thruster control literature —
  this triad (Yoerger 1991, Whitcomb & Yoerger 1999, Fossen & Blanke 2000) is what any AUV
  thruster-control design should be checked against.
- **Could not verify**: the actual numeric results, the identity of the "two novel" algorithms,
  or whether they used torque, voltage, or model-based feedforward. This is the single biggest
  gap in this sweep — flagged in §8 below.

---

## 3. Companion model: Bachmayer, Whitcomb & Grosenbaugh (2000)

**"An Accurate Four-Quadrant Nonlinear Dynamical Model for Marine Thrusters: Theory and
Experimental Validation,"** IEEE J. Oceanic Engineering, Vol. 25, No. 1, pp. 146–159.
Semantic Scholar: https://www.semanticscholar.org/paper/An-accurate-four-quadrant-nonlinear-dynamical-model-Bachmayer-Whitcomb/1870cd7a3a5fc936371ff9edfff4dbaec4f0e63a

**REAL-HARDWARE** (bench thruster rig, force/torque/fluid-velocity instrumented). Two specific
contributions over prior four-quadrant models: (1) incorporates **rotational fluid velocity and
inertia** effects on thruster response (prior models used thin-airfoil theory with only axial
flow), (2) a new method for experimentally determining **non-sinusoidal** lift/drag curves
(prior four-quadrant models assumed sinusoidal lift/drag, which the paper shows is inaccurate).
Validated against real transient and steady-state thruster data; the enhanced model "provide[s]
superior accuracy in both transient and steady-state responses" vs. the simpler four-quadrant
models it supersedes.

Relevance to us: this is the accepted reference dynamical model for spin-up/spin-down and
quadrant-reversal behavior (§5) — any RPM-based control law or fault detector should be checked
against a model of this shape, not the naive `thrust = k·n²` static map alone.

---

## 4. thrust ~ k·n² — how good, and how much does k vary with advance ratio / voltage?

**Advance-ratio dependence (dominant effect, and the one the firmware does NOT claim to fix):**
Classical propeller theory: T = KT·ρ·n²·D⁴, but **KT is a function of advance ratio
J = Va/(nD)**, not a constant. Source: https://www.sciencedirect.com/topics/engineering/propeller-characteristic
and https://www.sciencedirect.com/topics/engineering/advance-coefficient — "the thrust
coefficient for a ship's propeller is a function of the advance ratio, Reynolds number, and
cavitation number, but over a wide range of operating conditions, Reynolds number and
cavitation number have relatively little influence." J is "the primary non-dimensional
parameter describing propeller operating condition." This is exactly what the Sørensen et al.
(2009) bench data (§1) shows quantitatively: **at constant n commanding 100 N bollard thrust,
real thrust falls to 18 N at Va = 1.5 m/s** — an 82% loss, entirely a J effect, nothing to do
with battery voltage. **This confirms the note already in our task brief that "bollard-only
data over-predicts thrust at speed"** — it is not a minor correction, it is the dominant
uncertainty in any RPM→thrust map for a vehicle that is actually moving (which a 500 Hz
depth/attitude controller will be, continuously).

**Voltage/Reynolds-number dependence (what `thrust_trim`'s comment actually claims):** at FIXED
advance ratio (e.g. bollard, J≈0 — which is the regime `thrust_trim`'s comment describes:
"same PWM, different battery voltage"), the physics argument is sound: KT depends only weakly
on Reynolds number, and Reynolds number effects are second-order relative to J. Source (general
propeller-affinity literature via search): "while thrust coefficient does vary with rotor speed
due to Reynolds number effects... it exhibits relatively weak dependence over a wide range of
operating conditions." This is textbook propeller-similarity (affinity-law) theory (Fossen's
Handbook of Marine Craft Hydrodynamics and Motion Control treats KT, KQ as functions of J with
Reynolds-number corrections as a secondary term) — so the *shape* of the firmware's claim
("thrust/RPM² is ~invariant across the 12–20 V range, because that range only changes n, not
J") is **physically well-founded and consistent with classical propeller theory.**

**Could NOT verify the specific number.** The firmware claims "measured thrust/RPM² invariant
to ~3% across the whole 12–20 V range." Blue Robotics' own public T200/T500 performance data
(bluerobotics.com/learn/thruster-usage-guide/, github.com/bluerobotics/bluerobotics.github.io)
publishes thrust/current/power **vs. PWM at various voltages**, but does **not publish RPM** in
its public charts — so the firmware's 3% figure cannot be cross-checked against a public Blue
Robotics dataset; it must come from the team's own bench instrumentation (a real RPM sensor on
a T200-class thruster), which this sweep could not independently access. Flagged for
verification against our own bench log rather than literature. A peer-reviewed characterization
study exists — Propeller Characterization Testing of a Blue Robotics T200 Thruster, IEEE
Xplore https://ieeexplore.ieee.org/document/10244513/ — but it was paywalled to this agent
(ResearchGate mirror also 403'd); title and existence confirmed, contents not verified.

---

## 5. Thruster dynamics: time constant, spin-up/down asymmetry, four-quadrant behavior

Source: Custódio, Brandão et al. (exact author list unconfirmed from the fetched PDF header —
title readable), **"Modeling and Soft-fault Diagnosis of Underwater Thrusters with Recurrent
Neural Networks,"** arXiv:1807.04109. https://arxiv.org/pdf/1807.04109 (IFAC workshop paper).
**REAL-HARDWARE** — "several Bluerobotics T100 thrusters" driven by a Graupner T35 ESC (300–4200
RPM range, 130 W, 2.36 kgf nominal thrust), instrumented for rotational speed, current, voltage,
temperature.

**Step response (0.25 of max input steps):** "The system presents **a second-order-like
response with average deadtime of 0.59 seconds and average settling time of 2.95 seconds**...
showed a second-order overdamped system with some dead-time." This is a *slow* thruster —
deadtime of ~0.6 s alone is far slower than any reasonable attitude-loop bandwidth, which is
exactly the mismatch the firmware's comment describes ("thruster dynamics are a slow nonlinear
lag").

**Deadband:** "the thruster presents a dead band of ±25 µs" (per manufacturer spec), which
combined with the dead-time "presents an observable hysteresis" in the open-loop
control→response map — i.e. a naive static thrust/RPM lookup will show hysteresis between
increasing and decreasing commands, not just a symmetric nonlinearity.

**Could NOT verify a numeric spin-up vs. spin-down asymmetry** (this paper reports averaged
deadtime/settling time across "multiple steps," not separated by direction) or a four-quadrant
transient model specific to a T-series thruster. The accepted reference *model* for the
asymmetric, quadrant-dependent transient (not the T100-specific numbers) is Bachmayer, Whitcomb
& Grosenbaugh 2000 (§3) — it models exactly this rotational-fluid-inertia asymmetry, but the
paper's own numeric results were not accessible to this agent.

---

## 6. RPM-based fault detection / FDI — what's detectable from what signal

Same source as §5 (arXiv:1807.04109), which is directly on point: this is an underwater-thruster
FDI study, real T100 hardware, six labeled operating conditions (nominal at 15.0 V / 13.0 V /
11.8 V, one broken propeller blade, two broken blades, and a propeller "impregnated with
silicon to simulate biofouling").

**Two feature sets compared, both classified with MLP and LSTM:**
1. Raw signals: control input + voltage + rotational speed + current (4 features).
2. **Residuals** (measured RPM and current minus a NARX nominal-model prediction) — i.e. RPM
   and current compared against an expected model, not used raw.

**Result: "the use of computed residuals (rotational speed and current) as features for
classification lead to improved results compared to a full four-feature vector"** — the
model-residual approach beat raw-signal classification. Best classifier (MLP on residuals) —
**78% average accuracy on an 11,200-sample held-out test set, 3-fold time-series
cross-validation.** Explicit finding: **"identifying nominal operation is confused with a
broken propeller... biofouling is also hard to identify, with typical confusion with nominal
operation."** The paper's own conclusion: "we only achieve 78% accuracy on our dataset, these
results also show that the problem of classifying soft-faults is hard, and there is lots of
room for improvement."

**Verdict for our design:** this is real, if modest, evidence that (a) **RPM alone is not
sufficient** — every configuration tested used RPM *and* current together, never RPM alone,
because a broken-blade fault mostly looks like a low-torque-load change that plain RPM under
closed-loop control will suppress; (b) **soft faults (biofouling, one broken blade) are
genuinely hard** to tell apart from a benign low-voltage state even with both signals — 78%
accuracy on a controlled 6-class bench problem is a low bar, and our board will not have a
cleaner signal underwater; (c) hard faults (open circuit, stalled shaft) are the ones RPM alone
detects well — those are the "shaft speed is very low/zero" case.

**Separate finding on hard-fault edge cases:** a different search result (unverified beyond the
abstract snippet — Chinese-language underwater-thruster FDI literature, not independently
fetched) states that "when thruster shaft speed is very low or doesn't rotate, reliable fault
detection is impossible; the solution involves excluding critical zones from FDI... where the
algorithm enters sleep mode and outputs an 'Invalid' state." **Flagged as claims-I-could-not-
verify** — this is a search-engine paraphrase of a paper this sweep did not open directly, so
treat as a hypothesis to check against a primary source before relying on it, but it matches
intuition: a stalled thruster reads RPM≈0 regardless of whether the fault is "stuck" or
"unpowered," so RPM alone cannot distinguish those two hard-fault modes — current (near-zero
for unpowered, high/stalled-current for a mechanically jammed shaft) is what disambiguates
them. This is exactly the case in our task brief where "we will have RPM AND may build
per-thruster current sensing" — the literature says current is not optional for fault
*classification*, only for fault *presence* detection does RPM alone suffice.

---

## 7. eRPM telemetry quality on bidirectional DShot / Bluejay

**REAL-HARDWARE / protocol-spec facts**, not a controlled study — this section is protocol
documentation, not a measurement paper. No source found gave hard latency/noise numbers from a
bench test; flagged accordingly.

- Bluejay is "an open source successor to BLHeli_S... supports DShot 150, 300 and 600 digital
  signal protocols, **bidirectional DShot with RPM telemetry**, selectable PWM frequency of 24,
  48 and 96 kHz, and PWM dithering for 11-bit effective throttle resolution." Also implements
  **Extended DShot Telemetry (EDT)**, created by the Bluejay team, "allows ESCs without a
  separate telemetry UART to send additional telemetry alongside RPM data" (current, voltage,
  temperature, per other sources — not independently confirmed which fields EDT carries on
  Bluejay specifically). Source: https://github.com/mathiasvr/bluejay (now maintained at
  https://github.com/bird-sanctuary/bluejay).
- Mechanism (general bidirectional-DShot spec, confirmed independently by two sources — INAV PR
  #11605 https://github.com/iNavFlight/inav/pull/11605 and Bitcraze's Crazyflie writeup
  https://www.bitcraze.io/2026/06/bidirectional-dshot-and-erpm-telemetry-for-the-crazyflie-2-1-brushless/):
  "After each DShot output frame the motor pin is switched to timer input capture mode via
  per-channel DMA to receive the ESC's GCR-encoded eRPM response" — i.e. **eRPM is returned
  once per DShot command frame, on the same wire, with no separate UART needed.** This means
  the telemetry rate is tied to the command rate (potentially hundreds of Hz to kHz), a
  qualitatively different regime from UART-based ESC telemetry.
- ArduPilot's own docs give the one hard comparative number found: "the maximum rate that can
  be sustained [for legacy UART-based] ESC Telemetry is about 100Hz," whereas the default
  harmonic-notch-filter update is 200 Hz and **"bi-directional dshot with ESC Telemetry
  reporting of RPM"** is explicitly called out as faster/more responsive than the UART path —
  ArduPilot's harmonic notch literature treats bidir-DShot RPM as good enough to drive a 200 Hz
  control-adjacent filter loop. Source:
  https://ardupilot.org/copter/docs/common-esc-telem-based-notch.html . This is consistent with
  (does not contradict) the firmware's own choice to run `thrust_trim` at ~10 Hz — 10 Hz is
  comfortably inside what bidir-DShot telemetry can sustain, so the bottleneck in the firmware's
  design is deliberately the *control law* (avoid fighting the 500 Hz attitude loop), not a
  telemetry-rate limit.
- **Could not verify**: quantization/resolution of the eRPM value itself (bidirectional DShot
  encodes an e-period, i.e. time between commutations, GCR-encoded — quoted elsewhere as giving
  frequency resolution "to within 1 Hz" for FPV drone motors, but that figure was from a
  thin/low-confidence blog aggregator, not a primary source, so it is NOT included as a
  verified number here); dropout/packet-loss rate under water-loaded, high-current conditions
  specific to a T200-class thruster (all sources found are quadcopter-context); and whether
  Bluejay's specific EDT implementation on our target hardware (Bluejay + Bluejay-compatible
  ESC on Hengla) carries current telemetry, which is the signal §6 says fault *classification*
  actually needs.

---

## 8. Claims this sweep could NOT verify (explicit list)

1. **The core comparative numbers from Whitcomb & Yoerger (1999a, 1999b)** — both papers
   (IEEE JOE 24(4):481–494 and 495–506) are paywalled (IEEE Xplore and academia.edu both
   returned 403 to this agent). Only abstracts/citation records were obtained. We know (from
   the abstract snippet) that paper 1999b compares "two novel and one conventional" thrust
   control algorithms on real hardware, and both papers are the standard citation (alongside
   Yoerger, Cooke & Slotine 1991 and Fossen & Blanke 2000) for underwater-vehicle-specific
   thruster control — but the actual recommended algorithm, its numeric performance advantage,
   and whether it was RPM-based, torque-based or fully model-based feedforward could not be
   confirmed. **This is the single most load-bearing citation for question 1/2 of this task and
   it is the one this sweep could not open.** Recommend obtaining these two papers via
   institutional/library access before finalizing any control-law decision that cites them by
   name.
2. **The firmware's specific "~3% invariance" number for thrust/RPM² across 12–20 V** (§4) —
   physically plausible per classical propeller-affinity theory, but not cross-checked against
   any public dataset; likely only checkable against the team's own bench log.
3. **Spin-up vs. spin-down time-constant asymmetry with actual numbers** for a T-series or
   similar thruster (§5) — the one real-hardware step-response study found (arXiv:1807.04109)
   reports averaged deadtime/settling time, not separated by direction.
4. **The claim that RPM≈0 forces an FDI "Invalid" sleep state** (§6, last paragraph) — sourced
   only from a search-engine paraphrase of an unopened paper; treat as unverified.
5. **eRPM quantization/resolution and dropout behavior under realistic underwater current draw**
   for a Bluejay-class ESC specifically (§7) — no bench data found for anything but quadcopter
   contexts; our own hardware-in-the-loop test once real thrusters are fitted is the only way to
   get this number.
6. **Bachmayer/Whitcomb/Grosenbaugh (2000)'s own numeric transient-accuracy results** — the
   paper's existence, methodology and qualitative conclusion ("superior accuracy... in both
   transient and steady-state responses") are confirmed via Semantic Scholar's abstract, but
   the paper itself was not opened, so no numbers from it are reported here.

---

## Bottom line for the firmware's design choices

- The `thrust_trim` design — RPM as a **slow (~10 Hz), bounded, thrust-*normalisation* gain that
  can never out-fight the 500 Hz attitude loop** — sits on the *conservative and defensible* side
  of the published literature. The literature's alternative to pure shaft-speed setpoint
  control is not "put RPM in the fast loop instead," it is torque/power control (or a
  model-based feedforward), which needs a torque/current signal, not RPM, to get the *actually
  documented* improvement (§1). RPM is being used correctly here as a **correction to a duty
  command**, not as the fast-loop control variable — which sidesteps the failure mode
  (oscillation from closing an attitude-bandwidth loop around a 0.6 s-deadtime plant, §5) that
  the firmware's own comment describes, and that is consistent with why Sørensen et al. found
  pure shaft-speed feedback to be the *worst* of the three basic strategies (§1).
- The one design choice this literature would flag for a second look: `rpm_mode` (full PI on
  shaft speed with feedforward) is explicitly the control law the surface-vessel DP literature
  found gives "the thrust, torque, and power with the largest variance" and "the least robust to
  disturbances" (§1) — if it is ever used for anything faster than the same ~10 Hz/bounded-gain
  regime as `thrust_trim`, that is precisely the configuration the literature says loses. Using
  it as a slow motor-tuning/characterization aid (fitting the throttle→RPM plant, per the task
  brief's description of `motor_tune`) rather than as a fast in-loop controller is consistent
  with the literature's warning.
- The literature's stronger recommendation — torque or power control, or the combined
  torque/power controller which "gave the overall best improvement in the performance from low
  to high loadings" (§1) — is not available to us without a torque/current signal per thruster.
  If per-thruster current sensing is built (as the task brief flags as a possibility), it opens
  the door to a torque-style low-level controller that the literature ranks strictly above both
  pure RPM and pure duty control — worth flagging as the next capability tier, not the current
  one.

