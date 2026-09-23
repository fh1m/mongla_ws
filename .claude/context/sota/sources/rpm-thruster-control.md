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

