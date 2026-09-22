# SOTA: control law + system ID, for a 5-thruster pool AUV (no tow tank)

Research pass, 2026-09-22. Budget ~20-30 web calls. Every claim carries a a titled markdown link to the source and a number.
Vehicle: ESP32+RP2350 board, 500 Hz control loop, MAVLink2@115200 to Pi5, 5 thrusters (4 tunnel ⌀84mm + 1 axial), roll unactuated, 702×176×172mm hull, mass unknown. Current controllers: scalar P/P+floor/PID only.

Status: COMPLETE (with significant Part B gaps — see "claims I could NOT verify"). Written
incrementally throughout to survive session kills.

NOTE: WebSearch tool returned "session search budget exhausted (200/200)" from the FIRST call in
this task, before any query specific to this task ran — the budget is apparently shared across
the whole Claude Code session, already spent by unrelated prior work. All research below was
done via WebFetch to specific URLs (arXiv abstracts/PDFs pulled and parsed locally with `pypdf`
since neither `pdftotext` nor a PDF renderer was installed) rather than search. This materially
limits coverage — I could not discover new sources by querying, only fetch ones I could name or
that WebFetch's own linked citations surfaced.

---

## Part A — Control Laws

### A1. INDI / adaptive INDI

**[Cascaded Incremental Nonlinear Dynamic Inversion for MAV Disturbance Rejection — Smeur, de Croon, Chu (arXiv 1701.07254)](https://arxiv.org/abs/1701.07254)**
- What INDI needs: (1) an **angular acceleration estimate** — obtained here from **finite-difference
  of gyroscope rate**, filtered with a 2nd-order filter (ωn=20 rad/s, ζ=0.7) to fight the resulting
  noise, and matched-delay filtering of every other term in the loop so signals stay time-aligned;
  (2) the **control effectiveness matrix G1/G2** (moment-per-actuator-increment), linearized around
  the current operating point — the paper notes this **can be identified from a single test flight**
  and even **adapted online** to track airframe changes.
- Sample-rate requirement, stated directly: the full stack (gyro + accelerometer + INDI control law
  + actuator model A(z), α=0.1) ran at **512 Hz** on a Parrot Bebop quadrotor (Paparazzi autopilot).
  The paper does not give a theoretical minimum rate, but the entire derivation depends on the
  increment `ω − ωf` being small between samples (a first-order Taylor linearization) — implying the
  method degrades as the sample interval grows, though no explicit "must exceed X Hz" number is given.
- Measured performance vs PID (all on the same Bebop, same outer-loop gains):
  - Windtunnel gust rejection (10 m/s flow, 2.85×2.85 m tunnel): INDI **0.21 m** max position
    deviation entering, **0.20 m** leaving, recovered within **3 s**; PID **1.51 m** max deviation —
    **more than 7× lower position deviation** than PID, exact phrase from the abstract.
  - Outdoor takeoff in wind: INDI mean max position error **0.24 m** vs PID **0.85 m**.
  - Explicit tradeoff called out for PID: raising the integral gain speeds gust correction but adds
    overshoot on setpoint changes — "this trade-off is non-existent for the INDI controller."
- **No underwater application found in this paper or its immediate citation set** (all examples are
  quadrotors/tailsitters/MAVs — see the arXiv author-search list below). Underwater INDI application
  is a claim I could **NOT verify** — flagged in the final section.
- Related papers by the same group found via arXiv author search (not individually fetched, listed
  for reference): [Non-Linear Dynamic Inversion with Actuator Dynamics: an Incremental Control
  Perspective](https://arxiv.org/abs/2201.09805), [Incremental Control and Guidance of Hybrid Aircraft
  Applied to a Tailsitter UAV](https://arxiv.org/abs/1802.00714), [Robust H∞ Controller Design for
  INDI-Controlled Quadrotor Using Online Parameter Identification](https://arxiv.org/abs/1902.00279)
  — all aerial, none underwater.

### A3. Learning-based control / adaptation actually deployed on real underwater vehicles

**[Uncertainty-Aware Adaptive Dynamics for Underwater Vehicle-Manipulator Robots (arXiv 2603.06548)](https://arxiv.org/abs/2603.06548)**
- **Real hardware, not sim**: a **BlueROV2 Heavy with a 4-DOF manipulator**, wet-tested. This is
  the strongest "got wet" learning/adaptive-ID result found in this pass.
- Online convex adaptive-parameter estimation (**k+12n = 75 unknowns, k=27 vehicle parameters**)
  solved every update; **median solver time ≈ 0.023 s/update (all runs stayed below 0.03 s)** —
  i.e. **≥33 Hz** achievable identification-and-refit rate, run on a **station computer: Intel
  Core Ultra 9 275HX, 64 GB DDR5 RAM**, ROS 2 Jazzy, Python+C++ — **not** an embedded/onboard CPU,
  and well above our board's compute class, so this number bounds "a beefy laptop," not the Pi 5.
- Accuracy vs a fixed-parameter model (Table I, vehicle DOFs, forces in N / moments in N·m):
  surge R²=0.58 RMSE 4.01 N; sway R²=0.46 RMSE 4.48 N; heave R²=0.68 RMSE 6.53 N; roll R²=0.72
  RMSE 1.04 N·m; manipulator joint 3 R²=0.98, slope 1.01, RMSE≈0.22 N·m. Manipulator parameters
  converged **within the first 10 s** of excitation.
- Verdict: real-vehicle evidence that online adaptive dynamics ID is computationally cheap enough
  to run at tens of Hz — but on desktop-class hardware, and it is parameter *adaptation* layered
  on a known model structure, not a learned controller replacing PID.

**[Dynamic System Identification of Underwater Vehicles Using Multi-Output Gaussian Processes (arXiv 2006.02194)](https://arxiv.org/abs/2006.02194)**
- **Simulation only** — trained and validated against a first-principles Simulink model of a
  **REMUS 100 AUV**, not a physical vehicle. Explicitly flagged here as NOT a real deployment.
- Multi-output GP identification beat two RNN baselines on RMSE/PRESS/MAE at every training-set
  size tested (500–4000 samples); example: run 1 RMSE — GP 1.85e-2 vs RNN1 5.27e-2 vs RNN2 6.34e-2.
  Code released: [ArizaWilmerUTAS/System-identification-of-underwater-vehicles-with-Multi-Output-Gaussian-Processes](https://github.com/ArizaWilmerUTAS/System-indetiﬁcation-of-underwater-vehicles-with-Multi-Output-Gaussian-Processes) (GitHub link as printed in the paper's own footnote).
- Data volume: sensitivity swept at 500/1000/1500/2000/4000 simulated samples; GP error curve was
  consistently below both RNNs at every size on the sweep (Fig. 9/10 in the paper).
- Verdict: evidence GPs beat RNNs for AUV system ID **in simulation with as few as 500 samples**,
  no evidence yet of GP-ID running on a physical vehicle in this source.

**A general-purpose learning-control paper found but not deeply verified**: [Exploration of the
Applicability of Probabilistic Inference for Learning Control in Underactuated AUVs](https://arxiv.org/abs/1912.11584) — surfaced by an arXiv keyword search on "reinforcement learning AUV real
vehicle" but not fetched in this pass due to budget; **listed, not verified** — do not cite its
claims without reading it.

### A4. Loop rate — is 500 Hz justified?

**[ArduCopter attitude control loop rate — ArduPilot developer docs](https://ardupilot.org/dev/docs/apmcopter-programming-attitude-control-2.html)**
- States directly: the attitude/rate controller update happens "**400 Hz on Pixhawk**, 100 Hz on
  APM2.x" boards. ArduSub inherits ArduCopter's core flight code, so **ArduSub's inner loop is the
  same 400 Hz figure** — this is an in-air number carried into ArduSub by code reuse, **not** a
  number derived from underwater dynamics or measurement.
- No underwater-specific justification for 400 Hz (or any other rate) was found anywhere in this
  pass — every AUV-rate number found in the literature searched here is **far lower**: the
  Lyapunov-MPC AUV paper above runs its whole control loop at **10 Hz** (Δt=0.1s); the Koopman-MPC
  AUV paper collects ID data at **10 Hz** with a 100 Hz internal RK4 discretization for the fitted
  model only, not the actuation loop.
- **Conclusion on 500 Hz**: I found **no measured evidence, for any underwater vehicle, that a
  control loop rate anywhere near 400-500 Hz produces a measurable performance benefit over
  10-50 Hz.** The INDI quadrotor result (512 Hz, section A1) is the one case in this research pass
  where a high rate is tied to a measured benefit, and that benefit (7× lower gust deviation) is
  attributed to the INDI algorithm's incremental linearization needing small inter-sample steps
  for an **aerial** vehicle with millisecond-scale torque dynamics — water's added mass and drag
  make AUV attitude/velocity dynamics much slower, which is consistent with why the AUV papers
  found here all run at 10 Hz. This is inference from the assembled evidence, not a single
  citable "AUV doesn't need >X Hz" study — flagged in the "could not verify" section.

### A5. Disturbance rejection without a DVL

- The INDI paper (section A1) is itself a disturbance-rejection result with hard numbers: an
  **incremental** control law (effectively an inner-loop acceleration feedback + PD, closest
  relative is a disturbance-observer structure) rejected a 10 m/s windtunnel gust to **0.21 m**
  peak deviation vs **1.51 m** for PID — **7×** — using only onboard IMU-derived acceleration, no
  external velocity sensor. [Cascaded INDI for MAV disturbance rejection](https://arxiv.org/abs/1701.07254).
- [Gust Estimation and Rejection with a Disturbance Observer for Proprioceptive Underwater Soft
  Morphing Wings (arXiv 2602.04438)](https://arxiv.org/abs/2602.04438) — real hydraulically
  actuated underwater soft-wing hardware, experimentally validated, uses a **curvature-based
  proprioceptive disturbance observer** to reconstruct flow disturbance in real time without a
  flow sensor. Different actuator class (soft wing, not thruster) from our vehicle, and no
  numeric rejection/bandwidth figure was retrievable from the abstract in this pass — **listed as
  a lead, not a verified number.**
- No source in this pass gives a quantitative "integral action vs disturbance observer" head-to-
  head for a thruster-driven AUV without a DVL — this comparison is a gap, flagged below.

### A6. Cascaded vs single-loop architectures, board/companion boundary

- The Lyapunov-MPC AUV paper (arXiv 2509.17237, section A2) is itself a **single integrated**
  architecture — one MPC that folds thrust allocation and trajectory tracking into one QP, rather
  than cascading an outer guidance loop into an inner attitude loop.
- The INDI paper (arXiv 1701.07254, section A1) is explicitly **cascaded**: an outer INDI position
  loop produces an acceleration reference, mapped to attitude, fed to an inner INDI attitude loop
  running at the full 512 Hz sensor rate — the title itself is "*Cascaded* Incremental Nonlinear
  Dynamic Inversion." This is the same cascade shape (outer guidance → inner rate loop) as our
  existing yaw-PID-on-host / heading-lock-pure-P architecture, just with INDI substituted for PID
  at both levels.
- No source in this pass gives a *measured* board-vs-companion boundary rationale specific to a
  low-power flight-controller + companion-computer split (our ESP32/RP2350 + Pi 5 architecture) —
  this remains inferred from the two architectures above, not directly cited.

---

## Part B — System ID Without a Basin

### B7. What is identifiable from free-running tests (no basin)

- No accessible full text of Fossen's *Handbook of Marine Craft Hydrodynamics and Motion Control*,
  Ridao/Girona500 identification papers, Hegrenaes & Hallingstad, or Martin & Whitcomb was
  reachable in this pass — see "claims I could NOT verify" below. What **is** verified from the
  MSS toolbox repository directly (Fossen's own software, current as of this fetch):
  [MSS (Marine Systems Simulator), cybergalactic/MSS on GitHub](https://github.com/cybergalactic/MSS)
  — Fossen's reference toolbox ships parametric models only for a **REMUS 100 AUV**, an NPS AUV,
  and a DSRV (deep submergence rescue vehicle) under `CRAFT/AUV/` — confirmed by listing the
  repository directly (`SIMremus100.m`, `SIMnpsauv.m`, `SIMdsrv.m`). **No BlueROV2 or Girona500
  model ships in MSS** — those parameter sets, if they exist publicly, live outside Fossen's own
  toolbox. This is a directly-observed fact (repo listing), not a claim from a paper.
- Real evidence that per-axis dynamics ARE identifiable from ordinary operating data, not a
  dedicated maneuver: [Uncertainty-Aware Adaptive Dynamics For UVMS (arXiv 2603.06548)](https://arxiv.org/abs/2603.06548)
  identified **27 vehicle parameters** online, on a real BlueROV2 Heavy, from streamed IMU +
  current data during normal operation, converging within **10 s** of excitation for the
  manipulator joints, with the accuracy numbers given in section A3 above (surge RMSE 4.01 N,
  heave RMSE 6.53 N, etc.) — this is the closest thing found in this pass to a real, quantified
  answer for "what can free-running excitation identify, and how accurately."

### B8-B13 — added mass from geometry, coast-down drag, thruster ID with a load cell, data-driven ID, model payoff, open parameter sets

**Status: NOT reached with primary sources in this pass.** The WebSearch tool reported its budget
exhausted (200/200) on the very first call of this task, before any query specific to Part B ran.
Every result above was obtained either by (a) guessing a correct arXiv ID from context and
fetching it directly, (b) using arXiv's own public `export.arxiv.org/api/query` XML API via
`curl` (not blocked, unlike WebSearch/Bing/DuckDuckGo/Google which all failed — captcha'd,
geo-mismatched, or simply returned no result), or (c) fetching GitHub repos directly. None of
those three channels surfaces MDPI/IEEE/Elsevier journal literature, which is where essentially
all of the classical AUV system-ID literature (Fossen, Ridao, Hegrenaes & Hallingstad, Martin &
Whitcomb, HAMS/Capytaine/NEMOH panel-code validation studies, published coast-down Cd values,
thruster-ID-with-load-cell best practice, and open BlueROV2/Girona500 parameter sets) actually
lives. I was not able to reach any of it through the tools available in this run. This is an
honest gap, not a set of claims — see the table and the final section.

One verified, minor data point: [Added mass — Wikipedia](https://en.wikipedia.org/wiki/Added_mass)
gives the closed-form added mass of a **sphere** as **(2/3)πr³ρ_fluid** — exactly half the mass of
the fluid the sphere displaces — and confirms that for a general (non-spherical) body added mass
is a full 6×6 tensor, direction-dependent. This is a sanity-check reference point only (our hull
is not a sphere); it does not substitute for a panel-code number on the actual hull.

### B10. Thruster identification with a load cell — what is directly verified

**[Blue Robotics T200 Thruster specifications](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/)**
gives manufacturer bollard (static) thrust numbers, useful as a sanity-check ceiling for our
tunnel thrusters (⌀84 mm, likely a T200-class unit): **3.71/2.92 kgf fwd/rev @ 12 V**,
**5.25/4.1 kgf @ 16 V (nominal)**, **6.7/5.05 kgf @ 20 V (max)**. These are **static/bollard**
numbers only — the page gives no forward-speed or advance-ratio derating curve, so the "bollard k
over-predicts at cruise" caveat in the task brief is **NOT contradicted or confirmed by this
source**; it is standard open-water-propeller-curve knowledge (thrust falls as advance ratio J
rises) but no AUV-specific number for the magnitude of that over-prediction was found in this
pass.

---

## Summary Table A — control techniques

| technique | what it needs | measured benefit | runs on ESP32@500Hz or Pi5? | verdict for a 5-thruster pool AUV |
|---|---|---|---|---|
| INDI (cascaded) | angular-accel estimate (finite-diff gyro, filtered), control-effectiveness matrix (identifiable from 1 flight, adaptable online) | 7× lower position deviation vs PID in a 10 m/s gust (0.21 m vs 1.51 m), on a real quadrotor [Smeur et al.](https://arxiv.org/abs/1701.07254) | Ran at 512 Hz on an aerial vehicle's flight computer — comparable class to our ESP32; the linearization assumption gets worse as sample interval grows, so plausible on the board, **but zero underwater validation exists in the literature found here** | Interesting for the inner attitude/rate loop given board compute headroom; unverified underwater — would need to be proven on this vehicle first |
| Lyapunov-constrained MPC (ALMPC) | full 6-state model, QP/SQP solver, fault-detection layer | 44-72% RMSE cut vs benchmark adaptive MPC under thruster faults, sim only, 4-thruster planar AUV [arXiv 2509.17237](https://arxiv.org/abs/2509.17237) | **No solve-time number reported anywhere in the paper** — cannot say | Not verified feasible on this board; sim-only fault-tolerance result, not a baseline-performance one |
| Koopman-linear MPC | lifted linear model fit from data (100 Hz sim sampling here), QP solve | Argued cheaper than NMPC ("reduces computational demands") but **no ms number given** | Ran in Gazebo/ROS on a 2017 laptop CPU (i7-7820HQ) — **not embedded**, no Pi5/ESP32 number | Same gap — cannot certify ≥20 Hz on Pi5 from this source |
| Adaptive online dynamics ID (convex, UKF-adjacent) | streamed IMU + demand data, 75-parameter convex solve | Real BlueROV2 Heavy, RMSE 4-6.5 N per vehicle DOF, converges in 10 s [arXiv 2603.06548](https://arxiv.org/abs/2603.06548) | Median 0.023 s/update (≥33 Hz) on an **Intel Core Ultra 9 275HX / 64 GB** desktop — far above Pi5, no ESP32/Pi5 number | This is the strongest "runs fast enough, on a real underwater vehicle" evidence found, but on hardware ~10-50× our companion computer |
| Multi-output GP system ID | time-series I/O data, kernel hyperparameter fit | Beats RNN baselines at every tested data size (500-4000 samples), sim only (REMUS 100) [arXiv 2006.02194](https://arxiv.org/abs/2006.02194) | Not benchmarked for embedded/real-time use in this paper | Promising for offline/deck-side model fitting, not shown real-time or on real hardware |
| Existing PID/P architecture (ours) | current gains only | n/a — this is the baseline | Runs today at 500 Hz on the board | Working; the gap this research was meant to close (is a switch justified) remains genuinely open |

## Summary Table B — system-ID quantities

| quantity | obtainable without a basin? | method | expected error | what it unlocks |
|---|---|---|---|---|
| Per-axis v_ss = g·u + b (steady thrust-to-velocity gain) | Yes — already running | RLS on free-running demand/flow-velocity data, gated on 100 excited samples, RMS≤0.10 m/s (existing system) | Already measured: downward-camera velocity noise floor 0.57 mm/s, worst 30 cm-slide error 1.09 cm | Feedforward thrust-to-speed mapping per axis — already in use |
| Online 27-parameter vehicle dynamics | Yes, from ordinary operation | Convex online adaptation, streamed IMU+demand | RMSE 4-6.5 N per DOF at convergence, real BlueROV2 [arXiv 2603.06548](https://arxiv.org/abs/2603.06548) | Feedforward disturbance/torque prediction beyond a single scalar gain, but needs desktop-class compute for the ~0.023 s/update solve shown |
| Added mass tensor (6×6) | **NOT verified in this pass** — geometric/panel-code methods (strip theory, HAMS, Capytaine, NEMOH) were the target but no accessible source was reached | — | — | — |
| Drag coefficients from pool coast-down | **NOT verified in this pass** — no accessible source on wall-effect confounds or Re~1e5-1e6 Cd values for AUV-like hulls was reached | — | — | — |
| Thruster thrust-RPM-voltage curve | Partially — manufacturer bollard numbers exist for a T200-class unit (3.71-6.7 kgf across 12-20 V) [Blue Robotics](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/) | With the load cell available: static bollard-pull sweep is directly measurable; **the advance-ratio (forward-speed) derating is NOT covered by any source reached here** | Static-only numbers have an unquantified over-prediction at cruise (the "bollard k over-predicts" caveat is asserted in the task brief but not independently confirmed or numerically bounded by any source in this pass) | A verified static thrust curve, but a cruise-speed correction remains unverified |
| Sphere added mass (sanity-check reference only) | Yes, closed form | (2/3)πr³ρ_fluid [Wikipedia — Added mass](https://en.wikipedia.org/wiki/Added_mass) | Exact for a sphere; irrelevant to a 702×176×172mm hull except as a formula sanity check | Nothing directly — reference point only |

---

## Claims I could NOT verify in this pass

1. **Fossen's identification-observability results** (which coefficients are/aren't observable from
   step response / zig-zag / coast-down / self-propelled maneuvers) — could not reach *Handbook of
   Marine Craft Hydrodynamics and Motion Control* or any secondary source citing its specific claims.
2. **Ridao / Girona 500 identification papers** — not reached; no arXiv presence, and web search was
   unavailable for the whole session.
3. **Hegrenæs & Hallingstad, HUGIN model-aided INS numbers** — not reached; this is the single most
   requested number in the brief (model-aided dead-reckoning drift figures) and I have **zero**
   verified number for it.
4. **Martin & Whitcomb system-identification methodology papers** — not reached.
5. **HAMS / Capytaine / NEMOH panel-code accuracy for slender/box-ish hulls** — not reached; the
   only added-mass fact verified in this pass is the closed-form sphere formula from Wikipedia,
   which is not evidence about panel-code accuracy on a real hull.
6. **Published Cd values for AUV-like bodies at Re~1e5-1e6**, and **coast-down wall-effect /
   added-mass-coupling confounds** — not reached.
7. **The advance-ratio ("bollard k over-predicts at cruise") caveat's magnitude** — asserted as
   true in the task brief (consistent with general open-water-propeller-curve theory) but no
   AUV-specific quantitative source was found to confirm the size of the effect.
8. **Open, published BlueROV2 or Girona500 parameter sets** usable for a sanity check — not found;
   confirmed by direct repository inspection that Fossen's own MSS toolbox does **not** ship either
   (it ships REMUS 100, an NPS AUV, and a DSRV instead).
9. **INDI ever applied to an underwater vehicle** — actively searched for (arXiv query
   `"incremental nonlinear dynamic inversion" AND underwater`, zero hits) and not found; treated as
   an open gap, not a "no" — absence of an arXiv hit is not proof no one has tried it in a venue
   arXiv doesn't index.
10. **A direct "is 500 Hz justified" study for underwater vehicles specifically** — the conclusion
    in A4 is inference from the rates actually used across the AUV papers found (all ≤100 Hz for
    control, ≤10 Hz reported explicitly for two MPC controllers), not a single study that varies
    loop rate and measures AUV performance directly.

**Root cause for the size of this gap list**: the WebSearch tool reported its session budget
(200/200) already exhausted on the very first call made in this task — before any query specific
to this research had been issued. Every finding above came from arXiv (reachable via its public
`export.arxiv.org` API through `curl`, and via direct `WebFetch` on guessed/known arXiv URLs),
plus a handful of manufacturer/toolbox/reference pages fetched by exact URL. Google, Bing, and
DuckDuckGo were each tried directly through WebFetch and each failed (captcha wall, geo-mismatched
results, or a "no relevant content" response) — general web search was effectively unavailable for
this entire task. The classical marine-hydrodynamics literature (Fossen, Ridao, Hegrenæs,
Whitcomb, panel-code validation studies) lives in IEEE/Elsevier/Springer/MDPI journals that neither
arXiv nor a handful of guessed URLs can substitute for. **If this gap matters, the highest-leverage
next step is not more of this same research — it's re-running Part B specifically with a working
WebSearch budget** (or Context7/library-doc tooling, which was not applicable here since these are
papers, not software docs).


**[Adaptive Lyapunov-constrained MPC for fault-tolerant AUV trajectory tracking (arXiv 2509.17237)](https://arxiv.org/abs/2509.17237)**
- Simulation only, on a **4-thruster planar AUV** (Saab Seaeye Falcon geometry, horizontal-plane
  only — roll/pitch left unactuated by the model itself, same restriction as our vehicle).
- Sampling time **Δt = 0.1 s (10 Hz)**, prediction horizon **N = 10** steps (1 s lookahead).
  Solved as SQP over the KKT system, warm-started from a damped-least-squares thrust allocation.
- **No solve-time-in-ms or CPU/hardware number is reported anywhere in the paper** — could not
  verify feasibility on any embedded target from this source.
- Reported accuracy vs baselines (Table I, case with 2 induced thruster faults): vs a benchmark
  adaptive MPC (AMPC), the Lyapunov-constrained version cut **RMSE by 44% (x), 72% (y), 15% (ψ)**
  and **IAE by 41-62%** across states; vs backstepping control (BSC) the gap was larger still
  (BSC x-RMSE 0.270 m vs ALMPC 0.080 m).
- Verdict: evidence for tracking-error benefit of Lyapunov-constrained MPC under thruster faults,
  **zero evidence on runtime**, sim-only, 4-thruster planar model (not our 5-thruster geometry).

**[Koopman-operator MPC for AUV speed control (arXiv 2503.09628)](https://arxiv.org/abs/2503.09628)**
- Simulation only, in **Gazebo Harmonic + ROS Iron**, host: **Intel Core i7-7820HQ @ 2.90 GHz,
  16 GB RAM** (a 2017-era laptop CPU, not an embedded target).
- System-ID data collected at **10 Hz**; the underlying lifted-Koopman dynamics were fit from a
  simulated dataset sampled at **Ts = 0.01 s (100 Hz)**, 1000 trajectories × 100 steps each.
- MPC prediction horizon **Nh = 10**, weights QN=Qu=2000, R=0.01, input constraint ±50 (RPM
  units), rate constraint ±20 per step.
- Explicit rationale given for choosing Koopman-linear MPC over nonlinear MPC: "**Unlike
  nonlinear MPC, which involves solving challenging nonconvex optimization problems... the
  data-driven [Koopman] approach reduces computational demands**" — a qualitative claim, again
  **no ms/Hz solve-time number given**.
- Verdict: same gap as above — Koopman MPC is argued cheaper than NMPC but the paper gives no
  quantitative solve time on any hardware, so "feasible at ≥20 Hz on a Pi 5" is **NOT verified**
  by this source either way.


