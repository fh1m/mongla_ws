# System identification of AUV dynamics without a towing tank, basin, or CFD

Scope: what is obtainable from a swimming pool, a bench, and a desk, for a ~702×176×172 mm
hull (fineness 3.99), 4 tunnel thrusters + 1 axial, roll unactuated, mass currently unknown,
sensors = 500 Hz IMU + Bar30 depth + downward camera (verified bottom-track velocity sensor,
worst error 1.09 cm over 30 cm slides). No DVL, no GPS, no towing tank, no CFD budget, no
force balance except a bench load cell for bollard thrust.

Status legend: **[WATER]** needs the pool · **[BENCH]** needs a bench/load cell/scale ·
**[DESK]** pure computation, no test needed.

---

## 1. Free-decay / free-oscillation identification of rotational damping — **[WATER]**

**Method** (standard in ship/offshore hydrodynamics, ported to small ROV/AUV work): excite a
single DOF (yaw, roll, or heave) to some initial amplitude, cut all commands so the body
oscillates or decays freely, then fit a 1-DOF damped equation to the logged
angle/angular-rate time series:

```
(I_zz + N_ṙ) * ṙ + N_r*r + N_|r|r*r*|r| + restoring(if any) = 0
```

For yaw there is normally no restoring term (no righting moment), so a pure rotation cut from
some initial rate ω₀ decays as a first-order nonlinear ODE in ω alone:

```
(I_zz + N_ṙ) * dω/dt = -N_r*ω - N_|r|r*ω*|ω|
```

- **Estimator options reported in the literature**:
  - *Logarithmic decrement* — only valid when there is a restoring term (roll/pitch with
    metacentric righting, or heave with buoyancy stiffness) so the record is genuinely
    oscillatory; successive peak amplitudes ζₙ fit ln(ζₙ/ζₙ₊₁) ≈ const for linear damping, or
    a modified decrement for combined linear+quadratic damping (Roll motion free-decay
    papers use this — see ResearchGate "A Study of the Roll Motion by Means of a Free Decay
    Test").
  - *Nonlinear least-squares fit of the ODE* — for yaw (no restoring term) this is the
    correct approach: numerically integrate the assumed ODE with (N_ṙ, N_r, N_|r|r) as free
    parameters, minimize the sum-of-squares error against the measured ω(t) trace. This is
    what is used for AUV/ROV "free-decay pendulum" papers referenced below.
- **Units**: N_r is [N·m·s/rad] (linear damping torque per unit rate), N_|r|r is
  [N·m·s²/rad²] (quadratic damping torque per rate², i.e. the K in `torque = -K·ω·|ω|`).
  N_ṙ is the yaw added-inertia term [kg·m²], inseparable from I_zz using this test alone
  (decay only constrains the *sum* I_zz + N_ṙ, not each term individually) — a known
  limitation flagged repeatedly in the literature.
- **Accuracy / error sources reported**:
  - Free-decay in ROV work: "Estimation of the Hydrodynamics Coefficients of an ROV using
    Free Decay Pendulum Motion" (Academia.edu mirror) reports close matches between
    identified and reference quadratic-damping coefficients in yaw specifically, but the
    paper is only accessible as an abstract-level summary here — **treat the quantitative
    accuracy number as unverified** until the full PDF is read.
    https://www.academia.edu/25845722/Estimation_of_the_Hydrodynamics_Coefficients_of_an_ROV_using_Free_Decay_Pendulum_Motion
  - A cautionary result from offshore engineering: linear+quadratic damping models fitted
    from free-decay **do not hold across the whole amplitude range** — the fitted (N_r,
    N_|r|r) pair is only valid near the amplitude range it was excited at, and re-fitting at
    a different initial amplitude gives different coefficients. Source: ScienceDirect,
    "Linear and quadratic damping coefficients of a single module of a very large floating
    structure over variable bathymetry: Physical and numerical free-decay experiments."
    https://www.sciencedirect.com/science/article/pii/S2468013321001261
  - Known confounders specific to our situation: pool-wall proximity (blockage) inflates
    apparent damping vs. open water; near-surface waves radiated during a yaw free-decay if
    the vehicle is shallow; residual thruster drag (props windmilling / cogging) even with
    zero command adds an unmodeled damping term that free-decay cannot separate from
    hydrodynamic N_r unless props are physically locked or the same test is repeated with
    props removed.
  - Source (general free-decay + least-squares parameter-fit methodology for underwater
    vehicles): "Experimental validation of open-frame ROV model for virtual reality
    simulation and control," J. Marine Sci. Technol., Springer.
    https://link.springer.com/article/10.1007/s00773-017-0469-3

**Practical recipe for our vehicle**: spin up in yaw with the axial+lateral thrusters to a
known ω₀ (read from the 500 Hz IMU gyro, which is far higher rate than needed), cut all
motor commands simultaneously, log gyro-z at full rate through the decay, repeat at 3-4
different ω₀ to check whether N_r/N_|r|r are amplitude-invariant (per the confound above),
and fit by nonlinear least squares. Roll is unactuated on this hull, so this method cannot be
used for roll damping identification — only yaw (and pitch/heave if we can excite them).

---

## 2. Coast-down for translational drag — **[WATER]**

**Method**: accelerate to a steady surge speed, cut thrust, log velocity decay (from the
downward-camera bottom-track velocity, our only real speed sensor) versus time, fit
`(m + X_u̇)·dU/dt = -X_u·U - X_|u|u·U|U|` the same way as the rotational case.

- Automotive coast-down (SAE J1263) is the ancestor of this method and is well documented:
  MathWorks "Estimate Vehicle Drag Coefficients by Coast-Down Testing" describes fitting
  fixed + linear (rolling/drivetrain) + quadratic (aerodynamic) resistance terms to a
  velocity-vs-time coast-down trace via a Parameter Estimator / least-squares fit — directly
  transferable to `X_u`, `X_|u|u` in the surge equation.
  https://www.mathworks.com/help/sldo/ug/estimate-vehicle-drag-coefficients-by-coast-down-testing.html
- Underwater-specific: "Identification of Drag Force of the Underwater Vehicles" (Journal of
  Applied Fluid Mechanics) performs an inverse analysis of drag coefficient from measured
  wake velocity using momentum/mass conservation over a control volume — this is a *wake
  survey* method, not a coast-down, and needs velocity-field measurement in the wake, which
  we cannot do with a single downward camera. Flagged as **not directly applicable** to our
  sensor set; listed for completeness.
  https://www.jafmonline.net/article_426_60b78f8955145427316be02b2ec0d5bc.pdf (PDF text not
  machine-extractable in this session — treat method summary above as from the abstract only,
  **unverified in detail**.)
- **Confounders specific to a small pool + our sensor set**:
  - *Added-mass coupling*: any residual yaw/sway during the coast-down couples through the
    off-diagonal added-mass terms and biases the pure-surge fit; the vehicle must coast dead
    straight (verify with gyro) or the cross terms must be included in the fit.
  - *Thruster drag when off*: a stationary (non-spinning) propeller in the flow still adds
    parasitic drag not present in the "bare hull" coefficient a control designer usually
    wants — same caveat as free-decay above.
  - *Wall/blockage effects*: a swimming pool is a bounded channel; at vehicle-length-scale
    proximity to walls, blockage measurably raises drag versus open water (classic tow-tank
    blockage-correction problem, e.g. ITTC guidelines) — for a 702 mm hull this is likely
    only significant within roughly 1-2 hull lengths of a wall, so keep coast-down runs away
    from the pool edges.
  - *Surface/free-surface effects*: within about 2-3 hull diameters of the free surface,
    wave-making drag and altered added mass appear; run coast-downs at mid-depth in the pool
    to avoid this.
  - *Downward-camera velocity floor/noise*: our only speed sensor is the bottom-track camera,
    validated to 1.09 cm worst-case error over 30 cm — good near the bottom but degrades with
    altitude (scale-dependent optical flow) and is unusable at low light/low texture; coast-
    down should be run at the altitude band where that sensor was actually validated.

---

## 3. ⭐ Added mass analytically from geometry — **[DESK]**, needs no water

This is the highest-value item for us because it needs zero pool time.

### 3.1 Classical formulas — prolate spheroid (ellipsoid of revolution), Lamb's k-factors

For a prolate spheroid of semi-major axis `a` (half-length) and semi-minor axis `b` = `c`
(half-beam, axisymmetric), define the eccentricity `e = sqrt(1 - (b/a)²)` and the fineness
ratio `a/b` (our hull: length 702 mm / diameter ~176-172 mm → a/b ≈ 3.99, i.e. **the fineness
ratio of 3.99 already quoted in CLAUDE.md is directly the a/b input to this formula**).

Lamb's classical integrals (Lamb, *Hydrodynamics*, 6th ed., §§114-119; also reproduced in
Imlay 1961 DTIC report, "The Complete Expressions for Added Mass of a Rigid Body Moving in an
Ideal Fluid," AD0263966 — PDF fetch blocked in this session by the DTIC server, 403; treat the
formula forms below as standard textbook material, cross-checked against the MIT 13.012
slender-body-theory handout (Reading7, A. Techet, 2004) whose PDF text also could not be
machine-extracted this session — **both primary derivations unverified line-by-line here,
flagged below**; the formula forms themselves are extremely standard and appear consistently
across secondary sources):

```
α0 = (2*(1-e²)/e³) * [ (1/2)*ln((1+e)/(1-e)) - e ]
β0 = (1/e²) - ((1-e²)/(2e³)) * ln((1+e)/(1-e))          [used for k2, k' below is symmetric]

k1 = α0 / (2 - α0)      (added mass coefficient for axial (surge) motion, along major axis a)
k2 = β0 / (2 - β0)      (added mass coefficient for transverse (sway/heave) motion, along
                          minor axis b)
k' = (a²-b²)² * (α0 - β0) / [ 2*(a²-b²) + (a²+b²)*(β0-α0) ]
                          (added *inertia* coefficient for rotation about the minor axis,
                          i.e. pitch/yaw rotational added inertia)
```

Then the added-mass entries are:
```
X_u̇ = -k1 * (4/3)*π*ρ*a*b²        (surge added mass, along the long axis)
Y_v̇ = Z_ẇ = -k2 * (4/3)*π*ρ*a*b²   (sway/heave added mass, along the short axes)
N_ṙ = M_q̇ = -k' * (4/15)*π*ρ*a*b²*(a²+b²)   (yaw/pitch added moment of inertia)
```
where ρ = 1000 kg/m³ (fresh pool water) or ~1025 kg/m³ if ever run in seawater, and
(4/3)πab² is the spheroid's own displaced volume.

**For our hull, a/b ≈ 3.99 (fineness 3.99):** standard tabulated Lamb k1/k2 values (widely
reproduced, e.g. in Newman's *Marine Hydrodynamics* Table 3.1, and in the AUV added-mass
literature) at a/b ≈ 4 are approximately **k1 ≈ 0.209, k2 ≈ 0.702** (values read from the
standard k1/k2-vs-fineness-ratio curve/table at a/b=4; we could not pull the exact DTIC/Lamb
table text this session — **flagged unverified, re-derive numerically from the α0/β0
integrals above rather than trust the quoted 0.209/0.702 pair**, since those are recalled
from memory of the standard table, not read from a fetched source this session).

**Non-spheroidal, non-axisymmetric hull correction**: our hull is not a perfect prolate
spheroid (it carries external tunnel-thruster housings and is not exactly axisymmetric in
cross-section). Practice for this case, per "Sensitivity of AUV added mass coefficients to
variations in hull and control plane geometry" (Ocean Engineering, Humphreys & Watkinson
lineage):

> Fit an "equivalent ellipsoid" by matching displaced volume and either max diameter or
> wetted length to the real hull, then apply Lamb's k-factors to that equivalent body; then
> add appendage/control-surface added mass by strip theory or slender-wing formulas
> separately and sum.

https://www.sciencedirect.com/science/article/abs/pii/S0029801802000410 (abstract-level only,
full text paywalled — **could not verify the reported accuracy number against CFD/experiment
from this source in this session**.)

### 3.2 Strip theory (better than single-ellipsoid for a non-ellipsoidal hull, still desk-only)

Strip theory sections the hull into 2D cross-sections along its length, applies 2D added-mass
coefficients (e.g. 2D circular-cylinder added mass = ρπr² per unit length for sway/heave) to
each section, and integrates along the length — standard in submarine/AUV hydrodynamics
(referenced generically by the Girona 500 discussion below: "strip theory cannot be used...
[for non-torpedo hulls]," implying it IS the default method for a torpedo-shaped hull like
ours). This is more work than the single-ellipsoid fit but captures the local diameter
variation (nose taper, thruster pods) that a single ellipsoid cannot. No source with a fully
worked strip-theory example was retrieved this session — **method described from secondary
references only, flagged for a follow-up fetch**.

### 3.3 Generic numerical 6-DOF formulation (still desk-only, more setup)

"Generic 6-DOF Added Mass Formulation for Arbitrary Underwater Vehicles" (DiVA portal thesis)
claims a method to compute the full 6×6 added-mass matrix for an arbitrary-shaped vehicle from
geometry alone (not just axisymmetric bodies) — **could not fetch full text this session**
(connection refused to diva-portal.org). Flagged for retry; likely the most relevant single
source for a non-spheroidal hull with 5 asymmetric thruster pods, since it explicitly targets
"arbitrary" shapes rather than assuming a spheroid.
https://www.diva-portal.org/smash/get/diva2:1127931/FULLTEXT01.pdf

### 3.4 Accuracy of analytical vs measured added mass — FOUND (adjacent domain, not underwater)

"Experimental Calculation of Added Masses for the Accurate Construction of Airship Flight
Models" (MDPI *Aerospace* 11(11):872, 2024). https://doi.org/10.3390/aerospace11110872

- A **prolate ellipsoid of revolution**, aspect ratio 6 (close in spirit to our fineness 3.99,
  same shape family), length 1 m, mass 482 g, tested in a **wind tunnel** (same potential-flow
  added-mass physics as water — added mass depends only on fluid density and geometry, not
  on the fluid being air vs water, so this cross-domain result is a legitimate stand-in for
  "how good is the analytical ellipsoid formula against a direct physical measurement").
- **Result: only 2.1% deviation between the measured virtual/added mass and the theoretical
  (Lamb-type) value**, after applying proper signal processing (high-pass filtering +
  second-order Fourier fit to the oscillation record — i.e. essentially the same free-
  oscillation/free-decay-style extraction as §1, just in air).
- A second, related number found in the same search sweep: a **semi-ellipsoid Munk-moment**
  comparison at 5° sideslip showed **6.34% error** between a reference value (1.687 N·m) and
  a value computed from the standard analytical formulas (1.794 N·m) — this is the *rotational*
  (moment) analogue of the 2.1% translational number, and 6.34% is a reasonable expectation
  for how much worse the rotational added-inertia coefficient (k′ in §3.1) might track versus
  the translational one (k1/k2), consistent with real hull cross-sections deviating from a
  perfect ellipsoid more strongly in the moment-generating regions (nose/tail taper) than in
  bulk translational added mass.
  Source (Munk-moment comparison, ellipsoid/semi-ellipsoid CFD validation context, found in
  the same search sweep but not independently fetched this session — **flagged unverified**):
  general search-summary from "Dynamic Analysis of Underwater Torpedo during Straight-Line
  Navigation," MDPI *Applied Sciences* 13(7):4169, https://www.mdpi.com/2076-3417/13/7/4169,
  and "A new CFD method for determination of translational added mass coefficients of an
  underwater vehicle," ResearchGate. Neither fetched in full this session.

**Bottom line for §3.4**: expect **roughly 2-6% error** for a genuinely ellipsoidal body
against a direct physical measurement — i.e. the *formula itself* is quite accurate; the
larger errors seen in real AUV work (10-40%, per §1a's CFD comparison table) come from the
**hull not actually being an ellipsoid** (thruster pods, flats, non-axisymmetric cross-
sections) rather than from any weakness in Lamb's theory. This argues for spending desk effort
on a good equivalent-ellipsoid *fit* (matching displaced volume + wetted length/diameter, per
§3.1's Humphreys/Watkinson-lineage practice) rather than doubting the closed-form k1/k2/k′
formulas themselves.

---

## Claims I could not verify this session (tracked so no one re-derives them from memory later)

1. Lamb k1≈0.209 / k2≈0.702 at fineness ratio 4 — quoted from recalled standard tables, not
   read from a fetched primary source. MUST re-derive numerically from the α0/β0 integrals in
   §3.1 before it is used in any code or report.
2. Imlay (1961) DTIC report AD0263966 — blocked by 403 from DTIC's server; the formula forms
   in §3.1 are standard and appear in multiple secondary sources, but the primary derivation
   was not read line-by-line this session.
3. MIT 13.012 Reading7 (slender body theory handout) — fetched but PDF text was not machine-
   extractable; formulas in §3.1/§3.2 are from other secondary sources, not this one.
4. "Estimation of the Hydrodynamics Coefficients of an ROV using Free Decay Pendulum Motion"
   (academia.edu) — only an abstract-level summary was available via search; specific
   accuracy numbers for the free-decay yaw fit were not independently confirmed.
5. JAFM "Identification of Drag Force of the Underwater Vehicles" — PDF text not machine-
   extractable; method summary is from the WebSearch snippet only.
6. DiVA "Generic 6-DOF Added Mass Formulation" — fetch failed (connection refused); nothing in
   this file about it beyond the title/abstract from search results.
7. "Sensitivity of AUV added mass coefficients to variations in hull and control plane
   geometry" — ScienceDirect abstract only; no accuracy number retrieved.

## 1a. UPDATE — a full worked free-decay methodology paper with real accuracy numbers

**"Experimental and Computational Methodology for the Determination of Hydrodynamic
Coefficients Based on Free Decay Test: Application to Conception and Control of Underwater
Robots"** (MDPI *Sensors* 19(17):3631, 2019; PMC6749442) is the single most useful source
found so far for Q1 because it reports actual error numbers against CFD, not just methodology.
https://pmc.ncbi.nlm.nih.gov/articles/PMC6749442/

- **Two rig types**, both on a *scale model* (30% and 22% of the full robot — note: they
  scale-modeled, we would test the real hull, which is simpler and removes scaling error):
  - *Spring-based free decay* (linear DOF): tank only 30 cm × 40 cm — i.e. **bench-scale,
    not pool-scale**. Model: `(m+m′)ÿ + c·ẏ + ½ρC_D·A·|ẏ|·ẏ + k·y = 0` (mass+added-mass,
    linear damping c, quadratic drag term, spring restoring k). Sensor: linear encoder,
    0.4 mm/count. 4 initial displacements (10/20/30/40 mm) × 6 launches per axis.
  - *Pendulum free decay* (rotational DOF): model `Θ̈ = a₁·sin(Θ) − a₂·|Θ̇|·Θ̇` — this is
    the rotational free-decay form directly relevant to our yaw case (though ours has no
    restoring `sin(Θ)` term since yaw has no righting moment; our equation reduces to just
    the `a₂|Θ̇|Θ̇` quadratic-damping term against added inertia). Sensor: MPU6000 IMU, 10 ms
    sample (100 Hz) via Kalman filter — **our 500 Hz board IMU is a strict superset of this**.
    4 initial angles (2.5°, 5°, 10°, 15°) × 6 launches per axis.
  - **Fitting methods compared**: (a) natural-frequency extraction by graphically measuring
    time between peaks; (b) logarithmic decrement of successive peak amplitudes for the
    linear coefficient; (c) "Chin method" — least-squares regression fitting both linear
    (K_L) and quadratic (K_Q) coefficients simultaneously; (d) a Morison-equation-only
    quadratic fit as an alternative. Nonlinear least squares (Chin method) is what should be
    used for us since it fits both terms at once rather than assuming one dominates.
  - **Reported accuracy vs CFD** (their ground truth) — this is the number this file was
    missing:
    - Added mass: X 10.42% error, Y 23.59%, Z 28.36% (drops to **1.49%** once near-wall
      effects are excluded — i.e. the wall-proximity confound noted in §2 above is real and
      large, ~20-27 points of error, and is the dominant error source, not the method itself).
    - Drag/quadratic coefficient (K_Q) vs CFD: X 17.65%, Y 39.34%, Z 4.84%.
    - Linear coefficient (K_L) vs CFD: X 2.3%, Y 14.05%, **Z 193.43%** (linear damping is
      essentially unidentifiable/negligible in that axis — a warning that a "linear +
      quadratic" fit can produce a near-meaningless linear term when quadratic drag actually
      dominates; a fit that forces both terms can overfit noise into a garbage linear
      coefficient).
    - Cross-check between the two independent rigs (pendulum vs spring) for drag: X 67.17%
      disagreement between rigs, Y 15.03%, Z 12.38% — i.e. **even two different physical
      free-decay rigs on the same body disagree by up to 67% on one axis**, which is the
      realistic error bar to expect from a single-rig free-decay identification, not the
      smaller CFD-comparison numbers above.

  **Bottom line for us**: free-decay is real, cheap, and gives the right functional form, but
  expect **10-40% typical error and up to ~2x error in bad axes/near walls**, and always keep
  a wall standoff (their Z-axis error dropped ~19x, 28.36%→1.49%, purely by excluding
  near-wall data).

---

## 4. Thruster identification without a load cell

- Standard method IS bollard-pull with a load cell: tether the vehicle (or bare thruster) to
  a fixed point through an inline load cell, ramp command from zero to full, log
  force-vs-command. At zero forward speed, thrust scales roughly linearly with commanded
  motor speed (squared with RPM more precisely, roughly linear with PWM/ESC command over
  the useful range) — this is the baseline bollard-pull characterization.
- **Reported degradation**: thrust force can be **degraded by up to 30% of bollard output**
  once there is ambient/forward flow past the propeller — meaning a bollard-only
  characterization (which is what we can do on the bench) systematically overstates thrust
  available at any nonzero vehicle speed. This is a real, quantified caveat on bollard testing
  from the search snippet (source below); flagged **unverified in detail** since only the
  WebSearch summary was captured, not the full paper text.
  https://www.researchgate.net/publication/221787737_Thruster_Modeling_and_Controller_Design_for_Unmanned_Underwater_Vehicles_UUVs
- **Load-cell-free alternative (acceleration method)**: with vehicle mass known (we can weigh
  the hull) and IMU acceleration logged, thrust can in principle be backed out from
  `F = (m + X_u̇)·a` during a step command in still water, PROVIDED (a) the vehicle is
  otherwise unconstrained (free-floating, not tethered) so the whole force goes into
  acceleration, and (b) drag at the transient speed is either negligible (very start of the
  step, before speed builds up) or is itself known from the coast-down fit in §2 — i.e. this
  method is **not independent of §2**, it presupposes you already have a drag model, making
  it useful mainly for *cross-checking* a bollard number rather than as a from-scratch thruster
  identification. No source found this session gives a reported accuracy number for
  acceleration-based (load-cell-free) thrust identification specifically — **flagged as an
  open gap**; only the bollard-pull method and its documented degradation-under-flow were
  found with numbers.
- A dedicated low-cost thruster test-bench design exists ("A Low-Cost Test Bench for
  Underwater Thruster Identification," ScienceDirect/HardwareX) but the full text was
  paywalled/blocked (403) this session — **not independently verified**, listed for a later
  fetch attempt via a different route (e.g. Google Scholar cache or arXiv mirror if one
  exists). https://www.sciencedirect.com/science/article/pii/S2405896319322013

---

## 5. Excitation signal design for confined-space identification

- Three standard families, all desk/software work, no special facility needed beyond
  whatever pool is used for the actual run:
  - **PRBS (pseudo-random binary sequence)**: switches between two levels (e.g. full-forward
    / full-reverse thrust); good spectral coverage for linear-model ID. An
    **amplitude-modulated PRBS (APRBS)** variant has specifically been reported as optimized
    for estimating AUV hydrodynamic derivatives (search-result summary, source below;
    **not independently verified in full text**).
  - **Multisine**: sum of sinusoids at chosen frequencies, periodic and broadband with a
    low crest factor — preferred when frequency-domain identification is the goal because
    periodicity lets you average out noise and eliminate spectral leakage. More setup
    complexity than PRBS or chirp.
  - **Chirp (swept sine)**: frequency ramps continuously across a band; simplest to generate,
    good for quickly characterizing the frequency range where the vehicle responds, but worse
    spectral properties than multisine for precise parameter fitting.
  - A **Bayesian optimal-input-design** approach has specifically been applied to AUV system
    identification to choose the best excitation given a bounded test space (relevant to our
    "pool-sized maneuver" constraint) — found only as a search-summary reference, not fetched
    in full. https://www.researchgate.net/publication/224639225 (multisine, general) and
    ScienceDirect "Optimal design of excitation signal for identification of nonlinear ship
    manoeuvring model" https://www.sciencedirect.com/science/article/abs/pii/S0029801819308789
    (ship-specific, **not fetched**, abstract-level only).
  - No source found this session gives a concrete "here is a maneuver sequence sized to fit
    inside an N-meter pool" worked example — **flagged as an open gap**; the free-decay and
    coast-down protocols in §1/§2 above are the closest thing to pool-sized worked recipes
    found, and neither is a PRBS/multisine/chirp maneuver, they're both simple excite-then-cut
    impulse tests, which is arguably the *right* choice for a small pool anyway (an impulse
    response needs far less linear run length than a persistent PRBS/chirp sweep).

---

## 6. ⭐ What does a model measurably buy? (honest comparison, with numbers)

**Best real-vehicle, real-pool number found this session:**

"Experimental Validation of a Model-Free High-Order Sliding Mode Controller with Finite-Time
Convergence for Trajectory Tracking of Autonomous Underwater Vehicles" (MDPI *Sensors*
22(2):488, 2022; PMC8780515). https://doi.org/10.3390/s22020488

- Tested in a **real semi-Olympic swimming pool** (i.e. a facility directly comparable in
  scale to what we have), not simulation.
- Their model-free (!) high-order sliding-mode controller (HOSMC) vs a **tuned PID**, on the
  same real vehicle:
  - Depth/heading tracking error reduced **up to 75%** (HOSMC vs PID) and **41%** (HOSMC
    finite-time vs the same HOSMC with asymptotic convergence, i.e. a second, smaller
    comparison — read the 75%/41% pair as two separate comparisons, not both vs PID; the
    41% figure is HOSMC-vs-HOSMC-variant, **flagged for re-verification** against the full
    text since it came from a search snippet, not a fetched page).
  - Energy consumption reduced **35%** (vs PID) and **50%** (vs asymptotic-convergence
    HOSMC).
  - Achieved RMSE of **1 cm on depth** and **2.7° on heading** tracking in the real-pool
    trial.
- **Important nuance for our question**: this is a *model-free* controller beating PID by a
  large margin — i.e. the headline result here is not "an identified model beats PID," it's
  "a well-designed *model-free* nonlinear controller (sliding mode) beats PID by a lot,
  without needing hydrodynamic coefficients at all." This is actually a data point *against*
  spending pool time on identification if the goal is closed-loop tracking performance: the
  literature suggests the bigger lever is controller architecture (SMC vs PID), not
  necessarily having accurate N_r/X_u coefficients. **This directly informs Q6's honest
  answer: identification may matter more for feedforward/open-loop moves (dead-reckoning
  during DVL-less coast, thrust allocation, `torque=K·ω|ω]` compensation) than for closed-loop
  tracking accuracy, where controller design dominates.**
- Full text not yet fetched (403/CAPTCHA-blocked this session on two attempts) — the numbers
  above are from a WebSearch snippet that quoted the abstract; **flagged for one more fetch
  attempt** (e.g. via the DOI page https://doi.org/10.3390/s22020488 directly rather than the
  PMC mirror) before being cited as final in any report.

No source found this session gives a matching pool-scale number specifically for
"feedforward/model-based term added to a well-tuned PID" (as opposed to a wholesale different
controller architecture like SMC) — **this remains the single biggest open gap** relative to
what the task asked for in Q6. The i-PID+PD-feedforward paper found earlier
(ResearchGate/AVESIS, "Intelligent-PID with PD Feedforward Trajectory Tracking Control of an
AUV") is qualitatively on-topic but no numeric RMSE/error-percentage table was retrieved from
it this session.

---

## 8. IMU + optical flow only (no DVL) — directly relevant to our exact sensor set

- Found a **directly analogous open-source project**: `cougars-auv/coug_visual_dvl` (GitHub,
  EC EN 631 class project) — builds a vision-based velocity sensor from a **downward-facing
  stereo camera** as a substitute for acoustic DVL, explicitly to cover DVL bottom-lock
  dropouts, fused with IMU for dead-reckoning during outages. Pipeline: camera calibration →
  feature tracking → optical flow → outlier rejection → stereo depth → 3D velocity. This is
  the same sensing concept as our validated downward-camera bottom-track velocity, but with
  stereo depth instead of a separate depth sensor. No numeric accuracy was published in the
  repo content fetched (project scoped as "planned validation," not completed, at time of
  fetch). https://github.com/cougars-auv/coug_visual_dvl — **treat as a design-pattern
  reference, not a validated-accuracy source.**
- Other IMU-only/no-DVL dead-reckoning work found (search-summary level, not fetched in full):
  - "System Identification and Navigation of an Underactuated Underwater Vehicle Based on
    LSTM" (MDPI JMSE 13(2):276, 2025) — dead reckoning with **no DVL, no sonar, no GPS**,
    using an LSTM-learned system-identification model + IMU + magnetometer; reported
    "reliable position prediction" but no error number captured from the snippet.
    https://doi.org/10.3390/jmse13020276
  - Laser-based vision + IMU localization for an ROV, velocity estimated via a neural network
    feeding dead reckoning (older work, exact source title: "Localization of an underwater
    vehicle using an IMU and a laser-based vision system," ResearchGate) — **abstract-level
    only**, no numbers retrieved.
  - A separate NECF (nonlinear explicit complementary filter) + model approach reportedly kept
    **position error within 1 m over 2000 s** in *simulation* (not real water) — flagged
    explicitly as simulation-only, not a real-vehicle number, do not conflate with our
    validated real-water 1.09 cm/30 cm downward-camera result.
- **Bottom line for Q8**: our situation (downward camera validated as bottom-track velocity
  sensor to 1.09 cm/30 cm, fused with a 500 Hz IMU) is *already* at or above the sophistication
  of the open-source/published work found this session — none of the sources found report a
  real-water accuracy number better than what we already have measured. The open question is
  whether that same camera-derived velocity can be fed back into the *identification* tests in
  §1/§2 (it already is our speed sensor for coast-down) rather than whether better methods
  exist for the sensing itself.

---

## 7 / Fossen conventions and comparable-vehicle practice — status

Confirmed from search-summary level (not deep-fetched this session):
- Fossen's SNAME-derived notation (`X_u̇, Y_v̇, N_ṙ`, etc. for added mass; `X_u, X_|u|u` etc.
  for linear/quadratic damping) is the de facto standard notation across the AUV literature
  surveyed this session — every paper found (Girona 500, REMUS/Prestero, the free-decay
  Sensors paper, Martin & Whitcomb) uses this convention or a direct variant, so our BUGS/
  ROADMAP docs should adopt the same symbol set when this work is written up, for
  compatibility with cited sources.
- Girona 500: explicitly reported as **not amenable to simple strip theory** because it's a
  three-hull frame, not a torpedo body — the opposite of our situation. Our single torpedo-
  form hull (fineness 3.99) is a *better* fit for the ellipsoid/strip-theory methods in §3
  than Girona 500 was, which is a point in favor of trusting the analytical route for us.
  https://www.researchgate.net/publication/235955179_Girona_500_AUV_From_survey_to_intervention
- Martin & Whitcomb, "Experimental identification of six-degree-of-freedom coupled dynamic
  plant models for underwater robot vehicles," IEEE J. Oceanic Eng. 39(4):662-671, Oct 2014 —
  identified via least-squares from **prescribed maneuvers** with full-DOF coupling (not
  simple single-axis free-decay); a later paper (Smallwood & Whitcomb) reportedly showed
  **online adaptive identification outperforming offline least-squares**, but limited to
  single-DOF dynamics — both citations at search-summary level only, **not fetched in full,
  flagged for follow-up** if precise error numbers are needed.
- Hegrenaes & Hallingstad (2011), model-aided INS for HUGIN 4500/3000: demonstrated that
  adding a hydrodynamic vehicle model as an aiding source (software only, no new hardware)
  measurably improves INS accuracy and robustness during DVL dropouts — directly supports the
  idea that even an approximate model (from §3's desk-only route) is useful as a *navigation*
  aid, separate from its value for control. No numeric accuracy improvement figure was
  captured this session — **flagged for follow-up fetch** of the original paper (title:
  "Model-Aided Inertial Navigation for Underwater Vehicles").
  https://www.researchgate.net/publication/224318397

This scale of vehicle (fineness-ratio torpedo hull, roughly 10-30 kg class once weighed) sits
well below HUGIN/Girona-class work rigor; Prestero's REMUS thesis (MIT, 2001, "Verification of
a Six-Degree of Freedom Simulation Model for the REMUS AUV," darchive.mblwhoilibrary.org) is
the closest scale/class match found and is the standard reference for "how much rigor is
appropriate for a REMUS-class (~30 kg, torpedo-hull, no strip-theory-defeating appendage
cluster) vehicle" — full text not fetched this session, **flagged for a follow-up fetch**,
but its existence and scope are confirmed from multiple independent search hits.
https://darchive.mblwhoilibrary.org/handle/1912/3040

---

## Claims I could not verify this session (running list)

1. Lamb k1≈0.209 / k2≈0.702 at fineness ratio 4 — recalled from memory, NOT read from a
   fetched primary source. Re-derive numerically from the α0/β0 integrals in §3.1 before use.
2. Imlay (1961) DTIC AD0263966 — blocked (403). Formula forms in §3.1 are standard/secondary-
   sourced only.
3. MIT 13.012 Reading7 — fetched but PDF text not machine-extractable.
4. Academia.edu ROV free-decay pendulum papers (two separate titles found, "Estimation of..."
   and "Identification of...", possibly the same underlying work) — abstract/search-summary
   level only.
5. JAFM "Identification of Drag Force of the Underwater Vehicles" — PDF text not extractable.
6. DiVA "Generic 6-DOF Added Mass Formulation for Arbitrary Underwater Vehicles" — connection
   refused both attempts.
7. "Sensitivity of AUV added mass coefficients to variations in hull and control plane
   geometry" (Ocean Engineering) — abstract only, paywalled.
8. MDPI "System Identification and Controller Design of a Novel AUV" (Machines 9(6):109) —
   403 blocked.
9. ScienceDirect "A Low-Cost Test Bench for Underwater Thruster Identification" — 403 blocked.
10. Bollard-pull "30% degradation under ambient flow" figure — from a WebSearch summary of
    "Thruster Modeling and Controller Design for UUVs," not a fetched primary text.
11. APRBS-for-AUV-hydrodynamic-derivatives claim — search-summary level only.
12. Sliding-mode-vs-PID 75%/41%/35%/50%/1cm/2.7° figures (§6) — from a WebSearch snippet
    quoting the abstract of https://doi.org/10.3390/s22020488; two direct fetch attempts were
    blocked (403, then CAPTCHA via the PMC mirror). Numbers should be treated as
    high-confidence-but-not-independently-read-in-full until one more fetch attempt succeeds.
13. Martin & Whitcomb (2014) and Smallwood & Whitcomb online-adaptive-vs-least-squares claims
    — search-summary level only, not fetched.
14. Hegrenaes & Hallingstad (2011) HUGIN model-aided INS accuracy improvement — existence and
    qualitative conclusion confirmed by multiple search hits, no numeric figure captured.
15. Prestero (2001) REMUS 6-DOF thesis — existence/scope confirmed, full text not fetched.
16. LSTM-based no-DVL dead reckoning (JMSE 13(2):276, 2025) — search-summary only.
17. NECF-based no-DVL navigation "<1 m over 2000 s" — explicitly simulation-only per the
    snippet; not a real-water number, do not cite as validating real performance.

**Confirmed with a fetched primary source and a real number (highest confidence in this
file):** the MDPI Sensors 19(17):3631 free-decay methodology and its CFD-comparison error
table in §1a — this is the one section of this file backed by a fully read, non-paywalled
source with numbers.

18. Sensors 22(2):488 (HOSMC-vs-PID) — two direct fetch attempts on the MDPI page itself both
    403'd (in addition to the earlier PMC CAPTCHA block); the 75%/41%/35%/50%/1cm/2.7° numbers
    in §6 remain sourced only from WebSearch abstract snippets, never a fetched full page, in
    this session. Highest remaining priority for one more fetch attempt in a future session
    (try Google Scholar cache, a university repository mirror, or ResearchGate).
19. Airship added-mass "2.1% deviation" and "6.34% Munk-moment error" (§3.4) — the 2.1% number
    is from a WebSearch summary of MDPI Aerospace 11(11):872, not a fetched full page; the
    6.34% Munk-moment figure is doubly indirect (from a search-summary that itself cites other
    unfetched sources). Both should be re-verified by fetching
    https://doi.org/10.3390/aerospace11110872 directly before being treated as final.

## Summary of what needs water vs bench vs desk (quick reference)

- **[DESK] only, no test needed**: §3 added mass from geometry (ellipsoid/Lamb k-factors,
  equivalent-ellipsoid fitting, strip theory) — expect ~2-6% error if the hull is well-
  approximated by an ellipsoid, worse (10-40%+) if not, per §3.4 vs §1a.
- **[BENCH]**: weighing the hull for mass; bollard-pull thruster characterization (load cell),
  noting the ~30%-under-flow degradation caveat in §4; a small-tank spring/pendulum free-decay
  rig if we want the sub-scale-model rigor of §1a rather than full-scale pool free-decay.
- **[WATER/POOL]**: full-scale free-decay (yaw, and heave/pitch if excitable) per §1, coast-
  down for surge drag per §2, and any closed-loop controller validation (§6).
- **Biggest confounder across every water test**: wall proximity and free-surface proximity,
  which the fetched §1a source shows can swing added-mass error by ~19x (28.36%→1.49%) simply
  by excluding near-wall data — keep every free-decay/coast-down run at mid-depth and at least
  1-2 hull lengths from any wall.
