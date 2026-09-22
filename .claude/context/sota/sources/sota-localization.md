# SOTA: AUV State Estimation — Never Losing Where You Are, and Recovering When You Do

> Central question: how does a vehicle avoid ever losing where it is, and how does it recover
> when it does? Evidence only, every claim a titled markdown link to the source + a number. Researched 2026-09-22.
> **NOTE**: WebSearch budget for this session was exhausted (200/200) before this task could
> issue any searches — all research below comes from WebFetch against known/guessed URLs
> (arXiv abstracts, ar5iv full text, direct paper/vendor pages). This materially limited
> coverage of topics 5, 6, 8, 9 below (field-system reset practices, relocalization, cheap
> acoustics, team TDRs) where I had no seed URL to fetch. Flagged per-section.

## 1. Invariant EKF vs fixed-lag smoothing

### Right-invariant fixed-lag smoother (RI-FLS)

[Right Invariant Fixed-Lag Smoother](https://arxiv.org/abs/2102.08596) — first analysis of a
fixed-lag smoother (FLS) with a right-invariant error, framed as an alternative to
First-Estimate-Jacobian (FEJ) for consistency without absolute position measurements. Applied
to monocular VIO/SLAM.

On the EuRoC benchmark, RI-FLS vs an incremental (non-invariant) FLS, position RMSE:

| Sequence | Incremental FLS | RI-FLS | RI-FLS (exact) |
|---|---|---|---|
| MH_01 | 0.88 m | **0.53 m** | 0.82 m |
| MH_05 | 0.68 m | 0.89 m | 1.26 m |
| V1_02 | 0.28 m | 0.28 m | 0.39 m |
| V2_02 | 0.24 m | 0.29 m | 0.23 m |

Mixed result — RI-FLS wins 2 of 4 sequences, loses 2. Time horizon used for the smoother
window: **1 second** on real data (a much larger horizon was used only for a simulation
sanity check against iSAM2/batch). **No runtime/latency numbers are reported in the paper** —
cannot answer the Pi-5-feasibility question from this source.

### T-ESKF (transformed error-state KF)

[T-ESKF (arXiv 2510.23359)](https://arxiv.org/abs/2510.23359) — applies a linear
time-varying transformation to the ESKF error-state so the unobservable subspace stays
independent of the state estimate (an alternative consistency fix to FEJ/invariant filters).
100-run Monte Carlo, orientation(deg)/position(m) RMSE:

| Trajectory | T-ESKF | ESKF (baseline) |
|---|---|---|
| Udel-Gore | 0.58° / 0.20 m | 1.13° / 0.26 m |
| Udel-Neighborhood | 6.83° / 41.8 m | 11.6° / 72.7 m |
| TUM-Corridor | 0.34° / 0.15 m | 0.68° / 0.28 m |

Real-world (EuRoC/TUM-VI) ATE on V1_01: both **0.75 m / 0.06** — no gain there. Runtime:
tested on an **R9 7950X @ 4.5 GHz** (desktop CPU, not embedded), reported qualitatively as
"comparable" per-frame time to plain ESKF — **no millisecond figure given**, and the hardware
is nowhere near a Pi 5, so this does not answer the fixed-lag-window-on-Pi-5 question either.

### I-EKF consistency theory

[Invariant-EKF SLAM consistency (arXiv 1702.06680)](https://arxiv.org/abs/1702.06680) —
proves RI-EKF output is invariant under any *stochastic* rigid-body transformation (SO(3)-EKF
SLAM is only invariant under *deterministic* ones); Monte Carlo shows RI-EKF beats SO(3)-EKF,
Robocentric-EKF and FEJ-EKF on 3D point-feature SLAM, but **no numeric RMSE values are stated
in the abstract/available text** — theory-only claim I could not quantify.

[RIEKF-VINS (arXiv 1702.07920)](https://arxiv.org/abs/1702.07920) — shows conventional EKF
VINS is *not* invariant under the stochastic unobservable transformation (translation + yaw
about gravity), which causes inconsistency; proposes RIEKF-VINS inside an MSCKF framework.
Validated in simulation + real experiments but **no numeric comparison extracted** from the
fetched content.

**Bottom line for our filter**: none of these four papers gives a CPU-cost number I can hold
against a Pi 5 budget. The consistent theme across all four is that invariant/consistency-
aware formulations buy accuracy (roughly 2× error reduction vs a naive baseline in the T-ESKF
numbers above) but the papers evaluated on desktop/laptop-class hardware, not embedded ARM.

### Observability theory (Barrau/Bonnabel symmetry-based EKFs)

[Exploiting Symmetries to Design EKFs with Consistency Properties (arXiv 1903.05384)](https://arxiv.org/abs/1903.05384)
— shows a symmetry-based ("equivariant") EKF linearization automatically captures the
unobservable directions of navigation/SLAM problems without the restrictive assumption that
Jacobians are evaluated at ground truth. Claims it "outperforms standard EKF" and reaches
"comparable performance to iSAM" on real multi-robot SLAM data — no numeric RMSE extracted
from the abstract, theory-only citation.

## 2. Factor graphs in practice

[Contact-Aided Factor-Graph Localization for Underwater Sampling (arXiv 2608.26932)](https://arxiv.org/abs/2608.26932)
— GTSAM-style factor graph fusing IMU+DVL+pressure+magnetometer (all **10 Hz**) + camera
(**2 Hz**) + manipulator-contact events as implicit loop-closure factors, local bundle
adjustment window tested at **10-pose** and **25-pose** widths. Measured numbers:

| Scenario | Metric | Value |
|---|---|---|
| Tank, straight motion (best case) | ATE | **0.0990 m** |
| Tank, depth+landmarks | Yaw RMSE / RPE | 0.0412 rad / 0.0489 m |
| Stonefish simulation | ATE range | 0.0847–0.998 m |
| Harbor (real deployment) | ATE / Yaw RMSE | **0.8223 m** / 0.2121 rad |
| Ablation: DVL+IMU+pressure only | ATE | 0.6780 m |
| Ablation: full system (+ vision + contact) | ATE | **0.0990 m — an 87 % reduction** |
| Harbor task-level revisit error | landmarks alone → contact-aided | 1.23 m → 1.12 m → **0.90 m (27 % further gain)** |

**No CPU/update-time number reported** — the paper does not say whether this runs in real
time on embedded hardware; it is a **DVL-equipped** platform, which we do not have.

## 3. Process noise

Not yet resourced with a number for Allan variance of a BNO085-class IMU — flagged below
under "claims I could not verify." Semantic Scholar / arXiv search located adjacent invariant-
filter tuning papers (see §1) but none gave a concrete Allan-variance number for a consumer
9-DoF IMU in the calls made so far.

## 6/7/8. Acoustics, observability with sparse sensors, single-beacon localization

[Synchronous-Clock Range-Angle Relative Acoustic Navigation (arXiv 2110.13825, published *Field Robotics* 2 (2022) 774–806)](https://arxiv.org/abs/2110.13825)
— single broadcasting acoustic beacon + chip-scale atomic clocks (CSAC) + a fixed USBL
receiver array on each vehicle, validated against a secondary long-baseline (LBL) system, on
a fleet of **three SandShark AUVs**. No numeric accuracy figure was recoverable from the
fetched abstract (34 pages, 17 figures — the number lives in the body). This is the clearest
example found of **single-beacon, no-inter-vehicle-comms acoustic localization** working in
the field, but the accuracy claim could not be verified here.

[Sensor Misalignment-Tolerant AUV Navigation with Passive DoA and Doppler (arXiv 2402.07218)](https://arxiv.org/abs/2402.07218)
— single acoustic array + dead reckoning, UKF + nonlinear least squares, tolerant to array
misalignment. No numeric accuracy extracted from the fetched abstract.

[Underwater Doppler Navigation with Self-Calibration (arXiv 1509.02054)](https://arxiv.org/abs/1509.02054)
— proves the IMU+DVL combined system is **observable under moderate conditions**, including
in-situ calibration of DVL scale factor and misalignment angle **without GPS or acoustic
aiding**. No numeric calibration-accuracy value extracted — theory/observability claim only.

[Unscented Kalman Filtering on Manifolds for AUV Navigation (arXiv 2210.06510)](https://arxiv.org/abs/2210.06510)
— fuses DVL + depth + acoustic range + GPS on a UKF-on-manifold, explicitly demonstrates
**convergence to correct heading from an arbitrarily large initial heading error** — directly
relevant to "how do you recover a heading after a blackout," but no numeric convergence time
or residual heading error was extracted from the abstract alone.

[FGO-ILNS (arXiv 2310.14163)](https://arxiv.org/abs/2310.14163) — factor-graph-optimization
tightly-coupled multi-sensor AUV nav, claims "multi-sensor plug-and-play despite data
frequency" and unifies above/below-water positioning via floating-LBL slant-range-difference
factors. No numeric accuracy extracted.

### Terrain-aided navigation (as the "one extra source" for topic 7)

[Contours-Seeking Proposal Density Particle Filter for Terrain-Referenced Navigation (arXiv 2608.15489)](https://arxiv.org/abs/2608.15489)
— addresses particle-filter degeneracy in terrain-referenced nav with a Gaussian-mixture
proposal density that steers particles toward plausible terrain contours; reports improved
weight-variance and effective-sample-size metrics under "severe prediction bias and
multimodal measurement noise" but **no numeric accuracy value extracted** from the abstract.
Relevant because we have **no terrain map** at all — this is the class of source topic 7 asks
about, and even in the literature it needs a bathymetry map we don't carry.

## 4. Consistency monitoring (NEES/NIS in practice)

[Weak in the NEES? Auto-tuning Kalman Filters with Bayesian Optimization (arXiv 1807.08855)](https://arxiv.org/abs/1807.08855)
and its follow-up
[Kalman Filter Auto-tuning through Enforcing Chi-Squared Normalized Error Distributions with Bayesian Optimization (arXiv 2306.07225)](https://arxiv.org/abs/2306.07225)
are the two clearest sources on NEES/NIS consistency testing found. Key statements pulled
from the full text of 1807.08855: NEES/NIS consistency is formalized as two-sided χ² tail
bounds **[l(α,N), u(α,N)]** at a chosen Type-I error rate **α**, evaluated over **N** Monte
Carlo runs — our own gate (χ² 99 %) is the same family of test, just one-sided and single-run
rather than tail-bounded over an ensemble. The follow-up paper's key empirical finding: **"NIS
and NEES errors are only chi-squared distributed for tuned estimators"** — i.e. a wrongly
tuned Q/R does not merely bias the filter, it makes the chi-squared consistency test itself
invalid, so a gate built on assumed-chi-squared innovations can silently stop meaning what it
claims to mean if Q/R is wrong. Neither paper gave concrete before/after percentage numbers in
the sections fetched — the worked examples were toy 1-D systems recovering a known true Q
(e.g. estimated V ≈ 1.000 against ground truth V = 1).

**No published numeric divergence-detector threshold or false-positive/detection-rate table
was located for a robot state estimator in the calls made.** This is a real gap in the
research, not just a fetch failure — see "claims I could not verify" below.

## 5. Covariance bounds, inflation, and filter resets

[An Optimal Experimental Design Framework for Adaptive Inflation and Covariance Localization for Ensemble Filters (arXiv 1806.10655)](https://arxiv.org/abs/1806.10655)
— the only inflation-specific source located. It is an **ensemble/data-assimilation** paper
(geoscience lineage, not robotics), proposing a variational framework to adaptively tune
inflation and localization parameters rather than fix them by hand. Confirms covariance
inflation is "ubiquitously employed to alleviate the effect of using ensembles of finite
size" — i.e. inflation is standard practice in that adjacent field — but gives **no numeric
inflation factor, no trigger threshold, and is not a filter-reset paper** in the sense our
system needs (our lockout break inflates P ×4 after 5 rejections; I found no published
robotics analogue with a comparable published number to check that ×4/5-rejection choice
against).

## 6b. Relocalization after a blackout — kidnapped-robot recovery

No paper with a quantified relocalization-accuracy or recovery-time number was located.
A targeted arXiv query for "kidnapped robot relocalization recovery localization failure"
returned only an unrelated basketball-robot localization paper. This is flagged as
unverified/unresourced below, not because the literature doesn't exist (kidnapped-robot
recovery via global relocalization — e.g. Monte Carlo Localization re-seeding, place
recognition — is a large, well-known field on land and in air) but because this session's
search tooling (see note at top of file) could not reach it.

## 9. What top RoboSub/SAUVC teams run

**Not resourced.** RoboSub Technical Design Reports (TDRs) are hosted on RoboNation's site
and individual team websites/GitHub repos as PDFs, not indexed by arXiv or Semantic Scholar,
and this session had no working web-search tool to locate them (WebSearch was already
exhausted for the session before this task started; DuckDuckGo and Bing were blocked by
CAPTCHA/empty results via WebFetch). No claim is made here about what BumblebeeAS or any
other team runs for state estimation beyond what is already in this repo's own
`bumblebee-doctrine` skill — that skill should be treated as the source for that question,
not this file.

## Summary table

| Source / method | What it observes | Accuracy | Pi-5 cost | What it needs that we lack |
|---|---|---|---|---|
| [RI-FLS](https://arxiv.org/abs/2102.08596) | monocular VIO pose | 0.23–1.26 m pos RMSE (EuRoC, mixed vs baseline) | not reported | camera+IMU only, no DVL — closest sensor match to us, but no CPU number |
| [T-ESKF](https://arxiv.org/abs/2510.23359) | VIO pose | 0.15–0.34 m / 0.34–0.58° (best cases) vs 0.26–0.28 m / 0.58–1.13° (ESKF) | tested on desktop R9 7950X only | consistency-preserving transform, not proven on ARM |
| [Contact-aided factor graph](https://arxiv.org/abs/2608.26932) | IMU+DVL+pressure+mag(10 Hz)+camera(2 Hz)+contact | 0.099 m ATE best case, 0.82 m harbor, 87% gain over DVL-only baseline | not reported | **DVL** (we have none), manipulator contact events |
| [UKF-on-manifold](https://arxiv.org/abs/2210.06510) | DVL+depth+acoustic range+GPS | converges from arbitrarily large initial heading error (no numeric residual found) | not reported | DVL + acoustic range + occasional GPS (surfaced) |
| [Synchronous-clock single-beacon USBL](https://arxiv.org/abs/2110.13825) | one acoustic beacon + CSAC clock + USBL array | validated vs LBL, no number recovered | not reported | one CSAC + one USBL array per vehicle — we have neither |
| [Contours-seeking terrain PF](https://arxiv.org/abs/2608.15489) | bathymetry contour matching | ESS/weight-variance improved, no accuracy number recovered | not reported | a terrain/bathymetry map — we have none |
| [Chi-squared auto-tuning](https://arxiv.org/abs/1807.08855) | NEES/NIS as tuning objective | recovers true Q to ~0.1% in toy 1-D case | trivial | an ensemble of Monte Carlo runs to evaluate tail bounds, not a single-run gate |

## How a system says "I am lost"

The literature gathered here is almost entirely about **preventing** inconsistency
(invariant filters, consistency-aware transforms, factor-graph smoothing with loop closures)
rather than **detecting and announcing** that the estimate has already diverged. Concretely:

- The NEES/NIS papers ([1807.08855](https://arxiv.org/abs/1807.08855), [2306.07225](https://arxiv.org/abs/2306.07225))
  treat chi-squared consistency as a **tuning objective** (offline, ensemble-based, used to
  pick Q/R) rather than an online, single-run "we are now lost" alarm — and the 2306.07225
  finding that NIS/NEES are only chi-squared-distributed *for an already-tuned filter* means a
  naive online chi-squared gate is not a reliable lost-detector on its own; it can pass
  cleanly while badly tuned, or fail cleanly while well tuned but genuinely diverged, and
  nothing in the fetched material gives a way to tell those apart from the gate statistic
  alone.
  - This directly supports why our own gate + lockout-break (bypass after 5 rejections, ×4
    inflation) is not, by itself, a divergence detector: it is a robustness mechanism against
    isolated bad measurements, not a monitor of whether the filter's own belief about its
    uncertainty is still correct.
- The ensemble-inflation paper ([1806.10655](https://arxiv.org/abs/1806.10655)) confirms
  covariance inflation is standard *mitigation* practice once divergence is suspected, but
  offers no trigger rule.
- **No published, quantified "I am lost" signal for a field robot state estimator was located
  in this session** — this is the single most important unresourced claim below.

## Claims I could NOT verify

- **Any published divergence detector for a robot state estimator with a stated
  detection-rate/false-alarm-rate table.** Searched via arXiv API and Semantic Scholar API
  (rate-limited to 3 successful calls before 429s); found tuning-oriented NEES/NIS papers but
  no online-monitoring papers with numeric performance.
- **CPU/wall-clock cost of a fixed-lag smoother or factor graph on embedded ARM (Pi-5 class)
  hardware.** All four smoother/factor-graph papers fetched (2102.08596, 2510.23359,
  2608.26932, 1903.05384) either report no timing number or benchmark on a desktop CPU
  (R9 7950X @ 4.5 GHz). Cannot answer "can a fixed-lag window run on a Pi 5 beside a vision
  graph" from evidence gathered.
- **Allan-variance numbers for a BNO085-class consumer IMU.** The CEVA/Hillcrest datasheet URL
  redirected/404'd and a Mouser-hosted copy timed out; Adafruit's product page (a common
  secondary source) does not list noise specs. No number obtained — so I also cannot say
  whether a zero position-process-noise block is defensible against a measured Allan-variance
  budget; that claim in the CONTEXT section remains unverified from a citable source.
  Semantic Scholar search for Allan-variance-focused papers 429'd before returning results.
  **Retry recommended**: fetch `https://www.ceva-ip.com` product pages (URL structure changed
  since 2019), or the CEVA "SH-2 Reference Manual," when web search tooling is available.
- **Numeric accuracy for the single-beacon synchronous-clock system**
  ([2110.13825](https://arxiv.org/abs/2110.13825)) — the abstract confirms the architecture
  and that it was validated against LBL, but the actual error-in-meters figure is in the
  34-page body, not the abstract; WebFetch on the abstract page could not reach it.
- **What top RoboSub/SAUVC teams run for state estimation, with a published drift number.**
  TDRs are not indexed by arXiv/Semantic Scholar; WebSearch was unavailable for the entire
  session (budget exhausted before this task began) and DuckDuckGo/Bing returned CAPTCHA or
  irrelevant results via WebFetch. Recommend re-running this specific topic once WebSearch
  quota resets, or fetching RoboNation's TDR archive URL directly if a link is known.
- **Kidnapped-robot / relocalization-after-blackout accuracy and recovery-time numbers.** A
  large body of land/air robotics literature on this exists (MCL re-seeding, global place
  recognition) but no specific paper with numbers was retrieved in the calls made; the arXiv
  query returned only an unrelated basketball-localization paper.
- **RI-EKF and RIEKF-VINS ([1702.06680](https://arxiv.org/abs/1702.06680),
  [1702.07920](https://arxiv.org/abs/1702.07920)) numeric Monte Carlo results** — both papers'
  abstracts assert RI-EKF/RIEKF-VINS "outperforms" baselines but neither fetch surfaced actual
  RMSE/ATE numbers; only the full PDF body would have them.

## Session tooling note

This subagent's WebSearch tool reported its budget (200/200) already exhausted before any
search in this task was issued — i.e., the limit is shared across the whole parent session,
not per-task. All research above was done via WebFetch against arXiv abstract pages, the
arXiv API (`export.arxiv.org/api/query`, unauthenticated, worked reliably), and the Semantic
Scholar Graph API (worked twice, then rate-limited at 429 for the remainder of the session).
DuckDuckGo and Bing were tried as WebFetch-only search substitutes and both failed (CAPTCHA /
irrelevant results). If this task is re-run, either fresh WebSearch quota or a working
proxy search tool would close most of the gaps listed above.
