# Localization, against the world

> Dive 3 of four. Method and rubric: [`README.md`](README.md). Our side is read out of the tree
> at `file:line`. Their side carries a URL and a number.

---

## 1. What we do today

### 1.1 The filter

`src/mongla_localization/mongla_localization/inekf.py`, 443 lines, pure numpy.

- **Right-invariant EKF on SE₂(3)**, `DIM = 15`, error order `[θ | v | p | b_g | b_a]`
  (`:158-166`). World is **NED**; the change from z-up is recorded with its evidence — 583 m →
  0.036 m over 50 s of real board data (`:76`).
- Predict (`:190-224`): `a_world = R a + g`, `p += v dt + ½a dt²`, `v += a dt`, `R ← R·exp(ω dt)`.
  Error transition `Φ = I + A dt` — **first-order Euler, no matrix exponential**; `Qd = Q·dt` —
  **linear in dt, no Van Loan**.
- Injection is **left**: `R ← δR·R`, `v ← δR·v + δv`, `p ← δR·p + δp` (`:422-436`). Biases are
  additive outside the exponential. The only manifold ops present are `so3_exp` / `so3_log` —
  **no SE₂(3) exp/log, no adjoint, no Γ functions**.
- **Q and P₀ are unmeasured nominal diagonals** (`:169-181`): σ_gyro 0.01, σ_accel 0.1, bias
  1e-4 / 1e-3; P₀ 0.3 / 0.5 / 1.0 / 0.01. **The position block of Q is exactly zero.** Nothing in
  the tree derives these from an Allan variance.
- One shared gate (`:355-420`): NIS against `CHI2_99 = {1: 6.635, 2: 9.210, 3: 11.345}`, plus a
  **lockout break** — after 5 consecutive rejections the gate is bypassed once and `P *= 4.0`.
  ⛔ That runs *toward* the measurement; it is not a divergence guard.

### 1.2 The eleven sources, and what ships on

| # | source | H | gate | default |
|---|---|---|---|---|
| 1 | IMU predict | — | dt ≤ 1e-4 dropped, dt > 0.25 s **skipped entirely** | on |
| 2 | board attitude | `H[:,0:3]=I` | σ 0.5°; the **first** valid attitude is *adopted*, not corrected (correcting into −168.3° cost 345 rejections and 0.9 m) | on |
| 3 | depth | `H[0,8]=1`, `−skew(p)[2]` | σ 0.02 m | on |
| 4 | board yaw | analytic ∂yaw/∂δθ | σ 2° | **off** (`use_yaw=False`) |
| 5 | optical flow | `H[:,3:6]=(Rᵀ)[:2,:]` | R from the flow node's RANSAC SEM | on when `flow:=true` |
| 6 | ZUPT | `H[:,3:6]=Rᵀ` | 50-sample stillness, ω ≤ 0.02 rad/s, a p-p ≤ 0.25 m/s², flow stale > 1 s | on |
| 7 | heading anchor | as #4 | support ≥ 6, spread ≤ 6° | latched, on arrival |
| 8 | tile grid | as #4 | snap mod 90°, refuse residual > 20° | **off** (`tile_m=0`) |
| 9 | lane line | as #4 | snap mod 180° | **off** |
| 10 | prop fix | `H[0,6]=1, H[1,7]=1` | σ from `point.z` | mission-called |
| 11 | command-velocity aid | as #5 | flow stale ≥ 1 s, demand ≤ 0.5 s, model ready | on, silent until learned |

⛔ **Grid and lane are inert until an anchor arrives** (`localization_node.py:495-507`), and the
course files ship with **every position unset and `measured: false`**, so `fix_position`,
`fix_from_prop` and `anchor_on` all refuse on an unsurveyed venue.

### 1.3 Retrodiction

`retro.py`, 103 lines. Every event — predicts *and* updates — is buffered with a **full filter
snapshot taken just before it ran**; a late event restores, truncates and replays the tail. Cost
is proportional to lateness, not horizon. The departure from textbook OOSM is deliberate and
measured: dropping attitude updates on replay diverged to **7.1e6 m in 35 s** (`:10-19`).
⛔ **Off by default** — `retrodict=False` (`localization_node.py:154`).

### 1.4 What happens when it is lost — the honest answer

**There is no divergence detector, no covariance bound, no P clamp, no filter reset and no
relocalization path.** A grep for `diverg|clip|clamp|bound|lost|reloc` across the package returns
docstrings and one `np.clip` on a uint8 image.

What exists instead: a 5 Hz `_diagnose` (`localization_node.py:569-601`) that logs
*"NO VELOCITY AIDING … do not act on it"* when the flow and ZUPT counters have not moved — a
**log line only**, nothing on a topic — written after a measured **635 m of drift in 95 s while
publishing a healthy-looking pose**. The `frame_id` flips `odom` ↔ `pool` on anchoring, and
`mongla.pose()` returns `None` outside `pool` — but that is frame validity, not estimate
validity, and the consumer **cannot see the no-aiding condition at all**.

### 1.5 What is not present

**DVL** — drivers exist (`mongla_sensors/sources/nucleus_dvl.py`), the DSL has `dvl_connect()`,
and the estimator has **zero DVL subscriptions**; every "flow DVL" in a docstring is the optical
substitute. **USBL / acoustic positioning** — absent entirely. **Hydrophone** — absent, and the
SAUVC pinger drum (50 points) is separable only acoustically. **Magnetometer** — deliberately not
fused (aluminium hull, eight ESCs at 10 A peaks). **Terrain / bathymetric nav** — absent; depth is
one scalar into `H[0,8]`, and the SAUVC floor is known to slope 1.6 → 1.2 m against a flat-plane
assumption. **Sonar** — absent. **GPS** — absent.

---

## 2. What the best work does

> Source dossier: [`sources/sota-localization.md`](sources/sota-localization.md). ⚠ This sweep ran
> with **zero web-search budget** (exhausted session-wide before its first query) and worked
> through the arXiv API, Semantic Scholar and direct fetches. Three topics could not be reached at
> all — they are named in §6, not papered over.

### 2.1 Invariant filter vs fixed-lag smoother — and the number nobody publishes

- The **right-invariant fixed-lag smoother** ([arXiv 2102.08596](https://arxiv.org/pdf/2102.08596))
  reports **0.23–1.26 m** position RMSE on EuRoC, mixed against a non-invariant baseline — the
  invariant error formulation fixes smoother inconsistency *without hand-corrected Jacobians*,
  which is the same property our RIEKF already buys.
- **T-ESKF** ([arXiv 2510.23359](https://arxiv.org/pdf/2510.23359)) roughly **halves** error
  against a plain ESKF (0.58° / 0.20 m vs 1.13° / 0.26 m) — but benchmarked on a **desktop
  R9 7950X**.

⛔ **No paper found gives a Pi-5-class CPU cost for any of it.** The question "can a fixed-lag
window run on a Pi 5 beside a vision graph" is **unanswered by the literature** — so moving off
our filter cannot be justified from published numbers, only from our own measurement. That is a
finding, not a gap in the search.

### 2.2 Factor graphs: excellent, and built on a sensor we do not have

**Contact-aided factor-graph localization** ([arXiv 2608.26932](https://arxiv.org/html/2608.26932))
is the standout result: **0.099 m ATE** best case, **87 % error reduction** over a DVL-only
baseline, 0.90 m harbour revisit error. ⛔ It **assumes a DVL**, and reports no timing figure.

The honest reading: the factor-graph literature's headline underwater numbers are *DVL numbers*.
Our 3.4 % flow drift floor is not being compared against 0.099 m ATE on equal terms.

### 2.3 The sharpest finding — our own gate may not mean what we think

Two NEES/NIS auto-tuning papers ([arXiv 1807.08855](https://arxiv.org/pdf/1807.08855),
[arXiv 2306.07225](https://arxiv.org/pdf/2306.07225)) state the property that matters:

> **NIS and NEES are chi-squared distributed only for an already-tuned filter.**

Our gate is a χ²-99 % test with **unmeasured Q and R**. So a rejection cannot distinguish
*"this measurement is an outlier"* from *"this filter is badly tuned"* — and our **lockout break**
(after 5 consecutive rejections, bypass the gate once and inflate `P` ×4) is a mechanism built on
top of a statistic that is not valid until the noise model is. **An Allan variance is therefore a
precondition for trusting the gate**, not an optional refinement.

⛔ **No published online divergence detector with a detection-rate table was found.** Covariance
inflation is confirmed as standard practice (ensemble/data-assimilation literature,
[arXiv 1806.10655](https://arxiv.org/pdf/1806.10655)) but **with no numeric trigger threshold**
comparable to our ×4-after-5.

### 2.4 Cheap acoustics, and the ceiling they buy

The strongest single-beacon example found is **synchronous-clock CSAC + USBL**
([arXiv 2110.13825](https://arxiv.org/pdf/2110.13825)), validated against LBL on **three real
AUVs** — the error-in-metres figure lives in the 34-page body, which the sweep could not reach.

For scale, from the VO dive: a **DVL-INS dead-reckons at 0.01–0.1 % of distance**, against our
**3.4 %**. We are 30–300× worse, and nothing in either sweep closes that without acoustics.

---

## 3. The gap

| # | what the best work does | what we do | the gap |
|---|---|---|---|
| L-1 | **say when the estimate is untrustworthy** | a 5 Hz **log line**, written after a measured **635 m of drift in 95 s** while publishing a healthy-looking pose | nothing on a topic; `pose()` sees only the frame id, never the aiding state |
| L-2 | tune Q and R from **measured** noise | nominal diagonals, **no Allan variance anywhere in the tree**; position process noise is **exactly zero** | our χ² gate is only valid for a tuned filter — so the gate, the lockout break and the ×4 inflation all rest on an unmeasured foundation |
| L-3 | recover an absolute fix after a blackout | **no relocalization path at all** | ⛔ and the sweep could not reach the underwater kidnapped-robot literature, so we do not even know the field's answer yet (§6) |
| L-4 | carry independent sources so one failure is survivable | 11 sources on paper, but **grid and lane are inert until an anchor arrives**, and the course files ship with **no positions**, so `fix_position`/`anchor_on` refuse | on an unsurveyed venue the map half is unreachable by design |
| L-5 | smooth over a window where it pays | filter only | **unanswered by the literature on our hardware** — a Pi-5 measurement is the only way to decide |

---

## 4. Candidate moves

- **The verdict on the wire (ledger G-02)** is still the highest-value row here, and §2.3
  strengthens it: without a tuned noise model the gate cannot tell an outlier from a mistuned
  filter, so the honest signal is *"aiding state + covariance bound + how long since a real
  correction"*, not *"the gate is happy"*.
- **Allan variance before anything else numerical.** It is an overnight bench run and it decides
  whether Q, the gate, the lockout break and a future better IMU mean anything.
- **Do not port to a smoother on faith.** The literature does not publish the cost on our class of
  hardware; measure our own filter's headroom first.

---

## 5. Rejected, with the reason

| rejected | why |
|---|---|
| **Moving to a factor-graph / fixed-lag smoother now** | The headline underwater numbers (0.099 m ATE, 87 % better) are **DVL numbers**, and no source gives a Pi-5-class cost. Revisit only with our own measurement, or if a DVL ever exists. |
| **Trusting the χ² gate as a divergence detector** | It is only chi-squared for an already-tuned filter. It stays as an outlier gate; it is not evidence of health. |
| **A magnetometer** | Already rejected on this hull with evidence: aluminium, eight ESCs at 10 A peaks. Nothing in this sweep changes that. |

---

## 6. Research still owed on this dive

| owed | why it is missing |
|---|---|
| **Allan variance figures for a BNO085-class IMU** | datasheet URLs 404'd; and with no search budget there was no way around it. **This is the one that blocks the most** |
| **Underwater kidnapped-robot / relocalization** | the arXiv query returned an unrelated paper; the topic needs a working search engine |
| **RoboSub/SAUVC state-estimation practice** | TDRs are grey literature, not indexed by arXiv or Semantic Scholar, and unreachable without search |
| **The single-beacon error figure** (arXiv 2110.13825) | it is in the 34-page body, not the abstract |

All four are **very likely answerable** with a working search quota. This dossier should be
re-run, not re-derived.
