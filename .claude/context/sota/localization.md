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

*(Pending the research sweep for this dive.)*

---

## 3. The gap

---

## 4. Candidate moves

---

## 5. Rejected, with the reason
