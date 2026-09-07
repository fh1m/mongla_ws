# Measured bars — every number the stack ships, and what measured it

**One table per subsystem.** Each row is a constant we ship, the value measured
for it, the conditions of that measurement, and the bar it has to clear.

**These are not gates.** Nothing here refuses to run. `test_bars.py` fails when
a shipped constant drifts away from the measurement that justified it, and each
row names the evidence so the next person can re-derive rather than re-guess.

**Why this file exists.** Round 34 produced four retractions in one round — the
25 s gap, cross-water transfer, `underwater.recommend()`, and CLAHE. Every one
was a number recorded *without a bar*: no stated pass/fail, no held-out
control, no guard. A number with no bar cannot be wrong, so it never gets
checked — it just gets quoted, and eventually shipped.

**How to read the "measured" column.** A value with no stated conditions is
not a measurement. Every row says what it was measured on, because the same
quantity has different values on different footage, and pretending otherwise
is how `recommend()` happened.

---

## 1. Vision — detection

| quantity | shipped (file) | measured | bar |
|---|---|---|---|
| detector `conf` floor | `0.10` — `profiles.py` murky/clear | 0.15→0.10 buys **+0.0..+3.0** pts recall, costs **0.0..−8.3** pts precision on 7 labelled held-out pairs; F1 knee 0.25–0.50 in 5 of 7 | **0.08 ≤ conf ≤ 0.15.** Below 0.08 is unmeasured; above 0.15 loses the cross-venue gate. Do NOT move on presence evidence — presence counts a false box as a detection |
| detector `conf` (close work) | `0.15` — `profiles.py` close/fast | same table; every knee ≥ 0.25 | ≥ 0.10 |
| preprocessing | `off` — **all four profiles** | CLAHE never positive in **17** configurations (4 props, 3 venues, 39× sharpness range); gate 30.4 → **1.2 %** | **stays off in every profile.** Guarded: `test_no_profile_enables_preprocessing` |
| tracker `high_conf_det_threshold` | clamped to live `conf` — `roboflow_tracker.py` | real underwater scores p10 **0.167** / p50 **0.258** / p90 0.439; only **0.6 %** cleared the old 0.6 | must be ≤ detector `conf`, or the tracker emits nothing |
| cross-session recall | — (model choice) | same prop, 3 venues, 16× sharpness: **97.5 → 93.3 %**. Same prop, 2 *models*: **73 % vs 29 %** | **≥ 60 % on the WORST held-out session.** Rank with `tools/model_select.py` |

## 2. Vision — continuity (the lock ladder)

Measured at **native frame rate**, 300 consecutive frames × 3 windows × 5 clips,
conf 0.10 — 71 real gaps. A *sampled* sweep cannot measure this at all
(`tools/gap_distribution.py` docstring says why).

    gap  p50 0.155 s   p90 0.651 s   p99 2.418 s   max 4.00 s

| rung | shipped | covers | bar |
|---|---|---|---|
| `VISION_FRESH_ZERO_S` | 0.20 s — `motion_vision.py` | 56.3 % of gaps | ≥ p50 (0.155 s) |
| `vision.coast_s` | 0.80 s — `vision_tunables.py` | **91.5 %** | **≥ p90 (0.651 s)** — currently 23 % margin |
| `vision.lost_grace_s` | 1.00 s — `vision_tunables.py` | 91.5 % | > `coast_s` |
| kalman `max_predict_s` | 1.50 s — `tracker.yaml` | 94.4 % | > `lost_grace_s` |
| `track_buffer` | 5.00 s — `tracker.yaml` | **100 %** | ≥ p99 (2.418 s) |

**Verdict: the ladder is correctly sized.** `coast_s` sits just above the p90,
which is exactly what it is for — routine flicker — and nothing in the archive
exceeds `track_buffer`. This is the first time any of it was checked against
footage rather than against the next rung up.

## 3. Vision — pipeline

| quantity | shipped | measured | bar |
|---|---|---|---|
| photon → detections | — | **18.0 ms** median (round 32) | ≤ 25 ms |
| detection rate, Hailo-8 | — | **80.9 Hz** standalone, **53.9 Hz** through the ROS graph | ≥ 30 Hz |
| `VISION_LOOP_HZ_SROT` | 50 — `motion_vision.py` | soaked 90 s: **49.86 Hz**, 0.00 % late ticks | ≥ 25 Hz sustained |
| camera FOV | 63.8° air / **46.7° water** ±0.7 | 25 views, `calibrateCameraRO`, held-out validated | ±1.5° |
| frame staleness | — | mailbox **16.9 ms** vs 396 ms on a plain queue | ≤ 50 ms |

## 4. Control — srot link

| quantity | shipped | measured | bar |
|---|---|---|---|
| GCS heartbeat | 2 Hz | vs `GCS_FAILSAFE_MS` 5000 → **10× margin** | ≥ 3× margin |
| `ATTITUDE` rate | 50 Hz | firmware floor `RATE_MIN_MS` 20 ms | ≥ 20 Hz |
| serial load | — | **18.8 %** of 11520 B/s | ≤ 60 % |
| `MANUAL_CONTROL` resend | ≥ 10 Hz | authority ramps to zero between `MANUAL_FRESH_MS` 1000 and `MANUAL_DECAY_MS` 1500 | ≥ 10 Hz |
| `DEPTH_P` | 0.5 (read from board) | board answers 0.5; a stale 3.0 fallback **failed open by 6×** on the arming guard | must be READ, never assumed |
| `FW_BEHAVIOUR_REV` | ≥ 10 required | board reports **14** | ≥ 10 |
| board clock jitter | — | `time_boot_ms` period **100.000 ms, sd 0.000, p2p 0.000** over 200 samples | — |
| host arrival jitter | — | **sd 4.741 ms, p2p 30.85 ms** for the same 200 samples | ≤ 1 % of any liveness bar |

### Arrival vs capture on the srot link — measured, and the round-37 plan was WRONG

The plan listed "srot telemetry still stamps on arrival" beside the vision
defects. It is **not** the same defect, and the board settles it: emission is
**exact** (100.000 ms, sd 0.000), so **every millisecond of the 30.85 ms
arrival spread is OURS** — USB, kernel scheduling, Python — not the link.

That splits cleanly by consumer, which is the part worth keeping:

* **Every current consumer is a LIVENESS test** — `link_alive` /
  `_LINK_STALE_S` 3.0 s, `_named_value(max_age_s)`, `get_batteries(max_age_s)`.
  For liveness, **arrival is the correct clock**: the question is "did anything
  reach me recently", not "when was this measured". Worse, a board-side stamp
  would be actively wrong here, because `time_boot_ms` **resets on reboot** —
  which is precisely what `check_for_reboot()` detects by watching it go
  backwards — so a liveness test built on it would break at the one moment it
  matters. **No change made. The claim is retracted.**
* **A future estimator fusing `ATTITUDE` is a different question.** 30.85 ms of
  jitter at 0.65 m/s cruise is ~2 cm of position uncertainty per sample, and it
  is *noise*, not a constant bias, so it does not calibrate out. The board
  already hands us a perfect capture instant for free and we discard it — so
  when item 4 lands, carry `time_boot_ms` (with a reboot-aware offset) rather
  than re-deriving this.

**A correction to round 26's record, which said `ATTITUDE` host inter-arrival
was `sd 0.07 ms`.** Measured here on the vehicle Pi: host **sd 4.741 ms**, 68×
larger, while the BOARD's own period is sd 0.000. A figure that small cannot be
host arrival on this path; it is the board's period. Flagged rather than
silently overwritten — it was taken on a different host, and the distinction it
missed (board clock vs host arrival) is exactly the one this section exists to
draw.

## 5. Known gaps — measured, and open

| gap | evidence |
|---|---|
| **slalom has no model** | 2,102 labelled images in `TASK_2_SALOM`; zero `salom`/`slalom` training runs; `task_slalom.py` expects `red_pipe` |
| **the return leg is untested** | 302 labelled back-side images (`RETURN/backside`, class `gate_backward`); every 2025 gate model scores **0.0 %**; production `gate_rescue_repair` declares `gate/rescue/repair` — no back-side class — and its `.pt` is not on the dev box. **The test set is ready; run it where the weights live** |
| **camera exposure is auto** | `exposure_dynamic_framerate=1` buys brightness with blur *and* a lower frame rate |
| **no blur/rotation augmentation** | none of the 25 archived training configs uses any |

---

## Method rules these bars encode

1. **Percentiles, never means** — a mean rate hides a stream that stopped.
2. **An instrument coarser than the thing it measures returns a plausible
   number and no error.** A sampled sweep reported 307/307 gaps over `coast_s`;
   at native rate it is 6/71.
3. **Read source, never import**, when checking a shipped constant — in a
   worktree an import resolves to another workspace's `install/` tree.
4. **A number implying an implausible mechanism is a bug until proven
   otherwise** — 3.6 % recall was an OBB parser bug; 2.6 % was a real
   forward-vs-downward camera mismatch. Both were worth chasing.
5. **Every guard must be verified to bite** by perturbing what it guards.
6. **Clear `__pycache__` after a perturb/restore verification.** Verifying a
   guard bites means editing a constant, running the test, and restoring the
   file. A same-length edit (`conf=0.10` → `conf=0.03`) restored inside the
   same mtime second leaves a `.pyc` that Python judges **valid** — after
   which the source reads `0.10` and the interpreter loads `0.03`. It cost
   four phantom failures here. Same family as the install-tree import: the
   file you are reading is not always the code that runs.

---

## 6. `range_gain_floor` — measured, and the answer is "leave it off"

Round 35. The knob ships at `1.0` (off) and `vision_tunables.py:56` states the
physics: *"loop gain rises ~1/range, so a kp stable far-field over-drives
close-in and the 20 kg hull oscillates off a small target."* Nobody had measured
what value it should take. Now measured, three ways, and **the answer is not the
one the physics argument predicts.**

### What was measured

Box-centre movement per frame, banded by bbox fill, over archived competition
footage and on the live Pi + Hailo pipeline.

| source | far → near growth |
|---|---|
| archive, **pooled across clips** | 16× (raw), 20× (non-clipped boxes only) |
| archive, **per clip** | **0.6× / 2.1× / 2.9× / 3.0×** |
| **live pipeline** (person, COCO YOLO11n, 7,939 dets) | **1.56×** |

### The 16× is RETRACTED — it was a composition artefact

Pooling across clips manufactured it. `torpedo_up_1` contributed **1,299**
far-field samples at jitter 0.00097 — the lowest of any clip — and **zero** near
samples. That single clip dragged the pooled far-field baseline down ~5× while
contributing nothing to the near bands, so the pooled ratio measured *which
clips landed in which bin*, not what happens as a target approaches.

The pooled table was clean, monotonic across six bands, and wrong. **Per-clip is
the only honest form here**, and per clip the growth is 0.6–3.0× — consistent
with the live rig's 1.56×, measured independently on different hardware, a
different model, and a different subject.

Two things that survived the retraction and are worth keeping: the effect is
**not** frame-edge clipping (excluding every box touching a border leaves it
intact), and box *size* stability degrades alongside centre stability.

### The bar

**`vision.range_gain_floor` stays at `1.0` (off).** A floor of `0.3` — the value
the parameter's own comment offers as "gentle" — would cut close-in authority by
3.3× to fight a 1.6× effect, i.e. it would *cause* the sluggishness it exists to
prevent. Nothing below **0.6** has any measured support, and even that is
marginal.

**If close-in instability appears in water, this is the wrong knob for it.**

### The right knob, which the data does point at

Jitter tracks **detection confidence**, not bbox fill:

| clip | conf p50 | growth |
|---|---|---|
| `octagon_1` | 0.932 | **0.6×** |
| `bin.mkv` | 0.906 | 2.1× |
| `bin_front_3` | **0.211** | **2.9×** |

A marginal detection produces a wandering box at any range. So the lever is
**`vision.ctrl_conf`** — the control-side confidence floor, already implemented
and also defaulting to off — which refuses to steer on boxes the detector is not
sure about. That is a different fix from the one the physics argument suggested,
and it is the one the measurement supports.

### Method note

**Pool only what is compositionally comparable.** Six bands, thousands of
samples, a monotonic trend and a plausible mechanism were all present, and the
result was still an artefact of which clip filled which bin. The tell was
running it per clip — which cost one extra script and reversed the conclusion.

### Strengthened, and checked for the bias it could have had

The retraction above made the per-clip number load-bearing, so it was re-run
across **11 clips at 2,600 frames each**, dropping any clip without ≥80 samples
in *both* bands rather than quoting a ratio built on 39:

    7 clips qualified    growth  min 0.5x   median 1.9x   max 4.1x

Median **1.9×**, against the live rig's 1.56× — and the pooled 16× stays dead.

**The selection this required is itself a hazard**, and it points the flattering
way: requiring both bands keeps only clips where the approach *succeeded*. If
detection degrades close-in, the worst cases lose their near-band boxes and get
dropped, and the survivors would confirm whatever was already argued.

Checked with two probes on the same frames — the detector's own confidence far
vs near, and the miss rate inside the approach segment:

| clip | conf far → near | Δ | miss % in approach |
|---|---|---|---|
| `octagon_2` | 0.944 → 0.965 | +0.021 | 0.0 % |
| `torp_down_1` | 0.946 → 0.965 | +0.019 | 0.0 % |
| `oct_front_1` | 0.923 → 0.935 | +0.012 | 0.0 % |
| `octagon_1` | 0.913 → 0.951 | +0.038 | 3.8 % |
| `octagon_Bottom` | 0.894 → 0.880 | −0.014 | 5.5 % |
| **`bin`** | **0.891 → 0.357** | **−0.534** | **12.1 %** |
| `bin_front_3` | 0.174 → 0.193 | +0.019 | 47.3 % |

**Five of seven show confidence rising or flat close-in with near-zero misses** —
those near-band boxes are genuinely good and nothing is being hidden.

`bin` is a real close-in collapse — and it is also the clip with the **highest**
jitter growth (4.1×). `bin_front_3` misses 47 % but at a confidence that is low
*everywhere* (0.17 far, 0.19 near): uniformly weak, not a range effect.

So the exception proves the rule rather than breaking it: **where confidence
holds, jitter does not grow; where confidence collapses, it does.** An
independent probe arriving at the same conclusion as the jitter data, which is
why `ctrl_conf` and not `range_gain_floor` is the lever.

**One further thing this exposes, unused today:** on `bin` the confidence fell
0.53 *before* the box began wandering. A confidence **trend** is an early
warning the control loop does not currently watch — it reads only an absolute
floor. Worth a look when there is water to validate against.

---

## 7. Confidence-adaptive measurement noise — measured, live and on real data

The lever §6 pointed at, built and verified. `kalman_adaptive_noise`, **ON by
default**.

NSA (GIAOTracker → StrongSORT) scales the Kalman measurement noise by the
detection score. **The published formula is nearly inert for us**, and our own
distributions say why:

| | conf p10 → p90 | R factor | dynamic range |
|---|---|---|---|
| MOT benchmark | 0.55 → 0.95 | 0.450 → 0.050 | **9.00×** |
| our underwater | 0.167 → 0.439 | 0.833 → 0.561 | **1.48×** |

Underwater scores are low because the water is hard, not because every box is
bad. `c` only means something *relative to what this detector produces*, so
`ConfidenceModel.normalise()` maps the detector's own p10..p90 onto [0,1]
first — **16.0× dynamic range** — and the percentiles are **observed at
runtime**, never baked. (Thresholds fitted to one water is exactly what
`underwater.recommend()` did before it was deleted for calling a third venue
wrong.)

### Archive: five competition clips

Deviation of the smoothed track from the raw box, **split by confidence** —
because a plain "smoother output" score is gameable, and infinite smoothing
wins it while lagging for ever.

| clip | fixed R | adaptive | lag on high-conf frames |
|---|---|---|---|
| `bin` | **0.96×** | 1.42× | 0.0397 → 0.0220 |
| `octagon_Bottom` | 1.48× | **3.27×** | 0.0248 → 0.0104 |
| `bin_front_3` | 1.84× | **3.36×** | 0.0312 → 0.0183 |
| `torp_down_1` | **0.64×** | 1.46× | 0.0116 → 0.0049 |
| `oct_front_1` | 1.74× | **3.38×** | 0.0119 → 0.0055 |

### Live vehicle: Pi + Hailo, person target, both arms 80 s

| | adaptive OFF | adaptive ON |
|---|---|---|
| deviation, high-conf (lag) | 0.00668 | **0.00190** |
| deviation, low-conf (rejection) | 0.00522 | **0.01020** |
| **selectivity** | **0.78×** | **5.37×** |

**6.9× better on the live stack**, and the direction of both halves is right:
it follows a good box 3.5× more closely *and* rejects a bad one 2× harder. Not
more smoothing — more **discrimination**.

### The bar

**Selectivity must exceed 1.0.** Below it the filter is *inverted* — following
doubtful boxes more closely than confident ones — which is what the shipped
fixed-R filter was doing on the live rig (0.78×) and on two of five archive
clips (0.96×, 0.64×). That inversion is the failure this fixes, and it was
invisible until the deviation was split by confidence.

### Caveats, stated

The two live arms are human-executed walks, so sample counts (1,367 vs 2,991)
and confidence quartiles (0.628/0.864 vs 0.770/0.890) differ. The effect —
0.78 → 5.37 — is far larger than that variation can account for, and it agrees
in direction and rough magnitude with the archive result measured on entirely
different data, hardware and targets. **In-water validation is still owed.**

---

## 8. The feature backend for the anchor rung — benched, not chosen

Round 36. The anchor rung (hold a target with **no bbox at all**) needs a local
feature. Three candidates, benched on **our own footage** with one protocol:
snap a reference at 40 % of the clip, match **frame-to-reference** at +1/3/5/8 s,
`USAC_MAGSAC` homography, count inliers. A homography needs ~15+ to be trusted.

### ORB is not viable — and this RETRACTS "the anchor dies in murky water"

| clip | ORB ref kp | ORB ok | **XFeat ok** |
|---|---|---|---|
| Mirpur torpedo (murky) | **71** | 0/4 | **4/4** (350→100) |
| Mirpur torpedo_1 | **7** | 0/4 | **4/4** (301→130) |
| Mirpur gate | 111 | 3/4 | **4/4** (753→682) |
| octagon (low texture) | 1205 | 1/4 | **3/4** |
| torpedo (clear control) | 1260 | 4/4 | 4/4 |

An earlier note in this file concluded the anchor **"dies in murky water"** and
proposed a keypoint-count gate on that basis. **That was an ORB limitation, not a
water limitation** — the same frames give XFeat 4096 keypoints and 100–350
inliers. The conclusion is withdrawn; the *gate* survives as a runtime health
check, but it is not a reason to distrust the rung.

### EdgePoint2's published advantage does not transfer to our domain

EdgePoint2 (2025) is documented as **2× faster than XFeat with competitive
IMC2022 results**. At 320×240, top_k 1024, on our clips:

| clip | XFeat | EP2-S64 | EP2-M64 |
|---|---|---|---|
| Mirpur torpedo | **4/4** (86–155) | 3/4 (7–33) | 2/4 |
| Mirpur torpedo_1 | **4/4** (65–190) | 1/4 | 2/4 |
| Mirpur gate | **4/4** (222–260) | 4/4 (90–142) | 4/4 |
| octagon | **2/4** | 1/4 | 1/4 |
| torpedo (clear) | **4/4** (445) | 4/4 (246) | 4/4 |

XFeat carries **2–5× more inliers** and wins or ties every case. IMC2022 is clear
natural imagery; **turbid underwater is a different domain and the ranking does
not survive the move.** Recorded because "2× faster and competitive" is exactly
the kind of claim that gets adopted without a domain check.

### It fits the vehicle — measured on the Pi

ONNX-exported (2.7 MB), `onnxruntime` CPU, with the vision stack running:

| resolution | 1 thread | 3 threads |
|---|---|---|
| 640×480 | 145.6 ms (6.9 Hz) | 88.0 ms (11.4 Hz) |
| **320×240** | **33.1 ms (30.2 Hz)** | 18.2 ms (55.1 Hz) |

And 320×240 **keeps the lock**: every murky clip still 4/4 (56–260 inliers), only
the already-marginal octagon drops 3/4 → 2/4. So the slow rung runs at 30 Hz on
**one core** while the fast rung (LK, 8.0 ms) runs at 125 Hz on another, with the
Pi still 72.8 % idle.

**The bar:** the anchor backend is **XFeat at 320×240**. Any replacement must be
benched on the **Mirpur** clips, not on a public leaderboard — that is where ORB
scored 7 keypoints and EdgePoint2 lost.


---

## 9. Plane tilt from the anchor homography — correct, and only COARSE on real water

`anchor/geometry.plane_geometry()` recovers the matched plane's orientation —
the torpedo shot's actual precondition, which a centred bbox cannot express.

### Synthetic ground truth: exact

| condition | result |
|---|---|
| every tilt 0–65°, noise-free | recovered to **< 0.5°** |
| + 1.55 px matcher noise, ≥ 27 px baseline | p50 **2.2°**, p90 **5.5°** |
| + hull rotation 2° between frames | p90 **6.7°** (5° of rotation → 18°) |
| pure camera rotation (no translation) | **refused** at 1/3/8° — verified |

### Real archive footage (`torpedo_shark_up_1`): coarse only

| | |
|---|---|
| per-frame tilt | p90 frame-to-frame swing **39°** (XFeat) / **47°** (ORB) |
| 31-frame rolling median | p90 swing **1.3°** — *looks* excellent |
| **two independent snaps of the same board** | **disagree 4.7° median, 17.5° p90** |

**The third row is the bar.** Smoothing buys stability and not accuracy: the
per-reference error is a *bias*, not zero-mean noise, so a rolling median
produces a confident, smooth, wrong angle. Reading only the second row would
have shipped exactly that. **The two-snap control is what caught it, and any
future attempt to rehabilitate this number must reproduce that control.**

**Verdict: usable as a coarse "is the board grossly off-square" indicator;
NOT a firing precondition. Not wired to control.**

### Eight causes tested and eliminated — do not re-derive

| hypothesis | measurement | verdict |
|---|---|---|
| insufficient baseline | real median 19–60 px vs a 12 px bar | not it (bar added anyway, justified separately) |
| scene is not planar | inlier reprojection RMS **1.55 px** | the homography fits |
| wrong camera matrix | p90 35–78° across a **6× focal sweep**, no minimum | not it |
| matcher noise | synthetic at 1.55 px predicts 2–5° | does not reach 39° |
| RANSAC re-choosing the plane | inlier overlap 0.48; |Δtilt| 14.0 (low) vs 11.5 (high) | ~2.5°, minor |
| hull rotation | 6.7° at 2°, 18° at 5° | insufficient |
| matcher quality | XFeat 38.9 vs ORB 46.9 p90 | barely different |
| ROI plane isolation | **worse** — 58–73° p90 | fewer keypoints, worse conditioning |

### The baseline bar, separately justified

Tilt is unobservable without translation (`H = K R K⁻¹` carries no plane term).
Measured at 0.5 px noise: p90 error 42.1° at 0.84 px displacement, 13.6° at
6.71, **5.6° at 11.75**, 1.9° at 26.9. `MIN_BASELINE_PX = 12.0`; `baseline_px`
is reported so a firing gate can demand 17 (p90 3°). This is a **precision**
bar — the dangerous degenerate case, pure rotation, is refused by the
decomposition itself.


---

## 10. Optical-flow velocity — measured on the bench, global-shutter camera

The velocity OBSERVATION the estimator was blocked on. `flow_velocity()`:
`v = h * (flow_px/dt - f*omega) / f`, where the asymmetry is the whole design —
translation scales with `1/h`, rotation does not, so rotation is subtracted
**before** scaling.

### Static, camera 0.77 m above a floor — truth is exactly zero

| | |
|---|---|
| intervals | **727** |
| peak raw flow | **0.069 px** |
| velocities reported | **0** |

Zero false positives from the magnitude floor alone. This is the bar: an
unmeasurable interval must not be reported as 0 m/s, because zero is a
*measurement* and a filter fed a confident zero will believe it.

### Moving, 50 cm one-way slide at 0.50 m, global shutter @ 210 fps

| dispersion gate | intervals | net cm | path cm |
|---|---|---|---|
| 0.5 (as first shipped) | 104 | −12.3 | 15.9 |
| 1.0 | 212 | −17.4 | 27.5 |
| 2.0 | 304 | −25.2 | 35.6 |
| 3.0 | 346 | −28.1 | 38.7 |
| **off** | 396 | **−30.9** | **41.6** |

**Cross-axis leakage 2.3–3.5 %** across runs: a straight slide put ~97 % of the
displacement on one axis, which nothing in the pipeline enforces — it falls out
of the projection being right, and is the check that would catch a scrambled
axis mapping before water.

**Scale: 41.6 cm path measured against 50 cm true = 83 %.** Not yet closed. The
leading candidates, in order: the 0.5 m was taped to the housing rather than
the sensor (a 5 cm error is 10 %), and slow ramp-in/ramp-out below the
14.7 mm/s floor is displacement the integral never sees. **Do not treat the
velocity as calibrated until this is closed** — the error is a clean multiplier
and will be invisible downstream.

### ⛔ A gate justified on contaminated evidence

`MAX_DISPERSION_RATIO` was set to 0.5 to exclude 21 large-flow intervals from a
"static" run. The operator had knocked the camera during it — those were REAL
MOTION, and they sit inside the distribution real motion has (ratio median
0.40–0.89, p90 1.02–3.76, because a downward camera is never exactly
fronto-parallel and tilt makes range vary across the image). At 0.5 the gate
discarded **60 % of a real 50 cm slide**. Now 5.0, and explicitly a sanity
bound rather than the noise discriminator — the static run shows the magnitude
floor already does that job.

**Baseline is not frame rate.** At 210 fps a 0.2 m/s slide moves the image ~1 px
per frame; at 30 fps the same motion moves ~7 px. SNR is set by displacement,
so the reference frame is held until ≥28 ms has passed. Frame rate buys
latency, not accuracy, here.


---

## 11. The inertial data the board actually gives — measured, not assumed

The estimator's architecture turns on one question, so it was checked on the
live board rather than inferred from the firmware docs.

### Rates: 5× available for free, no firmware change

`SET_MESSAGE_INTERVAL` works on this firmware:

| message | before | after |
|---|---|---|
| `SCALED_IMU2` | 10.0 Hz | **51.6 Hz** |
| `ATTITUDE` | 10.0 Hz | **51.6 Hz** |
| `SCALED_PRESSURE2` | 5.0 Hz | **51.6 Hz** |

⛔ **A 100 Hz request ACKed `ACCEPTED` and delivered 50** — the firmware's
`RATE_MIN_MS = 20` floor. The ACK is a claim; the arrival rate is the fact.
Same shape as `REQUEST_MESSAGE` accepting all 190 ids and emitting 7.

### What the inertial sensor is good for, and what it is not

Static, flat, 250 samples at 10 Hz:

| | |
|---|---|
| accel quantisation | **1 mg** (int16 mg on the wire) |
| accel bias (`xacc`) | **−17 mg = 0.167 m/s²** |
| gyro noise | sd **8–15 mrad/s** = 0.46–0.87 °/s |
| ATTITUDE roll/pitch stability | **0.10–0.16°** over 25 s |
| ATTITUDE yaw drift | 0.458° over 25 s |

**Integrating that accelerometer is hopeless, and the number says so:**

| integrated | phantom velocity | phantom position |
|---|---|---|
| 1 s | 0.17 m/s | 0.08 m |
| 5 s | 0.83 m/s | 2.08 m |
| 10 s | 1.67 m/s | **8.34 m** |
| 30 s | 5.00 m/s | **75.05 m** |

A 20 kg hull cruising at 0.65 m/s is buried by its own accelerometer inside
five seconds. **Velocity must be OBSERVED, never propagated** — which is what
RD-VIO (Applied Ocean Research 2023) and DeepVL (ICRA 2025) both conclude for
underwater vehicles, and what `nav_estimator.py` is built on.

**The gyro sets the floor on flow de-rotation.** At h = 0.5 m, f = 514 px,
dt = 33 ms, the measured gyro noise costs:

| axis | sd | de-rotation error | velocity error |
|---|---|---|---|
| xgyro | 0.46 °/s | 0.14 px | **4.0 mm/s** |
| ygyro | 0.49 °/s | 0.15 px | 4.2 mm/s |
| zgyro | 0.87 °/s | 0.26 px | 7.6 mm/s |

Our measured static velocity noise floor is ~15 mm/s (the 0.5 px flow floor),
so the gyro is the same order: **de-rotation cannot be better than its gyro**,
and buying a quieter flow measurement past this point buys nothing.

### Why scale survives a constant-velocity cruise

Monocular scale is unobservable without accelerometer excitation, and a transit
at constant velocity has none. Adding a **range** measurement removes scale
from the observability nullspace (Delaune & Bayard, *Range-Visual-Inertial
Odometry: Scale Observability Without Excitation*, RA-L 2021 — the approach
flown on NASA's Ingenuity). Our range is the altitude above the floor. That is
the theoretical reason a cruising velocity estimate is possible here at all,
and it is why the altitude input is not optional.

### Dead-reckoning budget — seconds of blind flight

From a converged filter (velocity fixed to ~0.02 m/s), time until position
sigma passes each bound:

| `q_vel` | 10 cm | 25 cm | 50 cm | 100 cm |
|---|---|---|---|---|
| **0.35** (default) | 0.5 s | 0.9 s | 1.5 s | 2.3 s |
| 0.15 | 0.9 s | 1.6 s | 2.5 s | 4.1 s |
| 0.05 | 1.8 s | 3.3 s | 5.3 s | 8.4 s |

**`q_vel` must be fitted in water from NIS** — a bench cannot produce the
accelerations that decide it, and lowering it without that measurement buys
coast time by asserting the hull is calmer than it is.

---

## 12. De-rotation, calibrated on the rigid camera+IMU rig

The srot board carries the downward global-shutter camera on a rigid mount, so
for the first time the gyro and the camera share a body. Four phases, each run
separately with the operator prompted between them.

**Height is LENS to floor, 0.70 m ("70ish").** Velocity scales linearly with
it, so the ±5 cm uncertainty is a **±7 % floor** under every scale number here.

### The mapping — one axis at a time, or it is not identifiable

| | value |
|---|---|
| excitation | gx 0.637, gy 0.534 rad/s (separate runs) |
| `cond(MᵀM)` | **27.7**, well-conditioned |
| image dx | `gx −1.150` (cross +0.058, −0.085), R² 0.982 (see caveat) |
| image dy | `gy −1.095` (cross −0.064, +0.020), R² 0.990 (see caveat) |
| false velocity, pure rotation | **575.7 → 57.1 mm/s (−90 %)** |

**Read `cond`, not R², as the evidence the fit is identifiable.** The held-out
split is even/odd intervals, so train and test are adjacent 30 ms frames —
near-duplicates during a 1 Hz swing. R² 0.99 therefore partly measures
interpolation, not generalisation across motion. **`cond(MᵀM)` = 27.7 is the
load-bearing number**; the R² values are supporting, not decisive.

**Three earlier attempts failed and all failed the same way.** Swinging by hand
rotates about two axes at once; correlated regressors leave the *split* between
`gx` and `gy` undetermined even though the fit looks fine, so the two gains
disagreed by 2x and swapped which one was well-explained between runs (dx 0.641
/ dy 0.936, then dx 0.785 / dy 0.497). **Excite one axis, then the other, then
fit the pooled data.** Least squares never refuses: with no excitation it
returns plausible +-1 coefficients and no error.

**⚠ The gains are −1.12, not −1.00, and the cause is OPEN.** Rotation about
the lens gives exactly `f·ω·dt`, so a 12 % excess needs an owner. Recorded here
because it is a real 12 % and the wrong explanation is easy to reach for:

- **A pivot lever arm was my first answer and its sign was WRONG.** With the
  hand *above* a downward-looking lens, the lens swings opposite to the tilt, so
  translation OPPOSES rotational flow: `f·(1 − r/h)`, a gain **below** 1.0. Only
  a pivot *below* the lens gives a gain above 1, which is possible depending on
  the grip but was not measured. **The r ≈ 8 cm first written here was reverse-
  fitted to cancel the residual, not derived. Retracted.**
- **The gain is a direct measurement of `f`** (no height term), giving
  `f_eff` = 591 px from dx and 563 from dy — **57.6° HFOV, not 63.8°**. Two axes
  agreeing is not noise. But round 25's calibration is the stronger evidence:
  25 views, `calibrateCameraRO`, k-fold held-out, and an external tape check
  that confirmed a *stated prediction* (23.2 predicted vs 23.0 measured).
- **A `dt` bias would do it too** — flow is `f·ω·dt`, so timestamps 12 % short
  inflate the gain identically. Not separable from the above without more data.

**Height cannot reconcile it**: the rotation gain has no height term, so `f` is
over-determined at 514 (calibration) vs 577 (gain), and a height error cannot
move either. Phase 3 then ties them together — `v ∝ h/f` — and is consistent
with **(f 514, h 0.70)** or **(f 577, h 0.78)**, so a careful lens-to-floor
measurement is the cheapest discriminator available.

Independent evidence the gain is too large: the 0.638 rad/s run's rotation
fraction reached **p90 156 %** — fitted rotational flow exceeding total observed
flow, which is what an overshooting correction looks like.

**For the vehicle**: the axis mapping and the method carry over; **this gain
does not**, whatever its cause. Re-derive it on the hull.

### The working envelope — de-rotation has a rotation-rate ceiling

| gyro rms | raw | de-rotated | verdict |
|---|---|---|---|
| 0.090 rad/s (pure translation) | 51.7 cm | 52.8 cm | benign, +2 % |
| **0.638 rad/s** | 69.3 cm (139 %) | **42.7 cm (85.5 %)** | **halves the error** |
| 1.128 rad/s | 30.6 cm (61 %) | 20.7 cm (41 %) | **makes it worse** |

Truth 50 cm in all three. The correction is `f·ω·Δt`, so **a camera↔gyro
time-sync error leaves uncorrected flow proportional to ω**: 5 ms costs 1.6 px
at 0.64 rad/s and 2.9 px at 1.13. **The residual is time-sync-limited, not
mapping-limited** — which is where to look for the next improvement.

### ⚠ `ROT_FRACTION_MAX = 0.80` is miscalibrated

`flow_velocity.py` refused **533/670** intervals of the 0.638 rad/s run
(rotation fraction median 99 %) — accepting 16 % — while de-rotation on that
same data returned **85.5 % of truth against raw's 139 %**. The threshold was
set before a validated axis mapping existed. **Do not raise it blind**: the
1.128 rad/s row shows a genuine ceiling. A residual-based criterion is the
better shape than a fraction-based one.

Do not read the "gated net 5.1 cm" figure as evidence: it integrates 16 % of
the travel and is not comparable to 50 cm.

### Numbers that replaced earlier ones

| quantity | was | is |
|---|---|---|
| **flow noise floor** | ~15 mm/s | **0.57 mm/s** median, p90 1.25, max 2.38 |
| **flow scale** | 41.6 cm vs 50 (83 %) | **51.7 cm vs 50 (103.4 %)** |

The old floor was **a rig being handled** — two independent untouched runs gave
0.46 and 0.57 mm/s at h = 0.70. The old scale gap was **a height error**, 50 cm
assumed against a ~70 cm lens height. 103.4 % is **consistent with no systematic
bias within the ±7 % height uncertainty** — it does not establish that there is
none, and the focal-length question above is exactly such a candidate.

**Phases 1 and 2 were recorded before height became a parameter and ran at
`HEIGHT_M = 0.50`.** Every velocity from them is restated here at 0.70 (×1.4).
The mapping coefficients are unaffected — that fit has no height term — and so
is the −90 % ratio.
Static flow is 0.01–0.02 px, so `MIN_NET_FLOW_PX = 0.5` correctly refuses every
static interval — verified against real numbers rather than assumed.

### Instrument traps, each of which returned a plausible number

- **A covered lens is not an error.** `goodFeaturesToTrack` still finds corners
  in sensor noise and LK still returns motion: a capped camera produced "11 px
  of flow" at 33 % survival, which was very nearly diagnosed as an optical-flow
  search-window problem. **Retracted** — with the cap off, 16 px of flow tracks
  at 93 %. Every phase now prints brightness / texture / corner count and
  refuses to run below a floor.
- **A motionless bench reports gyro rates of EXACTLY 0.0** — the firmware
  quantises sub-threshold rates away. This looked like a frozen `SCALED_IMU2`
  and was briefly recorded as a third permanent-zero field alongside
  `VFR_HUD.throttle`. **Retracted**: it reads 0.04–0.09 rad/s the moment
  anything moves. A preflight guard requiring *variance* would have refused
  phase 1, whose whole point is that nothing moves.
- **Gyro bias cannot be measured from a static interval on this board** — any
  real bias sits below the quantisation that reports zero.
- **An unattended protocol measures an untouched rig.** One full four-phase run
  was spent on a rig nobody was told to move. Prompt the operator, one phase
  per invocation, and print live feedback so they can correct mid-run.
- `SCALED_IMU2` is `ATTITUDE`'s rates × 1000 — same data, so use `ATTITUDE`.


## 13. The bottom camera AS A DVL — verified live against a tape, 2026-09-07

**Three axes, 30 cm each, real physical slides on the bench rig at
h = 0.72 m lens-to-floor, in air, `f = 513.94` px.**

| axis | measured | error | of truth | angle | implied h | pts | resid |
|---|---|---|---|---|---|---|---|
| lateral | **30.13 cm** | **+0.13** | 100.4 % | +88.4° | 0.72 m | 39 | 0.57 px |
| forward | **31.09 cm** | **+1.09** | 103.6 % | +12.9° | 0.69 m | 22 | 0.58 px |
| back | **31.04 cm** | **+1.04** | 103.5 % | −178.5° | 0.70 m | 24 | 0.62 px |

**Max error 1.09 cm on 30 cm — 3.6 %.** Nortek quote **0.5–1 %** for DVL
bottom-track; Ferrera et al. (Sensors 2019) report **0.89–1.88 %** ATE RMSE
for monocular VO in real turbid water. We are the same order as published
monocular VO and short of a real DVL, **in air, on a hand slide whose own
precision is roughly ±1 cm** — the operator's tape and hand are inside our
error bar, so this is an upper bound on the sensor's error, not a measurement
of it.

**The implied heights are the strongest single result.** Height is recovered
from `h · truth / measured`, and the three runs give **0.72 / 0.69 / 0.70 m**
against the 0.72 m measured with a tape. The scale chain closes independently.

**Cross-axis leakage is small and the angles are clean**: +88.4° for a lateral
slide, −178.5° for a back slide. The forward run's +12.9° is the operator's
line, not the sensor's — the same rig scored −171.8° on a deliberately
diagonal slide and reported its magnitude correctly.

**Synthetic control, same console, same optics, exact truth:** 29.98 / 29.99 /
30.02 cm, max error **0.02 cm**. The gap between 0.02 cm synthetic and 1.09 cm
physical is the hand, the tape and the height — not the algorithm.

### ⚠ THREE INSTRUMENT DEFECTS, ALL REPORTING A CORRECT SENSOR AS SHORT

Recorded because the pattern is the lesson, not any one bug. In every case the
console's own live trace showed the slide tracked correctly while the CAPTURE
threw the measurement away:

1. **The end-of-move check lived inside `if v.ok:`** and could never fire — a
   move ends with the rig stopping, a stopped rig produces refusals, so the
   stillness that defines the end was exactly the condition under which the
   check was skipped. Traced 29.89 cm, logged nothing.
2. **The capture started at first-motion**, discarding the slow acceleration
   and deceleration — a systematic under-count. Traced 30.3 and 30.5 cm,
   scored 19.6 / 26.3 / 21.6.
3. **Stillness was judged on instantaneous velocity**, and a REFUSED interval
   read as zero speed — so a burst of refusals mid-slide looked like stopping.
   Traced 24.9 and 30.0 cm, scored 12.1 and 8.5.

All three are the same mistake in different clothes: **treating "I could not
measure" as "it did not move."** The fix that ended it was not a better
heuristic — it was a button. The operator knows when they stopped; no window
short enough to be responsive can distinguish a slow hand from a still rig.

### What ships behind these numbers

- **adaptive keyframe baseline** — emit a velocity per ~8 px of accumulated
  displacement, not per frame. At the camera's native 210 Hz a fixed floor
  refuses everything below **0.147 m/s**; adaptive holds **99.9–100.5 %**
  across a 40× speed range (2 → 80 cm/s).
- **planar rigid fit** (`estimateAffinePartial2D`, RANSAC) instead of a median.
  Under 1.5° of rotation a median reports **4.79 px of translation that never
  happened**; the fit, 0.055 px.
- **forward-backward rejection at 2 px**, run only when an interval is ripe.
  Independently the same threshold Ferrera et al. use.
- **LK window 31** (measured 128→164 surviving points across 15→41).
- **de-rotation from the MEAN gyro rate** over the baseline, not a midpoint.
- **`f_water = 741`** in water vs `f_air = 513.94` — measured, ratio 1.44.

---

### ⛔ WHAT IS AND IS NOT VERIFIED ABOUT THE RECTIFIED PATH

Every velocity number above -- the 30 cm result, the 0.000 % synthetic
recovery -- was taken with **`medium='air'`** or through simulated physics.
`port=RECTIFIED` has been verified to **come up and be selected** on the
vehicle (§17), and the rectifier's maths is verified against a true pinhole
and against the n=1 identity. **The rectified water path has produced NO
MEASURED VELOCITY AT ALL**, because the room went dark before a slide could
be run through it.

So the refactor is verified as **arithmetic**, not as a **sensor**. The
regression check that closes this needs no water and no rig movement beyond
one hand slide: `medium:=air` through the launch, confirming ~30 cm still
comes out of the code path that now contains the rectifier. It is item 23 in
`water-owed.md`, and it is blocked only on light.

## 14. Board-clock stamping — measured on the vehicle, 2026-09-07

The timing round wired four corrections and unit-tested all of them. This is
the first one **measured end to end on the real link**, and it is the one that
needed no rig movement: the hull sat still, the board streamed, and both
stamps came off **one capture** so the two arms cannot differ by link load.

`ATTITUDE` at 50 Hz on `/dev/ttyUSB0`, 4400 frames over 88 s, through the
**shipped** cadence — `ClockMap(window_s=20, min_pairs=40)`, add every sample,
refit every 2.0 s — mirroring `auv_manager_node._imu_rates_tick` rather than a
parallel implementation of it.

**Every sample is held out**: it is scored against the fit in force when it
arrived, *before* being added to the map. Scoring a fit on its own training
data is how the round-25 `calibrateCameraRO` comparison came out invalid.

| stamp error vs the board's own clock, de-meaned | sd | p99 | worst |
|---|---|---|---|
| **arrival** (`time.monotonic()` at drain) | **6.402 ms** | 26.10 | **34.48 ms** |
| **mapped** (`ClockMap.to_host(board_ms)`) | **0.528 ms** | 0.87 | **1.22 ms** |

**12.1× on sd, 28× on the worst single stamp.** Against Qin & Shen's 6 ms
tolerance the arrival stamp was **OUTSIDE it before counting any offset at
all**; the mapped stamp is 11× inside it. The board's own interval is
**exactly 20.000 ms, sd 0.000** across all 4400 frames — its 500 Hz FreeRTOS
loop has no jitter to give, so every millisecond of the 6.4 was transport.

### ⛔ The number this REPLACES, and why the first one was wrong

A first run held the fit out for 65 s and reported **sd 0.334 ms** — better,
and misleading. `to_host()` is **affine**, so once fitted it adds no
randomness whatsoever: every mapped stamp error is `skew_err · elapsed +
offset_err`, a straight ramp. At the fitted +18 ppm over 65 s that ramp is
1.17 ms p2p, which is *exactly* the 1.158 measured. **That run measured how
stale the fit was, not how much jitter was removed.** The live path never runs
65 s on one fit. The 0.528 ms above is the operational figure, and it is
larger than the flattering one — which is the direction that makes it real.

Two harness bugs came first and both returned plausible numbers rather than
errors: `add()` takes **board SECONDS** and was fed milliseconds, and `fit()`
was never called, so `to_host()` was the identity. The output was a clean
1000× — the same units-and-no-error signature as the `.tlog` little-endian
round trip that made every time wrong by decades.

### Recorded, not a defect

The fitted skew wanders **−88.5 … +107.5 ppm (sd 31.0)** window to window —
the lower-envelope slope absorbs some noise. It costs nothing at this refit
rate (31 ppm × 2 s = **0.062 ms**) and the fit residual is **0.192 ms**, but
the skew term is not stably estimated and should not be quoted as a crystal
measurement.

### What this does NOT cover

The three other corrections — interval midpoint, half-exposure, and the `td`
estimate — are wired and unit-tested and **still unmeasured on hardware**.
`td` in particular has no in-water value: every number for it is
injected-offset recovery, which is why it is now bounded (`time_offset_max_s`
0.15) rather than trusted. Measuring the real `td` needs the rig **gently
oscillated about the optical axis** — a *changing* rate, since Li & Mourikis
show constant velocity is degenerate — and is a few seconds of hand movement.

---

## 15. The return leg — RETRACTING "0.0 %", in both directions, 2026-09-07

The carried ledger has said for three rounds: *"every 2025 gate model scores
**0.0 %** on the **302 labelled** back-side images; production
`gate_rescue_repair` declares gate/rescue/repair — no back-side class. One
command decides whether a phase-1 scoring task is silently dead."*

It was finally run. **Three separate claims in that sentence are wrong**, and
the correction goes in *both* directions — the situation is better than
recorded for the production model and much worse for the fix.

### The suspicion that prompted the re-test, and it was half right

The labels carry ONE class, `gate_backward`, which no production gate model
declares. **A score computed by class-name match is 0.0 % by construction** —
it measures the vocabulary, not the optics, and would read 0.0 % even if the
model boxed the gate perfectly. Same shape as the sim scorer grading a board
whose opening list it had wrong: the number was real and about the wrong
thing.

So the test was re-run **class-agnostically** — does the detector fire *any*
box, and does it overlap the label — at three confidences on the 72 labelled
frames.

| model | classes | conf | fired any box | IoU ≥ 0.5 | median IoU |
|---|---|---|---|---|---|
| **`gate_rescue_repair`** (PRODUCTION) | gate/rescue/repair | 0.10 | 52.8 % | **41.7 %** | 0.008 |
| | | 0.25 | 29.2 % | **26.4 %** | 0.000 |
| | | 0.40 | 18.1 % | 13.9 % | 0.000 |
| `robosub_gate` (2025, SAUVC family) | shark/shaw_fish | 0.10 | **0.0 %** | 0.0 % | 0.000 |
| `backside` specialist | gate_backward | 0.25 | 88.9 % | 88.9 % | 0.945 |

### Correction 1 — the production model is NOT blind. It is unreliable.

**41.7 % at conf 0.10**, held out (it was never trained on back-side frames),
firing its ordinary `gate` class on a gate seen from behind. That is genuine
generalisation and the ledger's 0.0 % erased it.

**But at the SHIPPED operating point it is ~26 %.** The Hailo path runs
`conf 0.15` and INT8 scores ~0.08 lower than fp32 (round 24), so the
fp32-equivalent bar is ≈0.23 — the 0.25 row. **A scoring task that works one
run in four is not a working scoring task.**

The `median IoU 0.008` beside `41.7 % at IoU ≥ 0.5` is not a contradiction —
it is **bimodal**. When it fires it locks well; more than half the time it
fires nothing. The median lands in the empty half. **Quoting the median alone
would have said "blind"; quoting the hit-rate alone would have said "fine".**

**Only the older SAUVC-family models score a true 0.0 %** — `robosub_gate`
fires *no box at all* at conf 0.10. So "every 2025 gate model" was wrong; the
right sentence names which family.

### Correction 2 — the specialist's 88.9 % is MEMORISATION. Do not deploy on it.

A trained back-side model exists in the archive (`yolov11_n_backside_100`,
5.2 MB, Aug 2025) and scores 98.6 % at conf 0.10, median IoU **0.948**.

**Its `data.yaml` says `val: train`.** It was validated on its own training
data, and the 72 labelled frames scored above **are that training set**. The
number is memorisation and carries **no** information about generalisation.
This is §6's saturated-validation trap exactly — mAP50 0.995 across all 25
archived runs, ranking nothing.

What it *does* prove: the class is learnable and the labels are consistent.
What it does **not** prove: that it would see a back-side gate at the pool.

### Correction 3 — it is 302 images and **72 labels**, not 302 labelled

The other 230 are unlabelled, which is also why this is the scarcest asset in
the archive for the one task nobody has measured.

### What actually follows

1. **The return leg is not dead — it is a coin flip**, and that is worse than
   dead in one respect: dead fails loudly, 26 % fails on the day.
2. **Deploying the specialist on its 88.9 % would be deploying a number that
   does not exist.** It needs a held-out *session* split — never a random
   split, since these frames are consecutive video and neighbouring frames are
   near-duplicates.
3. **Cheapest real fix**: label more of the 230, retrain with a held-out
   session, and evaluate the merged 4-class model against the production 3-class
   one on frames neither has seen.
4. **For the pool**: point the forward camera at the gate from the far side and
   record. Whatever we do about the model, we currently have 72 labelled
   back-side frames from one session and that is not enough to decide anything.

**Method note.** The class-agnostic re-test was worth running even though the
naming-artifact hypothesis turned out only half right: it converted "0.0 % by
class name" — which says nothing — into two different, actionable facts about
two different model families. **A metric that cannot distinguish "wrong
vocabulary" from "sees nothing" should never have been the one carried in the
ledger for three rounds.**

---

## 16. Gyro-aided LK: MEASURED AND REJECTED. Rotation ripeness instead. 2026-09-07

ROUND 38 item 4 proposed feeding the gyro prediction to LK as an initial
guess (pixel-aware gyro-aided KLT, IEEE TIM 2022), aimed at the measured
ceiling in §12: at 1.128 rad/s de-rotation made a 50 cm slide **worse**
(41 % of truth) where at 0.638 rad/s it recovered 85.5 %.

It was built, benched on **real bottom-camera floor texture** warped by exact
known motion, and **rejected on the numbers**.

### It does nothing at our operating point

Real frame, 30 ms baseline, forward-backward at 2 px, plain vs seeded:

| gyro | rotation | plain surv | err | seeded surv | err |
|---|---|---|---|---|---|
| 0.638 rad/s | 1.1° | 180/192 | 0.017 px | 180/192 | 0.017 px |
| 1.128 | 1.9° | 176/192 | 0.021 | 176/192 | 0.021 |
| 3.000 | 5.2° | 165/192 | 0.052 | 165/192 | 0.052 |

**Identical to three decimals** — because the displacement is already inside
LK's basin. With `winSize 31` and `maxLevel 3` the basin is ~120 px; 5.2° of
rotation moves a corner 33 px.

The harness was proven capable of showing a difference before this was
believed (round 28's lesson): the guess differs from the start point by
**96.8 px mean / 177.6 px max**, and at 40° of rotation with the FB gate off
it does move survival — 30 → 63 of 192 at maxLevel 3.

### And where it looked like it helped, the points are WRONG

Those recovered points **do not survive forward-backward validation**. With
the 2 px FB gate applied, at 48.5° the seed takes survivors from **0 → 5 of
192** — not enough to fit, and the "recovery" is bad matches. A seed that wins
back points the quality gate then throws away has bought nothing.

**So §12's 1.128 rad/s failure was NEVER an LK tracking failure.** LK tracks
fine there (176/192, 0.021 px). The failure is in the de-rotation itself,
which points at timing and mapping — the camera↔IMU work, not the tracker.
That reattribution is the most useful thing this experiment produced.

`predict_points` was deleted rather than left default-off. A helper nothing
calls is not a fix.

### What the data DID point at: rotation as a ripeness criterion

Ripeness asked about displacement, track count and TIME. Never rotation. The
baseline stretches when the hull moves **slowly** — station-keeping, the most
common state — so a hull holding position while yawing accumulates the whole
rotation inside one interval: **48.5° at 1.128 rad/s over the 0.75 s cap**,
which leaves **0 of 192 points**.

Survival and translation error vs accumulated rotation, real frames:

| rotation | survival | err | residual |
|---|---|---|---|
| 3° | 82.5 % | 0.014 px | 0.166 px |
| 6° | 74.7 % | 0.040 | 0.358 |
| 8° | 60.5 % | 0.091 | 0.537 |
| 10° | 51.1 % | 0.076 | 0.839 |
| 12° | 37.4 % | 0.191 | 0.797 |
| 20° | 12.4 % | 1.344 | — |

Graceful to ~10°, then a knee. **`max_rotation_deg = 6.0`** sits inside the
graceful region with margin.

### ⛔ The reassuring argument that turned out to be FALSE

The obvious objection is that rotation inflates the median, so
displacement-ripeness fires anyway — round 38 measured a median inventing
4.79 px under 1.5°. **Measured here, it does not.** With this node's grid
bucketing the corners are spread symmetrically about the principal point, and
the component-wise median of a pure rotation is then ~0: **2.28 px at 6°**
against an 8 px floor. Grid bucketing, added to improve the fit, **removed an
accidental protection nobody knew was load-bearing.**

What remains without the cap is REACTIVE: tracks die, `n_used` falls below
`_MIN_TRACKS`, ripeness fires having already spent the interval.

### What the cap actually buys, stated honestly

Simulated station-keep-while-yawing, 3 s runs through a real floor frame:

| speed | yaw | old: recovered / emitted | new: recovered / emitted |
|---|---|---|---|
| 3 cm/s | 0.000 | 87.9 % / 7 | 87.9 % / 7 |
| 3 | 0.638 | 98.7 % / 19 | 98.6 % / **23** |
| 3 | 1.128 | 102.9 % / 41 | 102.1 % / **45** |
| 10 | 0.638 | 99.9 % / 34 | 99.9 % / 34 |
| 10 | 1.128 | 102.6 % / 59 | 102.8 % / **61** |

**It buys MEASUREMENTS, not accuracy** — +10 to +21 % emitted intervals while
yawing, distance recovery unchanged within 1 %, and exactly nothing when not
yawing. Sold as that, not as an accuracy fix.

### Two harness defects, both caught by controls rather than by reasoning

- **A relative warp against an absolute anchor.** Frames were warped by their
  offset from the *current anchor* while the anchor image sat at its own
  absolute offset, so the displacement LK saw was not the one commanded. The
  **zero-rotation control read 0.1 % of truth** instead of ~100 %, which is
  the only reason it was caught. Fixed to an absolute timeline.
- **The camera went BLACK mid-session** — mean pixel 0.00, and
  `detect_corners` still returned 12–19 "corners" on it. The room lost light
  at ~06:20 local with nobody present. One run (the median-inflation test on
  real frames) was invalidated and re-derived from geometry, which needs no
  camera. **Every frame-based measurement must assert its own frame is not
  black**; a featureless frame produces confident numbers, which is the
  covered-lens finding from §12 all over again.

### Caveat on the whole section

The valid runs had **Laplacian sharpness 50–58**. The archive's gate clip is
321 and the bin clip 1180. This is dim, poorly-textured imagery, so the
absolute survival percentages are pessimistic; the A/B comparisons between
arms are not, because both arms saw the same frames.

---

## 17. The calibration was wired to the WRONG CAMERA, both ways. 2026-09-07

Found while checking that the flat-port rectification (§14 above) would
actually reach the running system — because "a config change that reaches
nothing" is already on this package's record three times, and the round-32
plan wrote down *"assume a fourth."* This is the fourth.

### What was live

`vision_pi.launch.py` shipped:

    fwd_calibration  default  pi_forward_1280x720.json
    dwn_calibration  default  ''

The file's own metadata says
**`camera: pi_test_global_shutter (Microdia USB, bench test unit)`** — USB
vendor **0c45**, which is the **Sonix** unit, which is the **DOWNWARD**
camera. It was captured **2026-09-03**, four days before `f347827` found the
udev rules had the two cameras **swapped**. It was named for the camera the
system then believed it was looking at, and the name outlived the fix.

**Both halves were wrong and neither logged anything:**

| | consequence |
|---|---|
| **forward** (Fantech) | published `CameraInfo` with **fx 1027.87 / HFOV 63.82°** belonging to a different lens. **LATENT, not live — correcting this entry's first version, which said "every pixel→bearing for the srot vision uplink used the wrong focal length."** That overstates it: the sole consumer of `vision_state.calibration()` is `_vision_uplink_tick`, and the uplink is **default-off** (`vision_uplink_camera` defaults to `''`, so the timer is never even created). What actually shipped wrong is the **`CameraInfo` on the forward topic**; nothing consumed it in a default run. The severity is "wrong the moment the uplink is switched on", not "wrong on every mission" — and `bearing.py` exists precisely because a **26 px** principal-point offset is a **+1.264°** aiming bias at frame centre, so wrong intrinsics would make that correction wrong too. |
| **downward** (the DVL) | got **no calibration at all**, so `flow_node` fell back to one focal length and the frame centre — which is exactly the state round 38 measured a **3.08 % axis asymmetry** in and fixed. **The fix was verified in a console tool that passed the path by hand and never reached the launch.** |

So the headline 30 cm result was real, and the code path that produced it is
not the one the vehicle would have run.

### The fix, and the guard

File renamed `pi_downward_1280x720.json`, the two launch arguments swapped,
and the Fantech marked **uncalibrated** explicitly rather than by an empty
default that reads as an oversight.

The durable part: the JSON now carries **`applies_to: ["pi_downward"]`** as
its first key, `Intrinsics.from_json` stops **discarding** that field (it did,
which is why nothing could ever have caught this), and
`test_calibration_binding.py` parses the launch's own defaults and fails if a
calibration is wired to a profile it does not claim. It **reads the files and
never imports them** — a test that imports in a worktree resolves against the
main workspace's stale `install/`, which is how the CLAHE retraction nearly
went the wrong way.

Verified by restoring the original defect exactly: **2 of 3 tests fail.**
Also bites on a missing `applies_to` and on one naming the wrong camera.

### The lesson, stated generally

**A verification that passes a path by hand does not verify the launch.** Every
number in §13 was produced by `flow_console.py --calibration <path>`; the
launch was never in that loop. When a result and a deployment disagree about
where a file comes from, the result is about the tool.

### Still open, and it is not small

**The Fantech forward camera has never been calibrated.** It is the camera the
vision uplink aims with. Until it is, `bearing.py` on the forward path is
running on intrinsics that belong to another lens, or on none. Added to
`water-owed.md`.

**Verified through the launch on the vehicle, 2026-09-07** — which is the
point of this entry, since every §13 number came from a tool passing the path
by hand:

```
[FLOW ] camera='downward' medium='water' f=685.1px (air 513.9 / water 741.0)
        port=RECTIFIED  pool_depth=1.60m
[FLOW ] VELOCITY PATH DISABLED -- pool_depth_m was never set     (flow:=true alone)
[FLOW ] REFUSING: no trackable texture (0/19 points survived)    (the dark room)
```

`flow_node` was **in no launch file at all** — the DVL had to be started by
hand — and now runs as `vision_pi.launch.py flow:=true pool_depth_m:=<m>`,
taking the same `dwn_calibration` the camera takes, by reference. A renamed
data file also needs `rm -rf build/<pkg> install/<pkg>` first: colcon caches
the file list and fails with `can't copy ...: doesn't exist` naming the file
you deliberately removed.
