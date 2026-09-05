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
