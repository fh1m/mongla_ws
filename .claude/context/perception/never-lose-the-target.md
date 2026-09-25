# Never losing the target — the research arsenal

> Read before touching the lock ladder, the tracker, the bank, or anything that
> decides whether the vehicle still has its target. Every claim here is from a
> source that was **read**, not cited, and each carries what it costs.

---

## 1. What the state of the art actually says

| source | mechanism | measured |
|---|---|---|
| **Enhanced CenterTrack**, 2026 | **three-stage cascade** — Mahalanobis motion → IoU → centroid; each stage sees only what the last could not match. Plus an occlusion-aware head predicting a *visibility state* | IDF1 **75.5 → 82.5**, MOTA 83.1 → 85.8 |
| **McByte++**, 2026 | training-free: mask propagation + conditional camera-motion compensation + **online re-identification** | up to **+6.1 IDF1**, order-of-magnitude speed-up |
| **BoT-SORT** | motion **+ appearance + explicit camera-motion compensation** | "maintain stable object identities" |
| **DAM4SAM** (CVPR'25 / IJCV'26) | **distractor-aware memory**: recent-appearance memory (RAM, FIFO) separate from distractor-resolving memory (DRM), with introspection deciding which | robustness **0.887 → 0.944**; SOTA on 10 of 13 benchmarks |
| Occlusion-aware SAM2, 2026 | **dual memory**: non-occlusion bank + occlusion bank, "to prevent memory contamination and improve re-detection after occlusion" | — |
| **DQR-RTDETR**, 2026 | degradation-aware **query reliability recalibration** for embedded underwater detection | — |
| the standing objection | *"a dedicated Re-ID embedding network... an extra **15–25 ms per frame**"* | why embedded trackers skip appearance |

**The convergent recipe:** motion alone cannot hold identity across a gap.
Every top result adds (a) an **appearance** cue, (b) **camera-motion
compensation**, and (c) **memory hygiene** — and the cascade is what makes (a)
affordable.

---

## 2. What we have that they do not

### ⭐ XFeat is already loaded
A dedicated Re-ID network costs 15–25 ms/frame, which is why embedded trackers
skip appearance. Ours runs at **701 FPS on the Hailo-8** (§38) and shares the
chip with the detector for ~10 % of its throughput (§32) — because we built it
for the anchor rung. **We get the one expensive ingredient for free.**

### ⭐⭐ Ego-motion is MEASURED, not fitted
BoT-SORT estimates camera motion by fitting an affine transform *between
images* — a surveillance camera has no other way. This vehicle has a **verified
velocity sensor** (downward camera, 1.09 cm over 30 cm) and attitude at 50 Hz
with **gyro bias already removed** (0.1 °/hour residual, §37). Two cues that do
not share a failure mode with the pixels being tracked.

### ⭐ One number, four jobs
XFeat's inlier count is simultaneously an **identity score** (re-ID), a
**turbidity proxy** (confidence calibration), a **memory-health signal**
(fouled lens vs changed world), and a **viewpoint-change detector** (evidence
accumulation). It is computed on every bank lookup and was being discarded.

---

## 3. ⛔ What the champions have that we do not

From **Bumblebee's own RoboSub 2026 paper**, read directly:

> "hybridised forward-looking sonar and camera feeds… cross-modal target
> state-estimation, ensuring high-accuracy object localisation **even when
> visual tracking degrades at longer ranges or in low-visibility competition
> environments**."

**Sonar is their answer to losing the target.** It is a genuinely decorrelated
modality and we have nothing equivalent. At long range in bad water, they see
and we do not. That is the honest gap.

### ⭐ And the weakness they state themselves

> "the Doppler Velocity Log (DVL) experiences acoustic beam reflection…"
> during high-rate angular manoeuvres such as a barrel roll, or **"when
> operating in close proximity to pool bottoms and walls"**.

**Their velocity sensor degrades exactly where ours improves** — optical flow
gets *better* as altitude drops, because the floor fills more of the frame.
Bin and grasping tasks happen near the floor. That is an asymmetric advantage
and it is in their own words.

They are also going **multi-vehicle with acoustic inter-vehicle comms** for
2026 — a large complexity budget we do not have to match.

---

## 4. The vision-only answer to sonar's job

Sonar gives them range and bearing when vision fails. We cannot get range
without vision — but we can get **expectation**:

- the course map holds where props are (per venue, deck-overridable)
- the localiser holds where the vehicle is
- ⭐ therefore the stack can say **where a prop should appear in frame** before
  the detector has seen anything

A detector searching a predicted region is more sensitive than one searching a
whole frame — the same argument `range_crop` already makes for size, driven by
**position** instead. `CheckpointBank.locate(near=…, radius_m=…)` already takes
the prior; nothing supplies it yet.

⚠ Unmeasured. Recorded as the plan, not as a result.

---

## 5. Built from this, and where each stands

| module | state |
|---|---|
| `anchor/health.py` — fouled lens vs changed world | **wired** into `lock_node` |
| `anchor/bank.py` contamination guard — refuses to remember a view of the fault | ⭐ **wired into `enrol()`**, the DAM4SAM principle without a second bank |
| `tracking/reid.py` — XFeat short-term identity memory, §25's 40-inlier bar, 1.5× margin | built, 13 tests, **deferred**: needs crop descriptors |
| `tracking/cascade.py` — motion → **measured ego-motion** → appearance, capped at 4 calls | built, 17 tests, **deferred**: touches the association hot path |
| `detection/confidence_calibration.py` — turbidity-aware, ships OFF | built; DQR-RTDETR validates the idea, our gain is still declared |

---

## 6. ⛔ What none of this fixes

Nothing here creates a detection where there is none. Every mechanism above
**holds an identity across a gap** or **stops a bad memory forming**; none
invents a box. The ladder's rule stands: a rung that cannot see the target
says so, and a verb that reports success while the vehicle does nothing is the
failure mode that ends runs.

**The gap that remains is sonar's job** — long range, low visibility, no
detection at all. §4 is the only vision-only answer, and it is unmeasured.


---

## 7. Zero-shot and open-vocabulary: wrong for flying, right for labelling

**YOLO-World** (35.4 AP LVIS, 52 FPS), **YOLOE-26**, **OWL-ViT**,
**Grounding DINO** — the promise is a detector that needs no training for a
new prop.

⛔ **They are not robust where we operate.** *Open-Vocabulary Object Detectors:
Robustness Challenges under Distribution Shifts* evaluates all three against
COCO-O / COCO-DC / COCO-C and finds they **"exhibit significant deviations in
performance"** under information loss, corruption and geometric deformation.
Turbidity, colour cast and backscatter are a severe distribution shift, and our
own closed-set recall already swings **29.2 / 72.7 / 68.3 %** across venues on
models trained *for* this water. An open-vocabulary model would start worse and
degrade faster.

⚠ OWL-ViT additionally needs image↔text embedding interaction **at inference**,
which is why NanoOWL exists and why it targets a Jetson rather than a Hailo.

⭐ **Where they do belong: labelling, not flying.** An open-vocabulary detector
is good enough on the EASY frames of our archive to pre-label them, and those
labels train the closed-set model that actually flies the hard ones. That is
ROADMAP **M6** (real-pool auto-labelling), still open, and this is the tool for
it. The robustness objection does not apply off-vehicle: a human reviews the
labels, and a wrong one costs a correction rather than a run.

---

## 8. ⛔ Corrections to our own backlog, found by reading our own measurements

**The Hailo async API is DONE, and it was measured as buying nothing.**
It was carried on the "open work" list as *13.4 ms → ~7 ms*. `hailo.py` already
uses `create_infer_model` + `run_async`, and `hailo-vision.md` records:

```
async depth 2      225 Hz raw (1.60x)   ->   98.2 Hz end-to-end (1.00x)
async depth 4      292 Hz raw (2.07x)   ->   98.2 Hz end-to-end (1.00x)
```

⭐ **2x raw throughput, zero end-to-end gain.** It was adopted for a completely
different reason, which is the part worth remembering:

```
InferVStreams.infer()        9.21 ms late, 99.6 %   (holds the GIL throughout)
InferModel run_async + wait  0.05 ms late,  0.0 %
```

The win was **the GIL, not the chip**. Anyone who re-opens this item looking
for throughput will re-measure 1.00x.

⚠ The real remaining gap is the graph, not the accelerator: **80.9 Hz
standalone against 53.9 Hz through the ROS graph**. That is where the
throughput went, and it has not been attacked.


---

## 9. ⭐⭐⭐ THE GOLDEN GOAL NEEDS ANTICIPATION, NOT BETTER RECOVERY

Everything in sections 1-8 makes the ladder **recover** better. None of it
changes the fact that **every rung fires after the detection is already gone**:
the follower takes over once there is nothing left to follow from, and the
anchor is asked once the box has vanished. By then the reference worth having —
the one from while the view was still good — is a second in the past and
unrecoverable.

### The SOTA name for the missing capability is INTROSPECTION

*Online Monitoring of Object Detection Performance During Deployment* and
*Per-frame mAP Prediction* both predict performance drops **from the detector's
own internal features, with no ground truth**, and frame the decision as
trading a false alarm against absenting from detection. Robotic introspection
is described as "the introspection capability of a mobile robot while operating
in an unknown environment".

What it requires is a **per-frame quality signal that needs no label**.

### ⭐⭐ We already compute one and were throwing it away

XFeat's inlier count against the bank falls as the water thickens, as the
target turns away, as range opens — **all the things that precede a lost
detection**. It is measured on every bank lookup and was used only to pick a
winner.

A **falling trend** is a prediction that the next seconds will be worse than
the last. `health.trend()` compares the recent half of the series against the
earlier half — not a slope, because a slope is dominated by its endpoints and
one catastrophic frame would swing it; halves are what a gap actually looks
like.

⭐ **Wired**: a falling trend is now a fourth reason to enrol, beside stale /
uncovered / room. It is the only one of the four that is **predictive** — the
others describe the bank, this describes where the view is heading.

⛔ **And it licenses exactly one action.** It says *prepare*, never *the target
is gone*. Snapping a reference early is cheap and reversible; snapping late
costs the very view it needed. A test asserts the trend object carries no
`lost` or `target_gone` field, because the moment this becomes a control input
the ladder's rule — no rung fabricates a position — is gone.

⚠ `TREND_FALL = 0.70` is **declared, not measured**. What is measured is that
the signal exists and moves the right way. The bar wants a pool day with real
gaps either side of it.

---

## 10. The shape of the whole thing

```
            ANTICIPATE      falling XFeat trend  ->  enrol NOW      (section 9)
                 |
    DETECT  ->  FOLLOW  ->  ANCHOR  ->  LOST
       |          |            |
       |          |            +-- bank, contamination-guarded   (section 5)
       |          +-- LK + forward-backward + RANSAC similarity  (scale!)
       +-- cascade: motion -> MEASURED ego-motion -> XFeat re-ID  (sections 2,4)
```

Each addition attacks a different half of the same failure:
**holding identity across a gap**, and **not making the gap worse by
remembering the wrong thing** — with one new stage that tries to see the gap
coming.


---

## 11. ⭐⭐⭐ THE LAST PIECE: THE VEHICLE IS THE LARGEST CAUSE OF LOSING THE TARGET

Sections 1–10 are all **perception**. Every one of them — the cascade, the
re-identification, the contamination guard, the anticipation — operates in the
**image plane**, downstream of a decision the vehicle has already made about
where to point.

⛔ **Grepped `mongla_control`: there is no field-of-view guard anywhere.**
Nothing stops the controller yawing the target out of frame, and **no tracker
survives that**. The perception stack can be perfect and still lose the lock.

### The SOTA says it twice

A 2026 safety-critical visual-servoing result observes that blocking the
camera–target line causes failure **"even when the robot remains physically
safe"**, and that holding visual contact **"can conflict with navigation
progress and collision avoidance"** — so it keeps collision constraints **hard**
and gives field of view **slack**. Perception-aware planning encodes the same
thing as a cost: keep the feature in the cone, keep its image-plane velocity
small.

The barrier is `h = β·(R e_c) − cos(ψ_F)`, and the property that matters here
is that a published splitting strategy makes it **robust to bounded distance
error** — it needs **bearing, not range**, which is exactly our situation.

### ⛔ And a real departure from the paper, forced by measurement

The published barrier is **conical**. A camera's image is a **rectangle**, and
they disagree precisely where it matters. Measured on our own geometry
(640×480, fx 500): a box sitting on the **right-hand edge** scores
**h = +0.061 against the conical barrier** — comfortably "safe" — while being
one pixel from gone. The cone closes only at the diagonal **corner**, so a
target can walk out of the left edge with margin reported the whole way.

⭐ So the shipped barrier is the fraction of half-width/half-height remaining,
whichever is smaller: **1.0 dead centre, 0.0 on any edge**. Bearing is still
reported because it is what a controller acts on; it is not what the barrier is
made of.

```
dx=  0  ok         h=1.000  scale=1.00
dx=260  ok         h=0.188  scale=1.00
dx=290  near_edge  h=0.094  scale=0.28
dx=310  critical   h=0.031  scale=0.00
dx=320  lost       h=0.000  scale=0.00
```

### ⭐⭐ The second half: leaving the frame must not mean forgetting

Image-plane tracking forgets the instant the box leaves. The fix is old and a
robot-behaviour patent states it plainly: store target position **"not on a
sensor coordinate system but on a world coordinate system"**, so it "remains
identical from the behaviour control perspective" however the vehicle moves.

`WorldTarget` projects a sighting into the pool frame using the localiser's
pose. Losing sight then stops being a perception failure and becomes a
**navigation** one — the vehicle still knows the bearing to turn back to.

⛔ It stores a **memory, never a sighting**: it refuses a non-finite or absurd
range rather than storing a guess, it answers `None` once the estimate is
staler than its horizon (a position from a minute ago is a rumour, not
knowledge), and a test asserts it carries **no `score`, `confidence` or
`detected` field** that anything downstream could mistake for a detection.

### Where the golden goal now stands

| failure | answer | state |
|---|---|---|
| target flickers | ladder + coast | shipped |
| target changes appearance | bank, contamination-guarded | shipped |
| identity fragments after a gap | XFeat re-ID + cascade | built |
| vehicle's own motion moves the box | measured ego-motion stage | built |
| the view is about to degrade | falling-trend anticipation | **wired** |
| target approaches or recedes | similarity fit tracks scale | **wired** |
| **the controller steers it out of frame** | **visibility guard** | built |
| **it leaves anyway** | **world-frame memory** | built |
| no detection at all, long range, bad water | ⛔ **sonar's job — still open** | §3 |

---

## §57 — The research that mattered turned out to be about EVALUATION, not tracking

Rounds of searching for a better tracker found the real gap somewhere else:
the way we chose and measured models. Four independent literatures say the
same thing, and all four match a defect we measured on our own footage the
same week.

**COCO-FP** (*A Deep Dive into Background False Positives for COCO
Detectors*) — background errors are false positives on **non-target visual
clutter**, and standard benchmarks do not contain enough of it to measure
them. Ours: `gate_rescue_repair` fires on pool structure — lane lines, floor
seams, the wall/floor horizon — at up to **0.92**, higher than the 0.89 it
gives the real gate (B-60).

**Egocentric visual query localisation** — *"models never see such negative
samples in both training and evaluation."* Ours, exactly: every bar in
`measured-bars.md` §1 came from labelled held-out **pairs**, frames that
contain the prop. A set built that way cannot measure a false positive on
open water, because it holds no open water.

**Deployment-aware model selection (2026)** — *"when deployment-aware methods
were applied, model choice changed from benchmark-based selection **in all
tested splits**."* Ours: recall-based ranking chose a model with **+0.0 points**
of separation between gate-present and gate-free footage. Negative-separation
ranking chose a different model already in the archive with **+100.0**.

**Saturated benchmarks** — *"benchmarks that expand rather than saturate,
metrics that diagnose rather than simply score."* Ours twice over: `mAP50`
saturated at 0.995 across all 25 archived runs (this is why
`model_select.py` exists), and then **recall itself saturated at 1.0**, which
is why `model_select.py` in turn ranked the wrong model.

⛔ **The pattern, and it is the durable lesson.** Each time we replaced a
saturated metric we replaced it with another metric measured **only where the
answer is yes**. mAP → recall → nothing, until a negative set was introduced.
The ladder was never the weak link; the instrument that told us the detector
was good was.

### What was tried against the false positives and FAILED

Three runtime rejectors, all dead, all measured (table in `BUGS.md`):
**temporal persistence** (the hard-negative-mining literature's "false
positives are isolated in time" does not transfer — ours persist 400 frames of
400, because the camera stares at the same empty water), **box geometry**
(area straddles both sides), **edge energy inside the box** (*backwards*:
6.69 real vs 6.82 and 9.40 hallucinated — turbid water is not smooth, and what
a human reads as "nothing there" is semantic, not photometric).

⭐ **The fix was not an algorithm.** It was looking at 34 clips' worth of
rendered boxes, and then searching the archive: **208 `.pt` on this box, 96
unique, 29 with a gate class**, against the 4 that ship. The better model had
been sitting there the whole time.

### FOV and object permanence — confirmed by others, still half-wired

**VISTA-CBF+ELR (2026)** keeps collision constraints hard while relaxing
field-of-view ones — independently confirming the `SLACK = True` decision
already written into `visibility.py`, and adding one idea we do not have:
**occlusion-evasive replanning**, acting before the target is lost rather than
after.

**Object permanence** (*Out of Sight, Still in Mind*; RoboStream 2026's causal
memory) — a target leaving frame should become a navigation problem, not a
forgotten one. `WorldTarget` implements this and is reached by **tests only**;
its blocker is two missing signals (yaw, absolute range), named precisely in
`ladder-register.md`. Do not wire it by inventing a range.

---

## §58 — Getting more out of XFeat, after the fine-tune said no

Round 7 closed NO: stock XFeat beat every checkpoint trained on our own water
(2237 inliers against 1986–2056). That is a real result about the descriptor,
and it removes *training* from the list of ways to improve the anchor. It does
not remove the others, and this session found that the largest one had nothing
to do with the network at all.

### ⭐ 1. It was running on the wrong processor (B-62, the big one)

Measured on the vehicle, 320×240, 30 runs:

| | per call | rate |
|---|---|---|
| ONNX on the Pi CPU — **what ran** | **32.9 ms** (p95 33.4) | 30.4 FPS |
| HEF on the Hailo-8 — what §28 measured | 1.43 ms | 701 FPS |

The resolver globbed `xfeat_*.onnx` only, so the compiled HEF **in the same
directory** was never a candidate. No amount of retraining would have closed a
23× gap that was a missing file extension.

### 2. What the literature actually offers

**xfeatSLAM** (`udaysankar01/xfeatSLAM`) — XFeat as the front end of ORB-SLAM3,
real-time. ⚠ Not something to adopt: a full SLAM system is a much larger claim
than our anchor rung makes, and we have no loop-closure budget on a 15-minute
run. What IS worth taking is the demonstration that **XFeat descriptors are
good enough to carry a map**, which is a stronger statement than "good enough
to re-find one patch" — our checkpoint bank is a deliberately small slice of
that, and the paper's result says the slice is safe.

**XfeatVINS** (2026) — monocular **thermal**-inertial SLAM on XFeat, for
all-time operation. The transferable part is not the thermal: it is that XFeat
holds up as the front end of a **tightly-coupled inertial** system. We already
have the IMU and the RIEKF; the anchor currently feeds neither.

**The XFeat paper itself** — it emits three heads: a keypoint heatmap, a 64-D
dense descriptor map, **and a reliability heatmap**. ⚠ We use two. The
reliability head is exactly a per-keypoint confidence, and our `min_cossim`
0.82 is a global constant doing that job badly. Worth measuring.

**Semi-dense matching** — XFeat's second mode, for when sparse matching finds
too few correspondences. Our failure mode in murky water is precisely too few
inliers, and we have never tried it.

### 3. The honest ranking of what is left

1. **run it on the chip** (B-62) — 23×, measured, no new maths;
2. **use the reliability head** — replaces a global cosine bar with a
   per-keypoint one, using an output we already compute and discard;
3. **semi-dense mode** when sparse falls below the inlier bar — turbidity is
   exactly the case it was built for;
4. ✅ **ALREADY DONE — this item was wrong.** Checked 2026-09-25: the anchor
   DOES feed the filter. `lock_node` publishes `/mongla/localization/fix`
   (a `PointStamped` carrying the closure position with the producer's own
   sigma in `z`), `localization_node.py:182` subscribes it, and
   `test_every_sensor_is_accounted_for.py` records it as "position: prop
   resection AND loop closure". The route is **loop closure**, not per-frame
   descriptor aiding, which is the right granularity: a bounded absolute fix
   rather than a stream the filter would have to de-weight.
   ⚠ What XfeatVINS does beyond this is tightly-coupled PER-FRAME feature
   aiding, which would need the forward metric range we do not have. That
   distinction is real; the claim "the anchor feeds neither" was not;
5. ⛔ **more training** — measured, closed, do not reopen without a new idea.

⚠ Items 2 and 3 are RESEARCH NOTES, not findings. Neither has been measured on
our footage, and §57's lesson is that an idea consistent with the symptom is
not evidence.

### ⛔ The reliability head does NOT predict which keypoints survive a match

§58 listed "use the reliability head" as the second-best remaining XFeat idea:
the paper emits one, we compute it on-chip, and `min_cossim = 0.82` was doing
that job as a single global constant. It is now decoded (`conv23`, 30x40x1,
exposed as `XFeatHailo.last_reliability`) and **measured on the vehicle**,
three consecutive frame pairs of `gate.mkv`:

| pair | matched kp | mean rel | unmatched kp | mean rel | ratio |
|---|---|---|---|---|---|
| 0 | 382 | 0.14 | 642 | 0.11 | **1.289** |
| 1 | 359 | 0.17 | 665 | 0.15 | **1.137** |
| 2 | 493 | 0.17 | 531 | 0.17 | **1.050** |

⛔ **A 5–29 % separation, decaying toward 1.0.** Keypoints that survive a match
score barely above those that do not, and by the third pair the two
populations are indistinguishable. That cannot replace a cosine bar which
currently rejects on a real similarity.

⚠ **Why the idea was still worth testing, and what the failure teaches.** The
reasoning was sound — a per-keypoint confidence should beat a global constant
— and it is the same shape as §57's lesson: *an idea consistent with the
symptom is not evidence*. Three ideas have now died this way (temporal
persistence, box geometry, edge energy) and this is the fourth. The decode
stays, because it costs nothing and a future consumer may want it; the
`min_cossim` bar stays as the thing that actually decides a match.

⚠ One caveat stated honestly: this measured reliability against
**frame-to-frame** matching on clear-ish pool footage. The anchor's real job is
frame-to-REFERENCE across seconds and viewpoint change, where the two
populations may separate differently. That is a different experiment and it
has not been run — but on the evidence here, nothing should act on this head.
