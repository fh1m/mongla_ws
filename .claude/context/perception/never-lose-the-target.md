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
