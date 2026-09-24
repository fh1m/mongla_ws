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
