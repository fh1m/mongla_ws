# Underwater computer vision, from 55 GB of our own data

> Every number here is measured on the BRACU Duburi 2025 archive — 21 labelled
> datasets (10,188 images), 25 training runs, 33 competition videos — plus live
> runs on the Pi 5 + AI HAT+. Where something is reasoning rather than
> measurement it says so. Where a hypothesis was disproved, the disproof is
> kept, because those are the entries that stop the mistake being repeated.
>
> **The goal is not to be good on this data.** It is to understand the problem
> well enough that a venue we have never seen is characterised in advance
> rather than discovered mid-run.

## 1. The thing that is actually different underwater

Air vision degrades mostly by *lack of light*. Water degrades by four
mechanisms that are physically distinct, and treating them as one
"visibility" number is why a fix that works in one pool fails in the next:

| mechanism | what it does | what measures it | what fixes it |
|---|---|---|---|
| **absorption** | kills wavelengths with range — red by ~5 m, then green | saturation, colour cast | nothing optical; shorten the range |
| **backscatter** | adds veiling light, crushes dynamic range | contrast (luminance σ) | local contrast enhancement (CLAHE) |
| **forward scatter** | blurs edges without dimming | sharpness (Laplacian var) | nothing cheap; shorten the range |
| **motion blur** | blurs edges, ∝ angular rate × exposure | sharpness | **exposure**, not preprocessing |

`duburi_vision/underwater.py` measures all five quantities. The key one is
**saturation**, and it is the one most people leave out: it separated the two
regimes where the same preprocessing helped by +42 points and hurt by −64,
*more sharply than blur did*.

**Blur and scattering are indistinguishable from a single frame** — both flatten
edges. They separate over *time*: motion blur tracks the vehicle's angular
rate, scattering does not. We publish body rates at 50 Hz and have never used
them for this.

## 2. What our archive actually contains — and does not

Sampled twice with different seeds and sample sizes, so the split is the venue
and not the sampling:

| | datasets | median colour cast |
|---|---|---|
| green / murky | **15** | **+100** |
| clear | 6 | +2 |

Two populations with **nothing between them**. Sharpness spans **9 → 3365**, a
370× range.

**71 % of our labelled data is one water type.** That is the generalisation
risk stated as a number, and it is invisible in any per-dataset metric.

And the range coverage, which matters more for missions than anyone had
checked:

| target size (∝ range) | share of all labelled objects |
|---|---|
| < 0.2 % of frame (far approach) | **0.5 %** |
| 0.2–1 % | 13.8 % |
| 1–5 % | 38.7 % |
| 5–20 % | 28.4 % |
| > 20 % (arrived) | 17.7 % |

The gate datasets are **46–65 % "arrived"** — a gate filling half the frame.
We have almost no data of a prop at the distance where an approach *begins*.

## 3. The three findings that generalise beyond RoboSub

### 3a. A validation score cannot predict a venue — and how I misread it

All 25 archived training runs report **mAP50 = 0.995**. Real recall, against
ground truth, per dataset:

| model → dataset | bin | octagon | torpedo | gate |
|---|---|---|---|---|
| bin | **100.0 %** | 1.4 % | 15.9 % | 2.0 % |
| octagon | 0.0 % | **88.4 %** | 10.2 % | 23.8 % |
| torpedo | 8.7 % | 1.8 % | **81.4 %** | 2.0 % |
| gate | 3.3 % | 0.7 % | 0.0 % | **24.8 %** |

Diagonal 81–100 %, everything off-diagonal 0–24 %.

**⛔ MY FIRST READING OF THIS TABLE WAS WRONG, and the correction is the more
useful result.** I concluded "these models learned their dataset, not their
object". That is not what the table shows: **each column is a DIFFERENT PROP**,
so a bin model scoring 1.4 % on octagon images is behaving correctly. The
table measures nothing about generalisation.

The right experiment is the same prop across different sessions:

| torpedo model → | v1 | v2 | Torpedo_Down | Torpedo_UP_1 |
|---|---|---|---|---|
| recall | 81.5 % | 72.9 % | **100.0 %** | **98.3 %** |
| sharpness of that set | 51 | 31 | 519 | **2761** |

**It generalises across a 54× sharpness range.** Different day, different
camera position, wildly different blur — 73–100 %.

And the same prop across the *water* boundary:

| octagon model → | Octagon | new/final | oct_zawad | Octagone_surface |
|---|---|---|---|---|
| recall | 99.2 % | 89.3 % | 68.9 % | **2.9 %** |
| colour cast | +3.9 | +4.2 | −12.9 | **+95.8** |
| classes | yellow_box… | yellow_box… | red_box… | **shark_octagon** |

The 2.9 % set crosses *both* a water boundary (+96 vs +4) **and** a class
boundary — it labels a different physical object. So even this does not
isolate water cleanly, and I am not going to claim it does.

**What the data actually supports**, stated no more strongly than it earns:

- Models transfer well across capture sessions of the same prop, including
  very different blur — better than I expected.
- Recall degrades gradually with optical distance from the training set
  (99 → 89 → 69 %) rather than falling off a cliff.
- The archive contains no clean same-prop, same-classes, cross-water pair, so
  **the pure water-transfer question is UNANSWERED with this data.** Collecting
  one prop in both waters would answer it, and is the cheapest experiment
  available.

**The generalisable rule that does survive: validate on a different capture
session, not a held-out split.** A split drawn from the same session shares
water, lighting and camera, and 0.995 mAP on it told us nothing that the
73–100 % cross-session numbers did not have to establish independently.

### 3b. Contrast enhancement is conditional, and the condition is measurable

| | gate approach | torpedo on bin |
|---|---|---|
| baseline | 53.6 % | 79.5 % |
| **+ CLAHE** | **95.4 %** (+42) | **15.3 %** (−64) |

CLAHE helps **blurry, low-contrast, saturated** water and **hurts** sharp,
desaturated water. Physically sensible: it amplifies local contrast, which
recovers a washed-out target and over-sharpens a crisp one past the model's
decision boundary.

**It does not fix the domain gap.** Applied across the cross-water matrix it
is neutral-to-worse off-diagonal. It is a *contrast* fix, not a *domain* fix,
and conflating the two would have been the round's biggest error.

### 3c. Recall falls off a cliff with range, and resolution is the lever

Simulated approach, real labelled data:

| ~range | full frame | centre 50 % crop |
|---|---|---|
| 1.0× | 100.0 % | 69.0 % |
| 2.9× | 98.4 % | 99.2 % |
| **4.0×** | **65.9 %** | **100.0 %** |
| 6.7× | 20.2 % | 67.4 % |

Beyond ~3× the training distance the detector is effectively blind. **Lowering
confidence does not help** (65.9 → 69.9 % at 4×) — this is a *resolution*
limit, not a threshold one, and conflating them wastes a round tuning the
wrong number. `imgsz` 960 recovers it (66 → 94 %) but is **baked into the HEF**
on the Hailo, so cropping is the runtime equivalent.

## 4. The engineering principle this forces

**Every gain is an exchange, and both sides must be measured.**

| lever | gains | costs |
|---|---|---|
| CLAHE | +42 pts presence on murky water | −64 pts on clear; 79 → 50 Hz |
| range crop | 66 → 100 % recall at 4× | −31 pts up close; half the FOV |
| conf 0.15 → 0.10 | +8.5 pts presence, jitter flat | more boxes to process |
| conf → 0.05 | +2.5 pts more | 35 % multi-box frames, jitter +53 % |

None is universally good. All are switchable, and the switch is `vision:=`
one word, because a competition day is not the time to reason about four
interacting knobs.

**And the mission framing above all**: a pipeline at 77 Hz that cannot see the
gate until it is 1 m away has gained nothing. Numbers that do not become
*behaviour* are not results.

## 5. What we should do that we have not

Ranked by measured evidence, not by novelty.

1. **Validate on a different capture session.** §3a says our validation
   protocol cannot detect the failure it exists to detect. This costs nothing
   but discipline.
2. **Collect long-range data.** 0.5 % of objects below 0.2 % of frame, and
   recall collapses exactly there. The cheapest dataset improvement available.
3. **Cap exposure.** Both cameras run auto with `exposure_dynamic_framerate=1`,
   so dark water buys brightness with blur *and* a lower frame rate. Blur ≈
   angular rate × exposure, and this is the only blur lever needing no
   retraining.
4. **Blur and rotation augmentation.** Not one of the 25 training configs used
   any (`degrees: 0.0`, `perspective: 0.0`, no blur). The models have never
   seen a blurred frame, and §2 says most of our water is blurry.
5. **Ego-motion compensation.** EMAP cuts ID switches 73 % on OC-SORT — our
   tracker. We publish the body rates and `rotation_flow_px()` already exists.

## 6. Disproved, and kept

- ~~"Stills-trained models fail on video."~~ The octagon model scores 96.7 % on
  labelled stills and 92.4 % on its own video. The *footage* is the variable.
- ~~"A gap that survives every threshold must be geometry."~~ The 25 s gap I
  called genuine absence collapsed to 2.7 s under CLAHE. The prop was in frame
  the whole time.
- ~~"CLAHE is a win."~~ It is a win in one measured regime and a 64-point loss
  in the other.
- ~~"Low confidence recovers occluded targets, so lower is better."~~ True to
  0.10, false below it: at 0.05 a third of frames carry spurious boxes.

Each was plausible, each was believed, and each was overturned by measuring
something a second time. **That is the method, and it is worth more than any
individual number above.**


## 7. A tool bug that produced a finding-shaped number

`slot-1_obb` measured **3.6 % recall** where a neighbouring dataset with the
same classes and nearly identical water measured 68.9 %. That 20× gap is not
something a model does, which is what made it worth chasing rather than
recording.

The dataset uses **oriented bounding boxes** — 9 fields, four corner points —
and my label parser read fields 1–5 as `xywh`. It did not raise. It produced
plausible boxes, some with **negative area (−9.08 %)**, and reported a working
model as broken. Fixed: **3.6 % → 69.1 %**.

Two things worth keeping from it:

- **A number that implies an implausible mechanism is a bug until proven
  otherwise.** Models degrade; they do not go from 69 % to 3.6 % between two
  near-identical datasets.
- **The neighbour is what made it visible.** A single measurement of
  `slot-1_obb` would have been believed. Two similar datasets disagreeing by
  20× could not be.

The cross-prop matrix in §3a is unaffected — none of those four datasets uses
oriented boxes — but it was re-run to confirm that rather than assumed.

---

## 6. Validation cannot rank our models — measured, three ways

Round 34 addendum. Everything above concerned the data; this concerns how we
**choose** what we train on it, and it is the finding with the most immediate
consequence.

### The measurement

Three octagon models from the 2025 archive, each reporting **mAP50 = 0.9950**
on its own validation split, evaluated on two sessions none of them trained on
(`tools/recall_matrix.py`, conf 0.15):

| model | val mAP50 | val mAP50-95 | `oct_zawad` | `slot-1_obb` |
|---|---|---|---|---|
| `robosub_octagon_n_200_v1` | 0.9950 | 0.8470 | **29.2 %** | **23.3 %** |
| `robosub_octagon_n_200_final` | 0.9950 | 0.9278 | **72.7 %** | **71.4 %** |
| `robosub_octagon_n_200_final_again3` | 0.9950 | 0.9570 | 68.3 % | 65.4 % |

A **2.5-3x spread in real recall**, with the ranking reproducible across both
unseen sessions — and `mAP50` reports all three as **exactly identical**.

### The part that is not a validation-set problem

`v1` and `final` have **literally identical training configurations**. A diff of
their `args.yaml` files, excluding only the run name and output path, is
**empty**: same `yolo11n.pt` base, same dataset, 200 epochs, `imgsz 640`,
`lr0 0.01`, `seed: 0`, `deterministic: true`.

Two runs that differ in nothing recorded produced a **43-point recall gap**.
Whatever varies (dataloader worker order, cuDNN kernel selection — `seed: 0`
plus `deterministic: true` evidently did not pin it) is invisible in every
artefact the training produces.

### What each metric is actually worth

* **`mAP50` is saturated and ranks nothing.** All 25 archived runs report
  0.995, and most reach it by **epoch 27-38** of a 200-300 epoch run — e.g.
  `robosub_gate_200_final2` at epoch 31/200, `robosub_bin_front_n_300_v1` at
  27/300. Validation loss is still falling at the end of every run, so nothing
  in the curve says "stop"; the metric simply ran out of resolution on the
  first tenth of the schedule.
* **`mAP50-95` retains signal within one dataset.** It is the only recorded
  number that separated the two identical-config runs, and it ranked them
  correctly (0.847 → 29 %, 0.928 → 73 %). Across *different* datasets it is not
  comparable — `again3` scores highest (0.957) and places second.
* **Cross-session recall is the only quantity that predicts deployment.**

### What to do instead

`tools/model_select.py` ranks candidates on held-out sessions and picks on the
**worst** one, not the mean — a model that is excellent at one venue and
useless at another loses the run it is used in. Train several with the same
config (the variance above means that is not a wasted run) and rank them here.

The cost is one evaluation pass per model per session: minutes. The cost of not
doing it is picking `v1`.

### Why this ranks above every tuning knob in §5

CLAHE moved presence 10.7 → 56.2 %. The conf floor moved it 8.5 points. Model
selection is a **48-point** swing on data we already own, and it is free.

---

## 7. Mirpur: a third water type, and two retractions it forced

The 2025 archive holds a venue nothing in §1–§6 had opened:
`raw_images/Mirpur/sun_june_29/` — **1,918 frames from a Bangladeshi pool in
June**, two months and a continent away from the RoboSub footage, holding the
**same props**: the RoboSub gate with its animal placard, and the RoboSub
torpedo board with its shark/sawfish artwork and two red rings.

That is the clean cross-water, same-prop experiment §5 said did not exist.

### The water

| set | sharpness | saturation | contrast | cast |
|---|---|---|---|---|
| **Mirpur gate** | **31** | 147.7 | 22.7 | **+91.7** |
| **Mirpur torpedo** | **36** | 154.0 | 22.1 | **+97.8** |
| RoboSub torpedo | 521 | 153.4 | 35.7 | +113.2 |
| RoboSub bin | 641 | 138.3 | 36.8 | +99.9 |
| final_fun bin | 1328 | 22.2 | 37.0 | −0.7 |

**~16× blurrier than RoboSub and 39× blurrier than the clearest set we own.**
This is by a wide margin the hardest water in the archive.

### Retraction 1 — cross-water transfer is FINE, and §5's worry was wrong

Same prop, RoboSub-trained model, RoboSub water vs Mirpur water:

| model | RoboSub | Mirpur |
|---|---|---|
| `torpedo_n_shark-up_200_final2` | 97.5 % | **93.3 %** |
| `torpedo_n_200_shark-up_v1` | 97.5 % | **93.3 %** |
| `torpedo_mini_final_day_1` | 96.7 % | 75.0 % |

**Four points lost across a 15× sharpness change, a different pool, a
different continent and a different season.** The models generalise. §5 ranked
"validate on a different capture session" first partly out of fear that they
did not; the fear was unfounded and the ranking should not rest on it. What §6
showed is the real problem: **the spread between two models of the same prop
(29 % vs 73 %) is far larger than the spread between two waters for one model
(97 % vs 93 %).** Which model you pick matters more than which pool you are in.

### Retraction 2 — CLAHE, and the heuristic built on it

Mirpur is precisely the water `underwater.recommend()` was built to catch:
blurry, saturated, heavily cast. It said `CLAHE ON`. CLAHE measurably does not
help there — and re-measuring properly showed it does not help anywhere.

Seventeen configurations, four props, three venues, five independent frame
samples of the original clip: **never meaningfully positive, and on the gate it
destroys 95 % of the detections** (30.4 % → 1.2 %). Full table and the
confounded-A/B that produced the original number:
[`detection-continuity.md`](detection-continuity.md) §4.

Consequences, all landed:

* `murky` profile no longer enables preprocessing (it keeps `conf 0.10` and
  the crop, which are unaffected and re-measured).
* `underwater.recommend()` and its four thresholds are **deleted**, not
  re-fitted. A rule fitted to two clips was wrong on the third; fitting it to
  three would be the same mistake with better manners. `WaterStats` /
  `analyse_frames` survive — the characterisation reproduced across all three
  venues and is a fact about the pool.
* A guard test asserts no profile turns preprocessing on, and asserts
  `recommend()` stays gone. Both verified to bite.

### The rule that replaces it

The models were trained on **unprocessed** frames. Any preprocessing that makes
an image look better **to a person** moves it away from the distribution the
detector learned. The prettier the result, the further it has moved. This is
why the answer to motion blur is exposure control and training-time
augmentation — changing the image the *camera* forms, or the images the *model*
learns — and not a filter placed between them.

---

## 8. The confidence floor, measured against false positives at last

`conf 0.15 → 0.10` shipped on **+8.5 points of presence**, with jitter flat.
Presence counts frames containing a box — **it cannot tell a right box from a
wrong one**. So the single cost of lowering a confidence floor is the one
quantity that decision never measured, on a vehicle where a false gate
detection steers the hull at a wall.

`tools/recall_matrix.py` has computed tp/fp/fn the whole time. It was never
pointed at this question. `--sweep` now does, from **one inference pass**
re-thresholded offline (verified equal to N independent passes on a pair where
the threshold actually moves the result — the first attempt agreed at every
threshold because every score was above 0.5, which proves nothing).

### What 0.15 → 0.10 actually buys and costs

| pair (held-out unless noted) | Δ recall | Δ precision | F1 knee |
|---|---|---|---|
| torpedo, `Torpedo_Down` | +0.0 | +0.0 | 0.50 |
| torpedo, `torpedo_v1` | +0.0 | +0.0 | 0.25 |
| bin, `bin-1` | +0.0 | +0.0 | 0.30 |
| bin, `Final_Bin` | +0.0 | **−1.5** | 0.50 |
| octagon, `oct_zawad` | +2.1 | **−3.2** | 0.25 |
| octagon, `slot-1_obb` | +3.0 | **−2.8** | 0.25 |
| **gate, Mirpur (cross-venue)** | +1.8 | **−8.3** | 0.05 |

**In four of seven pairs it changes nothing at all** — the scores are already
above 0.5. Where it does move, it buys 2–3 points of recall for 3 points of
precision, and on the hardest case it costs **8.3 points of precision for
1.8 of recall**.

**The +8.5 points of presence do not appear as recall.** That is the whole
lesson: presence rose because more boxes appeared, and on labelled data most of
the new ones are wrong.

### What the knee says, and the one place it must not be followed blindly

The F1 knee sits at **0.25–0.50 in five of seven pairs** — *higher* than either
shipped value. Only the cross-venue gate wants 0.05, and that is the case where
recall has collapsed to 26 % and any box is better than none.

F1 weights a miss and a false positive equally, and for this vehicle they are
not equal. A miss stalls the control loop — the user's own scenario: gate ahead,
no detections, thrusters still, clock running. A false positive is partly
filtered downstream by `lock_on` continuity, `ctrl_conf` and the
distinct-frame gate. So the operating point should sit **below** the F1 knee,
not on it.

**The defensible reading: 0.10 is fine and 0.15 is fine; the +8.5-presence
justification for preferring 0.10 is not.** Both sit below every knee, and the
difference between them is inside the noise for four of seven pairs. What is
*not* defensible is going lower on presence evidence.

### A model can be near-zero on its own task — camera view is not in the name

Found while checking a 2.6 % recall that looked like a parser bug and was not:

| model | `bin-1` (downward) | `Final_Bin` |
|---|---|---|
| `robosub_bin_front_n_300_v1` | **2.6 %** | **0.0 %** |
| `robosub_bin_200_v1` | **100 %** | **100 %** |
| `robosub_bin_final_day_1` | 100 % | 100 % |

A **forward**-camera bin model on **downward** bin imagery. Both report
mAP50 = 0.995 (§6), so validation cannot separate them, and the name says
"bin" in both cases. This is §6's finding with a sharper edge: picking the
wrong one for a downward bin drop gives **2.6 %**, and nothing in the artefacts
warns you.

### The gap this opened

**Slalom has 2,102 labelled images and no model at all.** No `salom`/`slalom`
run exists in the archive, while `task_slalom.py` expects a `red_pipe` class.
It is a scoring task with training data sitting ready and nothing trained on
it. That is the largest single actionable gap the archive has surfaced.
