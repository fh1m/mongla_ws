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
