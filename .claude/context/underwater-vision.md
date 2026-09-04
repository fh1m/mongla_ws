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

### 3a. A validation score cannot predict a venue

All 25 archived training runs report **mAP50 = 0.995**. Real recall, against
ground truth, per dataset:

| model → dataset | bin | octagon | torpedo | gate |
|---|---|---|---|---|
| bin | **100.0 %** | 1.4 % | 15.9 % | 2.0 % |
| octagon | 0.0 % | **88.4 %** | 10.2 % | 23.8 % |
| torpedo | 8.7 % | 1.8 % | **81.4 %** | 2.0 % |
| gate | 3.3 % | 0.7 % | 0.0 % | **24.8 %** |

Diagonal 81–100 %, **everything off-diagonal 0–24 %**. These models learned
their dataset, not their object. A held-out split drawn from the same water,
the same day, the same camera measures memorisation as generalisation.

**The generalisable rule: the only honest validation is a different capture
session.** Ideally a different venue.

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
