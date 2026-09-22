# SOTA 2026 — deployable edge object detector with minimal human labelling

Research pass for Mongla (Raspberry Pi 5 + Hailo-8, `.hef`, INT8, 640×640, CNN-only).
Every claim below carries a source link and a number. Unverified claims are quarantined
in the last section.

Status: **COMPLETE**.

---

## 1. SAM 3 (Meta) — promptable concept segmentation

**What it is.** One model that detects, segments and tracks from a *concept prompt* —
a short noun phrase ("yellow school bus"), an image exemplar, or both — and returns masks
plus identities for **all** matching instances, not one object per prompt
([SAM 3: Segment Anything with Concepts, arXiv 2511.16719](https://arxiv.org/abs/2511.16719)).

**Measured numbers** ([Ultralytics SAM 3 docs](https://docs.ultralytics.com/models/sam-3/)):

| Metric | SAM 3 | Prior best | Δ |
|---|---|---|---|
| SA-Co benchmark CGF1 | **65.0** | 34.3 (OWLv2) | +89.5 % |
| LVIS zero-shot **mask AP** | **47.0** | 38.5 | +22.1 % |
| LVIS zero-shot box AP | 53.5 | — | — |
| MOSEv2 J&F (video) | 60.1 | 47.9 (SAM 2.1 L) | +25.5 % |
| DAVIS 2017 J&F | 92.0 | — | — |

**Cost to run.** 473.6 M parameters, **3.45 GB** on disk; Ultralytics measures
**2921 ms/image on an H200** in their harness, while Meta quotes **30 ms per image with
100+ detected objects** on an H200 — the two are not the same measurement and the
discrepancy matters for any cost estimate (see §"claims I could not verify").
For scale: SAM 2-b is 162 MB / 80.8 M params / 857 ms per image in the same harness.

**Prompt refinement is the strongest lever for a new prop.** From the same doc, on SA-Co:

- text prompt only: **46.4 CGF1**
- text + **1 image exemplar**: **57.6 CGF1** (+11.2)
- text + **3 image exemplars**: **65.0 CGF1** (+18.6)

That is the single most relevant number in this whole report for a *novel* object like a
torpedo board: three clicked exemplars buy +18.6 CGF1 over a text phrase alone.

**Training scale behind it** (why it generalises to odd props): 5.2 M annotated images with
**4 M unique noun phrases**, 1.4 B synthetic masks across 38 M phrases, 52.5 K videos with
24.8 K unique concepts; the SA-Co benchmark itself holds **214 K unique concepts — 50× LVIS**.
SAM 3 reaches **88 % of the estimated human lower bound** on SA-Co/Gold.

---

## 2. DINOv3

**Family and sizes** ([Yang & Hostens, arXiv 2604.27128](https://arxiv.org/pdf/2604.27128)):
the distilled **ViT-S/16 is 21.6 M parameters with 384-dim embeddings** — the smallest public
DINOv3 — against **ViT-7B at 6716 M params / 4096-dim**: a **311× parameter ratio** and
**10.67× embedding-dimension ratio**. On disk ViT-S/16 is **82.4 MB fp32 / 41.2 MB fp16**
versus ≈**25 GB** for ViT-7B fp32. Weights:
[facebook/dinov3-vits16-pretrain-lvd1689m](https://huggingface.co/facebook/dinov3-vits16-pretrain-lvd1689m).

**Measured speed** (same paper): single-image ViT-S/16 inference **7.99 ms on an NVIDIA A10**
(fp16, 224×224); batched throughput bottoms at **0.42 ms/image at batch 16 = 2381 images/s**,
flat through batch 32 and 64. Distillation is the standard DINOv3 recipe with a **frozen
ViT-7B teacher**.

**Measured finding relevant to us**: the paper establishes empirically that the *pre-distilled*
ViT-S/16 is "a sufficient per-individual embedder" — i.e. you do not need to re-distill the
embedder yourself; the 21.6 M off-the-shelf checkpoint is enough for identity work.

**Measured label-efficiency with frozen DINOv3 features — and it is underwater.**
[*Label-efficient underwater species classification with logistic regression on frozen
foundation model embeddings*, arXiv 2604.00313](https://arxiv.org/abs/2604.00313):
DINOv3 **ViT-B/16** embeddings, backbone **frozen**, only a logistic regression fitted, on
the AQUA20 benchmark.

| Labels per category | Macro F1 |
|---|---|
| **13** (≈**4 %** of the official training partition) | **81.8 %** |
| **144** | **88.5 %** |
| all official training labels | **91.5 %** (bootstrap 95 % CI **89.0–93.7 %**) |
| *fully supervised ConvNeXt, complete training set* | *88.9 %* |

**13 labelled images per class buys 81.8 % macro F1; 144 matches a fully supervised ConvNeXt
(88.5 vs 88.9).** The authors note this runs on commodity hardware with no task-specific
neural-network training, and that the label-efficiency *curve shape* reproduced on a second
dataset. ⚠ This is **classification, not detection** — it tells you how cheaply a frozen
foundation feature separates underwater *categories*, not how cheaply it localises them.

---

## 3. Open-vocabulary detectors as auto-labellers

### 3a. YOLOE — [Real-Time Seeing Anything, ICCV 2025, arXiv 2503.07465](https://arxiv.org/abs/2503.07465) · [repo](https://github.com/THU-MIG/yoloe)

Zero-shot **LVIS minival**, text prompt (params / FPS on T4 / training cost):

| Model | AP | AP_r | AP_c | AP_f | Params | FPS (T4) | Train |
|---|---|---|---|---|---|---|---|
| YOLOE-v8-S | 27.9 | 22.3 | 27.8 | 29.0 | 12 M | **305.8** | 12.0 h |
| YOLOE-v8-M | 32.6 | 26.9 | 31.9 | 34.4 | 27 M | 156.7 | 17.0 h |
| YOLOE-v8-L | 35.9 | 33.2 | 34.8 | 37.3 | 45 M | 102.5 | 22.5 h |
| YOLOE-11-S | 27.5 | 21.4 | 26.8 | 29.3 | 10 M | 301.2 | 13.0 h |
| YOLOE-11-L | 35.2 | 29.1 | 35.0 | 36.5 | 26 M | 130.5 | 23.5 h |

**Visual prompt** (the mode that matters for a prop with no name in any vocabulary):

| Model | AP | AP_r | Params | FPS (iPhone 12) |
|---|---|---|---|---|
| YOLOE-v8-S | 26.2 | 21.3 | 13 M | 64.3 |
| YOLOE-v8-L | 34.2 | 33.2 | 50 M | 27.2 |

**Prompt-free**: YOLOE-v8-S **21.0 AP** @ 95.8 FPS; YOLOE-v8-L **27.2 AP** @ 25.3 FPS.
**COCO transfer, full tuning**: YOLOE-v8-L **53.0 AP^b / 42.7 AP^m**; YOLOE-11-L 52.6 / 42.4.

Headline vs the prior open-vocab YOLO: **YOLOE-v8-S beats YOLO-Worldv2-S by +3.5 AP with
3× less training cost and 1.4× inference speedup**; YOLOE-v8-L hits **23.5 AP^m** zero-shot
segmentation vs **19.8 AP^m** for a *fine-tuned* YOLO-Worldv2-L.

⚠ Note the whole table sits **below 36 AP**. The best open-vocab YOLO is a ~35-AP labeller.
That is the ceiling on what a text prompt alone can hand a student.

### 3b. Grounding DINO family — [arXiv 2303.05499](https://arxiv.org/pdf/2303.05499) · [Grounding DINO 1.5, arXiv 2405.10300](https://arxiv.org/abs/2405.10300v2)

| Model | COCO zero-shot AP | LVIS-minival zero-shot AP | Speed |
|---|---|---|---|
| Grounding DINO (orig.) † | **52.5** | — | — |
| Grounding DINO 1.5 **Pro** † | **54.3** | **55.7** | — |
| Grounding DINO 1.5 **Edge** † | **45.0** | **36.2** | **75.2 FPS** (TensorRT) |

Grounding DINO 1.5 Pro at **55.7 LVIS AP** is the strongest *box* labeller found in this
pass — ~20 AP above the best YOLOE. Its cost is the 38 min/VOC-dataset figure Voxel51
measured for Grounding DINO (§4a), i.e. **12.7× slower than YOLO-World** for the same job.

⛔ Grounding DINO is a **DETR-family transformer**. Fine as a teacher; it cannot be the
deployed student on Hailo-8.

### 3c. OWLv2 / OWL-ST — [Scaling Open-Vocabulary Object Detection, arXiv 2306.09683](https://arxiv.org/abs/2306.09683)

The self-training result that is the whole argument for pseudo-labelling: using an existing
detector to generate pseudo-boxes on image–text pairs, **OWL-ST at L/14 raises LVIS *rare*-class
AP from 31.2 → 44.6 (+43 % relative) for classes the model has seen zero human box
annotations for** †, scaling to **over 1 B examples**. OWLv2 beats the prior SOTA open-vocab
detectors already at **~10 M examples**.

This is the strongest published evidence that pseudo-labels alone move the rare-class needle —
and rare-class is our regime.

⛔ OWLv2 is a ViT. Teacher only.

### 3d. YOLOE-26 / YOLO26 auto-labelling workflow — *verified, but thin*

[Ultralytics YOLO26 docs](https://docs.ultralytics.com/models/yolo26) ·
[PyImageSearch: Train YOLO26 on a Custom Dataset with YOLOE-26 Auto-Labeling (2026-08-31)](https://pyimagesearch.com/2026/08/31/train-yolo26-on-a-custom-dataset-with-yoloe-26-auto-labeling/)

The workflow is real: **YOLOE-26** is the open-vocabulary, language-conditioned extension of
YOLO26; you prompt it in free text (or with visual reference boxes), export YOLO-format boxes,
and fine-tune a plain YOLO26 on them. Every published number from the walkthrough:

| Quantity | Value |
|---|---|
| Training images | **41** |
| Training boxes | **50** |
| Validation images / boxes | **9** / **14** |
| Classes | **2** |
| Reference (visual-prompt) images per class | **4–7** |
| Pseudo-label confidence threshold | **0.35** |
| Training images needing **human correction** | **2 of 41** (**4.9 %**) |
| Epochs run | 15, early-stopped with `patience=8`; best at epoch **7** |
| imgsz / batch / device | **960** / 8 / Apple MPS |
| Checkpoint | `yolo26s.pt` |

⛔ The walkthrough reports mAP50 and mAP50-95 only *qualitatively* ("strong numbers for a
small 2-class pilot") and gives **no auto-labelling wall-clock and no manual-labelling
comparison**. So the one genuinely transferable datum is **4–7 exemplars per class and
2/41 images needing a human fix**, which is a ~95 % hands-off rate at 41 images — far too
small a sample to carry weight, and on a pair of easy rigid boxes (perfume bottles), not
a submerged flare.

Also relevant for a human-in-the-loop path:
[Label Studio + YOLO26 pre-annotations](https://labelstud.io/blog/use-yolo26-with-label-studio-for-fast-bounding-box-pre-annotations/)
— setup quoted at **10 minutes**; annotators review and correct rather than draw. And
Ultralytics ships [`auto_annotate`](https://docs.ultralytics.com/usage/simple-utilities),
which pairs a trained YOLO detector with SAM to emit segmentation labels.


## 4. End-to-end auto-annotation pipelines — **the human-minutes and auto-vs-hand mAP numbers**

### 4a. Zero-shot auto-labeling (Voxel51) — the best number in this report

[*Auto-Labeling Data for Object Detection*, arXiv 2506.02359](https://arxiv.org/abs/2506.02359) ·
[Voxel51 write-up: "Verified Auto Labeling reduces annotation costs by 100,000x"](https://voxel51.com/blog/zero-shot-auto-labeling-rivals-human-performance)

**Auto-labelled vs hand-labelled, same downstream detector:**

| Dataset | mAP50, trained on **auto-labels** | mAP50, trained on **human labels** | Gap |
|---|---|---|---|
| VOC | **0.768** | 0.817 | **−0.049** |
| COCO | **0.538** | 0.588 | **−0.050** |
| LVIS | **< 0.10** | — | pipeline fails |

Auto-labeller F1 against ground truth: **VOC 0.785**, **COCO ≈0.640**, **LVIS 0.215**.
Voxel51 frame this as **90–95 % of human performance** on practical (non-long-tail) scenarios.

**The cost side, for 3.4 M labelled objects:**

| | Auto-label | Human (AWS SageMaker Ground Truth) |
|---|---|---|
| Wall-clock | **~1.27 h** (one NVIDIA L40S) | **~6 703 h** (≈7 000) |
| Dollars | **$1.18** | **$124 092** |

= **≈5 000× faster, ≈100 000× cheaper.** Labeller wall-clock on VOC:
**YOLO-World ≈3 min**, **Grounding DINO ≈38 min** — a **12.7×** speed spread between
auto-labellers for the same dataset.

**Read this carefully for our case.** Two things transfer and one does not:
- Transfers: a **~5 mAP50-point** penalty for going fully hands-off on *common* classes.
- Transfers: the labeller choice is a 12.7× compute decision, not an accuracy decision alone.
- Does **not** transfer: LVIS (**mAP < 0.10, F1 0.215**) is the long-tail regime. A "torpedo
  board" or "flare" is a long-tail concept with no COCO/VOC analogue — the LVIS row is the
  honest prior for a novel competition prop under a *text-only* prompt, which is exactly
  why SAM 3's **+18.6 CGF1 from 3 image exemplars** (§1) is the relevant escape hatch.

### 4b. SAM2Auto (arXiv 2506.07850) — verified, and it does **not** have the number

[SAM2Auto: Auto Annotation Using FLASH](https://arxiv.org/abs/2506.07850) ·
[HTML v1](https://arxiv.org/html/2506.07850v1). Fully automated video-dataset annotation,
no human intervention, no dataset-specific training. Components: **YOLO-World** (open-vocab
detection) + **SAM2** (mask generation) + **SAHI** (refinement) + **ByteTrack** association,
under SMART-OD and FLASH.

Reported numbers:

| Benchmark | Result |
|---|---|
| SMART-OD detection, MOT17 | precision **0.728**, recall **0.224**, MOTA **−0.014** |
| Full SAM2Auto, MOT17 | MOTA **−0.014 → 0.181** (paper calls it a 1 393 % increase) |
| FLASH tracking, MOT17 private | HOTA **43.4**, MOTA **28.7**, IDF1 **56.5** |
| MOT20 private | HOTA **38.6**, MOTA **22.3**, IDF1 **50.1** |
| DanceTrack | HOTA **62.0**, MOTA **64.1**, IDF1 **72.5** |
| BDD100K | mHOTA **53.7**, mIDF1 **47.8** |

⛔ **The paper claims "comparable accuracy to manual annotation while dramatically reducing
annotation time" in its abstract but publishes no annotation-time, cost, or head-to-head
manual-baseline measurement.** And **recall 0.224** on MOT17 is the number that should stop
us: for a system whose only predictive metric is cross-session *recall*, an auto-labeller
that finds 22.4 % of objects is a labeller that silently deletes three quarters of the truth.
Treat SAM2Auto as a tracking-identity contribution, not as an annotation-cost result.

### 4c. Autodistill (Roboflow)

[Autodistill docs](https://docs.autodistill.com/quickstart/) ·
[Roboflow: Distill Large Vision Models into Smaller, Efficient Models](https://blog.roboflow.com/autodistill/) ·
[Use Grounded SAM to train a YOLOv8 model](https://roboflow.com/train/grounded-sam-and-yolov8)

Structure is exactly the shape we want: **base models** (GroundedSAM, OWL-ViT, DETIC) →
ontology (plain-text prompt → class) → **target models** (YOLOv8, YOLO-NAS, YOLOv5) — all
three target families are CNN detectors, so all three are Hailo-compilable in principle.
Numbers: **none published** — the Roboflow blog's worked example (milk bottles, GroundedSAM -> YOLOv8, `epochs=50`) shows confusion matrices and sample predictions but **no mAP, no labelling time, no cost comparison**. See "claims I could NOT verify".


## 5. Distillation foundation-model → small model (method template)

**[Lightweight Distillation of SAM 3 and DINOv3 for Edge-Deployable Individual-Level
Livestock Monitoring, arXiv 2604.27128](https://arxiv.org/pdf/2604.27128)** (Yang & Hostens,
IEEE Access) is the cleanest template paper. What it actually did:

- Teacher: SAM 3's **446 M-parameter Perception Encoder (PE-ViT-L+)** backbone.
- Student: **40.66 M-parameter** multi-scale FPN encoder on a **TinyViT-21M-512** backbone,
  fusing three stages rather than projecting one.
- Loss: four-term **direction-then-scale** objective, weights **1.0 / 0.5 / 0.3 / 0.1**
  (L2-normalised feature MSE, cosine, moment-matching on µ/σ, raw MSE). Weights chosen on
  pilot runs that showed a **collapsed-variance failure mode** when the scale term was given
  equal weight to the directional terms.
- Inference trick: **backbone substitution** — the distilled encoder is dropped into the
  teacher's decoder/memory stack rather than retraining the whole system.

**Retention, on the Edinburgh Pig dataset:**

| | Teacher (SAM 3) | Student | Ratio |
|---|---|---|---|
| MOTA | — | **92.29 %** | −1.68 pp vs teacher |
| IDF1 | — | **96.15 %** | −0.84 pp vs teacher |
| Encoder params | 446 M | 40.66 M | **10.98×** |
| System params | — | — | **7.77×** |
| Peak streaming VRAM | 19.52 GB | 6.49 GB | **3.01×** (0.33×) |
| Per-frame latency | 407.70 ms | 309.84 ms | **−24.0 %** |
| Throughput | 1.08 FPS | 1.18 FPS | 1.09× |
| 9-class behaviour top-1 | — | **97.34 %** (macro-F1 91.67 %) | — |

**The headline retention number: ~1.7 pp of MOTA lost for 7.77× fewer parameters.**

⚠ Two honest caveats the paper itself states: (a) deployment validation is **analytic, not
measured** — the Orin NX 16 GB budget (11.1 GB used, **4.9 GB headroom**) is computed from
A10 measurements, and on-device benchmarking is listed as follow-up work; (b) the ~1 FPS
throughput on an A10 is an artefact of SAM 3's stateful streaming session, and the paper
notes Orin NX has ≈**1/5 the memory bandwidth and 1/10 the FLOPs** of an A10.

⚠ Also: this is a **tracking + embedding** distillation, not a detector auto-labelling
pipeline, and the student is a ViT — it would **not** compile for Hailo-8. It is a method
template for the *loss and the substitution trick*, not a recipe to copy.

---

## 6. Active learning / uncertainty sampling — measured labelling-effort reduction

The best-measured result for our situation, because it is a **single-class, domain-shifted,
robot-collected** dataset rather than COCO:

**[MaskAL: Active learning with MaskAL reduces annotation effort for training Mask R-CNN,
arXiv 2112.06586](https://arxiv.org/pdf/2112.06586)** (Blok et al.) — 16 000 field broccoli
images, **31 042 broccoli heads**, 26 fields in 4 countries, training pool 14 000, val 500,
**three separate 500-image test sets** (i.e. they evaluated cross-session, like us):

| Result | Number |
|---|---|
| MaskAL vs full training set (14 000 imgs) | **93.9 % of full performance with 17.9 % of the data** |
| Random sampling, same budget | **81.9 % of full performance with 16.4 % of the data** |
| Equivalence point | **MaskAL at 900 images = random sampling at 2 300 images** |

**That equivalence point is the number to quote: 2.56× fewer images annotated for the same
result, purely by choosing which frames a human looks at.** And the AL-vs-random gap
(93.9 % vs 81.9 % at essentially the same budget) is **12 percentage points** — larger than
anything quantisation or architecture choice buys.

Prior work cited in the same paper: **50 % of annotation time saved** by active learning for
object detection in cereal crops; **60 % of images needed** for comparable classification
performance on crops and weeds.

Other measured points:

| Source | Result |
|---|---|
| [Efficient annotation reduction with active learning for retail product recognition](https://link.springer.com/article/10.1007/s42001-024-00266-7) † | **20.83–24.34 % of the data reaches 95 % of full-dataset performance** |
| [Box-Level Active Detection (BiB), arXiv 2303.13089](https://arxiv.org/pdf/2303.13089) † | **97 % of fully-supervised Fast R-CNN with 10 % of fully-annotated images** on VOC07 |
| [Uncertainty Meets Diversity, CVPR 2025](https://openaccess.thecvf.com/content/CVPR2025/papers/Wang_Uncertainty_Meets_Diversity_A_Comprehensive_Active_Learning_Framework_for_Indoor_CVPR_2025_paper.pdf) | ≈**85 % of fully-labelled performance**, surpassing prior AL approaches (indoor 3D detection; I could not fetch the paper's own budget table — 403) |

**Why this matters more here than anywhere else in the report.** Our own datum — three models
at identical mAP50 **0.9950** scoring **29.2 % / 72.7 % / 68.3 %** cross-session recall —
says the training-set *composition* dominates, not the training recipe. Active learning is
precisely a method for choosing composition. MaskAL's 12-point AL-vs-random gap is the
closest published analogue to that 43-point spread.

## 7. Semi-/self-supervised detection — measured mAP at 1 % / 5 % / 10 % labels

**COCO-standard, Faster R-CNN**, from
[MixPL: Mixed Pseudo Labels for Semi-Supervised Object Detection, arXiv 2312.07006](https://arxiv.org/html/2312.07006v1):

| Method | 1 % | 5 % | 10 % | 100 % |
|---|---|---|---|---|
| Supervised baseline | 10.70 | 21.00 | 26.60 | **41.00** |
| Unbiased Teacher | 20.75 ±0.12 | 28.27 ±0.11 | 31.50 ±0.10 | 41.30 |
| Soft Teacher | 20.46 ±0.39 | 30.74 ±0.08 | 34.04 ±0.14 | 44.50 |
| ACRST | 26.07 ±0.46 | 31.35 ±0.13 | 34.92 ±0.22 | 42.79 |
| **MixPL** | **25.06 ±0.39** | **33.48 ±0.13** | **37.16 ±0.15** | **46.20** |

**The load-bearing numbers:**

- At **10 % of labels**, MixPL reaches **37.16 mAP = 90.6 % of the 100 %-supervised 41.00**.
  Teacher-student pseudo-labelling recovers **+10.56 mAP** over the 26.60 supervised
  baseline at the same label budget.
- At **1 % of labels**, MixPL's **25.06** *beats* the supervised baseline trained on **10 %**
  (26.60 — marginally below) and crushes the 1 % baseline's 10.70: **+14.36 mAP**.
- Even at **100 % labels**, unlabelled data still adds **+5.2 AP** (41.00 → 46.20).
  Pseudo-labelling is not only a low-label trick.

Corroborating († both of the following — snippet only, primary not fetched):
[LabelMatch, arXiv 2206.06608](https://arxiv.org/pdf/2206.06608) reports
**25.81 / 32.70 / 35.49 mAP at 1 / 5 / 10 %** vs Unbiased Teacher's 20.75 / 28.27 / 31.50 —
and notes that its 1 %-label result (25.81) exceeds the supervised baseline trained on 10 %.
[PL-DC](https://arxiv.org/html/2505.11075v1) reports gains of **+11.6 / +9.2 / +7.0 / +6.0
mAP at 1 / 2 / 5 / 10 %**.

⚠ Every number here is Faster R-CNN on COCO with an i.i.d. split. None of these papers
report cross-session or cross-venue recall, which is the only metric that has predicted
deployment for us. Treat the *shape* (10 % of labels → ~90 % of full mAP) as transferable
and the absolute values as not.


## 8. Hailo-8 specifics — the arbitrating source for "does it compile"

### 8a. Model Zoo, measured on Hailo-8 (COCO mAP, batch 1)

[hailo_model_zoo — HAILO8 object detection](https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8/HAILO8_object_detection.rst).
Every row is float mAP → **hardware (INT8) mAP** → FPS, at the listed input size.

| Model | Input | Float mAP | **HW mAP** | Quant loss | **FPS** |
|---|---|---|---|---|---|
| yolov8n | 640 | 37.0 | **36.4** | −0.6 | **1036** |
| yolov6n | 640 | 34.3 | 32.4 | −1.9 | **1250** |
| yolov10n | 640 | 38.5 | 36.6 | −1.9 | 194 |
| **yolov11n** | 640 | **39.0** | **37.5** | **−1.5** | **185** |
| **yolo26n** | 640 | **40.0** | **38.4** | **−1.6** | **155** |
| yolov12n | 640 | 40.5 | 39.2 | −1.3 | 46.2 |
| yolov8s | 640 | 44.6 | 43.9 | −0.7 | 491 |
| yolov11s | 640 | 46.3 | 45.1 | −1.2 | 111 |
| yolo26s | 640 | 47.5 | 45.3 | −2.2 | 97.8 |
| yolov10s | 640 | 45.9 | 44.9 | −1.0 | 117 |
| yolov8m | 640 | 49.9 | 49.2 | −0.7 | 66.9 |
| yolov11m | 640 | 51.1 | 49.9 | −1.2 | 50.2 |
| yolo26m | 640 | 52.3 | 50.6 | −1.7 | 48.7 |
| yolov11x | 640 | 54.1 | **53.1** | −1.0 | 18.3 |
| damoyolo_tinynasL20_T | 640 | 42.8 | 42.2 | −0.6 | **564** |
| nanodet_repvgg | 416 | 29.3 | 28.6 | −0.7 | **1261** |
| yolox_s_leaky | 640 | 38.1 | 37.3 | −0.8 | 385 |
| **detr_resnet_v1_18_bn** | **800** | 33.9 | **31.5** | −2.4 | **30.5** |

**Three conclusions from this one table:**

1. **INT8 quantisation costs 0.6–2.4 mAP points** across the whole detector family — a
   *smaller* penalty than the ~5-point auto-labelling penalty in §4a. Quantisation is not
   where the accuracy goes.
2. **Headroom is enormous at our operating point.** We measure 98.0 Hz through the pipeline;
   yolov8n runs at **1036 FPS device-only**, yolov11n at 185, yolo26n at 155, yolov11x at
   **18.3**. Even the x-model at 53.1 HW mAP would sustain ~18 Hz device-only. The
   accuracy/speed trade is far from exhausted.
3. ⚠ **Correction to the brief.** A transformer detector *is* in the Hailo-8 zoo —
   `detr_resnet_v1_18_bn`, 800×800, **31.5 HW mAP at 30.5 FPS**. So "transformers don't
   compile" is too strong as stated: *DETR-ResNet18 compiles and is benchmarked*. What I
   found **no** evidence for is **RT-DETR** on Hailo-8 — it is absent from the zoo list. The
   accurate statement is "RT-DETR is not in the Hailo-8 Model Zoo," not "transformers cannot
   compile." This does not change the recommendation (DETR-R18 at 31.5 mAP / 30.5 FPS is
   strictly worse than yolov11n at 37.5 / 185), but it changes the *reason*.

### 8b. `format="hailo"` — the 6-step manual DFC compile is obsolete

[Ultralytics Hailo export docs](https://docs.ultralytics.com/integrations/hailo/) ·
[Hailo Community announcement](https://community.hailo.ai/t/export-ultralytics-yolo-models-straight-to-hailo-format-hailo-is-now-available/19678)

Shipped in **ultralytics 8.4.97**: `model.export(format="hailo")` emits a `.hef` directly.

- Targets: **Hailo-8, 8L, 10H, 15H, 15L**. Compilation host must be **Linux x86_64**;
  HEFs deploy to ARM hosts including **Raspberry Pi with AI HAT+**.
- Covers **YOLOv8, YOLO11 and YOLO26** detection, **including custom detection models**.
- It auto-generates parser settings, calibration data, HailoRT post-processing, metadata
  and the final `.hef` — "replaces the previous lengthy manual ONNX-to-DFC workflow."
- **INT8 only**; 16-bit activations (`a16`) applied where a task needs the range.
- NMS: YOLOv8/YOLO11 use **HailoRT NMS**; YOLO26 keeps its **NMS-free** output path.
- Args: `imgsz` (default **640**), `fraction`, `name` (default `hailo8l`),
  `conf` **0.25**, `iou` **0.7**.

**Calibration: "use at least 1 024 calibration images for best accuracy."** With no dataset
supplied it silently falls back to **COCO128** — 128 out-of-domain images. For an underwater
prop that fallback is the exact silent-failure shape this codebase keeps finding.

**Measured INT8 mAP50 retention, Hailo-8L, with in-domain calibration:**

| Model | mAP50 retained |
|---|---|
| YOLOv8n | **~100 %** |
| YOLO11n | **~96 %** |
| YOLO26n | **~93 %** |
| YOLOv8n-seg | 98.0 % |
| YOLOv8n-pose | 98.1 % |
| YOLOv8n-cls | 92.6 % |

Note the ordering: the **newest** detector loses the **most** to INT8 (YOLO26n 93 % vs
YOLOv8n ~100 %). Newer is not automatically better after quantisation.

### 8c. Quantisation-aware training in the DFC — it exists, under another name

[Hailo: Quantization-Aware Fine-Tuning](https://hailo.ai/de/developer-zone/training-and-support/hailo-quantization-aware-fine-tuning/) ·
[Hailo Quantization Scheme](https://hailo.ai/de/developer-zone/training-and-support/hailo-quantization-scheme-2/) ·
[Model Zoo OPTIMIZATION.rst](https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/OPTIMIZATION.rst)

The DFC supports **4/8/16-bit int weights and 8/16-bit uint activations**, and Hailo ships
**QFT (Quantization-aware Fine-Tuning)** — additional training on the quantised graph with a
quantisation-aware loss that pushes weights onto the grid. For 4-bit layers, naïve calibration
is poor even after finetuning, so Hailo clips outlier weights before finetuning.

⚠ Hailo publishes the *method* but I found **no quantitative "QFT recovers X mAP points"
figure** in their docs. And `format="hailo"` does **not** expose QAT — it is
post-training-quantisation with calibration only. QAT is a DFC-level escape hatch, not a
one-liner.


---

---

## TABLE — pipeline vs labels vs accuracy vs teacher compute vs Hailo-8

Rows ordered by human effort, least first. **Read the "metric type" column before the
number**: a row measured on an i.i.d. split is not comparable to one measured across sessions.

| Pipeline | Human labels needed | Resulting mAP / recall | Metric type | Teacher compute | Student compiles for Hailo-8? |
|---|---|---|---|---|---|
| **Zero-shot auto-label → train CNN** ([arXiv 2506.02359](https://arxiv.org/abs/2506.02359) / [Voxel51](https://voxel51.com/blog/zero-shot-auto-labeling-rivals-human-performance)) | **0** | VOC mAP50 **0.768** (vs 0.817 hand); COCO **0.538** (vs 0.588); **LVIS < 0.10** | i.i.d. split | **1.27 h on 1× L40S = $1.18** for 3.4 M objects; YOLO-World ≈3 min vs Grounding DINO ≈38 min per VOC | **Yes** — student is a plain CNN detector |
| **Autodistill: GroundedSAM / OWL-ViT / DETIC → YOLOv8 / v5 / NAS** ([docs](https://docs.autodistill.com/quickstart/)) | **0** (text ontology only) | **no published mAP** | — | not published | **Yes** — all three target families are CNN |
| **YOLOE-26 visual-prompt auto-label → fine-tune YOLO26** ([PyImageSearch 2026-08-31](https://pyimagesearch.com/2026/08/31/train-yolo26-on-a-custom-dataset-with-yoloe-26-auto-labeling/)) | **4–7 exemplar images/class**, then **2 of 41** images corrected (**4.9 %**) | mAP reported **qualitatively only** | — | not published; YOLOE-v8-S runs **305.8 FPS on T4** | **Yes** — `yolo26n/s` are in the Hailo-8 zoo (**38.4 / 45.3 HW mAP**) |
| **SAM 3 concept prompt + 3 exemplars as labeller** ([arXiv 2511.16719](https://arxiv.org/abs/2511.16719)) | **3 exemplar boxes/class** | **65.0 CGF1** (vs 46.4 text-only); LVIS zero-shot **47.0 mask AP / 53.5 box AP** | zero-shot open-vocab bench | 473.6 M params, 3.45 GB; **2921 ms/img** (Ultralytics harness, H200) vs **30 ms/img @100+ objects** (Meta) | teacher only — student must be a separate CNN |
| **SAM2Auto** ([arXiv 2506.07850](https://arxiv.org/abs/2506.07850)) | **0** | MOT17 detection **recall 0.224**, precision 0.728; pipeline MOTA **0.181** | video tracking | YOLO-World + SAM2 + SAHI + ByteTrack; **not published** | n/a — annotator, not a student |
| **OWL-ST self-training** † ([arXiv 2306.09683](https://arxiv.org/abs/2306.09683)) | **0 human boxes for the rare classes** | LVIS **rare**-class AP **31.2 → 44.6** (+43 % rel.) | zero-shot rare classes | scales to **>1 B examples**; beats prior SOTA already at ~10 M | **No** — OWLv2 is a ViT; teacher only |
| **Semi-supervised teacher–student (MixPL)** ([arXiv 2312.07006](https://arxiv.org/html/2312.07006v1)) | **1 % / 5 % / 10 %** of a labelled set | **25.06 / 33.48 / 37.16** mAP vs **41.00** fully supervised (**90.6 % of full at 10 %**) | i.i.d. COCO split | Faster R-CNN scale; needs an unlabelled pool | **Yes if the student is a YOLO**; published numbers use Faster R-CNN |
| **Active learning (MaskAL)** ([arXiv 2112.06586](https://arxiv.org/pdf/2112.06586)) | **17.9 %** of 14 000 images | **93.9 %** of full-data performance (random: **81.9 %**); **900 AL images = 2 300 random** | **3 held-out test sets** — closest to cross-session | Mask R-CNN retrained each round | Mask R-CNN is CNN but not in the zoo; the *method* is architecture-agnostic |
| **Box-level active detection (BiB)** † ([arXiv 2303.13089](https://arxiv.org/pdf/2303.13089)) | **10 %** of images | **97 %** of fully-supervised Fast R-CNN | VOC07 i.i.d. | — | method-level, architecture-agnostic |
| **Frozen DINOv3 + linear head** ([arXiv 2604.00313](https://arxiv.org/abs/2604.00313)) | **13 images/class (≈4 %)** | **81.8 %** macro F1; **144/class → 88.5 %** vs fully supervised ConvNeXt **88.9 %** | **underwater** (AQUA20), *classification* | ViT-B/16 frozen; ViT-S/16 = **7.99 ms/img on A10**, 2381 img/s @batch 16 | **No** — ViT; and it classifies, it does not localise |
| **SAM 3 → TinyViT distillation** ([arXiv 2604.27128](https://arxiv.org/pdf/2604.27128)) | dataset-dependent | MOTA **92.29 %**, IDF1 **96.15 %** (−1.68 / −0.84 pp vs teacher) | video tracking | teacher peak **19.52 GB**; student **6.49 GB**, **1.18 FPS on A10** | **No** — TinyViT student; Orin-NX fit is **analytic, not measured** |
| **Hand labelling (today's Mongla baseline)** | **100 %** — 10 188 images archived | mAP50 **0.9950** on 3 models → cross-session recall **29.2 / 72.7 / 68.3 %** | **cross-session, held-out** | zero | **Yes** — yolo11n → `.hef` |

**Baseline the table is priced against.** On Hailo-8 at 640×640, `yolov11n` is
**39.0 float → 37.5 INT8 mAP at 185 FPS** and `yolo26n` is **40.0 → 38.4 at 155 FPS**;
INT8 costs **0.6–2.4 mAP** across the whole zoo
([Hailo Model Zoo](https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8/HAILO8_object_detection.rst)).
The pipeline runs at **98.0 Hz** today, so **every row above is compute-feasible on the
deployed side.** The entire decision is about labels, not inference.

---

## The fastest credible path from 200 raw frames of a NEW prop to a running `.hef`

Each step is marked **[sourced]** when a published number gives the time, or
**[no published time]** when it does not. No duration is invented.

**Step 0 — do not start from 200 frames alone.** No source measures the
200-frames-of-a-novel-class case end to end. The evidence says composition dominates:
MaskAL's **900 actively-sampled images = 2 300 randomly-sampled images**, and MixPL at
**1 % labels (25.06 mAP)** sits level with the supervised baseline at **10 % (26.60)**.
So the fastest credible path pools the 200 new frames **with the 10 188 archived images as
the unlabelled pool and negative set** — that is what makes the semi-supervised and
active-learning rows apply at all. **[no published time]**; no source quantifies mixing a new
class into an existing archive.

| # | Step | Time | Source |
|---|---|---|---|
| 1 | Pick **4–7 reference images** of the new prop, one box each, as visual prompts | **[no published time]** — the *count* is sourced | [PyImageSearch YOLOE-26](https://pyimagesearch.com/2026/08/31/train-yolo26-on-a-custom-dataset-with-yoloe-26-auto-labeling/) |
| 2 | Run an open-vocab labeller over the 200 frames + candidate archive frames | **≈3 min per VOC-scale dataset with YOLO-World**, **≈38 min with Grounding DINO** (VOC ≈17 k images, so 200 frames is far below this) **[sourced]** | [Voxel51](https://voxel51.com/blog/zero-shot-auto-labeling-rivals-human-performance) |
| 2b | *If text prompts fail* — the LVIS regime, **F1 0.215** — switch to SAM 3 with **3 image exemplars**: **46.4 → 65.0 CGF1** | **[no published time]** for a 200-frame run; per-image cost is contested: **2921 ms** vs **30 ms** | [Ultralytics SAM 3](https://docs.ultralytics.com/models/sam-3/) · [Meta](https://ai.meta.com/blog/segment-anything-model-3/) |
| 3 | Human review of pseudo-labels; expect to correct **≈4.9 %** of images (2 of 41 in the only published pilot) | **[no published time]** — the *rate* is sourced, the minutes are not | [PyImageSearch YOLOE-26](https://pyimagesearch.com/2026/08/31/train-yolo26-on-a-custom-dataset-with-yoloe-26-auto-labeling/) |
| 3b | Order that review queue by **model uncertainty**, not by filename | **[no published time]**; payoff is sourced: **93.9 % vs 81.9 %** at equal budget, **2.56×** fewer images for equal result | [MaskAL](https://arxiv.org/pdf/2112.06586) |
| 4 | Fine-tune the CNN student. Only published pilot: 15 epochs, `patience=8`, best at **epoch 7**, imgsz 960, batch 8, `yolo26s.pt` | **[no published time]** | [PyImageSearch YOLOE-26](https://pyimagesearch.com/2026/08/31/train-yolo26-on-a-custom-dataset-with-yoloe-26-auto-labeling/) |
| 5 | Assemble **≥1 024 in-domain calibration images** — from the 10 188 archive, **never the COCO128 fallback** | **[no published time]**; the count is a hard sourced requirement | [Ultralytics Hailo export](https://docs.ultralytics.com/integrations/hailo/) |
| 6 | `model.export(format="hailo", name="hailo8", imgsz=640)` → `.hef`. Replaces the 6-step manual DFC compile; needs a **Linux x86_64** host | **[no published time]** | [Ultralytics](https://docs.ultralytics.com/integrations/hailo/) · [Hailo Community](https://community.hailo.ai/t/export-ultralytics-yolo-models-straight-to-hailo-format-hailo-is-now-available/19678) |
| 7 | Verify INT8 retention before flying it. Published **on Hailo-8L, not Hailo-8**, with **in-domain** calibration: YOLOv8n **~100 %**, YOLO11n **~96 %**, YOLO26n **~93 %** of mAP50. ⚠ Different chip from step 6's `name="hailo8"`; no Hailo-8 figure published | **[no published time]** | [Ultralytics Hailo export](https://docs.ultralytics.com/integrations/hailo/) |

**What the accumulated numbers predict.** Fully hands-off costs about **5 mAP50 points** on
easy classes (VOC 0.768 vs 0.817; COCO 0.538 vs 0.588) and **collapses on long-tail classes
(LVIS < 0.10)**. INT8 then costs **0.6–2.4 mAP**. The dominant risk is **step 2, not step 6**.
The published escape hatches from the long-tail collapse, ranked by measured effect:
**image exemplars instead of text (+18.6 CGF1)**, **uncertainty-ordered human review
(+12 pp of full performance)**, **semi-supervised training on the unlabelled archive
(+10.56 mAP at a 10 % label budget)**.

⚠ **The step with the largest unmeasured risk is the one with no published number at all.**
Nothing in this literature measures held-out-*session* recall. A pipeline scoring 0.768 mAP50
on an i.i.d. split says nothing about the 29.2 %-vs-72.7 % cross-session spread already
observed on this vehicle at identical mAP50.

---

## Claims I could NOT verify

1. **"Human-minutes saved per class."** *Nobody publishes it.* The closest are (a) Voxel51's
   **6 703 h → 1.27 h for 3.4 M objects** — an aggregate over a whole dataset whose human
   side is *estimated from AWS SageMaker pricing*, not measured; and (b) MaskAL's
   **900 vs 2 300 images**, which counts images, not minutes. Per-class human-minutes is an
   empty cell across the entire 2026 literature searched here.
2. **SAM 3's true cost per image.** Ultralytics benchmarks **2921 ms/image on an H200**; Meta
   advertises **30 ms/image with 100+ objects** on an H200 — a **97× discrepancy**. Neither
   source states what its harness includes. **Do not budget SAM 3 compute from either number.**
3. **SAM2Auto's annotation-time claim.** The abstract claims "dramatically reducing annotation
   time and eliminating labor costs"; the paper contains **no annotation-time, cost, or
   manual-baseline measurement**, and its MOT17 detection **recall is 0.224**. The claim is
   unsupported by its own evidence.
4. **"RT-DETR and transformer detectors do NOT compile for Hailo-8."** Partly contradicted:
   **`detr_resnet_v1_18_bn` is in the Hailo-8 Model Zoo** at 800×800, **31.5 HW mAP, 30.5 FPS**.
   No RT-DETR entry exists, so the narrow claim ("RT-DETR is not in the Hailo-8 zoo") stands;
   the general one does not. I found no Hailo statement naming the blocking operators.
   *This does not change the recommendation* — DETR-R18 at 31.5 mAP / 30.5 FPS is strictly
   worse than yolov11n at 37.5 / 185 — but it changes the reason.
5. **Autodistill has no published accuracy numbers.** Workflow documented
   (GroundedSAM/OWL-ViT/DETIC → YOLOv8/v5/NAS, `epochs=50` in the worked example); **no mAP,
   no labelling time, no cost comparison** anywhere in its blog or docs.
6. **YOLOE-26 auto-labelling mAP.** The only walkthrough reports mAP50/mAP50-95
   **qualitatively** on **41 training images / 2 classes** of rigid perfume bottles. No
   at-scale YOLOE-26 auto-label → fine-tune benchmark was found.
7. **Hailo QAT/QFT accuracy recovery.** Hailo documents the method (QFT, 4/8/16-bit weights,
   outlier clipping before finetune) but publishes **no "QFT recovers X mAP" figure**, and
   `format="hailo"` does not expose it — that path is calibration-only PTQ.
8. **CVPR 2025 "Uncertainty Meets Diversity" ≈85 %-of-full-performance figure** came from a
   search snippet; **openaccess.thecvf.com returned HTTP 403** and I could not read the
   paper's own budget table. Second-hand.
9. **MDPI / ScienceDirect active-learning papers** (Sensors 25(6):1798, "35.10 mAP with 10 %
   of MS-COCO"; the construction-site active-transfer-learning study) both returned
   **HTTP 403**. Their numbers exist only in search snippets and are **excluded** from the
   sourced claims above.
10. **† Numbers marked with a dagger were read from a WebSearch snippet, not from the
    primary source.** I did not open: Grounding DINO / 1.5 ([arXiv 2405.10300](https://arxiv.org/abs/2405.10300v2)),
    OWL-ST ([arXiv 2306.09683](https://arxiv.org/abs/2306.09683)), BiB ([arXiv 2303.13089](https://arxiv.org/pdf/2303.13089)),
    LabelMatch ([arXiv 2206.06608](https://arxiv.org/pdf/2206.06608) — my fetch returned image
    streams), PL-DC, the retail-product-recognition study, or SFL-YOLO. Their numbers are
    plausible and consistently reported, but they are **relayed, not verified**. The OWL-ST
    and BiB rows in the comparison table both carry this mark.

11. **Any underwater-competition-prop result.** Underwater detection work exists —
    [SFL-YOLO **85.4 % mAP@0.5** on URPC2020](https://pubmed.ncbi.nlm.nih.gov/42373944/) †,
    [AquaYOLO26](https://www.mdpi.com/2073-8994/18/9/1442),
    [YOLO robustness study, arXiv 2509.17561](https://arxiv.org/abs/2509.17561) (YOLOv8–v12,
    **10 000 images**, six simulated underwater environments; YOLOv12 strongest overall but
    **highly vulnerable to noise**, which disrupts edge and texture features; that paper's
    own mAP-degradation table was not in the fetchable abstract) — but **nothing** on
    auto-labelling a novel rigid competition prop, and **nothing** reporting cross-session
    recall. That gap is ours to fill.

---

*37 web calls (including four that returned HTTP 403, two oversize-PDF failures and two
redirects). Every number above is attributed to the source that printed it; nothing is
inferred from a number a source did not state.*
