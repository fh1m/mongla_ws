# Vision, against the world

> Dive 2 of four. Method and rubric: [`README.md`](README.md). Our side is read out of the tree
> at `file:line`. Their side carries a URL and a number.

---

## 1. What we do today

`src/mongla_vision` — ~22.4 k lines across 60 modules, 824 tests.

### 1.1 The pipeline, stage by stage

| stage | file:line | what it is |
|---|---|---|
| **capture** | `cameras/v4l2_mailbox.py:180-216` | raw V4L2 ioctl + mmap **mailbox**, one deep, newest wins. Measured: plain `read()` 396.1 ms stale → mailbox **16.9 ms**, blocking 0 ms. `FrameMeta.fresh=False` rather than repeating a frame |
| **preprocess** | `detection/preprocess.py:21-41` | present and **off everywhere**. The file's own header retracts the original claim: 17 configurations, 4 props, 3 venues, never meaningfully positive; on the gate **30.4 % → 1.2 %** |
| **detect** | `detection/hailo.py` (1 082 lines) | Hailo-8, one process-wide `VDevice`, **configure once / activate many** (4.15 ms swap vs a 10.18 ms frame), async `run_async` chosen because it releases the GIL (another thread woke 9.21 ms late on the blocking API, 0.05 ms on async) |
| **decode** | `detection/seg_decode.py:12-32` | raw seg heads decoded **in the quantised domain** — the class head is `zp=0, scale=1/255`, so thresholding bytes is bit-exact with thresholding floats. **31.5 ms → 0.93 ms** |
| **range crop** | `detection/rangecrop.py:1-56` | hysteresis crop because `imgsz` is baked into the HEF. Measured recall: apparent size 0.25 → **65.9 % full frame / 100 % cropped** |
| **track** | `tracking/roboflow_tracker.py` | OC-SORT (default), ByteTrack fallback, + a CV Kalman output smoother whose `_DT_REF_S = 0.0325` is the **measured** p50 detection interval |
| **lock ladder** | `tracking/lock_state.py:8-169` | DETECTION → FOLLOW (LK, 8.0 ms on one core) → ANCHOR (XFeat homography, 33.1 ms at 320×240) → LOST. Strict priority, **never blending**. Authority ramps linearly 0.70 s → 2.50 s, sized on 71 real gaps (p50 0.155, p90 0.651, p99 2.418, max 4.00 s) |
| **optics** | `optics.py:74-209` | flat-port Pinax rectification. Local focal grows with field angle: **685.1 px on axis vs 757.7 at r=367 — 10.6 % centre to corner**. Skipping refraction reads a 4 m gate at **5.15 m (+29 %)** |
| **metric** | `target_geometry.py` + `config/target_geometry.yaml` | `m_per_px = width_m / w_px` from a table of real prop widths. ⛔ the metric branch **has never run on the vehicle** |
| **pose** | `anchor/pose.py` | IPPE for the interval, **SQPnP** for the point estimate (p90 yaw 1.40° vs 3.38° at our 1.55 px noise). ⛔ planar **flip ambiguity is worst head-on**, which is the firing geometry |
| **flow** | `flow/flow_node.py` (1 405 lines) | the downward camera as the DVL substitute. `v = h·(flow_px/dt − f·ω)/f` — rotation subtracted **before** scaling. Measured: 3 × 30 cm slides, worst error **1.09 cm**; noise floor 0.57 mm/s; de-rotation **575.7 → 57.1 mm/s** on pure rotation |

Measured end to end: **18.0 ms** photon → detection; **80.9 Hz** standalone, **53.9 Hz** through
the ROS graph; **98.0 Hz** pipeline against `hailortcli --hw-only` at 97.9 FPS — the host is at
100 % of the chip.

### 1.2 The model-production path — the part never examined

The entire training doctrine is `.claude/skills/train-model/SKILL.md`, **40 lines**:
`yolo train data=… model=yolo11n.pt epochs=100 imgsz=640`, copy the weights, edit a config.

- **No training script, no augmentation code, no dataset versioning, no labelling tooling, no CI
  anywhere in the repository.** Training is a manual Ultralytics CLI run on another machine.
- The datasets are not in the tree: `tools/dataset_survey.py:18`, `tools/return_check.py:25-27`
  and `tools/conf_bar.py:13` all point at **hard-coded absolute paths** outside the workspace
  (`~/Work/.../2025/datasets`, `/home/fh1m/Music/detect`). 10 188 labelled images. Versioning is
  directory names: `robosub_octagon_n_200_final`, `..._final_again3`.
- **Labelling: nothing.** No CVAT, Label Studio, labelImg or Roboflow-annotate integration.
  `ROADMAP` M6: *real-pool auto-labelling — OPEN, new work; Bumblebee's is dead code.*
- Augmentation: **none of the 25 archived training configs** uses blur, rotation, perspective or
  contrast augmentation (`preprocess.py:44-45`) — which is also why a contrast fix at inference
  time is a domain shift.
- Deployment to the Hailo is a 6-step manual DFC path (`perception/hailo-vision.md:743-775`) with
  recorded traps: auto-NMS config fails on a custom class count, `.alls` accepts no comments,
  **calibration must be RGB**, `PYTHONPATH` must be cleared. Calibration set = 320 real pool
  frames.
- Then: a hand-written `<stem>.yaml` sidecar (**the Hailo backend refuses to construct without
  it**), and a row in `config/target_geometry.yaml` or every metric consumer loses range.
- ⛔ **No compiled artifact exists in the repo**: five `.pt`, eight `.yaml`, **zero `.hef`**.
- **How long any of this takes is recorded nowhere.**

What we *do* have, and it is unusual: an evaluation fleet. `tools/model_select.py` shows three
octagon models at an identical **mAP50 = 0.9950** scoring **29.2 % / 72.7 % / 68.3 %**
cross-session recall — two of them with byte-identical training configs and a 43-point gap.
`tools/detector_viewpoint_check.py` scores the **trivial whole-frame model first**: on the return
leg, null = 15.3 % at IoU 0.5, our detector = **9.7 %** — worse than guessing.

### 1.3 What the vision system cannot do today (each stated in the tree)

1. **No re-identification** — zero hits for `reid|embedding|appearance` across the package. Both
   trackers associate on IoU + motion only.
2. **No multi-view / stereo** — two cameras, two independent detectors, no cross-camera
   association. They share a process only because the Hailo has one VDevice.
3. **No self-supervision, no online learning, no auto-labelling** (`ROADMAP` M6).
4. **No segmentation model on our classes** — the seg models are stock COCO-80,
   `competition: false`.
5. **No SLAM, no map.** Tile phase bounds drift to one tile; it does not localise.
6. **The lock ladder does not steer** — it publishes `/lock`, and the control loop only acts when
   `vision.lock_s > 0`, which ships 0.
7. **Slalom has no model** (651 unique labelled frames, zero training runs) and the **return leg
   is visually unsupported** (the `gate` class scores 0.0 % at every threshold on 72 back-side
   frames).
8. **Detection range per prop is unmeasured** — the 10 px floor is still a COCO `person` number.
9. **Tool offsets are all `unmeasured: true` and read as ZERO** — and the miss equals the offset
   **at every range**.

---

## 2. What the best work does

> **Coverage note.** Complete below: underwater detection, enhancement, sim-to-real, tracking,
> re-identification, feature anchors, segmentation on our accelerator, camera settings, what the
> winning teams run, and the alternatives to optical flow. **The model-production sweep
> (auto-labelling → Hailo student) is still running** and §2.8 is marked pending.

### 2.1 The enhancement question is closed, and it closes our way

Our own measurement — 17 configurations, 4 props, 3 venues, gate detections **30.4 % → 1.2 %** —
turns out to be an extreme instance of a **published** effect, corroborated four independent ways.

- **[Is Underwater Image Enhancement All Object Detectors Need? (IEEE TIP 2024 /
  arXiv 2311.18814)](https://ar5iv.labs.arxiv.org/html/2311.18814)** — the largest controlled
  study: **13 enhancement algorithms × 7 detectors = 98 retrained models** (extended to 18 × 7 =
  133), on URPC2020 (4 434 train / 1 019 test). The detectors were **retrained on the enhanced
  images**, so this is *not* a domain-shift artefact — and enhancement still lost:
  *"all detectors retrained on the raw domain can achieve the highest AP values than the detectors
  retrained on other domains"* — **raw wins 7 of 7**. Raw AP spans 35.2 (SSD) to **45.4 (TOOD)**;
  the worst enhanced domain (UGAN) spans 39.0–40.8.
- **[Beneath the Surface (arXiv 2411.14626)](https://arxiv.org/abs/2411.14626)** — 9 enhancement
  models × 3 detectors × 2 datasets: adverse effect at the dataset level, attributed to
  **diffused edges and increased noise**, and it *especially inhibits the hard cases*.
- **[*J. Imaging* 12(1), 18](https://doi.org/10.3390/jimaging12010018)** — same dataset-level
  verdict, plus the rule that matters operationally: *"traditional image quality metrics do not
  reliably predict detection performance"*. **Never tune an image stage on UIQM or PSNR.**
- **The fourth, and the closest to our own failure**: the "Clean-Water" row of
  [arXiv 2509.17561](https://arxiv.org/html/2509.17561) — contrast enhancement + soft sharpening
  applied *at test time* to models trained on unprocessed frames:
  **0.770 → 0.649 for YOLOv12m, 0.758 → 0.652 for YOLOv8m — every model loses.** A
  cleaner-looking image that detects worse. Same mechanism as ours, milder severity.

The one place gains are reported is **per-image oracle selection** (CUPDD, mAP 0.41 → 0.64,
+56 % relative) — an upper bound obtained by choosing the best enhancement per image *after the
fact*, not deployable without a selector nobody has built. The other is **joint
enhancement-plus-detection training**, which is a second training pipeline, not a filter.

✅ **Our retraction stands, and it is now the better-evidenced position.** `preprocess.py` shipping
off is not conservatism; it is what 98–133 retrained models say.

### 2.2 Sim-to-real detection: expect about half

[Towards Scaling Marine Perception with Synthetic Data
(arXiv 2609.20680)](https://arxiv.org/html/2609.20680) — OceanSim renderer, real test set of 179
images / 718 instances:

| training data | model | mAP50 | mAP50-95 |
|---|---|---|---|
| synthetic only, 1 800 | YOLOv9 | **0.341** | 0.110 |
| real only, 1 800 | YOLOv9 | **0.793** | 0.403 |

Synthetic-only reaches **43 % of real-trained mAP50 and 27 % of mAP50-95** — and the tight-IoU
column is the one alignment lives in. The authors: *"models trained on simulated data fall short
on being able to identify objects under heavy underwater degradation."*

✅ The published version of our own "sim imagery is too clean" finding, with a number. Note what
the top teams do instead: **CMU and ITU both use synthetic only as a supplement** for conditions
real data under-represents (extreme angles, high turbidity) — never alone.

### 2.3 Tracking: our tracker is one point off the best, and Re-ID is the wrong purchase

Head-to-head under identical detections
([Roboflow `trackers` benchmark](https://trackers.roboflow.com/latest/trackers/comparison/)),
MOT17 HOTA: SORT 58.4 · ByteTrack 60.1 · **OC-SORT 61.9 (ours)** · C-BIoU 63.0 · BoT-SORT 63.7.

Adding appearance re-identification, from **Deep OC-SORT's own table** (same detections, not a
cross-paper subtraction, [arXiv 2302.11813](https://arxiv.org/pdf/2302.11813)):

| benchmark | OC-SORT | Deep OC-SORT | Δ |
|---|---|---|---|
| MOT17-test | 63.2 | 64.9 | **+1.7** |
| MOT20-test | 62.1 | 63.9 | **+1.8** |
| DanceTrack-test | 55.1 | 61.3 | **+6.2** |

The large gain is **only** on DanceTrack — a benchmark deliberately built so that *all targets
look identical and motion is non-linear*. A prop course is the opposite: one to three objects of
**distinct classes**. The association ambiguity Re-ID resolves is largely absent here.

The cost: **OSNet is 2.2 M params and 979 MFLOPs *per crop, per frame***
([arXiv 1905.00953](https://arxiv.org/pdf/1905.00953)) — a per-frame bill of
`N_tracks × 979 MFLOPs` on top of a graph already at 53.9 Hz. And there is **no Re-ID model of any
kind in the Hailo-8 model zoo**.

⛔ **Zero of the five RoboSub TDRs read use re-identification, appearance embeddings or
cross-camera association.** Identity comes from **class + geometry**. **Re-ID is the lowest-value
addition on this list for this vehicle.**

### 2.4 The blackout is a geometry problem, and our anchor is the right one

At our measured p99 gap of **2.418 s**, a constant-velocity Kalman prediction has accumulated
roughly 0.7 m of unobserved displacement at 0.3 m/s — nothing in the SORT family models that, and
Re-ID has nothing to re-identify *against* mid-blackout. What survives is geometry carried forward
from the image, which is what the XFeat homography anchor does.

[XFeat (CVPR 2024, arXiv 2404.19174)](https://arxiv.org/html/2404.19174v1), all on an
**Intel i5-1135G7 at VGA** — the closest published proxy to Pi-5-class CPU at our 640×360:

| method | FPS | Acc@10° | inliers | HPatches viewpoint MHA |
|---|---|---|---|---|
| ORB | 44.3 | 43.1 | — | 71.4 |
| SuperPoint | 3.0 | 67.4 | 495 | 79.6 |
| ALIKE | 5.3 | 77.7 | 333 | 77.5 |
| DISK | 1.2 | 81.3 | 1 231 | 77.5 |
| **XFeat** (ours) | **27.1** | 74.9 | **892** | **81.1 — best in the table** |
| **XFeat\*** semi-dense | **19.2** | **85.1** | **1 885** | — |

✅ **The 2025 RoboSub champion runs the same anchor.** NUS Bumblebee's BBAUV 4.5 pipeline is
**YOLO11 → XFeat → PnP** ([RS25 TDR](https://robonation.org/app/uploads/sites/4/2025/07/RS25_TDR_National-University-of-Singapore-Bumblebee-compressed.pdf));
their 2023 stack was YOLOv8 + **SIFT** + PnP, so 2023→2025 is literally SIFT → XFeat.

The one published free-lunch axis in that family: **XFeat → XFeat\*** — **+111 % inliers
(892 → 1 885) and +10.2 Acc@10°** for **1.4× the time**. A dead end worth naming:
**LightGlue on CPU is 0.31 pairs/s ≈ 3.2 s per pair** — unusable inside a 2 s gap.

⚠ Benchmark mismatch, stated: MegaDepth and HPatches measure wide-baseline pose between image
*pairs*, not planar-anchor survival across a blackout. The ranking transfers; the absolutes do not.

### 2.5 Segmentation is cheap on our chip — and unproven for our purpose

Official [Hailo-8 model zoo](https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8/HAILO8_instance_segmentation.rst)
(PCIe Gen3 ×4, i5-9400 host, DFC 2.19.0, batch 1):

| model | task | COCO mAP | **FPS on Hailo-8** |
|---|---|---|---|
| yolov8n | det | 37.0 | **1036** |
| yolov11n | det | 39.0 | 185 |
| **yolov8n_seg** | inst-seg | 29.7 | **528** |
| **yolov8s_seg** | inst-seg | 36.4 | **107** |

**Instance masks are not the bottleneck** — `yolov8n_seg` runs ~10× our graph rate. Two hard
constraints follow:

⛔ **No SAM variant — FastSAM, MobileSAM, EdgeSAM — exists in the Hailo-8 zoo**, and no Hailo-8
compilation of one is published. Their quoted speeds are iPhone 14 and 2080 Ti numbers.
⛔ **No controlled study compares masks against boxes for *alignment* error.** The field routes
around it: oriented boxes (Cornell YOLO-7D), keypoints (CMU yolo26-pose), features + PnP
(Bumblebee). **The published route to alignment precision is geometry, not mask boundaries** —
which is the route our lock ladder already takes.

### 2.6 Exposure: the trade is asymmetric by ~2.5×

There is **no published fixed-vs-auto exposure detector study underwater**. The best available
bound is a robustness sweep — [arXiv 2509.17561](https://arxiv.org/html/2509.17561), 10 000
images, 5 YOLO variants, mAP@50:

| degradation | cost, relative |
|---|---|
| **noise** (σ=10 + s&p) | **−75 % to −81 %** (0.770 → 0.192 worst) |
| **blur** (19×19 Gaussian) | **−26 % to −34 %** (0.752 → 0.498) |
| colour cast (bluish/greenish) | **−2 % to −5 %** |

**Noise is catastrophic, blur is serious, colour is nearly free.** A short shutter is paid for in
gain and gain in noise, so **the noise penalty is ~2.5× the blur penalty** — shortening exposure is
only defensible while noise stays low, which makes lighting part of the same decision. ⚠ The
degradations are *synthetic*; a real shutter/gain sweep on our own footage would be **new
evidence, not a replication**. This independently reinforces §2.1: colour is not where wins are.

### 2.7 What the winning teams run — and the one thing they have that we do not

| team | place | detector | geometry | filter | compute | sensing |
|---|---|---|---|---|---|---|
| **NUS Bumblebee** BBAUV 4.5 | **RoboSub 2025 — 1st Autonomy, 1st Design Doc** | YOLO11 | **XFeat → PnP**, HDBSCAN, DepthAnything | UKF with extreme-reading rejection | Jetson Orin AGX 32 GB | **Oculus M750d multibeam sonar ($21 300)**, Pathfinder DVL, MUSIC DoA |
| **Cornell CUAUV** Orion 2.0 | RoboSub 2026 | **YOLO-7D** — 7-channel **oriented-box**: RGB + depth + normals | OBB + depth | multiplicative UKF + linear KF | Orin Nano, *explicitly compute-bound* | ZED stereo |
| **CMU TartanAUV** | RoboSub 2026 | **yolo26-pose — keypoints, not boxes** | keypoints → pose, RTAB-Map | EKF | AGX Orin | Oak-D W; synthetic only for extreme angles / turbidity |
| **ITU AUV (Taluy)** | RoboSub 2025 — 3rd Autonomy | YOLOv11 on Blender-synthetic **+ real** | stereo depth → TF frames | EKF | AGX Xavier + Orin Nano | 360° sonar + Ping 1D |
| **ASU Desert WAVE** | RoboSub 2025 — 2nd Autonomy | ML **visual servoing** | a-priori 2D map built with a **laser tape measure** | IMU yaw loop | Orin Nano | 160° FOV camera |

**Four things every top team does:**

1. **Nobody runs Re-ID.** Identity is class + geometry.
2. **Detection is a coarse ROI; precision comes from geometry.** The box is never the final answer.
3. **The winner runs our anchor** — XFeat, independently chosen.
4. **Compute discipline is a documented competitive lever.** Cornell's two biggest wins this cycle
   were **infrastructure, not models**: GPU-direct frame handling (**2× vision frame rate**) and
   moving the Kalman filter Python → C++/Eigen (**~10× CPU cut**), both purely to buy frames back
   for YOLO. Our Pi 5 + Hailo-8 split is the same strategy, in hardware.

⛔ **The one thing they have that we do not: sonar.** Bumblebee carries an Oculus M750d; ITU a
360° plus a Ping 1D. Both use it as the **modality fallback when optics fail** — exactly our 2–4 s
blackout regime. No team publishes a detection number for it, and **there is no software
substitute in the literature**.

### 2.8 Model production — what a new class should cost

> Source dossier: `scratchpad/sota-autolabel.md`, 606 lines, 62 source links.

**Fully hands-off auto-labelling costs about five mAP points — and collapses on the long tail.**
[Zero-shot auto-labeling (arXiv 2506.02359)](https://arxiv.org/abs/2506.02359) trains the *same*
detector on auto-labels and on human labels:

| dataset | auto-labelled | hand-labelled |
|---|---|---|
| VOC | 0.768 mAP50 | 0.817 |
| COCO | 0.538 | 0.588 |
| **LVIS** | **< 0.10 mAP, F1 0.215** | — |

The cost side is not close: **3.4 M objects labelled in 1.27 h for $1.18** on one L40S against
**6 703 h / $124 092** of human time — ~5 000× faster, ~100 000× cheaper. ⛔ But **LVIS is the
long-tail regime, and a torpedo board lives there**, not in VOC's twenty everyday classes.

**The escape from that collapse is image exemplars, not text prompts.** SAM 3 on SA-Co:
**text only 46.4 CGF1 → +1 exemplar 57.6 → +3 exemplars 65.0**; LVIS zero-shot 47.0 mask AP /
53.5 box AP. Three cropped examples of the actual prop beat any English description of it.

**The single biggest lever is active learning, and its evidence is a domain-shifted robot
dataset with three held-out test sets** — [MaskAL](https://arxiv.org/pdf/2112.06586):
**93.9 % of full-data performance from 17.9 % of the data**, against **81.9 % for random
sampling** at the same budget — a **12-point gap**, and **900 actively-sampled images ≡ 2 300
random ones**. That is the closest published analogue to our own 29.2 / 72.7 / 68.3 %
cross-session spread at identical mAP50: *which* frames you label dominates *how many*.

**The unlabelled archive is worth more than it looks.**
[MixPL](https://arxiv.org/html/2312.07006v1): at **10 % labels, 37.16 mAP = 90.6 % of the 41.00
fully-supervised**; at 1 % labels, 25.06 against a 10.70 baseline.

**And underwater specifically**, with frozen DINOv3 features
([arXiv 2604.00313](https://arxiv.org/abs/2604.00313)): **13 labelled images per category (≈4 %)
→ 81.8 % macro F1**; 144 per category → 88.5 %, level with a fully-supervised ConvNeXt at 88.9 %.
⚠ Classification, not detection — carried as an indication, not a promise.

**Our accelerator arbitrates the whole decision, and it says the bottleneck is labels.** Hailo-8
Model Zoo measured: `yolov11n` **39.0 float → 37.5 INT8 at 185 FPS**; `yolo26n` **40.0 → 38.4 at
155 FPS**; INT8 costs **0.6–2.4 mAP across the entire zoo** — *less than the auto-labelling
penalty*. `yolov8n` runs at **1036 FPS**. Against our 98.0 Hz pipeline the inference headroom is
enormous. **The whole decision is labels, not inference.**

**The six-step manual compile path is obsolete.** `model.export(format="hailo")` landed in
ultralytics **8.4.97** — one call to a `.hef`, covering YOLOv8 / YOLO11 / YOLO26 including custom
models. ⛔ **It requires ≥1 024 in-domain calibration images, and with none supplied it silently
falls back to COCO128** — precisely the silent-failure shape this codebase keeps finding, and the
same class of trap as our missing-sidecar-means-empty-allowlist rule.

**Three corrections to what we believed going in:**

1. **"Transformers do not compile for Hailo-8" was too strong.** `detr_resnet_v1_18_bn` **is** in
   the zoo (800×800, 31.5 HW mAP, 30.5 FPS). **RT-DETR** is genuinely absent. The conclusion
   stands — DETR-R18 is strictly worse than `yolov11n` on both axes — but the reason was wrong.
2. **SAM2Auto does not contain the number we went looking for.** It publishes no annotation-time,
   cost or manual baseline despite claiming savings, and its **MOT17 detection recall is 0.224**.
   For a project where cross-session recall is the only predictive metric, an auto-labeller that
   finds 22.4 % of objects **silently deletes three quarters of the truth**.
3. **"Human-minutes saved per class" is published by nobody.** The closest figures are an
   aggregate whose human side is *estimated from AWS pricing*, and MaskAL's image counts. It is an
   empty cell across the 2026 literature — so if we measure it on our own pipeline, that number
   does not currently exist anywhere.

⛔ **The standing gap in the entire field:** *nothing in this literature measures held-out-session
recall.* Every headline number is an i.i.d. split. **We already measure the thing the field does
not** (`tools/model_select.py`, `tools/domain_gap.py`, `tools/detector_viewpoint_check.py`) — that
is not a deficiency of ours, it is a lead.

### 2.9 Beyond optical flow: what would actually beat it

> Source dossier: `scratchpad/sota-vo-depth.md`, ~30 fetched sources.

**First, the yardstick that makes the comparison honest.** Published VO reports drift as a
percentage of trajectory; ours is a *velocity sensor*, where scale bias integrates linearly and
the noise floor random-walks to nothing. Our measured scale is **103.4 % of truth ⇒ a 3.4 % drift
floor** — and 30 cm × 3.4 % = **1.02 cm against a measured worst error of 1.09 cm**. **Our
worst-case flow error is our scale bias.** Every percentage below is read against that 3.4 %.

**The reference ceiling, stated once:** a DVL-INS dead-reckons at **0.01–0.1 % of distance**. We
are 30–300× worse, and **nothing in this survey closes that gap without acoustics.**

| method | accuracy | runtime on edge | what it needs that we lack | verdict vs our LK flow |
|---|---|---|---|---|
| **DeepVL** ([arXiv 2502.07726](https://arxiv.org/html/2502.07726), ICRA 2025, [code](https://github.com/ntnu-arl/DeepVL)) — velocity from IMU + thruster commands + battery voltage, **28 k params**, 3×GRU(40) | **3.9 % relative position error through a full visual blackout**; 0.39 m RMSE per 10 m over 88 trajectories; flew on a real BlueROV | **<5 ms on an Orin AGX** | ~4 h of DVL- or mocap-labelled velocity truth | **Same accuracy class as our 3.4 % floor — with the camera off.** The strongest candidate in this survey |
| **Monocular metric depth** — [benchmark on FLSea + SQUID](https://arxiv.org/abs/2507.02148) | Metric3D v2 collapses in water: **AbsRel 0.197 → 1.5331**; Depth Pro **3.2185** on SQUID. **Only UniDepth V2 ViT-L holds: AbsRel 0.093–0.116, δ₁ 0.91–0.94** | Hailo-8 zoo publishes `scdepthv3` at **929 FPS (1.08 ms)** — but **no published Hailo-8 number for UniDepth** | the model that works underwater is not the one we have a chip number for | the only route that removes our **known-height refusal** — and it is not yet a route on our chip |
| **Dense flow** — NeuFlow v2, RAFT | — | NeuFlow v2 **>20 FPS @512×384 on an Orin Nano GPU (≤50 ms)**, ≥4× our 12.34 ms; RAFT ~100 ms on a 1080 Ti. **No optical-flow model in the Hailo-8 zoo** | a GPU we do not carry | **rejected — same shape as our own Fourier-Mellin rejection** |
| **DIVO** ([arXiv 2607.04615](https://arxiv.org/pdf/2607.04615)) | ATE 0.233–0.530 m on quarry sequences | **8 fps on an i9 + RTX 2000 Ada (~125 ms)**, 10× our cost | a DVL | the transferable idea is the **SuperPoint+LightGlue frontend**, not the system |
| **Monocular SLAM underwater** — AQUALOC | ORB-SLAM3 tracked **1.426 m of a 16.212 m** trajectory before losing tracking | — | — | **it does not drift, it stops.** Our flow has no map to lose — a structural advantage |
| **Event cameras** | one benchmark exists ([AquaticVision, arXiv 2505.03448](https://arxiv.org/abs/2505.03448)); **no ATE, no success rate, no camera model extractable** | — | — | **not a candidate; the literature is thin and we say so** |
| **Pool-floor terrain nav** | barely exists — "terrain-aided navigation" underwater means **sonar bathymetry**. Closest number: **1.04 % relative distance error** for pool beacon ranging (PMC9611530) | — | a cooperative beacon | our tile/lane work is ahead of the literature here, not behind it |

**Refraction, independently confirmed:** underwater photogrammetry loses **~5× precision without
in-water recalibration and ~2× with it** (PMC4570311). Our 46.7° measured water FOV against the
63.8° air datasheet was **not optional** — and any metric-depth model carrying an in-air focal
starts with a **27 % focal error** before attenuation is considered.

**The sharpest finding — failure detection.** Super Odometry 2.0
([arXiv 2608.25427](https://arxiv.org/abs/2608.25427), *Science Robotics*) gates visual health on
the **Hessian of KLT tracking** — *the same matrix Shi-Tomasi already computes in our flow node*.
We are one eigenvalue away from a published health metric. It disables a modality when its
contribution stays below **10 % for 2–4 s** — the hysteresis matters as much as the threshold.

⛔ **But every published mechanism except an independent second estimator detects the *absence* of
signal.** Our measured caustics failure is the opposite: **a confident wrong signal** — tracking is
healthy, it is tracking the wrong thing. **Only a disagreeing independent estimator catches that.**
That is the real argument for a DeepVL-shaped model: not a fallback, a **second opinion**.

---

## 3. The gap

| # | what the best work does | what we do | the gap, in numbers |
|---|---|---|---|
| V-1 | **choose** which frames get labelled | label whatever was collected | MaskAL: **93.9 % of full performance from 17.9 % of the data** vs 81.9 % random — a **12-point** gap at equal effort. Our own 29.2/72.7/68.3 % spread is this effect, unmanaged |
| V-2 | exploit the unlabelled archive | 10 188 labelled images, unlabelled footage unused | MixPL: **10 % labels → 90.6 % of full supervision**. We have the footage and no pipeline that reads it |
| V-3 | prompt a foundation teacher with **image exemplars** | no teacher at all | SAM 3: text 46.4 → **3 exemplars 65.0 CGF1** on SA-Co |
| V-4 | one-call compile with in-domain calibration | 6 manual DFC steps, 320 calibration frames | `format="hailo"` since ultralytics 8.4.97 — but it needs **≥1 024** in-domain images or it **silently uses COCO128** |
| V-5 | a **second, independent** velocity estimator | one estimator (LK flow), refusing when it knows it cannot measure | DeepVL: **3.9 % error through a full blackout, 28 k params, <5 ms** — our own floor is 3.4 % |
| V-6 | gate visual health on the tracking **Hessian** | health = point survival fraction | Super Odometry 2.0 uses the **same matrix Shi-Tomasi already computes**; we are one eigenvalue from a published metric |
| V-7 | semi-dense features for the anchor | XFeat sparse | XFeat\*: **+111 % inliers, +10.2 Acc@10°, 1.4× time** |
| V-8 | a **sonar** fallback when optics fail | nothing — the blackout is absorbed by the ladder alone | 2 of 5 top TDRs carry sonar for exactly our 2.418 s p99 gap. **No software substitute exists in the literature** |
| V-9 | metric monocular depth to remove a known-height requirement | flow **refuses** without a height | only **UniDepth V2** survives water (AbsRel 0.093–0.116); **no published Hailo-8 number for it** |

---

## 4. Candidate moves

Ranked in [`SOTA-GAPS.md`](SOTA-GAPS.md). The shape of dive 2's answer:

- **The model pipeline's win is not "auto-label everything".** Fully hands-off costs ~5 mAP and
  collapses on long-tail classes — which is what our props are. The evidenced combination is
  **exemplar-prompted teacher → active selection of what a human checks → semi-supervised use of
  the rest**, with the human in the loop on ~18 % of frames rather than 100 %.
- **The one-call compile plus a 1 024-image in-domain calibration set is a same-week change**, and
  the silent COCO128 fallback is a guard we should write before we ever use it.
- **A DeepVL-shaped velocity model is the highest-value new capability in this dive** — not as a
  fallback but as the **second opinion** that can catch a confident wrong flow reading, which is
  the one failure mode nothing else in the literature catches.
- **XFeat → XFeat\*** is a measured free lunch inside a component we already run.
- **Sonar is the only place where the evidence says software cannot close the gap.** It is
  recorded as a hardware ask with its price, not smuggled in as a software move.

---

## 5. Rejected, with the reason

| rejected | why |
|---|---|
| **Image enhancement, in any form, as a preprocessing bolt-on** | Four independent studies plus our own 17-configuration result. Even **retraining on enhanced imagery loses, 7 detectors of 7**. Closed permanently; the only live variants are joint enhancement-plus-detection training (a second pipeline) and per-image oracle selection (not deployable). |
| **Appearance re-identification** | +1.7 / +1.8 HOTA on MOT17/MOT20, and the +6.2 is on DanceTrack, a benchmark of identical-looking targets. Costs `N_tracks × 979 MFLOPs` per frame, has **no Hailo-8 zoo entry**, and **zero of five top TDRs use it**. Wrong problem shape. |
| **SAM2Auto specifically** | **MOT17 detection recall 0.224.** An auto-labeller that finds 22.4 % of objects silently deletes three quarters of the truth, and our one predictive metric is recall. |
| **RT-DETR / transformer detectors on-chip** | RT-DETR is absent from the Hailo-8 zoo. DETR-R18 *is* present but is strictly worse than `yolov11n` on both mAP and FPS. |
| **Dense optical flow (RAFT, NeuFlow v2)** | NeuFlow v2 is **≤50 ms on an Orin Nano GPU** — ≥4× our 12.34 ms, on hardware we do not carry. No optical-flow model in the Hailo-8 zoo. Same verdict shape as our own Fourier–Mellin rejection. |
| **LightGlue as the anchor matcher** | **0.31 pairs/s on CPU ≈ 3.2 s per pair.** Unusable inside a 2 s blackout. |
| **Event cameras** | One underwater benchmark exists and **no ATE, success rate or camera model could be extracted from it**. Not a candidate; the literature is thin and this dossier says so rather than guessing. |
| **Monocular SLAM underwater** | AQUALOC: ORB-SLAM3 tracked **1.426 m of a 16.212 m** trajectory before losing tracking. It does not drift, it stops. Our flow has no map to lose. |

---

## 6. Research still owed on this dive

| owed | why |
|---|---|
| **A Hailo-8 number for UniDepth V2** | it is the only metric-depth model that survives water, and no published Hailo-8 figure exists — so the move that would remove our known-height refusal cannot yet be costed |
| **Held-out-session recall anywhere in the literature** | it does not exist. Every published number is an i.i.d. split, so no external baseline for our own metric can be quoted |
| **Human-minutes per class** | published by nobody. If we time our own pipeline, the number will be the first of its kind |
| **XFeat on a Raspberry Pi 5** | the published figure is an i5-1135G7 at VGA; a Pi 5 number has to be measured here |
| **Masks vs boxes for alignment error** | no controlled study exists in the field |
| **SAM 3 cost per image** | a **97× discrepancy** between two vendor figures on the same H200 could not be reconciled |
