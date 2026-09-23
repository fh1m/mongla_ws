# Camera-derived velocity and odometry — the deep pass

Compiled **2026-09-23**. This is a **delta on**
[`sota-vo-depth.md`](sota-vo-depth.md) (2026-09-22), which already covers NeuFlow v2, RAFT,
the event-camera verdict, the scale-recovery options 5a–5e, and the caustics framing. **Read
that first.** This document spends its budget on what that one does not have:

- GMFlow / FlowFormer / NeuFlow v1, and the **Hailo-8 compilability question** stated as a
  question with an answer, not a shrug
- DPVO · DROID-SLAM · TartanVO — learned VO rates on embedded
- **Feature-based frame-to-frame VO**: XFeat, LightGlue, SuperPoint+SuperGlue, ALIKED, and
  what they cost on a Pi 5 CPU
- the underwater **flow-failure** literature (caustics, turbidity, texture-poor floors)

## The bar this is judged against

Our incumbent, from `sota-vo-depth.md` and `measured-bars.md`:

| | |
|---|---|
| Method | sparse Shi-Tomasi + Lucas-Kanade + RANSAC similarity, downward camera, IMU de-rotated |
| Cost | **12.34 ms/pair, Pi 5 CPU** |
| Accuracy | worst **1.09 cm** over three 30 cm slides; noise floor **0.57 mm/s**; scale **103.4 %** |
| Known failures | bare floor (no texture), sun caustics, yaw ≥ **1.128 rad/s** |

⛔ **The decisive framing — and it is stronger than "our error is scale bias".** Read
`measured-bars.md` §13 before believing the 1.09 cm is the *sensor's* error. The three slides
were **in air, at h = 0.72 m, by hand, against a tape**, and the file says so:

> *"in air, on a hand slide whose own precision is roughly **±1 cm** — the operator's tape and
> hand are inside our error bar, so this is **an upper bound on the sensor's error, not a
> measurement of it**."*
> **"The gap between 0.02 cm synthetic and 1.09 cm physical is the hand, the tape and the
> height — not the algorithm."**

⭐ **The synthetic control, same console, same optics, exact truth: 29.98 / 29.99 / 30.02 cm,
max error 0.02 cm.** And against warped-truth frames the estimator returns **0.006–0.40 px**
(`measured-bars.md`, 2026-09-11).

⛔ **So the incumbent's algorithmic error is not 3.4 %, it is ~0.07 %, and nothing in this
document can improve on it.** Every headline number below — RAFT's EPE, NeuFlow's 4.33,
XFeat's inlier counts — measures something our incumbent has already saturated. A candidate
must win on a *failure regime*, not on nominal accuracy:

1. does it produce usable motion on a **bare pool floor**, where LK gets no tracks?
2. does it **reject caustic** motion (which is real image motion, not sensor noise)?
3. does it survive **yaw ≥ 1.128 rad/s**?

⛔ **The Hailo cost floor.** Measured here: at batch 1 the **PCIe round trip dominates**, and
our fastest *complete* loop — frame → letterbox → chip → NMS decode → boxes — is **7.99 ms
(125 Hz) for stock yolov8n at 640×640** (`measured-bars.md` §14.3). ⚠ So the floor is **under
8 ms for a 640×640 input**, and it presumably scales with tensor size; it is not a fixed
9.3 ms. LK already runs in 12.34 ms on a CPU core the Hailo does not free, and **the detector
already owns the chip**. The economic case for moving *anything* else there is thin before we
start.

---

## 0. What we already measured in-house — read this before any paper

Two local measurements decide more of this document than any publication, and both are in
[`measured-bars.md`](../../measured-bars.md).

### 0a. ⭐ XFeat is already benched on our Pi, on our footage (`measured-bars.md` §8)

ONNX-exported, **2.7 MB**, `onnxruntime` CPU, **with the vision stack running**:

| resolution | 1 thread | 3 threads |
|---|---|---|
| 640×480 | 145.6 ms (6.9 Hz) | 88.0 ms (11.4 Hz) |
| **320×240** | **33.1 ms (30.2 Hz)** | 18.2 ms (55.1 Hz) |

And on our own murky Mirpur clips, frame-to-reference with `USAC_MAGSAC`:

| clip | ORB ref kp | ORB ok | **XFeat ok** |
|---|---|---|---|
| Mirpur torpedo (murky) | 71 | 0/4 | **4/4** (350→100 inliers) |
| Mirpur torpedo_1 | 7 | 0/4 | **4/4** (301→130) |
| Mirpur gate | 111 | 3/4 | **4/4** (753→682) |
| octagon (low texture) | 1205 | 1/4 | **3/4** |

⭐ **This is the headline of the whole document.** The best *learned* feature front-end for
this problem is **already exported, already timed on the vehicle, and already proven on our
own turbid water** — at 30 Hz on one core while the LK rung runs at 125 Hz on another, with
the Pi 72.8 % idle. Nothing in §1–§4 below is better positioned than that, and nothing below
comes with a measurement on our footage.

⛔ **But read the clip names before reading the result.** *torpedo*, *gate* and *octagon* are
**props**, so this is **forward-camera footage of objects**, not **downward-camera footage of
a floor**. `measured-bars.md` separately distinguishes the two cameras on exactly this axis —
the caustic detector threshold is *"for the downward camera only"* because *"RoboSub FORWARD
clips read 0.04–0.06"*. ⚠ **So the XFeat win above is measured in a different domain from the
one the velocity sensor lives in**, and the EdgePoint2 lesson on the very next line of that
section is precisely a warning about cross-domain transfer. **The claim this document makes
for XFeat is therefore narrowed to: proven on turbid forward prop footage, unproven on the
downward floor.** Closing that gap is the first step of the experiment in §3d, and it is
cheap — the 30 cm slide recordings already exist.

⚠ **But it is benched as an *anchor* (frame-to-reference homography), not as a *velocity
sensor* (frame-to-frame).** That is a different problem with a different failure mode, and
`measured-bars.md` §9 is the warning: the same homography pipeline recovers plane tilt that
looks excellent under smoothing and is **wrong by 4.7° median / 17.5° p90** against a
two-snap control. Per-reference error there is a **bias, not zero-mean noise.** Any
frame-to-frame velocity built on XFeat must be validated against a known-truth slide, the
way the LK sensor was (three 30 cm slides), **not** against the LK sensor itself.

⛔ **EdgePoint2's published advantage did not transfer** (`measured-bars.md` §8): documented
as *2× faster than XFeat with competitive IMC2022 results*, it scored **1–4 inliers where
XFeat scored 86–155** on our clips. The standing rule this produced: *"any replacement must
be benched on the Mirpur clips, not on a public leaderboard."* **Apply that rule to every
candidate below.**

### 0b. ⛔ The Hailo batch-1 floor, measured and retracted-into (`measured-bars.md` §14)

| model | contexts | infer | loop |
|---|---|---|---|
| `sauvc_sim` — ours | 3 | 9.48 ms | 97.47 Hz |
| `yolov8s` — stock | 1 | 9.22 ms | 99.73 Hz |

A 4.9× throughput gap in `hailortcli benchmark --hw-only` **vanished end to end**: at batch 1
both are dominated by the **PCIe round trip, ~9.3 ms**. The consequence for this document:

> **Any model moved to the Hailo-8 is round-trip-bound, not compute-bound, at batch 1.** The
> best complete loop measured here is **7.99 ms** (yolov8n, 640×640, letterbox + NMS
> included); ours run at 10.20–10.26 ms. ⭐ **The decisive point is not the millisecond count
> — it is that the chip is a single shared resource the detector already occupies**, and the
> Pi is **72.8 % idle**. ⛔ Spending detector rate to save CPU time we are not short of is not
> a trade worth making, and it is settled before any paper is read. ⚠ It also means **every
> vendor or forum FPS figure in this document must be read against ~8 ms of round trip** —
> the same skepticism our own §14 retraction was about, applied uniformly.

---
## 1. Dense optical flow — and the Hailo question, answered

### 1a. ⭐ Can a dense-flow net run on a Hailo-8? **No — and it is an architectural "no", not a porting gap.**

`sota-vo-depth.md` closed this section with *"no one has published a flow model compiled for
Hailo-8 that I could find"* and left it on the unverified list. **It can now be closed
properly**, from Hailo's own mouth.

⭐ **Verified in the source, not assumed.** `gh search code grid_sample` returns, as of
2026-09-23:

- `neufieldrobotics/NeuFlow_v2:NeuFlow/corr.py` — *"Wrapper for grid_sample, uses pixel
  coordinates"*, `F.grid_sample(img, grid, align_corners=True)`; also `NeuFlow/utils.py`
- `haofeixu/gmflow:gmflow/geometry.py` and `gmflow/matching.py` —
  `F.grid_sample(feature1, sample_coords_norm, ...)`

So **the edge-targeted model (NeuFlow v2) and the global-matching model (GMFlow) both depend
on `grid_sample` directly**, not by family resemblance. ⚠ GMFlow does *global* matching rather
than iterative correlation lookup, so the mechanism differs — but it uses the same primitive
for feature warping and windowed sampling. RAFT's dependence is in its architecture
description: the lookup operator *"extracts correlation
features from the correlation volume by taking a fine feature pixel with its optical flow
field, computing a 2D grid around it with a predefined radius, and performing subpixel
bilinear resampling along the grid"*
([RAFT, arXiv 2003.12039](https://arxiv.org/abs/2003.12039), Mar 2020;
[explainer](https://learnopencv.com/optical-flow-using-deep-learning-raft/)).

[**Hailo Community — "Support for GridSample operation" (asked 17 Apr 2025, answered 24 Apr
2025)**](https://community.hailo.ai/t/support-for-gridsample-operation/13957). Omria, Hailo:

> *"Right now, GridSample isn't supported (**and there's no plan to add general support**)
> because the operation doesn't really fit with how our SDK works."*

The suggested workaround is to **run the sampling on the host CPU**, which a reporting user
did for a deformable-attention module and found it *"creates performance challenges due to
weight transfers between CPU and Hailo hardware."* For an *iterative* flow net that means one
PCIe round trip **per refinement iteration** — RAFT uses 12–32. At our measured **≥8 ms per
round trip** that is **~96–256 ms per frame** before any arithmetic. Corroborating:
[grid_sample problem](https://community.hailo.ai/t/grid-sample-problem/17078).

⛔ **Verdict: as of Sep 2026, no RAFT- or GMFlow-family dense flow runs usefully on our
Hailo-8.** Not "nobody has tried" — the vendor has declined to support the primitive these
architectures call directly. ⚠ **The vendor statement is dated 24 Apr 2025 and the
supported-layer list moves.** Re-open only on a Hailo DFC release note that names GridSample;
do not re-open on a new flow paper.

Corroborating negative: the [Hailo Model Zoo public
models](https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/PUBLIC_MODELS.rst) list
~30 task categories — Classification, Depth Estimation, Stereo Depth Estimation, Pose
Estimation, Super Resolution, Zero-shot Depth Estimation and so on. **There is no Optical
Flow category, and no local-feature/keypoint-matching category** (no SuperPoint, XFeat or
LoFTR). Stereo Depth Estimation exists, which matters for §5.

### 1b. The dense-flow models, with the hardware their numbers came from

| Model | Date | Published accuracy | Published rate — **and on what** | Hailo-8? |
|---|---|---|---|---|
| **NeuFlow v2** ([arXiv 2408.10161](https://arxiv.org/abs/2408.10161), Aug 2024, [code](https://github.com/neufieldrobotics/NeuFlow_v2)) | 2024 | KITTI-15 EPE **4.33**; claims **10×–70× speedup** over SOTA at comparable accuracy | **>20 FPS at 512×384 on a Jetson Orin Nano**, *half precision*, 1024 CUDA cores. Also benched on an RTX 2080. Trained FlyingThings, evaluated Sintel + KITTI-15 **training** sets | ⛔ GridSample |
| **NeuFlow v1** ([arXiv 2403.10425](https://arxiv.org/abs/2403.10425), Mar 2024) | 2024 | KITTI-15 EPE **12.4** (v2 paper's figure for v1) | edge-targeted, same Jetson class | ⛔ |
| **RAFT / RAFT-small** ([arXiv 2003.12039](https://arxiv.org/abs/2003.12039), Mar 2020) | 2020 | still the accuracy reference | ~**100 ms on a GTX 1080Ti**; RAFT-small is 1 M params. [DIFT (arXiv 2306.05691)](https://arxiv.org/abs/2306.05691) measures **<0.2 inferences/s for a single iteration** on a mobile platform — i.e. **seconds** per frame | ⛔ |
| **GMFlow / FlowFormer** | 2022 | higher accuracy still | desktop-GPU numbers only; both are transformer/attention stacks. Note the SuperGlue precedent below: **Hailo's parser rejects `einsum` attention** | ⛔ |

⚠ **Every rate in that table is a GPU rate.** None of these authors publish an ARM-CPU
number, and we are on a CachyOS x86 dev box — **we cannot produce a Pi 5 timing here**, and
an x86 timing would not transfer. Converting a Jetson figure to a Pi 5 figure by ratio would
be speculation and is not done.

### 1c. Would dense flow be better than sparse LK **for us**? No — and for a reason accuracy tables cannot show

1. **It solves a harder problem than we need.** A velocity sensor wants one global 2-DOF
   translation + 1 rotation. RANSAC over ~100 sparse tracks already yields that. A dense
   field is ~200 000 vectors of which we discard all but the consensus.
2. **⭐ Our error is not flow error.** 1.02 cm of our 1.09 cm worst case is the **3.4 % scale
   bias**, which is height/FOV/refraction. A dense net with EPE 4.33 on KITTI cannot touch
   it. **Dense flow buys accuracy we are not short of.**
3. **The failure regimes are shared, not fixed.** Caustics are *real image motion*: a dense
   net trained on FlyingThings will happily report the caustic's velocity as the scene's —
   with higher confidence than LK, because it has no inlier/outlier notion at all. On a bare
   floor, dense flow's regularisation **hallucinates** a smooth field where there is no
   evidence; LK at least returns *no tracks*, which is an honest refusal we can gate on.
   ⚠ This point is reasoning from the architectures, **not** from a measured underwater
   comparison — I found none; see §6.
4. **We already rejected the dense-ish alternative on measurement**: Fourier-Mellin phase
   correlation, **16.87 ms vs 12.34 ms and 10–30× the error** (`sota-vo-depth.md` §3).

⛔ **Verdict — (a) runnable? no on Hailo, unknown-but-implausible on the Pi CPU. (b) better
for us? no. (c) cost to adopt? irrelevant.** Closed.

---
## 2. Learned visual odometry — DPVO, DROID-SLAM, TartanVO

⚠ **First, the structural objection that answers most of this section before the numbers.**
All three solve **full 6-DOF monocular motion from an unknown scene** — a strictly harder
problem than our planar, known-normal, downward-looking case. And all three are **up to
scale**: TartanVO is explicit about it, training with an *"up-to-scale loss function."* So
they need a height/scale source **exactly as LK does** — they do not remove the requirement
that makes our flow sensor refuse. They would replace a 12.34 ms geometric estimator with a
GPU-class network, to get the same up-to-scale answer for a harder problem.

| System | Date | Accuracy claim | Rate — **and on what** | Memory |
|---|---|---|---|---|
| **DROID-SLAM** ([arXiv 2108.10869](https://arxiv.org/abs/2108.10869), Aug 2021) | 2021 | the accuracy reference for learned SLAM | **20 FPS on EuRoC at 320×512**; 30 FPS on TUM-RGBD at 240×320; ⛔ **8 FPS on TartanAir — not real-time** under fast camera motion | frontend needs **8 GB** GPU; **backend needs 24 GB** for EuRoC/TartanAir/ETH-3D. Reported to **OOM under an 8 GB laptop GPU** |
| **DPVO** ([arXiv 2208.04726](https://arxiv.org/abs/2208.04726) / [NeurIPS 2023](https://proceedings.neurips.cc/paper_files/paper/2023/file/7ac484b0f1a1719ad5be9aa8c8455fbb-Paper-Conference.pdf)) | 2023 | matches DROID accuracy, 1.5–8.9× faster at 57–29 % the memory | **60 FPS on an RTX-3090**; a fast variant to **120 FPS** | **4.9 GB** on RTX-3090 (vs DROID's 8.7 GB at 40 FPS) |
| **DPVO-QAT++** ([arXiv 2511.12653](https://arxiv.org/abs/2511.12653), 16 Nov 2025) | 2025 | ATE *comparable* to DPVO | **+52.1 % FPS / −29.1 % median latency / −64.9 % peak GPU memory** on TartanAir; +30.1 % / −23.1 % / −37.7 % on EuRoC | built on **GPU-native CUDA kernel fusion**; the paper does not address Jetson or any non-CUDA device |
| **TartanVO** ([arXiv 2011.00359](https://arxiv.org/abs/2011.00359), Nov 2020; [code](https://github.com/castacks/tartanvo)) | 2020 | generalises **synthetic-only training → KITTI and EuRoC with no finetuning**; beats geometric methods on challenging trajectories | **40 ms (~25 FPS) on an NVIDIA GTX 1080** | — |

**The Pi 5 has no CUDA device and 8 GB of shared system RAM.** DPVO's *best* number, 4.9 GB
and 60 FPS, is on a 350 W RTX-3090 — even DPVO-QAT++'s 64.9 % memory cut leaves ~1.7 GB of
**GPU** memory that does not exist on this vehicle, and the kernels it is built from are
CUDA. **⛔ None of these four runs on our hardware in any form.** There is no ONNX path
either: DROID/DPVO both use custom CUDA correlation ops, and the Hailo GridSample answer in
§1a applies to those too.

⭐ **The one genuinely interesting property is TartanVO's generalisation claim** — synthetic
training transferring to unseen real domains without finetuning. That is the property an
underwater deployment would need, and it is the one worth watching. But at 40 ms on a GTX
1080 it is **not a 2026 candidate for this vehicle**, and no underwater evaluation of it was
found.

**Underwater-specific learned VO**: see [`sota-vo-depth.md`](sota-vo-depth.md) §1 — DIVO
(McGill, Jul 2026) and the AQUALOC benchmark are covered there and are not re-researched
here. No conflict with that file was found.

⛔ **Verdict — (a) runnable? no, all four need a discrete CUDA GPU. (b) better for us? the
question does not arise; they are also up-to-scale, so they do not even remove our binding
constraint. (c) cost? unbounded — it is a different computer.** Closed.

---
## 3. Feature-based frame-to-frame VO — ⭐ the only live candidate in this document

This is the one family that is (a) runnable, (b) arguably better than LK **in the regime
where LK fails**, and (c) cheap to adopt — because **we already own the hard part**.

### 3a. The extractors

| Method | Date | Published speed — **on what** | On our hardware |
|---|---|---|---|
| **XFeat** ([arXiv 2404.19174](https://arxiv.org/abs/2404.19174), 30 Apr 2024, CVPR 2024; [code](https://github.com/verlab/accelerated_features)) | 2024 | *"real-time on an inexpensive laptop CPU"*; **up to 5× faster** than deep local features; ~1400 FPS batched on an RTX 4090 at VGA sparse. ⭐ **The comparison table is already in [`sota/vision.md`](../vision.md)** — i5-1135G7 at VGA: XFeat **27.1 FPS / Acc@10° 74.9 / 892 inliers / HPatches MHA 81.1 (best in table)**, vs ORB 44.3/43.1, SuperPoint 3.0/67.4, ALIKE 5.3/77.7, DISK 1.2/81.3 | ⭐ **MEASURED HERE: 33.1 ms @ 320×240, 1 thread, Pi 5, stack running** (`measured-bars.md` §8) — which ⭐ **closes `RESEARCH-OWED.md` R-12** (*"XFeat on a Raspberry Pi 5 — the published figure is an i5-1135G7 at VGA"*). ONNX, 2.7 MB |
| **XFeat\*** (semi-dense, same paper) | 2024 | **+111 % inliers (892 → 1 885)**, **+10.2 Acc@10° (74.9 → 85.1)** for **1.4× the time** (27.1 → 19.2 FPS) | ⭐ **already a ranked move: `SOTA-GAPS.md` G-18, value 15.0**, falsifier already written |
| **SuperPoint** (Magicleap, 2018) | 2018 | the reference learned detector; XFeat's "5× faster" baseline | **~300 FPS on Hailo-8 / Pi 5, reported working** — [Hailo Community, 27–28 Jul 2026](https://community.hailo.ai/t/has-anyone-successfully-deployed-superglue-on-hailo-8/19604). ⚠ **Apply §0b's skepticism to that figure too**: an unqualified 300 FPS is 3.3 ms, below our own measured complete-loop floor of 7.99 ms, so it is almost certainly a chip-only number, not a loop rate. **Not in the Model Zoo** — a community port |
| **ALIKED** | 2023 | used as LightGlue's front-end in the ONNX ecosystem | no Pi/Hailo number found. ⚠ It uses **deformable convolution**, which is a GridSample consumer — see §1a |
| **EdgePoint2** | 2025 | *2× faster than XFeat, competitive on IMC2022* | ⛔ **benched here and lost badly**: 1–4 inliers where XFeat got 86–155 on our Mirpur clips (`measured-bars.md` §8) |

### 3b. The matchers — and where the Hailo wall actually is

[**LightGlue**](https://arxiv.org/abs/2306.13643) (ICCV 2023): **35 % faster than SuperGlue**
at full depth, and adaptive depth/width pruning cuts runtime a further **33 %**, *"particularly
effective on easy pairs"* — frame-to-frame at 120 fps is the easiest pair there is, so the
pruning works in our favour. ONNX tooling is mature:
[LightGlue-ONNX](https://github.com/fabio-sim/LightGlue-ONNX) supports ORT/TensorRT/OpenVINO,
fused models with FlashAttention-2 under `onnxruntime>=1.16.0`, and there is an
[XFeat+LightGlue ONNX](https://github.com/noahzhy/xfeat_lightglue_onnx) combination. A
documented CPU lever: **cutting ALIKED's keypoint cap from 1432 to 600–800 reduces runtime
substantially while holding accuracy.**

⛔ **But this is already closed here, on a measured number, against LightGlue.**
[`sota/vision.md`](../vision.md) records: **"LightGlue on CPU is 0.31 pairs/s ≈ 3.2 s per
pair — unusable inside a 2 s gap."** At **3.2 seconds per pair** it is ~100× too slow to be a
velocity sensor at any frame rate. ⛔ **LightGlue is rejected for this vehicle. Do not
re-open** — and note that an earlier draft of this section listed its ONNX tooling
approvingly **before** checking our own file. The tooling is mature; the runtime is fatal.

⛔ **SuperGlue does not fit the Hailo-8.** From the same forum thread (Jul 2026), the failures
are named: *"keypoint encoder produces shape/layout errors"*, *"GNN attention layers use
unsupported **einsum** operations"*, *"`ReduceLogSumExp` crashes the parser."* A 4-layer GNN
variant compiles to 19.63 MB; the full 9-layer model must be **split into three chained HEFs**
at an estimated **70.8 % recall versus PyTorch**, and one user's compile ran **7 h 34 m at
"Finding the best partition to contexts"** before dying on a field-overflow error.

⭐ **The shape of the answer: extractors compile, matchers do not.** SuperPoint runs on the
chip; SuperGlue costs 30 % of its recall and three chained HEFs. Which is fine — our matching
is mutual-nearest-neighbour cosine on XFeat descriptors (`MIN_COSSIM = 0.82`, XFeat's own
default) plus `USAC_MAGSAC`, and that is **CPU work of negligible cost**.

⭐ **Keep XFeat on the CPU — and the reason is not the millisecond arithmetic, it is that we
are not short of CPU.** The Pi is **72.8 % idle** with both rungs running (§0a). Moving the
extractor to the Hailo would spend the detector's chip time to buy back a resource we have
in surplus. ⚠ It has also never been attempted here, so whether XFeat even compiles under
`hailomz` is unknown — see the unverified list.

### 3c. ⭐ Would XFeat frame-to-frame beat LK **as a velocity sensor** for us?

**Where it cannot help:** nominal accuracy. Our 1.09 cm is 1.02 cm of scale bias (§the bar).
A better matcher does not change height, FOV or refraction. **Expect zero improvement in the
nominal regime, and say so before running the experiment.**

⭐ And the incumbent's ceiling is far higher than "good enough". Scored against *truth* — a
real downward frame warped by a known (dx, dy, θ, scale), 25 frames per case, worst of the
swept range (30 px shift, 15° rotation, ×1.20 zoom) — LK + RANSAC similarity returns
**0.006–0.40 px translation, 0.002–0.05° angle, ≤0.001 scale**; on the synthetic slide
control, **0.02 cm on 30 cm**. ⛔ **There is no room above that in the nominal regime** — and
XFeat's own matcher noise, as measured here, is **1.55 px** (`measured-bars.md` §9), i.e.
**4–250× worse than what LK returns against truth.** A candidate that only matches the
incumbent has cost us 21 ms for nothing.

**Where it plausibly can — the two failure regimes, with our own evidence:**

1. ⭐ **Texture-poor scene.** This is the strongest signal in the whole document, and it is
   *ours*: on the **octagon (low-texture)** clip ORB managed **1/4** and XFeat **3/4**; on
   Mirpur torpedo_1 ORB found **7 keypoints total** and failed 0/4 while XFeat found **4096
   keypoints, 130–301 inliers, 4/4**. Shi-Tomasi — what our LK rung uses — is a *corner*
   detector of the same classical family as ORB's FAST front-end. ⛔ **Two gaps, both stated
   rather than papered over**: (i) the clips are **forward-camera prop footage**, not the
   downward floor (§0a); (ii) **Shi-Tomasi was never benched** — the inference that it fails
   where ORB fails is **reasoned, not measured**. ⭐ Both close with one cheap run: keypoint
   and inlier counts, **Shi-Tomasi vs XFeat, on the existing downward 30 cm-slide
   recordings**. **Do that before anything else in this document.**
2. **Turbidity.** Settled in our favour and already recorded: the earlier conclusion *"the
   anchor dies in murky water"* was **retracted** — *"That was an ORB limitation, not a water
   limitation."*

**Where it probably does not help:** caustics. XFeat will detect and match caustic structure
as readily as it matches floor texture — caustics are high-contrast, repeatable-over-a-few-
frames image structure. The defence there is **not** a better detector (§6).

**High yaw rate (≥1.128 rad/s):** unknown either way. Descriptor matching is more
rotation-tolerant than LK's small-displacement linearisation, so there is a plausible win
here, but **no source and no measurement — this is labelled speculation.**

### 3c-bis. ⭐ Prior art and existing gap rows — this is not a new idea here

- ✅ **The 2025 RoboSub champion runs this exact front-end.** NUS Bumblebee's BBAUV 4.5 is
  **YOLO11 → XFeat → PnP**
  ([RS25 TDR](https://robonation.org/app/uploads/sites/4/2025/07/RS25_TDR_National-University-of-Singapore-Bumblebee-compressed.pdf));
  their 2023 stack was YOLOv8 + **SIFT** + PnP, so 2023→2025 is literally SIFT → XFeat.
  ⚠ **But that is XFeat for *pose on a prop* — which we already do.** It is **not** evidence
  for XFeat as a downward velocity sensor, and this document must not borrow their
  endorsement for a use they do not make.
- **`SOTA-GAPS.md` already carries two rows here, both scored above anything below:**
  - ⭐ **G-19 — "Gate flow health on the KLT Hessian we already compute" · value 20.0 ·
    falsifier: replay recorded caustics footage.** ⛔ **This is the highest-value vision move
    on the board, it addresses §6's caustics problem directly — by making the incumbent
    refuse honestly rather than replacing it — and it costs a replay, not a component.**
  - **G-18 — XFeat → XFeat\* in the anchor rung · value 15.0.**
- ⛔ **Both outrank the frame-to-frame idea below on the project's own rubric** (value ÷ risk
  × effort), and both are cheaper. **Stated plainly: §3d is a third priority, not a first.**

### 3d. What adoption would cost

⭐ **Unusually little, because none of the expensive parts are new.** `tools/xfeat_export.py`,
the 2.7 MB ONNX, the matching + `USAC_MAGSAC` path in
`src/mongla_vision/mongla_vision/anchor/anchor.py`, and the Mirpur bench protocol all exist.
The work is: point it at consecutive downward frames instead of a stored reference, recover
the **similarity** (not homography — the floor is a known plane and the full 8-DOF fit is
over-parameterised and less stable), de-rotate with the IMU exactly as LK does, scale by
height, and feed the EKF's existing body-velocity channel. Budget: **one CPU core at 30 Hz**,
which §0a shows is available (Pi 72.8 % idle with both rungs running).

⛔ **Three refusals that must be written into the experiment before it starts:**

1. **Do not validate it against the LK sensor.** That measures agreement and cannot rank
   them (`README.md` rule 3). Validate on **known-truth slides** — the same three 30 cm
   slides, plus a **bare-floor** slide and a **caustics** slide, because those are the only
   regimes where a win is possible.
2. **Reproduce the two-snap control** of `measured-bars.md` §9. The same XFeat+homography
   pipeline produced a plane tilt that looked excellent under smoothing and was wrong by
   4.7° median / 17.5° p90, because its per-reference error is a **bias, not zero-mean
   noise**. Frame-to-frame velocity integrates bias linearly. A smooth, confident, wrong
   velocity is precisely the failure mode §8 of `CLAUDE.md` names.
3. **The falsifier, stated in advance:** *if XFeat frame-to-frame does not beat LK on the
   bare-floor slide, it has no case at all* — because it cannot beat it anywhere else.

**Verdict — (a) runnable? ⭐ yes, measured, 30 Hz on one core. (b) better for us? only in the
texture-poor regime, where our own data says it is dramatically better as a *matcher*;
unproven as a *velocity sensor*. (c) cost? low — the export, the model and the bench protocol
already exist.** ⭐ **This is the one thing in this document worth an experiment.**

---
## 4. Event cameras underwater — ⭐ the literature moved since 2026-09-22, and the verdict does not

[`sota-vo-depth.md`](sota-vo-depth.md) §6 recorded one source (AquaticVision, May 2025) and
honestly reported *"I could not extract a single quantitative event-vs-frame comparison from
it."* ⭐ **That gap is now partly filled — three newer papers exist, and they make the answer
for us worse, not better.**

| Work | Date | What it gives |
|---|---|---|
| [**AquaticVision**](https://arxiv.org/abs/2505.03448) | May 2025 | stereo DAVIS346 events + grayscale + IMU, 6-DoF mocap truth, clear→turbid. ⚠ **Correction:** an earlier draft attributed a "time surfaces degrade in turbidity" finding to this abstract. **I fetched the abstract: it says no such thing and carries no numbers.** The claim is withdrawn from this source |
| [**A High-accuracy Event-based Underwater SLAM System**](https://arxiv.org/abs/2606.18951) | **Jun 2026** | ⭐ **fetched, and it carries the turbidity finding verbatim**: *"event cameras offer immense potential for underwater SLAM, existing **Time Surface (TS)-based methods prove highly unreliable when deployed underwater**"*. It claims *"competitive accuracy … compared to the state-of-the-art **event-based** method"* — ⛔ again, event vs **event**, never event vs frame |
| [**eStonefish-Scenes**](https://arxiv.org/abs/2505.13309) | May 2025 | sim-to-real-validated robot-centric event flow dataset for underwater vehicles |
| [**UEOF**](https://arxiv.org/abs/2601.10054) | **15 Jan 2026** | first **synthetic** underwater event-flow benchmark: ray-traced RGBD → events via v2e. 12 m 51 s, 13 714 frames, 4.94 G events, 960×540 and 1280×720; mean flow magnitude 6.1 px |
| [**Aquatic Neuromorphic Optical Flow**](https://arxiv.org/abs/2605.07653) | **13 May 2026** | self-supervised SNN flow from events. **0.134 M params, 2.31 G ops, 2.08 mJ, 26.49 ms latency** |

⭐ **Two findings in this new work kill the idea for us more cleanly than the absence of data did.**

**1. Learned event flow is *worse* than model-based event flow underwater.** UEOF's own
benchmark: model-based **EINCM reaches 1.01 px minimum AEE**, while learning-based **E-RAFT
peaks at 16.32 px AEE** — a **16×** gap, in the RAFT family, on underwater data. That is the
same direction as our Fourier-Mellin and EdgePoint2 results: **the learned method's public
ranking does not survive the move to water.**

**2. ⛔ The best event-flow result's ground truth is *frame* flow.** Aquatic Neuromorphic
Optical Flow reports AEE 1.40 / 1.20 / 0.41 / 0.68 px across AquaticVision, Aqua-Eye,
OceanLab and DAVIS-NUIUIED — and states that ground truth was **"generated using RAFT on
consecutive frames."** So the headline event-camera numbers measure **agreement with a
frame-based method**, which by construction **cannot show events beating frames**. Both
papers confirm it explicitly: UEOF *"does NOT directly compare event-based optical flow
against frame-based methods"*, and the neuromorphic paper *"does not provide a direct
quantitative comparison showing event cameras outperforming frame-based approaches
underwater."* This is `README.md` rule 3 — *truth, not agreement* — appearing as a defect in
the published literature, and it is why the field's claim is still unsubstantiated in 2026.

⛔ **Verdict, unchanged from 2026-09-22 and now better-founded: dead end for us.** It is new
hardware (a DAVIS346 event camera; ⚠ *"costs more than our whole camera budget"* was asserted
in an earlier draft **without a price — treat as unsourced**), a new processing stack, and
**as of Sep 2026 there is still no published number showing an event camera beating a frame
camera for underwater odometry** — four papers looked, four papers compare events to events.
⭐ And the June 2026 SLAM paper says the dominant event representation is *"highly
unreliable when deployed underwater"*. Do not re-open without a paper that compares events to
frames against *non-frame* ground truth.

---

## 6. Why optical flow fails underwater — what the literature actually says, and what it uses instead

⚠ **Section numbers follow the research brief, not this file's order.** §6 sits before §5
deliberately: the failure modes motivate the scale discussion. Read straight through.

### 6a. The named failure modes

The field's own list, in its own words:

⚠ **Attribution caveat, stated up front.** The three bullets below were returned by a search
summary spanning
[Knowledge Distillation for Feature Extraction in Underwater VSLAM (arXiv 2303.17981)](https://arxiv.org/abs/2303.17981),
[Real-time Monocular VO for Turbid and Dynamic Underwater Environments (arXiv 1806.05842)](https://arxiv.org/abs/1806.05842)
and the ISPRS caustics paper. **I could not fetch 1806.05842** (the PDF did not decode) and
did not confirm which sentence belongs to which paper. **Treat them as the field's framing,
not as verbatim citations of a named source.** Only the fourth bullet is from a fetched page.

- Dynamic lighting from **caustics and turbidity causes tracking failure**; **marine snow** —
  suspended sediment and organic detritus — occludes and distracts visual features.
- Underwater scenes carry few discriminant features and **repetitive patterns**: coral
  branches, animal holes in sand, algae, sand ripples in shallow water. ⭐ **A tiled pool
  floor is the extreme case**: perfectly repetitive, which defeats descriptor matching by
  **aliasing**, not by absence. That is a *different* failure from "no texture" and one XFeat
  does **not** fix — a learned descriptor matches the wrong identical tile as confidently as
  the right one. ⛔ **This is the strongest argument against the §3 recommendation and it
  belongs in the experiment's design**: over tiles, prefer the *period* (`tile_grating.py`,
  §5a-bis) to the *correspondences*.
- Light attenuation, scattering and HDR: *"These effects degrade the performance of
  conventional cameras, especially during fast motion or in low-light, turbid, or
  high-dynamic range (HDR) conditions"* ([UEOF, Jan 2026](https://arxiv.org/abs/2601.10054)).
- [Optical Ocean Recipes (arXiv 2509.20171)](https://arxiv.org/abs/2509.20171) — recipes for
  building realistic underwater datasets, i.e. the field's admission that **its methods are
  evaluated on data that does not reproduce these conditions.**

### 6b. What is used instead — and the honest summary is: *nothing that helps us*

1. **A DVL.** This is the real answer in the literature, and it is the one we do not have.
   See [`sota-vo-depth.md`](sota-vo-depth.md) §1 (DIVO) and §2 (DeepVL).
2. **Acoustics.** *"Optical flow-based motion vector analysis offers a label-free route to
   tracking in turbid or dark water, where optical cameras fail"*, positioning acoustic
   sensing as the complement to optical
   ([arXiv 2412.20085](https://arxiv.org/abs/2412.20085)) — ⚠ paraphrased from a search
   summary, not a fetched abstract. New hardware; not a candidate either way.
3. ⭐ **Caustics suppression as a per-anchor pre-processing stage — WE ALREADY SHIP ONE, and
   an earlier draft of this section was wrong to imply otherwise.** `caustics` is a launch
   switch bound to `caustic_suppression` in
   `src/mongla_vision/mongla_vision/flow/flow_node.py`. What ships, from `measured-bars.md`:
   **grey erosion 7×7, applied per anchor only when the frame is caustic**; a detector of
   `top-hat(9×9) mean / frame mean` with threshold **0.07** (caustic-free Mirpur indoor and
   final_run max **0.051**; RoboSub downward-in-sun min **0.090**, up to 0.21), costing
   **4.73 ms per 640×480 on the Pi**; and a refusal on median patch NCC below **0.71, in sun
   only**. ⛔ **Its measured limit is exactly our problem case**: *"NOT fixed: a floor with no
   dark texture of its own (slalom, plain) still reports the waves."* Grey **opening** was
   tried and is worse. See also [`sota-vo-depth.md`](sota-vo-depth.md) §7, *"Caustics — our
   measured failure mode"*. ⭐ **So the bar a learned caustics remover must clear is not
   zero — it is a shipped, measured, 4.73 ms detector+eroder with a stated failure mode.**
4. **Learned caustics removal as a pre-processing stage.**
   [Self-Supervised Underwater Caustics Removal and Descattering via Deep Monocular SLAM](https://link.springer.com/chapter/10.1007/978-3-031-72907-2_13)
   (Springer, 2024/25) is the current representative. ⚠ **I could not fetch its numbers** —
   the Springer link is paywalled behind an IdP redirect. It is listed so it is not
   re-discovered, **not** as a supported recommendation.
   ⛔ **And our own standing measurement refuses this whole class anyway**: image
   preprocessing was measured here in **17 configurations across four props and three
   venues, never positive, and on the gate it destroyed 95 % of detections** (`CLAUDE.md`
   §4). A learned caustics remover is a preprocessing stage. **The bar it must clear is that
   result, and no paper above reports on a Pi 5.**
5. **Polarisation imaging for turbidity** — [division-of-focal-plane polarization for 3D
   structured light (ISPRS J., 2025)](https://www.sciencedirect.com/science/article/abs/pii/S0924271625002850).
   New sensor + projector. Not a candidate; noted under §5.
6. **Better learned features** — the knowledge-distillation VSLAM line above. ⭐ **This is the
   only *new* option here that is software-only and runs on our hardware**, and it is the same
   family as §3's XFeat conclusion. The literature and our own Mirpur bench agree here,
   which is the strongest convergence in this document.

⭐ **The convergence, stated plainly:** the published answer to *"optical flow fails
underwater"* is **a DVL or a sonar**. Where the answer is allowed to be software-only, it is
either **caustic-specific preprocessing — which we already ship, with measured bars and a
stated failure mode** — or **a learned local feature replacing a classical corner detector**,
which is exactly §3 and exactly the one experiment worth running.

---
## 5. Scale recovery — a monocular downward camera is a *direction* sensor until something supplies height

*(Brief item 5; it follows §6 here because §6 motivates it.)*

[`sota-vo-depth.md`](sota-vo-depth.md) §5 already enumerates the five options (5a pressure +
floor plane — what we do; 5b stereo; 5c laser scalers / structured light; 5d differential
pressure; 5e known-size objects). **Not re-argued here.** Three things that document does not
have:

### 5a. ⭐ What the field actually does: exactly what we do

The literature converges on our own architecture, which is worth stating because it means
**we are not behind here**. ⚠ **The two sentences below come from a search summary over
[arXiv 2608.26932](https://arxiv.org/abs/2608.26932) and neighbours and were not confirmed
against a fetched abstract — paraphrase, not quotation** (see the unverified list):

- metric scale is recovered from auxiliary sensors — **DVL or sonar altitude when available,
  otherwise pressure-derived depth**, with an IMU fallback;
- in planar, low-altitude operation monocular VO is *inherently noisy and scale-ambiguous*.

The *aerial* literature states the mechanism plainly: downward translational optic flow is
scaled by the current estimated **flight height** and integrated. That is
`v = h·(flow_px/dt − f·ω)/f` — **the aerial method and ours are the same method**, and it is
the standard PX4 arrangement ([PX4 optical flow](https://docs.px4.io/main/en/sensor/optical_flow)):
flow sensor + rangefinder, with **the rangefinder non-optional**.

⚠ **Our version has a hole the published one does not**: we have **no altimeter**. Pressure
gives *depth below surface*, not *height above floor*, so 5a is pressure **plus a flat-floor
assumption plus a known pool depth**. ⭐ In a competition pool that assumption is excellent
and is why this works at all; over a sloped or stepped floor it is a silent scale error. Our
downward-camera work validated it indirectly — implied heights **0.72 / 0.69 / 0.70 m against
a 0.72 m tape** — which is a *consistency* check of the assumption, **not** an independent
altitude measurement.

### 5a-bis. ⭐ We already have a second, independent height instrument — and it is not in `sota-vo-depth.md` §5

`src/mongla_localization/mongla_localization/tile_grating.py` recovers height from the
**spatial period of the pool's floor tiles**: `h = f_px · tile_m / pitch_px` — the same
pinhole relation, run backwards, with the tile pitch as the known length. `usable_height_m()`
bounds where it works (`MIN_PERIOD_PX`, frame size). There is a companion
`pool_lines.py`, whose docstring notes lane lines are a *"worldwide published standard, so
unlike `tile_m` this instrument needs no"* site calibration. ⭐ **This is §5e (known-size
objects) already implemented, as a launch switch (`tile_m`, `lane_lines`), and it is an
independent cross-check on the pressure + flat-floor assumption** — exactly the redundancy
the champion doctrine asks for. ⚠ Its in-water accuracy is not quoted here; read
`measured-bars.md` and the `mongla_localization` package page before relying on it.

⚠ **Correction to an earlier draft of this document**: `vision.mixer_aware` is **not** about
floor tiles. From `vision_tunables.py`: *"on srot, fit each vision frame to the mixer
yaw-first"*. It has nothing to do with repetitive-pattern aliasing.

### 5b. Online scale estimation from IMU + pressure alone — the software-only option

[Monocular Odometry for Underwater Vehicles with Online Estimation of the Scale
Factor](https://www.researchgate.net/publication/328410139) estimates the scene's scale
factor online *"thanks to the combined measures of a low-cost IMU and a pressure sensor"* —
i.e. treating scale as a state rather than a constant, observable through the accelerometer
during depth changes. ⭐ **This is the only scale option in the whole family that costs no
hardware**, and our estimator is already the right shape for it: the RI-EKF carries
SE₂(3) + IMU biases and could carry a scale state. ⚠ I could not fetch a quantitative
accuracy figure — the source is a ResearchGate landing page. **Listed as a direction with a
citation, not as a validated result.**

⛔ **The honest caveat**: scale-from-IMU is observable only under *acceleration*. A vehicle
holding depth and translating at constant velocity — our nominal survey condition — excites
it weakly or not at all. The observability gate in `measured-bars.md` §13.3 is the right
machinery and is itself *"validated only against synthetic noise"*. **Do not add a scale
state without an observability gate that refuses it when unexcited.**

### 5c. What the Hailo could actually contribute to scale — the one place the NPU has a role

The Hailo-8 Model Zoo has **no optical flow and no keypoint-matching category** (§1a), but it
*does* have depth. Published numbers, DFC v2.19.0, PCIe Gen3 ×4, batch 1:

| Model | Task | Input | Accuracy | **FPS, batch 1** |
|---|---|---|---|---|
| `fast_depth` | monocular depth | 224×224×3 | RMSE **0.61** (hardware) | **2519** |
| `scdepthv3` | monocular depth | 256×320×3 | RMSE **0.48** (hardware) | **929** |
| `stereonet` ⭐ | **stereo** depth | 368×1232×3 | Float EPE 8.22 / **HW EPE 10.3** (KITTI Stereo 2015) | **10.7** (11.6 at batch 8) |

⚠ **Read those FPS against §0b.** 2519 FPS is 0.4 ms — **not a rate a vehicle can have**,
when our fastest measured *complete* loop is 7.99 ms. These are the same `hw_only`-flavoured
figures our own §14 retraction was about. ⚠ The two depth models use small inputs (224×224,
256×320) against our detector's 640×640, so their round trip should be smaller than 7.99 ms
and a real ceiling cannot be quoted without measuring it — **stating one would repeat the
error §0b records.** The one number that is *not* flattered is StereoNet: **10.7 FPS =
93 ms/frame**, an order of magnitude above any plausible round trip, so it is genuinely
compute-bound — and its **hardware EPE 10.3 px on
KITTI** is poor. ⛔ **StereoNet is not a scale source for us.** It also needs a second
synchronised camera on a rigid baseline, i.e. §5b of the other document, i.e. hardware.

For the monocular depth models: ⛔ **they are trained on air imagery and `sota-vo-depth.md`
§4 already measured what water does to that class.** Read that section before reconsidering;
nothing found here contradicts it. The refraction trap (§4 of that document, and
`optics.py`) applies to every one of them.

⭐ **Verdict on scale: we are already doing what the field does, and the binding constraint is
an altimeter, not an algorithm.** The only software-only improvement with a citation is 5b
(scale as an estimated state), and it is observability-limited in exactly our flight regime.

---
## Master comparison — every candidate, judged on our hardware

| Candidate | (a) Runnable here? | (b) Better than sparse LK **for us**? | (c) Cost to adopt |
|---|---|---|---|
| **Sparse LK + RANSAC** (incumbent) | ⭐ yes — **12.34 ms** Pi 5 CPU | baseline: 1.09 cm / 30 cm, 0.57 mm/s floor, 3.4 % scale | — |
| ⭐ **XFeat frame-to-frame** | ⭐ **yes, measured: 33.1 ms @ 320×240 on the Pi with the stack up** | **plausibly, in the texture-poor regime only** — 3/4 vs ORB's 1/4 on octagon; 4/4 vs 0/4 on murky Mirpur. ⛔ **but those clips are FORWARD-camera prop footage, and the comparison is against ORB, not Shi-Tomasi.** **Zero** expected gain nominally; XFeat's own matcher noise here is **1.55 px** vs LK's 0.006–0.40 px against truth | ⭐ **low** — export, ONNX, matcher and bench protocol all already exist |
| **XFeat\*** semi-dense (anchor rung) | yes — 1.4× XFeat's time | ⭐ **+111 % inliers, +10.2 Acc@10°** — but for the *anchor*, not velocity | ⭐ **already ranked: `SOTA-GAPS.md` G-18, value 15.0** |
| **SuperPoint + LightGlue** | ⛔ **no — LightGlue is 0.31 pairs/s ≈ 3.2 s per pair on CPU**, measured here | ⛔ ~100× too slow | ⛔ rejected |
| SuperPoint + **SuperGlue** | ⛔ **no on Hailo** — einsum + `ReduceLogSumExp` unsupported; 3 chained HEFs at **70.8 % recall**; one compile ran 7 h 34 m and failed | — | ⛔ prohibitive |
| ALIKED | unknown; ⚠ deformable conv is a GridSample consumer | unknown | unknown |
| EdgePoint2 | yes | ⛔ **no — measured, lost 1–4 inliers vs XFeat's 86–155 on our clips** | ⛔ rejected |
| **NeuFlow v2** | ⛔ no — GridSample; >20 FPS figure is a **Jetson Orin Nano GPU** | ⛔ no — our error is scale bias, not flow error | ⛔ n/a |
| **RAFT / RAFT-small** | ⛔ no — GridSample; <0.2 inf/s per iteration on mobile | ⛔ no | ⛔ n/a |
| **GMFlow / FlowFormer** | ⛔ no — GridSample + einsum attention | ⛔ no | ⛔ n/a |
| **DPVO / DPVO-QAT++** | ⛔ no — CUDA kernels, 4.9 GB GPU (RTX-3090) | ⛔ also up-to-scale, so it does not remove our constraint | ⛔ n/a |
| **DROID-SLAM** | ⛔ no — 8 GB frontend / **24 GB** backend; OOMs on an 8 GB laptop GPU | ⛔ | ⛔ n/a |
| **TartanVO** | ⛔ no — 40 ms on a GTX 1080 | ⭐ its **synthetic→real generalisation with no finetuning** is the property worth watching | ⛔ n/a this season |
| **Event camera** | ⛔ new hardware | ⛔ **no published event-vs-frame underwater number exists** (4 papers, all event-vs-event), and a Jun 2026 SLAM paper calls Time-Surface methods *"highly unreliable when deployed underwater"* | ⛔ dead end |
| `stereonet` on Hailo | technically yes — **10.7 FPS = 93 ms**, compute-bound | ⛔ HW EPE **10.3 px** on KITTI; needs a second synchronised camera | ⛔ hardware + poor accuracy |
| Monocular metric depth on Hailo (`fast_depth`, `scdepthv3`) | yes in principle; ⚠ the quoted 2519/929 FPS are chip-only, real loop rate unmeasured | ⛔ see [`sota-vo-depth.md`](sota-vo-depth.md) §4 — measured, water breaks this class | ⛔ |
| **Scale as an EKF state** (IMU + pressure) | ⭐ yes, software only | ⚠ unquantified; ⛔ unobservable at constant velocity — our nominal regime | medium — needs an observability gate first |

## The recommendation — and it is smaller than this document's length implies

⛔ **First, the honest ranking. Nothing this sweep found outranks two moves already on the
board**, and a research pass that ends by proposing a *third* priority should say so:

| rank | move | where it comes from | value |
|---|---|---|---|
| 1 | **G-19** — gate flow health on the KLT Hessian we already compute | `SOTA-GAPS.md`, existing | **20.0** |
| 2 | **G-18** — XFeat → XFeat\* in the anchor rung | `SOTA-GAPS.md`, existing | **15.0** |
| 3 | ⭐ **step 0 below** — Shi-Tomasi vs XFeat on the downward slides | **new, from this sweep** | cheap, and it is a *measurement*, not a component |
| 4 | XFeat frame-to-frame as a second velocity source | new | only if step 0 says yes |

⭐ **G-19 deserves the emphasis.** §6 establishes that the field's software-only answer to
caustics and texture loss is either preprocessing (we ship it, with a measured failure mode)
or better features (§3, marginal). ⛔ **The thing that actually protects a run is the sensor
refusing when it should** — CLAUDE.md §8.6: *"the recurring defect in this codebase is a
plausible number standing in for an absent measurement."* A flow-health gate is that, and it
is already ranked first.

⭐ **Step 0, and it is nearly free: bench Shi-Tomasi against XFeat for keypoint and inlier
count on the existing downward 30 cm-slide recordings.** Everything in §3 rests on a
cross-domain inference (forward prop footage → downward floor) and on a detector that was
never benched (**Shi-Tomasi, not ORB**). One run settles both, and it may well end §3.

**Then, only if step 0 favours XFeat:** bench XFeat frame-to-frame similarity as a second
velocity source against known-truth slides — one textured floor, one bare floor, one under
caustics.

Everything else in this document is either architecturally impossible on a Hailo-8
(`grid_sample`, verified in NeuFlow v2's and GMFlow's own source), needs a CUDA GPU, needs
hardware we do not have, or has already been measured and lost here. The XFeat move is the
only one where the model is already exported and the rate is already measured on the vehicle.

⛔ **Its falsifier is stated in advance (§3d): if it does not beat LK on the bare-floor
slide, it has no case at all** — because §3c shows the incumbent already returns 0.006–0.40 px
against truth in the nominal regime, and §6a shows a learned descriptor is *worse* than a
period estimator over identical tiles.

⭐ **And the largest number anywhere near this subsystem is not in this document at all**: the
ROS graph costs **45 %** of the vision rate (98 Hz standalone vs 53.9 Hz through the graph,
`measured-bars.md` §14.3). ⛔ **That dwarfs every millisecond argued over above.**

⭐ **The most useful output of this sweep is arguably not a move at all — it is three
closures**: dense flow on Hailo-8 is architecturally dead (`grid_sample`, verified in NeuFlow
v2's and GMFlow's own source, plus a vendor "no plan to add support"); LightGlue is dead on
our CPU (**3.2 s per pair**, our own measurement); and the event-camera case is still
unsubstantiated after four papers (all event-vs-event). **Three directions that will not need
researching again.**

---

## Claims I could NOT verify

1. ⛔ **Shi-Tomasi fails where ORB fails.** §3c's central inference. ORB was measured;
   Shi-Tomasi (our LK front-end) was **not**. **Load-bearing, and reasoned rather than
   measured.**
1b. ⛔ **The XFeat clips are the wrong camera.** *torpedo*, *gate*, *octagon* are props, i.e.
   **forward**-camera footage. The velocity sensor is the **downward** camera over a floor.
   `measured-bars.md` explicitly separates the two cameras on caustic statistics, so the
   domains are known to differ. **Items 1 and 1b close together with one run on the existing
   downward slide recordings, and nothing in §3 should be acted on before that.**
2. **XFeat frame-to-frame velocity accuracy.** No number exists, ours or published. §3's
   33.1 ms is an *anchor* (frame-to-reference) timing, and the rung it serves runs at 3 Hz by
   design — ⚠ **so 30 Hz has never been demanded of it in anger.**
3. **Descriptor matching survives yaw ≥ 1.128 rad/s better than LK.** Labelled speculation
   in §3c. No source, no measurement.
4. ⭐ **CLOSED, not unverified: LightGlue on CPU.** `sota/vision.md` has it — **0.31 pairs/s,
   ≈ 3.2 s per pair.** ⚠ The methodology behind that figure was not re-derived here; it is
   trusted as a project measurement. The published 35 %-faster-than-SuperGlue and
   33 %-from-pruning figures remain relative, on unnamed hardware.
5. **ALIKED on Pi 5 or Hailo-8.** No number found. The deformable-conv/GridSample concern is
   architectural inference, not a confirmed compile failure.
6. **Caustics-removal quantitative results.** The Springer chapter is paywalled behind an IdP
   redirect; I have the title and venue only, no numbers.
7. **The online scale-factor paper's accuracy.** ResearchGate landing page only.
8. **GMFlow and FlowFormer edge runtimes.** No edge number found for either; the "⛔ no"
   rests on the GridSample/einsum architecture argument, which is sound but is an inference
   about *those* models rather than a measured compile attempt.
9. **Whether `hailomz` would actually compile XFeat.** Not attempted. §3b argues it is not
   worth doing (the Pi is 72.8 % idle; it would spend detector chip time to buy a resource we
   have in surplus) — but that is a *value* argument, not a compile result. Unverified, not
   disproven.
9b. **The Hailo round trip as a function of tensor size.** Our 7.99 ms is one input shape
   (640×640). Every "is it fast enough on the chip" judgement here would be sharper with a
   round-trip-vs-input-size sweep, which is a half-hour bench.
9c. **SuperPoint's "~300 FPS on Hailo-8"** — a forum figure with no methodology. Read as
   chip-only until someone reports a complete loop.
9d. **The §5a and §6a and §6b quotations** — see the inline ⚠ caveats. Paraphrased from
   search summaries; the underlying abstracts were not individually fetched, and
   arXiv 1806.05842's PDF did not decode. **Nothing in this document's verdicts rests on
   them.**
9e. **`tile_grating.py`'s in-water accuracy** (§5a-bis). The code and the relation were read;
   no measured error was looked up.
10. **Which LK number is the baseline.** `measured-bars.md` carries two: **12.34 ms** for the
    full *LK + RANSAC similarity* (4-DOF: dx dy yaw scale) in the truth-warp bench, and
    **8.0 ms** for the lock ladder's *fast rung*. Read together they are different code paths,
    not a contradiction — but ⚠ the XFeat experiment must name which one it is replacing
    before it claims a win, because 33.1 ms is 2.7× the first and 4.1× the second.

11. ⚠ **This document's own process defect, recorded.** Three claims were written from web
    sources before the project's own files were checked, and all three were wrong or
    redundant: LightGlue's CPU rate (`sota/vision.md` already had it, and it is fatal),
    caustics preprocessing (`flow_node.py` already ships a measured one), and the XFeat
    benchmark table (`sota/vision.md` already had it). ⛔ **The lesson is the one
    `measured-bars.md` keeps re-teaching: check the tree before the literature.** A fourth
    claim — `mixer_aware` as a tile-aliasing mitigation — was invented and is corrected
    inline in §5a-bis.
12. ⭐ **What this sweep did NOT look at, and a reader should not assume it ruled out:**
    IMU-only dead reckoning between flow fixes, acoustic/sonar velocity, DeepVL-style learned
    velocity from thrust (that is `sota-vo-depth.md` §2 and `SOTA-GAPS.md` G-17), and
    anything about how the RI-EKF *weights* a velocity source. **The question asked was
    narrow: better camera-derived velocity. The answer is narrow too.**
