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

⛔ **The decisive framing** (advisor, and it survives the numbers below): our 1.09 cm worst
error **is** our 3.4 % scale bias — 30 cm × 3.4 % = 1.02 cm of the 1.09. Scale bias comes
from **height, FOV and refraction**, *not* from the flow algorithm. **A better EPE on Sintel
or KITTI therefore cannot improve our 1.09 cm.** Any candidate has to win on a *failure
regime*, not on nominal accuracy:

1. does it produce usable motion on a **bare pool floor**, where LK gets no tracks?
2. does it **reject caustic** motion (which is real image motion, not sensor noise)?
3. does it survive **yaw ≥ 1.128 rad/s**?

⛔ **The Hailo cost floor.** Measured here: batch-1 PCIe round trip **~9.3 ms** dominates. So
*no* model hosted on the Hailo-8 beats ~10 ms/frame end to end. LK already runs in 12.34 ms
on the CPU, on a core the Hailo does not free. **The Hailo can at best halve this, and only
if the model itself is free.** That is the whole economic case, and it is thin before we
start.

---

<!-- APPEND LOG BEGINS — entries are written as each source is fetched -->
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

> **Any model moved to the Hailo-8 costs ≥ ~9.3 ms before it computes anything.** A flow or
> feature network there cannot beat ~10 ms/frame. LK is 12.34 ms on a CPU core that the
> Hailo does not free — and the detector already owns the chip. The ceiling on the entire
> "put flow on the NPU" idea is therefore **a ~20 % saving, purchased with the detector's
> rate.** ⛔ That is not a trade worth making, and it is settled before any paper is read.

---
## 1. Dense optical flow — and the Hailo question, answered

### 1a. ⭐ Can a dense-flow net run on a Hailo-8? **No — and it is an architectural "no", not a porting gap.**

`sota-vo-depth.md` closed this section with *"no one has published a flow model compiled for
Hailo-8 that I could find"* and left it on the unverified list. **It can now be closed
properly**, from Hailo's own mouth.

Every modern dense-flow architecture — RAFT, GMFlow, FlowFormer, NeuFlow v1/v2 — is built
around **iterative correlation-volume lookup with sub-pixel bilinear resampling**, which in
ONNX is `GridSample`. From the RAFT description: the lookup operator *"extracts correlation
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
PCIe round trip **per refinement iteration** — RAFT uses 12–32. At our measured **~9.3 ms
per round trip** that is **110–300 ms per frame** before any arithmetic. Corroborating:
[grid_sample problem](https://community.hailo.ai/t/grid-sample-problem/17078).

⛔ **Verdict, settled: no RAFT-family dense flow will ever run usefully on our Hailo-8.** Not
"nobody has tried" — the vendor has declined to support the primitive the architecture is
built on. **Do not re-open this without a Hailo DFC release note that names GridSample.**

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
