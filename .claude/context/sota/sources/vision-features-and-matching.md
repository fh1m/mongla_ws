# SOTA: local features, matching and place recognition for an edge AUV

Compiled 2026-09-23. Every claim carries a source link and a number, **and the hardware the
number was measured on**. Where a number could not be found in the source it is listed under
**Claims I could NOT verify** at the bottom rather than guessed.

Each item is labelled **RUNS TODAY** (measured on our hardware or our footage) ·
**BUILT, NEVER FLOWN** · **BLOCKED** · **NO** (recommended against, with the reason).

---

## 0. Our baseline, for comparison throughout

Everything below is judged against what the vehicle already does. Copied from the code, not
from memory:

**XFeat as the anchor rung, forward camera only** (`mongla_vision/anchor/xfeat_onnx.py`,
`tools/xfeat_export.py`). ONNX + numpy, no torch. Measured on the Pi 5 **with the vision
stack running**:

| input | 1 thread | 3 threads |
|---|---|---|
| 640x480 | 145.6 ms | 88.0 ms |
| **320x240** | **33.1 ms** | **18.2 ms** |

320x240 is the shipped default because it was *verified to keep the lock*, not because it was
faster: every murky archive clip still reaches a trusted homography at that size (56–260
inliers). The export is **fixed-shape by design** — another size throws rather than resizing,
because a silent resize changes the pixel scale of every pose the anchor reports.

**Why XFeat and not ORB — measured on our own footage** (same docstring). Reference snapped at
40 % of each clip, matched frame-to-reference at +1/3/5/8 s, `USAC_MAGSAC` homography,
≥15 inliers to trust it:

| clip | ORB ref kp | ORB | XFeat |
|---|---|---|---|
| Mirpur torpedo (murky) | 71 | 0/4 | **4/4** (350→100 inliers) |
| Mirpur torpedo_1 | 7 | 0/4 | **4/4** (301→130) |
| Mirpur gate | 111 | 3/4 | **4/4** (753→682) |
| octagon (low texture) | 1205 | 1/4 | 3/4 |
| torpedo (clear control) | 1260 | 4/4 | 4/4 |

ORB finds **seven** keypoints in an entire Mirpur frame. **EdgePoint2**, published as "2x
faster than XFeat with competitive IMC2022 results", was benched on the same clips and **loses
on every murky clip** (torpedo_1: XFeat 4/4, EP2-S64 1/4). That is the single most important
result in this document: *IMC2022 is clear natural imagery; turbid water is a different domain
and the published ranking does not survive the move.*

**Downward camera**: sparse Shi-Tomasi + LK + RANSAC **similarity** (4-DoF: tx, ty, theta,
scale), **12.34 ms/pair** on the Pi 5, IMU-de-rotated. Worst error 1.09 cm over three 30 cm
slides; scale 103.4 % of truth ⇒ a **3.4 % trajectory-drift floor**; noise floor 0.57 mm/s.
Needs a known height. Fails on bare floor, sun caustics, and yaw ≥1.128 rad/s.

**Hailo-8 budget**: yolov8-class detector at batch 1 = **98 Hz / 10.2 ms end-to-end**, of
which the **PCIe round trip is ~9.3 ms**. That number is the ceiling on every "put another
network on the Hailo" idea below: a second network is a second round trip, so the floor for
*any* second Hailo model is ~9.3 ms of latency plus whatever it steals from the detector's
duty cycle. A model that takes 2 ms of NPU time still costs ~11 ms wall clock.

**Already built on the geometry side, and it matters for Q5**:
`anchor/geometry.py` already decomposes the anchor homography to plane tilt/normal;
`anchor/pose.py` already does metric 6-DoF via IPPE + SQPnP;
`mongla_localization/tile_grating.py` reads the tiled floor as a 2-D grating (FFT phase
demodulation → heading mod 90°, height, sub-tile phase);
`mongla_localization/pool_lines.py` reads a lane line (heading mod 180°, lateral offset).

---

## 1. XFeat — the full capability set, and whether it runs on the Hailo

[XFeat: Accelerated Features for Lightweight Image Matching, CVPR 2024](https://arxiv.org/abs/2404.19174)
· [verlab/accelerated_features](https://github.com/verlab/accelerated_features) (Apache 2.0).

### 1.1 What the model actually offers

Three modes, not one. We ship **only the first**.

| mode | what it is | what it costs |
|---|---|---|
| **sparse** | top-k keypoints + 64-D descriptors, nearest-neighbour or MNN matching | the baseline; what `xfeat_onnx.py` does |
| **XFeat\*** (semi-dense) | matches at 1/8 resolution over ~10 000 features, then an MLP **match-refinement module** predicts a per-match (x, y) offset back to full-resolution pixel accuracy | paper: **+11 % inference overhead** vs plain NN matching for ~10 000 descriptors ([paper §3](https://arxiv.org/html/2404.19174v1)) |
| **XFeat + LighterGlue** | a distilled [LightGlue](https://github.com/cvg/LightGlue) retrained on XFeat's 64-D descriptors; the repo README calls it "a lighter version of LightGlue with fewer parameters and approximately **three times faster** than the original" | a whole second network (attention/transformer), see §3 |

Accuracy, from the paper. Relative-pose AUC:

| | MegaDepth-1500 @5/10/20° | ScanNet-1500 @5/10/20° |
|---|---|---|
| XFeat (sparse, 4096 kp) | 42.6 / 56.4 / 67.7 | 16.7 / 32.6 / 47.8 |
| **XFeat\* (semi-dense, 10k)** | **50.2 / 65.4 / 77.1** | **18.4 / 34.7 / 50.3** |

LighterGlue on MegaDepth-1500 (repo README): 0.444/0.610/0.746 AUC@5/10/20 at 640 px "fast",
0.564/0.710/0.819 at 1024 px "accurate".

**Speed, on the paper's own CPU (Intel i5-1135G7 @ 2.40 GHz, VGA 640×480):**

| method | FPS | vs SuperPoint |
|---|---|---|
| ORB | 44.3 ± 1.18 | 14.8× |
| **XFeat** | **27.1 ± 0.33** | 9.0× |
| XFeat\* | 19.2 ± 1.12 | 6.4× |
| ALIKE | 5.3 ± 0.33 | 1.8× |
| SuperPoint | 3.0 ± 0.07 | 1.0× |
| DISK | 1.2 ± 0.01 | 0.4× |

⚠ **That i5 is not our Pi.** Our own measurement of the same network at the same VGA
resolution is **145.6 ms (1 thread) / 88.0 ms (3)** = 6.9–11.4 FPS, i.e. the Pi 5 is
**2.4–4× slower than the paper's laptop CPU**. Read every laptop-CPU figure in this document
through that factor.

### 1.2 Can XFeat run ON a Hailo-8? — **YES, and somebody has already compiled it.**

This is the strongest finding in the document, and it is primary evidence: I downloaded and
inspected the artifacts rather than reading a claim about them.

[guyp98/accelerated_features](https://github.com/guyp98/accelerated_features) — the fork named
in the [Hailo community thread "XFeat Compilation"](https://community.hailo.ai/t/xfeat-compilation/14926)
(asked 2025-05-23, answered by Hailo staff 2025-06-16) — ships **working Hailo-8 HEF files**:

```
hailo_files/model_224_320/..._224_320_sim.hef      2 595 744 bytes
hailo_files/model_480_640/..._sim.hef              3 141 640 bytes
```

The shipped `onnx_to_hailo.sh` is the whole recipe and it is three lines:

```sh
hailo parser onnx  ..._224_320_sim.onnx \
  --start-node-names /block1/block1.0/layer/layer.0/Conv \
                     /keypoint_head/keypoint_head.0/layer/layer.0/Conv \
                     /skip1/skip1.0/AveragePool \
  --end-node-names OUTPUT_1 OUTPUT_2 OUTPUT_3
hailo optimize ... --calib-set-path ../calibsets/combined_array_transposed_....npy
hailo compiler ..._optimized.har
```

**What had to come off the graph, verified by loading the ONNX myself:**

The Hailo-bound graph is 480×640×3 in, three outputs (64×60×80 feats, 64×60×80, 1×60×80),
opset 13, and its op histogram is:

```
Conv 27 · BatchNormalization 23 · Relu 23 · Add 3 · Resize 2 · AveragePool 1
InstanceNormalization 1 · ReduceMean 1 · ReduceL2 1 · Div 1 · Clip 1 · Expand 1
Sigmoid 1 · Softmax 1 · Slice 1
```

That is **a plain CNN with a handful of awkward edges**, which is exactly the shape the Hailo
likes. The graph has **88 nodes**; the split-off "head" that stays on the CPU contains
**exactly two of them, at positions 0 and 1**:

```
ReduceMean 1 · InstanceNormalization 1     IN (1,3,224,320) -> OUT (1,1,224,320)
```

i.e. the *only* thing the Hailo cannot swallow is XFeat's **input normalisation** (RGB→mean→
instance-norm). Everything else — the whole backbone, both heads — compiles.

⚠ **Read the filename the right way round.** `without_pixel_unshuffle_normilize_softmax_slice`
means *pixel_unshuffle removed, normalise+softmax+slice **added in***. Stock XFeat does
`F.normalize` and the 65-channel softmax in its **Python wrapper**; this fork pulled them into
the graph. I verified the node positions: `ReduceL2`/`Clip`/`Expand`/`Div` sit at 64–67 and
`Softmax`/`Slice` at 86–87 — and `--end-node-names` is `OUTPUT_1 OUTPUT_2 OUTPUT_3`, which
*are* the graph outputs. **Nothing is stripped at the tail. Those ops are inside the HEF.**
That is also why `OUTPUT_2` has 64 channels and not 65: the dustbin is already dropped on-chip.

⛔ **This sharpens the INT8 risk rather than softening it.** In the fork's HEF the 64-D
descriptor is **L2-normalised in INT8, on-chip**, and the keypoint map is **softmaxed in
INT8**. A softmax over 65 channels and an L2 normalise are both operations whose whole job is
to resolve small differences; 8-bit is the worst place to do them.

**What our re-export would look like, and it is more than two lines.** Our
`tools/xfeat_export.py` already stops at the three raw heads (`keypoints` is **65 channels**,
softmax/slice live in our numpy) and our input is already 1-channel — so keeping normalise and
softmax **on the host in float** is our design choice and is strictly safer than the fork's.
But the fork's three `--start-node-names` mean the HEF takes **three inputs**: the normalised
image into `block1`, the same image into the `skip1` AveragePool, and the **64-channel
`unfold2d`/pixel-unshuffle tensor** into `keypoint_head`. So `pixel_unshuffle` moves to numpy
too, and the host sends roughly **twice the input bytes** over the same ×1 PCIe link that is
already the bottleneck. Budget accordingly.

**(a) Rate.** Unknown — **no measured Hailo-8 FPS for XFeat exists anywhere I could find**,
not in the fork, not in the thread, not in the model zoo. What we *can* bound: the network is
~3 MB and conv-only, so NPU compute will be small; the binding constraint is the **~9.3 ms
PCIe round trip we measured**. A realistic Hailo XFeat at 640×480 is therefore **~10–13 ms**
against our measured **88.0 ms (3 threads) on CPU** — call it **6–8× at VGA**. At our shipped
320×240 the comparison is **~10 ms against 18.2 ms**, i.e. **under 2×**, and that 2× is bought
by taking a PCIe slot and NPU duty cycle away from the detector that is currently at 98 Hz.

**(b) What it buys.** Two things, and they are not the same thing:
1. *Cheaper anchor* — frees ~18 ms of CPU per anchor tick. Real but modest.
2. **VGA-rate XFeat**, which is different in kind. Today VGA costs 88 ms so we run 320×240 and
   accept the keypoint count that implies. At ~11 ms, **full-resolution XFeat on the downward
   camera as well** becomes arithmetically possible — that is the interesting door, not the
   CPU saving.

**(c) Cost, honestly.**
- The HEFs are a **third-party artifact, last real commit 2024-07-22** (the 2025-06-16 commit
  only adds `split_onnx.py`). They are RGB 3-channel; ours is grayscale. Treat them as a
  worked example, not as a drop-in.
- ⛔ **INT8 is the risk, and it lands exactly on the thing we care about.** The Hailo quantises
  to INT8/UINT8. XFeat's output is a 64-D **L2-normalised descriptor**; quantising the
  descriptor head is the single most plausible way to lose murky-water matching while every
  health metric still looks fine. **No Hailo XFeat verdict is admissible until the 4/4 murky
  archive-clip table above is re-measured through the HEF.** Our own history says this loudly:
  EdgePoint2 was "2× faster with competitive IMC2022 results" and lost 4/4 → 1/4 on our water.
- The calibration set must be **our turbid footage**, not theirs. A calib set of clear natural
  images will quantise the dynamic range of a low-contrast murky frame into nothing.
- The Dataflow Compiler is x86-only; this is a desk-side build step, not something the Pi does.

**Verdict: BUILT, NEVER FLOWN — by someone else, not by us. Worth one afternoon of the
compile, gated behind re-running the murky clip table. Do not put it on the critical path.**

### 1.3 What XFeat gives beyond re-finding a lost target

We use it as the anchor rung. The same correspondences already support more, and part of it is
already built:

| product | status here | notes |
|---|---|---|
| **homography** → tx, ty, θ, scale | **RUNS TODAY** (`anchor/anchor.py`) | the lock |
| **plane normal / tilt** | **RUNS TODAY** (`anchor/geometry.py`, `decomposeHomographyMat`) | measured *coarse*: 4.7° median / 17.5° p90 reference-dependent |
| **metric 6-DoF pose** of a known-size planar target | **RUNS TODAY** (`anchor/pose.py`, IPPE interval + SQPnP point) | needs a known patch size in metres |
| **relative pose** (essential matrix, 5-point) | not built | gives rotation + translation *direction only*, no scale, and on a low-parallax underwater approach the E-matrix is badly conditioned. **Not worth it** — we already get a better-conditioned answer from the plane, because everything we look at *is* a plane |
| **place recognition** | **no** | XFeat is a *local* feature. It has no global descriptor. Matching every frame against every keyframe is O(N²) and there is no aggregation head. See §4 |

**The honest summary of §1.3: XFeat's extra modes (XFeat\*, LighterGlue) buy relative-pose AUC
on MegaDepth. We do not estimate relative pose. We estimate a homography to a planar target
and we already clear ≥15 inliers 4/4 on our worst footage. There is no measured problem for
XFeat\* or LighterGlue to solve on this vehicle — and LighterGlue is a transformer we would
have to port to numpy or a second Hailo round trip. Recommendation: NO, for now.**

---

## 2. DINOv2 / DINOv3 — dense semantic features

[DINOv2 (arXiv 2304.07193)](https://arxiv.org/abs/2304.07193) ·
[DINOv3, released 2025-08-13 (arXiv 2508.10104)](https://arxiv.org/html/2508.10104v1) ·
[Meta announcement](https://ai.meta.com/blog/dinov3-self-supervised-vision-model/).

### 2.1 What they are, and which variant is even a candidate

DINOv3's teacher is a **ViT-7B (6.7 B params, 40 layers, 4096-d, patch 16)**. Nothing about
that is relevant to us. What *is* relevant is the distilled family it produced:

- **ViT**: S, S+, B, L, H+, 7B — **ViT-S is 21.6 M params** (from the paper).
- **ConvNeXt**: T, S, B, L — **29 M to 200 M params**, explicitly distilled from the ViT-7B
  "to accommodate varying compute constraints". `convnext_tiny.dinov3_lvd1689m` is published
  on timm ([HF](https://huggingface.co/timm/convnext_tiny.dinov3_lvd1689m)).

⛔ **The ConvNeXt distillations are the only serious Hailo candidates, and that is the whole
point of them.** The Hailo-8 is a dataflow CNN accelerator. Convolutions map onto it; attention
does not, and the model zoo numbers below show the penalty directly.

### 2.2 (a) Can it run on our hardware, and at what rate?

**Hailo-8, from the vendor's own
[HAILO8_classification.rst](https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8/HAILO8_classification.rst)** (224×224×3, **batch 8**):

| model | FPS @ batch 8 | top-1 |
|---|---|---|
| mobilenet_v1 | 3305 | 70.3 |
| resnet50 | 1372 | 74.7 |
| fastvit_sa12 | 1113 | 76.7 |
| levit256 | 522 | 79.1 |
| **vit_small** | **309** | 80.0 |
| vit_base | 108 | 83.2 |
| deit_small | 309 | 77.0 |

⚠ **Two corrections before anyone quotes 309 FPS.**
1. Those are **batch 8 on a PCIe Gen3 ×4** host. The
   [Hailo forum states plainly](https://community.hailo.ai/t/official-fps-benchmark-on-hailo-8-using-raspberry-pi-5/18873)
   that "official benchmarks are recorded while running on a four-lane PCIe interface, while
   the Raspberry Pi 5 uses PCIe Gen3 ×1". That is exactly the gap our own 9.3 ms round trip
   measures.
2. At batch 1 the round trip dominates. 309 FPS @ batch 8 is ~3.2 ms of NPU time per image;
   on our ×1 link at batch 1 that lands at **~12–13 ms wall clock**, i.e. roughly a second
   detector's worth of budget — and `resnet50` is **4.4× faster than vit_small** on the same
   silicon, which is the attention penalty made numeric.

**Pi 5 CPU**: no. At 518×518 with patch 14 a ViT-S sees **1369 tokens** against 196 for the
224/16 configuration the Hailo numbers above use — **7× the token count**, with attention on
top of that. Our own anchor data point is that a **2.7 MB** conv net costs 88 ms at VGA on
3 threads; a 21.6 M-param ViT at 7× the tokens is orders above that.
**Single-digit FPS at best, and that is without the detector running.**

⚠ **We have a nearby model and have never timed its encoder.**
`mongla_vision/depth/models/depth_anything_v2_small.onnx` (see
[`perception/depth-estimation.md`](../../perception/depth-estimation.md)) **is a
DINOv2 ViT-S/14 encoder** with a DPT head. It is in the repo, it runs through onnxruntime, and
**nobody has recorded its Pi 5 latency** — not in `measured-bars.md`, not in the depth doc.
That single measurement would replace every estimate in this section with a number.
`sota-vo-depth.md` already calls ViT-S "the plausible Hailo candidate" for depth; if that
compile ever happens, the encoder is shared and the DINOv2 question answers itself for free.

### 2.3 (b) What would it actually buy an AUV?

Three plausible uses. They are not equally plausible.

| use | honest verdict |
|---|---|
| **Place recognition** (global descriptor) | The real one. [AnyLoc](https://anyloc.github.io/) — VLAD-aggregated DINOv2 patch features — is the method now used as the loop-closure frontend in e.g. [DPV-SLAM loop closure work (arXiv 2601.02723)](https://arxiv.org/html/2601.02723v1). But see §4: on a tiled pool floor the input has no unique content to recognise. |
| **Segmentation without labels** | Genuinely attractive — it would let us segment "pool floor / wall / prop / water column" with no labelling pass. But the output is a **patch-resolution** mask: at 224×224 with patch 14 that is a **16×16 grid**. For a prop that subtends 40 px in a 640-px frame, that is one patch. Useless at our scale unless run at high resolution, which we cannot afford. |
| **Correspondence** | DINOv2 features do match across extreme appearance change, but **coarsely** — they localise to a patch, i.e. ±7 px at 1× resolution. Our anchor already gets sub-pixel matches from XFeat with 56–260 inliers on the *worst* footage we own. This is strictly worse for our purpose. |

### 2.4 (c) Adoption cost, and the verdict

- Patch-14 models require input a **multiple of 14**. Our cameras are 1280×720 / 640×400 and
  our pipeline is built on 320×240 and 640×480. Every DINOv2 input needs a resize to e.g.
  518×518 — a non-trivial CPU cost on its own, and a distortion of aspect ratio.
- A second Hailo network = a second ~9.3 ms round trip **plus** contention with the 98 Hz
  detector, which is our only actually-working perception capability.
- ⛔ **INT8 on a self-supervised embedding space has no label to defend it.** A classifier that
  loses 1 % top-1 is still a classifier. An embedding whose cosine geometry has been squashed
  to 8 bits fails silently — which is the exact failure class this codebase's safety rules name
  ("a plausible number standing in for an absent measurement").
- ⚠ Nobody has published DINOv2/v3 on a Hailo-8. The 309 FPS above is **supervised ViT-S/16
  with a classification head**, not a DINOv2 backbone, and I am extrapolating between them.

> **Verdict: NO for the current campaign.** DINOv3-ConvNeXt-T is the only variant that is even
> architecturally sane on this board, and the capability it would buy (place recognition) is
> defeated by the environment (§4), while the capability that sounds most attractive
> (label-free segmentation) is defeated by patch resolution. Revisit only if we ever need
> *semantic* scene understanding — "is this a wall or open water" — which we currently do not.

### 2.5 The one 2026 edge distillation I could find — and it argues for the NO

[Lightweight Distillation of SAM 3 and DINOv3 for Edge-Deployable Individual-Level Livestock
Monitoring (arXiv 2604.27128)](https://arxiv.org/abs/2604.27128), submitted 2026-04-29,
v2 2026-06-15. This is precisely the class of work Q2 asked about — somebody distilling DINOv3
specifically to get it onto an edge board — so it is the fairest available test of whether the
§2 verdict is merely pessimism.

| | |
|---|---|
| student | **40.66 M params**, TinyViT-21M-512 + FPN |
| embedding model | **DINOv3 ViT-S/16, 21.6 M** (used as-is, not shrunk further) |
| target hardware | **NVIDIA Jetson Orin NX 16 GB** |
| peak VRAM | **6.49 GB** (vs 19.52 GB for the teacher) |
| accuracy | 92.29 % MOTA / 96.15 % IDF1; −1.68 / −0.84 pp vs the SAM 3 teacher |
| FPS / latency | **not stated in the abstract** |

⛔ **Read the hardware line.** After distillation, and after cutting VRAM by two thirds, the
result still targets a **Jetson Orin NX with 16 GB** and consumes **6.49 GB**. The Raspberry
Pi 5 has **8 GB of LPDDR4X shared with the entire ROS 2 graph, both cameras and the detector**,
and the Hailo-8 has no usable DRAM of its own for a model of this shape. Note also that the
distillation **kept DINOv3 ViT-S/16 unchanged at 21.6 M** — the thing they compressed was SAM 3,
not DINOv3. **ViT-S is already the floor.**

**This is the strongest evidence in §2**: the state of the art in making DINOv3 edge-deployable,
as of mid-2026, lands on a board an order of magnitude beyond ours. The §2 verdict is not
pessimism; it is the published result.

---

## 3. The rest of the field: LightGlue, ALIKED, SuperPoint+SuperGlue, DeDoDe, RoMa, MASt3R/DUSt3R

### 3.1 The sparse detectors — and why XFeat already won this

The XFeat paper benchmarks all of them on **one CPU (i5-1135G7, VGA)** which is the only
apples-to-apples CPU table I found. Applying our own measured Pi-5-vs-that-laptop penalty
(**2.4–4×**, derived in §1.1 from running the *same* XFeat graph on both):

| detector | i5 FPS (published) | i5 ms | **implied Pi 5 ms** | verdict |
|---|---|---|---|---|
| ORB | 44.3 | 22.6 | **not measured** (scaling OpenCV C++ by a PyTorch ratio is not valid) | RUNS TODAY, and **loses 0/4 on murky clips** |
| **XFeat** | **27.1** | **36.9** | **88.0 measured @VGA / 18.2 @320×240** | **RUNS TODAY** |
| XFeat\* | 19.2 | 52.1 | ~125–210 | no measured need (§1.3) |
| ALIKE | 5.3 | 188.7 | ~450–750 | **NO** |
| SuperPoint | 3.0 | 333.3 | **~800–1300** | **NO** on CPU |
| DISK | 1.2 | 833.3 | ~2000–3300 | **NO** |

**ALIKED** is ALIKE's successor (deformable-conv descriptors). The only runtime I could find is
GPU: **ALIKED 8.89 ms, SuperPoint 6.46 ms, DeDoDe-v2 42.76 ms — all on an NVIDIA A100**
([Mismatched, arXiv 2408.16445](https://arxiv.org/pdf/2408.16445)). An A100 figure tells us
nothing about a Pi except the ordering, and the ordering already says ALIKED sits with ALIKE.
⚠ ALIKED's deformable convolutions are also **not Hailo-friendly** — the offsets are
data-dependent gathers, which a static dataflow compiler cannot schedule.

### 3.2 The matchers

[LightGlue (ICCV 2023, arXiv 2306.13643)](https://arxiv.org/pdf/2306.13643) — adaptive-depth,
adaptive-width attention matcher; the headline is that it is **"as fast as" and more accurate
than SuperGlue**. Published numbers: **SuperPoint+LightGlue ≈ 44 ms per image pair on GPU**;
one tracking application reports **LightGlue 38 ms init / 25 ms per frame vs SuperGlue 250 ms
/ 120 ms** ([SuperPose, arXiv 2409.19986](https://arxiv.org/pdf/2409.19986)).

⛔ **Every one of those is a GPU number for a transformer.** We have no GPU, our ONNX runtime is
CPU, and the Hailo compiles convolutions. A 9-layer cross/self-attention stack over up to 2048
keypoints is the single worst-shaped workload on this board.

⭐ **And we already have the CPU number, from our own sibling doc.**
[`sota/vision.md`](../vision.md) records **LightGlue on CPU at 0.31 pairs/s ≈ 3.2 s per pair** —
"unusable inside a 2 s gap". That is the number to quote, not the 44 ms GPU figure.

⛔ **And the Hailo route for matchers is closed, with named errors.**
[`vision-velocity-and-odometry.md`](vision-velocity-and-odometry.md) §3b records a Hailo forum
thread (Jul 2026) on SuperGlue: *"keypoint encoder produces shape/layout errors"*, *"GNN
attention layers use unsupported **einsum**"*, *"`ReduceLogSumExp` crashes the parser"*; the
9-layer model needs **three chained HEFs** at an estimated **70.8 % recall vs PyTorch**, after
a compile that ran **7 h 34 m** before dying. The shape of the answer, in that doc's words:
**extractors compile, matchers do not.**

**LighterGlue** (in the XFeat repo, ~3× lighter than LightGlue, retrained on XFeat's 64-D
descriptors) is the only version worth a second look, and only because it rides descriptors we
already compute. But: no published CPU or edge runtime exists, and §1.3 records that we have
**no measured matching failure for it to fix** — our worst clip is 4/4 with 56–260 inliers.

**SuperGlue**: superseded by LightGlue on every axis, and restrictively licensed
(Magic Leap non-commercial). **NO, on both counts.**

**DeDoDe / DeDoDe-v2**: decouples detection from description, strong on MegaDepth.
42.76 ms **on an A100** for detection alone. **NO.**

### 3.3 The 3D foundation models — RoMa, DUSt3R, MASt3R

These are ViT-L-class encoder–decoder models that regress dense pointmaps or dense warps.
They are the accuracy state of the art and they are **categorically out of reach**:

| model | measured cost | source |
|---|---|---|
| MASt3R | **198.16 ms per image pair on an NVIDIA A40** | [Speedy MASt3R, arXiv 2503.10017](https://arxiv.org/abs/2503.10017) |
| Speedy MASt3R | 91 ms per pair on an A40 (−54 %) | same |
| DUSt3R | **>16 GB VRAM** beyond ~120 images even with one-ref pairing | [DUSt3R analysis](https://learnopencv.com/dust3r-geometric-3d-vision/) |
| RoMa v2 (Nov 2025) | beats MASt3R/DUSt3R on pose; same ViT-L class | [RoMa v2, arXiv 2511.15706](https://www.emergentmind.com/papers/2511.15706) |

A 40 GB datacentre GPU at 198 ms/pair is ~5 pairs/s. The Pi 5 has no GPU, 8 GB of shared LPDDR,
and a 26 TOPS INT8 NPU that cannot hold a ViT-L. **NO. Do not spend another hour on these.**

> **§3 verdict: nothing in this section beats XFeat *on our hardware*, and the one thing that
> beats it on accuracy (LightGlue-family matching) is a transformer we cannot run and cannot
> compile. The field's rankings are measured on GPUs and on MegaDepth; our own EdgePoint2
> result is the standing proof that neither transfers to turbid water.**

---

## 4. Place recognition / loop closure on a pool floor

The families: **NetVLAD** (learnable VLAD aggregation), **CosPlace** (classification-trained
global descriptors over non-overlapping place groups), **EigenPlaces**
([arXiv 2308.10832](https://arxiv.org/pdf/2308.10832), viewpoint-robust training), and the
2024–2026 successors **MixVPR, SALAD, AnyLoc, MegaLoc, SuperVLAD**.

### 4.1 The geometric argument, which settles it before any benchmark

⛔ **A periodic floor aliases at the tile pitch for *every* appearance-based global descriptor.
This is a property of the scene, not a weakness of the method.** A global descriptor answers
"have I seen this image before?". Over a uniform tiled floor at fixed altitude, **every** image
is the same image up to a sub-tile phase shift. There is no descriptor — learned, foundational
or otherwise — that can distinguish tile (3,7) from tile (9,2) when the only thing in frame is
tile. Retrieval would return a confident, wrong match, which is worse than no match.

The VPR literature confirms this empirically, in the mild case:
[Evaluation of VPR methods for image-pair retrieval (arXiv 2603.13917)](https://arxiv.org/html/2603.13917)
reports **CosPlace "completely fails with perceptual aliasing"** on repeated urban content
(two similar black cars), while newer MegaLoc is robust — and
[NYC-Indoor-VPR (arXiv 2404.00504)](https://arxiv.org/pdf/2404.00504) singles out a subway
station whose **"hallways with repetitive features cause perceptual aliasing"**. A car park with
similar cars is a *mild* aliasing problem. A surveyed tile grating is aliasing at 100 %.

Same source on runtime: **"CNN-based methods perform below one second, whereas the ViT-based
methods take more than one or multiple seconds on average"** — and that is on their evaluation
hardware, not a Pi 5.

### 4.2 What this means given what we already built

`mongla_localization/tile_grating.py` already takes the correct position on this, and states it
in its own docstring: *"Phase is absolute only modulo the tile pitch — it bounds drift to one
tile, it does not localise the vehicle in the pool. Counting which tile you are on needs
continuity."*

**That is exactly the gap a place-recognition system would have to fill, and it is exactly the
gap that aliasing makes unfillable from floor appearance alone.** The best any VPR method could
do is recover the *integer tile index*, and only from **non-periodic** content: lane lines
(`pool_lines.py` already reads these), drains, the end cross-line, wall transitions, props.

So the honest framing is: **we do not need a learned global descriptor. We need the
non-periodic cues, and we already have purpose-built, physics-grounded readers for two of
them** — and those readers use published survey constants (World Aquatics lane centres at
exactly 2.5 m) rather than learned weights, so they need no venue training data.

### 4.3 The verdicts

| | (a) rate on our hardware | (b) what it adds | (c) cost | verdict |
|---|---|---|---|---|
| NetVLAD (VGG16 backbone) | Pi 5 CPU: ~1 s/frame class; Hailo: VGG16 compiles but is heavy | nothing on bare floor | large | **NO** |
| CosPlace / EigenPlaces (ResNet-50 backbone) | ResNet-50 is **1372 FPS @ batch 8** on Hailo-8 (PCIe ×4) → realistic **~11–14 ms at batch 1 on our ×1 link**, plus detector contention | nothing on bare floor; **documented to fail under aliasing** | a second Hailo net + a descriptor database + INT8 risk on an embedding | **NO** |
| AnyLoc / SALAD / MegaLoc (DINOv2 backbone) | see §2 — ViT on a ×1 link, ~13 ms best case, likely worse | the most aliasing-robust of the family | everything in §2.4 | **NO** |
| **`tile_grating.py` + `pool_lines.py`** | **RUNS TODAY**, one FFT / one line fit | heading (mod 90° / 180°), height, sub-tile phase, lateral offset — from published survey constants | already paid | **keep, extend** |

> **§4 verdict: NO to all of NetVLAD/CosPlace/EigenPlaces on the downward camera. The one place
> worth a separate experiment is the FORWARD camera near structure** — a gate, an octagon, a
> wall corner is non-periodic content, and re-recognising it is a real capability. But note what
> that reduces to: re-finding a specific known object, which is **what the XFeat anchor already
> does, with measured 4/4 on our worst footage**. Test the anchor against that use before
> buying a descriptor database.

---

## 6. Underwater-specific feature matching: what survives turbidity and caustics

⚠ *Placed before §5 deliberately: §5's verdict on the downward camera depends on what §6.2
measures about caustics. Read in file order, not in number order.*

### 6.1 Our own evidence is the strongest evidence we have

The clip table in §0 is a controlled, same-footage, same-protocol comparison on **our** water:
ORB **0/4** where XFeat is **4/4**; ORB finds **7 keypoints in an entire Mirpur frame**. And
EdgePoint2 — published as faster-than-XFeat with competitive IMC2022 results — collapses to
**1/4** on the same murky clip. ⛔ **Published rankings on clear natural imagery do not survive
the move to turbid water.** Treat every number in this section accordingly.

### 6.2 What the literature says, and where it agrees with us

**Learned features are not automatically better underwater.**
[Image-Based Relocalization and Alignment for Long-Term Monitoring of Dynamic Underwater
Environments (arXiv 2503.04096)](https://arxiv.org/abs/2503.04096) — Gorry, Fischer, Milford,
Fontan; submitted 2025-03-06, v2 2025-12-02; it combines VPR, feature matching and segmentation
and introduces the **SQUIDLE+ VPR Benchmark** (⭐ the one underwater VPR benchmark I found, and
relevant to §4 if we ever revisit it). The finding attributed to it is that among conventional
features **ROOT-SIFT and SIFT perform best, significantly better than ORB**, while
**SuperPoint "had highly variable performance across different sequences with lower mean number
of inlier matches than other conventional features."** That is the same shape as our EdgePoint2
result, from an independent group on different water.

⚠ **Attribution hedge, stated rather than hidden.** That quote reached me through a search
summary; the PDF exceeded the fetch limit and **the abstract does not contain it**. The quote
is real; *which* paper it belongs to is my inference. **The claim is therefore listed below as
unverified** — but it does not change action #1, because benching ROOT-SIFT on our own clips
is cheap and settles the question with our own number regardless of who said what.

⭐ **Note what this implies for us, and I checked rather than assumed: SIFT has never been
benched on this vehicle.** `grep -i 'sift\|akaze\|brisk' measured-bars.md` returns **one**
line, and it is about auto-exposure (DRL-AE, "+38 % SIFT..."), not about our footage;
`grep -i sift src/mongla_vision/` returns **nothing**. Our comparison was
ORB-vs-XFeat-vs-EdgePoint2 only. The literature's strongest *classical* underwater performer
is ROOT-SIFT, and OpenCV ships it (patent expired 2020, `cv2.SIFT_create` is in the main
build). **That is a cheap, genuinely missing data point** — one afternoon, same five clips,
same 40 %-reference / +1/3/5/8 s / `USAC_MAGSAC` / ≥15-inlier protocol.

⚠ It will very likely lose to XFeat on **speed** regardless (SIFT at VGA is tens of ms on a
Pi), so this is not a candidate to adopt — **it is a control**. If ROOT-SIFT also reads 4/4 on
the murky clips, then our 4/4 is telling us the clips are easier than we think, not that XFeat
is special. That is worth knowing either way, and it is the kind of check this codebase's own
rule asks for: *truth tests, not agreement tests.*

**Enhancement makes matching worse — externally confirmed.**
[Impact of Underwater Image Enhancement on Feature Matching (arXiv 2507.21715, Jul 2025)](https://arxiv.org/html/2507.21715v1)
evaluated SIFT, ORB, BRISK, KAZE, AKAZE and SuperPoint against six enhancement methods
(Ancuti 2017, Demir & Kaplan 2023, FUnIE-GAN, WaterNet, WaveNet, UVENet):

> "In the majority of cases the videos produced after enhancement were equal or worse in
> performance compared to the unaltered original video."

| Seabed, ORB, frame offset 1 | inliers |
|---|---|
| **original** | **442.21** |
| WaterNet | 440.29 |
| Demir & Kaplan | 328.03 |

And the systems-level result, which is the damning one: **every enhanced video caused ORB-SLAM3
tracking loss and ZERO loop closures across all five runs, while the original video achieved
loop closure.**

⛔ **This is independent, published confirmation of our own standing rule** — *"image
preprocessing stays off; measured in 17 configurations across four props and three venues:
never positive; on the gate it destroyed 95 % of detections."* Two groups, different water,
different detectors, same conclusion. The test that keeps preprocessing off should cite this.

**Caustics specifically.** The enhancement paper does **not** address caustics. The one source
that does, [UKDM (arXiv 2504.11063, Apr 2025)](https://arxiv.org/pdf/2504.11063), reports that
in caustic environments **FSpiral-GAN was best on precision and repeatability, ORB achieved
the highest AUC but struggled on repeatability, and SIFT gave a moderate balance.** ⚠ I could
not verify the absolute numbers; treat this as a direction, not a measurement.

**The mechanism matters more than the ranking.** A caustic is a moving, high-contrast pattern
**projected onto** the floor, not attached to it. So:
- **Detectors** that fire on high contrast (ORB, and any corner detector) will happily put
  keypoints on caustics. Those keypoints move with the sun and the surface waves, not with the
  vehicle.
- **RANSAC rejects them only if they are a minority.** On a bare pool floor in sunlight the
  caustics can be the *majority* of the contrast in frame — at which point RANSAC fits the
  caustic motion and returns a confident, wrong velocity. This is the exact failure our
  measured bars already record for LK ("fails on bare floor, sun caustics").
- **Learned detectors do not fix this** — they were trained to find repeatable structure and a
  caustic looks like repeatable structure for the ~100 ms it persists.
  `vision-velocity-and-odometry.md` §3c reaches the same conclusion about XFeat specifically.

⭐ **And we have already measured all of this, on real caustics, with a shipped fix.**
`measured-bars.md`, *"Sun caustics on the floor: flow tracks the waves"* (2026-09-15): real
RoboSub sun caustics composited over three real floors, moved by a **known 13.4 px baseline**,
27 pairs per row, through the production `detect_corners` + LK + `robust_flow`:

| floor | raw | erode 3 | erode 5 | **erode 7** | no caustics |
|---|---|---|---|---|---|
| tiles | 11.19 px | 2.40 | 0.18 | **0.09** | 0.000 |
| octagon mat | 1.01 | 0.22 | 0.14 | **0.11** | 0.000 |
| slalom floor | 13.31 | 12.82 | 12.86 | **12.47** | 0.000 |
| plain (synthetic) | 13.13 | 13.48 | 13.07 | **14.10** | refuses |

**Shipped: grey erosion 7×7, applied per anchor only when the frame is caustic** (detector:
top-hat(9×9) mean / frame mean, threshold **0.07**, **4.73 ms on the Pi** per 640×480, downward
camera only). Plus a provisional refusal at **median patch NCC < 0.71, in sun only**.

⛔ **This corrects a claim I was about to make.** I had written that the physical prior fixes
caustics. The measurement says something sharper and less comfortable:

1. **On a floor with dark texture of its own, caustics are already solved** — 11.19 px of
   error → **0.09 px** on tiles. A morphological erosion, not a better detector, and not a
   learned anything. **Nothing in this entire document improves on that.**
2. ⛔ **On a floor with NO dark texture of its own (slalom, plain concrete), it is NOT fixed**
   — **12.47 px of error survives erosion 7**, i.e. the estimator still reports the waves at
   ~93 % of the true baseline. That is the open problem, and it is the single most concrete
   perception gap this document has touched.
3. **Does the FFT grating read through it?** It *should* — a caustic is broadband where the
   tile pitch is narrowband — but **that is my argument, not a measurement**, and the erosion
   table shows the tiled case is already solved without it. ⚠ The case where the grating would
   matter is case 2, **and a floor with no dark texture also has no tile grating to demodulate**.
   The prior does not obviously rescue the regime that actually needs rescuing. **Listed as
   unverified; do not quote it as a reason.**

---

## 5. Homography / plane-induced motion on the DOWNWARD camera

**The question as posed contains a false premise, and clearing it up is most of the answer.**

### 5.1 We already fit a plane-induced transform. The choice is 4-DoF vs 8-DoF, not "flow vs
homography"

`mongla_vision/flow/flow_math.py` does **not** integrate a raw flow field. It fits a global
model to the LK correspondences:

```python
M, inl = cv2.estimateAffinePartial2D(p0, p1, method=cv2.RANSAC, ransacReprojThreshold=ransac_px, ...)
```

`estimateAffinePartial2D` is a **4-DoF similarity** — tx, ty, θ, scale — and the module's own
comment says why: *"the flow field is a single similarity transform and every point must agree
with it."* It even records that `USAC_MAGSAC` is **not accepted** by that call (only RANSAC and
LMEDS are), which is a real, measured constraint on any upgrade path.

So we already extract "translation + rotation + scale directly". **The question is only whether
the remaining 4 degrees of freedom of a full homography are worth buying.**

### 5.2 What the extra 4 DoF are, and who already supplies them

A plane-induced homography **H = K(R − t·nᵀ/d)K⁻¹** has 8 DoF. Beyond the similarity's 4, the
extra ones encode **the tilt of the plane relative to the camera** — i.e. **vehicle roll and
pitch**. On this vehicle:

- Roll and pitch come from the board's **500 Hz IMU/AHRS**, which is already how the flow is
  de-rotated (measured: de-rotation takes pure-rotation error **575.7 → 57.1 mm/s**).
- The floor is **flat and horizontal by survey**, and depth is measured. So n and d are known
  *a priori*, not things we need vision to estimate.
- ⚠ **A 46.7° in-water FOV is narrow.** Perspective effects — the signal the extra DoF live
  on — grow with field angle. In a narrow-FOV, near-nadir, small-tilt view the homography is
  numerically **close to degenerate** with the similarity: you are asking RANSAC to estimate
  4 more parameters from the same noisy points, in a regime where they are barely observable.
  The predictable outcome is **higher variance, not higher accuracy**.

⛔ **And a homography does not give metric translation either.** It recovers **t/d** — the
translation *scaled by plane distance*. Height is still required, exactly as now. The premise
that a homography "gives translation directly" means *image* translation, which the similarity
already gives.

### 5.3 What the literature actually does

The continuous-homography formulation ([Attitude, Linear Velocity and Depth Estimation of a
Camera Observing a Planar Target Using Continuous Homography and Inertial Data](https://www.researchgate.net/publication/327804540_Attitude_Linear_Velocity_and_Depth_Estimation_of_a_Camera_Observing_a_Planar_Target_Using_Continuous_Homography_and_Inertial_Data))
and the UAV ego-motion line recover **scaled** linear + angular velocity from the continuous
homography constraint, then **fuse with the IMU to retrieve metric scale**. One reported result:
vertical distance, velocity vector and feature position to **5 cm in under 30 s**.

⚠ **5 cm is worse than our measured 1.09 cm worst error over three 30 cm slides.** Different
setup, different scale — but it is not a number that argues for switching.

The production counter-example is instructive: [PX4's optical-flow
stack](https://docs.px4.io/main/en/sensor/optical_flow) uses a downward camera **plus a
downward distance sensor**, and fuses flow with gyro. It does not fit a homography. The
industry answer to "how do I get metric velocity from a downward camera" is *measure the
height*, not *estimate more DoF*.

### 5.4 The verdicts

| | (a) rate | (b) what it adds over today | (c) cost | verdict |
|---|---|---|---|---|
| **similarity + IMU de-rotation** (today) | **12.34 ms/pair, RUNS TODAY** | — | paid | **keep** |
| full 8-DoF homography on LK points | `findHomography` + MAGSAC is a few ms; no real speed penalty | **roll/pitch, which the 500 Hz IMU already gives better**; still no metric scale | re-plumbing the estimator into a regime where 4 extra params are weakly observable at 46.7° FOV | **NO — this is not a new capability** |
| **XFeat correspondences instead of LK, on the downward camera** | 18.2 ms @320×240 (3 threads), **+18 ms on top of the 12.34** | **robust correspondences where LK has none** — the bare-floor and murky cases where LK fails outright | needs the §1.2 Hailo path to be affordable at rate; needs a murky-clip re-measurement on *downward* footage, which we have never done | **worth an experiment — this is the real §5 idea** |
| **`tile_grating.py` phase** | RUNS TODAY | height, absolute heading mod 90°, drift-bounded sub-tile phase | already paid | **keep, extend** — but see §6.2: caustics on a *tiled* floor are already fixed by a 4.73 ms erosion, so this is not the caustic answer |

⭐ **`vision-velocity-and-odometry.md` §3d reached the identical conclusion independently**,
and in the same words: recover *"the **similarity** (not homography — the floor is a known
plane and the full 8-DOF fit is over-parameterised and less stable)"*. Two separate passes,
same answer. That is as close to settled as this gets without a measurement.

⛔ **And there is a measured ceiling that closes the door properly.** Scored against
constructed truth — a real downward frame warped by a known (dx, dy, θ, scale), 25 frames per
case, worst of the swept range (30 px shift, 15° rotation, ×1.20 zoom) — the incumbent LK +
RANSAC similarity returns **0.006–0.40 px translation, 0.002–0.05° angle, ≤0.001 scale**
(`measured-bars.md`, 2026-09-11). **There is no room above that for anything in this document
to win in the nominal regime.** Our 1.09 cm is 1.02 cm of *scale bias* — height, FOV and
refraction — and no matcher and no extra degree of freedom touches any of those three.

> **§5 verdict: NO to the homography upgrade. The downward camera's problem is not the model
> we fit — it is that on a bare or caustic-lit floor LK has nothing to track. Fixing that means
> changing the *correspondence source* (XFeat, or the FFT grating), not adding degrees of
> freedom to the fit.**

---

## 7. The whole thing on one page

| candidate | Pi 5 CPU | Hailo-8 | gives us that we lack | verdict |
|---|---|---|---|---|
| **XFeat sparse** | **18.2 ms @320×240 / 88.0 @VGA (3 thr)** | **compilable — HEFs exist**, ~10–13 ms est., unmeasured | *is* the anchor | **RUNS TODAY** |
| XFeat on Hailo | — | see above | VGA-rate XFeat; downward-camera XFeat | **try it, gated on re-measuring the 4/4 murky table through INT8** |
| XFeat\* (semi-dense) | ~125–210 ms est. | refinement MLP would be a 3rd graph | +7.6 AUC@5 on MegaDepth | **NO — no measured need** |
| LighterGlue | unmeasured; transformer | attention, ill-suited | better matching we cannot show we need | **NO for now** |
| LightGlue / SuperGlue | **0.31 pairs/s ≈ 3.2 s/pair** (our own `vision.md`) | **NO** — einsum, ReduceLogSumExp, 3 chained HEFs, 70.8 % recall | — | **NO** |
| SuperPoint | ~800–1300 ms est. | **~300 FPS reported on Hailo-8** (community port, not in the Model Zoo) | worse underwater than SIFT; and we already have XFeat | **NO on CPU; on Hailo it works but buys nothing over XFeat** |
| ALIKED / ALIKE | ~450–750 ms est. | deformable conv → **not compilable** | — | **NO** |
| DeDoDe-v2 | 42.76 ms **on an A100** | — | — | **NO** |
| RoMa / DUSt3R / MASt3R | — | — | — | **NO — 198 ms/pair on an A40; >16 GB VRAM** |
| DINOv2 / DINOv3 ViT-S | single-digit FPS | ViT-S ~13 ms est. @batch 1; **4.4× slower than ResNet-50 on the same chip** | semantics we do not need; patch-resolution too coarse | **NO** |
| DINOv3 ConvNeXt-T (29 M) | no | the only architecturally sane variant | same as above | **NO** — and §2.5: the 2026 state of the art in edge-DINOv3 targets a **Jetson Orin NX 16 GB at 6.49 GB peak** |
| NetVLAD / CosPlace / EigenPlaces | ~1 s (CNN) | ResNet-50 backbone ~11–14 ms est. | **nothing on a tiled floor — aliasing is total** | **NO** |
| AnyLoc / SALAD / MegaLoc | worse | worse | most aliasing-robust, still defeated by a pure tile field | **NO** |
| homography (8-DoF) on downward LK | free | n/a | roll/pitch the IMU already gives better; **still no metric scale** | **NO** |
| **ROOT-SIFT on our own murky clips** | tens of ms @VGA | n/a | **a missing data point** — the literature's best classical underwater performer, never benched here | **DO THIS — one afternoon** |
| underwater image enhancement | — | — | **negative**: 442.21 → 328.03 inliers, and **zero** ORB-SLAM3 loop closures | **NO — now externally confirmed** |

### The three things actually worth doing, in order

1. **Bench ROOT-SIFT on the five archive clips**, same 40 %-reference / +1/3/5/8 s /
   `USAC_MAGSAC` / ≥15-inlier protocol. Cheapest item here, closes a real gap in our own
   evidence, and the published underwater result says it might win on robustness.
   *(BUILT? no — but it is a one-file script against footage we already own.)*
2. ⚠ **DEMOTED — see §8.1.** *Compile XFeat to a HEF with a calibration set drawn from our
   turbid footage, then re-run the murky clip table through it.* The recipe is three lines and
   published, and the finding that **it compiles** (§1.2) closes a standing unknown. But the Pi
   is **72.8 % idle** with both rungs running, so this spends scarce Hailo duty cycle to buy
   back abundant CPU. **Do it only if action #3 succeeds and needs VGA.** If it is ever done:
   **a HEF that compiles is not a result; a HEF that still reads 4/4 on the murky clips is.**
   Requires an x86 box with the Dataflow Compiler — not installed here (`which hailo` → not
   found).
3. **Try XFeat correspondences on the DOWNWARD camera** on bare-floor footage, where LK is
   measured to fail. This is the only item in the whole document that offers a capability we
   do not have rather than a cheaper version of one we do — and
   `vision-velocity-and-odometry.md` §3c has already scoped it, down to the right first
   experiment: **keypoint and inlier counts, Shi-Tomasi vs XFeat, on the existing downward
   30 cm-slide recordings.** It also names the gap honestly — Shi-Tomasi was never benched, so
   "it fails where ORB fails" is reasoned, not measured. ⛔ **Note the scope cut: not caustics.**
   §6.2 shows XFeat matches caustic structure as readily as LK tracks it, and on a *tiled* floor
   the shipped 7×7 erosion already takes 11.19 px → 0.09 px. The bare-floor case is the one
   worth the 18 ms.

Everything else in this document is a **no**, and most of them are a no for the same reason:
the published ranking was measured on a GPU, on clear natural imagery, against a metric
(MegaDepth relative-pose AUC) that is not the thing this vehicle needs.

---

## 8. ⛔ Where this document DISAGREES with its siblings

One truth, two copies is the bug. These are named rather than left to be discovered.

### 8.1 XFeat on the Hailo — "try it" (here) vs "keep it on the CPU" (`vision-velocity-and-odometry.md` §3b)

That doc concludes: *"putting the extractor on the Hailo costs a 9.3 ms round trip and takes
chip time from the detector, against 33.1 ms saved. **Marginal at best, and it fights the
detector. Keep XFeat on the CPU.**"*

⛔ **On re-reading it, that doc wins, and its winning argument is one I did not have.** It does
not rest on the millisecond arithmetic. It says: *"the reason is not the millisecond
arithmetic, it is that **we are not short of CPU**. The Pi is **72.8 % idle** with both rungs
running. Moving the extractor to the Hailo would spend the detector's chip time to buy back a
resource we have in surplus."*

**That is correct and it reframes my §1.2.** A 6–8× speedup of a thing that is not the
bottleneck is not a win — it is a trade of a scarce resource (Hailo duty cycle, feeding our
only working perception capability at 98 Hz) for an abundant one (idle CPU).

**What survives of my position, narrowed to what it can carry:** the Hailo path is interesting
**only** if we want XFeat on the **downward camera at VGA simultaneously with the forward
anchor** — i.e. only if §7 action #3 succeeds *and* 320×240 proves insufficient there. Two
conditionals deep. ⚠ It also **doubles the input bytes** over the ×1 link (§1.2, three graph
inputs), which eats into the 6–8× before anything else does.

**Resolution: `vision-velocity-and-odometry.md` §3b stands — keep XFeat on the CPU.** I am
demoting §7 action #2 accordingly (see the note there). The genuinely new contribution of
§1.2 is not "we should do this"; it is **"it compiles, here are the HEFs, and here is exactly
which two nodes come off"** — which closes that doc's own unverified item *"whether XFeat even
compiles under `hailomz` is unknown"*.

### 8.2 XFeat★ — "NO, no measured need" (here) vs "a measured free lunch" (`vision.md` V-7)

`vision.md` calls **XFeat → XFeat\*** *"the one published free-lunch axis in that family —
**+111 % inliers (892 → 1 885) and +10.2 Acc@10°** for **1.4× the time**"*, and lists it as
recommendation **V-7**.

**Both statements are true; they answer different questions.** The +111 % is real and
published. My **NO** is a *cost* judgement on *this vehicle*: 1.4× of 18.2 ms is ~25 ms, and
our worst murky clip already clears the ≥15-inlier bar at **4/4 with 56–260 inliers**. More
inliers on a homography that is already trusted buys nothing we can point at.

⚠ **But `vision.md` notes the caveat that decides it, and I should repeat it here**: *"MegaDepth
and HPatches measure wide-baseline pose between image pairs, not planar-anchor survival across
a blackout. The ranking transfers; the absolutes do not."*

⛔ **Resolution — and I have to correct myself here too: XFeat\* is NOT "just a flag" for us.**
In the upstream PyTorch API it nearly is. In *our* deployment it is not, because
`tools/xfeat_export.py` exports **only `net`** — the three heads. XFeat\*'s gain comes from the
**match-refinement MLP**, which is a *separate module* that is **not in our ONNX graph at all**.
Adopting it means a second export plus a numpy port of the refinement, on top of the coarse
1/8-resolution matching path. That is a day, not a flag.

**Treat V-7 as live but correctly priced, ranked below the three actions in §7.** If the Hailo
path in §8.1 ever lands, 1.4× of ~11 ms is cheap and the answer flips — but the porting work
does not go away.

### 8.3 DINOv3 — "NO" (here) vs an underwater result cited in `vision.md`

`vision.md` cites [arXiv 2604.00313](https://arxiv.org/abs/2604.00313): frozen DINOv3 features
underwater give **13 labelled images per category (≈4 %) → 81.8 % macro F1**, and 144 per
category → 88.5 %, level with a fully-supervised ConvNeXt at 88.9 %.

**No conflict, and it is worth being precise about why.** That is **offline, desk-side
few-shot labelling** — DINOv3 as a *label-efficiency tool on a workstation*. My **NO** in §2 is
about running DINOv3 **on the vehicle, in the loop, at rate**. Those are different machines and
different budgets. ⭐ The offline use is genuinely attractive and is an
[`sota-autolabel.md`](sota-autolabel.md) matter, not a runtime one.

### 8.4 Agreements worth recording

- ALIKED's deformable convolution as a GridSample consumer — `vision-velocity-and-odometry.md`
  §1a reaches the same conclusion independently.
- **Similarity, not homography, on the downward camera** — §5 here and
  `vision-velocity-and-odometry.md` §3d, arrived at separately, same reasoning
  ("over-parameterised and less stable" over a known plane).
- **Caustics are not a detector problem** — §6.2 here and that doc's §3c
  ("XFeat will detect and match caustic structure as readily as it matches floor texture").
  `measured-bars.md` then supplies what neither of us had: the fix is a **7×7 grey erosion**,
  11.19 px → 0.09 px on tiles, and it is already shipped.
- The 2025 RoboSub champion (NUS Bumblebee, BBAUV 4.5) runs **YOLO11 → XFeat → PnP**; their
  2023 stack was YOLOv8 + **SIFT** + PnP. So the best team in the world made exactly the
  SIFT→XFeat move we made — which is an argument for §7 action #1 being a *sanity check*, not
  a likely reversal.
- The ~9.3 ms PCIe round trip is measured, not assumed: commit `68fc383` retracts an earlier
  4.9× multi-context claim after an end-to-end run showed ours (3 contexts, 9.48 ms, 97.47 Hz)
  and stock `yolov8s` (1 context, 9.22 ms, 99.73 Hz) are **identical at batch 1**, both
  dominated by the round trip.

---

## Claims I could NOT verify

- **Any measured FPS or latency for XFeat on a Hailo-8.** The HEFs exist and I inspected them;
  no benchmark for them exists in the fork, the Hailo thread, or the model zoo. My ~10–13 ms
  estimate is **extrapolation from our own 9.3 ms PCIe round trip**, not a measurement.
- **Accuracy of XFeat after INT8 quantisation**, on any imagery, let alone ours. Nobody has
  published it. This is the load-bearing unknown for §1.2.
- **Whether `x_feature_13_..._sim.hef` targets Hailo-8 or Hailo-8L.** The repo does not say.
- **Parameter count of XFeat** — not stated in the paper; only "2.7 MB ONNX" from our export.
- **DINOv2/DINOv3 on a Hailo-8 at all.** The 309 FPS `vit_small` figure is supervised ViT-S/16
  with a classification head at batch 8 on PCIe ×4; I extrapolated to batch 1 on ×1 and to a
  different backbone. Both steps are unverified.
- **Pi 5 CPU rates for SuperPoint, ALIKE, ALIKED, DISK.** Derived from the XFeat paper's i5
  numbers scaled by our own measured 2.4–4× Pi-vs-i5 penalty on XFeat. Plausible, not measured.
- **UKDM's caustic numbers** (FSpiral-GAN / ORB / SIFT) — I have the direction from a search
  summary, not the table.
- ⚠ **The ROOT-SIFT > SIFT > ORB / SuperPoint-variable finding of §6.2.** Attributed to
  arXiv 2503.04096 on the strength of a search summary; the PDF fetch failed (>10 MB) and the
  abstract does not carry it. The paper's title, authors, date and subject matter are verified
  and consistent with the claim; the sentence itself is not.
- ~~2026 edge distillations of DINOv3 — not fetched.~~ **Fetched; see §2.5. It makes §2's NO
  stronger, not weaker.**
- ⛔ **That the FFT grating reads through caustics.** Argued from the spectra (broadband noise
  on a narrowband carrier), never measured. And the regime that needs it — a floor with no dark
  texture, where erosion leaves 12.47 px of error — is a floor with no grating to demodulate.
  **This argument may not survive contact with the case it was invented for.**
- **The claim that the 46.7° FOV makes the homography's extra DoF ill-conditioned.** This is a
  sound argument from the geometry, but I did not find a paper stating a FOV threshold, and we
  have not measured the conditioning on our own data.
- **arXiv 2603.13917 is dated 2026** and postdates my training cutoff; the specific
  CosPlace-aliasing quote comes from a search summary rather than a fetch of the paper body.
  ⚠ Note that §4's verdict does **not** rest on it — it rests on the geometric argument in
  §4.1, which needs no citation.
- **Pi 5 latency of `depth_anything_v2_small.onnx`** — a DINOv2 ViT-S/14 encoder sitting in
  our own tree, never timed. The cheapest possible way to replace every estimate in §2.
