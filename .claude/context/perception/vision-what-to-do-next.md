# The vision subsystem — what to add, what to fix, what to stop

> Written 2026-09-23 from three SOTA dossiers (2313 lines, in
> [`sota/sources/vision-*.md`](../sota/sources/)) **plus** measurements taken the
> same day on the vehicle with the Hailo live and the global-shutter camera
> connected.
>
> **The goal, as stated:** *never lose detection in workable conditions, while
> keeping frames as fast as possible so the control loop stays responsive.*

---

## ⭐ The one-paragraph answer

**The detector is not the problem and neither is ROS.** The forward camera
delivers **30 Hz** into a chip that could do 98, and our models are compiled
**multi-context**, which costs 4.3× of chip time *and* blocks the 2.7–4.3× that
async pipelining would otherwise give. Fix those two and the forward path is
bounded by the camera rather than by us, with **~7.8 ms less photon-to-decision**.

⛔ **Nothing on the "better velocity than optical flow" list survives contact with
our own numbers** — the incumbent's algorithmic error is ~0.07 % and the
literature's headlines measure a thing it has already saturated (§4).

⭐ **The one genuine perception gap is caustics on an UNTEXTURED floor** — 12.47 px
of surviving error where a tiled floor is already down to 0.09 px (§6). That, not
speed, is what a new front-end would be for.

---

## 1. ⛔ What we are WASTING — ranked, all measured

### 1.1 The forward camera is the real ceiling

`pi_forward` is a **Fantech**, and it saturates — counted by *distinct header
stamps*, not topic `hz`:

| requested | delivered |
|---|---|
| 30 | 28.03 Hz |
| **60 (shipped)** | **30.18 Hz** |
| 90 | 30.18 Hz |

End to end: **image 30.03 Hz, detections 27.11 Hz, CPU 89.3 % idle.**

⭐ **The chip is idle two-thirds of the time on the forward path.** The Sonix
global-shutter unit — the *downward* camera — does **211 Hz** at 640×360 and
**120 Hz** at 720p ([§17.1](../measured-bars.md)). A second Sonix-class unit on
the forward mount is the **single largest vision win available, and it is a
purchase, not a sprint.**

⚠ And the consumer ticks at **50 Hz** (`VISION_LOOP_HZ_SROT`, soaked 49.86 Hz).
Above ~50 delivered, extra frames buy **latency and freshness, not rate** — so
the target is 50–60 Hz on the forward camera, not 200.

### 1.2 Multi-context compilation — 4.3× of chip, and it blocks pipelining

[§14.4](../measured-bars.md), measured through HailoRT's `create_infer_model`:

| model | contexts | sync | async (8) | gain |
|---|---|---|---|---|
| `yolov8n` stock | **1** | 153.3 Hz | **419.7 Hz** | 2.74× |
| `yolov8s` stock | **1** | 112.7 Hz | **479.6 Hz** | 4.25× |
| `sauvc_sim` **ours** | **3** | 98.4 Hz | 98.4 Hz | **1.00×** |
| `grr_lowth` **ours** | **3** | 98.2 Hz | 98.2 Hz | **1.00×** |

**Single-context models pipeline; multi-context models cannot** — they serialise
on weight swaps, so there is nothing to overlap. Ours cost **10.16 ms** of chip
against yolov8n's **2.38 ms**.

⭐ **The prize is LATENCY, and only latency.** ~7.8 ms less photon-to-decision —
the number the control loop actually feels.

⚠ **Not "room for a second network", which is how I first framed it.** The Pi is
**72.8 % idle** with both perception rungs running, so freed *chip* time is not a
resource we are short of. Spending it would mean moving work off an idle CPU onto
a busy accelerator. The latency case stands on its own; the headroom case does
not.

### 1.3 ⚠ The OpenCV fallback halves the frame rate

`cameras/webcam.py:87` sets `CAP_PROP_BUFFERSIZE = 1`. Measured: **1.8–2.0×
loss**. The flight path uses `v4l2` and is fine — but `_build_v4l2` **falls back
to `webcam`** when the device is not a real V4L2 node, and **nothing checks the
achieved rate against the profile's request.** A camera that fell back would run
at half rate silently.

---

## 2. ⭐⭐ What we can ADD — the short list that survived

### 2.1 XFeat on the Hailo — ⚠ PROVEN POSSIBLE, AND DEMOTED ANYWAY

⛔ **This section was written before the research agent's final position, which
argues against it — and the argument is right.** Recorded in full because
"possible" and "worth doing" are different questions and the gap between them is
the useful part.

**A working Hailo-8 HEF already exists** ([`guyp98/accelerated_features`](https://github.com/guyp98/accelerated_features),
from the [Hailo community thread](https://community.hailo.ai/t/xfeat-compilation/14926)),
and the dossier verified it by downloading and loading the artifacts rather than
reading a claim. Only XFeat's **input normalisation — 2 nodes of 88** — stays on
the CPU. The whole backbone and both heads compile.

Today XFeat runs on the **CPU at 33 ms / 320×240**, forward camera only, as the
anchor rung. On the chip it becomes cheap enough to run on **both** cameras.

⛔ **But the trade is wrong, and the sibling dossier said so first.** The
argument is not about milliseconds: **the Pi is 72.8 % idle with both rungs
running.** Moving XFeat to the chip spends **scarce Hailo duty cycle** — which
currently feeds our only working perception capability at 98 Hz — to buy back
**abundant CPU**. A 6–8× speedup of something that is not the bottleneck is not
a win.

⛔ **And the specific HEF is worse than it looks.** That fork's graph takes
**three inputs**, including the pixel-unshuffled tensor — roughly **double the
bytes over the ×1 PCIe link that is already the constraint**. It also
L2-normalises the 64-D descriptor and softmaxes the keypoint map **in INT8,
on-chip**, which *sharpens* the quantisation risk. Our own export keeps both on
the host in float and is the safer artifact.

⚠ **XFeat★ is not a flag for us.** Its refinement MLP is a separate module
absent from our ONNX export — a day of work, not a toggle.

⚠ **And every laptop figure in the XFeat literature needs repricing**: the paper's
CPU is an i5-1135G7 at VGA, and **our Pi is 2.4–4× slower than it** on the same
graph.

**So the falsifier, if this is ever reopened:** a HEF that compiles is not a
result — one that still reads 4/4 on our own murky clips *through INT8* is.

### 2.2 Segmentation is already on the chip

`yolov8n_seg` measures **85.2 Hz end to end**. Masks feed plane fitting and the
floor normal — real geometry for localization, at a rate we already have.

---

## 3. ⛔ What to STOP considering — refuted, with reasons

| candidate | why not |
|---|---|
| **RAFT / NeuFlow / GMFlow** dense flow | ⛔ **Architectural.** All use `GridSample`, which the Hailo parser does not support. Not a porting gap |
| **DPVO / DROID-SLAM / TartanVO** | ⛔ No ONNX path; DROID needs 8 GB frontend + 24 GB backend |
| **SAM2 / EdgeTAM / XMem / Cutie** | ⛔ Below our LK follower's **125 Hz**. And SAM2 underwater is good *when you hand it the answer* — the prompt does the work |
| **LightGlue / SuperGlue** matchers | ⛔ Hailo's parser rejects `einsum` attention. Named error, not a guess |
| **DINOv2 / DINOv3** | ⛔ Only ConvNeXt distillations are even candidates, and **INT8 on a self-supervised embedding has no label to defend it** |
| **NetVLAD / CosPlace place recognition** | ⛔ Settled **geometrically**: a periodic tile floor aliases at the tile pitch for *every* appearance-based global descriptor |
| **Test-time augmentation** | ⛔ Worse than "no" here — it is a wide interface for a measured non-gain |
| **Image preprocessing** | ⛔ Already measured in 17 configurations: never positive; on the gate it destroyed 95 % of detections |

---

## 4. ⭐⭐ The reframing that matters most: velocity

The flow estimator's **algorithmic** error is not 3.4 % — it is **~0.07 %**.
The synthetic control, same console, same optics, exact truth: **29.98 / 29.99 /
30.02 cm, max error 0.02 cm**; against warped-truth frames, **0.006–0.40 px**.

The often-quoted 1.09 cm was measured **in air, by hand, against a tape**, and
`measured-bars.md` says so itself: *"the operator's tape and hand are inside our
error bar — an upper bound on the sensor's error, not a measurement of it."*

⛔ **So every headline in the velocity literature — RAFT's EPE, NeuFlow's 4.33,
XFeat's inlier counts — measures something our incumbent has already saturated.**

⭐ **A replacement must therefore win on a FAILURE REGIME, not on accuracy:**

1. does it produce usable motion on a **bare pool floor**, where LK gets no tracks?
2. does it **reject caustic** motion — which is real image motion, not noise?
3. does it survive **yaw ≥ 1.128 rad/s**?

**Those three questions are the entire brief for any velocity work.** XFeat is
the only candidate that plausibly answers (1), because it finds features where
corner detectors do not.

---

## 5. The order to do it in

| # | action | kind | why it is first |
|---|---|---|---|
| 1 | **A faster forward camera** | purchase | unlocks 2× on the forward path; everything else is downstream of a 30 Hz feed |
| 2 | **Recompile our models single-context** | build step | 4.3× chip time, and it is the precondition for async AND for a second network |
| 3 | **Async inference in the detector node** | code | 2.7–4.3×, but *only after* (2) — worth 1.00× before it |
| 4 | **XFeat on the DOWNWARD camera — on the CPU** | experiment | the only item offering a capability we lack. Bare-floor footage, Shi-Tomasi vs XFeat keypoint and inlier counts on the existing 30 cm-slide recordings. ⛔ **Not caustics** — see §6 |
| 5 | **Rate assertion at bring-up** | guard | catch a silent fallback to half rate before a run, not after |
| — | ~~XFeat on the Hailo~~ | ⛔ **demoted** | possible, but trades scarce chip for idle CPU — §2.1 |

### 5.1 ⭐ Two measurements that cost almost nothing and settle a lot

1. **Bench ROOT-SIFT on the five archive clips — as a CONTROL, not a candidate.**
   It has never been run here (`grep -i sift` finds nothing in `src/mongla_vision/`).
   It will lose on speed. But if it *also* reads 4/4, then our 4/4 says **the
   clips are easy**, not that XFeat is special. That is a truth test rather than
   an agreement test, and it is the cheapest way to find out whether our headline
   feature result means anything.
2. ⭐ **Time `depth_anything_v2_small.onnx` on the Pi.** It is **already in our
   tree**, it *is* a DINOv2 ViT-S/14 encoder, and it has **never been timed**.
   One measurement replaces every estimate in the DINO section — including the
   verdict that DINO is out of reach.

---

## 6. ⛔ The real open gap: caustics on an UNTEXTURED floor

Not what I would have guessed, and the dossier corrected itself mid-write.

On a **tiled** floor the problem is already solved and shipped: a **7×7 grey
erosion** takes **11.19 px → 0.09 px** against a known 13.4 px baseline, over
three real floors of RoboSub caustics (detector: top-hat 9×9, threshold 0.07,
**4.73 ms/frame** on the Pi).

⛔ **But on a floor with no dark texture of its own — slalom, plain concrete —
12.47 px of error survives.** And the FFT-demodulation idea does not rescue it:
**a floor with no grating has nothing to demodulate.**

⭐ **That is the one genuine perception gap this whole sweep found**, and it is
also the one that maps onto a competition floor. It is the strongest reason to
try XFeat on the downward camera — not speed, and not caustics on tiles.

---

## 7. An outside confirmation of a rule we already had

[arXiv 2507.21715](https://arxiv.org/html/2507.21715v1) measured underwater image
enhancement against ORB-SLAM3: inliers fell **442.21 → 328.03**, and it produced
**zero loop closures across all five runs** where the *original* imagery achieved
them.

⭐ We banned preprocessing on our own evidence (17 configurations, 95 % of gate
detections destroyed). **This is independent confirmation from a different
pipeline**, and the test in our repo that enforces the rule should cite it.

⚠ **Nothing above is a new algorithm**, and that is deliberate. The dossiers
looked hard for one and the honest answer is that our ladder, our flow estimator
and our tracker are not what is costing us — **a 30 Hz camera and a compilation
flag are.**
