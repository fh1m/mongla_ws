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
async pipelining would otherwise give. Fix those two and the forward path goes
from 27 detections/s to something bounded by the camera, with ~8 ms of chip
freed per frame — **which is exactly the room needed to put XFeat on the Hailo,
which is proven possible.** Nothing on the "better velocity than optical flow"
list survives contact with our own numbers.

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

⭐ **The prize is not 480 Hz.** It is **~7.8 ms of chip freed per frame**, and
**~7.8 ms less photon-to-decision** — the number the control loop feels.

### 1.3 ⚠ The OpenCV fallback halves the frame rate

`cameras/webcam.py:87` sets `CAP_PROP_BUFFERSIZE = 1`. Measured: **1.8–2.0×
loss**. The flight path uses `v4l2` and is fine — but `_build_v4l2` **falls back
to `webcam`** when the device is not a real V4L2 node, and **nothing checks the
achieved rate against the profile's request.** A camera that fell back would run
at half rate silently.

---

## 2. ⭐⭐ What we can ADD — the short list that survived

### 2.1 XFeat on the Hailo — proven, not speculative

**A working Hailo-8 HEF already exists** ([`guyp98/accelerated_features`](https://github.com/guyp98/accelerated_features),
from the [Hailo community thread](https://community.hailo.ai/t/xfeat-compilation/14926)),
and the dossier verified it by downloading and loading the artifacts rather than
reading a claim. Only XFeat's **input normalisation — 2 nodes of 88** — stays on
the CPU. The whole backbone and both heads compile.

Today XFeat runs on the **CPU at 33 ms / 320×240**, forward camera only, as the
anchor rung. On the chip it becomes cheap enough to run on **both** cameras.

⛔ **The risk is INT8 on the 64-D descriptor**, and the dossier says it lands
exactly on the thing we care about. **Falsifier before adoption:** match the
INT8 HEF against the float ONNX on our own murky clips and compare inlier
counts — not on clean natural imagery, where published rankings do not transfer.

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
| 4 | **XFeat on the Hailo, downward first** | integration | answers the bare-floor failure regime; HEF already exists |
| 5 | **Rate assertion at bring-up** | guard | catch a silent fallback to half rate before a run, not after |

⚠ **Nothing above is a new algorithm**, and that is deliberate. The dossiers
looked hard for one and the honest answer is that our ladder, our flow estimator
and our tracker are not what is costing us — **a 30 Hz camera and a compilation
flag are.**
