# SOTA sweep — never lose the target, and keep the frames fast

**The goal, as the project lead stated it:** *"the vision never losing detection in ideal
conditions while keeping frames as fast as possible for control-system responsiveness."*

Two halves that pull against each other. Everything here is judged on both.

Status legend on every claim: **REAL-HARDWARE** (measured on a physical device / in water) ·
**BENCH** (GPU/desktop benchmark, no vehicle) · **SIM/MODEL-ONLY** · **UNVERIFIED** (seen in a
search result, source not fetched — these live in §10 and nowhere else).

Rules from [`../README.md`](../README.md): a citation is a URL that was actually fetched; every
candidate move carries its falsifier; retractions stay.

---

## 0. The numbers this dossier is measured against, reconciled

The brief that commissioned this sweep quoted **98 Hz / 10.2 ms**; `CLAUDE.md` quotes
**80.9 Hz standalone / 53.9 Hz through the graph**. Both are in
[`../../measured-bars.md`](../../measured-bars.md) and they are **not** in conflict — they are
three different things, and using the wrong one is how a move gets mis-ranked:

| figure | what it is | where |
|---|---|---|
| **10.20 ms / 98.01 Hz** | `grr_lowth`, end-to-end **batch 1**, frame → letterbox → chip → NMS decode → boxes, 2026-09-23 | `measured-bars.md` §14.3 |
| **80.9 Hz** | earlier standalone detection-path figure | `measured-bars.md` L59 |
| **53.9 Hz** | **delivered through the ROS graph** | `measured-bars.md` L59 |
| 477.85 FPS | `hailortcli benchmark --hw-only` on stock `yolov8s` | ⛔ **retracted**, §14.1–14.2 — not a rate a vehicle can have |

⛔ **The "graph costs 45 %" reading is RETRACTED and this dossier must not repeat it.**
`measured-bars.md` **§17.3** overturned §14.3 the same day, with a real ROS node, a real camera
and a real Hailo publishing a real `Detection2DArray` on a real topic:

| | rate |
|---|---|
| Hailo standalone, from files | 97.47 Hz |
| **real ROS node + real camera, on the topic** | **94.11 Hz** (p95 10.75 ms) |

> "**The graph costs 3.4 %, not 45 %.** A correctly-written node keeps essentially all of the
> chip. So if our own detector delivers 53.9 Hz, the loss is **in our node**, not in ROS … The
> 53.9 Hz figure needs re-measuring against the current code before anything is built on it."

⛔ **And the real ceiling is neither the chip nor the graph — it is the forward camera.**
`config.py` `'pi_forward'` is the **Fantech**, and its measured rate, counted by **distinct header
stamps** (a topic `hz` cannot tell a real frame from a republished one):

| requested | delivered |
|---|---|
| 15 | 14.63 Hz |
| 30 | 28.03 Hz |
| **60 (shipped)** | **30.18 Hz** |
| 90 | 30.18 Hz |

End to end at the shipped setting: **image 30.03 Hz, detections 27.11 Hz, CPU 89.3 % idle.** The
Sonix global-shutter unit — the one that does 210 Hz — is the **downward** camera
(`measured-bars.md` §17.1; the two profiles were once swapped, which is the round-38 trap).

⭐ **So on the forward path the chip is idle two thirds of the time.** 98 Hz of detector behind a
30 Hz camera is not a throughput problem at all. And the consumer, `VISION_LOOP_HZ_SROT`, ticks at
**50 (soaked 49.86 Hz, 0.00 % late ticks)** — so above ~50 Hz delivered, extra frames buy the
control loop **latency and freshness, not rate**. §8 works this through; **every rate claim in
§1–§7 must be read against 27–30 Hz on the forward camera, not 98.**

**Which camera the FOV belongs to.** The **46.7° ± 0.7 in water** is the *measured* figure
(`measured-bars.md` L61, 25 views, `calibrateCameraRO`, held-out validated). ⚠ `measured-bars.md`
L2035 separately records the forward calibration at **fx 851.2 / HFOV 73.9° in air** and L2235
records that **the downward camera's in-water FOV is unknown**, with L1108 documenting a live
mix-up of which intrinsics belonged to which lens. **The table below is therefore an order-of-
magnitude instrument, not a calibration.** If the forward camera is nearer ~59° in water the
pixel counts shrink by ~20 %; every verdict in §4 survives that, because none of them turns on
±20 %.

**Geometry this dossier uses** (derived from the measured FOV, with the caveat above): at
46.7° across a 640-px frame, **0.073°/px**, i.e. **≈13.7 px per degree**.

| target | 5 m | 8 m | 12 m |
|---|---|---|---|
| 1.5 m gate (span) | 233 px | 146 px | 98 px |
| 0.6 m drum | 94 px | 59 px | 39 px |
| 0.2 m torpedo hole | 31 px | 20 px | 13 px |

Hold these numbers. Q4 (small objects / tiling) is decided by them and by nothing else.

### 0.1 ⭐ The question the brief did not ask, which this sweep thinks is the real one

The goal says *"never losing detection **in ideal conditions**"*. ⚠ **Nothing in §1–§7 found a
method that improves detection continuity in ideal conditions**, for a reason worth stating
plainly: **in ideal conditions the detector already works.** The 71 real detection gaps the ladder
was sized against, and the p99 **2.418 s** gap `../vision.md` §3 V-8 records, are not
ideal-condition events — they are turbidity, occlusion, and the target leaving a **46.7°** frame.

So the literature answer to the goal as worded is: **the continuity problem in ideal conditions is
solved, and the remaining loss is not the detector's.** What this sweep found instead, reading the
vehicle's own numbers, is that in ideal conditions the pipeline is **fed 27–30 Hz by a camera in
front of a 98 Hz detector, into a 50 Hz consumer** (§8.1). That is the ideal-conditions defect.
It is not a perception-method problem and no paper here fixes it.

⛔ **The corollary is a scope warning.** If "never lose the target" is meant to include the hard
frames, then §1–§7 say the ladder is already the right architecture and the gains are elsewhere:
calibration (§5), a second opinion (`../vision.md` V-5), and — the one place the evidence says
software cannot close the gap — **sonar** (`../vision.md` V-8: 2 of 5 top TDRs carry it, and "no
software substitute exists in the literature").

---

## 1. Q1 — detection continuity: what beats detector + tracker + flow + anchor?

**Short answer: nothing in the 2024–2026 literature beats the ladder we already run, on our
hardware, for our failure mode.** Three families were checked. Two are the wrong problem shape
and one is already inside our stack.

### 1.1 Track-before-detect — **NO. Wrong field.**

TBD integrates raw sensor energy over a motion hypothesis *before* thresholding, so that a
target too weak to detect on one frame is detected across N. The entire deployed literature is
**radar and infrared point targets** — unresolved blobs of a few pixels on a near-static
background. A search of the 2024–2026 small-target literature returns infrared-small-target
segmentation reviews and nothing on optical underwater TBD:
- Review: *Infrared Dim Small Target Detection Networks: A Review*, PMC11207645 (2024) —
  https://pmc.ncbi.nlm.nih.gov/articles/PMC11207645/ **BENCH**, IR only.
- *Deep Learning based Infrared Small Object Segmentation: Challenges and Future Directions*,
  arXiv:2502.14168 (Feb 2025) — https://arxiv.org/html/2502.14168v1 **BENCH**, IR only.

TBD's premise is a *sub-threshold but persistent* signal on a *stationary* background. Our
target at 5 m is **233 px across** (§0) on a camera mounted to a moving, rolling vehicle. The
premise does not hold. ⛔ **Rejected — not edge-infeasible, simply not our problem.** The one
idea worth stealing from it is stated in §3.

### 1.2 Streaming / temporal detection (StreamYOLO, LongShortNet, DAMO-StreamNet) — **NO, for an arithmetic reason.**

- StreamYOLO, arXiv:2207.10433 (21 Jul 2022) — https://arxiv.org/abs/2207.10433 **BENCH**
  (Argoverse-HD). Core idea, verbatim from the abstract: the world changes during inference, so
  the detector is given "the capacity of predicting the future". **+4.7 sAP / +8.2 VsAP** over
  the baseline, via a Dual Flow Perception module and a Trend Aware Loss.
- LongShortNet, arXiv:2210.15518 (ICASSP 2023) — https://arxiv.org/pdf/2210.15518 **BENCH**:
  **37.1 sAP**, 42.7 at large input, "**an almost negligible increase in latency (around
  0.1+ ms)**" over StreamYOLO's 36.1.
- DAMO-StreamNet, IJCAI 2023, arXiv:2303.17144 — https://www.ijcai.org/proceedings/2023/0090.pdf
  **BENCH**: **37.8 sAP** normal, **43.3** large.

Every one of these is measured on **Argoverse-HD at 30 FPS**, where one frame is **33 ms** and
the scene contains cars crossing the frame. The whole gain is "predict one frame of ego-and-target
motion forward".

**Our arithmetic.** Frame interval through the graph is **1/53.9 = 18.6 ms**. At §0's
155 px/m for a 1.5 m gate at 5 m, and a realistic closing speed of 0.5 m/s, one frame of motion
is **9.3 mm ≈ 1.4 px**. Even at 2 m/s it is 5.7 px — inside the box-regression noise. The entire
premise of streaming perception is a latency gap that our chip has already closed.

⛔ **And we already do a version of the one thing it does.** `tracking/kalman.py` runs
`predict()` on frames where the tracker held the track without a detection, and gives up after
`max_predict_frames` (default **5**) consecutive predicted frames — so the track is propagated,
not merely dropped.

⚠ **Stated precisely, because the difference is real:** our filter is **constant-velocity in
image space** and takes **no gyro input** — it has no model of the *vehicle's* motion, only of the
box's recent pixel velocity. StreamYOLO's prediction is learned from the scene. So the honest
claim is *not* "we already do it"; it is "we do the linear part, and the non-linear part is worth
**≈1.4 px** at our frame interval." At 18.6 ms even a hard yaw contributes a displacement the
constant-velocity term absorbs. ⭐ **If this is ever reopened, the cheap version is not a retrained
detector head — it is feeding the board's 50 Hz gyro into `kalman.py`'s predict step**, which is
the same information at effort 1 instead of effort 5.

| | verdict |
|---|---|
| (a) edge-feasible? | would need a full custom retrain + DFP/TAL heads compiled to Hailo; **no zoo entry for any streaming detector** |
| (b) buys over what we have | ≈1.4 px of one-frame prediction we already get from the Kalman |
| (c) adoption cost | a new training pipeline and a new compile — effort 5 |
| (d) underwater evidence | **none. Zero underwater streaming-perception papers found.** |

**Rejected.** Falsifier, if anyone reopens it: measure the p95 pixel displacement of a tracked
box between consecutive delivered frames on real footage. If it is under ~5 px, prediction-in-the-head
cannot buy anything. We predict it is 1–2 px.

### 1.3 Memory-based trackers — the only family with a real claim. See §2.

### 1.4 What the evidence actually says about our ladder

The honest reading: our four-rung ladder (detector → OC-SORT → LK follower → XFeat anchor)
**is** the edge instantiation of what the memory-tracker literature does with a learned memory,
and [`../vision.md`](../vision.md) §2.4 already established that the blackout is a geometry
problem and the anchor is the right shape of answer. The 2024–2026 work does not offer a fifth
rung. It offers a *better third rung*, and that is §2.

⭐ **The largest continuity win available is not a new rung at all — it is
`../vision.md` V-5/V-6: a second, independent estimator that can contradict a confident wrong
one.** A ladder with four rungs and no cross-check fails silently when rung 2 hands rung 3 a
wrong box. Nothing in this sweep changed that conclusion; it strengthened it.


## 2. Q2 — SAM2 / EfficientTAM / EdgeTAM / XMem / Cutie: masks through occlusion, on this chip

**Answer: (a) not Hailo-compilable today, (b) real accuracy through occlusion but on GPUs/phones,
(c) high, (d) underwater evidence exists and it is a warning, not an endorsement.**

### 2.1 Is any of it on the Hailo-8?

**No.** The Hailo Model Zoo's HAILO8 instance-segmentation table was fetched
(https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8/HAILO8_instance_segmentation.rst)
and contains **17 models, all YOLO-family**, and **zero SAM variants**:

| model | FPS (batch 1) | mAP-seg |
|---|---|---|
| `yolov8n_seg` | **528** | 29.7 |
| `yolov5n_seg` | 464 | 22.9 |
| `yolov5s_seg` | 331 | 30.7 |
| `yolov11n_seg` | 132 | 31.2 |
| `yolov8s_seg` | 107 | 36.4 |
| `yolov11s_seg` | 90.9 | 37.2 |
| `yolov5m_seg` / `yolov8m_seg` | 68.7 / 51.4 | 36.6 / 40.1 |
| `yolact_regnetx_1.6gf` | 59.0 | 27.2 |
| `yolov11m_seg` … `yolo26x_seg` | 37.3 … 10.8 | 40.7 … 44.6 |
| **FastSAM, MobileSAM, SAM, SAM2, EdgeTAM, EfficientTAM** | **absent** | — |

⚠ Those FPS are the zoo's published figures, which `measured-bars.md` §14.2 proved are
**`hw_only`-style numbers, not rates a vehicle can have** — our own `yolov8n_seg` measures
**11.74 ms / 85.2 Hz end to end with masks at 6 detections** (`measured-bars.md`, Hailo
segmentation decode, 2026-09-11). Read the zoo column as a ranking, never as a rate.

The only community attempt on record: a user compiling **FastSAM-S for Hailo-8L on 19 Aug 2024**,
which **failed at the allocator** — `"[error] Mapping Failed ... No successful assignment for:
format_conversion2, concat18, feature_splitter9, shortcut_softmax1..."` —
https://community.hailo.ai/t/sam-2-in-hailo8l/2614/4. **No Hailo staff reply; thread unresolved
two years later.** That is the state of SAM on this chip.

**Why it will stay that way, structurally.** SAM2's continuity comes from *memory attention* —
stacked transformer blocks doing cross-attention against a **memory bank of past frames**
(SAM 2, arXiv:2408.00714 — https://arxiv.org/html/2408.00714v1). That is (i) attention, which
the Dataflow Compiler handles poorly enough that the FastSAM failure above is at the allocator,
and (ii) **stateful across frames**, which a dataflow accelerator compiled as a feed-forward
graph does not express at all. You would have to run the memory bank on the Pi 5 CPU and only the
image encoder on the chip — and EdgeTAM's own finding is that **memory attention, not the
encoder, is the latency bottleneck**.

### 2.2 What the fast variants actually cost, on their own hardware

- **EdgeTAM** (CVPR 2025, arXiv:2501.07256, 13 Jan 2025) — https://arxiv.org/abs/2501.07256
  **REAL-HARDWARE (phone)**: **22× faster than SAM 2**, **16 FPS on an iPhone 15 Pro Max**,
  unquantised. J&F **87.7 DAVIS-2017 / 70.0 MOSE / 72.3 SA-V val / 71.7 SA-V test**. Its stated
  innovation is a 2D Spatial Perceiver precisely *because* "memory attention blocks are also the
  latency bottleneck".
- **Cutie** (CVPR 2024 Highlight, arXiv:2310.12982, 19 Oct 2023 / rev. Apr 2024 —
  https://arxiv.org/abs/2310.12982, **fetched**) **BENCH (GPU, device not stated)**: **+8.7 J&F
  over XMem** on **MOSE** at comparable running time, and **+4.2 J&F over DeAOT while running 3×
  faster**. The per-variant FPS figures (Cutie-small 45.5 vs XMem 22.6 vs DeAOT 11.7) and the
  memory figures come from a search snippet, **not** from the fetched abstract — see §10.

**Translate to our box.** 16 FPS is the number an A17 Pro with a 35 TOPS NPU achieves. The
Hailo-8 is 26 TOPS but is a *dataflow* part with a **~9.3 ms PCIe round trip at batch 1**
(`measured-bars.md` §14.2) — and EdgeTAM cannot use it at all, so it falls to the Pi 5's four
Cortex-A76 cores. A phone-NPU 16 FPS becomes, generously, **low single-digit Hz on CPU**.
⛔ **That is below our LK follower's 125 Hz and below the 30 Hz floor in `measured-bars.md`.**
I did not find a published Pi-5 EdgeTAM figure; it goes in §10.

### 2.3 (d) Underwater evidence — the finding that decides it

*Evaluation of Segment Anything Model 2: The Role of SAM2 in the Underwater Environment*,
arXiv:2408.02924 (6 Aug 2024) — https://arxiv.org/abs/2408.02924 **BENCH** (UIIS, USIS10K):

> "When using the ground truth bounding box as prompt, SAM2 performed excellently in the
> underwater instance segmentation domain." … "**When running in automatic mode, SAM2's ability
> with point prompts to sense and segment underwater instances is significantly degraded.**"

⛔ **Read that carefully: SAM2 underwater is good when you hand it the answer.** The prompt in
our pipeline would come from the detector — and the whole point of the exercise is the frames
where *the detector has failed*. The one paper that tested SAM2 underwater found its
self-prompting mode degrades exactly where we would need it. The abstract carries no mIoU/J&F
numbers; those go in §10.

Corroborating, from the same year: USIS10K / USIS-SAM (ICML 2024, arXiv:2406.06039 —
https://arxiv.org/abs/2406.06039) built a 10 632-image, 7-category underwater salient instance
dataset **and had to add an Underwater Adaptive ViT encoder and a Salient Feature Prompter
Generator** to make SAM work there. Stock SAM was not sufficient; it needed an underwater-specific
retrain *and* a learned prompter.

### 2.4 Verdict

| | |
|---|---|
| (a) edge-feasible | **no.** Zero SAM/TAM entries in the Hailo-8 zoo; the one FastSAM attempt failed at the allocator; memory attention is stateful and does not map to dataflow. CPU fallback ≪ 30 Hz. |
| (b) buys over ours | a *pixel mask* that survives occlusion — genuinely better than a box. But `../vision.md` §6 already records that **no controlled study of masks vs boxes for alignment error exists**, so the benefit is unpriced. |
| (c) adoption cost | effort **5**: a new runtime, a CPU-side memory bank, an underwater retrain (per USIS-SAM), and a prompt policy for the case where the detector already failed. |
| (d) underwater evidence | **exists and cuts against**: automatic/point-prompt mode degrades underwater (arXiv:2408.02924). |

⛔ **Rejected for this vehicle.** Falsifier if reopened: run EdgeTAM on a Pi 5 against 30 s of
our own gate footage containing a real detection gap, and measure (i) Hz and (ii) whether the mask
survives the gap **when prompted only from the last good detector box**. If it is under 10 Hz or
the mask drifts off the gate within the 2.418 s p99 gap, it is closed permanently.

⭐ **The cheap 80 % of this idea is already on our chip.** `yolov8n_seg` measures **85.2 Hz end
to end with masks** on the vehicle. If masks are worth anything to us, that is where to find out
— not by importing a memory tracker. That question is §7.


## 3. Q3 — temporal fusion, consecutive-frame detection, test-time augmentation

**Answer: the fusion half is already in the stack and the TTA half is a direct trade against the
second half of the goal.** But there is one narrow version worth an experiment.

### 3.1 We already fuse across consecutive frames — and ⚠ the brief names the wrong tracker

⚠ **Correction to the brief, from the tree.** The shipped default is **not** ByteTrack.
`tracker_node.py:170` declares `tracker_type` defaulting to **`'ocsort'`** — the Roboflow
`trackers` library — with the inline comment *"ocsort (Roboflow, default — best dropout
recovery)"*. `ByteTrackWrapper` (`tracking/bytetrack.py`, the supervision wrapper) is reached only
via `tracker_type:='legacy_bytetrack'`, or as the **fallback when the Roboflow library fails to
import** (`tracker_node.py:96-98`). ⛔ **That fallback silently changes the tracking algorithm**,
which matters for anything tuned against one of them.

So the two mechanisms in play are different, and both already do a form of temporal fusion:

- **ByteTrack's** contribution is the **second association pass over low-confidence detections**
  (`bytetrack.py:32-33`, "IoU threshold for the second association pass (low-confidence…)") —
  weak detections are matched to tracks instead of discarded.
- **OC-SORT's**, the one actually shipped (CVPR 2023, arXiv:2203.14360, 27 Mar 2022 / final
  16 Mar 2023 — https://arxiv.org/abs/2203.14360, **fetched**, **BENCH**): rather than trusting
  the linear state estimate through an occlusion, it uses "object observations … to compute a
  **virtual trajectory over the occlusion period** to fix the error accumulation of filter
  parameters" — i.e. once the target reappears it **re-derives the track backwards across the
  gap** instead of carrying the drift forward. SOTA on MOT17, MOT20, KITTI, head tracking and
  "especially **DanceTrack**". **Runs at 700+ FPS on a single CPU**, so it costs us nothing.

  ⭐ **That is the correct rung for our failure mode and it is already shipped.** `tracker_node.py`'s
  comment "best dropout recovery" is right for a citable reason. ⚠ Note the benchmark it wins
  hardest on is **DanceTrack** — which `../vision.md` §5 already flagged, rejecting Re-ID, as "a
  benchmark of identical-looking targets". The caveat there applies here too: OC-SORT's headline
  strength is measured on a target population unlike our props.

Either way the point stands and hardens: **any proposal of the form "detect on N consecutive
frames and fuse" must state its marginal delta over the shipped tracker, not over one-shot
detection.** The literature's framing is against one-shot, so its headline numbers do not
transfer.

### 3.2 Test-time augmentation — **NO, and it is worse than "no" here.**

Ultralytics' own TTA documentation, **fetched** —
https://docs.ultralytics.com/yolov5/tutorials/test-time-augmentation — **BENCH**, COCO val2017
(5 000 images) on a **Tesla P100-PCIE-16GB**: mAP@0.5:0.95 **0.504 → 0.516**, mAR
**0.681 → 0.696**. Stated cost: "inference with TTA enabled will typically take about 2-3X the
time of normal inference", and the measured rows are **22.4 ms → 80.6 ms per image, ≈3.6×** —
worse than the prose claims, because TTA is run at 832 px rather than 640.

**On our vehicle that trade is inverted.** +1.2 mAP for **2–3.6×** the 10.20 ms loop means
**98 Hz → 27–49 Hz standalone**. ⚠ Against the forward camera's 30 Hz (§0) that might *look*
survivable — but the chip's idle time is the budget §4.4 and §7.3 both want, and TTA spends all
of it on every frame for +1.2 mAP. The project lead's goal has two halves and TTA sells the
second to buy 1 % of the first.

⛔ **Rejected.** It also fails a structural test from `../README.md` rule 6: TTA is a wide,
always-on multiplier applied to *every* frame to help the rare hard one. Compare §4.3, which does
the same work conditionally.

### 3.3 The one version worth an experiment: **a confidence prior from the track, not from the frame**

Strip both ideas to their mechanism and what remains is: *a detection that agrees with where the
track says the target is should be believed at a lower threshold than one that does not.*

- It costs **zero extra inference** — the Kalman prediction exists, the low-confidence
  detections exist (ByteTrack sees them at its second pass), and `tracking/confidence.py` already
  owns the arbitration.
- It is a **spatially-gated threshold**, which is the same shape of idea as ROI-Gated SAHI
  (arXiv:2608.23923 — https://arxiv.org/abs/2608.23923, **BENCH**): "a lightweight proposer to
  localize foreground regions and restrict sliced refinement to informative areas". The
  transferable principle is *spend the extra evidence only where the prior says to look*.
- ⚠ **The honest risk, stated first:** this makes the detector agree with the tracker, which is
  exactly the **agreement-not-truth** failure `../README.md` rule 3 and `CLAUDE.md` §9 both name.
  A wrong track would lower the bar for detections that confirm it, and lock the error in.

**Falsifier, and it must be run before this ships:** take the **71 real detection gaps** the lock
ladder was sized against. Replay them with the track-gated threshold. Measure (i) how many gaps
close and (ii) **how many gaps close onto the wrong object**. If (ii) is non-zero at any gate
setting that meaningfully improves (i), the idea is refuted — a gap is recoverable, a confidently
wrong lock is not.

| | |
|---|---|
| (a) edge-feasible | **yes, free.** No extra inference, no new model, runs in `confidence.py`. |
| (b) buys | closes some fraction of the 71 gaps at zero frame cost |
| (c) cost | effort **1–2**, one file plus the replay harness |
| (d) underwater evidence | **none specific.** The mechanism is domain-neutral; the risk is not. |


## 4. Q4 — small objects at range: resolution, tiling (SAHI), crops

**Answer: SAHI is the wrong tool for our image, a higher input resolution is a bad trade, and a
conditional second-stage crop is the only version that survives the arithmetic.**

### 4.1 Our targets are not small in the SAHI sense. Run the numbers from §0.

SAHI (arXiv:2202.06934, 14 Feb 2022 / final Oct 2022 — https://arxiv.org/abs/2202.06934)
**BENCH**: **+6.8 / +5.1 / +5.3 AP** (FCOS / VFNet / TOOD) inference-only, **+12.7 / +13.4 /
+14.5** with slicing-aided fine-tuning. The datasets are **VisDrone and xView** — aerial imagery,
multi-thousand-pixel frames, targets of a few tens of pixels occupying a vanishing fraction of the
image.

Our regime, from §0's 13.7 px/degree at 46.7° in-water HFOV:

| target | 5 m | 8 m | 12 m |
|---|---|---|---|
| 1.5 m gate | **233 px** | 146 px | 98 px |
| 0.6 m drum | 94 px | 59 px | 39 px |
| 0.2 m torpedo hole | 31 px | **20 px** | 13 px |

A gate is never a small object for us — it is a third of the frame at competition range. **Only
the torpedo hole at ≥8 m enters SAHI's regime**, and a hole that small at that range is also
beyond where the light and the water will carry contrast, which is the real limit
(`../sota-underwater-perception.md` and `../vision.md` §2.1).

### 4.2 The cost SAHI does not print on the tin

Its own framing acknowledges the overhead. ROI-Gated SAHI (arXiv:2608.23923, 25 Aug 2026 —
https://arxiv.org/abs/2608.23923, **fetched**, **BENCH**) exists precisely because SAHI "often
spends substantial compute on background tiles", and finds "**ROI-gating is most beneficial in
sparse scenes**" — speedups **0.96× to 6.90×** across three test images, **mean 3.41×**.
**Our image is exactly that sparse scene.**

⚠ **And read its own result honestly, because it argues against tiling generally:** static
ROI-gating measured **slower than full SAHI (0.88×) with lower accuracy (0.6602 vs 0.7569
mAP@0.5)**; the adaptive policy managed only "**a slight gain of 1.02× over Full SAHI**". The
2026 state of the art in making tiling cheap is a 2 % speedup. That is the ceiling on this
entire direction.

⛔ **N tiles divides our rate by N.** A 2×2 grid takes 98 Hz → 24.5 Hz standalone, which is below
the 30 Hz bar in `measured-bars.md`. A 3×3 takes it to 10.9 Hz. **SAHI is rejected outright.**

### 4.3 Higher input resolution — **no, and the camera settles it before the compiler does**

⚠ **A correction to an earlier draft of this section, kept because the mistake is instructive.**
`measured-bars.md`'s 145.6 ms / 33.1 ms table and its "the already-marginal octagon drops 3/4 →
2/4" belong to **XFeat on the Pi 5 CPU**, not to detector input resolution. They were misread
here first. The right reading of that table is in §4.7.

The real argument is simpler. **The forward camera delivers 640×360 (`config.py` `'pi_forward'`).**
A 640×640 letterbox of a 640×360 frame already contains every pixel the camera produced; raising
the *model's* input to 832 or 960 **upsamples**, and invents nothing. It would also cost a new
compile, a new in-domain calibration set (`../vision.md` V-4: **≥1 024 images or the toolchain
silently uses COCO128**) and a roughly quadratic rate hit, to resample data that does not exist.

⛔ **Rejected.** There is exactly one version of "more pixels" that is real on this vehicle, and
it is a *camera*, not a model input: the Fantech's 640×360 at 30 Hz against the Sonix's measured
**1280×720 at 120.4 Hz** on the downward mount (`measured-bars.md` §17.1). **The forward camera
is the resolution ceiling and the frame-rate ceiling at the same time.** That is a hardware
observation, recorded per `../README.md` rule 4, not smuggled in as a software move.

### 4.4 The conditional second-stage crop — ⛔ **dead on the forward camera as configured**

The idea: when the track's predicted box is small, crop the **native-resolution** region around
the Kalman prediction, letterbox that crop to 640×640 and run **one** extra inference. It is the
right *shape* — conditional rather than always-on, with the tracker as the free proposer that
ROI-Gated SAHI has to learn — and it would cost **+10.20 ms only on the frames that need it**
(10 % of frames → 98 → 89 Hz mean).

⛔ **But there is no native resolution to crop from.** `config.py` `'pi_forward'` is **640×360**.
A crop of a 640×360 frame upsampled to 640×640 contains no pixel the whole-frame pass did not
already see. ⚠ **An earlier draft of this section cited `cameras.yaml:30-31`'s 1280×720 — that is
the `laptop` profile, and `cameras.yaml`'s own header says in full caps that the file is NOT
LOADED AT RUNTIME (defect B49); the loaded copy is `CAMERA_PROFILES` in `config.py`.** The move
was built on a dev-machine number.

**It revives under exactly one condition, and it is a camera change.** The downward Sonix does
**1280×720 at 120.4 Hz** measured. A forward camera of that class would restore a genuine 2×
linear discard to exploit, and *then* this move is the best use of it — better than raising the
model's input (§4.3), because it is conditional.

⚠ Even then: it is **scale-shifted input**. A detector trained on whole frames meets the object at
a scale it never saw, so expect crop-augmented retraining — effort 3, not 2.

**Falsifier (only worth running if the forward camera changes):** on held-out footage of the
smallest real prop, measure recall whole-frame vs crop **at matched confidence**, bucketed by
range. If crop recall does not beat whole-frame in the ≥8 m bucket, the limit was **contrast**,
not pixels, and no resolution fixes contrast.

### 4.5 ⚠ Anchor tuning — **there is no such knob**

The brief asked about anchor tuning. **YOLOv8 and YOLOv11 are anchor-free** — they regress box
distances from grid points and have no anchor priors to tune. There is nothing to adjust, and any
advice to "tune the anchors for small objects" is advice for YOLOv5-era architectures. Closed on
the architecture, not on the evidence.

### 4.6 ⚠ The annotation finding that matters more than any of this

*Why Domain Matters: Domain-Aware Benchmarking of Underwater Object Detection and Annotation
Quality*, arXiv:2607.10575 (12 Jul 2026) — https://arxiv.org/html/2607.10575 **BENCH**
(DUO + RUOD-R + UTDAC2020; 18 852 images, 196 102 annotations). Comparing original RUOD against
re-annotated RUOD-R:

> **small-object domains showed annotation correction rates 4× higher than large-object domains.**

⭐ **Read that as a warning about our own labels.** If small-object annotations are 4× more often
wrong in the published underwater corpora, then our own small-prop labels are the most likely
thing wrong with small-prop recall — and that is a **relabelling** job, not an inference-architecture
job. It is also far cheaper than any move in this section. Check it before building §4.4.


### 4.7 ⭐ What that XFeat table actually says — an owed research item, already paid

`../vision.md` §6 lists "**XFeat on a Raspberry Pi 5**" as research still owed, on the grounds
that the published figure is an i5-1135G7 at VGA. ⭐ **It is not owed. It is measured**, in
`measured-bars.md`, ONNX-exported (2.7 MB), `onnxruntime` CPU, **with the vision stack running**:

| resolution | 1 thread | 3 threads |
|---|---|---|
| 640×480 | 145.6 ms (6.9 Hz) | 88.0 ms (11.4 Hz) |
| **320×240 — the shipped bar** | **33.1 ms (30.2 Hz)** | 18.2 ms (55.1 Hz) |

And 320×240 keeps the lock: every murky clip still 4/4 (56–260 inliers); only the already-marginal
octagon drops 3/4 → 2/4. "So the slow rung runs at 30 Hz on **one core** while the fast rung (LK,
8.0 ms) runs at 125 Hz on another, with the Pi still **72.8 % idle**."

Two consequences worth carrying out of this dossier:
1. `../vision.md` §6's owed item should be **struck** — the number exists.
2. ⭐ **The Pi is 72.8 % idle with the anchor running.** Every "no CPU budget" argument against a
   host-side method in §1–§7 needs to clear that number first. The scarce resource on this vehicle
   is the **chip** and the **camera**, not the CPU.

## 5. Q5 — confidence calibration: when to trust a detection underwater

**Answer: yes, detectors are miscalibrated, the underwater-specific evidence is about domain
*shift* rather than calibration per se, and the fix is cheap, post-hoc and measurable. This is the
best value-per-effort item in the whole sweep after §8.**

### 5.1 What the general literature settled in 2024

*On Calibration of Object Detectors: Pitfalls, Evaluation and Baselines*, ECCV 2024 (Oral),
arXiv:2405.20459 (30 May 2024) — https://arxiv.org/abs/2405.20459 · library:
https://github.com/fiveai/detection_calibration **BENCH** (COCO):

- Existing evaluation frameworks, metrics **and the use of Temperature Scaling** "have notable
  drawbacks leading to incorrect conclusions". ⚠ **So the obvious first move — temperature
  scaling — is the one this paper specifically warns against for detection.**
- They tailor **Platt Scaling and Isotonic Regression** to detection and find post-hoc calibrators
  are "**extremely cheap to build and use**" and "much more powerful and effective than the recent
  train-time calibration methods".
- Headline: **D-DETR with post-hoc Isotonic Regression beats train-time SOTA Cal-DETR by more
  than 7 D-ECE on COCO.**

The metric is **D-ECE** (Detection Expected Calibration Error), which scores confidence against
realised precision jointly with AP — you cannot buy calibration by destroying accuracy.

### 5.2 The underwater-specific evidence

There is no paper that measures D-ECE on underwater detection. What exists measures the thing
that *causes* the miscalibration — domain shift — and it is severe. From arXiv:2607.10575 (same
source as §4.5), **mAP50 gaps within one mixed-domain test set**:

| axis | gap, mAP50 points |
|---|---|
| **colour** (natural vs blue) | **≈33** |
| **visibility** (low vs high) | ≈28 |
| **perspective** (nadir vs front-view) | ≈22 |
| **scale** (small vs large objects) | ≈21 |
| **layout** (sparse vs crowded) | ≈20 |

⛔ **A ≈33-point mAP50 swing between a natural-coloured and a blue image is a 33-point swing in
what a given confidence number means.** A single global `conf` threshold — which is what we ship
— is therefore right in at most one venue and wrong in every other. This is the mechanism behind
our own **29.2 / 72.7 / 68.3 % recall spread** already recorded in `../vision.md` V-1.

See also *Underwater Object Detection Under Domain Shift*, IEEE (2024) —
https://ieeexplore.ieee.org/document/10679365/ **UNVERIFIED-NUMBERS** (paywalled; listed here
for the title only, and again in §10).

### 5.3 ⭐ The move, and why it is the cheapest real one here

Fit an **isotonic regression** calibrator, per camera and per class, on our own labelled footage,
and validate it with a **reliability diagram on a held-out venue** — not a held-out i.i.d. split,
because `../vision.md` §6 already records that *every* published number is an i.i.d. split and
that held-out-session recall does not exist anywhere in the literature. Held-out **venue** is the
condition that matters for us and it is the condition that makes the result novel.

What it changes downstream: `tracking/confidence.py` and `lock_state.py` currently arbitrate on
raw detector scores. Calibrated scores mean the lock ladder's rung thresholds become statements
about **probability the target is really there**, comparable across venues, classes and cameras —
which is the precondition for §3.3 to be safe, and for the "refuse loudly" rule in `CLAUDE.md` §9
to have a principled bar rather than a tuned one.

| | |
|---|---|
| (a) edge-feasible | **yes, and it is free at runtime.** Isotonic regression is a monotone lookup — sub-microsecond, no chip time, no extra inference. |
| (b) buys | a threshold that means the same thing in every venue; the ladder's rungs become probabilistic |
| (c) cost | effort **2**: fit offline on data we already have, ship a table, one file changes |
| (d) underwater evidence | **indirect but strong**: ≈33 mAP50 colour-domain gap is precisely the miscalibration this fixes. No published underwater D-ECE — that gap is ours to fill. |

**Falsifier:** plot the reliability diagram on a held-out venue *before* and *after*. If the raw
detector is already well-calibrated across venues (D-ECE near zero out of the box), the move buys
nothing and is closed. ⚠ **Run the diagram first.** It is an afternoon's work and it decides the
move without writing any vehicle code.


## 6. Q6 — genuinely wasted effort

The repo already owns half this answer. It is not repeated; it is **cited and closed**, per
`../README.md` rule 5.

### 6.1 Already closed in this repo — do not re-research

| closed | where | one-line reason |
|---|---|---|
| **Image enhancement as a preprocessing bolt-on** (CLAHE, dehazing, colour restoration, in any form) | [`../vision.md`](../vision.md) §2.1, §5 · our own 17 configurations, 4 props, 3 venues; **95 % of gate detections destroyed** | four independent studies **plus** the finding that *retraining on enhanced imagery loses, 7 detectors of 7* |
| **Appearance Re-ID on the tracker** | `../vision.md` §5 | +1.7/+1.8 HOTA on MOT17/20; `N × 979 MFLOPs`/frame; no Hailo-8 zoo entry; **0 of 5 top TDRs use it** |
| **RT-DETR / transformer detectors on-chip** | `../vision.md` §5 | absent from the zoo; DETR-R18 is worse than `yolov11n` on both axes |
| **Dense optical flow (RAFT, NeuFlow v2)** | `../vision.md` §5 | ≥4× our 12.34 ms on an Orin Nano GPU we do not carry |
| **LightGlue as the anchor matcher** | `../vision.md` §5 | 0.31 pairs/s on CPU ≈ **3.2 s per pair**, inside a 2 s blackout |
| **Monocular SLAM underwater** | `../vision.md` §5 | AQUALOC: ORB-SLAM3 tracked **1.426 m of 16.212 m** before losing tracking |
| **`hailortcli --hw-only` FPS as a planning number** | `measured-bars.md` §14.2 | a 4.9× that vanishes at batch 1; **not a rate a vehicle can have** |
| **yolov11 as an upgrade** | `measured-bars.md` §14.3 | `yolov11s` = **42.57 Hz** end to end, *below* what we already deliver |

### 6.2 Added by this sweep

| wasted | why, with the number |
|---|---|
| **SAHI / tiled inference** | §4.2. Divides our rate by N for an aerial-imagery problem we do not have — our gate is **233 px at 5 m**. ROI-Gated SAHI's own motivation is that tiling is "highly inefficient when … most of the image contains only background", which describes our every frame. |
| **Test-time augmentation** | §3.2. **2–3× inference** for **+1.2 mAP** (0.504 → 0.516). Takes 98 Hz to 33–49 Hz — at or below what the pipeline already delivers. Sells half the goal to buy 1 % of the other half. |
| **Streaming detectors (StreamYOLO / LongShortNet / DAMO-StreamNet)** | §1.2. Their gain is one frame of prediction at 30 FPS; our frame is **18.6 ms ≈ 1.4 px** of target motion, and `kalman.py` already applies it. Zero underwater papers. |
| **Track-before-detect** | §1.1. A radar/IR point-target method. Our target is 233 px on a moving mount. Wrong premise, not wrong hardware. |
| **SAM2 / EdgeTAM / Cutie / XMem as a lock rung** | §2. No Hailo entry, the one FastSAM compile attempt failed at the allocator, memory attention is stateful and does not map to dataflow, and the **one underwater SAM2 evaluation found automatic/point-prompt mode degrades** — which is the only mode we could use. |
| **Temperature scaling for detector calibration** | §5.1. The ECCV 2024 paper names **TS specifically** as a source of "incorrect conclusions" for detection. Do isotonic or Platt instead. It would have been the obvious first thing to try. |
| ⚠ **Any move ranked against the "graph costs 45 %" figure** | §0. Retracted by `measured-bars.md` §17.3. A move justified by recovering that 45 % is justified by a number that does not exist. |

### 6.3 The pattern under all of them

Every wasted technique in both tables is one of three shapes, and naming the shapes is cheaper
than re-testing each new arrival:

1. **An always-on multiplier bought to help the rare hard frame** — enhancement, TTA, SAHI,
   higher input resolution. It taxes 100 % of frames for a benefit concentrated in a few percent.
   The conditional form of the same idea (§3.3, §4.4) is the one that survives.
2. **A benchmark number from hardware we do not have, or a mode we cannot run** — `hw_only`
   FPS, Cutie's GPU 45.5, EdgeTAM's phone-NPU 16, NeuFlow on an Orin. The Hailo-8 at batch 1 is
   **PCIe-round-trip-bound at ~9.3 ms** and that dominates almost everything else.
3. **A method whose published gain is measured against a baseline weaker than ours** — "fuse
   consecutive frames beats one detection" when the shipped tracker already reconstructs through gaps;
   "predict the next frame" when the Kalman already does. ⭐ **Always demand the delta against
   the incumbent component, not against the paper's ablation.**


## 7. Q7 — geometry and semantics beyond boxes, for localization

⚠ **First finding: most of this is already built, and the brief did not know.** Before proposing
anything, the tree:

| already in `src/mongla_localization/mongla_localization/` | what it does |
|---|---|
| `floor_plane.py` | range to a point on a **known horizontal plane** from one pixel, no object size — "a ray through the pixel where the object meets that plane hits it at exactly one point". Derives its own sigma, and documents where it is *worse* than `range_to` (1° of unmodelled pitch = **17 cm** at h=1 m, R=3 m). |
| `pool_lines.py` | **heading off a lane line**, using World Aquatics' certified constants (lane centres exactly **2.5 m**, marking width 0.2–0.3 m). Needs no on-deck survey. |
| `tile_grating.py` | the tiled floor as a **2-D optical grating**: one FFT gives absolute heading mod 90°, with no prop and no detection. Built on the inversion that the texture which *defeats* LK is the texture a phase method is *strongest* on. |
| `retro.py`, `inekf.py` | retrodiction and the right-invariant EKF these feed |

Plus launch switches `lane_lines`, `tile_m`, `caustics` (`CLAUDE.md` §6). **The "cheap geometry"
half of Q7 is done.** What follows is only what is *not* there.

### 7.1 Vanishing points for heading — **redundant, skip**

A vanishing point from pool lane lines gives heading. `pool_lines.py` already gives heading from
the same pixels with better-characterised error, and `tile_grating.py` gives an *absolute* heading
mod 90° from a different physical mechanism. ⛔ **A third estimator of the same quantity from the
same pixels is not redundancy** — it is correlated error wearing a second hat. It would fail the
same way at the same time. Skip.

### 7.2 Horizon / floor-normal — **no horizon underwater; the normal is already had**

There is no horizon in a pool. The floor normal is what `floor_plane.py` assumes, and it comes
free and better from the board's IMU at 500 Hz. ⛔ Nothing to buy.

### 7.3 ⭐ Segmentation masks for plane fitting — the one real candidate, and it is cheap

This is the only item in Q7 with an unexploited mechanism, and unusually it is **already on our
chip**:

- `measured-bars.md` (Hailo segmentation decode, 2026-09-11, vehicle, `yolov8n_seg` 640×640):
  chip inference **6.28 ms**, host decode with masks **2.98 ms at 6 detections**, **end to end
  11.74 ms = 85.2 Hz**. Detection path with seg resident: **95.1 Hz vs 95.3 Hz seg-absent — no
  regression.**
- ⚠ Mask cost is **linear in detection count**: 1 det = 10.2 ms, 6 det = 12.5 ms, ≈**0.45 ms per
  mask**. `max_det` defaults to 100. Never quote 2.98 ms without the count.

**What a mask buys that a box cannot.** `floor_plane.py`'s whole premise is *the pixel where the
object meets the plane*. From a box that pixel is the **bottom edge midpoint** — an estimate that
is wrong whenever the object is tilted, partly occluded, or not axis-aligned, and the module's own
sensitivity is **17 cm per degree** at 3 m. A mask gives the **actual contact contour**, so the
contact pixel is measured rather than assumed. That is a direct, quantifiable improvement to a
module that already exists and already publishes a sigma.

The same mask also gives an **orientation** for a planar prop (a bin, a gate face) via the
second moment of the mask — which boxes structurally cannot represent, since an axis-aligned box
is orientation-blind.

⚠ **The honest caveat, from the repo, stated before the proposal:** `../vision.md` §6 records
that **no controlled study of masks vs boxes for alignment error exists in the field**, and §2.5
records segmentation as "cheap on our chip — and unproven for our purpose". This sweep did not
find one either. It also does not need one, because the falsifier below is cheap and the truth is
knowable *here*.

| | |
|---|---|
| (a) edge-feasible | **yes, measured on this vehicle: 85.2 Hz end to end with masks, no regression on the detection path.** |
| (b) buys | a measured contact pixel instead of an assumed one, feeding a module whose error is **17 cm/degree**; plus prop orientation, which boxes cannot express |
| (c) cost | effort **3**: train a seg head for our classes, and a `floor_plane` variant that takes a contour. Both the chip path and the decode already exist and are tested (`detection/seg_decode.py`, `test/test_seg_decode.py`). |
| (d) underwater evidence | **for the mechanism, none** (no published masks-vs-boxes study). **For segmentation being possible underwater, yes**: USIS10K/USIS-SAM, ICML 2024, arXiv:2406.06039 — 10 632 images, 7 categories — but note it needed an **underwater-specific encoder and prompter** to work, so expect to train on our own data, not to import. |

**Falsifier — and this one is free, offline, on footage we already have.** Take frames where the
prop's contact line is visible and surveyable. Compute the range from (i) the box bottom-edge
midpoint and (ii) the mask contact contour, against tape truth. If the mask's range error is not
materially below the box's, ⛔ close it: the bottom edge was good enough and segmentation costs
0.45 ms/detection for nothing. ⭐ **This is the masks-vs-boxes study the field does not have**, and
`../vision.md` §6 already lists it as owed. Running it pays a research debt and decides a move at
the same time.

### 7.4 What Q7 does *not* buy

⛔ **None of this helps "never lose detection".** Geometry feeds localization; it does not keep
the lock. Do not let it be ranked as a continuity move — it is a localization move that happens to
reuse the perception chip. The two goals compete for the same ~40 % of spare chip time as §4.4's
crop detector, and only one can have it.


## 8. The other half of the goal — "frames as fast as possible"

⭐ **This section outranks every perception move above, and it costs no new model, no new library
and no new dependency.**

### 8.1 State the chain honestly, end to end

| stage | rate | source |
|---|---|---|
| chip, batch 1, end to end | **98.01 Hz** (10.20 ms) | `measured-bars.md` §14.3 |
| chip through a correctly-written ROS node + real camera, on a real topic | **94.11 Hz** (p95 10.75 ms) | §17.3 — ⚠ **that camera was the Sonix** (§17's header), i.e. the 210 Hz part, **not** the forward Fantech |
| **forward camera (Fantech), distinct stamps, shipped `fps: 60`** | **30.18 Hz** | `config.py` `'pi_forward'` |
| forward path end to end, image → detections | **30.03 → 27.11 Hz**, CPU **89.3 % idle** | `config.py` |
| downward camera (Sonix) via `V4L2MailboxCamera` | **211.0 Hz** @640×360, **120.4 Hz** @1280×720 | §17.1 |
| **the consumer**, `VISION_LOOP_HZ_SROT` | **50** (soaked 49.86 Hz, 0.00 % late) | `measured-bars.md` L60 |
| the recorded 53.9 Hz | ⚠ **stale, "needs re-measuring"** | §17.3 |

⚠ **That column is not apples to apples, and the difference is the whole point.** §17's header
says the 94.11 Hz run was on the **Sonix**; `config.py`'s 30.18 Hz is the **Fantech**, measured
2026-09-09 on a different day with a different node. **So 94.11 Hz is the rate the pipeline
achieves when the camera can feed it, and 27–30 Hz is the rate it achieves on the forward mount.**
Two cameras, two rates, one chip that is fast enough for both.

⭐ **Read down that column anyway.** The detector is not the bottleneck on the forward path and
neither is ROS: **the forward camera is**, at 30 Hz, with the chip idle roughly two thirds of the
time and the CPU 89 % idle. The vehicle's own numbers say the forward frame budget was spent
before any software in this dossier gets a turn. ⚠ Both camera figures predate the current code;
**N-0 must re-measure them side by side, on one day, on one build**, or this table is two
archaeological layers wearing one heading.

⛔ **And note where the consumer sits.** The control loop ticks at **50 Hz**. Detections arrive at
**27**. So the vision loop is currently **acting on a stale detection about half its ticks** — not
because the pipeline is slow, but because the camera cannot feed it. Above ~50 Hz delivered,
further throughput buys **latency and freshness**, not rate; below it, every frame is rate.

**Step zero for this entire dossier, in order:**
1. **Re-measure the delivered detection rate on current code, by distinct capture stamp.**
   ⚠ `config.py`'s own retraction is the reason: "a topic `hz` cannot tell a real frame from a
   republished one, which is how a self-imposed cap reads as a hardware limit." That exact mistake
   has been made twice in this repo already.
2. **Ask why the forward camera is a 30 Hz part when the downward one is a 210 Hz part.** That is
   the largest single number in this dossier and it is a **camera swap**, not an algorithm.

### 8.2 What the repo already knows about where frames go

`src/mongla_vision/test/test_composed_vision.py` names the per-frame cost of the camera→detector
topic hop, measured:

```
cv_bridge encode   0.845 ms
serialise          1.469 ms
transport          1.760 ms
imgmsg_to_cv2      a second full-frame copy
```

≈**4.07 ms plus a full-frame copy** — and, in the file's own words, "worth more than all of it,
**THE PUBLISHER'S CLOCK**": on the topic path the detector acts on whichever frame the publish
throttle handed over, whereas composed, the camera decodes when inference goes idle so the picture
is the **newest one that exists**. That is a latency and a *freshness* win, and freshness is what
a control loop actually consumes.

⛔ **And the repo already knows the trap:** `ComposableNodeContainer` **buys none of this** —
"rclpy has no intra-process comms, so composed Python nodes still traverse rmw. The gain is the
direct Python reference." Anyone reading the ROS 2 composition literature will reach for
`ComposableNodeContainer` first; on a Python stack it is a no-op.

**This is why `detector_dual_node.py` exists**, and it is already the shipped path: both cameras
and both detectors in one process, because the chip allows **one VDevice per process** — the
alternatives were measured and each fails: two processes →
`HAILO_OUT_OF_PHYSICAL_DEVICES (74)`; `multi_process_service` + scheduler → `InvalidOperation`
(the `hailort_service` daemon is not installed); one process with a ROUND_ROBIN scheduler and two
graphs → **SIGSEGV**. One process, one VDevice, both graphs taking turns → **98.2 Hz**, and
**37.5 Hz per pair** when both models alternate every frame.

⛔ **So the camera→detector hop is already closed**, which is a further reason to suspect the
53.9 Hz figure predates the current code. ⚠ **And 37.5 Hz per pair is the number that matters
when both cameras are live** — any move in §1–§7 that wants chip time is competing against the
*second camera*, not against idle silicon.

### 8.3 What the external literature adds, and its ceiling

*Impact of ROS 2 Node Composition in Robotic Systems*, arXiv:2305.09933 —
https://ar5iv.arxiv.org/html/2305.09933 **REAL-HARDWARE (Raspberry Pi 4B, Cortex-A72 1.5 GHz —
one generation below ours)**:

| configuration | VGA 3-channel goodput |
|---|---|
| multi-process | **cannot sustain 20 Hz uncompressed** |
| composition | **85 Hz** |
| composition + IPC | **415 Hz** |

Also: composition halves both CPU and latency at every message size; with IPC, **constant 40 µs
latency independent of message size**; loaned messages crash above 2 MB.

⚠ **The 415 Hz is C++.** The IPC column is exactly the row our Python stack cannot have, which is
the point `test_composed_vision.py` already makes. The transferable finding is the **85 Hz
composition column vs multi-process failing at 20 Hz on a Pi 4** — a >4× gap from process layout
alone, on weaker hardware than ours. It corroborates the direction the stack already took and it
sets the scale of what is still on the table downstream of the detector
(detector → tracker → lock are three more hops).

| | |
|---|---|
| (a) edge-feasible | **already done for the camera→detector hop.** The detector→tracker→lock hops are unexamined. |
| (b) buys | on the forward path, **nothing until the camera changes** — the chip is already idle. Downstream hops buy **freshness** for a 50 Hz consumer fed at 27 Hz. |
| (c) cost | effort **1** to measure, effort **2–3** to act |
| (d) evidence | our own `test_composed_vision.py`, `config.py` `'pi_forward'`, `measured-bars.md` §17.1/§17.3; arXiv:2305.09933 on a Pi 4 |

**Falsifier:** count **distinct capture stamps** on detections through the current
`detector_dual_node` bringup — never `ros2 topic hz`, per `measured-bars.md` L1995 and
`config.py`'s own retraction. If it reports ~27–30 Hz, §8.2's hop-flattening buys latency only and
the camera is the whole story. If it reports ~54 Hz with one camera, something in the node is
still costing frames and `measured-bars.md` L59 stands.

### 8.4 ⚠ The fallback that would halve it silently

Two paths in this chain degrade **without an error**, and either would swamp every gain discussed
in this dossier:

- `cameras/webcam.py:87` sets `CAP_PROP_BUFFERSIZE = 1` "to avoid stale frames" — measured,
  that is a **1.8–2.0× throughput loss** (640×400: 117.2 vs **209.9 Hz**; 1280×720: 59.8 vs
  **119.8 Hz**). And `factory.py:44-53` **falls back to the OpenCV `webcam` source whenever the
  configured device is not a `/dev/...` path** — deliberately, so a dev box "gets OpenCV instead
  of a stack trace". ⚠ It logs a warning, so it is not silent; but **the warning is the only
  signal**, nothing downstream checks the achieved rate, and the vehicle profiles reach the
  mailbox only because they carry `device_path: /dev/mongla_cam_*` symlinks — **the same udev
  symlinks that were once bound to the wrong cameras** (`config.py`, the round-38 trap).
  (`measured-bars.md` §17.2.)
- `tracker_node.py:96-98` **falls back from OC-SORT to legacy ByteTrack** when the Roboflow
  library fails to import — a different tracking algorithm, chosen by an import error.

⭐ **Both are the failure this repo keeps naming: a plausible result standing in for an absent
measurement.** A gate that asserts the achieved frame rate against the profile's request, and one
that asserts the tracker actually in use, are each an afternoon and would protect every number
above.


## 9. Candidate moves, each with its falsifier

Scored by `../README.md`'s rubric — **value ÷ (risk × effort)**, all 1–5. Ordered by what should
happen first, which is not the same as by score: **the first two are measurements, and they
change the scores of everything below them.**

| # | move | value | risk | effort | falsifier |
|---|---|---|---|---|---|
| **N-0** | ⭐ **Re-measure the delivered detection rate** on current code, **by distinct capture stamp, both cameras, one day, one build** (§8.1) | 5 | 1 | **1** | it comes back ~94 Hz on the forward path → the 53.9 figure *and* the 30.18 Fantech figure were stale, and `measured-bars.md` L59 is corrected. Either outcome is a win. |
| **N-1** | ⭐ **Reliability diagram on a held-out venue**, raw detector confidence (§5.3) | 5 | 1 | **1** | the detector is already well-calibrated across venues → N-3 closed for an afternoon's work |
| **N-2** | ⭐ **Price a 120 Hz global-shutter forward camera** against the Fantech's measured 30.18 Hz (§8.1). The Sonix on the downward mount already does 120.4 Hz at 720p. | **5** | 2 | 2 | N-0 shows the forward path is not camera-limited after all |
| **N-3** | **Isotonic (not temperature) post-hoc calibration**, per camera per class (§5) | 4 | 2 | 2 | N-1's diagram shows no gain after fitting, or D-ECE improves while AP falls |
| **N-4** | **Masks-vs-boxes contact-pixel study** for `floor_plane.py`, offline on footage we have (§7.3) | 4 | 1 | 2 | mask contact range error ≈ box bottom-edge error against tape → segmentation closed for this purpose. **A field-level research debt is paid either way.** |
| **N-5** | ⭐ **Two silent-degradation gates**: assert achieved frame rate against the profile's request, and assert the tracker actually in use (§8.4) | 4 | 1 | **1** | neither fallback can fire in practice — but `measured-bars.md` §17.2 already measured one firing |
| **N-6** | **Flatten the detector→tracker→lock hops** for freshness (§8.2) — the consumer ticks at 50 Hz and is fed at 27 | 3 | 2 | 3 | the hops measure sub-millisecond and the staleness is all upstream |
| **N-7** | **Relabel / audit small-prop data** before any small-object architecture change (§4.6) | 3 | 1 | 3 | our small-prop labels audit clean → the 4×-correction-rate finding does not apply to us |
| **N-8** | **Track-gated confidence threshold** in `confidence.py` (§3.3) | 3 | **4** | 2 | replay the **71 real gaps**: if *any* gap closes onto the **wrong object** at a setting that helps, refuted. ⚠ highest-risk item here — it is an agreement mechanism, which `../README.md` rule 3 exists to catch. |
| **N-9** | **Feed the board's 50 Hz gyro into `kalman.py`'s predict step** — the filter is constant-velocity in image space with no vehicle-motion model (§1.2) | 3 | 2 | **1** | measure p95 box displacement between consecutive delivered frames during a hard yaw; if the constant-velocity term already absorbs it, closed |
| ⛔ | ~~conditional native-resolution crop~~ | — | — | — | **dead as configured** (§4.4): the forward camera is 640×360, there is nothing to crop. Revives only with N-2. |

⚠ **Sequencing matters more than the scores.** N-0, N-1 and N-5 are each about a day, and **N-0
and N-1 can each close a lower row outright**. N-2 is a hardware ask with a measured price and it
gates the only "more pixels" idea that was ever real. N-7 answers the same question as any
small-object architecture work and is far cheaper — do it first.

**What would have changed this answer.** Stated so the sweep is falsifiable as a whole:
a SAM2-family model in the Hailo-8 zoo with a published batch-1 figure; a published underwater
D-ECE showing detectors are *already* calibrated across venues; a masks-vs-boxes study showing
boxes suffice for plane contact; or an N-0 measurement showing the forward path already delivers
~94 Hz. **The first three do not exist and the fourth is a day's work.** ⭐ Three of the four
would *close* moves rather than open them — which is the point of running the measurement first.

**The shape of this dive's answer, in one paragraph.** The 2024–2026 literature does **not** offer
a better continuity ladder than detector → OC-SORT → LK → XFeat on this hardware; the
memory-tracker family that would is structurally un-compilable to a dataflow accelerator, and its
only underwater evaluation warns about the exact mode we would have to use. **The binding
constraints are not perception methods at all.** They are a **30 Hz forward camera in front of a
98 Hz detector and a 50 Hz consumer** (§8.1), a **confidence number that means something different
in every venue** (§5, ≈33 mAP50 of colour-domain swing), **two fallbacks that degrade silently**
(§8.4), and a **contact pixel `floor_plane.py` assumes when a mask could measure it** (§7.3).
Everything else here is an always-on multiplier bought to rescue a rare frame — and this vehicle
has measured, more than once, exactly what those cost.


## 10. Claims I could NOT verify

Per `../README.md` rule 1, these were seen but the number could not be pinned to a fetched
primary source. **None of them is used anywhere above.**

| claim | why it is unverified |
|---|---|
| **EdgeTAM or EfficientTAM rate on a Raspberry Pi 5 (CPU)** | **no published figure exists.** §2.2's "low single-digit Hz" is my inference from a 16 FPS phone-NPU number, not a measurement. It is the single largest hole in §2 and it is cheap to close: clone `facebookresearch/EdgeTAM` and time it. |
| **SAM2's underwater mIoU / J&F numbers** | arXiv:2408.02924's abstract carries the *direction* ("significantly degraded" in automatic mode) but **no scores**. The qualitative claim is quoted; no number is used. |
| **`Underwater Object Detection Under Domain Shift`, IEEE 10679365** | paywalled, not fetched. Listed by title in §5.2 and contributes nothing. |
| **Whether a Hailo-8 can host *any* stateful cross-frame model** | argued structurally in §2.1 from the dataflow compilation model plus one failed community compile. **No Hailo statement was found either way** — the thread has no staff reply. Treat §2.1's "will stay that way" as reasoned, not sourced. |
| **The ROI-Gated SAHI numbers** (arXiv:2608.23923) | only the abstract framing was fetched; its efficiency gains are cited as a *principle* (gate by a proposer) and **no number of its is used**. |
| **Cutie's 45.5 FPS on what GPU** | the figure appears without the device in the sources fetched. Unusable as a rate; used only to rank Cutie above XMem. |
| **Ultralytics TTA's 2–3× on what hardware** | vendor documentation, GPU-implied, no device stated. Used only for the *ratio*, which is architectural (N forward passes), not for an absolute rate. |
| **A published underwater D-ECE, anywhere** | **does not exist as far as this sweep could find.** That is why §5.3's held-out-venue reliability diagram would be novel as well as useful. |
| **Held-out-session or held-out-venue recall, anywhere in the detection literature** | still absent — `../vision.md` §6 recorded this and this sweep independently failed to find a counterexample. |
| **OC-SORT's measured advantage over ByteTrack, in HOTA/MOTA** | ⭐ the paper **was** fetched (§3.1) and its mechanism and 700+ FPS CPU figure are now sourced — but **its abstract gives no head-to-head numbers against ByteTrack**, only "state-of-the-art on multiple datasets". So *which* tracker is better **for us** remains unmeasured, and the fallback at `tracker_node.py:96-98` silently swaps between them. |
| **Cutie's 45.5 FPS and the GPU it was measured on** | the abstract (arXiv:2310.12982) **was** fetched and gives the J&F deltas (+8.7 over XMem, +4.2 over DeAOT at 3× its speed) but **no FPS and no device**. The 45.5/22.6/11.7 figures remain snippet-only. Used to rank, never as a rate. |
| **USIS-SAM's accuracy numbers** | arXiv:2406.06039 was fetched; the abstract claims "superior performance" with **no figures**. Only the dataset size (10 632 images, 7 categories) and the architectural fact that it needed an underwater-specific encoder + prompter are used. |
| **Whether our forward camera's in-water FOV is 46.7°** | the 46.7° is measured, but `measured-bars.md` L2035 puts the forward lens at **73.9° in air** and L2235 records the **downward** in-water FOV as unknown, with L1108 documenting a live intrinsics mix-up. §0's pixel table is an order-of-magnitude instrument; **a per-camera in-water FOV is genuinely owed.** |


## 11. Rejected, with the reason

Recorded so they are never re-opened (`../README.md` rule 5). The reasons are in full above; this
is the index.

| rejected | one-line reason | §|
|---|---|---|
| **Track-before-detect** | a radar/IR point-target method; our target is **233 px at 5 m** on a moving mount. Wrong premise. | §1.1 |
| **Streaming detectors** (StreamYOLO, LongShortNet, DAMO-StreamNet) | their whole gain is one frame of prediction at 30 FPS; our frame is **18.6 ms ≈ 1.4 px** of motion and `kalman.py` already applies it. No Hailo entry, no underwater paper. | §1.2 |
| **SAM2 / EdgeTAM / EfficientTAM / Cutie / XMem as a lock rung** | zero SAM entries in the Hailo-8 zoo; the one FastSAM-S compile **failed at the allocator** (19 Aug 2024, unanswered); memory attention is stateful and does not map to dataflow; CPU fallback ≪ 30 Hz; and the only underwater SAM2 evaluation found **automatic prompting degrades**, which is the only mode we could use. | §2 |
| **Test-time augmentation** | **2–3× inference for +1.2 mAP**. Takes 98 → 33–49 Hz, at or below what the pipeline already delivers. | §3.2 |
| **SAHI / tiled inference** | divides our rate by N to solve an aerial-imagery problem we do not have; its own successor paper says tiling is inefficient on sparse background scenes, which is every frame we take. | §4.2 |
| **Higher detector input resolution (832/960)** | the **forward camera is 640×360** — a larger model input upsamples and invents nothing, at a new compile, a new ≥1 024-image calibration set and a ~quadratic rate cost. | §4.3 |
| **Temperature scaling as the calibration method** | ECCV 2024 names **TS specifically** as producing incorrect conclusions in detection. Use isotonic or Platt. | §5.1 |
| **Vanishing points for heading** | `pool_lines.py` and `tile_grating.py` already give heading from the same pixels by two different mechanisms. A third correlated estimator is not redundancy. | §7.1 |
| **Horizon / floor-normal estimation** | there is no horizon in a pool, and the normal comes free and better from the board IMU at 500 Hz. | §7.2 |
| **`ComposableNodeContainer` for the Python vision nodes** | **rclpy has no intra-process comms** — composed Python nodes still traverse rmw. It is a no-op here; the shipped answer is the direct Python reference in `detector_dual_node.py`. | §8.2 |
| ⚠ **Any move justified by "the ROS graph costs 45 %"** | **retracted** by `measured-bars.md` §17.3: a correctly-written node measured **94.11 Hz** against 97.47 standalone — **3.4 %**. The loss is in our node, and the 53.9 Hz figure itself needs re-measuring. | §0, §8.1 |
| **Anchor tuning for small objects** | **YOLOv8/v11 are anchor-free.** There is no such knob; the advice is YOLOv5-era. | §4.5 |
| **Conditional native-resolution crop**, *as the vehicle is configured* | the forward camera is **640×360**; there is no native resolution to crop from. Parked, not closed — it revives if the forward camera becomes a 720p part. | §4.4 |

### Corrections this sweep made to its own drafts, kept per rule 5

| I wrote | the truth | how it was caught |
|---|---|---|
| "the camera delivers 1280×720" | that is the **`laptop`** profile in `cameras.yaml` — a file whose own header says **NOT LOADED AT RUNTIME** (B49). The loaded forward profile is **640×360** in `config.py`. | reading `config.py` instead of the YAML |
| "the graph costs 45 %" | **3.4 %** — `measured-bars.md` §17.3 retracted §14.3 the same day | reading further down the same file |
| "we run ByteTrack" (from the brief) | the default is **OC-SORT**; ByteTrack is the `legacy_` path and the **import-failure fallback** | `tracker_node.py:170, 96-98` |
| the 145.6/33.1 ms table is detector input resolution | it is **XFeat on the Pi CPU**, and it shows the Pi **72.8 % idle** — an argument *for* host-side methods | reading the table's own caption |

⭐ **All four were errors of quoting a number without its subject.** That is the same defect
`CLAUDE.md` §9 names as the recurring one in this codebase, and this dossier reproduced it four
times in one afternoon. **Ask what a number is a number *of* before ranking anything by it.**

---

## Sources actually fetched

- SAHI — https://arxiv.org/abs/2202.06934 (14 Feb 2022 / Oct 2022)
- StreamYOLO — https://arxiv.org/abs/2207.10433 (21 Jul 2022)
- LongShortNet — https://arxiv.org/pdf/2210.15518 (ICASSP 2023)
- DAMO-StreamNet — https://www.ijcai.org/proceedings/2023/0090.pdf (IJCAI 2023)
- ROS 2 node composition on a Pi 4 — https://ar5iv.arxiv.org/html/2305.09933
- On Calibration of Object Detectors (ECCV 2024 Oral) — https://arxiv.org/abs/2405.20459 (30 May 2024) · code https://github.com/fiveai/detection_calibration
- USIS10K / USIS-SAM (ICML 2024) — https://arxiv.org/abs/2406.06039 · https://github.com/LiamLian0727/USIS10K
- SAM 2 — https://arxiv.org/html/2408.00714v1
- SAM2 underwater evaluation — https://arxiv.org/abs/2408.02924 (6 Aug 2024)
- EdgeTAM (CVPR 2025) — https://arxiv.org/abs/2501.07256 (13 Jan 2025) · https://github.com/facebookresearch/EdgeTAM
- Cutie (CVPR 2024 Highlight) — https://arxiv.org/abs/2310.12982 (19 Oct 2023, rev. Apr 2024) · https://github.com/hkchengrex/Cutie
- OC-SORT (CVPR 2023) — https://arxiv.org/abs/2203.14360 (27 Mar 2022 / 16 Mar 2023) — **the shipped tracker**
- Infrared dim small target review — https://pmc.ncbi.nlm.nih.gov/articles/PMC11207645/ (2024)
- IR small object segmentation directions — https://arxiv.org/html/2502.14168v1 (Feb 2025)
- Why Domain Matters (underwater domain + annotation quality) — https://arxiv.org/html/2607.10575 (12 Jul 2026)
- ROI-Gated SAHI — https://arxiv.org/abs/2608.23923
- Hailo Model Zoo, HAILO8 instance segmentation — https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8/HAILO8_instance_segmentation.rst
- Hailo community, SAM 2 on Hailo-8L — https://community.hailo.ai/t/sam-2-in-hailo8l/2614/4
- Ultralytics TTA — https://docs.ultralytics.com/yolov5/tutorials/test-time-augmentation

**In-tree evidence used:** `.claude/context/measured-bars.md` (§14.2, §14.3, §17.3, Hailo
segmentation decode, L59, L61, L1995) · `.claude/context/sota/vision.md` (§2.1, §2.4, §2.5, §5,
§6, V-1, V-4, V-5, V-6) · `src/mongla_vision/test/test_composed_vision.py` ·
`src/mongla_vision/mongla_vision/detector_dual_node.py` ·
`src/mongla_vision/mongla_vision/tracking/{bytetrack,kalman,confidence,lock_state}.py` ·
`src/mongla_localization/mongla_localization/{floor_plane,pool_lines,tile_grating}.py` ·
`src/mongla_vision/config/cameras.yaml`.
