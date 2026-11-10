# The Hailo-8 frontier — what is left in the chip, and what we owe the people who use it next

> Written 2026-11-10 after 24 searches across Hailo's forum, its repositories, the Model Zoo,
> Ultralytics, arXiv and the trade press, and a measurement of `gate_sharks.hef` on Kutum the
> same day. It does not repeat [the measured pipeline](hailo-vision.md); it starts where that
> document stops. Every lever below has its scorer and its falsifier written **before** the run,
> because the two Hailo numbers this project trusted without one (round 24's 4.5x, round 29's
> 4.2x) were both wrong about our pipeline.

## Why a Hailo, and not the obvious thing

Most robots that see run on NVIDIA. That is a reasonable default and we are not against it. We
are against defaults nobody can explain.

Mongla is ours from the firmware to the thruster: the board (**Hengla**), the wire, the
companion (**Kutum**), the models, the missions. The vision accelerator is the one part we did
not design, so we picked the one we could understand end to end: a fixed-function dataflow chip
that does nothing but multiply the numbers a network is made of, at about **2.5 W typical**
(Hailo's figure for the module), behind a runtime and a kernel driver whose source we can read
and build. Kutum builds both from source (`kutum/setup.sh`), because a vendor apt upgrade
removed the accelerator once ([hailo-vision.md](hailo-vision.md), 2026-09-05) and we would rather
own the build than trust the mirror.

The honest edges of that claim, stated so nobody has to find them for us:

| layer | licence | can we read it? |
|---|---|---|
| `hailo_pci` kernel driver | GPL-2.0 | yes, and we build it |
| libhailort, pyhailort, `hailortcli` | MIT | yes, and we build it |
| chip firmware | Hailo EULA | no |
| Dataflow Compiler (DFC) | proprietary, registration required | no |

So the claim is not "open all the way down". It is: **every line that decides what the vehicle
does is ours, and every line between our code and the silicon that we can read, we do.** A
black box is allowed in the chain only where it is a compiler or a firmware blob with a fixed,
measured contract, and that contract is written down ([hailo-vision.md](hailo-vision.md) §1,
§11, §12).

We found no other open-source AUV stack running on a Hailo-8, and no official Hailo ROS 2
package (Hailo's own answer on the forum points at examples, not a package). That makes the
measurements in this folder some of the only public ones taken on a moving robot instead of a
desk, which is the reason for the last section.

## What is settled, so nobody spends a day on it

| question | answer | evidence |
|---|---|---|
| Can the clock go above 400 MHz? | No. 400 MHz is the nominal clock for 26 TOPS; the firmware's fixed caps are 100, 200 and 400 | Hailo forum (fixed clock option); `requested_temperature_clock_freq` reads 400000000 on Kutum |
| Can the chip report its own power? | No, not on the AI HAT+: it has no current monitor, so `measure-power` returns an unsupported opcode | Hailo forum, three threads; measure at the 5 V rail instead |
| Is PCIe a lever? | No. Gen 3 x1 is the Pi 5 connector's limit | [hailo-vision.md](hailo-vision.md) §13 |
| Does async help our models? | No: 1.00x on a chip-bound three-context graph | [hailo-vision.md](hailo-vision.md) §5 |
| Does it throttle, and when? | Yes. Orange zone at 104 °C, red at 120 °C; each step from 104 / 108 / 112 / 116 °C drops the clock 400 → 350 → 300 → 250 MHz | read off the chip on Kutum; now a row in the manager's health report (`kutum.hailo`) |
| Is Hailo-10H the upgrade? | Not for vision. It matches Hailo-8 on detection; its edge is language models, which we do not run | Hailo forum benchmarks, 10H vs 8 |

## What changed in 2026, and what it means for us

**The Hailo-8 software track is now the older one.** Hailo split its stack: DFC 5.x and
HailoRT 5.x, with the Model Zoo `master` branch, serve Hailo-10 and Hailo-15 only. Hailo-8 stays
on **DFC 3.x / HailoRT 4.x / Model Zoo v2.x** (v2.18 pairs with DFC 3.33.1; Kutum runs
HailoRT 4.24.0). The 2026-01 Suite release shipped for 10/15 first. Fixes still land on the 4.x
line, but new features land on 5.x first or only.

**Microchip is buying Hailo.** Microchip announced a definitive agreement in July 2026; trade
press reported the deal closed on 2026-09-21 (we have not seen Microchip confirm the close).
Both sides say existing products stay supported. No lifecycle date for Hailo-8 has been
published.

**What we do about both, now, while it costs nothing:**

1. **The build is the insurance.** `kutum/setup.sh` builds driver and runtime from pinned source
   tags. The source tarballs for those tags belong in `kutum/backup.sh`'s pull, so a deleted
   GitHub release cannot strand the vehicle. *Bead filed.*
2. **The compiler is the exposure.** The DFC is the one tool we cannot rebuild. The
   [compile harness](../../../tools/bench/hailo_compile.sh) pins DFC 3.34 for HailoRT 4.24; the
   wheel itself must be kept beside the backups, not re-downloaded on the day. The 5.x wheel
   produces a HEF the Pi refuses as an architecture mismatch (the harness's TRAP 2).
3. **Nothing in `src/` may depend on a 5.x-only API.** The 4.x Python API is what we have
   measured; a call that exists only in 5.x would fail on the vehicle and pass on a dev box with
   newer wheels.

## The levers left, ranked, each with its scorer written first

The chip is not on the critical path today: 98 Hz is above the camera (about 79 Hz) and the
50 Hz control loop ([hailo-vision.md](hailo-vision.md) §7). So the ranking is by **latency and
headroom for a second model**, not by frames per second.

### 1. A single-context compile of the model we already train

**Why it is first.** [hailo-vision.md](hailo-vision.md) §3a concluded a single-context YOLO11
was "very unlikely to be available", because Hailo's own `.alls` splits YOLO11n into three. That
conclusion rested on Hailo's default allocator. The DFC has explicit knobs we have never tried:

```
context_switch_param(mode=disabled)                    # forbid a split; fail if it does not fit
resources_param(max_compute_utilization=…,
                max_control_utilization=…)             # let the allocator pack harder
quantization_param({conv*}, precision_mode=a8_w4)      # 4-bit weights on the largest layers
allocator_param(timeout=…)                             # give the solver longer
```

Forum threads report forcing three contexts down to two for a YOLOv8m-pose and getting the FPS
back. Nobody has published it for YOLO11n at 640 with three classes. The prize, if it fits, is
the yolov8n row of §3: about **4x hardware-only, a 2.4 ms graph instead of 10.2 ms**, and async
then becomes worth its 1.6x (§5).

**Scorer, before the run.** Compile `gate_sharks` from its ONNX with each knob alone, then
combined. A result counts only if **all three** hold:

- `hailortcli parse-hef` reports `Single Context`;
- `hailortcli benchmark` hardware-only FPS is at least **2x** today's 95.6;
- on the **labelled** held-out validation split the model was trained with, run on the chip,
  mAP50 is within **1 point** of the three-context HEF and no class loses more than 2. Labels,
  not the old HEF's detections: comparing against the incumbent measures agreement, not truth.
  `tools/bench/hailo_int8_shift.py` then says whether any loss is the quantiser's.

**What would falsify it.** The allocator refuses (`agent infeasible`, `not enough resize units`)
in every configuration, or the single-context HEF passes FPS and fails recall: then 4-bit
weights cost more accuracy than the chip has to give, and §3a was right for a different reason.
Either outcome is written into §3a, not silently.

### 2. `ultra_performance`, measured twice and still unused

`--power-mode ultra_performance` measured **+2.2 %** on the retired model in September and
**+2.1 %** on `gate_sharks` on 2026-11-10 (95.6 → 97.6 FPS hardware-only). It is used nowhere
in `src/` or `tools/`. It was left out for a reason nobody wrote down; the only one that holds up
is heat. Kutum sits in a sealed can, and ultra performance trades power for rate.

**Scorer.** Run the detector at its camera-bound rate for 20 minutes in each mode, reading
`kutum.hailo` from the health report every 10 s. Adopt it only if the chip stays in the green
zone (below 104 °C) **and** the Pi stays below its 80 °C soft throttle in the closed hull, with
the published detection rate up by at least 1 %.

**Falsifier.** The rate does not move (the stack is camera-bound, so +2 % on the chip is
invisible end to end), in which case it is not worth the heat and stays off, with that written
here.

### 3. YOLO26, a different family with no NMS on the host

Hailo's Model Zoo now ships YOLO26 n/s/m for Hailo-8, and Ultralytics documents an export path
validated on Hailo-8L with HailoRT 4.23 / DFC 3.33. YOLO26 drops NMS: the decode is a top-k on
the host. Two things are unknown, and both are cheap to learn:

- **How many contexts does the zoo's `yolo26n.hef` compile to on Hailo-8?** One `parse-hef`
  answers it. If it is single, this lever and lever 1 meet, and retraining on YOLO26 is the path.
- **Does the host decode cost more than our 0.02 ms baked NMS?** It cannot cost less.

**Scorer.** Same three conditions as lever 1, against the same footage, with the host decode
timed inside `infer()`, not beside it.

**Falsifier.** Three contexts again, or a recall loss: then the family that chose our context
count also chooses YOLO26's, and we stop.

A caution from the one independent case study we found (Dubinsky, YOLO26 on Hailo-8L): the
quantisation-aware fine-tune at optimisation level 3 bought little, and a DFC run without a GPU
silently drops to level 0. Read the compile log for the level actually used before believing
any accuracy number.

### 4. Two models on one chip, under the scheduler

The board can hold two configured network groups at once ([hailo-vision.md](hailo-vision.md)
§11), and the Model Scheduler time-slices separate HEFs. An NPU evaluation on arXiv
([2609.13166](https://arxiv.org/pdf/2609.13166)) measured concurrent jobs scaling throughput
about 6x on a **single-context** network, because the dataflow pipeline fills. On a
three-context network that headroom is mostly spent on reconfiguration, so this lever is worth
measuring only after lever 1 or 3 produces a single-context model.

**Scorer.** Forward detector plus downward detector, both through the scheduler: each keeps at
least 90 % of its solo rate and its p95 latency grows by less than 2 ms.

### 5. Things we looked at and are not doing

| idea | why not |
|---|---|
| overclocking | not possible; the caps are fixed |
| Hailo-10H | same vision throughput; its gains are language models |
| transformer backbones (ViT, DINOv2) | LayerNorm and transpose lowering is the forum's most common failure; Hailo removed ViT-CLIP from the zoo in v2.16 |
| stereo depth on the chip | about 10 FPS for StereoNet; relative-depth `scdepthv3` runs at 30 FPS but gives no scale |
| SuperPoint for features | the forum steers Hailo users to XFeat, which we already export (`tools/bench/xfeat_export_hailo.py`) |
| batching | 2x throughput for 38 ms of latency; wrong for control ([hailo-vision.md](hailo-vision.md) §3) |

## What we can give back

Five findings in this repository are, as far as 24 searches can tell, not published anywhere
else. Each is reproducible from a tool in `tools/bench/`. In the order they would save someone
the most time:

1. **The blocking `InferVStreams.infer()` holds the Python GIL for the whole inference**
   ([hailo-vision.md](hailo-vision.md) §10, `tools/bench/hailo_async_gil.py`). Any ROS 2 node
   that runs a camera callback beside it loses that time.
2. **`InferModel.configure()` costs chip SRAM and `activate()` does not** (§11). Swapping models
   by reconfiguring fills the chip with `CONTEXT_SWITCH_STATUS_SRAM_MEMORY_FULL` after a few dozen
   swaps. The forum has the symptom ("device still reported as busy") without the cause.
3. **Async is 1.00x on a chip-bound graph and 2.4x on a transfer-bound one** (§5). The first
   published batch-1 numbers we found.
4. **Segmentation decoded without leaving the quantised domain**
   ([hailo-vision.md](hailo-vision.md), *Segmentation*): the threshold moves to `u8 >= ceil(conf·255)`
   and the 160x160x32 prototype never becomes float.
5. **The chip's health, read while it runs.** Temperature zones, throttle level and the
   requested clock come from the device's health query while another process holds the chip,
   in 12 ms. The thresholds above came from it; we found no public page that lists them for
   Hailo-8.

Plus one thing no-one else has: **a ROS 2 vision stack on a Hailo-8 that has to be right on a
vehicle**, with its measurements beside its code.

**How it gets shared.** Each finding becomes one post on the Hailo community forum, written as
a reproduction (the tool, the command, the number, the HailoRT version), and linked back here.
Nothing is posted without the owner reading it first. Drafts live outside the repository until
then.

## Sources

Hailo's forum threads are cited by title; the site's search finds each one.

- Hailo community: *Hailo-8 fixed systems clock speed option*; *Hailo-8 AI HAT clock rate
  locked at 400 MHz*; *Measuring power consumption on Hailo8L with RP5 AI kit*; *Re-posting
  power measurement not supported error*; *How to force 2-context compilation for custom
  yolov8m-pose*; *How to reduce contexts and optimize model for 30 FPS*; *Model could not fit in
  single context*; *Switching between two ROS 2 nodes using Hailo on RPi5: device still reported
  as busy*; *Is there an official ROS2 package for running YOLO on Raspberry Pi 5 with AI HAT*;
  *2026-01 for Hailo-8/8L release*; *Deploying SuperPoint/LightGlue on Hailo-8*;
  *Hailo-10H on RPi5: DFC conversion failures with transformer-based models*;
  *Microchip Technology signs definitive agreement to acquire Hailo*.
- [Microchip press release](https://ir.microchip.com/news-events/press-releases/detail/1406/microchip-technology-signs-definitive-agreement-to-acquire-hailo)
  (agreement); closing as reported by
  [DatacenterDynamics](https://datacenterdynamics.com/en/news/microchip-technology-acquires-israeli-chip-startup-hailo-for-undisclosed-amount).
- [hailo-ai/hailort](https://github.com/hailo-ai/hailort),
  [hailo-ai/hailort-drivers](https://github.com/hailo-ai/hailort-drivers),
  [hailo-ai/hailo_model_zoo](https://github.com/hailo-ai/hailo_model_zoo) (v2.18 release notes).
- [Ultralytics: Hailo integration](https://docs.ultralytics.com/integrations/hailo).
- [NPU Hardware Evaluation v1.0, arXiv 2609.13166](https://arxiv.org/pdf/2609.13166).
- D. Dubinsky, [quantisation noise in YOLO26](https://danieldubinsky.github.io/personal-site/case-studies/quantization_noise_analysis_yolo26/)
  and [YOLO26n on Hailo-8L](https://danieldubinsky.github.io/personal-site/case-studies/yolo26n-hailo-L8/).
- Raspberry Pi: [Heating and cooling Raspberry Pi 5](https://www.raspberrypi.com/news/heating-and-cooling-raspberry-pi-5/)
  (80 °C soft throttle, 85 °C limit); [AI HAT+ documentation](https://www.raspberrypi.com/documentation/accessories/ai-hat-plus.html).
- Community ROS 2 work on Hailo, all small and none for a vehicle:
  `kyrikakis/hailo_tappas_ros2`, `ar-ray-code/yolo-hailort-ros2`, `mjforan/rpicam_hailo_ros`.
