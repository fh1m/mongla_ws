<!-- MERGED 2026-09-07 (srot -> main). These were TWO DIFFERENT DOCUMENTS
     that happened to share a filename: they share ZERO of 18 headings. Neither
     is a newer version of the other, so neither was discarded.
       Part 1 (from srot) -- the compile/runtime deep dive, rounds 29+.
       Part 2 (from main) -- the round-24 hardware evaluation that chose the HAT.
     If they ever disagree on a number, the one with the later round wins and the
     disagreement should be written down rather than silently reconciled. -->

# The Hailo-8 pipeline, measured — the CUDA-equivalent reference we never wrote

> Every number here was measured on **our** Pi 5 + AI HAT+ on 2026-09-03, with
> the harnesses in `tools/hailo_{stages,async,latency}.py`. Published figures are
> quoted only where marked, and where they disagree with our measurements the
> measurement wins and the disagreement is stated.

## 0. What the hardware actually is

```
Device Architecture   HAILO8            (26 TOPS, not the 8L)
HailoRT / driver      4.24.0            userspace and hailo_pci version-matched
PCIe                  Gen3 x1           8.0 GT/s, width 1 of the chip's 4
                                        -- x1 is the Pi 5 slot's limit, not a downtrain
```
**PCIe is already maxed for this host.** The chip can do x4; the Pi 5 connector
gives x1. There is no link tuning left, so PCIe is a fixed constraint rather
than a lever.

## 1. Every HEF we ship, parsed

```
gate_rescue_repair  3 classes   Multi Context x3   score th 0.050   IoU 0.70
bin_fire_blood      2 classes   Multi Context x3   score th 0.050   IoU 0.70
sauvc_sim          11 classes   Multi Context x3   score th 0.050   IoU 0.70
yolov11n (stock)   80 classes   Multi Context x3   score th 0.200   IoU 0.70
```
All: `UINT8 NHWC(640x640x3)` in, `FLOAT32 HAILO NMS BY CLASS` out.

**Two things this settles.** `hailo.py`'s claim that our models are compiled at
**0.05 is correct** — the DFC's `nms_scores_th` default is 0.3, and ours did not
take it. And **NMS is genuinely baked on-chip** (`HAILO NMS`, `Op YOLOV8`), which
is why host decode measures 0.02 ms.

⚠ **The stock models are baked at 0.200**, so the 0.12-0.15 operating point
`hailo.py` recommends is reachable only on our custom models.

## 2. Where a frame's time actually goes

`tools/hailo_stages.py`, synthetic frames, `gate_rescue_repair`:

```
  stage         median      p95     share
  letterbox       0.65     0.69     6.3 %
  infer           9.54     9.59    93.5 %
  decode          0.02     0.02     0.2 %
  TOTAL          10.20    10.23   -> 98.0 Hz
```

**And `hailortcli benchmark --hw-only` on the same HEF reports 97.9 FPS.**

So our Python pipeline runs at **100 % of the chip's own capability**. There is
no host overhead to recover on this model. That is the opposite of the case the
literature describes (a documented example ran 40-45 FPS at 15-20 % chip
utilisation, a 25x host-bound gap) — **on our model the chip is the bottleneck.**

## 3. THE FINDING: context count sets the ceiling — and the family sets the context count

```
model                    contexts        HW-only FPS
yolov8s  (11.3 MB)       Single             462.4
yolov8n  ( 5.1 MB)       Single             414.6
bin_fire_blood (2 cls)   Multi x3            98.6
gate_rescue_repair       Multi x3            98.1
sauvc_sim (11 cls)       Multi x3            98.3
yolov11n (80 cls)        Multi x3            92.7
yolov11s                 Multi x3            42.9
```

**Every multi-context model lands at ~98 FPS regardless of class count** (2, 3
and 11 classes are indistinguishable), and a *larger* single-context model is
**4.7x faster** than a smaller multi-context one. The binding constraint is the
**per-frame context reconfiguration over a x1 link**, not compute and not
classes.

Corroborated from the other side: batching amortises exactly that cost —
```
batch  1   99.6 FPS      batch  4  182.9 FPS
batch  2  142.3 FPS      batch  8  209.8 FPS      batch 16  226.9 FPS
```
2.07x at batch 8 is the context switch being paid once per eight frames instead
of once per frame. **Not usable for control** — filling a batch of 8 costs 38 ms
of latency — but it identifies the cause.

`--power-mode ultra_performance` is **+2.2 % for free** (98.1 -> 100.3) and we
have never used it.

### 3a. The confound, and the check that separated it

The paragraph above was first written as *"the binding constraint is context
reconfiguration, not compute"*, which does not follow from that table on its
own: **every multi-context model there is YOLO11 and both single-context models
are YOLOv8**, so "context count" and "model family" moved together. The Model
Zoo's own table (yolov11n 185 vs yolov8n 1036) says the same thing, and
distrusting it was correct — it is the round-24 trap — but distrusting a number
is not the same as separating a cause.

The discriminating evidence was already on the Pi. All four of those are
**Hailo's own Model Zoo compiles at the same 640 input on the same chip**:

```
yolov8n    5.1 MB   Single Context
yolov8s   11.3 MB   Single Context      <- LARGER and still single
yolov11n   7.0 MB   Multi Context x3
yolov11s  18.0 MB   Multi Context x3
```

It splits cleanly by **family**, not by size — the 11.3 MB YOLOv8 fits one
context while the 7.0 MB YOLO11 does not. So the DFC's allocator splits a
YOLO11 graph at 640 into three contexts even with Hailo's own tuned `.alls`,
and our custom 3-class YOLO11n is ×3 for the same structural reason their
80-class one is.

**Consequence, stated so it is not rediscovered:** recompiling our model to a
single context is very unlikely to be available. The 4.2x is not a win we are
leaving on the table — it is a win that belongs to a different architecture,
and reaching it means retraining on YOLOv8, which is out of this round's scope
and would have to be justified against accuracy, not FPS.

## 4. The blocking API's cost is constant, and only visible on a fast graph

```
model                     chip/frame   our infer   overhead
gate_rescue_repair (x3)     10.19 ms     9.54 ms    ~0 ms
yolov8n (single ctx)         2.41 ms     7.12 ms    4.71 ms
```
`InferVStreams.infer()` costs a fixed ~4.7 ms per call. Behind a 10 ms graph
that is invisible; in front of a 2.4 ms graph it is **66 % of the frame**.

## 5. Async inference — the number nobody had published

`tools/hailo_async.py`. The research found the API and **no measured batch-1
delta anywhere**. Ours:

```
                          yolov8n (single ctx)   gate_rescue_repair (x3)
blocking InferVStreams          141 Hz                   98.2 Hz
async depth 2                   225 Hz  (1.60x)          98.2 Hz  (1.00x)
async depth 4                   292 Hz  (2.07x)          98.2 Hz  (1.00x)
async depth 8                   343 Hz  (2.43x)
```

**The 1.00x is the result that proves the model.** Async recovers exactly the
overhead identified in §4 and nothing else: 2.43x where 66 % was overhead, and
*precisely nothing* where the graph was already chip-bound. Async is not a
general speed-up — it is the fix for one specific cost.

**Two prerequisites, or `run_async` raises `HAILO_STREAM_NOT_ACTIVATED (72)`:**
call `cm.activate()`, and `cm.wait_for_async_ready()` before each dispatch. The
device's own queue is 27 deep.

## 6. Throughput is not latency, and control pays latency

`tools/hailo_latency.py` — submit-to-result for the frame you actually steer on:

```
  depth  throughput      median      p95      max
      1   153.2 Hz      6.51 ms   6.55 ms   7.01 ms
      2   224.5 Hz      7.91 ms   8.82 ms  10.57 ms
      4   291.6 Hz     10.32 ms  13.44 ms  15.46 ms
      8   343.4 Hz     14.70 ms  22.68 ms  22.76 ms
```

**Depth 2 is the control-loop choice**: +47 % throughput for +1.4 ms of box age.
Depth 8 buys +124 % throughput and costs +8.2 ms — against a 50 Hz loop's 20 ms
budget, a p95 of 22.7 ms is a missed tick.

## 7. What this means for us, in order

**First, the thing that stops this being a race.** 98 Hz is already **above
the camera (~79 Hz)** and well above the **50 Hz control loop**. Chip
throughput is not on the critical path for anything the vehicle does. What is
still worth having is **latency**, because loop delay is what limits how hard
terminal alignment can be pushed before the hull oscillates.

1. **Do not chase the 4.2x.** §3a shows it belongs to the YOLOv8 family, not to
   a compile setting we have not tried. Changing families means retraining, and
   would have to be argued on accuracy.
2. **Async buys nothing on our current graph** — measured **1.00x**, not
   estimated. Revisit only if a single-context model ever ships.
3. `ultra_performance` is **+2.2 % free and unused** — the one unambiguous win
   in this whole campaign.
4. **The real levers for "never lose a detection" are not on this list.** They
   are the operating threshold (0.35-0.45 shipped against a 0.12-0.15 intended
   point), the tracker, and the coast — none of which are FPS problems.
5. Batching is the wrong tool for control, but it is the diagnostic that
   identified the context switch.

## 8. The trap this repeats

Round 24 recorded: `hailortcli benchmark` claimed **4.5x** between two model
families and the real pipeline gave **9 %**. It happened again here — the
benchmark says single-context is **4.2x** and our real pipeline gives **1.32x**
(98 -> 129 Hz), because the blocking API's fixed cost eats the difference.

The reconciliation is the useful part: the benchmark and the pipeline **agree
exactly** on a chip-bound model (97.9 vs 98.0) and **disagree by 3x** on a
transfer-bound one. A benchmark measures the chip; whether that is your number
depends on which side of the bottleneck you are on. **Both measurements were
right; they were answering different questions.**

---

## 9. The operating point, measured — and what the measurement does NOT show

Every launch path shipped conf **0.35-0.45**, the CUDA number, against HEFs
baked at 0.05 and a documented INT8 penalty of ~0.08. The Pi paths now default
to **0.15**. That was an inference until this run; 605 frames through
`gate_rescue_repair` on the forward camera, one pass, thresholds applied to the
same boxes:

```
  102 boxes above the HEF floor
  scores  min 0.050   p50 0.085   p90 0.215   max 0.850

    conf   boxes kept
    0.05         102
    0.12          26
    0.15          20
    0.20          15
    0.35           5
    0.45           5      <- 0.15 keeps 4.0x more
```

**The score distribution is real evidence and the recall claim is not.** p50 at
0.085 and p90 at 0.215 is a distribution squashed against the floor, exactly
the shape the INT8 penalty predicts, and it is why 0.45 is the wrong knob
setting for this backend.

But **the camera was pointed at a room, not at a prop.** Most of those 102
boxes are therefore false positives, and 0.45 rejecting them is the threshold
working, not a loss. So this measures the distribution, NOT recall — the plan's
own verification row ("the fix must show up as detections") is **still open**
and needs a gate/rescue/repair prop in frame.

0.15 stands on the distribution plus the baked floor. Re-run this against a
real prop before treating it as tuned.

---

# Round 32 — the GIL, the SRAM, and the wire format

Round 29 asked whether async inference is FASTER and answered no (§5, and that
still stands: 98.2 Hz either way on our multi-context graph). This round asked
a different question and the answer changed the whole pipeline.

## 10. THE BLOCKING API HOLDS THE GIL FOR ITS ENTIRE INFERENCE

`tools/gil_probe.py` — a sampler thread `time.sleep(1 ms)` in a loop, recording
how LATE it woke, with two controls, because a gap number without controls
cannot separate the subject from a badly-scheduling machine:

```
  CONTROL sleep 10 ms (releases the GIL)   woke late 0.05 ms   >4 ms:  0.0 %
  CONTROL pure-Python spin (holds it)                5.11 ms   >4 ms: 99.9 %
  InferVStreams.infer()                              9.21 ms   >4 ms: 99.6 %
  InferModel run_async + job.wait()                  0.05 ms   >4 ms:  0.0 %
```

The blocking call is **worse than a pure-Python spin**: CPython's spin yields
every 5 ms switch interval, and a C extension that never releases never yields
at all. So during every inference, in that process, **nothing else Python runs**
— not the camera's capture pump, not an rclpy executor thread, not a param
callback. 70+ times a second.

**This is what made composition look harmful.** With camera and detector in one
process the stage profile read:

```
  capture -> pump store    4.19 ms
  waiting in the slot     13.82 ms   <- 49 % of the budget
    of which our decode    1.93 ms
  inference               10.22 ms
  CAPTURE -> DETECTIONS   28.06 ms   period 12.16 ms
```

11.9 ms of that slot wait is the pump being frozen, so the consumer always took
a frame from *before* the current inference began. After switching to async:

```
  waiting in the slot      4.29 ms
  CAPTURE -> DETECTIONS   18.01 ms   (-36 %)
  pump captured (15 s)      2360 -> 4016   (the camera's full 268 Hz)
  pump skipped              1670 -> 22
```

**The rule, generally: any C extension you call at 70 Hz must be checked for GIL
behaviour before anything else in the process is blamed.** Two hypotheses died
here first — "the pump serves a stale driver backlog" (it does fall behind;
skipping to the newest moved the result 0.04 ms, because a full V4L2 queue is a
FOSSIL RECORD and skipping through fossils still yields a fossil) and "the
kernel is dropping frames" (that was our own skip counted as a drop: 1688
apparent, 7 real).

## 11. `configure()` COSTS CHIP SRAM; `activate()` DOES NOT

The single most expensive mistake available on this chip, and it is silent
until it is catastrophic.

* `InferModel.configure()` → allocates the network group into the Hailo-8's
  **on-chip SRAM**. Expensive, scarce.
* `ConfiguredInferModel.activate()` → makes an already-resident group the
  running one. Cheap; this is the camera-swap operation.

Calling `configure()` per swap fills the chip after a few dozen switches:

```
  CONTEXT_SWITCH_STATUS_SRAM_MEMORY_FULL
  HAILO_OUT_OF_FW_MEMORY (71)
```

…and then **every** inference fails forever in a tight retry loop — measured on
the vehicle as 98 % CPU, zero detections, and the image topic starved from 36 Hz
to 1.6. A single-detector profile never sees it, because it never swaps.

Two configured groups resident at once is fine — it is what the blocking path
always did (one `VDevice.configure` per detector at construction).

**Why SRAM is this tight here:** our HEFs are **multi-context**. `parse-hef`
says `Multi Context - Number of contexts: 3`, and the firmware string is
`4.24.0 (release,app,extended context switch buffer)`. A multi-context graph
does not fit on-chip at once; the firmware pages contexts through SRAM during
inference. That is also §3's ceiling mechanism seen from the other side.

## 12. THE TWO APIs RETURN DIFFERENT WIRE FORMATS

`InferVStreams` → a dict keyed by vstream name, holding a ragged per-class
object array. `InferModel` → writes into the buffer **you** bound, flat float32:

```
  [ count_0, (y1 x1 y2 x2 score) * count_0,
    count_1, (y1 x1 y2 x2 score) * count_1, ... ]
```

**PACKED, not fixed-stride** — and the buffer is sized for the worst case, which
makes fixed-stride look right. `parse-hef` states it exactly:

```
  HAILO NMS BY CLASS(number of classes: 3,
                     maximum bounding boxes per class: 100,
                     maximum frame size: 6012)
```

6012 bytes = 1503 floats = `3 * (1 + 100 * 5)`. The arithmetic is perfect for a
fixed stride and the layout is not one. Reading it that way returns **nothing
for every class after the first** — 22 detections where there were 56 — without
raising, and with every surviving box in exactly the right place.

`tools/hailo_api_equivalence.py` runs both APIs over the same frames and
compares detections. It caught this; a smoke test would not have, nor would any
check that only looked at class 0. Keep the blocking path
(`DUBURI_HAILO_FORCE_BLOCKING=1`) alive for exactly this comparison.

Also from `parse-hef`, worth knowing without re-deriving: `Score threshold:
0.050` (the baked floor — runtime `conf` can only tighten), `IoU threshold:
0.70`, input `UINT8 NHWC(640x640x3)`.

## 13. PCIe IS NOT A LEVER ON THIS BOARD — SETTLED

Round 29's research flagged that PCIe generation matters for **multi-context**
HEFs (Hailo staff: 281 vs 355 FPS Gen2 vs Gen3; YOLOv7 9 vs 25), and ours are
multi-context. So it was worth checking. It is already maxed:

```
  LnkCap: Speed 8GT/s, Width x4
  LnkSta: Speed 8GT/s, Width x1 (downgraded)
```

**8GT/s is Gen 3** — no `dtparam=pciex1_gen=3` needed, and none is set. The x1
width is the Pi 5's physical connector against a card capable of x4; it is not
configurable. Do not spend a reboot on this.

## 14. WHERE WE ACTUALLY ARE, AND WHAT IS LEFT

Chip ceiling, measured with `hailortcli benchmark`:

```
  FPS (hw_only) = 97.92        Latency (hw) = 8.34 ms
```

Ours: `infer()` takes **9.85 ms**, i.e. **1.51 ms of host work** on top of the
chip (letterbox, the buffer copy, the NMS decode). Full ROS stack, both cameras,
one paused, composed: **77.3 Hz**, 79 % of the chip's ceiling.

**A lever considered and rejected on the arithmetic: pipelining the decode.**
Now that the GIL is free, the camera *could* decode the next frame during
inference, taking the period from 11.76 ms toward 9.85 (≈101 Hz). It would make
detections OLDER: the decoded frame would wait a whole inference before its own
inference began, adding ~9.85 ms to age to buy ~20 % of rate. **Control pays
latency, not throughput** (§6). Decoding fresh at idle is the right design and
this is why.

---

## The accelerator disappearing after an apt upgrade (2026-09-05)

**Symptom:** `lspci` shows `Hailo-8 AI Processor (rev 01)`, `hailo_platform`
imports, `hailortcli` is on `PATH` — and there is no `/dev/hailo0`, no module in
`lsmod`, and `modinfo hailo_pci` says *"Module not found"*. Vision fails at
`VDevice()`, not at import, which is why it does not look like a missing driver.

**Cause:** the module was built by hand on 2026-08-29 and dropped into
`/usr/lib/modules/6.8.0-1063-raspi/kernel/drivers/misc/`. The Pi has since
booted **6.8.0-1064-raspi**. Two tells, both quick:

```bash
uname -r                                     # 6.8.0-1064-raspi
find /lib/modules -name 'hailo*'             # ...1063.../hailo_pci.ko  <- stale
dpkg -S <that path>                          # "no path found" -> hand-placed
ls /usr/lib/modules/<ver>/kernel/drivers/misc/ | head   # everything else is .ko.zst
```

A hand-placed module is **unowned and uncompressed** in a directory of
`.ko.zst`. An `apt` kernel bump takes it away silently, and it reads as "the
Hailo broke".

**Fix — DKMS, never another hand-built `.ko`.** Upstream ships `install_dkms`
with `AUTOINSTALL=yes`, which rebuilds on every future kernel.

```bash
git clone --depth 1 -b v4.24.0 https://github.com/hailo-ai/hailort-drivers.git ~/hailort_drivers
# assemble the layout the Kbuild expects (see the trap below), then:
sudo make -C <stage>/hailort/drivers/linux/pcie install_dkms
sudo install -m 0644 ~/hailort_drivers/linux/pcie/51-hailo-udev.rules /etc/udev/rules.d/
sudo modprobe hailo_pci
```

**⛔ The trap that makes `install_dkms` fail on a standalone clone.** The
driver's `Kbuild` hardcodes `COMMON_INCLUDE_DIRECTORY=../../../../common/include`
— it expects to live at `<hailort_root>/hailort/drivers/linux/pcie`. From a bare
`hailort-drivers` clone that path escapes the repo, and `install_dkms` dies on
`cp: cannot stat '../../../../common/include'`.

**The standalone build still SUCCEEDS**, because a missing `-I` path is not an
error — so you get a module that compiles and an install target that does not,
which is a confusing pair of symptoms. Assemble the tree instead of patching
upstream: `common/include` from the HailoRT source (`~/hailort_src`), plus the
clone's `common/` and `linux/` under `hailort/drivers/`. The HailoRT tarball's
own `hailort/drivers/` holds only `common` and `win` — it does **not** ship the
Linux driver, which is why the separate repo exists.

**The udev rule is not optional.** Without `51-hailo-udev.rules`, `/dev/hailo0`
is root-only and every ROS node fails to open it. With it: `crw-rw-rw-`.

**Verified, in this order — each step rules out a different failure:**

| check | good answer |
|---|---|
| `dkms status` | `hailo_pci/4.24.0, 6.8.0-1064-raspi, aarch64: installed` |
| `ls -la /dev/hailo0` | `crw-rw-rw-` (0666 — the udev rule took) |
| `hailortcli fw-control identify` | FW `4.24.0`, **`Device Architecture: HAILO8`** |
| `modinfo hailo_pci \| grep alias` | `pci:v00001E60d00002864...` — udev autoloads at boot, so no `modules-load.d` entry is needed (the existing `/etc/modules-load.d/hailo_pci.conf` is **empty** and does nothing) |
| **our own stack** | `HailoDetector(model_path=…).infer()` → **94.7 Hz, 10.56 ms/frame** |

The last row is the one that matters: a loaded module and a live `hailortcli`
both pass while the Python path is broken. Construct the real detector.

**Two diagnostic notes.** `dmesg` on this Pi is `dmesg_restrict=1`, so an empty
hailo grep as a normal user is **no evidence either way** — a wrong inference I
made and had to withdraw. And `hailortcli fw-control identify` exits **0** with
no output when there is no device, so its exit code is not a probe.

---

## The chip dropping off the PCIe bus (2026-09-06)

**Symptom, and it does not look like a hardware fault.** Every detector init
fails with `Failure in hailort driver ioctl`. `lspci` still lists the Hailo-8.
`hailortcli scan` still lists the device. Nothing holds `/dev/hailo0`
(`fuser` is empty). A `modprobe -r hailo_pci && modprobe hailo_pci` "succeeds"
— and afterwards there is **no `/dev/hailo0` at all**.

**The kernel says what is actually wrong**, and `dmesg` is the only place it is
written (`dmesg_restrict=1`, so read it with sudo):

```
hailo 0000:01:00.0: Failed writing fw control to pcie
hailo 0000:01:00.0: hailo_nnc_driver_down, timeout waiting for shutdown response
hailo 0000:01:00.0: Device disconnected while opening device      <- repeatedly
```

The device stopped answering on PCIe. `lspci` still shows it because that reads
cached config space, not a live device — which is why every userspace check
says "present" while every open fails.

**Recovery, without rebooting the Pi:** remove the device from the bus and
rescan, then reload the driver.

```bash
sudo modprobe -r hailo_pci
echo 1 | sudo tee /sys/bus/pci/devices/0000:01:00.0/remove
echo 1 | sudo tee /sys/bus/pci/rescan
sudo modprobe hailo_pci
hailortcli fw-control identify        # expect HAILO8, FW 4.24.0
```

Verified: `NNC Firmware loaded successfully`, `FW loaded, took 147 ms`,
`/dev/hailo0` back, and the full stack returned to 29.1 Hz detections.

**Likely cause, stated as a suspicion rather than a finding:** repeated
`kill -9` of processes holding the device. The driver logs
`timeout waiting for shutdown response` immediately before the disconnects,
which is what an ungraceful teardown of the NNC would produce. **Prefer SIGTERM
and let the node close its `VDevice`**; the restart scripts here use `kill -9`
and should not.

**Why this was expensive to diagnose, and the lesson:** the failure presented
as a ROS problem. `/detections` published nothing while `/image_raw` and
`/lock` ran at 30 Hz, QoS matched RELIABLE/VOLATILE on both sides, publisher
and subscriber were matched, `ros2 param get` answered, and a bare RELIABLE
pub/sub between two fresh processes worked. Six layers all said healthy. The
device had been gone the whole time, and the only honest signal — the detector
init failure — was one FATAL line at the top of a log full of INFO.

---

# APPENDIX (from `main`) — the round-24 evaluation that chose this hardware

> Kept verbatim from the pre-merge `main`. This is the *selection* evidence --
> whether the AI HAT+ was worth adopting at all -- where the part above is the
> *operating* reference. Both are measured; they answer different questions.

# Raspberry Pi AI HAT+ (Hailo-8) as a vision brain for Mongla

> **Status: MEASURED, and the answer is yes.** Round 24, 2026-09-02.
> Every number below was measured on the actual hardware, batch 1, and is
> reproducible with the harnesses named at the bottom. Nothing here is quoted
> from a datasheet or a vendor benchmark unless it says so.

## Why this exists

RoboSub 2025 cost us a **burnt Jetson Orin**, and the entire vision pipeline
runs on that one part. This is the diversification check.

## The headline

**It is a Hailo-8 (26 TOPS), not the Hailo-8L (13 TOPS) that was believed
bought** — and it sustains **~54 Hz end-to-end on YOLO11n @640 from a real
camera**, against a **20 Hz** vision control loop and a Jetson TensorRT
baseline of 20-30 Hz. It is roughly **2x the Jetson** on the same model, with
**no thermal throttling over 13 minutes**.

**How the identity was proven, rather than trusted.** `fw-control identify`
reporting `HAILO8` is one string from one tool. The control that actually
settles it: the HEFs used throughout are the Model Zoo's **`hailo8/`** builds,
and **HailoRT refuses to load a HEF compiled for a different architecture** — an
8L physically cannot run these. They load and run, and `hailortcli parse-hef`
prints `Architecture HEF was compiled for: HAILO8`. A device that failed this
would fail loudly, which is the point.

## Measured

**Hardware.** Pi **5** Model B, 4 GB, 4 cores, Ubuntu 24.04.4 (kernel 6.8,
aarch64), ROS 2 **Jazzy**. Hailo-8 on PCIe **Gen3 x4**, fw 4.24.0, core clock
400 MHz. Camera: `0c45:636d` USB **2.0** global shutter, MJPG 1280x720@120 /
640x400@210.

**M1 — chip only** (`hailortcli benchmark`, batch 1). Note `hw_only` excludes
the host entirely; `streaming` includes the PCIe round trip:

| model | hw_only FPS | streaming FPS | HW latency |
|---|---|---|---|
| yolov8n  | 416.7 | 329.7 | 3.47 ms |
| yolov8s  | 476.1 | 232.8 | 5.82 ms |
| **yolov11n** | **92.4** | **91.9** | **7.81 ms** |
| yolov11s | 42.8 | 43.0 | 19.66 ms |

**M3/M4 — end-to-end, real camera, 1280x720 MJPG, batch 1.** This is the number
a mission actually gets:

| model | grab | letterbox | infer | decode | **loop** | **Hz** |
|---|---|---|---|---|---|---|
| **yolov11n** | 4.67 | 1.35 | 11.41 | 0.04 | **17.47 ms** | **57.2** |
| yolov8n | 7.10 | 1.37 | 7.50 | 0.04 | 16.01 ms | 62.5 |

From in-memory frames (no camera), yolov11n reaches **82.8 Hz**.

**M6 — 13-minute soak**, continuous camera inference:

```
first minute 57.53 Hz -> last minute 54.13 Hz  (-5.9 %)
min 52.96   max 57.53   median 53.57 Hz
temperature 55.1 -> 56.2 C, never above 58.4 C, ARM clock pegged 2.4 GHz
```

The dip is **settling, not decay** — it bottoms at minute 4 (52.96) and climbs
back to 54.13 by minute 13. No active throttle bit ever set.

## Three findings that change how to read the vendor numbers

**1. The published model-family gap does NOT survive contact with a mission,
and this retires a risk this project had written down.** The raw benchmark says
yolov8n is **4.5x** faster than yolov11n (417 vs 92 FPS), which looked like a
reason to abandon YOLO11 and retrain everything on YOLOv8. In the real pipeline
the gap is **1.5x on inference** (7.50 vs 11.41 ms) and **9 % on the loop**
(62.5 vs 57.2 Hz). The reason is that throughput and batch-1 latency are
different quantities: a benchmark saturates the chip with queued frames, and an
AUV has exactly one frame in flight. **Keep YOLO11.** The retraining that the
benchmark seemed to demand would have bought 9 %.

**2. ~~NMS runs ON-CHIP~~ — RETRACTED. HailoRT does NMS on the HOST, inside
its own library call.** The observation was right and the explanation was
wrong. The Model Zoo HEFs carry a `yolov8_nms_postprocess` op and the output is
already-decoded boxes, so a Python-side decode timer reads **0.04 ms** — but
that is because the work happens *inside* `pipe.infer()` and gets attributed to
"infer", not because the accelerator did it. The DFC's own tables settle it:

```
NN_CORE_META_ARCHS = [SSD, YOLOV5, CENTERNET, YOLOV6]
CPU_META_ARCHS     = [YOLOV5, YOLOX, YOLOV5_SEG, SSD, YOLOV8, DAMOYOLO]
```

YOLOV8 — which is YOLO11's head — appears **only in the CPU list**. Asking for
`engine=nn_core` is refused outright: *"The specified meta architecture yolov8
cannot be run on chip."* So for our model family the NMS is host CPU work.

It is still **cheap** — e2e infer 11.41 ms against `hailortcli` hw_only
10.8 ms puts it under a millisecond — but it is cheap **because our models have
3 classes, not 80**, and it will grow with class count. The correct statement is
"HailoRT's NMS is fast enough to disappear into the infer call here", not "the
chip does it".

**3. The bottleneck is now the CAMERA.** Of a 17.47 ms loop, `grab` is
4.67-7.10 ms and letterbox 1.35 ms; the accelerator is 11.41 ms and everything
else is noise. Further speed comes from overlapping capture with inference (the
loop measured here is strictly serial), not from a smaller model.

## Consequences for the pipeline — read before integrating

- **`conf` can only be tightened, never loosened.** The HEF bakes in
  `Score threshold: 0.200` and `IoU: 0.70` at compile time. `detector.yaml`'s
  live `conf` param and `vision.ctrl_conf` still work *above* 0.2, but nothing
  at runtime can go below it. To detect fainter targets the HEF must be
  **recompiled**. This differs from both the `.pt` and `.engine` backends, where
  `conf` is a per-`predict()` argument.
- **Class count is baked in too** (`Classes: 80` on the stock HEFs). Our models
  carry 2-11 classes, so the NMS config must be set per model at compile time.
- **The `.yaml` sidecar becomes mandatory, not merely advisable.** A `.hef`
  carries no `names` table, so the documented fallback to the model's embedded
  names does not exist. With no sidecar the allowlist is empty and the detector
  returns `[]` **every frame** with a single warning. `yolo.py:52-63` already
  derives the sidecar by suffix-swap, so `gate_rescue_repair.hef` ->
  `gate_rescue_repair.yaml` resolves with no code change.

## Building the Python bindings (the wheel is NOT public)

`hailo_platform` is not on PyPI and there is no apt repo; the wheel sits behind
the Hailo Developer Zone login. It builds from source, and this is the recipe
that worked:

```bash
git clone --depth 1 -b v4.24.0 https://github.com/hailo-ai/hailort.git
cd hailort && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j2
cd hailort/libhailort/bindings/python/platform
pip install --break-system-packages --no-build-isolation .
```

Producing `hailort-4.24.0-cp312-cp312-linux_aarch64.whl`.

Two traps: **`-DHAILO_BUILD_PYBIND=ON` on the top-level cmake does nothing** (it
lands in the cache as `UNINITIALIZED` and no `_pyhailort.so` is produced) — the
bindings are built by their own `setup.py`, which invokes cmake itself. And
**`-j4` OOMs on a 4 GB Pi**; use `-j2`.

**The bindings version must equal the installed `libhailort` version.** Ours is
4.24.0 both sides.

## M2 / M5 — our own model, compiled and measured (2026-09-02)

`gate_rescue_repair` (YOLO11n, 3 classes) is compiled and running on the chip.

**M2 — our model is FASTER than the stock proxy**, which retires the caveat this
document previously carried:

| | hw_only FPS | latency |
|---|---|---|
| stock COCO yolov11n (80 classes) | 92.4 | 7.81 ms |
| **ours, `gate_rescue_repair` (3 classes)** | **97.8** | 8.34 ms |

5.8 % faster, exactly as predicted from the smaller NMS workload. **The stock-HEF
proxy was fair and slightly conservative.**

**M5 — the measurement that decides it.** 100 **real pool** frames (disjoint
from the 320 used for calibration), conf 0.20 / IoU 0.70 on both sides.

**The result that matters: INT8 does not move the box.**

| comparison | recall | centre error |
|---|---|---|
| `.pt` fp32 -> `.onnx` fp32 — **the harness's own noise floor** | 64.2 % | **2.72 px** |
| `.onnx` fp32 -> `.hef` int8 | 49.3 % | **2.14 px** |

Centre error under quantization (2.14-2.65 px across every threshold tested) is
**at or below the floor two full-precision runs of the same weights produce**.
Since `vision.align` steers on centre offset and the torpedo lock holds against
a 47.5 mm opening, **this is the number that had to be small, and it is.**

**Read the recall numbers only as differences, never as absolutes.** Two fp32
runs of identical weights only "agree" 64.2 % of the time under greedy IoU>0.5
matching, because overlapping same-class boxes defeat greedy assignment. **Had I
not run that fp32 control I would have reported "quantization destroys 54 % of
detections", which is badly wrong.**

**What quantization actually costs: about 0.08 of confidence.** The int8 model
finds the same objects and scores them lower, so they fall under a threshold
tuned for fp32.

**And a hypothesis of mine that the measurement KILLED.** I predicted that
compiling with a lower baked threshold and filtering back to 0.20 would recover
them. It does not — the 0.05 build filtered to 0.20 returns the *identical* 104
detections. The detections are not being clipped at compile time; the int8 model
genuinely scores them lower. The fix is to **actually run at a lower threshold**,
which the low baked threshold is what *permits*:

| runtime th | detections | recall vs fp32 | centre px |
|---|---|---|---|
| 0.20 | 104 | 50.0 % | 2.48 |
| 0.15 | 141 | 57.6 % | 2.54 |
| **0.12** | **173** | **63.9 %** | **2.55** |
| 0.10 | 210 | 69.4 % | 2.52 |

**At 0.12 the int8 model reaches 63.9 % against the 64.2 % fp32-vs-fp32 ceiling
— statistically indistinguishable from a full-precision run.**

### The recipe, therefore

1. Compile the HEF with `nms_scores_th` **well below** the intended runtime
   value (0.05 used here). It costs nothing — 97.7 FPS either way — and it is
   the only thing that makes step 2 possible, since **runtime conf can only
   tighten**.
2. Run at roughly **0.12-0.15** where fp32 would use 0.20. Budget ~0.08 of
   confidence for INT8.
3. **Do not compensate for localization.** It is not degraded.

### Reproducing the compile

```bash
# ONNX: no NMS in the graph, opset 11, Hailo adds its own
yolo export model=gate_rescue_repair.pt format=onnx opset=11 nms=False

hailo parser onnx gate_rescue_repair.onnx --hw-arch hailo8 \
  --start-node-names images \
  --end-node-names /model.23/cv2.0/cv2.0.2/Conv /model.23/cv3.0/cv3.0.2/Conv \
                   /model.23/cv2.1/cv2.1.2/Conv /model.23/cv3.1/cv3.1.2/Conv \
                   /model.23/cv2.2/cv2.2.2/Conv /model.23/cv3.2/cv3.2.2/Conv
hailo optimize gate_rescue_repair.har --hw-arch hailo8 \
  --calib-set-path calib_set.npy --model-script gate_rescue_repair.alls
hailo compiler gate_rescue_repair_optimized.har --hw-arch hailo8
```

**Traps, each of which cost real time:**

- **The DFC's auto NMS config fails** on a custom class count with
  `The layer named  doesn't exist in the HN` (note the empty name). Write the
  config JSON explicitly, mapping HAR layer names to strides. Get them from the
  HAR, not the ONNX — ours were `conv51/conv54` (stride 8), `conv62/conv65`
  (16), `conv77/conv80` (32).
- **`.alls` accepts NO comments** — not `//`, not `#`.
- **Calibration must be RGB.** Ultralytics converts BGR->RGB before the network,
  so the graph expects RGB; calibrating on BGR quantizes the wrong channel
  statistics and nothing reports it.
- **The venv is not enough.** This dev box sources ROS, which sets `PYTHONPATH`
  ahead of the venv's own site-packages, so a correctly-installed protobuf
  3.20.3 still imported 7.36.1 from `~/.local` and died with
  `'MessageFactory' object has no attribute 'GetPrototype'`. Clear `PYTHONPATH`.
- **DFC 3.34.0 HEFs load fine on HailoRT 4.24.0** — verified, despite the
  version numbers looking unrelated.

**Calibration set: 320 REAL pool frames**, not sim. Found in an old pendrive
backup (`gate_abid`, genuine underwater footage with surface caustics). The eval
100 are disjoint from the 320.

## Performance: 41.5 -> 82.3 Hz, with no loss of field of view

The first end-to-end harness was strictly serial and used the camera at 1280x720
@90. Re-measured and tuned, on our own model:

| configuration | e2e Hz |
|---|---|
| 1280x720@90, serial (the original baseline) | **41.5** |
| 1280x720@90, pipelined | 46.4 |
| 640x360@90, pipelined | 79.5 |
| **640x360@210, pipelined** | **82.3** |

**1.98x, and it costs nothing in image quality.** Two measurements make that
safe to say rather than hope:

1. **640x360 has the SAME field of view as 1280x720.** Measured, not eyeballed —
   a centre patch matched into the reference across scales gives **scale 1.01,
   correlation 0.98**. A first attempt to judge this by eye was worthless
   because the scene moved between captures. This mattered: 640x400 *looks*
   like a crop, and this codebase already has one FOV error on record (the
   in-air 80 deg figure used where the in-water 57.7 deg applies).
2. **There is no resolution loss either.** The network input is a 640x640
   letterbox, so a 1280x720 frame is downscaled to 640x360 *anyway*. Capturing
   at 640x360 natively produces the identical network input and skips the
   downscale.

**The camera was never bandwidth-bound — it was a SETTING.** Sweeping the
requested rate at 640x360 MJPG:

| asked fps | driver grab | JPEG decode | delivered |
|---|---|---|---|
| 30 | 30.49 ms | 1.53 ms | 30.2 Hz |
| 90 | 10.52 ms | 1.52 ms | 68.8 Hz |
| **210** | **5.06 ms** | 2.92 ms | **127.8 Hz** |

JPEG decode is **1.5 ms** — trivial. The cost was *waiting for the sensor* at a
frame interval nobody had set. At 210 the camera stops being the bottleneck
entirely (127.8 Hz against a ~95 Hz chip).

**Where the remaining gap is.** With frames preloaded and no camera at all, the
loop runs at **95.4 Hz**; live it is 82.3. The ~1.7 ms difference is thread
contention on 4 cores (grabber + preprocess + infer + HailoRT's own threads).
Closing it further has poor returns against a **20 Hz** requirement.

### Two chip facts the benchmark hides

**`hailortcli benchmark` feeds synthetic data**, so host-side NMS has nothing to
do and every model reports ~98 FPS. On real frames:

| model | classes | infer ms | Hz | dets/frame |
|---|---|---|---|---|
| stock COCO yolov11n | 80 | 12.24 | 81.7 | 0.3 |
| gate_rescue_repair | 3 | 10.49 | 95.4 | 3.3 |
| bin_fire_blood | 2 | 10.16 | 98.4 | 1.4 |
| sauvc_sim | 11 | 10.34 | 96.7 | 0.0 |

**Class count costs (~2 ms from 3 to 80 classes, via the output tensor);
detection count does not.** I predicted the opposite — that the low 0.05
threshold would cost frame rate through extra NMS work — and the measurement
killed it: the model with the *most* detections per frame is among the fastest.
2 -> 11 classes is only 0.18 ms, so `sauvc_sim` needed no special handling.

### Both cameras on one chip

Mongla runs a forward and a downward detector. Measured:

- **A second `VDevice` is refused** — `HAILO_OUT_OF_PHYSICAL_DEVICES (74)`. One
  process gets one VDevice; two independent inference processes will not work.
- **Both HEFs on ONE VDevice does work** (two network groups).
- Solo on that shared device: 91.5 and 97.3 Hz.
- **Switching network group every frame — the worst case — costs 13.90 ms,
  i.e. 71.9 Hz combined, ~36 Hz per camera.** Still 1.8x the control loop.

In practice it is better than that: `detector_node` already latches
`active_camera` and pauses the inactive detector, so the switch happens when a
mission changes camera, not every frame — the active camera gets the full ~92 Hz.

### Settings to carry into the integration

```
camera : 640x360 MJPG, CAP_PROP_FPS 210, CAP_PROP_BUFFERSIZE 1
loop   : capture thread + preprocess thread, single-slot drop-stale queues
conf   : 0.12-0.15  (fp32 would use 0.20)
```

## Verdict

| question | answer |
|---|---|
| How good is it? | **Hailo-8, 26 TOPS. 54-57 Hz sustained on YOLO11n @640, ~2x the Jetson's TensorRT 20-30 Hz.** |
| Can it fly RoboSub/SAUVC? | **Yes.** ~2.7x headroom over the 20 Hz control loop, no thermal decay over 13 min, and **quantization does not move the bbox centre** (2.5 px, at the harness's own fp32-vs-fp32 noise floor of 2.72 px). |
| Best performance from it? | Overlap capture with inference (the camera is the bottleneck, not the chip); keep YOLO11; consider **yolo11s** — at 42.8 FPS chip-side it still clears 20 Hz and buys real accuracy. |

**What is still a proxy, and what is not.** The *speed* numbers in the timing
table (57 Hz e2e, the 13-minute soak) were taken on **stock COCO** HEFs; M2 then
measured our own model at **97.8 vs 92.4 FPS**, so those figures are
conservative by ~6 % and the proxy is retired. The *accuracy* result (M5) was
measured on our own `gate_rescue_repair` weights against real pool frames — no
proxy involved.

**The remaining gap is scope, not doubt:** only `gate_rescue_repair` has been
compiled. `bin_fire_blood`, `sauvc_sim` and the rest follow the same recipe, and
`sauvc_sim` has 11 classes, so its NMS cost (host-side, see above) is worth
re-measuring rather than assuming.

## Reproducing

Harnesses (scratchpad, not in the repo): `host_cost.py` (host-only stage
timing), `cam_cost.py` (camera capture path), `e2e.py` (M3/M4 full pipeline),
`soak.py` (M6). Each prints medians over a sustained window and `e2e.py`
**reports total detections and warns when they are zero** — a pipeline that
returns nothing posts a beautiful frame rate otherwise.

## Not addressed here

**ROS.** The Pi runs **Jazzy**; duburi_ws is **Humble**; the two do not
interoperate — and worse than "unsupported", a Jazzy `ros2 topic list` can drive
a Humble subscriber out of memory (`ros2/rmw_fastrtps#797`). Deliberately left
open until the numbers justified going further. They now do.