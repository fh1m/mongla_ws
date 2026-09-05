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
