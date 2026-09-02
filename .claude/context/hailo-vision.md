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
