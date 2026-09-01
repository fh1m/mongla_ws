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

**2. NMS runs ON-CHIP, and that is why a 4-core Pi keeps up.** The Model Zoo
HEFs carry a `yolov8_nms_postprocess` op, so the output is already-decoded
boxes rather than raw feature maps. Host-side decode measures **0.04 ms**. On
boards where NMS lands on the CPU this is normally the bottleneck; here it is
free.

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

## Still blocked: compiling OUR models

**M2 and M5 are not done**, and they are the remaining gap. The Hailo
**Dataflow Compiler is x86_64-only** — the exact mirror of the existing rule
that TensorRT engines must be built *on* the Jetson, pointing the opposite way,
which is a good way to lose a day. It is also **not on PyPI**: it needs a Hailo
Developer Zone account (free) to download.

The dev box is a valid DFC host: x86_64, Ubuntu 22.04, **Python 3.10** — which
is exactly the `cp310` the DFC wheel targets.

Once available:

```
YOLO11n .pt --export--> ONNX (NMS out of the graph; Hailo does it on-chip)
            --DFC on the dev box--> .hef      + calibration set (~1024 frames)
            --copy--> Pi, beside the existing .yaml sidecar
```

**Training does not change** — keep fine-tuning YOLO11n on the RTX 2060.

**The calibration set must be real pool imagery, not sim renders.** Sim frames
are documented in `CLAUDE.md` as too clean, and thresholds tuned on them do not
transfer; calibrating INT8 on them would bake that optimism into the weights
themselves.

**M5, when it can run, is the one that decides competition viability**, and the
metric is Mongla-specific: not mAP, but **how far quantization moves the bbox
CENTRE in pixels**. The control stack is pixel-native — `vision.align` steers on
centre offset, and `precision-alignment.md` holds the hull against a torpedo
opening of radius **47.5 mm**. A model that keeps its mAP while jittering its
centres is worse for us than one that loses a little recall.

## Verdict

| question | answer |
|---|---|
| How good is it? | **Hailo-8, 26 TOPS. 54-57 Hz sustained on YOLO11n @640, ~2x the Jetson's TensorRT 20-30 Hz.** |
| Can it fly RoboSub/SAUVC? | **On throughput and thermals, yes, with ~2.7x headroom over the 20 Hz control loop.** Not yet proven on quantized accuracy (M5). |
| Best performance from it? | Overlap capture with inference (the camera is the bottleneck, not the chip); keep YOLO11; consider **yolo11s** — at 42.8 FPS chip-side it still clears 20 Hz and buys real accuracy. |

**The honest caveat:** every measurement above used **stock COCO** HEFs as a
proxy for our models. That is a fair proxy for *speed* — same architecture, same
resolution, and our 2-11 classes make NMS cheaper than COCO's 80, so the real
models should be marginally faster. It is **no proxy at all for accuracy**,
which is exactly what M5 exists to measure.

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
