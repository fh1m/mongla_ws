# Vision, against the world

> Dive 2 of four. Method and rubric: [`README.md`](README.md). Our side is read out of the tree
> at `file:line`. Their side carries a URL and a number.

---

## 1. What we do today

`src/mongla_vision` — ~22.4 k lines across 60 modules, 824 tests.

### 1.1 The pipeline, stage by stage

| stage | file:line | what it is |
|---|---|---|
| **capture** | `cameras/v4l2_mailbox.py:180-216` | raw V4L2 ioctl + mmap **mailbox**, one deep, newest wins. Measured: plain `read()` 396.1 ms stale → mailbox **16.9 ms**, blocking 0 ms. `FrameMeta.fresh=False` rather than repeating a frame |
| **preprocess** | `detection/preprocess.py:21-41` | present and **off everywhere**. The file's own header retracts the original claim: 17 configurations, 4 props, 3 venues, never meaningfully positive; on the gate **30.4 % → 1.2 %** |
| **detect** | `detection/hailo.py` (1 082 lines) | Hailo-8, one process-wide `VDevice`, **configure once / activate many** (4.15 ms swap vs a 10.18 ms frame), async `run_async` chosen because it releases the GIL (another thread woke 9.21 ms late on the blocking API, 0.05 ms on async) |
| **decode** | `detection/seg_decode.py:12-32` | raw seg heads decoded **in the quantised domain** — the class head is `zp=0, scale=1/255`, so thresholding bytes is bit-exact with thresholding floats. **31.5 ms → 0.93 ms** |
| **range crop** | `detection/rangecrop.py:1-56` | hysteresis crop because `imgsz` is baked into the HEF. Measured recall: apparent size 0.25 → **65.9 % full frame / 100 % cropped** |
| **track** | `tracking/roboflow_tracker.py` | OC-SORT (default), ByteTrack fallback, + a CV Kalman output smoother whose `_DT_REF_S = 0.0325` is the **measured** p50 detection interval |
| **lock ladder** | `tracking/lock_state.py:8-169` | DETECTION → FOLLOW (LK, 8.0 ms on one core) → ANCHOR (XFeat homography, 33.1 ms at 320×240) → LOST. Strict priority, **never blending**. Authority ramps linearly 0.70 s → 2.50 s, sized on 71 real gaps (p50 0.155, p90 0.651, p99 2.418, max 4.00 s) |
| **optics** | `optics.py:74-209` | flat-port Pinax rectification. Local focal grows with field angle: **685.1 px on axis vs 757.7 at r=367 — 10.6 % centre to corner**. Skipping refraction reads a 4 m gate at **5.15 m (+29 %)** |
| **metric** | `target_geometry.py` + `config/target_geometry.yaml` | `m_per_px = width_m / w_px` from a table of real prop widths. ⛔ the metric branch **has never run on the vehicle** |
| **pose** | `anchor/pose.py` | IPPE for the interval, **SQPnP** for the point estimate (p90 yaw 1.40° vs 3.38° at our 1.55 px noise). ⛔ planar **flip ambiguity is worst head-on**, which is the firing geometry |
| **flow** | `flow/flow_node.py` (1 405 lines) | the downward camera as the DVL substitute. `v = h·(flow_px/dt − f·ω)/f` — rotation subtracted **before** scaling. Measured: 3 × 30 cm slides, worst error **1.09 cm**; noise floor 0.57 mm/s; de-rotation **575.7 → 57.1 mm/s** on pure rotation |

Measured end to end: **18.0 ms** photon → detection; **80.9 Hz** standalone, **53.9 Hz** through
the ROS graph; **98.0 Hz** pipeline against `hailortcli --hw-only` at 97.9 FPS — the host is at
100 % of the chip.

### 1.2 The model-production path — the part never examined

The entire training doctrine is `.claude/skills/train-model/SKILL.md`, **40 lines**:
`yolo train data=… model=yolo11n.pt epochs=100 imgsz=640`, copy the weights, edit a config.

- **No training script, no augmentation code, no dataset versioning, no labelling tooling, no CI
  anywhere in the repository.** Training is a manual Ultralytics CLI run on another machine.
- The datasets are not in the tree: `tools/dataset_survey.py:18`, `tools/return_check.py:25-27`
  and `tools/conf_bar.py:13` all point at **hard-coded absolute paths** outside the workspace
  (`~/Work/.../2025/datasets`, `/home/fh1m/Music/detect`). 10 188 labelled images. Versioning is
  directory names: `robosub_octagon_n_200_final`, `..._final_again3`.
- **Labelling: nothing.** No CVAT, Label Studio, labelImg or Roboflow-annotate integration.
  `ROADMAP` M6: *real-pool auto-labelling — OPEN, new work; Bumblebee's is dead code.*
- Augmentation: **none of the 25 archived training configs** uses blur, rotation, perspective or
  contrast augmentation (`preprocess.py:44-45`) — which is also why a contrast fix at inference
  time is a domain shift.
- Deployment to the Hailo is a 6-step manual DFC path (`perception/hailo-vision.md:743-775`) with
  recorded traps: auto-NMS config fails on a custom class count, `.alls` accepts no comments,
  **calibration must be RGB**, `PYTHONPATH` must be cleared. Calibration set = 320 real pool
  frames.
- Then: a hand-written `<stem>.yaml` sidecar (**the Hailo backend refuses to construct without
  it**), and a row in `config/target_geometry.yaml` or every metric consumer loses range.
- ⛔ **No compiled artifact exists in the repo**: five `.pt`, eight `.yaml`, **zero `.hef`**.
- **How long any of this takes is recorded nowhere.**

What we *do* have, and it is unusual: an evaluation fleet. `tools/model_select.py` shows three
octagon models at an identical **mAP50 = 0.9950** scoring **29.2 % / 72.7 % / 68.3 %**
cross-session recall — two of them with byte-identical training configs and a 43-point gap.
`tools/detector_viewpoint_check.py` scores the **trivial whole-frame model first**: on the return
leg, null = 15.3 % at IoU 0.5, our detector = **9.7 %** — worse than guessing.

### 1.3 What the vision system cannot do today (each stated in the tree)

1. **No re-identification** — zero hits for `reid|embedding|appearance` across the package. Both
   trackers associate on IoU + motion only.
2. **No multi-view / stereo** — two cameras, two independent detectors, no cross-camera
   association. They share a process only because the Hailo has one VDevice.
3. **No self-supervision, no online learning, no auto-labelling** (`ROADMAP` M6).
4. **No segmentation model on our classes** — the seg models are stock COCO-80,
   `competition: false`.
5. **No SLAM, no map.** Tile phase bounds drift to one tile; it does not localise.
6. **The lock ladder does not steer** — it publishes `/lock`, and the control loop only acts when
   `vision.lock_s > 0`, which ships 0.
7. **Slalom has no model** (651 unique labelled frames, zero training runs) and the **return leg
   is visually unsupported** (the `gate` class scores 0.0 % at every threshold on 72 back-side
   frames).
8. **Detection range per prop is unmeasured** — the 10 px floor is still a COCO `person` number.
9. **Tool offsets are all `unmeasured: true` and read as ZERO** — and the miss equals the offset
   **at every range**.

---

## 2. What the best work does

*(Pending the research sweep for this dive.)*

---

## 3. The gap

---

## 4. Candidate moves

---

## 5. Rejected, with the reason
