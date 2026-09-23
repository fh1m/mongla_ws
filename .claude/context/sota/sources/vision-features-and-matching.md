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

