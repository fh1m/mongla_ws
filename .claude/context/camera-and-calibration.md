# Cameras and calibration — what the field knows, and what we measured

> Round 39. Everything here is either **cited with its number** or **measured
> on our own hardware**. Where a source disagrees with a measurement of ours,
> the measurement wins and the disagreement is stated.

---

## 1. THE HEADLINE DEFECT: our tilt metric read ZERO at 55°

The operator reported being unable to reach "steeply tilted" no matter how
far they tilted. **They were right, and it was our bug.**

The metric compared the board's two **diagonals**. Tilting about a horizontal
or vertical axis — the motion a person actually makes — projects a rectangle
to a **symmetric trapezoid, whose diagonals are equal by symmetry**. On
synthetic ground truth:

| tilt | old diagonal metric | ROS skew |
|---|---|---|
| 15° | **0.0000** | 0.054 |
| 30° | **0.0000** | 0.115 |
| 55° | **0.0000** | 0.284 |

Only a *diagonal-axis* tilt moved it at all, and only to 0.0059 — against a
"steep" threshold of 0.36. **The band was unreachable by construction.**

Proven on the operator's own 16 frames, which is what settles it:

| band | OLD metric | NEW metric |
|---|---|---|
| square-on | **14** | 4 |
| slight | 2 | 3 |
| clear | 0 | 1 |
| **steep** | **0** | **8** |

They were tilting properly the whole time.

**Replacement**: the skew `ros-perception/image_pipeline` has used for years —
how far a corner of the projected quad departs from 90°:

    skew = min(1, 2·|π/2 − angle(up_left, up_right, down_right)|)

Its whole progress model is worth copying and now is: `param_names =
["X","Y","Size","Skew"]`, `param_ranges = [0.7, 0.7, 0.4, 0.5]`, a sample is
rejected if within **0.2** L1 of any existing one, and it is done at **40
samples or full coverage**. It also has a `max_chessboard_speed` gate that
rejects a *moving* board — a blur guard.

---

## 2. TILT STOPS AT 45°, and "as steep as you can" is wrong

calib.io, explicitly: use fronto-parallel **and** up to **±45° in both
horizontal and vertical**, because *"tilting more is usually not a good idea
as feature localization accuracy suffers and can become biased."*

Measured skew↔angle on our geometry: **0.03 = 10°, 0.075 = 20°, 0.13 = 33°,
0.20 = 45°**. Our bands are set from that, and the page names the **angle**
rather than an adjective.

Other rules from the same source, with numbers: **≥6 images** minimum; target
should **cover ≥half the frame** fronto-parallel; calibrate **at the working
distance** and never change focus after; and — the one that matters most —
*"low reprojection error does not equal a good camera calibration"*, with
**parameter uncertainties** the better indicator.

---

## 3. REPROJECTION RMS IS NOT A QUALITY METRIC. Max ERE is.

Published: *"low reprojection errors of less than 0.3 can occur even when
calibration is obviously off."* We have three instances of exactly that —
fx **835.7 / 969.9 / 1011.2**, each with a comfortable residual.

**AprilCal** (Richardson et al., IROS 2013) is the answer, and its measured
result is the reason this round implemented it rather than citing it.
Novices with guidance vs novices with plain OpenCV:

| | OpenCV | AprilCal |
|---|---|---|
| mean reprojection error | 0.728 px | **0.229 px** |
| **worst-case** reprojection | 38.646 px | **1.651 px** (23×) |
| focal length std dev | 9.0 | **1.2** (7.5×) |

**6–8 images** to under 1 px. **13 of 16 subjects had never calibrated
anything**, and all produced good calibrations.

**Max ERE** — sample the posterior, project a grid of test points through
each draw, take the worst point's spread. It reports the part of the image
the data has *not* constrained, which is the part that silently ruins a
bearing. Measured on our own 19 frames: **37.18 px**, while σ(fx) said
"HIGH" and the tilt check said "OK".

⚠ Our posterior is the k-fold fits rather than a sampled covariance, so it is
a **lower bound** on the spread, and is recorded as such.

---

## 4. NEXT-BEST-POSE, implemented and verified to beat random

For each candidate pose: synthesise the observation it would produce under
the current calibration, add it, refit, recompute Max ERE, ask for the best.
Measured on the real frames:

| | Max ERE |
|---|---|
| current (12 views) | 36.17 px |
| **best suggestion** (centre, 10°) | **12.63 px** |
| random candidate mean | 15.83 px |
| worst candidate | 20.88 px |

One well-chosen view takes 36 → 12.6, and the suggestion beats a random pose
by 20 %. 12 candidates in 5.6 s on the Pi.

Two traps found while building it, both recorded because both are silent:
synthetic views must carry **noise** (a noiseless one fits better than any
real view, so the suggester would score a view that cannot be captured), and
`calibrateCameraRO` **refuses** some synthetic-augmented sets — it returns a
scalar where the released object points belong — which crashed the suggester
on exactly the inputs it exists to evaluate.

The related VI-calibration NBV work (arXiv 2309.14514) confirms the design:
they use **Fisher information / mutual information** over an **expert-chosen
candidate set** precisely to stay real-time, and stop when the gain falls
below a threshold. **381 s vs Kalibr's 455 s**, with lower Shannon entropy.

**Calibration Wizard** (ICCV 2019) minimises the **trace of the intrinsics
covariance** (A-optimality) and weights corners by their **autocorrelation
matrix** as an inverse-covariance estimate. Its result: **15 guided images
beat 50 freely-acquired ones**; `3 free + 1 wizard` (1.455) beats `10 free`
(1.664).

---

## 5. BLUR IS A CAPTURE GATE, and OpenCV ships the metric

`cv2.estimateChessboardSharpness`, documented target **< 3 px**. On the 21
frames captured under the old auto-exposure: **median 7.30 px**, and only
**3 of 13** in spec. A blurred view is not a cheap view — it biases the
corners the whole calibration is built on, and nothing downstream can tell it
from a good one.

---

## 6. THE CAMERA: what we measured on the Fantech Luminous C30

**Auto exposure chose a 200 ms shutter.** Same room, same lens:

| setting | mean | clipped | sharpness |
|---|---|---|---|
| auto (`exposure_time_absolute = 2000`) | 26.6 | — | 163 |
| **manual exp 50 (5 ms), brightness 150** | **135.0** | 6.5 % | 122 |
| manual exp 50, brightness 255 | 220.2 | 53.5 % | 57 |

Blur from rotation is `f·ω·t`, so 200 ms at f 514 and 0.64 rad/s smears
**66 px**. **`gain` is INERT on this unit** — 20/50/100 give identical
frames. It is the camera, not the room: the downward camera reads mean 179
in the same room.

**THE FRAME RATE IS THE CAMERA LYING.** V4L2 accepts and *reports* 30.000 fps
(30/1); raw `v4l2-ctl --stream-mmap`, with no OpenCV anywhere, delivers
**15.25 fps**. And it is **exactly 15.0 under every condition** — every
resolution, MJPG and YUYV, all three anti-flicker settings, exposures from
1 ms to 200 ms, auto or manual, alone on its own USB bus. So it is **not**
bandwidth, not exposure, not anti-flicker, and not our software.

The historical "Fantech ran at 30.3 Hz" note is most likely a
**misattribution**: round 38 found the udev rules had the two cameras
**swapped**, and the global-shutter unit does 210 fps in MJPG.

**No focus control exists in v4l2** on this camera, despite the marketing
claiming autofocus. If it does refocus, we cannot lock it — and calib.io's
rule is that focus must not change after calibration. **Open risk**, and a
candidate explanation for the historical fx spread.

Spec vs measured: marketing says **106° FOV** (diagonal, and generous);
we measure **HFOV 73.1°**.

---

## 7. Settled negatives — measured or cited, do not re-derive

- **Deep single-image calibration** (AnyCalib, DeepCalib, WildCamera,
  UniDepth): median focal error **4–27 %**. We need ~1 % — velocity scale is
  linear in `f`. **10–25× too coarse.** Auto-calibration from the scene
  cannot replace a target for our metric path.
- **`findChessboardCornersSB`**: 38.1 % hit rate against 76.2 % for
  `findChessboardCorners` on our own 21 frames.
- **Full-resolution detection**: 61.9 % vs 76.2 % at 480 px, and 4.5× the
  cost. 480 px is the optimum, beating both smaller and larger.
- **`CALIB_CB_FAST_CHECK`**: a wash (306.8 vs 307.4 ms, identical hit rate).
- **Deblurring before detection**: blind deconvolution *"fails frequently for
  larger blur"* and assumes spatially-invariant blur, which rotation
  violates. Shorten the shutter instead.
- **Limiting OpenCV threads** to protect the capture loop: **15.0 fps in both
  arms.** Refuted.

---

## 8. THE LIBRARY: every calibration is kept, and re-appliable

`src/mongla_vision/config/calibration/` is the vehicle's permanent store.
Every calibration this project has produced lives there, ships with the
package, and is listed in the web tool under **Saved calibrations** with the
fields an operator actually picks by — medium, resolution, fx, view count,
Max ERE, the camera it was taken on, and when.

**One click applies a saved calibration to a CONNECTED camera.** That is the
whole point: a swapped camera on competition ground costs two minutes
instead of ten with a printed board in a lit room. The apply button is only
offered for cameras `/cameras` currently sees — applying to an absent device
would write a file claiming a unit nobody can check.

⛔ **APPLYING IS AN ASSERTION, AND THE FILE SAYS SO.** The operator is
claiming "this is the same physical unit". The written file records
`identity_source: operator asserted at apply time` plus `applied_from` and
`applied_on`, and never pretends the calibration was captured on that
camera. The day somebody asks why a calibration claims a camera it never
saw, that field is the answer — and it is the difference between a shortcut
and a forged provenance. An existing file at the destination is moved to
`.bak-HHMMSS`, never overwritten: it cost somebody a board and ten minutes.

## 9. CALIBRATING IN WATER — a validation, not the method

`--medium air|water`, and a matching pair of buttons on the page.

**The primary method is still AIR.** The Pinax model says calibrate once in
air and correct the flat port's refraction analytically, which is what
`RefractiveRectifier` already does.

**But the correction itself has never been checked, and the record said
otherwise.** `f_water = 741` was not measured — it is `f_air = 514` put
through Snell, so quoting it as a measured result and then "validating" the
model against it is circular. An in-water calibration measures the
air-plus-port-plus-water system directly, which makes it the first genuinely
independent number for `f_water` rather than a nicety. `BUGS.md` §11 §1a
carries the full correction.

Three consequences, all implemented and tested:

1. **The medium is in the filename.** `pi_forward_1280x720.json` vs
   `pi_forward_1280x720_water.json`. Air and water differ by **1.442×** on
   this hull — 63.8° air **measured**, 46.7° water **derived from it by
   Snell** (`sin(31.9°)/sin(23.36°)` = 1.3333, plain water; see the
   correction in `BUGS.md` §1a, which previously described that
   tangent ratio as a 44 % disagreement with the literature's 25–33 %
   *sine* ratio — it was never a disagreement). So a water file that
   overwrote the air one would be a silent 44 % scale error on every range
   and velocity downstream — the size that still looks plausible. The naming
   rule has **one definition**, `solver.calibration_filename()`, shared by
   the writer and the download route; a second copy is how a calibration
   once outlived the camera it was named for.
2. **In water the FITTED FOV is the WATER figure.** The solver measures
   whatever medium the board was actually in. So in water mode
   `hfov_deg_water` takes the measured value and the air figures become
   inverse-Snell derivations, flagged `air_fov_note: DERIVED …`. Running the
   air→water conversion on an already-in-water measurement would apply the
   refraction twice.
3. **The launch loads the AIR file.** The water one is evidence, not a
   replacement, and the solver says so on install rather than implying the
   file goes live.

## 10. ⛔ THE WHOLE PAGE WAS INERT, AND EVERY TEST WAS GREEN

Commit `1e27851` shipped a calibration page whose entire `<script>` block was
a **JavaScript SyntaxError**. Not one line of it ran: no status polling, no
camera list, no buttons. It renders as a page that has simply stopped
updating, which is indistinguishable from a stalled camera thread — and it
was published as "the web app now does the WHOLE calibration job".

**The cause is specific to writing JS inside a Python string.** `PAGE` is a
plain triple-quoted string, so:

| in the Python source | what reaches the browser |
|---|---|
| `\'` | `'` — closing a JS string early |
| `\n` | a real newline — inside a JS string literal |

`sw(\''+c.device+'\')` therefore emitted `sw(''+c.device+'')`: two adjacent
string literals, and the parser gives up on the **whole block**. You need
`\\'` and `\\n` in the source.

**The guard is one command and it would have caught it from the start:**
`node --check` on the extracted script (`test_calibration_page.py`, verified
against the exact 1e27851 defect). A second test asserts every route the page
`fetch`es is a route the server serves — the page's per-poll `catch(e){}`
turns a renamed route into the same "panel stopped updating" symptom.

**The general lesson, and it is the one this project keeps relearning:** a
server that returns 200 for every route proves the server works, not that the
page works. Python tests cannot see inside a Python string. Anything that
ships a language inside another language needs a parser for the inner one.

## 11. Not yet done, and worth doing

- **ChArUco.** OpenCV recommends it over a plain chessboard, and OpenCV 4.6
  on the Pi has the full legacy API (`CharucoBoard_create`,
  `interpolateCornersCharuco`, `calibrateCameraCharuco`). The advantage is
  exactly our failure mode: **partial views are fine** — the board may leave
  the frame — and the 180° pose ambiguity of a symmetric chessboard is gone.
  Kalibr recommends AprilGrid for the same reasons.
- **A screen as the target.** An LCD's planarity is ~**0.05 µm** against a
  printed board's bow, which our own solver reports in *millimetres*. Since
  intrinsics are invariant to square size, the pixel pitch need not even be
  known for FOV. Caveat: moiré, mitigated with large squares and no extreme
  close-ups. **This is the competition-day answer when no printed board
  exists.**
- **Autofocus**: determine whether this unit actually refocuses. If it does
  and cannot be locked, its intrinsics are not constant and every bearing
  built on them drifts.
