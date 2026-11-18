# mongla_vision audit — every file, and what it gives the vehicle

Read on 2026-11-18, on the branch that removed the depth chain and the side deck files.
Method: a line-by-line read of each file, sweeps across all packages for each defect class
the read found, every suite on the dev box, and the bench rig on Kutum. Each state below is
one of the three this repository uses: **RUNS TODAY**, **BUILT, NEVER FLOWN**, **BLOCKED**.
No part of this package has been in water on the current platform.

## What vision gives Mongla, in one paragraph

Two cameras, two detectors on the Hailo-8, and one lock ladder per camera that keeps a
target position through the detector's gaps (detection, then a weak box where the lock
already is, then optical flow in the last box, then an XFeat match against stored
references). The control loop steers on the ladder's box and scales by its authority,
which decays from the last real detection, so a carried box can never pass for a fresh
one. The downward camera also measures the vehicle's own velocity from the floor, which is
the only velocity sensor on the vehicle. Everything else in the package serves those
three jobs (calibration, refraction, timing, the operator's views) or is a tool for the
bench and the studies.

## What was deleted, and whether it was helping

| Removed | What it claimed | What the measurement said | Did the ladder need it? |
|---|---|---|---|
| `detection/preprocess.py` (CLAHE) | better detection in murky water | never positive in 17 configurations, four props, three venues; on the gate 30.4 % to 1.2 % | no; the models were trained on unprocessed frames |
| `detection/profiles.py` (`vision:=` profiles) | per-venue settings | `murky` equalled `clear`, `close` equalled `fast`; its confidence was never applied (the launch always passed its own) | no; its one live effect, `range_crop`, is now a plain switch |
| `tracking/presence.py` | track-before-detect on weak boxes | +2.9 points of recall on top of a tracker that already adds 22 (measured-bars section 88) | no; the HELD rung keeps a lock on weak boxes where the lock already is |
| `tracking/reid.py`, `tracking/cascade.py` | fewer identity switches | 20 switches in 592 s on the bins, none on the gate, and no consumer pays for them: align re-locks on the next real detection | no; never wired |
| `confidence_calibration` | calibrated scores | its gain was declared, never measured | no |
| `time_to_contact` | when the target will be reached | p90 jitter 27.2 s | no |
| `approach` | back off when too close | bands from a retired model, and nothing acted on BACK_OFF | no, but the problem it named is real: close range is carried today by the follow and anchor rungs, and B-118 below was in exactly that path. `tools/studies/approach_band.py` stays for re-deriving it |
| `depth/depth_estimation_node.py` | a relative depth map (Depth Anything V2) | off by default, launched only by the generic launch, and its one consumer field in the manager was never read; the site stills showed an invented 0.42 | no; the model is in `~/attic/2026-11-18/depth_models/` |
| `vision_node.py` | a third camera-and-detector pipeline | CLI only, stamped its debug image with `now()`; `vision_check` and `vision.launch.py` cover it | no |
| `ladder_config.py`, `~/.mongla/ladder.yaml`, `~/.mongla/loop_closure.yaml` | deck-editable ladder bars | neither file existed on the vehicle or the dev box; "equal to the default means unset" made a deliberate default unreachable | the bars are kept, as node parameters under `nodes:` in `bringup.yaml` |

**Key point:** nothing removed was on the path the vehicle steers by. Every item either
measured as no help, was never wired, or duplicated something that runs.

## Defects this read found

| | What | State |
|---|---|---|
| B-117 | every loop closure was refused as a self-closure: the guards read the newest place, not the matched one | FIXED, truth test, injection-verified; BUILT, NEVER FLOWN |
| B-118 | the anchor rung's box was in the backend's 320x240 grid on a 640x360 frame, so `/lock`, the follower cross-check and the reseed used the wrong place | FIXED, truth test, injection-verified; BUILT, NEVER FLOWN |
| pose ambiguity | two perfectly fitting IPPE branches reported ambiguity 0.0 ("certain"), and `pnp_node` wrote an unknown ambiguity or reprojection as 0.0 | FIXED at both ends; unknown is now the value that refuses |
| visibility bearing | the field-of-view warning computed its angle with the backend K on full-frame pixels (the state itself was right) | FIXED, test |
| flow distance | a distance started with no heading took an invented 0 deg axis and later mixed a real heading against it | FIXED: body frame for the whole run, and said |
| floor sigma | a floor depth with no range gave the velocity covariance zero floor uncertainty, silently | the node warns and `bringup_check` grades it WARN; no spread is invented |
| `VisionState.close` | missed the lock and pose subscriptions, leaking two per camera switch | FIXED, injection-verified |
| `tools/bench/dataset_survey.py` | imported functions deleted months ago, so it died on start | FIXED |
| `Follower.drop` | defined twice, the first silently replaced | FIXED; no other duplicate method in `src/` |
| absent values | `draw_strip` drew a missing depth, yaw or battery as 0.0 | FIXED: `--` |
| track size | `tracker_node` smoothed box size per frame: 0.1 s at 30 Hz, 0.4 s on the forward camera at 10 fps; the frame-count defect B-114 removed from the coast windows | FIXED: capture time, unchanged at 30 Hz; test with the old form failing |
| uncalled code | `ConfidenceModel.authority()` and the detectors' `class_names()` had no caller; the tracker's `camera_info` subscription fed fields nothing read | DELETED |
| guard blind spot | `test_no_function_is_built_and_uncalled.py` matches methods by name, so a method sharing a common name (`authority`) reads as called | open; a stricter sweep found only the two above |

## Cameras

The Fantech (colour, USB 2) is the forward camera and the Sonix global-shutter mono is the
downward camera, for detection and flow. Measured on Kutum the evening of 2026-11-18: the
Fantech delivers 10.0 fps at mean grey 28, 15 at 125 and 30 in afternoon light in every
mode, and ignores manual exposure; the Sonix delivers 210.99, 120.72 and 60.42 fps in the
same light. Colour is what the forward detector needs; rate is what flow needs. The
record is `measured-bars.md`, "Forward camera rate". **Bench tests of vision use the Sonix**,
because the Fantech's rate follows the room's light and confounds any rate or ladder
measurement.

## From the competition's point of view

What has to happen before a run counts, in the order it bites:

1. **Re-derive `act_conf` for the model that flies.** The acting bar ships at 0.45, below
   the 0.60 knee measured on one model, one class and one venue (B-59), and that model is
   retired. Models change on the day, so this is a short sweep on archive footage per
   model, not a constant.
2. **Tape the tool offsets.** `tool_geometry.yaml` has the torpedo, dropper and grabber at
   zero with `unmeasured: true`; a fire through them warns. A launcher 10 cm off the
   camera misses a small target at any range.
3. **Give flow its floor.** A course file with `pool: depth_m` and its range, or
   `tile_m` once the tiles are measured. Without a range the velocity's covariance treats
   the typed depth as exact, and the gate now says so.
4. **Check the anchor rung on Kutum with the Sonix.** B-118 changed where its box lands;
   the bench check is a box held still in front of the camera, with the anchor rung's
   centre compared against the detection's.
5. **Leave these off until measured:** loop closure (needs a working barometer and the
   floor height, never fired), lane lines (a path marker reads as a lane), `range_crop`
   (halves the field of view; its gain was measured offline only).
6. **`identity.MIN_OVERLAP` (0.55) and `SIDE_DEADBAND` (0.10)** decide which side of a gate
   a symbol is on. Both are declared from the failure they prevent, not measured; a
   contact sheet of gate frames settles them.

What runs today: both cameras, both detectors, the ladder, flow, the operator's views, on
the bench. What is built and never flown: every rung's behaviour in water, flow's velocity
against a real floor, loop closure. What is blocked: anything needing the board in the
loop on Kutum, or the rig that gives a ground truth to measure the ladder against.

## Every file

The read column says whether the file was read line by line in this audit; "pending"
files pass their tests and the sweeps but have not been read yet.

| File | Lines | Reached from | Read | Finding |
|---|---|---|---|---|
| `__init__.py` | 35 | vehicle + other packages | yes | works; docstring lists the real exports |
| `_text_engine.py` | 149 | vehicle | pending | tests pass; line read pending |
| `anchor/__init__.py` | 0 | vehicle + other packages | pending | tests pass; line read pending |
| `anchor/anchor.py` | 311 | vehicle | pending | tests pass; line read pending |
| `anchor/bank.py` | 576 | vehicle | pending | tests pass; line read pending |
| `anchor/bank_lookup.py` | 285 | vehicle | pending | tests pass; line read pending |
| `anchor/bank_store.py` | 107 | vehicle | yes | works; refuses a bank from another resolution |
| `anchor/geometry.py` | 223 | vehicle | pending | tests pass; line read pending |
| `anchor/health.py` | 186 | vehicle | pending | tests pass; line read pending |
| `anchor/loop_closure.py` | 291 | vehicle | yes | FIXED B-117: guards now read the matched place; per-place age, excursion and sigma |
| `anchor/pose.py` | 375 | other packages | pending | FIXED: ambiguity reported 0.0 ("certain") when both IPPE branches fit perfectly |
| `anchor/xfeat_hailo.py` | 248 | vehicle | pending | tests pass; line read pending |
| `anchor/xfeat_onnx.py` | 276 | vehicle | pending | works; docstring said INPUT pixels, they are BACKEND pixels |
| `calibration/__init__.py` | 26 | vehicle + other packages | yes | works |
| `calibration/binding.py` | 79 | vehicle | yes | works |
| `calibration/calibrate_node.py` | 401 | CLI tool + other packages | pending | tests pass; line read pending |
| `calibration/guide_capture.py` | 377 | CLI tool + other packages | pending | tests pass; line read pending |
| `calibration/guide_files.py` | 290 | CLI tool + other packages | pending | tests pass; line read pending |
| `calibration/solver.py` | 766 | CLI tool + other packages | pending | tests pass; line read pending |
| `camera_node.py` | 917 | vehicle | pending | tests pass; line read pending |
| `cameras/__init__.py` | 6 | vehicle + other packages | yes | works |
| `cameras/camera.py` | 99 | vehicle + other packages | yes | works; docstring corrected (is_healthy drives the reopen watchdog) |
| `cameras/discover.py` | 43 | vehicle | yes | works; docstring named the wrong consumer, fixed |
| `cameras/reopen.py` | 82 | vehicle | yes | works |
| `cameras/ros_topic.py` | 133 | vehicle + other packages | pending | tests pass; line read pending |
| `cameras/v4l2_mailbox.py` | 794 | vehicle + other packages | pending | tests pass; line read pending |
| `cameras/video_file.py` | 246 | vehicle + other packages | pending | tests pass; line read pending |
| `cameras/webcam.py` | 219 | vehicle + other packages | pending | tests pass; line read pending |
| `config.py` | 186 | vehicle + other packages | yes | works; the Fantech is forward for colour, the Sonix downward for rate (measured 2026-11-18) |
| `continuity.py` | 280 | none | pending | tools/studies only (written reason in the register) |
| `detection/__init__.py` | 17 | vehicle + other packages | yes | works |
| `detection/contours.py` | 159 | vehicle | pending | tests pass; line read pending |
| `detection/detector.py` | 92 | vehicle + other packages | yes | CHANGED: class_names() removed from the interface (never called); stale text fixed |
| `detection/factory.py` | 82 | vehicle | yes | works |
| `detection/gpu.py` | 86 | vehicle + other packages | pending | tests pass; line read pending |
| `detection/hailo.py` | 781 | vehicle | pending | CHANGED: class_names() never called, removed from the interface |
| `detection/hailo_device.py` | 249 | vehicle + other packages | pending | tests pass; line read pending |
| `detection/hailo_seg.py` | 226 | vehicle | pending | tests pass; line read pending |
| `detection/hef_file.py` | 102 | vehicle | yes | works |
| `detection/messages.py` | 224 | vehicle | yes | works; class ids derived from labels, so a model swap cannot merge classes |
| `detection/rangecrop.py` | 138 | vehicle | pending | works; off by default (halves the field of view; offline-only measurement) |
| `detection/seg_decode.py` | 306 | vehicle | pending | tests pass; line read pending |
| `detection/yolo.py` | 337 | vehicle | pending | CHANGED: same |
| `detector_dual_node.py` | 478 | vehicle | yes | works; the vehicle process (both cameras, detectors, locks, flow) |
| `detector_frames.py` | 182 | vehicle | pending | tests pass; line read pending |
| `detector_node.py` | 647 | vehicle | yes | works; stray comments cleaned |
| `detector_params.py` | 80 | vehicle | pending | tests pass; line read pending |
| `detector_publish.py` | 181 | vehicle | yes | CHANGED: debug image signature trimmed to what it uses |
| `detector_setup.py` | 482 | vehicle | pending | tests pass; line read pending |
| `draw.py` | 44 | vehicle | yes | CHANGED: legacy aliases and a swallowing **kwargs removed; one entry point |
| `draw_strip.py` | 437 | none | pending | tools/site only (written reason in the register); FIXED absent values drew as 0.0 |
| `draw_video.py` | 315 | vehicle | yes | CHANGED: depth inset and depth borders removed |
| `draw_widgets.py` | 404 | vehicle | pending | tests pass; line read pending |
| `factory.py` | 172 | vehicle + other packages | pending | tests pass; line read pending |
| `flow/__init__.py` | 0 | vehicle + other packages | pending | tests pass; line read pending |
| `flow/flow_checks.py` | 268 | vehicle | pending | tests pass; line read pending |
| `flow/flow_constants.py` | 178 | vehicle | pending | tests pass; line read pending |
| `flow/flow_features.py` | 210 | vehicle | pending | tests pass; line read pending |
| `flow/flow_floor.py` | 128 | vehicle | yes | works; tile grating and lane lines, both off until the venue is measured |
| `flow/flow_frame.py` | 423 | vehicle | pending | FIXED: same (no heading mixed against an invented axis) |
| `flow/flow_math.py` | 583 | vehicle | pending | tests pass; line read pending |
| `flow/flow_node.py` | 434 | vehicle | pending | CHANGED: warns when the floor depth has no range (covariance treats it as exact) |
| `flow/flow_publish.py` | 140 | vehicle | pending | tests pass; line read pending |
| `flow/flow_setup.py` | 436 | vehicle | pending | FIXED: distance with no heading at start is body-frame for the whole run |
| `flow/flow_timing.py` | 451 | vehicle + other packages | pending | tests pass; line read pending |
| `flow/flow_velocity.py` | 207 | vehicle | pending | tests pass; line read pending |
| `flow/scale_check.py` | 108 | vehicle | yes | works; wired into flow_floor |
| `identity.py` | 199 | other packages | yes | works; MIN_OVERLAP 0.55 declared, not measured; retired class names removed from text |
| `image_msg.py` | 102 | vehicle + other packages | yes | CHANGED: 32FC1 and the byte-order path removed with the depth map |
| `live_params.py` | 31 | vehicle + other packages | yes | works |
| `lock_anchor.py` | 254 | vehicle | yes | FIXED B-118: anchor box scaled from backend 320x240 to camera pixels |
| `lock_frames.py` | 245 | vehicle | yes | works; act_conf 0.45 rests on a retired model (re-derive per flown model) |
| `lock_inputs.py` | 245 | vehicle | yes | FIXED B-117 caller; floor-height age constant moved here |
| `lock_node.py` | 347 | vehicle | pending | tests pass; line read pending |
| `lock_publish.py` | 244 | vehicle | yes | FIXED: field-of-view bearing used backend K on full-frame pixels |
| `lock_setup.py` | 409 | vehicle | pending | CHANGED: ladder bars are plain node parameters (no side deck files) |
| `lock_wiring.py` | 313 | vehicle | pending | CHANGED: same; loop-closure params read plainly |
| `optics.py` | 277 | vehicle + other packages | pending | tests pass; line read pending |
| `preflight.py` | 133 | vehicle + other packages | yes | works; stale reference to a deleted function removed |
| `qos.py` | 81 | vehicle + other packages | yes | works |
| `seeing.py` | 92 | vehicle | yes | works |
| `stamps.py` | 66 | vehicle + other packages | yes | works |
| `target_geometry.py` | 314 | vehicle + other packages | pending | works; per-class widths from the rulebooks, deck-overridable |
| `tool_geometry.py` | 104 | other packages | yes | works; torpedo, dropper and grabber offsets UNMEASURED (warns at fire) |
| `tracker_node.py` | 417 | vehicle | yes | FIXED: box-size smoothing was per frame (0.1 s at 30 Hz, 0.4 s at 10 fps), now on capture time; dead camera_info subscription removed; coasted-score comment corrected |
| `tracking/__init__.py` | 9 | vehicle | yes | works |
| `tracking/bytetrack.py` | 163 | vehicle | pending | tests pass; line read pending |
| `tracking/confidence.py` | 132 | vehicle | yes | works for the Kalman (nsa_factor); FIXED: authority() was never called, deleted |
| `tracking/follower.py` | 267 | vehicle | yes | FIXED: drop() defined twice, the first shadowed |
| `tracking/kalman.py` | 259 | vehicle | yes | works; Q re-discretised per interval, expiry in seconds |
| `tracking/lock_state.py` | 335 | vehicle | yes | works; the ladder core (strict priority, decay from the last real detection) |
| `tracking/roboflow_tracker.py` | 268 | vehicle | pending | works; a coasted box carries score 0.0 and a window in capture seconds |
| `tracking/tracker.py` | 100 | vehicle | yes | works |
| `tracking/visibility.py` | 207 | vehicle | pending | works; advisory only (no demand is scaled); stale re-ID text fixed |
| `trigger_stamps.py` | 130 | none | yes | waits on the board camera trigger (upstream); written reason in the register |
| `underwater.py` | 119 | CLI tool | yes | works (characterisation only, no verdict); stale prediction table removed |
| `utils/__init__.py` | 7 | vehicle | yes | works |
| `utils/check_pipeline.py` | 264 | CLI tool | pending | tests pass; line read pending |
| `utils/check_thrust.py` | 116 | CLI tool | pending | tests pass; line read pending |
| `utils/check_tracker.py` | 167 | CLI tool | pending | tests pass; line read pending |
| `utils/display_draw.py` | 200 | vehicle | pending | tests pass; line read pending |
| `utils/display_keys.py` | 115 | vehicle | yes | CHANGED: the D key removed |
| `utils/display_node.py` | 672 | vehicle | pending | CHANGED: depth map, vis_range and the D key removed |
| `utils/switch_camera.py` | 93 | CLI tool | yes | works; false SRAM claim replaced with the measured both-live rates |
| `utils/water_check.py` | 111 | CLI tool | yes | works; orphan comment removed |

"Reached from" is the import closure of what each launch and entry point starts:
*vehicle* is `vision_pi.launch.py`, the launch `bringup` includes; *CLI tool* is a
`ros2 run` entry point; *none* is a module with a written reason in
`test_no_capability_is_built_and_unreachable.py`.

---

Related: [the package page](README.md) · [Bug register](../../BUGS.md) ·
[Measured bars](../../measured-bars.md)
