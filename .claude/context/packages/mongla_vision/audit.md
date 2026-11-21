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
| B-118 | the anchor rung's box was in the backend's 320x240 grid on a 640x360 frame, so `/lock`, the follower cross-check and the reseed used the wrong place | FIXED, truth test, injection-verified; on Kutum with the Sonix the old release returned a box 300 px off and the fixed one within 4 px, on both backends; BUILT, NEVER FLOWN |
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
| guard blind spot | `test_no_function_is_built_and_uncalled.py` matched methods by name, so a method sharing a common name (`authority`) read as called | FIXED: a method counts only a call, a decorator, a getattr string or a load of a name no field shares; it then found `CheckpointBank.yields` and a `td` property, both deleted |
| NaN velocity | a tile grating the node cannot size gives a NaN height, which replaced the depth path and passed `flow_velocity`'s `<= 0` guard: an ok=True NaN velocity was published and the distance read NaN until the next start | FIXED at both ends, injection-verified |
| stale image yaw | a failed planar fit left the previous interval's image yaw and divergence, which the yaw check paired with this interval's gyro, feeding the time-offset correlation, and the height check with this baro rate | FIXED: absent is NaN and both checks skip it |
| letterbox box | a detection wholly inside the letterbox bars clamped to a zero-area box and was published as a target | FIXED in `hailo` and `hailo_seg`, injection-verified |
| camera log and calibration | the fps-override warning fired on every V4L2 start; a calibration of another aspect ratio was stretched onto the frame | FIXED: the warning fires only for an override, a cross-aspect file is refused (flow already refused it) |
| pixel distance path | `DistanceAccumulator.add` and its helpers integrated raw pixels; nothing on the vehicle called them | DELETED; the distance is the integral of gated velocity |
| scale cross-check | kept every pair since start and walked them all on each check | FIXED: only its window is kept |
| empty snap | `Anchor.snap` with a box on a frame with no keypoints raised instead of returning 0, so the bank's `'empty'` refusal was never reached | FIXED, injection-verified |
| bank scores after eviction | `last_lookup_scores` keyed by reference index survived an eviction, pairing one reference's current yield with another's best for the camera-health verdict | FIXED: eviction and clear reset them |
| written, never read | a scan of every package for attributes assigned and never loaded: in vision, display_node's `yaw_source` parameter and four HUD fields, ByteTrack's copies of its settings, the lock's `_anchor_best`, and five more | DELETED; subscriptions held for teardown are kept |
| ambiguity knob | `solve_pnp` took `ambiguity_max` (and `pnp_node` exposed it live, "to re-solve a bag with a different gate") but never compared it: since the first commit the flip is reported and the consumer gates on off-axis plus spread | DELETED, so no operator tunes a knob that does nothing; the reprojection gate stays |
| plane tilt | `anchor/geometry.plane_geometry` runs only when the bank is given a K, and the lock gives it none, so on the vehicle it never runs (the orphan guards see an import) | recorded, not wired: measured coarse on footage (measured-bars §9), and the shot gate reads the PnP pose |
| water calibration | CameraInfo carries the air lens and every consumer applies the flat port itself; a calibration fitted with the board in water already contains the port, and the binding or an explicit `calibration:=` would have published it, refracting twice (every range off by n) | FIXED: both refuse a non-air file, test; latent, no water file is installed |
| `switch_camera` | paused every other detector before finding the requested one was not running, so a mistyped name left no detector live | FIXED: the target is checked first, test |
| allowlist on the YOLO backend | an allowlist naming no class the model emits returned `[]` every frame, with the node looking healthy; the Hailo backend already refused to narrow | FIXED: one rule, `detector.allow_ids_for`, behavioural test over both backends |
| stale odometry | the lock stored the odometry's arrival time and never read it, so loop closure would search and enrol at a position however old; the localiser publishes on a timer, so even arrival time could not see a frozen position | FIXED: closure refuses odometry whose input stamp has not moved for 0.5 s (five periods; worst gap 105.76 ms on Kutum under the vision graph, measured-bars section 91), test |

## Second pass, 2026-11-19

A second line-by-line read of every file in the package, by a separate reader,
found 23 things. Each was checked against the code before it was acted on.

| | What | State |
|---|---|---|
| F1 | `loop_closure:=true` raised at setup on every camera (`self._camera`, then a qos helper that does not exist) | FIXED (B-122); `test_lock_node_stands_up.py` stands the node up with it on |
| F2 | the anchor pose's object points used the ROI's centre in ROI pixels against full-frame points, shifting every `pnp_node` range by the ROI offset | FIXED (B-122); offset-ROI truth test |
| F3, F11 | the composed detector, the bare node and three launches each stated their own `conf` (0.35 or 0.45) and NMS floor; the bare node filtered to `person`, a COCO class no competition model emits | FIXED: `detector_params.CONF` and `NMS_FLOOR`, read everywhere; `classes` defaults to every class |
| F4 | `vision_dual`'s `flow:=true` started a flow node with no calibration, which can only refuse | FIXED: `dwn_calibration` argument, empty by default (the source there is a webcam or a clip, not the Sonix) |
| F5-F7 | launch docstrings said the opposite of what the files do (downward video, the calibration wiring) | FIXED |
| F8, F12 | the lock declares the XFeat HEF off and prefers the ONNX | BY DESIGN, comment rewritten: a standalone lock process opening the chip races the detector process for it; only the composed Pi launch, which shares one VDevice, turns it on |
| F9 | a refused live parameter could leave the authority pair half-applied | FIXED: the whole call is validated before anything is applied |
| F10 | a camera that failed to open left its detector's worker thread and network groups running | FIXED: `DetectorNode.destroy_node`; `test_detector_node_stops.py`, injection-verified |
| F13 | a generic `lock_X` override beat the per-camera `fwd_lock_X` | FIXED: per-camera applied last; a real rclpy node reads the winner, injection-verified |
| F14 | the lock's ladder thread started before the rung counters existed | FIXED |
| F15 | the XFeat top-k restated beside both backends' own defaults | FIXED: the backends' defaults are used |
| F16 | the lock's `pool_depth_m` "unset" was 0.0 | FIXED: NaN, as in the flow node |
| F17 | an unmeasured tool offset was returned as a zero correction | FIXED: `pixel_offset` refuses it; the camera is aimed and the warning stands |
| F18 | repeated widths in `target_geometry.yaml` | KEPT for props that only share a handbook size; `shaw_fish` aliases `shark`, the one case that is the same board |
| F19, F20 | `vision_dual` documented a `D` viewer key that was removed and simulator topics that no longer exist | FIXED |
| F21 | a non-zero `fps` also caps publishing | NOT A DEFECT as shipped; the default is 0 |
| F22 | the anchor bank loaded with pickling on, so a bank file from off the vehicle could run code | FIXED: bank version 3 stores text, loads with pickling off and closes the archive; a crafted file is refused without running (injection-verified) |
| F23 | `CheckpointBank.reference_image` had no caller, and with it went `Anchor.reference_image` and the full frame copy every reference kept for it | DELETED |

## Second read, 2026-11-19

A second line-by-line read of every file in the package, by a separate reader,
then each finding checked against the code before anything changed.

| | What | State |
|---|---|---|
| B-122 | `loop_closure:=true` raised in setup on every camera (`self._camera`, then a qos helper that does not exist), so closure was built, tested and unreachable | FIXED; `test_lock_node_stands_up.py` stands the node up with it on and walks a closure to the fix message; BUILT, NEVER FLOWN |
| B-122 | the anchor pose's object points used the ROI's own centre and width against full-frame reference points, shifting the object plane by the ROI offset | FIXED: `pose.patch_object_points`, offset-ROI test; `target_pose` (tests only) deleted |
| detector defaults | the composed detector and `ros2 run` started with different `conf` (0.45 against 0.35) and classes (`''` against a COCO `person` that filters every competition model to nothing); the NMS "is it the default" check restated 0.05 | FIXED: `CONF` and `NMS_FLOOR` in `detector_params.py`, read by the node, the composed process and every launch |
| dual-launch flow | `vision_dual flow:=true` started a flow node with no calibration, which can only refuse | FIXED: `dwn_calibration` argument, empty by default (a webcam or clip is not the vehicle's lens) |
| docstrings | `video.launch` said forward only; `vision_pi` said no launch passes the calibration; `vision_dual` named a `D` viewer key and simulator topics that do not exist | FIXED |
| anchor backend | a standalone lock looks for the ONNX before the HEF | by design, comment corrected: a separate lock process opening the chip races the detector process for it; the composed Pi launch sets the HEF. The fallback warning quoted the old chip-only 1.43 ms; now 11.4 ms a frame (measured-bars 92) |
| live parameters | an authority pair applied before a later parameter in the same call was refused | FIXED: the whole call is validated first |
| detector teardown | a composed camera that failed to open destroyed its detector, whose worker kept running and whose models stayed configured on the chip | FIXED: `destroy_node` stops the worker and closes every loaded model; `test_detector_node_stops.py`, injection-verified |
| lock overrides | a generic `lock_X` beat a per-camera `fwd_lock_X`, because rclpy keeps the last override and they were built in sorted order | FIXED, injection-verified (latent: no shipped launch collided) |
| startup race | the lock's ladder thread started before its rung counters existed | FIXED |
| `pool_depth_m` | the lock's unset value was 0.0, the flow node's NaN | FIXED: NaN; the `> 0` guard already refused both |
| tool offsets | `pixel_offset` returned a zero correction for an unmeasured tool, against its own docstring | FIXED: None, so the stack aims the camera and warns, as it said |
| prop widths | the geometry table claims one copy of each number, and repeats 0.305 and 0.6096 | the repeats are different props sharing a handbook size and stay separate; the one true duplicate (`shaw_fish`, the same board as `shark`) is now an alias |
| bank file | the bank loader unpickled an operator-given file and never closed it | FIXED: labels stored as text, read with pickling off inside `with`; `SAVE_VERSION` 3 refuses a version-2 bank with the reason; injection-verified |
| uncalled | `CheckpointBank.reference_image`, then `Anchor.reference_image` and the full-frame copy each reference kept for it | DELETED |
| `top_k` | the lock restated 1024 beside both backends' defaults | FIXED: the defaults are used |
| `fps` | a non-zero `fps:=` on `vision.launch` also caps publishing | not a defect at the default (0); noted |

**Model sidecars, 2026-11-20.** `models/yolov11n_seg.yaml` was a byte-for-byte copy of
`yolov8n_seg.yaml` (the 80 COCO classes), named by nothing, with no model on the dev box
or Kutum: deleted. A stock `yolov11n_seg` HEF dropped in later takes a copy of
`yolov8n_seg.yaml` under its own stem. `octagon` and `yolov8n_seg` also have no model on
either machine; they stay, because missions and the launch name them and a model is
dropped in on the day.

## Live run on Kutum, 2026-11-19

The whole of `vision_pi.launch.py`, on the release after the second read,
with both real cameras and no board (`paused:=false`; the launch default
starts both detectors paused for the mission to resume). Every output topic
exists (50, once discovery has had six seconds). Rates, measured while 22
rate-checkers loaded the Pi, so low:

| | camera | detections | lock | tracks |
|---|---|---|---|---|
| forward, Fantech in evening light | 11.4 fps | 15.0 Hz | 15.0 Hz | 15.0 Hz |
| downward, Sonix | 85 fps | 31.4 Hz (56.8 with 2 checkers) | 43.0 Hz | 31.5 Hz |

Silent, each for a reason that is not a defect: `velocity` (flow refuses
without `pool_depth_m`, and says so as an ERROR), `floor_height` (needs the
board's barometer), `target_pose` and `correspondences` (need a prop the model
knows in view; the bench has none). Two defects, both fixed (B-123): a Ctrl-C
interrupted every node's teardown, inside the Hailo's release in the
detector, and every launch printed a tracker clamp warning its own defaults
caused. The dead-code sweep finds one module with no live importer,
`trigger_stamps.py`, which waits on the board's camera trigger (upstream ask
`pr-w`) and says so where the guard lists it.

**Re-measured 2026-11-20**, release `00f54f90`, one subscriber process for every
`/mongla` topic (best effort, 15 s) instead of 22 checkers, no board:

| `paused:=false` | camera | detections | tracks | lock |
|---|---|---|---|---|
| forward, Fantech, daylight | 27-29 fps, 0 dropped | 25.5 Hz | 25.4 Hz | 30.0 Hz |
| downward, Sonix | 99-104 fps, 0 dropped | 25.8 Hz | 25.9 Hz | 99.6 Hz |

Paused (the default), the downward camera runs at 162 fps; with both detectors
inferring it falls to about 100, and the pair gets about 51 inferences a second
between them. That is the chip budget of measured-bars 95 (two 3-context models
ask 128 % of the Hailo), and it is what the YOLO-family A/B is for. Flow,
given `medium:=air pool_depth_m:=1.0`, passes the pool-depth refusal and stops
at the next one, "no fresh depth": the height above the floor needs the board's
barometer, so flow velocity on hardware waits for the board (po7.7). SIGINT
stopped the whole launch inside 10 s; only `hailort_service`, the system
daemon, remained.

## Cameras

The Fantech (colour, USB 2) is the forward camera and the Sonix global-shutter mono is the
downward camera, for detection and flow. Measured on Kutum the evening of 2026-11-18: the
Fantech delivers 10.0 fps at mean grey 28, 15 at 125 and 30 in afternoon light in every
mode, and ignores manual exposure; the Sonix delivers 210.99, 120.72 and 60.42 fps in the
same light. Colour is what the forward detector needs; rate is what flow needs. The
record is `measured-bars.md`, "Forward camera rate". **Bench tests of vision use the Sonix**,
because the Fantech's rate follows the room's light and confounds any rate or ladder
measurement.

**The plan from 2026-11-20 (owner):** the forward camera becomes a second
global-shutter camera, 120 fps class, when it arrives; the Fantech is the
interim forward camera only. Plan and measure for two global-shutter cameras
until that changes. At 120 fps each, the current 3-context models would ask
about 350 % of the chip and single-context v8-family models about 58 %
(measured-bars 95), so the detector family decides whether both cameras can
detect at their own rate.

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
4. **Watch the anchor rung on a detected prop.** B-118's grid is verified on Kutum with
   the Sonix (a box comes back within 4 px); what is not yet seen is the rung taking over
   from a real detection on a prop the model knows, and its box agreeing with the
   follower's.
5. **Leave these off until measured:** loop closure (needs a working barometer and the
   floor height, never fired), lane lines (a path marker reads as a lane), `range_crop`
   (halves the field of view; its gain was measured offline only).
6. **`identity.MIN_OVERLAP` (0.55) and `SIDE_DEADBAND` (0.10)** decide which side of a gate
   a symbol is on. Both are declared from the failure they prevent, not measured; a
   contact sheet of gate frames settles them.
7. **Flow subtracts no rotation on the vehicle.** `gyro_gain_x/y` default to 0 by recorded
   decision: the coupling was measured about the lens, not the pivot the hull turns about,
   and 0 is the only gain that cannot double the error. A pitch or roll during a move is
   read as translation. A high-rotation test at the real pivot sets the gains.
8. **The camera-gyro time offset is coarse when the hull is slow.** Its sub-millisecond
   figure was measured with a 50 Hz gyro and 25 Hz image samples. The node feeds it one
   pair per flow interval: on a known 30 ms offset, intervals of 0.15 s gave 0.7 ms,
   0.3 s gave 4.5 ms, 0.5 s about 12 ms, against a 6 ms tolerance. A full-rate gyro helps
   at 0.3 s (1.3 ms) and not at 0.5 s, where the image side's sampling limits it. The
   offset starts from the measured -13.4 ms (flight mode, measured-bars 108) and is bounded at 0.15 s. The half-exposure
   term is 0 on the vehicle, because nothing sets flow's `exposure_us`. It is microseconds
   now, as at the camera; it was 100 us units under the same name, and the webcam source
   took microseconds as 100 us units too (a 4 ms ask gave a 400 ms shutter). Fixed.

What runs today: both cameras, both detectors, the ladder, flow, the operator's views, on
the bench. What is built and never flown: every rung's behaviour in water, flow's velocity
against a real floor, loop closure. What is blocked: anything needing the board in the
loop on Kutum, or the rig that gives a ground truth to measure the ladder against.

## Every file

The read column says whether the file was read line by line in this audit. All 106 were.

| File | Lines | Reached from | Read | Finding |
|---|---|---|---|---|
| `__init__.py` | 35 | vehicle + other packages | yes | works; docstring lists the real exports |
| `_text_engine.py` | 149 | vehicle | yes | read; sound |
| `anchor/__init__.py` | 0 | vehicle + other packages | yes | empty package marker |
| `anchor/anchor.py` | 314 | vehicle | yes | FIXED: a box on a featureless frame raised instead of snapping nothing |
| `anchor/bank.py` | 576 | vehicle | yes | FIXED: eviction and clear left per-index scores naming other references |
| `anchor/bank_lookup.py` | 287 | vehicle | yes | FIXED: an empty search left the previous scores; identity bar docstring quoted the retired 60 |
| `anchor/bank_store.py` | 114 | vehicle | yes | FIXED: load skipped the scales; a preloaded bank covered no size and its first eviction raised. Bank version 2: a bank of pre-upstream descriptors is refused (B-121) |
| `anchor/geometry.py` | 223 | vehicle | yes | works; never runs on the vehicle (the lock passes no K), recorded |
| `anchor/health.py` | 186 | vehicle | yes | works; thresholds declared, reported only |
| `anchor/loop_closure.py` | 291 | vehicle | yes | FIXED: offset centre was a 320x240 constant; required argument now |
| `anchor/place_signature.py` | 90 | vehicle | yes | NEW 2026-11-19: the bank's signature is VLAD-16 over its descriptors, not the 64-D mean (Recall@1 forward 52.5 to 65.8 %, downward 47.9 to 77.1 %, measured-bars 20.3); `config/vlad16.npz` is its vocabulary |
| `anchor/pose.py` | 368 | other packages | yes | FIXED: the ambiguity gate it accepted was never compared; deleted |
| `anchor/xfeat_hailo.py` | 224 | vehicle | yes | CHANGED: its copy of the descriptor sampling replaced by the shared one (outputs equal within 9e-8). FIXED: the reliability head was read and dropped; it now ranks the top-k (+25.9 % correct matches on the chip). Semi-dense removed: it fit the true view change half as often (B-121) |
| `anchor/xfeat_onnx.py` | 286 | vehicle | yes | CHANGED: keypoints and descriptors in one function both backends call; unread logger argument removed. FIXED: not upstream XFeat (ranking without reliability, bilinear raw descriptors, edge keypoints kept); now upstream's exactly: +38 % correct matches on warped frames, +8 % on real pairs (B-121, measured-bars section 92) |
| `calibration/__init__.py` | 26 | vehicle + other packages | yes | works |
| `calibration/binding.py` | 79 | vehicle | yes | works |
| `calibration/calibrate_node.py` | 401 | CLI tool + other packages | yes | read; FIXED applying a water fit said the stack would use it |
| `calibration/guide_capture.py` | 377 | CLI tool + other packages | yes | read; FIXED a stale refit error and pose suggestion stayed on the panel, and a water capture showed its FOV as the air one |
| `calibration/guide_files.py` | 290 | CLI tool + other packages | yes | read; FIXED the camera list showed a water fit as the camera's lens |
| `calibration/solver.py` | 766 | CLI tool + other packages | yes | read; FIXED mixed-resolution captures fitted one K; a bad --identity is reported. The published K is air only: binding and camera_node refuse a water fit, which would be refracted twice |
| `camera_node.py` | 931 | vehicle | yes | FIXED: the fps-override warning fired on every V4L2 start; a calibration of another aspect ratio is refused, not stretched |
| `cameras/__init__.py` | 6 | vehicle + other packages | yes | works |
| `cameras/camera.py` | 99 | vehicle + other packages | yes | works; docstring corrected (is_healthy drives the reopen watchdog), and says what each source stamps: the mailbox the kernel's capture time, the webcam source the read's return |
| `cameras/discover.py` | 43 | vehicle | yes | works |
| `cameras/reopen.py` | 82 | vehicle | yes | works |
| `cameras/ros_topic.py` | 135 | vehicle + other packages | yes | FIXED: wall stamp taken at read, monotonic at arrival |
| `cameras/v4l2_mailbox.py` | 791 | vehicle + other packages | yes | works; ioctl struct sizes checked; buffer-count text said four, it is sixteen |
| `cameras/video_file.py` | 248 | vehicle + other packages | yes | FIXED: stamped before the pacing sleep (333 ms early at 0.1x) |
| `cameras/webcam.py` | 220 | vehicle + other packages | yes | FIXED: exposure_us written raw into 100 us units (4 ms asked, 400 ms set) |
| `config.py` | 186 | vehicle + other packages | yes | works; the Fantech is forward for colour, the Sonix downward for rate (measured 2026-11-18) |
| `continuity.py` | 280 | none | yes | read; tools/studies only (written reason in the register) |
| `detection/__init__.py` | 17 | vehicle + other packages | yes | works |
| `detection/contours.py` | 159 | vehicle | yes | works |
| `detection/detector.py` | 123 | vehicle + other packages | yes | CHANGED: holds allow_ids_for, the one allowlist rule |
| `detection/factory.py` | 82 | vehicle | yes | works; the extension list is the one the resolver reads |
| `detection/gpu.py` | 86 | vehicle + other packages | yes | works (dev box only) |
| `detection/hailo.py` | 760 | vehicle | yes | CHANGED: allowlist rule moved to detector.allow_ids_for |
| `detection/hailo_device.py` | 249 | vehicle + other packages | yes | works |
| `detection/hailo_seg.py` | 253 | vehicle | yes | FIXED: decoded at conf so the association band was empty; an edge mask was squashed not cropped; nms_iou refused live. No seg HEF on Kutum: built, not run on the chip with our models |
| `detection/hef_file.py` | 102 | vehicle | yes | works |
| `detection/messages.py` | 224 | vehicle | yes | works; class ids derived from labels, so a model swap cannot merge classes |
| `detection/rangecrop.py` | 138 | vehicle | yes | works; off by default (halves the field of view; gain measured offline only) |
| `detection/seg_decode.py` | 306 | vehicle | yes | works |
| `detection/yolo.py` | 317 | vehicle | yes | FIXED: its own copy of the extension order; an all-unknown allowlist returned [] every frame (one rule now, detector.allow_ids_for) |
| `detector_dual_node.py` | 478 | vehicle | yes | works; the vehicle process (both cameras, detectors, locks, flow) |
| `detector_frames.py` | 182 | vehicle | yes | works |
| `detector_node.py` | 646 | vehicle | yes | works; stray comments cleaned |
| `detector_params.py` | 83 | vehicle | yes | FIXED: a malformed model_conf pair was skipped and the live set reported success; now refused by name |
| `detector_publish.py` | 181 | vehicle | yes | CHANGED: debug image signature trimmed to what it uses |
| `detector_setup.py` | 490 | vehicle | yes | FIXED: model_conf refused whole when malformed; a live set is all or nothing (every refusal is decided before anything is applied) |
| `draw.py` | 44 | vehicle | yes | CHANGED: legacy aliases and a swallowing **kwargs removed; one entry point |
| `draw_strip.py` | 437 | none | yes | read; tools/site only (written reason in the register); FIXED absent values drew as 0.0; unused strip-height export removed |
| `draw_video.py` | 315 | vehicle | yes | CHANGED: depth inset and depth borders removed |
| `draw_widgets.py` | 404 | vehicle | yes | read; sound |
| `factory.py` | 172 | vehicle + other packages | yes | read; sound |
| `flow/__init__.py` | 0 | vehicle + other packages | yes | empty |
| `flow/flow_checks.py` | 275 | vehicle | yes | FIXED: a failed planar fit left the last image yaw, paired with this gyro for the time offset |
| `flow/flow_constants.py` | 178 | vehicle | yes | works; gyro de-rotation gain 0 by recorded decision (measured about the lens, not the hull pivot) |
| `flow/flow_features.py` | 210 | vehicle | yes | works |
| `flow/flow_floor.py` | 132 | vehicle | yes | FIXED: a grating it cannot size gave a NaN height that replaced the depth path |
| `flow/flow_frame.py` | 422 | vehicle | yes | FIXED: no heading mixed against an invented axis; reset races the worker (one logged error, not a wrong number) |
| `flow/flow_math.py` | 541 | vehicle | yes | FIXED: the pixel distance path nothing ran is deleted; the refraction model is imported from optics only |
| `flow/flow_node.py` | 435 | vehicle | yes | CHANGED: image yaw and scale rate start NaN, not 0.0; floor depth with no range warns |
| `flow/flow_publish.py` | 142 | vehicle | yes | works; half-exposure term is 0 on the vehicle (exposure_us never set), now said |
| `flow/flow_setup.py` | 442 | vehicle | yes | FIXED: an unstamped frame timed on the monotonic clock; exposure_us is microseconds |
| `flow/flow_timing.py` | 449 | vehicle + other packages | yes | works; time-offset accuracy holds only for short flow intervals (see competition items) |
| `flow/flow_velocity.py` | 210 | vehicle | yes | FIXED: a NaN height, focal length or dt published an ok=True NaN velocity |
| `flow/scale_check.py` | 110 | vehicle | yes | FIXED: kept every pair since start; now only its window |
| `identity.py` | 199 | other packages | yes | works; MIN_OVERLAP 0.55 declared, not measured; retired class names removed from text |
| `image_msg.py` | 102 | vehicle + other packages | yes | CHANGED: 32FC1 and the byte-order path removed with the depth map |
| `live_params.py` | 31 | vehicle + other packages | yes | works |
| `lock_anchor.py` | 253 | vehicle | yes | FIXED B-118: anchor box scaled from backend 320x240 to camera pixels |
| `lock_frames.py` | 245 | vehicle | yes | works; act_conf 0.45 rests on a retired model (re-derive per flown model) |
| `lock_inputs.py` | 246 | vehicle | yes | FIXED: the closure offset measured at a 320x240 constant; now the bank grid centre. FIXED: closure refuses odometry whose input stamp stopped moving (measured-bars section 91) |
| `lock_node.py` | 349 | vehicle | yes | works; docstring said the score was trust x decay, it is the authority (#78) |
| `lock_publish.py` | 244 | vehicle | yes | FIXED: field-of-view bearing used backend K on full-frame pixels; the warning now prints the bearing, and `_vis_state`, read only by a test, is gone (B-119) |
| `lock_setup.py` | 408 | vehicle | yes | FIXED: releasing target_class stayed on the pinned class (the filter is latched and was dropped while pinned). `anchor_semi_dense` removed with the half-port (B-121) |
| `lock_wiring.py` | 311 | vehicle | yes | works; holds the last single-class filter |
| `optics.py` | 276 | vehicle + other packages | yes | works; a redundant import removed |
| `preflight.py` | 133 | vehicle + other packages | yes | works; stale reference to a deleted function removed |
| `qos.py` | 81 | vehicle + other packages | yes | works |
| `seeing.py` | 92 | vehicle | yes | works |
| `stamps.py` | 66 | vehicle + other packages | yes | works |
| `target_geometry.py` | 317 | vehicle + other packages | yes | FIXED: an override width that is not a number was skipped without a word; now reported like an implausible one |
| `tool_geometry.py` | 104 | other packages | yes | works; torpedo, dropper and grabber offsets UNMEASURED (warns at fire) |
| `tracker_node.py` | 417 | vehicle | yes | FIXED: box-size smoothing was per frame (0.1 s at 30 Hz, 0.4 s at 10 fps), now on capture time; dead camera_info subscription removed; coasted-score comment corrected |
| `tracking/__init__.py` | 9 | vehicle | yes | works |
| `tracking/bytetrack.py` | 159 | vehicle | yes | FIXED: a coasting track reported class_id 0 (fallback when trackers will not import; Kutum runs OC-SORT) |
| `tracking/confidence.py` | 132 | vehicle | yes | works for the Kalman (nsa_factor); FIXED: authority() was never called, deleted |
| `tracking/follower.py` | 267 | vehicle | yes | FIXED: drop() defined twice, the first shadowed |
| `tracking/kalman.py` | 259 | vehicle | yes | works; Q re-discretised per interval, expiry in seconds |
| `tracking/lock_state.py` | 335 | vehicle | yes | works; the ladder core (strict priority, decay from the last real detection) |
| `tracking/roboflow_tracker.py` | 267 | vehicle | yes | works; two values stored and never read removed |
| `tracking/tracker.py` | 100 | vehicle | yes | works |
| `tracking/visibility.py` | 206 | vehicle | yes | works; advisory only (no demand is scaled); SLACK flag deleted. FIXED: it went silent when the anchor bank failed to load; K now stays on the camera grid without a bank |
| `trigger_stamps.py` | 130 | none | yes | waits on the board camera trigger (upstream); written reason in the register |
| `underwater.py` | 119 | CLI tool | yes | works (characterisation only, no verdict); stale prediction table removed |
| `utils/__init__.py` | 7 | vehicle | yes | works |
| `utils/check_pipeline.py` | 264 | CLI tool | yes | read; sound (detector node names match the launch) |
| `utils/check_thrust.py` | 116 | CLI tool | yes | read; the outcome table is a copy of the verb's, held equal by a test |
| `utils/check_tracker.py` | 167 | CLI tool | yes | read; uses the shared QoS and class decoder |
| `utils/display_draw.py` | 200 | vehicle | yes | read; FIXED absent depth, distance and height printed as numbers; now -- |
| `utils/display_keys.py` | 115 | vehicle | yes | CHANGED: the D key removed |
| `utils/display_node.py` | 644 | vehicle | yes | read; CHANGED: depth map, vis_range and the D key removed; FIXED video keys drove the start camera after a switch; unread track count removed |
| `utils/switch_camera.py` | 93 | CLI tool | yes | works; false SRAM claim replaced with the measured both-live rates |
| `utils/water_check.py` | 111 | CLI tool | yes | works; orphan comment removed |

"Reached from" is the import closure of what each launch and entry point starts:
*vehicle* is `vision_pi.launch.py`, the launch `bringup` includes; *CLI tool* is a
`ros2 run` entry point; *none* is a module with a written reason in
`test_no_capability_is_built_and_unreachable.py`.

---

Related: [the package page](README.md) · [Bug register](../../BUGS.md) ·
[Measured bars](../../measured-bars.md)
