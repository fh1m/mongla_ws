# mongla_vision — eyes, and everything that keeps them honest

**[`src/mongla_vision`](../../../../src/mongla_vision)**

The largest package, because seeing underwater is the hard part. Water bends light, absorbs
red first, moves particles through the frame and throws caustics across the floor. A detector
trained in one swimming pool can fail in a different swimming pool.

So this package is not just "run a neural network". It is: get the *newest* frame, know what
the lens really does, detect, keep hold of the target when the detector blinks, and measure
motion from what the floor does between frames.

---

## The path a frame takes

```
camera ──► detector ──► tracker ──► lock ──► the control loop
   │                                  │
   └──► optical flow ─► velocity, height above floor, heading from the floor
```

### 1. Get the newest frame, not the oldest

A camera driver's queue hands you the *oldest* frame it is holding. Underwater that is a frame
from a vehicle that has since moved. The camera path keeps a mailbox instead — one slot,
newest wins — measured at **16.9 ms** stale against **396 ms** for a plain queue.

### 2. Know what the lens does

A camera behind a flat window is not the camera in the datasheet: water bends every ray, and
the effect grows toward the edges of the frame. Our downward camera measured **63.8° in
air** over 25 views, and Snell's law puts it at **46.7° in water** (computed; no camera has
been calibrated underwater yet). The forward camera is 73.9° in air and 53.6° in water the same way, and every metric
consumer goes through the same refraction correction (`optics.py`).

Skip that and a gate 4 m away reports as 5.15 m — a 29 % error no amount of control tuning
can fix.

### 3. Detect, on a chip built for it

The Hailo-8 runs the network. Measured: **99.0 Hz** standalone (re-taken after the B-61
colour fix), **53.9 Hz** through the live ROS graph (older, not yet re-taken), **18.0 ms** from photon to detection. Segmentation masks are decoded without ever
leaving integer arithmetic — 31.5 ms down to **0.93 ms**, bit-for-bit identical.

The detector picks its runtime from the artifact it is given: a Hailo binary (`.hef`) on the
vehicle, or plain PyTorch (`.pt`) on a dev box. Same interface, so nothing above it changes.

The weights themselves (`models/*.pt`, `*.onnx`, `*.hef`) are versioned in **Git LFS**, beside
the `<stem>.yaml` sidecar each one needs. A clone that never ran `git lfs pull` holds small text
pointers under the models' names; the detector and `bringup_check` refuse a pointer by name with
that fix, rather than hand it to the chip. CI checks out without LFS, so no CI run spends the
repository's LFS bandwidth. Nothing under `docs/` is ever LFS: GitHub Pages would serve the
pointer text (`test/test_lfs_holds_only_the_weights.py`).

### 4. Hold the target when the detector blinks

Detectors flicker. Measured over 71 real gaps in real footage, the median gap is 0.155 s and
the 90th percentile 0.651 s. Rather than tune a timeout by feel, there is a ladder — tracker,
then an optical-flow follower, then a feature anchor that once held **175 consecutive frames**
the detector had lost. Each rung is sized against that measured distribution.

### 5. Measure motion from the floor

The downward camera is a velocity sensor. Rotation is subtracted before scaling, because
translation scales with height and rotation does not. Against a tape: three 30 cm slides,
worst error **1.09 cm**, with the implied height coming back 0.72 / 0.69 / 0.70 m against a
0.72 m measurement. When the image cannot support a measurement it reports **nothing** — never
a confident zero.

---

## Nodes

| Node | Publishes | Purpose |
|---|---|---|
| `camera_node` | `image_raw`, `camera_info` | the mailbox camera path; also plays video files, steered by the display node's `video_seek_rel`, `video_seek_frame` and `video_speed` (playback speed, 1.0 = real time) |
| `detector_node` | `detections`, `candidates`, `contours`, `image_debug` | inference, plus the always-on operator alignment line |
| `tracker_node` | `tracks` | stable ids, and a predicted box across a short gap |
| `lock_node` | `lock`, `correspondences`, `anchor_decomposition` | the continuity ladder and the feature anchor. `anchor_decomposition` is a second range estimate, fitted frame-to-reference rather than frame-to-frame; published for a consumer to fuse, acted on by nothing yet |
| `flow_node` | `velocity`, `floor_height`, `floor_grid_deg`, `lane_heading_deg`, `distance_traveled`, `distance_debug`, `flow_calibration`, `flow_tracks`, `/mongla/flow/height_source` | everything the downward camera can tell us. `distance_debug` is `[distance m, height m, tracks, active]` for the display and Batighor's flow plots. `/mongla/flow/height_source` (latched) says where the height comes from: `tiles`, `survey`, `survey, no position`, `pool_depth` or `none`, and a mission refuses to start on `none` ([the course file](../../missions/course-file.md)). `flow_tracks` is the point pairs each accepted interval measured, and `floor_read` (`FloorRead`, `flow/floor_read.py`) where on the picture the tile grating and the lane line were read: both only while someone watches, and Batighor draws them over the downward picture (every grout line, the lane centreline) beside velocity, distance and quality. A measured slide is `tools/bench/flow_launch_check.py` |

`lock_node` is one class in seven files. `lock_node.py` holds the per-frame loop (detection
in, rung chosen, anchor run) and the list of files. The other six are mixins of the same class,
each with one job: `lock_setup.py` (parameters, the anchor model and the checkpoint bank, live
retuning), `lock_wiring.py` (the steps `__init__` runs, in order: state, publishers,
subscriptions), `lock_frames.py` (the image and detection callbacks), `lock_inputs.py` (the
IMU, state, odometry and floor-height callbacks, each with its freshness check),
`lock_anchor.py` (the anchor rung) and `lock_publish.py` (every topic the node writes, plus the
health log). To find where a `self._x` is defined, grep them together:
`grep -n "def _x" lock_*.py`.

Under `vision_pi`, flow runs inside the vision process beside the downward camera and takes its
frames by reference (`direct_feed`, measured-bars section 89); `vision` and `vision_dual` still
start it as `flow_node` on the topic. It is split the same way, under `flow/`: `flow_node.py` (the per-frame pipeline),
`flow_setup.py` (parameters, intrinsics, input callbacks), `flow_checks.py` (every check that
may refuse a frame), `flow_floor.py` (tile-grid heading and lane lines), `flow_publish.py`
(topics and the log) and `flow_constants.py` (the measured constants, each in
`measured-bars.md`).

**Key point:** `flow_node` publishes no velocity without a calibration it can use: one that
loads, was taken dry, names this camera or its `profile` in `applies_to`, and has the frames'
aspect ratio. With no `calibration` path it binds one from `profile`, as `camera_node` does.
`allow_uncalibrated:=true` runs on the focal parameters anyway with a wider sigma, and the
latched `flow_calibration` topic says which state it is in; localization reports
`aided (flow uncalibrated)` from it (B-106).

`detector_node` likewise: `detector_node.py` (the inference loop and its recovery),
`detector_setup.py` (parameters, model loading, live changes), `detector_frames.py` (mailbox
and direct feed), `detector_publish.py` (topics and operator log lines) and
`detector_params.py` (pure parameter parsing).

## Tools

```bash
ros2 run mongla_vision vision_check          # grade the whole mission graph; exits 1 on a FAIL
ros2 launch mongla_batighor calibrate.launch.py device_path:=<camera>   # measure it, air or water
ros2 run mongla_vision vision_thrust_check   # detection → thrust, disarmed and safe
```

## What is deliberately *not* here

**Image enhancement.** Contrast equalisation before the detector was measured in 17
configurations across four props and three venues and was never positive — on the gate it
destroyed 95 % of detections. The preprocessor is deleted, and
`tools/studies/recall_matrix.py --clahe` re-runs the measurement. The instinct is strong;
the data says no. What else was removed, and whether any of it was helping, is in the
[audit](audit.md).

## Testing

```bash
python3 -m pytest -q src/mongla_vision/test
```

The tests include ones that run on real recorded footage rather than synthetic images, and
one that reads `measured-bars.md` so a documented threshold cannot drift from the shipped
value.

---

Related: [the audit, every file](audit.md) ·
[`mongla_localization`](../mongla_localization/README.md) (what consumes the
velocity) · [Capability Map](../../capability-map.md) ·
[`measured-bars.md`](../../measured-bars.md)

## Every module

<!-- modules:begin -->
Generated from each module's docstring by `tools/ci/gen_reference.py`; a module
with no summary line shows as *(no summary)* until it gets one.

| module | what it is |
|---|---|
| `_text_engine.py` | TrueType text rendering engine for HUD widgets. |
| `camera_node.py` | read from a Camera and publish standard ROS image topics. |
| `config.py` | Named camera profiles. THIS DICT IS THE ONE THE CODE READS. |
| `continuity.py` | Detection continuity: the statistics the whole lock ladder is tuned on. |
| `detector_dual_node.py` | The whole vision stack in ONE process: two cameras, two detectors, no hop. |
| `detector_frames.py` | DetectorNode, part 2 of 4: how a camera frame reaches the detector. |
| `detector_node.py` | subscribe to image_raw, run YOLO11 (yolov11n), publish detections. |
| `detector_params.py` | How the detector's parameters are read. |
| `detector_publish.py` | DetectorNode, part 3 of 4: every topic the node writes, and its logs. |
| `detector_setup.py` | DetectorNode, part 1 of 4: parameters, model loading, and live parameter changes. |
| `draw.py` | the detector's debug image and the display node's HUD frame. |
| `draw_strip.py` | mission-control UI strip below the video frame. |
| `draw_video.py` | video section overlays for the mission-control HUD. |
| `draw_widgets.py` | reusable HUD micro-widgets for the mission-control strip. |
| `factory.py` | Single dispatch point: source-name -> Camera instance. |
| `identity.py` | Take GEOMETRY from the structure, IDENTITY from the symbol on it. |
| `image_msg.py` | `sensor_msgs/Image` to a NumPy array and back, with no `cv_bridge`. |
| `live_params.py` | Which of a node's parameters change while it runs, and a refusal for the rest. |
| `lock_anchor.py` | The anchor rung: enrol a reference or locate against the bank, and box what it found. |
| `lock_frames.py` | LockNode's image and detection inputs: by topic, or by reference when composed. |
| `lock_inputs.py` | LockNode's inputs besides images and detections: odometry, floor |
| `lock_node.py` | run the ladder, publish one target position. |
| `lock_publish.py` | LockNode's outputs: the /lock box, its decomposition, the pose |
| `lock_setup.py` | LockNode's setup: parameters, the anchor and the bank, and live retuning. |
| `lock_wiring.py` | The steps LockNode.__init__ runs, in order: what it locks onto, the inputs, |
| `optics.py` | Flat-port refraction -- the ONE place the water refractive index lives. |
| `pnp_node.py` | One PnP solver for every evidence source. Subscribe correspondences, publish a pose. |
| `pose_fuse_node.py` | Subscribe single-frame poses, publish the one the support agrees on. |
| `preflight.py` | Vision preflight -- block until the perception pipeline is alive. |
| `qos.py` | One QoS table for the vision topics, imported by every end of every link. |
| `seeing.py` | Can this camera see at all? A blinded camera and an empty scene look alike. |
| `stamps.py` | When was this actually SEEN? One implementation, every consumer. |
| `target_geometry.py` | Committed prop widths, keyed by the class name the detector emits. |
| `tool_geometry.py` | Where each tool is, relative to the camera that aims it. |
| `tracker_node.py` | subscribe to detections, publish stable tracks. |
| `trigger_stamps.py` | Which trigger made this frame: pairing camera frames with the board's pulses. |
| `underwater.py` | Underwater image statistics, and what they predict about detection. |
| `anchor/anchor.py` | Hold a target with NO detection at all. |
| `anchor/bank.py` | Several references instead of one, and a policy for when to add another. |
| `anchor/bank_lookup.py` | Asking the bank where the vehicle is: locate, recognise, verify. |
| `anchor/bank_store.py` | Saving the bank to disk and loading it back (a practice bank, or the last run). |
| `anchor/geometry.py` | The half of the homography we were throwing away. |
| `anchor/health.py` | A fouled lens and a changed world look identical, and need opposite answers. |
| `anchor/loop_closure.py` | When may a recognised place become a position fix? |
| `anchor/place_signature.py` | The bank's place signature: VLAD over the descriptors it already holds. |
| `anchor/pose.py` | Metric 6-DoF pose of a known-size planar target. The firing solution. |
| `anchor/xfeat_hailo.py` | XFeat on the Hailo-8, so the anchor rung stops burning the Pi's CPU. |
| `anchor/xfeat_onnx.py` | XFeat on the vehicle: ONNX + numpy, no torch. |
| `calibration/binding.py` | Which calibration belongs to which camera -- resolved ONCE, from the data. |
| `calibration/calibrate_node.py` | Guided camera calibration as a node; Batighor is its page. |
| `calibration/guide_capture.py` | The capture half of the guided calibration: the pose script and its threads. |
| `calibration/guide_files.py` | Where calibrations and captures live on disk, and which camera is which. |
| `calibration/solver.py` | Camera calibration that PROVES its own answer instead of asserting it. |
| `cameras/camera.py` | Abstract Camera — every image source implements this. |
| `cameras/discover.py` | discover_cameras() -- probe connected USB cameras. |
| `cameras/reopen.py` | When a camera that has stopped delivering is closed and opened again. |
| `cameras/ros_topic.py` | subscribes to a sensor_msgs/Image topic. |
| `cameras/v4l2_mailbox.py` | A mailbox, not a queue: always the newest frame, with the kernel's timestamp. |
| `cameras/video_file.py` | run the vision pipeline on a pre-recorded video file. |
| `cameras/webcam.py` | local USB / built-in webcam via cv2.VideoCapture. |
| `detection/contours.py` | Detections -> outlines. A box and a mask become the same thing here. |
| `detection/detector.py` | Abstract Detector — every detection backend implements this. |
| `detection/factory.py` | Pick a detector backend from the resolved model path. One seam, three callers. |
| `detection/gpu.py` | single decision point for the .pt path on a dev box. |
| `detection/hailo.py` | Hailo-8 detector backend: a compiled ``.hef`` behind the same `Detector` API. |
| `detection/hailo_device.py` | The process's one Hailo VDevice, shared by every detector and XFeat anchor. |
| `detection/hailo_seg.py` | `HailoSegDetector`: a raw segmentation HEF behind the same `Detector` API. |
| `detection/hef_file.py` | What a `.hef` and its sidecar say about themselves, read without a device. |
| `detection/messages.py` | Convert internal Detection objects to vision_msgs. |
| `detection/rangecrop.py` | Trade field of view for range, automatically, when the target is far. |
| `detection/seg_decode.py` | Host-side decode of a RAW YOLOv8/v11-seg head, in the QUANTISED domain. |
| `detection/yolo.py` | Ultralytics YOLO11 (or any compatible model file). |
| `flow/floor_read.py` | Where the floor's tile grid and lane line lie in the picture, for an operator to see. |
| `flow/flow_checks.py` | FlowVelocityNode, part 2 of 5: the checks that decide a measurement may be published. |
| `flow/flow_constants.py` | Measured constants shared by `flow_node` and its mixins. |
| `flow/flow_features.py` | Which pixels the flow node may trust: corners, caustics, and the round trip. |
| `flow/flow_floor.py` | FlowVelocityNode, part 3 of 5: what the floor itself says -- tile grid heading and lane lines. |
| `flow/flow_frame.py` | The flow node's per-frame work: pick the points, wait for the motion, judge it. |
| `flow/flow_math.py` | pure functions for downward-camera optical-flow distance. |
| `flow/flow_node.py` | the bottom camera AS A VELOCITY SENSOR. Our DVL substitute. |
| `flow/flow_publish.py` | FlowVelocityNode, part 4 of 5: every topic the node writes, and its periodic log. |
| `flow/flow_setup.py` | FlowVelocityNode, part 1 of 5: parameters, camera intrinsics, and the input callbacks. |
| `flow/flow_timing.py` | Timing. The thing that decides whether de-rotation helps or hurts. |
| `flow/flow_velocity.py` | Optical flow -> BODY VELOCITY. The observation an estimator can actually use. |
| `flow/pool_lines.py` | The lane line is a surveyed straight edge. Read a heading off it. |
| `flow/scale_check.py` | Is the height that scales every flow velocity actually right? |
| `flow/tile_grating.py` | The pool floor is a surveyed 2-D grating. Read it like an optical encoder. |
| `tracking/bytetrack.py` | ByteTrack wrapper using supervision.ByteTrack. |
| `tracking/confidence.py` | Detection confidence as the vehicle's measure of how much to trust a box. |
| `tracking/follower.py` | The fast rung: carry a box across the frames the detector has nothing for. |
| `tracking/kalman.py` | Per-track 4-state constant-velocity Kalman smoother. |
| `tracking/lock_state.py` | One target position, from whichever rung can still supply it. |
| `tracking/roboflow_tracker.py` | Roboflow `trackers` backend (OC-SORT / ByteTrack) behind the Tracker ABC. |
| `tracking/tracker.py` | Tracker ABC and TrackedDetection dataclass. |
| `tracking/visibility.py` | Keep the target in frame — and know where it went if it leaves anyway. |
| `utils/check_pipeline.py` | grade the vision graph a mission runs on, before it runs. |
| `utils/check_thrust.py` | end-to-end smoke test: detection -> a thrust command. |
| `utils/check_tracker.py` | health probe for the tracking pipeline. |
| `utils/display_draw.py` | What the vision display draws besides the frame: the splash, the |
| `utils/display_keys.py` | Keyboard handling for the vision display: camera switching and video playback. |
| `utils/display_node.py` | smooth OpenCV viewer for the perception pipeline. |
| `utils/switch_camera.py` | resume ONE detector, pause the others (CLI exclusivity). |
| `utils/water_check.py` | Which water is this? Characterise the pool before trusting a model in it. |
<!-- modules:end -->
