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
the effect grows toward the edges of the frame. Our forward camera is **63.8° in air and
46.7° in water**, measured over 25 views rather than taken from a spec sheet, and every metric
consumer goes through the same refraction correction (`optics.py`).

Skip that and a gate 4 m away reports as 5.15 m — a 29 % error no amount of control tuning
can fix.

### 3. Detect, on a chip built for it

The Hailo-8 runs the network. Measured: **80.9 Hz** standalone, **53.9 Hz** through the live
ROS graph, **18.0 ms** from photon to detection. Segmentation masks are decoded without ever
leaving integer arithmetic — 31.5 ms down to **0.93 ms**, bit-for-bit identical.

The detector picks its runtime from the artifact it is given: a Hailo binary (`.hef`) on the
vehicle, or plain PyTorch (`.pt`) on a dev box. Same interface, so nothing above it changes.

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
| `camera_node` | `image_raw`, `camera_info` | the mailbox camera path; also plays video files |
| `detector_node` | `detections`, `contours`, `image_debug` | inference, plus the always-on operator alignment line |
| `tracker_node` | `tracks` | stable ids, and a predicted box across a short gap |
| `lock_node` | `lock`, `correspondences` | the continuity ladder and the feature anchor |
| `flow_node` | `velocity`, `floor_height`, `floor_grid_deg`, `lane_heading_deg`, `distance_traveled` | everything the downward camera can tell us |
| `depth_estimation_node` | `vis_range` | monocular proximity, off by default |
| `mission_web_node` | — | the pool-day browser console |

`lock_node` is one class in four files. `lock_node.py` holds the per-frame loop (detection in,
rung chosen, anchor run). The other three are mixins of the same class, each with one job:
`lock_setup.py` (parameters, the anchor model and the checkpoint bank), `lock_inputs.py`
(the IMU, state, odometry and floor-height callbacks, each with its freshness check) and
`lock_publish.py` (every topic the node writes, plus the health log). To find where a
`self._x` is defined, grep the four files together: `grep -n "def _x" lock_*.py`.

`flow_node` is split the same way, under `flow/`: `flow_node.py` (the per-frame pipeline),
`flow_setup.py` (parameters, intrinsics, input callbacks), `flow_checks.py` (every check that
may refuse a frame), `flow_floor.py` (tile-grid heading and lane lines), `flow_publish.py`
(topics and the log) and `flow_constants.py` (the measured constants, each in
`measured-bars.md`).

`detector_node` likewise: `detector_node.py` (the inference loop and its recovery),
`detector_setup.py` (parameters, model loading, live changes), `detector_frames.py` (mailbox
and direct feed), `detector_publish.py` (topics and operator log lines) and
`detector_params.py` (pure parameter parsing).

## Tools

```bash
ros2 run mongla_vision vision_check          # is the pipeline alive and seeing?
ros2 run mongla_vision calibrate             # measure this camera, in water
ros2 run mongla_vision vision_thrust_check   # detection → thrust, disarmed and safe
ros2 launch mongla_vision mission_web.launch.py   # both cameras in a browser
```

## What is deliberately *not* here

**Image enhancement.** Contrast equalisation before the detector was measured in 17
configurations across four props and three venues and was never positive — on the gate it
destroyed 95 % of detections. It ships off in every profile, and a test keeps it off. The
instinct is strong; the data says no.

## Testing

```bash
python3 -m pytest -q src/mongla_vision/test
```

The tests include ones that run on real recorded footage rather than synthetic images, and
one that reads `measured-bars.md` so a documented threshold cannot drift from the shipped
value.

---

Related: [`mongla_localization`](../mongla_localization/README.md) (what consumes the
velocity) · [Capability Map](../../capability-map.md) ·
[`measured-bars.md`](../../measured-bars.md)

## Every module

<!-- modules:begin -->
Generated from each module's docstring by `tools/gen_reference.py`; a module
with no summary line shows as *(no summary)* until it gets one.

| module | what it is |
|---|---|
| `_text_engine.py` | TrueType text rendering engine for HUD widgets. |
| `approach.py` | How close should the vehicle get before it trusts what it sees? |
| `camera_node.py` | read from a Camera and publish standard ROS image topics. |
| `config.py` | Named camera profiles. THIS DICT IS THE ONE THE CODE READS. |
| `continuity.py` | Detection continuity: the statistics the whole lock ladder is tuned on. |
| `detector_dual_node.py` | The whole vision stack in ONE process: two cameras, two detectors, no hop. |
| `detector_frames.py` | DetectorNode, part 2 of 4: how a camera frame reaches the detector. |
| `detector_node.py` | subscribe to image_raw, run YOLO11 (yolov11n), publish detections. |
| `detector_params.py` | How the detector's parameters are read. |
| `detector_publish.py` | DetectorNode, part 3 of 4: every topic the node writes, and its logs. |
| `detector_setup.py` | DetectorNode, part 1 of 4: parameters, model loading, and live parameter changes. |
| `draw.py` | thin dispatcher for the mission-control HUD. |
| `draw_strip.py` | mission-control UI strip below the video frame. |
| `draw_video.py` | video section overlays for the mission-control HUD. |
| `draw_widgets.py` | reusable HUD micro-widgets for the mission-control strip. |
| `factory.py` | Single dispatch point: source-name -> Camera instance. |
| `identity.py` | Take GEOMETRY from the structure, IDENTITY from the symbol on it. |
| `ladder_config.py` | One deck file for the ladder's switches, read by the node itself. |
| `lock_anchor.py` | The anchor rung: enrol a reference or locate against the bank, and box what it found. |
| `lock_inputs.py` | LockNode's inputs besides images and detections: attitude, odometry, |
| `lock_node.py` | run the ladder, publish one target position. |
| `lock_publish.py` | LockNode's outputs: the /lock box, its decomposition, the pose |
| `lock_setup.py` | LockNode's setup: parameters, the anchor and the bank, and live retuning. |
| `optics.py` | Flat-port refraction -- the ONE place the water refractive index lives. |
| `preflight.py` | Vision preflight -- block until the perception pipeline is alive. |
| `qos.py` | One QoS table for the vision topics, imported by every end of every link. |
| `seeing.py` | Can this camera see at all? A blinded camera and an empty scene look alike. |
| `stamps.py` | When was this actually SEEN? One implementation, every consumer. |
| `target_geometry.py` | Committed prop widths, keyed by the class name the detector emits. |
| `time_to_contact.py` | Seconds until contact, for a MOVING observer. |
| `tool_geometry.py` | Where each tool is, relative to the camera that aims it. |
| `tracker_node.py` | subscribe to detections, publish stable tracks. |
| `underwater.py` | Underwater image statistics, and what they predict about detection. |
| `vision_node.py` | single-process diagnostic for camera + detector. |
| `anchor/anchor.py` | Hold a target with NO detection at all. |
| `anchor/bank.py` | Several references instead of one, and a policy for when to add another. |
| `anchor/bank_lookup.py` | Asking the bank where the vehicle is: locate, recognise, corroborate, verify. |
| `anchor/bank_store.py` | Saving the bank to disk and loading it back (a practice bank, or the last run). |
| `anchor/geometry.py` | The half of the homography we were throwing away. |
| `anchor/health.py` | A fouled lens and a changed world look identical, and need opposite answers. |
| `anchor/loop_closure.py` | When may a recognised place become a position fix? |
| `anchor/pose.py` | Metric 6-DoF pose of a known-size planar target. The firing solution. |
| `anchor/xfeat_hailo.py` | XFeat on the Hailo-8, so the anchor rung stops burning the Pi's CPU. |
| `anchor/xfeat_onnx.py` | XFeat on the vehicle: ONNX + numpy, no torch. |
| `calibration/binding.py` | Which calibration belongs to which camera -- resolved ONCE, from the data. |
| `calibration/guide.py` | Step-by-step guided camera calibration in a browser. |
| `calibration/guide_capture.py` | The capture half of the guided calibration: the pose script and its threads. |
| `calibration/guide_page.py` | The calibration page `guide.py` serves at `/`: one HTML document, inline |
| `calibration/solver.py` | Camera calibration that PROVES its own answer instead of asserting it. |
| `cameras/camera.py` | Abstract Camera — every image source implements this. |
| `cameras/discover.py` | discover_cameras() -- probe connected USB cameras. |
| `cameras/ros_topic.py` | subscribes to a sensor_msgs/Image topic. |
| `cameras/v4l2_mailbox.py` | A mailbox, not a queue: always the newest frame, with the kernel's timestamp. |
| `cameras/video_file.py` | run the vision pipeline on a pre-recorded video file. |
| `cameras/webcam.py` | local USB / built-in webcam via cv2.VideoCapture. |
| `depth/depth_estimation_node.py` | monocular depth estimation for detected objects. |
| `detection/confidence_calibration.py` | A confidence of 0.5 does not mean the same thing in every pool. |
| `detection/contours.py` | Detections -> outlines. A box and a mask become the same thing here. |
| `detection/detector.py` | Abstract Detector — every detection backend implements this. |
| `detection/factory.py` | Pick a detector backend from the resolved model path. One seam, three callers. |
| `detection/gpu.py` | single decision point for the .pt path on a dev box. |
| `detection/hailo.py` | Hailo-8 detector backend: a compiled ``.hef`` behind the same `Detector` API. |
| `detection/hailo_seg.py` | `HailoSegDetector`: a raw segmentation HEF behind the same `Detector` API. |
| `detection/messages.py` | Convert internal Detection objects to vision_msgs. |
| `detection/preprocess.py` | Contrast preprocessing for underwater frames, measured on real footage. |
| `detection/profiles.py` | ONE WORD instead of five knobs, because competition day is not the time. |
| `detection/rangecrop.py` | Trade field of view for range, automatically, when the target is far. |
| `detection/seg_decode.py` | Host-side decode of a RAW YOLOv8/v11-seg head, in the QUANTISED domain. |
| `detection/yolo.py` | Ultralytics YOLO11 (or any compatible model file). |
| `flow/flow_checks.py` | FlowVelocityNode, part 2 of 5: the checks that decide a measurement may be published. |
| `flow/flow_constants.py` | Measured constants shared by `flow_node` and its mixins. |
| `flow/flow_floor.py` | FlowVelocityNode, part 3 of 5: what the floor itself says -- tile grid heading and lane lines. |
| `flow/flow_math.py` | pure functions for downward-camera optical-flow distance. |
| `flow/flow_node.py` | the bottom camera AS A VELOCITY SENSOR. Our DVL substitute. |
| `flow/flow_publish.py` | FlowVelocityNode, part 4 of 5: every topic the node writes, and its periodic log. |
| `flow/flow_setup.py` | FlowVelocityNode, part 1 of 5: parameters, camera intrinsics, and the input callbacks. |
| `flow/flow_timing.py` | Timing. The thing that decides whether de-rotation helps or hurts. |
| `flow/flow_velocity.py` | Optical flow -> BODY VELOCITY. The observation an estimator can actually use. |
| `flow/scale_check.py` | Is the height that scales every flow velocity actually right? |
| `tracking/bytetrack.py` | ByteTrack wrapper using supervision.ByteTrack. |
| `tracking/cascade.py` | Never lose the target: an association cascade, cheapest cue first. |
| `tracking/confidence.py` | Detection confidence as the vehicle's measure of how much to trust a box. |
| `tracking/follower.py` | The fast rung: carry a box across the frames the detector has nothing for. |
| `tracking/kalman.py` | Per-track 4-state constant-velocity Kalman smoother. |
| `tracking/lock_state.py` | One target position, from whichever rung can still supply it. |
| `tracking/presence.py` | The rung BELOW detection: accumulate evidence the detector refused to call. |
| `tracking/reid.py` | Stitch a track's identity back together after a gap. The 179-switch fix. |
| `tracking/roboflow_tracker.py` | Roboflow `trackers` backend (OC-SORT / ByteTrack) behind the Tracker ABC. |
| `tracking/tracker.py` | Tracker ABC and TrackedDetection dataclass. |
| `tracking/visibility.py` | Keep the target in frame — and know where it went if it leaves anyway. |
| `utils/check_pipeline.py` | standalone health probe for the perception pipeline. |
| `utils/check_thrust.py` | end-to-end smoke test: detection -> a thrust command. |
| `utils/check_tracker.py` | health probe for the tracking pipeline. |
| `utils/display_node.py` | smooth OpenCV viewer for the perception pipeline. |
| `utils/switch_camera.py` | resume ONE detector, pause the others (CLI exclusivity). |
| `utils/water_check.py` | Which water is this? Characterise the pool before trusting a model in it. |
| `web/dashboard_state.py` | Pure (rclpy-free) helpers for the mission-web console. |
| `web/mission_web_node.py` | browser mission-control console for the Mongla vision stack. |
<!-- modules:end -->
