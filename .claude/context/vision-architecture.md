# Vision Architecture (`duburi_vision`)

Authoritative design notes for the perception package. Mirrors the
`duburi_sensors` pattern: ABC + factory + per-source class + standalone
diagnostic node.

## File map

```
src/duburi_vision/duburi_vision/
  factory.py             # make_camera(name, **kw) + BUILDERS dict
  config.py              # CAMERA_PROFILES dict (mirrors config/cameras.yaml)
  draw.py                # cv2/supervision overlays + draw_track_ids()
  camera_node.py         # publish image_raw + camera_info
  detector_node.py       # subscribe image_raw -> detections + image_debug
  tracker_node.py        # subscribe detections -> tracks (ByteTrack + Kalman)
  vision_node.py         # in-process diag (cousin of sensors_node)
  cameras/
    camera.py            # Camera ABC (the only base in the tree)
    webcam.py            # cv2.VideoCapture wrapper
    ros_topic.py         # subscribes sensor_msgs/Image (Gazebo / BlueOS)
    {jetson,blueos,mavlink}_stub.py
  detection/
    detector.py          # Detector ABC + Detection dataclass
    yolo.py              # YoloDetector (Ultralytics YOLO26)
    gpu.py               # select_device() -- fail-fast CUDA check
    messages.py          # Detection -> vision_msgs converters (+ array_to_detections)
  tracking/
    __init__.py          # exports Tracker, TrackedDetection, ByteTrackWrapper, TrackKalmanSmoother
    tracker.py           # Tracker ABC + TrackedDetection dataclass
    bytetrack.py         # supervision.ByteTrack wrapper (occlusion bridging via lost_tracks)
    kalman.py            # PerTrackKalman + TrackKalmanSmoother (filterpy 4-state CV)
  preflight.py           # assert_vision_ready / wait_vision_state_ready
  utils/
    check_pipeline.py    # `vision_check`        CLI -- topic-only smoke test
    check_thrust.py      # `vision_thrust_check` CLI -- detection -> RC echo
    check_tracker.py     # `tracker_check`       CLI -- tracking smoke test
  filters/PLAN.md        # v3 -- folded into tracker_node (Kalman in kalman.py)
config/
  cameras.yaml           # camera profiles
  detector.yaml          # model + class params
  tracker.yaml           # ByteTrack + Kalman thresholds (all as ROS params)
```

Naming rule: every file is named after the thing inside it. No `base.py`,
no `to_ros.py`, no `nodes/` or `viz/` subfolders. Per-user request,
`duburi_sensors` is NOT renamed to match.

## Topic contract

```
/duburi/vision/<cam>/image_raw      sensor_msgs/Image            (camera_node)
/duburi/vision/<cam>/camera_info    sensor_msgs/CameraInfo       (camera_node)
/duburi/vision/<cam>/detections     vision_msgs/Detection2DArray (detector_node)
/duburi/vision/<cam>/tracks         vision_msgs/Detection2DArray (tracker_node, optional)
/duburi/vision/<cam>/image_debug    sensor_msgs/Image            (detector_node, rate-limited)
```

`/tracks` uses the same message type as `/detections`. The difference:
- `Detection2D.tracking_id` is populated with a stable ByteTrack integer ID (stringified)
- Bbox center (`cx`, `cy`) is Kalman-smoothed; jitter from YOLO NMS is filtered out
- Entries with `score=0.0` are Kalman-only predictions (detector missed that frame)

`VisionState(use_tracks=True)` subscribes `/tracks` instead of `/detections`. Enable globally:
```bash
ros2 param set /duburi_manager vision.use_tracks true
```
Or per-goal: `duburi vision_align_yaw --tracking true` (CLI) / `duburi.vision.yaw(..., tracking=True)` (DSL).

## Dataflow (with tracker_node)

```
camera_node  →  /image_raw  →  detector_node  →  /detections  →  tracker_node  →  /tracks
                                    ↓                                                  ↓
                              /image_debug                                    VisionState (use_tracks=True)
                                                                                        ↓
                                                              VisionState (use_tracks=False) ←──/detections
```

`<cam>` is the camera profile name (`laptop`, `sim_front`, `sim_bottom`, ...).
`vision_msgs/Detection2D.results[0].hypothesis.class_id` is a **string**
(rosidl-required); we publish the human label (`'person'`, `'gate'`)
straight in there so downstream filters stay readable
(`vision_state.bbox_error('person')`). When the model didn't ship a
name table we fall back to the numeric class id stringified so the
field is never empty.

## GPU contract

`select_device(device, logger=None)` is the single decision point. Called
once per detector at construction.

| `device` value     | Behavior                                                                           |
|--------------------|------------------------------------------------------------------------------------|
| `cuda:N` (default) | Require CUDA. Raise `RuntimeError` if unavailable. Logs the canary line.           |
| `cpu`              | Explicit CPU. Logs canary line with `(requested)`.                                 |
| `auto`             | Prefer CUDA, fall back to CPU silently. **Tests/CI only.**                         |
| anything else      | `ValueError`.                                                                      |

Canary log line (grep for this on every machine):

```
[VIS ] using cuda:0 (NVIDIA GeForce RTX 2060)  torch=2.11.0+cu128  cuda=12.8
```

## Visualization layers

`draw.render_all(frame, detections, ...)` paints in this order:

1. raw frame
2. dashed center reticle
3. all detections (light boxes + labels)
4. primary target (thick amber box + filled corners)
5. crosshair on primary
6. offset arrow center -> primary
7. alignment readout panel (bottom-left, deadband-aware border color)
8. status badge (top-left, healthy=green / unhealthy=red border)
9. red full-width "STALE FRAME" banner if `healthy=False`

Every glyph answers a specific operator question. Every diagnostic state
is visible in one frame — that's how we let the user paste a screenshot
into a bug report.

## Adding a new camera source

1. Create `cameras/<name>.py` with a class that subclasses `Camera` and
   implements `read() / is_healthy() / info() / close()`.
2. Add a `_build_<name>` function in `factory.py` and a row in `BUILDERS`.
3. Add a row in `CAMERA_PROFILES` and `config/cameras.yaml` if you want
   it pickable by `profile:=`.
4. Optionally add a unit test that asserts the source raises a clear
   error when its required kwargs are missing.

That's it. No node code changes — `camera_node` and `vision_node` route
through the factory.

## Adding a new detector backend

1. Subclass `Detector` in a new file under `detection/`.
2. Implement `infer(frame_bgr) -> list[Detection]` and `class_names()`.
3. Wire it from `detector_node.py` behind a `backend:=` ROS param if you
   want runtime selection (today there's only `yolo`, so it's hardcoded).

The pipeline downstream of `Detector` doesn't care which model is in the
box.

## Vision verbs (v4) -- where the closed loop lives

The control loop owns thrust; the planner asks for an outcome. Vision
verbs are the bridge.

```
mission script
  v
DuburiClient.send('vision_align_3d', ...)
  v
/duburi/move action  (Move.Goal carries 12 vision fields)
  v
auv_manager_node
  +-- _vision_state_for(camera)        -- lazy VisionState per camera
  +-- wait_vision_state_ready(...)     -- one-time preflight, polling-only
  +-- duburi.vision_align_3d(...)
        v
duburi_control.motion_vision.vision_track_axes
  +-- 20 Hz  send_rc_override(forward, lateral, yaw)        -- one packet
  +-- 5  Hz  set_target_depth(depth_setpoint += clamp(ey * Kp_depth))
  +-- on stale > stale_after  -> neutral + (fail | hold)
```

Why the loop is in the manager, not the client:

- One MAVLink owner. Anything else risks two writers fighting for Ch4/5/6.
- Sub-100 ms feedback path. ActionClient -> manager -> Pixhawk RTT is
  one process hop; running the loop inside the manager removes the
  network/IPC jitter.
- Preflight is a polling check on an existing `VisionState`. No
  `spin_once` from inside an action callback, no second subscription set.

Axis composition is by CSV: `axes='yaw,forward,depth'`. The convenience
verbs (`vision_align_yaw`, `vision_align_lat`, `vision_align_depth`,
`vision_hold_distance`) are just `vision_align_3d` with `axes` pinned --
one canonical loop, no copy-paste. `vision_acquire` is a separate
function because its early-exit semantics differ (any detection wins).

Verifying the chain before pool day:

```
ros2 run duburi_vision vision_check                 # detector publishing?
ros2 run duburi_vision vision_thrust_check          # detection -> RC echo?
ros2 run duburi_planner mission find_person_demo    # full mission rehearsal
```
