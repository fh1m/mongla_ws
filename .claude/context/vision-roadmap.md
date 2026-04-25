# Vision Roadmap

Phased plan for `duburi_vision` and its planner integration. Phases v1
and v4 are shipped; v2/v3/v5 land in already-named placeholder files so
the diff stays small at each step.

## v1 -- Detection (DONE)

- `Camera` ABC + factory + `webcam` + `ros_topic` + `video_file` sources
  - `video_file` wraps `cv2.VideoCapture(path)` for offline pre-pool testing;
    loops at EOF by default (`loop:=false` to stop). Launch with `video_file:=<path>`.
- Stubs for jetson / blueos / mavlink that raise `NotImplementedError`
  with a friendly "use webcam, ros_topic, or video_file" message
- `YoloDetector` wrapping Ultralytics YOLO26 with class allowlist, conf/iou,
  warmup, and a fail-fast `select_device()` for GPU-first inference
- `vision_msgs/Detection2DArray` publishing
- `draw.render_all` with status badge, dashed reticle, primary highlight,
  crosshair, alignment readout, stale banner
- Unit tests (factory + detector + gpu + messages) -- 26 tests, no GPU
  needed in CI
- Launch files: `cameras_` (canonical), `webcam_demo` (deprecated stub), `sim_demo`

Acceptance: `ros2 launch duburi_vision webcam_demo.launch.py` opens an
rqt window showing your webcam feed with person boxes drawn on it; the
status badge says `cuda:0` and the inference loop runs at >= camera FPS
on RTX 2060. Same node binary on Jetson should work after only an
ultralytics + matching torch install.

## v2 -- Tracking (DONE)

Temporal continuity + occlusion bridging. Shipped in tracking integration commit.

- `tracking/tracker.py` — `Tracker` ABC + `TrackedDetection` dataclass
- `tracking/bytetrack.py` — `supervision.ByteTrack` wrapper; emits `predicted=True`
  entries for lost-but-buffered tracks via `self._bt.lost_tracks` loop (occlusion bridging)
- `tracking/__init__.py` — exports `Tracker`, `TrackedDetection`, `ByteTrackWrapper`, `TrackKalmanSmoother`
- `tracker_node.py` — ROS node; subscribes `/detections` + `camera_info`, publishes `/tracks`
  (same `Detection2DArray`, `tracking_id` populated + Kalman-smoothed cx/cy)
- `draw.py` — `draw_track_ids()` with 12-color stable palette (track_id % 12)
- `utils/check_tracker.py` — `tracker_check` CLI smoke test
- `config/tracker.yaml` — all 8 tracker params; all declared as ROS params for live tuning
- `Move.action` + `commands.py` — `tracking` bool field on all 6 vision verbs
- `vision_state.py` — `use_tracks=True` subscribes `/tracks`; `track_id` field on `Sample`
- `auv_manager_node.py` — `vision.use_tracks` ROS param; per-goal `tracking=True` sets it
- Launch files (`cameras_.launch.py`, `sim_demo`) — `with_tracking:=false` opt-in arg

Acceptance: `ros2 run duburi_vision tracker_check --camera laptop --duration 5 --require-class person`
exits 0 with a stable track ID across ≥ 3 frames.

## v3 -- Filtering (DONE, folded into tracker_node)

Per-track 4-state CV Kalman smoother (`cx, cy, vx, vy`) shipped inside `tracker_node`,
not a separate node (shared per-track state, avoids extra ROS hop).

- `tracking/kalman.py` — `PerTrackKalman` (filterpy `KalmanFilter(dim_x=4, dim_z=2)`)
  + `TrackKalmanSmoother` dict manager; `step()` skips measurement update on `predicted=True`
- `tracker_node.py` — runs Kalman smoothing pass after ByteTrack association;
  rebuilds xyxy with smoothed cx/cy; skips expired tracks (`predict_streak >= max_predict_frames`)
- Config: `kalman_process_noise`, `kalman_measurement_noise`, `max_predict_frames` in `tracker.yaml`

Decision: particle filters skipped — underwater single-target tracking is unimodal; the 4-state
CV Kalman is appropriate and has near-zero overhead.

## v4 -- Vision verbs in DuburiClient (DONE)

Vision is a first-class verb on `/duburi/move`, not a side channel. The
closed loop runs INSIDE `auv_manager_node` (the single MAVLink owner)
so the latency stays bounded and there's no risk of two processes
fighting for thrust.

Pieces:

- `Move.action` carries 12 new fields (`camera`, `target_class`, `axes`,
  `deadband`, `kp_yaw`, `kp_lat`, `kp_depth`, `kp_forward`,
  `target_bbox_h_frac`, `visual_pid`, `on_lost`, `stale_after`) and the
  `cmd` enum is extended.
- `duburi_control/commands.py` adds 6 verbs:
  - `vision_align_3d`        -- pick axes via CSV `axes`
  - `vision_align_yaw`       -- one-axis convenience wrapper
  - `vision_align_lat`       -- one-axis convenience wrapper
  - `vision_align_depth`     -- one-axis convenience wrapper (incremental)
  - `vision_hold_distance`   -- forward thrust from bbox height
  - `vision_acquire`         -- wait (optionally driving) until target seen
- `duburi_control/motion_vision.py` owns `vision_track_axes` (multi-axis
  P loop @ 20 Hz RC + 5 Hz depth) and `vision_acquire`.
- `duburi_manager/vision_state.py` holds the per-camera subscriber pool
  and exposes `largest()`, `bbox_error()`, `is_fresh()`.
- `duburi_vision/preflight.py` provides `assert_vision_ready` (CLI-side)
  and `wait_vision_state_ready` (manager-side, polling-only -- safe to
  call from inside an action callback).
- Mission: `duburi_planner/missions/find_person_demo.py` walks every
  verb in turn (acquire -> yaw -> hold -> 3D -> lose+reacquire -> 3D+depth).

CLI utilities for verifying the pipeline before touching the planner:

- `ros2 run duburi_vision vision_check` -- pure topic probe; reports
  image_raw rate, camera_info presence, detections rate, classes seen.
- `ros2 run duburi_vision vision_thrust_check` -- sends one
  `vision_align_yaw` goal to `/duburi/move` and reports the result; pair
  with `[RC   ] Yaw:NNN` lines in the manager log to confirm the chain
  closed end-to-end.

## v5 -- Real-vehicle camera sources

Drop in:
- `cameras/jetson.py` -- replaces the stub. V4L2 + MJPG + locked exposure
  for the two Blue Robotics Low-Light HD USB cams
- `cameras/blueos.py` -- replaces the stub. Open the BlueOS RTSP URL via
  GStreamer (`cv2.VideoCapture` with the gstreamer backend)
- `cameras/mavlink.py` -- replaces the stub. Pull the RTSP URL out of
  `VIDEO_STREAM_INFORMATION`, then delegate to the BlueOS path

No node-side changes: factory + node code stays as-is. `cameras.yaml`
profiles for `jetson_front`, `jetson_bottom`, `blueos`, `mavlink` flip
from "raises NotImplementedError" to "actually works".

## Always-on rules

- One source per launch. No mid-run camera switching, no auto-fallback.
  Same rule as `duburi_sensors` yaw sources.
- One canonical canary log line at startup so a broken Jetson is loud,
  not slow. Grep `[VIS ]` to verify GPU/CPU + model + classes.
- Every diagnostic state must be visible in one frame of `image_debug`.
  If you find yourself adding a `print()` to debug, add a `draw.*` overlay
  instead.
- vision_msgs is the cross-package contract. The planner MUST NOT import
  ultralytics or supervision -- it consumes only `Detection2DArray`.
