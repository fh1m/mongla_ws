# Vision Roadmap

Phased plan for `duburi_vision` and its planner integration. Phases v1
and v4 are shipped; v2/v3/v5 land in already-named placeholder files so
the diff stays small at each step.

## v1 -- Detection (DONE)

- `Camera` ABC + factory + `webcam` + `ros_topic` sources
- Stubs for jetson / blueos / mavlink that raise `NotImplementedError`
  with a friendly "use webcam or ros_topic" message
- `YoloDetector` wrapping Ultralytics YOLO26 with class allowlist, conf/iou,
  warmup, and a fail-fast `select_device()` for GPU-first inference
- `vision_msgs/Detection2DArray` publishing
- `draw.render_all` with status badge, dashed reticle, primary highlight,
  crosshair, alignment readout, stale banner
- Unit tests (factory + detector + gpu + messages) -- 26 tests, no GPU
  needed in CI
- Launch files: `webcam_demo`, `sim_demo`, `debug_view`

Acceptance: `ros2 launch duburi_vision webcam_demo.launch.py` opens an
rqt window showing your webcam feed with person boxes drawn on it; the
status badge says `cuda:0` and the inference loop runs at >= camera FPS
on RTX 2060. Same node binary on Jetson should work after only an
ultralytics + matching torch install.

## v2 -- Tracking

Persist `track_id` across frames. Land in `duburi_vision/tracking/`:

- `tracker.py` Tracker ABC, `bytetrack.py` (supervision.ByteTrack wrapper)
- New `tracker_node.py` at package root subscribing `detections`, publishing
  `tracks` (also Detection2DArray, with `tracking_id` populated)
- Extend `draw.py` so each track keeps the same color
- Tests: ID stability across two consecutive frames with overlapping boxes

Detailed notes: `src/duburi_vision/duburi_vision/tracking/PLAN.md`.

## v3 -- Filtering

Per-track 4-state Kalman smoother (`cx, cy, vx, vy`) so the visual-PID
setpoint isn't chasing per-frame jitter. Land in `duburi_vision/filters/`:

- `smoother.py` ABC, `kalman.py` implementation (`filterpy` or hand-rolled)
- Either fold into `tracker_node` or add a separate `kalman_node.py`
  depending on measured Jetson Orin latency
- Tests: stationary target -> smoothed center variance < raw center variance

Detailed notes: `src/duburi_vision/duburi_vision/filters/PLAN.md`.

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
