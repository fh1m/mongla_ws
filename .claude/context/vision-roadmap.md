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
- `YoloDetector` wrapping Ultralytics YOLO11 (yolov11n default) with class allowlist, conf/iou,
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

## v2b -- Tracker backend upgrade (deferred, post-competition)

The current tracker already wraps `supervision.ByteTrack` (not a custom implementation).
Swapping to OC-SORT is a **one-line change** in `tracking/bytetrack.py`:

```python
# Replace:   self._bt = sv.ByteTrack(...)
# With:      self._bt = sv.OCSORT(...)   # HOTA 61.9 vs 60.1 on MOT17
```

The supervision API is identical — no other code changes needed.
Decision: keep ByteTrack until after pool day to avoid pre-competition risk.

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

## v4c -- HUD Round 6: font scaling, tracking robustness, bbox visuals (DONE)

- `draw_strip.py` — `sf = max(1.0, w / 640)` scale factor; all row heights, font sizes, and
  widget calls multiply by `sf`; dynamic `strip_h` returned instead of constant;
  Zone C (duplicate ERR sparklines) removed from Row 4; STATE panel expanded to 55% width;
  font scale reduced to `_SFS=0.374`, line height `_SLH=12` for legibility
- `draw_video.py` — bright cyan reticle `(0,200,200)`; glow hairlines (3px dim + 1px bright
  double-pass); `arrowedLine` from target center toward frame center (green/amber);
  on-frame `X:+0.12 Y:-0.05  87%` text below bbox; wider 12px alignment bar at `h-14`; 
  deadband fill (15% green tint when aligned); extra `cv2.putText` track ID badge
- `draw.py` — `_RENDER_SCALE = 2.0`: renders at 2× then downscales to output size via
  `cv2.INTER_AREA` for crisp subpixel antialiasing on 1080p displays
- `config/tracker.yaml` — `track_buffer: 150` (5 s occlusion buffer), `min_hits: 3`
  (suppress turbidity spurious detections), `track_activation_threshold: 0.40`
- `tracking/bytetrack.py` — `track_activation_threshold` param exposed; `reset()` method added
- `tracker_node.py` — `track_activation_threshold` declared as ROS param; tracker + kalman
  reset on `classes` param change (flushes stale gate IDs when operator switches to flare)

## v4b -- Mission-control HUD v2 (DONE)

Instruments, active-class panel, and rich bounding-box annotators for the operator display.

- `draw.py` — upgraded to supervision's rich annotator suite:
  `RoundBoxAnnotator` (class colors), `PercentageBarAnnotator` (confidence bar),
  `BoxCornerAnnotator`, `LabelAnnotator` with track IDs, `HaloAnnotator` for primary target
- `draw.draw_depth_gauge()` — vertical 0–10 m depth slider that tracks `DuburiState.depth_m`
- `draw.draw_heading_tape()` — horizontal ±60° compass tape from `DuburiState.yaw_deg` with cardinals
- `draw.draw_classes_panel()` — shows configured class list; detected classes light up teal
- `detector_node.py` — publishes `/duburi/vision/<cam>/classes_filter` (std_msgs/String)
  on startup and on every live `classes` param change; any source that updates the param
  (CLI, DSL, mission) triggers a re-publish automatically
- `display_node.py` — subscribes `classes_filter`; prefers Kalman-smoothed `/tracks`
  detections for bbox display (stable IDs + smooth positions) over raw `/detections`;
  passes `track_ids` to supervision annotators for consistent per-track colors

Desk test:
```bash
ros2 run duburi_vision vision_display --ros-args \
    -p launch_pipeline:=true -p camera:=laptop \
    -p model:=yolov11n -p classes:=person
# CLASSES panel shows [PERSON], lights up teal on detection
# Depth gauge + heading tape appear when /duburi/state is publishing
# Live class switch: ros2 param set /duburi_detector classes gate,flare
```

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

## v4d -- Round 7 HUD Polish (DONE)

- `draw_widgets.py` — `_fill_rect()` helper uses `cv2.addWeighted` alpha blending;
  all widget fills (altimeter, needle gauge, confidence bar, battery, video progress)
  now semi-transparent so scale marks remain visible through the fill
- `draw_widgets.py` — `altimeter_depth`: 8m max scale; fill-before-ticks draw order
  (was ticks-then-fill, which hid all scale marks); 8m color zones (green<2m,
  accent<4.5m, amber<6.5m, red≥6.5m); tick step 1m; `fs_lbl=0.34` (larger readout)
- `draw_widgets.py` — `_FONT = FONT_HERSHEY_SIMPLEX` throughout (was DUPLEX);
  N cardinal highlighted with `C_TEXT` (others remain `C_DIM`)
- `draw_video.py` — `_SV: dict = {}` keyed by `round(sf,2)`;
  `_get_sv(sf)` creates LabelAnnotator with `text_scale=0.38*sf` (sf=2→0.76);
  `_FONT = FONT_HERSHEY_SIMPLEX`; semi-transparent pill behind offset label text;
  confidence pips (colored circle per bbox, green≥75%/amber≥50%/red<50%);
  all cv2.putText font sizes, arrow thickness, and alignment bar height scale with sf
- `draw_strip.py` — `altimeter_depth(max_depth=8.0)`;
  `depth_rate` param wired through `render_ui_strip` → `_draw_instruments_row`;
  STATE panel DEPTH row shows `↑`/`↓` rate when |rate|>0.02 m/s
- `draw.py` — `render_all` signature gains `depth_rate: float = 0.0`
- `utils/display_node.py` — `_depth_history: deque[tuple[float,float]]`; `_on_state`
  computes `_depth_rate` from first→last history entry; passes to `render_all`
- `README.md` + `detection/yolo.py` — YOLO 26 / YOLO26 references updated to YOLO11

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

## v4d — Lateral Alignment Fix (2026-05)

- **Bug fix**: `motion_vision.py` lateral control sign corrected.
  - Was: `lat_pct = -ex * kp_lat` (wrong negation; caused divergence away from target)
  - Now: `lat_pct = +ex * kp_lat`
  - Root cause: comment claimed "Ch6 > 1500 pushes LEFT" — contradicted by
    `motion_lateral.py` ground truth (Ch6 > 1500 = strafe RIGHT).
  - Verified by IBVS interaction matrix theory: positive image error → positive
    lateral velocity → Ch6 > 1500 (no negation needed).
- **Yaw convention confirmed correct**: `yaw_pct = -ex * kp_yaw` stays negated
  because Ch4 > 1500 = yaw LEFT (inverted stick convention).
- `move_and_see.py` docstring/code sync: `yaw=False` (was `True`).

## v4e — Display System Overhaul (2025–2026)

- **2× render upscale**: `_RENDER_SCALE = 2.0` in `utils/display_node.py` — native 640 px
  frame upscaled to 1280 px before draw calls; all font sizes scale with `sf = w/640`.
- **PIL TrueType fonts**: Replaced OpenCV Hershey fonts with PIL/Pillow TrueType for
  strip panel text. Font priority: IosevkaNerdFontMono → NotoSansMono → DejaVu.
- **Splash screen + auto-pause**: holds video at frame 0 with branded splash until
  first detection received; fades over 400 ms on ready.
- **`video` namespace**: `video_file:=<path>` launch arg sets `effective_cam='video'`
  so detection and display topics route correctly for replay debugging.
- **Strip panel additions**: real-time compass needle, depth gauge (8 m scale), heading
  tape, PERCEPTION/CLASSES/ALIGNMENT/STATE panels, correction arrow, sparklines.
- **Dashboard cleanup** (v4e-cleanup): dead `draw_detections` removed from `draw_video.py`;
  unused `battery_bar` + `C_HEADER` pruned from `draw_widgets.py`; splash font migrated
  to PIL TrueType for visual consistency; `_start_pipeline` source param fixed.

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
