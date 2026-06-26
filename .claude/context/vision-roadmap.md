# Vision Roadmap

Phased plan for `duburi_vision` and its planner integration. Phases v1–v4f
and the **v5 two-verb vision rewrite** are shipped; v6 (real-vehicle camera
sources) lands in already-named placeholder files so the diff stays small at
each step.

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
- Launch graph — `tracker_node` runs behind `with_tracking:=true` (default on) in
  `cameras_.launch.py` / `sim_demo`; it consumes `/detections` and republishes `/tracks`
- `/tracks` feeds the **HUD / depth overlay only**. The vision control loop reads
  `/detections` directly, so there is **no** per-goal `tracking` field and **no**
  `vision.use_tracks` manager param (both removed in the v5 two-verb rewrite)

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
  in-band fill (15% green tint when aligned); extra `cv2.putText` track ID badge
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

> **Superseded by the v5 two-verb rewrite (below).** v4 originally shipped a
> multi-verb pixel API (`vision_align_yaw/lat/depth`, `vision_align_3d`,
> `vision_hold_distance`, `vision_acquire`, ...) plus the `Move.action` knobs
> `deadband` / `target_bbox_h_frac` / `visual_pid` / `on_lost`. Those verbs and
> knobs were removed in 2026-06. The surviving infrastructure (which the two
> verbs reuse) is listed here.

Pieces that remain (re-pointed at the two verbs):

- `duburi_control/motion_vision.py` owns the closed-loop engine — now the two
  loops `align_loop` (multi-axis P @ 20 Hz RC + 5 Hz depth) and `move_loop`
  (drive-forward-to-fill).
- `duburi_manager/vision_state.py` holds the per-camera subscriber pool and
  exposes `bbox_error()`, `image_size()`, `info_seen()`.
- The closed loop still runs INSIDE `auv_manager_node` (the single MAVLink
  owner) so latency stays bounded and no second process fights for thrust.

CLI utilities for verifying the pipeline before touching the planner:

- `ros2 run duburi_vision vision_check` -- pure topic probe; reports
  image_raw rate, camera_info presence, detections rate, classes seen.
- `ros2 run duburi_vision vision_thrust_check` -- sends one `vision_align`
  goal to `/duburi/move` and reports the result; pair with `[RC   ] Yaw:NNN`
  lines in the manager log to confirm the chain closed end-to-end.

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

## v6 -- Real-vehicle camera sources

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
- **Yaw convention corrected (2026-06, pool-verified)**: `yaw_pct = +ex * kp_yaw`
  (NO negation -- same polarity as the lateral axis above). The earlier
  "confirmed correct, stays negated" claim was a reasoning-confirmation derived
  from the "Ch4 > 1500 = yaw LEFT" label, not an empirical toward-target test;
  in the water the `-ex` negation drove the AUV *away* from the target. The
  working lateral axis and `heading_lock` (which derives Ch4 sign from heading
  math) both confirm the un-negated sign.
- `demo_move_see.py` (formerly `move_and_see.py`) docstring/code sync: `yaw=False` (was `True`).

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

## v4f — vis_range Depth Pipeline (DONE — 2026-05)

End-to-end monocular proximity estimate integrated into vision stack:

- **`depth/depth_estimation_node.py`** — `DepthEstimationNode`; dual mode:
  - *fallback* (no model): bbox-area proxy `sqrt(w_frac * h_frac)` — zero-cost
  - *ONNX*: Depth Anything V2-Small (364×364 NCHW, ImageNet-normalised, inverted+normalised)
  - EMA temporal smoothing (alpha=0.40, reset on detection count change) for stability
  - `use_tracks` param: subscribes `/tracks` when True (keeps parallel to tracker ordering)
  - Throttled `[VISRNG]` log every 2 s
  - `publish_depth_map: True` in launch → `/vis_range_map` Image topic for debug overlay
- **`depth/models/`** — model storage folder; `.gitignore` excludes binaries; `README.md`
  documents placement + `optimum-cli` re-export; `depth_anything_v2_small.onnx` placed here
- **`cameras_.launch.py`** — `depth:=true` + `depth_model:=<path>` args launch the node
- **`draw_video.py`** — vis_range overlay on bboxes:
  - Bbox border color: blue=far → green=mid → red=close (via `_depth_color()`)
  - Label suffix: `~0.72 CLOSE` qualitative label (via `_vr_label()`)
  - Depth map inset top-right corner (TURBO colormap), shown only when `depth_map_bgr` passed
- **`draw_strip.py`** — vis_range in dashboard strip:
  - VIS_R row in STATE panel: shows `0.72 CLOSE` (green/amber/dim by threshold)
  - PROX bar in Zone D above altimeter: proportional fill, color-coded
  - Row heights reduced ~16% for tighter layout; font scales adjusted
- **`utils/display_node.py`** — vis_range integration:
  - Subscribes `/vis_range` (Float32MultiArray) + `/vis_range_map` (Image, TURBO colourised)
  - D-key toggle: depth map inset shown only on keypress → saves frame copy + resize at 30 Hz
  - `depth` health indicator in pipeline health row
  - Index-safe coordinate match for `primary_vr` (handles post-scale-dets identity break)
- **`test/test_depth_estimation.py`** — standalone 9-check test (no ROS env needed):
  - `onnxruntime` importable, `bbox_area_fallback` correctness, ONNX session load, inference,
    shape validation, finite values, normalised range [0, 1]
  - Run: `python src/duburi_vision/test/test_depth_estimation.py [model_path]`

Launch with depth:
```bash
ros2 launch duburi_vision cameras_.launch.py depth:=true \
    depth_model:=/path/to/duburi_vision/depth/models/depth_anything_v2_small.onnx
```

Press **D** in the display window to toggle the depth map inset on/off.

## v5 -- Two-verb vision rewrite (vision_align + vision_move) (DONE — 2026-06)

The v4 multi-verb pixel API collapsed into **exactly two** mission verbs, both
pixel-native and recover-don't-fail. Engine: `motion_vision.align_loop` /
`move_loop`; DSL: `vision_dsl.py`; FSM wrappers: `states/vision.py`.

- **`vision_align`** — `duburi.vision.align(target, *, lat=None, yaw=None,
  depth=None, err=40, duration=20, gain=30, fallback=None, camera=None)`.
  Centre the target on the named axes; each of `lat`/`yaw`/`depth` is `None`
  (axis off) or a **signed pixel offset** from centre (`0` = centre). At least
  one axis required.
- **`vision_move`** — `duburi.vision.move(target, *, fwd=95, mode='area',
  maintain=None, hold=None, err=40, duration=20, gain=30, fallback=None,
  camera=None)`. Drive forward until the bbox fills `fwd`% of the frame
  (`mode` = area/width/height). `maintain` holds a ±px lateral offset; `hold`
  station-keeps once reached. Never re-centres yaw/depth.
- `gain` is a **hard max-speed cap** (% thrust), not a target speed — the
  P-controller output is clamped to it.
- **Never-fail contract:** the server always returns `success=True`; the outcome
  rides in `Move.Result.final_value` (`ALIGNED`=0, `LOST`=1, `TIMEOUT`=2,
  `NO_CAMERA`=3, `ABORTED`=4). The DSL returns a `VisionResult` (truthy only on
  `ALIGNED`; a server/setup error surfaces as non-fatal `FAILED`) and never
  aborts a mission.
- **`fallback`** = mission-authored `fn(duburi[, should_stop])` search run on a
  real target loss; the verb re-enters within the same `duration` budget.
- FSM states: `VisionSearchState` (open-loop search) / `VisionAlignState` /
  `VisionMoveState`.

Removed in this pass: verbs `vision_align_yaw/lat/depth`, `vision_align_3d`,
`vision_hold_distance`, `vision_lock_fire`, `vision_acquire`, `look_around`; DSL
`find/home/turn/slide/hover/approach/track/scan/hold`; `Move.action` knobs
`deadband`, `lock_mode`, `distance_metric`, `target_bbox_h_frac`, `visual_pid`,
`on_lost` (and `stale_after`, kept only on `detected()`); plus the `--tracking`
flag (control reads `/detections`; `/tracks` is HUD-only).

New `/duburi_manager` ROS params (`vision_tunables.py`): `vision.kp_lat=60`,
`vision.kp_yaw=60`, `vision.kp_depth=0.05`, `vision.kp_forward=200`,
`vision.lost_grace_s=1.0`, `vision.frame_fill_default=95`,
`vision.align_stable_frames=3`.

**Harmony fixes shipped with the rewrite:**

- **Heading-lock release** — `align_loop` / `move_loop` honour `release_yaw`:
  when the background heading lock owns Ch4, the verbs leave yaw released
  (`send_rc_translation`) instead of racing the lock's 20 Hz Ch4 stream.
- **Surface RLock** — `Duburi.lock` is a reentrant `threading.RLock`, so
  `surface()` (which nests `set_depth()` in a second command scope on the same
  thread) no longer self-deadlocks (the original P0 bug).
- **NO_CAMERA gating** — both loops return `NO_CAMERA` until `info_seen()` is
  true, so the controller never steers on a mis-scaled pixel error before
  `camera_info` is published.
- **DSL never-die** — every server/setup exception is caught in
  `_orchestrate` and turned into a non-fatal `FAILED`; even a raising
  `fallback` can't kill the mission.

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
