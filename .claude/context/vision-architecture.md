# Vision Architecture (`mongla_vision`)

> **Backend note.** The vehicle is the SROT board (firmware Hengla) + a Raspberry Pi 5 with a
> Hailo-8. `lock_heading`, `move_*_dist`, `arc` and `style_yaw` are **refused** there, `ALT_HOLD`
> is not one of its modes, and the depth-setpoint vision axes are refused. Where this page shows
> an older idiom, the current contract is [`command-reference.md`](command-reference.md) and the
> legacy path is [`legacy-pixhawk-and-sitl.md`](legacy-pixhawk-and-sitl.md).

Authoritative design notes for the perception package. Mirrors the
`mongla_sensors` pattern: ABC + factory + per-source class + standalone
diagnostic node.

## File map

```
src/mongla_vision/mongla_vision/
  factory.py             # make_camera(name, **kw) + BUILDERS dict
  config.py              # CAMERA_PROFILES dict (mirrors config/cameras.yaml)
  draw.py                # cv2/supervision overlays — rich annotator suite + AUV instruments
                         #   draw_depth_gauge()   vertical depth slider (MonglaState.depth_m)
                         #   draw_heading_tape()  horizontal compass tape (MonglaState.yaw_deg)
                         #   draw_classes_panel() configured-class list; detected classes light up
  camera_node.py         # publish image_raw + camera_info
  detector_node.py       # subscribe image_raw -> detections + image_debug
  tracker_node.py        # subscribe detections -> tracks (Roboflow OC-SORT/ByteTrack + Kalman)
  vision_node.py         # in-process diag (cousin of sensors_node)
  cameras/
    camera.py            # Camera ABC (the only base in the tree)
    webcam.py            # cv2.VideoCapture wrapper (live webcam / USB cam)
    video_file.py        # cv2.VideoCapture(path) wrapper (pre-recorded video offline testing)
    ros_topic.py         # subscribes sensor_msgs/Image (Gazebo / BlueOS)
    {jetson,blueos,mavlink}_stub.py
  detection/
    detector.py          # Detector ABC + Detection dataclass
    yolo.py              # YoloDetector (Ultralytics YOLO11, yolov11n default)
    gpu.py               # select_device() -- fail-fast CUDA check
    messages.py          # Detection -> vision_msgs converters (+ array_to_detections)
  tracking/
    __init__.py          # exports Tracker, TrackedDetection, RoboflowTracker, ByteTrackWrapper, TrackKalmanSmoother
    tracker.py           # Tracker ABC + TrackedDetection dataclass (predicted=True/score=0.0 = coasted)
    roboflow_tracker.py  # Roboflow `trackers` OC-SORT/ByteTrack (DEFAULT); coasted boxes from tracked_objects
    bytetrack.py         # supervision.ByteTrack wrapper (legacy_bytetrack fallback)
    kalman.py            # PerTrackKalman + TrackKalmanSmoother (filterpy 4-state CV)
  preflight.py           # assert_vision_ready / wait_vision_state_ready
  web/
    mission_web_node.py  # `mission_web` -- browser mission console: SSE data + control
    dashboard_state.py   # pure (rclpy-free) helpers: snapshot/param-type/exclusivity/offset math
    static/{index.html,style.css,app.js}  # self-contained SPA (vanilla, no build/CDN)
  utils/
    check_pipeline.py    # `vision_check`        CLI -- topic-only smoke test
    check_thrust.py      # `vision_thrust_check` CLI -- detection -> RC echo
    check_tracker.py     # `tracker_check`       CLI -- tracking smoke test
  filters/PLAN.md        # v3 -- folded into tracker_node (Kalman in kalman.py)
  depth/
    __init__.py
    depth_estimation_node.py  # DepthEstimationNode: monocular vis_range (0=far, 1=close)
                              #   fallback: bbox-area proxy (no model needed)
                              #   model: Depth Anything V2-Small ONNX (364×364 NCHW)
                              #   EMA temporal smoothing (alpha=0.40) for stable estimates
                              #   subscribes /tracks when use_tracks=True (matches tracker ordering)
    models/
      depth_anything_v2_small.onnx   # DA V2-Small (not tracked in git — see .gitignore)
      README.md                      # placement, launch params, re-export instructions
      .gitignore                     # ignores *.onnx *.pt *.bin *.pth
config/
  cameras.yaml           # camera profiles
  detector.yaml          # model + class params
  tracker.yaml           # tracker_type (ocsort default) + association + Kalman thresholds (ROS params)
test/
  test_depth_estimation.py  # standalone 9-check test: onnxruntime, bbox fallback, ONNX inference
```

Naming rule: every file is named after the thing inside it. No `base.py`,
no `to_ros.py`, no `nodes/` or `viz/` subfolders. Per-user request,
`mongla_sensors` is NOT renamed to match.

## Topic contract

```
/mongla/vision/<cam>/image_raw        sensor_msgs/Image            (camera_node)
/mongla/vision/<cam>/camera_info      sensor_msgs/CameraInfo       (camera_node)
/mongla/vision/<cam>/detections       vision_msgs/Detection2DArray (detector_node)
/mongla/vision/<cam>/classes_filter   std_msgs/String              (detector_node, CSV class list)
/mongla/vision/<cam>/tracks           vision_msgs/Detection2DArray (tracker_node, optional)
/mongla/vision/<cam>/vis_range        std_msgs/Float32MultiArray   (depth_estimation_node, one float per detection, 0=far 1=close)
/mongla/vision/<cam>/vis_range_map    sensor_msgs/Image            (depth_estimation_node, float32 depth map, debug only)
/mongla/vision/<cam>/image_debug      sensor_msgs/Image            (detector_node, rate-limited)
```

`/classes_filter` is published once at `detector_node` startup with the initial `classes` param,
and re-published on every live `ros2 param set /mongla_detector_<camera> classes <...>` change. Any
consumer — `vision_display`, logging nodes, future HUD overlays — can subscribe to get the
current class filter without polling `ros2 param get`. This means class changes from CLI, DSL
(`mongla.set_classes()`), or mission code (`mongla.models(...)`) all propagate automatically.

### Mission console (`mission_web`) — HTTP surface, not a ROS topic

`mission_web` (`web/mission_web_node.py`) is a monitoring+control node, not part of the
mission path. It SUBSCRIBES `detections`, `vis_range`, `camera_info` per camera + latched
`/mongla/vision/active_camera` + `/mongla/state`, and POLLS the detector params it reflects
(`active_model`/`conf`/`models`/`paused`/**`classes`**) at 1 Hz via `get_parameters`. NOTE:
`classes` is polled, **not** read from `/classes_filter` — that topic is published VOLATILE
(depth 10), so a TRANSIENT_LOCAL sub gets nothing (durability mismatch) and a VOLATILE sub
misses the retained startup value on a late join → the console showed empty class chips.
Polling the `classes` param is join-order-proof. It WRITES control through the **exact DSL
surface** — `SetParameters` on `/mongla_detector_<cam>` and the latched `active_camera`
publish + pause-others/resume-target — so UI and DSL converge. Video is served by a
co-launched `web_video_server` (MJPEG of `image_debug`, `:8080`); the node never touches
image bytes. Browser routes (`:8090`): `GET /` + `/static/*` (SPA), `GET /events` (SSE
~12 Hz JSON snapshot), `POST /api/detector/<cam>/param {name,value}`,
`POST /api/active_camera {camera}`. Launch: `mission_web.launch.py` (see JETSON_SETUP §5
Option D) — `cameras:=both` (vision_dual) | `forward` | `downward` (single via
vision.launch.py; the node's `cameras` param is set to match so a single-camera console
shows one panel, no phantom). **Robust to a missing `web_video_server`** (apt pkg, not a
repo dep): the launch resolves it defensively and degrades to console-only with an
`apt install` hint rather than aborting the whole launch. Convergence is
one-directional-solid: **DSL→UI** always reflects. **UI→running-mission** is inherently
racy — a UI camera-switch mid-verb can be overridden by the next DSL verb, which steers by
its own `_live_camera`. Fine for a monitor; drive competition runs from the DSL and use the
console to watch/plan/tune.

`/tracks` uses the same message type as `/detections`. The difference:
- `Detection2D.id` is populated with a stable tracker integer ID (stringified; Roboflow OC-SORT by default)
- Bbox center (`cx`, `cy`) is Kalman-smoothed; jitter from YOLO NMS is filtered out
- Entries with `score=0.0` are Kalman-only predictions (detector missed that frame)

**The vision control loop reads `/detections` first; `/tracks` only fills a gap
when `vision.coast_s>0`.** `VisionState` (in `mongla_manager`) subscribes
`/mongla/vision/<cam>/detections` (primary, authoritative) + `camera_info`, and
computes `bbox_error()` in normalized pixels via `info_seen()`-gated scaling — so
a box visible on the operator HUD is a box the controller acts on. It **also**
subscribes `/tracks` as the **coast source**, but consults it ONLY when
`coast_s>0` AND no live detection matches this tick: it then steers on the
tracker's coasted (Kalman-predicted) box of the **locked target id**, at decaying
authority, for up to `coast_s` (the gap-bridging path — see `BUGS.md`
D10). **`coast_s=0` (default) ⇒ control reads raw `/detections` exactly as
before** — a live box always overrides a coast, and a coasted box is conf-exempt
only for the locked id. There is **no** `--tracking` flag and **no**
`vision.use_tracks` param: outside the opt-in coast, `/tracks` (Roboflow OC-SORT
ids + Kalman smoothing) feeds `vision_display` / analysis for the HUD only.
(`depth_estimation_node` has its own independent `use_tracks` param that merely
selects which topic *it* reads for detection ordering.)

## Dataflow (with tracker_node)

```
camera_node ──/image_raw──▶ detector_node ──/detections──┬──▶ tracker_node ──/tracks──▶ vision_display (HUD)
                                  │                       │                              │
                            /image_debug                  └──▶ VisionState ──▶ motion_vision
                           /classes_filter                     (reads /detections; /tracks
                                  │                             ONLY as coast source if coast_s>0) ◀── /tracks
                                  └──▶ vision_display (subscribes image_raw + detections + tracks +
                                       state + classes_filter; renders HUD overlay)
```

The control loop (`VisionState` → `motion_vision`) reads `/detections` as its
authoritative input; `/tracks` is a display / analysis convenience EXCEPT for the
opt-in gap-bridging coast (`vision.coast_s>0`), where it is consulted only to fill
a tick that has no live detection — never to override one.

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

### TensorRT engine (Pi FPS)

`_resolve_model_path` **prefers `<stem>.engine` over `<stem>.pt`** when both
sit in `models/`. On the Pi 5 raw PyTorch @640 is ~3-4 Hz
(inference-bound); a TensorRT FP16 engine is ~20-30 Hz (nano/small) / ~10-15 Hz
(medium). Confirm the fast path via the backend canary:

```
[YOLO ] backend=TensorRT engine  (gate_flare_medium_100ep.engine)
[YOLO ] backend=PyTorch .pt       (gate_flare_medium_100ep.pt)     ← fallback
```

Engines are **device + TRT/JetPack-version locked** — build them ON the Pi
(`ros2 run mongla_vision export_engine --all`, FP16, imgsz must match the
detector's `imgsz`), rebuild after a JetPack/TRT upgrade, and never commit them
(`*.engine` gitignored). A dev box without an engine falls back to `.pt`
transparently. Also set MAXN: `sudo nvpmodel -m 0 && sudo jetson_clocks`
(~2× alone; `bringup_check` warns if not set). The debug overlay is skipped
when no one subscribes to `image_debug` (`viewer:=false`).

### FPS ↔ control coupling

The vision loop runs at `VISION_LOOP_HZ` (20 Hz) but the detector may publish
far slower, so the same bbox is re-used for several ticks. `align_loop`/
`move_loop` **freshness-decay** the translational command (`_freshness(age_s)`:
full authority while fresh, linearly to zero by `VISION_FRESH_ZERO_S`, hard-zero
when blind) on **lat/fwd only** — yaw is a rate ArduSub bleeds, depth is its
hold. At healthy FPS the factor is 1.0 (no behaviour change); at low/variable
FPS it caps the per-frame over-drive (`Kp·e·T_frame`) that otherwise makes the
hull twitch on stale data. Raising FPS (TensorRT) is the primary fix; this is
the per-frame safety guard, complementary to the arrival brake (end-of-command).

## Visualization layers

`draw.render_all(frame, detections, *, ..., state, configured_classes, track_ids)` returns
`np.vstack([video_section, ui_strip])` — output height = `frame_h + 150 px`.

**Video section** — only visual overlays (no text panels, preserves operator view of the scene):
 1. Dashed center reticle + deadband rectangle
 2. Motion trails (`sv.TraceAnnotator`) — drawn before boxes so trails render behind
 3. All detections: `BoxAnnotator` (thin class-colored) + `BoxCornerAnnotator` (white brackets)
    + `TriangleAnnotator` (lock indicator) + `PercentageBarAnnotator` + `LabelAnnotator`
    (class name + optional `#track_id`)
 4. Primary target: thick ACCENT border + crosshair + offset arrow from frame center
 5. Red "STALE FRAME" banner across top if `healthy=False`

**UI strip (150 px below video)** — `_render_ui_strip()` builds on a dark `C_BG` surface:
 - Brand header: `● BRACU  DUBURI ●` centered; accent separator below
 - Left block: `[PERCEPTION]` → `[CLASSES]` → `[ALIGNMENT]` panels (SRC/FPS/DET/TGT status)
 - Right block (right-anchored):
   - `[STATE]` panel: DEPTH / YAW / MODE / BATT / ARMED
   - `[HEADING SRC]` panel: SRC label + ACTIVE/NO DATA status
   - Compass needle (radius 20 px): direction indicator, updates at 20 Hz
   - Depth gauge (22×72 px, 0–5 m scale): fill + indicator line, always visible with `?` when no data; updates at 20 Hz
 - Footer: full-width heading tape (±60°, cardinal marks, 3-digit readout above center)

**Real-time instruments**: `auv_manager_node` publishes `/mongla/state` at 20 Hz via
`_fast_state_tick()` (fresh AHRS2 yaw + depth, reusing cached armed/mode/battery). The slower
`telemetry_tick` at 2 Hz handles logging and armed/mode/battery cache updates. This keeps the
compass needle and depth bar latency under 50 ms even though a full telemetry log line only
prints on change.

`track_ids` is a `list[int|None]` parallel to `detections`; supervision annotators use it for
stable per-track coloring. `vision_display` passes Kalman-smoothed tracks from `/tracks` as the
detection list (and their IDs as `track_ids`) for smooth bboxes, falling back to raw
`/detections` when tracker is not running.

Every glyph answers a specific operator question. Every diagnostic state
is visible in one frame — that's how we let the user paste a screenshot
into a bug report.

## Model and class selection

### Model files

Drop `<stem>.pt` + `<stem>.yaml` pairs into `src/mongla_vision/models/`.
The YAML maps integer class ids to human names:

```yaml
# gate_v1.yaml
names:
  0: gate
```

The detector logs the full class table at startup. Pass the **stem** (no `.pt`):

```bash
ros2 launch mongla_vision vision.launch.py camera:=forward model:=gate_v1 classes:=gate
ros2 launch mongla_vision vision.launch.py camera:=forward model:=flare_v1 classes:=flare
ros2 launch mongla_vision vision.launch.py camera:=forward model:=gate_flare_v1 classes:=gate,flare
```

### Switching class filter live

The `classes` param is a post-inference allowlist — the model runs its full
forward pass; only matching boxes are published. Change it without restarting
(node = `/mongla_detector_<camera>`):

```bash
ros2 param set /mongla_detector_forward classes gate
ros2 param set /mongla_detector_forward classes "gate,flare"
```

### Offline testing with `video_file`

Run the full pipeline on a pre-recorded `.mp4` / `.avi`:

```bash
ros2 launch mongla_vision vision.launch.py camera:=forward \
    video_file:=/tmp/pool_run.mp4 model:=gate_v1 classes:=gate
```

`video_file` is one of the camera sources in `BUILDERS` (`factory.py`).
When `video_file:=` is non-empty in the launch file, `camera_node`
switches `source` to `'video_file'` automatically; `detector_node` and
downstream vision verbs see identical topics either way.

**Playback controls** (HUD window, active when `video_file:=` set):
Space=pause, →/←=±1 s, ↑/↓=±10 s, `.`/`,`=±1 frame (best while paused).

**Full workflow** — recording pool runs, running mission scripts against video,
tracker tuning, ROS2 service/topic control, sim+video setup:
→ [`video-testing.md`](video-testing.md)

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

## Vision verbs -- where the closed loop lives

The control loop owns thrust; the planner asks for an outcome. Vision
verbs are the bridge. The 2026-06 rewrite replaced the old 9-verb axis API
with **exactly two** pixel-native verbs, `vision_align` and `vision_move`,
backed by two loops in `mongla_control/motion_vision.py`:

| Loop (`motion_vision.py`) | Action verb | Facade (`vision_verbs.py`) | DSL (`vision_dsl.py`) |
|---------------------------|-------------|----------------------------|-----------------------|
| `align_loop` | `vision_align` | `VisionVerbs.vision_align` | `mongla.vision.align` |
| `move_loop`  | `vision_move`  | `VisionVerbs.vision_move`  | `mongla.vision.move`  |

```
mission script
  v
mongla.vision.align(target, yaw=0, lat=0)      # vision_dsl._VisionDSL
  v
/mongla/move action  (Move.Goal carries the vision_align / vision_move fields)
  v
auv_manager_node
  +-- _vision_state_for(camera)        -- lazy VisionState per camera
  +-- wait_vision_state_ready(...)     -- one-time preflight, polling-only
  +-- mongla.vision_align(...)         -- VisionVerbs mixin
        v
mongla_control.motion_vision.align_loop   (move_loop for vision_move)
  +-- reads VisionState.bbox_error(target)            -- normalized pixel error
  +-- 20 Hz  send_rc_override / send_rc_translation (lateral / yaw / forward)
  +-- 5  Hz  set_target_depth(setpoint += clamp(ey * kp_depth))  -- align depth axis
  +-- gain clamps every axis (hard max-speed cap)
  +-- returns Outcome code: ALIGNED / LOST / TIMEOUT / NO_CAMERA / ABORTED
```

Why the loop is in the manager, not the client:

- One MAVLink owner. Anything else risks two writers fighting for Ch4/5/6.
- Sub-100 ms feedback path. ActionClient -> manager -> Pixhawk RTT is
  one process hop; running the loop inside the manager removes the
  network/IPC jitter.
- Preflight is a polling check on an existing `VisionState`. No
  `spin_once` from inside an action callback, no second subscription set.

**`align_loop`** centres the target on any subset of `{lat, yaw, depth}`,
each axis carrying a signed pixel offset (`0` = centre). It exits ALIGNED
when every active axis stays within `err_px` for `align_stable_frames`
consecutive ticks. **`move_loop`** drives forward until the bbox fills
`fwd_fill` of the frame (metric `mode` = `area` / `width` / `height`),
optionally holding a lateral pixel offset (`maintain`); it never re-centres
yaw/depth (ArduSub holds depth, the heading lock holds yaw).

Search and recovery are **not** verbs: the mission DSL owns them via
`mongla.detected()` poll loops and the `fallback=` search function passed
to either verb (see [`client-and-dsl-api.md`](client-and-dsl-api.md)).
Model + class switching lives in `mongla_planner/model_context.py`
(`ClassRef` → `set_model` + `set_classes`) and runs before each goal.

These two verbs plus the standalone `fire` verb (no vision — direct ESP32
serial; `fire_channel` 1/2=torpedo, 3/4=dropper) are the entire vision
surface. `target_class` matching is case-insensitive.

**Loss handling:** a detection older than `_STALE_LIMIT_S` (1.0 s) counts
as "no target this tick". The loop coasts (neutral RC) until `lost_grace_s`
(default 1.0 s) elapses, then returns `LOST` so the DSL can run its
`fallback`; with `hold_through_loss=True` (set by the DSL when no fallback
is supplied) it instead coasts through the loss until `duration` expires.

### Downward camera contract

`vision_verbs.vision_align` applies one automatic adaptation for a
floor-facing camera:

```python
is_downward = camera in ('downward', 'sim_bottom')
depth_sign  = -1 if is_downward else +1     # negate the depth-axis correction
```

`depth_sign = -1` flips the depth-axis nudge so a target that grows in the
downward view (vehicle already close) does not command a further descent.
That is the engine's only orientation special-case — `vision_move` has
none. On a floor-facing camera, drive lateral centring off the `lat` axis
(`ex` → Ch6 strafe); yaw cannot be inferred from a bbox looking straight
down, so leave it off:

```python
# Strafe-centre a bin under the AUV on the downward camera
mongla.camera = 'downward'
mongla.vision.align('bin', lat=0, err=30, duration=20)
```

### Camera switching in a mission

`mongla.camera` is a sticky string attribute on `MonglaMission`. Assign it to switch which camera
all subsequent `mongla.vision.*` calls use:

```python
# Phase 1: gate with forward camera (default)
mongla.camera = 'forward'
mongla.vision.align('gate', yaw=0, lat=0, duration=15)
mongla.vision.move('gate', fwd=80, mode='area', gain=35, duration=20)

# Switch to downward camera for bin task
mongla.camera = 'downward'
mongla.target  = 'bin'
mongla.vision.align(lat=0, err=30, duration=20)   # target falls back to mongla.target='bin'
```

The camera name must match a running `camera_node` profile — verify with
`ros2 topic list | grep image_raw`.

Verifying the chain before pool day:

```
ros2 run mongla_vision vision_check                 # detector publishing?
ros2 run mongla_vision vision_thrust_check          # detection -> RC echo?
ros2 run mongla_planner mission demo_find_person    # full mission rehearsal
```

### Payload actuation (PayloadDriver)

`mongla_control/payload.py` — write-only ESP32-C3 USB serial driver. Fires torpedoes (ch 1/2)
and droppers (ch 3/4) by sending ASCII digit bytes `b'1'`..`b'4'` over USB CDC.

```
MonglaMission.fire(channel)
  v
Mongla.fire(fire_channel)            # 'fire' COMMANDS verb (command-scoped)
  v
Mongla._fire_payload(channel)        # raw helper; also for mission "align then fire"
  v
PayloadDriver.fire(channel)          # serial.write(bytes([0x30 + channel]))
  v
ESP32-C3 GPIO → relay → actuator
```

**Auto-detect at startup**: `auv_manager_node` scans Espressif/CH340 USB-by-id globs,
excludes the BNO085 port, and connects the first match. Startup banner:
```
[PAYLOAD] connected on /dev/serial/by-id/usb-Espressif_...
[PAYLOAD] not found — fire() calls will log-stub only
```

Check: `mongla.payload_ready` → `bool`.

The `fire` verb (and the raw `_fire_payload` helper) decides:
- `fire_channel > 0` → ESP32 serial (`PayloadDriver`)
- `fire_channel == 0` → log-only stub

There is **no** vision-fire verb. Missions compose vision with a fire call,
e.g. `if mongla.vision.align('torpedo_hole', yaw=0, lat=0, depth=0).ok: mongla.fire(1)`.
