# Video-File Testing Guide

Run the full vision pipeline on pre-recorded `.mp4` / `.avi` / `.mkv` files
instead of live hardware. Lets you iterate on model weights, class filters,
tracker tuning, and mission logic without pool time.

---

## Quick start

```bash
# Full pipeline (camera + detector + tracker + HUD viewer) on a recording:
ros2 launch mongla_vision vision.launch.py \
    video_file:=/path/to/pool_run.mp4 \
    classes:=gate \
    model:=gate_flare_medium_100ep

# Loop disabled — stop at end of file:
ros2 launch mongla_vision vision.launch.py \
    video_file:=/tmp/gate_run.mp4 classes:=gate loop:=false

# Use pretrained YOLO11 (person class) for desk testing without custom weights:
ros2 launch mongla_vision vision.launch.py \
    video_file:=/tmp/test.mp4 model:=yolov11n classes:=person
```

`video_file:=<path>` overrides the `camera:=` profile — the entire pipeline
(camera_node → detector_node → tracker_node → vision_display) sees identical
topics regardless of whether the source is live or recorded.

---

## Dual-camera video (`video.launch.py`) — gate clip + bin clip at once

`vision.launch.py` drives **one** camera. To rehearse a full mission that uses
**both** cameras — gate/slalom/torpedo on the forward camera, bin/drop on the
downward camera — use `video.launch.py`, a video-tuned preset over
`vision_dual.launch.py`:

```bash
# Forward = gate clip, downward = bin clip; both detectors LIVE, both loop:
ros2 launch mongla_vision video.launch.py \
    fwd_video:=/path/to/gate.mp4 \
    dwn_video:=/path/to/bin.mp4

# One camera only (forward), custom model/classes to match the clip:
ros2 launch mongla_vision video.launch.py \
    fwd_video:=/path/to/gate.mp4 \
    fwd_model:=gate_flare_medium_100ep fwd_classes:=gate,flare

# Single pass (no loop) for a clean one-shot mission-sequence run:
ros2 launch mongla_vision video.launch.py fwd_video:=/path/gate.mp4 loop:=false
```

The clips publish under `/mongla/vision/forward/*` and
`/mongla/vision/downward/*` exactly like live cameras, so `vision.align` /
`vision.move` and `mongla.detected('...', camera='downward')` work unchanged —
the manager resolves each camera's `VisionState` by name. Defaults vs
`vision_dual`: detectors start **live** (`paused:=false`) so verbs see
detections immediately, the HUD enables video playback controls + warm-up
splash, and both videos loop. `video.launch.py` requires at least one of
`fwd_video` / `dwn_video` (it errors clearly otherwise).

> `vision_dual.launch.py` itself also accepts `fwd_video:=` / `dwn_video:=`
> (plus `fwd_loop` / `dwn_loop`); `video.launch.py` is just the convenience
> preset with video-first defaults. Either runs a camera off a file while the
> other stays a live webcam — pass only the side you have a clip for.

> **Loop ↔ `detected()` interaction.** With `loop:=true`, a looping clip
> re-shows the target at EOF, which can re-trigger `detected()` acquisition
> mid-test (great for iterating verbs/gains, confusing for one-shot sequencing).
> Use `loop:=false` for a clean single-pass mission run.

> **Splash screen / auto-pause**: When `video_file:=` is set, the HUD holds the video
> at frame 0 and displays a branded splash overlay until the detector publishes its
> first detection (model fully loaded, first bbox seen). Once warm, the splash fades
> over 400 ms and playback proceeds at normal speed. This prevents missed-frame false
> readings during model warm-up.

---

## Playback controls (keyboard in the HUD window)

| Key          | Action                          |
|--------------|---------------------------------|
| `Space`      | Pause / Resume                  |
| `→` Right    | Seek +1 second forward          |
| `←` Left     | Seek −1 second back             |
| `↑` Up       | Seek +10 seconds forward        |
| `↓` Down     | Seek −10 seconds back           |
| `.` Period   | Step **+1 frame** forward (paused) |
| `,` Comma    | Step **−1 frame** back (paused) |
| `Q`          | Quit                            |

Frame-step is most useful while paused: press Space to freeze, then `,`/`.`
to advance frame-by-frame and inspect detection results on individual frames.

---

## ROS2 topics/services for programmatic control

The camera_node exposes these when `source=video_file`:

| Interface | Type | Description |
|-----------|------|-------------|
| `/mongla/vision/<cam>/video_pause` | `std_srvs/SetBool` service | `true`=pause, `false`=resume |
| `/mongla/vision/<cam>/video_seek_rel` | `std_msgs/Float32` topic | seek ±N seconds |
| `/mongla/vision/<cam>/video_seek_frame` | `std_msgs/Int32` topic | seek ±N frames |

```bash
# Pause / resume from CLI:
ros2 service call /mongla/vision/video/video_pause std_srvs/srv/SetBool "{data: true}"
ros2 service call /mongla/vision/video/video_pause std_srvs/srv/SetBool "{data: false}"

# Seek forward 5 seconds:
ros2 topic pub --once /mongla/vision/video/video_seek_rel std_msgs/msg/Float32 "{data: 5.0}"

# Step back 1 frame (useful while paused):
ros2 topic pub --once /mongla/vision/video/video_seek_frame std_msgs/msg/Int32 "{data: -1}"
```

---

## Workflow: testing a mission on recorded video

### 1. Record a pool run

On the AUV with the vision pipeline running:

```bash
# Record /mongla/vision/forward/image_raw to a bag:
ros2 bag record /mongla/vision/forward/image_raw \
    -o ~/bags/pool_run_$(date +%Y%m%d_%H%M)
```

Convert to a playable `.mp4` for offline use (optional — ROS bag replay is
also supported, see below):

```bash
# Install ros2_video_converter or use ffmpeg on the bag's extracted frames.
# Simplest: use the ros2bag → mp4 script in scripts/ (if present).
```

### 2. Replay through the pipeline

```bash
# Option A: mp4 via video_file source (simplest, full pipeline runs normally)
ros2 launch mongla_vision vision.launch.py \
    video_file:=/home/fh1m/bags/pool_run.mp4 \
    model:=gate_flare_medium_100ep \
    classes:=gate,flare \
    conf:=0.45 \
    loop:=false

# Option B: ROS2 bag replay (preserves original timestamps, no video_file needed)
ros2 bag play /home/fh1m/bags/pool_run_20250515 \
    --topics /mongla/vision/forward/image_raw \
    --rate 0.5   # play at half speed for easier inspection
# Then start detector+tracker separately:
ros2 run mongla_vision detector_node --ros-args \
    -p camera:=forward -p model_path:=gate_flare_medium_100ep -p classes:=gate
```

### 3. Iterate on model / classes without relaunching

```bash
# While video is running, switch class filter live:
ros2 param set /mongla_detector classes flare
ros2 param set /mongla_detector classes "gate,flare"

# Switch to a different model in the registry (if launched with models:=...):
ros2 param set /mongla_detector active_model combined
```

### 4. Run a mission script against the video replay

Since `video_file` publishes identical ROS topics to a live camera, mission
scripts work unchanged. Run the control stack in sim mode (no actual thrusters)
and let the mission react to detections from the recording:

```bash
# Terminal 1: SITL sim (no real Pixhawk needed — just provides /mongla/state)
sim_vehicle.py -L RATBeach -v ArduSub -f vectored_6dof --model=JSON \
    --out=udp:0.0.0.0:14550

# Terminal 2: control stack
ros2 run mongla_manager start --ros-args -p mode:=sim

# Terminal 3: video pipeline (replays pool run)
ros2 launch mongla_vision vision.launch.py \
    video_file:=/home/fh1m/bags/pool_run.mp4 \
    camera:=forward model:=gate_flare_medium_100ep classes:=gate

# Terminal 4: run the mission
ros2 run mongla_planner mission gate_flare_prequal
```

The mission calls `mongla.vision.align(target='gate', ...)` and
`mongla.vision.move(...)` against the recorded frames. Thrust commands go to
SITL (or are silently dropped if the AUV isn't armed), so the mission *logic*
is exercised without water.

> **⚠️ Video testing is OPEN-LOOP on vision — know what it does and does NOT
> validate.** The clip plays on its own timeline and does **not** react to
> commanded thrust. So it validates: detections firing on real imagery
> (model / conf / class tuning); mission **logic** — `detected()` transitions,
> verb dispatch, fallback search, model/class switching, the downward-camera
> handoff; and thrust **direction/sign** for a given bbox position. It does
> **NOT** validate closed-loop dynamics — convergence, overshoot, the
> `align(hold=)` station-keep, or arrival braking — because the scene won't move
> when the AUV does. `align`/`move` will chase a bbox that moves on the file's
> schedule and never truly converge unless the clip happens to show a centred
> target. For dynamics use Gazebo/SITL; for detection + sequencing use video.

---

## Tips

**Slow down for inspection**

```bash
# Launch at ¼ FPS for frame-by-frame inspection:
ros2 launch mongla_vision vision.launch.py \
    video_file:=/tmp/run.mp4 fps:=4 classes:=gate
# Then use Space + ,/. to step through detections one frame at a time.
```

**Check detection rate on a recording**

```bash
ros2 run mongla_vision vision_check \
    --camera video --duration 30 --require-class gate
```

**Tracker tuning on replay**

Adjust tracker params live while video plays and observe track stability:
```bash
ros2 param set /mongla_tracker track_buffer 60
ros2 param set /mongla_tracker min_hits 2
```

**video_file with the manager's launch file**

The `bringup.launch.py` in `mongla_manager` does not directly support
`video_file:=` (it is the control+vision path); use `vision.launch.py` (one
camera) or `video.launch.py` (forward+downward clips) from `mongla_vision`
instead for video-based testing, then start the manager separately.

---

## How it works

`VideoFileCamera` wraps `cv2.VideoCapture(path)`. It implements the same
`Camera` ABC as `WebcamCamera` and `RosTopicCamera`, so `camera_node` and
every downstream node are completely unaware of the source.

Playback state machine:
- **Playing**: `read()` returns the next decoded frame with `fresh=True`.
- **Paused**: `read()` returns the last good frame with `fresh=False`.
  `camera_node._capture_loop` sleeps 20 ms and skips publication; the HUD
  window keeps the last rendered frame visible.
- **Step** (`,`/`.` keys): sets a one-shot `_step_pending` flag. Next
  `read()` call ignores the paused flag for exactly one frame, returns
  `fresh=True`, then re-enters paused state.
- **Seek**: calls `cv2.VideoCapture.set(CAP_PROP_POS_FRAMES, target)`. A seek
  while paused also sets `_step_pending` so the seeked frame becomes visible
  immediately without requiring a resume.
- **Loop**: at EOF, rewinds to frame 0 automatically. Set `loop:=false` to
  stop instead.

All state is protected by a `threading.Lock` shared between the ROS callback
thread (service/topic handlers) and the capture loop thread.
