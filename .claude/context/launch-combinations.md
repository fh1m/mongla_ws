# Launch Combinations — the "so we never fail" master reference

> Every launch / run command for the **tested competition path**, with all args and
> combinations. Copy-paste ready. Scope (per pool-day decision): **controls (all), object-detection
> vision, DSL, CLI, mission_web viewer.** The experimental estimators (`depth:=`, `distance:=`,
> optical-flow) are **off by default** and listed once at the end — do not enable them on a scored run
> unless you've tested them.

Source every shell first:
```bash
cd ~/Ros_workspaces/duburi_ws && source install/setup.bash
```

---

## 0. Preflight (always, start of session)

```bash
ros2 run duburi_manager bringup_check          # 12-section per-subsystem preflight (A–L)
ros2 run duburi_manager bringup_check --strict # any WARN -> non-zero (hard pre-mission gate)
ros2 run duburi_manager bringup_check --skip-mavlink   # skip the UDP 14550 probe (bench)
ros2 run duburi_vision  vision_check --camera forward --require-class gate  # topic health
```

---

## 1. Control node (`duburi_manager`) — the MAVLink brain

The single node that owns the autopilot link + `/duburi/move` action. One of these per session.

### 1a. Bare `start` (node defaults: `mode=auto`, `yaw_source=mavlink_ahrs`)
```bash
ros2 run duburi_manager start                              # auto-resolve link, bench/sim
ros2 run duburi_manager start --ros-args -p mode:=sim      # Gazebo / SITL
ros2 run duburi_manager start --ros-args -p mode:=pool     # Jetson on AUV, BlueOS pushes 14550
ros2 run duburi_manager start --ros-args -p mode:=desk     # Pixhawk USB via BlueOS
ros2 run duburi_manager start --ros-args -p mode:=laptop   # tether laptop on the switch
ros2 run duburi_manager start --ros-args -p debug:=true    # [MAV ...] frame trace
```

`yaw_source` (heading authority): `mavlink_ahrs` (bench/sim) · `bno085` (pool, no DVL) ·
`dvl` (pool + DVL heading+position) · `bno085_dvl` (BNO heading + DVL position):
```bash
ros2 run duburi_manager start --ros-args -p mode:=pool -p yaw_source:=bno085
```

### 1b. `bringup.launch.py` — control + optional vision + Foxglove (the operator launch)
Args: `mode yaw_source dvl_host dvl_port dvl_auto_connect vision camera model models active_model
classes conf imgsz max_det viewer foxglove foxglove_port`
```bash
# control only, BNO heading (pool default this launch: mode=pool, yaw_source=dvl)
ros2 launch duburi_manager bringup.launch.py mode:=pool yaw_source:=bno085
# control + a single-model forward detector, no HUD window (headless)
ros2 launch duburi_manager bringup.launch.py mode:=pool yaw_source:=bno085 \
    vision:=true camera:=forward model:=gate_rescue_repair classes:=gate,rescue,repair viewer:=false
# control + vision + Foxglove bridge (ws://<ip>:8765)
ros2 launch duburi_manager bringup.launch.py mode:=pool yaw_source:=bno085 vision:=true foxglove:=true
```

---

## 2. Object-detection vision — `vision.launch.py` (ONE camera)

Args: `camera device device_path width height fps video_file topic loop model models active_model
classes conf iou device_cls imgsz max_det paused debug_image_hz viewer tracking tracker_type
track_buffer min_hits max_predict depth depth_model distance pool_depth_m camera_focal_px hud_distance`

**Identity is the model STEM** (the `.pt`/`.engine` basename) — `model:=`, `models:=`, ClassRef,
`set_model('<stem>')` all match it.

```bash
# SINGLE camera, SINGLE model
ros2 launch duburi_vision vision.launch.py camera:=forward \
    model:=gate_rescue_repair classes:=gate,rescue,repair conf:=0.55

# SINGLE camera, port-stable by-path device (2 identical USB cams -> pin the port)
ros2 launch duburi_vision vision.launch.py camera:=forward \
    device_path:=/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1:1.0-video-index0 \
    model:=gate_rescue_repair classes:=gate,rescue,repair

# SINGLE camera, MULTI-model registry (switch active_model live from DSL/UI)
ros2 launch duburi_vision vision.launch.py camera:=forward \
    models:=gate_rescue_repair,slalom_red_pipe,torpedo_blood_hole \
    classes:=gate,rescue,repair,red_pipe,torpedo,blood,hole conf:=0.55

# headless (no cv2 HUD window) — pair with web_video_server / mission_web
ros2 launch duburi_vision vision.launch.py camera:=forward model:=gate_rescue_repair viewer:=false

# per-model conf override in a registry (name=conf, live-tunable too)
ros2 launch duburi_vision vision.launch.py camera:=forward \
    models:=gate_rescue_repair,torpedo_blood_hole \
    model_conf:=torpedo_blood_hole=0.65 conf:=0.50

# DATASET VIDEO (no hardware — drives the full camera->detector->image_debug chain)
ros2 launch duburi_vision vision.launch.py camera:=forward video_file:=/path/clip.mp4 loop:=true \
    model:=gate_rescue_repair classes:=gate,rescue,repair
```
Knobs: `imgsz:=640` (export-baked for `.engine`; re-scales `.pt` only) · `max_det:=100` ·
`conf:=` (whole-registry) · `paused:=true` (start not inferring) · `debug_image_hz:=12` (MJPEG
smoothness) · `tracker_type:=ocsort|bytetrack`.

---

## 3. Object-detection vision — `vision_dual.launch.py` (TWO cameras)

Args: `fwd_device dwn_device fwd_device_path dwn_device_path fwd_model fwd_models fwd_classes
dwn_model dwn_models dwn_classes fwd_conf dwn_conf fwd_model_conf dwn_model_conf device_cls imgsz
max_det paused debug_image_hz viewer tracking tracker_type fwd_video dwn_video fwd_loop dwn_loop`

```bash
# DOUBLE camera, one model each
ros2 launch duburi_vision vision_dual.launch.py \
    fwd_model:=gate_rescue_repair fwd_classes:=gate,rescue,repair \
    dwn_model:=bin_fire_blood   dwn_classes:=fire,blood

# DOUBLE camera, MULTI-model forward registry + single downward
ros2 launch duburi_vision vision_dual.launch.py \
    fwd_models:=gate_rescue_repair,slalom_red_pipe,torpedo_blood_hole \
    fwd_classes:=gate,rescue,repair,red_pipe,torpedo,blood,hole \
    dwn_model:=bin_fire_blood dwn_classes:=fire,blood

# DOUBLE camera, port-stable by-path (the 2-identical-cam case)
ros2 launch duburi_vision vision_dual.launch.py \
    fwd_device_path:=/dev/v4l/by-path/...-video-index0 \
    dwn_device_path:=/dev/v4l/by-path/...-video-index0 \
    fwd_model:=gate_rescue_repair dwn_model:=bin_fire_blood

# DOUBLE dataset video (no hardware)
ros2 launch duburi_vision vision_dual.launch.py \
    fwd_video:=/path/fwd.mp4 dwn_video:=/path/dwn.mp4 fwd_loop:=true dwn_loop:=true \
    fwd_model:=gate_rescue_repair dwn_model:=bin_fire_blood
```

---

## 4. Mission console — `mission_web.launch.py` ★ (pool-day browser surface)

One command: camera(s) + detector(s) + `web_video_server` (MJPEG) + the `mission_web` console
(SSE + control), auto-opens `http://localhost:8090`. Args: `cameras web_port video_port no_browser
paused debug_image_hz viewer fwd_* dwn_* imgsz max_det tracking`.

```bash
# DOUBLE camera (default cameras:=both -> vision_dual), one model each
ros2 launch duburi_vision mission_web.launch.py \
    fwd_model:=gate_rescue_repair fwd_classes:=gate,rescue,repair \
    dwn_model:=bin_fire_blood dwn_classes:=fire,blood

# DOUBLE camera, MULTI-model forward registry
ros2 launch duburi_vision mission_web.launch.py \
    fwd_models:=gate_rescue_repair,slalom_red_pipe,torpedo_blood_hole \
    fwd_classes:=gate,rescue,repair,red_pipe,torpedo,blood,hole dwn_model:=bin_fire_blood

# SINGLE forward (cameras:=forward -> vision.launch.py; console shows ONE panel, no phantom 2nd)
ros2 launch duburi_vision mission_web.launch.py cameras:=forward \
    fwd_model:=gate_rescue_repair fwd_classes:=gate,rescue,repair

# SINGLE downward, device override + own model
ros2 launch duburi_vision mission_web.launch.py cameras:=downward \
    dwn_device:=0 dwn_model:=bin_fire_blood dwn_classes:=fire,blood

# DATASET VIDEO (no hardware)
ros2 launch duburi_vision mission_web.launch.py fwd_video:=/path/fwd.mp4 dwn_video:=/path/dwn.mp4

# ports / no auto-open (headless box, forward the port yourself)
ros2 launch duburi_vision mission_web.launch.py web_port:=8090 video_port:=8080 no_browser:=true
```
- `cameras:=both|forward|downward`. A one-camera box **must** use `forward`/`downward` (else the
  absent 2nd `camera_node` crashes).
- Missing `web_video_server` (apt pkg, not a repo dep) degrades to console-only with an
  `apt install ros-humble-web-video-server` hint — it does **not** abort the launch.
- Everything the UI switches (camera / model / conf / classes / pause) writes the **same ROS surface
  the DSL writes**, so UI and a running DSL mission stay in lock-step.

---

## 5. CLI — direct `/duburi/move` verbs (`duburi` entry, needs a running control node)

```bash
ros2 run duburi_planner duburi arm
ros2 run duburi_planner duburi set_depth --target -0.5
ros2 run duburi_planner duburi turn --target 90              # absolute heading, dir auto
ros2 run duburi_planner duburi yaw_right --target 90
ros2 run duburi_planner duburi move_forward --duration 5 --gain 80
ros2 run duburi_planner duburi arc --duration 4 --gain 50 --yaw_rate_pct 30
ros2 run duburi_planner duburi lock_heading --target 0 --timeout 120
ros2 run duburi_planner duburi unlock_heading
ros2 run duburi_planner duburi calibrate_depth              # re-zero baro at surface (disarmed)
ros2 run duburi_planner duburi fire --channel 1             # torpedo(1/2)/dropper(3/4)
ros2 run duburi_planner duburi disarm
# vision verbs (need a detector running):
ros2 run duburi_planner duburi vision_align --camera forward --target_class gate \
    --axes yaw,lat --err_px 40 --gain 30 --duration 20
ros2 run duburi_planner duburi vision_move --camera forward --target_class gate \
    --fwd_fill 80 --mode area --gain 35 --duration 20
```
Full verb set: `arm disarm set_mode head stop mission_reset calibrate_depth surface pause
move_forward move_back move_left move_right style_roll style_yaw arc yaw_left yaw_right turn
set_depth lock_heading unlock_heading dvl_connect move_forward_dist move_back_dist
move_lateral_dist vision_align vision_move fire` (+ `calc_distance`, experimental).
Live gain tune (applies on the NEXT goal): `ros2 param set /duburi_manager vision.kp_yaw 80.0`.

---

## 6. Missions — the runner (`mission` entry)

```bash
ros2 run duburi_planner mission --list
# detected()-paradigm task chunks + full run:
ros2 run duburi_planner mission task_gate
ros2 run duburi_planner mission task_full_2026
# YASMIN FSM (per-task or full):
ros2 run duburi_planner mission fsm_full_2026
# pool-day practice:
ros2 run duburi_planner mission pool_day_practice
```
Available: `task_{gate,slalom,bin,torpedo,return}`, `task_full_2026`,
`fsm_{slalom,bin,torpedo,return,full_2026}`, `gate_flare_fsm`, `prequal_fsm`, `gate_then_bin_fsm`,
`gate_flare_autonomous`, `pool_day_{practice,torpedo}`, `robosub_{gate_rescue,prequal}`,
`demo_{arc,find_person,heading_lock,move_see,square,pursue,dual_camera}`.

---

## 7. Detection FPS (Jetson) — build TensorRT engines ON THE JETSON

```bash
sudo nvpmodel -m 0 && sudo jetson_clocks          # MAXN (~2x); bringup_check warns if not set
ros2 run duburi_vision export_engine --all        # <stem>.pt -> <stem>.engine (device+JetPack locked)
ros2 run duburi_vision export_engine --all --imgsz 640
```
Detector auto-prefers `<stem>.engine` over `<stem>.pt`. Keep each `<stem>.yaml` sidecar beside the
engine (class labels come from it). Live `conf`/`classes`/`max_det` still apply to an engine; only
`imgsz`/`half` are export-baked.

---

## 8. Experimental estimators — OFF by default, NOT for a scored run untested

```bash
# monocular depth proximity (vis_range) — depth:=true
ros2 launch duburi_vision vision.launch.py camera:=forward model:=gate_rescue_repair depth:=true
# DVL-free downward optical-flow distance (calc_distance) — distance:=true + hud_distance:=true
ros2 launch duburi_vision vision.launch.py camera:=downward distance:=true hud_distance:=true \
    pool_depth_m:=4.0
```
`calc_distance('stop')` now reports a **failure** (not a phantom 0.0 m) if the estimator node isn't
publishing — so a distance-gated move can't silently think it travelled 0 m. Still: prove it in
practice before trusting it on a scored task.
```
