# Configuration guide

> Parameters on `auv_manager_node` that change how the stack behaves.
> Change them with `--ros-args -p name:=value` at startup, or live with
> `ros2 param set /duburi_manager <name> <value>` between goals.

---

## Core parameters

| Parameter          | Type     | Default          | Values / effect |
|--------------------|----------|------------------|-----------------|
| `mode`             | `string` | `auto`           | `auto` (probes UDP 14550 + Pixhawk USB → picks `pool`/`desk`/`sim`), or pin one of `sim`, `pool`, `laptop`, `desk` |
| `smooth_yaw`       | `bool`   | `false`          | `true` → `yaw_glide` (smootherstep sweep; reduces overshoot) |
| `smooth_translate` | `bool`   | `false`          | `true` → trapezoid thrust envelope; softer start/stop on Ch5/Ch6 |
| `yaw_source`       | `string` | `mavlink_ahrs`   | `mavlink_ahrs` (ArduSub AHRS2) or `bno085` (ESP32-C3 + BNO085) |
| `bno085_port`      | `string` | `auto`           | `auto` (probes `usb-Espressif*` → `ttyACM*`) or explicit path |
| `bno085_baud`      | `int`    | `115200`         | BNO085 USB CDC baud rate |

```bash
# Defaults (bang-bang yaw + constant thrust)
ros2 run duburi_manager start

# Smoother yaw only
ros2 run duburi_manager start --ros-args -p smooth_yaw:=true

# Both smoothed, pool mode
ros2 run duburi_manager start --ros-args \
    -p mode:=pool -p smooth_yaw:=true -p smooth_translate:=true
```

A YAML preset lives at
[src/duburi_manager/config/modes.yaml](../src/duburi_manager/config/modes.yaml):

```bash
ros2 run duburi_manager start \
    --ros-args --params-file src/duburi_manager/config/modes.yaml
```

---

## Vision parameters (`vision.*`)

All resolved at goal-dispatch time in this priority order:
1. Value the goal supplied (per-call kwarg wins)
2. Live `ros2 param set /duburi_manager vision.X Y` value
3. Hardcoded default in `COMMANDS` registry

```bash
# Tune live between goals (no restart)
ros2 param set /duburi_manager vision.kp_yaw 80.0
ros2 param set /duburi_manager vision.deadband 0.08
ros2 param set /duburi_manager vision.target_bbox_h_frac 0.55
```

Full defaults live in
[src/duburi_manager/config/vision_tunables.yaml](../src/duburi_manager/config/vision_tunables.yaml).

| Param                  | Default | Effect |
|------------------------|---------|--------|
| `vision.kp_yaw`        | 60.0    | Yaw P-gain (Ch4 percent per unit horizontal error). Raise if slow, lower if oscillating. |
| `vision.kp_lat`        | 60.0    | Lateral P-gain (Ch6). Same tuning guidance. |
| `vision.kp_depth`      | 0.05    | Depth nudge per unit vertical error per tick. Small — depth setpoints accumulate. |
| `vision.kp_forward`    | 200.0   | Forward P-gain (Ch5). Large because the distance error is itself small (< 0.1). |
| `vision.deadband`      | 0.18    | Settle tolerance (0..1). 0.18 is loose (webcam). Tighten to 0.08–0.10 for pool. |
| `vision.target_bbox_h_frac` | 0.30 | Stop-distance threshold. 0.30 = target fills ~30% of frame height. Raise to 0.5–0.6 for getting closer. |
| `vision.stale_after`   | 1.5     | Seconds before a detection is treated as lost. Drop to 0.6–0.8 for clean pool cameras. |
| `vision.on_lost`       | `fail`  | `fail` (abort on lost target) or `hold` (pause thrust, keep waiting). |
| `vision.depth_anchor_frac` | 0.5 | Which vertical point on the bbox to align to the image centre. 0=top, 0.5=centre, 1=bottom. Use **0.2** for tall objects (person standing, pole) to avoid depth-controller stall. |
| `vision.lock_mode`     | `settle` | When to exit the vision loop. `settle` (exit when centred), `follow` (track until duration), `pursue` (forward-only until target fills frame). |
| `vision.distance_metric` | `height` | How to measure target size from bbox. `height` (h_frac), `area` (√(h×w)), `diagonal` (√(h²+w²)/√2). |
| `vision.acquire_yaw_rate_pct` | 22.0 | Yaw stick percent during a `vision.find` sweep. |
| `vision.acquire_gain`  | 25.0    | Forward thrust during a `vision.find` sweep. |

> **Yaw-mode note:** ArduSub honours `SET_ATTITUDE_TARGET` only in
> `ALT_HOLD` / `POSHOLD` / `GUIDED`. The node auto-engages `ALT_HOLD`
> if you call `yaw_left`/`yaw_right` from `MANUAL` or `STABILIZE`.

---

## Yaw source — `duburi_sensors`

ArduSub's onboard AHRS is the default and works fine in sim. For pool
runs where the Pixhawk compass is noisy near aluminium frames or
thrusters, swap in an external source without touching any
control-side code. The package is **sensor-only** — no fusion, no
fallback, no mid-run switching.

### Architecture

```
+-----------------------------+         +------------------------+
|   auv_manager_node          |         |   sensors_node         |
|   (control + ActionServer)  |         |   (diagnostic only)    |
+-----------------------------+         +------------------------+
              |                                    |
              | yaw_source param                   |
              v                                    v
        +-----------------------------------------------+
        |   make_yaw_source(name, **kw)  (factory)      |
        +-----------------------------------------------+
          |              |              |             |
          v              v              v             v
   MavlinkAhrs   BNO085Source    DVLSource (stub)   WitMotion (stub)
   (default)     (USB CDC JSON)  raises NotImpl     raises NotImpl
```

Every source: `read_yaw()` → degrees in `[0, 360)`, Earth-referenced
(magnetic north, +CW from above).

### Switching source

```bash
# Default — ArduSub onboard AHRS
ros2 run duburi_manager start_node

# External BNO085 (auto-probes port)
ros2 run duburi_manager start_node --ros-args -p yaw_source:=bno085

# Pin a specific port
ros2 run duburi_manager start_node \
    --ros-args -p yaw_source:=bno085 \
               -p bno085_port:=/dev/ttyACM0 \
               -p bno085_baud:=115200
```

If the source fails to initialise the node **fails loudly at startup** — no silent fallback.

### Bootstrap calibration (BNO085 only)

The BNO085 runs `SH2_GAME_ROTATION_VECTOR` — gyro + accelerometer,
magnetometer **disabled** (immune to thruster + hull interference).
The chip's "yaw=0" is whatever direction it was pointing at boot, so
one-shot calibration fixes the offset:

```
offset_deg = pixhawk_yaw  -  bno_raw_yaw    # captured once, locked
earth_yaw  = (bno_raw + offset_deg) mod 360
```

- Pixhawk mag is read **once** at startup, then never again.
- Calibrate at the surface, AUV held level, away from large metal.
- Drift: ~0.5°/min in steady conditions — fine for 2–3 min missions.
- Startup banner prints: `Yaw source: BNO085 (/dev/ttyACM0)  Earth-ref offset: +124.30°`
- To re-zero: restart the manager.

Test the calibration path without launching the full manager:

```bash
ros2 run duburi_sensors sensors_node --ros-args \
    -p yaw_source:=bno085 -p calibrate:=true \
    -p bno085_port:=/dev/ttyACM0
```

### Standalone diagnostic node

```bash
# AHRS via MAVLink (stop the manager first — UDP port collision)
ros2 run duburi_sensors sensors_node

# BNO085 only (no MAVLink at all)
ros2 run duburi_sensors sensors_node \
    --ros-args -p yaw_source:=bno085 -p bno085_port:=/dev/ttyACM0
```

Output every 0.5 s: `[INFO] [SENSOR] yaw=123.45°  healthy=True  rx_hz=49.8  total=1240`

`rx_hz` for a healthy BNO085 should sit at ~50 Hz. `STALE` = wire
broken, firmware crashed, or port held by another process.

### Adding a new source

1. Write `src/duburi_sensors/duburi_sensors/sources/my_sensor.py` subclassing `YawSource`.
2. Add builder entry to `_BUILDERS` in `factory.py`.
3. Document wire format in `src/duburi_sensors/firmware/<sensor>.md`.

No changes to `duburi_control` or `duburi_manager`.

### BNO085 firmware

MCU-side contract (JSON-line over USB CDC, 115200 baud, 50 Hz) and
reference Arduino sketch:
[src/duburi_sensors/firmware/esp32c3_bno085.md](../src/duburi_sensors/firmware/esp32c3_bno085.md).
Smoke-test: `cat /dev/ttyACM0` — if no JSON appears, ROS2 won't see it either.

---

## Vision pipeline — `duburi_vision`

Camera factory + YOLO26 detector + six `vision_*` verbs on
`/duburi/move`. The closed loop runs **inside the manager** — vision
and control share the same MAVLink owner and never fight for thrust.

### Architecture

```
+------------------------+     +-------------------------+     +-----------------------+
|  camera_node           | --> |  detector_node          | --> |  auv_manager_node     |
|  Camera factory:       |     |  YoloDetector (YOLO26)  |     |  VisionState pool     |
|  webcam / ros_topic /  |     |  + draw.render_all()    |     |  + motion_vision loop |
|  jetson / blueos /     |     |  -> Detection2DArray    |     |  RC Ch4/5/6 + depth   |
|  mavlink (stubs)       |     |  -> image_debug         |     +-----------------------+
+------------------------+     +-------------------------+              ^
                                                                        |
                                                       /duburi/move (vision_* verbs)
```

### Topics

| Topic | Type | Notes |
|-------|------|-------|
| `/duburi/vision/<cam>/image_raw`    | `sensor_msgs/Image`            | Camera source rate |
| `/duburi/vision/<cam>/camera_info` | `sensor_msgs/CameraInfo`       | Size only (no calibration yet) |
| `/duburi/vision/<cam>/detections`  | `vision_msgs/Detection2DArray` | One row per detection |
| `/duburi/vision/<cam>/image_debug` | `sensor_msgs/Image`            | Rate-limited overlay (5 Hz) |

### GPU contract

`detector_node` logs on startup:
```
[VIS ] using cuda:0 (NVIDIA GeForce RTX 2060)  torch=2.11.0+cu128  cuda=12.8
```
`device:=cuda:0` is fail-fast — broken CUDA raises on init. Use
`device:=cpu` to opt out, `device:=auto` for CI.

### Visual cues in `image_debug`

| Glyph | Question answered |
|-------|------------------|
| Top-left status badge (src/fps/dev/det) | Is the camera streaming? Which device? |
| Dashed centre reticle | Operator reference frame |
| Light-gray boxes + labels | All detections |
| Thick amber box + filled corners + crosshair | Which target is being chased |
| Arrow centre → target | Which way must the AUV rotate/strafe |
| Bottom-left alignment readout (err_x, err_y) | How well aligned is the AUV (-1..+1) |
| Red full-width "STALE FRAME" banner | Source unhealthy |

### Vision verbs

| Verb | Axes | Use case |
|------|------|---------|
| `vision_align_yaw`     | Ch4 only         | Centre horizontally (yaw) |
| `vision_align_lat`     | Ch6 only         | Centre horizontally (strafe) |
| `vision_align_depth`   | Depth setpoint   | Centre vertically |
| `vision_hold_distance` | Ch5 only         | Maintain standoff distance |
| `vision_align_3d`      | Any subset of Ch4/Ch5/Ch6 + depth | Multi-axis simultaneous hold |
| `vision_acquire`       | Sweep verb       | Rotate/move until target detected |

Full param docs: [`.claude/context/command-reference.md`](../../../.claude/context/command-reference.md).

### Quickstart

```bash
# Laptop webcam + YOLO26 + rqt viewer
ros2 launch duburi_vision webcam_demo.launch.py

# Subscribe to a Gazebo image topic
ros2 launch duburi_vision sim_demo.launch.py topic:=/your/image/topic

# CPU fallback (no CUDA)
ros2 launch duburi_vision webcam_demo.launch.py cls_device:=cpu
```

### Pipeline diagnostics

```bash
# Topic-only health probe (no thrust, no manager)
ros2 run duburi_vision vision_check --camera laptop --require-class person

# Detection -> RC echo (watch [RC] Yaw on the manager)
ros2 run duburi_vision vision_thrust_check --camera laptop --duration 4
```

Full vision architecture: [`.claude/context/vision-architecture.md`](../../../.claude/context/vision-architecture.md).
