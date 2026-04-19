# BRACU DUBURI — AUV ROS2 Control Stack

> **Team**: BRACU Duburi, Bangladesh | [bracuduburi.com/auv/4.2](https://bracuduburi.com/auv/4.2)
> **Competitions**: RoboSub (8th 2025, 2nd 2023), SAUVC
> **This codebase**: `~/Ros_workspaces/duburi_ws` (ROS2 Humble, Ubuntu 22.04 in distrobox)

> **Precedence note for agents:** if anything below contradicts the
> actual package layout in `src/`, the package layout wins. The
> canonical hardware reference is
> [`.claude/context/vehicle-spec.md`](.claude/context/vehicle-spec.md);
> tracked code bugs are in
> [`.claude/context/known-issues.md`](.claude/context/known-issues.md).
> Some legacy `.claude/context/*.md` files (notably `proven-patterns.md`)
> describe historical 2023/2025 codebases, not this workspace.

---

## 1. Hardware Overview (BRACU Duburi 4.2)

> Full spec lives in [`.claude/context/vehicle-spec.md`](.claude/context/vehicle-spec.md). Short table here.

| Component             | Spec                                                         |
|-----------------------|--------------------------------------------------------------|
| Hull                  | Octagonal, **Marine 5083 aluminum**, in-house                |
| Frame type (ArduSub)  | `vectored_6dof` (8× T200) — same as BlueROV2 Heavy           |
| Flight controller     | Pixhawk 2.4.8 running ArduSub 4.x                            |
| Companion             | Raspberry Pi running BlueOS                                  |
| Main SBC              | Nvidia Jetson Orin Nano (all ROS2 nodes live here)           |
| Depth sensor          | Bar30 (read via ArduSub `AHRS2.altitude`)                    |
| External IMU          | **ESP32-C3 + BNO085** over USB CDC, opt-in via `yaw_source`  |
| DVL                   | Nortek Nucleus1000 @ `192.168.2.201` — **stub only**         |
| Cameras               | Blue Robotics Low-Light HD USB (forward + downward)          |
| Tether                | FathomX power-over-Ethernet                                  |
| Power                 | Dual LiPo (propulsion + compute on isolated rails)           |
| Payload               | Slingshot torpedo, aluminum grabber (current-sensed), solenoid dropper |

> **Sim proxy:** Gazebo runs the BlueROV2 Heavy model because it shares the `vectored_6dof` frame. Hull shape and exact mass differ; control behavior matches.

> **Why no VectorNav**: TDR Appendix A lists VN200; we use BNO085 instead — see `vehicle-spec.md` §"Why BNO085 instead of the TDR's VectorNav VN200".

---

## 2. Network Topology (AUV Internal Ethernet)

```
[Onboard Ethernet Switch]
       ├── Jetson Orin Nano  → 192.168.2.69   (static, ROS2 host, UDP 14550 listener)
       ├── Raspberry Pi 4B   → 192.168.2.1    (BlueOS — MAVLink router, web UI)
       │      Gateway         → 192.168.2.2
       ├── DVL Nucleus1000   → 192.168.2.201  (driver TODO)
       └── Pixhawk 2.4.8     → via BlueOS over USB

MAVLink endpoint (configured in BlueOS web UI):
  Name: "inspector"  |  Type: UDP Client
  IP: 192.168.2.69 (Jetson)  |  Port: 14550

Ground Station → Remote Desktop / SSH to Jetson (192.168.2.69)
              → BlueOS UI via http://192.168.2.1
```

Connection strings live in `src/duburi_manager/duburi_manager/connection_config.py` under `PROFILES`. Default for every profile is `udpin:0.0.0.0:14550` (Jetson is the listener; BlueOS pushes to it).

---

## 3. Operating Modes

The codebase exposes four `mode:=` profiles via `auv_manager_node`. Default is **`sim`** so development works immediately.

| `mode:=`   | Connection string         | Use case                                         |
|------------|---------------------------|--------------------------------------------------|
| `sim`      | `udpin:0.0.0.0:14550`     | Docker dev + Gazebo SITL (ArduSub `--out` to us) |
| `pool`     | `udpin:0.0.0.0:14550`     | Pool testing — Jetson on AUV, BlueOS pushes      |
| `laptop`   | `udpin:0.0.0.0:14550`     | Tether laptop on the switch instead of Jetson    |
| `desk`     | `udpin:0.0.0.0:14550`     | Pixhawk plugged directly via USB through BlueOS  |

> All four profiles use the same listener line. The difference is documentation + the printed startup banner / sanity hints. There is **no** `HARDWARE` mode.

SIM startup commands (run before ROS2 nodes):

```bash
# Terminal 1: ArduSub SITL
sim_vehicle.py -L RATBeach -v ArduSub -f vectored_6dof --model=JSON \
  --out=udp:0.0.0.0:14550 --out=udp:127.0.0.1:14551 --console

# Terminal 2: Gazebo (BlueROV2 Heavy world — sim proxy for Duburi 4.2)
cd ~/Ros_workspaces/colcon_ws
gz sim -v 3 -r src/bluerov2_gz/worlds/bluerov2_underwater.world
```

---

## 4. Software Architecture

### 4.1 Package Map (real, today)

```
duburi_ws/src/
├── duburi_interfaces/    # ROS2 message + action defs
│   ├── action/Move.action        # single dispatcher action
│   └── msg/DuburiState.msg       # typed state snapshot for /duburi/state
├── duburi_control/       # MAVLink layer + Duburi facade
│   └── duburi_control/
│       ├── pixhawk.py            # Pixhawk class — arm / mode / RC / setpoints / AHRS2
│       ├── commands.py           # COMMANDS registry (single source of truth)
│       ├── motion_profiles.py    # smoothstep / smootherstep / trapezoid_ramp
│       ├── motion_yaw.py         # yaw_snap + yaw_glide (SET_ATTITUDE_TARGET)
│       ├── motion_linear.py      # drive_constant + drive_eased + brake (RC override)
│       ├── motion_depth.py       # hold_depth + prime_alt_hold (SET_POSITION_TARGET)
│       ├── duburi.py             # Duburi facade: lock + dispatch on smooth_* flags
│       └── errors.py             # MovementError / MovementTimeout / ModeChangeError
├── duburi_manager/       # ROS2 node, action server, telemetry, CLI, mission
│   └── duburi_manager/
│       ├── auv_manager_node.py   # owns MAVLink + /duburi/move ActionServer
│       ├── connection_config.py  # PROFILES + NETWORK constants
│       ├── client.py             # blocking ActionClient wrapper (DuburiClient)
│       ├── cli.py                # argparse auto-built from COMMANDS (`duburi` entry)
│       └── test_runner.py        # scripted mission demo
└── duburi_sensors/       # YawSource abstraction (sensors-only, read-only)
    ├── duburi_sensors/
    │   ├── factory.py            # make_yaw_source(name, **kw)
    │   ├── sensors_node.py       # standalone diagnostic node (no thrusters)
    │   └── sources/
    │       ├── base.py               # YawSource ABC
    │       ├── mavlink_ahrs.py       # default — wraps Pixhawk.get_attitude
    │       ├── bno085.py             # USB CDC reader + one-shot calibration
    │       ├── dvl_stub.py           # Phase-4 placeholder
    │       └── witmotion_stub.py     # Phase-4 placeholder
    ├── firmware/
    │   └── esp32c3_bno085.md     # MCU-side wire contract + reference Arduino
    └── config/sensors.yaml       # yaw_source / bno085_port / bno085_baud
```

> **Adding a new command**: add a row in `duburi_control/commands.py` and a same-named method on `Duburi`. The action server, the `duburi` CLI, and the Python `DuburiClient` all pick it up automatically — no other file needs editing.

> Packages **not** in this repo (despite older context files mentioning them): `duburi_bringup`, `duburi_driver`, `duburi_teleop`, `duburi_mission`, `duburi_vision`. They were aspirational sketches; ignore them when you read `proven-patterns.md` etc.

### 4.2 Data flow (real)

```
[duburi CLI] ──┐
               ├──/duburi/move (action goal)──→ [auv_manager_node]
[test_runner]──┘                                      │
                                                      │
                                                      ├──→ Duburi facade (lock + dispatch via COMMANDS)
                                                      │       ├── motion_yaw    (SET_ATTITUDE_TARGET ×10 Hz)
                                                      │       ├── motion_linear (RC_CHANNELS_OVERRIDE ×20 Hz)
                                                      │       └── motion_depth  (SET_POSITION_TARGET_GLOBAL_INT ×5 Hz)
                                                      │
                                                      ├──→ Pixhawk ──UDP 14550──→ [BlueOS] ──USB──→ [Pixhawk / ArduSub]
                                                      │                                                    │
                                                      │                                              ATTITUDE / AHRS2 (50 Hz)
                                                      │                                              HEARTBEAT / SYS_STATUS
                                                      │                                              BATTERY_STATUS (1 Hz)
                                                      │                                              RC_CHANNELS (5 Hz)
                                                      │                                              STATUSTEXT
                                                      │                                              COMMAND_ACK
                                                      │
                                                      └──→ /duburi/state (duburi_interfaces/DuburiState, on change)

[duburi_sensors.sensors_node]   ←── separate, diagnostic-only, never runs in mission path
```

> Telemetry rates above are explicitly pinned at startup via `MAV_CMD_SET_MESSAGE_INTERVAL` in `auv_manager_node.MESSAGE_RATES` — without this, ArduSub picks defaults (~4 Hz for AHRS2) which silently caps loop tightness.

### 4.3 Node responsibilities

| Node                             | Package         | Owns                                                      |
|----------------------------------|-----------------|-----------------------------------------------------------|
| `auv_manager_node` / `auv_manager` | `duburi_manager` | The single MAVLink connection, `/duburi/move` ActionServer, telemetry publisher, ROS params |
| `sensors_node`                   | `duburi_sensors`| Standalone yaw-source diagnostic — does NOT touch thrusters or arming |

There is exactly **one** node that touches `pymavlink` in the live mission path: `auv_manager_node`. The CLI and test_runner are ROS2 ActionClients of `/duburi/move`.

---

## 5. MAVLink / ArduSub Patterns (live code)

The actual implementations live in `src/duburi_control/duburi_control/pixhawk.py`. The snippets below show the *shape* — for current-truth, read the file.

### 5.1 Connection

```python
import os
os.environ['MAVLINK20'] = '1'  # MAVLink 2.0 (18 RC channels)
from pymavlink import mavutil

master = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
master.wait_heartbeat()
boot_time = time.time()
```

### 5.2 Arm / Disarm

```python
# Returns (success, reason). reason is a MAV_RESULT name or NO_ACK.
ok, reason = pixhawk.arm()
ok, reason = pixhawk.disarm()
```

### 5.3 Mode switching (no ACK from SET_MODE — we poll the heartbeat)

```python
ok, reason = pixhawk.set_mode("ALT_HOLD")
# ArduSub mode names:
# MANUAL | STABILIZE | ALT_HOLD | POSHOLD | GUIDED | AUTO | SURFACE
```

### 5.4 RC channel override (linear motion)

```python
# Channel mapping (1-indexed, ArduSub default):
# Ch1=Pitch  Ch2=Roll  Ch3=Throttle(depth)  Ch4=Yaw
# Ch5=Forward  Ch6=Lateral
# PWM: 1100..1900 µs | 1500=neutral | 65535=no override
pixhawk.send_rc_override(forward=1700, lateral=1500, throttle=1500, yaw=1500)
pixhawk.send_neutral()         # all six channels = 1500 (active hold)
pixhawk.release_rc_override()  # all channels = 65535 (autopilot takes over)
```

### 5.5 Attitude target (yaw to absolute heading)

```python
pixhawk.set_attitude_setpoint(yaw_deg=90.0)
# Requires ALT_HOLD / POSHOLD / GUIDED. In MANUAL it's silently dropped;
# in STABILIZE it's interpreted as a yaw RATE, which is not what we want.
# Duburi._ensure_yaw_capable_mode() auto-engages ALT_HOLD.
```

### 5.6 Depth setpoint

```python
pixhawk.set_target_depth(-1.5)   # negative = below surface
# Requires ALT_HOLD. motion_depth.hold_depth() does:
#   1. set_mode("ALT_HOLD")
#   2. prime_alt_hold(0.5 s) — drain stale I-term by streaming current depth
#   3. wait_for_depth — stream target at 5 Hz until |error| < 0.07 m or timeout
```

### 5.7 Reading attitude

```python
att = pixhawk.get_attitude()    # {'yaw': deg, 'roll': deg, 'pitch': deg, 'depth': m}
# Backed by AHRS2; cached and refreshed by the single MAVLink reader thread.
```

### 5.8 Heartbeat

`auv_manager_node` ticks a heartbeat at ≥ 1 Hz from a ROS2 timer. Don't roll your own.

### 5.9 Servo PWM (payload — torpedo / grabber / dropper)

> `pixhawk.set_servo_pwm(aux_n, pwm)` takes the AUX index (1..6, AUX1..AUX6 silkscreen) and adds the ArduSub +8 offset internally — pwm is clamped to 1100..1900. Out-of-range `aux_n` raises `ValueError`.

### 5.10 Stream rate control (MAV_CMD_SET_MESSAGE_INTERVAL)

```python
# Pin telemetry rates explicitly; see auv_manager_node.MESSAGE_RATES.
pixhawk.set_message_rate(MAVLINK_MSG_ID_AHRS2,           50)   # Hz
pixhawk.set_message_rate(MAVLINK_MSG_ID_BATTERY_STATUS,   1)
pixhawk.set_message_rate(MAVLINK_MSG_ID_RC_CHANNELS,      5)
```

---

## 6. Control philosophy — ArduSub does the inner loop

We never close a Python control loop in the live path. ArduSub's onboard 400 Hz stabilizer + EKF3 owns yaw and depth; we only stream setpoints.

| Axis    | Setpoint message                  | Loop that closes it           | Our role                     |
|---------|-----------------------------------|-------------------------------|------------------------------|
| Yaw     | `SET_ATTITUDE_TARGET` (10 Hz)     | ArduSub 400 Hz attitude PID   | stream + watch AHRS yaw      |
| Depth   | `SET_POSITION_TARGET_GLOBAL_INT` (5 Hz) | ArduSub ALT_HOLD position PID | stream + watch AHRS depth    |
| Linear  | `RC_CHANNELS_OVERRIDE` Ch5/Ch6 (20 Hz) | open loop (timed thrust)      | shape the thrust envelope    |

The two ROS params `smooth_yaw` / `smooth_linear` (both default `false`) optionally shape the *setpoint* (smootherstep / trapezoid_ramp) before it reaches the autopilot — they don't replace the autopilot's inner loop.

> Earlier revisions kept `movement_pids.py` (`DepthPID` / `YawPID`) as a "hot-fix fallback" reference. That file has been removed — ArduSub's inner loop is the only PID in the live path. If you need the math again, see `.claude/context/pid-theory.md` or pull it from git history.

---

## 7. JSF-AV Principles (still apply)

Adapted for our context:

| Principle                       | What it means here                                                  |
|---------------------------------|---------------------------------------------------------------------|
| Single entry / exit             | Each motion helper has one return path; facade is a dispatch table  |
| No dynamic allocation in loop   | RC arrays are reused, no list-comp inside 20 Hz loops               |
| Bounded loops                   | Every `while` has an explicit timeout                               |
| Fail-safe defaults              | On any error → `Duburi.stop()` (active RC neutral) or `pause()` (release) |
| Clear interfaces                | Cross-package surface = `Move.action` + `DuburiState.msg` + `Pixhawk` verbs |
| No global mutable state         | All state lives on `Duburi` / `Pixhawk` instances                   |
| Defensive input validation      | `percent_to_pwm` clamps; CLI argparse rejects out-of-range values   |
| Deterministic timing            | ROS timers + `time.monotonic()`; no `time.sleep` in callbacks       |

---

## 8. ROS2 surface (real, today)

### Action

- `/duburi/move` — `duburi_interfaces/action/Move`
  - One verb per goal; `auv_manager_node.execute_callback` dispatches via the `COMMANDS` registry.
  - See [`.claude/context/ros2-conventions.md`](.claude/context/ros2-conventions.md) for the verb list and field semantics.

### Topic

- `/duburi/state` — `duburi_interfaces/msg/DuburiState`
  - Typed snapshot (`armed`, `mode`, `yaw_deg`, `depth_m`, `battery_voltage`) with `std_msgs/Header`. Missing numerics are `NaN`, missing strings are `''`. Published only when something changes (or every ~1 s as a heartbeat).

### ROS params on `auv_manager_node`

| Param           | Type   | Default        | Notes                                                         |
|-----------------|--------|----------------|---------------------------------------------------------------|
| `mode`          | string | `sim`          | `sim`, `pool`, `laptop`, `desk` (see §3)                      |
| `smooth_yaw`    | bool   | `false`        | `true` → `yaw_glide` (smootherstep setpoint sweep)            |
| `smooth_linear` | bool   | `false`        | `true` → `drive_eased` (trapezoid thrust, settle-only brake)  |
| `yaw_source`    | string | `mavlink_ahrs` | `mavlink_ahrs` \| `bno085`                                    |
| `bno085_port`   | string | `/dev/ttyACM0` | USB CDC device path                                           |
| `bno085_baud`   | int    | `115200`       | BNO085 stream baud rate                                       |

> Older context files reference `/duburi/arm`, `/duburi/depth_cmd`, `/duburi/attitude`, `Attitude.msg`, `RCOverride.msg`, `VehicleState.msg`. **None of these exist.** Single action + single state topic + ROS params is the entire surface.

---

## 9. ArduSub modes reference

| Mode      | Use case in this stack                                              |
|-----------|---------------------------------------------------------------------|
| `MANUAL`  | Raw RC override, arm/disarm                                         |
| `STABILIZE` | Attitude-stabilized; `SET_ATTITUDE_TARGET` is interpreted as a *rate* (so we avoid it for absolute yaw) |
| `ALT_HOLD` | The only mode we use during a mission. Depth-hold + absolute yaw setpoints both work. |
| `POSHOLD` | Position hold (needs GPS/DVL) — not used today                      |
| `GUIDED`  | Waypoint following from GCS — not used today                        |
| `SURFACE` | Emergency surface — manual fallback only                            |

Typical mission sequence: `MANUAL` → `arm` → first `set_depth` engages `ALT_HOLD` → mission verbs (`yaw_left`, `move_forward`, ...) → `disarm`.

---

## 10. Reference codebases (study these for patterns, not for package layout)

| Location                                  | Era              | Lessons                                                   |
|-------------------------------------------|------------------|-----------------------------------------------------------|
| `Reference CodeBase/2023/`                | RoboSub 2nd 2023 | Core pymavlink patterns, heading PID, depth PID           |
| `Reference CodeBase/Robosub-2025-Duburi/` | RoboSub 8th 2025 | YASMIN FSM, DVL integration                                |
| `Reference CodeBase/ardusub-interface/`   | BumblebeeAS      | ROS2 + behavior trees, setpoint-based control             |
| `Reference CodeBase/BareMinimum/`         | Internal         | Minimal working pymavlink                                  |
| `Reference CodeBase/` (team archives)     | BRACU Duburi     | Joy → ROS2 → pymavlink bridge; standalone mission scripts; time-based movements |

> **The 2023 codebase is the ground truth for proven MAVLink patterns.** Names like `duburi_driver` etc that appear in `proven-patterns.md` come from those eras — *not* from this workspace.

---

## 11. Development workflow

### Step 1: Sim first

```bash
# Bring up sim (in docker terminals — see §3)
cd ~/Ros_workspaces/duburi_ws
./build_duburi.sh
source install/setup.bash
ros2 run duburi_manager auv_manager_node --ros-args -p mode:=sim
```

### Step 2: Verify connectivity

```bash
ros2 topic echo /duburi/state            # Should show armed=false, mode=MANUAL, yaw, depth, battery
ros2 node info /duburi_manager           # ActionServer should be listed
```

### Step 3: Test control via CLI

```bash
ros2 run duburi_manager duburi arm
ros2 run duburi_manager duburi set_depth -0.5
ros2 run duburi_manager duburi yaw_right 90
ros2 run duburi_manager duburi move_forward 5 80
ros2 run duburi_manager duburi disarm
```

---

## 12. Environment & paths

```bash
# Docker env
ROS_DOMAIN_ID=42
GZ_SIM_RESOURCE_PATH=~/Ros_workspaces/colcon_ws/src/bluerov2_gz/models:...
GZ_SIM_SYSTEM_PLUGIN_PATH=~/stuff/ardupilot_gazebo/build

# Workspaces
~/Ros_workspaces/colcon_ws   # bluerov2_gz sim (DO NOT MODIFY)
~/Ros_workspaces/duburi_ws   # OUR codebase (this workspace)

# Tools
~/stuff/ardupilot/            # ArduSub SITL
~/stuff/ardupilot_gazebo/     # Gazebo-ArduPilot bridge plugin
```

---

## 13. Safety rules (non-negotiable)

1. **Always have a disarm path** — Ctrl-C on the manager triggers `Duburi.stop()` + `disarm()`.
2. **Heartbeat must keep ticking** — owned by the manager's ROS2 timer; nothing in the action callback may block long enough to break it.
3. **Neutral on startup** — RC defaults to 1500 (not 65535) until a movement is active.
4. **Pool test checklist** — propellers clear, tether on, topside can ping the Jetson.
5. **Autonomous mission** — timer-delayed start (run code, wait N seconds, remove tether).
6. **DVL offset** — `dvl_depth_match = 0.78` (calibrated value from 2025 competition; will move into `duburi_sensors` when the DVL driver lands).

---

## 14. Context files (in `.claude/context/`)

| File                            | Contents                                                            |
|---------------------------------|---------------------------------------------------------------------|
| `vehicle-spec.md`               | **Canonical** Duburi 4.2 spec + TDR-vs-implementation delta         |
| `known-issues.md`               | Tracked code bugs from the 2026-04 audit, scoped per file           |
| `mavlink-reference.md`          | MAVLink messages, type masks, enums                                 |
| `ardusub-reference.md`          | ArduSub-specific parameters, modes, quirks                          |
| `pid-theory.md`                 | PID design notes (LEGACY column = REFERENCE only)                   |
| `proven-patterns.md`            | Patterns from 2023/2025 codebases — names are **historical**        |
| `sim-setup.md`                  | Detailed simulation bring-up                                        |
| `mission-design.md`             | YASMIN FSM patterns (TDR target — current repo uses `test_runner`)  |
| `hardware-setup.md`             | Pool setup, BlueOS, network topology                                |
| `ros2-conventions.md`           | ROS2 coding conventions for this project (real surface only)        |
| `yaw-stability-and-fusion.md`   | Yaw drift research; cross-links to `sensors-pipeline.md`            |
| `sensors-pipeline.md`           | `duburi_sensors` design rules + BNO085 calibration model            |
