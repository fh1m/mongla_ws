# Vehicle Spec — BRACU Duburi 4.2

This file is the **canonical source of truth** for what the real AUV is.
Anything in any other doc that contradicts this file is wrong; fix the
other doc.

> **One-liner:** BlueROV2 Heavy is the **Gazebo SITL target** (it shares
> the `vectored_6dof` 8-thruster frame). The real AUV is **BRACU Duburi
> 4.2** — built in-house on an octagonal Marine 5083 aluminum hull.

Source: BRACU Duburi 2025 Technical Design Report (RoboSub 2025, Irvine
CA). The TDR is the authoritative document for the physical sub; this
file mirrors only the parts that affect the software stack.

---

## Mechanical

| Item              | Spec                                                                         |
|-------------------|------------------------------------------------------------------------------|
| Hull              | Octagonal, **Marine 5083 aluminum**, low-drag profile                        |
| Windows           | 3 acrylic in brass frames — front cam, bottom cam, system access             |
| Lower stand       | Modular, hydrodynamic, standardized mounts for rapid mission-hardware swap   |
| Kill switch       | **Latex-balloon, fully non-magnetic** (replaced magnetic switch — caused compass interference) |
| Frame type (FCU)  | `vectored_6dof` (same ArduSub frame as BlueROV2 Heavy)                       |
| Thrusters         | **8x Blue Robotics T200**, repositioned depth thrusters for level submersion |

## Payload actuators (real, not stub)

| Actuator | Mechanism                                  | Notes                                                           |
|----------|--------------------------------------------|-----------------------------------------------------------------|
| Torpedo  | Slingshot, linear-rail guided              | CFD-tuned hydrodynamic body. Field-tested ≥ 4 ft range.         |
| Grabber  | Aluminum, in-house machined                | **Current sensor** detects successful grasp + safety trip.      |
| Dropper  | Solenoid-based                             | Plastic-coated aluminum marker, deviation-free descent.         |

Torpedo and dropper are actuated **NOT** through the Pixhawk — there is no
`MAV_CMD_DO_SET_SERVO` / AUX path and no `Pixhawk.set_servo_pwm` method.
They are driven from an **ESP32-C3 over USB serial** (separate from the
BNO085 board): the Python surface is `duburi.fire(n)` →
`duburi_control/payload.py` `PayloadDriver`, which writes a single ASCII
digit (`b'1'`..`b'4'`) over USB CDC; the ESP32 firmware pulls the matching
GPIO to fire the relay/solenoid. Channels: 1/2 = torpedo, 3/4 = dropper.
The board is auto-detected at manager startup by USB VID/PID (CH340), and
`duburi.payload_ready` reports connection state. See `known-issues.md` #4
and the `project_payload_actuation` memory. (Stepper grabber needs an
Actuation-Board step/dir interface — phase-2, not yet wired.)

## Electronics & power

| Item                | Spec                                                                                |
|---------------------|-------------------------------------------------------------------------------------|
| Autopilot           | Pixhawk 2.4.8 running ArduSub 4.x                                                   |
| Companion           | **Raspberry Pi running BlueOS** (MAVLink router, web UI, video)                     |
| Onboard compute     | **Nvidia Jetson Orin Nano** (autonomy + ROS2 stack)                                 |
| Communication       | **FathomX power-over-Ethernet** to ground station (replaced fragile fiber link)     |
| Power               | **Dual LiPo** — one battery for propulsion, one for compute + sensors               |
| Power distribution  | Custom PCB, dual-rail, isolated propulsion vs compute (eliminates sensor-bus noise) |
| ESCs                | 8x BlueRobotics Basic ESC, consolidated on a single horizontal board                |
| Cameras             | 2x BlueRobotics Low-Light HD USB (front + bottom)                                   |

## Sensors

| Sensor             | Hardware                                       | Status in this codebase                                |
|--------------------|------------------------------------------------|--------------------------------------------------------|
| Depth (Bar30)      | Stock ArduSub Bar30                            | Read via `AHRS2.altitude` through `Pixhawk`            |
| Compass / mag      | Pixhawk internal magnetometer                  | Used **once at boot** for BNO085 Earth-reference       |
| External heading   | **ESP32-C3 + BNO085**, USB CDC (gyro+accel)    | `BNO085Source` in `duburi_sensors`, opt-in via param   |
| DVL                | **Nortek Nucleus1000** at `192.168.2.201`      | **Working driver** — `NucleusDVLSource` (`nucleus_dvl.py` + `nucleus_parser.py`): TCP auth, AHRS heading, bottom-track position integration, backoff reconnect. Lazy-connect via `dvl_connect` / auto-connect. POSHOLD/EKF3 fusion still TODO. |
| Hydrophones        | None                                           | Out of scope                                           |

### Why BNO085 instead of the TDR's VectorNav VN200

The 2025 TDR Appendix A lists "VectorNav VN200" as the IMU. **This
codebase deviates intentionally**:

1. The Pixhawk + a small external IMU is enough for our control loop
   (yaw control delegates to ArduSub's onboard 400 Hz attitude
   stabilizer; we only need a clean heading source for *error
   measurement*).
2. Cost — VN200 is ~$5K; BNO085 + ESP32-C3 is ~$30 total.
3. Magnetic interference inside the aluminum hull (8 thrusters +
   battery currents) wrecks any always-on magnetometer-based heading.
   The BNO085 is run in `SH2_GAME_ROTATION_VECTOR` mode (gyro+accel
   only, **mag disabled on chip**) and we capture a one-shot
   Pixhawk-mag offset at boot to align it with Earth.

See [sensors-pipeline.md](./sensors-pipeline.md) §"Calibration model"
for the full rationale and `src/duburi_sensors/firmware/esp32c3_bno085.md`
for the firmware contract.

## Network

| Endpoint                | IP / port              | Notes                                            |
|-------------------------|------------------------|--------------------------------------------------|
| Jetson Orin Nano        | `192.168.2.69` static  | UDP listener for MAVLink, ROS2 host              |
| BlueOS (Raspberry Pi)   | `192.168.2.1`          | MAVLink router, web UI, gateway `192.168.2.2`    |
| DVL Nucleus1000         | `192.168.2.201`        | Reserved; not yet integrated                     |
| MAVLink endpoint name   | `inspector`            | UDP **Client** in BlueOS, IP=Jetson, Port=14550  |

The Jetson opens `udpin:0.0.0.0:14550` and BlueOS pushes packets to it
(BlueOS = client, Jetson = server). Codified in
`src/duburi_manager/duburi_manager/connection_config.py`.

---

## Software stack (this repo, today)

```
src/
├── duburi_interfaces/       # ROS2 action + msg defs
│   ├── action/Move.action      # ONLY action — single dispatcher
│   └── msg/DuburiState.msg     # typed snapshot for /duburi/state
├── duburi_control/          # MAVLink layer + per-axis motion + COMMANDS registry
│   ├── pixhawk.py              # arm/disarm/set_mode/RC/setpoints/AHRS2/set_message_rate
│   ├── commands.py             # COMMANDS registry (single source of truth)
│   ├── motion_easing.py        # smoothstep / smootherstep / trapezoid_ramp
│   ├── motion_writers.py       # shared constants + Writers (lock-aware) + thrust_loop
│   ├── motion_yaw.py           # yaw_snap + yaw_glide (SET_ATTITUDE_TARGET)
│   ├── motion_forward.py       # drive_forward_* + arc (Ch5 / Ch5+Ch4 RC override)
│   ├── motion_lateral.py       # drive_lateral_* (Ch6 RC override)
│   ├── motion_depth.py         # hold_depth (SET_POSITION_TARGET_GLOBAL_INT, lock-aware)
│   ├── heading_lock.py         # background SET_ATTITUDE_TARGET streamer (yaw cousin of depth-hold)
│   ├── duburi.py               # Duburi facade + lock + dispatch + heading_lock owner
│   └── errors.py               # MovementError / Timeout / ModeChangeError
├── duburi_manager/          # ROS2 node + action server + telemetry
│   ├── auv_manager_node.py     # owns MAVLink reader + /duburi/move action
│   └── connection_config.py    # PROFILES (sim/pool/laptop/desk) + NETWORK
├── duburi_planner/          # Mission planner: Python client + CLI + missions
│   ├── client.py               # blocking ActionClient wrapper (DuburiClient)
│   ├── cli.py                  # argparse auto-built from COMMANDS ('duburi' entry)
│   ├── mission.py              # 'mission' runner — dispatches into missions/<name>.run
│   ├── missions/               # square_pattern, arc_demo, heading_lock_demo
│   └── state_machines/         # reserved for YASMIN-based plans
└── duburi_sensors/          # YawSource abstraction (sensors-only, read-only)
    ├── factory.py              # make_yaw_source(name, **kw)
    ├── sensors_node.py         # standalone diagnostic node
    └── sources/
        ├── base.py                 # YawSource ABC
        ├── mavlink_ahrs.py         # default — wraps Pixhawk.get_attitude
        ├── bno085.py               # USB CDC reader + one-shot calibration
        ├── _discovery.py           # BNO085 USB VID/PID auto-detect probe
        ├── nucleus_dvl.py          # Nortek Nucleus1000 TCP driver (heading + position)
        ├── nucleus_parser.py       # Nucleus binary packet decoder (bottom-track + AHRS)
        ├── composite_bno_dvl.py    # BNO085 heading + DVL position (yaw_source=bno085_dvl)
        └── witmotion_stub.py       # Phase-4 placeholder
```

### Control philosophy in one diagram

```
        ┌─────────────────┐    ┌────────────────────┐
yaw  →  │ RC_OVERRIDE Ch4 │    │                    │
        │ yaw rate (10Hz) │    │                    │
        └─────────────────┘    │                    │
                               │                    │
        ┌─────────────────┐    │ ArduSub            │
depth → │ SET_POSITION_   │ →  │ onboard 400 Hz     │ → 8x T200 ESCs
        │ TARGET... (5Hz) │    │ stabilizer + EKF3  │
        └─────────────────┘    │                    │
                               │                    │
        ┌─────────────────┐    │                    │
linear→ │ RC_CHANNELS_    │    │                    │
        │ OVERRIDE (20Hz) │    │                    │
        └─────────────────┘    └────────────────────┘
```

We never close a Python control loop. ArduSub's onboard PIDs do all the
work; we just stream setpoints + RC overrides. The two `smooth_*` ROS
params optionally shape the *setpoint* (smootherstep / trapezoid_ramp)
before it reaches the autopilot — they don't replace the autopilot's
inner loop.

### Reference-only / legacy

- `movement_pids.py` (`DepthPID`, `YawPID`) was deleted in the 2026-04
  cleanup — ArduSub's onboard PID is the only loop in the live path.
  Tuning theory still lives in [`pid-theory.md`](./pid-theory.md), and
  the Python implementation can be recovered from git history if it
  ever becomes a hot-fix fallback.
- `proven-patterns.md` records 2023/2025 codebase patterns for
  reference; package and node names there are **historical**, not the
  current layout.

---

## TDR vs implementation delta

> **State key:** ✅ built · 🟦 committed phase-2 (not yet implemented) · ✏️ corrected. See audit P0.1 Decision Record (2026-05-31).

| TDR section                          | TDR description                            | This codebase                                                                 |
|--------------------------------------|--------------------------------------------|-------------------------------------------------------------------------------|
| §I — Second vehicle                  | **Dubomini 2.0** (agile, 8-thruster `vectored_6dof`, no DVL/manipulators, VN-200 IMU) | 🟦 **Committed phase-2.** No Dubomini profile / frame / thruster map / control path yet — single-vehicle Duburi today. |
| Appendix A — IMU (Duburi)            | VectorNav VN200                            | ✅ **BNO085 + ESP32-C3** (gyro+accel, one-shot Pixhawk-mag offset). Dubomini's VN-200 is phase-2. |
| §II.C.2 Autonomous — vision          | ✏️ TDR says YOLO26; **corrected to YOLO11** | ✅ `duburi_vision`: YOLO11 (`yolov11n`) + ByteTrack/Kalman, 30fps. YOLO26 legacy-only. |
| §II.C.3 Planning — FSM               | ROS2 finite state machine                  | 🟦 **Committed YASMIN FSM (phase-2)** at `duburi_planner/state_machines/`. ✅ Today: `detected()` missions + `DuburiClient` (the proto/test/FSM-fallback layer the FSM will wrap). |
| §I.D / §II.B.5 — IVC                 | Acoustic inter-vehicle comms               | 🟦 **Committed phase-2** — no transport/node yet. |
| §II.C.1 Control                      | ROS2 + Pixhawk + PyMavlink + EKF3 fusion   | ✅ **Implemented.** EKF3 fusion is ArduSub-side; we stream setpoints over MAVLink. |
| §II.C.1 — DVL fusion                 | Nucleus1000 → ArduSub EKF3                 | Parser + driver ✅ (`nucleus_parser` + `nucleus_dvl`, used by `move_*_dist` closed-loop); POSHOLD/EKF3 fusion still TODO. |
| Appendix C — Tether                  | FathomX over Ethernet                      | ✅ Network constants in `connection_config.py`; no extra software needed.       |
| §II.A — Payload actuation            | Dropper / torpedo / grabber                | 🟦 **Phase-2.** Dropper+torpedo are **ESP32-over-USB-serial** (PySerial→GPIO→relay), **NOT** Pixhawk AUX — see `project_payload_actuation` memory. Stepper grabber needs an Actuation-Board step/dir interface. No payload verbs yet. |

---

## Cross-references

- [`README.md`](../../README.md) — workspace docs, build/run guide
- [`CLAUDE.md`](../../CLAUDE.md) — agent memory index
- [`hardware-setup.md`](./hardware-setup.md) — wiring + driver runbook
- [`sensors-pipeline.md`](./sensors-pipeline.md) — `duburi_sensors` design rules
- [`ardusub-reference.md`](./ardusub-reference.md) — ArduSub quirks
- [`sim-setup.md`](./sim-setup.md) — Gazebo + SITL bring-up
- [`known-issues.md`](./known-issues.md) — tracked code bugs from the audit
- [TDR PDF](https://robonation.org/app/uploads/sites/4/2025/) — RoboSub 2025
