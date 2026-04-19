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

Actuator wiring goes through the **MOSFET-based actuation board**
(replaced relays for faster switching + signal isolation). Driven from
the autopilot via `MAV_CMD_DO_SET_SERVO` on AUX outputs. The Python
surface is `Pixhawk.set_servo_pwm(aux_n, pwm)` — `aux_n` is the
silkscreen index (1..6, AUX1..AUX6); the ArduSub `+8` offset and PWM
clamping are handled internally.

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
| DVL                | **Nortek Nucleus1000** at `192.168.2.201`      | **Stub only** (`dvl_stub.py`) — driver is a TODO       |
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
│   ├── motion_profiles.py      # smoothstep / smootherstep / trapezoid_ramp
│   ├── motion_yaw.py           # yaw_snap + yaw_glide (SET_ATTITUDE_TARGET)
│   ├── motion_linear.py        # drive_constant + drive_eased (RC override)
│   ├── motion_depth.py         # hold_depth (SET_POSITION_TARGET_GLOBAL_INT)
│   ├── duburi.py               # Duburi facade + lock + dispatch on 2 smooth flags
│   └── errors.py               # MovementError / Timeout / ModeChangeError
├── duburi_manager/          # Action server, telemetry, CLI, mission script
│   ├── auv_manager_node.py     # owns MAVLink reader + /duburi/move action
│   ├── connection_config.py    # PROFILES (sim/pool/laptop/desk) + NETWORK
│   ├── client.py               # blocking ActionClient wrapper (DuburiClient)
│   ├── cli.py                  # argparse auto-built from COMMANDS ('duburi' entry)
│   └── test_runner.py          # scripted mission demo
└── duburi_sensors/          # YawSource abstraction (sensors-only, read-only)
    ├── factory.py              # make_yaw_source(name, **kw)
    ├── sensors_node.py         # standalone diagnostic node
    └── sources/
        ├── base.py                 # YawSource ABC
        ├── mavlink_ahrs.py         # default — wraps Pixhawk.get_attitude
        ├── bno085.py               # USB CDC reader + one-shot calibration
        ├── dvl_stub.py             # Phase-4 placeholder
        └── witmotion_stub.py       # Phase-4 placeholder
```

### Control philosophy in one diagram

```
        ┌─────────────────┐    ┌────────────────────┐
yaw  →  │ SET_ATTITUDE_   │    │                    │
        │ TARGET (10 Hz)  │    │                    │
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

| TDR section                          | TDR description                            | This codebase                                                                 |
|--------------------------------------|--------------------------------------------|-------------------------------------------------------------------------------|
| Appendix A — IMU                     | VectorNav VN200                            | **BNO085 + ESP32-C3** (gyro+accel, one-shot Pixhawk-mag offset)               |
| §II.C.2 Autonomous — vision          | YOLOv11 + DeepSORT                         | Out of scope here. Will live in a separate `duburi_vision` package.           |
| §II.C.3 Planning — FSM               | ROS2 finite state machine                  | Currently `test_runner.py` + `DuburiClient` against the `Move` action.        |
| §II.C.1 Control                      | ROS2 + Pixhawk + PyMavlink + EKF3 fusion   | **Implemented.** EKF3 fusion is ArduSub-side; we send setpoints over MAVLink. |
| §II.C.1 — DVL fusion                 | Nucleus1000 → ArduSub EKF3                 | Stub only. Driver work tracked in [known-issues.md](./known-issues.md).       |
| Appendix C — Tether                  | FathomX over Ethernet                      | Network constants in `connection_config.py`; no extra software needed.        |
| §II.B.6 Actuation board              | MOSFET-based, AUX from Pixhawk             | `Pixhawk.set_servo_pwm(aux_n, pwm)` with internal +8 AUX offset and PWM clamping (since 2026-04). |

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
