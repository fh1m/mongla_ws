# Legacy: the Pixhawk backend and ArduSub SITL

**This is not the vehicle.** The vehicle is the SROT board and a Raspberry Pi 5 —
see [The Shift](../../../docs/the-shift.md). This page exists because two things still depend on
the older path and would otherwise be undocumented:

1. **The `pixhawk` backend** in `mongla_control/fc/pixhawk_fc.py`, behind the same interface
   as the board.
2. **The simulator** in [`sim/`](../../../sim), which runs **ArduSub SITL by design** — it is a
   physics environment, not a vehicle.

Everything else from that era lives on the **`pixhawk` branch** (commit `b483722`), including
the full ArduSub reference, the MAVLink catalogue, the heading-lock design, the Jetson setup
runbooks and the pool-day checklist as they stood. Read them there; do not copy them back.

---

## What the backend gives you

`flight_controller:=pixhawk` selects `PixhawkFC`, which *is-a* `Pixhawk` — the raw MAVLink
layer — so this path is byte-identical to the configuration that placed 8th at RoboSub 2025.

| Concept | On ArduSub | On the SROT board |
|---|---|---|
| Depth | `ALT_HOLD` mode, host streams `SET_POSITION_TARGET_GLOBAL_INT` at 5 Hz | the board's own mode; **no setpoint stream exists** |
| Translation and yaw | `RC_CHANNELS_OVERRIDE`: Ch4 yaw rate, Ch5 forward, Ch6 lateral; 65535 releases a channel | one `MANUAL_CONTROL` frame, all four axes, no per-channel release |
| Heading hold | a host loop streaming Ch4 at 50 Hz against `yaw_source` | the board holds heading itself at 500 Hz |
| Moves | timed host loops | one on-board primitive per move |
| Depth reading | `AHRS2.altitude` | `VFR_HUD.alt` — same sign, negative below the surface |

**RC directions on the 4.2-era hull** (pool-verified 2026-06): Ch4 > 1500 = yaw **right**,
Ch5 > 1500 = **forward**, Ch6 > 1500 = strafe **right**. Polarity is a frame-configuration
property — re-confirm per hull with a bare `Ch4=1600` check before trusting it.

**Typical sequence on that path:** `MANUAL` → `arm` → the first `set_depth` engages `ALT_HOLD`
→ mission verbs → `disarm`.

## Running the simulator

```bash
cd sim && ./build_sim.sh
ros2 run mongla_sim_bringup mongla_sim stop      # one sim at a time, always
ros2 run mongla_sim_bringup mongla_sim sim
ros2 run mongla_sim_bringup mongla_sim stack --no-vision
```

`flight_controller:=pixhawk` is required there and **inert on the default branch** — extra
launch arguments are dropped silently, which is exactly why
`test_sim_contract_drift.py` asserts on it rather than trusting the launch to complain.

Bare SITL without Gazebo, for arming and motion checks:

```bash
sim_vehicle.py -L RATBeach -v ArduSub -f vectored_6dof --model=JSON \
  --out=udp:0.0.0.0:14550 --console
```

⚠ `mavlink_check` binds 14550 too. Run it with the stack **down**, or it silently steals the
autonomy link and looks like a sim fault.

## What transfers, and what does not

| Transfers | Does not |
|---|---|
| Control behaviour and every `/mongla/move` verb | detection thresholds — sim imagery is too clean |
| Mission structure and the DSL | vision gains tuned against sim footage |
| The action contract and outcome codes | anything depending on board telemetry (RPM, two packs, leak, kill) |

Full simulator documentation: [`sim/README.md`](../../../sim/README.md) ·
[`sim/.context/INDEX.md`](../../../sim/.context/INDEX.md)

---

*If you are here because a document referenced ArduSub, a Pixhawk parameter, the heading lock
or a Jetson runbook: that content was removed from the default branch on 2026-09-19 and is
preserved on the `pixhawk` branch. The current equivalents are
[`command-reference.md`](../missions/command-reference.md), [`srot-integration.md`](srot-integration.md)
and [`packages/`](../packages/README.md).*

## Documents retired into this history (2026-09-21)

Two context documents described the old vehicle and were deleted rather than
half-updated, because a doc that is 80 % true is worse than one that is absent.
They are in git history at `ae61b3b^` if the numbers in them are ever needed:

| file | why it went | what replaced it |
|---|---|---|
| `perception/camera-calibration.md` | superseded almost line for line | [`perception/camera-and-calibration.md`](../perception/camera-and-calibration.md), which carries the measured in-water figures |
| `missions/mission-design.md` | the 2025 (Pixhawk + DVL) mission architecture, including calibrated values for a hull we no longer fly | [`missions/fsm-guide.md`](../missions/fsm-guide.md) for the current FSM layer |
