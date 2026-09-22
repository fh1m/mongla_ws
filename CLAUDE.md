# Mongla — the AUV autonomy stack

> **Project**: Mongla — an independent ROS 2 control / mission / vision stack for autonomous underwater vehicles.
> **This codebase**: `mongla_ws`. **Default platform**: the **SROT board** (firmware
> **Hengla**) + **Raspberry Pi 5 with a Hailo-8 AI HAT**.
> The folder name `mongla_ws` and the `/mongla/*` namespace are kept for the test vehicle's
> tooling — **do not bulk-rename them.**

**Read first, in this order:**

| Document | What it gives you |
|---|---|
| [`docs/the-shift.md`](docs/the-shift.md) | the architecture from first principles: why reflexes live on the board and thinking on the Pi |
| [`docs/capability-map.md`](docs/capability-map.md) | every capability with its evidence and an honest verification state |
| [`.claude/context/ROADMAP.md`](.claude/context/ROADMAP.md) | **the one status file** — where we are, what is left, what is blocked |
| [`.claude/context/packages/`](.claude/context/packages/README.md) | one page per package |
| [`.claude/context/BUGS.md`](.claude/context/BUGS.md) | **the single defect register** |
| [`.claude/context/README.md`](.claude/context/README.md) | **the book** — every context doc, in reading order |

**Precedence:** if anything here contradicts `src/`, the code wins. If a doc contradicts a
measurement, the measurement wins — and both belong in
[`measured-bars.md`](.claude/context/measured-bars.md).

**Three states, never blurred.** Say which one you mean:
**RUNS TODAY** (on the bench / on footage) · **BUILT, NEVER FLOWN** · **BLOCKED** (on
hardware, water, or a firmware merge). Nothing on this platform has been in water.

---

## 1. The vehicle

| Part | What |
|---|---|
| Flight controller | **SROT board**, firmware **Hengla** — ESP32 (dual core) + RP2350 Pico |
| Control loop | **500 Hz** on ESP32 core 1 (sensors, control, DShot). Core 0: MAVLink 100 Hz, LoRa/SD 20 Hz, display 30 Hz |
| Companion | **Raspberry Pi 5 + Hailo-8 AI HAT** (ROS 2 Jazzy on the vehicle; Humble on dev boxes) |
| Link | **one USB-C cable**, MAVLink 2 at 115200, compid **191** (`MAV_COMP_ID_ONBOARD_COMPUTER`) |
| IMU / depth / leak / kill / batteries / ESC RPM | all **on the board** |
| Thrusters | **⚠ two answers — see below.** Firmware mixes for 8 × T200 vectored (M1–M4 horizontal at 45°, M5–M8 vertical); the **current CAD is a 5-thruster hull**. Bluejay ESCs, bidirectional DShot either way |
| Cameras | forward + downward USB; measured calibration in `mongla_vision/config/calibration/` |
| Velocity / distance | **the downward camera** — no DVL is fitted, and none has ever been validated in water |
| Payload | board channels over MAVLink (`fire(N)` = **board channel N**, 1–16) |


> **⚠ The hull in CAD is not the hull the mixer was written for.** Measured off the CAD
> geometry on 2026-09-20 and validated against Onshape to 0.08 %: the vehicle is
> **702.0 × 176.1 × 172.1 mm**, fineness **3.99**, with **four ⌀84 mm tunnel thrusters**
> (lateral pair 350 mm apart, vertical pair 519 mm apart) plus **one axial unit in the nose** —
> five thrusters, giving **5 of 6 DOF with roll unactuated**. The firmware's mixer is
> `vectored_6dof` for **eight** T200s. Both statements are true of *something*; they are not
> true of the same vehicle.
>
> Until that is settled, **say which hull a number belongs to.** The 8-thruster figures come
> from the competition vehicles (RoboSub 2023/2025); the geometry above comes from the CAD that
> is being built now. Full read, including what is deliberately *not* concluded (net buoyancy,
> CoB above CoM, drag coefficient, thrust per tunnel — Onshape reports **no material assigned
> to any part**, so mass genuinely cannot be computed): the *body* section of the site, and
> `tools/pack_hull.py`, which reads the geometry rather than quoting it.

Full hardware detail: [`vehicle-spec.md`](.claude/context/platform/vehicle-spec.md) ·
[`srot-architecture.md`](.claude/context/platform/srot-architecture.md) ·
[`srot-board-soul.md`](.claude/context/platform/srot-board-soul.md)

### The other three repositories

`srot-control-board` (**Hengla**, firmware) · `srot-ground-station` (**Bondor**, GCS +
LoRa bridge) · `srot-esc-flasher` (bench tool). **Firmware development team lead: Rakibul
Islam.**

⛔ **We never commit to their repos; they never commit here.** Pull requests only. The shared
wire constants are frozen on both sides, `fc/srot_protocol.py` is our **single copy**, and
`test_srot_protocol_drift.py` reads the firmware headers to prove it has not drifted.
Asks we have sent: [`upstream/`](.claude/context/upstream/README.md). Rules:
[`cross-repo-contract.md`](.claude/context/platform/cross-repo-contract.md).

**A missing low-level feature is a pull request, not a host workaround.** The hardware is not
final either — a capability that needs a new sensor is a conversation with the hardware team.

---

## 2. Working with the board — the traps that cost us runs

⛔ **The depth loop has never run closed.** `SROT_MOVE` enters the board's automatic mode and
**every** primitive there closes the depth loop — including a plain `move_forward`. Two armed
bench checks gate all of them. An in-air move is *not* partial validation (at ~0 m, target and
measurement agree). [`srot-integration.md`](.claude/context/platform/srot-integration.md)

⛔ **Firmware version is a hull-safety interlock.** The board reports `SROT_FW_BEHAVIOUR_REV`;
`SrotFC.check_behaviour_rev()` runs at connect **and inside `arm()`**. The floor is
`FW_BEHAVIOUR_REV_REQUIRED` in `srot_protocol.py` — **not** a number to quote from memory,
because revision 10 **inverted yaw** and a board below the floor takes every turn backwards.
The board currently reports rev 14. `test_doc_drift.py` fails any doc that states a stale
value. Override (knowing all this): `allow_fw_behaviour_mismatch:=true`.

⛔ **Rev 7 shipped `FRAME_REVERSE`** — a parameter, default 0 but **set to 1 on our hull**,
that negates all six axis demands before the mixer. It fixes "every axis is backwards" at the
axis layer, so the `[-1] × 8` motor directions restored on 2026-08-06 are **no longer the
intended configuration** and would cancel it. Read the parameters before arming.

⛔ **An unhealthy barometer refuses `DEPTH_HOLD`/`AUTO`/`PATTERN`** — and since `SROT_MOVE`
enters `AUTO`, that means **every move verb is denied**: the vehicle arms and simply will not
move. `bringup_check --srot` reads this off `SYS_STATUS`. ⚠ `VFR_HUD` is *not* gated on baro
health and is where we read depth, so **never infer sensor health from the presence of a depth
value**.

⛔ **`AUTO` never exits by itself.** After a move, MANUAL_CONTROL frames are discarded until
the mode changes. The vision verbs handle this by setting the mode and **verifying it took**
(`set_mode` is best-effort on this wire — a silent refusal looks exactly like success).

**pymavlink decoding traps, each of which returns a plausible number:**
- `BATTERY_STATUS` is instanced and pymavlink caches per msgid — sampling that slot alternates
  between the electronics pack (~1.35 V) and the thruster pack (~14.7 V). De-multiplex by
  `id`, as `SrotFC.note_battery` does.
- `SYS_STATUS` extended health (where LEAK moved) **cannot be decoded** by pymavlink, so LEAK
  is read from `NAMED_VALUE_FLOAT` — through a per-name table fed by the reader thread,
  because seven names burst inside one tick and pymavlink's single slot keeps only the last.
- `ESC_STATUS(291)` is undecodable here; per-ESC RPM comes through the manager's own path.
  **958/958 frames read exactly 0 with nothing attached**, so a message count can never prove
  a thruster is alive.

**Absence renders `--`, never `0.0`.** From rev 3 the board suppresses values it cannot stand
behind; rendering that as zero recreates the bug the suppression fixed. Same rule on our side:
a missing number in `MonglaState` is `NaN`.

### Reading and gating the board

```bash
ros2 run mongla_manager connect --watch      # everything the board sends; no ROS graph needed
ros2 run mongla_manager bringup_check --srot # grades each subsystem, exits non-zero on a fault
```

`connect` **reports** (always exits 0); `bringup_check` **grades and gates**. Use the first to
look, the second to decide.

### Verb routing on srot

| Group | Verbs | Where it runs |
|---|---|---|
| On the board | `move_forward` `move_back` `move_left` `move_right` `yaw_left` `yaw_right` `turn` `set_depth` `stop` `pause` `style_roll` | one `SROT_MOVE`, braked on the board |
| Host | `arm` `disarm` `set_mode` `head` `surface` `mission_reset` `calibrate_depth` `calc_distance` `fire` `vision_align` `vision_move` … | a loop here, or one message |
| **Refused** | `lock_heading` `move_forward_dist` `move_back_dist` `move_lateral_dist` `arc` `style_yaw` | `srot_fc.UNSUPPORTED_VERBS` — refused *before* dispatch |

`vision_align` / `vision_move` came out of the refusal set on 2026-09-03 and actuate through
`MANUAL_CONTROL` in STABILIZE. ⛔ On srot the **depth-setpoint axes are refused**: the forward
`depth` axis, and the downward fill-driven descent. Downward lat + surge runs normally.

There is **no** `set_target_depth`, no `ALT_HOLD`, no `RC_CHANNELS_OVERRIDE` and no
per-channel release on this backend. Modes are STABILIZE · ACRO · DEPTH_HOLD · SURFACE ·
MANUAL · AUTO plus the tuning modes. Depth is reported as altitude: **negative below the
surface**.

---

## 3. Architecture

```
[mongla CLI] ─┐
[mission]    ─┼── /mongla/move (one action, 30 verbs) ──► auv_manager_node ──USB-C──► SROT board
[run_plan]   ─┘                                              │                         (500 Hz)
                                                             └──► /mongla/state, /mongla/imu,
                                                                  /mongla/esc_rpm, /mongla/demand
[vision nodes] ──► detections · lock · velocity ──► [localization] ──► /mongla/odom
```

**Exactly one node touches the board**: `auv_manager_node`. Everything else is a client.

Seven packages, one page each in [`.claude/context/packages/`](.claude/context/packages/README.md):
`mongla_control` (verbs + the flight-controller boundary) · `mongla_manager` (the node) ·
`mongla_vision` · `mongla_localization` · `mongla_planner` (CLI, DSL, missions) ·
`mongla_sensors` · `mongla_interfaces` (one action, one state topic).

**Every command, generated from the code**:
[`reference/commands.md`](.claude/context/reference/commands.md) — verbs, fields, defaults, what
the board runs vs refuses, executables, launch arguments, node parameters and wire constants.
Regenerate with `python3 tools/gen_reference.py`; a test fails if it drifts.

**Adding a verb** touches two files: a row in `mongla_control/commands.py` and a method of the
same name on the facade. The CLI, the action server and the Python client pick it up
automatically.

**Vision verbs**: two, pixel-native, never raise —
[`command-reference.md`](.claude/context/missions/command-reference.md) ·
[`vision-results.md`](.claude/context/missions/vision-results.md) ·
[`precision-alignment.md`](.claude/context/missions/precision-alignment.md).
**Mission DSL**: [`client-and-dsl-api.md`](.claude/context/missions/client-and-dsl-api.md) ·
[`mission-cookbook.md`](.claude/context/missions/mission-cookbook.md) ·
[`detected-paradigm.md`](.claude/context/missions/detected-paradigm.md).

### ROS surface

- **Action** `/mongla/move` (`mongla_interfaces/action/Move`) — one verb per goal.
- **Topic** `/mongla/state` (`MonglaState`) — armed, mode, yaw, depth, battery; `NaN` when
  absent, published on change.
- Params: `flight_controller` (`srot` default), `mode`, `yaw_source` (`mavlink_ahrs` — reads
  the board), plus the `vision.*` tunables in `vision_tunables.py`.

Older context files mention `/mongla/arm`, `/mongla/depth_cmd`, `Attitude.msg`,
`RCOverride.msg`. **None of these exist.**

---

## 4. Vision, in brief

Hailo-8 detection (measured **80.9 Hz** standalone, **53.9 Hz** through the ROS graph, 18.0 ms
photon-to-detection), a camera **mailbox** that keeps the newest frame (16.9 ms stale vs
396 ms for a queue), per-camera calibration measured in water (**46.7°**, not the datasheet's
63.8° in air), flat-port refraction correction in `optics.py`, and a lock ladder sized against
**71 real detection gaps**.

⛔ **Image preprocessing stays off.** Measured in 17 configurations across four props and three
venues: never positive; on the gate it destroyed 95 % of detections. A test keeps it off.

⛔ **A model's `<stem>.yaml` sidecar must ship beside the artifact.** Missing sidecar → empty
allowlist → a silent `[]` every frame, with the pipeline looking healthy.

Detail: [`hailo-vision.md`](.claude/context/perception/hailo-vision.md) ·
[`vision-architecture.md`](.claude/context/perception/vision-architecture.md) ·
[`underwater-vision.md`](.claude/context/perception/underwater-vision.md) ·
[`camera-and-calibration.md`](.claude/context/perception/camera-and-calibration.md) ·
[`downward-camera.md`](.claude/context/perception/downward-camera.md) (the axis remap) ·
environment pins: [`pi-and-env-traps.md`](.claude/context/platform/pi-and-env-traps.md).

---

## 5. Localization

No GPS, no DVL. A right-invariant EKF predicts on the board's IMU and corrects with depth,
optical-flow velocity, headings and prop fixes; late measurements are **replayed at the
instant they describe** rather than applied on arrival. Course priors are per venue and
overridable on the deck (`~/.mongla/courses`) with a survey tool to measure the real thing.

Verified: the downward camera as a velocity sensor — three 30 cm slides, worst error
**1.09 cm**, implied height 0.72 / 0.69 / 0.70 m against a 0.72 m tape.

[`packages/mongla_localization`](.claude/context/packages/mongla_localization/README.md)

---

## 6. Running it

```bash
./build_mongla.sh                 # mirrors ~/models and ~/missions in, then builds
source install/setup.bash

ros2 launch mongla_manager bringup.launch.py vision:=true    # the vehicle
ros2 run mongla_manager bringup_check --srot                 # can it arm?
ros2 run mongla_planner mongla arm                           # one verb
ros2 run mongla_planner mission --list                       # missions
```

**Every capability is an opt-in switch with a defensible default** (ROADMAP §9): `vision`,
`localization`, `flow`, `lock`, `retrodict`, `zupt`, `demand_aid`, `use_yaw`, `tile_m`,
`lane_lines`, `caustics`, `mixer_aware`, `velocity_uplink`, `position_uplink`. Bring the
vehicle up bare, then add one at a time and measure what it costs.
[`launch-combinations.md`](.claude/context/platform/launch-combinations.md)

**Operator tooling** (off the mission path): `scripts/pool_session.sh` pins one folder per
run; `scripts/pool_record.sh` records and replays a bag; scorecards land in `MONGLA_RUN_DIR`.
[`foxglove-and-bags.md`](.claude/context/platform/foxglove-and-bags.md) · [`pool-day.md`](.claude/context/platform/pool-day.md)

---

## 7. Simulator

[`sim/`](sim/) is a second colcon workspace in this repo: Gazebo, a pool with courses and
props, both cameras, ground truth, and an operator lab. It runs **ArduSub SITL by design** —
it is a physics environment, not the vehicle — so it uses `flight_controller:=pixhawk`.

What transfers: control behaviour and every verb. What does **not**: detection thresholds and
vision gains, because sim imagery is too clean.

[`sim/README.md`](sim/README.md) · [`sim/.context/INDEX.md`](sim/.context/INDEX.md) ·
[`legacy-pixhawk-and-sitl.md`](.claude/context/platform/legacy-pixhawk-and-sitl.md)

---

## 8. Safety rules (non-negotiable)

1. **Always have a disarm path.** Ctrl-C on the manager stops and disarms.
2. **Cooperative abort.** Every motion loop checks the abort flag once per tick; `disarm`,
   `stop` and `surface` bypass the busy gate so they always execute.
3. **Heartbeat keeps ticking.** Nothing in an action callback may block long enough to break
   it — the board surfaces after 5 s of silence.
4. **Neutral on startup.** No axis is commanded until a verb asks for it.
5. **Propellers clear, and a human on the kill switch**, before anything arms.
6. **Never claim a verb worked without seeing the value it produced.** The recurring defect in
   this codebase is a plausible number standing in for an absent measurement.

---

## 9. How we work

- **Measure, then ship.** Every shipped threshold is in
  [`measured-bars.md`](.claude/context/measured-bars.md) with the method, the conditions and
  the bar it must clear — including the numbers we later retracted. Tests read that file.
- **Injection-verify a guard.** A test that has never failed against a real defect is not a
  guard. Break it deliberately, watch it fail, restore it.
- **Truth tests, not agreement tests.** Comparing a new estimator against the incumbent
  measures agreement and cannot rank them; construct a case where truth is known.
- **Refuse loudly.** A verb that reports success while the vehicle does nothing is the failure
  mode that ends competition runs.
- **One truth, two copies is the bug.** When two places state the same constant, make one read
  the other, or have a test compare them.

---

## 10. Context index

`.claude/context/` is shelved by subject. Five anchors stay at the top because they are the
entry points and because tests read them:

| Anchor | What it is |
|---|---|
| [`ROADMAP.md`](.claude/context/ROADMAP.md) | the one status file — where we are, what is blocked |
| [`capability-map.md`](.claude/context/capability-map.md) · [`the-shift.md`](.claude/context/the-shift.md) | what the vehicle can do, and why it is built this way |
| [`measured-bars.md`](.claude/context/measured-bars.md) | every shipped constant with the measurement behind it |
| [`BUGS.md`](.claude/context/BUGS.md) | the defect register |

**[`platform/`](.claude/context/platform/)** — the board, the firmware contract, the vehicle,
bring-up and operations: `srot-architecture` · `srot-integration` · `srot-board-soul` ·
`cross-repo-contract` · `vision-control-split` · `vehicle-spec` ·
`pi-hailo-vision-box` · `pi-and-env-traps` · `launch-combinations` · `pool-day` ·
`remote-access` · `foxglove-and-bags` · `ros2-conventions` · `system-harmony` · `mongla-sim` ·
`legacy-pixhawk-and-sitl`

**[`perception/`](.claude/context/perception/)** — what the vehicle sees and how it is trusted:
`hailo-vision` · `vision-architecture` · `underwater-vision` · `camera-and-calibration` ·
`camera-calibration` · `camera-latency` · `detection-continuity` · `depth-estimation` ·
`downward-camera` · `dual-camera-setup` · `pipeline-hardening` · `sensors-pipeline` ·
`video-testing`

**[`missions/`](.claude/context/missions/)** — how it is asked to do things:
`command-reference` · `client-and-dsl-api` · `mission-cookbook` · `detected-paradigm` ·
`precision-alignment` · `vision-results` ·
`mission-design`

**[`packages/`](.claude/context/packages/README.md)** — one page per ROS package.
**[`upstream/`](.claude/context/upstream/README.md)** — the asks sent to the firmware team.
**[`scouting/`](.claude/context/scouting/README.md)** · **`future/`** — competitor notes, parked ideas.

## 11. Claude automations

**Subagents** (`.claude/agents/`): `srot-reviewer` (control changes against the board's
contract) · `mission-reviewer` · `doc-verifier` · `context-doc-sync` ·
`robosub-task-architect` · `vision-model-reviewer`.

**Skills** (`.claude/skills/`): `pool-day` · `add-command` · `new-mission` · `train-model` ·
`verify-docs` · `geohot-guidelines` · `bumblebee-doctrine`.

> **`geohot-guidelines` is standing, not optional**, for anything larger than a one-liner:
> complexity is the enemy; ask whether a thing can *not* exist before adding a layer; a wide
> interface means the abstraction is wrong; never claim code works without seeing the value.
> Its fence matters here more than usual — this is a vehicle, and several "simplifications" in
> `BUGS.md` were guards someone removed.

**Hooks** (`.claude/hooks/`): block edits to the generated `install/` tree; `py_compile` after
an edit; run the matching test file for an edited source file.

## graphify

Knowledge graph in `graphify-out/`. For a codebase question run `graphify query "<question>"`
(a scoped subgraph, usually far smaller than raw grep). `graphify path "<A>" "<B>"` for
relationships, `graphify explain "<concept>"` for one concept. After changing code, run
`graphify update .` (AST-only, no API cost).
