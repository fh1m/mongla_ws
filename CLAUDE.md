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
hardware, water, or a firmware merge). **This stack and the Mongla hull have never been in
water.** The SROT board has: on 2026-08-06 and 2026-08-07, in the old Dubomini hull, driven by the
firmware team's own tools (their commits `09d0033`, `f533bd2`; our record is
`srot-integration.md`, *IN-WATER 2026-08-06*).

---

## 1. The vehicle

| Part | What |
|---|---|
| Flight controller | **SROT board**, firmware **Hengla** — ESP32 (dual core) + RP2350 Pico |
| Control loop | **500 Hz** on ESP32 core 1 (sensors, control, DShot). Core 0: MAVLink 100 Hz, LoRa/SD 20 Hz, display 30 Hz |
| Companion | **Raspberry Pi 5 + Hailo-8 AI HAT** (ROS 2 Jazzy on the vehicle; Humble on dev boxes) |
| Link | **one USB-C cable**, MAVLink 2 at 115200, compid **191** (`MAV_COMP_ID_ONBOARD_COMPUTER`) |
| IMU / depth / leak / kill / batteries / ESC RPM | all **on the board** |
| Thrusters | **final hull: 7** — the 5 in the current CAD (lateral pair, vertical pair, axial nose unit) plus **2 roll thrusters, one per side** (CAD pending). The firmware still mixes for 8 × T200 vectored: that matrix is from the old Dubomini vehicle (firmware PR #39 replaces it with geometry). Bluejay ESCs, bidirectional DShot |
| Cameras | forward + downward USB; measured calibration in `mongla_vision/config/calibration/` |
| Velocity / distance | **the downward camera** — no DVL is fitted, and none has ever been validated in water |
| Payload | board channels over MAVLink (`fire(N)` = **board channel N**, 1–16) |


> **Three thruster counts, one vehicle.** Say which one a number belongs to.
>
> | count | what it is | where it lives |
> |---|---|---|
> | **7** | **the final Mongla hull.** The five below plus two roll thrusters, one on each side, so all six axes are actuated | the CAD for the roll pair is not done; positions come from it, never from an estimate |
> | **5** | the current CAD: **702.0 × 176.1 × 172.1 mm**, fineness **3.99**, four ⌀84 mm tunnels (lateral pair 350 mm apart, vertical pair 519 mm apart) and one axial unit in the nose; roll unactuated | `mongla_control/config/hull_geometry.yaml`, `geometric_allocation.py`, `tools/pack_hull.py` |
> | **8** | the old Dubomini vehicle. Its `vectored_6dof` matrix (8 × T200 at 45°) is **still what the firmware mixes for**, so `allocation.py` mirrors it for as long as the board flies it | firmware `mixer.cpp`; replaced by geometry in [srot-control-board#39](https://github.com/RakibulIslam1/srot-control-board/pull/39) |
>
> The 8-thruster figures in older docs (RoboSub 2023/2025) belong to the old vehicle. Mass,
> net buoyancy and drag are not concluded: Onshape assigns **no material to any part**, and a
> float test measures net buoyancy better than CAD can (`hull_geometry.yaml`, `unknown:`).

Full hardware detail: [`vehicle-spec.md`](.claude/context/platform/vehicle-spec.md) ·
[`srot-architecture.md`](.claude/context/platform/srot-architecture.md) ·
[`srot-board-soul.md`](.claude/context/platform/srot-board-soul.md)

### The other three repositories

`srot-control-board` (**Hengla**, firmware) · `srot-ground-station` (**Bondor**, GCS +
LoRa bridge) · `srot-esc-flasher` (bench tool). **Firmware development team lead: Rakibul
Islam.**

**Warning: We never commit to their repos; they never commit here.** Pull requests only. The shared
wire constants are frozen on both sides, `fc/srot_protocol.py` is our **single copy**, and
`test_srot_protocol_drift.py` reads the firmware headers to prove it has not drifted.
Asks we have sent: [`upstream/`](.claude/context/upstream/README.md). Rules:
[`cross-repo-contract.md`](.claude/context/platform/cross-repo-contract.md).

**A missing low-level feature is a pull request, not a host workaround.** The hardware is not
final either — a capability that needs a new sensor is a conversation with the hardware team.

---

## 2. Working with the board — the traps that cost us runs

**Warning: The depth loop has never run closed under this stack.** `SROT_MOVE` enters the board's
automatic mode and **every** primitive there closes the depth loop — including a plain
`move_forward`. Two armed bench checks gate all of them. The 2026-08-06/07 water tests closed it
in `DEPTH_HOLD` on the old hull: it bounced at `DEPTH_P` 3.0, was tuned in water to
P 0.5 / I 0.1 / D 0.01 (now the defaults), and a `DEPTH_HOLD` to `AUTO` transition crashed the
board, which the bench could not reproduce because it needs a live barometer. So nothing has
validated `AUTO`, the mode every move verb uses. The firmware's own comments still say "never
run closed", which is older than that test. An in-air move is *not* partial validation (at ~0 m, target and
measurement agree). [`srot-integration.md`](.claude/context/platform/srot-integration.md)

**Warning: Firmware version is a hull-safety interlock.** The board reports `SROT_FW_BEHAVIOUR_REV`;
`SrotFC.check_behaviour_rev()` runs at connect **and inside `arm()`**. The floor is
`FW_BEHAVIOUR_REV_REQUIRED` in `srot_protocol.py` — **not** a number to quote from memory,
because revision 10 **inverted yaw** and a board below the floor takes every turn backwards.
The board currently reports rev 14. `test_doc_drift.py` fails any doc that states a stale
value. Override (knowing all this): `allow_fw_behaviour_mismatch:=true`.

**Warning: Rev 7 shipped `FRAME_REVERSE`** — a parameter, default 0 but **set to 1 on our hull**,
that negates all six axis demands before the mixer. It fixes "every axis is backwards" at the
axis layer, so the `[-1] × 8` motor directions restored on 2026-08-06 are **no longer the
intended configuration** and would cancel it. Read the parameters before arming.

**Warning: An unhealthy barometer refuses `DEPTH_HOLD`/`AUTO`/`PATTERN`** — and since `SROT_MOVE`
enters `AUTO`, that means **every move verb is denied**: the vehicle arms and simply will not
move. `bringup_check` reads this off `SYS_STATUS`. `VFR_HUD` is *not* gated on baro
health and is where we read depth, so **never infer sensor health from the presence of a depth
value**.

**Warning: An UNKNOWN kill switch refuses arming.** The kill state comes from the second
board over ESP-NOW; with no link it is UNKNOWN, and `arm` is refused unless
`allow_unknown_kill_switch` is set (bench only, under `nodes:` in `~/.mongla/bringup.yaml`). A
kill while armed, or a mission that dies armed, latches: the manager keeps disarming or
surfacing until the board confirms it, and refuses every verb but `disarm`, `stop` and
`surface` meanwhile (`BUGS.md` B-93, B-94).

**Warning: `AUTO` never exits by itself.** After a move, MANUAL_CONTROL frames are discarded until
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
ros2 run mongla_manager bringup_check # grades each subsystem, exits non-zero on a fault
```

`connect` **reports** (always exits 0); `bringup_check` **grades and gates**. Use the first to
look, the second to decide.

### Where each verb runs

23 verbs; the generated list is [`reference/commands.md`](.claude/context/reference/commands.md).

| Group | Verbs | Where it runs |
|---|---|---|
| On the board | `move_forward` `move_back` `move_left` `move_right` `yaw_left` `yaw_right` `turn` `set_depth` `stop` `pause` `style_roll` | one `SROT_MOVE`, braked on the board |
| Host | `arm` `disarm` `set_mode` `head` `surface` `mission_reset` `calibrate_depth` `calc_distance` `fire` `motor_test` `vision_align` `vision_move` | a loop here, or one message |

The board holds heading and depth at 500 Hz, so there is no host heading lock and no
distance-closed move: those verbs needed a host-side heading source or a DVL and went with the
old backend. `vision_align` / `vision_move` actuate through `MANUAL_CONTROL` in STABILIZE.
**Warning: The vision loops never command depth**: the forward camera's image-Y axis and a
fill-driven descent both raise `ValueError` — `set_depth` first, then align. Downward lat +
surge runs normally.

**Warning: `surface()` is the last motion.** Every board verb — `stop` included — is a `SROT_MOVE`,
which enters AUTO and so takes the hull back out of SURFACE. A cleanup that ends
`surface(); stop()` holds the vehicle at depth; `test_sauvc_srot_port.py` refuses it.

Modes are STABILIZE · ACRO · DEPTH_HOLD · SURFACE · MANUAL · AUTO · PATTERN plus the tuning
modes; a name the board does not have (`ALT_HOLD`, `POSHOLD`, `GUIDED`) resolves to nothing and is refused.
There is no `RC_CHANNELS_OVERRIDE`. Depth is reported as altitude: **negative below the
surface**.

---

## 3. Architecture

```
[mongla CLI] ─┐
[mission]    ─┼── /mongla/move (one action, 23 verbs) ──► auv_manager_node ──USB-C──► SROT board
[run_plan]   ─┘                                              │                         (500 Hz)
                                                             └──► /mongla/state, /mongla/imu,
                                                                  /mongla/esc_rpm, /mongla/demand
[vision nodes] ──► detections · lock · velocity ──► [localization] ──► /mongla/odom
```

**Exactly one node touches the board**: `auv_manager_node`. Everything else is a client.

Six packages, one page each in [`.claude/context/packages/`](.claude/context/packages/README.md):
`mongla_control` (verbs + the board driver) · `mongla_manager` (the node) ·
`mongla_vision` · `mongla_localization` · `mongla_planner` (CLI, DSL, missions) ·
`mongla_interfaces` (one action, one state topic).

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
- Params: `mav_device` (empty = find the board), `payload_channels`, `velocity_uplink`,
  `position_uplink`, `allow_fw_behaviour_mismatch`, plus the `vision.*` tunables in
  `vision_tunables.py`. There is one board, one heading (the board's AHRS) and one link, so
  there is no backend selector. Launch defaults can come from `~/.mongla/bringup.yaml`
  (`$MONGLA_BRINGUP`); anything typed as `name:=value` still wins. **Node parameters
  go in the same file, under `nodes:`** -- one file for the deck, and every node
  refuses a misspelt key or one a launch argument already sets. The vision launch
  files read it too, each from `launch: <file>:`, only when typed themselves.

Older context files mention `/mongla/arm`, `/mongla/depth_cmd`, `Attitude.msg`,
`RCOverride.msg`. **None of these exist.**

---

## 4. Vision, in brief

Hailo-8 detection (measured **99.0 Hz** standalone on the fixed colour path; **53.9 Hz** through
the ROS graph, an older figure not yet re-taken; 18.0 ms photon-to-detection -- measured-bars §3), a camera **mailbox** that keeps the newest frame (16.9 ms stale vs
396 ms for a queue), per-camera calibration measured in air and carried into water by Snell's law (the downward
camera: **63.8°** air, so **46.7°** water -- computed, not yet checked in water), flat-port refraction correction in `optics.py`, and a lock ladder sized against
**71 real detection gaps**.

**Warning: Image preprocessing stays off.** Measured in 17 configurations across four props and three
venues: never positive; on the gate it destroyed 95 % of detections. A test keeps it off.

**Warning: A model's `<stem>.yaml` sidecar must ship beside the artifact.** Missing sidecar → empty
allowlist → a silent `[]` every frame, with the pipeline looking healthy.

Detail: [`hailo-vision.md`](.claude/context/perception/hailo-vision.md) ·
[`vision-architecture.md`](.claude/context/perception/vision-architecture.md) ·
[`underwater-vision.md`](.claude/context/perception/underwater-vision.md) ·
[`camera-and-calibration.md`](.claude/context/perception/camera-and-calibration.md) ·
[`downward-camera.md`](.claude/context/perception/downward-camera.md) (the axis remap) ·
environment pins: [`pi-and-env-traps.md`](.claude/context/kutum/pi-and-env-traps.md).

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
# Kutum, the vehicle's Pi 5 + Hailo-8 (kutum/README.md). The machine is made
# once with `sudo kutum/setup.sh --system` on it; deploy after every change.
# Deploy stops on a doctor FAIL and exits NON-ZERO if the vehicle's own gate refuses.
kutum/deploy.sh fh1m@mongla.local              # sync + doctor + build + gate
kutum/deploy.sh fh1m@mongla.local --check-only # the doctor alone
kutum/backup.sh fh1m@mongla.local              # pull what only the vehicle has

./build_mongla.sh                 # mirrors ~/models and ~/missions in, then builds
source install/setup.bash

ros2 launch mongla_manager bringup.launch.py vision:=true    # the vehicle
ros2 run mongla_manager bringup_check                 # can it arm?
ros2 run mongla_planner mongla arm                           # one verb
ros2 run mongla_planner mission --list                       # missions
```

**Every capability is a switch with a defensible default** (ROADMAP §9): `vision`,
`localization`, `flow`, `lock`, `retrodict`, `zupt`, `demand_aid`, `use_yaw`, `tile_m`,
`lane_lines`, `caustics`, `mixer_aware`, `velocity_uplink`, `position_uplink`. Bring the
vehicle up bare, then add one at a time and measure what it costs.
[`launch-combinations.md`](.claude/context/platform/launch-combinations.md)

**Operator tooling** (off the mission path): `scripts/pool_session.sh` pins one folder per
run; `scripts/pool_record.sh` records and replays a bag; scorecards land in `MONGLA_RUN_DIR`.
[`foxglove-and-bags.md`](.claude/context/platform/foxglove-and-bags.md) · [`pool-day.md`](.claude/context/platform/pool-day.md)

### Kutum, the vehicle's computer

Kutum is the Pi 5 + Hailo-8 AI HAT+ on the vehicle: Ubuntu 24.04, ROS 2 Jazzy, answering as
**`mongla.local`** on Ethernet and Wi-Fi alike. `kutum/` is that machine as files, and
[`kutum/README.md`](kutum/README.md) is the runbook (network, clock, rebuilding from a blank
card). The chapter is [`.claude/context/kutum/`](.claude/context/kutum/README.md).

| script | runs on | what it does |
|---|---|---|
| `setup.sh --system` | the Pi, with sudo, **once per machine** | Hailo driver, firmware and runtime from source; udev, systemd, netplan; passwordless sudo for `fh1m`; then the user half and the doctor. The operator runs it, because it asks for the Wi-Fi password |
| `deploy.sh fh1m@mongla.local` | the dev box, after every change | sync the tree, run the doctor, clean build, run `bringup_check`. Stops on a doctor `FAIL`, exits non-zero if the gate refuses, and refuses while a `mongla_manager` runs there |
| `doctor.sh` | either (read-only) | `ok` / `WARNING` / `FAIL` for every assumption the stack makes about the machine; exits 1 on `FAIL` |
| `backup.sh fh1m@mongla.local` | the dev box | pull what only the vehicle has, into `~/kutum-backup/mongla/` (`--runs` adds runs and footage) |
| `env.sh` | every shell on the Pi | ROS and the workspace, already sourced |

**Caution:** the Pi's Python must keep **NumPy 1.x**. `cv_bridge` and the system `cv2` are
built against it, and a NumPy 2 pulled in by any `pip install` kills every vision node at
import. `kutum/constraints.txt` pins it for every install, and the doctor checks it. The
other environment traps (shadowed OpenCV, the empty `trackers 2.5.0` wheel, the board with
no `/dev/ttyUSB*`) are in [`pi-and-env-traps.md`](.claude/context/kutum/pi-and-env-traps.md).

Scripts reach the Pi with `ssh -o BatchMode=yes`, by key, never by password. Run long
sessions inside **mosh + tmux**
([`remote-access.md`](.claude/context/kutum/remote-access.md)), so a run outlives a dropped
tether.

---

## 7. What is not on main

`main` is the Pi + Hailo-8 + SROT vehicle and nothing else. The earlier stack — the Pixhawk /
ArduSub backend, the Jetson, the BNO085/DVL sensor package and the Gazebo + ArduSub SITL
simulator — is preserved, whole, at the tag **`last-with-pixhawk-and-sim`**. Read it there; do
not port it back. A capability that needs something from it is rebuilt against the board.

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

- **How code and docs are written.** Plain words, no emojis: in docs a marker that matters is a
  label (**Warning:**, **Caution:**, **Key point:**); in operator output it is `WARNING:` /
  `ERROR:` / `ok` / `FAIL`. Comments are sentence case and say *why the code is this way
  now*; the history (what a value used to be, who found the bug, on which day) belongs in
  `BUGS.md`, `measured-bars.md` and the git log, with a pointer (`measured-bars §21`) where
  the reader needs it. A class too long for one file is split into mixins named by job
  (`lock_setup.py`, `lock_inputs.py`, `lock_publish.py`), with a file map in the node's
  docstring; tests read it whole through `tools/code_text.py:source_of`.
- **Write the scorer BEFORE the run, and say what would falsify it.** Measured
  the hard way this session: six model-free candidates were built, and the four
  whose pass/fail criterion was written down first were each settled in one
  run. The two scored afterwards both produced a confident number that was
  wrong — a vanishing point jittering 20 227 px while its own quality metric
  read 0.958, and a "top 10 offenders" list that turned out to be measuring
  base64 on disk rather than context. `tools/tau_from_scale.py --self-test`
  is the shape to copy: it runs the ranking against synthetic sources whose
  bias was chosen, so a bug in the scorer cannot be mistaken for a defect in a
  sensor on the day — and it corrected its own author before any water.
  **Warning: A quality number a method computes from the data it just fit is not
  evidence.** The only trustworthy check is a second, independent observation.
- **A citation is not a measurement.** `approach.py` shipped a band citing
  §23, correctly, around numbers §23 never states — §23 measured pixels of
  sqrt-area and the code compared fraction of frame height. It survived weeks.
  `test_shipped_constants_match_the_ledger.py` now compares shipped constants
  against the digits in `measured-bars.md`, in **both** directions, and caught
  a missing entry on its first run.
- **Warning: THE MODELS IN `models/` ARE DEVELOPMENT HARNESSES.** They exercise
  logic and pipelines; the models that matter are chosen and swapped **on the
  day** during water testing, and verified against archive footage. So **never
  hard-code a class list in more than one place.** Retiring one model on
  2026-09-28 touched 85 files by textual substitution and left `task_gate` and
  `pool_day_practice` steering on `rescue`, which the new graph does not emit —
  both ran, reported success and thrust open loop, because
  `detection/hailo.py` logs an error and **ignores** an all-unknown allowlist.
  Checks key on the one line that already names the model (`set_model`), so a
  swap is a one-line edit:
  `test_missions_only_name_classes_a_model_emits.py`.
- **Measure, then ship.** Every shipped threshold is in
  [`measured-bars.md`](.claude/context/measured-bars.md) with the method, the conditions and
  the bar it must clear — including the numbers we later retracted. Tests read that file.
- **Injection-verify a guard.** A test that has never failed against a real defect is not a
  guard. Break it deliberately, watch it fail, restore it.
- **Warning: No capability ships unreachable.** A module is either **imported by something that
  runs**, or it carries a written reason naming the consumer it waits for. There is no third
  state. This is the repo's oldest and most expensive defect — the lock ladder in no launch
  file, `lock_s` held at 0, the bank storing a position nothing passed, `device_path` read and
  ignored, loop closure wired into the launch `bringup` does not include, and **ten orphaned
  modules found on 2026-09-24, five of them written that day**. Every one looked finished:
  correct code, passing tests, and nothing on the vehicle. Decide at write time — that is the
  only moment anyone knows whether a consumer exists.
  `test_no_capability_is_built_and_unreachable.py` enforces it for modules,
  `test_no_function_is_built_and_uncalled.py` for functions. **A test is not a
  caller**: until 2026-11-06 both counted one, a stem match let `anchor/draw.py`
  pass because a different `draw.py` was imported, and 34 functions only tests
  ever ran read as live. Imports resolve by dotted name now, in one place
  (`tools/orphan_sweep.py`); a deliberate wait goes in `DEFERRED_FUNCTIONS`
  with its consumer named.
- **The docs are read and edited in the [GitHub wiki](https://github.com/fh1m/mongla_ws/wiki).**
  Each page is also a file under `.claude/context/` (or `docs/`). The wiki workflow turns
  every wiki edit into a **review branch and pull request**; only a merged, checked PR reaches
  `main`. A push to `main` rebuilds the wiki. The bot must never push an unreviewed wiki edit
  straight to `main`, even when its own doc guards pass. If GitHub suppresses CI on a PR created
  with its workflow token, edit the PR description to trigger the integrity/provenance checks.
  If Actions cannot create the PR, the branch remains and must be reviewed manually. Editing
  the file here updates the wiki only after that PR merges. `tools/wiki_build.py` defines
  conversion; `test/test_wiki_edits_pass_pr_gate.py` protects the workflow contract.
- **Warning: Docs change in the same commit as the code they describe — not after.** A
  removal is not finished until every file that names the removed thing says what
  is true now: docs, skills, agents, launch comments, test docstrings, the site.
  This rule exists because the 2026-11-06 backend removal deleted the code in one
  commit and left **53 live files** describing it — a package page drawing the
  deleted driver, a runbook listing "refused" verbs that no longer existed, a guide
  whose first step started the deleted simulator. Three guards hold it, and the
  edit hook runs the first two on every doc edit:
  `test/test_the_retired_stack_stays_retired.py` (every tracked file, every
  retired token, each exemption with its reason), `test_docs_contract.py` (every
  context doc reachable from the book, under its own title; every link resolves),
  and `tools/gen_reference.py` + `test_operator_commands_are_real.py` (every
  command a doc tells an operator to type exists). A doc that cannot be made true
  is deleted, not left.
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

Everything else is shelved by subject under `platform/`, `perception/`, `missions/`,
`packages/` (one page per package), `upstream/` (asks sent to the firmware team), `scouting/`
and `future/` — `ls` them. Start with **`platform/first-pool-verification.md`**: what water
has to decide, in order.

## 11. Claude automations

Subagents (`.claude/agents/`) and skills (`.claude/skills/`) are listed by the harness every
session; two notes on them:

> **`geohot-guidelines` is standing, not optional**, for anything larger than a one-liner:
> complexity is the enemy; ask whether a thing can *not* exist before adding a layer; a wide
> interface means the abstraction is wrong; never claim code works without seeing the value.
> Its fence matters here more than usual — this is a vehicle, and several "simplifications" in
> `BUGS.md` were guards someone removed.

**Hooks** (`.claude/hooks/`): block edits to the generated `install/` tree; `py_compile` after
an edit; run the matching test file for an edited source file, and the doc guards for an
edited doc, skill, agent or config.
