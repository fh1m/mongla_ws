# Mongla / Duburi AUV — Development History

Ledger of every shipped phase from initial commit to present. Each entry:
- **Goal**: what problem it solved
- **Files touched**: key additions / changes
- **Acceptance test**: how "done" was verified
- **What moved to next phase**: leftovers or follow-on work

---

## Phase 0 — Initial ROS2 skeleton (commit `1886e8b`)

**Goal:** Bootstrap a working ROS2 Humble control stack from scratch.
Pure pymavlink connection + MAVLink send/receive loop + first node.

**Files touched:**
- `duburi_manager/`, `duburi_control/` — initial layout
- `duburi_interfaces/` — `Move.action` (first version)

**Acceptance test:** `ros2 run duburi_manager auv_manager_node` connects to SITL, armed flag changes in response to `arm` command.

**Moved to next:** COMMANDS registry to eliminate copy-paste verb dispatch.

---

## Phase 1 — Clean-code rename + COMMANDS registry (commit `33bab6c`)

**Goal:** Replace one-off `if verb == 'x': ...` dispatch with a single
registry table (`commands.py`). Add `duburi_sensors` package for
yaw-source abstraction.

**Files touched:**
- `duburi_control/commands.py` — COMMANDS dict
- `duburi_sensors/` — `YawSource` ABC + `mavlink_ahrs.py` source
- `duburi_control/duburi.py` — facade uses COMMANDS

**Acceptance test:** `ros2 run duburi_planner duburi arm` dispatches via COMMANDS without any `if` chain.

**Moved to next:** Motion axis split (forward/yaw/lateral/depth as separate files).

---

## Phase 2 — Axis-split motion stack + planner + heading lock + arc (commit `f1d4516`)

**Goal:** Split all motion into dedicated modules (one per axis). Add
`duburi_planner` with blocking `DuburiClient`, CLI auto-built from COMMANDS,
and mission runner. Add heading-lock background thread. Add `arc` command.

**Files touched:**
- `duburi_control/motion_{yaw,forward,lateral,depth,vision}.py`
- `duburi_control/heading_lock.py`
- `duburi_planner/client.py`, `cli.py`, `mission.py`
- `duburi_planner/missions/{square_pattern,arc_demo,heading_lock_demo}.py`
- `duburi_interfaces/action/Move.action` — extended with all verb fields

**Acceptance test:** `ros2 run duburi_planner mission square_pattern` drives a 4-corner square in sim with no Python errors.

**Moved to next:** Depth as first-class axis (Bar30 setpoint, not just PWM).

---

## Phase 3 — Depth first-class + BNO loop fix + lateral inversion fix (commits `a02e7ff`–`2f18117`)

**Goal:** Replace throttle-channel depth with `SET_POSITION_TARGET_GLOBAL_INT`
in ALT_HOLD mode. Fix BNO085 yaw loop (sign inversion). Fix lateral PWM direction.
Add auto-mode detection (`auto`, `sim`, `pool`, `desk`).

**Files touched:**
- `duburi_control/motion_depth.py` — `hold_depth()` + `prime_alt_hold()`
- `duburi_control/pixhawk.py` — `set_target_depth()`
- `duburi_sensors/sources/bno085.py` — sign fix
- `duburi_manager/connection_config.py` — `resolve_mode()` + `PROFILES`

**Acceptance test:** `ros2 run duburi_planner duburi set_depth -- -1.0` holds depth within 7 cm in sim.

**Moved to next:** Pre-pool polish (docs, arm safety check, MAVLink debug trace).

---

## Phase 4 — Pre-pool polish + arm safety + MAVLink trace (commits `ebe3cb1`–`6367c8f`)

**Goal:** Arm safety check before any motion (raise if disarmed). Full MAVLink
per-call trace (`[MAV file:func cmd=verb]`). `debug:=true` param. Intuitive
names (`yaw_left/right`, `move_forward`, etc.). Pool-day README.

**Files touched:**
- `duburi_control/duburi.py` — `_require_armed()`, `_require_mode()`
- `duburi_control/pixhawk.py` — `[MAV ]` log prefix
- All mission files — renamed method calls to match intuitive API
- `README.md` — pool checklist, sequence diagram, video thumbnails

**Acceptance test:** Calling `move_forward` while disarmed raises `MovementError` immediately. `debug:=true` prints every MAVLink frame to `ros2 log`.

**Moved to next:** Vision pipeline (camera factory + YOLO + detection overlays).

---

## Phase 5 — Vision pipeline v1 (commit `4dc2c79` and following)

**Goal:** Camera factory, YOLO26 detector node, supervision overlays, visual
alignment (P-loop on bbox centre + height). Sim demo + webcam demo launches.

**Files touched:**
- `duburi_vision/` — `factory.py`, `config.py`, `draw.py`
- `duburi_vision/cameras/{camera,webcam,ros_topic}.py`
- `duburi_vision/detection/{detector,yolo,gpu,messages}.py`
- `duburi_vision/camera_node.py`, `detector_node.py`, `vision_node.py`
- `duburi_control/motion_vision.py`, `vision_verbs.py`
- `duburi_manager/vision_state.py`
- `duburi_planner/missions/find_person_demo.py`

**Acceptance test:** `ros2 launch duburi_vision cameras_.launch.py` streams annotated video. `vision_thrust_check` confirms Ch4 nudges in the right direction when person is off-centre.

**Moved to next:** Depth anchor, `lock_mode`, `distance_metric` vision params.

---

## Phase 6 — Vision param extensions: depth_anchor, lock_mode, distance_metric (commit `3b2325e`)

**Goal:** Three new vision verb fields:
- `depth_anchor_frac`: hold the target at a configurable vertical position in frame
- `lock_mode`: `'settle'` (exit on deadband) vs `'follow'` (run for `duration`)
- `distance_metric`: `'height'` vs `'area'` vs `'diagonal'` for distance proxy

**Files touched:**
- `duburi_interfaces/action/Move.action` — three new float fields
- `duburi_control/motion_vision.py` — `vision_track_axes()` updated
- `duburi_planner/duburi_dsl.py` — DSL `lock()` updated
- `src/duburi_manager/config/vision_tunables.yaml` — defaults
- `README.md`, `command-reference.md` — param tables updated

**Acceptance test:** `vision.lock(..., lock_mode='follow', duration=5.0)` runs for the full 5 s without exiting early when the target is centred.

**Moved to next:** Full doc restructure + `pursue_demo` mission.

---

## Phase 7 — Doc restructure + pursue_demo mission (commits `4468bd4`–`fc8f1bb`)

**Goal:** Move architecture/config docs to `docs/` sub-pages. Add `pursue_demo`
mission (torpedo-style constant-approach pattern). Full command-reference
parameter tables for all vision fields.

**Files touched:**
- `docs/` — `architecture.md`, `configuration.md`, `sensor-fusion.md`, `hardware.md`
- `duburi_planner/missions/pursue_demo.py`
- `.claude/context/command-reference.md` — full param tables
- `.claude/context/client-and-dsl-api.md` — DSL vision API docs

**Acceptance test:** `ros2 run duburi_planner mission pursue_demo` drives the AUV toward a detected person at constant fractional speed without overshooting.

**Moved to next:** ByteTrack tracking integration (v2) + Kalman smoother (v3).

---

## Phase 8 — Tracking integration: ByteTrack + Kalman v2/v3 (commit `6204b5c`)

**Goal:** Add stable track IDs that survive occlusion. Per-track 4-state CV
Kalman smoother to remove bbox jitter. `with_tracking:=true` opt-in.

**Files touched:**
- `duburi_vision/tracking/{__init__,tracker,bytetrack,kalman}.py` — full module
- `duburi_vision/tracker_node.py` — `/detections` → `/tracks` ROS node
- `duburi_vision/draw.py` — `draw_track_ids()` overlay
- `duburi_vision/detection/messages.py` — `array_to_detections()` inverse converter
- `duburi_interfaces/action/Move.action` — `tracking` bool field
- `duburi_planner/duburi_dsl.py` — `tracking=True` kwarg
- `duburi_manager/auv_manager_node.py` — `vision.use_tracks` param
- `duburi_manager/vision_state.py` — predicted-frame (score=0.0) guard
- `duburi_vision/utils/check_pipeline.py` — `tracker_check` CLI

**Acceptance test:** `ros2 run duburi_vision tracker_check --camera laptop --class person` shows track IDs persisting across 1-second hand occlusion.

**Moved to next:** Model loading from named directory, class index YAML, `cameras_.launch.py` rename, RoboSub prequal mission.

---

## Phase 9 — Model loading + class index + cameras launch + prequal (commit `ac6bed2`)

**Goal:** Named model resolution (`models/<stem>.pt`), sidecar class index YAML,
startup class list log, canonical `cameras_.launch.py` (webcam_demo deprecated),
numeric coercion for int→float kwargs, RoboSub pre-qualification mission.

**Files touched:**
- `duburi_vision/detection/class_index.py` — new: load `<stem>.yaml` sidecar
- `duburi_vision/detection/yolo.py` — `_resolve_model_path()` + startup class log
- `duburi_vision/config/detector.yaml` — `model_path: yolo26_nano_pretrained`
- `duburi_vision/models/yolo26_nano_pretrained.yaml` — full COCO 80-class index
- `duburi_vision/setup.py` — install `models/*.yaml` to share dir
- `duburi_vision/launch/cameras_.launch.py` — new canonical launch
- `duburi_vision/launch/webcam_demo.launch.py` — deprecation stub
- `duburi_planner/duburi_dsl.py` — `_to_float()` + apply in `_send()`
- `duburi_planner/client.py` — int→float coercion in `send()`
- `duburi_planner/missions/robosub_prequal.py` — full 10-phase mission

**Acceptance test:**
- `ros2 launch duburi_vision cameras_.launch.py model:=yolo26_nano_pretrained` prints 80-class table at startup
- `ros2 run duburi_planner mission robosub_prequal` runs all 10 phases in sim without TypeError on int constants

**Moved to next (current session):** Mission abort banner, stale doc cleanup, tracking+motion docs in cookbook, `goals.md` (this file).

---

## Phase 10 — Mission abort safety + tracking cookbook + doc cleanup (current)

**Goal:** Safe Ctrl-C abort sequence with aesthetic banner and countdown.
Remove stale PLAN.md "future work" text. Fix all stale `auv_manager` refs.
Add §3.3 tracking-while-moving to `mission-cookbook.md`.

**Files touched:**
- `duburi_planner/mission.py` — `_abort_sequence()` + countdown banner
- `duburi_vision/tracking/PLAN.md` — replaced with DONE stub
- `duburi_vision/filters/PLAN.md` — replaced with DONE stub
- `duburi_vision/filters/__init__.py` — remove "v3 placeholder" text
- `duburi_vision/__init__.py` — update docstring (remove "v2/v3 in PLAN.md")
- `CLAUDE.md` — file-map: tracking/filters dirs, cameras_ launch list
- `.claude/context/testing-guide.md` — all `auv_manager` → `start`
- `.claude/context/mission-cookbook.md` — new §3.3 "Tracking while moving"
- `.claude/context/goals.md` — this file

**Acceptance test:**
- Ctrl-C during a running mission prints the abort banner and attempts stop+disarm
- `grep -r 'v2/v3' duburi_vision/` returns nothing stale
- `grep -rn 'auv_manager' .claude/context/testing-guide.md` returns nothing

---

## What's next (not yet started)

| Priority | Work item |
|---|---|
| HIGH | Pool day calibration: tune `GATE_STANDOFF_FRACTION`, `FLARE_STANDOFF_FRACTION`, gate strafe duration vs measured gate width |
| HIGH | Custom YOLO model: train on gate + flare + buoy + dropper target classes, drop as `gate_flare_v1.pt` |
| MED | DVL driver: Nortek Nucleus1000 @ 192.168.2.201 — `dvl_stub.py` → real driver |
| MED | YASMIN FSM: convert prequal from linear script to state machine for re-entry safety |
| MED | `arc()` orbit alternative: replace 12-step polygon orbit with smooth `arc()` calls |
| LOW | Per-class Kalman noise tuning: `gate` vs `flare` vs `buoy` have different velocity profiles |
| LOW | `vision.follow()` DSL shorthand: wrapper for `lock(lock_mode='follow', duration=∞)` |
