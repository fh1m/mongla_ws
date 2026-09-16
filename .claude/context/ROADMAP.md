# ROADMAP — the one place for where Mongla is headed and what is left

> **Verified against the tree on 2026-09-16** (base `09cf744` plus this audit's commit), by grep
> and by running code, not by reading earlier plans. Every "OPEN" row below was checked absent in
> `src/`; every "BUILT" row has a caller. Board and Pi were NOT connected for this pass, so
> anything that needs them is stated as last-known with its date.
>
> **This file supersedes, for status:** `development-board.md` (last updated 2026-06-24,
> Pixhawk era), `robosub-2026-roadmap.md`, `.claude/plans/eventual-bubbling-cocoa.md`,
> `bumblebee-study/CAPABILITY-MAP.md` (body), `bumblebee-study/rounds/round19-gap-reaudit.md`,
> the findings ledger `bumblebee-study/rounds/round12-findings-ledger.md` (statuses only — its
> evidence stays there), and the memory `project_research_implement_queue.md`. Those files keep
> their DETAIL and EVIDENCE; they are no longer where status is decided. When one of them
> disagrees with this file, this file wins until re-verified against the tree.
>
> **Rule for keeping it true:** a row moves to DONE only with a commit, a production caller, an
> executed test and an injection that bites. Re-grep before trusting any OPEN row older than a
> week — a stale ledger is confidently wrong about the one thing it exists to answer.

---

## 1. Where we are headed

**Platform (decided, built):** the SROT board runs every inner loop at 500 Hz with the BNO085 in
hand; the Raspberry Pi + Hailo-8 does perception and estimation and feeds the board. Pixhawk /
ArduSub is the preserved `pixhawk` branch, not the target. Missions and the YASMIN FSM are still
Pixhawk-era and have never run on srot + Pi.

**Competitions:** SAUVC 2026 (indoor 25 × 16 m pool, breach ends the attempt, flare order told to
the team after Navigation) and RoboSub 2026 (outdoor TRANSDEC). Capabilities are keyed by LABEL,
not by competition, with a deck-side override for every venue constant.

**The capability arc, in dependency order:**

1. **Get wet safely** — board and hull can arm, hold depth closed-loop, move and stop.
   *(blocked on the gate in §2)*
2. **Know where we are without a DVL** — RIEKF on board IMU + downward flow + demand model +
   depth + prop fixes, uplinked to the board. *(host side built; board side is PRs #11, #18, #23)*
3. **Close vision loops on the board** — bearings in radians uplinked, board servoes at 500 Hz.
   *(host side built; board side is PR #18, then stages 2–4 of `VISION_API.md`)*
4. **Missions that score on srot** — port missions off Pixhawk-era verbs, add course priors,
   budget and fallbacks. *(not started on srot)*
5. **Run-level autonomy** — budget-driven task choice, score-aware abandonment, explainable misses.

---

## 2. The gate to water (nothing below it matters until these close)

| # | Item | State | Owner / evidence |
|---|---|---|---|
| G1 | **Board depth loop has never run closed.** Every srot move enters AUTO, which closes depth, so this gates `move_forward` too | OPEN (last read 2026-09-15: `DEPTH_OUT` absent while disarmed) | bench: the two armed checks in `srot-integration.md` |
| G2 | **Thrusters fitted and ESC RPM non-zero.** 958/958 ESC frames read 0 with nothing attached; `THR_TRIM_EN=0` WARN | OPEN (hardware) | blocks `thrust_model`, the RPM velocity sensor, ESC current as a BLOCKED channel |
| G3 | **Firmware PR merge.** 21 of our PRs open on `srot-control-board`, none merged since 2026-09-03 | OPEN | `gh pr list -R RakibulIslam1/srot-control-board`; see §3 |
| G4 | Board config applied 2026-09-15: Bar30 re-zeroed (+0.005 m), `LEAK_EN=1`, `MOT_BAT_V_MAX=16.8`, read back | DONE (re-verify after any reflash) | operator |
| G5 | Tool offsets: torpedo and dropper are `unmeasured: true`, so every fire through them warns and aims the camera centre | OPEN (tape measure) | `src/duburi_vision/config/tool_geometry.yaml` |

---

## 3. Firmware PR queue — ours, open, in merge order

Ordered by what each unlocks. **Do not author a new firmware PR without checking this list
first**; twice already the thing to ask for was already open.

| Tier | PR | What it unlocks |
|---|---|---|
| Safety | **#15** AUTO never exits (MANUAL_CONTROL discarded after a move) | any mix of host servo and board moves |
| Safety | **#8** SROT_MOVE reports ACCEPTED at 100 % while still running | trustworthy move completion |
| Safety | **#12** KILL=0 is ambiguous; nothing refuses a cut kill switch | arm interlock |
| Safety | **#14** `wrapPi` never terminates for a large finite input from PARAM_SET | a param write cannot hang the loop |
| Safety | **#1** water-test readiness (stale `DEPTH_OUT`) | G1 diagnosis |
| Link | **#17** companion link 115200 → 1 Mbaud (51.8 % full at idle) | headroom for every uplink |
| Link | **#5** protocol honesty (REQUEST_MESSAGE, TIMESYNC) | a measured clock instead of an estimated one |
| Nav | **#11** OPTICAL_FLOW_RAD ingest | flow on the board |
| Nav | **#23** DIST leg (KF + cascaded PID, variance weighting, VISION_POSITION stage 1) | `move_*_dist` on srot — see §4 C1 |
| Vision | **#18** LANDING_TARGET ingest + echo, actuates nothing | stages 2–4 of `VISION_API.md` |
| Vision | **#19** standoff validity bound (35°) | safe standoff on the board |
| Vision | **#2** VISION_API FOV + 31001 allocated twice | ID hygiene |
| Control | **#20** mixer saturation computed then discarded; PIDs wind up | board anti-windup; host `allocator` reporter becomes real |
| Control | **#21** setpoint shaping: Ruckig measured, NOT recommended | decision record only |
| Health | **#10** ESC presence dropped one line from the wire | thruster health without inference |
| Health | **#4** Pico ESC voltage/current/temp decoded then discarded | per-thruster health, ESC current channel |
| Health | **#3** telemetry: log the decisions (+54 B/record) | post-run explanation |
| Sensors | **#22** thruster as a velocity sensor | needs G2 |
| Docs | **#16** MANUAL_CONTROL.z deviates from spec | — |
| SAUVC | **#24** flare order over LoRa (`MAV_CMD_SROT_FLARE_ORDER` 31020) | Task 4; also needs H1 |
| GS | ground station **#4** absent telemetry renders 0.00 m; joystick disable sends no neutral | operator display honesty |

**Drafted, NOT sent (need operator OK):**
`srot-control-board/PR_DEMAND_ECHO_AND_HEAP_MIN_2026-09-14.md` (DEM_FWD/DEM_LAT echo, HEAP_MIN)
and `PR_SENSOR_HEALTH_AND_TIME_2026-09-11.md`. Mixer `maxabs` is already inside #20 — do not
re-ask.

---

## 4. Host work that is open — verified absent in `src/`

### Control

| # | Item | State | Note |
|---|---|---|---|
| C1 | `move_*_dist`, `arc`, `style_yaw`, `lock_heading` on srot | REFUSED (`srot_fc.UNSUPPORTED_VERBS`) | un-refuse `move_*_dist` ONLY after #23 merges, and in that same change flip `velocity_uplink` default to true (DIST refuses to start with no velocity) |
| C2 | `velocity_uplink` / `position_uplink` on by default | OFF (both `False`) | gated on #23 and G1 |
| C3 | Altitude HOLD verb (height above floor as a Z mode) | OPEN | height is published (`floor_height`, `duburi.floor_height()`); holding it needs G1 |
| C4 | Near-surface gain set | OPEN | no code |
| C5 | Autotune driven from a mission | OPEN | `SrotFC.autotune` exists for the operator only |
| C6 | Goal id on goal/feedback/result; time-margin signal to missions | OPEN | no `trajectory_id`; see M3 |

### Localization

| # | Item | State | Note |
|---|---|---|---|
| L1 | `message_filters` time-synced pairing | OPEN (0 uses) | stamps are now correct on every board input, so pairing is meaningful |
| L2 | Battery voltage as a demand-model input | OPEN | `command_velocity` uses demand only |
| L3 | DeepVL evaluation | OPEN, wants G2 | only a docstring mention |
| L4 | Magnetometer / MAG_CAL consumers | OPEN | mag-free by design; landmark anchor + tile grid bound drift instead |
| L5 | Course priors with measured positions | **DATA OPEN** | only `courses/robosub26.yaml` exists and **every prop position is `None`**; **no SAUVC course file exists**, so `fix_position` / `fix_from_prop` cannot succeed anywhere yet |
| L6 | `floor_range` validated at taped range in water | OPEN (measurement) | `rounds/round17-range-without-size.md` |
| L7 | Rewind-and-replay lag correction | OPEN | stamps are right; no replay |

### Perception

| # | Item | State | Note |
|---|---|---|---|
| P1 | Hailo model LIFECYCLE release between tasks | OPEN (no `LifecycleNode`) | the resident-group SRAM ceiling; Bumblebee `yolo_ros_trt` pattern |
| P2 | Consumers for published-but-unread topics | OPEN | `*/contours` (segmentation outlines) and `*/vision_info` (model provenance) have **no subscriber** in `src/` |
| P3 | OBB angle / class posterior on the wire | OPEN | contours yes, posterior no |
| P4 | Monocular depth on the vehicle launch | OPEN | node only in `vision.launch.py` |
| P5 | Verify the actuation by looking (post-fire ROI check) | OPEN | we fire and assume |
| P6 | Per-class observed detection range from real class widths | OPEN (data) | by-product of the rejected water-clarity study |
| P7 | SAUVC bump flares are 16 mm wide | **RISK** | at 2 m the forward camera (634 px focal at 640 wide) sees ~5 px, against a measured ~10 px detection cliff; plan the approach from `standoff_for_prop`, not habit |

### Missions / autonomy (all blocked on porting missions to srot)

| # | Item | State | Note |
|---|---|---|---|
| M1 | Port missions + FSM off Pixhawk-era verbs | OPEN | operator ruling: no fallback audit until they run on srot |
| M2 | SAUVC Task 4: listen station → `duburi.flare_order(timeout=)` → bump in order, else bump all | OPEN | host API and operator tool built (`09cf744`); needs H1 and #24 |
| M3 | Wire `run_budget` into a real mission | OPEN | `run_budget.py` has **no production caller** |
| M4 | Score-aware abandonment mid-task | OPEN | no code |
| M5 | Scorecard records the perception state behind each verb | OPEN | verbs only today |
| M6 | Real-pool auto-labelling | OPEN | new work; Bumblebee's is dead code |

### Ops / tooling

| # | Item | State |
|---|---|---|
| O1 | Per-task Foxglove/Lichtblick layouts | OPEN |
| O2 | `ImageAnnotations` from perception | OPEN (0 uses) |
| O3 | `warmup()` before a node advertises | OPEN |
| O4 | Tiled sim floor texture (so the tile grating can be rehearsed in sim) | OPEN |

---

## 5. Hardware asks (board or BOM, not code)

| # | Item | Why |
|---|---|---|
| H1 | **External LoRa antenna feedthrough** on the aluminium hull, insulated from water | flare order (#24) cannot work through a Faraday cage; pool range untested |
| H2 | One hydrophone | thruster ego-noise altimeter (20–25 kHz, sim-only, round 20) AND the 45 kHz SAUVC pinger (50-point drum), separate bands |
| H3 | Laser-line altimeter | altitude on a textureless floor, where flow scale and tile height fail |
| H4 | PMW3901 flow ASIC as a second velocity sensor | untested underwater |
| H5 | Second synchronised downward camera | caustic-flicker stereo — parked |

---

## 6. Research — closed, so nobody re-runs it

| Topic | Verdict | Record |
|---|---|---|
| Ruckig jerk shaping on the ESP32 | REJECTED on measurement | PR #21, round 15 |
| ADRC / STSMC for the distance leg | REJECTED: 2–3x reversals / 300–486 per stop on the relay actuator | `rounds/dist_leg/` |
| CLAHE before the detector | REJECTED: 17 configs, destroys 95 % of gate detections | `measured-bars.md` |
| Water clarity from detector contrast | REJECTED: 4 slopes from one clip, one negative | round 18 |
| Fourier-Mellin phase correlation | CLOSED: slower (16.87 vs 12.34 ms), 10–30x the error | `95cd45a` |
| Camera-based flare order | REVERTED `2993d89` (operator) | ledger R |
| Pinax refraction LUT | SCOPED OUT: ~90 % of the error is the central Snell map | round 9 |
| Hailo mask prototype as a descriptor | LOSES to a 160 px grey thumbnail at 1/9.5 the cost | memory `project_hailo_seg_budget` |

---

## 7. This audit (2026-09-16): wiring and arithmetic of the last tiers

**Method.** Mechanical first, then by execution: an AST topic graph (publisher ↔ subscriber), a
caller census of every non-test module, launch parameters against `declare_parameter`, launch
include arguments against `DeclareLaunchArgument`; then independent TRUTH checks of the
arithmetic — numerical Jacobians, a ray-traced flat port, synthetic PnP scenes — never the code's
own formula. Dev suite before any change: 3220 passed, 0 failed.

### Defects found and fixed

| # | Defect | Consequence | Fix + guard |
|---|---|---|---|
| A1 | **The DSL used ONE in-water FOV (46.7°) for every camera.** It was measured on the global-shutter unit while that was "forward"; since the 2026-09-07 camera swap that unit is DOWNWARD. The forward Fantech is 73.9° air → 53.6° water | forward `range_to` / `floor_range` / `fix_from_prop` / `standoff_for_prop` read **17 % long**; forward `bearing_to` angles **~13 % short** | per-camera FOV from the live `CameraInfo` K, fallback table derived from the committed calibrations; tests pin each camera, a live K, and the table against the JSON files; injecting one shared value fails 3 tests |
| A2 | **`floor_range` used the frame centre as the principal point.** Forward cy is 290 px at 720 p, 70 px off | ~6° of pitch-equivalent: roughly a metre of range bias at 3 m | principal point from K; truth test at the real cy; injection fails |
| A3 | **RIEKF update Jacobians disagreed with the filter's own error definition** (`_inject`): body velocity carried a spurious `Rᵀ[v]×` attitude block (the true block is exactly 0); depth and position fixes lacked the `-[p]×` coupling; yaw lacked tilt terms | simulated: **no accuracy change inside a pool** (radius ≤ 15 m, 1 rejection either way); 410 vs 20 gate rejections far from the origin (\|p\| ≈ 48 m) | H corrected; `test_inekf_jacobians.py` checks every H against a finite difference through `_inject`; injecting the old blocks fails |
| A4 | **Bearing refraction was per-axis**; a flat port refracts the polar angle and keeps azimuth, which is the model `optics.RefractiveRectifier` already uses | +0.7° / +1.4° at the forward frame corner, ~0.2° mid-frame | radial `_refract_point`; ray-traced corner truth test fails on the old model. Two existing tests had put their "on-axis" point at `h/2` instead of `cy` and now sit on the true principal row |
| A5 | `vision.launch.py` passed `max_predict_frames` to `tracker_node`, which had renamed it `max_predict_s` | the launch argument did nothing (rclpy ignores undeclared params silently) | launch passes `max_predict_s` (seconds) |
| A6 | Three shims (`duburi_vision.{pose_cluster,heading_anchor,resection}`) left by the Tier 5.2 move, **zero importers** anywhere | a second copy waiting to drift | deleted |

### Verified correct (by truth, not agreement)

- **Heading anchor end to end** through the real `solve_pnp`: 6 hull/board geometries, error
  < 0.05°. New `test_heading_anchor_truth.py`; flipping the yaw sign fails 5 of 6.
- **Resection** `fix_from_bearings`: exact on a synthetic 3-prop fix.
- **Position uplink covariance** slots: upper-triangle row starts 0/6/11/15/18/20 match `POS_COV_IDX`.
- **Twist covariance** rotated to the body frame (`Rᵀ P R`) matches the body-frame twist.
- **Published position covariance**: the invariant `P[6:9]` against the world-frame `J P Jᵀ` made
  no measurable NEES difference in simulation (conservative either way) — left as is.
- Every argument `bringup.launch.py` forwards is declared by its target launch.
- Every `fwd_*` / `dwn_*` / shared parameter the Pi launch passes to the dual detector is declared.

### Built but not used — recorded, not deleted

- `run_budget.py` (Tier 5.1) — no production caller (M3).
- `duburi_control/nav_filter.py` — no production caller since `e9528ad` (2026-09-03).
- `estimator/thrust_model.py` — uncalled on purpose until G2.
- Topics `*/contours`, `*/vision_info` — no subscriber (P2).

---

## 8. Documentation debt found in this pass

| Where | Claim | Truth |
|---|---|---|
| `CLAUDE.md` §2b | firmware "behaviour rev 7" | `config.h:805` is **rev 14** |
| `CLAUDE.md` §2b | `vision_align` / `vision_move` refused on srot | un-refused 2026-09-03; `UNSUPPORTED_VERBS` is `lock_heading`, `move_*_dist`, `arc`, `style_yaw` |
| `CLAUDE.md` §2b | `move_*_dist` "stay refused permanently" | planned un-refusal after PR #23 (C1) |
| `development-board.md` | "single source of truth", updated 2026-06-24 | Pixhawk era; superseded by this file |
| `CAPABILITY-MAP.md` body | "Nothing is built yet" | Tiers 0–5 largely built; §4 lists what is not |
