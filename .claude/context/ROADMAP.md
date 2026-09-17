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
| C4 | Near-surface gain set | **DEFERRED (evidence) 2026-09-17** | board depth PID is one fixed gain set (`depth_control.cpp:40`); near-surface suction/wave coupling is a moving-body-in-waves effect and SAUVC 2026 is indoor. After G1, log `DEPTH_ERR` vs depth; add gains (firmware PR) only if the data show a near-surface error |
| C5 | Autotune entry point | **DONE 2026-09-17 (operator, not mission)** | `ros2 run duburi_manager autotune` prints the live PID briefing; `--confirm "RUN AUTOTUNE IN WATER"` runs it; Ctrl-C aborts to STABILIZE + disarm; refuses a port the manager holds. Deliberately NOT a mission verb. Needs G1 before it is useful |
| C6 | Goal id on goal/feedback/result; time-margin signal to missions | **DONE 2026-09-17** | scoreboard rows carry the first 8 hex of the action goal UUID, the manager logs the same on `[ACT]`; time margin = `duburi.task(deadline_s=)` / budget `remaining_s()` |

### Localization

| # | Item | State | Note |
|---|---|---|---|
| L1 | Time-correct fusion | **DONE 2026-09-17 (opt-in)** | the defect was stamp MISUSE, not missing synchronizers: flow/depth/fix/heading were applied on arrival. `retro.Retrodictor` + `retrodict:=true`; fixes carry the detection capture stamp; `pose_fuse` pairs the heading in effect at capture. Pi cost not yet measured |
| L2 | Battery voltage as a demand-model input | **CLOSED 2026-09-17** | the board mixer already scales thrust by PM2 voltage (`mixer.cpp:77-125`, `MOT_BAT_V_MAX=16.8` set 2026-09-15); host compensation would double-count |
| L3 | DeepVL evaluation | OPEN, wants G2 | only a docstring mention |
| L4 | Magnetometer / MAG_CAL consumers | **CLOSED (by design) 2026-09-17** | board `yaw_ref` does the one-shot mag alignment (`MAG_YAW_REF=1`); host reads `YAW_REF` for health; drift bounded by landmark anchor + tile grid |
| L5 | Course priors with measured positions | **TOOLING DONE 2026-09-17; DATA OPEN** | `courses/sauvc26.yaml` template (classes, rulebook dims, positions unset -- the rulebook gives zones, not points); `ros2 run duburi_localization course_survey --course sauvc26 --prop final_gate --x .. --y .. --bearing ..` writes the deck copy (`measured: true`) the loader reads first. Positions still need measuring at the venue |
| L6 | `floor_range` validated at taped range in water | OPEN (measurement) | `rounds/round17-range-without-size.md` |
| L7 | Rewind-and-replay lag correction | **DONE 2026-09-17 (opt-in)** | merged into L1: every filter event buffered with its prior snapshot, late ones inserted and the tail replayed |

### Perception

| # | Item | State | Note |
|---|---|---|---|
| P1 | Hailo model release between tasks | **RE-SCOPED 2026-09-17** | not a binding limit today: configure-once, activate-per-turn, **≥3 groups measured resident together** (`hailo.py:611-626`; the "2-group ceiling" in `measured-bars.md` corrected). Opt-in `release_model(stem)` only if a 4th model is needed -- Pi measurement first (plan C1) |
| P2 | Consumers for published-but-unread topics | **DONE 2026-09-16** | `duburi.outline(cls)` reads `*/contours` (polygon, area, OBB angle); `duburi.active_models(cam)` + `set_model(..., confirm_s=)` read `*/vision_info`. Both opt-in (subscribe on first call) |
| P3 | OBB angle / class posterior on the wire | **CLOSED 2026-09-17** | OBB angle already on `contours` (`duburi.outline().angle_deg`). Posterior: detection HEFs run NMS on-chip per class, so no distribution survives to send. Masks now used in `identity` (symbol pixels vs structure BOX -- a gate's mask is its pipes); `side_on(..., use_outline=True)` opt-in |
| P4 | Monocular depth on the vehicle launch | **MEASURE FIRST** | relative (per-frame min-max) depth, ONNX CPU, no control consumer, Pi cost never measured -- default stays off the vehicle launch unless a Pi measurement and a consumer justify it |
| P5 | Verify the actuation by looking | **PARTIAL 2026-09-17** | `VisionResult.fired` = board outcome per channel (`ch1:FIRED`/`none`/`pending`); opt-in `align(evidence=True)` / `duburi.save_evidence()` writes the annotated frame to the run folder. An automatic hit/miss judgement still needs a model class for the shot itself |
| P6 | Detection range per prop | **TABLE CORRECTED 2026-09-17; DATA OPEN** | `measured-bars.md` range table used the downward AIR focal (≈514 px); recomputed per camera at the rectified centre focal. The 10 px floor is still a COCO `person` number -- re-run per competition model |
| P7 | SAUVC bump flares are 16 mm wide | **RISK (quantified)** | pole detectable only inside **~0.91 m** on the forward camera (567 px centre focal, 10 px floor); golf ball ~2.4 m. Task 4 approach must come from course priors (`course_survey`), not detection at range |

### Missions / autonomy (all blocked on porting missions to srot)

| # | Item | State | Note |
|---|---|---|---|
| M1 | Port missions + FSM off Pixhawk-era verbs | IN PROGRESS | **B1 DONE 2026-09-17:** srot now refuses only the axes that MOVE a depth setpoint (forward `depth`, downward fill->depth descent); downward lat + surge runs, and `align_loop` no longer streams `set_target_depth` on srot (`test_srot_vision_actuation.py`, 3 injections bite). **Open for water, not host:** in STABILIZE nothing holds depth during an align. Measure first (armed, STABILIZE, zero heave, log depth ~20 s); only a real sag justifies an opt-in DEPTH_HOLD vision mode, gated like AUTO (depth loop never run closed; a baro refusal silently forces STABILIZE). B2/B3 mission ports next |
| M2 | SAUVC Task 4: listen station → `duburi.flare_order(timeout=)` → bump in order, else bump all | OPEN | host API and operator tool built (`09cf744`); needs H1 and #24 |
| M3 | Use `run_budget` in a real mission | OPEN (API done) | `duburi.use_budget()` / `duburi.worth_attempting()` wired 2026-09-16, opt-in; no mission calls it yet (missions are Pixhawk-era, M1) |
| M4 | Score-aware abandonment mid-task | **DONE 2026-09-17 (opt-in)** | `with duburi.task(name, deadline_s=)` cancels the goal in flight at the deadline and raises `TaskAbandoned` for the fallback; defaults to what the budget has left; safety verbs never blocked |
| M5 | Scorecard records the perception state behind each verb | **DONE 2026-09-17** | each vision row carries `vision: {target, camera, outcome, saw_target, x_px, y_px, fill, fired, model}` |
| M6 | Real-pool auto-labelling | OPEN | new work; Bumblebee's is dead code |

### Ops / tooling

| # | Item | State |
|---|---|---|
| O1 | Per-task Foxglove/Lichtblick layouts | **DONE 2026-09-17** -- `foxglove/{bins_downward,localization,board_health}.json`; `test_foxglove_layouts.py` fails when a layout plots a topic nothing publishes |
| O2 | `ImageAnnotations` from perception | **DEFERRED** -- `foxglove_msgs` not installed; burned-in `image_debug` covers it |
| O3 | Warmup | **DONE 2026-09-17** -- warmup already existed; now TIMED: Hailo logs first vs second infer ms, YOLO first vs last pass (`warmup_ms`) |
| O4 | Tiled sim floor texture | **DROPPED** -- sim deprioritised by the operator |

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
| A1 | **The DSL's metric vision (`range_to`, `floor_range`, `bearing_to`, `fix_from_prop`, `standoff_for_prop`) used ONE in-water focal length for every camera**, from a 46.7° FOV measured on the global-shutter unit while it was "forward"; since the 2026-09-07 swap that unit is DOWNWARD. Worse, a flat port has no single focal: the in-water focal grows with field angle (forward camera at 640 px: ~567 px centre, ~634 edge) | 1.5 m gate at 4.0 m, forward camera: old constant read **5.15 m on axis (+29 %)**, 4.59 m at 1.2 m off-axis, 3.33 m at 2.2 m off-axis (−17 %). A per-camera FOV alone would still read 4.40 m on axis (+10 %) | pixels go through the SAME flat-port rectifier `lock_node`/`pnp_node` use (`optics.rectifier_for`, built from the live `CameraInfo` K), then a pinhole at `K_rect`. `medium` (`DUBURI_MEDIUM`, default water) matches those nodes. No calibration published → per-camera fallback table (53.6° forward / 46.7° downward), tested against the calibration JSONs. Ray-traced off-axis truth tests for `range_to` and `floor_range` (within 1 % / 0.5 %); both fail with the rectifier removed |
| A2 | **`floor_range` used the frame centre as the principal point.** Forward cy is 290 px at 720 p, 70 px off centre | ~6° of pitch-equivalent: roughly a metre of range bias at 3 m | principal point now comes with `K_rect`; covered by the ray-traced truth test above |
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

### Built but not used — RESOLVED 2026-09-16

| Item | Resolution |
|---|---|
| `run_budget.py` (Tier 5.1) | wired into the DSL, opt-in: `use_budget(total_s, reserve_s)` starts the clock on a successful `arm()`; `worth_attempting(name, points=, worst_case_s=, fallback_s=, fallback_points=)` returns full / fallback / skip and logs it on the scoreboard. No budget = always attempt |
| `duburi_control/nav_filter.py` | **false positive** of the `src/`-only census: `tools/srot_console_server.py` and `tools/srot_replay.py` import it. Kept |
| `estimator/thrust_model.py` | stays uncalled on purpose until G2 (thrusters fitted) |
| topic `*/vision_info` | `duburi.active_models(cam)`; `set_model(name, confirm_s=N)` waits for the detector to announce `name`, then drops detections cached from the old model. Matched by NAME, not epoch (a restarted detector resets the epoch). `confirm_s=0` (default) is the old behaviour |
| topic `*/contours` | `duburi.outline(cls)` → largest `Outline(class_name, score, angle_deg, area_px, points)`; a box model gives 4 corners, a seg model the mask outline |

Tests: `test_opt_in_consumers.py` (9, real rclpy publishers), each injection-verified.

---

## 8. Documentation debt found in this pass

| Where | Claim | Truth |
|---|---|---|
| `CLAUDE.md` §2b | firmware "behaviour rev 7" | `config.h:805` is **rev 14** |
| `CLAUDE.md` §2b | `vision_align` / `vision_move` refused on srot | un-refused 2026-09-03; `UNSUPPORTED_VERBS` is `lock_heading`, `move_*_dist`, `arc`, `style_yaw` |
| `CLAUDE.md` §2b | `move_*_dist` "stay refused permanently" | planned un-refusal after PR #23 (C1) |
| `development-board.md` | "single source of truth", updated 2026-06-24 | Pixhawk era; superseded by this file |
| `CAPABILITY-MAP.md` body | "Nothing is built yet" | Tiers 0–5 largely built; §4 lists what is not |

---

## 9. Feature switches — test each alone, then in combinations

Every switch defaults to what ships, so a plain launch is unchanged. Flip ONE at a time first.
`test_feature_switches.py` fails if a bringup switch's default drifts from its node's, or if it is
declared but not forwarded (a knob wired to nothing).

### Launch (`ros2 launch duburi_manager bringup.launch.py <arg>:=<value>`)

| Switch | Default | Gates | Lands on |
|---|---|---|---|
| `vision` | `false` | the whole vision stack | include |
| `vision_stack` | `pi` | `pi` (dual camera + Hailo) or `generic` | include |
| `localization` | `true` | the RIEKF node | `duburi_localization` |
| `flow` | `false` | downward-camera velocity (needs `pool_depth_m`) | `flow_node` |
| `lock` | `true` | XFeat anchor + LK follower ladder | `lock_node` |
| `paused` | `true` | detectors idle until a mission resumes one | detectors |
| `medium` | `water` | flat-port rectification in flow, lock, pnp | three nodes |
| `velocity_uplink` | `false` | RIEKF body velocity → board (103) | manager |
| `position_uplink` | `false` | RIEKF pose → board (102) | manager |
| `mixer_aware` | `true` | srot vision frames prioritised yaw-first + saturation-aware anti-windup | manager `vision.mixer_aware` |
| `zupt` | `true` | zero-velocity updates when still | localization |
| `demand_aid` | `true` | velocity from commanded demand on a blank floor | localization |
| `use_yaw` | `false` | fuse the landmark heading anchor | localization |
| `retrodict` | `false` | apply flow/depth/fixes at their own stamp, replaying later events (dev: 0.8 ms per sample 60 ms late) | localization |
| `caustics` | `true` | sun-caustic erosion + bare-floor refusal | flow `caustic_suppression` |
| `lane_lines` | `false` | lane-line heading (mod 180) yaw bound | flow |
| `tile_m` | `0.0` | tile grating height + yaw bound (0 = off; the venue's tile size = on) | flow |
| `baro_calibration` | `true` | re-zero the Bar30 in `mission_reset` | manager |
| `foxglove` | `false` | telemetry bridge | include |
| `viewer` | `false` | on-vehicle HUD | vision |
| `allow_fw_behaviour_mismatch` | `false` | arm below firmware rev 2 (safety override) | manager |

### Live (`ros2 param set /duburi_manager vision.<name> <value>`, next goal)

`vision.coast_s` (0.8, 0 = off), `vision.lock_s` (1.0, 0 = ladder off), `vision.mixer_aware`
(true), `vision.range_gain_floor` (1.0 = off), `vision.ki_lat` (0 = off), `vision.ctrl_conf`
(0 = off).

### Mission DSL (opt-in by calling; not calling = old behaviour)

| Call | Adds |
|---|---|
| `use_budget(total_s, reserve_s=)` + `worth_attempting(...)` | run clock, full/fallback/skip |
| `with task(name, deadline_s=)` | cancel the goal in flight at the deadline, raise `TaskAbandoned` |
| `set_model(name, confirm_s=N)` | wait for the model to be live, drop pre-switch detections |
| `active_models(cam)` / `outline(cls)` | model provenance / polygon + OBB angle |
| `side_on(sym, use_outline=True)` | side of the structure from the symbol's segmentation outline, not its box |
| `flare_order(timeout=)` | SAUVC order received over LoRa this mission |
| `floor_height()` / `floor_range(...)` / `range_to(...)` | metric range from the floor and the props (rectified) |
| `anchor_on(prop)` / `fix_position()` / `fix_from_prop(prop)` | absolute heading and pool fixes (need a course with positions, L5) |
| `can_see()` / `motion()` | blind-camera and BLOCKED-hull checks |
| `align(..., evidence=True)` / `save_evidence(cam, tag)` | annotated frame of how a task ended, saved beside the scorecard |
| `DUBURI_MEDIUM=air` (env) | DSL metric vision as a plain pinhole for bench runs |
| `align(..., lock_on=, settle=, hold_heading=, fire_pass=, tool=)` | per-call precision knobs |
