# mongla_localization audit — every file, and how a mission reaches it

Read on 2026-11-20. Method: a line-by-line read of each module, then every consumer in the
other packages (the vision lock, the manager's uplink, the planner's DSL and missions,
Batighor's views), every suite on the dev box, and a clean colcon build of the workspace.
States are the three this repository uses: **RUNS TODAY**, **BUILT, NEVER FLOWN**,
**BLOCKED**. No part of this package has been in water on any platform.

## What localization gives Mongla, in one paragraph

An answer to "where am I, which way am I facing, and am I moving the way I was told", with
no DVL and no GPS. A right-invariant EKF predicts on the board's 50 Hz IMU and corrects with
depth, the downward camera's velocity, the board's attitude, ZUPT, the floor's grid and lane
line, an anchored heading and prop fixes; late measurements can be replayed at the instant
they describe. Around the filter: course priors (where each prop is, refusing what was never
measured), the heading anchor (a prop of known bearing turns boot-relative yaw into a pool
heading), resection and placement (position from props, props from position). The board
holds attitude and depth at 500 Hz; this package owns pool-frame position, which the board
does not have.

## What moved, and why

The package had become the place anything went that every other package could import.

| Moved | From | To | Why |
|---|---|---|---|
| `spin`, `orphan`, `params_file`, `launch_defaults`, `launch_respawn`, `realtime` | `mongla_localization` | new package `mongla_runtime` | node plumbing; vision, the manager and Batighor depended on the estimator to start a node |
| `pnp_node`, `pose_fuse_node` | `mongla_localization` | `mongla_vision` | they solve a target's pose in the camera frame from the lock's evidence: vision's output |
| `pool_lines`, `tile_grating` | `mongla_localization` | `mongla_vision/flow/` | they read a grey image; only `flow_floor` runs them |
| `snap_to_grid` | `tile_grating` | `heading_anchor` | the one part the filter uses: an angle, not a pixel |

`pose_cluster` stays: `heading_anchor` uses it. Localization no longer imports cv2, and its
package description says so. `gen_reference.py` now finds packages instead of listing them,
so a new package cannot go without a module map.

## Defects found

| Id | What | State |
|---|---|---|
| B-124 | a measurement the filter refused was counted as fused; rejected flow kept "NO VELOCITY AIDING" silent | fixed, guard fails on the old code |
| B-125 | the fused target pose published 0.0 for every float it never fused; Batighor plotted a zero bearing | fixed, injection-verified |
| B-126 | `pose()` returned the first odometry it cached for the rest of the run; `motion()` read a dead localiser's last latched verdict forever | fixed: both need `/mongla/odom` whose stamp moved within `ODOM_MAX_AGE_S`, the one bound the lock already used, now beside the timer it is five periods of |
| B-127 | a vision placement was written to the course as `measured: true`, so one drifted pose froze itself into the course and no later placement could correct it | fixed: placements are written unmeasured with their sigma; only the tape survey makes a prop measured |
| B-128 | course and frames files dropped unknown keys (`bearing:` loaded as unset; `downwards_cam:` as not measured) and read a quoted `"false"` as measured | fixed: refused at load, naming the nearest key |
| — | `survey_course` ignored a failed anchor and flew the route for nothing | fixed: it stops, says why, still surfaces |
| — | `use_course` said every unmeasured prop "refuses a dead-reckon"; only an unset one does | fixed: unset and nominal are logged apart |
| — | `pose_cluster`'s paired path took the newest heading that was not None, so a NaN one gave `decided=True` with a NaN yaw (unreachable from today's node, which maps NaN to None) | fixed, as the legacy path already did |
| — | `on_danger_circle` checked a three-prop solve that was never built | deleted with its tests and register entry |
| — | a course naming a class the live model cannot emit was caught by nothing (robosub26 names four) | fixed: the detector publishes `<ns>/model_classes`; `use_course` and `set_model` warn |
| B-132 | "aided" before any input; a never-thrust hull waited the 3 tau coast settle for its first ZUPT (0.71 m in 2.7 s on the real bench recording) | fixed, measured-bars 99 |
| B-133 | the floor grid and lane line bounds held the image-to-heading residual at ZERO, so every lane read along the pool's x axis was refused and the grid pulled yaw under any mount not square to the hull | fixed: the residual is referenced at the first read after each anchor; a truth test renders lane and tile images through the real detectors under a 37 deg mount |
| — | `usable` trusted the observability gate, which reads aided on a cold filter: 20 s of "aided" while the position ran 9 m | fixed: usable also needs a flow or ZUPT fused within `AID_MAX_AGE_S` (2 s) |
| — | `tile_decimate:=1` turned the grating off silently: it refused a blurred, noisy grid at every angle | fixed: refused at start-up while `tile_m` is set |
| — | `Grating.second_period_px` (rows-only bins, no consumer) and a two-axis `phase` that held one number | deleted / made one number |
| B-134 | the manager's uplink tick ran the 10 s vision preflight on `timer_group`, stalling `/mongla/state`, health and the board-link tick | fixed: the uplink builds without the preflight |
| B-135 | the accelerometer-bias prior (0.01) was 17 sigma off the measured 0.167 m/s^2; a still hull walked 15.5 cm in 90 s of the real bench recording | fixed: `ACCEL_BIAS_PRIOR` 0.2, 0.94 cm |
| B-136 | one fix in, the filter was certain and wrong: flow's scale error grows with distance, its sigma did not, and the next correct fix was refused 3 times in 5 against exact truth | fixed: `FLOW_SCALE_SIGMA` floor, 15 of 15 accepted |
| B-137 | a position fix the gate refused was silent; `fix_from_prop` had already said it worked | fixed: counted and warned |
| B-138 | a zero quaternion was fused as a level hull at the board's 0.5 deg sigma | fixed: no attitude, sample skipped |
| B-139 | the ZUPT's accelerometer veto read the magnitude, which a sideways push barely moves: a hull shoved along with the gyro quiet read as still | fixed: per-axis spread of the body's own acceleration; A/B on the real bench recording, no cost at rest |
| — | flow's scale error stated twice, 0.036 in mongla_vision and a hand-rounded 0.035 here | one value, `test_one_scale_error.py`; the truth study re-run at 0.036, same verdicts |
| — | `_wrap180` in four modules, `resection._wrap360` never called, an `except` after the reject inflation that cannot run | one `_wrap180` in `heading_anchor.py`; the dead ones deleted |

## How a mission reaches it

Localization is used the way vision is: a state anyone can read, verbs that act on it, and a
mission that checks it. Before 2026-11-20 only `survey_course` called it and nothing could
ask whether it was working; that is what this section replaced.

**The state.** `/mongla/localization/state` (`LocalizationState`), 10 Hz, published whether
or not anything feeds the filter. `usable` is the one answer and `reason` says why not, in
the order an operator fixes them: no IMU, IMU silent, heading not anchored, position
unaided. It also carries the pool position, depth, heading, the worse-axis horizontal sigma
(attitude terms included), aiding, motion, the age of each input and the flow
fused/refused counts. Batighor's localization view plots `usable` and `reason` beside
aiding and motion.

| Interface | What it does | State |
|---|---|---|
| `mongla locate` (CLI) · `locate` verb | the state on one line; `success` = usable; works disarmed | BUILT, NEVER FLOWN |
| `mongla go_to --x_m X --y_m Y` · `go_to` verb | surge and sway to a pool point in STABILIZE; the board holds depth and heading. Refused before any thrust unless usable and sigma within `tolerance_m`; stops when the estimate goes bad or motion reads blocked | BUILT, NEVER FLOWN |
| `mongla.localization()` | the state, from the topic; None once the localiser stops | BUILT, NEVER FLOWN |
| `mongla.go_to(x, y)` · `mongla.go_to_prop(prop, standoff=)` | the verb; to a course prop's position, short of it | BUILT, NEVER FLOWN |
| `use_course` · `anchor_on` · `fix_from_prop` · `fix_position` · `place_prop` | priors, heading, position fixes, placement | BUILT, NEVER FLOWN |
| `pose()` · `motion()` | the bare tuple and the motion verdict | BUILT, NEVER FLOWN |
| mission `localization_check` | the six-stage check below | BUILT, NEVER FLOWN |
| mission `survey_course` | places props from one station | BUILT, NEVER FLOWN |

`go_to`'s speed ramp (`GO_TO_FULL_SPEED_M`, full gain at 1 m of error) is a starting value,
not a measurement; the first pool run sets it.

## The water test: `localization_check`

One mission, one verdict per stage, the first failure being the one to fix:

1. **live**: the state publishes, before arming.
2. **aided**: hovering, flow or ZUPT is fused (`aiding` = aided), within 10 s.
3. **anchored**: `anchor_on` puts the estimate in the pool frame.
4. **out** and 5. **back**: `go_to` a point 1.5 m ahead and back. This is the filter
   agreeing with itself, which is necessary and is not evidence.
6. **truth**: a range-and-bearing fix off the anchor prop, an observation the filter did not
   make, agrees with the filter's position to 0.5 m. This is the stage that ranks it.

Before it: tape `frames.yaml` (both lever arms and the baro offset; until then flow is
fused without its lever arm and the frames health line says DEGRADED), and record a bag.

## Every line, executed

Read is not run. On 2026-11-20 the package's suite was measured with line coverage
(`coverage run --source=mongla_localization -m pytest src/mongla_localization/test
src/mongla_planner/test`): 91 % of 1603 statements. Every unexecuted line was then read
again and given a test or deleted; three were defects (B-138, B-139, and the zero-sigma
update that would have raised `LinAlgError` and killed the node, now refused).
`test_the_unrun_branches.py` holds them, each injection-verified, and
`test_the_real_node_constructs.py` builds the real node through rclpy, which no test did:
the stubbed tests pass with a topic renamed on one side.

Now **99 % of 1582**. What does not run under pytest, and where it does run:

| Lines | What | Runs |
|---|---|---|
| `localization_node.main()` | `rclpy.init`, spin, shutdown | every `ros2 run` and every Kutum replay (measured-bars 99) |
| `course_survey`, the `__main__` line | `sys.exit(main())` | `test_the_survey_runs_as_a_program`, in a subprocess coverage does not follow |

## Read, and judged sound

`inekf.py`, `localization_node.py`, `localization_inputs.py`, `localization_publish.py`,
`command_velocity.py`, `retro.py`, `heading_anchor.py`, `resection.py`, `course_map.py`,
`course_survey.py`, `prop_placement.py`, `floor_plane.py`, `frames.py`, `pose_cluster.py`,
`frames.yaml`, the three course files, `courses/__init__.py`, `__init__.py`, `package.xml`
(every declared dependency is imported), `setup.py`, `setup.cfg` and the package README.
`inekf.py` was read a second time for the fix path, which found B-136. The two image readers
moved to `mongla_vision/flow/` were read with B-133 (`tile_grating`, `pool_lines`).

## Is this the right design? Research, 2026-11-20

The question: is an invariant EKF on the board's IMU, aided by downward optical
flow, ZUPT, depth, prop fixes and floor features, with no DVL, the right
estimator for a pool AUV, or should something replace or join it?

| Candidate | What the evidence says | Verdict |
|---|---|---|
| Invariant EKF itself | Published for underwater navigation with IMU + DVL ([Potokar et al., RA-L 2021](https://unpaywall.org/10.1109%2FLRA.2021.3085167)), better long-term than a quaternion EKF. Our flow is the same measurement as a DVL's: body velocity, so the invariant structure carries over | **keep** |
| Off-the-shelf VIO (ORB-SLAM3, OpenVINS, VINS, SVIn2) | In texture-less pool sequences ORB-SLAM3 failed every one and SVIn2 failed one ([GeVI-SLAM, 2025](https://arxiv.org/pdf/2510.24533)); OpenVINS diverges without a static start ([Joshi et al.](https://ar5iv.labs.arxiv.org/html/1904.02215)); all show large errors under poor visibility ([AQUA-SLAM, 2025](https://arxiv.org/pdf/2503.11420)). Our own XFeat VO lost to LK flow inside the same solver (measured-bars 94) | **reject**, as primary; reconsider only with a stereo pair and a textured venue |
| Downward camera + IMU + depth | The established DVL-free design on ROVs, pool-tested ([LIRMM](https://hal-lirmm.ccsd.cnrs.fr/lirmm-01567463/document)); a low-cost vision solution reported comparable to IMU-DVL dead reckoning ([UW APL, Oceans 2025](https://depts.washington.edu/uwaplopal/research/papers/Oceans2025_Marburg_Low_Cost_Visual_Mapping_and_Navigation.pdf)). Ours measured 1.09 cm worst on 30 cm slides (measured-bars 13) | **keep**: this is the design |
| Drift without an external reference | Flow + IMU alone still drifts; an external reference is required (the point made in [this Blue Robotics thread](https://discuss.bluerobotics.com/t/optical-flow-for-rov-2d-trajectory-mapping/23165)). Ours are prop fixes (`fix_from_prop`), the heading anchor, and floor grid/lane bounds | **keep**; B-133 made the floor bounds actually work |
| Learned inertial odometry ([TLIO](https://arxiv.org/pdf/2007.01867), [AirIO 2025](https://arxiv.org/html/2501.15659v2)) | A network regresses velocity or displacement into an EKF; AirIO reports 23.8 % from body-frame features on drones. Needs training data from this hull moving in water, and we have none | **defer**: the demand model (`command_velocity`) is the same idea fitted online; revisit after water data exists |
| RoboSub teams without a DVL | METU (2025) uses VIO; IIT Bombay (2026) reports DVL-free localisation with a map-correction layer ([search results](https://robonation.org/app/uploads/sites/4/2026/07/RS26_TDR_IITBombay_AUV-IITB.pdf)). Their reports were fetched but could not be text-extracted here, so nothing beyond those summaries is claimed | read in full before the pool days; the map-correction idea matches our prop fixes |

What would reverse this: a stereo camera on the hull, a DVL fitted, or in-water
data where flow refuses most frames (then VIO on texture, or a learned inertial
model trained on that data, is the next thing to A/B).

### What the wider survey changed (2026-11-20, second pass)

A survey of the top RoboSub teams' design reports, competition teams without a DVL, recent
papers and practitioner forums found no DVL-free pool vehicle reporting better than about
2-4 % of distance travelled (DeepVL, [arXiv 2502.07726](https://arxiv.org/abs/2502.07726)).
Every verified top RoboSub team that names a velocity sensor carries a DVL, with dead
reckoning only as the dropout bridge; CMU fuses object detections as factors (GTSAM). An
open-source low-cost competition AUV with no DVL ([OpenMantaClaus](https://github.com/kushagra77/OpenMantaClaus))
runs an EKF on a thruster-feedback model plus YOLO landmark corrections, and its author
calls the thruster model "a huge source of drift and error": our demand model stays a
fallback only, and flow is the velocity.

### Every candidate, and what happened to it

| # | Candidate (survey rank) | Tested on | Result | Verdict |
|---|---|---|---|---|
| 1 | flow scale against a taped floor target | — | needs water; in air it is 3.6 % worst (measured-bars 13) | **pool day**, bar <= 2 % mean |
| 2 | close the at-rest miss | the real bench recording, 1x on Kutum | the bias prior; walk 15.5 cm -> 0.94 cm in 90 s | **shipped** (B-135) |
| 2b | the same prior, in motion | truth study A1 | rms 0.050 vs 0.051 m | **shipped**, no cost |
| 3 | altitude for flow scale: floor map, else a Ping2 | course file arithmetic | a sloping floor is up to 30 % of scale at 0.5 m depth | **bead es5.7**, needs the venue floor surveyed |
| 3b | the same chain on real underwater data | AQUALOC harbour, sequences 02-07, height fitted on 01 (`tools/studies/aqualoc_replay.py`) | flow holds 5.8-29.5 m final error (24-104 % of path) against kilometres for IMU and depth alone; a fitted height does not carry between runs (1.52 vs 0.60 m) | **runs on real water; not an accuracy result**: harbour floor, fitted height and a poor gyro, against 2-4 % for the best DVL-free systems |
| 4 | camera-to-IMU lag and de-rotation | — | the board is not on Kutum; no recording has camera and gyro together in rotation | **blocked** (po7.13) |
| 5 | heading at surveyed headings | — | needs the pool | **pool day**, bar <= 2 deg |
| 5b | heading from image rotation | the same AQUALOC replay | yaw error 75-177 deg without it; with it final error fell on 6 of 6 but halved on only 3 | **bar missed**; heading against the board's AHRS is bead 5ag.9, needs the board |
| 6 | prop fixes gated into the filter | truth study A3 | the gate refused correct fixes from a confidently wrong filter | **fixed and shipped** (B-136, B-137) |
| 7 | thruster model only as a dropout bridge | truth study A4 | a 12 s flow dropout costs 1.1 cm on this route; the demand model stays a fallback | **kept as is** |
| 8 | IMU or FOG upgrade | — | only if 2 had failed on a measured bias; it did not | **discarded** |
| — | OpenMantaClaus's bearing-only EKF SLAM: prop positions learned while the run flies, on thruster dead reckoning and IMU yaw | their source (`src/ekfslam/`) and design notes; no accuracy figure published | the same information reaches us in two separate steps: `mission survey_course` maps the props from sightings beforehand, then `resection` fixes position against that map. Mapping while flying lets a drifted pose move the map it is being corrected by; the gap between their dead reckoning and our flow is what candidate 7 measured | **not adopted**; ours covers it |
| — | off-the-shelf VIO as primary, compass near thrusters, Ping360, USBL, hydrophone ranging | evidence only | no pool result beats what we have, or the sensor is not fitted | **discarded** |

The study is `tools/studies/localization_candidates.py` (self-tested; bars written first);
its output for 2026-11-20 is in measured-bars 99.


## Open, and what decides it

Every item still open for this package, with the GitHub issue and the bead that holds its bar.
None of them can be settled at a desk; each needs the board moving, or the pool.

| What | Issue | Bead | Decided by |
|---|---|---|---|
| flow scale in water | — | 5ag.1 | a taped floor target; pass at 2 % mean, 3 % worst |
| a floor that slopes scales flow wrongly | [#35](https://github.com/fh1m/mongla_ws/issues/35), altimeter option [#50](https://github.com/fh1m/mongla_ws/issues/50) | es5.7 | the venue's floor surveyed into the course file, or a Ping2 fitted |
| the board's attitude counted as independent of the gyro it came from | [#36](https://github.com/fh1m/mongla_ws/issues/36) | 5ag.5 | the board moving against an independent heading truth, scored by NEES. A still bench cannot size it: 10 min still, yaw Allan deviation at most 0.002 deg. The recovery half is fixed: the anchor is a yaw offset, not a gated update |
| props as continuous landmark states | [#39](https://github.com/fh1m/mongla_ws/issues/39) | 5ag.6 | PnP yaw sigma on a prop at 2-4 m in the pool; built only if one-shot anchoring leaves more than 2 deg after a 10 m leg |
| `localization_check`, all six stages | — | 5ag.2 | the first pool session |
| `go_to`'s speed ramp | — | 5ag.3 | the same session |
| camera-to-IMU lag, de-rotation gains | — | 5ag.4 | the board on Kutum |
