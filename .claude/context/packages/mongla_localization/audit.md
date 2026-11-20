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
| open | a course naming a class the live model cannot emit is caught by nothing (robosub26 names four) | in progress: the detector publishes its classes |

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

## Read, and judged sound

`inekf.py`, `localization_node.py`, `localization_inputs.py`, `localization_publish.py`,
`command_velocity.py`, `retro.py`, `heading_anchor.py`, `resection.py`, `course_map.py`,
`course_survey.py`, `prop_placement.py`, `floor_plane.py`, `frames.py`, `pose_cluster.py`.
Not yet read line by line: the two image readers now in `mongla_vision/flow/`, which belong
to the next vision read.
