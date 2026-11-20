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
| open | a course naming a class the live model cannot emit is caught by nothing (robosub26 names four) | bead under `mongla_ws-es5`: needs the detector to publish its classes |

## How a mission reaches it, today

| DSL call | What it does | Used by a shipped mission |
|---|---|---|
| `use_course(name)` | loads the priors; gives flow the venue's floor depth | `survey_course` |
| `anchor_on(prop)` | heading zero from a prop's fused pose and its course bearing; tells the filter | `survey_course` |
| `place_prop(prop, commit=)` | a prop's position from sightings | `survey_course` |
| `pose()` · `motion()` | read the filter: pool position (None until anchored or once stale); `ok` / `blocked` / `unknown` | none |
| `fix_position()` · `fix_from_prop()` | push a resected position into the filter | none |
| `absolute_heading()` · `absolute_to_relative()` | turn world headings into the hull's relative `turn()` | none |

**Key point:** no competition mission reads the filter yet. The vehicle's own consumers are
the vision lock's loop closure and the manager's velocity and position uplinks, which the
board does not consume yet (producer only). So the filter is **BUILT, NEVER FLOWN**, and its
first water job is to be watched, not obeyed.

## Ready for the first water test

What water has to decide, in order, all with the stack as it is:

1. Bring up with `localization:=true` (the default) and record a bag. The 5 s diagnostic line
   must show flow and ZUPT fused, not refused; a refused count that climbs is the B-124 case
   now made visible.
2. Tape `frames.yaml` (both lever arms and the baro offset): until then flow is fused without
   its lever arm, and the manager's frames health line says DEGRADED.
3. Anchor on one prop with a taped bearing and check `/mongla/odom` switches to `pool`.
4. Drive a taped straight line and compare the filter's distance with the tape: the truth
   test that ranks it, the one no bench run can.

## Read, and judged sound

`inekf.py`, `localization_node.py`, `localization_inputs.py`, `localization_publish.py`,
`command_velocity.py`, `retro.py`, `heading_anchor.py`, `resection.py`, `course_map.py`,
`course_survey.py`, `prop_placement.py`, `floor_plane.py`, `frames.py`, `pose_cluster.py`.
Not yet read line by line: the two image readers now in `mongla_vision/flow/`, which belong
to the next vision read.
