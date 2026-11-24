# The course file — the venue in one place

Everything the stack must know about a pool lives in one file:
`~/mongla/deck/courses/<venue>.yaml`. That means the floor and its slope, the tile size, the
lane lines, the depth the hull must never pass, and where every prop is, which way it faces
and how wide it is. The deck file names the venue with one line,
`course: <venue>` in `~/mongla/deck/bringup.yaml`, and every package reads the rest from the
course.

**Key point:** a venue number that appears in the course AND in the deck file is refused at
launch. Two copies of one fact means nobody can say which one the run used. A typed
`name:=value` on the command line still wins over both, for a one-off test.

## Where things are on the day

```
~/mongla/deck/                  the deck: what the operator sets, nothing else
    bringup.yaml                the vehicle and the run: models, switches, nodes:
                                course: <venue>   names the venue's file below
    courses/<venue>.yaml        the venue: pool, floor, depth floor, props
    README.txt                  this layout, on the vehicle
~/.mongla -> mongla/deck        a link, so every older path still works
```

`kutum/setup.sh` (the user half) makes the folder and the link and moves an existing
`~/.mongla` into it, `kutum/doctor.sh` checks the link, and `kutum/backup.sh` copies the
folder itself (a link alone would copy as a link). Nothing in the deck is touched by a
deploy or by `release.sh prune`.

The packaged courses in `src/mongla_localization/mongla_localization/courses/` are
templates: classes and nominal sizes, every position unset. The loader reads the deck copy
first (`$MONGLA_COURSE_DIR`, then `~/mongla/deck/courses`, then the package), so a survey
on the day needs no rebuild.

## What the file holds, and who reads each part

| Key | Meaning | Read by | When unset |
|---|---|---|---|
| `frame` | the pool frame, stated once: origin, +x (bearing 0), +y (bearing 90) | people | — |
| `pool: depth_m` | depth of the floor (POSITIVE metres) flow assumes when nothing better is known | bringup → flow `pool_depth_m` | flow has no height |
| `pool: depth_min_m` / `depth_max_m` | where a sloping or stepped floor really lies | bringup → flow; widens the velocity covariance | treated as exact |
| `pool: floor` | soundings `[x, y, depth]` in the pool frame | mission runner → flow `pool_floor_xyz` | no slope model |
| `pool: tile_m` | floor tile pitch: a second height and a yaw bound | bringup → flow `tile_m` | off |
| `pool: lane_lines` | the floor has lane lines (a yaw bound, mod 180) | bringup → flow `lane_lines` | off |
| `pool: shield_max_depth_m` | the manager surfaces a hull past this depth for 1 s | bringup → manager | **no depth floor**, warned at every mission start |
| `props: <name>: detect_class` | the class the detector emits for it | DSL, vision | the prop name |
| `props: <name>: x_m, y_m, depth_m` | where it is | `go_to_prop`, `fix_from_prop`, `resection` | the loader refuses to answer, never a zero |
| `props: <name>: bearing_deg` | the compass direction its face points | `anchor_on`, `go_to_prop` | no heading anchor from it |
| `props: <name>: width_m` | the width of what the detector BOXES, measured on the real prop | the pose path's range (`target_geometry`) | the shipped table's nominal |
| `props: <name>: measured` | true: taped at the venue; false: a nominal or a vision placement | `place_prop` never overwrites a measured prop | false |
| `zones: <name>: area` / `line` | where the rulebook says a prop MAY be: two opposite corners, or a line's two ends, `[x, y]` in the pool frame; `source: rulebook` (a distance its text states) or `figure` (read off its not-to-scale drawing) | Batighor's map and 3D pool, nothing else (a test holds that) | not drawn |

A zone is never a position. The rulebook places the gate "anywhere along a line" and the
orange flare anywhere in a 4 m band, so `sauvc_olympic.yaml` ships its zones and still no
prop position: `go_to_prop` and the prop fixes refuse every prop there until it is surveyed.

Depth signs, because they differ on purpose: `pool:` depths are **positive** metres below
the surface (they are distances to a floor), a prop's `depth_m` is **negative** below the
surface like every depth on the wire.

## Filling it in at the venue

```bash
# the floor: one sounding per point, more where it slopes (pool frame, depth positive)
ros2 run mongla_localization course_survey --course venue \
    --floor 0 0 1.28 --floor 17.5 0 2.0 --floor 8.75 4 1.64 --shield 1.5 --tile 0.25
# a prop: where it is, which way it faces, how wide the detector's box is
ros2 run mongla_localization course_survey --course venue --prop gate \
    --x 16.0 --y 0.0 --bearing 0 --width 1.50
# read it back
ros2 run mongla_localization course_survey --course venue --list
ros2 run mongla_manager bringup_check
```

Each call writes the deck copy and re-reads it through the loader, so a bad value is
refused on the deck, not in the water. A vehicle survey is the other way in:
`mission survey_course` places props from sightings (`place_prop`), and a prop already
`measured: true` is only checked, never moved.

## What stops a faulty start

Two numbers decide whether a run is safe and whether it can navigate.

**Flow's height.** All the vehicle measures is depth; until an altimeter exists, the height
above the floor is the floor's depth minus the hull's. Without a floor, flow refuses every
interval and localization drifts on the IMU alone while looking healthy. So:

- `bringup_check` grades it: PASS with a floor survey or a depth range, WARN otherwise;
- flow publishes where its height comes from, latched, on `/mongla/flow/height_source`:
  `tiles`, `survey`, `survey, no position`, `pool_depth` or `none`;
- the mission runner gives flow the course's floor, then **refuses to start** while that
  topic says `none`. A bench run with no floor says so: `mission <name> --allow-no-height`.

**The depth floor.** Unset, nothing below the mission stops a dive past the pool; the board
has no floor of its own yet (upstream ask Y). It stays a warning, because the deck may be a
bench, and it is printed by `bringup_check` and at every mission start until it is set.

### The floor model on a slope

`flow_math.floor_depth_at` turns the soundings into the floor under the hull: one point is a
flat floor, two (or a line of them) a slope along that line, three or more a least-squares
plane. It never extrapolates past the deepest or shallowest sounding. Its uncertainty is
the plane's fit residual plus the slope times the position's sigma, so a steep floor under a
poorly known position is honestly uncertain.

The survey is used only once localization gives a real pool position: the heading anchored
**and** at least one prop position fix fused (`LocalizationState.anchored` and `fixes > 0`).
An anchor alone turns the axes but leaves the origin where the filter started; reading the
survey there would be a confident wrong floor. Before that, flow scales on the middle of the
soundings with their spread as its uncertainty (`survey, no position`).

**Measure `tile_m` carefully: every height scales with it.** Grout centre to grout centre,
across ten tiles with a tape and divided by ten, on both axes. If the two differ the tiles
are not square, and the grating may read either axis (measured-bars section 101), so say
so in the notes and do not trust tile heights there yet. Before trusting it in water, hold
the downward camera at a taped height over the tiles for ten seconds and grade the clip:
`tools/studies/tile_grating_real.py CLIP --tape-m H --tile-m T --water` (median < 3 %).

**On a slope, set `tile_m` if the floor is tiled.** The tile grating measures the height
directly and comes first (`tiles` above), so neither the survey nor its middle is used while
the grating is read. Without tiles, the gap before the first fix is real: at a 0.72 m spread
the middle is off by up to 0.36 m, which is about 30 % of speed at 1.2 m. Height from the
image's magnification over a barometer-measured depth change was built to close it, and
failed real water: 36 % median error on a harbour seabed (measured-bars §100). Plan the run
so the first prop fix comes early, or survey a flat stretch for the start.

Measured on a synthetic floor sloping 1.28 to 2.00 m over 17.5 m, position known to 1 m:
RMS height error under 0.05 m, against up to 0.72 m for one typed depth
(`src/mongla_vision/test/test_floor_depth_follows_the_slope.py`; the bar as first written,
worst case under 0.10 m, missed at 0.107 m, and the test says why).

## Localization on the day, in order

1. **Bring up** with `course: <venue>` in the deck file. The log line `[bringup] venue from
   course ...` lists every number taken from it.
2. **`bringup_check`** — the flow height and depth floor lines must not WARN on a pool day.
3. **Start the mission.** The runner gives flow the floor and refuses without one.
4. **Anchor the heading** (`mongla.anchor_on('gate')`). The board's yaw is relative to where
   it booted; a prop with a `bearing_deg` makes it absolute. Until then
   `/mongla/localization/state` says `heading not anchored`.
5. **Fix the position** (`mongla.fix_from_prop('gate')`, range and bearing to one surveyed
   prop). From here flow follows the floor survey.
6. **Navigate** with `go_to(x, y)` or `go_to_prop(name)`; each refuses unless the state is
   `usable` and its sigma is inside the arrival tolerance.

Is it working? `ros2 run mongla_planner mongla locate`, or the `reason` field of
`/mongla/localization/state`. Its reasons, in the order to fix them: no IMU sample,
IMU silent, heading not anchored, unaided (no flow and no ZUPT fused).

## Prop widths, and the older override file

A prop's `width_m` in the course sets the width the pose path ranges on, on top of the
shipped table (`mongla_vision/config/target_geometry.yaml`). The older override file
`~/mongla/deck/target_geometry.yaml` is still read, below the course, for classes no course
names; a measured prop belongs in the course.

---

Related: [`mongla_localization`](../packages/mongla_localization/README.md) ·
[mission cookbook](mission-cookbook.md) · [launch combinations](../platform/launch-combinations.md)
