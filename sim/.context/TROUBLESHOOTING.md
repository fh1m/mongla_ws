# Troubleshooting

> ℹ **Absorbed 2026-08-27.** This workspace is no longer the sibling tree
> `Ros_workspaces/duburi-sim_ws`; it lives inside the `duburi_ws` repo at
> `duburi_ws/sim/` and is under version control. Paths below have been
> updated; any remaining "sibling" phrasing is historical.

## Lost manual control / arm NO_ACK

**Cause:** Two managers (or anything else) competing on UDP **14550**, or ArduSub
failsafe on pilot input.

**Fix:**

```zsh
ros2 run duburi_sim_bringup duburi_sim stop
ros2 run duburi_sim_bringup duburi_sim sim
ros2 run duburi_sim_bringup duburi_sim stack --no-vision
# ensure only one /duburi_manager
ros2 node list | grep duburi_manager
```

Sim params set `FS_PILOT_INPUT` appropriately in `duburi_sub.parm`.

## No AUV in Gazebo / empty world

**Cause:** Second headless sim already owns world / FDM port **9002**.

**Fix:** `duburi_sim stop`, then one `sim` with GUI. Look at **x ≈ −11.8**.

## Black / missing lab cameras

1. Confirm `contract_check` and `/api/cameras/front/jpeg` returns image/jpeg.  
2. Status `link.cams` / `cameras.front`.  
3. Rebuild frontend static if UI is stale.  
4. MJPEG URL: `/api/cameras/front/mjpeg` — some clients show first frame slowly.

## Lab connection refused / wrong port

Lab auto-binds if 28765 busy (Electron often steals 8088 historically).

```zsh
cat /tmp/duburi_lab_port.txt   # if your launcher wrote it
echo $DUBURI_LAB_PORT
ss -tlnp | grep 2876
```

Open `http://localhost:<actual_port>/`.

## Teleop connected false / no motion

- Arm first (`● arm` or planner `arm`).
- Hold D-pad (not tap); teleop only streams while axes non-zero.
- Endpoint must be `tcp:127.0.0.1:5763` (ArduSub serial1-style TCP).
- Do not point teleop at 14550.

## Record stop fails / empty zip / no meta.json

- Wait until recording is actually live (lab waits for `.ready`).
- Very short clips can finalize with `counts: 0` (warning in stdout).
- Orphan `record_cameras`: `pkill -f record_cameras` then retry.
- Dataset API hides dirs without `meta.json`.

## Props spawn errors

- Ensure `prop_manager` running with matching `world:={course}`.
- Lab World restart restarts prop_manager after course change.
- `props list` shows **catalog**, not spawned names — use your instance `name` for remove.

## X11 / Gazebo GUI fails

- Set `DISPLAY` and readable `XAUTHORITY` (helper tries mutter cookie).
- Or use `duburi_sim sim --headless` (physics still runs).

## gz-transport flaky / commands dropped

```zsh
export GZ_IP=127.0.0.1
```

Required on hosts with many NICs; launch also sets this.

## Verb audit — what each verb PHYSICALLY does

Measured against `/duburi/sim/ground_truth`, 2026-08-28. Rerun with:

```bash
ros2 run duburi_sim_bridge verb_audit            # all cases
ros2 run duburi_sim_bridge verb_audit --only turn,arc
```

The stack's telemetry cannot referee itself — that is how the AHRS2 depth offset
and the `move_forward_dist` dead-reckoning both survived. Two failure shapes:
**Type A** (verb fails, physics succeeded — `surface()`) and **Type B** (verb
succeeds, physics did something else — `move_forward_dist`). Type B is the
dangerous one.

| verb | measured | verdict |
|---|---|---|
| `move_forward` | +2.80 m body-x, 0.01 m cross | ok |
| `move_back` | −2.82 m body-x | ok |
| `move_right` | starboard; Ch6 = 1720 (>1500) | ok |
| `move_left` | port | ok |
| `yaw_right` 45° | GT −44.09° (CW) | ok, 0.9° error |
| `yaw_left` 45° | GT +43.87° (CCW) | ok |
| `turn` 60/120/0 | stack 58.6 / 118.8 / 0.9 | ok, ≤1.4° error |
| `arc` | +20.6° over a 5 s curve | ok |
| `head` | reports heading | ok |
| `style_yaw` | −356.5° (one flip) | ok |
| `style_roll` | +362° reported | ok |
| `set_depth` −0.5 / −1.0 | true z −0.522 / −1.025 | ok, ≤2.5 cm |
| `set_mode ALT_HOLD` | MAVLink HEARTBEAT confirms | ok |
| `lock_heading` / `unlock_heading` | locks, releases | **inconclusive — see below** |
| `move_forward_dist` 1 m | 1.26 m (was 2.361 m) | ok since the DVL landed |
| `move_back_dist` 1 m | −1.11 m body-x, DVL said 1.04 | ok |
| `move_lateral_dist` 1 m | 1.15 m starboard, DVL said 1.03 | ok |
| `fire` | fails loudly | ok — no payload hardware in sim |
| `dvl_connect` | fails without a DVL | ok since it stopped claiming success |

### `lock_heading` cannot be validated in sim

The disturbance test (lock, then strafe, then measure heading) is **inconclusive
here**: the hull drifts only 1.4° over a 6 s strafe *without* the lock, so there
is nothing for the lock to resist and locked-vs-unlocked cannot be separated.

The simulator under-represents the lateral-to-yaw coupling of the real hull —
the coupling that `CLAUDE.md` documents as causing align-yaw jitter, and that
`heading_lock`'s tapered floor exists to handle. **Heading-lock tuning must be
validated in the pool, not here.**

### Two traps for anyone extending the audit

1. **Never measure rotation from a before/after pair.** The hull carries angular
   momentum through the settle and wraps past ±180°, so the same teleop input
   measured −27.7° once and +169.7° the next time. Sample continuously and
   accumulate unwrapped, which `verb_audit.py` does. To settle a direction
   question, the RC PWM is unambiguous where ground truth is not.
2. **The two yaw frames differ by a negation AND an offset.** Measured, with
   residuals of −0.0 / −0.2 / +0.2° over targets 60 / 120 / 0:

   ```
   gt_yaw = 90 - stack_yaw
   ```

   The stack is a compass heading (CW-positive, zero at north); ground truth is
   ENU (CCW-positive, zero at +x/east). Using the negation alone reports an ~88°
   error on a turn that is accurate to 1.4° — a correct verb flagged Type B by a
   wrong harness.

## Verb audit — what each verb PHYSICALLY does

Measured against `/duburi/sim/ground_truth`, not against the verb's own result.
Rerun with `ros2 run duburi_sim_bridge verb_audit`.

Two failure shapes are worth naming, because unit tests cannot see either:

- **Type A, false negative** — the verb fails, the physics succeeded.
- **Type B, false positive** — the verb reports success while doing nothing, the
  wrong thing, or something it cannot measure. Far more dangerous.

| verb | measured | verdict |
|---|---|---|
| `move_forward` / `move_back` | +2.80 m / −2.82 m body-x, cross-track ~0.01 m | OK |
| `move_right` / `move_left` | −2.53 m / +2.48 m body-y (**+y is PORT**, REP-103) | OK |
| `yaw_right` / `yaw_left` | 45° commanded → 44.1° / 43.9° ground truth | OK |
| `turn` (absolute) | 45° commanded → 43.7° | OK |
| `head` | reports 46.5° vs truth 43.6° | OK |
| `lock_heading` | **0.0° drift under lateral thrust** (the disturbance test) | OK |
| `unlock_heading` | releases | OK |
| `set_mode` | `/duburi/state` confirms ALT_HOLD and MANUAL | OK |
| `set_depth` | true depth within 2.5 cm of command | OK |
| `style_yaw` | −58.6° rotation | moves; times out short (see below) |
| `style_roll` | needs headroom; inconclusive near the surface | retest at depth |
| `arc` | drives forward 0.58 m vs 0.60 m control ✓, heading short | **fixed, see below** |
| `move_forward_dist` | 1.0 m → 1.26 m true (was **2.361 m**) | fixed last round |
| `move_back_dist` / `move_lateral_dist` | refuse without DVL; stall guard was too tight | guard widened |
| `fire` | fails loudly with no payload | OK |
| `surface` | never confirms in sim | Type A, documented above |
| `vision_align` / `vision_move` | never-fail contract holds | see "class allowlist" below |

### `Failed to render beam markers` in a headless run

Harmless. `<visualize>true</visualize>` on the DVL draws the four beams in the
**Gazebo GUI**, and headless has no renderer for GUI markers, so gz warns once
per attempt. The beams are still published as an RViz `MarkerArray` on
`/duburi/sim/dvl/beams`, which is the path that works headless.

### Verifying RViz displays: count subscribers BY NAME

`ros2 run duburi_sim_bridge rviz_check` asserts every topic the config
references is subscribed **by RViz specifically**, not merely subscribed.

The first version counted subscribers and gave a **false pass**: `underwater_fx`
also subscribes to both camera `image_raw` topics, so the count was >=1 whether
or not the display was switched on -- and the camera displays were in fact
saved with `Enabled: false`. `ros2 topic info -v` names the subscribing nodes;
ask for RViz by name.

Two related traps:
- RViz **rewrites the config on exit**. If it was opened without the sim running
  it will have reset `Fixed Frame` (no `odom` frame exists) and left displays
  off, and then saved that. A `*` in the title bar means the file on disk is
  about to change.
- A display that is off looks identical to a display whose topic is wrong.

### The vehicle shadows the drum it is about to drop into

Measured from the bottom camera hovering over a drum, i.e. the exact instant of
the Target Acquisition drop:

| surface | RGB | colour spread |
|---|---|---|
| pool floor (control) | `[125, 154, 172]` | **46.8** |
| drum base, emissive 0.25 | `[115, 122, 129]` | **13.5** |
| drum base, emissive 0.55 | `[138, 149, 160]` | **22.0** |

The drum base is the entire colour cue for choosing blue-vs-red, and at drop
range it desaturates towards grey. This is **not a texture failure** -- the
albedo maps are correct (`drum_wall_blue.png` mean RGB `[18, 59, 189]`) and the
drums render distinctly blue and red from the FRONT camera at normal range. It
is the vehicle casting its own shadow on the prop directly beneath it.

That is physically right, and it is a real problem competition teams have. The
emissive lift on the drum base is raised to 0.55 to recover what can be
recovered without making the drum glow. **Do not tune a bin detector on the
front camera and assume it transfers to the drop.**

Measure it yourself rather than eyeballing a fogged frame:

```bash
ros2 topic echo /duburi/sim/bottom_camera/image_raw --once   # RAW render
```

`image_raw` is the clean Gazebo render; `underwater_fx` publishes its degraded
copy to **`image_fx`**, a different topic. Measuring the wrong one will tell you
the renderer is broken when it is not.

### The class allowlist, and the silent `[]`

A detector that matches **zero** classes returns `[]` on every frame forever. It
is not an error and the pipeline looks healthy: topics publish, FPS is normal,
the HUD draws. The one signal is a single line at startup:

```
[YOLO ] class_allowlist=[...] matched 0 classes in model. Detector will return []
        for every frame. Available: [...]
```

Two things make this easy to trip:

- **`bin_fire_blood.pt` has NO `bin` class.** The stem names the dataset, not the
  classes — the weights contain exactly `{0: blood, 1: fire}`. A bin mission
  asking for `classes:=bin` detects nothing, silently. The sim's downward
  detector therefore defaults to `dwn_classes:=fire,blood`.
- **A missing `<stem>.yaml` sidecar is NOT itself the failure.** The loader falls
  back to the weights' embedded `names`. The failure is when the allowlist a
  mission asks for does not intersect whatever names are in play.

`gate_rescue_repair.pt` is `{0: gate, 1: rescue, 2: repair}`.

### A live sim/vision stack fails the autonomy tests

`test_detected_live.py::test_where_uses_live_camera_info` failed once during
this round and passed in isolation. Cause: the vision stack was still running,
publishing real detections onto the same topics the test spins its own node on.

Shut the sim and vision down before `colcon test`. A test that passes alone and
fails in the suite is usually ordering; a test that passes alone and fails with
the suite *while a stack is up* is the ROS graph, not the test.

### `ShaderParam` DOES reach camera sensors (unlike `<fog>`) — verified

Worth stating because the obvious prior is wrong. gz-sim renders **two separate
scenes**: a server-side one created by the `Sensors` system, which is what
cameras see, and a client-side one created by `MinimalScene`, which is what the
GUI shows. The world's `<scene><fog>` reaches only the second — that is the whole
turbidity bug above. So "custom shaders work" needed proving, not assuming.

**They work.** Controlled A/B on one scene, headless, only the plugin differing:

| | result |
|---|---|
| Gerstner shader on | rippled, shaded water surface with a horizon |
| shader stripped, same mesh, same pose | flat untextured plane |

48.7 % of pixels changed, mean absolute difference 61/255.

**Two traps cost most of the time it took to establish that**, both worth knowing:

1. ~~**A shader that fails renders NOTHING, and logs nothing.**~~ **RETRACTED
   2026-09-01 (round 23), and the truth is more useful.** A fragment shader with
   a deliberate syntax error does not fail quietly — it **aborts the server**:

   ```
   OGRE EXCEPTION(3:RenderingAPIException): Fragment Program
     _gz_PoolSurface_fs_330.glsl failed to compile.
   process has died [exit code -6, cmd 'gz sim -v 2 -s -r ...']
   ```

   That is at `-v 2`, the verbosity the sim actually runs at. So on gz-sim 8 a
   **live sim is positive evidence that the GLSL compiled**, which is a much
   stronger diagnostic than the old line allowed — it was what proved round 23's
   water shader was compiling and being applied while still producing a static
   image. The original observation (a visual that vanished with no log) was real
   but had a different cause; do not read a missing object as a failed compile.
2. **Check framing with a control before concluding anything.** Two intermediate
   readings here were nonsense because the 1000 m wave plane was simply outside
   the camera frustum. The control that settled it was the *same mesh at the same
   pose with the plugin stripped* — if the control is also invisible, the finding
   is about framing, not about the feature.

### The animated bounded surface: geometry IN, shader OUT (round 23)

The surface is a **subdivided, double-sided mesh** now (`gen_prop_meshes.water_grid`,
one per pool: 20x12 and 25x16, 0.25 m cells). It is **not animated**. Everything
below is measured, so the next attempt starts here instead of from scratch.

**Three defects on the way, and every one of them produced a plausible number
rather than an error.** The order matters: each was hidden by the one before it.

| # | what looked true | what was true | what caught it |
|---|---|---|---|
| 1 | "the shader does not animate" | the water surface **was not in frame at all** — the pose was pinned on `duburi`, and a buoyant hull with CoB above CoM rights itself in milliseconds, so a pitched-up camera is level again before the next frame | painting the surface opaque magenta and counting magenta pixels: **0.00 %** |
| 2 | "the mesh fails to load" | the visual had **no `<geometry>` wrapper**. `_geometry_box()` wraps; the hand-written `<mesh>` string did not, SDF dropped it, and the visual rendered nothing — silently | box control also 0.00 % magenta, which ruled the mesh out |
| 3 | "the shader is inert" | the sheet was **single-sided facing +z**, and every camera here is UNDER it, so Ogre2 culled it. Not an error, nothing logged | box (has a downward face) **100 %** magenta vs mesh **0.00 %**, same material, same pose |

**The magenta control is the tool.** A frame-to-frame diff cannot tell "static
surface" from "no surface"; an unmistakable colour can. Three interpretations
were wrong in a row because the diff kept returning a number.

**Where it actually stands.** With geometry fixed and the surface filling the
frame, the shader:

- **compiles** — a deliberately broken fragment shader aborts gz (see the
  retraction above), and this one does not, so it is being built and bound;
- **is applied** — **100 % of pixels** differ from the same visual with the
  plugin stripped (97.667 vs 111.452 mean);
- **does not execute its fragment stage.** A fragment shader consisting of
  nothing but `fragColor = vec4(1.0, 0.0, 1.0, 1.0)` — **no uniforms at all** —
  renders **0.00 % magenta** and the same **97.667**. That is the whole answer,
  and it took a fourth wrong diagnosis to get to it.

**RETRACTING THIS ROUND'S OWN FIRST CONCLUSION.** It was written up, committed
and pushed as *"every `<param>` arrives as zero"*, inferred from two experiments
— rewriting the fragment shader wholesale, and changing `tau` by three orders of
magnitude. **Neither one discriminated.** Under a zero-uniform hypothesis both
shaders reduce to `col = vec3(0)` with `alpha = 0`, so both were invisible
either way; `tau` only scales an amplitude that was already zero. Two
experiments that cannot distinguish the hypotheses were read as confirming one.
The uniform-free magenta shader is the test that separates them, and it is one
sim launch.

So the open question is **why the custom fragment program is not running**, not
`<param>` delivery. Three distinct means bracket it: **122.836** with no water
surface at all, **111.452** with the surface and its normal material, **97.667**
with ShaderParam attached — so the visual *is* still drawn with the plugin on,
just not by the shader as written. The wiring emitted was
`<plugin filename="gz-sim-shader-param-system">` on the visual, shader paths
relative to the `.world`, `float_array` for the vectors and `<value>TIME</value>`
with no `<type>` for `t`, copied from the Fuel `waves` model — which is authored
for **ign-gazebo6**, not gz-sim 8, and is the first thing to check.

**The mesh is kept** because it is verified equivalent to the box it replaced
(probe-camera mean **111.452 vs 111.424**) and because a `<box>` is 8 vertices —
having nothing to displace is exactly why this item was carried and cut across
rounds 12, 13, 14, 19 and 22.

**Not a defect**: Ogre logs three `Cannot locate an appropriate 2D texture
coordinate set … to create tangents` exceptions on `rs_task_slalom`, which
contains neither `torpedo_plate.obj` nor `crate_wall.obj`. Counted both ways:
**3 with the water mesh, 3 with the box**. Pre-existing, and not the water sheet.

**Measurement rig**: spawn a static probe model with its own camera and read it
off gz-transport. Do **not** use `EntityFactory`'s `sdf:` pose — it is
overridden by the request's own `pose` field, and a probe authored with a pose
in its SDF spawns at the origin looking at sky, reporting an identical mean in
every arm.

### Animated water surface: `water_surface: gerstner`

```yaml
scene:
  lighting: competition
  water_surface: gerstner    # plane (default) | gerstner | none
```

Uses **Gazebo's own first-party `openrobotics/waves` model**, referenced by Fuel
URI rather than vendored, so Gazebo downloads and caches it and nothing of
unclear provenance enters this repo. **The first run of a gerstner course needs
network.** Its shaders carry the Apache-2.0 UUV Simulator header; other teams'
copies of this model are copies of exactly this one, and theirs ship the meshes
as git-lfs pointer files rather than usable geometry.

The static generated plane is still the default and is skipped when gerstner is
selected — two surfaces at z=0 z-fight.

Only visible looking up or near the surface: at 0.82 m depth on a level camera
it is out of frame entirely, which is correct.

### Turbidity: the world's `<fog>` is INERT, `underwater_fx` is the lever

**Measured 2026-08-28 on gz-sim 8 (Harmonic): `<scene><fog>` has no effect on
camera-sensor renders.** Dropping `<end>` from 18 m to 3 m in `sauvc26_final`
left the 25 m far wall pixel-for-pixel identical. The same edited world *did*
lose its clouds when `<sky>` was stripped, which proves the file was actually
loaded — so this is a real no-op, not a stale-build artifact.

That mattered because the three `lighting:` presets (`clear` / `competition` /
`murky`) only ever wrote fog numbers. They were documented as "the single
biggest lever on how hard the perception task is" and they changed **nothing**
in the image. Every dataset captured before this date has the same clarity
regardless of which preset its course named.

**Now:** each preset also carries an `fx` block. `gen_world.py` writes it beside
the world as `<course>.fx.yaml`; `bridge.launch.py` layers it over
`config/underwater_fx.yaml` and hands it to `underwater_fx`, which post-processes
`image_raw` -> `image_fx` in ROS. Verify a course's water actually took:

```bash
ros2 param get /underwater_fx turbidity      # murky -> 0.8, competition -> 0.45
```

Do **not** tune turbidity by editing `<fog>` in a world or the template. Nothing
will change and the numbers will read as authoritative to the next person.

> **RETRACTED 2026-08-31.** This paragraph used to say attenuation was "still
> uniform, not range-dependent … Not done", and recommended switching the front
> camera to an `rgbd_camera`. Both halves are wrong now.
>
> **Per-pixel Beer–Lambert is built and verified** (`underwater_fx.py:268-290`),
> keyed on a range image. It is **off by default**, and only because the two
> extra render passes took the cameras from **12 Hz to 4 Hz**
> (`PHYSICS.md:361`). Turn it on with `range_cameras: true` in
> `duburi_heavy/configs.yaml` and accept that cost knowingly.
>
> And the `rgbd_camera` route it recommended was **consciously rejected**
> (`model.sdf.in:229-232`): it moves the colour topic and breaks the `image_raw`
> contract every consumer relies on. A parallel `depth_camera` was added
> instead. Leaving the recommendation in place pointed the next reader straight
> at the one design that had already been discarded — which is exactly how a
> stale doc costs more than no doc.

### Vision on sim cameras silently received zero frames (QoS) — FIXED

`RosTopicCamera` subscribed with the rclpy default (depth-10 **RELIABLE**). Every
camera publisher we point it at — Gazebo's `ros_gz` `image_bridge`,
`underwater_fx`, BlueOS `image_transport` — publishes **BEST_EFFORT**. A RELIABLE
subscriber is QoS-incompatible with a BEST_EFFORT publisher, and rclpy does not
raise: one WARN line, then nothing delivered, forever.

The failure looks like success. Every node starts, no traceback, the launch is
clean, `ros2 topic list` shows all the vision topics — and the detector sees zero
frames. The only visible symptom was one line buried in the launch log:

```
New publisher discovered on topic '...', offering incompatible QoS.
No messages will be received from it. Last incompatible policy: RELIABILITY
```

Fixed by subscribing with `qos_profile_sensor_data`. BEST_EFFORT subscribers
accept **both** kinds of publisher, so this is strictly more permissive.

If you write your own probe against a sim camera topic, use sensor QoS or you
will reproduce the bug in your tool and blame the sim.

### Running the full vision pipeline on the sim's cameras

```bash
# T1  sim (image_raw -> underwater_fx -> image_fx)
ros2 run duburi_sim_bringup duburi_sim sim --headless course:=sauvc26_final

# T2  both detectors on the sim's two cameras
ros2 launch duburi_vision vision_dual.launch.py \
    fwd_topic:=/duburi/sim/front_camera/image_fx \
    dwn_topic:=/duburi/sim/bottom_camera/image_fx \
    model:=gate_rescue_repair dwn_model:=bin_fire_blood \
    device_cls:=cpu paused:=false
```

**`paused:=false` is not optional here.** `vision_dual.launch.py` defaults
`paused:=true` on purpose — missions resume the one detector a task needs — so
without it the HUD sits at `det=ERR trk=ERR` with `dets=0` and looks broken. It
is not; nothing has resumed the detector. Resume live with
`ros2 param set /duburi_detector_forward paused false`.

`device_cls:=cpu` on a box with no CUDA: registry mode raises and the detector
node **dies** if left at the `cuda:0` default.

Point at `image_fx`, not `image_raw`, or the model sees water that no pool has.

Verified end to end 2026-08-28: `cam=OK det=OK trk=OK`, boxes drawn on
`image_debug`. Note the competition weights are trained on *real* RoboSub props
and mis-fire on the SAUVC sim geometry (a floor edge scored `gate 46%`) — which
is the argument for capturing a sim dataset and training against it, not a
pipeline fault.

### Vision in sim: `vision:=true` used to start nothing at all

Fixed 2026-08-28. `IncludeLaunchDescription` does not scope its
`launch_arguments`, so the manager include's `vision: 'false'` leaked into the
outer scope and overwrote `stack.launch.py`'s own `vision` argument. The vision
include then skipped itself via `IfCondition` — no error, no node, for any value
of `vision:=`. That is why the older notes here tell you to run `--no-vision`.

Both cameras now run: `duburi_sim stack` gives
`/duburi_detector_forward` (sim front camera) and `/duburi_detector_downward`
(sim bottom camera). If only one appears, check the `GroupAction(scoped=True)`
around the manager include is still there — `test_sim_contract_drift.py` asserts
it precisely because the failure is silent.

### `arc` reported a perfect result while 165° off  — FIXED

`command-reference.md` advertises arc's `error_value` as "heading drift vs
expected". It was a hardcoded `0.0`. Commanded to `target_yaw=200` for 6 s the
hull finished at **5.1°** and reported `err=0.000, "arc: completed"`.

Ending short is legitimate — `duration` bounds the manoeuvre. Claiming to have
ended on target is not. It now reports
`completed at 5.1deg (-165.1deg from target 200)`.

### Two open items

- **`style_yaw` times out.** It rotates (−58.6° measured) but does not reach its
  target inside 30 s: `yaw_style_yaw timeout after 30.0s -- cur=162.6 tgt=199.4
  err=+36.8`. Tuning, not a false report — it fails honestly.
- **`style_roll` needs vertical headroom.** Near the surface it has nowhere to
  go; retest at depth before drawing a conclusion.

### Traps when adding checks to the harness

- **Body +y is PORT** (REP-103). An earlier pass wrongly flagged
  `move_left`/`move_right` as inverted over this.
- **Ground truth cannot settle a yaw DIRECTION.** The hull carries angular
  momentum through the settle and wraps past ±180°, so the measured sign flips
  between runs. Use the RC PWM instead: `Ch4 > 1500 = RIGHT`.
- **Reset the sim between runs.** The hull drifts into a wall (pool spans
  x = ±12.5) and then nothing moves, which reads exactly like a dead verb.

## `surface()` times out / `calibrate_depth` refuses / depth reads deeper than it is

**One cause, three symptoms.** Measured 2026-08-27 against ground truth.

Depth telemetry is read from MAVLink **`AHRS2.altitude`** — ArduSub's *secondary*
DCM estimate. ArduSub closes its own depth loop on **EKF3**. In the simulator the
two disagree:

| true `z` (ground truth) | `AHRS2.altitude` (what we read) | `GLOBAL_POSITION_INT.relative_alt` (EKF3) | error in ours |
|---|---|---|---|
| −0.036 m (floating, at rest) | **−0.370** | −0.028 | **0.334 m** |
| −0.522 m | **−0.680** | — | 0.158 m |
| −1.025 m | **−1.180** | — | 0.155 m |

**The offset is not constant — it is largest near the surface** (~0.33 m there,
~0.16 m at depth). Do not memorise one number; the surface case is precisely the
one that breaks things.

What follows from it:

- **`surface()` never confirms.** It targets 0.00 m and waits on the number *we*
  can see. AHRS2 plateaus near −0.4, so it burns its full ascent budget and
  raises, *after* the hull has physically surfaced (ground truth `z` returns to
  −0.036). The vehicle is fine; the confirmation is not.
- **`mission_reset()`'s baro re-zero is refused.** `calibrate_depth` is gated on
  `|depth| <= 0.30 m` as a surface proxy, and AHRS2 reads −0.36 while floating.
  You will see `[BARO ] REFUSE depth calibration -- pre-cal depth -0.36m exceeds
  surface bound 0.30m`. The two interlock: the re-zero that would fix the offset
  is blocked *by* the offset.
- **`set_depth` reports a deeper `final=` than you commanded** — e.g. `final=-1.370`
  for a −1.20 m command. **The hull is at the right depth**; measured true `z` was
  within 2.5 cm of the command at both −0.5 and −1.0 m. Only the readback is off.

**Not a bug to "fix" by changing the depth source.** `AHRS2.altitude` is the
pool-verified path on the real vehicle, where a surface `calibrate_depth` zeroes
the Bar30 properly. Missions that must survive both should treat a `surface()`
timeout as non-fatal in sim and real on hardware — see
`missions/sim_shakedown.py` for the pattern.

## Depth bouncing / GPS spam

Historical: duplicate sims; fake GPS. Current `duburi_sub.parm` disables GPS
noise paths — keep one sim only.

## Stack cannot find duburi_ws

```zsh
export DUBURI_WS=/path/to/duburi_ws
# must contain install/setup.bash
```

## srot branch hunts USB board

Always use `duburi_sim stack` (forces `flight_controller:=pixhawk`) or pass that
arg yourself. See [INTEGRATION_DUBURI_WS.md](INTEGRATION_DUBURI_WS.md).


## Marine snow is in the GUI but not in any image topic

**Gazebo particle emitters do not render to camera sensors.** This is the same
GUI-scene / sensor-scene split that makes `<scene><fog>` inert: gz-sim renders
one scene for the GUI and another for sensors, and particles only reach the
first.

Measured: 0.4 m particles at 4000/s, the emitter confirmed alive (`gz topic -i
-t /marine_snow` shows a subscriber, the rate command returns rc=0), and the
frame off `/duburi/sim/front_camera/image_fx` came back as clean as with the
emitter switched off. Per-pixel stddev over 14 frames was **1.4700 with snow
and 1.4678 without** — indistinguishable, and the "without" arm was marginally
higher.

So **raising the `snow:` rate in a course will make the GUI prettier and change
no dataset whatsoever.** The particulate the vision pipeline sees is composited
in `underwater_fx.ParticleField` and is controlled by the `particulate`
parameter:

```bash
ros2 param set /underwater_fx particulate 0.6    # more
ros2 param set /underwater_fx particulate 0.0    # off
```

Measured cost on `image_fx`: 6.35 → 6.06 Hz (4.6 %). The GUI emitter at 900/s
costs nothing measurable (7.63 vs 7.61 Hz on `image_raw` with the GUI up).

Particles there **persist and drift** rather than being resampled per frame.
That is deliberate: a per-frame speckle is just `noise`, which the filter
already applies and which a detector ignores. Coherent motes that move slowly
are what put spurious small blobs in front of a bounding box across consecutive
frames. `test_particulate.py` asserts both halves — that the field moves at all,
and that it does not move so far in 100 ms that consecutive frames share no
particles.

## Lab loads a blank page: `/` is 200 but `/assets/index-<hash>.js` is 404

The served `index.html` is current and the assets it names are from the
previous frontend build. Nothing logs an error — the page is delivered, the
browser then fails to fetch its own bundle, and the served directory looks
populated because it is full of *older* hashed files.

Cause: `colcon build` installs `index.html` as a **symlink back to source**, so
it tracks every `npm run build` immediately — but `setup.py::_static_files()`
enumerates the hashed asset **filenames** at colcon-build time. Rebuild the
frontend without rebuilding the package and the two halves are from different
builds.

`server._static_dir()` now rejects a candidate directory whose `index.html`
references assets that are not in it, and falls through to the next candidate
(the source `static/`, which is always self-consistent). So the lab serves a
working page either way, and prints a WARNING naming the stale directory.

To clear the warning properly, rebuild the package after a frontend build:

```bash
cd sim && ./build_sim.sh          # or: colcon build --base-paths src --packages-select duburi_sim_web
```

Note the installed `static/assets/` accumulates every past build's hashes plus
dangling symlinks into `build/`. That is untidy, not broken — the page only
ever loads the two names its own `index.html` cites.

## `surface()` never confirms, and `mission_reset` refuses to re-zero the baro

Both were one cause, and it was in the COURSE, not the autonomy code.

The stack reads depth from `AHRS2.altitude` — the pool-verified hardware path.
On the real vehicle that reads true depth because the hull is powered on
*floating at the surface*, so the reference is captured there. Every sim course
**spawned the vehicle submerged**, each at its own depth, so the reference was
captured under water and every later reading carried a constant offset.
Measured against Gazebo ground truth (steady to four decimals for 220 s,
identical armed and disarmed):

| course | spawn z | offset |
|---|---|---|
| `sauvc26_qualification` | −0.8 | **−0.344 m** |
| `robosub26_full` | −0.5 | −0.044 m |
| `sauvc26_final` | −0.3 | +0.016 m |

The offset tracks the spawn depth. Consequences, which interlock:

- `surface()` commands 0.0 m. ArduSub takes the hull up correctly — it controls
  on **EKF3, whose altitude is accurate** (−0.007 against a true −0.036) — but
  the readback plateaus near −0.4, so the verb never confirms.
- `mission_reset()`'s baro re-zero is REFUSED: pre-cal −0.38 exceeds
  `_BARO_SURFACE_BOUND_M` (0.30). **Do not widen that bound** — it catches a
  real pool baro fault.

**Fix: every course now spawns at −0.4 m**, near the surface like a real launch.
Measured on the worst course afterwards: offset +0.017 m, `surface()` confirms
in 12.3 s with the hull genuinely at −0.074 m, and depth readback tracks truth
to 7 mm at −0.95 m.

`depth_reference` (in `duburi_sim_bridge`, on by default) re-measures this at
every startup and fails loudly if a new course spawns too deep. It only
reports — see below for why it must never correct.

### Two things that look like fixes and are not

**`BARO_ALT_OFFSET`.** It zeroes the surface reading, and then the barometer
**stops tracking depth**: measured, readback frozen at −0.030 m with the hull
at −1.206 m, so `surface()` *CONFIRMED while 1.2 m down*. A false pass is worse
than the hang it replaced.

**Calibrating the baro.** ArduSub SITL **ACKs `MAV_CMD_PREFLIGHT_CALIBRATION`
as ACCEPTED without calibrating** — the exact command `calibrate_depth` and
QGC's "Calibrate Pressure" send. At best nothing happens; at worst it re-zeros
ground pressure treating water pressure as *air*, and a few centimetres of hull
draft became **+20.3 m** of apparent altitude, after which every depth verb was
garbage. The sim launch therefore passes `baro_calibration:=false`
(`stack.launch.py` → `bringup.launch.py` → the manager). **The pool default is
unchanged and stays `true`** — on the real Bar30 that calibration is real and is
what fixes the pre-dive drift.
