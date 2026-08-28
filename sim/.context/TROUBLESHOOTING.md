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
| `vision_align` / `vision_move` | never-fail contract holds | see sidecar note |

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
