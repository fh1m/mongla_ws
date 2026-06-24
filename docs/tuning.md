# Tuning guide

---

## Depth — onboard ArduSub controller

Depth is driven by `SET_POSITION_TARGET_GLOBAL_INT` to ArduSub's
onboard 400 Hz position PID. No Python PID in the live path.

Flow in `motion_depth.hold_depth`:

1. Engage `ALT_HOLD` (the only mode that honours absolute Z setpoints).
2. **`prime_alt_hold` (0.5 s)** — stream the *current* depth as target while sending neutral RC. Drains the stale ALT_HOLD integrator from the previous mode (known ArduSub quirk).
3. **`wait_for_depth`** — stream the real target at 5 Hz; exit when `|error| < 0.07 m` for 0.5 s.

Tune depth response on the **ArduSub side** via QGC (params live on Pixhawk):

| ArduSub param   | Effect |
|-----------------|--------|
| `PSC_POSZ_P`    | Depth position gain. Default 1.0. Raise for snappier response. |
| `PSC_VELZ_P`    | Depth velocity gain. Default 5.0. |
| `PILOT_SPD_UP`  | Max ascend rate (cm/s). Default 50. |
| `PILOT_SPD_DN`  | Max descend rate (cm/s). Default 50. |
| `PILOT_ACCEL_Z` | Vertical accel limit. Lower = smoother profile. |

PID theory: [`.claude/context/pid-theory.md`](../../.claude/context/pid-theory.md).

---

## Vision gains

P-gains and grace tune live between goals — no restart needed (they apply
on the **next** vision goal):

```bash
ros2 param set /duburi_manager vision.kp_yaw 80.0
ros2 param set /duburi_manager vision.kp_forward 180.0
ros2 param set /duburi_manager vision.lost_grace_s 1.5
```

| Param | Sim default | Pool recommendation | Effect |
|-------|-------------|---------------------|--------|
| `vision.kp_yaw` / `vision.kp_lat` | 60.0 | 60–80 | Raise if centring is sluggish; lower if oscillating |
| `vision.kp_depth` | 0.05 | 0.05–0.10 | Small — depth nudges accumulate |
| `vision.kp_forward` | 200.0 | 150–250 | `vision_move` forward gain; large OK because fill error is small |
| `vision.lost_grace_s` | 1.0 s | 1.0–1.5 s | Coast on target loss before reporting `LOST` (then the DSL `fallback` runs). Raise for turbid pools |
| `vision.frame_fill_default` | 95.0 | per-task | `vision_move` fill target when the mission leaves `fwd` at 0 |
| `vision.align_stable_frames` | 3.0 | 3–5 | Ticks each axis must hold within `err_px` before `vision_align` reports `ALIGNED` |

`err` (pixel tolerance) and `gain` (hard max-speed cap, % thrust) are
**per-call** on `vision.align(...)` / `vision.move(...)`, not ROS params —
tighten `err` and lower `gain` for a slow, precise final lock.

Full param descriptions: [`configuration.md`](configuration.md#vision-parameters-vision).

---

## Smoothing flags

Two independent flags on `auv_manager_node`:

| Flag                | Math | When to enable |
|---------------------|------|----------------|
| `smooth_yaw`        | Setpoint swept via `smootherstep(t/dur)` | Yaw overshoots or fights inertia |
| `smooth_translate`  | Thrust = `gain × trapezoid_ramp(t, dur, ramp=0.4 s)` | Lurch at start or backward drift at end |

```bash
ros2 run duburi_manager start --ros-args \
    -p smooth_yaw:=true -p smooth_translate:=true
```

---

## Translation brake

| Variant             | Exit behaviour |
|---------------------|----------------|
| `drive_*_constant`  | Reverse kick 25% × 0.2 s, then 1.2 s settle. Needed because constant gain exits at full velocity. |
| `drive_*_eased`     | No reverse kick. 1.2 s settle — the trapezoid ease-out is the brake. |
| `arc`               | Streams Ch5+Ch4 for `duration` s, then settles via `final_settle()`. |

Per-verb `settle=` extends the post-command neutral hold for extra inertia bleed-off.

Tunables at the top of `motion_writers.py`:

```python
THRUST_RATE_HZ   = 20.0    # RC override publish rate
EASE_SECONDS     = 0.4     # ease-in/out duration for drive_*_eased
REVERSE_KICK_PCT = 25      # %, higher = stronger brake
REVERSE_KICK_SEC = 0.20    # s
SETTLE_SEC       = 1.2     # s, default for both variants
```

---

## Yaw loop rate

`yaw_snap` / `yaw_glide` stream `SET_ATTITUDE_TARGET` at 10 Hz.
Increase only if ArduSub can handle it (BlueOS default is fine at 10 Hz).

Constants at top of `motion_yaw.py`:

```python
YAW_RATE_HZ = 10.0   # SET_ATTITUDE_TARGET stream rate
YAW_TOL_DEG = 2.0    # settle tolerance
YAW_LOCK_N  = 5      # consecutive frames within tol = success
```
