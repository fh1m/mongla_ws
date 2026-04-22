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

All tunable live between goals — no restart needed:

```bash
ros2 param set /duburi_manager vision.kp_yaw 80.0
ros2 param set /duburi_manager vision.deadband 0.08
ros2 param set /duburi_manager vision.target_bbox_h_frac 0.55
```

| Param | Sim default | Pool recommendation | Effect |
|-------|-------------|---------------------|--------|
| `vision.kp_yaw` / `vision.kp_lat` | 60.0 | 60–80 | Raise if centring is sluggish; lower if oscillating |
| `vision.kp_depth` | 0.05 | 0.05–0.10 | Small — depth nudges accumulate |
| `vision.kp_forward` | 200.0 | 150–250 | Large OK because distance error is small |
| `vision.deadband` | 0.18 | 0.08–0.10 | Tighten for pool; 0.18 tolerates webcam noise |
| `vision.target_bbox_h_frac` | 0.30 | 0.45–0.60 | Raise to get closer to target |
| `vision.stale_after` | 1.5 s | 0.6–0.8 s | Drop for clean pool cameras |
| `vision.depth_anchor_frac` | 0.5 | 0.2 for tall targets | 0.2 = align near top of bbox; prevents depth stall on tall objects |

Full param descriptions: [`configuration.md`](configuration.md#vision-parameters-vision).

---

## Smoothing flags

Two independent flags on `auv_manager_node`:

| Flag                | Math | When to enable |
|---------------------|------|----------------|
| `smooth_yaw`        | Setpoint swept via `smootherstep(t/dur)` | Yaw overshoots or fights inertia |
| `smooth_translate`  | Thrust = `gain × trapezoid_ramp(t, dur, ramp=0.4 s)` | Lurch at start or backward drift at end |

```bash
ros2 run duburi_manager auv_manager --ros-args \
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
