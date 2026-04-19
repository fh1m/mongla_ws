# Yaw Stability & Sensor Fusion — Research Notes

> Captured: 2026-04-18 · Scope: reference / roadmap, no immediate code impact.
> Related files: [motion_yaw.py](../../src/duburi_control/duburi_control/motion_yaw.py),
> [pixhawk.py](../../src/duburi_control/duburi_control/pixhawk.py),
> [ardusub-reference.md](ardusub-reference.md).

Living document: add findings from pool tests / hardware bring-up under the
"Field notes" section at the bottom.

> **Status update (2026-04):** Phase 3 sensor package is **done**.
> The `duburi_sensors` package now ships a `YawSource` ABC,
> `MavlinkAhrsSource` (default — wraps ArduSub AHRS), and
> `BNO085Source` (ESP32-C3 over USB CDC, **gyro+accel only**, with a
> one-shot Pixhawk-mag offset captured at boot for Earth reference).
> See [`sensors-pipeline.md`](./sensors-pipeline.md) §"Calibration model"
> for the BNO calibration math; see
> [`vehicle-spec.md`](./vehicle-spec.md) §"Why BNO085 instead of the
> TDR's VectorNav VN200" for the hardware rationale. The
> `robot_localization` EKF is **still deferred** (Phase 4) — we don't
> fuse anything yet.

## 1. Why this note exists

After landing axis-split + smootherstep yaw + trapezoid linear ramps, the
next question is: how far can we push heading stability with software, and
what is the payoff vs waiting for vision-based localisation? This note
answers that so we don't over-invest before pool tests.

## 2. Veteran-team techniques for taming a noisy compass

Ordered roughly by cost / benefit. A checkmark means ArduSub already does
it for us via EKF3; a dash means we'd have to implement it ourselves.

| # | Technique                                       | Ours? | Notes |
|---|-------------------------------------------------|:-----:|-------|
| 1 | Hard / soft-iron magnetometer calibration       |  ✓*   | ArduSub has the algorithm; `*` = still requires on-vehicle calibration flight/dance. Biggest single lever. |
| 2 | Multi-mag redundancy + inlier voting            |  ✓    | Explains the `[ARDUB] EKF3 IMU0 switching to compass 0/1` log lines. |
| 3 | Tilt-compensated heading                        |  ✓    | EKF3 projects mag vector using full attitude. |
| 4 | Magnetic declination correction                 |  ✓    | Auto via GPS or manual `COMPASS_DEC`. |
| 5 | Gyro/mag complementary filter (EKF3)            |  ✓    | High-pass gyro + low-pass mag, fused with accel. |
| 6 | Yaw-rate command instead of yaw-angle command   |  —    | `SET_ATTITUDE_TARGET` body-rate field — good for fine tracking under vision. |
| 7 | Outer-loop heading + inner-loop rate PID        |  ✓    | ArduSub: `ATC_ANG_YAW_P` → `ATC_RATE_Y_*` cascade. |
| 8 | Median / Hampel outlier filter on yaw           |  —    | Cheap to bolt on if we see glitches; not needed yet. |
| 9 | DVL dead-reckoning + IMU integration            |  —    | Expensive hardware (~$5k); eliminates short-term drift independently of compass. |
|10 | Vision-aided heading (AprilTag / VIO)           |  —    | The real drift killer for pool missions — see §4. |
|11 | EKF/UKF fusion across all sources (robot_localization) | — | Deferred to Phase 3+. |
|12 | Online sensor-bias learning                     |  ✓    | EKF3 estimates gyro bias continuously. |

### Takeaway

Most of the gems are **already on by default** because ArduPilot's EKF3 is
a mature industrial-grade filter. The one lever we *don't* control from
software is **field calibration** — it has to be done on the real vehicle
(compass dance or rover-style circle). Simulation hides this problem.

## 3. What we layered on top (recent work)

These are the Python-side moves that complement ArduSub's onboard filter:

* **`yaw_glide` (opt-in)** — streams a `smootherstep`-interpolated setpoint
  to ArduSub at 10 Hz, so its 400 Hz stabiliser tracks a *smoothly
  decelerating* target. Prevents setpoint-driven overshoot without
  touching PID gains. Default off; enable via `-p smooth_yaw:=true`.
* **`drive_eased` (opt-in)** — `trapezoid_ramp` throttle envelope:
  smootherstep ease-in → cruise → smootherstep ease-out. Ease-out IS the
  brake, so `drive_eased` exits with low residual velocity and uses
  settle-only instead of the `drive_constant` variant's reverse kick.
* **Axis-isolated exit semantics** — each variant owns its brake so
  swapping profiles doesn't mis-apply a kick tuned for a different
  envelope. See the isolation contract in
  [duburi.py](../../src/duburi_control/duburi_control/duburi.py).

## 4. Will vision + Kalman make compass drift irrelevant?

Short answer: **mostly yes**, once vision is online and reliable, but keep
the compass path clean because it's the failover.

### When vision dominates

* Pool water is usually clear → AprilTag / ORB-SLAM / RTAB-Map can give
  5-15 Hz absolute pose with sub-degree heading.
* A Kalman filter weighted toward vision (low R covariance) ignores
  compass noise automatically.
* Competition AUVs (RoboSub, SAUVC, etc.) typically consider compass a
  bootstrap signal once vision has converged.

### Caveats the user should know

1. **Vision has gaps.** Motion blur, low light, turbidity, no features
   → pose estimate stale or invalid. Compass is the fallback.
2. **Update-rate mismatch.** Camera 30 fps but pose estimation often
   5-15 Hz after feature extraction. Compass @ 10 Hz fills the gap.
3. **Initialisation.** Vision needs a known reference (tag, map, loop
   closure). Compass gives the bootstrap heading so vision has a
   chance to lock onto the right hypothesis.
4. **Covariance is additive in the EKF.** A cleaner compass tightens the
   *fused* estimate even when vision is weighted high — the posterior
   covariance is a weighted harmonic mean, not a winner-takes-all.
5. **Failover smoothness.** If vision drops mid-mission, the estimate
   snaps back toward compass. Smaller snap = tuned compass.

### Recommended path

1. **Now:** keep AHRS2 yaw + smootherstep setpoint shaping (`smooth_yaw:=true`).
   The Pixhawk compass calibration flight is **fallback-only** now that BNO085
   is wired in — only matters if the BNO can't be brought up.
2. **Phase 3 — sensor package: DONE.** `duburi_sensors` exposes a
   `YawSource` interface; switch to BNO085 with
   `-p yaw_source:=bno085 -p bno085_port:=/dev/ttyACM0`. Calibration
   model in [`sensors-pipeline.md`](./sensors-pipeline.md).
3. **Phase 4 — when vision is online:** spin up `robot_localization`'s
   `ekf_node` with vision pose + AHRS2 + BNO085-derived yaw + depth.
   Tune Q / R so vision dominates when available and BNO085/compass
   becomes the failover.
4. **Only if pool tests show >5° sustained drift on BNO+offset:**
   revisit yaw-rate command mode (technique #6), median filter (#8),
   or DVL heading (#9).

## 5. Open-source references worth keeping around

* [auv_controllers](https://github.com/Robotic-Decision-Making-Lab/auv_controllers) — `ros2_control` AUV controller reference.
* [orca4](https://github.com/clydemcqueen/orca4) — ROS2 + ArduSub with mavros + visual localisation.
* [imu_tools](https://github.com/CCNYRoboticsLab/imu_tools) — complementary / Madgwick filters for IMU-only heading.
* [robot_localization](https://docs.ros.org/en/humble/p/robot_localization/) — reference EKF/UKF node, the target for Phase 3 fusion.
* [compass (CTU-VRAS)](https://github.com/ctu-vras/compass) — magnetic-to-true heading pipeline with declination & calibration.
* [PID without a PhD](../../Info%20&%20Docs/PID-without-PhD.pdf) — derivative-on-measurement, anti-windup, feed-forward recipes.

## 6. Field notes (append pool-test findings here)

_Empty — fill in after first hardware run._
