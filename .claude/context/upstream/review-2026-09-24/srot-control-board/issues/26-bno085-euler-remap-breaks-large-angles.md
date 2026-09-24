# The BNO085 mount remap is done on Euler angles, not the quaternion: body roll is capped at ±90°, and style_roll commands full-authority pitch halfway through

Part of the 2026-09-24 ecosystem review, round 2 (see ../README.md).

## Summary
The driver converts the sensor-frame quaternion to ZYX Euler angles, then swaps roll and pitch and negates yaw to express them in the body frame. That swap equals the true mount rotation (180° about the x=y axis) only for single-axis rotations near level. In particular, **body roll is taken from the sensor's `asinf` pitch, which is bounded to ±90°**. At 120° of real body roll the output reads roll ≈ 60° and pitch ≈ 180°.

`style_roll` runs on the board (movement STYLE → `stunt::start(ROLL, n·360°)`). During the roll, the stunt controller holds the passive pitch axis level with `(0 − meas_pitch)·ANG_PIT_P`. When the reported pitch jumps to about π, that becomes about −π × 6 ≈ −19 rad/s of pitch-rate demand with the default `ATC_ANG_PIT_P = 6.0`. That saturates the pitch loop, so **halfway through the roll the hull is told to pitch at full authority**. In normal flight, combined roll and pitch also carry a second-order error from the wrong composition order. *Not verified in water.*

The round-2 review used `ANG_PIT_P = 4.5` (about −14 rad/s). The default at `f1d3ba9` is 6.0. Either way it saturates.

## Evidence
At commit `f1d3ba9`.

Euler angles are taken from the **sensor** quaternion (`src/drivers/bno085.cpp:109-117`):
```cpp
static void quatToEuler(float qr, float qi, float qj, float qk,
                        float& roll, float& pitch, float& yaw) {
    float sqr = sq(qr), sqi = sq(qi), sqj = sq(qj), sqk = sq(qk);
    yaw   = atan2f(2.0f * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ...
    pitch = asinf(constrain(sarg, -1.0f, 1.0f));
    roll  = atan2f(2.0f * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
}
```
and then remapped as scalars (`bno085.cpp:272-273`, `:292-293`):
```cpp
                quatToEuler(out.qw, out.qx, out.qy, out.qz,
                            out.roll, out.pitch, out.yaw);
                ...
                { float t = out.roll; out.roll = out.pitch; out.pitch = t; }
                out.yaw = -out.yaw;
```
The comment above it correctly identifies (x,y,z) → (y,x,−z) as "a genuine 180 deg rotation about the x=y axis". But applying that rotation to *Euler angles* is not the same as applying it to the attitude. Conjugating a ZYX sequence by that rotation gives a Z-X-Y sequence, not Z-Y-X.

The stunt holds pitch with the measured Euler pitch (`src/control/stunt.cpp:36-37`):
```cpp
    float roll_rate  = (s_axis == StuntAxis::ROLL)  ? spin : (0 - meas_roll)  * g_params.ang_rll_p;
    float pitch_rate = (s_axis == StuntAxis::PITCH) ? spin : (0 - meas_pitch) * g_params.ang_pit_p;
```
`style_roll` reaches it from the move primitive (`src/control/movement.cpp:109-112`):
```cpp
        case Type::STYLE: {
            int n = (int)primary; if (n < 1) n = 1;
            stunt::start(StuntAxis::ROLL, n * 360.0f, 90.0f);
```
`include/config.h:446`: `#define DEF_ANG_PIT_P 6.0f`.

The stunt's own completion counter integrates the gyro (`stunt.cpp:45-48`), so the roll does complete. The problem is the passive-axis hold during it.

## Failure scenario
1. On the 8-thruster frame (roll is actuated), the companion sends `style_roll` (count 1).
2. Roll-rate demand is 90°/s. At about 90° of body roll, the reported roll stops increasing, and the reported pitch flips towards ±180°.
3. The passive pitch hold commands about −19 rad/s pitch rate, which saturates. The vertical group now drives pitch at full authority while the roll continues. The result is an uncommanded tumble in a different axis. The safety monitor's angle guard is exempted for STYLE (`task_control_loop.cpp:577-578`). The rate guard should eventually trip it, which **disarms at depth**.
4. In ordinary flight, a 20° roll with a 20° pitch reads with a small cross-axis error, which the angle loops then "correct".

On the 5-thruster tunnel hull, roll is unactuated, and round-1 draft 00 already asks for `style_roll` to be refused there. This issue matters for the 8-thruster frame, and for any attitude outside ±90° on either frame (for example a capsize during handling or a failed ballast).

## Suggested fix
- **Rotate the quaternion, then take Euler angles once:** `q_body = q_sensor ⊗ q_mount`, with `q_mount` = 180° about (1,1,0)/√2 = `(w,x,y,z) = (0, ½√2, ½√2, 0)`. (Check left or right multiplication against the axis convention with the bench test below. The yaw sign is already known from the 2026-08-07 water measurement.) Store the body quaternion in `out.q*` too, so that `ATTITUDE_QUATERNION` and the SD log are in the body frame.
- **Stunts:** hold the passive axes with a quaternion or rotation-vector error (as ArduPilot's attitude controller does), or with rate-integrated angles about the body axes, not Euler angles, which are singular at ±90° pitch.
- Keep `BNO_SWAP_ROLL_PITCH` as the switch that selects `q_mount`, so a board mounted the other way is one constant.

## How to verify (bench)
- **Unit test (host):** for a set of body attitudes (single-axis ±170°, and combined 30°/30°/30°), build the sensor quaternion as `q_body ⊗ q_mount⁻¹`, run the driver math, and compare with the true body Euler angles. **Today:** single-axis roll beyond 90° fails, and the combined case has a small error. **After:** every case agrees within 1e-4 rad.
- **By hand:** with the board powered and `ATTITUDE` streaming, roll it slowly about the body x axis past 90°. **Today:** pitch jumps by 180° and roll turns back. **After:** roll passes smoothly through 90° to 180°.
- **Injection:** restore the Euler swap and confirm the unit test fails.

## Severity: High
A board-side primitive that the companion is allowed to call drives an uncommanded, saturated pitch halfway through its own manoeuvre. The same transform also sits under every attitude number the board reports.

## Related
- Round-1 draft 00 (refuse `style_roll` on the tunnel frame).
- Round-1 draft 18, item 2 (Euler-angle errors fed straight into body-rate PIDs). Fixing both with a quaternion attitude error removes the pair.

---
_Generated by [Claude Code](https://claude.ai/code)_
