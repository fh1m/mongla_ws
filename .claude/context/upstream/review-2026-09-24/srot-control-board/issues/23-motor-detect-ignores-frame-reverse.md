# MOTOR_DETECT ignores FRAME_REVERSE: on a hull with FRAME_REVERSE=1, a detect run that reports SUCCESS reverses every axis

Part of the 2026-09-24 ecosystem review, round 2 (see ../README.md).

## Summary
MOTOR_DETECT pulses each thruster through the direct test path, which bypasses the `FRAME_REVERSE` negation. It compares the gyro response against the **un-negated** mixer row, and then composes `motor_dir *= agree`. That drives every conclusive motor to "a positive command produces +eᵢ". In flight, `FRAME_REVERSE=1` then negates every axis demand before the mixer. So after a successful detect run on a hull that has `FRAME_REVERSE=1`, **all six axes are backwards**. That is the same failure as the 2026-08-07 flip, now reached from a routine that reports SUCCESS.

Only one line of firmware reads `frame_reverse`. Neither the detect routine nor the Motor Test path accounts for it, and nothing warns the operator.

Today this is handled by **operator procedure** on the companion side: mongla_ws `BUGS.md` "GATE 1" tells the operator to run MOTOR_DETECT and then set `FRAME_REVERSE = 0`, in that order. The firmware should enforce or handle this itself. A calibration routine whose correct use depends on a checklist the board cannot see will eventually be run without the checklist.

## Evidence
At commit `f1d3ba9`.

`FRAME_REVERSE` is read in exactly one place, just before the mixer (`src/tasks/task_control_loop.cpp:893-898`):
```cpp
            if (g_params.frame_reverse > 0.5f) {
                roll = -roll; pitch = -pitch; yaw = -yaw;
                thr  = -thr;  fwd   = -fwd;   lat = -lat;
            }
            mixer::mix(roll, pitch, yaw, thr, fwd, lat, norm);
            mixer::toDshot(norm, in.dir, armed, dshot);
```
(`grep -rn frame_reverse src/` finds only this, plus the parameter table and a `CFG` telemetry line.)

The detect pulse goes through the test override, which never reaches that block (`task_control_loop.cpp:809-819`):
```cpp
        if (test_active) {
            for (int i = 0; i < NUM_THRUSTERS; ++i) dshot[i] = DSHOT_DISARMED;
            if (in.test_motor >= 0 && in.test_motor < NUM_THRUSTERS) {
                ...
                dshot[in.test_motor] = mixer::oneToDshot(in.test_throttle, in.dir[in.test_motor]);
```

Detect projects onto the un-negated row and composes (`src/control/calibration.cpp:346-348`, `:401-406`):
```cpp
                    float e[3]; mixer::motorAngular(s_motor_idx, e[0], e[1], e[2]);
                    const float en = sqrtf(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]);
                    if (en > 0.1f) {
        ...
                    int8_t agree = (peak > 0) ? 1 : -1;
                    ...
                        g_state.cal.motor_dir[s_motor_idx] = (int8_t)(cur * agree);
```

`include/config.h:514-528` explains that `FRAME_REVERSE` exists *instead of* per-motor sign flips: "the correct place to fix 'every axis is backwards' is the AXIS layer, not MOT_n_DIRECTION".

The arithmetic: suppose every thruster, with `dir = +1`, produces −eᵢ. That is the hull `FRAME_REVERSE=1` was introduced for. Detect measures `agree = −1` for every motor and stores `dir = −1`, so a positive command now produces +eᵢ. In flight, a demand `d` becomes `mix(−d)`, which produces **−d** on every axis.

## Failure scenario
1. The hull is set up as documented: `FRAME_REVERSE=1`, `MOT_n_DIRECTION` at their defaults. It flies correctly.
2. After a thruster swap, the operator runs MOTOR_DETECT in the water. It reports SUCCESS and persists `CAL_MDIR*`.
3. The first STABILIZE or DEPTH_HOLD dive has every loop inverted: yaw hold spins the hull, and depth hold drives away from the target. This is the 2026-08-07 signature again. Nothing on the GCS explains it, because both the detect run and the parameters look correct.

## Suggested fix
Pick one, in firmware:
- **Account for it (preferred).** In the detect routine, multiply the expected vector by `(frame_reverse ? -1 : 1)` before projecting. Do the same wherever Motor Test is used to verify direction, so that "Motor Test shows what the vehicle will actually do" (the stated intent in `task_control_loop.cpp:812-818`) is true. Detect then converges to "the frame, as reversed, is correct", which is idempotent under both settings.
- **Or refuse it.** Refuse MOTOR_DETECT while `FRAME_REVERSE=1`, with a PreArm-style STATUSTEXT: "MOTOR_DETECT: set FRAME_REVERSE=0 first".
- **Or finish the job.** On SUCCESS, set `FRAME_REVERSE=0` and persist it in the same save, with a STATUSTEXT that says so. This turns the mongla GATE 1 procedure into firmware behaviour.

Whichever is chosen, bump `SROT_FW_BEHAVIOUR_REV`, since the companion has a documented workaround for the current behaviour.

## How to verify (bench)
- **Host unit test** on the math: with every motor's intrinsic sign at −1, `FRAME_REVERSE=1`, run the detect arithmetic, then compute the achieved wrench of `mix(−d)` for d = each unit axis. Today every axis comes out negated. After the fix every axis must come out positive. Repeat with `FRAME_REVERSE=0` to confirm that result does not change.
- **Injection:** remove the `frame_reverse` factor from detect and confirm the test fails.
- **In water** (needs the pool): dump `CAL_MDIR1..8` and `FRAME_REVERSE` before and after a detect run, then do a MANUAL yaw check and a STABILIZE heading hold.

## Severity: High
A calibration routine that reports success leaves the vehicle with every closed loop inverted. The only protection today is an operator checklist in another repository.

## Related
- Round-1 draft 02 (a defaults reset silently flips `FRAME_REVERSE` and the motor directions).
- mongla_ws PR #6 (`fd701fc`), which adopted "detect, then `FRAME_REVERSE=0`" as a procedure.
- Round-1 draft 00 (tunnel 5-DOF frame): any new frame has to go through the same detect/reverse interaction.

---
_Generated by [Claude Code](https://claude.ai/code)_
