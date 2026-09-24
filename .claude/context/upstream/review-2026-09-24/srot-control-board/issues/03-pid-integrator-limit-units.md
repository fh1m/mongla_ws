# The PID integrator limit is in error·seconds, not output units, so the ArduSub IMAX values cap the I-term at 4 % (roll/pitch), 0.4 % (yaw) and 10 % (depth)

Part of the 2026-09-24 ecosystem review (tracking issue in fh1m/mongla_ws, link to follow).

## Summary
`PID` integrates the raw error, `_integ += err*dt`, and clamps that sum to `±imax`. The output then uses `ki*_integ`. So `IMAX` limits error·seconds, and the largest possible I contribution is `ki*imax`. The IMAX defaults are ArduSub's (0.444 / 0.222). In ArduSub (`AC_PID`) they limit the I-term **in output units**. Here they cap the I-term at a few percent of full output. A buoyancy or trim offset larger than that is never trimmed out.

## Evidence
At commit `f1d3ba9`.

`src/control/pid.h:94-101`:
```cpp
        const float out_unsat = _kp * err + _ki * _integ + _kd * _deriv;
        const bool  push_high = (out_unsat >=  _outmax) && (err > 0.0f);
        const bool  push_low  = (out_unsat <= -_outmax) && (err < 0.0f);
        if (!push_high && !push_low) {
            _integ = constrain(_integ + err * dt, -_imax, _imax);
        }
        const float out = _kp * err + _ki * _integ + _kd * _deriv;
```
The limits are set in `src/control/attitude_control.cpp:42-44` and `src/control/depth_control.cpp:41`:
```cpp
    s_rate_roll.setLimits(g_params.rat_rll_imax  > 0 ? g_params.rat_rll_imax  : 0.5f, 1.0f);
    ...
    s_pid.setLimits(1.0f, 1.0f);   // depth
```
The defaults are in `src/comms/params.cpp:161,176,181` (IMAX 0.444 / 0.444 / 0.222) and `include/config.h:449,452,455,612` (KI 0.090 / 0.090 / 0.018, `DEPTH_I` 0.1).

| Loop | ki | imax (err·s) | Largest I-term (fraction of full output) |
|---|---|---|---|
| roll rate | 0.090 | 0.444 | **0.040** |
| pitch rate | 0.090 | 0.444 | **0.040** |
| yaw rate | 0.018 | 0.222 | **0.004** |
| depth | 0.1 | 1.0 | **0.10** |

There are two knock-on effects:
- `depth::update()` calls `setGains()` on **every** cycle (`depth_control.cpp:40`), and the attitude loops do the same through `loadGains()`. Changing `KI` in flight therefore rescales the stored integral immediately, and the output jumps by `Δki*_integ`.
- The CoB auto-trim (`src/control/feedforward.cpp:72-83`, `TRIM_EN`, off by default) moves `leak*integral()` into a trim that is in **output units**, and removes the same number from the integrator, which is in **err·s**. So the I-term loses `ki*x` of effort while the trim gains `x`. For roll that is 0.09x against 1.0x. The comment claims the effort is conserved. It is not: every transfer adds `(1-ki)*x` of net effort.

## Failure scenario
1. The vehicle is 3 % heavy, or its centre of buoyancy is offset so that holding level takes 6 % roll torque. That is ordinary before ballasting is finished.
2. In `DEPTH_HOLD` the depth integrator saturates at 10 % of throttle. If the vehicle needs more than that, it settles below target with a steady error of `(need − 0.10)/DEPTH_P`. For example, 15 % needed with P=0.5 gives a 0.1 m sag.
3. In roll the integrator saturates at 4 %. With `ANG_P=6` and `RAT_P=0.135`, the remaining 2 % has to come from P, so the vehicle holds a steady list of about 0.02/(6*0.135) ≈ 0.025 rad ≈ 1.4°. It grows linearly with any larger offset.
4. Raising IMAX to compensate would also allow that much stored error-time, so the windup behaviour changes too. The parameter does not mean what ArduSub users expect.

## Suggested fix
Accumulate in output units, as `AC_PID` does:
```cpp
if (!push_high && !push_low)
    _integ = constrain(_integ + _ki * err * dt, -_imax, _imax);   // _integ now in output units
const float out = _kp * err + _integ + _kd * _deriv;              // and in out_unsat
```
- Make `integral()` and `bleedIntegral()` work in output units. The CoB trim then really conserves effort.
- Changing `ki` no longer rescales the stored integral.
- Keep the IMAX defaults (0.444 / 0.222), because they now mean what ArduSub means. Choose the depth `imax` deliberately, for example 0.3–0.5. Record the change in `PARAMETERS.md` and bump the firmware behaviour revision, since the tuned responses change.

## How to verify
- **Unit test (host build of `pid.h`):** kp=0, ki=0.09, imax=0.444, constant error 1.0, dt=0.002, 2000 steps. The output must reach 0.444, where today it stops at 0.040. Change `ki` to 0.18 halfway through: the output must not jump.
- **Trim test:** with `TRIM_EN=1` and a constant-error plant, `ki*integral + trim` must be unchanged immediately before and after a transfer.
- **Bench:** tape a known offset mass to one side, hold level in STABILIZE, and read `ATTITUDE.roll`. Today there is a steady list; after the fix it trends to zero.
- **Injection test:** revert to `err*dt` and confirm the unit test fails.

## Severity: High
With these defaults the I-term cannot remove more than 4 % roll or 10 % depth offset, which guarantees a steady list or depth sag on an untrimmed hull.

## Related
- PR #20 (mixer scale-down and PID windup). Fix the two together, because the anti-windup test uses the same `out_unsat`.

---
_Generated by [Claude Code](https://claude.ai/code)_
