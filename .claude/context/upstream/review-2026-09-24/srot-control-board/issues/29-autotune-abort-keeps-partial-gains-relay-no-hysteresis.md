# Autotune: an aborted tune leaves partially tuned gains live (and the next save persists them), and the relay switches on the raw gyro sign with no hysteresis

Part of the 2026-09-24 ecosystem review, round 2 (see ../README.md).

## Summary
Two independent defects in `autotune.cpp`, plus the tuning-rule critique from the round-2 control review:

1. **No snapshot, no restore.** `applyGains()` writes straight into `g_params` as each phase finishes. `abort()` is documented as "stop now, keep gains tuned so far". A safety-monitor abort during an angle phase, a disarm, or (after draft 22 is fixed) a failsafe therefore leaves the rate gains the tune has just derived **live**. The next `PREFLIGHT_STORAGE`, or any later tune's `requestSaveAll()`, persists them. ArduPilot snapshots the original gains, applies the tuned set only on success, and restores the originals on stop.
2. **The relay has no hysteresis.** The relay output switches on the sign of the raw gyro (`rate > 0`). Only the *measurement* has a Schmitt band. Near zero, gyro noise chatters the relay (up to 500 Hz), which drives the plant with a noise-modulated square wave. `MIN_HALF_MS` hides this in the period estimate but does not stop the actuation. Åström–Hägglund relay tuning uses a hysteretic relay, and the Ku formula takes the hysteresis ε into account.
3. **The tuning rule and the amplitude estimate** (from the control review, lower priority): the amplitude is the maximum |signal| after settling, so a static offset (buoyancy on depth, a CoB moment on pitch) biases it high and Ku low. And Ziegler–Nichols "some overshoot" is aggressive for delay-dominated thruster plants.

## Evidence
At commit `f1d3ba9`.

Gains are written into the live parameter set phase by phase (`src/control/autotune.cpp:214-217`):
```cpp
    switch (phase) {
        case P_RATE_ROLL:  g_params.rat_rll_p = Kp; g_params.rat_rll_i = Ki; g_params.rat_rll_d = Kd; break;
        case P_RATE_PITCH: g_params.rat_pit_p = Kp; g_params.rat_pit_i = Ki; g_params.rat_pit_d = Kd; break;
        case P_RATE_YAW:   g_params.rat_yaw_p = Kp; g_params.rat_yaw_i = Ki; g_params.rat_yaw_d = Kd; break;
```
Abort keeps them (`autotune.cpp:495`):
```cpp
void abort() { s_phase = P_DONE; s_need_init = false; }   // stop now, keep gains tuned so far
```
The relay switches on the raw sign (`autotune.cpp:422-423`, and the same pattern at `:437` and `:457`):
```cpp
            float rate = (s_phase == P_RATE_ROLL) ? gx : (s_phase == P_RATE_PITCH) ? gy : gz;
            float relay = (rate > 0) ? -A_TORQUE : A_TORQUE;
```
while the measurement alone has a band (`autotune.cpp:236-239`):
```cpp
static bool measure(float signal, uint32_t now, float hyst) {
    int newsign = s_sign;
    if (signal >  hyst) newsign =  1;
    else if (signal < -hyst) newsign = -1;
```
The amplitude is a running maximum (`autotune.cpp:257`), and the rule is ZN (`autotune.cpp:185-187`):
```cpp
    if (s_cross_n >= SETTLE_CROSSINGS) s_amp = max(s_amp, fabsf(signal));
    ...
    float Kp = 0.33f * Ku;
    float Ki = (Tu > 0) ? Kp / (0.5f * Tu) : 0.0f;
    float Kd = Kp * (Tu / 3.0f);
```
Reference (`libraries/AC_AutoTune/AC_AutoTune.cpp:90-94` and `:134-146` @ ArduPilot `ee0c343`): `stop()` calls `load_gains(GainType::ORIGINAL)`, and `disarmed()` saves the tuned gains only if the tune completed (or the pilot was testing tuned gains). Otherwise it calls `reset()`.

## Failure scenario
1. Autotune completes the three rate phases. They write aggressive rate gains, some clamped ("CLAMP").
2. During the roll angle phase, the rate guard trips on a gust or a tether snag. The safety monitor aborts and disarms.
3. The vehicle is now on the half-tuned rate gains. The operator does not notice, because the only message was "Disarmed: rate limit". They re-arm and fly. Later a routine parameter save (or the next tune's save-all) writes those gains to flash, and the pre-tune set is gone.
4. Separately: during each rate phase, gyro noise around zero flips the relay at a high rate. The plant sees a chattering torque rather than a clean relay, so the measured Tu and amplitude, and therefore the gains, depend on the noise level on the day.

## Suggested fix
- **Snapshot and restore (the `AC_AutoTune` pattern).** At `start()`, copy the full gain set. Keep derived gains in a local "tuned" set. Apply them to `g_params` only when *all* phases complete, or apply per phase for the cascade but restore the snapshot on `abort()`. Never call `requestSaveAll()` on an aborted or partially failed tune unless the operator confirms.
- **Relay hysteresis.** Switch the relay with a Schmitt band ε (the existing `H_RATE` / `H_ANG` / `H_DEPTH` are natural choices), and use the corrected `Ku = 4d / (π·√(a² − ε²))`. *The formula is quoted from the control review, which gives it from memory as standard.*
- **Amplitude.** Use the mean of the half-cycle peak-to-peak values divided by 2, and consider a biased relay (adjust the relay bias until the half-periods are equal) so that a static offset does not bias Ku.
- **Rule (later, needs water).** Keep the relay to find Tu, and add a per-axis step to fit a first-order-plus-dead-time model. Tune with SIMC (τc = θ gives Ms ≈ 1.6–1.7 and a gain margin of about 3, per Skogestad). This belongs with the chirp system-ID mode in the control-upgrades proposal (draft 41).

## How to verify (bench)
- **Unit test (host):** start a tune, let two rate phases complete, call `abort()`. `g_params.rat_*` must equal the pre-start snapshot. **Today** they hold the tuned values.
- **Bench, props off, restrained:** log the relay output during a rate phase at 500 Hz (the SD log cannot do this today; see draft 40). **Today** there are sign flips spaced far less than `MIN_HALF_MS` apart. **After** the hysteresis fix there are none.
- **Injection:** remove the restore from `abort()` and confirm the unit test fails.

## Severity: Medium
Gains from an unfinished tune are left live with nothing to tell the operator, and can end up persisted. The relay chatter degrades every tune result. Neither is immediately dangerous, but both corrupt the one tool used to set the vehicle's gains.

## Related
- Draft 22 in this batch (a running autotune overrides the SURFACE failsafe). Its fix makes aborts more frequent, which makes the restore more important.
- Draft 27 in this batch (the autotune Ki also moves the I-term ceiling).
- Round-1 draft 01 (NVS write stalls core 1): autotune's `requestSaveAll()` runs while armed.

---
_Generated by [Claude Code](https://claude.ai/code)_
