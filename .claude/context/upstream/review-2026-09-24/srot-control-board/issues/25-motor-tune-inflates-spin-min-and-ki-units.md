# MOTOR_TUNE writes an inflated MOT_SPIN_MIN (used on every flight) and an RPM Ki that is 2000× too aggressive for the Pico

Part of the 2026-09-24 ecosystem review, round 2 (see ../README.md). This extends issue #13 (MOT_SPIN_MIN relay) with new evidence. It is filed separately so that #13's thread stays about the relay.

## Summary
MOTOR_TUNE persists three results that are wrong, each for its own reason.

1. **`MOT_SPIN_MIN` is measured through itself and then inflated by the hold time.** The spin-up ramp sends its level through `mixer::oneToDshot()`, which already applies the *current* `MOT_SPIN_MIN` lift and the expo curve. So the motor breaks away almost at once. The ramp adds 0.002 per 2 ms cycle (1.0 per second), and the level is recorded only after RPM has stayed above 200 for 200 ms. The stored "break-away" is therefore at least 0.2 plus ESC start-up and filter lag, whatever the real break-away is. `finishAndSave()` writes that into `g_params.mot_spin_min` and persists it. On the default raw-DShot path (`RPM_LOOP=0`), `oneToDshot()` uses it as the floor for **every non-zero demand on every flight**. The smallest possible command becomes roughly 20–30 % of the band. *The magnitude is estimated from the code, not measured.*
2. **Ki is in 1/s, but the Pico uses it per cycle.** `Ki = Kp/(0.5·Tu)` is a continuous-time gain. The Pico adds `g_rpm_ki * err` to its integrator on every 2 kHz cycle. As written, it is 2000× too aggressive. The clamp (≤1.0 "per cycle") permits 2000/s.
3. **Motors whose PI phase failed still count.** `s_acc_n++` runs even when no Kp/Ki was accumulated, so the kp/ki averages are diluted towards zero.

## Evidence
At commit `f1d3ba9`.

The ramp and the hold (`src/control/motor_tune.cpp:15-17`, `:101-106`):
```cpp
static const float    RAMP_STEP    = 0.0020f;  // throttle/cycle in the idle ramp (~1 s to full)
static const int16_t  SPIN_RPM     = 200;      // rpm that counts as "spinning"
static const uint32_t SPIN_HOLD_MS = 200;      // must exceed SPIN_RPM this long → min-spin found
        ...
            s_out += RAMP_STEP;
            if (meas > SPIN_RPM) { if (s_spin_ok_since == 0) s_spin_ok_since = now; }
            else s_spin_ok_since = 0;
            if (s_spin_ok_since && now - s_spin_ok_since > SPIN_HOLD_MS) {
                s_acc_spin += s_out;                 // min active throttle
```
The pulse goes through the shaped output path (`src/tasks/task_control_loop.cpp:826`):
```cpp
            if (armed && am >= 0 && am < NUM_THRUSTERS) dshot[am] = mixer::oneToDshot(mtthr, 1);
```
which lifts every non-zero level onto the existing floor (`src/control/mixer.cpp:110`, `:127`):
```cpp
    float spin_min = constrain(g_params.mot_spin_min, 0.0f, 0.9f);
    ...
    float shaped = constrain(spin_min + (1.0f - spin_min) * thr, 0.0f, 1.0f);
```
The result is persisted (`motor_tune.cpp:61`, `:67-71`):
```cpp
        g_params.mot_spin_min = s_acc_spin / s_acc_n;
        ...
        g_params.rpm_ki       = s_acc_ki   / s_acc_n;
        params::requestSaveAll();            // Core 0 writes flash
```
Ki units, and the unconditional count (`motor_tune.cpp:157-160`):
```cpp
                    float Ki = (Tu > 0) ? constrain(Kp / (0.5f * Tu), 0.0f, 1.0f) : 0.02f;
                    s_acc_kp += Kp; s_acc_ki += Ki;
                }
                s_acc_n++;               // count a completed motor
```
The Pico applies it per cycle at 2 kHz (`src/pico/main.cpp:51`, `:69`, `:246`):
```cpp
static float          g_rpm_ki    = 0.02f;    // PI integral step per cycle
static const uint32_t OUT_PERIOD_US = 500;         // 2 kHz
                        s_integ[i] += g_rpm_ki * err;
```
`include/config.h:466-474` records the history: a 0.10 floor gave "~26 % of full speed from the smallest possible input", 0.02 stalled, and 0.15 was chosen to match the operator's ArduSub configuration. A MOTOR_TUNE run silently replaces that deliberately chosen value.

## Failure scenario
1. Before a pool session, the operator runs MOTOR_TUNE to check the thrusters. It reports "MTune done: spin_min=0.26 …" and saves.
2. From now on, a 1 % yaw correction from the heading-hold loop, or a small `vision_align` lateral nudge, is sent as about 26 % thrust. Precision alignment, which depends on low-end resolution, degrades into bang-bang. The depth loop limit-cycles around neutral.
3. If anyone later sets `RPM_LOOP=1`, the Pico integrator uses `rpm_ki` about 2000× too hot and saturates within a few cycles. That compounds the windup in round-1 draft 17.

## Suggested fix
- **Drive the spin-up raw.** Send the ramp level straight to DShot (`1049 + level·998`), bypassing lift, expo and battery scaling. Then the measurement does not depend on the value being measured.
- **Staircase, not a ramp.** Step the level (for example +0.005 every 150 ms), and record the level at the **first** step where RPM crosses the threshold. If a ramp is kept, subtract `ramp_rate × (SPIN_HOLD_MS + latency)`.
- **Do not overwrite `MOT_SPIN_MIN` silently.** Report the measured break-away in its own parameter (for example `MTUNE_SPIN`), and copy it into `MOT_SPIN_MIN` only on explicit operator confirmation.
- **Units:** `Ki_cycle = Ki · dt_pico` (dt = 0.0005 s). Clamp in per-second units before converting.
- **Averages:** increment `s_acc_n` only for motors whose PI phase produced a result, or keep separate counters for the spin, FF and PI phases.

## How to verify (bench)
In air, props on, vehicle restrained, one T200:
1. Record `MOT_SPIN_MIN`. Run a manual slow staircase via Motor Test while watching RPM on `ros2 run mongla_manager connect --watch`, and note the DShot value at break-away.
2. Run MOTOR_TUNE and record the new `MOT_SPIN_MIN`. **Today:** it is far above the manual break-away, converted to the same units. **After:** within ±0.01 of it.
3. Run MOTOR_TUNE twice with `MOT_SPIN_MIN` set to 0.05 and then 0.30 beforehand. **Today:** the result depends on the starting value. **After:** it does not.
4. Unit test: `rpm_ki` written by the tune must equal `Ki · 0.0005`.

## Severity: High
One run of a routine that looks like a harmless check permanently removes the low end of every thruster's range on the default output path, and nothing reports it.

## Related
- Issue #13 (MOT_SPIN_MIN relay), which this extends.
- Round-1 draft 17 (Pico RPM PI windup).
- Round-1 draft 01 (NVS write on core 0 stalls core 1): `requestSaveAll()` at the end of MOTOR_TUNE runs while armed.
- `include/config.h:676-677` (`PARAM_DEFAULTS_VER` 2 was already bumped once to clear a value "that MOTOR_TUNE's saveAll() had frozen in").

---
_Generated by [Claude Code](https://claude.ai/code)_
