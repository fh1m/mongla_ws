# A lost IMU starves the barometer, leak and battery reads: the self-heal blocks the sensor task and those reads are scheduled by iteration count, not time

Part of the 2026-09-24 ecosystem review (tracking issue in fh1m/mongla_ws, link to follow).

## Summary
Once attitude has been missing for 2 s, `bno085::poll()` calls `begin_I2C()`, and on failure calls it a second time at 0x4B. Both calls block. The retry gate is measured from the **start** of the last attempt with a 200 ms period. So any attempt that blocks for 200 ms or more re-triggers on the very next iteration, and the sensor task runs about one iteration per blocking call.

The baro, leak and battery reads are decimated by **loop-iteration count**: `BARO_DIV=25` and `ANLG_DIV=50`. When iterations take hundreds of ms, depth is refreshed only every several seconds, and leak and battery only every tens of seconds. `DEPTH_STALE_MS` is 1500, so depth goes invalid: `DEPTH_HOLD` and `AUTO` are refused and `SURFACE` runs without depth feedback.

## Evidence
At commit `f1d3ba9`.

`src/drivers/bno085.cpp:219-234`:
```cpp
        bool starved = s_attitude_ever && (now_ms - s_last_attitude_ms > 250) &&
                       (now_ms >= s_recovery_until);
        if (starved && now_ms - s_last_heal_ms > 200) {
            s_last_heal_ms = now_ms;
            if (now_ms - s_last_attitude_ms > 2000) {
                s_reinit_count++;
                if (s_bno.begin_I2C(I2C_ADDR_BNO085, &Wire) ||
                    s_bno.begin_I2C(0x4B, &Wire)) {
```
`src/tasks/task_sensor_read.cpp:41-44`, `:69-72` and `:113-117`: the schedule counts iterations.
```cpp
    const uint16_t BARO_DIV  = TASK_SENSOR_HZ / 20;   // ~20 Hz when healthy
    const uint8_t  ANLG_DIV  = TASK_SENSOR_HZ / 10;   // ~10 Hz
    ...
        if (++baro_cnt >= baro_div) {
    ...
        if (++anlg_cnt >= ANLG_DIV) {
```
The loop is paced by `vTaskDelayUntil(&last, period)` (`:299`). When the body overruns, it returns immediately. It does not make up the missed reads.

`include/config.h:618`: `#define DEPTH_STALE_MS 1500`.

**Not verified here:** how long `Adafruit_BNO08x::begin_I2C()` blocks. The library is not vendored. From memory, its I2C HAL open does a soft reset followed by `delay(300)`, but this needs to be measured. Our own `bno085::begin()` waits 300 ms on a warm reset (`bno085.cpp:125`), which suggests the part needs that long. If one attempt takes 300 ms or more, and two attempts are made when 0x4A fails, one sensor-task iteration takes 300–600 ms or more. The effect is then:
- baro every 25 iterations: **7.5–15 s** (limit 1.5 s)
- leak and battery every 50 iterations: **15–30 s**

## Failure scenario
1. At depth, the BNO085 stops reporting. This part has a history of reset storms on the shared I2C0 bus (ROADMAP 2026-07-24).
2. After 2 s the heal path calls `begin_I2C` on every iteration, and each call blocks for about 300 ms or more.
3. Depth is refreshed every 7 s or more. `depth_ok` is false most of the time, so `SYS_STATUS` withdraws baro health and the host's depth-gated verbs refuse.
4. Any `SURFACE` from now on, whether a failsafe or the operator's `surface` verb, has no fresh depth: the ascent runs open-loop and the vehicle cannot tell when it has reached the surface.
5. The leak input is sampled every 15 s or more for the whole period, including the moment a leak begins.

Even if `begin_I2C` turns out to be fast, the iteration-count schedule remains a latent bug: any future blocking call in this task shifts every decimated read.

## Suggested fix
1. **Schedule by time:**
```cpp
static uint32_t t_baro = 0, t_anlg = 0;
const uint32_t now = millis();
if (now - t_baro >= baro_period_ms) { t_baro = now; baro_new = bar30::read(...); ... }
if (now - t_anlg >= 100)            { t_anlg = now; analog_mon::read(...); }
```
2. **Make the heal non-blocking and back off:** retry at 0.5 s, 1 s, 2 s, 4 s and so on, capped at 5 s. Measure the gate from the **end** of the attempt. Try only the address that worked at boot. Move the reset wait out of the call into a state machine: issue the soft reset, return, and check it at least 300 ms later.
3. Count heal attempts and time spent blocked, and report both.

## How to verify
- **Measure:** time `begin_I2C()` with `micros()` around the call, with the BNO physically disconnected and connected.
- **Bench:** unplug the BNO085's SDA while the board is running. Today `SCALED_PRESSURE2` and `LEAK` stop updating, or update every several seconds; watch this with `connect --watch` on the host. After the fix they stay at 20 Hz and 10 Hz while the heal retries in the background.
- **Injection test:** add `delay(300)` in the heal branch of the fixed code and confirm the baro rate does not change.

## Severity: High
One IMU fault removes depth and leak sensing exactly when the failsafe needs them.

## Related
- The in-flight IMU-lost attitude fix (it stops the torque, but does not help the baro)
- "Proposal: move the IMU off shared I2C0" in this batch

---
_Generated by [Claude Code](https://claude.ai/code)_
