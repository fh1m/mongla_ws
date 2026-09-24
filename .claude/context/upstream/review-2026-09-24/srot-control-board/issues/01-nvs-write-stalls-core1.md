# Every NVS write on core 0 freezes core 1 (control loop and thruster output), including saves that run while armed

Part of the 2026-09-24 ecosystem review (tracking issue in fh1m/mongla_ws, link to follow).

## Summary
On the classic ESP32, a SPI-flash write or erase disables the cache on **both** CPUs. The comments in `params.cpp` and `mav_commands.cpp` say the deferred saves "never stall the 500 Hz loop". They do stall it: every NVS write stops core 1 for as long as the flash operation takes. Nothing refuses a write while armed. `PARAM_SET`, the end of `AUTOTUNE`, `MOTOR_TUNE` and `PREFLIGHT_STORAGE` all write flash with the thrusters live.

## Evidence
At commit `f1d3ba9`. The board is `esp32doit-devkit-v1` (`platformio.ini:25`), a classic dual-core ESP32 without flash auto-suspend.

`src/comms/params.cpp:640-663`: `params::set()` runs on every `PARAM_SET` and does not check the arm state:
```cpp
    *s_table[i].ptr = value;
    s_prefs.begin(NVS_NS_PARAMS, false);
    ...
        float have = s_prefs.getFloat(s_table[i].nvskey, NAN);
    ...
    bool wrote = (s_prefs.putFloat(s_table[i].nvskey, value) != 0);
    s_prefs.end();
```
The same misunderstanding appears in two comments. `src/comms/params.cpp:690-691`:
```cpp
// Deferred save: the flight loop (autotune) requests; Core 0 services it so the
// blocking flash write never stalls the 500 Hz loop.
```
`src/comms/mav_commands.cpp:792-793`:
```cpp
    // Deferred NVS flash writes run HERE (Core 0 / comms task) so a calibration- or
    // autotune-complete never stalls the 500 Hz flight loop on Core 1.
```
These are the writers that can run while armed:
- `mav_commands.cpp:633`: `PARAM_SET` calls `params::set()` synchronously.
- `mav_commands.cpp:804`: `calibration::saveToNVS()`, about 16 `putFloat`s.
- `mav_commands.cpp:828-829`: `params::serviceSaveAll()`, which walks about 190 rows.
- `control/autotune.cpp:370` and `:394`, and `control/motor_tune.cpp:71`: `params::requestSaveAll()` when a tune finishes. Both tunes run armed, so this save always happens with the thrusters live.
- `mav_commands.cpp:450-477`: `PREFLIGHT_STORAGE` param1=1 queues a save-all and a calibration save.

The downstream timeout is `src/pico/main.cpp:37` and `:202`:
```cpp
static const uint32_t LINK_TIMEOUT_MS   = 150;   // no valid frame this long -> stop
...
    bool link_ok  = (now - s_last_cmd_ms) < LINK_TIMEOUT_MS && s_last_cmd_ms != 0;
```

## Failure scenario
1. The operator tunes in the pool with the vehicle armed and holding depth. They send `PARAM_SET ATC_RAT_RLL_P`, or `AUTOTUNE` finishes and requests a save-all.
2. Core 0 calls `Preferences::putFloat`. ESP-IDF runs `spi_flash_disable_interrupts_caches_and_other_cpu()`, and core 1 (the sensor, control and DShot tasks) spins in IRAM until the operation completes.
3. A write that fits the current NVS page stalls core 1 for a short time: tens of µs up to about 1 ms. With roughly 3 NVS entries per float, a page fills after a few dozen changed-value writes. The next write then erases a 4 KB sector. The erase is typically about 45 ms, and flash datasheets give a worst case of several hundred ms. A save-all that rewrites many rows, or triggers NVS garbage collection, can erase more than one sector.
4. For the whole stall no attitude or depth update runs. The Pico keeps driving the last command it received. If the stall exceeds 150 ms, the Pico's `LINK_TIMEOUT_MS` expires and it stops all eight thrusters while the vehicle is at depth.
5. Core 0 is also stalled, so no heartbeat or telemetry goes out during the stall.

The stall lengths above are not measured on this board. The mechanism is standard ESP-IDF behaviour on the classic ESP32. The claim that the loop is not stalled is contradicted by the platform.

## Suggested fix
- **Apply live, persist later.** `params::set()` updates the RAM value, sets a per-row dirty bit and returns `persisted=false`. The dirty rows are committed on the **disarm edge**, or on an explicit save while disarmed. Report the pending state, for example with a `STATUSTEXT "N params unsaved"`, so a power-off before disarm is not silent.
- **Refuse bulk flash writes while armed.** `PREFLIGHT_STORAGE` param1=1 and the calibration save return `TEMPORARILY_REJECTED`. Autotune and motor-tune completion call `requestSaveAll()`, which already defers. Make `serviceSaveAll()` wait until the vehicle is disarmed.
- Correct the two comments quoted above.
- Optional: raise the Pico `LINK_TIMEOUT_MS` above the measured worst case only after it has been measured. Do not guess this value.

## How to verify
- **Measure the stall.** Toggle a spare GPIO at the start of every control-loop iteration and put a logic analyser on it. Script 500 `PARAM_SET`s that alternate the value of one row, so every call writes, and run a `PREFLIGHT_STORAGE 1` after changing about 50 rows. Record the longest gap between toggles. Expect short stalls on most writes and tens of ms whenever a sector is erased.
- **Injection test.** Revert the fix, arm on the bench with the props off, and send `PARAM_SET`s in a loop. The Pico link-loss flag or buzzer should fire once the longest gap exceeds 150 ms. With the fix applied, the gap while armed must stay under 2.5 ms, and the values must be persisted after disarm and a power cycle.

## Severity: High
While armed, flash writes freeze control and output for anywhere from µs to hundreds of ms. Tuning is exactly when this happens. The Critical case, a cut past 150 ms, is plausible but not yet measured.

## Related
- #17 (companion link): a stalled core 0 also stops the link, so the host sees a gap in telemetry and cannot tell it from a board reset.
- The in-flight output-watchdog change (`loop_stamp_ms`) cannot catch this. It runs on the same core and is frozen as well.

---
_Generated by [Claude Code](https://claude.ai/code)_
