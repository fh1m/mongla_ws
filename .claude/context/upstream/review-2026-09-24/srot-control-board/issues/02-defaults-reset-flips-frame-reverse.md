# A defaults reset, an NVS reformat or a PARAM_DEFAULTS_VER bump silently sets FRAME_REVERSE and the motor directions back to 0 and ±1, and arming is still allowed (params also have no bounds)

Part of the 2026-09-24 ecosystem review (tracking issue in fh1m/mongla_ws, link to follow).

## Summary
`FRAME_REVERSE` defaults to 0. On the vehicle's hull it must be 1, because it negates all six axis demands, depth included. Three routine paths rewrite every non-calibration parameter to its build default:
- an NVS reformat
- a `PARAM_DEFAULTS_VER` bump in a new firmware
- `PREFLIGHT_STORAGE` param1=2

After any of them the board arms normally. The only notice is one boot `STATUSTEXT`, sent before the companion is usually connected. Separately, `params::set()` accepts any finite value for any row: it has no bounds table.

## Evidence
At commit `f1d3ba9`.

`include/config.h:529`: `#define DEF_FRAME_REVERSE 0.0f`. `src/comms/params.cpp:147` and `:232-239` give `FRAME_REVERSE` and `MOT_n_DIRECTION` their defaults.

`src/comms/params.cpp:440-445` reformats NVS when it is unusable:
```cpp
        esp_err_t e = nvs_flash_init();
        if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            nvs_flash_erase();
            s_nvs_was_reformatted = (nvs_flash_init() == ESP_OK);
        }
```
`src/comms/params.cpp:454-463` resets every row whenever the build's defaults version differs from the stored one:
```cpp
    bool force_defaults = (stored_ver != (uint32_t)PARAM_DEFAULTS_VER);
    for (uint16_t i = 0; i < s_n; ++i) {
        if (s_table[i].cal_id != CALP_NONE) continue;
        if (force_defaults) {
            *s_table[i].ptr = s_table[i].def;
            s_prefs.putFloat(s_table[i].nvskey, s_table[i].def);
```
`src/comms/params.cpp:589-597`: `resetAllToDefaults()` does the same.

`src/tasks/task_mavlink.cpp:37-47` is the only consequence. It sends two boot-time `STATUSTEXT`s ("Params reset to build defaults" and "NVS reformatted ..."). `src/control/arming.cpp` has no check for either condition.

`src/comms/params.cpp:630-663`: `set()` stores any finite float. The only guard is `isfinite` in `onParamSet` (`mav_commands.cpp:627`). Nothing stops `FRAME_REVERSE=7`, `MOT_3_DIRECTION=0`, `DEPTH_P=-5`, `ATC_RAT_RLL_IMAX=1e6` or `RPM_KP=1e9`; the last one is forwarded to the Pico, see the Pico RPM loop issue.

## Failure scenario
1. The firmware team ships a build that changes one PID default and bumps `PARAM_DEFAULTS_VER` from 4 to 5, as the comment at `params.cpp:449-452` says a changed default requires.
2. On first boot every row returns to default, including `FRAME_REVERSE` (1 → 0) and the operator's motor directions. One STATUSTEXT is sent roughly 1 s after boot, while the Pi is still starting up, and nobody sees it.
3. The operator arms at the pool. `canArm()` passes, and every axis is now backwards. The depth loop commands descent when it should ascend, so a `SURFACE` failsafe drives the vehicle to the bottom. The host's `check_behaviour_rev()` cannot catch this, because the firmware revision is correct and only a parameter changed.
4. An NVS reformat after a partition or flash fault does the same.

## Suggested fix
1. **Latch a "params defaulted" pre-arm refusal.** Set it on `force_defaults`, on `s_nvs_was_reformatted` and in `resetAllToDefaults()`. `canArm()` returns `"params reset - review + ACK"` until an explicit acknowledgement arrives, for example `PARAM_SET PARAMS_ACK=1`, which the companion's bring-up gate can send after it has checked `FRAME_REVERSE` and the directions. Report the latch continuously as a `SYS_STATUS` or `NAMED_VALUE_FLOAT` bit, not as a single boot message.
2. **Keep the hull-identity rows through a defaults bump,** the same way `CAL_*` rows are kept: `FRAME_REVERSE`, `MOT_n_DIRECTION`, `PM*_VMULT`. They describe the hull, not a tune. Add a `persist_across_defaults` flag to the table row.
3. **Add a bounds table.** Give each row `min` and `max`, plus `enum {-1, 1}` for directions and `{0, 1}` for booleans. `set()` returns false and sends `STATUSTEXT "<NAME> rejected: out of range [lo,hi]"`. Apply the same check when loading from NVS: a stored value outside its bounds falls back to the default and sets the latch from step 1.

## How to verify
- **Bench:** set `FRAME_REVERSE=1` and flash a build with `PARAM_DEFAULTS_VER+1`. Today the board arms with `FRAME_REVERSE=0`. With the fix, arming is refused until the acknowledgement, and `FRAME_REVERSE` survives the bump if step 2 is implemented.
- **Unit test:** `set("MOT_1_DIRECTION", 0.3)` returns false, and `set("FRAME_REVERSE", 2)` returns false.
- **Injection test:** remove the latch check in `canArm()` and confirm the test above fails. Restore it.

## Severity: High
One ordinary firmware update inverts every axis on this hull, including the depth failsafe, and the vehicle still arms.

## Related
- Issue #7 (FRAME_REVERSE undocumented)
- PR #25 (mixer vs hull)
- Issue #6 (arm paths that bypass `canArm()`: the latch must be enforced on those as well)

---
_Generated by [Claude Code](https://claude.ai/code)_
