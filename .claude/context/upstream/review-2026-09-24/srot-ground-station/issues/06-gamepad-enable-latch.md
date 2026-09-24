# Gamepad "Control" stays enabled through a pad unplug and a link drop, and streams the sticks again as soon as either returns. It also ignores gamepad.mapping and is exposed to background timer throttling

Part of the 2026-09-24 ecosystem review (tracking issue in fh1m/mongla_ws to follow).

## Summary
The shared poller in `useGamepad.ts` has a module-level `enabled` flag that is only changed by the Switch. Four separate defects follow from that:
1. **Pad unplug while enabled:** `tick()` returns early. No neutral frame is sent (the last MANUAL_CONTROL on the wire can be a full deflection), and **`enabled` stays `true`**. When the pad reconnects (Bluetooth pads drop and rejoin on their own), streaming **resumes immediately** at whatever the sticks read, with no operator action.
2. **Link drop and reconnect:** the Switch is `disabled={!connected}` but its *value* is not cleared. `tick()` sends while `enabled` whatever the connection state is, so after a reconnect the sticks are live again without being re-enabled.
3. **No `pad.mapping === 'standard'` check:** the code hard-codes `axes[0..3]` as LX/LY/RX/RY. On a pad that Chromium does not map (`mapping === ''`, which is common on Linux for off-brand and some Xbox pads), `axes[2]` is often a trigger that **rests at -1.0**, so `r = -1000` (**full yaw**) the moment Control is enabled with hands off the sticks. `getGamepads().find(Boolean)` also picks whichever device enumerates first (a 3D mouse, wheel or virtual pad).
4. **Background throttling:** `BrowserWindow` does not set `backgroundThrottling: false`. When the Bondor window is minimised or fully occluded, Chromium throttles the 25 Hz `setInterval` to ≤1 Hz, and after 5 minutes intensive throttling applies. That is exactly the board's `MANUAL_FRESH_MS = 1000` boundary, so authority pulses as frames arrive at ~1 Hz.

## Evidence (at `1adc14c`)
`bondor/src/renderer/src/joystick/useGamepad.ts:55-63`: pad gone means return, with no neutral frame and no disable:
```ts
const pad = navigator.getGamepads().find(Boolean)
if (!pad) { if (live.connected) { live = EMPTY; notify() } return }
```
`:64-68`: axes by fixed index, no mapping check. `:89` `if (enabled) void window.bondor.sendManualControl(cmd)`: not gated on the link. `:116-119`: `enabled` is written only by `setEnabled`.
`DiveView.tsx:350` / `JoystickView.tsx:217-219`: `checked={gp.enabled} disabled={!gp.connected || !connected}`, so the switch keeps showing checked, greyed out.
`bondor/src/main/index.ts:31-35`: `webPreferences` has no `backgroundThrottling: false`.
Board context: `srot-control-board/include/config.h:649-656`: `MANUAL_FRESH_MS 1000`, `MANUAL_DECAY_MS 1500` (stale sticks ramp to neutral), which bounds (1) to ~1.5 s of stale input.

## Failure scenario
The pilot is flying in MANUAL and the Bluetooth pad drops for 3 s. The vehicle coasts on the last command for ~1-1.5 s and then decays. The pad rejoins while the pilot is resting a thumb on the yaw stick, and the vehicle yaws at once. Nobody re-enabled anything, and the switch was greyed out the whole time. Separately, on the bench a new operator plugs in an unmapped pad and flips "Control": the vehicle yaws at full rate with the sticks centred.

## Suggested fix
- In `tick()`: if `!pad && enabled`, send **one neutral frame** (`{x:0,y:0,z:500,r:0,buttons:0}`) and set `enabled = false`. Do the same when `status.connected` goes false (subscribe in `setStatus`). This complements PR #4, which adds the neutral frame on a manual disable.
- Refuse to enable, or show a warning, unless `pad.mapping === 'standard'`. Select the pad by `pad.mapping === 'standard'` first, and remember `pad.index`/`pad.id` so a different device cannot take over.
- Set `backgroundThrottling: false` in `webPreferences`, **or** move the MANUAL_CONTROL cadence into main (the renderer posts the latest stick state; main sends at a fixed 25 Hz and sends neutral if no update for 200 ms).

## How to verify
- Enable Control, hold full forward, unplug the pad. Before: the last frame on the wire is x = 1000 and the switch is still checked. After: a neutral frame is sent and the switch is off. Reconnect the pad: nothing is sent until the switch is flipped again.
- In DevTools, stub `navigator.getGamepads` to return `{mapping:'', axes:[0,0,-1,0]}` and enable Control. Before: `r = -1000`. After: enable is refused with a warning.
- Minimise the window while Control is enabled and measure the MANUAL_CONTROL rate at the board. Before: ~1 Hz. After: 25 Hz.

## Severity: Medium
The board's stick-staleness ramp bounds the damage to ~1.5 s, but (1) and (3) produce **uncommanded motion with no operator action**, which the "neutral on startup / no axis commanded until asked" rule exists to prevent.

## Related
- PR #4 (joystick disable sends no neutral).
- Cross-repo: the board's single `s_prev_buttons` edge detector (`mav_commands.cpp:691-733`) is shared by every MANUAL_CONTROL sender, and `JS_ARM` in `runButtonFunction` calls `setArmed(true)` **without `arming::canArm()`**. Both are for srot-control-board and are recorded in the tracking issue.

---
_Generated by [Claude Code](https://claude.ai/code)_
