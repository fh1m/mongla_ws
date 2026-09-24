# After a window reload Bondor shows "Link OK" with every command disabled, including Disarm, while the main process keeps heartbeating the vehicle. The GCS heartbeat does not depend on the UI being alive

Part of the 2026-09-24 ecosystem review (tracking issue in fh1m/mongla_ws to follow).

## Summary
The MAVLink connection and its 1 Hz GCS heartbeat live in the **main** process. The UI state lives in the **renderer**. The two are joined only by *change* events (`EVT_STATUS`), and the renderer never asks for the current status at startup. So:
1. **Reload desync.** After a renderer reload (Ctrl+R / View → Reload / HMR, the path that `1adc14c` just made survivable), the store boots with `status: { connected: false }`. Telemetry keeps flowing (EVT_MESSAGE), so the chip reads **"Link OK"**, but every button gated on `status.connected` is **disabled**: Arm/**Disarm**, mode buttons, Surface, Reboot, the joystick enable. No status event arrives to correct it, because the serial link only emits on open/close and UDP only on stall/recover. The vehicle keeps getting Bondor's heartbeat, and the operator has no disarm control until they click Connect again, which tears down and reopens the link.
2. **Heartbeat outlives the UI.** A hung or crashed renderer (no `render-process-gone` / `unresponsive` handler), or on macOS a closed window (`window-all-closed` does not quit on darwin), leaves the main process heartbeating at 1 Hz with **no operator interface at all**. The board's GCS failsafe (`GCS_FAILSAFE_MS`) is fed by that heartbeat, so it can never fire for "the pilot's console is gone".

## Evidence (at `1adc14c`)
`bondor/src/renderer/src/store/telemetry.ts:218`: initial state `status: { connected: false }`. `:628-632` `wireTelemetry()` subscribes to events but never calls `window.bondor.getStatus()`. `grep -rn getStatus bondor/src/renderer` finds no call site, although `IPC.GET_STATUS` is implemented (`main/index.ts:72`) and exposed (`preload/index.ts`).

`DiveView.tsx:269`: `disabled={!connected}` on the Arm/Disarm button (same on the mode buttons at `:277`, and `ModesView.tsx:419` Surface).

`ConnectionBar.tsx:138-142`: `label={alive ? 'Link OK' : ...}`, where `alive` comes from heartbeat age and not from `status.connected`, so the UI says "Link OK" and "Connect" at the same time.

`bondor/src/main/mavlink/connection.ts:150-164`: the heartbeat is an unconditional `setInterval(beat, 1000)` in main, independent of any renderer liveness.

`bondor/src/main/index.ts:93-100`: on darwin the app stays alive with no window. `activate` calls `createWindow()` again, which **re-registers every `ipcMain.handle`**. Electron throws "Attempted to register a second handler" and a second `MavlinkConnection` is constructed, while the first one (still connected and heartbeating) is orphaned with no handler pointing at it.

## Failure scenario
The operator is flying over USB and hits Ctrl+R because a chart froze. Bondor comes back showing attitude and "Link OK", but the Disarm button is grey. The vehicle is armed in MANUAL. The pilot sticks decay to neutral after 1.5 s (board `MANUAL_DECAY_MS`), but the vehicle stays **armed**, and the only way to disarm is to find the Connect button and reconnect.

## Suggested fix
1. In `wireTelemetry()`, after subscribing: `void window.bondor.getStatus().then((s) => useTelemetry.getState().setStatus(s))`. This is a one-line fix for (1).
2. Have the renderer send an **"operator alive" ping** (for example every 500 ms over IPC). In main, **stop the GCS heartbeat if no ping has arrived for ~2 s** (or on `render-process-gone` / `unresponsive` / window `closed`). That way "Bondor's heartbeat is on the wire" means "a human has a working console". Also, over LoRa, stop writing to USB so the bridge's `s_last_usb_ms` gate lapses.
3. Create the `MavlinkConnection` and register the `ipcMain.handle`s **once** at app scope (outside `createWindow`), and point `sendToRenderer` at the current window. On darwin, either quit on `window-all-closed` or disconnect.
4. Do not gate **Disarm** on `status.connected`. Disarm should always be clickable, because sending into a closed link is harmless.

## How to verify
- Connect over USB, arm (props off), press Ctrl+R. Before the fix the Disarm button is disabled and the chip says "Link OK". After the fix the button is enabled and the status is restored.
- Kill the renderer (`process.crash()` from DevTools, or `kill -STOP` on the renderer PID). Before the fix, the heartbeat continues (watch it in `connect --watch` from duburi_ws or Wireshark). After the fix the heartbeat stops within ~2 s and the board's GCS failsafe fires after 5 s.

## Severity: High
It removes the disarm control during a routine action (reload) while the vehicle stays armed, and it lets the vehicle's GCS failsafe be satisfied by a process that no human can operate.

## Related
- `1adc14c` "reloading the window crashed the whole main process" fixed the crash, but this desync is the next thing that goes wrong on that same path.
- Issue on source filtering of HEARTBEAT (filed alongside) covers the same Disarm toggle from a different direction.

---
_Generated by [Claude Code](https://claude.ai/code)_
