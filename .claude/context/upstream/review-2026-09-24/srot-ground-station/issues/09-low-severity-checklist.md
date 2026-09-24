# Low-severity checklist from the 2026-09-24 review: Electron hardening, LoRa diagnostics, input parsing, doc drift

Part of the 2026-09-24 ecosystem review (tracking issue in fh1m/mongla_ws to follow).

## Summary
Small, independent items found in the same read. Each is concrete, with evidence at `1adc14c`. None needs another to land first. Tick them off separately.

## Evidence / checklist

### Electron
- [ ] **`shell.openExternal` on any URL.** `bondor/src/main/index.ts:40-43` passes `details.url` from `setWindowOpenHandler` straight to `shell.openExternal`, with no scheme allowlist. The renderer loads only local content under a CSP (`renderer/index.html:7`), so the practical risk is low. Still, `file:`, `smb:` or custom-protocol URLs (for example from a STATUSTEXT that is later rendered as a link) would be handed to the OS. Fix: allow only `https:`/`http:`.
- [ ] **`sandbox: false`** (`index.ts:33`). The preload only uses `contextBridge` and `ipcRenderer`, which both work sandboxed, so enable it. Also add a `will-navigate` handler that calls `preventDefault()` for anything other than the app's own URL.
- [ ] **IPC arguments are not validated in main** (`index.ts:70-81`). For example, `SEND_COMMAND_LONG` accepts any `command` number and any param array length, and `SET_SETTINGS` writes arbitrary JSON to disk. The renderer is trusted today, so this is defence in depth: type-check and clamp at the IPC boundary.

### LoRa bridge firmware (`src/groundstation/main.cpp`)
- [ ] **`LoRa.begin()` return value ignored** (`:408`). A missing or miswired SX127x boots "fine" and just never receives anything, which looks the same as being out of range. Fix: check it, blink an error pattern, and emit `STATUSTEXT "LoRa radio init FAILED"` at 1 Hz over USB.
- [ ] **Blocking `Serial.write` on a stalled host** (`:270-283`). This is already documented as deferred. It is listed here only so the tracking issue has it: the planned fix is an outbound ring buffer.

### Bondor UI
- [ ] **`UL_DROP` and `KILL` are never displayed.** The bridge emits both (`main.cpp:385,397`), but `grep -rn "UL_DROP\|'KILL'" bondor/src` finds nothing. UL_DROP is the one counter that `main.cpp:394-396` says "must stay 0 during a parameter import". Add both to the LoRa tab, and show KILL on the Dive tab next to Leak.
- [ ] **The firmware revision probe is never retried** (`store/telemetry.ts:257-260`). `REQUEST_MESSAGE(AUTOPILOT_VERSION)` is sent once per connection. If that frame, or its reply, is lost (UDP via BlueOS measured ~8-9 % loss, and LoRa is worse), the Firmware tile says "— no reply" for the whole session. Fix: retry every 2 s until `fwBehaviourRev >= 0`.
- [ ] **Move console turns typos into zeros** (`views/ModesView.tsx:286-290`). `Number(primary) || 0` sends DIVE to 0 m or TURN by 0° for any non-numeric input. For TURN, `s > 1 ? s : 0` silently replaces a yaw rate of ≤ 1 °/s with 0 (= the firmware default). Fix: refuse to send on `!Number.isFinite`, and pass the rate through.
- [ ] **`MV_STATE`/`MV_PROG` are read non-reactively** (`ModesView.tsx:386-387` uses `useTelemetry.getState()` inside render), so they only refresh when something else re-renders the view. (The `?? 0` → `0.00` half of this line is already in #2.)
- [ ] **UDP peer auto-learn trusts any sender** (`main/mavlink/udpLink.ts:124`). The first and every later datagram from any host re-targets Bondor's commands (ARM included) to that host. On a shared pool LAN, another team's traffic on 14550 would take them. Fix: learn once from a HEARTBEAT with sysid 1, then accept data only from that peer (or warn on a peer change).
- [ ] **Serial open asserts DTR, and the comment accepts a board reboot** (`serialLink.ts:41-48`: "A DevKit on direct USB may reboot once on connect — harmless"). A reboot of the flight controller is not harmless if the vehicle is powered and armed on a tether. Verify on the SROT board whether DTR alone can reset it. If it can, make DTR assertion opt-in (only the ESP32-C3 bridge needs it) and rename the option accordingly.

### Docs vs code
- [ ] `LoraView.tsx:33-37` still says LoRa is *"receive-only; command uplink still goes over UDP/USB"*, and `:73` says the frame is *"~4 Hz"*. The bridge has a bidirectional TDM uplink, and the board sends a frame every 120 ms (~8 Hz; `srot-control-board/src/tasks/task_lora_sd.cpp:66`). `README.md` "Connecting" table: "Receive-only". AGENTS.md already says to fix this prose.
- [ ] `AGENTS.md:18-19` still says "a compact **39-byte** telemetry frame". It is 41 bytes since `aux_mv` (`shared/lora_telem_proto.h:5-9`).

## Failure scenario
Each item above describes its own. None is a direct vehicle hazard on its own.

## Suggested fix
Inline per item.

## How to verify
Per item: for the UI items, a screenshot of the new tile or a unit test on the parsing. For `openExternal`, a unit test with `file:///etc/passwd` that expects a refusal. For `LoRa.begin`, boot the bridge with the radio unplugged and see the STATUSTEXT.

## Severity: Low
Defence in depth, diagnostics and documentation. They are grouped so they do not crowd the higher-severity issues.

## Related
#2, PR #4. The LoRa uplink-queue issue filed alongside (UL_DROP visibility).

---
_Generated by [Claude Code](https://claude.ai/code)_
