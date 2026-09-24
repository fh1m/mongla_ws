# Bondor accepts HEARTBEAT and telemetry from any sysid/compid. Behind a MAVLink router, duburi_ws's heartbeat flips ARMED to DISARMED, and the toggle button then sends ARM when the operator means DISARM

Part of the 2026-09-24 ecosystem review (tracking issue in fh1m/mongla_ws to follow).

## Summary
`MavlinkConnection.handlePacket` throws away the MAVLink header (sysid/compid) before forwarding to the renderer, and the store's `HEARTBEAT` handler sets `armed`, `mode` and `lastHeartbeatTs` from **every** heartbeat. On the documented concurrent setup (Bondor over UDP behind a MAVLink router / BlueOS, with duburi_ws on the same router), the router forwards duburi_ws's own heartbeat (255/191, `MAV_TYPE_ONBOARD_CONTROLLER`, `base_mode = 0`, `custom_mode = 0`) to Bondor. Bondor then shows **DISARMED / STABILIZE** at ~1 Hz, alternating with the vehicle's real heartbeat.

The Dive and Setup tabs' Arm/Disarm is a **single toggle computed from that state** (`arm(!t.armed)`), so a click during a "DISARMED" window **sends ARM (p1 = 1) to an armed vehicle instead of DISARM**. This is the only disarm control in the UI.

## Evidence (at `1adc14c`)
`bondor/src/main/mavlink/connection.ts:133-148`: the source ids are dropped:
```ts
this.onMessage({ id: clazz.MSG_ID, name: clazz.MSG_NAME, fields, ts: Date.now() })
```
(`packet.header.sysid` / `compid` are never forwarded.)

`bondor/src/renderer/src/store/telemetry.ts:243-248`: any heartbeat is taken as the vehicle's:
```ts
case 'HEARTBEAT': {
  set({
    lastHeartbeatTs: m.ts,
    armed: (Number(f.baseMode) & MAV_MODE_FLAG_SAFETY_ARMED) !== 0,
    mode: Number(f.customMode)
  })
```
`bondor/src/renderer/src/views/DiveView.tsx:269` (and `SetupView.tsx:131`): the toggle:
```tsx
<Button ... onClick={() => arm(!t.armed)}>{t.armed ? 'Disarm' : 'Arm'}</Button>
```
duburi_ws heartbeat (mongla_ws `src/mongla_control/mongla_control/fc/srot_fc.py:483-485`):
```py
self.master.mav.heartbeat_send(MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID, 0, 0, 0)
```
`ConnectionBar.tsx:28-36`: "link alive" is `Date.now() - lastHeartbeatTs < 3000`, so **any** heartbeat keeps the link chip green.

`udpLink.ts:124`: the UDP peer is also re-learned from **any** datagram source (`this.remote = rinfo`), so the same lack of source filtering applies to where commands are sent.

## Failure scenario
1. Pool day: duburi_ws runs a mission through BlueOS, and Bondor monitors over UDP 14550 via the router (the configuration AGENTS.md recommends for running both at once). The vehicle is ARMED.
2. Bondor's ARMED chip and button flicker. At the moment the operator clicks the red button to disarm, the last heartbeat was duburi_ws's, so the button reads **"Arm"** and `arm(true)` is sent. The vehicle stays armed.
3. Separately: if the vehicle's USB link to the Pi dies but duburi_ws stays up, Bondor's link chip stays **green** from duburi_ws's heartbeats while the vehicle is silent.

## Suggested fix
1. In `connection.ts:147`, forward `sysid: packet.header.sysid, compid: packet.header.compid` in `MavMessage` (add the fields to `protocol.ts`).
2. In `ingest`, accept `HEARTBEAT`, and derive armed/mode/lastHeartbeatTs from it, **only from the vehicle**: `sysid === TARGET_SYSTEM_ID && compid === TARGET_COMPONENT_ID` (also `autopilot !== MAV_AUTOPILOT_INVALID`, the same filter duburi_ws uses in `_vehicle_hb`). Consider applying the same filter to COMMAND_ACK, PARAM_VALUE and STATUSTEXT, and show "other MAVLink sources seen: 255/191" in the Inspector.
3. **Split Arm and Disarm into two explicit buttons** (Disarm always enabled while the transport is open), so a disarm click can never be turned into an arm by stale or foreign state. A keyboard shortcut for disarm (such as Space or Esc) that works on every tab would close the e-stop gap too.

## How to verify
- Unit: feed `ingest` a vehicle heartbeat (1/1, armed) followed by a 255/191 heartbeat with baseMode 0. Before the fix `armed === false`; after the fix it stays `true`.
- Bench: run `mavproxy.py --master=<board> --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551` with a pymavlink script sending 255/191 heartbeats at 1 Hz into the router. Arm from Bondor and watch the chip. Before the fix it flickers; after, it is steady.
- Injection check: revert the sysid filter and confirm the unit test fails.

## Severity: High
The e-stop path. The only disarm control can **send ARM instead of DISARM** in a documented configuration, and the link-alive indicator can lie. It needs a second source on the link, which is exactly the pool-day configuration.

## Related
- #2 (values never expire): with that issue, a stale `armed` also drives this same toggle.
- `srot-control-board` `mav_commands.cpp` `onManualControl`: `s_prev_buttons` is a single global. If duburi_ws (buttons = 0) and Bondor (a button held) both stream MANUAL_CONTROL, every Bondor frame is a new rising edge, so `JS_ARM_TOGGLE` / `JS_GAIN_*` fire repeatedly. That needs a board fix (edge detection per source). Bondor cannot see the other sender, so it is noted here for the cross-repo tracker.

---
_Generated by [Claude Code](https://claude.ai/code)_
