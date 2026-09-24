# LoRa bridge: queued uplink commands never expire and DISARM has no priority. A stale ARM/SROT_MOVE runs when the link comes back, and a full queue silently drops DISARM

Part of the 2026-09-24 ecosystem review (tracking issue in fh1m/mongla_ws to follow).

## Summary
The ESP32-C3 bridge's uplink queue (`s_ulq`, 24 entries) is FIFO with **no timestamps, no expiry and no priority**. Two consequences:
1. **Stale commands.** The queue drains only when a downlink frame arrives (TDM slot). While LoRa is out of range, whatever the operator clicks (ARM, SROT_MOVE forward 3 s, DO_MOTOR_TEST keep-alives, a mode change) accumulates. When the link comes back (seconds, minutes or hours later, possibly after Bondor has been closed and reopened) the bridge sends all of it and the vehicle **acts on commands that Bondor's UI stopped tracking long ago**.
2. **DISARM / STOP / SURFACE wait behind anything else**, and are **dropped outright** when the queue is full. Disarm competes FIFO with parameter reads; if 22+ entries are queued, the three DISARM copies hit `ulqFull()` and are discarded (only `UL_DROP` counts it, and Bondor never shows `UL_DROP`).

## Evidence (at `1adc14c`)
`src/groundstation/main.cpp:139-147`: overflow is a counter bump, whatever the message:
```cpp
static void ulqPush(const mavlink_message_t& m) {
  if (ulqFull()) { s_ul_drop++; return; }
  s_ulq[s_ulq_head] = m;
```
`main.cpp:214-236` (`queueUplink`): nothing is stamped when it is enqueued. State commands are sent 3× and superseded by class, and **everything else (SROT_MOVE, DO_MOTOR_TEST, PARAM_*) is queued once and never removed except by popping**.

`main.cpp:253-258`: the queue is drained strictly FIFO, one per slot, and nothing checks age:
```cpp
static void sendUplinkSlot() {
  mavlink_message_t q;
  if (ulqPop(q)) { loraSendMsg(q); return; }
```
`main.cpp:434-450`: `sendUplinkSlot()` only runs when a valid telemetry frame is received, so with no downlink the queue is frozen, not flushed. The USB-activity gate at `:448` (`now - s_last_usb_ms < 2000`) pauses the drain but does not clear the queue either.

Bondor never surfaces the drop counter: `grep -rn UL_DROP bondor/src` returns nothing. `LoraView.tsx` shows USB_RX/UP_TX/UL_RX only.

Board side (for context): `srot-control-board/src/comms/mav_commands.cpp` `MAV_CMD_SROT_MOVE` sets `mode = AUTO` and starts the leg on receipt, and COMPONENT_ARM_DISARM arms immediately if `canArm()` passes. Neither has any notion of command age.

## Failure scenario
- The vehicle is at the edge of LoRa range. The operator clicks **Arm** (no response, because the link is down), then sends **SROT_MOVE FORWARD 3 s** twice from the Modes tab, gives up and walks to the pool edge. Two minutes later the vehicle drifts back into range: the bridge sends ARM ×3, then both FORWARD moves. **The vehicle arms and drives forward 6 s with nobody expecting it.**
- During a lossy LoRa parameter download, the gap-filler (`telemetry.ts:637-650`) queues up to 8 `PARAM_REQUEST_READ` per 900 ms. The operator presses **Disarm**. The DISARM copies either sit behind the backlog (~125 ms per slot, so up to ~3 s) or are silently dropped if the queue is full.

## Suggested fix
All in the bridge, with no wire change:
1. Stamp each queue entry with `millis()` on push. In `sendUplinkSlot()`, **discard entries older than a short TTL** (such as 2 s for COMMAND_LONG and SET_MODE, and longer for PARAM_* if you want), and count them in a new `UL_STALE` diagnostic.
2. **Priority lane**: when `queueUplink` sees COMPONENT_ARM_DISARM with p1 = 0, SET_MODE/DO_SET_MODE to SURFACE, or SROT_MOVE type STOP (6), put it at the **head** of the queue (or in a dedicated 1-slot "urgent" register that `sendUplinkSlot` checks first), and **never drop it**. If necessary, evict the oldest non-state entry to make room.
3. On a USB gap (`now - s_last_usb_ms >= 2000`), **flush the queue** so a Bondor restart cannot inherit a previous session's commands.
4. In Bondor, show `UL_DROP` (and `UL_STALE`) on the LoRa tab, and warn when `UL_DROP` increases during `writeParams`.

## How to verify
- Bench: power the bridge with the vehicle off (no downlink). Click Arm and send one SROT_MOVE, wait 30 s, then power the vehicle on. Before the fix, `UL_RX` increments by 4 and the vehicle arms. After the fix, nothing is delivered and `UL_STALE` = 4.
- Fill the queue: temporarily lower `UL_Q` to 4, queue 4 PARAM reads, then press Disarm. Before the fix `UL_DROP` rises and the board stays armed. After the fix DISARM is sent in the next slot.
- Injection check: remove the TTL check and confirm the first test fails again.

## Severity: High
The uplink can deliver an **ARM or a motion command that nobody is expecting**, and the one command that must always get through (DISARM) can be delayed or dropped with no operator-visible signal. Mitigations: SROT_MOVE requires armed (fw rev 13+), and ARM still passes `canArm()`.

## Related
- AGENTS.md "LoRa is a 4 Hz TDM slot, not a pipe" documents the silent discard but treats it as a pacing concern, not a safety one.
- #1 (motor-test keep-alive leak): over LoRa, leaked keep-alives also queue behind this FIFO.
- The board's `AUDIT.md` defers CRC on mission upload. The bridge side has the SX127x hardware CRC enabled (`main.cpp:415`), so that part is fine here.

---
_Generated by [Claude Code](https://claude.ai/code)_
