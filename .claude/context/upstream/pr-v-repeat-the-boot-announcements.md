# PR V — repeat the boot announcements on request

**Branch:** `duburi-ws/boot-announcements-on-request` (builds for
`esp32doit-devkit-v1`). **Status:** opened 2026-11-06 as
[PR #32](https://github.com/RakibulIslam1/srot-control-board/pull/32).

## The problem

The board says the things that change how it behaves **once**, in a burst at
boot (`task_mavlink.cpp`, the block after `sendAutopilotVersion()`): the reset
reason, *"Params reset to build defaults"* (pilot gain back to half, leak
failsafe off), *"NVS reformatted"* (calibration lost), the calibration state,
and the migrations. Nothing else on the wire carries them.

A companion that was not listening at that moment never learns them. On USB
that is rare, because opening the port reboots the board (`port_guard.py`) and
the burst follows. But it is the normal case on any link without the
auto-reset, on a board revision that defeats it, and whenever a reader starts
late.

`REQUEST_MESSAGE` already ACCEPTs every id (PR D), so asking for `STATUSTEXT`
today returns ACCEPTED and nothing.

## The ask

- Move the burst into `announceBoot()`, unchanged, and call it from the boot path.
- Call it again on `MAV_CMD_REQUEST_MESSAGE` with `param1 = 253` (STATUSTEXT).
- The command handler runs on `Task_MAVLink`, the same task as the boot path,
  so there is no new concurrency. The heartbeat and `AUTOPILOT_VERSION` stay
  boot-only; `AUTOPILOT_VERSION` already has its own request.

## What mongla_ws already does

`SrotFC.request_boot_announcements()` sends the request at preflight, and reads
the answer off the burst **arriving** (its first line, `RST: ...`), never off the
ACK. If the burst still has not come, `boot_warnings()` returns `None`, and the
manager logs *boot announcements NOT SEEN* with how to read them, rather than
printing nothing, which reads as "nothing wrong". The boot lines are kept
outside the 64-line STATUSTEXT ring, so chatter cannot push them out before
preflight reads them.
