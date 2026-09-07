# Upstream asks — `duburi_ws` → `srot-control-board`

We do not develop the firmware. srot / Hengla / Bondor belong to the firmware
and GCS teams, and their `AGENTS.md` states the rule from their side too: *"we
never commit to duburi_ws, and they never commit here."* PRs and issues are the
channel, so these are written here, versioned with the evidence that produced
them, and opened on their repo.

Every claim in these documents is either read in their source (file and line) or
measured on the live board on 2026-09-03. Where the two disagreed, the board
won and the document says so.

| file | ask | cost to them |
|---|---|---|
| [`pr-a-vision-api.md`](pr-a-vision-api.md) | the measured camera FOV that was blocking `LANDING_TARGET`; the pinhole bearing model; **the `31001` id allocated twice**; the missing `coasted`/gap-age fields | a doc fix and one decision |
| [`pr-b-telemetry-budget.md`](pr-b-telemetry-budget.md) | populate `ControlState.out_*` (six lines; `VFR_HUD.throttle` is a measured permanent zero); enrich the SD `Record` by **+54 B/record = 0.116 %** of the card | small, costed |
| [`pr-c-pico-esc-health.md`](pr-c-pico-esc-health.md) | the Pico decodes ESC voltage/current/temperature into `tval` and discards it; `mav_stream` then packs zeros into fields we already decode | a dual reflash |
| [`pr-f-esc-presence-on-the-wire.md`](pr-f-esc-presence-on-the-wire.md) | `esc_present`/`esc_fault` are **already members of `Snap`**, in scope at the packing site, while `ESC_TELEMETRY.count` ships literal zeros — presence never reaches the wire, so our thruster health can only ever say UNKNOWN | **one line, ESP32 only, zero bandwidth** |
| [`pr-h-kill-is-ambiguous.md`](pr-h-kill-is-ambiguous.md) | `KILL = 0` means "power live" OR "no 2nd-board link" and nothing tells them apart; **nothing refuses a cut kill switch on either side**, so a killed hull arms, runs every verb and fires the payload motionless | **one line, ESP32 only** — their own suppression rule, already applied twice in the same function |
| [`pr-g-optical-flow-ingest.md`](pr-g-optical-flow-ingest.md) | accept `OPTICAL_FLOW_RAD` (106) — the bottom camera is a bottom-track velocity sensor now (30 cm ±1.09 cm) and the board has **no velocity ingest of any kind** | a handler + one state field |
| [`pr-d-protocol-honesty.md`](pr-d-protocol-honesty.md) | `REQUEST_MESSAGE` ACCEPTs all 190 ids and emits 7 (including ids `-1`/`-2`); no `TIMESYNC`; the SD log cannot be pulled over the link | one switch; ten lines for TIMESYNC |

**Ranked, if only one lands:** PR A §3, the `31001` collision. It is the only
item that is cheap now and irreversible later, and both claimants are ours.

**Ranked, if only one CAPABILITY lands: PR F, and it now outranks PR C.**
Measured 2026-09-07: 958 CRC-valid `ESC_STATUS` frames with no ESCs attached,
every rpm exactly `0` — the board fills all eight slots regardless, so nothing
on the present wire separates eight healthy thrusters from none. PR F is one
line in a function that already holds the answer; PR C is a dual reflash for
the richer half. Presence first, then instrumentation.

**Filed as issues rather than PRs**, so severity stays legible — both are in
[`pr-b-telemetry-budget.md`](pr-b-telemetry-budget.md) §5:
the `JS_ARM` / `JS_ARM_TOGGLE` pre-arm bypass (safety), and four points of doc
drift (low).

## What we owe them, and its state

| they asked | state |
|---|---|
| camera FOV (`JETSON_FEEDBACK.md`: *"Still blocked on us, not you"*) | **delivered** — 63.82° air / 46.72° water ±0.7, held-out validated |
| adopt the `SYS_STATUS` leak health bit they added for us in rev 3 | **done this round** — see `srot-board-soul.md` §4 |
| measure `MV_PROG` on TURN in water | outstanding — needs water |

## The reciprocal rule

Their `TASKS_FROM_DUBURI_WS.md` §6 asks that any wire change be named
explicitly in the commit message and that `SROT_FW_BEHAVIOUR_REV` be bumped in
the same commit. The reason is recorded on both sides: our
`test_srot_protocol_drift.py` greps their C++ to detect behaviour changes, and
it **stayed green through their entire `MOVE_STOP` fix**. Source text is a
brittle proxy for behaviour; a number bumped deliberately is not.

So each PR above names the rev bump it needs. Where we ask for one, it is
because we gate on the rev and will otherwise keep using the old path — which
is exactly what happened to the rev-3 leak bit for four rounds.
