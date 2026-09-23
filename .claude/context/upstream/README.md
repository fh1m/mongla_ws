# Upstream asks — `mongla_ws` → `srot-control-board`

We do not develop the firmware. srot / Hengla / Bondor belong to the firmware
and GCS teams, and their `AGENTS.md` states the rule from their side too: *"we
never commit to mongla_ws, and they never commit here."* PRs and issues are the
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
| [`pr-e-vision-offload.md`](pr-e-vision-offload.md) | move the terminal visual-servo hold onto the board's fixed 500 Hz tick, replacing companion-side constants (continuity-lock gate, `align_stable_frames`, Kalman coast) that silently rescale with whatever the perception stack's rate happens to be that day | depends on PR A; a new board-side control mode |
| [`pr-j-simulated-esc-bench-mode.md`](pr-j-simulated-esc-bench-mode.md) | a simulated-ESC bench mode: the Pico already runs this model forward at `main.cpp:242`, so synthesising rpm for an ABSENT thruster is that line read backwards. Unblocks the display's thruster indicators, `motor_tune`'s plant fit, and every `ESC_TELEMETRY` consumer we have written but never executed. **We ask them to decline it rather than ship it unmarked** | small, Pico only |
| [`pr-i-spin-min-relay.md`](pr-i-spin-min-relay.md) | `MOT_SPIN_MIN` turns every actuator below its floor into a relay — a 16-point DShot cliff at `t = 0.005` — and mixed-axis commands bend thrust direction as a consequence; filed as an issue, not a PR | ranked above #4, alongside #10 |
| [`pr-k-the-mixer-is-for-a-different-hull.md`](pr-k-the-mixer-is-for-a-different-hull.md) | the mixer is `vectored_6dof` for **eight** T200s at 45°; the hull being built is **five** orthogonal thrusters with roll unactuated, so every axis row is wrong for it. Asks for a frame selector — or better, `M` computed from geometry at boot. ⭐ Explicitly asks them to **keep** the per-group saturation scaling | a frame table, or a boot-time derivation |
| [`pr-m-thruster-requirements-from-control.md`](pr-m-thruster-requirements-from-control.md) | **to the hardware team, not firmware.** What the control layer needs from the custom thruster: ⭐ turn smoothly from zero so `MOT_SPIN_MIN` can go to 0 (measured: a 5 % yaw stick produces **0.0 %** output; the first demand that moves the hull produces **16.8 %**), symmetric reverse, rpm **and current** telemetry, a measured thrust curve at our voltage, and named thruster bodies with materials in CAD | a design constraint, taken before the thruster is cut |

**Ranked, if only one lands:** PR A §3, the `31001` collision. It is the only
item that is cheap now and irreversible later, and both claimants are ours.

**2026-09-22 — PR I's table is now MEASURED, and PR F outranks a board we were going to build.**
Predicting all eleven points of a demand ladder from their own source agreed with the live board to
**0.30 %** worst case (under one count of 999 everywhere but saturation), so PR I §1 is confirmed
rather than derived — see its addendum. Separately, reading `main.cpp:393-405` and `mav_stream.cpp:190`
settled that thruster **presence is already measured on the Pico and simply never reaches the wire**,
which ranks PR F above **H-4, our own per-thruster current-sense board** — a multi-week build we had
scheduled, against one line in a function that already holds the answer.

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
