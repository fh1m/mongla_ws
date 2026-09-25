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
| [`pr-n-caps-literals-and-the-heading-hold-setpoint.md`](pr-n-caps-literals-and-the-heading-hold-setpoint.md) | ⭐ **`attitude::holdYaw()` already exists and is already in the flight path** (`attitude_control.h:30`, called from `task_control_loop.cpp:238`) — but only from AUTO, so STABILIZE, the one mode that honours `MANUAL_CONTROL`, cannot reach it. Measured: yaw is **17.42 % → 70.67 % of full scale and zero below**, a 4.06:1 range, because the `0.02f` stick gate meets `MOT_SPIN_MIN`. A heading-hold setpoint makes small yaw a continuous function of heading error instead of a lurch. Plus: six control literals that should be parameters, and ⚠ `JS_GAIN` silently halving every axis with no way to set the live value | a handler and one call, then an audit |
| [`pr-o-thruster-step-response.md`](pr-o-thruster-step-response.md) | ⭐ **to the hardware team: one afternoon on a load cell closes FOUR open asks.** Measured on the closed-loop bench, thruster lag beats control-loop rate by 1-2 orders of magnitude -- tau 0.59 s costs **131x** the heading deviation of tau 0, while 500 Hz -> 50 Hz costs 21 % -- and we do not know ours. The break-away pair (step from stopped vs from running) decides whether `MOT_SPIN_MIN` can go to 0; the plateaus give the thrust curve, `REVERSE_EFFICIENCY` and `k_n_per_rpm2`; the transients give tau. Analysis ships with the ask and is validated against traces whose answer we chose | one rig, one afternoon |
| [`pr-p-the-allocator-they-asked-for.md`](pr-p-the-allocator-they-asked-for.md) | ⭐ **PR K's §4.2, implemented.** A geometry-driven allocator behind `FRAME_CLASS` (0 = today's ±1 matrix, bit for bit, and the default). Makes the six axes commensurate for the first time -- sway and yaw differ by **5.714×**, exactly the reduced matrix's condition number, and a ±1 mixer called both "1.0". Cancels the axial unit's 8.06 mm pitch moment. ⚠ **Also corrects PR K's own geometry**: its 346.6 / 520.4 mm came from the decimated render asset and the CAD says **350.00 / 519.00**, and the axial unit it marked UNKNOWN is at +330.60 with its duct ring at +351.00 | a patch, applied; 85 host-side checks |
| [`pr-q-motor-detect-and-frame-reverse.md`](pr-q-motor-detect-and-frame-reverse.md) | ⛔ **CRITICAL, and it outranks the rest of this batch.** `FRAME_REVERSE` appears at exactly ONE site in the control path, inside the mixer branch -- the motor-test and MOTOR_TUNE paths bypass it and `motorAngular()` knows nothing about it. So MOTOR_DETECT converges to a configuration correct only when `FRAME_REVERSE = 0`, and **on our hull it is 1**: a successful detect leaves every axis inverted in every closed-loop mode. The same gap makes Motor Test show the opposite of what the vehicle will do. Matches the 2026-08-07 water incident and the `[-1] × 8` state our CLAUDE.md already warns about | one multiply, or negate `M` once at boot |
| [`pr-r-thrust-trim-predicts-the-wrong-duty.md`](pr-r-thrust-trim-predicts-the-wrong-duty.md) | `thrust_trim` forms `meas/pred` with `pred = rpm_max · demand` -- the **pre-shaping** demand -- while `meas` is the RPM from the **post-shaping** duty, after `thstExpo`, `MOT_SPIN_MIN` and the voltage feedforward. The error is one-sided (`shaped(d) ≥ d`), so with the shipped 0.65 / 0.15 defaults `THR_TRIM_EN = 1` drives **every gain to its −25 % clamp on every dive**. Latent only because it defaults to 0 | publish the shaped duty; one line of plumbing |
| [`pr-s-a-timed-out-move-reports-accepted.md`](pr-s-a-timed-out-move-reports-accepted.md) | `MOVE_DIVE` and `MOVE_TURN` that never reach their target brake out on the global timeout, hit the SAME `PH_DONE` a success does, and are ACKed `MAV_RESULT_ACCEPTED` progress 100. Nothing carries why the move ended. ⚠ **Distinct from #8** (which is a timing problem); this fires after the move correctly finished, having failed. `s_remain` is already in scope at the site | one enum, one field |
| [`pr-t-the-brake-ignores-how-far-you-travelled.md`](pr-t-the-brake-ignores-how-far-you-travelled.md) | `PH_BRAKE`'s impulse is `g·k·v²` -- **no dependence on leg duration at all**. Below ≈0.21 s the brake impulse exceeds the forward impulse and the vehicle ends up **behind** where it started; at 3 s it removes 4.6 %. `abort()` already brakes on the ACHIEVED speed and normal completion on the COMMANDED one, in the same file. Short legs are precision alignment | accumulate `s_cur_speed·dt`, brake against that |

**Ranked, if only one lands:** ⛔ **PR Q.** It is safety-critical, it is one
multiply, and it describes the configuration our hull is in *right now* — a
MOTOR_DETECT run on this vehicle today leaves every axis inverted. PR A §3 (the
`31001` collision) remains the ranking cheap-now-irreversible-later item.

## 2026-09-25 — a batch of five, and ⚠ a correction to one we already sent

PR P **implements** what PR K asked for, and in the course of measuring the hull
properly it found that **PR K's own geometry is wrong** — 346.6 / 520.4 mm read off
the decimated web-viewer asset, against a true 350.00 / 519.00 from the CAD. Our
own `hull_geometry.yaml` retracted those figures the day after PR K was sent and
nobody went back to the PR. It is still the document the firmware team would
implement from, and it is in a different axis frame from our shipped constants
with nothing saying so. **Post the correction on their #25 before anyone builds
from it.** This is the `one truth, two copies is the bug` failure arriving across a
repo boundary, where no test of ours can reach it.

⏳ **ALL FIVE ARE UNSENT.** The session that wrote them had read-only GitHub access
to `srot-control-board` (`403 Resource not accessible by integration` on branch,
issue and PR creation). Everything needed to send them is here: the four issue
bodies above, and PR P's firmware patch in
[`patches/srot-geometric-allocator.patch`](patches/srot-geometric-allocator.patch),
which applies to their `main` at `f1d3ba9`.

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

## Where each ask actually is

⚠ **Written here is not filed.** K, M, N and O sat written and unopened for a week;
this table exists so that cannot happen silently again. Every row is a live URL on
`RakibulIslam1/srot-control-board`.

| ask | filed as | opened |
|---|---|---|
| A | PR [#2](https://github.com/RakibulIslam1/srot-control-board/pull/2) | earlier |
| B | PR [#3](https://github.com/RakibulIslam1/srot-control-board/pull/3), and the allocator half as [#20](https://github.com/RakibulIslam1/srot-control-board/pull/20) | earlier |
| C | PR [#4](https://github.com/RakibulIslam1/srot-control-board/pull/4) | earlier |
| D | PR [#5](https://github.com/RakibulIslam1/srot-control-board/pull/5) | earlier |
| E | issue [#9](https://github.com/RakibulIslam1/srot-control-board/issues/9) | earlier |
| F | PR [#10](https://github.com/RakibulIslam1/srot-control-board/pull/10) | earlier |
| G | PR [#11](https://github.com/RakibulIslam1/srot-control-board/pull/11) | earlier |
| H | PR [#12](https://github.com/RakibulIslam1/srot-control-board/pull/12) | earlier |
| I | issue [#13](https://github.com/RakibulIslam1/srot-control-board/issues/13) | earlier |
| **K** | PR [**#25**](https://github.com/RakibulIslam1/srot-control-board/pull/25) | **2026-09-24** |
| **M** | PR [**#26**](https://github.com/RakibulIslam1/srot-control-board/pull/26) | **2026-09-24** |
| **N** | PR [**#27**](https://github.com/RakibulIslam1/srot-control-board/pull/27) | **2026-09-24** |
| **O** | PR [**#28**](https://github.com/RakibulIslam1/srot-control-board/pull/28) | **2026-09-24** |

Each of the four adds exactly one document at their repo root and changes no code,
so none of them needs a `SROT_FW_BEHAVIOUR_REV` bump to merge — the bump each *ask*
would require is named inside the document, per the reciprocal rule below.

⚠ **J is written and NOT filed**, deliberately: it asks for a simulated-ESC bench
mode and asks them to decline it rather than ship it unmarked. File it only when
somebody is ready to own that conversation.

Their repos are also checked out locally at
`/home/fh1m/Envs/dockers/auv-ros2/Mongla_others/{srot-control-board,srot-ground-station,srot-esc-flasher}`.
Read the firmware there; ⛔ still never commit to it.

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
