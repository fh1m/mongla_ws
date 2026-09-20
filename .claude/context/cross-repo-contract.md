# Cross-repo contract — mongla_ws ↔ SROT ↔ Bondor

This is `mongla_ws`'s copy of the shared agent instructions. Each sibling repo carries the same
**Shared invariants** section verbatim in its `AGENTS.md`.

## The system

```
Jetson Orin Nano  ──USB serial 115200──  SROT board  ──1 Mbaud UART──  RP2350 Pico ── 8× ESC
  mongla_ws (this repo):                  "Hengla"                      thruster/RPM
  ROS 2, YOLO11, missions,                (srot-control-board)
  payload, mission DSL                        │
                                              └── LoRa ── Bondor (srot-ground-station)
```

**No Raspberry Pi, no BlueOS, no UDP router.** The Jetson connects to the board's USB-C port
directly. Bondor is a parallel, independent link and is **not** in the control path.

| Repo | Owns |
|---|---|
| **mongla_ws** (here) | perception, mission logic, the action/DSL surface, payload decisions |
| **srot-control-board** | every real-time control loop at 500 Hz, arming, all failsafes |
| **srot-ground-station** | parameters, calibration, tuning, motor test, manual piloting |
| **srot-esc-flasher** | one-time bench tool; Bluejay onto the ESCs (precondition for RPM) |

**The division is deliberate: the Jetson does perception; the board does control.** See
[`vision-control-split.md`](vision-control-split.md) for the vision half of that contract.

---

## Shared invariants (identical in every repo's AGENTS.md)

These are **co-owned across repos**. Changing one unilaterally breaks a partner silently — no
exception is raised, the vehicle just behaves wrong.

1. **The wire constants are frozen unless changed on both sides in the same PR.**
   `MAV_CMD_SROT_MOVE = 31000`; the `SROT_MOVE` p1 type codes and their ordering; the
   `FlightMode` integers; `PCA_RELAY_BASE_CH = 8`; `MAVLINK_BAUD = 115200`;
   `GCS_FAILSAFE_MS = 5000`.
   We mirror them in `src/mongla_control/mongla_control/fc/srot_protocol.py` — **our one
   copy** — and `test_srot_protocol_drift.py` reads the firmware headers directly and fails on
   divergence.

2. **`movement::Type` is append-only.** The wire mapping is `mv_type = wire + 1`, so inserting
   a value silently renumbers every verb after it — `move_forward` would become a strafe.

3. **Every command reaches exactly one terminal ACK.** `ACCEPTED` / `CANCELLED` / `FAILED` /
   `DENIED`, plus `TEMPORARILY_REJECTED` on a mutex miss. `IN_PROGRESS` is not terminal. An
   action client that never gets a terminal result **hangs** — worse than any error.

4. **Depth sign: `VFR_HUD.alt` is negative below the surface.** Our whole stack compares depth
   against negative constants (`STYLE_ROLL_SURFACE_GUARD_M`, `_MIN_DEPTH_M`, `max_depth_m`,
   `depth_ceiling_m`), so a sign flip does not error — it silently disables every depth guard.
   We shipped exactly that bug and it was invisible until read.

5. **A heartbeat ≥ 1 Hz is mandatory** or the board surfaces after `GCS_FAILSAFE_MS`.

6. **Never break the contract to fix a bug.** If the right fix changes the wire, say so and
   coordinate — do not add a compensating hack on one side. Both codebases are in active
   development; a clean change on both sides is cheaper than a workaround that outlives its
   reason.

---

## Rules specific to this repo

**`srot_protocol.py` is the one copy.** Every SROT wire constant lives there and nowhere else.
There are four copies system-wide (firmware `config.h`, `shared/lora_telem_proto.h`, Bondor's
`protocol.ts`, ours) and they are hand-maintained by design — three repos, three languages.
The drift test is what makes that safe. Do not add a fifth copy.

**Read `JETSON_COMMS.md` for the wire, but verify against the firmware source.** The board's
docs have drifted from its code repeatedly, and we have been misled by it — a documented
"clamped at ≥ 0" that was not clamped, a "brake to a halt" that coasts, a four-result ACK table
missing a fifth result. When a doc and the code disagree, **the code is ground truth.**

**Workarounds for firmware defects must be documented and self-retiring.** We carry a host-side
brake because `MOVE_STOP` applies zero thrust; the drift test **fails when the firmware fixes
it**, so we remove the workaround rather than braking twice. Any future workaround should have
the same property.

**Send findings upstream, ranked by what blocks us.** `srot-control-board/JETSON_FEEDBACK.md`
is ours to maintain. It is filtered to "what can the companion not fix from its own side" —
not general firmware review.

**The two backends must stay separable.** `flight_controller:=pixhawk|srot` selects at runtime;
`PixhawkFC` **is-a** `Pixhawk`, so the legacy path is byte-identical. Do not let srot-specific
behaviour leak into shared code — gate on `_is_srot` or put it behind the HAL.

**Verb support on srot is partial and honest about it.** `srot_fc.UNSUPPORTED_VERBS` refuses
what is not ported rather than letting it fail obscurely. Removing a verb from that set is how
the port lands — implement, then delete the line. `test_no_facade_mode_gate_is_reachable_on_srot`
enforces that nothing reaches an ArduSub-only mode gate.

---

## ⚖ Ownership, settled 2026-08-07 after the water test

**We develop `mongla_ws` only.** No agent working in this repo writes code in
`srot-control-board`, `srot-ground-station` or `srot-esc-flasher` any more — the firmware and
GCS teams own those, with their own agents. Our obligation to them is exactly one thing:
**stay in harmony**, and prove it mechanically.

| direction | mechanism |
|---|---|
| **them → us** | a **pull request on `mongla_ws`** ([#5](https://github.com/fh1m/mongla_ws/pull/5) is the template). They never commit here directly. |
| **us → them** | a **pull request on their repo**, appending a numbered Round to `TASKS_FROM_DUBURI_WS.md` ([srot-control-board#1](https://github.com/RakibulIslam1/srot-control-board/pull/1) is the template). Symmetric with the above: a PR is a *request*, and they merge it. |
| **proof of harmony** | `Mongla_others/srot-control-board` is our **read-only mirror** of the firmware. Fast-forward it to the flashed baseline, then run `test_srot_protocol_drift.py` — it reads their headers directly and fails on any divergence. |

**We hold WRITE on their repo, so the discipline is ours to keep, not GitHub's.** Never push
to their `main`, never edit their C/C++, and never "helpfully" fix a firmware defect in their
tree — describe it, cite `file:line` against a named commit, and suggest a fix they can reject.
The reason is not politeness: their agent reasons about code it wrote, and a silent edit from
us breaks that. Work on a `mongla-ws/<topic>` branch, then **return the mirror to their `main`
and re-run the drift suite** — a checked-out feature branch would make the mirror lie about
what is flashed, which is the one thing it exists to tell the truth about.

⚠ **Cite line numbers against a commit, and re-verify them at write time.** Round 8 inherited
`task_control_loop.cpp:236-237` from an earlier round; the depth call had moved to `:240-241`.
A stale citation in a cross-repo PR sends someone to the wrong function in their own code.

So the routine when a firmware revision lands is fixed and short:

```bash
git -C Mongla_others/srot-control-board fetch origin && git -C Mongla_others/srot-control-board merge --ff-only origin/main
SROT_FW_DIR=$PWD/Mongla_others/srot-control-board python -m pytest src/mongla_control/test/test_srot_protocol_drift.py -q
# then bump FW_BEHAVIOUR_REV in srot_protocol.py to the new rev and document what it means
```

⚠ **A green drift suite proves the wire matches, not that the vehicle behaves the same.**
Rev 7 passed clean while shipping `FRAME_REVERSE`, which inverts every axis on our hull —
a *param*, invisible to a header-reading test. Read the release's commits, not only its rev
number, and record any behaviour delta in the `FW_BEHAVIOUR_REV` comment block.

## When your change crosses a repo boundary

1. Say so explicitly in the commit message, naming the other repo.
2. Update `srot_protocol.py` and expect the drift test to fail until the other side lands —
   that is the design, not a problem.
3. If it changes the wire, update `JETSON_COMMS.md` on the board side too.
4. If it is a firmware defect we cannot fix here, add it to `JETSON_FEEDBACK.md` with a
   `file:line` and a concrete suggested fix, not just a complaint.

## ⛔ Standing safety gate

The board's **depth loop has never run closed** — the sign was inverted until 2026-07-30 and
the Bar30 was not fitted while it was written. Two bench checks
(`DUBURI_WS_INTEGRATION.md:295-301`) gate every dive-dependent verb: DEPTH_HOLD must push back
toward the latched depth when the sensor is hand-moved, and a forced leak failsafe must
**ascend**. A successful in-air `move_forward` does not count as partial validation.
