# Mongla / mongla_ws — Unified Bug Register

> **This file is the SINGLE tracker for code defects in `mongla_ws`.**
> It replaces the bug content previously spread across `known-issues.md`,
> `ROADMAP.md`, `ROADMAP.md`, `water-owed.md` and
> `srot-pre-dive-gates.md`. See §6 for exactly what was migrated, what was
> deliberately left in place, and why.
>
> Produced by the system-wide audit of 2026-09-08, post the `srot`→`main` merge
> (`6db956a`). Scope: controls, vision, planner, sensors, managers, plus the
> three sibling repos.
>
> **STATUS: 42 of 48 fixed (2026-09-08).**
> B01, B02, B03, B05, B09, B10, B21 (the first SROT-path batch) · B16, B22, B23,
> B27 (vision/tooling) · B18, B30 (the srot vision axes) · B25, B26, B29 — found
> while fixing the others. Each landed with a test **verified to fail without the
> fix**.
>
> **No open item is reachable on the live srot flight path.** That claim is
> checked mechanically by `tools/srot_reachability.py`, which walks CALLS from the
> real srot entry verbs — **not** by grep and not by which modules are imported.
> Both of those gave wrong answers in the same pass (B12 false positive, B18 false
> negative), and chasing the contradiction is what surfaced **B30**, a live
> default-on `AttributeError` in the vision arrival brake. Re-run that tool before
> ever writing "not on this path" in here.
>
> The remaining open items are ArduSub-path (B06, B07, B08, B12, B13, B15, B17,
> J01 — preserved on the `pixhawk` branch), DVL (B04, B11, B14, B19, B20 — not
> fitted), FSM (J02, J03 — never run), or docs-only (B24, whose doc half is done
> and whose code half is B06).
>
> ⚠ **B28 is the one srot-path item still open, and only half of it is ours.** The
> host guard is fixed and tested; the firmware fix is upstream PR
> srot-control-board#15, source-verified but **not bench-demonstrated** — the board
> refuses to arm without the thruster pack.
>
> ⚠ **J02 is inert only because J03 is true.** `SurfaceState` swallows the failure
> of `set_depth(0.0)` and returns `SUCCEED` unconditionally — a failed ascent
> reported as success. Fix it **before** the FSM is ever switched on for a run.

## How to read this

Severity is about *consequence on the vehicle*, not about how hard the fix is.

| Grade | Meaning |
|---|---|
| **CRITICAL** | Can hurt the hull or hide a condition that can. Fix before water. |
| **HIGH** | Wrong control output, or a wrong number reported as right. |
| **MEDIUM** | Correct today, wrong under a reachable condition. |
| **LOW** | Real but bounded; style/robustness debt with a named trigger. |

`REPRODUCED` means a runnable check was written and executed in this pass, and
the output is quoted. Everything else is read from source and cited by line.

**The single most useful sentence in this document:** in the three worst
findings the *analysis was already correct and the wiring was not* — a leak
reporter that is never registered (B01), a calibration flag no caller consults
(B04), and a guard disarmed by its own caller (B05). Reviewing for "is this
logic right?" would have passed all three.

---

## 1. CRITICAL

### B53 — every payload channel the missions use is REFUSED by this board `MEASURED 2026-09-22, re-read 2026-10-01` ✅ **MAP REMOVED 2026-10-01** — channels await the wired payload
**`missions/competition_config.py` vs the board's `SERVOn_ROLE` params**

Read live off the board, all 16 channels, twice each (retry on timeout):

```
FIREABLE (SWITCH):  9, 10, 12, 13, 14, 15, 16
SERVO (must not drive): 1, 2, 3, 5, 6, 7, 8
NONE (unroled): 4, 11
```

What the missions believe (`competition_config.py:60,189`, `pool_day_practice.py:63,65`):

> `1=torpedo_1, 2=torpedo_2` · `3 = dropper_1, 4 = dropper_2`

**Every one of those is SERVO or NONE on this board.** Resolved by AST — five real
`fire()` call sites, not grep guesses:

| call site | channel | role | result |
|---|---|---|---|
| `sauvc_target_acquisition.py:88` | 3 | SERVO | `FIRE_REJECTED_ARM` |
| `task_bin.py:96` | 3 | SERVO | `FIRE_REJECTED_ARM` |
| `pool_day_practice.py:178` | 1 | SERVO | `FIRE_REJECTED_ARM` |
| `pool_day_practice.py:206` | 3 | SERVO | `FIRE_REJECTED_ARM` |
| `demo_dual_camera.py:68` | 3 | SERVO | `FIRE_REJECTED_ARM` |

⛔ **If this board's roles are representative, every drop and every torpedo scores
zero** — the dropper and target-acquisition tasks in full.

✅ **The guard works.** `fire()` fails closed: it refuses a SERVO channel rather
than writing `DO_SET_SERVO` to the on-board arm, and refuses an unreadable role
rather than guessing. Nothing here silently no-ops. That is the design working
exactly as intended, and it is why this was findable on a bench.

⚠ **TWO HYPOTHESES, AND THIS ONE IS NOT OURS TO CLOSE.**

1. **The mission channel map is wrong** and should point at 9/10/12–16.
2. **This bench board's `SERVOn_ROLE` params are simply unconfigured.** "1–8 servo,
   9–16 switch" is exactly the folklore `preflight_roles`' own docstring says the
   full read exists to retire — and this board almost matches it, with two
   unexplained holes at 4 and 11. A default or partial config would look like this.

The roles are **firmware state set in Bondor**, not ours, so the answer is an
operator/GCS fact, not a code fact. Recorded rather than "fixed", because
repointing five missions at channels nobody has confirmed would be swapping one
unverified map for another.

**To close it:** on the competition hull, run the channel read and compare. If the
hull agrees with this board, the missions move; if it disagrees, this bench board
gets configured and nothing in `src/` changes.

**2026-10-01 — the invented map is gone, and an unassigned channel is loud.** The
user confirmed nothing is wired to the payload yet, and the bench board re-read
identically (SWITCH 9,10,12–16; SERVO 1,2,3,5–8; unroled 4,11). So instead of
swapping one unverified map for another:

- `competition_config.py` is the **one** place channels are assigned:
  `TORPEDO_1/2_CHANNEL`, `DROPPER_1/2_CHANNEL`, all **0 = NOT ASSIGNED**, with the
  board's live read written beside them. `BIN_`/`SAUVC_DROPPER_CHANNEL` are aliases.
  Five missions that hard-coded 1 or 3 now read those names.
- ⛔ **A silent drop was found on the way.** `vision_verbs._parse_channels` dropped
  anything outside 1..16 — so `align(fire=0)` would have fired *nothing* with no
  error. Out-of-range numbers now pass through, and `fire()` refuses each with a
  reason that lands in the verb's payload outcome.
- `fire(0)` refuses as **"payload channel NOT ASSIGNED"**, and every refusal (0,
  SERVO, disabled) now names the board's own SWITCH channels from the roles read at
  bring-up — the operator gets the answer, not just the "no".
- `test_missions_never_hard_code_a_payload_channel.py` fails on a literal channel
  in any `fire(...)` / `fire=` or any `*_CHANNEL = <n>` outside the config.

**To close fully:** wire the payload to SWITCH channels, set the four names, and
update `test_the_channels_are_unassigned_until_someone_confirms_them`.
