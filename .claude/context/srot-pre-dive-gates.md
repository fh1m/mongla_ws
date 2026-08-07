# SROT pre-dive gates — final review before water, 2026-08-07

> Firmware on the vehicle: `srot-control-board` @ `3bd247b`, **`SROT_FW_BEHAVIOUR_REV 9`**,
> flashed and confirmed over USB. 232 params re-read after the flash: **zero drift, zero
> missing**. `CAL_MAG_*`, `CAL_LVL_*` and `FS_GCS_COMPID = 191` all intact.
>
> Read this with [`srot-integration.md`](srot-integration.md). This file is only the
> go/no-go list for the next in-water session.

---

## ⛔ GATE 0 — the axis configuration is UNKNOWN. Settle it first.

**This supersedes the "config is correct" conclusion in `srot-integration.md` §8.2.** That
block was written against `CAL_MDIR = [+1]×8` and no longer describes the board.

`MOT_n_DIRECTION` and `CAL_MDIR` **multiply** (`task_control_loop.cpp:140-145`). Read off
the vehicle 2026-08-07:

```
MOT_n_DIRECTION : [-1, +1, +1, +1, +1, +1, +1, -1]
CAL_MDIR        : [-1, +1, +1, +1, +1, +1, +1, -1]     <-- NOT all +1
effective       : [+1, +1, +1, +1, +1, +1, +1, +1]
```

`CAL_MDIR` changed **between 10:02 and 10:27 on 2026-08-07** (snapshot-diffed). Nobody has
claimed the write. It was `[+1]×8` before and has been `[-1,+1,+1,+1,+1,+1,+1,-1]` since.

### Why this is a gate and not a note

With the effective product, the earlier derivation's conclusion **inverts**:

```
w = [-1,+1,+1,+1,+1,+1,+1,-1]           (IF M1/M8 are the backwards-wired pair)
s_net = -(w · effective_dir)
      = -(w · [+1]*8) = [+1,-1,-1,-1,-1,-1,-1,+1]      NON-UNIFORM
```

M1/M8 out of step with M2–M7 — the **2026-08-06 yaw-spin fault**, pushed through the yaw
column `[+1,-1,-1,+1]` it collapses to three thrusters pushing the same way.

**But this is NOT a claim that the vehicle is broken.** It depends entirely on `w`, the
true wiring, and **parameters cannot tell us `w`**. If all eight are in fact wired correctly
(`w = [+1]×8`), then effective `[+1]×8` gives `s_net = [-1]×8` — uniform, and fine. Both
readings of the same numbers are self-consistent. **Only physical thrust separates them.**

### How to settle it

Either is sufficient, and one of them **must** run before any armed manoeuvre:

1. **Per-thruster physical check**, props on, hull restrained, one motor at a time — confirm
   each thruster pushes the direction `docs/THRUSTER_MAP.md` says it should.
2. **In-water MOTOR_DETECT, armed, hull free to rotate** — this is gate 3 anyway. Require
   **SUCCESS**; a `FAIL` is inconclusive and writes nothing, so re-run rather than editing
   params by hand.

Until one passes, **treat every axis as unverified**.

---

## GATE 1 — MOTOR_DETECT, then `FRAME_REVERSE = 0`, in that order

Adopted in `fd701fc` (PR #6) and unchanged. Restating because gate 0 makes it live.

The detect run itself is safe — `driveTestMotor()` never reaches `mixer::mix()`, so
`FRAME_REVERSE` cannot contaminate what it measures. The problem is what detect **converges
to**: every thruster ends up agreeing with its mixer column (`s = +1` uniformly), and
`FRAME_REVERSE = 1` then negates all six demands and turns that corrected frame back into a
uniform `[-1]×8` flip.

```
1. record   MOT_1..8_DIRECTION, CAL_MDIR1..8, FRAME_REVERSE      (before arming)
2. detect   in water, armed, free to rotate -- require SUCCESS
3. set      FRAME_REVERSE = 0     and save
4. verify   MANUAL axes, then STABILIZE, then autotune
```

Wait for the `"Params saved to flash"` statustext at step 3, **not** the `COMMAND_ACK` — the
ACK fires the instant the command is parsed and the NVS write is deferred. Match on
`"Params saved"`, not `"saved"`: the board emits the *calibration* line first.

If axes are still inverted at `FRAME_REVERSE = 0` after a SUCCESS detect, the inversion is
**not** in the motor directions. Look at the attitude/gyro convention, and do not simply put
`FRAME_REVERSE` back — that hides which of the two is wrong.

---

## GATE 2 — the depth loop has still never run closed

Unchanged and still the hard blocker. It gates **every `SROT_MOVE`**
(`task_control_loop.cpp:240-241`), including `move_forward`. An in-air `move_forward` is not
partial validation.

Rev 9 makes the disarmed half honest rather than fixing this: `DEPTH_ERR`/`DEPTH_OUT` are now
**absent** while the controller is not running, instead of streaming a frozen register.

⚠ **`check_depth_loop_settled()` is now structurally dead and must be moved.** It is only
ever called while disarmed, and under rev 9 `DEPTH_OUT` is *always* absent then — so it can
never gate anything. Move it onto **`DEPTH_CMD`**, which is deliberately still live:

- Same `±1.0` clamp, so the 2026-08-02 phantom-baro case (`−3…−6.7 m` → `3.0 × −3.1`)
  still pins at **−1.00**.
- **Better than the old signal**: `preview()` is proportional-only with no integrator, so it
  reflects the *current* baro sample rather than accumulated windup. For "is the baro lying
  right now", that is the property you want.
- ⚠ **Derive the threshold from `DEPTH_P`, don't hardcode `0.90`.** The mapping is
  `|DEPTH_CMD| ≥ 0.90 ⟺ |depth − 0.10| ≥ 0.30 m` **at `DEPTH_P = 3.0`**, and it moves with
  the gain. A healthy surface reading (`depth ≈ 0.026`) gives `≈ −0.22`, comfortably clear.

---

## GATE 3 — `FRAME_REVERSE` under thrust

Still unverified. MANUAL stick direction first, STABILIZE second, autotune last. Gate 0 has
to clear before this means anything.

---

## Verified, no action needed

| item | state |
|---|---|
| `YAW_REF` | **2 = LOCKED** at `MAGACC 2` — heading is absolute, absolute `MOVE_TURN` is safe |
| `DEPTH_ERR`/`DEPTH_OUT` disarmed | **absent** (rev 9, as specified) |
| `DEPTH_CMD` | live, `−0.648` |
| `FS_GCS_COMPID` | **191**, in flash, survived the flash |
| params after flash | 232 read, **0 drift, 0 missing** |
| CFG banner | repeats ~60 s; caught gate 0 on its first run |
| wildcard warning | correctly silent (`COMPID = 191`, not `0`) |

## NOT verified — needs arming, therefore an operator decision

- `DEPTH_ERR`/`DEPTH_OUT` **reappearing** when armed in DEPTH_HOLD. Rev 9's suppression is
  confirmed; the un-suppression is not. **Props off.**
- The `companionLost()` seen-then-lost latch on hardware.

---

## Order of operations for the session

1. **Gate 0** — physical thrust check *or* in-water MOTOR_DETECT (SUCCESS).
2. **Gate 1** — `FRAME_REVERSE = 0`, saved and confirmed by statustext.
3. Armed, props off: confirm `DEPTH_ERR`/`DEPTH_OUT` reappear in DEPTH_HOLD.
4. **Gate 2** — depth hold closed-loop, in water.
5. **Gate 3** — MANUAL axes → STABILIZE → autotune.
6. Only then: `SROT_MOVE` primitives.

Nothing in steps 4–6 is meaningful until 1 and 2 have passed.
