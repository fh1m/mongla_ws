# PR S — a move that gives up reports `ACCEPTED` at 100 %

**Target:** `srot-control-board` · **File as an ISSUE**
**Status:** ⏳ NOT SENT — read-only GitHub access this session.

`MOVE_DIVE` to a depth it never reaches, and `MOVE_TURN` to a heading it never
reaches, both end by the global timeout and are reported to the companion as
`MAV_RESULT_ACCEPTED` with progress 100 — **indistinguishable from success.** These
are the two closed-loop verbs, i.e. the two a mission most needs to trust.

⚠ Distinct from their **#8** (`ACCEPTED at 100 % while still running`), which is a
timing problem. This one fires *after* the move has correctly finished — having
failed.

---

## 1. The path, three files

`movement.cpp:171` — the timeout brakes out and records nothing:

```cpp
if (s_running && s_phase != PH_BRAKE && (now - s_start_ms) > s_timeout_ms) {
    s_phase = PH_BRAKE; s_start_ms = now; s_brake_ms = brakeMs(s_cur_speed);
}
```

For DIVE and TURN `s_uf == s_ul == 0` and `s_cur_speed == 0`, so `PH_BRAKE` commands
nothing, `brake_ms` is 0, and it falls to `PH_DONE` on the next tick — the same
terminal state a successful leg reaches. `PH_DONE` clears `s_type`, so:

`task_control_loop.cpp:945` — the falling edge latches completion:

```cpp
if (mv_was_active && !now_active) g_state.control.mv_done_seq = prev_mv_seq;
```

`mav_stream.cpp:623` — and completion has exactly one outcome:

```cpp
bool done = (s.mv_done_seq == s_seq) || (s_seen_active && !s.mv_active);
if (done) {
    sendMoveAck(s, MAV_RESULT_ACCEPTED, 100);
    ...
```

Nothing anywhere carries *why* the move ended.

## 2. When this bites

- **DIVE.** Completion needs `|depth - goal| < 0.15 m` (`movement.cpp:206`). A hull
  that is buoyant, trimmed wrong, or holding station against a current never gets
  there. Note `progress()` is honest right up to the end — capped at 0.99 — and then
  the terminal ACK overwrites it with 100.
- **TURN.** Completion needs `|err| < 0.03 rad`. A saturated or failed yaw axis, or
  a current, leaves it short. ⛔ **On our hull yaw is the scarcest axis by 5.7×**
  (see [PR P](pr-p-the-allocator-they-asked-for.md) §3), so it is the most likely
  one to run out of authority.
- **Any translate leg** whose timeout is shorter than its duration.

In all of them the mission believes it is somewhere it is not, and every
subsequent leg is measured from that wrong place. On a vehicle with no position
estimate this is unrecoverable state — nothing on the board or on our side knows
what happened.

This is our own rule 6 (*never claim a verb worked without seeing the value it
produced*) from the other side of the wire, and their own `AUDIT.md` framing: a
verb that reports success while the vehicle did nothing is the failure mode that
ends competition runs.

## 3. The fix — one enum and one field

1. A terminal outcome in `movement`: `REACHED` / `TIMED_OUT` / `ABORTED` /
   `CANCELLED`, set where the phase becomes `PH_DONE` (four sites) and readable as
   `movement::outcome()`.
2. Carry it in `ControlState` beside `mv_done_seq`.
3. `mav_stream.cpp:623` sends `MAV_RESULT_ACCEPTED` only for `REACHED`;
   `MAV_RESULT_FAILED` for `TIMED_OUT`. Their §5 `NAMED_VALUE_FLOAT` path already
   exists, so `MV_STATE` can carry the outcome and an `MV_RESID` the residual error
   (`s_remain`, already maintained by `update()` for `progress()` — so the number is
   in scope at the site).

⭐ **The residual is the valuable half.** "Timed out 0.4 m short" is actionable in a
mission; "failed" is only slightly better than "succeeded".

⚠ **Wire compatibility:** we currently treat any terminal ACK as success, so
sending `FAILED` will change our behaviour — which is the point, but it wants the
same-PR update on our side. We will take `srot_protocol.py` in step; say the word
and we will land ours first so a firmware merge cannot break a running mission.

---

*Read at `movement.cpp:159-235`, `task_control_loop.cpp:930-950`,
`mav_stream.cpp:550-640`. Source-read; not bench-verified.*
