# The verbs the board could give us, and has not yet

> Measured on `tools/bench/control_bench` against the live board's parameters
> (`workbench/data/board_params_20260923.json`), and read in the firmware at
> `bff7400`.

The board exposes capabilities with no verb on our side yet. Most of them have a
working MAVLink handler on one side and a driver method on the other, with
nothing joining them. A verb that needs a firmware change is a pull request,
listed in §3 with the ask that carries it.

---

## 1. Shippable today — the board handler AND the host driver already exist

| verb | what it does | what it unblocks | state |
|---|---|---|---|
| **`motor_test`** | spin ONE thruster (1–8) at a signed percentage for N s | **Key point:** thrust curve (`k_n_per_rpm2` is `null`), dead-thruster identification, motor-direction verification, and the bollard curve ask M §2.4 needs. A **signed** throttle runs it backwards, which is how `REVERSE_EFFICIENCY = 0.77` stops being a quoted number | **shipped** `f594ecf` |
| **`get_param` / `set_param`** | read/write one board parameter by MAVLink name | every tuning experiment becomes a one-liner instead of a Python session. `SrotFC.get_param` already exists; today the only way to set one is a script | pending: next |
| **`rate`** | body-rate command in **ACRO** | **Key point:** free-decay rotation identification (Round 4a → `ATC_DRAG_*`), and the command shape INDI needs. See §2 | pending: needs a host loop |
| **`style_pitch`** | N × 360° pitch spin | `MAV_CMD_USER_2` is implemented on the board and has no verb, while `style_roll` (USER_3) does. A pure symmetry gap | pending: trivial |
| **`autotune`** | board-side relay autotune | `CMD_USER_5` plus progress readback both exist. It tunes the rate PIDs *on the vehicle*, which is the loop we cannot tune from here | pending: |
| **`pattern`** | the board's multi-step pattern | `CMD_USER_4` exists | pending: low value |

---

## 2. ACRO is the real unlock, and it is not what it looks like

`attitude::acro()` (`attitude_control.cpp:117-124`) is the inner rate PID alone:
expo, scale by `MAX_ACRO_RATE` (4.0 rad/s), rate PID. **No stick gate.**
`stabilize()` gates the yaw stick at a hardcoded `0.02` and hands anything
smaller to heading hold; `acro()` does not.

Measured, live-board parameters:

| yaw stick | STABILIZE output | ACRO output |
|---|---|---|
| 1.00 % | **0.00 %** | **16.32 %** |
| 2.86 % | 17.42 % | 18.42 % |
| 100 % | 70.67 % | **84.78 %** |

So ACRO commands yaw where STABILIZE commands nothing — **2.86× finer command
resolution** — and reaches 84.8 % of full scale against 70.7 %, because
`MAX_ACRO_RATE` (4.0 rad/s) exceeds `PILOT_YAW_RATE` (2.793 rad/s).

**Warning: What it does NOT fix, and the reason it is not the answer to precision
alignment.** `MOT_SPIN_MIN` lives in the mixer, downstream of both modes: the
smallest non-zero thrust is ~16 % of full scale either way. **ACRO buys command
resolution and headroom, not precision.** And it costs heading hold and attitude
stabilisation entirely.

**Therefore:** wrong trade for a vision servo; the right tool for system
identification and for an incremental controller. A `rate` verb is an
identification instrument, and it should be documented as one rather than
offered to missions.

---

## 3. Needs firmware — filed, not guessed

| verb | the firmware change | ask |
|---|---|---|
| **`hold_heading`** | **Key point:** `attitude::holdYaw(float)` **already exists**, is declared at `attitude_control.h:30` and is already called in the flight path from `task_control_loop.cpp:238` — but only in AUTO. STABILIZE, the one mode that honours `MANUAL_CONTROL`, cannot reach it. A `MAV_CMD` and one call | [`pr-n`](../upstream/pr-n-caps-literals-and-the-heading-hold-setpoint.md) §2 |
| **a distance-closed move**, **`arc`** | these need closed-loop distance, which needs velocity ingest on the board | [`pr-g`](../upstream/pr-g-optical-flow-ingest.md) |
| **a per-thruster demand** | the mixer takes six body axes. `motor_test` reaches ONE motor at a time through a keep-alive, which is an instrument, not an actuator. A real allocator needs eight simultaneous per-thruster demands | [`pr-k`](../upstream/pr-k-the-mixer-is-for-a-different-hull.md) |

---

## 4. The rule this file exists to enforce

**Warning: A verb is not shipped because it is possible. It is shipped because a
measurement or a mission needs it.** `pattern` is listed above and is bottom of
the list precisely because nothing needs it.

And the harder rule, from `CLAUDE.md` §8 rule 6: **never claim a verb worked without
seeing the value it produced.** Every verb here that reaches the board must
return what the board did, not what we asked it to do.
