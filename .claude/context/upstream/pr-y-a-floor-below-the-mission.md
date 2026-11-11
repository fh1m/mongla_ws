# Ask: a floor below the mission, enforced on the board

From: `mongla_ws` (companion, MAVLink compid 191)
Date: 2026-11-10
Status: opened 2026-11-10 as [issue #77](https://github.com/RakibulIslam1/srot-control-board/issues/77).
Firmware read: `main` at `f1d3ba9`, behaviour rev 14

The board is the one part of the vehicle that keeps running when the companion
is wrong. It already acts on that: a silent link, a leak and a low thruster
pack each force SURFACE (`task_control_loop.cpp:616`). Each of those is a
failure the board can see for itself. This asks for the same treatment of
three failures only the board can stop, because in each one the companion is
alive, heartbeating and commanding.

## 1. `FS_DEPTH_MAX`: past this depth, SURFACE

### What we read

`include/config.h` has no maximum-depth parameter. `FS_SURFACE_DEPTH_M`
(`config.h:647`) is the other end, "at the surface". The only depth ceiling in
the tree is the autotuner's amplitude (`autotune.cpp:99`). A `MOVE_DIVE` to any
depth is accepted, and DEPTH_HOLD latches whatever depth the hull is at.

### Why it matters

A mission with a sign error or a units error (centimetres read as metres)
commands a dive with nothing under it but the pool floor. Our host refuses a
target past 10 m and, from today, surfaces a hull that passes a floor the deck
sets (`shield_max_depth_m`, `node_support.depth_shield`). Both live in the
process that made the mistake.

### What we ask for

A parameter `FS_DEPTH_MAX` (metres, positive down, 0 = off). When the measured
depth has been past it for `FS_DEPTH_HOLD_MS` (we use 1000 on the host, so one
bad barometer sample cannot trip it), force SURFACE through the same path the
leak failsafe takes, with a STATUSTEXT naming it. It should not apply while the
barometer is unhealthy; that case already refuses the depth modes.

## 2. `FS_AIDED_MS`: no AUTO on a position the estimator has disowned

### What we read

`mav_commands.cpp:77-84` tracks two link ages, the GCS heartbeat and the
companion heartbeat. Both answer "is the process alive". Nothing answers "does
the process still know where the hull is".

### Why it matters

Our estimator publishes `aided` or `unaided`. Unaided means velocity is not
observed and horizontal position is dead reckoning; we measured 635 m of drift
in 95 s in that state. The heartbeat keeps ticking through it, so a mission
steering on that position passes every check the board has.

### What we ask for

1. Accept `NAMED_VALUE_INT` with name `AIDED` and value 1 or 0 from compid 191.
   We will send it at 2 Hz. We chose a named value because it needs no new
   message id; a bit in the companion heartbeat's `custom_mode` would do as
   well, and the choice is yours.
2. A parameter `FS_AIDED_MS` (0 = off, the default). When it is set and no
   `AIDED = 1` has arrived for that long, a move that needs position is
   refused, and one in flight ends in DEPTH_HOLD.

Off by default matters to us too: our first swims are control-only, with no
estimator running, and must not be refused.

Our side fails closed from today: an estimator whose observability gate cannot
be read reports `unaided` (`localization_inputs._report_aiding`). The host does
not send `AIDED` yet, because nothing reads it.

## 3. A yaw-rate envelope: sustained rotation nobody asked for ends the move

Behaviour rev 10 inverted yaw. A board and a companion that disagree about the
sign turn a heading hold into positive feedback, and the hull spins up to the
rate limit and stays there. A parameter `FS_YAWRATE_MAX` (deg/s) with a hold
time, tripping to a stop and STABILIZE when the gyro rate exceeds it while the
commanded rate is below it, catches that class without knowing its cause.

## 4. Optional: `FS_AUTO_MAX_MS`

`AUTO` never exits by itself. A longest time in AUTO with no new `SROT_MOVE`
bounds a companion that is alive, heartbeating and stuck.

## Where this belongs

The round-2 review proposed one failsafe evaluator that runs after every mode
and tuner decision (#66 and #67 on this repo both point at it). These
four are inputs to it. If that evaluator is not planned, each of these is one
more block beside the leak and battery checks at `task_control_loop.cpp:616`.

## How to verify, disarmed on the bench

1. Spoof depth past `FS_DEPTH_MAX` through the Bar30 test hook: the mode
   becomes SURFACE and the STATUSTEXT names the depth.
2. Set `FS_AIDED_MS`, send `AIDED = 1` at 2 Hz, then stop: a move is refused
   after the window and accepted again when the value returns.
3. With `FS_AIDED_MS = 0`, moves are accepted with no `AIDED` ever sent.

Each parameter needs a `SROT_FW_BEHAVIOUR_REV` bump only if its default is on.
We ask for defaults of off, and will set them from the deck.

## What we do on our side when it lands

`bringup_check` reads the four parameters and refuses a run whose depth floor
is unset; the manager sends `AIDED`; `shield_max_depth_m` stays as the second
copy of the floor, compared against the board's at connect.
