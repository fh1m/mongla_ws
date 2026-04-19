# Axis isolation -- first principles, sharp turns, curved turns

> Companion to [`heading-lock.md`](./heading-lock.md) and
> [`proven-patterns.md`](./proven-patterns.md).

## 1. The problem in one sentence

A four-thruster vectored AUV cannot move purely in one axis: every
forward push injects a parasitic yaw moment, every yaw rotation
injects a small lateral surge, and every depth correction reduces
horizontal speed. If we don't actively pin the axes we don't care
about, "drive forward 5 m" turns into "drive forward, drift right,
yaw 12 degrees, sink 30 cm".

## 2. Where the cross-coupling comes from

| Source                              | Symptom                              | Why                                                                 |
| ----------------------------------- | ------------------------------------ | ------------------------------------------------------------------- |
| Asymmetric thruster offset from CG  | move_forward -> slow yaw drift       | T200 mounts are never *exactly* on the body axis through CG.        |
| Magnetic interference from ESCs     | ArduSub compass yaw wanders          | ESC current peaks at 10A; field swamps the geomagnetic vector.       |
| Drag coefficient mismatch left/right | move_forward -> small lateral skew  | Hull plates / payload offsets break left-right symmetry.            |
| Buoyancy != weight                  | move_lateral -> rises or sinks       | Trim weights only zero this in calm water at one operating depth.   |
| Inertia from previous command       | move_forward immediately after yaw   | Body is still rotating when the new RC override arrives.            |

The first three are physical realities of the vehicle. The last two
are *control-stack* problems we can solve in code.

## 3. Newton's 1st law applied to thrusters

A thruster running at 1500 us PWM produces zero force. A thruster
running at 1700 us produces (roughly) constant force *while the PWM
is held*. The instant we drop back to 1500, the body is still moving
-- water drag is the only thing that brakes it. For a Duburi-class
hull at 1 m/s forward, that's a stop distance of ~0.4 m and a settle
time of ~1.2 s. Failing to wait that out before commanding a yaw =
yawing while still drifting forward = arc, not a sharp turn.

Two implications:

1. **Constant-gain commands need an active brake.**
   `drive_forward_constant`, `drive_lateral_constant`. They exit at
   full velocity; the per-axis module fires a 200 ms reverse kick
   (`REVERSE_KICK_PCT` = 25%) before the settle.
2. **Eased commands brake themselves.** `drive_forward_eased`,
   `drive_lateral_eased`. Trapezoid_ramp drops thrust to zero over
   `EASE_SECONDS` = 0.4 s on the way out -- by the time the loop
   exits, the body's already decelerating.

Both then settle for `SETTLE_SEC` = 1.2 s before returning so the
next command starts from a known state.

## 4. The four mechanisms at our disposal

| Mechanism            | What it does                                         | When to use                                  |
| -------------------- | ---------------------------------------------------- | -------------------------------------------- |
| `stop`               | 1500 PWM on every channel (active hold).              | Between commands in a script.                |
| `pause(d)`           | 65535 (NO_OVERRIDE) for `d` seconds.                  | Stabilisation between mode changes.          |
| Per-command `settle` | Extra `n` seconds of neutral after the axis brake.    | Tune one command's exit without scripting.   |
| `lock_heading`       | Background SET_ATTITUDE_TARGET stream.                | Multi-command sequence at a fixed heading.   |

`stop` and `pause` look similar but differ in whose loop is running:

* `stop` keeps us on the loop -- 1500 us tells ArduSub "pilot wants
  zero motion". The autopilot's heading-hold integrator latches at
  the current heading and resists drift. *We* are the pilot.
* `pause` releases us -- 65535 tells ArduSub "no pilot input". The
  autopilot's *own* automation runs (ALT_HOLD just sits, MANUAL
  drifts). Use this when you want ArduSub to take over briefly
  -- e.g. to drain a stale ALT_HOLD integrator after a mode change.

## 5. Sharp vs. curved turns

### 5a. Sharp turn -- pivot in place

```python
client.move_forward(duration=5, gain=60)
client.pause(duration=1.5)            # let momentum bleed
client.yaw_right(target=90)           # pivot
client.pause(duration=1.0)            # let body settle
client.move_forward(duration=5, gain=60)
```

Each verb runs to completion under the action server's single-flight
lock. The `pause` between move and yaw is the inertia-bleed window;
without it the yaw sees forward velocity as a yaw-rate disturbance
and the heading-hold integrator winds up.

### 5b. Curved turn -- car-style arc

```python
client.arc(duration=4, gain=50, yaw_rate_pct=30)   # right-hand arc
client.arc(duration=4, gain=50, yaw_rate_pct=-30)  # then left-hand arc
```

`arc` writes Ch5 (forward) and Ch4 (yaw) in *the same* RC override
packet at 20 Hz. The body never settles between them -- which is
exactly what we want for a smooth curved trajectory. Trade-off: yaw
rate is open-loop, so a 4-second arc at +30% yaw stick covers
roughly N degrees but the precise heading at exit depends on payload
mass and water current. Use a `lock_heading` -> straight-line ->
`unlock_heading` envelope around an arc if you need a known final
heading.

## 6. Cookbook: per-command settle

`settle` is a per-command knob in `Move.action`. Useful when one
specific verb in an otherwise tight sequence needs more time:

```python
# Move + yaw + move, but the yaw needs an extra 1 s settle because
# the heading-hold integrator is sluggish at this depth.
client.move_forward(duration=4, gain=60, settle=0.0)   # default brake
client.yaw_right(target=90, settle=1.0)                # +1s extra
client.move_forward(duration=4, gain=60)
```

Equivalent without `settle`:

```python
client.move_forward(duration=4, gain=60)
client.yaw_right(target=90)
client.pause(duration=1.0)        # 1 s of NO_OVERRIDE
client.move_forward(duration=4, gain=60)
```

Use `settle` for inline tuning, `pause` for a deliberate "release the
override" pause.

## 7. Heading lock -- the meta-axis pin

When a multi-command sequence has to keep the same heading, layer a
`lock_heading` on top:

```python
client.lock_heading(target=0.0, timeout=120)   # 0 = lock current
client.move_forward(duration=3, gain=50)       # body weather-cocks
client.move_left(duration=3, gain=50)          #   ...lock corrects
client.move_forward(duration=3, gain=50)       #   ...lock corrects
client.unlock_heading()
```

While the lock is active, motion commands switch from
`send_rc_override` to `send_rc_translation` (Ch4 stays released,
SET_ATTITUDE_TARGET wins). See [`heading-lock.md`](./heading-lock.md)
for the full design and failure modes.

## 8. File map (clean code, one axis per file)

| File                                      | Responsibility                          |
| ----------------------------------------- | --------------------------------------- |
| `motion_writers.py`                       | Constants, `Writers`, `thrust_loop`, brake/settle. |
| `motion_forward.py`                       | Ch5 forward/back, plus `arc` (Ch5 + Ch4). |
| `motion_lateral.py`                       | Ch6 strafe.                             |
| `motion_yaw.py`                           | Ch4 yaw via SET_ATTITUDE_TARGET.        |
| `motion_depth.py`                         | Ch3 depth via SET_POSITION_TARGET.      |
| `heading_lock.py`                         | Background SET_ATTITUDE_TARGET streamer. |
| `duburi.py`                               | High-level facade: serialises commands, picks the right axis module. |

The split makes the call sites unambiguous: `move_forward` ->
`drive_forward_*(+1, ...)`, `move_back` -> `drive_forward_*(-1, ...)`,
`move_left` -> `drive_lateral_*(-1, ...)`, `move_right` ->
`drive_lateral_*(+1, ...)`. No more "linear" doing both forward and
lateral in the same module.
