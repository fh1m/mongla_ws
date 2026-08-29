# Flying the AUV with a gamepad

For dataset collection and for feeling out how the vehicle actually moves, a
stick beats typing verbs. This works the way QGC and ArduSub's own joystick
page do: the stick is read where the operator is, mapped to RC channels, and
streamed to the vehicle.

## Where the pad plugs in — both are supported

| Pad is on… | Read by | Use when |
|---|---|---|
| the machine running the **browser** | the lab UI, via the Gamepad API | the lab is on another machine (the QGC arrangement) |
| the machine running the **lab** | `duburi_sim_web/joystick.py` | lowest latency, and no browser needed at all |

Both drive the **same `TeleopStreamer`**, so there is never a second RC writer.
That streamer connects on **tcp:5763** — the port that exists precisely so
teleop never fights the manager's `udpin:14550`.

```bash
ros2 run duburi_sim_bringup duburi_sim sim
ros2 run duburi_sim_bringup duburi_sim stack --no-vision
ros2 run duburi_sim_bringup duburi_sim lab        # open the printed URL
#   [JOY  ] Logitech Gamepad F310 on /dev/input/js0

# pin a specific device instead of auto-detect:
DUBURI_JOYSTICK=/dev/input/js1 ros2 run duburi_sim_bringup duburi_sim lab

# terminal status, no browser:
ros2 run duburi_sim_bringup duburi_sim joystick
#   ACTIVE Logitech Gamepad F310  fwd----##|-------- lat--------|-------- … gain 0.55
```

The lab UI shows a **CONTROLLER ACTIVE** panel with the pad's name, which side
it is plugged into, and live axis bars.

## Mapping

Verified against a real Logitech F310 in XInput mode.

| Control | Action |
|---|---|
| Left stick | forward / strafe |
| Right stick | yaw / vertical |
| A | arm |
| B | disarm |
| LB / RB | gain down / up (0.10 – 1.00) |

Gain is a first-class control for the same reason QGC exposes it: one fixed
stick scale is either too coarse for alignment or too slow for transit.

**Axis and button numbers are NOT universal.** The kernel numbers a pad's
controls in the order it declares them, so a pad without triggers shifts every
later index — the F310 puts the right stick on axes 3/4 because the triggers
take 2/5. That is why the map is a parameter and why `/api/vehicle/joystick`
reports raw indices. It is also why QGC ships a joystick calibration page.

## Feel: deadzone and expo

- **Deadzone 0.08.** Sticks do not return to exactly zero; without it the
  vehicle creeps. Past the deadzone the axis is rescaled from 0, so leaving
  centre is smooth rather than a step to 8 % authority.
- **Expo 0.35.** Half stick gives 0.33 of authority, full stick still gives
  1.0. Small movements stay gentle, which is what makes close-in alignment
  possible; the top of the range is untouched.

Both are identical in the browser and the device reader, so a pad flies the
same wherever it is plugged in.

## The bug that made it twitchy, and the measurement that found it

**A held stick produces no events.** The kernel reports *changes*, so holding
full forward emits one event and then silence. The first version pushed to the
streamer only when an event arrived — and `TeleopStreamer` has a 0.35 s
watchdog that centres the vehicle when updates stop, which is right for a
dropped UI and wrong for a stick that is simply steady.

Measured, 6 s of full forward held from a standing start:

| | distance |
|---|---|
| push on event only | **0.188 m** |
| push at a fixed 50 Hz | **1.220 m** |

The reader now pushes every tick regardless of events. That 6.5× is the whole
difference between "twitchy" and "stable", and it is invisible in a log.

## Verification

The reader was tested against a **real virtual gamepad** created through
`/dev/uinput`, so the kernel's own joystick layer is in the loop rather than a
mocked file: all four axes with correct signs and inversion, deadzone, expo,
gain buttons, and the end-to-end path pad → reader → RC → the hull moving in
Gazebo.

One trap that cost a round: the kernel replays every control's **current
state** at open, with `JS_EVENT_INIT` set. Those axis values are real and must
be applied, but a button reported as held in that burst is a state report, not
an operator action — firing `arm` from it would arm the vehicle the moment the
pad is plugged in. A test asserts it does not.
