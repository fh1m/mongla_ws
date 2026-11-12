# PR W — the board triggers the eyes

**To:** the firmware team (sections 1 to 4) and the hardware team (section 5).
**Status:** opened 2026-11-10 as
[issue #44](https://github.com/RakibulIslam1/srot-control-board/issues/44). Read against `srot-control-board`
`origin/main` `f1d3ba9` (2026-08-08).

## The problem

The board knows when it moved the hull: its IMU and its control loop run on its
own clock at 500 Hz. The camera does not share that clock. Today each frame is
stamped on Kutum, when it arrives, by a webcam that decides its own exposure and
its own rate. Everything we steer on is a pairing of a frame with a body state,
and that pairing is a guess.

What the forward camera does today (Kutum, 2026-11-10, raw `v4l2-ctl
--stream-mmap`, no OpenCV, no ROS):

| what was asked | what it did |
|---|---|
| 30 or 60 fps at MJPG 640×360 | **14.97-15.06 fps**; the driver reports 30/1 back |
| manual exposure 5, 30, 100 ms | mean brightness **124.5-124.8** at every setting: the control is accepted, read back, and ignored |
| auto exposure | a **200 ms** shutter in a lit room; at 0.64 rad/s of yaw that smears **66 px** |

It is a rolling-shutter webcam, so a turning hull also skews every frame. None
of this can be fixed in software, and we have the trace to show it
(`measured-bars.md`, "Forward camera rate"; `camera-and-calibration.md` §6).

## 1. A trigger output on the ESP32

`GPIO13` and `GPIO14` are free in the default `THRUSTER_PICO` backend
(`config.h:104`: *"GPIO 0/12/13/14/15 are free for expansion sensors"*), and
unlike 0, 12 and 15 neither is a strapping pin. They are DShot outputs only in
the RMT backend, which our hull does not run.

- **`CAM_TRIG` on GPIO13**: a hardware-timed pulse train, from LEDC or MCPWM,
  not from a task loop, so its jitter is the timer's and not the scheduler's.
- **Phase-locked to the control tick.** Make the trigger period an integer
  number of 2 ms ticks (for example 50 Hz = every 10th tick), and fire it on the
  tick, so every frame lines up with one control step.
- **Parameters:** `CAM_TRIG_HZ` (0 = off, the default, so nothing changes for a
  board without a camera on the pin), `CAM_TRIG_US` (pulse width; some sensors
  read exposure from it), `CAM_TRIG_POL`.
- **`CAM_STROBE` on GPIO14, optional**: an input from the sensor's strobe
  output, edge-captured, which measures when the exposure actually happened
  rather than when it was asked for.

## 2. Say on the wire when each frame was triggered

MAVLink already has the message: `CAMERA_TRIGGER` (id 112), `time_usec` +
`seq`, in the library the board builds against.

- Send one per trigger, with `time_usec` taken from the **board's** clock at the
  edge (from the strobe edge when `CAM_STROBE` is wired) and `seq` counting
  triggers since boot.
- Cost: 24 bytes per message with MAVLink 2 framing. At 30 Hz that is 720 B/s,
  **6.25 %** of 115200 baud; at 60 Hz, 12.5 %. If that is too much, send it
  every Nth trigger with `seq`, and the host fills the gaps from the fixed
  period (the timer is crystal-timed).
- Without `TIMESYNC` (PR D) the host cannot map board time onto its own clock.
  This ask makes PR D's ten lines worth more.

## 3. `SROT_SERVO_FUNC_CAMERA` has no caller

`config.h:315` defines `SROT_SERVO_FUNC_CAMERA 5 // camera tilt / trigger`, and
nothing reads it. Servo functions drive the PCA9685 over I2C, which cannot time
a trigger to the microsecond. Please either make that function mean "this
channel is the camera trigger" and route it to section 1's timer, or remove
`/ trigger` from the comment so nobody wires a camera to a PCA9685 channel
expecting it to work.

## 4. Or on the Pico

The RP2350 has three PIO blocks, and `src/pico/main.cpp` uses two of them for
DShot (ESC1..8 on GP6..GP13). A PIO state machine on the third block can
generate the trigger with cycle accuracy and timestamp it, and it is free of
everything the ESP32 runs. The cost is one more field on the ESP32-Pico link, so
we would take section 1 first. This is listed so the choice is yours.

## 5. To the hardware team: the sensor

A forward camera that can be triggered:

- **Global shutter**, OV9281 class (1 MP, monochrome is fine for detection
  after retraining and better in low light), at least 60 fps at 640×400 or more.
- **An external trigger input at 3.3 V**, or at 1.8 V with a level shifter on
  the harness, and **a strobe output** if the module has one.
- **Exposure that obeys a command**: the module's manual exposure must change
  the image. The Fantech's does not.
- USB UVC or CSI-2. Raspberry Pi documents an external trigger for its own
  Global Shutter Camera (IMX296, CSI-2) at 1.8 V, which needs a board
  modification. Check the current documentation before ordering either.
- Two wires to the SROT board: trigger and strobe, plus ground, routed away
  from the thruster power.

## What mongla_ws does now

- `fwd_profile:=pi_forward_global_shutter` runs the Sonix global-shutter unit
  (measured MJPG 640×360 @ 210 fps) on the forward port, and udev names it
  `/dev/mongla_cam_forward` there. That gives 14× the frames and no skew, but
  it is still free-running and stamped on arrival.
- When section 2 lands, the host side is: `SrotFC` decodes `CAMERA_TRIGGER`
  (`time_usec`, `seq`) in the reader thread, and `camera_node` takes each
  frame's capture stamp from the nearest trigger in board time, mapped through
  `TIMESYNC`, instead of from arrival. The consumer is the capture stamp every
  freshness gate already reads (`camera_node._make_header`). The decode and
  that one call are not written, because a decoder for a message the board
  does not send would be a capability that nothing can reach.
- The part that could be settled without the board is settled:
  `mongla_vision/trigger_stamps.py` pairs each frame with the pulse that
  exposed it. A pulse is a frame's pulse when it lies in the camera's measured
  latency window before the arrival; a window narrower than the trigger
  period holds at most one, so the pairing is exact and learns nothing. On
  synthetic streams with 5 % of frames and 10 % of trigger messages lost, at
  50, 60 and 210 Hz, it named the wrong pulse **0 times**, stamped at least
  99 % of frames, and timed a pulse the wire did not deliver to within
  100 µs. A board that reports only every fourth pulse (section 2's
  link-budget option) is enough. It refuses, with the reason counted, when
  the window is as wide as the period, when the link has been silent for
  0.5 s, and across a board reboot.
- **What that asks of the bench when the camera arrives:** the latency window
  per camera and per mode, measured with a light the board fires on a known
  pulse. At 210 Hz it has to be known to better than 4.8 ms.
