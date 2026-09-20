#!/usr/bin/env python3
"""pursue_demo -- torpedo-style approach using align() then move().

Demonstrates the competition close-and-fire pattern with the two verbs:
  1. align -- yaw-centre the target (sweep fallback finds it).
  2. move  -- drive forward-only until the target fills APPROACH_FILL % of
              the frame, then back off.

Bench/sim test:
  ros2 run mongla_planner mission pursue_demo
  Watch the [VIS] log: bbox fill should grow toward APPROACH_FILL.

Tune: lower APPROACH_FILL (e.g. 55) for a safer standoff;
      lower APPROACH_GAIN for a slower approach.

WARNING: pursues until bbox fills APPROACH_FILL OR the time budget expires.
         On real hardware the vehicle will keep moving forward.
         Have a kill switch ready.
"""

CAMERA       = 'laptop'
TARGET_CLASS = 'person'
DIVE_DEPTH_M = -0.5

ALIGN_ERR_PX     = 50
ALIGN_GAIN       = 30
APPROACH_GAIN    = 35
APPROACH_FILL    = 80     # bbox height % to stop at (80% = contact range)
ALIGN_DURATION_S = 12.0
APPROACH_S       = 20.0


def run(mongla, log):
    mongla.mission_reset()
    mongla.camera = CAMERA
    mongla.target = TARGET_CLASS

    mongla.arm()
    mongla.set_mode('ALT_HOLD')
    mongla.set_depth(DIVE_DEPTH_M, settle=1.0)

    mongla.vision.align(
        TARGET_CLASS, camera=CAMERA, yaw=0,
        err=ALIGN_ERR_PX, gain=ALIGN_GAIN, duration=ALIGN_DURATION_S,
        fallback=sweep_yaw)

    # move: forward-only, exits when APPROACH_FILL reached. Never re-centres.
    mongla.vision.move(
        TARGET_CLASS, camera=CAMERA, fwd=APPROACH_FILL, mode='height',
        gain=APPROACH_GAIN, duration=APPROACH_S, fallback=creep_forward)

    mongla.move_back(2.0, gain=50.0)
    mongla.stop()
    mongla.set_depth(0.0)
    mongla.disarm()


# ── Mission-authored fallback search patterns (pure control) ────────────────────
def creep_forward(mongla):
    mongla.move_forward(0.6, gain=30)


def sweep_yaw(mongla, should_stop):
    for _ in range(6):
        mongla.yaw_right(30.0)
        if should_stop():
            return
