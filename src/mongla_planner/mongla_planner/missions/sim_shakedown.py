#!/usr/bin/env python3
"""sim_shakedown -- prove the sim <-> autonomy loop end to end.

arm -> hold depth -> forward N s -> settle -> back N s -> settle -> surface -> disarm.

The two legs are deliberately SYMMETRIC: equal duration, equal thrust, opposite
sign. On an open-loop timed verb that is the whole return-to-origin mechanism --
there is no position feedback anywhere in this mission, by design, because the
same file has to run unchanged on the real vehicle where no ground truth exists.
So it returns *approximately* to origin and the residual is the number worth
reading. In the simulator you can measure it exactly:

    ros2 topic echo /mongla/sim/ground_truth --once     # before and after

Expect a residual, not a zero. Water inertia means the hull is still moving when
a timed leg ends, and the two legs do not coast identically -- drag is not
symmetric fore/aft on this frame.

DEPTH
-----
`DEPTH_M` is NEGATIVE (below surface), matching set_depth's convention.

The SAUVC pool modelled in mongla_sim_worlds is **1.6 m deep** -- water surface
at z = 0, floor at z = -1.6 (`spec/arena.yaml: depth: 1.6`, and all three worlds
place pool_floor at z = -1.6). A target below that is unreachable: the hull sits
on the bottom, ArduSub keeps commanding down, and set_depth burns its whole
timeout without ever converging. -1.2 leaves ~0.4 m of floor clearance while
staying well submerged.

Override per run without editing this file:

    MONGLA_SHAKEDOWN_DEPTH=-0.8 ros2 run mongla_planner mission sim_shakedown

Run: ros2 run mongla_planner mission sim_shakedown
"""
import os

DEPTH_M      = float(os.environ.get('MONGLA_SHAKEDOWN_DEPTH', -1.2))
LEG_S        = float(os.environ.get('MONGLA_SHAKEDOWN_LEG_S', 5.0))
LEG_GAIN_PCT = float(os.environ.get('MONGLA_SHAKEDOWN_GAIN', 55.0))
SETTLE_S     = 4.0     # coast-down between legs; see note above about inertia
DEPTH_BUDGET_S = 45.0

# Pool geometry from mongla_sim_worlds/spec/arena.yaml. Advisory only -- the real
# vehicle is not in this pool, so this warns and continues rather than refusing.
SIM_POOL_FLOOR_M = -1.6


def run(mongla, log):
    if DEPTH_M > 0:
        raise ValueError(
            f'DEPTH_M={DEPTH_M} is positive. set_depth takes NEGATIVE metres '
            f'below the surface; {-abs(DEPTH_M)} is what you meant.'
        )
    if DEPTH_M < SIM_POOL_FLOOR_M:
        log(f'WARNING: depth {DEPTH_M:.2f} m is BELOW the simulated pool floor '
            f'({SIM_POOL_FLOOR_M} m). In sim the hull will bottom out and '
            f'set_depth will burn its full {DEPTH_BUDGET_S:.0f}s budget without '
            f'converging. Fine on a real vehicle in deeper water.')

    # mission_reset: clears any abort latched by a previous run, stops heading
    # lock, sends RC neutral, and re-zeroes the barometer while still disarmed.
    mongla.mission_reset()
    mongla.arm()

    # set_depth engages ALT_HOLD and streams the setpoint. Depth is ArduSub's
    # loop, not ours -- we only supply the target and watch AHRS depth.
    log(f'descending to {DEPTH_M:.2f} m')
    mongla.set_depth(DEPTH_M, timeout=DEPTH_BUDGET_S)

    log(f'leg 1/2: forward {LEG_S:.0f}s @ {LEG_GAIN_PCT:.0f}%')
    mongla.move_forward(LEG_S, gain=LEG_GAIN_PCT)

    # Settle before reversing. Without it the hull is still carrying forward
    # momentum when the reverse leg starts, so the return leg spends part of its
    # thrust cancelling that instead of retracing, and lands long.
    mongla.pause(SETTLE_S)

    log(f'leg 2/2: back {LEG_S:.0f}s @ {LEG_GAIN_PCT:.0f}% (retracing leg 1)')
    mongla.move_back(LEG_S, gain=LEG_GAIN_PCT)
    mongla.pause(SETTLE_S)

    # surface() is correct on hardware and KNOWN-DIVERGENT in the simulator, so
    # it must not decide whether this mission passed.
    #
    # Depth telemetry is read from MAVLink AHRS2.altitude (the secondary DCM
    # estimate). Measured in sim at a true depth of -0.036 m:
    #
    #     AHRS2.altitude .................. -0.370   <- what we read
    #     GLOBAL_POSITION_INT.relative_alt  -0.028   <- EKF3, matches truth
    #
    # ArduSub closes its depth loop on EKF3, so the hull DOES reach the commanded
    # depth (measured within 2.5 cm at -0.5 and -1.0 m). But surface() waits for
    # the number WE can see to reach 0.00, and AHRS2 plateaus around -0.4, so it
    # burns its full ascent budget after the vehicle has physically surfaced.
    # The same offset trips mission_reset's calibrate_depth guard
    # (|-0.36| > 0.30 m surface bound), so the re-zero that would fix it is
    # refused -- in sim those two interlock.
    #
    # Not "fixed" by switching the depth source: AHRS2 is the pool-verified
    # hardware path. Caught here, named, and reported instead.
    log('surfacing')
    try:
        mongla.surface()
    except Exception as exc:
        log(f'WARNING: surface() did not confirm ({exc}). Expected in SIM -- see '
            f'the AHRS2/EKF3 note in this file. On HARDWARE treat it as real: '
            f'check the Bar30 and re-run calibrate_depth at the surface.')

    mongla.disarm()
    log('sim_shakedown complete')
