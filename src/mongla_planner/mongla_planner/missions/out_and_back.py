"""out_and_back -- dive, swim straight, turn clockwise, swim back. Control only.

Nothing here looks through a camera or estimates a position: every move is one
board verb, timed and braked on the board, with the board holding depth and
heading at 500 Hz. So the run depends on four things only -- the board, Kutum,
the thrusters and the kill switch -- and a vision fault cannot end it.

    dive -> forward OUT_S -> clockwise turn -> forward BACK_S -> zero-translation-demand HOLD

At the end, HOLD sends zero commanded surge/sway, while the board holds
depth and heading; it is NOT XY position-hold and the hull can drift in a
current. It is also NOT another blind forward leg. An additional 20 s of open-loop translation after the return leg could
hit a pool wall. The hold gives the diver an underwater kill-switch window;
its actual thruster activity depends on buoyancy and the depth/attitude error.
The physical kill cuts thruster power. If it is operated underwater, SURFACE
cannot propel the AUV until power is deliberately and safely restored; a
diver must recover the vehicle. This is a VIDEO demonstration, not proof of a
fully autonomous post-kill surfacing manoeuvre. Do not restore power on an
armed board just to make SURFACE work (see kill-latch audit).

The turn is `yaw_right`: clockwise seen from above, which is the board's
positive yaw. `TURN = 'square'` swaps the in-place half turn for two right
angles with a short leg between them, a wider U.

Run it inside tmux, on the vehicle, with the hull pointing down the lane:

    tmux new -s run
    ros2 launch mongla_manager bringup.launch.py        # pane 1
    ros2 run mongla_planner mission out_and_back        # pane 2

The countdown refuses on a bare ssh session: pulling the tether would kill the
mission, and the manager would surface the vehicle.

BUILT, NEVER FLOWN. Every duration and gain below is unmeasured; the first wet
session fills them in (straight leg timed against a tape, turn checked on video).
Every move is a SROT_MOVE, so the two armed depth checks (CLAUDE.md section 2) gate it.
"""

TETHER_S      = 15       # countdown before arming, on the host
DEPTH_M       = -0.8     # negative is below the surface
DEPTH_TIMEOUT = 30.0
OUT_S         = 10.0     # forward leg, seconds of thrust
BACK_S        = 10.0     # the same leg home; make it equal OUT_S unless measured
GAIN          = 50.0     # percent forward demand on both legs
TURN          = 'yaw'    # 'yaw' = half turn in place, 'square' = two right angles
TURN_DEG      = 180.0
TURN_TIMEOUT  = 30.0
SQUARE_SIDE_S = 3.0      # the short leg between the two right angles
DIVE_HOLD_S   = 1.5      # explicit depth/heading HOLD after the dive
TURN_HOLD_S   = 1.5      # explicit hold after both 90-degree clockwise legs
KILL_WINDOW_S = 10.0     # depth/heading hold with zero translation demand for underwater kill shot


def run(mongla, log=None):
    mongla.mission_reset()
    try:
        mongla.countdown(TETHER_S)
        mongla.arm()
        mongla.set_depth(DEPTH_M, timeout=DEPTH_TIMEOUT)
        # The board DIVE terminal ACK only proves the depth entered its error
        # band, not that vertical velocity settled. `settle=` is the same board
        # HOLD since #86; it is spelled out here so the hold is visible in the
        # mission and in its test.
        mongla.pause(DIVE_HOLD_S)
        mongla.move_forward(OUT_S, gain=GAIN)
        _turn_round(mongla)
        # Two 90-degree turns avoid the ±180 ambiguity. Their ACKs do not
        # guarantee hydrodynamic settling; keep heading/depth neutral before
        # translating back toward the gate.
        mongla.pause(TURN_HOLD_S)
        mongla.move_forward(BACK_S, gain=GAIN)
        if log:
            log(f'[OUT  ] back leg done -- holding depth/heading {KILL_WINDOW_S:.0f}s for kill shot')
        mongla.pause(KILL_WINDOW_S)
    finally:
        # SURFACE is the last requested motion. Never send STOP afterwards:
        # SROT_MOVE would re-enter AUTO, undoing an operator-requested SURFACE.
        # An unplugged or faulty board may make SURFACE raise; still ATTEMPT
        # disarm. If the physical kill already cut power, SURFACE cannot thrust;
        # recover by diver, never by blindly re-enabling an armed thruster pack.
        try:
            mongla.surface()
        finally:
            mongla.disarm()


def _turn_round(mongla):
    if TURN == 'yaw':
        mongla.yaw_right(TURN_DEG, timeout=TURN_TIMEOUT)
    elif TURN == 'square':
        mongla.yaw_right(TURN_DEG / 2, timeout=TURN_TIMEOUT)
        mongla.move_forward(SQUARE_SIDE_S, gain=GAIN)
        mongla.yaw_right(TURN_DEG / 2, timeout=TURN_TIMEOUT)   # TURN_HOLD_S follows
    else:
        raise ValueError(f"TURN must be 'yaw' or 'square', got {TURN!r}")
