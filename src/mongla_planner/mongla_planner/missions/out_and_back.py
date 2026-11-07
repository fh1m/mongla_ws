"""out_and_back -- dive, swim straight, turn clockwise, swim back. Control only.

Nothing here looks through a camera or estimates a position: every move is one
board verb, timed and braked on the board, with the board holding depth and
heading at 500 Hz. So the run depends on four things only -- the board, Kutum,
the thrusters and the kill switch -- and a vision fault cannot end it.

    dive -> forward OUT_S -> clockwise turn -> forward BACK_S -> hold, thrusting

The last leg is a slow forward HOLD_S long, so the thrusters are visibly
running when a diver turns the kill switch; the mission then surfaces and
disarms. If the kill comes first, the manager aborts the mission and disarms
(`node_health._kill_watch_tick`), so turning the knob back does not resume a
move.

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
SETTLE_S      = 1.0      # let the hull coast still after each turn
HOLD_S        = 20.0     # slow forward at the end, for the kill-switch shot
HOLD_GAIN     = 25.0


def run(mongla, log=None):
    mongla.mission_reset()
    try:
        mongla.countdown(TETHER_S)
        mongla.arm()
        mongla.set_depth(DEPTH_M, timeout=DEPTH_TIMEOUT)
        mongla.move_forward(OUT_S, gain=GAIN)
        _turn_round(mongla)
        mongla.move_forward(BACK_S, gain=GAIN)
        if log:
            log(f'[OUT  ] back leg done -- holding {HOLD_S:.0f}s for the kill switch')
        mongla.move_forward(HOLD_S, gain=HOLD_GAIN)
    finally:
        # surface() is the last motion: any board verb after it re-enters AUTO
        # and holds the hull at depth.
        mongla.surface()
        mongla.disarm()


def _turn_round(mongla):
    if TURN == 'yaw':
        mongla.yaw_right(TURN_DEG, timeout=TURN_TIMEOUT, settle=SETTLE_S)
    elif TURN == 'square':
        mongla.yaw_right(TURN_DEG / 2, timeout=TURN_TIMEOUT, settle=SETTLE_S)
        mongla.move_forward(SQUARE_SIDE_S, gain=GAIN)
        mongla.yaw_right(TURN_DEG / 2, timeout=TURN_TIMEOUT, settle=SETTLE_S)
    else:
        raise ValueError(f"TURN must be 'yaw' or 'square', got {TURN!r}")
