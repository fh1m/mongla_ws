"""SAUVC Qualification — through the hanging gate, U-turn, back through it.

    "To qualify, an AUV must swim from the qualification starting line, pass
     completely through the qualification gate, perform a U-turn, and
     completely pass through the gate again."  -- https://sauvc.org/rulebook/

Only the top 15 qualifiers reach the finals, so this run gates every finals
point. It is also, move for move, the swim the pre-competition video must show
-- "dive, swim straight, turn 180° clockwise and swim back" -- which is why the
U-turn is `yaw_right` (clockwise) and not whichever way is shorter.

Run it, with the hull touching the marked section of the qualification wall
and pointing at the gate:

    ros2 launch mongla_manager bringup.launch.py vision:=true
    ros2 run mongla_planner mission sauvc_qualification

Shape: the Navigation mission's two-verb pass (centre, then drive through with
`fwd=None`) done twice around a clockwise turn, each pass with a blind leg as
its terminal branch. Surfacing, a gate touch and a floor touch all FAIL the
run, so every depth here sits in the 0.95 m opening under the surface and
inside the lane's own floor guard, and nothing surfaces until both passes are
done.

Not flown. Verified only that it imports, that it steers on a class the
shipped model emits, that its depths fit the opening and the lane's floor, that
its budgets fit the run, and that it sends no verb srot refuses. Every blind
duration and the standoff are unmeasured; the detector is sim-trained.

ON SROT, `move_*`, `set_depth`, `pause`, `stop` and `yaw_right` are SROT_MOVE
primitives: they enter AUTO, which closes the never-closed depth loop, so the
board denies them until the two bench checks pass -- the U-turn included. The
vision verbs run in STABILIZE and are not gated (test_sauvc_srot_port.py).
"""

from mongla_planner.missions.competition_config import (
    SAUVC_ALIGN_ERR_PX,
    SAUVC_ALIGN_GAIN,
    SAUVC_APPROACH_GAIN,
    SAUVC_BLIND_GAIN,
    SAUVC_GATE_COMMIT_S,
    SAUVC_QUAL_BLIND_BACK_S,
    SAUVC_QUAL_BLIND_ENABLED,
    SAUVC_QUAL_BLIND_OUT_S,
    SAUVC_QUAL_BUDGET_S,
    SAUVC_QUAL_GATE_ALIGN_S,
    SAUVC_QUAL_GATE_MOVE_S,
    SAUVC_QUAL_PASS_DEPTH_M,
    SAUVC_QUAL_STANDOFF_S,
    SAUVC_QUAL_TURN_DEG,
    SAUVC_QUAL_TURN_TIMEOUT_S,
    SAUVC_TETHER_PAUSE_S,
    SEARCH_CREEP_S,
    SEARCH_FORWARD_GAIN,
)

_FWD = '/mongla_detector_forward'

# The stem of the shipped SAUVC model, as in sauvc_navigation. `qual_gate` is
# the orange hanging gate; `final_gate` is the red/green finals gate.
_MODEL = 'sauvc_sim'
_GATE  = 'qual_gate'


def run(mongla, log=None):
    """Standalone qualification run: arm, out, U-turn, back, surface, disarm.

    Returns the number of passes the VISION transit confirmed (0-2). A blind
    leg is attempted but never counted: the judges score the gate, and a
    count that includes a guess is the plausible number this repo keeps
    finding standing in for a measurement.
    """
    mongla.mission_reset()
    confirmed = 0
    try:
        mongla.countdown(SAUVC_TETHER_PAUSE_S)
        mongla.arm()
        # Dive straight to the pass depth: the whole lane is the opening's
        # height band, and there is no search depth shallower than it that
        # is not closer to a surfacing failure.
        mongla.set_depth(SAUVC_QUAL_PASS_DEPTH_M, timeout=20)
        confirmed = qualify(mongla, log)
    finally:
        # Time stops at the second pass; surfacing after it costs nothing.
        mongla.surface()
        mongla.disarm()
    return confirmed


def qualify(mongla, log=None):
    """Out through the gate, clockwise U-turn, back through. Assumes armed and
    at the pass depth. Returns the vision-confirmed pass count."""
    mongla.resume_detector('forward')
    mongla.set_model(_MODEL, node=_FWD)
    mongla.set_classes(_GATE, node=_FWD)

    confirmed = int(_pass(mongla, SAUVC_QUAL_BLIND_OUT_S, 'out', log))

    # The hull is barely through. Swim on until the gate fits in the frame
    # again, THEN turn: an in-place turn leaves an align with no whole gate.
    mongla.move_forward(SAUVC_QUAL_STANDOFF_S, gain=SAUVC_BLIND_GAIN)
    mongla.yaw_right(SAUVC_QUAL_TURN_DEG, timeout=SAUVC_QUAL_TURN_TIMEOUT_S,
                     settle=1.0)

    confirmed += int(_pass(mongla, SAUVC_QUAL_BLIND_BACK_S, 'back', log))
    mongla.pause_detector('forward')
    if log:
        log(f'[SAUVC] qualification: {confirmed}/2 passes confirmed by vision')
    return confirmed


def _pass(mongla, blind_s, leg, log):
    """One gate pass: centre, drive through, else dead-reckon. True only when
    the vision transit reported the pass."""
    locked = mongla.vision.align(
        _GATE, camera='forward', yaw=0, lat=0,
        err=SAUVC_ALIGN_ERR_PX, gain=SAUVC_ALIGN_GAIN,
        duration=SAUVC_QUAL_GATE_ALIGN_S, fallback=creep_forward)
    passed = None
    if locked:
        # fwd=None is PASS-THROUGH: drive until the gate leaves the frame, then
        # commit to clear it -- "the back-most part of the AUV crosses the line".
        passed = mongla.vision.move(
            _GATE, camera='forward',
            fwd=None, hold=SAUVC_GATE_COMMIT_S,
            gain=SAUVC_APPROACH_GAIN, duration=SAUVC_QUAL_GATE_MOVE_S,
            fallback=creep_forward)
    if SAUVC_QUAL_BLIND_ENABLED and not passed:
        if log:
            log(f'[SAUVC] qual gate ({leg}) never locked -- blind leg '
                f'{blind_s:.0f}s at gain {SAUVC_BLIND_GAIN}')
        mongla.move_forward(blind_s, gain=SAUVC_BLIND_GAIN)
    return bool(passed)


def creep_forward(mongla):
    """One short forward creep, then return so the vision loop retries."""
    mongla.move_forward(SEARCH_CREEP_S, gain=SEARCH_FORWARD_GAIN)
