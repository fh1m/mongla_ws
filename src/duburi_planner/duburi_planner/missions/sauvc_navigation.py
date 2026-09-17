"""SAUVC 2026 Navigation — swim through the red/green gate. THE mandatory task.

The rulebook is unambiguous about why this mission exists and every other SAUVC
mission does not yet: *"The first task, Navigation, is mandatory and must be
completed before attempting any other task."* Fail this and the maximum
achievable score for the whole run is zero, so nothing else is worth building
until this is solid.

    "The aim of this task is to swim through a gate placed at the bottom of the
     pool." -- 15 Points. The gate is "150cm wide and 100cm tall" with "striped
     red and green markings", "approximately 16m away from the starting zone".

Shape follows task_gate.py: two-verb vision (align to centre, then move to
transit), a mission-authored creep as the search fallback, and no verb that
aborts the run. The one addition is the terminal branch -- if the gate is never
locked we dead-reckon the leg anyway, because a perception miss must not zero a
task that gates all scoring.

Run it:
    ros2 launch duburi_manager bringup.launch.py vision:=true
    ros2 run duburi_planner mission sauvc_navigation

`run()` is self-contained (arm, dive, navigate, surface, disarm) because
Navigation is currently the whole run. A future SAUVC combinator should call
`navigate()` instead and own arming itself.

⚠ NOT FLOWN. Verified only that it imports, that its class names match the
shipped sauvc_sim model, that its depths respect the sloping floor and that its
durations fit the run budget. The detector is sim-trained and unvalidated on
real water; SAUVC_BLIND_TRANSIT_S is an unmeasured guess.

⛔ ON SROT, `move_*`, `set_depth`, `pause` and `stop` are SROT_MOVE primitives: they
enter AUTO, which closes the never-closed depth loop, so the board denies them until
the two bench checks pass. That includes the search creep fallback. The vision
verbs themselves run in STABILIZE and are not gated (test_sauvc_srot_port.py).
"""

from duburi_planner.missions.competition_config import (
    SAUVC_ALIGN_ERR_PX,
    SAUVC_ALIGN_GAIN,
    SAUVC_APPROACH_GAIN,
    SAUVC_BLIND_GAIN,
    SAUVC_BLIND_TRANSIT_ENABLED,
    SAUVC_BLIND_TRANSIT_S,
    SAUVC_GATE_ALIGN_S,
    SAUVC_GATE_COMMIT_S,
    SAUVC_GATE_MOVE_S,
    SAUVC_GATE_PASS_DEPTH_M,
    SAUVC_NAV_BUDGET_S,
    SAUVC_SEARCH_DEPTH_M,
    SAUVC_TETHER_PAUSE_S,
    SEARCH_CREEP_S,
    SEARCH_FORWARD_GAIN,
)

_FWD = '/duburi_detector_forward'

# The stem of the shipped SAUVC model. `sauvc_sim.hef` is on the vehicle at
# ~/hailo_models; `sim_sauvc_v1` is NOT, so naming that stem would silently drop
# the detector onto the 3-4 Hz PyTorch path. Class names come from its own
# sauvc_sim.yaml sidecar -- 'final_gate' is the Navigation gate, 'qual_gate' is
# the separate full-height qualifying gate and is a different task.
_MODEL = 'sauvc_sim'
_GATE  = 'final_gate'


def run(duburi, log=None):
    """Full standalone SAUVC Navigation run: arm, navigate, surface, disarm."""
    duburi.mission_reset()
    try:
        duburi.pause(SAUVC_TETHER_PAUSE_S)
        duburi.arm()
        duburi.set_depth(SAUVC_SEARCH_DEPTH_M, timeout=30)
        # srot refuses `lock_heading`: the board holds heading itself at 500 Hz
        # (STABILIZE for vision, AUTO for moves). On ArduSub the host lock is
        # what keeps the blind-transit line straight.
        if duburi.backend != 'srot':
            duburi.lock_heading(0.0, timeout=SAUVC_NAV_BUDGET_S)
        navigate(duburi, log)
    finally:
        duburi.release_heading()
        # Surfacing at the end of the run is worth +5 in the rulebook and costs
        # nothing, so it is part of the mission rather than left to buoyancy.
        duburi.surface()
        duburi.stop()
        duburi.disarm()


def navigate(duburi, log=None):
    """The task itself. Assumes armed and at search depth.

    Returns True only when the vision transit reported the pass; a blind
    transit or no attempt returns False (the run may still proceed on it).
    """
    duburi.resume_detector('forward')
    duburi.set_model(_MODEL, node=_FWD)
    duburi.set_classes(_GATE, node=_FWD)

    # Drop to the pass depth BEFORE centring, so the align and the transit see
    # the gate from the same height and the hull is not changing depth while
    # driving at a 1.0 m tall opening.
    duburi.set_depth(SAUVC_GATE_PASS_DEPTH_M, timeout=20)

    # ── Centre the gate (yaw+lat); creep forward to find it if not yet seen ───
    locked = duburi.vision.align(
        _GATE, camera='forward', yaw=0, lat=0,
        err=SAUVC_ALIGN_ERR_PX, gain=SAUVC_ALIGN_GAIN,
        duration=SAUVC_GATE_ALIGN_S, fallback=creep_forward)

    # ── Drive THROUGH: fwd unset = pass-through, so the verb keeps driving
    #    until the gate leaves the frame and then commits for a further window
    #    to physically clear the posts. A fill stop would park us in front.
    passed = None
    if locked:
        passed = duburi.vision.move(
            _GATE, camera='forward',
            fwd=None, hold=SAUVC_GATE_COMMIT_S,
            gain=SAUVC_APPROACH_GAIN, duration=SAUVC_GATE_MOVE_S,
            fallback=creep_forward)

    # ── Selector[precise, always_act] ────────────────────────────────────────
    # Navigation gates every other point in the run, so a perception miss must
    # degrade to an attempt, never to a stop. The rulebook gives the distance
    # ("approximately 16m away from the starting zone"), which is exactly enough
    # to dead-reckon the leg. This is a PARAMETER, not a branch to comment out.
    #
    # ⛔ PRECONDITION, and it is not a comfortable one. This reaches the gate only
    # if the hull started inside the starting zone pointing at it and the heading
    # lock held the line. Perception failure and heading error are CORRELATED --
    # murky water or a bad start pose causes both -- so this branch fires exactly
    # when it is least likely to work, and a wall touch is -5. It is still the
    # right trade (zero is worse), but do not read it as a reliable fallback.
    if SAUVC_BLIND_TRANSIT_ENABLED and not passed:
        if log:
            log('[SAUVC] gate never locked -- blind transit, '
                f'{SAUVC_BLIND_TRANSIT_S:.0f}s at gain {SAUVC_BLIND_GAIN}')
        duburi.move_forward(SAUVC_BLIND_TRANSIT_S, gain=SAUVC_BLIND_GAIN)

    duburi.pause_detector('forward')
    return bool(passed)


# ── Mission-authored fallback search pattern (pure control) ────────────────────
def creep_forward(duburi):
    """One short forward creep, then return so the vision loop retries."""
    duburi.move_forward(SEARCH_CREEP_S, gain=SEARCH_FORWARD_GAIN)
