"""SAUVC 2026 full run: Navigation, Target Acquisition, then (opt-in) the flares.

One dive, so the timing bonus is reachable: "(900 - run) x 0.03", and it needs
two tasks. Each chunk is bounded by `mongla.task` and rationed by
`mongla.use_budget` / `worth_attempting`, so a slow task is abandoned rather
than eating the next one, and a skipped task says why on the scoreboard.

Chunks are called through `navigate()` / `acquire()`, never their `run()`:
each `run()` starts with `mission_reset()`, which re-zeroes the barometer and
must not happen at depth.

    "The first task, Navigation, is mandatory and must be completed before
     attempting any other task."  -- https://sauvc.org/rulebook/

A blind gate transit cannot confirm the pass. The run proceeds anyway (zero is
worse) and the scoreboard says `unconfirmed`, so a later score is not read as
built on a confirmed gate.

Task 3 (Target Reacquisition, 60) is skipped by name: it needs a gripper this
vehicle does not carry. Task 4 is OFF (`SAUVC_FLARES_ENABLED`) until the flare
detection range is measured in water.

Run it:
    ros2 run mongla_planner mission sauvc_full

⚠ NOT FLOWN. Tests execute the sequencing against a recorder, on both backends.

⛔ ON SROT, `move_*`, `set_depth`, `pause` and `stop` are SROT_MOVE primitives: they
enter AUTO, which closes the never-closed depth loop, so the board denies them until
the two bench checks pass. That includes the search creep fallback. The vision
verbs themselves run in STABILIZE and are not gated (test_sauvc_srot_port.py).
"""

from mongla_planner.client import TaskAbandoned
from mongla_planner.missions import sauvc_navigation, sauvc_target_acquisition
from mongla_planner.missions.competition_config import (
    SAUVC_DRUM_WORST_S,
    SAUVC_FLARE_ALIGN_S,
    SAUVC_FLARE_BACKOFF_S,
    SAUVC_FLARE_DEFAULT_ORDER,
    SAUVC_FLARE_GAIN,
    SAUVC_FLARE_LISTEN_S,
    SAUVC_FLARE_MOVE_S,
    SAUVC_FLARE_PUSH_S,
    SAUVC_FLARE_STOP_FILL,
    SAUVC_FLARES_ENABLED,
    SAUVC_FLARES_WORST_S,
    SAUVC_NAV_BUDGET_S,
    SAUVC_RESERVE_S,
    SAUVC_RUN_BUDGET_S,
    SAUVC_SEARCH_DEPTH_M,
    SAUVC_TETHER_PAUSE_S,
    SEARCH_CREEP_S,
    SEARCH_FORWARD_GAIN,
)

_FWD = '/mongla_detector_forward'
_MODEL = 'sauvc_sim'
# flare_order() speaks colours; the model speaks class names. Unmapped, a colour
# becomes an empty allowlist and a silent [] every frame.
_FLARE_CLASS = {'red': 'flare_red', 'yellow': 'flare_yellow', 'blue': 'flare_blue'}


def run(mongla, log=None):
    mongla.mission_reset()
    mongla.use_budget(SAUVC_RUN_BUDGET_S, reserve_s=SAUVC_RESERVE_S)
    try:
        mongla.pause(SAUVC_TETHER_PAUSE_S)
        mongla.arm()
        mongla.set_depth(SAUVC_SEARCH_DEPTH_M, timeout=30)
        if mongla.backend != 'srot':            # srot holds heading on the board
            mongla.lock_heading(0.0, timeout=SAUVC_RUN_BUDGET_S)

        _navigation(mongla, log)
        mongla.note('target_reacquisition', 'skipped: no gripper fitted', success=False)
        _target_acquisition(mongla, log)
        if SAUVC_FLARES_ENABLED:
            _flares(mongla, log)
        else:
            mongla.note('flares', 'skipped: SAUVC_FLARES_ENABLED is off '
                        '(flare detection range unmeasured)', success=False)
    finally:
        mongla.release_heading()
        mongla.surface()
        mongla.stop()
        mongla.disarm()


def _navigation(mongla, log):
    confirmed = False
    try:
        with mongla.task('navigation', deadline_s=SAUVC_NAV_BUDGET_S):
            confirmed = sauvc_navigation.navigate(mongla, log)
    except TaskAbandoned:
        pass
    mongla.note('navigation', 'confirmed' if confirmed else
                'unconfirmed: no vision pass (blind transit or abandoned)',
                success=bool(confirmed))


def _target_acquisition(mongla, log):
    v = mongla.worth_attempting('target_acquisition', points=30,
                                worst_case_s=SAUVC_DRUM_WORST_S)
    if not v.attempt:
        return
    try:
        with mongla.task('target_acquisition'):
            sauvc_target_acquisition.acquire(mongla, log)
    except TaskAbandoned:
        pass


def _flares(mongla, log):
    v = mongla.worth_attempting('flares', points=120, worst_case_s=SAUVC_FLARES_WORST_S)
    if not v.attempt:
        return
    try:
        with mongla.task('flares'):
            order = mongla.flare_order(timeout=SAUVC_FLARE_LISTEN_S)
            if order is None:
                mongla.note('flares', 'no LoRa order: bumping all, no order bonus',
                            success=False)
                order = SAUVC_FLARE_DEFAULT_ORDER
            mongla.use_camera('forward')
            mongla.set_model(_MODEL, node=_FWD)
            for colour in order:
                cls = _FLARE_CLASS.get(str(colour).lower())
                if cls is None:
                    mongla.note('flares', f'unknown colour {colour!r} skipped',
                                success=False)
                    continue
                bump(mongla, cls)
    except TaskAbandoned:
        pass


def bump(mongla, cls):
    """Centre on one flare, close to contact range, push, back off."""
    mongla.set_classes(cls, node=_FWD)
    locked = mongla.vision.align(cls, camera='forward', yaw=0, lat=0,
                                 gain=SAUVC_FLARE_GAIN, duration=SAUVC_FLARE_ALIGN_S,
                                 fallback=creep_forward)
    if not locked:
        mongla.note(cls, 'never centred: not bumped', success=False)
        return
    mongla.vision.move(cls, camera='forward', fwd=SAUVC_FLARE_STOP_FILL, mode='height',
                       gain=SAUVC_FLARE_GAIN, duration=SAUVC_FLARE_MOVE_S)
    mongla.move_forward(SAUVC_FLARE_PUSH_S, gain=SAUVC_FLARE_GAIN)
    mongla.move_back(SAUVC_FLARE_BACKOFF_S, gain=SAUVC_FLARE_GAIN)


def creep_forward(mongla):
    mongla.move_forward(SEARCH_CREEP_S, gain=SEARCH_FORWARD_GAIN)
