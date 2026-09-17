"""SAUVC 2026 full run: Navigation, Target Acquisition, then (opt-in) the flares.

One dive, so the timing bonus is reachable: "(900 - run) x 0.03", and it needs
two tasks. Each chunk is bounded by `duburi.task` and rationed by
`duburi.use_budget` / `worth_attempting`, so a slow task is abandoned rather
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
    ros2 run duburi_planner mission sauvc_full

⚠ NOT FLOWN. Tests execute the sequencing against a recorder, on both backends.

⛔ ON SROT, `move_*`, `set_depth`, `pause` and `stop` are SROT_MOVE primitives: they
enter AUTO, which closes the never-closed depth loop, so the board denies them until
the two bench checks pass. That includes the search creep fallback. The vision
verbs themselves run in STABILIZE and are not gated (test_sauvc_srot_port.py).
"""

from duburi_planner.client import TaskAbandoned
from duburi_planner.missions import sauvc_navigation, sauvc_target_acquisition
from duburi_planner.missions.competition_config import (
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

_FWD = '/duburi_detector_forward'
_MODEL = 'sauvc_sim'
# flare_order() speaks colours; the model speaks class names. Unmapped, a colour
# becomes an empty allowlist and a silent [] every frame.
_FLARE_CLASS = {'red': 'flare_red', 'yellow': 'flare_yellow', 'blue': 'flare_blue'}


def run(duburi, log=None):
    duburi.mission_reset()
    duburi.use_budget(SAUVC_RUN_BUDGET_S, reserve_s=SAUVC_RESERVE_S)
    try:
        duburi.pause(SAUVC_TETHER_PAUSE_S)
        duburi.arm()
        duburi.set_depth(SAUVC_SEARCH_DEPTH_M, timeout=30)
        if duburi.backend != 'srot':            # srot holds heading on the board
            duburi.lock_heading(0.0, timeout=SAUVC_RUN_BUDGET_S)

        _navigation(duburi, log)
        duburi.note('target_reacquisition', 'skipped: no gripper fitted', success=False)
        _target_acquisition(duburi, log)
        if SAUVC_FLARES_ENABLED:
            _flares(duburi, log)
        else:
            duburi.note('flares', 'skipped: SAUVC_FLARES_ENABLED is off '
                        '(flare detection range unmeasured)', success=False)
    finally:
        duburi.release_heading()
        duburi.surface()
        duburi.stop()
        duburi.disarm()


def _navigation(duburi, log):
    confirmed = False
    try:
        with duburi.task('navigation', deadline_s=SAUVC_NAV_BUDGET_S):
            confirmed = sauvc_navigation.navigate(duburi, log)
    except TaskAbandoned:
        pass
    duburi.note('navigation', 'confirmed' if confirmed else
                'unconfirmed: no vision pass (blind transit or abandoned)',
                success=bool(confirmed))


def _target_acquisition(duburi, log):
    v = duburi.worth_attempting('target_acquisition', points=30,
                                worst_case_s=SAUVC_DRUM_WORST_S)
    if not v.attempt:
        return
    try:
        with duburi.task('target_acquisition'):
            sauvc_target_acquisition.acquire(duburi, log)
    except TaskAbandoned:
        pass


def _flares(duburi, log):
    v = duburi.worth_attempting('flares', points=120, worst_case_s=SAUVC_FLARES_WORST_S)
    if not v.attempt:
        return
    try:
        with duburi.task('flares'):
            order = duburi.flare_order(timeout=SAUVC_FLARE_LISTEN_S)
            if order is None:
                duburi.note('flares', 'no LoRa order: bumping all, no order bonus',
                            success=False)
                order = SAUVC_FLARE_DEFAULT_ORDER
            duburi.use_camera('forward')
            duburi.set_model(_MODEL, node=_FWD)
            for colour in order:
                cls = _FLARE_CLASS.get(str(colour).lower())
                if cls is None:
                    duburi.note('flares', f'unknown colour {colour!r} skipped',
                                success=False)
                    continue
                bump(duburi, cls)
    except TaskAbandoned:
        pass


def bump(duburi, cls):
    """Centre on one flare, close to contact range, push, back off."""
    duburi.set_classes(cls, node=_FWD)
    locked = duburi.vision.align(cls, camera='forward', yaw=0, lat=0,
                                 gain=SAUVC_FLARE_GAIN, duration=SAUVC_FLARE_ALIGN_S,
                                 fallback=creep_forward)
    if not locked:
        duburi.note(cls, 'never centred: not bumped', success=False)
        return
    duburi.vision.move(cls, camera='forward', fwd=SAUVC_FLARE_STOP_FILL, mode='height',
                       gain=SAUVC_FLARE_GAIN, duration=SAUVC_FLARE_MOVE_S)
    duburi.move_forward(SAUVC_FLARE_PUSH_S, gain=SAUVC_FLARE_GAIN)
    duburi.move_back(SAUVC_FLARE_BACKOFF_S, gain=SAUVC_FLARE_GAIN)


def creep_forward(duburi):
    duburi.move_forward(SEARCH_CREEP_S, gain=SEARCH_FORWARD_GAIN)
