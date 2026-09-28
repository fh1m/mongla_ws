"""Gate task — find the gate and drive through it.

Two-verb vision: align() centres (creeping forward via its fallback until
the gate appears), then move() drives through on bbox-height fill. No
command aborts the mission — a miss just logs and the run continues.

Standalone test (gate_sharks.pt present):
    ros2 run mongla_planner mission task_gate

Called by full_mission_2026 combinator after arm + depth set.
"""

from mongla_planner.missions.competition_config import (
    GATE_PASS_DEPTH_M,
    GATE_PASS_FWD_FILL,
    ALIGN_ERR_PX,
    ALIGN_GAIN,
    APPROACH_GAIN,
    SEARCH_FORWARD_GAIN,
    SEARCH_CREEP_S,
)

_FWD = '/mongla_detector_forward'


def run(mongla, log=None):
    mongla.mission_reset()   # clear heading lock + abort from any previous run
    mongla.resume_detector('forward')
    mongla.set_model('gate_sharks', node=_FWD)
    mongla.set_classes('gate,shark,shaw_fish', node=_FWD)

    # ── Centre on gate (yaw+lat); creep forward to find it if not yet seen ────
    mongla.vision.align(
        'gate', camera='forward', yaw=0, lat=0,
        err=ALIGN_ERR_PX, gain=ALIGN_GAIN, duration=30,
        fallback=creep_forward)

    # ⛔ THE rescue/repair SLIDE PHASE WAS REMOVED 2026-09-28, not renamed.
    # `gate_rescue_repair` was retired (+0.0 points of separation between
    # gate-present and gate-absent footage; measured-bars section 59) and no
    # shipped model emits `rescue` or `repair`. The phase used to slide onto
    # the marker to pick a scoring side.
    #
    # ⚠ Deleting it beats leaving it: `hailo.py` LOGS AN ERROR AND IGNORES an
    # allowlist whose every name is unknown, so the detector keeps emitting
    # gate/shark/shaw_fish, `align('rescue')` never matches, the fallback
    # thrusts open-loop for its full duration, and the verb returns normally.
    # A mission that reports success while commanding nothing is the failure
    # CLAUDE.md section 8.6 names as the one that ends runs.
    #
    # To bring it back: train a model that emits the marker classes AND clears
    # the separation bar with tools/negative_clip_check.py first.

    # ── Descend to pass depth, re-filter for the gate outline, drive through ──
    mongla.set_depth(GATE_PASS_DEPTH_M, timeout=20)
    mongla.set_classes('gate', node=_FWD)
    mongla.vision.move(
        'gate', camera='forward',
        fwd=GATE_PASS_FWD_FILL, mode='height',
        gain=APPROACH_GAIN, duration=25,
        fallback=creep_forward)

    mongla.pause(2.0)
    mongla.pause_detector('forward')


# ── Mission-authored fallback search patterns (pure control) ────────────────────
def creep_forward(mongla):
    """One short forward creep, then return so the vision loop retries."""
    mongla.move_forward(SEARCH_CREEP_S, gain=SEARCH_FORWARD_GAIN)
