"""What one SROT_MOVE may ask for, and how long to wait for its answer.

The host-side motion envelope (each bound catches a units bug, not a real
request), the ACK-to-result table, the stall budget, and the builders that
turn a verb's fields into SROT_MOVE parameters. Shared by `srot_fc.py` and
`srot_motion.py`; nothing here touches the link.
"""
from __future__ import annotations
import math
from .base import SUCCEEDED, PREEMPTED, FAILED, DENIED
from . import srot_protocol as sp


# COMMAND_ACK.result -> MoveResult code. Terminal set + values live in
# srot_protocol (pinned, since older pymavlink dialects lack CANCELLED=6).
# TEMPORARILY_REJECTED means the board missed a state mutex at dispatch, so the
# move never started -- same outcome as FAILED for a caller, different reason text.
_ACK_TO_CODE = {sp.ACK_ACCEPTED: SUCCEEDED, sp.ACK_CANCELLED: PREEMPTED,
                sp.ACK_FAILED: FAILED, sp.ACK_DENIED: DENIED,
                sp.ACK_TEMPORARILY_REJECTED: FAILED,
                # UNSUPPORTED is a DENIED, not a FAILED: the board will never run
                # this command, so "could not start" invites a retry that cannot
                # ever succeed. It is how an older board answers a newer verb.
                sp.ACK_UNSUPPORTED: DENIED}


# How long a SUCCEEDED move waits for a HEARTBEAT newer than its ACK before
# judging the board's mode. The manager requests HEARTBEAT at 10 Hz
# (SROT_MESSAGE_RATES), so one normally lands inside 0.1 s.
_POST_ACK_HB_WAIT_S = 0.3


# How long an aborted or stalled move waits to consume the ACKs its STOP sets in
# flight (issue #10). The stream task answers the displacement on its next tick
# and the STOP brakes in well under a second on the bench; 1.0 s bounds an abort
# that the board never answers, which is then logged, not hung on.
MOVE_ACK_DRAIN_S = 1.0


# ── the host-side motion envelope (issue #15) ────────────────────────────────
# Each bound catches a UNITS BUG or a TYPO, never a real request -- the reason is
# written beside it, and none is a tuning value.
#
# Deepest venue in the campaign is TRANSDEC at ~5 m; SAUVC is 1.2-1.6 m. 10 m
# restricts no real pool and refuses -50 (cm typed as m). The hull's own
# pressure rating is NOT measured, so this is not a crush-depth claim.
MAX_DEPTH_M = 10.0


# A turn of more than a full revolution is a units bug (3600 = deg x 10).
MAX_TURN_DEG = 360.0


# A single leg longer than ten minutes is a typo in a run that lasts ~15; a
# negative one (-30 for -3.0) has no meaning on the board, which is unverified.
MAX_DURATION_S = 600.0


_ACK_MARGIN_S     = 5.0    # slack over the expected leg time before calling it a stall


_ACK_MIN_BUDGET_S = 8.0    # floor, so a 0.5 s leg still tolerates a slow first ACK


_STYLE_ROLL_S     = 360.0 / 90.0   # MOVE_STYLE is always a roll at 90 deg/s


# A progressing move may refresh its silence window this many times over before
# the absolute ceiling stops it. Bounds the worst case at a number an operator can
# predict, without punishing a board that is merely slower than predicted.
_STALL_HARD_MULT = 4.0


def _ack_budget_s(verb: str, p1: float, p2: float, p5: float) -> float:
    """How long to wait for a terminal ACK before declaring a stall.

    Why this is not just `p5 + margin`: p5 is the board's own *safety* timeout and
    is 0 (-> the board's 60 s default) for every verb whose `commands.py` row has no
    `timeout` field -- move_forward/left/right, stop, pause. A 3 s move would then
    hold the action thread for 65 s before giving up.

    This used to be the ONLY terminator: a failsafe or operator mode change mid-leg
    froze `mv_active` true and the board streamed IN_PROGRESS forever, so nothing
    but this deadline could end the action.

    Firmware behaviour rev 2 (fw AUDIT.md R35) fixed that at the source -- the board
    now ends the move when AUTO is displaced and publishes `mv_*` unconditionally,
    so the falling edge that latches the terminal ACK is observable from outside
    AUTO. This is therefore a BACKSTOP now, for a dead link or a wedged board, not
    the primary path.

    Kept, and kept generous, for exactly that reason: the failure it still covers is
    "no ACKs at all", where a too-tight deadline would report a bogus stall on a
    slow-but-alive board. Budget from the expected leg time; p5 is only a floor.
    """
    move_type = int(p1)
    if move_type in (sp.MOVE_FORWARD, sp.MOVE_BACK, sp.MOVE_STRAFE_L,
                     sp.MOVE_STRAFE_R, sp.MOVE_ARC, sp.MOVE_HOLD):
        expected = abs(float(p2))                       # p2 is duration_s
    elif move_type == sp.MOVE_TURN:
        expected = abs(float(p2)) / sp.MOVE_YAW_RATE    # degrees at the default rate
    elif move_type == sp.MOVE_DIVE:
        expected = abs(float(p2)) / sp.MOVE_DEPTH_RATE  # metres at the ramp rate
    elif move_type == sp.MOVE_STYLE:
        expected = abs(float(p2)) * _STYLE_ROLL_S       # 360 deg at 90 deg/s
    else:                                               # MOVE_STOP and anything new
        expected = 0.0
    # x2 covers ramp + brake + a slow board; the floor keeps a tiny leg's deadline
    # sane, and p5 (when the caller set one) is honoured as a lower bound.
    return max(expected * 2.0 + _ACK_MARGIN_S, _ACK_MIN_BUDGET_S, float(p5 or 0.0) + _ACK_MARGIN_S)


# Saturation is refused whatever its cause, because SATURATION ITSELF is the hazard:
# a pinned heave demand becomes full vertical thrust the instant the outputs go live
# (mixer throttle column is -1 on all four verticals, 0 on all four horizontals). But
# the cause changes what the operator should DO, and a large zero offset is far more
# common than a dead sensor -- so name the cheap fix first instead of sending someone
# to look for a hardware fault.
_DEPTH_ARM_TAIL = (
    'Arming would command FULL vertical thrust (the mixer throttle column is -1 on all '
    'four verticals) with the horizontals idle. If the barometer VARIANCE is healthy '
    'this is a large zero offset, not a dead sensor -- run `mongla calibrate_depth`, '
    'then re-check with `ros2 run mongla_manager connect`')


# ---------------------------------------------------------------------- #
#  The verb table -- mongla Move verb -> SROT_MOVE (p1..p5).             #
#  Kept module-level + pure so it unit-tests without a live master.      #
# ---------------------------------------------------------------------- #
def _speed_from_gain(kw) -> float:
    """Move.Goal.gain is a 0..100 % thrust cap -> SROT 0..1 speed (clamped to cruise).

    A non-finite gain is refused, not coerced. `sanitize_speed` maps NaN to 0.0,
    which is the correct fail-safe for the STREAMING path (`manual()` cannot raise
    once per 20 Hz tick, and zero means no motion). It is the wrong answer for a
    ONE-SHOT verb: the board would accept a perfectly valid "move at speed 0",
    the hull would not move, and the mission would carry on believing it had --
    absence rendered as a number, which is the defect this register keeps finding.
    A verb that cannot be executed should say so. (B35)
    """
    gain = float(kw.get('gain', 0.0) or 0.0)
    if not math.isfinite(gain):
        raise ValueError(
            f'non-finite gain: {gain} -- a NaN gain usually means it was computed '
            f'from a VisionResult that never saw its target (check saw_target)')
    return sp.sanitize_speed(gain / 100.0)


def _depth_to_dive(kw) -> float:
    """mongla set_depth target is NEGATIVE metres (below surface); SROT DIVE p2 is a
    POSITIVE depth. Refuse a positive target (would be above the surface).

    The non-finite check is first, and it is not decoration (B35). `target > 0.0`
    is False for NaN -- every comparison against NaN is -- so the one guard written
    to reject a bad depth target used to ACCEPT NaN and emit it as p2. `-inf` got
    through the same way and became `-(-inf)` = **+inf**: a request to dive to
    infinite depth. Found by hammering the verb table, not by reading it.
    """
    target = float(kw.get('target', 0.0) or 0.0)
    if not math.isfinite(target):
        raise ValueError(f'non-finite set_depth target: {target}')
    if target > 0.0:
        raise ValueError(f'set_depth target must be <=0 (below surface), got {target}')
    if target < -MAX_DEPTH_M:
        raise ValueError(f'set_depth target {target} m is deeper than the '
                         f'{MAX_DEPTH_M:.0f} m envelope -- a units bug (cm as m?), '
                         f'not a pool (#15)')
    return -target


def _finite_param(name: str, value: float) -> float:
    """A wire parameter, or a ValueError NAMING the field that was not finite.

    This is not what stops NaN REACHING THE WIRE. `move()` already refuses the
    whole frame via the `_finite(*vals)` predicate above -- and the board refuses
    it again (fw `mav_commands.cpp:279-292`). Both of those say only
    "non-finite parameter"; this says WHICH ONE, at the point where the field
    still has a name. That is the entire value it adds, and B35's entry is
    written to keep that honest.
    """
    v = float(value)
    if not math.isfinite(v):
        # Keep the words "non-finite" -- `move()`'s catch-all uses them and
        # test_move_denied_on_nonfinite_param matches on them. This adds the
        # FIELD NAME without changing the contract callers already read.
        raise ValueError(
            f'non-finite {name}: {v} -- a NaN here almost always comes from a '
            f'VisionResult whose target was never seen (x_px/y_px are NaN then; '
            f'check saw_target before using them)')
    return v


def _bounded_time(name: str, value) -> float:
    """Finite, not negative, not past MAX_DURATION_S (issue #15)."""
    v = _finite_param(name, value)
    if v < 0.0:
        raise ValueError(f'negative {name} {v} s -- a typo (-30 for -3.0?); the '
                         f'board was never verified on a negative {name} (#15)')
    if v > MAX_DURATION_S:
        raise ValueError(f'{name} {v:.0f} s is past the {MAX_DURATION_S:.0f} s '
                         f'envelope for one leg -- a typo, not a manoeuvre (#15)')
    return v


def _bounded_turn(kw) -> float:
    """A turn target within one revolution (issue #15)."""
    v = _finite_param('target', kw.get('target', 0.0) or 0.0)
    if abs(v) > MAX_TURN_DEG:
        raise ValueError(f'turn target {v} deg is more than one revolution -- '
                         f'a units bug (deg x 10?), not a heading (#15)')
    return v


def _build_params(verb: str, kw: dict):
    """mongla verb + Move.Goal-ish kwargs -> (p1, p2, p3, p4, p5). Raises KeyError
    for an unmapped verb, ValueError for a bad parameter. This is THE verb table.

    Every returned tuple is checked finite before it leaves (B35): a NaN reached
    the wire from `duration`, `target` or `gain`, and NaN has a live source --
    `VisionResult.x_px` is NaN when the target was never seen."""
    # Validated at the boundary, so no per-verb branch below can forget it.
    # `speed` needs no check: sanitize_speed maps NaN to 0.0 and clamps inf.
    dur   = _bounded_time('duration', kw.get('duration', 0.0) or 0.0)
    tmo   = _bounded_time('timeout',  kw.get('timeout', 0.0) or 0.0)
    speed = _speed_from_gain(kw)
    if verb == 'move_forward':
        return (sp.MOVE_FORWARD, dur, speed, 0.0, tmo)
    if verb == 'move_back':
        return (sp.MOVE_BACK, dur, speed, 0.0, tmo)
    if verb == 'move_left':
        return (sp.MOVE_STRAFE_L, dur, speed, 0.0, tmo)
    if verb == 'move_right':
        return (sp.MOVE_STRAFE_R, dur, speed, 0.0, tmo)
    if verb == 'yaw_left':
        return (sp.MOVE_TURN, -abs(_bounded_turn(kw)), 0.0,
                float(sp.TURN_RELATIVE), tmo)
    if verb == 'yaw_right':
        return (sp.MOVE_TURN, abs(_bounded_turn(kw)), 0.0,
                float(sp.TURN_RELATIVE), tmo)
    if verb == 'turn':
        # mongla 'turn' is an ABSOLUTE heading -> needs MAG_YAW_REF=1 on the board.
        return (sp.MOVE_TURN, _bounded_turn(kw), 0.0,
                float(sp.TURN_ABSOLUTE), tmo)
    if verb == 'set_depth':
        return (sp.MOVE_DIVE, _depth_to_dive(kw), 0.0, 0.0, tmo)
    if verb == 'stop':
        return (sp.MOVE_STOP, 0.0, 0.0, 0.0, tmo)
    if verb == 'pause':
        # No channel-release on SROT -> station-keep hold (type 7) until timeout.
        return (sp.MOVE_HOLD, dur, 0.0, 0.0, tmo)
    if verb == 'style_roll':
        return (sp.MOVE_STYLE, float(kw.get('flips', 1.0) or 1.0), 0.0, 0.0, tmo)
    raise KeyError(f"verb '{verb}' has no SROT_MOVE mapping")
