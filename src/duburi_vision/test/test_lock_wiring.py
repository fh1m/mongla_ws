"""The ladder reaching the CONTROL loop -- the wiring, and its safety.

`vision.lock_s` lets `bbox_error()` fall through to the ladder
(`lock_node`: follower + XFeat anchor) as the last rung before declaring loss.
It is the point where a followed or anchored box can move thrusters, so these
tests are about the properties that make that acceptable rather than about the
plumbing working.

Source-level assertions rather than a live graph: what must hold is structural
(the default is off, the fallback sits AFTER coast, the ladder's decay is not
applied twice), and a rclpy fixture would test the harness more than the code.
The behavioural half is covered by `test_lock_state.py`, which drives the
arbiter directly.
"""
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

ROOT = Path(__file__).resolve().parents[2]
VS = (ROOT / 'duburi_manager' / 'duburi_manager' / 'vision_state.py').read_text()
MV = (ROOT / 'duburi_control' / 'duburi_control' / 'motion_vision.py').read_text()
TUN = (ROOT / 'duburi_manager' / 'duburi_manager'
       / 'vision_tunables.py').read_text()


def test_the_ladder_is_ON_and_the_file_records_why():
    """ON since the deck watch of 2026-09-10, and the evidence lives beside the
    number rather than in a commit message nobody reads at 2 a.m.

    This test used to assert 0.0, on the argument that switching the ladder on
    "would put a followed box under the thrusters of every existing mission".
    That argument was right and it has been ANSWERED, not waived: 60 s on the
    vehicle with a live target gave 185 detection / 63 follow / 0 anchor and
    zero boxes the detector had not seen. The follower bridged 25 % of ticks.
    """
    m = re.search(r"'vision\.lock_s':\s*([0-9.]+)", TUN)
    assert m, 'vision.lock_s is not declared'
    assert float(m.group(1)) > 0.0, (
        'the ladder is off; the control loop will never consult the follower '
        'or the anchor and every detector gap ends in a declared loss')
    i = TUN.index("'vision.lock_s'")
    why = TUN[max(0, i - 900):i]
    assert 'deck watch' in why or '185 detection' in why, (
        'the default is ON with no measurement recorded next to it')


def test_it_is_the_LAST_rung_after_coast():
    """Order matters: a live detection, then the tracker's coast, then the
    ladder. Consulting the ladder first would substitute an anchored box for a
    coasted one the tracker was still perfectly able to supply."""
    i_coast = VS.index('_coast_sample(class_name, locked_id, coast_s')
    i_lock = VS.index('return self._lock_sample(')
    assert i_coast < i_lock


def test_coast_still_gets_first_refusal():
    """The ladder must only be consulted when coast produced nothing -- not
    instead of it."""
    seg = VS[VS.index('if best_detection is None:'):
             VS.index('return None', VS.index('if best_detection is None:'))]
    assert 'if cs is not None:' in seg, (
        'coast must be allowed to answer before the ladder is asked')


def test_the_ladder_decay_is_NOT_applied_twice():
    """`lock_node` already multiplies rung trust by time decay into `score`.
    Decaying it again here would make the fallback die roughly twice as fast as
    designed -- the same arithmetic error that made the uplink's coasted target
    expire inside 0.4 s."""
    seg = VS[VS.index('def _lock_sample'):VS.index('def bbox_error')]
    assert 'score=score' in seg
    for bad in ('score *', 'score*', 'authority_for', '* auth'):
        assert bad not in seg, f'the ladder score is being re-scaled by {bad!r}'


def test_an_absent_ladder_is_a_LOSS_not_a_hold():
    """THE safety property. `lock_node` publishes nothing once its own
    authority reaches zero, so "no message" must mean loss declared on
    schedule. Returning a stale cached box here would turn the decay into a
    suggestion."""
    seg = VS[VS.index('def _lock_sample'):VS.index('def bbox_error')]
    assert 'if arr is None or not arr.detections:' in seg
    assert seg.count('return None') >= 2      # empty array AND zero score


def test_a_zero_score_ladder_message_is_refused():
    """Score is the arbiter's confidence. Zero means it does not stand behind
    the position, and publishing the box anyway would be the one thing the
    ladder promises never to do."""
    seg = VS[VS.index('def _lock_sample'):VS.index('def bbox_error')]
    assert 'if score <= 0.0:' in seg


def test_both_control_loops_pass_it_through():
    """A parameter declared and not threaded is the shape of bug this package
    has shipped repeatedly -- `device_path` into `**_`, the unloaded YAML
    table, `ros2 param set` on a construction-time param."""
    assert MV.count('lock_s: float = 0.0,') == 2      # align_loop and move_loop
    assert MV.count('lock_s=lock_s') == 2


def test_the_deck_exposes_it_for_live_tuning():
    assert "'lock_s':              'vision.lock_s'," in TUN


def test_the_ladder_sample_is_marked_coasted():
    """Anything reading `Sample.coasted` must be able to tell a real sighting
    from a propagated one. The mid-hold FIRE gate reads exactly this."""
    seg = VS[VS.index('def _lock_sample'):VS.index('def bbox_error')]
    assert 'coasted=True' in seg
