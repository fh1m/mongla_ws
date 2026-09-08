"""The vision loops must actuate the SROT board, and must not touch ArduSub.

`vision_align` / `vision_move` came out of `UNSUPPORTED_VERBS` on 2026-09-03.
That set was the ONLY thing keeping the host motion stack unreachable on this
backend, so the port needs a check that runs the real loop rather than one that
greps for a function name.

A grep was written first and is recorded here as insufficient: renaming the
DEFINITION of `_srot_drive` left every call site containing the string, so the
assertion stayed green through exactly the change it existed to catch. Same
shape as the drift suite's own caveat -- grep for structure, exercise for
behaviour.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
# Reuse the existing align/move harness rather than re-building fakes that
# would drift from it. The test DIRECTORY has to be importable for that.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from duburi_control.motion_vision import (       # noqa: E402
    _is_srot, _srot_drive, align_loop, move_loop,
)

from test_motion_vision import (                 # noqa: E402
    _FakeVision, _FakeWriters, _Log, _sample,
)


class _FakeSrot:
    """A SrotFC-shaped double: it has `manual()` and NONE of the RC primitives.

    Deliberately missing `send_rc_override` / `send_rc_translation` /
    `set_target_depth`, exactly as the real class is. A loop that reaches for
    one raises AttributeError, which is the honest outcome -- a stub that
    silently accepted them would let the test pass on a broken port.
    """
    name = 'srot'

    def __init__(self):
        self.manual_calls = []

    def manual(self, fwd, lat, up, yaw):
        self.manual_calls.append((fwd, lat, up, yaw))

    # Read surface the loops touch.
    def get_attitude(self):
        return {'yaw': 0.0, 'depth': -0.5}

    def get_mode(self):
        return 'STABILIZE'


def test_the_backend_predicate_is_not_true_for_pixhawk():
    from test_motion_vision import _FakePixhawk
    assert _is_srot(_FakeSrot()) is True
    assert _is_srot(_FakePixhawk()) is False


def test_srot_drive_scales_percent_to_unit():
    """The loops speak percent (-100..100); MANUAL_CONTROL speaks -1..1. A
    missing /100 is a 100x command, which on a real vehicle is full deflection
    for every non-zero error."""
    fc = _FakeSrot()
    _srot_drive(fc, fwd_pct=50.0, lat_pct=-25.0, yaw_pct=10.0)
    fwd, lat, up, yaw = fc.manual_calls[-1]
    assert fwd == pytest.approx(0.5)
    assert lat == pytest.approx(-0.25)
    assert yaw == pytest.approx(0.1)
    assert up == 0.0, 'the depth axis is not ported; up must stay neutral'


# --------------------------------------------------------------------------- #
#  The real loops, driven end to end
# --------------------------------------------------------------------------- #
def _run_align(**kw):
    fc = _FakeSrot()
    writers = _FakeWriters()
    defaults = dict(
        pixhawk=fc, vision_state=_FakeVision(_sample(ex=0.6, ey=0.0)),
        target_class='gate', axes={'yaw', 'lat'}, offsets={}, err_px=40.0,
        duration=0.3, gain=30.0, align_stable_frames=3, lost_grace_s=0.1,
        writers=writers, log=_Log(), abort_fn=None)
    defaults.update(kw)
    align_loop(**defaults)
    return fc


def test_align_actuates_through_manual_control_on_srot():
    """The headline: a real align_loop against a srot backend must produce
    MANUAL_CONTROL frames. If the port regressed, the FakeSrot has no
    send_rc_override and this raises instead of silently doing nothing."""
    fc = _run_align()
    assert fc.manual_calls, 'align_loop drove nothing on the srot backend'
    lat_cmds = [c[1] for c in fc.manual_calls]
    assert any(abs(v) > 0.01 for v in lat_cmds), (
        'every lateral command was ~0 for a target at ex=+0.6 -- the loop ran '
        'but commanded nothing')


def test_a_target_to_the_right_drives_lateral_positive():
    """Sign, end to end through the real loop. Backwards here means the hull
    accelerates away from the target at exactly the rate it should close."""
    right = _run_align(vision_state=_FakeVision(_sample(ex=0.6, ey=0.0)))
    left = _run_align(vision_state=_FakeVision(_sample(ex=-0.6, ey=0.0)))
    r = max((c[1] for c in right.manual_calls), key=abs)
    l = max((c[1] for c in left.manual_calls), key=abs)
    assert r > 0 and l < 0, f'lateral sign wrong: right={r}, left={l}'


def test_every_command_is_inside_the_unit_range():
    """MANUAL_CONTROL axes are -1..1. The board clamps, but a host that relies
    on the clamp has lost its own gain limit: `gain` is a SPEED CAP, and a
    value that only survives because the far end truncates it is not capped."""
    fc = _run_align(gain=100.0)
    for fwd, lat, up, yaw in fc.manual_calls:
        for v in (fwd, lat, up, yaw):
            assert -1.0 <= v <= 1.0, f'axis out of range: {(fwd, lat, up, yaw)}'


def test_gain_actually_caps_the_command():
    fc = _run_align(gain=20.0)
    assert max(abs(c[1]) for c in fc.manual_calls) <= 0.20 + 1e-6


def test_move_actuates_through_manual_control_on_srot():
    fc = _FakeSrot()
    move_loop(pixhawk=fc, vision_state=_FakeVision(_sample(ex=0.0, h_frac=0.2)),
              target_class='gate', fwd_fill=80.0, mode='area',
              maintain_px=None, duration=0.3, gain=30.0, lost_grace_s=0.1,
              writers=_FakeWriters(), log=_Log(), abort_fn=None)
    assert fc.manual_calls, 'move_loop drove nothing on the srot backend'
    assert all(c[3] == 0.0 for c in fc.manual_calls), (
        'move must never command yaw -- it is true on the ArduSub path and must '
        'stay true here')


def test_no_frame_ever_commands_the_depth_axis():
    """`up` is not ported: SrotFC has no set_target_depth, and vision_verbs
    refuses a depth-axis align. A non-zero `up` slipping through would be
    uncommanded vertical thrust."""
    fc = _run_align()
    assert all(c[2] == 0.0 for c in fc.manual_calls)


# --------------------------------------------------------------------------- #
#  The mode gate -- a verb must not run where the board discards its output
# --------------------------------------------------------------------------- #
from duburi_control.vision_verbs import (        # noqa: E402
    _SROT_VISION_MODES, _require_srot_vision_mode,
)
from duburi_control.errors import MovementError  # noqa: E402


class _ModeFC:
    """A board whose mode is scriptable, and which records set_mode attempts.

    `accepts` models the firmware's real behaviour: `onSetMode` discards its own
    return value and sends no ACK, so a refused mode change is indistinguishable
    from a successful one unless the caller re-reads.
    """
    name = 'srot'

    def __init__(self, mode='STABILIZE', accepts=True):
        self._mode = mode
        self._accepts = accepts
        self.set_calls = []

    def get_mode(self):
        return self._mode

    def set_mode(self, mode, timeout=8.0):
        self.set_calls.append(mode)
        if self._accepts:
            self._mode = mode
        return (self._accepts, mode)


def test_surface_is_refused_rather_than_run():
    """THE round-26 defect. SURFACE is a failsafe destination and the firmware
    zeroes translation and yaw in it, so an align there streams 50 Hz of
    MANUAL_CONTROL, reports success, and moves nothing."""
    fc = _ModeFC(mode='SURFACE', accepts=False)
    with pytest.raises(MovementError) as exc:
        _require_srot_vision_mode(fc, None, 'vision_align')
    msg = str(exc.value)
    assert 'SURFACE' in msg
    assert 'move nothing' in msg or 'moving nothing' in msg
    # and it must say where to look, not just that it refused
    assert 'failsafe' in msg.lower()


def test_a_bad_mode_is_SET_before_it_is_refused():
    """Refusing without trying would strand an operator whose board simply
    booted into the wrong mode -- which is the common case, not the failsafe."""
    fc = _ModeFC(mode='SURFACE', accepts=True)
    _require_srot_vision_mode(fc, None, 'vision_align')
    assert fc.set_calls == ['STABILIZE']
    assert fc.get_mode() == 'STABILIZE'


def test_an_acceptable_mode_is_left_alone():
    """A board already in an acceptable mode must not be switched mid-verb."""
    for mode in _SROT_VISION_MODES:
        fc = _ModeFC(mode=mode)
        _require_srot_vision_mode(fc, None, 'vision_align')
        assert fc.set_calls == [], f'{mode} should not have been changed'


def test_MANUAL_is_switched_to_STABILIZE_and_not_accepted():
    """MANUAL passes translation through, which is why the first version of
    this gate accepted it. That was wrong: MANUAL has NO attitude or heading
    hold (JETSON_COMMS.md -- "the escape hatch, not a driving mode"), and
    MANUAL_CONTROL carries no roll/pitch field, so nothing corrects a
    disturbance. The bounding box then moves for reasons unrelated to the
    vehicle's position and the loop chases them.

    A vision loop in MANUAL is the same class of defect as one in SURFACE: it
    runs, it reports success, and the number it converged on means nothing.
    """
    assert 'MANUAL' not in _SROT_VISION_MODES
    fc = _ModeFC(mode='MANUAL')
    _require_srot_vision_mode(fc, None, 'vision_align')
    assert fc.set_calls == ['STABILIZE'], (
        'a board in MANUAL must be moved to STABILIZE, not accepted as-is')


def test_the_check_re_reads_rather_than_trusting_set_mode():
    """`set_mode` is best-effort on this wire: the firmware's `onSetMode`
    discards its return and sends no ACK, so a silent refusal looks exactly
    like success. Trusting it is how this class of bug survives."""
    fc = _ModeFC(mode='SURFACE', accepts=False)
    with pytest.raises(MovementError):
        _require_srot_vision_mode(fc, None, 'vision_move')
    assert fc.set_calls == ['STABILIZE'], 'it must at least try'


def test_an_unknown_mode_is_refused_not_assumed_fine():
    fc = _ModeFC(mode='', accepts=False)
    with pytest.raises(MovementError):
        _require_srot_vision_mode(fc, None, 'vision_align')


# --------------------------------------------------------------------------- #
#  ...and that the VERB actually calls it
# --------------------------------------------------------------------------- #
# The five tests above exercise `_require_srot_vision_mode` directly. That is
# NOT sufficient, and it was demonstrated: deleting the call site from
# `vision_align` left all 21 tests green. Same shape as the `_srot_drive` grep
# that stayed green through the exact change it guarded -- a unit test of a
# guard says nothing about whether the guarded path invokes it.
#
# So this drives the REAL verb against a board in SURFACE.
from contextlib import contextmanager                # noqa: E402
from duburi_control.vision_verbs import VisionVerbs  # noqa: E402


class _VerbHarness(VisionVerbs):
    """Enough facade for vision_align/vision_move to reach their mode gate.

    Everything after the gate raises, deliberately: if the gate does NOT fire we
    want a loud, distinguishable failure rather than a silently-passing test.
    """

    def __init__(self, fc):
        self.pixhawk = fc
        self.log = _Log()

    @contextmanager
    def _command_scope(self, _verb):
        yield

    def _send_neutral_and_settle(self):
        pass

    def _resolve_vision_state(self, _camera):
        raise AssertionError('reached the vision state -- the mode gate did NOT fire')

    def _ensure_alt_hold(self, _verb):
        raise AssertionError('took the ArduSub branch on a srot backend')

    def _make_result(self, *a, **k):
        raise AssertionError('produced a result -- the mode gate did NOT fire')


def test_vision_align_ITSELF_refuses_on_a_surface_board():
    h = _VerbHarness(_ModeFC(mode='SURFACE', accepts=False))
    with pytest.raises(MovementError) as exc:
        h.vision_align(camera='forward', target_class='gate', axes='lat,yaw')
    assert 'SURFACE' in str(exc.value)


def test_vision_move_ITSELF_refuses_on_a_surface_board():
    h = _VerbHarness(_ModeFC(mode='SURFACE', accepts=False))
    with pytest.raises(MovementError) as exc:
        h.vision_move(camera='forward', target_class='gate', fwd_fill=80.0)
    assert 'SURFACE' in str(exc.value)


def test_vision_align_SETS_the_mode_when_the_board_will_take_it():
    """The other half: a board that merely booted into the wrong mode must be
    corrected and allowed to proceed, not stranded."""
    fc = _ModeFC(mode='SURFACE', accepts=True)
    h = _VerbHarness(fc)
    with pytest.raises(AssertionError, match='mode gate did NOT fire'):
        h.vision_align(camera='forward', target_class='gate', axes='lat,yaw')
    assert fc.set_calls == ['STABILIZE']
    assert fc.get_mode() == 'STABILIZE'


# =========================================================================== #
#  B30 -- THE LOOPS, END TO END, THROUGH THE **REAL** WRITERS                  #
# =========================================================================== #
# ⛔ WHY THIS BLOCK EXISTS, AND WHY THE TESTS ABOVE DID NOT CATCH B30.
#
# Everything above passes `writers=_FakeWriters()`. `_FakeSrot` is correctly
# strict -- it deliberately lacks `send_rc_override` -- but the WRITERS were
# doubled, and `make_writers` is exactly where the bug lived: it built every
# Writers around `send_rc_override` / `send_rc_translation`, neither of which
# SrotFC implements. The suite faked the component under suspicion, so it could
# not observe it.
#
# `_FakeWriters`'s own docstring says "only the arrival brake (`_brake_axis`)
# writes through `writers.forward`/`.lateral`" -- the single real path was known
# and stubbed anyway.
#
# These tests therefore use the REAL `SrotFC` (on a fake MAVLink master) and the
# REAL `make_writers`. Nothing between the verb and the wire is doubled, so a
# method the backend lacks raises here exactly as it would on the vehicle.

import pytest

from duburi_control.fc.srot_fc import SrotFC
from duburi_control.motion_writers import make_writers


class _FakeMasterMav:
    def __init__(self, sent):
        self._sent = sent

    def manual_control_send(self, target, x, y, z, r, buttons):
        self._sent.append(('manual', x, y, z, r))

    def heartbeat_send(self, *a, **kw):
        self._sent.append(('heartbeat',))

    def command_long_send(self, *a, **kw):
        self._sent.append(('command_long',))


class _FakeMaster:
    def __init__(self, sent):
        self.messages = {}
        self.mav = _FakeMasterMav(sent)


def _real_srot(sent):
    """The REAL SrotFC -- its true method surface, on a fake link."""
    return SrotFC(_FakeMaster(sent))


def _run_align_real_writers(**kw):
    """align_loop with a real SrotFC and real writers. Nothing doubled."""
    sent = []
    fc = _real_srot(sent)
    defaults = dict(
        pixhawk=fc, vision_state=_FakeVision(_sample(ex=0.6, ey=0.0)),
        target_class='gate', axes={'yaw', 'lat'}, offsets={}, err_px=40.0,
        duration=0.3, gain=30.0, align_stable_frames=3, lost_grace_s=0.1,
        writers=make_writers(fc), log=_Log(), abort_fn=None)
    defaults.update(kw)
    align_loop(**defaults)
    return fc, sent


class _SpyWriters:
    """The REAL srot writers, wrapped to record that they were actually touched.

    Recording is the point. The brake is self-gating (`_brake_axis` returns early
    below VISION_BRAKE_MIN_PCT), so a loop can call it and never reach the writer
    -- which is how a first draft of these tests passed with B30 re-injected. A
    test that cannot tell "the path was clean" from "the path was never taken"
    is not a test, so every case below asserts it reached the writer.
    """
    def __init__(self, real):
        self._real = real
        self.forwards, self.laterals, self.neutrals = [], [], 0

    def forward(self, pwm):
        self.forwards.append(pwm); self._real.forward(pwm)

    def lateral(self, pwm):
        self.laterals.append(pwm); self._real.lateral(pwm)

    def neutral(self):
        self.neutrals += 1; self._real.neutral()

    def depth_keepalive(self):
        self._real.depth_keepalive()

    @property
    def touched(self):
        return bool(self.forwards or self.laterals)


# An approach that provably REACHES the arrival brake: hard off-centre long
# enough to build the lateral EMA past VISION_BRAKE_MIN_PCT, then snap to centre
# so the loop declares arrival while that momentum is still on the books. This is
# the fast-approach case that failed on the vehicle; a gentle convergence exits
# with ~0 EMA and is deliberately NOT kicked.
_APPROACH = [_sample(ex=0.95)] * 8 + [_sample(ex=0.0)] * 12


def test_align_reaches_the_arrival_brake_on_srot_through_REAL_writers():
    """The B30 regression. Verified to fail with the srot writers branch removed."""
    sent = []
    fc = _real_srot(sent)
    spy = _SpyWriters(make_writers(fc))
    try:
        align_loop(pixhawk=fc, vision_state=_FakeVision(list(_APPROACH)),
                   target_class='gate', axes={'lat'}, offsets={}, err_px=40.0,
                   duration=2.0, gain=100.0, align_stable_frames=3,
                   lost_grace_s=0.5, writers=spy, log=_Log(), abort_fn=None)
    except AttributeError as exc:
        pytest.fail(f'align_loop reached a method SrotFC lacks (B30): {exc}')

    # PRECONDITION, asserted rather than assumed: if the brake stopped firing
    # (a gain, EMA-alpha or deadband change), this test silently stops covering
    # B30. Fail loudly and re-tune _APPROACH instead.
    assert spy.touched, (
        'the arrival brake never reached the writer, so this test did NOT '
        'exercise the B30 path -- retune _APPROACH above VISION_BRAKE_MIN_PCT')
    assert sent, 'no wire traffic'
    assert all(s[0] in ('manual', 'heartbeat', 'command_long') for s in sent)


def test_the_brake_kick_leaves_as_MANUAL_CONTROL_not_an_RC_frame():
    """Reaching the wire is not enough -- it must be the right message."""
    sent = []
    fc = _real_srot(sent)
    spy = _SpyWriters(make_writers(fc))
    align_loop(pixhawk=fc, vision_state=_FakeVision(list(_APPROACH)),
               target_class='gate', axes={'lat'}, offsets={}, err_px=40.0,
               duration=2.0, gain=100.0, align_stable_frames=3,
               lost_grace_s=0.5, writers=spy, log=_Log(), abort_fn=None)
    assert spy.touched, 'brake never fired; retune _APPROACH'
    assert {s[0] for s in sent} <= {'manual', 'heartbeat', 'command_long'},         'srot must never emit an RC_CHANNELS_OVERRIDE frame'


def test_move_runs_end_to_end_on_srot_through_real_writers():
    sent = []
    fc = _real_srot(sent)
    try:
        move_loop(pixhawk=fc, vision_state=_FakeVision(_sample(ex=0.0, h_frac=0.2)),
                  target_class='gate', fwd_fill=60.0, mode='height',
                  err_px=40.0, duration=0.3, gain=30.0, lost_grace_s=0.1,
                  writers=make_writers(fc), log=_Log(), abort_fn=None)
    except AttributeError as exc:
        pytest.fail(f'move_loop reached a method SrotFC lacks (B30): {exc}')
    assert sent


def test_no_test_in_this_file_may_fake_the_writers_for_srot():
    """A guard on the GUARD: faking make_writers is what hid B30 for a whole round.

    Not a ban on `_FakeWriters` -- it is fine for the pixhawk-path tests above.
    This asserts only that at least one srot path exercises the real builder, so
    the backend-specific branch can never again be entirely stubbed out.
    """
    import inspect
    src = inspect.getsource(sys.modules[__name__])
    assert 'make_writers(fc)' in src, \
        'no srot test exercises the real make_writers; that is precisely the ' \
        'hole B30 went through'
