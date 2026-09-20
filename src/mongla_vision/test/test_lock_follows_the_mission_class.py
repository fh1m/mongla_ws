"""The ladder must aim where the MISSION is looking, without being told twice.

`lock_class` is a LAUNCH argument. A mission that switches target mid-run --
gate -> rescue -> red_pipe, which the DSL does on every vision verb through
`set_classes` -- left the ladder following the FIRST class for the rest of the
run. Silently, and only on the rung whose job is to save the lock when the
detector drops it. At the pool that presents as "the ladder is broken" when it
is simply aimed somewhere else, which is exactly the kind of thing that must
not be debugged in the water.

`classes_filter` is LATCHED, so a ladder that starts after the detector has
already been aimed still picks the class up.

Drives the REAL handler against a fake node rather than asserting on source:
the failure is behavioural (which class wins, and when).
"""
import pathlib
import sys
import types

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from mongla_vision.lock_node import LockNode                      # noqa: E402


class _Log:
    def __init__(self): self.lines = []
    def info(self, m): self.lines.append(('info', m))
    def warn(self, m): self.lines.append(('warn', m))
    def warning(self, m): self.lines.append(('warn', m))
    def error(self, m): self.lines.append(('error', m))
    def debug(self, m): self.lines.append(('debug', m))


class _Fake:
    """Only what the handler touches."""
    def __init__(self, pinned='', width=0.0, cls=''):
        self._cls_pinned = pinned
        self._cls = cls
        self._target_w_m = 0.0
        self._width_param = width
        self._log = _Log()
    def get_parameter(self, name):
        assert name == 'target_width_m', name
        return types.SimpleNamespace(value=self._width_param)
    def get_logger(self):
        return self._log


def _feed(fake, csv):
    # Bind the real nested method too -- the handler calls `_retarget_width`,
    # and a fake that stubs it would test the stub.
    fake._retarget_width = types.MethodType(LockNode._retarget_width, fake)
    LockNode._on_classes_filter(fake, types.SimpleNamespace(data=csv))


def test_a_single_class_filter_aims_the_ladder():
    f = _Fake()
    _feed(f, 'gate')
    assert f._cls == 'gate'


def test_switching_the_mission_target_re_aims_it():
    """The whole point: gate -> red_pipe mid-run."""
    f = _Fake()
    _feed(f, 'gate')
    _feed(f, 'red_pipe')
    assert f._cls == 'red_pipe'


def test_an_explicit_lock_class_is_never_overridden():
    """An operator who pinned the aim gets to keep it."""
    f = _Fake(pinned='gate', cls='gate')
    _feed(f, 'red_pipe')
    assert f._cls == 'gate'


@pytest.mark.parametrize('csv', ['', '   ', 'gate,rescue', 'a,b,c'])
def test_an_ambiguous_filter_is_not_an_aim(csv):
    """Empty or multi-class is not a target. Picking one of several would be a
    guess, and not guessing is the ladder's entire value."""
    f = _Fake(cls='gate')
    _feed(f, csv)
    assert f._cls == 'gate'


def test_the_width_is_re_resolved_for_the_new_class():
    """Keeping the previous class's width would publish a confident range for
    the wrong object -- worse than refusing."""
    from mongla_vision.target_geometry import width_for
    known = None
    for name in ('gate', 'red_pipe', 'drum_blue', 'bin', 'flare'):
        if width_for(name) > 0.0:
            known = name
            break
    if known is None:
        pytest.skip('no class in the committed table to test against')
    f = _Fake()
    _feed(f, known)
    assert f._target_w_m == pytest.approx(width_for(known))


def test_an_unknown_class_refuses_the_pose_instead_of_keeping_a_stale_width():
    f = _Fake()
    f._target_w_m = 1.234                      # a previous class's width
    _feed(f, 'definitely_not_a_committed_class')
    assert f._target_w_m == 0.0, 'stale width survived a re-aim'
    assert any('no committed width' in m for _, m in f._log.lines)


def test_an_explicit_width_parameter_still_wins():
    f = _Fake(width=0.5)
    f._target_w_m = 0.5
    _feed(f, 'definitely_not_a_committed_class')
    assert f._target_w_m == 0.5


def test_the_subscription_is_LATCHED():
    """A ladder that starts after the detector was aimed must still learn the
    class -- that is what TRANSIENT_LOCAL is for here."""
    src = (pathlib.Path(__file__).resolve().parents[1] / 'mongla_vision'
           / 'lock_node.py').read_text()
    i = src.index("f'{ns}/classes_filter'")
    assert '_qos.LATCHED' in src[i:i + 160], (
        'classes_filter is subscribed with a non-latched QoS, so a late ladder '
        'never learns the class the mission already set')


# --------------------------------------------------------------------------
#  Aimable while the mission runs, not only at launch
# --------------------------------------------------------------------------

def test_target_class_is_a_LIVE_parameter():
    """The detector and the tracker both take live parameters; the ladder took
    none, so a mission could not aim it without a relaunch."""
    src = (pathlib.Path(__file__).resolve().parents[1] / 'mongla_vision'
           / 'lock_node.py').read_text()
    assert 'add_on_set_parameters_callback' in src, (
        'lock_node registers no parameter callback, so `ros2 param set` and '
        'the DSL cannot aim it while a mission runs')


def _param(name, value):
    return types.SimpleNamespace(name=name, value=value)


def test_setting_target_class_live_pins_and_re_aims():
    f = _Fake()
    f._retarget_width = types.MethodType(LockNode._retarget_width, f)
    LockNode._on_param_change(f, [_param('target_class', 'red_pipe')])
    assert f._cls == 'red_pipe' and f._cls_pinned == 'red_pipe'
    # ...and the pin now beats the mission's own filter
    _feed(f, 'gate')
    assert f._cls == 'red_pipe'


def test_setting_it_back_to_empty_RELEASES_the_pin():
    """A mission takes one leg manually and hands control back."""
    f = _Fake(pinned='gate', cls='gate')
    f._retarget_width = types.MethodType(LockNode._retarget_width, f)
    LockNode._on_param_change(f, [_param('target_class', '')])
    assert f._cls_pinned == ''
    _feed(f, 'red_pipe')
    assert f._cls == 'red_pipe', 'the pin was not released'


def test_a_negative_publish_hz_is_refused_not_silently_clamped():
    f = _Fake()
    f._pub_min_dt = 0.0
    res = LockNode._on_param_change(f, [_param('publish_hz', -5.0)])
    assert res.successful is False and 'publish_hz' in res.reason
