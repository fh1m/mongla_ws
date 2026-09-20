"""SAUVC combinator: sequencing, budget/task use, and the two silent traps.

Executed against a recorder that plays the DSL. Nothing here flies.
"""
import contextlib
from types import SimpleNamespace

import pytest

import mission_ast as MA
from mongla_control.fc.srot_fc import UNSUPPORTED_VERBS
from mongla_planner.client import TaskAbandoned
from mongla_planner.missions import sauvc_full

_WIRE = {'release_heading': 'unlock_heading'}


class _Rec:
    def __init__(self, backend='srot', vision_ok=True, order=None, abandon=()):
        self.backend = backend
        self.calls = []          # (name, args, kwargs)
        self.notes = {}
        self._order = order
        self._abandon = set(abandon)
        rec = self

        class _V:
            def align(self, *a, **k):
                rec.calls.append(('vision_align', a, k))
                return vision_ok

            def move(self, *a, **k):
                rec.calls.append(('vision_move', a, k))
                return vision_ok
        self.vision = _V()

    @property
    def sent(self):
        return [c[0] for c in self.calls]

    @contextlib.contextmanager
    def task(self, name, deadline_s=None):
        self.calls.append(('task:' + name, (), {}))
        if name in self._abandon:
            raise TaskAbandoned(name)
        yield

    def worth_attempting(self, name, **_k):
        return SimpleNamespace(attempt=True, mode='full')

    def note(self, name, msg, success=True):
        self.notes.setdefault(name, []).append((msg, success))

    def flare_order(self, **_k):
        return self._order

    def __getattr__(self, name):
        def _call(*a, **k):
            self.calls.append((_WIRE.get(name, name), a, k))
            return True
        return _call


def _classes(rec):
    return [c[1][0] for c in rec.calls if c[0] == 'set_classes']


@pytest.mark.parametrize('flares', [False, True])
def test_no_refused_verb_on_srot(monkeypatch, flares):
    monkeypatch.setattr(sauvc_full, 'SAUVC_FLARES_ENABLED', flares)
    d = _Rec('srot')
    sauvc_full.run(d)
    assert not set(d.sent) & UNSUPPORTED_VERBS
    assert 'lock_heading' in [c[0] for c in _run('pixhawk').calls]


def _run(backend, **kw):
    d = _Rec(backend, **kw)
    sauvc_full.run(d)
    return d


def test_mission_reset_runs_once_before_arm_never_at_depth():
    """Each chunk's run() starts with mission_reset, which re-zeroes the baro;
    the combinator must call navigate()/acquire(), not run()."""
    s = _run('srot').sent
    assert s.count('mission_reset') == 1
    assert s.index('mission_reset') < s.index('arm')


def test_a_blind_or_failed_gate_is_recorded_unconfirmed():
    d = _run('srot', vision_ok=False)
    msg, ok = d.notes['navigation'][-1]
    assert not ok and msg.startswith('unconfirmed')
    assert _run('srot').notes['navigation'][-1] == ('confirmed', True)


def test_an_abandoned_navigation_still_attempts_target_acquisition():
    d = _run('srot', abandon={'navigation'})
    assert 'task:target_acquisition' in d.sent
    assert d.sent[-1] == 'disarm'


def test_flares_follow_the_lora_order_else_bump_all(monkeypatch):
    monkeypatch.setattr(sauvc_full, 'SAUVC_FLARES_ENABLED', True)
    ordered = _run('srot', order=('blue', 'red', 'yellow'))
    flare = [c for c in _classes(ordered) if c.startswith('flare_')]
    assert flare == ['flare_blue', 'flare_red', 'flare_yellow']
    default = _run('srot', order=None)
    assert [c for c in _classes(default) if c.startswith('flare_')] == \
        ['flare_red', 'flare_yellow', 'flare_blue']
    assert any('no LoRa order' in m for m, _ in default.notes['flares'])


def test_flares_are_off_by_default_and_say_so():
    assert sauvc_full.SAUVC_FLARES_ENABLED is False
    d = _run('srot')
    assert 'task:flares' not in d.sent
    assert any('skipped' in m for m, _ in d.notes['flares'])


def test_every_flare_class_exists_in_the_shipped_model():
    import yaml
    sidecar = MA.sidecar('sauvc_sim')
    if sidecar is None:
        pytest.skip('sauvc_sim.yaml sidecar is not on this host')
    names = set(yaml.safe_load(sidecar.read_text())['names'].values())
    assert set(sauvc_full._FLARE_CLASS.values()) <= names
    assert {c for c in sauvc_full.SAUVC_FLARE_DEFAULT_ORDER} <= set(sauvc_full._FLARE_CLASS)
