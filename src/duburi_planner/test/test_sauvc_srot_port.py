"""The SAUVC missions must run on srot without sending a verb srot refuses.

The manager answers a refused verb with success=False, which the DSL raises as
MoveFailed -- so one `lock_heading` aborts the whole Navigation run before the
gate. These tests EXECUTE each mission's `run()` against a recorder that plays
the DSL, and compare the verbs it would send to `srot_fc.UNSUPPORTED_VERBS`.
The pixhawk run is the control: the same mission must still lock heading there.
"""
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from duburi_control.fc.srot_fc import UNSUPPORTED_VERBS
from duburi_planner.missions import sauvc_navigation, sauvc_target_acquisition

# DSL method -> wire verb, where the names differ.
_WIRE = {'release_heading': 'unlock_heading'}


class _Recorder:
    """Every DSL call is recorded as the verb it would send, and succeeds."""

    def __init__(self, backend):
        self.backend = backend
        self.sent = []
        rec = self

        class _Vision:
            def align(self, *_a, **_k):
                rec.sent.append('vision_align')
                return True

            def move(self, *_a, **_k):
                rec.sent.append('vision_move')
                return True

        self.vision = _Vision()

    def __getattr__(self, name):
        def _call(*_a, **_k):
            self.sent.append(_WIRE.get(name, name))
            return True
        return _call


@pytest.mark.parametrize('mission', [sauvc_navigation, sauvc_target_acquisition])
def test_no_refused_verb_is_sent_on_srot(mission):
    d = _Recorder('srot')
    mission.run(d)
    assert d.sent, 'the mission sent nothing'
    refused = sorted(set(d.sent) & UNSUPPORTED_VERBS)
    assert not refused, f'{mission.__name__} sends {refused} on srot'


def test_navigation_still_locks_heading_on_pixhawk():
    d = _Recorder('pixhawk')
    sauvc_navigation.run(d)
    assert 'lock_heading' in d.sent


def test_the_drum_align_does_not_descend_on_srot():
    """A descent on the downward camera moves a depth setpoint, which srot
    refuses; with the fill configured OFF the align is lat + surge only."""
    from duburi_planner.missions import competition_config as CFG
    assert not CFG.SAUVC_DRUM_DESCEND_FILL


# ── `duburi.backend` reads the manager, once, and refuses to guess ──────────

def _dsl_reading(monkeypatch, value):
    from duburi_planner.duburi_dsl import DuburiMission
    monkeypatch.setattr('duburi_planner.duburi_dsl.rclpy.spin_until_future_complete',
                        lambda *a, **k: None)
    fut = MagicMock()
    fut.result.return_value = (None if value is None else
                               SimpleNamespace(values=[SimpleNamespace(string_value=value)]))
    node = MagicMock()
    node.create_client.return_value.call_async.return_value = fut
    m = DuburiMission.__new__(DuburiMission)
    m.__dict__['client'] = SimpleNamespace(node=node)
    return m, node


def test_backend_is_read_from_the_manager_and_cached(monkeypatch):
    m, node = _dsl_reading(monkeypatch, ' SROT ')
    assert m.backend == 'srot'
    assert m.backend == 'srot'
    assert node.create_client.call_count == 1


@pytest.mark.parametrize('value', [None, 'ardusub', ''])
def test_an_unreadable_or_unknown_backend_raises(monkeypatch, value):
    m, _ = _dsl_reading(monkeypatch, value)
    with pytest.raises(RuntimeError):
        _ = m.backend
