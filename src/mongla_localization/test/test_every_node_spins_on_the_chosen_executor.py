"""Every node main spins through `mongla_localization.spin`, or says why not.

The events executor delivers a callback for 0.037 ms of CPU against 0.251 ms
(measured-bars 73). One helper chooses it, so a node that calls `rclpy.spin`
itself quietly keeps the expensive one and nothing reports it.
"""
import pathlib
import re

import pytest

from mongla_localization import spin

SRC = pathlib.Path(__file__).resolve().parents[2]

# A node main that keeps its own executor, with the reason.
OWN_EXECUTOR = {
    'mongla_manager/mongla_manager/auv_manager_node.py':
        'action callbacks block for the length of a move while telemetry and '
        'health tick; the events executor has one thread',
    'mongla_vision/mongla_vision/utils/display_node.py':
        'an operator viewer off the vehicle, spinning several windows',
}


def _node_mains():
    for p in sorted(SRC.glob('mongla_*/mongla_*/**/*.py')):
        text = p.read_text(encoding='utf-8')
        if re.search(r'\brclpy\.spin\(|Executor\(\)', text):
            yield p.relative_to(SRC).as_posix(), text


def test_no_node_main_spins_on_its_own():
    bad = []
    for rel, text in _node_mains():
        if rel in OWN_EXECUTOR or rel.endswith('mongla_localization/spin.py'):
            continue
        if re.search(r'\brclpy\.spin\(', text) or 'make_executor' not in text:
            bad.append(rel)
    assert not bad, ('these node mains pick their own executor; use '
                     f'mongla_localization.spin, or add the reason: {bad}')


def test_every_exemption_still_exists():
    for rel in OWN_EXECUTOR:
        assert (SRC / rel).is_file(), f'{rel} is exempt but gone'


def test_the_choice_is_events_unless_told():
    assert spin.choice({}) == 'events'
    assert spin.choice({spin.ENV: 'classic'}) == 'classic'
    with pytest.raises(ValueError, match='evnts'):
        spin.choice({spin.ENV: 'evnts'})


def test_classic_gives_back_the_executor_the_node_used():
    class Mine:
        pass
    ex, name = spin.make_executor(classic=Mine, env={spin.ENV: 'classic'})
    assert isinstance(ex, Mine) and name == 'classic'


def test_an_rclpy_without_the_events_executor_falls_back_and_says_so(monkeypatch):
    import builtins
    real = builtins.__import__

    def no_experimental(name, *a, **k):
        if name == 'rclpy.experimental':
            raise ImportError(name)
        return real(name, *a, **k)
    monkeypatch.setattr(builtins, '__import__', no_experimental)

    class Mine:
        pass
    said = []
    log = type('L', (), {'info': lambda self, m: said.append(m)})()
    ex, name = spin.make_executor(log, classic=Mine, env={})
    assert isinstance(ex, Mine) and name == 'classic'
    assert said and 'no events executor' in said[0]
