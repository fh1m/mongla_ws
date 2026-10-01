"""Telemetry ages are measured on the monotonic clock, not the wall (issue #18).

The Pi has no RTC; NTP steps its wall clock at the pool. Ages were wall minus
wall, so a step back made a value that stopped arriving 20 s ago read FRESH --
LEAK / KILL last-known-good standing in for a silent sensor -- and a step
forward made every live value read absent. The reader now stamps `msg._mono`
on receipt and every age is measured from it.
"""
import pathlib
import re
import time

import pytest

from mongla_control.fc import srot_fc
from mongla_control.fc.srot_fc import _rx_age


class _Msg:
    def __init__(self, mono_ago=None, wall_ago=0.0):
        self._timestamp = time.time() - wall_ago
        if mono_ago is not None:
            self._mono = time.monotonic() - mono_ago


def test_a_step_BACK_of_the_wall_clock_cannot_make_a_silent_value_fresh():
    """Received 30 s ago; the wall clock has since stepped back 30 s, so its
    stamp looks brand new. Falsified if the age reads ~0."""
    m = _Msg(mono_ago=30.0, wall_ago=0.0)
    assert _rx_age(m) == pytest.approx(30.0, abs=0.5)


def test_a_step_FORWARD_cannot_make_a_live_value_absent():
    m = _Msg(mono_ago=0.1, wall_ago=40.0)
    assert _rx_age(m) == pytest.approx(0.1, abs=0.5)


def test_a_message_this_reader_never_stamped_falls_back_to_the_old_age():
    assert _rx_age(_Msg(mono_ago=None, wall_ago=5.0)) == pytest.approx(5.0, abs=0.5)


def test_the_named_value_cache_is_stamped_and_read_on_one_monotonic_clock():
    fc = srot_fc.SrotFC.__new__(srot_fc.SrotFC)
    fc._named_cache = {'LEAK': (0.0, time.monotonic() - 30.0)}
    fc._last_nvf = None
    fc.master = type('M', (), {'messages': {}})()
    assert fc._named_value('LEAK', max_age_s=5.0) is None, 'a 30 s old LEAK read fresh'


def test_the_reader_stamps_every_message_on_receipt():
    src = (pathlib.Path(__file__).resolve().parents[2] / 'mongla_manager'
           / 'mongla_manager' / 'auv_manager_node.py').read_text(encoding='utf-8')
    i = src.index('msg = self.master.recv_match(blocking=False)')
    assert 'msg._mono = time.monotonic()' in src[i:i + 600]


def test_no_new_wall_minus_wall_age_in_srot_fc():
    """`time.time() - x` is a DURATION on a clock that steps. The only one
    allowed is `_rx_age`'s documented fallback."""
    src = (pathlib.Path(srot_fc.__file__)).read_text(encoding='utf-8')
    hits = [l.strip() for l in src.splitlines()
            if re.search(r'time\.time\(\)\s*-', l) and not l.strip().startswith('#')]
    assert hits == ["return time.time() - getattr(msg, '_timestamp', 0.0)"], hits
