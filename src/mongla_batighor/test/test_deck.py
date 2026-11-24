"""The deck reads what an operator types, and never shows an absent number as zero."""
import json
import math
from types import SimpleNamespace

from mongla_batighor.deck import KEYS, parse_line, render
from mongla_batighor.helm import build_goal


def test_a_typed_line_becomes_the_goal_the_verb_takes():
    cmd, fields = parse_line(':move_forward duration=2 gain=40')
    goal, why = build_goal(cmd, fields)
    assert why == '' and goal.duration == 2.0 and goal.gain == 40.0
    cmd, fields = parse_line('set_mode target_name=DEPTH_HOLD')
    goal, why = build_goal(cmd, fields)
    assert why == '' and goal.target_name == 'DEPTH_HOLD'


def test_a_malformed_word_is_refused_not_dropped():
    import pytest
    with pytest.raises(ValueError, match="'duration' is not name=value"):
        parse_line('move_forward duration')


def test_the_three_keys_are_the_three_verbs_anyone_may_send():
    from mongla_batighor.seat import ANYONE
    assert set(KEYS.values()) == ANYONE


def test_an_absent_reading_shows_as_dashes_never_zero():
    v = SimpleNamespace(armed=False, mode='', depth_m=math.nan, yaw_deg=math.nan,
                        battery_voltage=math.nan)
    top = render(120, v, {}, [])[0]
    assert '0.00' not in top and 'depth --' in top and 'battery --' in top
    assert 'no /mongla/state' in render(120, None, {}, [])[0]


def test_the_log_says_who_and_what_and_why_a_refusal():
    lines = [json.dumps({'t': 0, 'operator': 'raf', 'op': 'command', 'cmd': 'arm',
                         'ok': False, 'message': 'arm needs both confirmations'}),
             json.dumps({'t': 0, 'event': 'ana is the pilot (took the free helm)'})]
    out = render(200, None, {'pilot': 'ana', 'asker': 'raf', 'mission': 'qualify'}, lines)
    text = '\n'.join(out)
    assert 'helm     ana   (raf is asking)   mission qualify' in text
    assert 'raf  command arm  REFUSED: arm needs both confirmations' in text
    assert 'ana is the pilot' in text
    assert all(len(l) <= 40 for l in render(40, None, {}, lines))


def test_an_unknown_arm_state_and_a_missing_time_show_as_dashes():
    # `armed` is False before the first heartbeat; it read "disarmed". A log
    # line with no `t` read as 1970 (audit 2026-11-24).
    v = SimpleNamespace(armed=False, mode='', depth_m=math.nan, yaw_deg=math.nan,
                        battery_voltage=math.nan)
    assert 'disarmed' not in render(120, v, {}, [])[0]
    v.mode = 'MANUAL'
    assert 'disarmed' in render(120, v, {}, [])[0]
    out = render(120, None, {}, [json.dumps({'op': 'x', 'ok': True})])
    assert any(line.startswith('--:--:--') for line in out)


def test_a_command_is_ok_only_once_the_manager_accepts_it():
    from mongla_batighor.deck import accepted_entry

    class _F:
        def __init__(self, handle=None, exc=None):
            self.h, self.e = handle, exc

        def result(self):
            if self.e:
                raise self.e
            return self.h
    refused = accepted_entry(_F(SimpleNamespace(accepted=False)), 'move_forward', '{}')
    took = accepted_entry(_F(SimpleNamespace(accepted=True)), 'move_forward', '{}')
    lost = accepted_entry(_F(exc=RuntimeError('gone')), 'move_forward', '{}')
    assert refused['ok'] is False and 'refused' in refused['message']
    assert took['ok'] is True and lost['ok'] is False
    sent = render(200, None, {}, [json.dumps({'t': 0, 'op': 'command', 'cmd': 'stop',
                                              'ok': None, 'message': 'sent'})])
    assert not any(' ok' in line.split('command stop')[-1] for line in sent if 'command stop' in line)
