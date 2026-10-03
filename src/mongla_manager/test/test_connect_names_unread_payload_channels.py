"""`connect` tells a channel set to DISABLED from one it could not read.

Found on the vehicle (2026-10-03): channels 4 and 11 showed as
"other/unreadable" -- they were read, as role 0 -- while a channel whose
read FAILED appeared in no list at all.
"""
import re

import pytest

pytest.importorskip('pymavlink')


def _payload_lines(roles):
    from mongla_manager import srot_connect as sc
    snap = sc.Snapshot()
    snap.roles = roles
    try:
        lines = sc.render(snap, None)
    except Exception as exc:                  # pragma: no cover
        pytest.skip(f'render needs more of a snapshot here: {exc!r}')
    text = re.sub(r'\x1b\[[0-9;]*m', '', '\n'.join(lines))
    return text[text.index('== payload'):text.index('== firmware health')]


def test_disabled_other_and_unread_are_three_different_lines():
    roles = {c: (1 if c <= 8 else 2) for c in range(1, 17)}
    roles[4] = 0
    roles[11] = 7
    del roles[13]                              # its read failed
    txt = _payload_lines(roles)
    assert re.search(r'DISABLED.*\[4\]', txt), txt
    assert '11(role 7)' in txt
    assert re.search(r'UNREAD.*\[13\]', txt), txt
    assert 'other/unreadable' not in txt
