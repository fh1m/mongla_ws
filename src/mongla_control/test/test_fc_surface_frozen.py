"""The set of Pixhawk-only methods called on an FC is FROZEN. A new one must be justified.

⛔ WHY. B30 was a call to `send_rc_override` on a `SrotFC`, reached through a
lambda in a dataclass field. It was found by accident, chasing an unrelated
contradiction, after shipping. The class of bug is "a backend-specific primitive
reached on a path the other backend also takes", and nothing enumerated that
class.

This freezes it. Each name below is a method that exists on PixhawkFC and NOT on
SrotFC, and each has been checked this session to be unreachable on srot -- the
verb is collapsed onto the board (MOVE_VERBS), refused (UNSUPPORTED_VERBS), or the
call sits inside an `is_srot` guard.

ADDING A NAME HERE IS A DECISION, NOT A CHORE. Before you do, prove the site
cannot run on srot (`tools/srot_reachability.py`), or give SrotFC an
implementation, or guard the call. The list is the point: it converts "someone
notices eventually" into "the suite fails now".
"""

import ast
import pathlib
import subprocess
import sys

import pytest

from mongla_control.fc.base import FlightController
from mongla_control.fc.srot_fc import SrotFC

# name -> why it cannot run on srot (checked 2026-09-08)
KNOWN_PIXHAWK_ONLY = {
    'send_rc_override':    'pause/style_roll are MOVE_VERBS (board); motion_* verbs '
                           'collapsed or refused; motion_vision/motion_writers sites are '
                           'inside the non-srot branch',
    'send_rc_translation': 'motion_vision + motion_writers, non-srot branch only',
    'send_rc_yaw_only':    'heading_lock -- lock_heading is in UNSUPPORTED_VERBS',
    'release_rc_override': 'mongla.pause -- MOVE_VERBS, collapsed onto the board',
    'set_target_depth':    'motion_depth (set_depth is MOVE_VERBS) and align_loop, '
                           'where vision_verbs refuses `touches_depth or is_downward` '
                           'on srot -- the exact condition gating stream_depth',
}


def _repo_root():
    return pathlib.Path(__file__).resolve().parents[3]


def _run_audit():
    root = _repo_root()
    tool = root / 'tools' / 'fc_surface_audit.py'
    if not tool.is_file():
        pytest.skip(f'{tool} not present in this checkout')
    out = subprocess.run([sys.executable, str(tool)], cwd=root,
                         capture_output=True, text=True, timeout=120)
    assert out.returncode == 0, f'audit tool failed:\n{out.stderr[-2000:]}'
    found = set()
    for line in out.stdout.splitlines():
        if line.startswith('  ') and not line.startswith('      ') and '(' in line:
            found.add(line.strip().split()[0])
    return found, out.stdout


def test_no_unknown_pixhawk_only_method_is_called_on_an_fc():
    found, report = _run_audit()
    new = found - set(KNOWN_PIXHAWK_ONLY)
    assert not new, (
        f'NEW backend-only method(s) called on a flight controller: {sorted(new)}.\n'
        f'This is B30\'s shape. Prove the site cannot run on srot '
        f'(tools/srot_reachability.py), implement it on SrotFC, or guard the call '
        f'with is_srot() -- then add it to KNOWN_PIXHAWK_ONLY with the reason.\n\n'
        f'{report}')


def test_the_frozen_list_has_not_gone_stale():
    """A name that is no longer called should leave the list, or it rots into noise."""
    found, _ = _run_audit()
    gone = set(KNOWN_PIXHAWK_ONLY) - found
    assert not gone, (
        f'{sorted(gone)} are in KNOWN_PIXHAWK_ONLY but no longer called anywhere. '
        f'Remove them -- a frozen list that describes the past stops being read.')


def test_every_frozen_name_really_is_absent_from_srot():
    """The premise itself: these are on PixhawkFC and NOT on SrotFC or the ABC.

    If one of these ever lands on SrotFC, the entry is obsolete and the guard
    around it may be dead code worth deleting.
    """
    for name in KNOWN_PIXHAWK_ONLY:
        assert not hasattr(SrotFC, name), (
            f'SrotFC now implements {name!r} -- drop it from KNOWN_PIXHAWK_ONLY '
            f'and re-check whether its is_srot guard is still needed')
        assert not hasattr(FlightController, name), \
            f'{name!r} is on the ABC, so it is not backend-specific'
