"""Kutum answers as kutum.local, and nothing tells an operator otherwise.

The Pi was renamed from its imaged name on 2026-11-18 (`kutum/setup.sh`
sets it, `kutum/doctor.sh` fails any other). A command left naming the old
host is a dead end typed under pressure, so the old name may appear only
where it is history.
"""
from __future__ import annotations

import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OLD = 'mongla' + '.local'           # split, or this file would name it

# Each exemption is history, with its reason.
HISTORY = {
    '.claude/context/BUGS.md': 'defects as they were seen, under the name of the day',
    '.claude/context/workbench/data/board_params_20260923.json': 'a dated capture',
}


def test_no_live_file_names_the_old_host():
    # The host name, not a prefix of it: `mongla.localization()` is a DSL call.
    files = subprocess.run(['git', 'grep', '-l', '-P', OLD.replace('.', r'\.') + r'(?![A-Za-z_])'], cwd=ROOT,
                           capture_output=True, text=True).stdout.split()
    stale = [f for f in files if f not in HISTORY]
    assert not stale, f'{OLD} is now kutum.local: {stale}'


def test_every_exemption_still_names_it():
    for path in HISTORY:
        assert OLD in (ROOT / path).read_text(), f'{path} no longer needs its exemption'
