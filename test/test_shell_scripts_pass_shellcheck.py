"""Every tracked shell script passes shellcheck at warning level.

The scripts here run the vehicle's deploy, backup and doctor (`kutum/`), the
build, and the pool-day recorder. A bash mistake in those shows up on the day,
on the vehicle, as a deploy that half-ran or a bag that never finalised. On
2026-11-20 the first run found five, none yet a failure: an `export` that hid
its command's exit status (SC2155) in the build, three unused loop variables,
and a sourced script with no shell named. An exception is a
`# shellcheck disable=SCxxxx` on the line, with the reason.

    shellcheck -S warning $(git ls-files '*.sh')
"""
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_every_script_passes_shellcheck():
    exe = shutil.which('shellcheck')
    if not exe:
        pytest.skip('shellcheck is not installed (the integrity gate installs shellcheck-py)')
    files = subprocess.run(['git', 'ls-files', '*.sh'], cwd=ROOT, capture_output=True,
                           text=True, check=True).stdout.split()
    assert len(files) > 10, f'only {len(files)} scripts found: wrong root?'
    got = subprocess.run([exe, '-S', 'warning', '-f', 'gcc'] + files, cwd=ROOT,
                         capture_output=True, text=True)
    assert got.returncode == 0, 'shellcheck findings:\n' + got.stdout[-6000:] + got.stderr[-2000:]
