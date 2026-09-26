"""The same rule as `test_no_capability_is_built_and_unreachable`, one level down.

⛔ THAT TEST CHECKS MODULES. A dead FUNCTION inside a module that is imported,
tested and running is invisible to it -- and to the topic sweep and the launch
sweep, which work on the ROS graph and on launch files. Found this way, every
one real:

  * `pool_lines._dominant_angle` -- 50 lines of correct axial statistics, born
    with the file and called by nothing ever, residue of a first design the
    module's own docstring records as rejected;
  * `tile_grating._annulus_values` -- same shape, no docstring;
  * `srot_format.fw_version` -- NOT residue: the renderer that needed it
    unpacked the bit-fields itself and printed `Hengla v0.0.0` for an
    unanswered version request;
  * `actuation_model.dshot_command` -- a host reimplementation of firmware
    arithmetic the bench already reaches through the firmware's own
    `one_to_dshot`;
  * `tracing.is_enabled`, `SrotFC.read_gain`, `Mongla.payload_ready`,
    `ClockMap.residual_ms`, `TimeOffset.peak_correlation`, `WaterStats.as_row`
    -- each an accessor whose live path either duplicated it inline or hid the
    number it exposes.

⚠ WHY `--private` IS THE MODE THAT MUST READ ZERO. Nothing outside a module can
reach an underscore name, so a private function with no caller in its own
package is a certainty, not a question. Public names have legitimate reasons to
look unused -- the mission DSL is called from scripts that are deliberately not
in git, `COMMANDS` dispatches facade verbs by dict key, and a ROS callback is
passed by reference -- and the tool knows about those, but the judgement there
is still a human one. So this test enforces the private mode, and
also fails on a PUBLIC hit that no deferred module explains -- the sweep files
those separately, so what reaches the assert is the list a human still owes an
answer for. `fw_version` is the reason the answer is not automatically
"delete": it was the CORRECT implementation sitting beside a buggy inline
copy.
"""
from __future__ import annotations

import os
import pathlib
import shutil
import subprocess
import sys

import pytest

_ROOT = pathlib.Path(__file__).resolve().parents[3]
_TOOL = _ROOT / 'tools' / 'dead_function_sweep.py'


def _run(*args, root=None):
    env = dict(os.environ)
    if root is not None:
        env['MONGLA_SWEEP_ROOT'] = str(root)
    r = subprocess.run([sys.executable, str(_TOOL), *args],
                       capture_output=True, text=True, cwd=_ROOT, env=env)
    assert r.returncode == 0, r.stdout + r.stderr
    return r.stdout


def test_the_sweep_exists_and_runs():
    """A guard whose tool has been moved or renamed silently stops guarding."""
    assert _TOOL.is_file()
    assert 'referenced' in _run('--private')


def test_no_private_function_is_uncalled():
    """⛔ THE ENFORCED HALF. An underscore name is unreachable from outside its
    module, so one with no caller is dead with certainty -- there is no
    external consumer it could be waiting for."""
    out = _run('--private')
    hits = [ln for ln in out.splitlines() if ln.startswith('src/')]
    assert not hits, (
        'private function(s) with no caller:\n  ' + '\n  '.join(hits) +
        '\nEither give it a caller or delete it -- CLAUDE.md section 9 has no '
        'third state, and a private name cannot be waiting for anything '
        'outside its own module.')


def test_the_public_list_is_only_the_two_deferred_modules():
    """⚠ REPORTED, NOT POLICED, except that it must not GROW silently. The two
    remaining entries live in modules `orphan_sweep` already lists with the
    consumer they wait for, so the sweep files them separately. A third entry
    appearing here is a question to answer, not automatically a defect -- but
    it must be answered rather than accumulate."""
    out = _run()
    unexplained = [ln for ln in out.splitlines() if ln.startswith('src/')]
    assert not unexplained, (
        'function(s) with no caller and no deferred module to explain them:\n  '
        + '\n  '.join(unexplained) +
        '\nName the caller out loud. If there is none it gets one now or it '
        'goes. `fw_version` turned out to be the CORRECT implementation beside '
        'a buggy inline copy, so check for a duplicate before deleting.')


def test_the_sweep_catches_a_planted_dead_function(tmp_path):
    """⛔ INJECTION-VERIFY THE GUARD ITSELF. A sweep that has never found a real
    defect is not a guard.

    ⚠ THE PLANT GOES IN A COPY, and that is not tidiness. The first version
    wrote into the real `ladder_config.py` and undid it in a `finally` --
    which does not run on SIGTERM, and this suite is run under `timeout`. A
    killed run would have left the plant in the source and failed `--private`
    on every run afterwards. It is also not concurrency-safe: a second sweep
    running at that moment sees the plant and reports a defect that does not
    exist. That was observed, once, before this was changed.
    """
    root = tmp_path / 'tree'
    pkg = root / 'src' / 'mongla_vision' / 'mongla_vision'
    pkg.mkdir(parents=True)
    (root / 'tools').mkdir()
    real = (_ROOT / 'src' / 'mongla_vision' / 'mongla_vision'
            / 'ladder_config.py')
    shutil.copy(real, pkg / 'ladder_config.py')

    clean = _run('--private', root=root)
    assert '_planted' not in clean, 'the copied tree is not clean to begin with'

    (pkg / 'ladder_config.py').write_text(
        real.read_text()
        + '\n\ndef _planted_and_never_called(x):\n    return x\n')
    out = _run('--private', root=root)
    assert '_planted_and_never_called' in out, \
        'the sweep did not notice a function with no caller at all'

    # and the REAL tree is untouched by any of it
    assert '_planted' not in real.read_text()
