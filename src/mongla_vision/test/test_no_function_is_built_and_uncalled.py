"""The same rule as `test_no_capability_is_built_and_unreachable`, one level down.

That test checks modules. A dead FUNCTION inside a module that is imported,
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

Why `--private` is the mode that must read zero. Nothing outside a module can
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
_TOOL = _ROOT / 'tools' / 'ci' / 'dead_function_sweep.py'


# Functions that only a test calls, on purpose, each with the consumer it
# waits for. Same contract as the module registers: a row is a debt with a
# name, not a silent gap. Until 2026-11-06 the sweep counted a TEST as a
# caller, so every one of these read as live -- and so did ten that were
# deleted that day (`pct_to_mc`, "the old percent API", among them) and two
# board checks that were wired into the manager's preflight
# (`check_yaw_authority`, `boot_warnings`), written and never called.
DEFERRED_FUNCTIONS = {
    'src/mongla_control/mongla_control/actuation_model.py:fraction_to_demand':
        'the inverse of demand_to_fraction, for a caller that plans in OUTPUT '
        'space; every verb today commands demand, so it waits for the first '
        'controller that asks for a known thrust fraction',
    'src/mongla_control/mongla_control/allocation.py:limiting_group':
        'names which authority saturated; waits for an operator readout of the '
        'host mixer model (the board mixes, so this is a model, not a reading)',
    'src/mongla_control/mongla_control/allocation.py:headroom':
        'mixer headroom before clipping; the lateral I-term in motion_vision '
        'clamps on COMMAND saturation instead, and would consult this only once '
        'the mixer model is validated against the hull being built',
    'src/mongla_control/mongla_control/fc/srot_bench.py:motor_tune':
        'the board\'s MOTOR_TUNE; same shape as autotune(), which has '
        '`ros2 run mongla_manager autotune`. It needs the same entry point, and '
        'its first run needs a hull with thrusters fitted (none are)',
    'src/mongla_control/mongla_control/fc/srot_health.py:thruster_stalled':
        'signed-RPM stall detection; needs non-zero ESC RPM on the wire, and '
        '958/958 frames read exactly 0 with nothing attached (CLAUDE.md §2). '
        'Its consumer is the manager health report once thrusters are fitted',
    'src/mongla_localization/mongla_localization/course_map.py:position_of':
        'refusing lookup of a surveyed prop position; waits for the first '
        'mission or fix that resolves a prop by name rather than iterating',
    'src/mongla_localization/mongla_localization/pool_lines.py:cross_track_m':
        'metres from a lane line; needs a height source on the lane-line path, '
        'which the tile grating provides once lane_lines and the grating run '
        'together in water',
    'src/mongla_localization/mongla_localization/tile_grating.py:usable_height_m':
        'the instrument envelope; waits for the tile size of the venue pool, '
        'which is a measurement on the day',
    'src/mongla_localization/mongla_localization/tile_grating.py:shift_from_phase':
        'phase-based displacement, the grating as a velocity sensor; blocked '
        'on the same pool-floor footage as usable_height_m',
    'src/mongla_manager/mongla_manager/health_reporters.py:target_lock':
        'the lock ladder as a health row; needs the ladder\'s decayed authority, '
        'and /lock carries a confidence instead -- an accessor on VisionState '
        'first (auv_manager_node, beside the health registrations)',
    'src/mongla_planner/mongla_planner/resilience.py:retry':
        'mission-author API: missions live in ~/missions, deliberately out of '
        'git, so an in-repo caller is not expected',
}


def _key(line: str) -> str:
    path, _ln, rest = line.split(':', 2)
    return f"{path}:{rest.strip().split('(')[0]}"


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
    """The enforced half. An underscore name is unreachable from outside its
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
    """Reported, not policed, except that it must not GROW silently. The two
    remaining entries live in modules `orphan_sweep` already lists with the
    consumer they wait for, so the sweep files them separately. A third entry
    appearing here is a question to answer, not automatically a defect -- but
    it must be answered rather than accumulate."""
    out = _run()
    unexplained = [ln for ln in out.splitlines()
                   if ln.startswith('src/') and _key(ln) not in DEFERRED_FUNCTIONS]
    assert not unexplained, (
        'function(s) with no caller and no deferred module to explain them:\n  '
        + '\n  '.join(unexplained) +
        '\nName the caller out loud. If there is none it gets one now or it '
        'goes. `fw_version` turned out to be the CORRECT implementation beside '
        'a buggy inline copy, so check for a duplicate before deleting.')


def test_the_sweep_catches_a_planted_dead_function(tmp_path):
    """Injection-verify the guard itself. A sweep that has never found a real
    defect is not a guard.

    The plant goes in a copy, and that is not tidiness. The first version
    wrote into a real module and undid it in a `finally` --
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
            / 'stamps.py')
    shutil.copy(real, pkg / 'stamps.py')

    clean = _run('--private', root=root)
    assert '_planted' not in clean, 'the copied tree is not clean to begin with'

    (pkg / 'stamps.py').write_text(
        real.read_text()
        + '\n\ndef _planted_and_never_called(x):\n    return x\n')
    out = _run('--private', root=root)
    assert '_planted_and_never_called' in out, \
        'the sweep did not notice a function with no caller at all'

    # and the REAL tree is untouched by any of it
    assert '_planted' not in real.read_text()


def test_a_deferred_function_that_got_a_caller_is_dropped_from_the_register():
    """Once something on the vehicle calls it, the row is fiction: drop it."""
    live = {_key(ln) for ln in _run().splitlines() if ln.startswith('src/')}
    stale = sorted(set(DEFERRED_FUNCTIONS) - live)
    assert not stale, f'now called (or gone) -- drop from DEFERRED_FUNCTIONS: {stale}'


def test_a_method_is_not_called_by_a_field_or_an_override_of_its_name(tmp_path):
    """The two shapes the sweep used to pass, both found 2026-11-18.

    `ConfidenceModel.authority()` read as called because a lock state has a
    FIELD `authority` that is read everywhere; `Detector.class_names()` was
    skipped as an override because both detectors implemented it, while
    nothing in the repository ever called it. A real call clears each.
    """
    root = tmp_path / 'tree'
    pkg = root / 'src' / 'mongla_vision' / 'mongla_vision'
    pkg.mkdir(parents=True)
    (root / 'tools').mkdir()
    plant = pkg / 'stamps.py'
    real = (_ROOT / 'src' / 'mongla_vision' / 'mongla_vision' / 'stamps.py')
    body = real.read_text() + '''

class _State:
    authority: float = 0.0


def read_state(s):
    return s.authority


class _Model:
    def authority(self):
        return 1.0


class _A:
    def class_names(self):
        return {}


class _B:
    def class_names(self):
        return {}
'''
    plant.write_text(body)
    out = _run(root=root)
    assert 'authority()' in out, 'a field read passed for a call'
    assert 'class_names()' in out, 'an uncalled interface passed as an override'

    plant.write_text(body + '\n\ndef use(m, a):\n    return m.authority(), a.class_names()\n')
    out = _run(root=root)
    assert 'authority()' not in out and 'class_names()' not in out
