"""`model.sdf` and the URDF are GENERATED. Nothing was checking they were current.

`configs.yaml` is the single source of truth for the vehicle -- mass, inertia,
added mass, drag, camera placement, thruster geometry, the buoyancy collision
box -- and both `model.sdf` and `urdf/duburi_heavy.urdf` are derived from it by
scripts that are **run by hand**. `CMakeLists.txt` only installs; there is no
`add_custom_command`, so a `configs.yaml` edit that is not followed by a manual
generator run **ships silently stale**, and the file's own header says not to
hand-edit the outputs.

That is this project's most expensive recurring failure: a value read from a
place that no longer agrees with the value that is used. It has cost a round
each time (the two-opening board the scorer still graded against, the bin box at
the model origin, the roughness map used as an albedo). Here it is cheap to
close -- regenerate into a temp dir and diff.

The guard matters most exactly when someone is editing `configs.yaml`, which is
when it is easiest to forget.
"""

import os
import subprocess
import sys
import tempfile

import pytest

DESC = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'duburi_sim_description'))
MODEL = os.path.join(DESC, 'models', 'duburi_heavy')
SCRIPTS = os.path.join(DESC, 'scripts')

CONFIG = os.path.join(MODEL, 'configs.yaml')
TEMPLATE = os.path.join(MODEL, 'model.sdf.in')
SDF = os.path.join(MODEL, 'model.sdf')
URDF = os.path.join(DESC, 'urdf', 'duburi_heavy.urdf')

pytestmark = pytest.mark.skipif(
    not os.path.isfile(CONFIG), reason='duburi_sim_description not present')


def _run(script, *args):
    r = subprocess.run([sys.executable, os.path.join(SCRIPTS, script), *args],
                       capture_output=True, text=True)
    assert r.returncode == 0, f'{script} failed:\n{r.stderr}'


def _diff(generated: str, committed: str, regen_cmd: str):
    with open(generated) as fh:
        fresh = fh.read()
    with open(committed) as fh:
        shipped = fh.read()
    if fresh == shipped:
        return
    import difflib
    delta = '\n'.join(list(difflib.unified_diff(
        shipped.splitlines(), fresh.splitlines(),
        'committed', 'regenerated', lineterm=''))[:40])
    pytest.fail(
        f'{os.path.basename(committed)} is STALE with respect to '
        f'configs.yaml.\n\nRegenerate it:\n    {regen_cmd}\n\n{delta}')


def test_the_committed_sdf_matches_configs_yaml():
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, 'model.sdf')
        _run('generate_model.py', TEMPLATE, out, CONFIG)
        _diff(out, SDF,
              'cd sim/src/duburi_sim_description && scripts/generate_model.py '
              'models/duburi_heavy/model.sdf.in models/duburi_heavy/model.sdf '
              'models/duburi_heavy/configs.yaml')


def test_the_committed_urdf_matches_configs_yaml():
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, 'duburi_heavy.urdf')
        _run('generate_urdf.py', CONFIG, out)
        _diff(out, URDF,
              'cd sim/src/duburi_sim_description && scripts/generate_urdf.py '
              'models/duburi_heavy/configs.yaml urdf/duburi_heavy.urdf')
