"""Every console script in every package must at least IMPORT.

⛔ THE DEFECT THIS CLOSES, FOUND BY ACCIDENT. `underwater.recommend()` was
deleted on purpose -- it mapped frame statistics onto a CLAHE verdict using
thresholds fitted to two clips, and re-measuring across 17 configurations
showed preprocessing was never positive. The deletion was right and the
reasoning was written where the function used to be.

`utils/water_check.py` kept importing it. So `ros2 run mongla_vision
water_check` died with `ImportError: cannot import name 'recommend'` BEFORE
parsing an argument -- a shipped console entry point, dead on arrival, for as
long as nobody happened to run it. No test imported an entry point, so the
whole class was invisible: `test_no_capability_is_built_and_unreachable.py`
checks that modules are reachable, not that reachable ones load.

⚠ IMPORT ONLY, NOT EXECUTION. Running these needs a board, a camera or a ROS
graph. But an entry point that cannot be imported can never run anywhere, and
that is decidable here in milliseconds.

⚠ A MISSING THIRD-PARTY DEPENDENCY IS NOT A FAILURE. `ultralytics`, `torch`,
`hailo_platform` and `cv_bridge` are absent on plenty of dev boxes; the test
skips those and fails on a name that does not exist in OUR code.
"""
from __future__ import annotations

import ast
import importlib
import pathlib
import re

import pytest

_ROOT = pathlib.Path(__file__).resolve().parents[3]
# Third-party modules a dev box legitimately lacks. A missing one of these says
# nothing about the entry point.
_OPTIONAL = ('ultralytics', 'torch', 'hailo', 'cv_bridge', 'supervision',
             'tflite', 'onnx', 'rclpy', 'serial', 'pymavlink', 'yaml',
             'scipy', 'matplotlib')


def _entry_points():
    out = []
    for setup in sorted(_ROOT.glob('src/*/setup.py')):
        txt = setup.read_text()
        for line in re.findall(r"'([^']+=[^']+)'", txt):
            if '=' not in line or ':' not in line:
                continue
            name, target = line.split('=', 1)
            mod = target.strip().split(':')[0].strip()
            if mod.startswith('mongla'):
                out.append((setup.parent.name, name.strip(), mod))
    return out


def test_there_are_entry_points_to_check():
    """A sweep that silently finds nothing is worse than no sweep."""
    eps = _entry_points()
    assert len(eps) > 20, f'only found {len(eps)} entry points'


@pytest.mark.parametrize('pkg,name,module',
                         _entry_points(),
                         ids=lambda v: v if isinstance(v, str) else '')
def test_the_entry_point_imports(pkg, name, module):
    try:
        mod = importlib.import_module(module)
    except ImportError as exc:
        missing = str(exc)
        if any(o in missing for o in _OPTIONAL):
            pytest.skip(f'{module}: optional dependency absent ({missing})')
        raise AssertionError(
            f'`ros2 run {pkg} {name}` cannot even be imported: {missing}. '
            f'This is what `water_check` did for as long as nobody ran it.'
        ) from exc
    assert hasattr(mod, 'main'), \
        f'{module} is an entry point with no main()'


def test_the_deleted_recommend_stays_deleted():
    """⛔ The specific name, because the deletion carries a measurement: the
    CLAHE rule was fitted to two clips and is negative on the third venue."""
    from mongla_vision import underwater

    assert not hasattr(underwater, 'recommend'), (
        'recommend() is back. Preprocessing measured NEVER positive across 17 '
        'configurations; on the gate it took detection 30.4 % -> 1.2 %.')
