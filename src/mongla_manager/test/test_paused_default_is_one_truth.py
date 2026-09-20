"""`paused` must default the same way everywhere, and mean what it says.

⛔ THE DEFECT THIS FIXES, measured on the vehicle. `vision_pi.launch.py` --
the VEHICLE launch, the one `bringup.launch.py` includes for a mission --
declared:

    DeclareLaunchArgument('paused', default_value='false',
        description='Start both detectors paused. ... leaving BOTH live makes
                     them compete for the chip (~35 Hz each instead of ~98).')

The description and the default contradicted each other INSIDE ONE CALL, and
they were written in the same commit (`ba44858`), so it was never a deliberate
flip. `vision_dual.launch.py` (the dev twin) defaulted to 'true' all along, and
`test_camera_exclusivity.py` calls `paused:=false` "a stray launch" -- three
files agreeing against the one that actually runs on the vehicle.

Cost, measured: two resident models alternating on the Hailo run 37.5 Hz per
pair against 95.3 Hz for a single resident model, and the chip logs
`has taken the activation 20 times` in a two-detector run.

The guard is narrow on purpose. It does not try to read every description --
it pins the ONE argument whose default was wrong, so the next edit to either
launch cannot silently re-split them.
"""
from __future__ import annotations

import ast
import pathlib

_SRC = pathlib.Path(__file__).resolve().parents[2]


def _paused_defaults() -> dict[str, str]:
    """{launch file name: default_value} for every declared `paused` arg."""
    out: dict[str, str] = {}
    for path in sorted(_SRC.glob('*/launch/*.launch.py')):
        for node in ast.walk(ast.parse(path.read_text())):
            if not (isinstance(node, ast.Call)
                    and getattr(node.func, 'id', '') == 'DeclareLaunchArgument'):
                continue
            if not (node.args and isinstance(node.args[0], ast.Constant)
                    and node.args[0].value == 'paused'):
                continue
            for kw in node.keywords:
                if kw.arg == 'default_value' and isinstance(kw.value, ast.Constant):
                    out[path.name] = kw.value.value
    return out


# Every launch that declares `paused`, with the default it MUST carry and why.
# FROZEN: a new launch declaring `paused` fails the completeness test below
# rather than quietly picking its own side of the argument.
_EXPECTED: dict[str, tuple[str, str]] = {
    # --- the mission path: one live detector, because one chip ---
    'vision_pi.launch.py':   ('true',  'vehicle dual-detector launch; two live '
                                       'models alternate on the Hailo'),
    'vision_dual.launch.py': ('true',  'dev twin of vision_pi; same invariant'),
    'bringup.launch.py':     ('true',  'forwards to vision_pi'),
    # --- surfaces where BOTH live is the point, and no mission is steering ---
    'vision.launch.py':      ('false', 'single camera: one detector, nothing to '
                                       'contend with'),
    'mission_web.launch.py': ('false', 'operator console: both panels live IS '
                                       'the product'),
    'video.launch.py':       ('false', 'offline dataset replay, not a mission'),
}


def test_every_paused_default_matches_the_frozen_table():
    defaults = _paused_defaults()
    wrong = {n: (v, _EXPECTED[n][0], _EXPECTED[n][1])
             for n, v in defaults.items()
             if n in _EXPECTED and v != _EXPECTED[n][0]}
    assert not wrong, f'`paused` default disagrees with its reason: {wrong}'


def test_the_table_covers_every_launch_that_declares_paused():
    # Completeness, so a NEW launch cannot pick a side unreviewed.
    missing = sorted(set(_paused_defaults()) - set(_EXPECTED))
    assert not missing, (
        f'these launches declare `paused` and are not in the table: {missing}')


def test_the_vehicle_launch_is_among_them():
    # Named explicitly: this is the file that had it backwards, and a rename
    # that dropped it from the glob would make the test above vacuously pass.
    assert 'vision_pi.launch.py' in _paused_defaults()


def test_bringup_forwards_paused_to_the_pi_stack():
    """Declaring it in bringup is not enough -- the include must pass it.

    `IncludeLaunchDescription` drops an undeclared key silently, and an
    un-forwarded one just leaves the child on its own default. Either way the
    operator's `paused:=false` would do nothing, with no log line.
    """
    src = (_SRC / 'mongla_manager' / 'launch' / 'bringup.launch.py').read_text()
    tree = ast.parse(src)
    forwarded = False
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call)
                and getattr(node.func, 'id', '') == 'IncludeLaunchDescription'):
            continue
        for kw in node.keywords:
            if kw.arg != 'launch_arguments':
                continue
            for sub in ast.walk(kw.value):
                if isinstance(sub, ast.Constant) and sub.value == 'paused':
                    forwarded = True
    assert forwarded, 'bringup.launch.py never passes `paused` to an include'
