"""A node that reads the camera matrix off `CameraInfo` reads the lens with it.

`camera_node` publishes K and D. A consumer that takes K alone treats a
distorted pixel as a pinhole one: on the forward lens (k1 = -0.363) that is a
3.5 to 7 degree bearing error in the outer third of the frame, with nothing in
any log to show it. Three consumers did exactly that (issue #42).

Rectifying the image once at the camera would remove the class, and was
measured and not adopted (measured-bars 75): 4.68 ms a frame at 1280x720
against a 4 ms bar, where removing the lens from the few points a consumer
uses costs 3.7 microseconds. So the lens comes off per consumer, and this
guard is what makes forgetting it a failed check instead of a biased bearing.
"""
from __future__ import annotations

import ast
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / 'src'

#: Modules that read K off a CameraInfo and legitimately never read D.
K_ONLY = {
}


def _reads(tree: ast.AST, attr: str) -> bool:
    return any(isinstance(n, ast.Attribute) and n.attr == attr
               and isinstance(n.ctx, ast.Load) for n in ast.walk(tree))


def readers() -> dict[str, bool]:
    """{module: reads D} for every non-test module that reads `CameraInfo.k`."""
    out = {}
    for path in sorted(SRC.glob('*/*/**/*.py')):
        rel = path.relative_to(SRC).as_posix()
        if '/test/' in f'/{rel}' or '/launch/' in f'/{rel}':
            continue
        text = path.read_text()
        if 'CameraInfo' not in text:
            continue
        tree = ast.parse(text)
        if _reads(tree, 'k'):
            out[rel] = _reads(tree, 'd')
    return out


def test_the_scan_finds_the_consumers_we_know():
    found = readers()
    for known in ('mongla_localization/mongla_localization/pnp_node.py',
                  'mongla_vision/mongla_vision/lock_inputs.py',
                  'mongla_planner/mongla_planner/dsl_detections.py',
                  'mongla_manager/mongla_manager/vision_state.py'):
        assert known in found, f'{known} reads CameraInfo.k and the scan missed it'


def test_every_reader_of_K_reads_D():
    bare = [m for m, has_d in readers().items() if not has_d and m not in K_ONLY]
    assert not bare, (
        f'{", ".join(bare)}: reads CameraInfo.k and never CameraInfo.d, so it '
        'treats a distorted pixel as a pinhole one. Remove the lens first '
        '(mongla_control.bearing, or mongla_vision.optics.rectifier_for), or '
        'add the module to K_ONLY with the reason it needs no lens.')


def test_every_exemption_still_exists():
    stale = [m for m in K_ONLY if m not in readers()]
    assert not stale, f'{stale}: exempt here and no longer a reader of K'
