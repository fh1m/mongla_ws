"""B22 -- the water refractive index has ONE source, and the two Snell
transforms stay inverses when it changes.

⛔ READ THIS BEFORE EDITING. The obvious test -- "air -> water -> air round-trips"
-- PASSES ON THE BROKEN CODE. The defect was that `solver.fov_for_medium` (the
inverse) hardcoded the literal `1.333` while `refract` (the forward) read
`N_WATER`, and those two numbers are equal today. A round-trip at the shipped
index therefore cannot tell the fixed code from the broken code.

The test that bites varies the constant. That is also why the transforms read
`N_WATER` at CALL time rather than as a default argument.
"""

import math

import pytest

from mongla_vision import optics


SALT = 1.34   # deliberately NOT the shipped 1.333


def test_known_physical_value():
    """An 80 deg air lens is ~57.7 deg underwater -- the number the docs quote."""
    assert optics.fov_air_to_water(80.0) == pytest.approx(57.66, abs=0.01)


@pytest.mark.parametrize('fov', [30.0, 57.7, 80.0, 120.0])
def test_transforms_are_inverses_at_the_shipped_index(fov):
    assert optics.fov_water_to_air(optics.fov_air_to_water(fov)) == pytest.approx(fov, abs=1e-9)


@pytest.mark.parametrize('fov', [30.0, 80.0, 120.0])
def test_transforms_are_inverses_at_a_DIFFERENT_index(fov, monkeypatch):
    """The biting case: both directions must read the SAME constant.

    With the index hardcoded on one side, this fails while the test above passes.
    """
    monkeypatch.setattr(optics, 'N_WATER', SALT)
    assert optics.fov_water_to_air(optics.fov_air_to_water(fov)) == pytest.approx(fov, abs=1e-9)
    # ...and the index must actually have taken effect, or the assert above is vacuous.
    assert optics.fov_air_to_water(80.0) != pytest.approx(57.66, abs=0.01)


def test_solver_forward_and_inverse_share_the_index(monkeypatch):
    """The actual B22 regression, through the two functions that disagreed.

    `solver.refract` (air->water) read N_WATER; `solver.fov_for_medium`
    (water->air) hardcoded 1.333. Under a changed index they stop being
    inverses, and the symptom is a plausible, self-inconsistent FOV.
    """
    solver = pytest.importorskip('mongla_vision.calibration.solver',
                                 reason='solver needs cv2')
    monkeypatch.setattr(optics, 'N_WATER', SALT)

    hfov_air, vfov_air = 80.0, 64.0
    # Forward: what the lens becomes in water. `refract` is a local inside
    # fov_from_K, so go through the same transform solver now delegates to.
    v = {'hfov_air': optics.fov_air_to_water(hfov_air),
         'vfov_air': optics.fov_air_to_water(vfov_air)}
    # Inverse: fov_for_medium is handed IN-WATER numbers under key `*_air` (that
    # is its contract -- the measurement was taken in water) and derives air.
    out, note = solver.fov_for_medium(v, 'water')

    assert out['hfov_air'] == pytest.approx(hfov_air, abs=1e-9), \
        'solver forward/inverse read different refractive indices (B22)'
    assert out['vfov_air'] == pytest.approx(vfov_air, abs=1e-9)
    assert note, 'a derived air figure must carry the "not measured" note'


def test_no_live_literal_copies_of_the_index_remain():
    """A second copy of a physical constant is how B22 happened."""
    import ast
    import pathlib
    root = pathlib.Path(optics.__file__).resolve().parent
    offenders = []
    for f in root.rglob('*.py'):
        if f.name == 'optics.py' or f.parts[-2:][0] == 'test':
            continue
        try:
            tree = ast.parse(f.read_text(encoding='utf-8'))
        except SyntaxError:                                   # pragma: no cover
            continue
        # AST, not a text scan: prose mentioning 1.333 in a docstring or an
        # operator-facing help string is documentation, not a second copy of the
        # constant. Only a NUMERIC literal is the defect.
        for node in ast.walk(tree):
            if isinstance(node, ast.Constant) and node.value == 1.333:
                offenders.append(f'{f.relative_to(root)}:{node.lineno}')
    assert not offenders, 'numeric literal 1.333 in live code; import N_WATER:\n' + '\n'.join(offenders)
