"""The shutter must be pinned, and the blur cap must be the right arithmetic.

⛔ WHY. `webcam.py` never touched exposure, so both mission cameras ran on
whatever the driver's auto picked. Measured on the forward Fantech that was
Aperture Priority at `exposure_time_absolute = 2000` -- a **200 ms shutter**.
Frames came out at mean 26.6 with every hand movement smeared; manual exp 50
(5 ms) at brightness 150 gave mean 135.0 with 6.5 % clipping, same room and
lens.

The consequence is not "dim pictures". Blur from rotation is
`f_px * omega * t_exp`, so at f = 514 and 0.64 rad/s a 200 ms exposure smears
**66 px** -- and the failure presents as "the detector is bad", which is
exactly how it was reported.

Reads the source and drives the pure arithmetic; it does NOT open a camera,
so it runs anywhere.
"""
import pathlib
import re

import pytest

_SRC = (pathlib.Path(__file__).resolve().parents[1]
        / 'mongla_vision' / 'cameras' / 'webcam.py')


def _shipped():
    """The REAL `blur_capped_exposure`, lifted out of the source file.

    ⛔ NOT A RESTATEMENT. The first version of this test reimplemented the
    rule locally, and an injected defect -- making the cap lengthen as well
    as shorten -- left it GREEN, because it was checking the copy. That is
    the round-33 failure verbatim: the one test that could have caught the
    Kalman bug reimplemented the node's loop and tested a copy without it.

    Extracted by AST rather than imported, because importing this package in
    a worktree resolves against the MAIN workspace's stale `install/` tree --
    which has already nearly sent one retraction the wrong way. Parsing the
    file reads the code that will actually ship.
    """
    import ast as _ast
    tree = _ast.parse(_SRC.read_text())
    for node in tree.body:
        if isinstance(node, _ast.FunctionDef) and \
                node.name == 'blur_capped_exposure':
            ns = {}
            exec(compile(_ast.Module([node], []), str(_SRC), 'exec'), ns)
            return ns['blur_capped_exposure']
    pytest.fail('blur_capped_exposure is gone from webcam.py -- if the rule '
                'moved, move this test with it rather than deleting it')


blur_capped_exposure = _shipped()


class TestBlurCap:
    def test_the_cap_is_f_omega_t(self):
        """One pixel of blur at our measured focal length and rotation rate.

        f = 513.94 px, omega = 0.638 rad/s (the rate §12 measured on the
        rig): t = 1 / (514 * 0.638) = 3.05 ms = 30 units of 0.1 ms.
        """
        got = blur_capped_exposure(500, 1.0, 513.94, 0.638)
        assert got == pytest.approx(30, abs=1)
        # and the relation itself, not just the one number
        assert blur_capped_exposure(500, 2.0, 513.94, 0.638) == \
            pytest.approx(2 * got, abs=2), 'blur allowance must scale linearly'
        assert blur_capped_exposure(500, 1.0, 513.94, 1.276) == \
            pytest.approx(got / 2, abs=2), 'twice the rate, half the shutter'

    def test_a_slower_vehicle_does_not_get_a_SHORTER_shutter(self):
        """The cap only ever tightens. It must never LENGTHEN the requested
        exposure into blur the operator did not ask for -- a cap that also
        raises is a different feature wearing a cap's name."""
        assert blur_capped_exposure(20, 1.0, 513.94, 0.01) == 20

    def test_it_never_returns_zero(self):
        """Zero means 'auto' everywhere else in this file, so a cap that
        rounds to 0 would silently re-enable the very thing being fixed."""
        assert blur_capped_exposure(500, 0.01, 2000.0, 50.0) >= 1

    def test_disabled_when_any_term_is_unknown(self):
        for kw in ({'max_blur_px': 0}, {'f_px': 0}, {'max_rate_rad_s': 0}):
            args = dict(max_blur_px=1.0, f_px=513.94, max_rate_rad_s=0.638)
            args.update(kw)
            assert blur_capped_exposure(500, **args) == 500, (
                f'with {kw} unknown the cap must not be invented')


class TestShipped:
    def test_exposure_zero_leaves_the_camera_on_AUTO(self):
        """0 must be the documented 'do not touch' value, so an unconfigured
        camera behaves exactly as it did before this landed."""
        src = _SRC.read_text()
        assert re.search(r'if not exposure_us:\s*\n\s*return', src), (
            'exposure_us=0 no longer short-circuits; an unset camera would '
            'get a pinned shutter it never asked for')

    def test_auto_is_disabled_BEFORE_the_exposure_is_set(self):
        """Ordering is load-bearing: `exposure_time_absolute` is silently
        ignored while auto exposure is engaged, so a reordering leaves the
        shutter on auto while every log line claims it was set."""
        src = _SRC.read_text()
        a = src.index("'auto_exposure'")
        b = src.index("'exposure_time_absolute'")
        assert a < b, 'auto_exposure must be written before the exposure time'

    def test_gain_is_not_offered(self):
        """Measured inert on this camera -- gain 20/50/100 give identical
        frames. A knob that does nothing is worse than no knob; this package
        has four already on its record."""
        src = _SRC.read_text()
        assert "'gain'" not in src, (
            'gain is exposed again -- it was measured to do nothing on the '
            'Fantech, so it can only mislead')
