"""The hull is ours, and bolting a gripper on must not change how it floats.

THE INVARIANT THIS PINS. `buoyancy_adjustment` is the NET figure -- displaced
mass minus mass -- so adding the gripper's mass ALREADY adds an equal
displacement implicitly. The correction is only the part the gripper fails to
displace (0.524 - 0.257 = 0.267 kg, which is its submerged weight, as it must
be). Adding the displacement on top of that double counted and put the vehicle
at +0.624 kg net -- over-buoyant, from a part that sinks.

That was caught by checking the derived net against the intended +0.1 rather
than by trusting the arithmetic, which is exactly what this test does now. The
collision box is DERIVED (`collision_z = displaced / (bx * by * rho)`), so a
mistake here is silent: the vehicle simply floats or sinks wrongly, and the
hydrodynamics were fitted to a measured 0.95 m/s top speed on the correct trim.
"""

import os
import re
import subprocess
import sys
import tempfile

import pytest

yaml = pytest.importorskip('yaml')

DESC = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'duburi_sim_description'))
MODEL = os.path.join(DESC, 'models', 'duburi_heavy')
CONFIG = os.path.join(MODEL, 'configs.yaml')
TEMPLATE = os.path.join(MODEL, 'model.sdf.in')

pytestmark = pytest.mark.skipif(
    not os.path.isfile(CONFIG), reason='duburi_sim_description not present')


def _cfg():
    with open(CONFIG) as fh:
        return yaml.safe_load(fh)


def _sdf(gripper_on: bool) -> str:
    """Regenerate with the gripper forced on or off, without touching the tree."""
    with open(CONFIG) as fh:
        text = fh.read()
    if gripper_on:
        text = text.replace('  enabled: false', '  enabled: true', 1)
    with tempfile.TemporaryDirectory() as tmp:
        cfg = os.path.join(tmp, 'configs.yaml')
        out = os.path.join(tmp, 'model.sdf')
        with open(cfg, 'w') as fh:
            fh.write(text)
        r = subprocess.run(
            [sys.executable, os.path.join(DESC, 'scripts', 'generate_model.py'),
             TEMPLATE, out, cfg], capture_output=True, text=True)
        assert r.returncode == 0, r.stderr
        with open(out) as fh:
            return fh.read()


def _net_buoyancy(sdf: str, cfg: dict) -> float:
    box = re.search(r'<size>([\d.]+) ([\d.]+) ([\d.]+)</size>', sdf)
    mass = float(re.search(r'<mass>([\d.]+)</mass>', sdf).group(1))
    bx, by, bz = (float(box.group(i)) for i in (1, 2, 3))
    displaced = bx * by * bz * cfg['fluid_density']
    return displaced - mass


def test_the_gripper_does_not_change_how_the_vehicle_floats():
    cfg = _cfg()
    want = cfg['buoyancy_adjustment']
    assert _net_buoyancy(_sdf(False), cfg) == pytest.approx(want, abs=1e-3)
    assert _net_buoyancy(_sdf(True), cfg) == pytest.approx(want, abs=1e-3)


def test_the_gripper_carries_its_real_mass_when_fitted():
    """The trim is FLOATATION, not a lighter gripper. A Newton is 524 g."""
    cfg = _cfg()
    off = float(re.search(r'<mass>([\d.]+)</mass>', _sdf(False)).group(1))
    on = float(re.search(r'<mass>([\d.]+)</mass>', _sdf(True)).group(1))
    assert on - off == pytest.approx(cfg['gripper']['mass'], abs=1e-6)


def test_the_trim_is_the_grippers_submerged_weight():
    """524 g in air minus 257 g displaced is 267 g submerged, which is the
    datasheet figure. If these ever disagree, one of the three is wrong."""
    g = _cfg()['gripper']
    assert g['mass'] - g['displacement'] == pytest.approx(g['trim_kg'], abs=1e-6)


def test_the_gripper_is_stripped_from_the_sdf_when_disabled():
    """Not merely switched off. A disabled feature left in the SDF is still
    paid for -- that is how the range cameras cost a third of the frame rate
    while claiming to be off."""
    assert 'gripper' not in _sdf(False)
    on = _sdf(True)
    assert on.count('<link name="gripper') == 3
    assert on.count('JointPositionController') == 2
    # NOT a DetachableJoint yet. The plugin names its child model at LOAD time
    # and a gripper does not know what it will grab, so declaring one here
    # would attach the jaw to the vehicle itself. It lands with the runtime
    # node that creates the joint against the model actually grasped.
    assert 'detachable-joint-system' not in on   # the PLUGIN, not the word


def test_the_hull_wears_our_livery_not_the_vendors():
    """The livery lives in the MESH, not in an SDF <material>.

    An SDF material on a mesh visual replaces the material for the WHOLE mesh,
    and this .dae carries 49 of them. The first version did exactly that and
    erased the vehicle -- white and featureless -- while a pixel-diff happily
    reported 33.8 % of the frame changed. So this asserts the shape of the fix:
    the visual points at the recoloured mesh and carries NO material override.
    """
    sdf = _sdf(False)
    assert 'duburi_heavy_livery.dae' in sdf
    vis = sdf[sdf.index('base_link_visual'):]
    vis = vis[:vis.index('</visual>')]
    # Strip XML comments first: the visual carries a long note explaining why
    # there is no <material> here, and that note contains the word.
    import re as _re
    vis = _re.sub(r'<!--.*?-->', '', vis, flags=_re.S)
    assert '<material>' not in vis, (
        'an SDF material here flattens all 49 of the mesh\'s materials')


def test_the_livery_mesh_keeps_the_vehicle_s_parts():
    """Preserving per-part detail is the POINT, not a nicety.

    The failure this guards against is not "wrong colour" -- it is a vehicle
    with no parts. Luminance range and distinct-material count are what say the
    housings are still dark against the foam.
    """
    import re as _re
    meshes = os.path.join(MODEL, 'meshes')
    src = os.path.join(meshes, 'duburi_heavy.dae')
    dst = os.path.join(meshes, 'duburi_heavy_livery.dae')
    if not (os.path.isfile(src) and os.path.isfile(dst)):
        pytest.skip('meshes not present')

    def stats(path):
        with open(path) as fh:
            cols = _re.findall(r'<color sid="diffuse">([^<]+)</color>', fh.read())
        vals = [tuple(float(v) for v in c.split()[:3]) for c in cols]
        lum = [0.299 * r + 0.587 * g + 0.114 * b for r, g, b in vals]
        return len(set(vals)), min(lum), max(lum)

    n_src, lo_src, hi_src = stats(src)
    n_dst, lo_dst, hi_dst = stats(dst)
    assert n_dst == n_src, 'the recolour collapsed distinct materials'
    assert hi_dst - lo_dst > 0.8, 'the recolour flattened the luminance range'


def test_the_livery_mesh_is_actually_different_from_stock():
    """Otherwise it is a copy with a new name.

    Two earlier passes were arithmetically correct and changed almost nothing:
    tinting greys toward a grey hull is the identity map, and forcing the
    accents to preserve luminance only made the stock cyan brighter.
    """
    import re as _re
    meshes = os.path.join(MODEL, 'meshes')
    src, dst = (os.path.join(meshes, f) for f in
                ('duburi_heavy.dae', 'duburi_heavy_livery.dae'))
    if not (os.path.isfile(src) and os.path.isfile(dst)):
        pytest.skip('meshes not present')

    def cols(path):
        with open(path) as fh:
            return [tuple(round(float(v), 4) for v in c.split()[:3])
                    for c in _re.findall(
                        r'<color sid="diffuse">([^<]+)</color>', fh.read())]

    a, b = cols(src), cols(dst)
    moved = sum(1 for x, y in zip(a, b)
                if max(abs(p - q) for p, q in zip(x, y)) > 0.02)
    assert moved >= len(a) // 3, (
        f'only {moved}/{len(a)} materials moved -- the livery is a no-op')
