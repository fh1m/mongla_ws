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
    """The mesh is a BlueROV2 Heavy and carried the vendor's colours; an SDF
    <material> overrides it, so this needs no mesh edit."""
    cfg, sdf = _cfg(), _sdf(False)
    hull = ' '.join(f'{c:.4g}' for c in cfg['livery']['hull'])
    assert f'<diffuse>{hull} 1</diffuse>' in sdf
    thruster = ' '.join(f'{c:.4g}' for c in cfg['livery']['thruster'])
    assert sdf.count(f'<diffuse>{thruster} 1</diffuse>') == 8


def test_the_emissive_is_per_channel_not_grey():
    """A flat grey emissive lifts all three channels equally, which is a
    desaturation term -- it is what washed the colour out of every prop before
    round 12, and the hull would go the same way."""
    sdf = _sdf(False)
    for m in re.finditer(r'<emissive>([\d.]+) ([\d.]+) ([\d.]+) 1</emissive>',
                         sdf):
        r, g, b = (float(m.group(i)) for i in (1, 2, 3))
        if r == g == b == 0.0:
            continue
        assert not (r == g == b), 'grey emissive desaturates the livery'
        break
    else:
        pytest.fail('no emissive found on the vehicle')
