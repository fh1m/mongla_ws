"""Dubomini is the team's real vehicle; these pin what "real" means here.

The failures these guard against are all silent ones -- a wrong scale, a wrong
frame, or a thruster that pushes 180 degrees from where it should. None of them
raises; the vehicle simply behaves wrongly and no log says why.
"""

import math
import os
import subprocess
import sys
import tempfile

import pytest

yaml = pytest.importorskip('yaml')

DESC = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'duburi_sim_description'))
MODEL = os.path.join(DESC, 'models', 'dubomini')
CONFIG = os.path.join(MODEL, 'configs.yaml')

pytestmark = pytest.mark.skipif(
    not os.path.isfile(CONFIG), reason='dubomini model not present')

# bracuduburi.com/auv/dubomini, quoted: 14.6 kg, 54.59 x 46.43 x 16.68 cm.
PUBLISHED_MASS = 14.6
PUBLISHED_MM = (545.9, 464.3, 166.8)


def _cfg():
    with open(CONFIG) as fh:
        return yaml.safe_load(fh)


def _sdf():
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, 'model.sdf')
        r = subprocess.run(
            [sys.executable, os.path.join(DESC, 'scripts', 'generate_model.py'),
             os.path.join(MODEL, 'model.sdf.in'), out, CONFIG],
            capture_output=True, text=True)
        assert r.returncode == 0, r.stderr
        with open(out) as fh:
            return fh.read()


def test_the_model_matches_the_published_vehicle():
    """If these drift, the sim is flying something that is not Dubomini."""
    c = _cfg()
    assert c['mass'] == pytest.approx(PUBLISHED_MASS, abs=1e-6)
    got = (c['bounding_box']['x'], c['bounding_box']['y'], c['bounding_box']['z'])
    for g, pub in zip(got, [v / 1000.0 for v in PUBLISHED_MM]):
        assert abs(g - pub) < 0.010, f'{g} vs published {pub}'


def test_forward_is_forward():
    """THE ONE ERROR THAT IS INVISIBLE IN A LOG.

    ArduSub's `vectored_6dof` forward mix is [-33, -27, +33, +27] across
    thrusters 1-4. Against the thruster yaws taken literally from the CAD it
    summed to (0.0, -8.5) N -- no forward thrust at all -- and the vehicle
    drifted diagonally while every thruster did exactly as it was told.

    A duct's axis is a LINE; which way it pushes is prop handedness and ESC
    wiring, which are not in the geometry. This asserts the signs that make the
    frame's own mixer produce surge.
    """
    yaws = [t['yaw_deg'] for t in _cfg()['thrusters'][:4]]
    mix = [-33.0, -27.0, 33.0, 27.0]
    fx = sum(c * math.sin(math.radians(y)) for c, y in zip(mix, yaws))
    fy = sum(c * -math.cos(math.radians(y)) for c, y in zip(mix, yaws))
    assert fx > 50.0, f'forward mix gives only {fx:.1f} N of surge'
    assert abs(fy) < 1.0, f'forward mix also produces {fy:.1f} N of sway'


def test_equal_commands_produce_no_yaw_moment():
    """A layout that yaws when asked to surge is mis-wired, not agile."""
    c = _cfg()
    th = c['thrusters'][:4]
    m = sum(t['x'] * -math.cos(math.radians(t['yaw_deg']))
            - t['y'] * math.sin(math.radians(t['yaw_deg'])) for t in th)
    assert abs(m) < 0.01, f'net yaw moment {m:.4f} from a pure surge command'


def test_the_thrusters_sit_where_the_cad_puts_them():
    """Recovered from the mesh, not typed. The horizontals are much further out
    in x than Duburi's (0.228 vs 0.14), which is this hull's yaw authority."""
    th = _cfg()['thrusters']
    assert len(th) == 8
    horiz = th[:4]
    assert all(abs(abs(t['x']) - 0.2265) < 0.005 for t in horiz)
    assert all(abs(abs(t['y']) - 0.1515) < 0.005 for t in horiz)


def test_added_mass_is_positive_and_heave_dominates():
    """Capytaine returns POSITIVE magnitudes for SDF's fluid_added_mass, while
    the Hydrodynamics plugin's <xDotU> family is negative -- mixing the two is
    the documented way to make this model diverge.

    Heave dominating by 4x is the signature of a flat plate and is the whole
    reason this vehicle feels different from Duburi.
    """
    am = _cfg()['added_mass']
    assert all(v > 0 for v in am.values()), 'SDF form takes positive magnitudes'
    assert am['zDotW'] > 3 * am['xDotU']


def test_the_mesh_is_referenced_and_the_collision_is_decomposed():
    sdf = _sdf()
    assert 'model://dubomini/meshes/dubomini.dae' in sdf
    assert 'convex_decomposition' in sdf, (
        'the hull collision must be decomposed by Gazebo, not left as a raw mesh')


def test_net_buoyancy_is_the_declared_trim():
    """The collision box is DERIVED from mass and trim; a mistake here floats or
    sinks the vehicle silently."""
    import re
    c, sdf = _cfg(), _sdf()
    box = re.search(r'<size>([\d.]+) ([\d.]+) ([\d.]+)</size>', sdf)
    mass = float(re.search(r'<mass>([\d.]+)</mass>', sdf).group(1))
    displaced = (float(box.group(1)) * float(box.group(2))
                 * float(box.group(3)) * c['fluid_density'])
    assert displaced - mass == pytest.approx(c['buoyancy_adjustment'], abs=1e-3)
