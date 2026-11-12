"""The hull Kutum hands to the 3D scene is the CAD, in the body frame, nose forward."""
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from mongla_batighor import model

_ROOT = Path(__file__).resolve().parents[3]
HULL = _ROOT / 'docs' / 'assets' / 'cad' / 'hull.mgla'
GEOMETRY = _ROOT / 'src' / 'mongla_control' / 'config' / 'hull_geometry.yaml'
THRUSTERS = model.thrusters(GEOMETRY)


def test_the_hull_is_the_cads_size_along_the_body_axes():
    """702 x 176 x 172 mm (CLAUDE.md, the current CAD): long along x, the
    width across y, the height down z. A wrong axis swap fails here."""
    pos, _, idx = model.load(HULL)
    ext = pos.max(0) - pos.min(0)
    assert abs(ext[0] - 0.702) < 0.002, ext
    assert abs(ext[1] - 0.1761) < 0.002, ext
    assert abs(ext[2] - 0.1721) < 0.002, ext
    assert idx.max() < len(pos)


def test_the_nose_is_forward():
    """The axial unit is at the nose (hull_geometry.yaml): in the body frame it
    is at +x, inside the hull's own length, on the centreline."""
    axial = dict((n, p) for n, p, _ in THRUSTERS)['axial']
    x, y, z = model.cad_to_body(axial)
    pos, _, _ = model.load(HULL)
    assert 0.3 < x <= pos[:, 0].max() and abs(y) < 1e-6 and abs(z) < 1e-3


def test_the_frame_change_is_a_rotation_not_a_mirror():
    m = np.array([model.cad_to_body(e) for e in np.eye(3)]).T
    assert abs(np.linalg.det(m) - 1) < 1e-9


def test_every_triangle_lands_in_exactly_one_livery_mesh():
    _, _, idx = model.load(HULL)
    out = model.meshes(HULL)
    counts = {n: struct.unpack_from('<I', b, 80)[0] for n, b in out.items()}
    assert sum(counts.values()) == len(idx)
    for n, b in out.items():
        assert len(b) == 84 + 50 * counts[n], n
    assert counts['fairing'] > 0                     # the red the hull is built in


def test_the_urdf_hangs_the_meshes_and_every_thruster_on_the_body_frame():
    names = sorted(model.meshes(HULL))
    root = ET.fromstring(model.urdf(names, THRUSTERS))
    links = root.findall('link')
    assert [ln.get('name') for ln in links] == ['mongla']         # frames.BODY
    meshes = [m.get('filename') for m in root.iter('mesh')]
    assert meshes == [f'hull-{n}.stl' for n in names]          # beside the URDF
    visuals = {v.get('name') for v in root.iter('visual')}
    assert {n for n, _, _ in THRUSTERS} <= visuals


def test_the_body_frame_name_is_the_localizations():
    from mongla_localization import frames
    assert frames.BODY == 'mongla'


def test_every_mesh_the_urdf_names_is_served_beside_it(tmp_path):
    import shutil
    shutil.copy(HULL, tmp_path / 'hull.mgla')
    shutil.copy(GEOMETRY, tmp_path / 'hull_geometry.yaml')
    out = model.files(tmp_path / 'hull.mgla')
    root = ET.fromstring(out['mongla.urdf'])
    assert {m.get('filename') for m in root.iter('mesh')} | {'mongla.urdf'} == set(out)


def test_the_thrusters_drawn_are_the_measured_five():
    """Read from the measured file; the allocator's mirror must agree with it
    (test_geometric_allocation.py), so this is the same five."""
    from mongla_control.geometric_allocation import THRUSTERS as MIRROR
    assert [(n, p, a) for n, p, a in THRUSTERS] == \
        [(n, tuple(map(float, p)), tuple(map(float, a))) for n, p, a in MIRROR]
