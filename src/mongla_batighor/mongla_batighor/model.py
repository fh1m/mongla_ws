"""The hull in Lichtblick's 3D scene: the site's CAD, in the body frame, with its thrusters.

One copy of the hull is in the repo, `docs/assets/cad/hull.mgla`, the
container the site's viewer reads (`tools/site/pack_hull.py`). Kutum turns it
into binary STL meshes when it starts, one per livery colour, and a URDF
whose root link is `mongla`, the body frame the localization publishes on
`/tf`. Lichtblick draws the URDF where `/tf` puts that frame.

Axes. The container is in glTF's axes, as Onshape exports them: z along the
702 mm long axis, y up, x across. `hull_geometry.yaml` puts the nose (the
axial unit) at CAD y = -0.3306, which glTF's z = -y_cad makes +z. The body
frame is FRD (x forward, y right, z down, `frames.py`), so

    body = ( z_gltf, -x_gltf, -y_gltf ) = ( -y_cad, -x_cad, -z_cad )

both proper rotations. The up sense rests on the export being Onshape's
standard Y-up, as the site's viewer, which draws the hull upright, assumes;
`hull_geometry.yaml` itself leaves signs to be fixed against the vehicle.

STL, not glTF: STL has no up convention to get wrong, and each livery colour
becomes one mesh with its own URDF material.
"""
from __future__ import annotations

import struct
from pathlib import Path

import numpy as np

# The livery: every CAD colour goes to the nearest of these. Red is the
# fairing (CAD material 4), amber the accents (0), slate the frame (1).
LIVERY = {
    'fairing': (0xff, 0x00, 0x00),
    'accent': (0xe8, 0x92, 0x2a),
    'frame': (0x43, 0x48, 0x4d),
    'grey': (0x99, 0x99, 0x99),
    'light': (0xe6, 0xe6, 0xe6),
}
SEA = (0x5a, 0x8d, 0xff)          # a thruster: what the machine acts with


def load(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(positions in metres in the body frame, Nx3; colours Nx3 uint8; triangles Mx3)."""
    b = Path(path).read_bytes()
    if b[:4] != b'MGLA':
        raise ValueError(f'{path}: not a hull container')
    ver, ibytes, nv, nt = struct.unpack_from('<HHII', b, 4)
    centre = np.array(struct.unpack_from('<3f', b, 16))
    scale = struct.unpack_from('<f', b, 28)[0]
    off = 32
    if ver >= 3:
        nparts = struct.unpack_from('<H', b, off)[0]
        off += 2 + nparts * 12
    q = np.frombuffer(b, '<i2', nv * 3, off).reshape(-1, 3).astype(np.float64)
    off += nv * 6
    col = (np.frombuffer(b, 'u1', nv * 3, off).reshape(-1, 3) if ver >= 2
           else np.full((nv, 3), 0x99, np.uint8))
    off += nv * 3 if ver >= 2 else 0
    off += nv if ver >= 3 else 0
    idx = np.frombuffer(b, '<u2' if ibytes == 2 else '<u4', nt * 3, off).reshape(-1, 3)
    g = q / 32767.0 * scale + centre                       # glTF axes, metres
    body = np.stack([g[:, 2], -g[:, 0], -g[:, 1]], axis=1)
    return body, col, idx.astype(np.int64)


def cad_to_body(p) -> tuple[float, float, float]:
    """A point or direction in the CAD frame (`hull_geometry.yaml`) in the body frame."""
    return (-p[1], -p[0], -p[2])


def _stl(pos: np.ndarray, tris: np.ndarray) -> bytes:
    a, b, c = pos[tris[:, 0]], pos[tris[:, 1]], pos[tris[:, 2]]
    n = np.cross(b - a, c - a)
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), dtype=[('n', '<f4', 3), ('v', '<f4', 9), ('attr', '<u2')])
    rec['n'] = n
    rec['v'] = np.concatenate([a, b, c], axis=1)
    return b'mongla hull'.ljust(80, b' ') + struct.pack('<I', len(tris)) + rec.tobytes()


def meshes(path: Path) -> dict[str, bytes]:
    """livery name -> a binary STL of the triangles in that colour."""
    pos, col, idx = load(path)
    names = list(LIVERY)
    ref = np.array([LIVERY[n] for n in names], dtype=np.float64)
    tri_col = col[idx[:, 0]].astype(np.float64)
    which = np.argmin(((tri_col[:, None, :] - ref[None]) ** 2).sum(-1), axis=1)
    return {names[k]: _stl(pos, idx[which == k]) for k in range(len(names)) if (which == k).any()}


def _rgba(rgb) -> str:
    return ' '.join(f'{c / 255:.3f}' for c in rgb) + ' 1'


def _rpy_for(axis) -> str:
    """Roll-pitch-yaw that turns a URDF cylinder's z onto `axis` (a body unit vector)."""
    x, y, z = (abs(c) for c in axis)
    if z >= max(x, y):
        return '0 0 0'
    if y >= x:
        return '1.5708 0 0'
    return '0 1.5708 0'


def urdf(names, thrusters) -> str:
    """The URDF: one visual per livery mesh, and each measured thruster as a ring.

    `thrusters` is (name, CAD position, CAD axis) from `thrusters()`. Mesh
    names are relative to the URDF's own URL, so the page works under
    whatever host name the deck reached Kutum by.
    """
    out = ['<?xml version="1.0"?>', '<robot name="mongla">']
    for n in names:
        out.append(f'  <material name="{n}"><color rgba="{_rgba(LIVERY[n])}"/></material>')
    out.append(f'  <material name="thruster"><color rgba="{_rgba(SEA)}"/></material>')
    out.append('  <link name="mongla">')
    for n in names:
        out.append(f'    <visual name="hull_{n}"><geometry>'
                   f'<mesh filename="hull-{n}.stl"/>'
                   f'</geometry><material name="{n}"/></visual>')
    for name, pos, axis in thrusters:
        p = cad_to_body(pos)
        out.append(f'    <visual name="{name}"><origin xyz="{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}" '
                   f'rpy="{_rpy_for(cad_to_body(axis))}"/>'
                   '<geometry><cylinder radius="0.046" length="0.012"/></geometry>'
                   '<material name="thruster"/></visual>')
    out += ['  </link>', '</robot>', '']
    return '\n'.join(out)


def thrusters(path: Path) -> list:
    """(name, CAD position, CAD axis) for each thruster in `hull_geometry.yaml`.

    The measured file itself, not `geometric_allocation.THRUSTERS`, its
    mirror: the allocator is not on the flight path yet, and the picture
    should not be the thing that puts it there.
    """
    import yaml
    doc = yaml.safe_load(Path(path).read_text())
    return [(t['name'], tuple(t['position_m']), tuple(t['axis'])) for t in doc['thrusters']]


def files(path: Path) -> dict[str, bytes]:
    """What `/model/` serves: `mongla.urdf` and one `hull-<livery>.stl` per colour.

    `path` is the hull container; `hull_geometry.yaml` sits beside it.
    Built once, in memory, when Kutum's server starts.
    """
    stl = meshes(path)
    out = {f'hull-{n}.stl': b for n, b in stl.items()}
    out['mongla.urdf'] = urdf(sorted(stl),
                              thrusters(Path(path).with_name('hull_geometry.yaml'))).encode()
    return out
