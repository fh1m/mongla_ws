#!/usr/bin/env python3
"""Read EXACT thruster stations out of an Onshape GLB export. Offline, no decode.

    python3 tools/hull_stations.py <Part_Studio.glb> [--bore 84] [--yaml]

⛔ WHY THIS EXISTS, AND WHAT IT REPLACES. Hull geometry has been measured two ways
in this tree and both had a problem:

  1. `tools/pack_hull.py` + `docs/assets/cad/hull.mgla`. That asset is vertex-
     clustered and int16-quantised for a WEB VIEWER. Measured off it, the two pair
     separations came out 346.6 and 520.4 mm against a true 350.0 and 519.0 --
     errors of 3.4 and 1.4 mm. Those figures reached upstream PR
     srot-control-board#25 before they were retracted. A decimated render asset is
     not a metrology source, and the only durable fix is to stop having a path
     where it can be used as one.

  2. The Onshape REST API (`/api/parts/.../boundingboxes`), which IS exact but
     needs an authenticated browser session, a document id, a workspace id and an
     element id. It cannot run in CI, it cannot run offline, and it cannot be
     re-run by anyone who does not have the CAD open.

⭐ THE THIRD WAY, AND IT WAS IN THE FILE ALL ALONG. glTF requires every POSITION
accessor to carry exact `min` and `max` -- that is the spec's own axis-aligned
bounding box, in metres, written by the exporter from the CAD. It is present even
when the vertex data is Draco-compressed, because the bounds live in the JSON
chunk and the compression only touches the binary chunk. So the per-part bounding
boxes can be read from an ORDINARY Onshape export with no decompressor, no numpy,
and no network -- and they agree with the REST API exactly.

`pack_hull.py` fails outright on a default Onshape export for the same reason it
never saw these numbers: it tries to DECODE the mesh, and Onshape emits
`KHR_draco_mesh_compression` unless you go out of your way. Geometry never needed
the mesh.

⚠ WHAT THIS DOES NOT GIVE YOU. Mass, centre of mass, centre of buoyancy, net
buoyancy, thrust per RPM. A bounding box is a box. See `hull_geometry.yaml` for
which of those are computable from shape alone (added-mass coefficients are) and
which need water (net buoyancy does).

⚠ THE FRAME. Onshape's glTF exporter writes glTF's Y-up convention, so the CAD's
long axis Y lands on the export's Z. Measured against this hull, the mapping is

    cad = (mesh_x, -mesh_z, mesh_y)

i.e. the CAD's +Y (forward) is the export's -Z. `--yaml` applies it; the table
prints raw mesh coordinates so the transform is never silently baked in. Getting
this backwards flips surge, which is why it is written down here rather than
remembered.
"""
from __future__ import annotations

import argparse
import json
import struct
import sys
from pathlib import Path

GLB_JSON, GLB_BIN = 0x4E4F534A, 0x004E4942


def read_gltf_json(path: Path) -> dict:
    """The JSON chunk of a .glb, or the whole file for a .gltf."""
    data = path.read_bytes()
    if data[:4] != b'glTF':
        if path.suffix.lower() == '.gltf':
            return json.loads(data)
        raise SystemExit(f'{path}: not a GLB (no glTF magic) and not a .gltf')
    magic, version, _length = struct.unpack_from('<III', data, 0)
    if version != 2:
        raise SystemExit(f'{path}: glTF version {version}, expected 2')
    off = 12
    while off < len(data):
        clen, kind = struct.unpack_from('<II', data, off)
        off += 8
        if kind == GLB_JSON:
            return json.loads(data[off:off + clen])
        off += clen
    raise SystemExit(f'{path}: no JSON chunk')


def node_matrix(node: dict):
    """The node's local transform as a 4x4 row-major list-of-lists."""
    def mul(a, b):
        return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
                for i in range(4)]
    m = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
    if 'matrix' in node:                      # glTF stores column-major
        c = node['matrix']
        return [[c[j * 4 + i] for j in range(4)] for i in range(4)]
    if 'scale' in node:
        s = node['scale']
        m = mul([[s[0], 0, 0, 0], [0, s[1], 0, 0], [0, 0, s[2], 0], [0, 0, 0, 1]], m)
    if 'rotation' in node:
        x, y, z, w = node['rotation']
        r = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0],
             [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0],
             [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0],
             [0, 0, 0, 1]]
        m = mul(r, m)
    if 'translation' in node:
        t = node['translation']
        m = mul([[1, 0, 0, t[0]], [0, 1, 0, t[1]], [0, 0, 1, t[2]], [0, 0, 0, 1]], m)
    return m


def _xform_box(m, lo, hi):
    """The AABB of a transformed AABB: all eight corners, re-bounded.

    ⚠ NOT the transform of the two corners. Under a rotation that is simply wrong,
    and it is wrong in a way that looks plausible -- a slightly small box. Onshape
    bakes part placement into the coordinates and emits no node transforms at all
    (asserted below), so on this file the distinction never bites; it is here so
    that an exporter which DOES emit them cannot quietly corrupt a station.
    """
    out_lo = [float('inf')] * 3
    out_hi = [float('-inf')] * 3
    for i in range(8):
        p = [lo[0] if i & 1 else hi[0],
             lo[1] if i & 2 else hi[1],
             lo[2] if i & 4 else hi[2]]
        for a in range(3):
            v = sum(m[a][b] * p[b] for b in range(3)) + m[a][3]
            out_lo[a] = min(out_lo[a], v)
            out_hi[a] = max(out_hi[a], v)
    return out_lo, out_hi


def parts(gltf: dict):
    """[(name, lo_mm, hi_mm)] -- one per mesh-bearing node, world frame."""
    found = []
    identity_only = True

    def walk(idx, m):
        nonlocal identity_only
        node = gltf['nodes'][idx]
        if any(k in node for k in ('matrix', 'translation', 'rotation', 'scale')):
            identity_only = False
        m = node_matrix(node) if any(
            k in node for k in ('matrix', 'translation', 'rotation', 'scale')) else m
        if 'mesh' in node:
            lo = [float('inf')] * 3
            hi = [float('-inf')] * 3
            for prim in gltf['meshes'][node['mesh']]['primitives']:
                acc = gltf['accessors'][prim['attributes']['POSITION']]
                # ⛔ The spec REQUIRES min/max on a POSITION accessor. If an
                # exporter omits them, say so rather than guessing: a missing
                # bound silently shrinks a part and moves its centre.
                if 'min' not in acc or 'max' not in acc:
                    raise SystemExit(
                        f"accessor for node {node.get('name', idx)!r} carries no "
                        f"min/max. Nothing here can recover it without decoding the "
                        f"mesh; re-export with bounds, or use a tool that decodes "
                        f"KHR_draco_mesh_compression.")
                for a in range(3):
                    lo[a] = min(lo[a], acc['min'][a])
                    hi[a] = max(hi[a], acc['max'][a])
            lo, hi = _xform_box(m, lo, hi)
            found.append((node.get('name', f'node{idx}'),
                          [v * 1000.0 for v in lo], [v * 1000.0 for v in hi]))
        for c in node.get('children', []):
            walk(c, m)

    ident = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
    for idx in gltf['scenes'][gltf.get('scene', 0)]['nodes']:
        walk(idx, ident)
    return found, identity_only


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('glb', type=Path)
    ap.add_argument('--bore', type=float, default=84.0,
                    help='tunnel bore in mm, to identify the tunnels (default 84)')
    ap.add_argument('--tol', type=float, default=1.0, help='bore match tolerance, mm')
    ap.add_argument('--yaml', action='store_true',
                    help='emit a hull_geometry.yaml thrusters block, in the CAD frame')
    args = ap.parse_args(argv)

    gltf = read_gltf_json(args.glb)
    ext = gltf.get('extensionsRequired', [])
    found, identity_only = parts(gltf)
    if not found:
        raise SystemExit('no mesh-bearing nodes')

    lo = [min(p[1][a] for p in found) for a in range(3)]
    hi = [max(p[2][a] for p in found) for a in range(3)]
    size = [hi[a] - lo[a] for a in range(3)]
    long_axis = max(range(3), key=lambda a: size[a])
    cross = sorted(size[a] for a in range(3) if a != long_axis)

    print(f'file       {args.glb.name}')
    print(f'generator  {gltf.get("asset", {}).get("generator", "?")}')
    print(f'required   {ext or "none"}'
          + ('   <- mesh data is compressed; the BOUNDS are not, and that is all '
             'this needs' if 'KHR_draco_mesh_compression' in ext else ''))
    print(f'bodies     {len(found)}')
    print(f'node xf    {"none (coordinates are baked)" if identity_only else "PRESENT"}')
    print(f'hull bbox  {size[0]:.2f} x {size[1]:.2f} x {size[2]:.2f} mm')
    print(f'fineness   {size[long_axis] / cross[-1]:.3f}  '
          f'(long axis = {"XYZ"[long_axis]})')
    print()

    rows = sorted(found, key=lambda p: (p[1][long_axis] + p[2][long_axis]) / 2)
    print(f'{"part":<12}{"ext_x":>9}{"ext_y":>9}{"ext_z":>9}   '
          f'{"cx":>9}{"cy":>9}{"cz":>9}  bore?')
    tunnels = []
    for name, plo, phi in rows:
        e = [phi[a] - plo[a] for a in range(3)]
        c = [(phi[a] + plo[a]) / 2 for a in range(3)]
        # A tunnel is a body with TWO extents at the bore and one longer: the long
        # one is the bore axis. Identified by shape, which is reliable for a tube
        # and is NOT reliable for a motor -- see hull_geometry.yaml on the A2212.
        at_bore = [a for a in range(3) if abs(e[a] - args.bore) <= args.tol]
        bore_ax = ''
        if len(at_bore) == 2:
            axis = [a for a in range(3) if a not in at_bore][0]
            if e[axis] > args.bore:
                bore_ax = '<- tunnel, bore along ' + 'XYZ'[axis]
                tunnels.append((name, c, axis))
        print(f'{name[:12]:<12}{e[0]:9.2f}{e[1]:9.2f}{e[2]:9.2f}   '
              f'{c[0]:9.2f}{c[1]:9.2f}{c[2]:9.2f}  {bore_ax}')

    if tunnels:
        print(f'\n{len(tunnels)} tunnel(s) at the {args.bore:.0f} mm bore:')
        by_axis: dict[int, list] = {}
        for name, c, axis in tunnels:
            by_axis.setdefault(axis, []).append((name, c))
        for axis, group in sorted(by_axis.items()):
            stations = sorted(c[long_axis] for _n, c in group)
            print(f'  bore along {"XYZ"[axis]}: stations '
                  + ', '.join(f'{s:+.2f}' for s in stations)
                  + (f'   separation {stations[-1] - stations[0]:.2f} mm'
                     if len(stations) == 2 else ''))
        print('\n⚠ Compare a separation against the documented value BEFORE using it.'
              '\n  A decimated mesh gave 346.6 / 520.4 here; the CAD gives 350.0 / 519.0.')

    if args.yaml:
        # cad = (mesh_x, -mesh_z, mesh_y) -- see the module docstring.
        def to_cad(c):
            return (c[0] / 1000.0, -c[2] / 1000.0, c[1] / 1000.0)
        print('\n# --- CAD frame (mesh Z -> CAD -Y), metres. Axes still need a '
              'sign decision. ---')
        print('thrusters:')
        for name, c, axis in tunnels:
            p = to_cad(c)
            # the bore axis maps the same way as a position direction
            ax = {0: (1, 0, 0), 1: (0, 0, 1), 2: (0, -1, 0)}[axis]
            print(f'  - name: {name.replace(" ", "_").lower()}\n'
                  f'    position_m: [{p[0]:.5f}, {p[1]:.5f}, {p[2]:.5f}]\n'
                  f'    axis: [{ax[0]}, {ax[1]}, {ax[2]}]\n'
                  f'    bore_mm: {args.bore:.0f}')
        print('# ⛔ The AXIAL unit is a motor, not a tunnel, so it is NOT in this list.'
              '\n#   Identify it by POSITION (on the long axis, past the last tunnel,'
              '\n#   with a duct ring just outboard). See hull_geometry.yaml.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
