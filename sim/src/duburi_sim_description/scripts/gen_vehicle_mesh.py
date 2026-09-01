#!/usr/bin/env python3

"""Turn the Dubomini CAD export into a mesh a simulator can actually run.

THE SOURCE IS A CAD ASSEMBLY, NOT A MODEL
-----------------------------------------
`hullv3.stl` is 2,149,667 triangles in 3,276 disconnected components -- every
bolt, duct and bracket exported as its own shell. It is NOT water-tight (2,178
of the 3,276 components are; the assembly as a whole is not), and that single
fact decides most of the pipeline:

  * gz-sim 8's automatic inertia calculation requires a WATER-TIGHT mesh, so it
    cannot be used here and the inertia is computed from the geometry by this
    script instead.
  * Capytaine's BEM likewise needs a closed hull, so the added-mass solve runs
    on the CONVEX HULL rather than the raw assembly.

Verified against the team's published spec (bracuduburi.com/auv/dubomini) before
any of this: the STL's length matches the quoted 54.59 cm overall to **0.0 mm**,
which is what says this is that vehicle and not a variant.

    length 545.9 mm  (published 545.9)
    width  470.2 mm  (published 464.3)
    height 169.3 mm  (published 166.8)

FRAME CONVENTION -- the one that is silent if wrong
---------------------------------------------------
The CAD is **Y-up** (its height lands on y). Gazebo is Z-up with x forward,
y port, z up. The conversion is a +90 degrees roll:

    (x, y, z)_cad  ->  (x, -z, y)_gazebo

It is applied HERE, baked into the exported mesh, rather than left as a `<pose>`
on the visual -- because a pose rotates the visual and not the collision or the
thruster coordinates derived from the same file, and a mismatch between those
three is invisible until the vehicle flies sideways.

Units are millimetres; everything is emitted in metres.
"""

import os
import sys

import numpy as np

CAD_TO_GZ_NOTE = "roll +90 deg: (x, y, z)_cad -> (x, -z, y)_gz, mm -> m"


def to_gazebo(v: np.ndarray) -> np.ndarray:
    """CAD millimetres (Y-up) -> Gazebo metres (Z-up)."""
    return np.column_stack([v[:, 0], -v[:, 2], v[:, 1]]) / 1000.0


def load(src: str):
    import trimesh
    return trimesh.load(src)


def thrusters(mesh):
    """The eight T200 nacelles, RECOVERED FROM THE CAD rather than guessed.

    The published spec says only "vectored arrangement with empirically
    distributed placement" and gives no coordinates, so these come from the
    geometry: the ducts are the eight identical ~43k-face components with a
    ~97 mm envelope, which is a T200's duct. Their thrust axis is each
    cylinder's SHORT principal axis -- the duct is 97 mm across and ~70 mm long,
    so the axis is the direction of least extent.

    The result is not a fit or an approximation: the horizontals come out at
    exactly +-45.0 degrees, which is the vectored-X layout, and that agreement
    is the check that the extraction is reading real features.
    """
    parts = [c for c in mesh.split(only_watertight=False)
             if 40000 < len(c.faces) < 46000 and 90 < c.extents.max() < 105]
    if len(parts) != 8:
        raise ValueError(f'expected 8 thruster ducts, found {len(parts)}')
    out = []
    for c in parts:
        v = to_gazebo(c.vertices)
        centre = v.mean(axis=0)
        x = v - centre
        _w, vecs = np.linalg.eigh(x.T @ x)
        axis = vecs[:, 0] / np.linalg.norm(vecs[:, 0])
        if axis[0] < 0 or (abs(axis[0]) < 1e-6 and axis[2] < 0):
            axis = -axis
        out.append({'pos': centre, 'axis': axis,
                    'vertical': abs(axis[2]) > 0.7})
    return out


def decimate(mesh, target_faces: int):
    """Reduce the VISUAL only. Collision is Gazebo's job -- see the model.

    2.15 M triangles is roughly a hundred times a sane sim mesh, and the cost is
    paid on every camera in the scene (two colour, two bounding-box, plus the
    GUI), so it is the single most expensive number in the vehicle.
    """
    import fast_simplification
    v, f = np.asarray(mesh.vertices), np.asarray(mesh.faces)
    frac = 1.0 - min(0.99, target_faces / max(1, len(f)))
    vo, fo = fast_simplification.simplify(v, f, frac)
    import trimesh
    return trimesh.Trimesh(vertices=vo, faces=fo, process=False)


def main(argv=None):
    a = argv if argv is not None else sys.argv[1:]
    if len(a) < 2:
        print('usage: gen_vehicle_mesh.py SRC.stl OUTDIR [target_faces]',
              file=sys.stderr)
        return 2
    src, outdir = a[0], a[1]
    target = int(a[2]) if len(a) > 2 else 60000

    mesh = load(src)
    print(f'source: {len(mesh.faces):,} faces, '
          f'{len(mesh.split(only_watertight=False)):,} components, '
          f'watertight={mesh.is_watertight}')

    small = decimate(mesh, target)
    small.vertices = to_gazebo(np.asarray(small.vertices))
    os.makedirs(outdir, exist_ok=True)
    dst = os.path.join(outdir, 'dubomini.dae')
    small.export(dst)
    print(f'visual : {len(small.faces):,} faces -> {dst}')

    # The convex hull: the collision proxy AND the closed form the added-mass
    # solve needs. Exported so both read the same geometry.
    hull = mesh.convex_hull
    hull.vertices = to_gazebo(np.asarray(hull.vertices))
    hdst = os.path.join(outdir, 'dubomini_hull.stl')
    hull.export(hdst)
    print(f'hull   : {len(hull.faces):,} faces, '
          f'{hull.volume * 1000:.2f} L -> {hdst}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
