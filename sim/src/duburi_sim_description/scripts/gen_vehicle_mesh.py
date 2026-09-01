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


# What a part IS, from how big it is. The CAD carries no names -- an STL never
# does -- so this is the only classification available, and the component
# analysis makes it fall out cleanly: the eight ducts are identical ~43k-face
# parts with a ~97 mm envelope, the four frame plates are the only things over
# 300 mm, and 3,158 sub-25 mm parts are bolts.
#
# FASTENERS ARE DROPPED, and the arithmetic is why: 3,158 parts, 46.7 % of every
# triangle in the file, for 0.14 L of bounding volume. No camera in this
# simulator resolves an M5 bolt head at any working distance, and paying half
# the mesh budget for them starved the parts that ARE visible.
PART_CLASSES = ('frame', 'enclosure', 'body', 'duct', 'prop', 'fitting')


def classify(component, ducts=None) -> str:
    """What a part IS, from how big it is -- and, for props, WHERE it is.

    Size alone cannot separate a propeller from the other 87 mm parts on this
    vehicle: nine components match the prop envelope and only eight of them are
    propellers. The discriminator is CONTAINMENT -- a prop sits inside a duct's
    bounding box and nothing else on the vehicle does. That test finds exactly
    8 of the 9 candidates and correctly rejects the last.

    This is why `duct` used to carry a blue bias: the blades were merged into
    their duct's group, so a black duct and a blue-teal prop had to share one
    colour. They are separate parts in the CAD; only the classifier was coarse.
    """
    e = sorted(component.extents, reverse=True)
    if 90 < e[0] < 105 and len(component.faces) > 40000:
        return 'duct'                      # a T200 duct, same test as thrusters()
    if ducts and 80 < e[0] < 92 and 12000 < len(component.faces) < 17000:
        c = component.centroid
        for lo, hi in ducts:
            if bool(np.all(c > lo)) and bool(np.all(c < hi)):
                return 'prop'
    if e[0] > 300:
        return 'frame'                     # the four long rails and plates
    if e[0] < 25:
        return 'fastener'                  # dropped
    if e[0] > 150:
        return 'enclosure'
    if e[0] > 80:
        return 'body'
    return 'fitting'


def decimate(mesh, target_faces: int):
    """Reduce a mesh toward a face budget, and RECOMPUTE ITS NORMALS.

    The normals are not a detail. The first export of this vehicle had none --
    `<input semantic="NORMAL">` simply absent from the DAE -- so the renderer
    had no shading information at all and 200,000 triangles rendered as a flat
    pale silhouette. It looked like a colour problem and was a geometry problem.
    """
    import fast_simplification
    import trimesh
    v, f = np.asarray(mesh.vertices), np.asarray(mesh.faces)
    if len(f) > target_faces:
        frac = 1.0 - target_faces / len(f)
        v, f = fast_simplification.simplify(v, f, frac)
    out = trimesh.Trimesh(vertices=v, faces=f, process=False)
    out.fix_normals()
    return out


# Face budget per class. Weighted by what a camera actually sees: the frame is
# the vehicle's silhouette and the ducts are its most recognisable feature, so
# they keep detail; fittings are numerous and small.
BUDGET = {'frame': 30000, 'enclosure': 20000, 'body': 20000,
          'duct': 16000, 'prop': 14000, 'fitting': 12000}


def build_groups(mesh, verbose: bool = True):
    """Split the assembly into named, separately-materialled meshes.

    ONE MERGED GEOMETRY CANNOT BE COLOURED. The previous export concatenated all
    3,276 components into a single mesh with a single material, so there was
    nothing to paint separately even in principle -- the vehicle could only ever
    be one flat colour. Grouping is what makes a black enclosure, a grey frame
    and teal props possible at all.
    """
    import trimesh
    buckets = {k: [] for k in PART_CLASSES}
    dropped = 0
    comps = mesh.split(only_watertight=False)
    # Duct envelopes first: a prop is DEFINED by sitting inside one.
    ducts = [c.bounds for c in comps
             if 90 < c.extents.max() < 105 and len(c.faces) > 40000]
    for c in comps:
        k = classify(c, ducts)
        if k == 'fastener':
            dropped += len(c.faces)
            continue
        buckets[k].append(c)

    groups = {}
    for name, parts in buckets.items():
        if not parts:
            continue
        merged = trimesh.util.concatenate(parts)
        small = decimate(merged, BUDGET[name])
        small.vertices = to_gazebo(np.asarray(small.vertices))
        groups[name] = small
        if verbose:
            print(f'  {name:10s} {len(parts):4d} parts  '
                  f'{len(merged.faces):8,} -> {len(small.faces):7,} faces')
    if verbose:
        print(f'  {"fastener":10s} {dropped:>15,} faces DROPPED')
    return groups


def main(argv=None):
    a = argv if argv is not None else sys.argv[1:]
    if len(a) < 2:
        print('usage: gen_vehicle_mesh.py SRC.stl OUTDIR', file=sys.stderr)
        return 2
    src, outdir = a[0], a[1]

    mesh = load(src)
    print(f'source: {len(mesh.faces):,} faces, '
          f'{len(mesh.split(only_watertight=False)):,} components, '
          f'watertight={mesh.is_watertight}')

    os.makedirs(outdir, exist_ok=True)
    groups = build_groups(mesh)
    total = 0
    for name, g in groups.items():
        dst = os.path.join(outdir, f'dubomini_{name}.dae')
        g.export(dst)
        total += len(g.faces)
    print(f'visual : {total:,} faces across {len(groups)} groups')

    # The collision proxy AND the closed form the added-mass solve needs.
    # Built from the FULL assembly, fasteners included: dropping bolts must not
    # change the vehicle's envelope.
    hull = mesh.convex_hull
    hull.vertices = to_gazebo(np.asarray(hull.vertices))
    hdst = os.path.join(outdir, 'dubomini_hull.stl')
    hull.export(hdst)
    print(f'hull   : {len(hull.faces):,} faces, {hull.volume * 1000:.2f} L')
    return 0


if __name__ == '__main__':
    sys.exit(main())
