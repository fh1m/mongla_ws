#!/usr/bin/env python3

"""Generate prop meshes that primitives cannot express -- starting with a plate
that has REAL HOLES in it.

WHY THIS EXISTS
---------------
The torpedo board's collision was genuinely open (a plate tiled into strips
around each opening) but its VISUAL was a single solid box with dark disks
painted on the texture. So the opening never parallaxed, never showed water or
props behind it, and never responded to light or fog. A detector trained on
that learns A PAINTED BULLSEYE, NOT A HOLE -- which is exactly the sim-to-pool
failure this simulator exists to prevent, and it is why the rendered board read
as clip art next to a photograph.

Resolution was never the bottleneck: the panel is already 512 px across 0.6 m,
about 853 px/m, some thirteen times the pool floor. What it lacked was a hole.

An SDF box cannot have a hole, so the visual becomes a mesh. The hole list comes
from `prop_library.torpedo_openings()` -- the SAME list the collision strips are
tiled from -- because those two drifted apart once already and a shot lined up
on the artwork struck solid board with nothing in any log to say why.

WHAT IT WRITES
--------------
Wavefront OBJ, because it is the one mesh format that is trivially generated,
diffable in review, and loaded by gz-sim without a converter. UVs are emitted
so the printed artwork lands square on the face: a box face maps its texture
exactly once, and a mesh has to be told to do the same.
"""

from __future__ import annotations

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import prop_library as pl                                        # noqa: E402

MESH_DIR = os.path.join(os.path.dirname(HERE), "models", "robosub_meshes",
                        "meshes")

# Segments per hole rim. 48 is smooth at the standoff a torpedo is fired from
# (1.0-1.5 m) without turning one board into a heavy mesh -- and RTF here is
# render-bound, so triangle count is not free.
HOLE_SEGMENTS = 48


def _ring(cy, cz, radius, segments):
    """Points around a hole, counter-clockwise seen from +x."""
    return [(cy + radius * math.cos(2.0 * math.pi * i / segments),
             cz + radius * math.sin(2.0 * math.pi * i / segments))
            for i in range(segments)]


def plate_with_holes(size, thickness, holes, segments=HOLE_SEGMENTS):
    """A square plate in the y-z plane with circular holes cut through it.

    Triangulated as one annular fan per hole against the plate's own corners:
    each hole owns the quadrant-ish span of border nearest it, so the surface
    between hole rims and the outer edge is covered without a general-purpose
    polygon triangulator. The plate is split into `segments` wedges radiating
    from each hole centre, which keeps every triangle well-formed.

    Returns (vertices, uvs, faces) with faces as 1-based OBJ index triples.
    """
    half = size / 2.0
    verts, uvs, faces = [], [], []

    def add(y, z, x):
        verts.append((x, y, z))
        # BOTH AXES ARE FLIPPED RELATIVE TO `prop_library._plate_uv`, and that
        # is deliberate -- measured off a front-camera render, not reasoned to.
        #
        # `_plate_uv` is the AUTHORING mapping: where the texture generator
        # PAINTS a ring or an image for a given plate coordinate. This is the
        # SAMPLING mapping, and two flips sit between them:
        #
        #   u  the plate is a mesh now, not a box, so Ogre's box-face convention
        #      no longer applies. Looking at the printed (+x) face means looking
        #      DOWN -x, which puts +y on the viewer's right -- so u has to grow
        #      with y or the artwork renders mirrored.
        #   v  OBJ `vt` is bottom-origin while PIL row 0 is the top of the image.
        #
        # The first render after the packer landed showed the panel rotated a
        # clean 180 degrees, which is exactly both flips at once. Flip these in
        # `_plate_uv` INSTEAD and nothing changes: the generator would paint at
        # the same place the mesh then samples, and the error cancels itself.
        uvs.append((0.5 + y / size, 0.5 + z / size))
        return len(verts)

    # A regular grid over the plate, with the cells that fall inside a hole
    # dropped and the cells straddling a rim snapped to it. Simple, robust, and
    # the artwork is what carries the detail -- the geometry only has to be
    # honestly open.
    # 96, not 64. The grid is snapped outward onto each rim and cells whose
    # centre falls inside a hole are dropped, so the cut overshoots the true
    # radius by up to half a cell. At 64 that overshoot ate the printed red
    # annulus around the SMALL openings entirely -- visible in a render as two
    # rimmed holes and two bare ones, on the diagonal that separates large from
    # small. Finer grid, tighter cut, rim survives.
    n = 96
    step = size / n
    grid = {}
    for i in range(n + 1):
        for j in range(n + 1):
            y = -half + i * step
            z = -half + j * step
            inside = None
            for (hy, hz, r) in holes:
                d = math.hypot(y - hy, z - hz)
                if d < r:
                    inside = (hy, hz, r, d)
                    break
            if inside is not None:
                hy, hz, r, d = inside
                if d < 1e-9:
                    grid[(i, j)] = None            # exact centre: no snap
                    continue
                # snap outward onto the rim so the hole edge is clean
                y = hy + (y - hy) / d * r
                z = hz + (z - hz) / d * r
                grid[(i, j)] = ("rim", y, z)
            else:
                grid[(i, j)] = ("out", y, z)

    def hole_of(y, z):
        for k, (hy, hz, r) in enumerate(holes):
            if math.hypot(y - hy, z - hz) < r - 1e-6:
                return k
        return None

    idx_front, idx_back = {}, {}
    for key, val in grid.items():
        if val is None:
            continue
        _, y, z = val
        idx_front[key] = add(y, z, +thickness / 2.0)
        idx_back[key] = add(y, z, -thickness / 2.0)

    for i in range(n):
        for j in range(n):
            quad = [(i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)]
            if any(grid.get(q) is None for q in quad):
                continue
            pts = [grid[q][1:] for q in quad]
            # drop a cell whose whole span lies inside a hole (all four corners
            # snapped to the same rim means it collapsed onto the edge)
            cy = sum(pt[0] for pt in pts) / 4.0
            cz = sum(pt[1] for pt in pts) / 4.0
            if hole_of(cy, cz) is not None:
                continue
            a, b, c, d = (idx_front[q] for q in quad)
            faces.append((a, b, c))
            faces.append((a, c, d))
            a, b, c, d = (idx_back[q] for q in quad)
            faces.append((a, c, b))                 # reversed: back face
            faces.append((a, d, c))

    # Rim walls, so a hole seen off-axis shows plate thickness rather than a
    # paper-thin slit -- that edge is a real cue for how far off-centre a shot
    # is lined up.
    for (hy, hz, r) in holes:
        ring = _ring(hy, hz, r, segments)
        base = len(verts)
        for (y, z) in ring:
            add(y, z, +thickness / 2.0)
            add(y, z, -thickness / 2.0)
        for k in range(segments):
            f0 = base + 2 * k + 1
            b0 = base + 2 * k + 2
            f1 = base + 2 * ((k + 1) % segments) + 1
            b1 = base + 2 * ((k + 1) % segments) + 2
            faces.append((f0, b0, b1))
            faces.append((f0, b1, f1))

    # Vertex normals. NOT optional: without them the loader hands Ogre2 a mesh
    # with no shading basis, the PBR material never samples its albedo map, and
    # the board renders as a FLAT WHITE plate -- geometry perfect, artwork gone,
    # and nothing in the log to say so (the mesh itself loads fine).
    norms = []
    for (x, y, z) in verts:
        if abs(x) >= thickness / 2.0 - 1e-9:
            norms.append((1.0 if x > 0 else -1.0, 0.0, 0.0))
        else:
            norms.append((0.0, 0.0, 1.0))
    return verts, uvs, faces, norms


def lattice_panel(width, height, thickness, cols, rows, bar):
    """A rectangular panel with a grid of rectangular holes -- a crate wall.

    The real CleverMade crate the rulebook names has latticed walls, and you can
    see into and through them. That matters here for two reasons: the downward
    camera has to see a marker land INSIDE a crate, and a solid slab reads as a
    featureless black box in fog, which is exactly what the render showed.

    It is GEOMETRY rather than an alpha-cutout texture, and not by preference:
    `SetAlphaFromTexture` exists only in the gz-rendering C++ API, there is no
    SDF element for it, and `visual()` exposes only a scalar `<transparency>`.
    So a see-through wall has to be built, not painted. The bars are quads, so
    a whole wall is a few hundred triangles rather than the tens of thousands a
    hole-cutting grid would cost.

    Returns (verts, uvs, faces, norms) like `plate_with_holes`.
    """
    verts, uvs, faces, norms = [], [], [], []
    half_w, half_h, half_t = width / 2.0, height / 2.0, thickness / 2.0

    def slab(y0, y1, z0, z1):
        """One bar, as a box. Six quads; hard edges need their own normals."""
        base = len(verts)
        corners = [(y0, z0), (y1, z0), (y1, z1), (y0, z1)]
        for nx, x in ((1.0, half_t), (-1.0, -half_t)):
            for (y, z) in corners:
                verts.append((x, y, z))
                uvs.append((0.5 + y / width, 0.5 + z / height))
                norms.append((nx, 0.0, 0.0))
        f, b = base + 1, base + 5
        faces.extend([(f, f + 1, f + 2), (f, f + 2, f + 3),
                      (b, b + 2, b + 1), (b, b + 3, b + 2)])
        # Four side walls, so a bar seen edge-on has thickness.
        for k in range(4):
            a0, a1 = f + k, f + (k + 1) % 4
            b0, b1 = b + k, b + (k + 1) % 4
            faces.extend([(a0, b0, b1), (a0, b1, a1)])

    # Vertical bars, then horizontal ones. The outer frame is one bar each side,
    # so the wall keeps a solid rim like the moulded crate does.
    pitch_y = width / cols
    pitch_z = height / rows
    for i in range(cols + 1):
        y = -half_w + i * pitch_y
        slab(max(-half_w, y - bar / 2.0), min(half_w, y + bar / 2.0),
             -half_h, half_h)
    for j in range(rows + 1):
        z = -half_h + j * pitch_z
        slab(-half_w, half_w,
             max(-half_h, z - bar / 2.0), min(half_h, z + bar / 2.0))
    return verts, uvs, faces, norms


# The water surface used to be a <box>: 8 vertices, which is why the animated
# surface was carried and cut across five rounds. A vertex-displacement shader
# needs vertices to displace, so the surface is a subdivided grid instead.
#
# One mesh PER POOL, not a unit mesh with <scale>. Gazebo would happily scale a
# 1x1 grid to 25x16, but the Gerstner vertex shader runs in OBJECT space and the
# scale is applied afterwards -- a non-uniform scale would stretch the waves by
# 25/16 along one axis and leave the wavelength meaningless. Two pools, two
# meshes, wavelengths that mean metres in both.
WATER_CELL_M = 0.25


def water_grid(length, width, cell=WATER_CELL_M):
    """A flat z=0 grid spanning exactly length x width, centred on the origin.

    UVs run 0..1 across the sheet so the existing water_surface.png tiles the
    same way it did on the box face.
    """
    nx = max(2, int(round(length / cell)))
    ny = max(2, int(round(width / cell)))
    verts, uvs, norms, faces = [], [], [], []
    for j in range(ny + 1):
        for i in range(nx + 1):
            u = i / nx
            v = j / ny
            verts.append((-length / 2.0 + u * length,
                          -width / 2.0 + v * width, 0.0))
            uvs.append((u, v))
            norms.append((0.0, 0.0, 1.0))
    # DOUBLE-SIDED, and this is the whole reason the surface rendered NOTHING
    # on the first attempt: a sheet wound to face +z is back-facing to every
    # camera in this simulator, all of which are UNDER it, and Ogre2 culls back
    # faces by default. It is not an error and nothing logs it. The control that
    # caught it was painting the surface opaque magenta: a <box> came back 100 %
    # magenta and this mesh came back 0.00 % with the same material at the same
    # pose. Emitting the mirrored winding costs 2x triangles on a sheet that is
    # ~8k, against a sim that is CPU-bound and 25 % GPU.
    base = len(verts)
    verts.extend(verts[:base])
    uvs.extend(uvs[:base])
    norms.extend([(0.0, 0.0, -1.0)] * base)
    for j in range(ny):
        for i in range(nx):
            a = j * (nx + 1) + i + 1          # OBJ indices are 1-based
            b = a + 1
            c = a + (nx + 1)
            d = c + 1
            faces.append((a, b, d))
            faces.append((a, d, c))
            # underside: same quad, reversed winding
            faces.append((a + base, d + base, b + base))
            faces.append((a + base, c + base, d + base))
    return verts, uvs, faces, norms


def write_obj(path, verts, uvs, faces, norms, name="plate"):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as fh:
        fh.write("# generated by gen_prop_meshes.py -- do not hand-edit\n")
        fh.write(f"o {name}\n")
        for (x, y, z) in verts:
            fh.write(f"v {x:.6f} {y:.6f} {z:.6f}\n")
        for (u, v) in uvs:
            fh.write(f"vt {u:.6f} {v:.6f}\n")
        for (x, y, z) in norms:
            fh.write(f"vn {x:.6f} {y:.6f} {z:.6f}\n")
        for f in faces:
            fh.write("f " + " ".join(f"{i}/{i}/{i}" for i in f) + "\n")
    return path


def main():
    import yaml

    spec_path = os.path.join(os.path.dirname(HERE), "spec", "robosub.yaml")
    with open(spec_path) as fh:
        spec = yaml.safe_load(fh)
    cfg = spec["props"]["torpedo_board"]
    holes = pl.torpedo_openings(spec)
    verts, uvs, faces, norms = plate_with_holes(
        cfg["size"], cfg["thickness"], holes)
    out = write_obj(os.path.join(MESH_DIR, "torpedo_plate.obj"),
                    verts, uvs, faces, norms, name="torpedo_plate")
    print(f"wrote {out}  ({len(verts)} verts, {len(faces)} tris, "
          f"{len(holes)} holes)")

    binc = spec["props"]["bin"]
    verts, uvs, faces, norms = lattice_panel(
        binc["length"], binc["height"], binc["wall"],
        binc.get("lattice_cols", 5), binc.get("lattice_rows", 4),
        binc.get("lattice_bar", 0.022))
    out = write_obj(os.path.join(MESH_DIR, "crate_wall.obj"),
                    verts, uvs, faces, norms, name="crate_wall")
    print(f"wrote {out}  ({len(verts)} verts, {len(faces)} tris, latticed)")

    for comp in ("robosub", "sauvc"):
        with open(os.path.join(os.path.dirname(HERE), "spec",
                               f"{comp}.yaml")) as fh:
            poolc = yaml.safe_load(fh)["pool"]
        verts, uvs, faces, norms = water_grid(poolc["length"], poolc["width"])
        out = write_obj(os.path.join(MESH_DIR, f"water_{comp}.obj"),
                        verts, uvs, faces, norms, name=f"water_{comp}")
        print(f"wrote {out}  ({len(verts)} verts, {len(faces)} tris, "
              f'{poolc["length"]}x{poolc["width"]} m)')

    cfgdir = os.path.dirname(MESH_DIR)
    with open(os.path.join(cfgdir, "model.config"), "w") as fh:
        fh.write('<?xml version="1.0"?>\n<model>\n'
                 '  <name>robosub_meshes</name>\n  <version>1.0</version>\n'
                 '  <sdf version="1.9">model.sdf</sdf>\n'
                 '  <description>Generated prop meshes (geometry only).'
                 '</description>\n</model>\n')
    with open(os.path.join(cfgdir, "model.sdf"), "w") as fh:
        fh.write('<?xml version="1.0"?>\n<sdf version="1.9">\n'
                 '  <model name="robosub_meshes">\n    <static>true</static>\n'
                 '  </model>\n</sdf>\n')


if __name__ == "__main__":
    main()
