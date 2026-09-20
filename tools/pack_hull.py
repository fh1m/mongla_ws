#!/usr/bin/env python3
"""Pack a GLB hull export into the compact .mgla container the site's viewer reads.

The Onshape GLB is ~12 MB of hard-edged CAD, which no web page should fetch.
This decimates it by vertex clustering, quantises positions to int16 against the
model's bounding sphere, and stores no normals at all — the viewer recovers the
facet normal from screen-space derivatives, which is the correct shading for a
machined part and costs nothing.

    python3 tools/pack_hull.py <input.glb> [--cell-mm 0.9] [-o docs/assets/cad/hull.mgla]

Container:
    'MGLA' | u16 version | u16 indexBytes | u32 verts | u32 tris
    f32 centre[3] | f32 scale | int16 pos[3]*verts | uint16|uint32 idx[3]*tris
"""
import argparse, gzip, json, struct, sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
DEFAULT_OUT = REPO / "docs" / "assets" / "cad" / "hull.mgla"

_CTYPE = {5120: "b", 5121: "B", 5122: "h", 5123: "H", 5125: "I", 5126: "f"}
_NCOMP = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}


def _read_glb(path: Path):
    data = path.read_bytes()
    if data[:4] != b"glTF":
        raise SystemExit(f"{path}: not a GLB")
    off, chunks = 12, {}
    while off < len(data):
        length, kind = struct.unpack_from("<II", data, off)
        off += 8
        chunks[kind] = data[off:off + length]
        off += length
    return json.loads(chunks[0x4E4F534A]), chunks[0x004E4942]


def _node_matrix(node):
    if "matrix" in node:
        return np.array(node["matrix"]).reshape(4, 4).T
    m = np.eye(4)
    if "scale" in node:
        m = np.diag(list(node["scale"]) + [1.0]) @ m
    if "rotation" in node:
        x, y, z, w = node["rotation"]
        rot = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        t = np.eye(4)
        t[:3, :3] = rot
        m = t @ m
    if "translation" in node:
        t = np.eye(4)
        t[:3, 3] = node["translation"]
        m = t @ m
    return m


def load_mesh(path: Path):
    """Return (vertices Nx3 float64 in metres, faces Mx3 int64), world-transformed."""
    gltf, blob = _read_glb(path)

    def accessor(i):
        acc = gltf["accessors"][i]
        view = gltf["bufferViews"][acc["bufferView"]]
        start = view.get("byteOffset", 0) + acc.get("byteOffset", 0)
        count = acc["count"] * _NCOMP[acc["type"]]
        if acc.get("normalized") or "KHR_mesh_quantization" in gltf.get("extensionsUsed", []):
            raise SystemExit("quantised accessors are not supported: export without --quantize")
        arr = np.frombuffer(blob, dtype=np.dtype(_CTYPE[acc["componentType"]]),
                            count=count, offset=start)
        return arr.reshape(acc["count"], _NCOMP[acc["type"]])

    verts, faces = [], []

    def walk(idx, parent):
        node = gltf["nodes"][idx]
        m = parent @ _node_matrix(node)
        if "mesh" in node:
            for prim in gltf["meshes"][node["mesh"]]["primitives"]:
                pos = accessor(prim["attributes"]["POSITION"]).astype(np.float64)
                pos = (m[:3, :3] @ pos.T).T + m[:3, 3]
                tri = accessor(prim["indices"]).ravel().astype(np.int64).reshape(-1, 3)
                faces.append(tri + sum(len(v) for v in verts))
                verts.append(pos)
        for child in node.get("children", []):
            walk(child, m)

    scene = gltf["scenes"][gltf.get("scene", 0)]
    for idx in scene["nodes"]:
        walk(idx, np.eye(4))
    return np.vstack(verts), np.vstack(faces)


def cluster(verts, faces, cell_m):
    """Vertex-cluster decimation: merge every vertex inside a cell_m cube."""
    lo = verts.min(0)
    grid = np.floor((verts - lo) / cell_m).astype(np.int64)
    key = grid[:, 0] * 1_000_003 + grid[:, 1] * 1009 + grid[:, 2]
    _, inverse = np.unique(key, return_inverse=True)
    merged = np.zeros((inverse.max() + 1, 3))
    np.add.at(merged, inverse, verts)
    merged /= np.bincount(inverse, minlength=len(merged))[:, None]
    tri = inverse[faces]
    keep = (tri[:, 0] != tri[:, 1]) & (tri[:, 1] != tri[:, 2]) & (tri[:, 0] != tri[:, 2])
    return merged, tri[keep]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("glb", type=Path, help="GLB exported from CAD, Y-up, uncompressed")
    ap.add_argument("--cell-mm", type=float, default=0.9,
                    help="clustering cell in mm (default 0.9 — keeps an 84 mm bore round)")
    ap.add_argument("-o", "--out", type=Path, default=DEFAULT_OUT)
    args = ap.parse_args(argv)

    verts, faces = load_mesh(args.glb)
    print(f"in   {len(faces):>8,} tris  {len(verts):>8,} verts")
    verts, faces = cluster(verts, faces, args.cell_mm / 1000.0)
    print(f"out  {len(faces):>8,} tris  {len(verts):>8,} verts   (cell {args.cell_mm} mm)")

    lo, hi = verts.min(0), verts.max(0)
    size_mm = (hi - lo) * 1000
    print(f"bbox {size_mm[0]:.1f} x {size_mm[1]:.1f} x {size_mm[2]:.1f} mm")

    centre = (lo + hi) / 2
    scale = float((hi - lo).max() / 2)
    quant = np.clip(np.round((verts - centre) / scale * 32767), -32767, 32767).astype("<i2")
    idx_dtype = "<u2" if len(verts) < 65536 else "<u4"
    idx = faces.astype(idx_dtype)

    header = struct.pack("<4sHHII3ff", b"MGLA", 1, np.dtype(idx_dtype).itemsize,
                         len(verts), len(faces), *centre, scale)
    blob = header + quant.tobytes() + idx.tobytes()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_bytes(blob)
    print(f"wrote {args.out.relative_to(REPO)}  {len(blob) / 1024:.0f} KB "
          f"({len(gzip.compress(blob, 9)) / 1024:.0f} KB gzipped)")


if __name__ == "__main__":
    sys.exit(main())
