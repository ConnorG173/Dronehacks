#!/usr/bin/env python3
"""Convert a CAD mesh of the course into a planner map (DHMAP point file).

Runs on a laptop, not on the drone. Needs numpy + trimesh. STEP input also
needs CadQuery (otherwise export STL/OBJ from your CAD tool).

    python3 tools/cad_to_map.py course.stl --out maps/course.bin \
        --scale 0.001 --up y --yaw-deg 0 --translate 0 0 0 --spacing 0.08

Transform, applied to every CAD vertex p (see README "Frames"):

    p_map = Rz(yaw) * U * (scale * p) + translate

  scale      CAD units -> metres (0.001 for millimetres)
  U          up-axis fix: --up z is identity; --up y maps CAD (x, y, z) to
             (x, -z, y) so the CAD +y (up) axis becomes map +z
  Rz(yaw)    rotation about map +z by --yaw-deg (counter-clockwise from above)
  translate  metres, applied last

The map frame must be right-handed with z up (GCOPTER assumes gravity along
-z). Surfaces are sampled on a barycentric grid with spacing <= --spacing,
which must be finer than VoxelWidth (default 0.08 m vs 0.25 m voxels).
"""
import argparse
import math
import os
import struct
import sys
import tempfile

import numpy as np
import trimesh


def load_mesh(path):
    if path.lower().endswith((".step", ".stp")):
        try:
            import cadquery as cq
        except ImportError:
            sys.exit("STEP input needs CadQuery (pip install cadquery), or export STL/OBJ from CAD.")
        stl = os.path.join(tempfile.mkdtemp(), "course.stl")
        cq.exporters.export(cq.importers.importStep(path), stl, tolerance=0.1, angularTolerance=0.1)
        path = stl
    mesh = trimesh.load(path, force="mesh")
    if len(mesh.faces) == 0:
        sys.exit("no triangles in " + path)
    return mesh


def sample_triangles(tris, spacing):
    """Points on every triangle (T x 3 x 3), spacing <= `spacing` along each edge."""
    edges = np.stack([np.linalg.norm(tris[:, i] - tris[:, (i + 1) % 3], axis=1) for i in range(3)], axis=1)
    ns = np.maximum(1, np.ceil(edges.max(axis=1) / spacing)).astype(int)
    out = []
    for n in np.unique(ns):
        i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1), indexing="ij")
        keep = i + j <= n
        w = np.stack([i[keep], j[keep], n - i[keep] - j[keep]], axis=1) / n  # K x 3 barycentric
        group = tris[ns == n]
        chunk = max(1, 4_000_000 // len(w))
        for s in range(0, len(group), chunk):
            out.append(np.einsum("kj,tjd->tkd", w, group[s:s + chunk]).reshape(-1, 3))
    return np.concatenate(out)


def transform_vertices(vertices, scale, up, yaw_deg, translate):
    """CAD vertices (N x 3) -> map frame: Rz(yaw) * U * (scale * p) + translate."""
    v = np.asarray(vertices, dtype=np.float64) * scale
    if up == "y":
        v = np.stack([v[:, 0], -v[:, 2], v[:, 1]], axis=1)
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return v @ np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]).T + np.array(translate, dtype=np.float64)


def map_points(tris, spacing, add_floor=False):
    """Surface samples of map-frame triangles (T x 3 x 3), deduplicated, as float32."""
    pts = sample_triangles(tris, spacing)
    if add_floor:
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        gx, gy = np.meshgrid(np.arange(lo[0], hi[0], spacing), np.arange(lo[1], hi[1], spacing))
        pts = np.concatenate([pts, np.stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)], axis=1)])
    # Drop near-duplicates (shared edges/vertices) on a spacing/2 grid.
    _, idx = np.unique(np.round(pts / (0.5 * spacing)).astype(np.int64), axis=0, return_index=True)
    return pts[np.sort(idx)].astype("<f4")


def write_map(path, pts):
    with open(path, "wb") as f:
        f.write(b"DHMAP\0\0\0" + struct.pack("<IIQ", 1, 0, len(pts)))
        f.write(pts.tobytes())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mesh", help="STL / OBJ / PLY (anything trimesh loads), or STEP with CadQuery")
    ap.add_argument("--out", required=True, help="output .bin map file")
    ap.add_argument("--scale", type=float, default=1.0, help="CAD units to metres (default 1)")
    ap.add_argument("--up", choices=["z", "y"], default="z", help="CAD up axis (default z)")
    ap.add_argument("--yaw-deg", type=float, default=0.0, help="rotation about map +z (default 0)")
    ap.add_argument("--translate", type=float, nargs=3, default=[0.0, 0.0, 0.0], metavar=("X", "Y", "Z"))
    ap.add_argument("--spacing", type=float, default=0.08, help="max sample spacing in metres (default 0.08)")
    ap.add_argument("--add-floor", action="store_true",
                    help="also sample the plane z = 0 over the mesh footprint (if the CAD has no floor)")
    args = ap.parse_args()

    mesh = load_mesh(args.mesh)
    v = transform_vertices(mesh.vertices, args.scale, args.up, args.yaw_deg, args.translate)
    pts = map_points(v[np.asarray(mesh.faces)], args.spacing, args.add_floor)
    write_map(args.out, pts)

    lo, hi = pts.min(axis=0), pts.max(axis=0)
    print("wrote %s: %d points from %d triangles" % (args.out, len(pts), len(mesh.faces)))
    print("map-frame bounds: x %.3f..%.3f  y %.3f..%.3f  z %.3f..%.3f" % (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))
    print("MapBound must contain every point you fly near; the planner treats outside MapBound as occupied.")


if __name__ == "__main__":
    main()
