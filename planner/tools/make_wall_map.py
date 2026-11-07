#!/usr/bin/env python3
"""Write the synthetic wall course used by the tests and config/course.yaml.

Map frame (metres, z up). MapBound: x -1..13, y -6..6, z 0..4.
  - floor: the plane z = 0
  - wall:  x 5.0..5.4, full width and height, with one opening
           y 1.0..4.0, z 0.6..3.0
Points are on a 0.1 m grid offset by 0.05 m (finer than the 0.25 m voxels,
and never exactly on a voxel boundary). Python standard library only.

    python3 tools/make_wall_map.py maps/wall_demo.bin
"""
import struct
import sys

MAP_BOUND = (-1.0, 13.0, -6.0, 6.0, 0.0, 4.0)
WALL_X = (5.0, 5.4)
OPENING_Y = (1.0, 4.0)
OPENING_Z = (0.6, 3.0)


def grid(lo, hi, step=0.1):
    n = int(round((hi - lo) / step))
    return [lo + step * (i + 0.5) for i in range(n)]


def wall_course_points():
    x0, x1, y0, y1, z0, z1 = MAP_BOUND
    pts = [(x, y, 0.05) for x in grid(x0, x1) for y in grid(y0, y1)]  # floor
    for x in grid(*WALL_X):
        for y in grid(y0, y1):
            for z in grid(z0, z1):
                in_opening = OPENING_Y[0] < y < OPENING_Y[1] and OPENING_Z[0] < z < OPENING_Z[1]
                if not in_opening:
                    pts.append((x, y, z))
    return pts


def write_map(path, pts):
    """DHMAP v1: b'DHMAP\\0\\0\\0', uint32 version=1, uint32 0, uint64 N, N x float32 xyz (LE)."""
    with open(path, "wb") as f:
        f.write(b"DHMAP\0\0\0" + struct.pack("<IIQ", 1, 0, len(pts)))
        f.write(struct.pack("<%df" % (3 * len(pts)), *[c for p in pts for c in p]))


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    pts = wall_course_points()
    write_map(sys.argv[1], pts)
    print("wrote %s: %d points, MapBound %s" % (sys.argv[1], len(pts), " ".join(str(b) for b in MAP_BOUND)))
