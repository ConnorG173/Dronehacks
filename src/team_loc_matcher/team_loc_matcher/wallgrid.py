"""Course map (DHMAP occupied points, planner/README.md) -> 2-D wall grid for the matcher. PURE numpy.

Derived at node startup from the published layout (no saved artefact, rule 6.4; nothing
hand-edited). Nothing course-specific lives here: the only inputs are the map file, the start
position (to tell the inside of the cage from the outside) and band/resolution parameters.

Recipe:
1. Points inside a height band (default 0.3-1.6 m above the floor) are binned in res-m cells. A
   cell is a WALL candidate if its points span >= min_span_m of height: vertical faces only, so the
   floor, roof, boulders and low barriers drop out.
2. Flood fill over non-wall cells (walls closed by one cell, so 4 cm sampling cannot leak) from
   the grid border = OUTSIDE, and from the start position = INSIDE. If they meet, the map is not a
   closed cage and we fail loudly.
3. Wall cells touching OUTSIDE are the cage boundary (nets: the camera sees through them). They are
   not walls, and neither are wall cells within net_margin_m of them (wall ends meet the net, and
   as-built gaps there are common). The cage mask is everything not OUTSIDE and not boundary.
4. dist_m = distance to the nearest wall cell; lik = exp(-d^2 / 2 sigma^2) inside the cage, 0
   outside (returns from the net and the room beyond score the same at every pose).

Output dict keys (match.CourseMap.from_grid): wall, cage, lik, dist_m, res_m, origin_x, origin_y,
start_xy, start_heading_rad, band_lo_m, band_hi_m, boundary.
"""
from __future__ import annotations

import math

import numpy as np

MAGIC = b"DHMAP\0\0\0"


def read_dhmap(path):
    raw = open(path, "rb").read()
    if raw[:8] != MAGIC:
        raise ValueError(f"{path}: not a DHMAP file")
    ver, _ = np.frombuffer(raw, "<u4", 2, 8)
    if ver != 1:
        raise ValueError(f"{path}: DHMAP version {ver} unsupported")
    n = int(np.frombuffer(raw, "<u8", 1, 16)[0])
    p = np.frombuffer(raw, "<f4", n * 3, 24).reshape(n, 3).astype(np.float64)
    return p[np.isfinite(p).all(1)]


def read_course_config(path):
    """Numbers per key from the planner's flat config (first occurrence of each key)."""
    out = {}
    for line in open(path):
        line = line.split("#")[0].strip()
        if ":" not in line:
            continue
        k, v = (s.strip() for s in line.split(":", 1))
        try:
            nums = [float(x) for x in v.replace("[", " ").replace("]", " ").replace(",", " ").split()]
        except ValueError:
            continue
        if nums and k not in out:
            out[k] = nums
    return out


def _dilate(m, r=1, conn8=True):
    """Binary dilation by r cells (square if conn8, else cross), edges not wrapped."""
    out = m.copy()
    for _ in range(r):
        a = out.copy()
        a[1:, :] |= out[:-1, :]
        a[:-1, :] |= out[1:, :]
        a[:, 1:] |= out[:, :-1]
        a[:, :-1] |= out[:, 1:]
        if conn8:
            a[1:, 1:] |= out[:-1, :-1]
            a[:-1, :-1] |= out[1:, 1:]
            a[1:, :-1] |= out[:-1, 1:]
            a[:-1, 1:] |= out[1:, :-1]
        out = a
    return out


def _flood(seed, passable):
    """4-connected flood fill of `passable` cells from `seed` (bool masks)."""
    m = seed & passable
    while True:
        n = _dilate(m, 1, conn8=False) & passable
        if (n == m).all():
            return m
        m = n


def _dist_to(mask, res, chunk=4096):
    """Exact Euclidean distance (m) from every cell centre to the nearest True cell centre."""
    src = np.argwhere(mask).astype(np.float64)
    shape = mask.shape
    if len(src) == 0:
        return np.full(shape, np.inf)
    allc = np.indices(shape).reshape(2, -1).T.astype(np.float64)
    d = np.empty(len(allc))
    for i in range(0, len(allc), chunk):
        c = allc[i:i + chunk]
        d[i:i + chunk] = np.sqrt(((c[:, None, :] - src[None, :, :]) ** 2).sum(-1).min(1))
    return (d * res).reshape(shape)


def build_wallgrid(points, start_xy, start_heading_rad=0.0, bound_xy=None, res=0.05, band=(0.3, 1.6),
                   min_span_m=0.15, min_hits=2, net_margin_m=0.15, sigma_m=0.08, floor_z=None,
                   net_mode="weak", net_band_m=0.25, net_neutral_lik=0.5):
    """points: (N, 3) map frame. bound_xy: (xmin, xmax, ymin, ymax) of the grid (default: points'
    extent plus 3 cells). floor_z: floor height (default: the lowest 1st percentile of points)."""
    P = np.asarray(points, float)
    if floor_z is None:
        floor_z = float(np.percentile(P[:, 2], 1))
    lo, hi = floor_z + band[0], floor_z + band[1]
    if bound_xy is None:
        m = 3 * res
        bound_xy = (P[:, 0].min() - m, P[:, 0].max() + m, P[:, 1].min() - m, P[:, 1].max() + m)
    x0, x1, y0, y1 = (float(v) for v in bound_xy)
    nx, ny = int(math.ceil((x1 - x0) / res)), int(math.ceil((y1 - y0) / res))

    B = P[(P[:, 2] >= lo) & (P[:, 2] <= hi)]
    ij = np.floor((B[:, :2] - (x0, y0)) / res).astype(np.int64)
    ok = (ij[:, 0] >= 0) & (ij[:, 0] < nx) & (ij[:, 1] >= 0) & (ij[:, 1] < ny)
    ij, z = ij[ok], B[ok, 2]
    cnt = np.zeros((nx, ny), np.int64)
    zmin = np.full((nx, ny), np.inf)
    zmax = np.full((nx, ny), -np.inf)
    np.add.at(cnt, (ij[:, 0], ij[:, 1]), 1)
    np.minimum.at(zmin, (ij[:, 0], ij[:, 1]), z)
    np.maximum.at(zmax, (ij[:, 0], ij[:, 1]), z)
    occ = (cnt >= min_hits) & (zmax - zmin >= min_span_m)

    closed = _dilate(occ, 1)
    border = np.zeros_like(occ)
    border[0, :] = border[-1, :] = border[:, 0] = border[:, -1] = True
    outside = _flood(border, ~closed)
    s = np.floor((np.asarray(start_xy, float)[:2] - (x0, y0)) / res).astype(int)
    if not (0 <= s[0] < nx and 0 <= s[1] < ny):
        raise ValueError(f"start {start_xy} is outside the grid")
    if closed[s[0], s[1]]:
        raise ValueError(f"start {start_xy} is on a wall cell")
    seed = np.zeros_like(occ)
    seed[s[0], s[1]] = True
    inside = _flood(seed, ~closed)
    if (inside & outside).any():
        raise ValueError("the map's walls do not close a cage around the start (inside meets outside)")

    boundary = occ & _dilate(outside, 2)
    if not boundary.any():  # no gap between cage and grid edge: the outermost wall ring is the cage
        boundary = occ & _dilate(border, 2)
    near_boundary = _dist_to(boundary, res) <= net_margin_m if boundary.any() else np.zeros_like(occ)
    wall = occ & ~boundary & ~near_boundary
    # The cage boundary (nets). Whether it returns depth is unknown on the real course (the sim
    # renders it opaque; real netting may or may not give stereo points, and the room beyond does).
    #   invisible: nets are not in the likelihood, and points near them score the floor (prototype).
    #     In slot 1 (gt05) the matcher then slid scans 0.2-0.4 m to put net points onto interior walls.
    #   surface:   nets are walls. Fixes the sim, but room returns just beyond a see-through net get
    #     pulled onto it (synthetic sweep: 28/400 overconfident fixes).
    #   neutral:   points within net_band_m of the net score a flat net_neutral_lik (safe in synthetic
    #     tests, but the sim error stays 15 cm).
    #   weak:      nets are walls at half strength. The default: sim scans 25/37 accepted at 4.2 cm
    #     median; synthetic sweeps 1/320 marginally overconfident (d2 13), which the fuser gate covers.
    gauss = lambda d: np.exp(-0.5 * (np.minimum(d, 10 * sigma_m) / sigma_m) ** 2)
    d_wall = _dist_to(wall, res)
    if net_mode == "invisible":
        cage = ~outside & ~_dilate(boundary, 1)
        lik = np.where(cage, gauss(d_wall), 0.0)
    else:
        cage = ~outside | _dilate(boundary, int(round(net_band_m / res)))
        d_net = _dist_to(boundary, res)
        if net_mode == "surface":
            net = gauss(d_net)
        elif net_mode == "weak":
            net = 0.5 * gauss(d_net)
        elif net_mode == "neutral":
            net = np.where(d_net <= net_band_m, net_neutral_lik, 0.0)
        else:
            raise ValueError(f"net_mode {net_mode!r}")
        lik = np.where(cage, np.maximum(gauss(d_wall), net), 0.0)
    dist = d_wall
    return {
        "wall": wall, "cage": cage, "boundary": boundary, "lik": lik.astype(np.float32),
        "dist_m": dist.astype(np.float32), "res_m": res, "origin_x": x0, "origin_y": y0,
        "start_xy": np.asarray(start_xy, float)[:2], "start_heading_rad": float(start_heading_rad),
        "band_lo_m": float(band[0]), "band_hi_m": float(band[1]), "floor_z": floor_z,
    }


def map_file_of(course_config):
    """The config's MapFile, resolved relative to the config file (planner/README.md)."""
    import os
    for line in open(course_config):
        line = line.split("#")[0].strip()
        if line.startswith("MapFile:"):
            v = line.split(":", 1)[1].strip()
            return v if os.path.isabs(v) else os.path.normpath(os.path.join(os.path.dirname(course_config), v))
    raise ValueError(f"{course_config}: no MapFile")


def build_from_files(map_file, course_config, **kw):
    """The grid for a course: map from `map_file` (None: the config's MapFile), start and bounds
    from the planner config."""
    if not map_file:
        map_file = map_file_of(course_config)
    cfg = read_course_config(course_config)
    start = cfg["StartPos"][:3]
    heading = math.radians(cfg.get("StartHeadingDeg", [0.0])[0])
    bound = cfg.get("MapBound")
    return build_wallgrid(read_dhmap(map_file), start[:2], heading,
                          bound_xy=None if bound is None else bound[:4], **kw)
