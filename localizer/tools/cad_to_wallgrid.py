#!/usr/bin/env python3
"""Course CAD point map -> 2-D wall grid + likelihood field for the course matcher. Laptop tool.

    py localizer/tools/cad_to_wallgrid.py planner/config/course_v3.yaml \
        --out localizer/data/course_v3_wallgrid.npz          (also writes the .png next to it)

Input: the planner config (for MapFile, StartPos, StartHeadingDeg, HoverHeight, Checkpoint, Goal)
and the DHMAP v1 point list it names (planner/README.md). Course frame = planner map frame:
metres, right-handed, z up, floor top at z = 0.

Wall recipe (same as bev.py): points with z in [band_lo, band_hi] are binned into res cells by
floor(xy / res); a cell is wall when its points span at least min_span m of height (vertical faces
only, so floor/roof/tube-top edges seen edge-on do not count).

Grid layout in the .npz: cell [i, j] covers x in origin_x + [i, i+1) * res, y in origin_y + [j, j+1) * res
(i along course +x, j along course +y). Layers:
  wall   bool   wall cells, perimeter net EXCLUDED
  cage   bool   cell centre at least net_margin_m inside the net; observed points outside it are
                to be ignored (the net is see-through and sags; its returns are bad data)
  dist_m f32    distance from the cell centre to the nearest wall cell centre, truncated at trunc_m
  lik    f32    exp(-0.5 (dist_m / sigma_m)^2): the scan-matching likelihood field
plus pad pose (start_xy, start_heading_rad, hover_height_m), checkpoints (K, 6) and goal (3,).
"""
from __future__ import annotations

import argparse
import math
import os

import numpy as np


def read_dhmap(path):
    raw = open(path, "rb").read()
    if raw[:8] != b"DHMAP\0\0\0":
        raise ValueError(f"{path}: not a DHMAP file")
    version, _ = np.frombuffer(raw, "<u4", count=2, offset=8)
    if version != 1:
        raise ValueError(f"{path}: DHMAP version {version}, expected 1")
    n = int(np.frombuffer(raw, "<u8", count=1, offset=16)[0])
    pts = np.frombuffer(raw, "<f4", count=3 * n, offset=24).reshape(n, 3).astype(np.float64)
    return pts[np.isfinite(pts).all(1)]


def read_planner_config(path):
    """Flat 'Key: value' planner config -> {key: [list of float lists or str]} (repeats kept)."""
    out = {}
    for line in open(path, encoding="utf-8"):
        line = line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, val = (s.strip() for s in line.split(":", 1))
        toks = val.replace("[", " ").replace("]", " ").replace(",", " ").split()
        try:
            v = [float(t) for t in toks]
        except ValueError:
            v = val
        out.setdefault(key, []).append(v)
    return out


def build_wallgrid(points, cage_half_m, res=0.05, band=(0.3, 1.6), min_span=0.15, pad_m=0.5,
                   trunc_m=0.5, sigma_m=0.08, net_margin_m=0.15):
    """Point cloud (N, 3) -> dict of grid layers (see module docstring)."""
    from scipy.ndimage import distance_transform_edt   # laptop only; the runtime loads the npz

    # cell boundaries on multiples of res (as bev.py's floor(x / res)), cage plus a margin
    lo = math.floor((-cage_half_m - pad_m) / res)
    hi = math.ceil((cage_half_m + pad_m) / res)
    n = hi - lo
    origin = lo * res

    b = points[(points[:, 2] >= band[0]) & (points[:, 2] <= band[1])]
    ij = np.floor(b[:, :2] / res).astype(np.int64) - lo
    ok = (ij >= 0).all(1) & (ij < n).all(1)
    ij, z = ij[ok], b[ok, 2]
    lin = ij[:, 0] * n + ij[:, 1]
    zmin = np.full(n * n, np.inf)
    zmax = np.full(n * n, -np.inf)
    np.minimum.at(zmin, lin, z)
    np.maximum.at(zmax, lin, z)
    wall = (zmax - zmin >= min_span).reshape(n, n)

    c = origin + (np.arange(n) + 0.5) * res                       # cell centres, same for x and y
    cx, cy = np.meshgrid(c, c, indexing="ij")
    cheb = np.maximum(np.abs(cx), np.abs(cy))                     # the cage is a centred square
    cage = cheb <= cage_half_m - net_margin_m
    wall &= cage                                                  # the net is not a wall

    dist = (distance_transform_edt(~wall) * res).astype(np.float32)
    dist = np.minimum(dist, np.float32(trunc_m))
    lik = np.exp(-0.5 * (dist / sigma_m) ** 2).astype(np.float32)
    return dict(wall=wall, cage=cage, dist_m=dist, lik=lik,
                res_m=np.float64(res), origin_x=np.float64(origin), origin_y=np.float64(origin),
                band_m=np.array(band, float), min_span_m=np.float64(min_span),
                trunc_m=np.float64(trunc_m), sigma_m=np.float64(sigma_m),
                cage_half_m=np.float64(cage_half_m), net_margin_m=np.float64(net_margin_m))


def save_png(g, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    res, ox, oy = float(g["res_m"]), float(g["origin_x"]), float(g["origin_y"])
    n = g["wall"].shape[0]
    ext = (ox, ox + n * res, oy, oy + n * res)
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(g["lik"].T, origin="lower", extent=ext, cmap="Greys", vmin=0, vmax=1)
    h = float(g["cage_half_m"])
    ax.plot([-h, h, h, -h, -h], [-h, -h, h, h, -h], "b-", lw=0.8, label="net (not used)")
    m = h - float(g["net_margin_m"])
    ax.plot([-m, m, m, -m, -m], [-m, -m, m, m, -m], "b--", lw=0.8, label="cage mask")
    sx, sy = g["start_xy"]
    yaw = float(g["start_heading_rad"])
    ax.arrow(sx, sy, 0.4 * math.cos(yaw), 0.4 * math.sin(yaw), width=0.04, color="g", label="pad")
    for k, cp in enumerate(g["checkpoints"]):
        ax.add_patch(plt.Rectangle((cp[0] - cp[3], cp[1] - cp[4]), 2 * cp[3], 2 * cp[4],
                                   fill=False, ec="m"))
        ax.text(cp[0], cp[1] + cp[4] + 0.05, f"C{k + 1}", color="m", ha="center", fontsize=8)
    ax.plot(*g["goal"][:2], "rx")
    ax.set_title(f"wall grid {res * 100:.0f} cm, band {g['band_m'][0]}-{g['band_m'][1]} m, "
                 f"{int(g['wall'].sum())} wall cells")
    ax.set_xlabel("course x (m)")
    ax.set_ylabel("course y (m)")
    ax.set_aspect("equal")
    ax.legend(loc="lower right", fontsize=8)
    fig.savefig(path, dpi=110, bbox_inches="tight")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("config", help="planner config, e.g. planner/config/course_v3.yaml")
    ap.add_argument("--out", required=True, help=".npz output (a .png preview is written next to it)")
    ap.add_argument("--cage-half", type=float, default=3.049, help="cage half-width (m), centred")
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument("--band", type=float, nargs=2, default=(0.3, 1.6))
    ap.add_argument("--min-span", type=float, default=0.15)
    ap.add_argument("--trunc", type=float, default=0.5, help="distance-field truncation (m)")
    ap.add_argument("--sigma", type=float, default=0.08, help="likelihood-field sigma (m)")
    a = ap.parse_args()

    cfg = read_planner_config(a.config)
    map_path = os.path.join(os.path.dirname(os.path.abspath(a.config)), cfg["MapFile"][0])
    pts = read_dhmap(map_path)
    g = build_wallgrid(pts, a.cage_half, a.res, tuple(a.band), a.min_span, trunc_m=a.trunc, sigma_m=a.sigma)
    start = cfg["StartPos"][0]
    g.update(start_xy=np.array(start[:2]), start_heading_rad=np.float64(math.radians(cfg["StartHeadingDeg"][0][0])),
             hover_height_m=np.float64(cfg["HoverHeight"][0][0]),
             checkpoints=np.array(cfg.get("Checkpoint", []), float).reshape(-1, 6),
             goal=np.array(cfg["Goal"][0], float), source=np.array(os.path.basename(map_path)))
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    np.savez_compressed(a.out, **g)
    png = os.path.splitext(a.out)[0] + ".png"
    save_png(g, png)
    n = g["wall"].shape[0]
    print(f"{len(pts)} points -> {n} x {n} grid at {a.res} m, origin ({float(g['origin_x']):.3f}, "
          f"{float(g['origin_y']):.3f}); {int(g['wall'].sum())} wall cells (net excluded)")
    print(f"pad {g['start_xy']} heading {math.degrees(float(g['start_heading_rad'])):.1f} deg; "
          f"{len(g['checkpoints'])} checkpoints\nwrote {a.out}\nwrote {png}")


if __name__ == "__main__":
    main()
