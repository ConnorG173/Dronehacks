#!/usr/bin/env python3
"""Plot a gcopter_planner path inside the map it was planned in (laptop tool).

    python planner/tools/plot_path.py FOLDER [FOLDER ...] [--config course.yaml]
        [--out-dir DIR] [--dilated] [--show-floor] [--max-voxels N] [--show]

FOLDER holds path.bin and route.txt (or has them in FOLDER/out, the test
layout). Config: --config if given, else course.yaml in FOLDER or its parent.
Writes DIR/<name>.png (default planner/out/plots/) and prints sanity checks;
exits 1 if any check fails. Formats and frames: planner/README.md.

Map frame: metres, z up (map file, route.txt, config). The path file is PX4
local NED; it is converted back to the map frame with the inverse of the
README transform, using StartPos and StartHeadingDeg from the config.
numpy + matplotlib only.
"""
import argparse
import math
import os
import re
import sys

import numpy as np
import matplotlib

PLANNER_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ---------------------------------------------------------------- inputs

def load_config(path):
    """Flat 'Key: value' lines as src/main.cpp reads them. Returns {key: [value text, ...]}."""
    cfg = {}
    with open(path) as f:
        for n, line in enumerate(f, 1):
            line = re.sub(r"[\[\],\t\r]", " ", line.split("#", 1)[0])
            if not line.strip():
                continue
            if ":" not in line:
                sys.exit("%s:%d: expected 'Key: value'" % (path, n))
            key, val = line.split(":", 1)
            cfg.setdefault(key.strip(), []).append(val.strip())
    return cfg


def cfg_nums(cfg, key, n):
    v = [float(t) for t in cfg[key][0].split()]
    assert len(v) == n, "config %s: expected %d numbers" % (key, n)
    return np.array(v)


def resolve(cfg_path, p):
    """Same rule as main.cpp: absolute if it starts with '/' or contains ':', else relative to the config."""
    if p.startswith("/") or ":" in p:
        return p
    return os.path.join(os.path.dirname(os.path.abspath(cfg_path)), p)


def load_map_points(path):
    raw = open(path, "rb").read()
    assert raw[:8] == b"DHMAP\0\0\0", "%s: not a DHMAP file" % path
    version = int(np.frombuffer(raw, "<u4", count=1, offset=8)[0])
    n = int(np.frombuffer(raw, "<u8", count=1, offset=16)[0])
    assert version == 1, "%s: map version %d" % (path, version)
    pts = np.frombuffer(raw, "<f4", count=3 * n, offset=24).reshape(-1, 3).astype(float)
    return pts[np.isfinite(pts).all(axis=1)]


def load_path(path):
    raw = open(path, "rb").read()
    assert raw[:8] == b"DHPATH\0\0", "%s: not a DHPATH file" % path
    version, cols = np.frombuffer(raw, "<u4", count=2, offset=8)
    rows = int(np.frombuffer(raw, "<u8", count=1, offset=16)[0])
    assert version == 1 and cols == 12, "%s: version %d, %d columns" % (path, version, cols)
    data = np.frombuffer(raw, "<f8", count=rows * 12, offset=32).reshape(rows, 12)
    return data


# ---------------------------------------------------------------- voxel map

class Grid:
    """The planner's VoxelMap: origin (xmin, ymin, zmin), size int(extent / w), index trunc((p - o) / w)."""

    def __init__(self, bound, w, pts, dilate_radius):
        self.o = bound[[0, 2, 4]]
        self.w = w
        self.size = np.array([int((bound[1] - bound[0]) / w), int((bound[3] - bound[2]) / w),
                              int((bound[5] - bound[4]) / w)])
        self.occ = np.zeros(self.size, bool)
        idx = np.trunc((pts - self.o) / w).astype(int)
        inside = ((idx >= 0) & (idx < self.size)).all(axis=1)
        self.occ[tuple(idx[inside].T)] = True
        # VoxelMap::dilate(r): r passes of 26-neighbour growth = Chebyshev box of radius r.
        self.r = int(math.ceil(dilate_radius / w))
        self.dil = self.occ.copy()
        for ax in range(3):
            src = self.dil.copy()
            for s in range(1, self.r + 1):
                lo = [slice(None)] * 3
                hi = [slice(None)] * 3
                lo[ax], hi[ax] = slice(0, -s), slice(s, None)
                self.dil[tuple(lo)] |= src[tuple(hi)]
                self.dil[tuple(hi)] |= src[tuple(lo)]

    def centers(self, mask):
        return np.argwhere(mask) * self.w + self.o + 0.5 * self.w

    def query(self, p):
        """0 free, 1 dilated margin, 2 occupied, 3 outside MapBound (counts as occupied)."""
        idx = np.trunc((p - self.o) / self.w).astype(int)
        inside = ((idx >= 0) & (idx < self.size)).all(axis=1)
        out = np.full(len(p), 3)
        i = tuple(idx[inside].T)
        out[inside] = np.where(self.occ[i], 2, np.where(self.dil[i], 1, 0))
        return out


def surface(mask):
    """Occupied voxels with at least one free (or out-of-grid) 6-neighbour."""
    pad = np.pad(mask, 1, constant_values=False)
    interior = np.ones_like(mask)
    for ax in range(3):
        for s in (-1, 1):
            interior &= np.roll(pad, s, axis=ax)[1:-1, 1:-1, 1:-1]
    return mask & ~interior


# ---------------------------------------------------------------- frames

def ned_matrix(psi0):
    """README: x_ned = c dx + s dy, y_ned = s dx - c dy, z_ned = -dz. R is symmetric and orthogonal: R^-1 = R."""
    c, s = math.cos(psi0), math.sin(psi0)
    return np.array([[c, s, 0.0], [s, -c, 0.0], [0.0, 0.0, -1.0]])


def path_to_map(rows, start, psi0):
    R = ned_matrix(psi0)  # its own inverse
    t = rows[:, 0]
    p = start + rows[:, 1:4] @ R.T
    v = rows[:, 4:7] @ R.T
    a = rows[:, 7:10] @ R.T
    heading = np.unwrap(psi0 - rows[:, 10])  # map heading, CCW from map +x
    return t, p, v, a, heading


# ---------------------------------------------------------------- checks

class Checks:
    def __init__(self):
        self.failed = 0

    def __call__(self, ok, text):
        print("  %s  %s" % ("PASS" if ok else "FAIL", text))
        self.failed += not ok


def first_entry(t, p, center, half, t_from):
    inside = (np.abs(p - center) <= half).all(axis=1) & (t >= t_from)
    return t[np.argmax(inside)] if inside.any() else None


def run_checks(name, t, p, route, grid, start_hover, goal, cps, summary_times):
    print("%s:" % name)
    check = Checks()
    fmt = lambda x: "(%.3f, %.3f, %.3f)" % tuple(x)
    tol = 1e-3
    check(np.linalg.norm(p[0] - start_hover) < tol, "path row 0 %s = StartPos + (0,0,HoverHeight) %s" % (fmt(p[0]), fmt(start_hover)))
    check(np.linalg.norm(p[-1] - goal) < tol, "path last row %s = Goal %s" % (fmt(p[-1]), fmt(goal)))
    check(np.linalg.norm(route[0] - start_hover) < tol, "route start %s = hover point" % fmt(route[0]))
    check(np.linalg.norm(route[-1] - goal) < tol, "route end %s = Goal" % fmt(route[-1]))

    # Dense 1 ms samples (linear between rows) against the map.
    td = np.arange(0.0, t[-1], 0.001)
    pd = np.column_stack([np.interp(td, t, p[:, k]) for k in range(3)])
    q = grid.query(pd)
    check((q == 3).sum() == 0, "never outside MapBound (%d of %d 1-ms samples)" % ((q == 3).sum(), len(q)))
    check((q == 2).sum() == 0, "never in an occupied voxel (%d samples)" % (q == 2).sum())
    print("  info  %d samples inside the DilateRadius margin (allowed)" % (q == 1).sum())
    rq = grid.query(route)
    check((rq >= 2).sum() == 0, "route vertices free (%d in occupied voxels)" % (rq >= 2).sum())

    t_from, times = 0.0, []
    for i, (c, h) in enumerate(cps):
        te = first_entry(td, pd, c, h, t_from)
        times.append(te)
        if te is None:
            check(False, "checkpoint %d box %s never entered (after t=%.3f)" % (i + 1, fmt(c), t_from))
            continue
        msg = "checkpoint %d entered at t = %.3f s" % (i + 1, te)
        ok = True
        if i < len(summary_times):
            ok = abs(te - summary_times[i]) < 0.02
            msg += " (summary.txt %.3f)" % summary_times[i]
        check(ok, msg)
        t_from = te
    return check.failed, times


# ---------------------------------------------------------------- plotting

def box_edges(lo, hi):
    """12 edges of an axis-aligned box as (2, 3) segments."""
    c = np.array([[lo[0] if i & 1 == 0 else hi[0], lo[1] if i & 2 == 0 else hi[1], lo[2] if i & 4 == 0 else hi[2]]
                  for i in range(8)])
    return [c[[a, b]] for a in range(8) for b in range(a + 1, 8) if bin(a ^ b).count("1") == 1]


def colored_line(ax, xy, speed, norm, cmap, lw):
    from matplotlib.collections import LineCollection
    from mpl_toolkits.mplot3d.art3d import Line3DCollection
    seg = np.stack([xy[:-1], xy[1:]], axis=1)
    cls = Line3DCollection if xy.shape[1] == 3 else LineCollection
    lc = cls(seg, cmap=cmap, norm=norm, linewidths=lw)
    lc.set_array(0.5 * (speed[:-1] + speed[1:]))
    if xy.shape[1] == 3:
        ax.add_collection3d(lc)
    else:
        ax.add_collection(lc)
    return lc


def tilt_deg(cfg, v, a):
    """GCOPTER FlatnessMap::forward thrust direction zu = a + dh/m (1 + cp sqrt(|v|^2 + eps)) v + g z; tilt = angle to z."""
    m, g = float(cfg["VehicleMass"][0]), float(cfg["GravAcc"][0])
    dh, cp, eps = float(cfg["HorizDrag"][0]), float(cfg["ParasDrag"][0]), float(cfg["SpeedEps"][0])
    w = 1.0 + cp * np.sqrt((v * v).sum(axis=1) + eps)
    zu = a + (dh / m) * w[:, None] * v + np.array([0.0, 0.0, g])
    return np.degrees(np.arccos(np.clip(zu[:, 2] / np.linalg.norm(zu, axis=1), -1.0, 1.0)))


def plot(name, cfg, grid, t, p, v, heading, tilt, route, cps, cp_times, start, start_hover, goal, bound, args):
    """Returns (3D figure, figure with projections and time plots)."""
    import matplotlib.pyplot as plt
    from matplotlib.colors import Normalize

    speed = np.linalg.norm(v, axis=1)
    cmap = plt.get_cmap("viridis")
    norm = Normalize(0.0, max(speed.max(), 1e-6))
    # The bottom voxel layer counts as a floor if at least half of it is occupied. Unless
    # --show-floor, it is hidden, and so is its dilation (the r layers above it).
    has_floor = grid.occ[:, :, 0].mean() >= 0.5
    hide = has_floor and not args.show_floor
    occ_draw = grid.occ.copy()
    margin = grid.dil & ~grid.occ
    if hide:
        occ_draw[:, :, 0] = False
        margin[:, :, :1 + grid.r] = False
    floor_note = ", floor layer hidden" if hide else ""

    # ---- 3D
    fig3 = plt.figure(figsize=(14, 9))
    ax3 = fig3.add_subplot(projection="3d")
    rng = np.random.default_rng(0)

    def scatter_voxels(mask, color, alpha, label):
        c = grid.centers(surface(mask))
        n = len(c)
        if n > args.max_voxels:
            c = c[rng.choice(n, args.max_voxels, replace=False)]
        ax3.scatter(*c.T, s=10, marker="s", c=color, alpha=alpha, linewidths=0, depthshade=False,
                    label="%s (%d surface voxels%s)" % (label, n, ", %d drawn" % len(c) if n > len(c) else ""))

    scatter_voxels(occ_draw, "0.3", 0.25, "occupied" + floor_note)
    if args.dilated:
        scatter_voxels(margin, "tab:orange", 0.05, "dilated margin")
    for e in box_edges(bound[[0, 2, 4]], bound[[1, 3, 5]]):
        ax3.plot(*e.T, color="0.6", lw=0.6)
    ax3.plot(*route.T, color="k", lw=0.8, marker=".", ms=4, label="RRT route (route.txt)")
    lc = colored_line(ax3, p, speed, norm, cmap, 3.0)
    for i, (c, h) in enumerate(cps):
        for e in box_edges(c - h, c + h):
            ax3.plot(*e.T, color="tab:red", lw=1.2)
        ax3.text(*(c + h), " C%d" % (i + 1), color="tab:red")
    ax3.scatter(*start, c="tab:green", marker="^", s=60, label="StartPos (pad)")
    ax3.scatter(*start_hover, c="tab:green", marker="o", s=40, label="hover start")
    ax3.scatter(*goal, c="tab:red", marker="*", s=150, label="Goal")
    ext = bound[[1, 3, 5]] - bound[[0, 2, 4]]
    zscale = max(ext[2], 0.25 * max(ext[0], ext[1]))
    ax3.set_box_aspect((ext[0], ext[1], zscale), zoom=1.2)
    ax3.set(xlim=bound[0:2], ylim=bound[2:4], zlim=bound[4:6], xlabel="x (m)", ylabel="y (m)", zlabel="z (m)")
    ax3.view_init(elev=30, azim=-60)
    ax3.legend(loc="upper left", fontsize=9)
    fig3.colorbar(lc, ax=ax3, shrink=0.5, pad=0.12, label="|v| (m/s)")
    fig3.suptitle("%s: 3D, map frame%s" % (name, " (z exaggerated x%.1f)" % (zscale / ext[2]) if zscale > ext[2] else ""),
                  fontsize=14)
    fig3.subplots_adjust(left=0.0, right=0.95, bottom=0.02, top=0.95)

    # ---- projections + time plots
    fig = plt.figure(figsize=(18, 10.5), layout="constrained")
    gs = fig.add_gridspec(2, 3, height_ratios=[2.2, 1.0])

    def projection(ax, k0, k1, k_proj, title):
        # Share of occupied cells along the projection axis (floor layer excluded if hidden):
        # an opening in a wall shows as a lighter patch.
        frac = occ_draw.mean(axis=k_proj)  # remaining axes in order (k0, k1)
        ax.imshow(frac.T, origin="lower", cmap="Greys", vmin=0, vmax=1.4 * max(frac.max(), 1e-9),
                  interpolation="nearest", aspect="equal",
                  extent=(bound[2 * k0], bound[2 * k0 + 1], bound[2 * k1], bound[2 * k1 + 1]))
        if args.dilated:
            d = margin.any(axis=k_proj) & ~occ_draw.any(axis=k_proj)
            cen = lambda k, n: bound[2 * k] + grid.w * (np.arange(n) + 0.5)
            ax.contour(cen(k0, d.shape[0]), cen(k1, d.shape[1]), d.T.astype(float), levels=[0.5],
                       colors="tab:orange", linewidths=0.8)
        ax.plot(route[:, k0], route[:, k1], color="k", lw=0.8, marker=".", ms=4)
        colored_line(ax, p[:, [k0, k1]], speed, norm, cmap, 2.5)
        for i, (c, h) in enumerate(cps):
            ax.add_patch(matplotlib.patches.Rectangle((c[k0] - h[k0], c[k1] - h[k1]), 2 * h[k0], 2 * h[k1],
                                                      fill=False, ec="tab:red", lw=1.2))
            ax.annotate("C%d" % (i + 1), (c[k0] + h[k0], c[k1] + h[k1]), color="tab:red")
        ax.plot(start_hover[k0], start_hover[k1], "o", color="tab:green")
        ax.plot(goal[k0], goal[k1], "*", color="tab:red", ms=12)
        ax.set(xlim=bound[2 * k0:2 * k0 + 2], ylim=bound[2 * k1:2 * k1 + 2], title=title,
               xlabel="xyz"[k0] + " (m)", ylabel="xyz"[k1] + " (m)")

    projection(fig.add_subplot(gs[0, 0]), 0, 1, 2, "top (x-y); gray = share of z occupied")
    projection(fig.add_subplot(gs[0, 1]), 0, 2, 1, "side (x-z); gray = share of y occupied")
    projection(fig.add_subplot(gs[0, 2]), 1, 2, 0, "front (y-z); gray = share of x occupied")

    for col, (y, lab) in enumerate([(speed, "|v| (m/s)"), (tilt, "tilt (deg)"),
                                    (np.degrees(heading), "map heading (deg, CCW from +x)")]):
        ax = fig.add_subplot(gs[1, col])
        ax.plot(t, y, lw=1.2)
        for i, te in enumerate(cp_times):
            if te is not None:
                ax.axvline(te, color="tab:red", lw=0.8, ls="--")
                ax.annotate(" C%d" % (i + 1), (te, 1.0), xycoords=("data", "axes fraction"), color="tab:red", va="top")
        ax.set(xlabel="t (s)", ylabel=lab, xlim=(0, t[-1]))
        ax.grid(alpha=0.3)
    fig.colorbar(matplotlib.cm.ScalarMappable(norm, cmap), ax=fig.axes[:3], shrink=0.8, label="|v| (m/s)")
    fig.suptitle("%s: %.3f s, max |v| %.2f m/s, max tilt %.1f deg, map %s (%dx%dx%d @ %.2f m, dilate %d)"
                 % (name, t[-1], speed.max(), tilt.max(), os.path.basename(args._map_file), *grid.size, grid.w,
                    grid.r), fontsize=13)
    return fig3, fig


# ---------------------------------------------------------------- main

def find_inputs(folder, config):
    folder = os.path.abspath(folder)
    data = folder if os.path.exists(os.path.join(folder, "path.bin")) else os.path.join(folder, "out")
    if not os.path.exists(os.path.join(data, "path.bin")):
        sys.exit("%s: no path.bin (or out/path.bin)" % folder)
    if config is None:
        for c in (os.path.join(data, "course.yaml"), os.path.join(os.path.dirname(data), "course.yaml")):
            if os.path.exists(c):
                config = c
                break
        else:
            sys.exit("%s: no course.yaml in the folder or its parent; pass --config" % data)
    return data, config


def plot_name(data):
    """out/jetson/course -> jetson_course; out/tests/x/out -> tests_x."""
    d = data[:-4] if os.path.basename(data) == "out" else data
    out_root = os.path.join(PLANNER_DIR, "out")
    rel = os.path.relpath(d, out_root) if os.path.abspath(d).startswith(out_root) else os.path.basename(d)
    return rel.replace(os.sep, "_").replace("/", "_")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folders", nargs="+")
    ap.add_argument("--config", help="course.yaml that produced the path(s)")
    ap.add_argument("--out-dir", default=os.path.join(PLANNER_DIR, "out", "plots"))
    ap.add_argument("--dilated", action="store_true", help="also draw the DilateRadius margin")
    ap.add_argument("--show-floor", action="store_true", help="draw the bottom voxel layer when it looks like a floor (>= 50%% occupied; hidden by default)")
    ap.add_argument("--max-voxels", type=int, default=40000, help="cap on 3D scatter points (random subsample)")
    ap.add_argument("--show", action="store_true", help="open interactive windows after saving")
    args = ap.parse_args()
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(args.out_dir, exist_ok=True)
    failed = 0
    for folder in args.folders:
        data, cfg_path = find_inputs(folder, args.config)
        cfg = load_config(cfg_path)
        bound = cfg_nums(cfg, "MapBound", 6)
        start = cfg_nums(cfg, "StartPos", 3)
        psi0 = math.radians(float(cfg["StartHeadingDeg"][0]))
        hover = float(cfg["HoverHeight"][0])
        goal = cfg_nums(cfg, "Goal", 3)
        cps = []
        for text in cfg.get("Checkpoint", []):
            v = np.array([float(x) for x in text.split()])
            cps.append((v[:3], v[3:]))
        args._map_file = resolve(cfg_path, cfg["MapFile"][0])
        pts = load_map_points(args._map_file)
        args._npts = len(pts)
        grid = Grid(bound, float(cfg["VoxelWidth"][0]), pts, float(cfg["DilateRadius"][0]))

        t, p, v, a, heading = path_to_map(load_path(os.path.join(data, "path.bin")), start, psi0)
        route = np.loadtxt(os.path.join(data, "route.txt"), ndmin=2)
        summary = os.path.join(data, "summary.txt")
        summary_text = open(summary).read() if os.path.exists(summary) else ""
        summary_times = [float(m) for m in re.findall(r"reached at t = ([-\d.]+) s", summary_text)]

        name = plot_name(data)
        print("config %s, map %s" % (cfg_path, args._map_file))
        start_hover = start + np.array([0.0, 0.0, hover])
        nfail, cp_times = run_checks(name, t, p, route, grid, start_hover, goal, cps, summary_times)
        failed += nfail
        tilt = tilt_deg(cfg, v, a)
        m = re.search(r"max tilt\s*:\s*([\d.]+) deg", summary_text)
        if m:  # summary.txt uses 1-ms samples, the path file 10-ms rows
            ok = abs(tilt.max() - float(m.group(1))) < 0.5
            print("  %s  max tilt %.1f deg (summary.txt %s)" % ("PASS" if ok else "FAIL", tilt.max(), m.group(1)))
            failed += not ok
        figs = plot(name, cfg, grid, t, p, v, heading, tilt, route, cps, cp_times, start, start_hover, goal,
                    bound, args)
        for fig, suffix in zip(figs, ("_3d", "")):
            png = os.path.join(args.out_dir, name + suffix + ".png")
            fig.savefig(png, dpi=110)
            print("  wrote %s" % png)
            if not args.show:
                plt.close(fig)
    if args.show:
        plt.show()
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
