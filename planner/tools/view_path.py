#!/usr/bin/env python3
"""Interactive 3D viewer: the course voxels plus a translucent drone every SPACING metres along the path.

    python planner/tools/view_path.py FOLDER [FOLDER ...] [--config course.yaml]
        [--out-dir DIR] [--spacing 0.1] [--arm 0.125] [--rotor 0.064] [--show-floor] [--open]

Same inputs and config lookup as plot_path.py. Writes DIR/<name>.html (default
planner/out/viewer/), one self-contained page per folder; open it in a browser.
The page loads three.js 0.160.0 from cdn.jsdelivr.net, so it needs internet.

Everything is in the map frame (metres, z up). Each drone is drawn at a path
row: position from the path, body z = GCOPTER's thrust direction (with the
config's drag terms, as plot_path.py's tilt), body x = the path's yaw (map
heading) projected perpendicular to body z. Drone size is cosmetic (--arm =
centre-to-motor, --rotor = prop radius). Ghost spacing and playback are
adjustable in the page.
"""
import argparse
import json
import os
import sys
import webbrowser

import numpy as np

import plot_path as P


def body_axes(case):
    """(N, 9): body x, y, z unit vectors in the map frame for every path row."""
    bz = P.thrust_dir(case.cfg, case.v, case.a)
    h = np.column_stack([np.cos(case.heading), np.sin(case.heading), np.zeros(len(case.t))])
    bx = h - (h * bz).sum(axis=1)[:, None] * bz
    bx /= np.linalg.norm(bx, axis=1)[:, None]
    by = np.cross(bz, bx)
    return np.hstack([bx, by, bz])


def page_data(case, args):
    g = case.grid
    hide_floor = g.occ[:, :, 0].mean() >= 0.5 and not args.show_floor  # same rule as plot_path.py
    occ = g.occ.copy()
    if hide_floor:
        occ[:, :, 0] = False
    # Enclosure (net cage, roof): any voxel layer, along any axis, at least half occupied.
    # Drawn faint and see-through so it does not hide the course; interior walls stay solid.
    enclosure = np.zeros_like(occ)
    for ax in range(3):
        share = occ.mean(axis=tuple(k for k in range(3) if k != ax))
        sl = [slice(None)] * 3
        sl[ax] = share >= 0.5
        enclosure[tuple(sl)] = True
    enclosure &= occ
    occ &= ~enclosure
    cage = np.argwhere(P.surface(enclosure))
    ijk = np.argwhere(P.surface(occ))
    n_surface = len(ijk)
    if n_surface > args.max_voxels:
        ijk = ijk[np.random.default_rng(0).choice(n_surface, args.max_voxels, replace=False)]
        print("  %d surface voxels, drawing a random %d (--max-voxels)" % (n_surface, args.max_voxels))
    speed = np.linalg.norm(case.v, axis=1)
    tilt = P.tilt_deg(case.cfg, case.v, case.a)
    length = np.linalg.norm(np.diff(case.p, axis=0), axis=1).sum()
    r4 = lambda a: np.round(np.asarray(a, float), 4).ravel().tolist()
    return {
        "name": case.name,
        "stats": "%.3f s, %.1f m, max |v| %.2f m/s, max tilt %.1f deg\nmap %s, %dx%dx%d voxels @ %.2f m, %d surface voxels%s"
                 % (case.t[-1], length, speed.max(), tilt.max(), os.path.basename(case.map_file), *g.size, g.w,
                    n_surface, (", floor layer hidden" if hide_floor else "")
                    + (", %d enclosure voxels see-through" % len(cage) if len(cage) else "")),
        "bound": r4(case.bound),
        "voxel": g.w,
        "voxels": ijk.ravel().tolist(),
        "cage": cage.ravel().tolist(),
        "floor_z": float(case.bound[4] + g.w) if hide_floor else None,
        "t": r4(case.t),
        "p": r4(case.p),
        "speed": r4(speed),
        "R": r4(body_axes(case)),
        "route": r4(case.route),
        "cps": [r4(np.concatenate([c, h])) for c, h in case.cps],
        "start": r4(case.start),
        "goal": r4(case.goal),
        "spacing": args.spacing,
        "arm": args.arm,
        "rotor": args.rotor,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folders", nargs="+")
    ap.add_argument("--config", help="course.yaml that produced the path(s)")
    ap.add_argument("--out-dir", default=os.path.join(P.PLANNER_DIR, "out", "viewer"))
    ap.add_argument("--spacing", type=float, default=0.1, help="initial ghost spacing along the path (m)")
    ap.add_argument("--arm", type=float, default=0.125, help="drone centre-to-motor length (m), cosmetic")
    ap.add_argument("--rotor", type=float, default=0.064, help="prop radius (m), cosmetic")
    ap.add_argument("--show-floor", action="store_true", help="draw the floor voxel layer instead of a grid")
    ap.add_argument("--max-voxels", type=int, default=200000, help="cap on drawn surface voxels")
    ap.add_argument("--open", action="store_true", help="open the page(s) in the default browser")
    args = ap.parse_args()

    template = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "view_path.html")).read()
    os.makedirs(args.out_dir, exist_ok=True)
    for folder in args.folders:
        case = P.load_case(folder, args.config)
        data = json.dumps(page_data(case, args), separators=(",", ":")).replace("</", "<\\/")
        html = template.replace("__TITLE__", case.name + " path viewer").replace("__DATA__", data)
        out = os.path.join(args.out_dir, case.name + ".html")
        with open(out, "w", encoding="utf-8") as f:
            f.write(html)
        print("%s: wrote %s (%.1f MB)" % (case.name, out, len(html) / 1e6))
        if args.open:
            webbrowser.open("file:///" + os.path.abspath(out).replace(os.sep, "/"))


if __name__ == "__main__":
    sys.exit(main())
