#!/usr/bin/env python3
"""Plot dumped matcher scans (wall_matcher debug_dump_dir) over the course wall grid.

    python3 tools/loc_dev/plot_scan.py --config planner/config/course_v3.yaml --out DIR scan_*.npz \
        [--gt x,y,yaw_deg]   (or --gt-csv t,x,y,yaw_rad rows to look up by stamp)

Each panel: CAD walls (grey), the scan's wall points at the prior (blue), at the matched pose (red),
and at ground truth (green, if given). Laptop tool (matplotlib).
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src", "team_loc_matcher"))
from team_loc_matcher import wallgrid  # noqa: E402


def place(W, pose):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return W @ np.array([[c, s], [-s, c]]) + (x, y)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--gt-csv", default="")
    ap.add_argument("scans", nargs="+")
    a = ap.parse_args()
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    g = wallgrid.build_from_files(None, a.config)
    wi = np.argwhere(g["wall"]) * g["res_m"] + (g["origin_x"], g["origin_y"]) + g["res_m"] / 2
    gt = np.loadtxt(a.gt_csv, delimiter=",") if a.gt_csv else None
    os.makedirs(a.out, exist_ok=True)
    for f in a.scans:
        z = np.load(f)
        W, prior, pose, t = z["W"], z["prior"], z["pose"], float(z["stamp"])
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.scatter(wi[:, 0], wi[:, 1], s=2, c="0.6", label="map walls")
        for p, col, lab in ((prior, "tab:blue", "at prior"), (pose, "tab:red", "at match")):
            q = place(W, p)
            ax.scatter(q[:, 0], q[:, 1], s=3, c=col, label=lab)
        if gt is not None:
            k = int(np.argmin(np.abs(gt[:, 0] - t)))
            pg = gt[k, 1:4]
            q = place(W, pg)
            ax.scatter(q[:, 0], q[:, 1], s=3, c="tab:green", label=f"at GT (dt {gt[k,0]-t:+.3f}s)")
            ax.plot(pg[0], pg[1], "g^", ms=10)
        ax.plot(prior[0], prior[1], "b^", ms=8)
        ax.plot(pose[0], pose[1], "r^", ms=8)
        c = pose[:2]
        ax.set_xlim(c[0] - 3.5, c[0] + 3.5)
        ax.set_ylim(c[1] - 3.5, c[1] + 3.5)
        ax.set_aspect("equal")
        ax.legend(loc="upper right", fontsize=7)
        ax.set_title(f"{os.path.basename(f)}  t={t:.2f}  {len(W)} wall pts")
        fig.savefig(os.path.join(a.out, os.path.basename(f).replace(".npz", ".png")), dpi=90)
        plt.close(fig)
    print(f"wrote {len(a.scans)} plots to {a.out}")


if __name__ == "__main__":
    main()
