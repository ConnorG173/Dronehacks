#!/usr/bin/env python3
"""Localization vs ground truth around one course point (e.g. where a run touched a wall).

    python3 tools/loc_dev/spot_report.py <bag> --config <course cfg> --spot x,y[,z] [--radius 0.6]

For the airborne samples whose true position is within --radius (xy) of the spot: true clearance
(3D slab, as loc_eval), the error of /team/odom_map and of raw VIO (GT-anchored), and the clearance a
tracker flying on each estimate would have had. Answers "localization error or tracking error?".
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import loc_eval as E  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--config", required=True)
    ap.add_argument("--spot", required=True)
    ap.add_argument("--radius", type=float, default=0.6)
    a = ap.parse_args()
    spot = np.array([float(v) for v in a.spot.split(",")])
    m = E.read_bag(a.bag, [E.T_GT, E.T_VIO, E.T_OUT])
    gt = E.odom_track(m[E.T_GT])
    clr = E.Clearance(a.config)
    ests = {}
    if m[E.T_OUT]:
        ests["odom_map"] = E.odom_track(m[E.T_OUT])
    if m[E.T_VIO]:
        ests["raw_vio_gt0"] = E.anchor_with_gt(E.odom_track(m[E.T_VIO]), gt)
    for name, est in ests.items():
        p, _ = gt.interp(est.t)
        ok = np.isfinite(p[:, 0]) & E.airborne_mask(gt, est.t)
        near = ok & (np.hypot(p[:, 0] - spot[0], p[:, 1] - spot[1]) < a.radius)
        if not near.any():
            print(f"{name}: never within {a.radius} m of the spot")
            continue
        P, Q = p[near], est.p[near]
        err = np.hypot(Q[:, 0] - P[:, 0], Q[:, 1] - P[:, 1])
        c_true = clr(P)
        mir = P.copy()
        mir[:, :2] = P[:, :2] - (Q[:, :2] - P[:, :2])
        c_pred = clr(mir)
        i = int(np.argmin(c_true))
        print(f"{name:12s} n={near.sum():4d}  error p50 {np.median(err):.3f} max {err.max():.3f} m | "
              f"true clearance min {c_true.min():.3f} m at ({P[i,0]:.2f},{P[i,1]:.2f},{P[i,2]:.2f}) | "
              f"if flown on it: min {c_pred.min():.3f} m | error vector at closest "
              f"({Q[i,0]-P[i,0]:+.3f},{Q[i,1]-P[i,1]:+.3f})")


if __name__ == "__main__":
    main()
