#!/usr/bin/env python3
"""How wrong is VIO, in the course frame, over a run?  python3 vio_err.py BAG [--states course]

Two alignments of /visual_slam/tracking/odometry (odom) to /sim/ground_truth/odom (course):
  at-go    rigid SE(3) yaw-only transform fixed by the first VIO/GT pair after the takeoff starts.
           This is what a tracker with a perfect launch pose sees: the error is pure VIO drift.
  best-fit yaw + translation (+ optional scale) least squares over the window. Separates a scale
           error and a heading error from random drift.
Also: VIO latency (lag that minimises the error), path-length scale, and error per phase.
Needs a sourced ROS 2 shell (rosbag2_py)."""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from eval_bag import read_bag  # noqa: E402


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def fit_yaw_t(P, Q, scale=False):
    """min || s R(yaw) P + t - Q ||  (P, Q: N x 3); returns yaw, t, s."""
    pm, qm = P.mean(0), Q.mean(0)
    A, B = P - pm, Q - qm
    h = A[:, :2].T @ B[:, :2]
    yaw = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    R = rotz(yaw)
    s = float(np.sum(B * (A @ R.T)) / np.sum(A * A)) if scale else 1.0
    return yaw, qm - s * R @ pm, s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--states', default='takeoff,course,landing')
    ap.add_argument('--dt', type=float, default=0.05)
    a = ap.parse_args()
    m = read_bag(a.bag, ['/sim/ground_truth/odom', '/visual_slam/tracking/odometry', '/team/mission/state'])
    gt, vio, st = m['/sim/ground_truth/odom'], m['/visual_slam/tracking/odometry'], m['/team/mission/state']
    if not gt or not vio:
        sys.exit(f'need both: gt {len(gt)} vio {len(vio)}')

    def arr(msgs):
        t = np.array([s for s, _ in msgs])
        p = np.array([[x.pose.pose.position.x, x.pose.pose.position.y, x.pose.pose.position.z] for _, x in msgs])
        y = np.unwrap(np.array([yaw_of(x.pose.pose.orientation) for _, x in msgs]))
        return t, p, y
    tg, pg, yg = arr(gt)
    tv, pv, yv = arr(vio)
    phases = []
    for s, x in st:
        if not phases or phases[-1][1] != x.data:
            phases.append((s, x.data))
    keep_states = a.states.split(',')
    spans = [(s, phases[i + 1][0] if i + 1 < len(phases) else tg[-1], n) for i, (s, n) in enumerate(phases)]
    win = [(s0, s1, n) for s0, s1, n in spans if n in keep_states]
    if not win:
        sys.exit(f'no states {keep_states} in the bag: phases {[n for _, n in phases]}')
    t0, t1 = win[0][0], win[-1][1]
    t = np.arange(max(t0, tv[0], tg[0]), min(t1, tv[-1], tg[-1]), a.dt)
    print(f'window {t[-1] - t[0]:.1f} s ({",".join(n for _, _, n in win)}), vio {len(tv)} msgs '
          f'({len(tv) / (tv[-1] - tv[0]):.1f} Hz, max gap {np.max(np.diff(tv)):.2f} s)')

    def at(tt, ts, p):
        return np.stack([np.interp(tt, ts, p[:, k]) for k in range(p.shape[1])], 1)
    G = at(t, tg, pg)

    # VIO latency: lag (VIO stamped late by L means VIO(t + L) ~ GT(t)) minimising best-fit error
    best = None
    for lag in np.arange(-0.10, 0.31, 0.02):
        V = at(t + lag, tv, pv)
        yaw, tr, _ = fit_yaw_t(V, G)
        e = np.linalg.norm((V @ rotz(yaw).T + tr) - G, axis=1)
        r = float(np.sqrt(np.mean(e ** 2)))
        if best is None or r < best[0]:
            best = (r, lag)
    print(f'VIO latency (bag receive time vs GT): {best[1]:+.2f} s (best-fit rms {best[0]:.3f} m)')

    V = at(t, tv, pv)
    # at-go: rigid transform from the first pair (yaw from the orientations)
    k0g, k0v = np.searchsorted(tg, t[0]), np.searchsorted(tv, t[0])
    dyaw = yg[min(k0g, len(yg) - 1)] - yv[min(k0v, len(yv) - 1)]
    R0 = rotz(dyaw)
    tr0 = G[0] - R0 @ V[0]
    E0 = V @ R0.T + tr0 - G
    yaw, tr, s = fit_yaw_t(V, G, scale=True)
    E1 = s * V @ rotz(yaw).T + tr - G
    yaw_n, tr_n, _ = fit_yaw_t(V, G)
    E2 = V @ rotz(yaw_n).T + tr_n - G
    dG = np.linalg.norm(np.diff(G[:, :2], axis=0), axis=1)
    dV = np.linalg.norm(np.diff(V[:, :2], axis=0), axis=1)
    mov = dG / a.dt > 0.2
    print(f'horizontal path length: VIO {dV[mov].sum():.2f} m, GT {dG[mov].sum():.2f} m -> scale '
          f'{dV[mov].sum() / max(dG[mov].sum(), 1e-9):.3f}')
    print(f'best fit: scale {s:.3f}, heading offset vs at-go {math.degrees(yaw - dyaw):+.1f} deg')

    def rep(name, E):
        h, z = np.linalg.norm(E[:, :2], axis=1), np.abs(E[:, 2])
        print(f'{name:22s} horiz rms {np.sqrt(np.mean(h ** 2)):.3f} max {h.max():.3f} end {h[-1]:.3f} m | '
              f'z rms {np.sqrt(np.mean(z ** 2)):.3f} max {z.max():.3f} m')
        return h
    h0 = rep('at-go (tracker view)', E0)
    rep('best fit yaw+t', E2)
    rep('best fit yaw+t+scale', E1)
    for s0, s1, n in win:
        k = (t >= s0) & (t < s1)
        if k.any():
            print(f'  {n:8s} {s1 - s0:5.1f} s  at-go horiz rms {np.sqrt(np.mean(h0[k] ** 2)):.3f} max {h0[k].max():.3f} m')
    print('time   GT (x, y, z)          at-go error (x, y, z)')
    for k in range(0, len(t), max(1, int(2.0 / a.dt))):
        print(f'{t[k] - t[0]:5.1f}  ({G[k, 0]:6.2f},{G[k, 1]:6.2f},{G[k, 2]:5.2f})  '
              f'({E0[k, 0]:+.3f},{E0[k, 1]:+.3f},{E0[k, 2]:+.3f})')


if __name__ == '__main__':
    main()
