#!/usr/bin/env python3
"""Compare VIO odometry and EKF2 velocity against sim ground truth (scale, latency).
    python3 vio_scale.py BAG"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from eval_bag import read_bag  # noqa: E402

m = read_bag(sys.argv[1], ['/sim/ground_truth/odom', '/visual_slam/tracking/odometry', '/fmu/out/vehicle_local_position'])


def series(topic, f):
    return np.array([[t] + f(x) for t, x in m[topic]]) if m[topic] else None


gt = series('/sim/ground_truth/odom', lambda x: [x.pose.pose.position.x, x.pose.pose.position.y, x.pose.pose.position.z])
vio = series('/visual_slam/tracking/odometry', lambda x: [x.pose.pose.position.x, x.pose.pose.position.y, x.pose.pose.position.z])
lp = series('/fmu/out/vehicle_local_position', lambda x: [x.x, x.y, x.z, x.vx, x.vy, x.vz])


def resample(a, t):
    return np.stack([np.interp(t, a[:, 0], a[:, k]) for k in range(1, a.shape[1])], 1)


t = np.arange(gt[0, 0] + 1, gt[-1, 0] - 1, 0.05)
G = resample(gt, t)
moving = np.r_[False, np.linalg.norm(np.diff(G[:, :2], axis=0), axis=1) / 0.05 > 0.2]
print(f'samples {len(t)}, moving {moving.sum()}')
if vio is not None:
    V = resample(vio, t)
    dG = np.diff(G, axis=0)[moving[1:]]
    dV = np.diff(V, axis=0)[moving[1:]]
    sG, sV = np.linalg.norm(dG[:, :2], axis=1).sum(), np.linalg.norm(dV[:, :2], axis=1).sum()
    print(f'VIO horizontal path length {sV:.2f} m vs ground truth {sG:.2f} m: scale {sV / sG:.3f}')
    for lag in (0.0, 0.05, 0.1, 0.15, 0.2):
        Vl = resample(vio, t + lag)
        e = np.linalg.norm((Vl - Vl[0]) - (G - G[0]), axis=1)
        print(f'  VIO lag {lag:.2f}s: median |dpos err| {np.median(e[moving]):.3f} m')
if lp is not None:
    L = resample(lp, t)
    vg = np.gradient(G, 0.05, axis=0)
    vl = np.stack([L[:, 4], L[:, 3], -L[:, 5]], 1)   # NED -> ENU (odom axes)
    sl = np.linalg.norm(vl[moving, :2], axis=1).mean()
    sg = np.linalg.norm(vg[moving, :2], axis=1).mean()
    print(f'EKF2 horizontal speed (moving) {sl:.3f} vs ground truth {sg:.3f}: ratio {sl / sg:.3f}')
