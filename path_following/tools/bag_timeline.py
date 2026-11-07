#!/usr/bin/env python3
"""Print a timeline from a run bag: mission_tracker debug + ground truth, every --step s.
    python3 bag_timeline.py BAG [--step 0.5] [--from-state takeoff] [--secs 25]"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from eval_bag import read_bag  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('bag')
ap.add_argument('--step', type=float, default=0.5)
ap.add_argument('--secs', type=float, default=25.0)
a = ap.parse_args()
m = read_bag(a.bag, ['/team/tracker/debug', '/sim/ground_truth/odom', '/skybeta/state', '/fmu/in/trajectory_setpoint'])
dbg = m['/team/tracker/debug']
labels = dbg[0][1].layout.dim[0].label.split(',')
ix = {n: i for i, n in enumerate(labels)}
D = np.array([list(x.data) for _, x in dbg])
T = np.array([t for t, _ in dbg])
gt = m['/sim/ground_truth/odom']
GT = np.array([[t, x.pose.pose.position.x, x.pose.pose.position.y, x.pose.pose.position.z] for t, x in gt])
fly = np.nonzero(D[:, ix['state']] >= 1)[0]
if not len(fly):
    sys.exit('never left idle')
t0 = T[fly[0]]
print(f'first non-idle at bag t={t0:.2f}; columns: state tau taudot | p (node) | gt | ref | vcmd | v(meas) | src')
nxt = t0
for k in range(fly[0], len(T)):
    if T[k] < nxt:
        continue
    nxt += a.step
    if T[k] - t0 > a.secs:
        break
    r = D[k]
    g = GT[np.argmin(np.abs(GT[:, 0] - T[k]))]
    f = lambda *names: ' '.join(f'{r[ix[n]]:6.2f}' for n in names)  # noqa: E731
    print(f'{T[k] - t0:5.1f} st={int(r[ix["state"]])} {f("tau", "taudot")} | p {f("p_x", "p_y", "p_z")} | '
          f'gt {g[1]:6.2f} {g[2]:6.2f} {g[3]:6.2f} | ref {f("ref_x", "ref_y", "ref_z")} | '
          f'u {f("vcmd_x", "vcmd_y", "vcmd_z")} | v {f("v_x", "v_y", "v_z")} | src {int(r[ix["pose_src"]])}')
sp = m['/fmu/in/trajectory_setpoint']
if sp:
    last = sp[min(len(sp) - 1, len(sp) // 3)][1]
    print('sample /fmu/in/trajectory_setpoint velocity (NED):', list(last.velocity), 'position:', list(last.position))
