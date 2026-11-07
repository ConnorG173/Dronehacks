#!/usr/bin/env python3
"""VIO odometry vs ground truth over a run, every 2 s.   python3 vio_check.py BAG"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from eval_bag import read_bag  # noqa: E402

m = read_bag(sys.argv[1], ['/sim/ground_truth/odom', '/visual_slam/tracking/odometry'])
gt, vio = m['/sim/ground_truth/odom'], m['/visual_slam/tracking/odometry']
print(f'messages: gt {len(gt)}, vio {len(vio)}')
if not vio:
    sys.exit()
tv = np.array([t for t, _ in vio])
print(f'vio span {tv[-1] - tv[0]:.1f} s, mean rate {len(tv) / max(tv[-1] - tv[0], 1e-6):.1f} Hz, max gap {np.max(np.diff(tv)):.2f} s')
tg = np.array([t for t, _ in gt])
nxt = tg[0]
for t, x in gt:
    if t < nxt:
        continue
    nxt += 2.0
    k = int(np.argmin(np.abs(tv - t)))
    v = vio[k][1].pose.pose.position
    g = x.pose.pose.position
    print(f'{t - tg[0]:6.1f}s  gt ({g.x:6.2f},{g.y:6.2f},{g.z:5.2f})  vio ({v.x:6.2f},{v.y:6.2f},{v.z:5.2f})  '
          f'vio age {t - tv[k]:+.2f}s')
