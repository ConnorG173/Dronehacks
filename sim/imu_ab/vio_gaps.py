#!/usr/bin/env python3
"""Frame gaps and vo_state of a replay bag, and where the error vs GT first exceeds 0.2 m.
    python3 vio_gaps.py INPUT_BAG REPLAY_BAG"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'path_following', 'tools'))
from eval_bag import read_bag  # noqa: E402


def hs(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


inp = read_bag(sys.argv[1], ['/sim/ground_truth/odom', '/oak/left/image_mono', '/drone1/sensors/imu'])
out = read_bag(sys.argv[2], ['/visual_slam/tracking/odometry', '/visual_slam/status'])
tv = np.array([hs(m) for _, m in out['/visual_slam/tracking/odometry']])
pv = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for _, m in out['/visual_slam/tracking/odometry']])
tg = np.array([hs(m) for _, m in inp['/sim/ground_truth/odom']])
pg = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for _, m in inp['/sim/ground_truth/odom']])
ti = np.array([hs(m) for _, m in inp['/left' if False else '/oak/left/image_mono']])
G = np.stack([np.interp(tv, tg, pg[:, k]) for k in range(3)], 1)
d = (pv - pv[0]) - (G - G[0])           # crude (no yaw alignment): onset detector only
e = np.linalg.norm(d, axis=1)
k = int(np.argmax(e > 0.2)) if (e > 0.2).any() else None
lift = tg[int(np.argmax(pg[:, 2] > pg[0, 2] + 0.05))]
print(f'input frames {len(ti)}, replay VIO {len(tv)} ({100 * len(tv) / max(len(ti), 1):.1f}% of frames)')
gaps = np.diff(tv)
big = np.where(gaps > 0.05)[0]
print(f'VIO gaps > 50 ms: {len(big)}: ' + ', '.join(f'{tv[i] - lift:+.2f}s ({gaps[i] * 1000:.0f} ms)' for i in big[:12]))
st = [(hs(m), m.vo_state) for _, m in out['/visual_slam/status']]
bad = [(s - lift, v) for s, v in st if v != 1]
print(f'vo_state != 1: {len(bad)}' + (f' first at {bad[0][0]:+.2f}s' if bad else ''))
print('error > 0.2 m first at', f'{tv[k] - lift:+.2f}s after lift-off' if k is not None else 'never')
