#!/usr/bin/env python3
"""Diagnose a fusion run: header stamps vs receive times for IMU / VIO / GT, VIO vs GT position by
header stamp, IMU specific force. python3 diag_bag.py BAG [--t0 -2 --t1 12]"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'path_following', 'tools'))
from eval_bag import read_bag  # noqa: E402


def hs(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


ap = argparse.ArgumentParser()
ap.add_argument('bag')
ap.add_argument('--step', type=float, default=0.5)
a = ap.parse_args()
T = ['/drone1/sensors/imu', '/visual_slam/tracking/odometry', '/sim/ground_truth/odom', '/visual_slam/status',
     '/team/mission/state']
m = read_bag(a.bag, T)
for t in T[:4]:
    x = m[t]
    if not x:
        print(t, 'none'); continue
    rt = np.array([r for r, _ in x]); st = np.array([hs(v) for _, v in x])
    print(f'{t}: n {len(x)}, recv {len(x) / (rt[-1] - rt[0]):.1f} Hz, stamp {len(x) / (st[-1] - st[0]):.1f} Hz, '
          f'stamp span {st[-1] - st[0]:.1f} s vs recv span {rt[-1] - rt[0]:.1f} s, stamp dt max {np.diff(st).max():.3f} '
          f'min {np.diff(st).min():.4f}, recv-stamp median {np.median(rt - st):.3f}')
ph = []
for r, s in m['/team/mission/state']:
    if not ph or ph[-1][1] != s.data:
        ph.append((r, s.data))
print('phases (recv):', [(round(r, 1), d) for r, d in ph])
st_vo = [(hs(v), v.vo_state) for _, v in m['/visual_slam/status']]
bad = [s for s, v in st_vo if v != 1]
print(f'vo_state != 1: {len(bad)} of {len(st_vo)}' + (f', first at stamp {bad[0]:.2f}' if bad else ''))
g = m['/sim/ground_truth/odom']; v = m['/visual_slam/tracking/odometry']; im = m['/drone1/sensors/imu']
tg = np.array([hs(x) for _, x in g]); pg = np.array([[x.pose.pose.position.x, x.pose.pose.position.y, x.pose.pose.position.z] for _, x in g])
tv = np.array([hs(x) for _, x in v]); pv = np.array([[x.pose.pose.position.x, x.pose.pose.position.y, x.pose.pose.position.z] for _, x in v])
vv = np.array([[x.twist.twist.linear.x, x.twist.twist.linear.y, x.twist.twist.linear.z] for _, x in v])
ti = np.array([hs(x) for _, x in im]); ai = np.array([[x.linear_acceleration.x, x.linear_acceleration.y, x.linear_acceleration.z] for _, x in im])
wi = np.array([[x.angular_velocity.x, x.angular_velocity.y, x.angular_velocity.z] for _, x in im])
# takeoff start by GT z
k = np.argmax(pg[:, 2] > pg[0, 2] + 0.05)
t_to = tg[k]
print(f'GT lift-off at stamp {t_to:.2f}')
print(' t-lift   GT z   VIO z  VIO-GT(xyz) [VIO from its first sample]    VIO vel z   IMU az    |w|')
for t in np.arange(t_to - 3, min(tg[-1], tv[-1]), a.step):
    i, j, q = np.searchsorted(tg, t), np.searchsorted(tv, t), np.searchsorted(ti, t)
    if i >= len(tg) or j >= len(tv) or q >= len(ti):
        break
    d = (pv[j] - pv[0]) - (pg[i] - pg[0])
    print(f'{t - t_to:+6.1f} {pg[i, 2]:6.2f} {pv[j, 2]:7.3f}  ({d[0]:+.2f},{d[1]:+.2f},{d[2]:+.2f})   {vv[j, 2]:+7.3f}  '
          f'{ai[max(0, q - 12):q + 12, 2].mean():7.3f} {np.linalg.norm(wi[max(0, q - 12):q + 12].mean(0)):6.3f}')
