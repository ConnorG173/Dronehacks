#!/usr/bin/env python3
"""Time offsets between sensor header stamps and the truth, from one sim bag:
    python3 time_offsets.py BAG
Body angular rate from GT orientations, from VIO orientations (image stamps) and the IMU gyro, all on
header stamps; lag = shift of each stream that best matches GT's |w| (positive = that stream's stamps are
LATE relative to the motion, i.e. the data happened earlier than stamped). Also the per-axis gyro fit
(IMU axes vs GT body axes) as an extrinsic check."""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'path_following', 'tools'))
from eval_bag import read_bag  # noqa: E402
from vio_drift import qmat  # noqa: E402


def hs(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def logm(R):
    c = np.clip((np.trace(R) - 1) / 2, -1, 1)
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    return th / (2 * math.sin(th)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def rates(msgs, step=1):
    t = np.array([hs(m) for _, m in msgs])
    R = [qmat(m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z,
              m.pose.pose.orientation.w) for _, m in msgs]
    w = np.array([logm(R[i].T @ R[i + step]) / (t[i + step] - t[i]) for i in range(0, len(R) - step)])
    return 0.5 * (t[:-step] + t[step:]), w


m = read_bag(sys.argv[1], ['/sim/ground_truth/odom', '/visual_slam/tracking/odometry', '/drone1/sensors/imu'])
tg, wg = rates(m['/sim/ground_truth/odom'])
tv, wv = rates(m['/visual_slam/tracking/odometry'])
ti = np.array([hs(x) for _, x in m['/drone1/sensors/imu']])
wi = np.array([[x.angular_velocity.x, x.angular_velocity.y, x.angular_velocity.z] for _, x in m['/drone1/sensors/imu']])
# common grid while moving
t = np.arange(max(tg[0], tv[0], ti[0]) + 0.5, min(tg[-1], tv[-1], ti[-1]) - 0.5, 0.002)


G = np.stack([np.interp(t, tg, wg[:, k]) for k in range(3)], 1)
for name, ts, w in (('VIO (image stamps)', tv, wv), ('IMU', ti, wi)):
    best = None
    for lag in np.arange(-0.08, 0.0801, 0.002):
        S = np.stack([np.interp(t + lag, ts, w[:, k]) for k in range(3)], 1)
        mov = np.linalg.norm(G, axis=1) > 0.2
        e = np.sqrt(np.mean(np.sum((S[mov] - G[mov]) ** 2, 1)))
        if best is None or e < best[1]:
            best = (lag, e)
    S = np.stack([np.interp(t + best[0], ts, w[:, k]) for k in range(3)], 1)
    mov = np.linalg.norm(G, axis=1) > 0.2
    A, B = S[mov], G[mov]
    M = np.linalg.lstsq(A, B, rcond=None)[0].T
    print(f'{name:20s} lag {best[0] * 1000:+.0f} ms (rate rms err {best[1]:.3f} rad/s, |w|rms {np.sqrt(np.mean(np.sum(B ** 2, 1))):.2f}); '
          f'axis map GT ~ M @ stream: {np.round(M, 2).tolist()}')
