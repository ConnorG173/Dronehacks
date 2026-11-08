#!/usr/bin/env python3
"""Read-only IMU calibration check against the CAMERA (publishes nothing). Bench, disarmed, hand-held.

    python3 calib_check.py [--secs 30] [--calib calib.json] [--write]

The shadow cuVSLAM needs three things right: the IMU->camera rotation, the IMU->camera time offset and
the accelerometer scale. The teammates' imu_calib.json measured the rotation against PX4's gyro; this
checks it against what cuVSLAM itself sees: body rates from successive flight-VIO orientations
(/visual_slam/tracking/odometry, header stamps = image stamps) vs /oak/imu/data (header stamps,
same depthai device clock). Steps: hold still >= 3 s (accel scale), then rotate smoothly about all three
axes for --secs. Prints: rotation (Kabsch) and its angle from calib.json, time offset by cross-correlation
(positive = IMU stamps late), residual, accel |g| and scale. --write saves calib.json (old one kept as
calib.json.bak) -- only if the fit is well excited (all singular values > 0.2) and the residual is small.
"""
import argparse
import json
import math
import os
import shutil
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

HERE = os.path.dirname(os.path.abspath(__file__))


def qm(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def logm_vec(R):
    c = np.clip((np.trace(R) - 1) / 2, -1, 1)
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    return th / (2 * math.sin(th)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def mat_to_quat(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return [x, y, z, w]


def kabsch(A, B):
    """R minimising |B - R A| (rows are vectors)."""
    U, S, Vt = np.linalg.svd(A.T @ B)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    return Vt.T @ D @ U.T, S / S.max()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--secs', type=float, default=30.0)
    ap.add_argument('--calib', default=os.path.join(HERE, 'calib.json'))
    ap.add_argument('--write', action='store_true')
    a = ap.parse_args()
    rclpy.init()
    n = Node('team_imu_calib_check')
    imu, vio = [], []
    n.create_subscription(Imu, '/oak/imu/data', lambda m: imu.append(
        (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.angular_velocity.x, m.angular_velocity.y,
         m.angular_velocity.z, m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)),
        qos_profile_sensor_data)

    def on_vio(m):
        q = m.pose.pose.orientation
        vio.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, qm(q.x, q.y, q.z, q.w)))
    n.create_subscription(Odometry, '/visual_slam/tracking/odometry', on_vio, qos_profile_sensor_data)
    print('hold the drone still...', flush=True)
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(n, timeout_sec=0.02)
    if len(imu) < 100:
        raise SystemExit(f'no OAK IMU ({len(imu)} samples)')
    still = np.array(imu)
    print(f'{time.strftime("%T")} now rotate smoothly about roll, pitch and yaw, several times each, for '
          f'{a.secs:.0f} s', flush=True)
    imu.clear(); vio.clear()
    t0 = time.time()
    while time.time() - t0 < a.secs:
        rclpy.spin_once(n, timeout_sec=0.01)
    I = np.array(imu)
    tv = np.array([v[0] for v in vio])
    # body rates from consecutive VIO orientations (base_link FLU), at mid-interval stamps
    wv = np.array([logm_vec(vio[i][1].T @ vio[i + 1][1]) / (tv[i + 1] - tv[i]) for i in range(len(vio) - 1)])
    tm = 0.5 * (tv[1:] + tv[:-1])
    print(f'samples: OAK IMU {len(I)} ({len(I) / (I[-1, 0] - I[0, 0]):.0f} Hz), VIO {len(vio)} '
          f'({len(vio) / (tv[-1] - tv[0]):.1f} Hz)')
    # smooth IMU to the VIO bandwidth, then time offset by |w| cross-correlation
    win = max(1, int(round((tv[1:] - tv[:-1]).mean() * len(I) / (I[-1, 0] - I[0, 0]))))
    k = np.ones(win) / win
    Is = np.stack([np.convolve(I[:, c], k, 'same') for c in (1, 2, 3)], 1)
    best = None
    for lag in np.arange(-0.10, 0.1001, 0.002):
        wi = np.stack([np.interp(tm, I[:, 0] - lag, Is[:, c]) for c in range(3)], 1)
        c = np.corrcoef(np.linalg.norm(wi, axis=1), np.linalg.norm(wv, axis=1))[0, 1]
        if best is None or c > best[1]:
            best = (lag, c)
    lag = best[0]
    wi = np.stack([np.interp(tm, I[:, 0] - lag, Is[:, c]) for c in range(3)], 1)
    mov = np.linalg.norm(wv, axis=1) > 0.3
    R, sv = kabsch(wi[mov], wv[mov])
    res = wv[mov] - wi[mov] @ R.T
    rms, sig = float(np.sqrt((res ** 2).sum(1).mean())), float(np.sqrt((wv[mov] ** 2).sum(1).mean()))
    g = still[:, 4:7].mean(0)
    out = {'R_base_imu': R.round(5).tolist(), 'quat_xyzw': [round(v, 6) for v in mat_to_quat(R)],
           'time_offset_imu_minus_cam_s': round(float(lag), 3), 'corr': round(float(best[1]), 3),
           'excitation_singular_values': [round(float(v), 3) for v in sv], 'n_moving': int(mov.sum()),
           'rate_rms_residual': round(rms, 4), 'rate_rms_signal': round(sig, 4),
           'still_accel_imu': g.round(4).tolist(), 'accel_scale': round(9.80665 / float(np.linalg.norm(g)), 4),
           'reference': 'flight VIO orientation (camera)'}
    print(json.dumps(out, indent=1))
    old = json.load(open(a.calib)) if os.path.exists(a.calib) else None
    if old:
        Ro = np.array(old['R_base_imu'])
        d = math.degrees(math.acos(np.clip((np.trace(R @ Ro.T) - 1) / 2, -1, 1)))
        print(f'rotation differs from {os.path.basename(a.calib)} by {d:.2f} deg; accel scale {old.get("accel_scale")} '
              f'-> {out["accel_scale"]}')
    good = min(sv) > 0.2 and rms < 0.15 * sig and best[1] > 0.9
    print('fit quality:', 'GOOD' if good else 'POOR (more rotation about every axis, or smoother motion)')
    if a.write:
        if not good:
            raise SystemExit('not writing a poor fit')
        if old:
            shutil.copy(a.calib, a.calib + '.bak')
            out['tf_translation'] = old.get('tf_translation', [0, 0, 0])
        json.dump(out, open(a.calib, 'w'), indent=1)
        print('wrote', a.calib)


if __name__ == '__main__':
    main()
