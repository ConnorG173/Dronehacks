#!/usr/bin/env python3
"""Read-only logger + guard for the shadow-VIO test on drone 12. Publishes nothing.

  --check          one-shot: PX4 telemetry fresh, DISARMED and LANDED, flight VIO alive -> exit 0, else 1
  --out DIR        log at 10 Hz to DIR/track.jsonl: flight VIO raw (/visual_slam/tracking/odometry), levelled
                   (/claude/vio_leveled/odometry), shadow (/team_imu/visual_slam/tracking/odometry), PX4
                   attitude (gravity reference), vo states, armed/landed; console summary every 2 s
  --guard          with --out: if PX4 reports ARMED or NOT LANDED (or its telemetry goes stale > 3 s),
                   stop the shadow processes (cuVSLAM, IMU prep, TF) at once and exit.
Telemetry comes from the USB adapter's return feed (/claude/usb/out/*), as the dashboard uses it.
"""
import argparse
import json
import math
import os
import subprocess
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=5)
EST = {'raw': '/visual_slam/tracking/odometry', 'lev': '/claude/vio_leveled/odometry',
       'imu': '/team_imu/visual_slam/tracking/odometry'}
STATUS = {'raw': '/visual_slam/status', 'imu': '/team_imu/visual_slam/status'}
FLU_FRD = np.diag([1.0, -1.0, -1.0])


def qm(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def up_body(R_world_body, ned=False):
    """World 'up' expressed in body FLU."""
    if ned:          # PX4: R maps FRD body -> NED; up = -z_ned
        return FLU_FRD @ (R_world_body.T @ np.array([0, 0, -1.0]))
    return R_world_body.T @ np.array([0, 0, 1.0])


def ang(a, b):
    return math.degrees(math.acos(float(np.clip(np.dot(a, b) / np.linalg.norm(a) / np.linalg.norm(b), -1, 1))))


def yaw(R):
    return math.degrees(math.atan2(R[1, 0], R[0, 0]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--out')
    ap.add_argument('--guard', action='store_true')
    a = ap.parse_args()
    from px4_msgs.msg import VehicleAttitude, VehicleLandDetected, VehicleStatus
    rclpy.init()
    n = Node('team_imu_log')
    s = {'armed': None, 'landed': None, 't_status': 0.0, 't_land': 0.0, 'px4': None, 't_px4': 0.0}
    for k in EST:
        s[k], s[k + '_t'], s[k + '_n'], s[k + '_vo'] = None, 0.0, 0, None

    def on_status(m):
        s['armed'] = int(m.arming_state) == 2          # VehicleStatus.ARMING_STATE_ARMED
        s['t_status'] = time.time()

    def on_land(m):
        s['landed'] = bool(m.landed)
        s['t_land'] = time.time()

    def on_att(m):
        s['px4'] = qm(float(m.q[1]), float(m.q[2]), float(m.q[3]), float(m.q[0]))
        s['t_px4'] = time.time()

    n.create_subscription(VehicleStatus, '/claude/usb/out/vehicle_status_v1', on_status, PX4_QOS)
    n.create_subscription(VehicleLandDetected, '/claude/usb/out/vehicle_land_detected', on_land, PX4_QOS)
    n.create_subscription(VehicleAttitude, '/claude/usb/out/vehicle_attitude', on_att, PX4_QOS)

    def odo(k):
        def cb(m):
            q, p = m.pose.pose.orientation, m.pose.pose.position
            s[k] = (qm(q.x, q.y, q.z, q.w), np.array([p.x, p.y, p.z]), (q.x, q.y, q.z, q.w))
            s[k + '_t'] = time.time()
            s[k + '_n'] += 1
        return cb
    for k, t in EST.items():
        n.create_subscription(Odometry, t, odo(k), qos_profile_sensor_data)
    try:
        from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
        for k, t in STATUS.items():
            n.create_subscription(VisualSlamStatus, t, (lambda kk: lambda m: s.__setitem__(kk + '_vo', int(m.vo_state)))(k),
                                  qos_profile_sensor_data)
    except ImportError:
        pass

    def safe():
        now = time.time()
        fresh = now - s['t_status'] < 3.0 and now - s['t_land'] < 3.0
        return fresh and s['armed'] is False and s['landed'] is True, fresh

    if a.check:
        t0 = time.time()
        while time.time() - t0 < 6.0:
            rclpy.spin_once(n, timeout_sec=0.05)
        ok, fresh = safe()
        vio_hz = s['raw_n'] / 6.0
        print(f"PX4 telemetry fresh: {fresh}  armed: {s['armed']}  landed: {s['landed']}  "
              f"flight VIO {vio_hz:.1f} Hz vo_state {s['raw_vo']}  levelled VIO {s['lev_n'] / 6.0:.1f} Hz  "
              f"shadow VIO {s['imu_n'] / 6.0:.1f} Hz")
        print('OK: disarmed and landed' if ok else 'NOT OK: need fresh telemetry, disarmed AND landed')
        raise SystemExit(0 if ok and vio_hz > 20 else 1)

    os.makedirs(a.out, exist_ok=True)
    f = open(os.path.join(a.out, 'track.jsonl'), 'a')
    ref = {}
    t_log = t_print = t_start = time.time()
    rate = {k: 0 for k in EST}
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.01)
        now = time.time()
        if a.guard and now - t_start > 5.0:     # 5 s grace for the first telemetry after start-up
            if s['armed'] is not False or s['landed'] is not True or now - s['t_status'] > 3.0 \
                    or now - s['t_land'] > 3.0:
                msg = (f"GUARD {time.strftime('%T')}: armed={s['armed']} landed={s['landed']} "
                       f"status age {now - s['t_status']:.1f} s -> stopping the shadow")
                print(msg, flush=True)
                f.write(json.dumps({'t': now, 'guard': msg}) + '\n')
                f.close()
                for pat in ('__node:=team_imu_vio', 'team_imu_prep.py', '__node:=team_imu_tf'):
                    subprocess.run(['pkill', '-f', pat], check=False)
                return
        if now - t_log >= 0.1:
            t_log = now
            row = {'t': round(now, 3), 'armed': s['armed'], 'landed': s['landed']}
            if s['px4'] is not None:
                row['px4_R'] = s['px4'].round(5).tolist()
            for k in EST:
                if s[k] is not None and now - s[k + '_t'] < 0.5:
                    row[k] = {'p': s[k][1].round(4).tolist(), 'q': [round(v, 5) for v in s[k][2]], 'vo': s[k + '_vo'],
                              'n': s[k + '_n']}
            f.write(json.dumps(row) + '\n')
        if now - t_print >= 2.0:
            dt = now - t_print
            t_print = now
            line = [time.strftime('%T')]
            for k in EST:
                if s[k] is None or now - s[k + '_t'] > 0.5:
                    line.append(f'{k}: --')
                    continue
                R, p, _ = s[k]
                ref.setdefault(k, (p.copy(), yaw(R)))
                tilt_err = ang(up_body(R), up_body(s['px4'], ned=True)) if s['px4'] is not None else float('nan')
                d = p - ref[k][0]
                dy = (yaw(R) - ref[k][1] + 180) % 360 - 180
                line.append(f"{k}: {(s[k + '_n'] - rate[k]) / dt:4.1f}Hz vo={s[k + '_vo']} tiltErr {tilt_err:5.1f} "
                            f"dyaw {dy:+6.1f} moved ({d[0]:+.2f},{d[1]:+.2f},{d[2]:+.2f})")
                rate[k] = s[k + '_n']
            print(' | '.join(line), flush=True)
            f.flush()


if __name__ == '__main__':
    main()
