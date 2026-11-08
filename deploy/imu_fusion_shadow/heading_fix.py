#!/usr/bin/env python3
"""Gyro heading correction for drone 12's VIO (SHADOW: publishes only /team_heading/odometry).

Problem: PX4 flies on /claude/vio_leveled/odometry. The leveller fixes roll/pitch with PX4's attitude, but
heading comes from cuVSLAM alone (no compass), and VIO heading drifts during the turns. A 600-700 deg
flight ends ~50 cm off. PX4's own yaw follows that VIO heading (EV yaw fusion), so it cannot fix it.

Fix: take heading INCREMENTS from the gyro and everything else from the levelled VIO.
  delta    heading correction (rad). Each step: delta += (gyro heading change - VIO heading change),
           then a very slow pull back toward VIO: delta -= delta * dt / tau (tau 1 h: VIO heading error is
           permanent in its frame; the pull only bounds residual gyro-bias drift over a whole boot).
  gyro     body rates, bias = time-weighted mean of the gyro over all still time (gyro AND VIO quiet for
           >= 2 s; >= 0.5 s of still data before it is used; 2 min forgetting for warm-up drift); heading
           change = yaw change of R_vio_prev @ exp(rate * dt), so roll/pitch come from the levelled VIO.
  output   R = Rz(delta) R_vio;  p += Rz(delta) (p_vio - p_vio_prev)  (re-integrated, not rotated about the
           origin). A VIO jump (> jump_m in one step, or > jump_deg of heading) is absorbed: position does not
           jump with it, and the gyro holds the heading through it.
Body-frame twist is unchanged.

  python3 heading_fix.py offline <log dir or track.jsonl> [--src g_px4|g_px4usb|g_oak] [--tau 3600]
      replays a shadow log (team_imu_log.py rows: 'lev' + gyro intervals), writes 'hfix' rows to
      track_hfix.jsonl next to it; analyze.py --hfix scores it like the other estimators.
  python3 heading_fix.py node [--gyro-topic /fmu/out/sensor_combined] [--tau 3600]
      live shadow node: /claude/vio_leveled/odometry + PX4 gyro -> /team_heading/odometry (frame
      team_heading_odom). Nothing reads it.
"""
import argparse
import json
import math
import os

import numpy as np


def rodrigues(v):
    th = float(np.linalg.norm(v))
    if th < 1e-12:
        return np.eye(3)
    k = np.asarray(v) / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def yaw(R):
    return math.atan2(R[1, 0], R[0, 0])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def qm(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


class HeadingFix:
    def __init__(self, tau_s=3600.0, still_rate=0.03, still_move=0.03, still_s=2.0, bias_tc_s=120.0,
                 jump_m=0.5, jump_deg=20.0):
        self.tau, self.still_rate, self.still_move, self.still_s = tau_s, still_rate, still_move, still_s
        self.bias_tc, self.jump_m, self.jump = bias_tc_s, jump_m, math.radians(jump_deg)
        self.delta, self.bias, self.bias_n = 0.0, np.zeros(3), 0
        self.bias_r, self.bias_T, self.bias_min_s = np.zeros(3), 0.0, 0.5   # time-weighted still-gyro sums
        self.p_out = self.p_prev = self.R_prev = None
        self.still_for, self.jumps, self.still_anchor = 0.0, 0, None

    def step(self, R_vio, p_vio, rotvec, dt):
        """R_vio, p_vio: levelled VIO now. rotvec: body rotation (FLU, rad) since the previous step, dt its
        duration (gyro stamps). Returns (R_out, p_out)."""
        R_vio, p_vio = np.asarray(R_vio, float), np.asarray(p_vio, float)
        if self.p_out is None:
            self.p_out, self.p_prev, self.R_prev = p_vio.copy(), p_vio.copy(), R_vio.copy()
            return R_vio, p_vio.copy()
        r = np.asarray(rotvec, float) if rotvec is not None else None
        dp = p_vio - self.p_prev
        dpsi_v = wrap(yaw(R_vio) - yaw(self.R_prev))
        jumped = float(np.linalg.norm(dp)) > self.jump_m
        if r is not None and dt > 0:
            rate = np.linalg.norm(r) / dt
            # bias: learn while gyro and VIO are both quiet for still_s
            if rate < self.still_rate:
                if self.still_anchor is None or float(np.linalg.norm(p_vio - self.still_anchor)) > self.still_move:
                    self.still_anchor, self.still_for = p_vio.copy(), 0.0    # (re)start the still window
                else:
                    self.still_for += dt
            else:
                self.still_anchor, self.still_for = None, 0.0
            if self.still_for >= self.still_s:
                # time-weighted mean of the still gyro over ALL still time, forgetting with bias_tc of still time
                # (tracks warm-up drift). Not a per-sample blend: one 16 ms sample is far too noisy to seed it.
                f = math.exp(-dt / self.bias_tc)
                self.bias_r = self.bias_r * f + r
                self.bias_T = self.bias_T * f + dt
                if self.bias_T >= self.bias_min_s:
                    self.bias = self.bias_r / self.bias_T
                self.bias_n += 1
            rc = r - self.bias * dt
            dpsi_g = wrap(yaw(self.R_prev @ rodrigues(rc)) - yaw(self.R_prev))
            if abs(wrap(dpsi_v - dpsi_g)) > self.jump:
                jumped = True
            self.delta += dpsi_g - dpsi_v          # heading increment from the gyro, not from VIO
            self.delta -= self.delta * min(1.0, dt / self.tau)
        if jumped:
            self.jumps += 1                       # VIO reset/jump: keep position, gyro held heading above
        else:
            self.p_out = self.p_out + rotz(self.delta) @ dp
        self.p_prev = p_vio
        if r is not None and dt > 0:
            # Advance the heading reference ONLY when gyro data covered this step. With VIO at 60 Hz and the
            # gyro at ~110 Hz some VIO steps carry no gyro sample; the next gyro sample then spans both steps,
            # so the VIO heading change must be measured over the same span (else every skipped step's real
            # turn was added to the correction: live over-correction, 2026-11-08 walk 3).
            self.R_prev = R_vio
        return rotz(self.delta) @ R_vio, self.p_out.copy()


def offline(a):
    path = a.log if a.log.endswith('.jsonl') else os.path.join(a.log, 'track.jsonl')
    rows = [json.loads(x) for x in open(path) if x.strip()]
    src = a.src
    if src == 'auto':
        cnt = {k: sum(1 for r in rows if k in r) for k in ('g_px4', 'g_px4usb', 'g_oak')}
        src = max(cnt, key=cnt.get)
        print('gyro rows per source:', cnt, '-> using', src)
    hf = HeadingFix(tau_s=a.tau)
    acc_r, acc_dt = np.zeros(3), 0.0
    out = []
    for r in rows:
        if src in r:                              # accumulate gyro until the next VIO row
            acc_r = acc_r + np.array(r[src]['r'])  # (sub-0.1 s intervals: rotation vectors add to ~1e-3)
            acc_dt += r[src]['dt']
        if 'lev' in r:
            Rv = qm(*r['lev']['q'])
            R, p = hf.step(Rv, r['lev']['p'], acc_r if acc_dt > 0 else None, acc_dt)
            acc_r, acc_dt = np.zeros(3), 0.0
            r = dict(r)
            r['hfix'] = {'p': [round(float(x), 4) for x in p], 'q': [round(float(x), 5) for x in mat_to_quat(R)],
                         'vo': r['lev'].get('vo'), 'n': r['lev'].get('n')}
        out.append(r)
    dst = os.path.join(os.path.dirname(path), 'track_hfix.jsonl')
    with open(dst, 'w') as f:
        for r in out:
            f.write(json.dumps(r) + '\n')
    print(f'wrote {dst}: final heading correction {math.degrees(hf.delta):+.2f} deg, gyro bias '
          f'{np.round(np.degrees(hf.bias), 4).tolist()} deg/s ({hf.bias_n} still samples), VIO jumps absorbed {hf.jumps}')


def node(a):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from px4_msgs.msg import SensorCombined
    rclpy.init()
    n = Node('team_heading_fix')
    pub = n.create_publisher(Odometry, '/team_heading/odometry', 10)
    hf = HeadingFix(tau_s=a.tau)
    g = {'R': np.eye(3), 'dt': 0.0, 't': None}

    def on_gyro(m):
        t = int(m.timestamp) * 1e-6
        w = np.array([float(m.gyro_rad[0]), -float(m.gyro_rad[1]), -float(m.gyro_rad[2])])   # FRD -> FLU
        if g['t'] is not None and 0.0 < t - g['t'] < 0.1:
            g['R'] = g['R'] @ rodrigues(w * (t - g['t']))
            g['dt'] += t - g['t']
        g['t'] = t

    def on_vio(m):
        q, p = m.pose.pose.orientation, m.pose.pose.position
        rv = None
        if g['dt'] > 0:
            c = float(np.clip((np.trace(g['R']) - 1) / 2, -1, 1))
            th = math.acos(c)
            rv = np.zeros(3) if th < 1e-9 else th / (2 * math.sin(th)) * np.array(
                [g['R'][2, 1] - g['R'][1, 2], g['R'][0, 2] - g['R'][2, 0], g['R'][1, 0] - g['R'][0, 1]])
        R, po = hf.step(qm(q.x, q.y, q.z, q.w), [p.x, p.y, p.z], rv, g['dt'])
        g['R'], g['dt'] = np.eye(3), 0.0
        o = Odometry()
        o.header = m.header
        o.header.frame_id = 'team_heading_odom'
        o.child_frame_id = m.child_frame_id
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = (float(x) for x in po)
        qq = mat_to_quat(R)
        o.pose.pose.orientation.x, o.pose.pose.orientation.y, o.pose.pose.orientation.z, o.pose.pose.orientation.w = qq
        o.pose.covariance = m.pose.covariance
        o.twist = m.twist
        pub.publish(o)

    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=100)
    n.create_subscription(SensorCombined, a.gyro_topic, on_gyro, qos)
    n.create_subscription(Odometry, '/claude/vio_leveled/odometry', on_vio, 10)
    n.create_timer(5.0, lambda: n.get_logger().info(
        f'heading correction {math.degrees(hf.delta):+.2f} deg, bias {np.round(np.degrees(hf.bias), 3).tolist()} '
        f'deg/s from {hf.bias_T:.0f} s still, jumps absorbed {hf.jumps}'))
    rclpy.spin(n)


def main():
    ap = argparse.ArgumentParser()
    sp = ap.add_subparsers(dest='cmd', required=True)
    o = sp.add_parser('offline')
    o.add_argument('log')
    o.add_argument('--src', default='auto', choices=['auto', 'g_px4', 'g_px4usb', 'g_oak'])
    o.add_argument('--tau', type=float, default=3600.0)
    nd = sp.add_parser('node')
    nd.add_argument('--gyro-topic', default='/fmu/out/sensor_combined')
    nd.add_argument('--tau', type=float, default=3600.0)
    a = ap.parse_args()
    offline(a) if a.cmd == 'offline' else node(a)


if __name__ == '__main__':
    main()
