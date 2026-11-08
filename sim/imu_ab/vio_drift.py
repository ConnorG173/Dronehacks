#!/usr/bin/env python3
"""VIO drift vs Gazebo ground truth over one run, for the cuVSLAM IMU-fusion A/B.

    python3 vio_drift.py BAG [--states takeoff,course,landing] [--json out.json]
    python3 vio_drift.py INPUT_BAG --vio-bag REPLAY_BAG --stamps header     (offline replay, sim/imu_ab/replay.sh)

/visual_slam/tracking/odometry (base_link in odom) against /sim/ground_truth/odom (base_link in course),
both by bag receive time. Alignment = what a tracker with a perfect launch pose sees: one rigid
yaw-only transform fixed at the first sample of the window (same as path_following/tools/vio_err.py
'at-go'). Reports
  horizontal ATE (rms / max / end), z error, and
  attitude drift: aligned-VIO roll, pitch and yaw minus ground truth, as the change since the window
  start (so a constant mounting/alignment offset is not counted) -- 'tilt' is the angle between the
  VIO and GT body z axes, minus its value at the start.
Also the GT total rotation |yaw| over the window (how much the route turns).
Needs a sourced ROS 2 shell (rosbag2_py)."""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'path_following', 'tools'))
from eval_bag import read_bag  # noqa: E402

GT, VIO, ST = '/sim/ground_truth/odom', '/visual_slam/tracking/odometry', '/team/mission/state'


def qmat(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rpy(R):
    """ZYX Euler (roll, pitch, yaw) of a rotation matrix."""
    return (math.atan2(R[2, 1], R[2, 2]), math.asin(max(-1.0, min(1.0, -R[2, 0]))), math.atan2(R[1, 0], R[0, 0]))


def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def series(msgs, header=False):
    t = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 if header else s for s, m in msgs])
    p = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for _, m in msgs])
    R = np.array([qmat(m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z,
                       m.pose.pose.orientation.w) for _, m in msgs])
    return t, p, R


def nearest(ts, tt):
    k = np.clip(np.searchsorted(ts, tt), 1, len(ts) - 1)
    return np.where(np.abs(ts[k - 1] - tt) < np.abs(ts[k] - tt), k - 1, k)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--states', default='takeoff,course,landing')
    ap.add_argument('--dt', type=float, default=0.05)
    ap.add_argument('--json')
    ap.add_argument('--vio-bag', help='take VIO from this bag (a replay) instead of BAG')
    ap.add_argument('--stamps', default='header', choices=['recv', 'header'],
                    help='time base: message header stamps (sim time of the sensor data: default, no VIO-latency '
                         'artifact in turns) or bag receive time')
    a = ap.parse_args()
    m = read_bag(a.bag, [GT, ST] + ([] if a.vio_bag else [VIO]))
    if a.vio_bag:
        m[VIO] = read_bag(a.vio_bag, [VIO])[VIO]
    if not m[GT] or not m[VIO]:
        sys.exit(f'need both: gt {len(m[GT])} vio {len(m[VIO])}')
    hdr = a.stamps == 'header'
    tg, pg, Rg = series(m[GT], hdr)
    tv, pv, Rv = series(m[VIO], hdr)
    st_msgs = m[ST]
    rg = np.array([r for r, _ in m[GT]])
    sg = np.array([x.header.stamp.sec + x.header.stamp.nanosec * 1e-9 for _, x in m[GT]])
    rtf = (sg[-1] - sg[0]) / max(rg[-1] - rg[0], 1e-9)   # sim seconds per wall second while recording
    if hdr:   # mission state has no header: map its receive time onto GT's stamp clock
        st_msgs = [(float(np.interp(r, rg, tg)), x) for r, x in st_msgs]
    phases = []
    for s, x in st_msgs:
        if not phases or phases[-1][1] != x.data:
            phases.append((s, x.data))
    keep = a.states.split(',')
    spans = [(s, phases[i + 1][0] if i + 1 < len(phases) else tg[-1], n) for i, (s, n) in enumerate(phases)]
    win = [sp for sp in spans if sp[2] in keep]
    if not win:
        sys.exit(f'no states {keep} in the bag: phases {[n for _, n in phases]}')
    t = np.arange(max(win[0][0], tv[0], tg[0]), min(win[-1][1], tv[-1], tg[-1]), a.dt)
    ig, iv = nearest(tg, t), nearest(tv, t)
    G = np.stack([np.interp(t, tg, pg[:, k]) for k in range(3)], 1)
    V = np.stack([np.interp(t, tv, pv[:, k]) for k in range(3)], 1)
    RG, RV = Rg[ig], Rv[iv]
    # at-go alignment: yaw-only, from the first orientations
    dyaw = rpy(RG[0])[2] - rpy(RV[0])[2]
    A = rotz(dyaw)
    E = V @ A.T + (G[0] - A @ V[0]) - G
    h = np.linalg.norm(E[:, :2], axis=1)
    # attitude: aligned VIO orientation vs GT
    RVa = np.einsum('ij,njk->nik', A, RV)
    ev = np.array([rpy(r) for r in RVa]); eg = np.array([rpy(r) for r in RG])
    d = ev - eg
    d = (d + math.pi) % (2 * math.pi) - math.pi
    d = np.unwrap(d, axis=0)
    d -= d[0]
    tilt = np.degrees(np.arccos(np.clip(np.einsum('ni,ni->n', RVa[:, :, 2], RG[:, :, 2]), -1, 1)))
    tilt_drift = tilt - tilt[0]
    gyaw = np.unwrap(eg[:, 2])
    turned = float(np.degrees(np.abs(np.diff(gyaw)).sum()))
    dur = float(t[-1] - t[0])
    vgap = float(np.max(np.diff(tv)))
    rd, pd, yd = (np.degrees(d[:, k]) for k in range(3))
    res = dict(
        window_s=round(dur, 1), vio_hz=round(len(tv) / (tv[-1] - tv[0]), 1), vio_max_gap_s=round(vgap, 2),
        gt_turned_deg=round(turned, 0),
        ate_h_rms=round(float(np.sqrt(np.mean(h ** 2))), 3), ate_h_max=round(float(h.max()), 3),
        end_h=round(float(h[-1]), 3), z_rms=round(float(np.sqrt(np.mean(E[:, 2] ** 2))), 3),
        end_z=round(float(E[-1, 2]), 3),
        roll_end_deg=round(float(rd[-1]), 2), pitch_end_deg=round(float(pd[-1]), 2),
        tilt_end_deg=round(float(tilt_drift[-1]), 2), tilt_max_deg=round(float(np.abs(tilt_drift).max()), 2),
        yaw_end_deg=round(float(yd[-1]), 2), yaw_max_deg=round(float(np.abs(yd).max()), 2),
        yaw_rate_deg_per_100deg_turn=round(float(yd[-1]) / max(turned, 1e-9) * 100, 3),
        tilt0_deg=round(float(tilt[0]), 2), rtf_from_stamps=round(float(rtf), 3), stamps=a.stamps,
    )
    print(f"vio_drift: window {res['window_s']} s, VIO {res['vio_hz']} Hz (max gap {res['vio_max_gap_s']} s), "
          f"route turned {res['gt_turned_deg']:.0f} deg, RTF (GT stamps / wall) {res['rtf_from_stamps']:.2f}"
          + ('  ** RTF < 0.9 **' if res['rtf_from_stamps'] < 0.9 else ''))
    print(f"  horiz ATE rms {res['ate_h_rms']:.3f} max {res['ate_h_max']:.3f} end {res['end_h']:.3f} m | "
          f"z rms {res['z_rms']:.3f} end {res['end_z']:+.3f} m")
    print(f"  attitude drift at end: roll {res['roll_end_deg']:+.2f} pitch {res['pitch_end_deg']:+.2f} "
          f"tilt {res['tilt_end_deg']:+.2f} (max {res['tilt_max_deg']:.2f}) yaw {res['yaw_end_deg']:+.2f} "
          f"(max {res['yaw_max_deg']:.2f}) deg; initial VIO-vs-GT tilt {res['tilt0_deg']:.2f} deg")
    print('  time   GT (x, y, z)          err (x, y, z) m          yaw err  tilt err deg')
    for k in range(0, len(t), max(1, int(4.0 / a.dt))):
        print(f'  {t[k] - t[0]:5.1f}  ({G[k, 0]:6.2f},{G[k, 1]:6.2f},{G[k, 2]:5.2f})  '
              f'({E[k, 0]:+.3f},{E[k, 1]:+.3f},{E[k, 2]:+.3f})  {yd[k]:+6.2f}  {tilt_drift[k]:+5.2f}')
    out = a.json or os.path.join(os.path.dirname(os.path.abspath(a.bag)), 'vio_drift.json')
    with open(out, 'w') as f:
        json.dump(res, f, indent=1)


if __name__ == '__main__':
    main()
