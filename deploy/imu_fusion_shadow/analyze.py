#!/usr/bin/env python3
"""Score a shadow-VIO log (track.jsonl from team_imu_log.py). Pure numpy, runs anywhere (laptop or drone).

    python3 analyze.py logs/<stamp>_oak [--still 10]

Ground truth on the bench: the drone is put back on its taped start mark at the start heading, so for
every estimator the start-to-end change is pure drift (loop closure). PX4's attitude is the gravity
reference for tilt (PX4 has no compass; its yaw follows the flight VIO, so it is NOT a heading truth).
Estimators: raw = flight cuVSLAM (no IMU), lev = the teammates' levelled copy PX4 flies on,
imu = the shadow with IMU fusion, hfix = lev with the gyro heading correction (--hfix).

Reports per estimator, over the first and last --still seconds of the log:
  closure horizontal / z error (m), closure yaw error (deg), tilt error vs PX4 at start / end / max (deg),
  path length (m), message rate (Hz), share of time with vo_state != 1.
'raw' tilt is vs its own start-up frame, which is not level (the leveller exists for that): read its tilt
numbers as drift only (end minus start).
"""
import argparse
import json
import math
import os

import numpy as np

FLU_FRD = np.diag([1.0, -1.0, -1.0])


def qm(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def unwrap_valid(a):
    """np.unwrap that skips NaNs (one missing sample must not poison the rest)."""
    a = np.asarray(a, float)
    out = np.full_like(a, np.nan)
    ok = np.isfinite(a)
    if ok.any():
        out[ok] = np.unwrap(a[ok])
    return out


def ang(a, b):
    return math.degrees(math.acos(float(np.clip(np.dot(a, b) / np.linalg.norm(a) / np.linalg.norm(b), -1, 1))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--still', type=float, default=10.0, help='seconds at each end treated as the still windows')
    ap.add_argument('--hfix', action='store_true', help='also score the gyro heading correction (runs heading_fix.py offline)')
    a = ap.parse_args()
    path = a.log if a.log.endswith('.jsonl') else os.path.join(a.log, 'track.jsonl')
    if a.hfix:
        import subprocess
        import sys
        subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'heading_fix.py'),
                        'offline', path], check=True)
        path = os.path.join(os.path.dirname(path), 'track_hfix.jsonl')
    rows = [json.loads(x) for x in open(path) if x.strip()]
    guard = [r['guard'] for r in rows if 'guard' in r]
    rows = [r for r in rows if 'guard' not in r]
    t = np.array([r['t'] for r in rows])
    T = t[-1] - t[0]
    print(f'{path}: {T:.0f} s, {len(rows)} rows' + (f'  GUARD: {guard}' if guard else ''))
    s0, s1 = t < t[0] + a.still, t > t[-1] - a.still
    px4 = [np.array(r['px4_R']) if 'px4_R' in r else None for r in rows]
    pyaw = unwrap_valid([math.atan2(R[1, 0], R[0, 0]) if R is not None else np.nan for R in px4])
    turned = np.degrees(np.nansum(np.abs(np.diff(pyaw))))
    print(f'rotation over the test (PX4 attitude, sum |d yaw|): {turned:.0f} deg')
    out = {}
    print(f"{'est':4s} {'Hz':>5s} {'lost%':>6s} {'closure h':>9s} {'z':>7s} {'yaw':>7s} {'tilt0':>6s} {'tiltEnd':>7s} "
          f"{'tiltMax':>7s} {'path':>6s}")
    for k in ('raw', 'lev', 'imu', 'hfix'):
        have = np.array([k in r for r in rows])
        if have.sum() < 20:
            print(f'{k:4s} no data')
            continue
        P = np.array([r[k]['p'] if k in r else [np.nan] * 3 for r in rows])
        R = [qm(*r[k]['q']) if k in r else None for r in rows]
        vo = np.array([r[k].get('vo') if k in r else None for r in rows], dtype=object)
        ns = np.array([r[k].get('n', np.nan) if k in r else np.nan for r in rows], dtype=float)
        hz = (np.nanmax(ns) - np.nanmin(ns)) / (t[have][-1] - t[have][0]) if np.isfinite(ns).any() else float('nan')
        lost = 100.0 * np.mean([v is not None and v != 1 for v in vo[have]])
        yaw = unwrap_valid([math.atan2(r[1, 0], r[0, 0]) if r is not None else np.nan for r in R])
        tilt = np.array([ang(R[i].T @ [0, 0, 1.0], FLU_FRD @ (px4[i].T @ [0, 0, -1.0]))
                         if R[i] is not None and px4[i] is not None else np.nan for i in range(len(rows))])
        a0, a1 = s0 & have, s1 & have
        dp = np.nanmean(P[a1], 0) - np.nanmean(P[a0], 0)
        dyaw = (math.degrees(np.nanmean(yaw[a1]) - np.nanmean(yaw[a0])) + 180) % 360 - 180   # same heading on the mark
        Pv = P[have]
        path_len = float(np.nansum(np.linalg.norm(np.diff(Pv[:, :2], axis=0), axis=1)))
        res = dict(hz=round(hz, 1), lost_pct=round(lost, 1), closure_h=round(float(np.hypot(dp[0], dp[1])), 3),
                   closure_z=round(float(dp[2]), 3), closure_yaw_deg=round(dyaw, 2),
                   tilt_start=round(float(np.nanmean(tilt[a0])), 2), tilt_end=round(float(np.nanmean(tilt[a1])), 2),
                   tilt_max=round(float(np.nanmax(tilt)), 2), path_m=round(path_len, 2))
        out[k] = res
        print(f"{k:4s} {res['hz']:5.1f} {res['lost_pct']:6.1f} {res['closure_h']:9.3f} {res['closure_z']:+7.3f} "
              f"{res['closure_yaw_deg']:+7.2f} {res['tilt_start']:6.2f} {res['tilt_end']:7.2f} {res['tilt_max']:7.2f} "
              f"{res['path_m']:6.2f}")
    out['turned_deg'] = round(float(turned), 0)
    out['duration_s'] = round(float(T), 1)
    json.dump(out, open(os.path.join(os.path.dirname(path), 'summary.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
