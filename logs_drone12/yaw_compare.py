"""VIO heading vs gyro heading vs EKF heading over a shadow/record log (team_imu_log track.jsonl)."""
import json, math, sys
import numpy as np
sys.path.insert(0, 'deploy/imu_fusion_shadow')
from heading_fix import qm, rodrigues, yaw, wrap

rows = [json.loads(l) for l in open(sys.argv[1]) if l.strip() and 'guard' not in l]
still_s = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
t = np.array([r['t'] for r in rows]); t0 = t[0]
FLU = np.diag([1.0, -1, -1])
def ekf_R(r):  # PX4 NED/FRD -> ENU-ish FLU for yaw comparison (sign of yaw flips)
    return FLU @ np.array(r['px4_R']) @ FLU if 'px4_R' in r else None
# gyro bias per source from the first still_s seconds
srcs = [s for s in ('g_px4', 'g_oak', 'g_px4usb') if any(s in r for r in rows)]
bias = {}
for s in srcs:
    R = np.zeros(3); T = 0.0
    for r in rows:
        if r['t'] - t0 > still_s: break
        if s in r: R += np.array(r[s]['r']); T += r[s]['dt']
    bias[s] = R / T if T > 0 else np.zeros(3)
    print(f'{s}: bias from first {still_s:.0f} s = {np.degrees(bias[s]).round(3).tolist()} deg/s')
# integrate heading changes
out = {k: [0.0] for k in ['lev', 'raw', 'ekf'] + srcs}
tt = {k: [t0] for k in out}
prev = {}
for r in rows:
    Rl = qm(*r['lev']['q']) if 'lev' in r else None
    for k in ('lev', 'raw'):
        if k in r:
            y = yaw(qm(*r[k]['q']))
            out[k].append(out[k][-1] + (wrap(y - prev[k]) if k in prev else 0.0)); prev[k] = y; tt[k].append(r['t'])
    E = ekf_R(r)
    if E is not None:
        y = yaw(E); out['ekf'].append(out['ekf'][-1] + (wrap(y - prev['ekf']) if 'ekf' in prev else 0.0)); prev['ekf'] = y; tt['ekf'].append(r['t'])
    for s in srcs:
        if s in r and Rl is not None:
            rc = np.array(r[s]['r']) - bias[s] * r[s]['dt']
            d = wrap(yaw(Rl @ rodrigues(rc)) - yaw(Rl))
            out[s].append(out[s][-1] + d); tt[s].append(r['t'])
P = np.array([r['lev']['p'] for r in rows if 'lev' in r])
dp = np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1)
print(f'\nduration {t[-1]-t0:.0f} s, levelled-VIO path {dp.sum():.2f} m, net {np.linalg.norm(P[-1,:2]-P[0,:2]):.2f} m, '
      f'jumps >0.25 m per 0.1 s: {int((dp > 0.25).sum())}, max step {dp.max():.2f} m')
print('net heading change over the log (deg):  ' + '  '.join(f'{k} {math.degrees(v[-1]):+7.2f}' for k, v in out.items()))
print('total |turn| (deg):                      ' + '  '.join(f'{k} {math.degrees(np.abs(np.diff(v)).sum()):7.1f}' for k, v in out.items()))
print('\nheading change since start, every 5 s (deg), on a common time base:')
keys = ['lev', 'ekf'] + srcs
grid = np.arange(t0, t[-1], 5.0)
I = {k: np.interp(grid, tt[k], out[k]) for k in keys}
print('    t      ' + ''.join(f'{k:>10s}' for k in keys) + '   lev-g_px4  ekf-lev')
for i, g in enumerate(grid):
    print(f'  {g-t0:5.0f}s  ' + ''.join(f'{math.degrees(I[k][i]):+10.2f}' for k in keys) +
          f'   {math.degrees(I["lev"][i]-I["g_px4"][i]):+8.2f}  {math.degrees(I["ekf"][i]-I["lev"][i]):+7.2f}')
