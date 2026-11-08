"""Summarize what PX4 received as vision (adapter sent.jsonl): NED position, FRD q [w,x,y,z]."""
import json, math, sys
import numpy as np

def load(p):
    t, P, Y, rc, sp = [], [], [], [], []
    for l in open(p):
        try:
            d = json.loads(l)
        except Exception:
            continue
        if d.get('kind') == 'vision':
            w, x, y, z = d['q']
            t.append(d['sample_us'] * 1e-6); P.append(d['position'])
            Y.append(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))); rc.append(d.get('reset_counter', 0))
        elif d.get('kind') == 'setpoint':
            sp.append(d['wall_ns'] * 1e-9)
    return np.array(t), np.array(P), np.unwrap(np.array(Y)), np.array(rc), np.array(sp)

for path in sys.argv[1:]:
    t, P, Y, rc, sp = load(path)
    if len(t) < 10:
        print(path, 'no vision'); continue
    dt = np.diff(t); dp = np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1)
    v = dp / np.maximum(dt, 1e-3)
    print(f'\n=== {path.split("/")[-2]}: {len(t)} vision msgs over {t[-1]-t[0]:.0f} s ({len(t)/(t[-1]-t[0]):.1f} Hz), max gap {dt.max():.2f} s, '
          f'setpoints {len(sp)}, reset_counter {rc.min()}..{rc.max()}')
    jumps = np.where(dp > 0.25)[0]
    print(f'  jumps > 0.25 m in one sample: {len(jumps)}' + ''.join(f'\n    t+{t[i]-t[0]:6.1f}s jump {dp[i]:.2f} m  yaw step {math.degrees(Y[i+1]-Y[i]):+.1f} deg' for i in jumps[:10]))
    # moving segments: speed > 0.15 m/s smoothed over 1 s
    k = max(1, int(round(1.0 / np.median(dt))))
    vs = np.convolve(v, np.ones(k) / k, 'same')
    mov = vs > 0.15
    segs, i = [], 0
    while i < len(mov):
        if mov[i]:
            j = i
            while j < len(mov) and mov[j]:
                j += 1
            if t[j] - t[i] > 5:
                segs.append((i, j))
            i = j
        i += 1
    for a, b in segs:
        L = dp[a:b].sum(); turn = math.degrees(np.abs(np.diff(Y[a:b + 1])).sum()); net = math.degrees(Y[b] - Y[a])
        print(f'  moving t+{t[a]-t[0]:6.1f}..{t[b]-t[0]:6.1f}s ({t[b]-t[a]:5.1f} s): path {L:5.2f} m, turned {turn:5.0f} deg (net {net:+5.0f}), '
              f'start ({P[a,0]:+.2f},{P[a,1]:+.2f},{P[a,2]:+.2f}) end ({P[b,0]:+.2f},{P[b,1]:+.2f},{P[b,2]:+.2f})')
