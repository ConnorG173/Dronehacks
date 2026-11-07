#!/usr/bin/env python3
"""Fit the PX4 velocity loop as seen through the flight interface: a first-order lag
tau_v plus a dead time d, from commanded vs measured velocity.

    v[k+1] = v[k] + (dt / tau_v) (u[k - d/dt] - v[k])     (horizontal axes, map ENU)

Input: a rosbag2 directory with /team/tracker/debug (mission_tracker's debug topic;
needs rosbag2_py, i.e. a sourced ROS 2 shell), or a .npz with arrays t, u (N,3), v (N,3).
Prints tau_v and t_delay to put into config/tracker.yaml (tracker.tau_v, tracker.t_delay).

    python3 fit_lag.py BAG_DIR_OR_NPZ [--states course] [--dt 0.02]
"""
import argparse
import sys

import numpy as np

FIELDS = None


def load_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import Float64MultiArray
    reader = None
    for sid in ('mcap', 'sqlite3'):
        try:
            r = rosbag2_py.SequentialReader()
            r.open(rosbag2_py.StorageOptions(uri=path, storage_id=sid), rosbag2_py.ConverterOptions('', ''))
            reader = r
            break
        except Exception:  # noqa: BLE001
            continue
    if reader is None:
        sys.exit(f'cannot open bag {path}')
    rows, labels = [], None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != '/team/tracker/debug':
            continue
        m = deserialize_message(data, Float64MultiArray)
        if labels is None and m.layout.dim:
            labels = m.layout.dim[0].label.split(',')
        rows.append(list(m.data))
    if not rows:
        sys.exit('no /team/tracker/debug messages in the bag')
    a = np.array(rows)
    ix = {n: i for i, n in enumerate(labels)}
    return dict(t=a[:, ix['t']], state=a[:, ix['state']],
                u=a[:, [ix['vcmd_x'], ix['vcmd_y'], ix['vcmd_z']]],
                v=a[:, [ix['v_x'], ix['v_y'], ix['v_z']]])


def fit(t, u, v, dt, delays=np.arange(0.0, 0.41, 0.02)):
    """Least squares for b = dt/tau per candidate delay; best residual wins."""
    tg = np.arange(t[0], t[-1], dt)
    U = np.stack([np.interp(tg, t, u[:, k]) for k in range(3)], 1)
    V = np.stack([np.interp(tg, t, v[:, k]) for k in range(3)], 1)
    best = None
    for axes, name in (((0, 1), 'xy'), ((2,), 'z')):
        res = []
        for d in delays:
            n = int(round(d / dt))
            x, y = [], []
            for k in axes:
                uk = U[: len(U) - 1 - n, k] if n == 0 else U[: len(U) - 1 - n, k]
                vk = V[n: len(V) - 1, k]
                dv = V[n + 1:, k] - vk
                x.append(uk - vk)
                y.append(dv)
            x, y = np.concatenate(x), np.concatenate(y)
            if np.dot(x, x) < 1e-9:
                continue
            b = float(np.dot(x, y) / np.dot(x, x))
            r = float(np.mean((y - b * x) ** 2))
            res.append((r, d, dt / b if b > 1e-6 else float('inf')))
        res.sort()
        if not res:
            continue
        r, d, tau = res[0]
        print(f'{name}: tau_v = {tau:.3f} s, t_delay = {d:.2f} s (residual {r:.2e}, {len(tg)} samples)')
        if name == 'xy':
            best = (tau, d)
    return best


def fit2(t, u, v, dt, delays=np.arange(0.0, 0.41, 0.02)):
    """Second-order ARX per horizontal axis pair: v[k+1] = a1 v[k] + a2 v[k-1] + b u[k-n].
    Reports natural frequency, damping, dead time and steady-state gain b / (1 - a1 - a2):
    a gain != 1 means PX4 achieves a multiple of the command (e.g. a velocity-estimate scale error)."""
    tg = np.arange(t[0], t[-1], dt)
    U = np.stack([np.interp(tg, t, u[:, k]) for k in range(3)], 1)
    V = np.stack([np.interp(tg, t, v[:, k]) for k in range(3)], 1)
    best = None
    for d in delays:
        n = int(round(d / dt))
        rows, y = [], []
        for k in (0, 1):
            m = len(V) - 1
            idx = np.arange(max(1, n), m)
            rows.append(np.stack([V[idx, k], V[idx - 1, k], U[idx - n, k]], 1))
            y.append(V[idx + 1, k])
        X, Y = np.concatenate(rows), np.concatenate(y)
        if len(Y) < 50:
            continue
        theta, *_ = np.linalg.lstsq(X, Y, rcond=None)
        r = float(np.mean((Y - X @ theta) ** 2))
        if best is None or r < best[0]:
            best = (r, d, theta)
    if best is None:
        return None
    r, d, (a1, a2, b) = best
    gain = b / (1 - a1 - a2) if abs(1 - a1 - a2) > 1e-9 else float('inf')
    poles = np.roots([1.0, -a1, -a2]).astype(complex)
    s = np.log(poles) / dt
    wn = float(np.sqrt(abs(s[0] * s[1])))
    zeta = float(-(s[0] + s[1]).real / (2 * wn)) if wn > 0 else float('nan')
    print(f'xy 2nd order: wn = {wn:.2f} rad/s, zeta = {zeta:.2f}, t_delay = {d:.2f} s, steady-state gain = '
          f'{gain:.2f} (residual {r:.2e})')
    if zeta < 0.7:
        print('  underdamped: the first-order fit above under-describes this loop; treat tau_v with care')
    if abs(gain - 1.0) > 0.15:
        print('  steady-state gain far from 1: the vehicle achieves a multiple of the command (velocity-estimate scale?)')
    return wn, zeta, d, gain


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src')
    ap.add_argument('--states', default='takeoff,course,landing')
    ap.add_argument('--dt', type=float, default=0.02)
    ap.add_argument('--window', type=float, nargs=2, metavar=('START', 'END'),
                    help='seconds after the first selected sample (e.g. skip a crash)')
    a = ap.parse_args()
    if a.src.endswith('.npz'):
        z = np.load(a.src)
        data = dict(t=z['t'], u=z['u'], v=z['v'], state=z['state'] if 'state' in z else np.full(len(z['t']), 2))
    else:
        data = load_bag(a.src)
    names = ['idle', 'takeoff', 'course', 'landing', 'landed', 'done', 'abort']
    keep = np.isin(data['state'], [names.index(s) for s in a.states.split(',')])
    keep &= np.all(np.isfinite(data['u']), 1) & np.all(np.isfinite(data['v']), 1)
    if a.window and keep.any():
        t0 = data['t'][keep][0]
        keep &= (data['t'] >= t0 + a.window[0]) & (data['t'] <= t0 + a.window[1])
    if keep.sum() < 50:
        sys.exit(f'fit_lag: only {int(keep.sum())} samples in states {a.states}: nothing to fit')
    best = fit(data['t'][keep], data['u'][keep], data['v'][keep], a.dt)
    fit2(data['t'][keep], data['u'][keep], data['v'][keep], a.dt)
    if best:
        print(f'\nconfig/tracker.yaml:  tracker.tau_v: {best[0]:.2f}   tracker.t_delay: {best[1]:.2f}')


if __name__ == '__main__':
    main()
