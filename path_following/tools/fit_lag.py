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
    reader = rosbag2_py.SequentialReader()
    fmt = 'mcap' if path.endswith('.mcap') or 'mcap' in path else 'sqlite3'
    try:
        reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=fmt), rosbag2_py.ConverterOptions('', ''))
    except Exception:
        reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3' if fmt == 'mcap' else 'mcap'),
                    rosbag2_py.ConverterOptions('', ''))
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src')
    ap.add_argument('--states', default='takeoff,course,landing')
    ap.add_argument('--dt', type=float, default=0.02)
    a = ap.parse_args()
    if a.src.endswith('.npz'):
        z = np.load(a.src)
        data = dict(t=z['t'], u=z['u'], v=z['v'], state=z['state'] if 'state' in z else np.full(len(z['t']), 2))
    else:
        data = load_bag(a.src)
    names = ['idle', 'takeoff', 'course', 'landing', 'landed', 'done', 'abort']
    keep = np.isin(data['state'], [names.index(s) for s in a.states.split(',')])
    keep &= np.all(np.isfinite(data['u']), 1) & np.all(np.isfinite(data['v']), 1)
    best = fit(data['t'][keep], data['u'][keep], data['v'][keep], a.dt)
    if best:
        print(f'\nconfig/tracker.yaml:  tracker.tau_v: {best[0]:.2f}   tracker.t_delay: {best[1]:.2f}')


if __name__ == '__main__':
    main()
