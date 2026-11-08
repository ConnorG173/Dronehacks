#!/usr/bin/env python3
"""Score a recorded run against ground truth: the kinsim metrics, from a rosbag2.

Needs a sourced ROS 2 shell (rosbag2_py). Uses /sim/ground_truth/odom (map frame),
/team/mission/state and /team/tracker/debug; the plan is read from the node's out_dir.

    python3 eval_bag.py BAG_DIR COURSE.yaml PATH.bin [--name run12] [--append docs/results/path_following.md]
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'team_path_following'))
from team_path_following.clearance import ClearanceMap  # noqa: E402
from team_path_following.course import load_course, read_dhmap, score_checkpoints  # noqa: E402
from team_path_following.pathfile import Trajectory  # noqa: E402


def read_bag(path, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
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
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {t: [] for t in topics}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic in out:
            out[topic].append((stamp * 1e-9, deserialize_message(data, get_message(types[topic]))))
    return out


def evaluate(bag, course, traj, radius=0.20):
    msgs = read_bag(bag, ['/sim/ground_truth/odom', '/team/mission/state', '/team/tracker/debug'])
    gt = msgs['/sim/ground_truth/odom']
    if not gt:
        sys.exit('no ground truth in bag')
    t = np.array([s for s, _ in gt])
    p = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for _, m in gt])
    st = msgs['/team/mission/state']
    first = {}
    for s, m in st:
        first.setdefault(m.data, s)
    t0, t1 = first.get('course'), first.get('landing')
    tl = first.get('landed')
    tk = first.get('takeoff')
    inc = (t >= (t0 or np.inf)) & (t <= (t1 if t1 else np.inf))
    clear = ClearanceMap(read_dhmap(course.map_file), course.map_bound)
    c = clear.distance(p[inc]) if inc.any() else np.array([np.nan])
    # cross-track: distance to the plan polyline (nearest sample)
    xt = np.array([np.min(np.linalg.norm(traj.p - q, axis=1)) for q in p[inc]]) if inc.any() else np.array([np.nan])
    fly = (t >= (t0 or np.inf)) & (t <= (tl or t[-1]))
    passed, miss = score_checkpoints(p[fly], course.checkpoints)
    hit, worst = sum(passed), (max(miss) if miss else 0.0)
    touches = int(np.sum(np.diff((c < radius).astype(int)) == 1) + (1 if len(c) and c[0] < radius else 0))
    # course time on the node's own clock (sim time in Gazebo with tracker_use_sim_time): RTF-independent
    t_node = None
    dbg = msgs['/team/tracker/debug']
    if dbg and dbg[0][1].layout.dim:
        lab = dbg[0][1].layout.dim[0].label.split(',')
        it, ist = lab.index('t'), lab.index('state')
        rows = [(m.data[it], m.data[ist]) for _, m in dbg if len(m.data) > max(it, ist)]
        tc = next((tt for tt, ss in rows if ss == 2), None)
        tlnd = next((tt for tt, ss in rows if ss == 3), None)
        if tc is not None and tlnd is not None:
            t_node = round(float(tlnd - tc), 2)
    return dict(
        result=st[-1][1].data if st else '?',
        mission_time_s=round(tl - tk, 2) if tl and tk else None,
        course_time_s=round(t1 - t0, 2) if t1 and t0 else None,
        course_time_node_s=t_node,
        plan_time_s=round(traj.duration, 2),
        max_xt_m=round(float(np.nanmax(xt)), 3),
        rms_xt_m=round(float(np.sqrt(np.nanmean(xt ** 2))), 3),
        min_clear_m=round(float(np.nanmin(c)), 3),
        touches=touches,
        checkpoints=f'{hit}/{len(course.checkpoints)}',
        cp_missed=[i + 1 for i, ok in enumerate(passed) if not ok],
        cp_worst_miss_m=round(worst, 3),
        land_err_m=round(float(np.linalg.norm(p[-1, :2] - traj.p[-1, :2])), 3),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('course')
    ap.add_argument('path')
    ap.add_argument('--name', default='')
    ap.add_argument('--notes', default='')
    ap.add_argument('--append')
    ap.add_argument('--via', default='', help='1-based checkpoint numbers that are routing aids, not course '
                    'checkpoints (e.g. 5,6 for course_v3_gateb): left out of real_checkpoints')
    ap.add_argument('--cp-tol', type=float, default=0.15, help='real checkpoint: within this of its planning box (m)')
    ap.add_argument('--pad-r', type=float, default=0.51, help='pad ring radius for in_pad (m; CAD ring r20 in)')
    a = ap.parse_args()
    course = load_course(a.course)
    traj = Trajectory.load(a.path, course)
    import warnings
    warnings.simplefilter('ignore', RuntimeWarning)
    r = evaluate(a.bag, course, traj)
    via = {int(x) for x in a.via.split(',') if x.strip()}
    real = [k for k in range(1, len(course.checkpoints) + 1) if k not in via]
    # a real checkpoint is an opening, wider than its planning box: crossed if the path comes within
    # --cp-tol of the box (planning boxes are a few cm; the openings are 0.9 m)
    from team_path_following.course import Box
    msgs = read_bag(a.bag, ['/sim/ground_truth/odom', '/team/mission/state'])
    pts = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]
                    for _, m in msgs['/sim/ground_truth/odom']])
    tt = np.array([s_ for s_, _ in msgs['/sim/ground_truth/odom']])
    first = {}
    for s_, m in msgs['/team/mission/state']:
        first.setdefault(m.data, s_)
    t0, tl = first.get('course', np.inf), first.get('landed', tt[-1] if len(tt) else np.inf)
    sel = pts[(tt >= t0) & (tt <= tl)]
    loose = [Box(b.center, b.half + a.cp_tol) for b in course.checkpoints]
    lp, _ = score_checkpoints(sel, loose) if len(sel) else ([False] * len(loose), None)
    missed_real = [k for k in real if not lp[k - 1]]
    r['real_cp_tol_m'] = a.cp_tol
    r['real_checkpoints'] = f'{len(real) - len(missed_real)}/{len(real)}'
    r['real_missed'] = missed_real
    r['in_pad'] = bool(r['land_err_m'] <= a.pad_r)
    print(json.dumps(r))
    if a.append:
        cols = ['result', 'mission_time_s', 'course_time_s', 'plan_time_s', 'max_xt_m', 'rms_xt_m',
                'min_clear_m', 'touches', 'checkpoints', 'cp_worst_miss_m', 'land_err_m']
        with open(a.append, 'a') as f:
            f.write('| gazebo | ' + (a.name or os.path.basename(a.bag.rstrip('/'))) + ' | '
                    + ' | '.join(str(r[c]) for c in cols) + f' | {a.notes} |\n')


if __name__ == '__main__':
    main()
