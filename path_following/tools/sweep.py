#!/usr/bin/env python3
"""Grid sweep of tracker / sim / mission parameters in the kinematic sim.

    python3 sweep.py COURSE.yaml PATH.bin t_lead=0,0.3,0.6 kc=1,1.5 sim.setpoint_rate_hz=10,50 \
        [--seeds 3] [--speed-scale 1.0] [--base kz=2] [--csv out.csv]

Each grid point runs --seeds noise seeds; reports worst-case cross-track/clearance
and mean course time.
"""
import argparse
import csv
import itertools
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'team_path_following'))

import kinsim  # noqa: E402
from team_path_following.course import load_course, read_dhmap  # noqa: E402
from team_path_following.clearance import ClearanceMap  # noqa: E402
from team_path_following.mission import MissionParams  # noqa: E402
from team_path_following.pathfile import Trajectory  # noqa: E402
from team_path_following.tracker import TrackerParams  # noqa: E402


def apply(objs, key, value):
    tp, sp, mp = objs
    target = tp
    if key.startswith('sim.'):
        target, key = sp, key[4:]
    elif key.startswith('mission.'):
        target, key = mp, key[8:]
    cur = getattr(target, key)
    setattr(target, key, value if isinstance(cur, str) else type(cur)(value))


def run_point(course, traj, clear, settings, seeds):
    rows = []
    for seed in range(seeds):
        sp = kinsim.SimParams(seed=seed)
        tp = TrackerParams()
        mp = MissionParams()
        objs = (tp, sp, mp)
        for k, v in settings:
            apply(objs, k, v)
        if not any(k == 'v_max' for k, _ in settings):
            tp.v_max = sp.max_speed_mps - 0.1
        rows.append(kinsim.run(course, traj, tp, mp, sp, clearance=clear))
    ok = all(r['result'] == 'done' and r['checkpoints'].split('/')[0] == r['checkpoints'].split('/')[1]
             and r['touches'] == 0 for r in rows)
    return dict(
        ok=ok,
        course_s=round(float(np.mean([r['course_time_s'] for r in rows])), 2),
        mission_s=round(float(np.mean([r['mission_time_s'] for r in rows])), 2),
        land_err=max(r['land_err_m'] for r in rows),
        max_xt=max(r['max_xt_m'] for r in rows),
        rms_xt=round(float(np.mean([r['rms_xt_m'] for r in rows])), 3),
        min_clear=min(r['min_clear_m'] for r in rows),
        touches=sum(r['touches'] for r in rows),
        clamped=sum(r['clamped_ticks'] for r in rows),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('course')
    ap.add_argument('path')
    ap.add_argument('axes', nargs='*')
    ap.add_argument('--seeds', type=int, default=2)
    ap.add_argument('--speed-scale', type=float, default=1.0)
    ap.add_argument('--base', action='append', default=[])
    ap.add_argument('--csv')
    a = ap.parse_args()
    course = load_course(a.course)
    traj = Trajectory.load(a.path, course)
    if a.speed_scale != 1.0:
        traj = traj.time_scaled(a.speed_scale)
    clear = ClearanceMap(read_dhmap(course.map_file), course.map_bound)
    base = [tuple(kv.split('=', 1)) for kv in a.base]
    axes = [(kv.split('=', 1)[0], kv.split('=', 1)[1].split(',')) for kv in a.axes]
    names = [k for k, _ in axes]
    out = []
    print(f'plan {traj.duration:.2f} s')
    header = names + ['ok', 'mission_s', 'land_err', 'course_s', 'max_xt', 'rms_xt', 'min_clear', 'touches', 'clamped']
    print('  '.join(f'{h:>9s}' for h in header))
    for combo in itertools.product(*[v for _, v in axes]):
        settings = base + list(zip(names, combo))
        r = run_point(course, traj, clear, settings, a.seeds)
        row = list(combo) + [r[h] for h in header[len(names):]]
        out.append(row)
        print('  '.join(f'{str(x):>9s}' for x in row), flush=True)
    if a.csv:
        with open(a.csv, 'w', newline='') as handle:
            w = csv.writer(handle)
            w.writerow(header)
            w.writerows(out)


if __name__ == '__main__':
    main()
