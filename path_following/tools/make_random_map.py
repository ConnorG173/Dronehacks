#!/usr/bin/env python3
"""Random maze-style courses for generality tests (DHMAP map + planner config).

Each course: a floor, a boundary cage, 2-4 interior walls across the cage, each with
one opening (a full-height gap or a window). Checkpoints sit in the openings, in
order. The whole layout is rotated by a random angle and the launch heading is
random, so frame handling is exercised. Planner/optimizer keys are copied from a
template config (default planner/config/course_v3.yaml).

    python3 make_random_map.py OUT_DIR --seed 3 [--template CONFIG.yaml]
writes OUT_DIR/map.bin and OUT_DIR/course.yaml
"""
import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'team_path_following'))
from team_path_following.course import read_config_lines, write_dhmap  # noqa: E402

MISSION_KEYS = {'MapFile', 'MapBound', 'StartPos', 'StartHeadingDeg', 'HoverHeight', 'Checkpoint', 'Goal', 'OutDir'}
SPACING = 0.04


def grid(lo, hi, step=SPACING):
    n = max(1, int(round((hi - lo) / step)))
    return lo + step * (np.arange(n) + 0.5)


def wall(x0, x1, y0, y1, z0, z1, hole=None):
    """Box wall surface points (dense volume sampling of a thin slab is enough here)."""
    X, Y, Z = np.meshgrid(grid(x0, x1), grid(y0, y1), grid(z0, z1), indexing='ij')
    pts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1)
    if hole is not None:
        (hy0, hy1, hz0, hz1) = hole
        keep = ~((pts[:, 1] > hy0) & (pts[:, 1] < hy1) & (pts[:, 2] > hz0) & (pts[:, 2] < hz1))
        pts = pts[keep]
    return pts


def make(seed):
    rng = np.random.default_rng(seed)
    L = rng.uniform(5.0, 7.5)       # cage length (x)
    W = rng.uniform(4.0, 6.0)       # cage width (y)
    H = 3.0
    fly_z = rng.uniform(0.8, 1.2)
    t = 0.08                         # wall thickness
    pts = [wall(-L / 2, L / 2, -W / 2, W / 2, 0.0, 0.02)]                 # floor
    for x in (-L / 2 - t, L / 2):
        pts.append(wall(x, x + t, -W / 2, W / 2, 0.0, H))
    for y in (-W / 2 - t, W / 2):
        pts.append(wall(-L / 2, L / 2, y, y + t, 0.0, H))
    n_walls = int(rng.integers(2, 5))
    xs = np.linspace(-L / 2, L / 2, n_walls + 2)[1:-1] + rng.uniform(-0.3, 0.3, n_walls)
    cps = []
    for x in xs:
        if rng.random() < 0.7:   # full-height gap
            gw = rng.uniform(0.9, 1.3)
            gy = rng.uniform(-W / 2 + 0.4 + gw / 2, W / 2 - 0.4 - gw / 2)
            hole = (gy - gw / 2, gy + gw / 2, -1.0, H + 1.0)
            cps.append((x + t / 2, gy, fly_z, 0.10, min(0.15, gw / 2 - 0.35), 0.25))
        else:                     # window
            ws = rng.uniform(0.85, 1.0)
            gy = rng.uniform(-W / 2 + 0.4 + ws / 2, W / 2 - 0.4 - ws / 2)
            hole = (gy - ws / 2, gy + ws / 2, fly_z - ws / 2, fly_z + ws / 2)
            cps.append((x + t / 2, gy, fly_z, 0.08, 0.08, 0.10))
        pts.append(wall(x, x + t, -W / 2, W / 2, 0.0, H, hole))
    pts = np.concatenate(pts)
    start = np.array([-L / 2 + 0.6, rng.uniform(-W / 2 + 0.6, W / 2 - 0.6), 0.0])
    goal = np.array([L / 2 - 0.6, rng.uniform(-W / 2 + 0.6, W / 2 - 0.6), fly_z])

    rot = rng.uniform(-math.pi, math.pi)
    c, s = math.cos(rot), math.sin(rot)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    pts = pts @ R.T
    start, goal = R @ start, R @ goal
    cps = [tuple(R @ np.array(cp[:3])) + tuple(np.abs(R[:2, :2]) @ np.array(cp[3:5])) + (cp[5],)
           for cp in cps]
    # rotated axis-aligned boxes grow; keep them small so they stay inside the opening
    cps = [(cx, cy, cz, min(hx, 0.12), min(hy, 0.12), hz) for cx, cy, cz, hx, hy, hz in cps]
    r = math.hypot(L / 2 + t, W / 2 + t) + 0.1
    bound = (-r, r, -r, r, 0.0, H + 0.1)
    heading = math.degrees(rng.uniform(-math.pi, math.pi))
    return dict(points=pts.astype(np.float32), bound=bound, start=start, goal=goal, cps=cps,
                heading=heading, hover=fly_z, seed=seed)


def write(out_dir, course, template):
    os.makedirs(out_dir, exist_ok=True)
    write_dhmap(os.path.join(out_dir, 'map.bin'), course['points'])
    fmt = lambda v: '[' + ', '.join(f'{x:.4f}' for x in v) + ']'  # noqa: E731
    with open(os.path.join(out_dir, 'course.yaml'), 'w') as f:
        f.write(f'# random course, seed {course["seed"]} (path_following/tools/make_random_map.py)\n')
        f.write('MapFile: map.bin\n')
        f.write(f'MapBound: {fmt(course["bound"])}\n')
        f.write(f'StartPos: {fmt(course["start"])}\n')
        f.write(f'StartHeadingDeg: {course["heading"]:.2f}\n')
        f.write(f'HoverHeight: {course["hover"]:.3f}\n')
        for cp in course['cps']:
            f.write(f'Checkpoint: {fmt(cp)}\n')
        f.write(f'Goal: {fmt(course["goal"])}\n')
        f.write('OutDir: out\n')
        for k, v in read_config_lines(template):
            if k not in MISSION_KEYS:
                f.write(f'{k}: {v}\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out_dir')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--template', default=os.path.join(HERE, '..', '..', 'planner', 'config', 'course_v3.yaml'))
    a = ap.parse_args()
    c = make(a.seed)
    write(a.out_dir, c, a.template)
    print(f'{a.out_dir}: {len(c["points"])} points, {len(c["cps"])} checkpoints, heading {c["heading"]:.1f} deg')


if __name__ == '__main__':
    main()
