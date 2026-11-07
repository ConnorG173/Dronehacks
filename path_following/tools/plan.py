#!/usr/bin/env python3
"""Plan a course with the GCOPTER planner, with config overrides.

Uses planner/build/gcopter_planner when present (VM, Jetson); otherwise runs the
planner inside the pf-dev:humble Docker image (laptop), building it there once.

    python3 plan.py CONFIG.yaml OUT_DIR [MaxVelMag=1.2 ...]
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(HERE, '..', 'team_path_following'))

from team_path_following.planner_runner import run_planner  # noqa: E402

NATIVE = os.path.join(REPO, 'planner', 'build', 'gcopter_planner')
DOCKER_BUILD = os.path.join(REPO, 'planner', 'build-docker')


def docker_wrapper():
    if not os.path.exists(os.path.join(DOCKER_BUILD, 'gcopter_planner')):
        subprocess.run(['docker', 'run', '--rm', '-v', f'{REPO}:{REPO}', '-w', f'{REPO}/planner', 'pf-dev:humble',
                        'bash', '-c', 'cmake -S . -B build-docker -DCMAKE_PREFIX_PATH=/opt/ros/humble >/dev/null'
                        ' && cmake --build build-docker -j8 >/dev/null'], check=True)
    return ['docker', 'run', '--rm', '-v', f'{REPO}:{REPO}', 'pf-dev:humble'], os.path.join(DOCKER_BUILD, 'gcopter_planner')


def plan(config, out_dir, overrides, attempts=3):
    if os.path.exists(NATIVE):
        wrapper, binary = None, NATIVE
    else:
        wrapper, binary = docker_wrapper()
    return run_planner(binary, os.path.abspath(config), os.path.abspath(out_dir), overrides,
                       attempts=attempts, wrapper=wrapper, timeout_s=180.0)


if __name__ == '__main__':
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    ov = dict(kv.split('=', 1) for kv in sys.argv[3:])
    path, summary = plan(sys.argv[1], sys.argv[2], ov)
    print(summary)
