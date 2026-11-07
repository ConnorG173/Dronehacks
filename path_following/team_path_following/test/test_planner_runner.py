"""plan_best with a fake planner: crashes and failures are retried, the fastest plan wins."""
import os
import stat
import textwrap

import numpy as np
import pytest

from conftest import REPO
from team_path_following.planner_runner import plan_best, plan_duration


def fake_planner(tmp_path, script_body):
    """A stand-in 'gcopter_planner CONFIG --out DIR' driven by a counter file."""
    p = os.path.join(tmp_path, 'fake_planner.py')
    with open(p, 'w') as f:
        f.write('#!/usr/bin/env python3\n' + textwrap.dedent(script_body))
    os.chmod(p, os.stat(p).st_mode | stat.S_IEXEC)
    return p


BODY = '''
import os, signal, struct, sys
out = sys.argv[sys.argv.index('--out') + 1]
counter = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'count')
n = int(open(counter).read()) if os.path.exists(counter) else 0
open(counter, 'w').write(str(n + 1))
plan = {plan}
kind = plan[n % len(plan)]
if kind == 'segv':
    os.kill(os.getpid(), signal.SIGSEGV)
if kind == 'fail':
    print('ERROR: no route'); sys.exit(1)
T = float(kind)
rows = [[t * 0.01] + [0.0] * 11 for t in range(int(T / 0.01))] + [[T] + [0.0] * 11]
with open(os.path.join(out, 'path.bin'), 'wb') as f:
    f.write(b'DHPATH\\0\\0' + struct.pack('<IIQd', 1, 12, len(rows), 0.01))
    for r in rows:
        f.write(struct.pack('<12d', *r))
print('GCOPTER plan OK')
'''

CONFIG = os.path.join(REPO, 'planner', 'config', 'course_v3.yaml')


def test_crash_and_failure_are_retried_and_fastest_kept(tmp_path):
    exe = fake_planner(tmp_path, BODY.format(plan="['segv', '12.0', 'fail', '9.5', '11.0']"))
    out = os.path.join(tmp_path, 'out')
    path, _ = plan_best(exe, CONFIG, out, {'MaxVelMag': 1.2}, candidates=3, budget_s=60, log=lambda *_: None)
    assert path == os.path.join(out, 'path.bin')
    assert np.isclose(plan_duration(path), 9.5)


def test_all_attempts_fail_raises(tmp_path):
    exe = fake_planner(tmp_path, BODY.format(plan="['segv']"))
    with pytest.raises(RuntimeError):
        plan_best(exe, CONFIG, os.path.join(tmp_path, 'out'), candidates=2, budget_s=5, log=lambda *_: None)
    assert not os.path.exists(os.path.join(tmp_path, 'out', 'path.bin'))
