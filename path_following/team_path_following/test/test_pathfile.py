import math
import os

import numpy as np

from team_path_following import frames
from team_path_following.pathfile import Trajectory, read_dhpath, write_dhpath


def _rows(T=3.0, dt=0.01):
    t = np.append(np.arange(0, T, dt), T)
    x = 0.5 * t ** 2
    rows = np.zeros((len(t), 12))
    rows[:, 0] = t
    rows[:, 1] = x            # NED x forward
    rows[:, 3] = -1.0         # 1 m up
    rows[:, 4] = t
    rows[:, 7] = 1.0
    rows[:, 10] = 0.0
    return rows, dt


def test_read_write_roundtrip(tmp_path):
    rows, dt = _rows()
    p = os.path.join(tmp_path, 'path.bin')
    write_dhpath(p, rows, dt)
    back, dt2 = read_dhpath(p)
    assert dt2 == dt and np.array_equal(back, rows)


def test_from_planner_and_sample():
    rows, _ = _rows()
    S = np.array([1.0, 2.0, 0.0])
    tr = Trajectory.from_planner(rows, S, math.radians(90))   # NED x = map +y
    p, v, a, yaw, _ = tr.sample(2.0)
    assert np.allclose(p, [1.0, 2.0 + 2.0, 1.0], atol=1e-9)
    assert np.allclose(v, [0, 2.0, 0], atol=1e-9)
    assert np.allclose(a, [0, 1.0, 0], atol=1e-9)
    assert math.isclose(yaw, math.radians(90), abs_tol=1e-9)
    # clamped outside [0, T]
    assert np.allclose(tr.sample(-1)[0], tr.p[0]) and np.allclose(tr.sample(99)[0], tr.p[-1])
    assert np.allclose(tr.tangent(0.0), [0, 1, 0])   # at rest: nearest moving direction


def test_time_scaled_is_exact():
    rows, _ = _rows()
    tr = Trajectory.from_planner(rows, np.zeros(3), 0.0)
    k = 1.7
    sc = tr.time_scaled(k)
    assert math.isclose(sc.duration, k * tr.duration)
    p1, v1, a1, _, _ = tr.sample(1.3)
    p2, v2, a2, _, _ = sc.sample(1.3 * k)
    assert np.allclose(p1, p2) and np.allclose(v1 / k, v2) and np.allclose(a1 / k ** 2, a2)


def test_planner_output_if_present():
    from conftest import REPO
    from team_path_following.course import load_course
    cfg = os.path.join(REPO, 'planner', 'config', 'course_v3.yaml')
    out = os.path.join(REPO, 'path_following', 'runs', 'course_v3_v12', 'path.bin')
    if not os.path.exists(out):
        return
    course = load_course(cfg)
    tr = Trajectory.load(out, course)
    assert np.allclose(tr.p[0], course.hover_point, atol=1e-6)
    assert np.allclose(tr.p[-1], course.goal, atol=1e-6)
    _ = frames
